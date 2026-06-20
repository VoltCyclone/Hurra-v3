// Host-native unit test for the WS2812 pure logic (HSV→GRB + throttle).
// Compiles ws2812.c with -DWS2812_HOSTTEST so the PIOC hardware glue is
// excluded and no SDK headers are pulled in.
//
//   cc -std=c11 -O2 -Wall -DWS2812_HOSTTEST -Isrc \
//      -o /tmp/ws2812_test test/ws2812_test.c src/ws2812.c
#include <stdio.h>
#include <stdint.h>
#include "ws2812.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

int main(void) {
    uint8_t grb[3];

    /* (A) Anchors on the hue wheel. Byte order is G,R,B. */
    ws2812_hsv_to_grb(0,   255, 255, grb);   /* red   */
    CHECK(grb[1] == 255 && grb[0] == 0 && grb[2] == 0, "h=0 -> pure red (R=255)");
    ws2812_hsv_to_grb(85,  255, 255, grb);   /* green */
    CHECK(grb[0] == 255 && grb[1] == 0 && grb[2] == 0, "h=85 -> pure green (G=255)");
    ws2812_hsv_to_grb(170, 255, 255, grb);   /* blue  */
    CHECK(grb[2] == 255 && grb[0] == 0 && grb[1] == 0, "h=170 -> pure blue (B=255)");

    /* (B) v==0 -> all channels off. */
    ws2812_hsv_to_grb(123, 255, 0, grb);
    CHECK(grb[0] == 0 && grb[1] == 0 && grb[2] == 0, "v=0 -> dark");

    /* (C) Brightness ceiling: full v never exceeds 255 on any channel. */
    for (int h = 0; h < 256; h++) {
        ws2812_hsv_to_grb((uint8_t)h, 255, 255, grb);
        CHECK(grb[0] <= 255 && grb[1] <= 255 && grb[2] <= 255, "channel in range");
    }

    /* (D) should_send: idle (no hue change) never sends. */
    CHECK(ws2812_should_send(10, 10, 0, 1000) == 0, "unchanged hue -> no send");

    /* (E) should_send: hue changed but throttle not elapsed -> no send. */
    CHECK(ws2812_should_send(10, 13, 100, 100 + (WS2812_MIN_INTERVAL_MS - 1)) == 0,
          "throttle window blocks send");

    /* (F) should_send: hue changed and interval elapsed -> send. */
    CHECK(ws2812_should_send(10, 13, 100, 100 + WS2812_MIN_INTERVAL_MS) == 1,
          "changed hue past interval -> send");

    /* (G) mode: not relaying -> ERROR, regardless of report recency. */
    CHECK(ws2812_mode(0, 1000, 1000)  == WS2812_MODE_ERROR, "not relaying -> ERROR (just moved)");
    CHECK(ws2812_mode(0, 99999, 0)    == WS2812_MODE_ERROR, "not relaying -> ERROR (long idle)");

    /* (H) mode: relaying + recent report -> ACTIVE; relaying + stale -> IDLE. */
    CHECK(ws2812_mode(1, 1000, 900)                   == WS2812_MODE_ACTIVE, "recent report -> ACTIVE");
    CHECK(ws2812_mode(1, 1000 + WS2812_IDLE_MS - 1, 1000) == WS2812_MODE_ACTIVE, "just under idle threshold -> ACTIVE");
    CHECK(ws2812_mode(1, 1000 + WS2812_IDLE_MS, 1000)     == WS2812_MODE_IDLE,   "at idle threshold -> IDLE");

    /* (I) breathe: trough at phase 0 and at the period boundary, peak at half. */
    CHECK(ws2812_breathe_v(0,   1000, 2, 32) == 2,  "breathe phase 0 -> min");
    CHECK(ws2812_breathe_v(500, 1000, 2, 32) == 32, "breathe half period -> max");
    CHECK(ws2812_breathe_v(1000,1000, 2, 32) == 2,  "breathe full period -> min (wraps)");
    {
        uint8_t q = ws2812_breathe_v(250, 1000, 0, 32);  /* quarter -> midpoint */
        CHECK(q >= 14 && q <= 18, "breathe quarter -> ~mid");
    }

    /* (J) compose: ERROR is red (R>0, G=B=0) and pulses within error V range. */
    {
        uint8_t e_lo[3], e_hi[3];
        ws2812_compose(WS2812_MODE_ERROR, 85 /*hue ignored*/, 0, e_lo);            /* trough */
        ws2812_compose(WS2812_MODE_ERROR, 85, WS2812_ERROR_PULSE_MS / 2, e_hi);    /* peak   */
        CHECK(e_lo[0] == 0 && e_lo[2] == 0 && e_hi[0] == 0 && e_hi[2] == 0, "ERROR is pure red (G=B=0)");
        CHECK(e_hi[1] > e_lo[1], "ERROR pulses brighter at half period");
        CHECK(e_hi[1] <= WS2812_ERROR_MAX_V, "ERROR peak within max V");
    }

    /* (K) compose: ACTIVE uses the hue at full brightness; IDLE breathes same hue dimmer at trough. */
    {
        uint8_t act[3], idle_lo[3];
        ws2812_compose(WS2812_MODE_ACTIVE, 0 /*red*/, 0, act);
        ws2812_compose(WS2812_MODE_IDLE,   0 /*red*/, 0, idle_lo);   /* phase 0 -> trough */
        CHECK(act[1] == WS2812_BRIGHTNESS, "ACTIVE red at full brightness");
        CHECK(idle_lo[1] == WS2812_IDLE_MIN_V, "IDLE red at trough brightness");
        CHECK(idle_lo[0] == 0 && idle_lo[2] == 0, "IDLE keeps the hue (red -> G=B=0)");
    }

    if (failures) { printf("%d FAILED\n", failures); return 1; }
    printf("ws2812: all passed\n");
    return 0;
}
