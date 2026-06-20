#pragma once
#include <stdint.h>

/* Tunables. */
#define WS2812_HUE_STEP        1      /* hue units advanced per forwarded report */
#define WS2812_BRIGHTNESS      32     /* active/idle-peak V (0..255); pixel-friendly */
#define WS2812_MIN_INTERVAL_MS 16u    /* min ms between PIOC pushes (~60 Hz cap)  */

/* Idle/error animation tunables. */
#define WS2812_IDLE_MS         3000u  /* no report for this long → IDLE breathe    */
#define WS2812_IDLE_BREATHE_MS 3000u  /* idle breathe period (full up+down cycle)  */
#define WS2812_IDLE_MIN_V      2u     /* idle breathe trough brightness            */
#define WS2812_ERROR_PULSE_MS  700u   /* "not relaying" red pulse period           */
#define WS2812_ERROR_MIN_V     2u     /* red pulse trough brightness               */
#define WS2812_ERROR_MAX_V     48u    /* red pulse peak brightness (reads clearly) */

/* LED behaviour modes, highest priority first when composing a frame. */
typedef enum {
    WS2812_MODE_ACTIVE = 0,   /* reports flowing: per-report rainbow, full bright  */
    WS2812_MODE_IDLE   = 1,   /* no recent report: breathe the last hue            */
    WS2812_MODE_ERROR  = 2,   /* relay down: pulse red, overrides motion and idle  */
} ws2812_mode_t;

/* ── Pure logic (host-testable, no hardware) ─────────────────────────────── */

/* HSV→GRB for WS2812 wire order. out[0]=G, out[1]=R, out[2]=B.
 * s/v are 0..255; v==0 yields {0,0,0}. */
void ws2812_hsv_to_grb(uint8_t h, uint8_t s, uint8_t v, uint8_t out[3]);

/* Throttle/skip decision. Returns 1 iff the hue changed AND at least
 * WS2812_MIN_INTERVAL_MS have elapsed since the last push. */
int ws2812_should_send(uint8_t prev_hue, uint8_t cur_hue,
                       uint32_t last_ms, uint32_t now_ms);

/* Select the behaviour mode. ERROR if not relaying (overrides all); else IDLE
 * if no report for >= WS2812_IDLE_MS; else ACTIVE. */
ws2812_mode_t ws2812_mode(int relaying, uint32_t now_ms, uint32_t last_report_ms);

/* Triangle-wave brightness: min_v at phase 0, max_v at the half-period, back to
 * min_v at the period boundary. period_ms must be > 0. */
uint8_t ws2812_breathe_v(uint32_t now_ms, uint32_t period_ms,
                         uint8_t min_v, uint8_t max_v);

/* Build the GRB frame for a mode: ACTIVE = hue at full brightness; IDLE = hue
 * breathing between WS2812_IDLE_MIN_V and WS2812_BRIGHTNESS; ERROR = red pulsing
 * between WS2812_ERROR_MIN_V and WS2812_ERROR_MAX_V. */
void ws2812_compose(ws2812_mode_t mode, uint8_t hue, uint32_t now_ms,
                    uint8_t out[3]);

/* ── Hardware API (no-op under WS2812_HOSTTEST) ──────────────────────────── */

/* Bring up PIOC on PF13/IO1 and load the waveform program. Call once on V5F
 * before the relay loop. */
void ws2812_init(void);

/* O(1): advance the hue accumulator and record the report time (for idle
 * detection). Call once per forwarded mouse report. */
void ws2812_note_report(uint32_t now_ms);

/* Throttled, non-blocking pixel push. `relaying` is this board's relay-up flag
 * (false → red error pulse). Call once per relay-loop iteration. */
void ws2812_service(uint32_t now_ms, int relaying);
