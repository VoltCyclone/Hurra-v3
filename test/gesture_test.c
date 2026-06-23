#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "gesture.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

int main(void) {
    /* (A) Struct byte budgets match the spec. */
    CHECK(sizeof(gst_sample_t) == 8,  "gst_sample_t is 8 bytes");
    CHECK(sizeof(gst_knot_t)   <= 16, "gst_knot_t fits (float+float+float+uint16_t)");
    /* gst_shape_t: 48*knot_size + 2 + 4 + 2 + 1 + 1 <= 800 bytes */
    CHECK(sizeof(gst_shape_t)  <= 800, "gst_shape_t within 800-byte budget");

    /* (B) Fixed sizes are exactly the spec values. */
    CHECK(GST_CAP_RING   == 256, "capture ring is 256");
    CHECK(GST_KNOTS_MAX  == 48,  "knots max is 48");
    CHECK(GST_LIB_SHAPES == 32,  "library holds 32 shapes");
    CHECK(GST_WARM_MIN   == 6,   "warm threshold is 6");

    /* (C) init is callable and clears state (no crash). */
    gesture_init(1000);
    CHECK(1, "gesture_init runs");

    /* ── Task 2: capture ring ── */
    gesture_init(1000);
    CHECK(gesture_capture_count() == 0, "ring starts empty");

    gesture_capture_push(3, -1, 1000);
    gesture_capture_push(5,  2, 2000);
    CHECK(gesture_capture_count() == 2, "ring counts pushes");

    gst_sample_t s;
    CHECK(gesture_capture_get(0, &s) && s.dx == 5 && s.dy == 2 && s.t_us == 2000,
          "age 0 is most recent");
    CHECK(gesture_capture_get(1, &s) && s.dx == 3 && s.dy == -1,
          "age 1 is older");
    CHECK(!gesture_capture_get(2, &s), "age past count fails");

    /* Overflow: pushing > GST_CAP_RING keeps the newest GST_CAP_RING and
     * saturates the count. */
    gesture_init(1000);
    for (int i = 0; i < GST_CAP_RING + 50; i++)
        gesture_capture_push((int16_t)i, 0, (uint32_t)(i * 1000));
    CHECK(gesture_capture_count() == GST_CAP_RING, "count saturates at ring size");
    gesture_capture_get(0, &s);
    CHECK(s.dx == (int16_t)(GST_CAP_RING + 49), "newest survives overflow");

    /* ── Task 3: reconstruction ── */
    {
        /* A straight 3-sample move: (2,0),(2,0),(1,0). Cumulative x: 2,4,5. */
        gst_sample_t in[3] = {
            { 2, 0, 1000 }, { 2, 0, 2000 }, { 1, 0, 3000 },
        };
        gst_point_t pts[8];
        uint16_t np = gesture_reconstruct(in, 3, pts, 8);
        CHECK(np == 3, "reconstruct returns sample count");
        CHECK(fabsf(pts[0].x - 2.0f) < 1e-4f, "cum x[0]=2");
        CHECK(fabsf(pts[1].x - 4.0f) < 1e-4f, "cum x[1]=4");
        CHECK(fabsf(pts[2].x - 5.0f) < 1e-4f, "cum x[2]=5");
        CHECK(fabsf(pts[2].f - 1.0f) < 1e-4f, "last fraction is 1.0");
        CHECK(fabsf(pts[0].f - 0.4f) < 1e-3f, "fraction tracks path length (2/5)");
        /* cumulative time relative to first sample */
        CHECK(pts[0].t_us == 0u,    "t starts at 0");
        CHECK(pts[2].t_us == 2000u, "t accumulates dt");
    }
    {
        /* Zero-length input: no NaN, fractions all zero. */
        gst_sample_t in[2] = { {0,0,1000}, {0,0,2000} };
        gst_point_t pts[4];
        uint16_t np = gesture_reconstruct(in, 2, pts, 4);
        CHECK(np == 2, "zero-length still returns points");
        CHECK(pts[0].f == 0.0f && pts[1].f == 0.0f, "zero-length fractions are 0");
    }

    /* ── Task 4: resample ── */
    {
        /* A diagonal ramp, 5 points. Resampled to GST_KNOTS_MAX knots that
         * still start at origin-ish and end exactly at the last point. */
        gst_point_t pts[5];
        for (int i = 0; i < 5; i++) {
            pts[i].x = (float)(i * 10);   /* 0,10,20,30,40 */
            pts[i].y = (float)(i * 10);
            pts[i].t_us = (uint32_t)(i * 1000);
            pts[i].f = (float)i / 4.0f;
        }
        gst_knot_t kn[GST_KNOTS_MAX];
        uint16_t nk = gesture_resample(pts, 5, kn);
        CHECK(nk == GST_KNOTS_MAX, "resample emits fixed knot count");
        CHECK(fabsf(kn[0].f - 0.0f) < 1e-4f, "first knot fraction 0");
        CHECK(fabsf(kn[GST_KNOTS_MAX-1].f - 1.0f) < 1e-4f, "last knot fraction 1");
        /* endpoint exactness */
        CHECK(fabsf(kn[GST_KNOTS_MAX-1].ux - 40.0f) < 1e-2f, "endpoint x preserved");
        CHECK(fabsf(kn[GST_KNOTS_MAX-1].uy - 40.0f) < 1e-2f, "endpoint y preserved");
        /* monotonic along a straight ramp: ux strictly non-decreasing */
        int mono = 1;
        for (int i = 1; i < GST_KNOTS_MAX; i++)
            if (kn[i].ux < kn[i-1].ux - 1e-3f) mono = 0;
        CHECK(mono, "resampled knots monotonic on a straight ramp");

        CHECK(gesture_resample(pts, 1, kn) == 0, "too-few points returns 0");
    }

    if (failures) { printf("%d FAILURES\n", failures); return 1; }
    printf("ALL GESTURE TESTS PASSED\n");
    return 0;
}
