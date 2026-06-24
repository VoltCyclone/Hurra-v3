#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "humanize.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

int main(void) {
    humanize_init(1000);            /* 1 ms frame */
    CHECK(1, "scaffold");

    /* (A) Conservation: summed output == summed injected, within rounding. */
    humanize_init(1000);
    humanize_set_level(2);
    long sx = 0;
    for (int i = 0; i < 5000; i++) {           /* steady 3 px/frame stream */
        int16_t dx = 3, dy = 0;
        humanize_filter(&dx, &dy);
        sx += dx;
    }
    for (int i = 0; i < 200; i++) { int16_t dx = 0, dy = 0; humanize_filter(&dx,&dy); sx += dx; }
    CHECK(labs(sx - 5000L*3) <= 2, "conservation: output sum tracks input sum");

    /* (B) Idle gate: zero in, settled -> zero out (no tremor on still cursor). */
    humanize_init(1000);
    for (int i = 0; i < 50; i++) { int16_t dx=0, dy=0; humanize_filter(&dx,&dy); }
    int moved = 0;
    for (int i = 0; i < 500; i++) { int16_t dx=0, dy=0; humanize_filter(&dx,&dy); if (dx||dy) moved=1; }
    CHECK(!moved, "idle gate: still cursor stays still");

    /* (C) Human cap: a huge single injection never emits a teleport frame. */
    humanize_init(1000);
    humanize_set_level(2);
    int16_t bx = 30000, by = 0; long total = 0; int maxframe = 0;
    humanize_filter(&bx, &by); total += bx; if (abs(bx) > maxframe) maxframe = abs(bx);
    for (int i = 0; i < 4000; i++) { int16_t dx=0,dy=0; humanize_filter(&dx,&dy); total += dx; if (abs(dx)>maxframe) maxframe=abs(dx); }
    CHECK(maxframe <= 127, "cap: no single frame exceeds human ceiling");
    CHECK(labs(total - 30000) <= 4, "cap: clamped motion is carried, not dropped");

    /* (D) Tier-1: a constant-velocity stream must not emit a long run of
     *     identical values (anti-quantization) and must vary frame-to-frame. */
    humanize_init(1000);
    humanize_set_level(2);
    int max_run = 0, run = 0; int16_t prev = -999;
    for (int i = 0; i < 3000; i++) {
        int16_t dx = 5, dy = 5; humanize_filter(&dx, &dy);
        if (dx == prev) { run++; if (run > max_run) max_run = run; } else run = 0;
        prev = dx;
    }
    CHECK(max_run < 200, "anti-quantization: no long identical-value run");

    /* (E) In-frame delivery: a single small injection (<= cap) must land THIS
     *     frame, not be smeared across many frames. Pure-X move has no
     *     perpendicular noise on X, so dx should be ~the full injected amount. */
    humanize_init(1000);
    humanize_set_level(2);
    int16_t edx = 20, edy = 0;
    humanize_filter(&edx, &edy);
    CHECK(edx >= 18, "in-frame: small injection delivered same frame");

    /* (F) Field-clip carry: motion returned (because the report field couldn't
     *     carry it this frame) is redelivered, not lost. Models an 8-bit field
     *     with only 50 counts of headroom/frame; all 200 must still arrive. */
    humanize_init(1000);
    humanize_set_level(2);
    {
        long delivered = 0;
        int16_t fdx = 200, fdy = 0;
        humanize_filter(&fdx, &fdy);
        for (int i = 0; i < 80; i++) {
            int acc = fdx > 50 ? 50 : (fdx < -50 ? -50 : fdx); /* field headroom */
            humanize_return((int16_t)(fdx - acc), 0);          /* carry the rest */
            delivered += acc;
            fdx = 0; fdy = 0;
            humanize_filter(&fdx, &fdy);
        }
        CHECK(labs(delivered - 200) <= 2, "field-clip carry: returned motion is redelivered");
    }

    /* (G) Measured interval: a steady 1000 µs cadence converges the EWMA to
     *     ~1000 µs, and humanize_target_ldval converts it correctly. */
    humanize_init(1000);
    humanize_set_level(2);
    {
        uint32_t t = 50000;                 /* arbitrary GPT2 start */
        for (int i = 0; i < 64; i++) { humanize_record_arrival(t); t += 1000; }
        uint32_t meas = humanize_measured_interval_us();
        CHECK(meas >= 980 && meas <= 1020, "measured interval converges to ~1000us");
        /* At 24 MHz PIT clock, 1000 µs → 24000 counts → LDVAL 23999. */
        uint32_t ld = humanize_target_ldval(24000000u);
        CHECK(ld >= 23000 && ld <= 25000, "target ldval ~= 24000-1 for 1ms @ 24MHz");
    }

    /* (H) Confidence gate: fewer than HZ_MEAS_MIN_COUNT reports → target 0. */
    humanize_init(1000);
    humanize_set_level(2);
    {
        uint32_t t = 1000;
        humanize_record_arrival(t); t += 1000;
        humanize_record_arrival(t);                 /* only 2 reports */
        CHECK(humanize_target_ldval(24000000u) == 0, "no target before min count");
    }

    /* (I) Outlier rejection: a single huge gap (dropout) must not yank the EWMA. */
    humanize_init(1000);
    humanize_set_level(2);
    {
        uint32_t t = 0;
        for (int i = 0; i < 32; i++) { humanize_record_arrival(t); t += 1000; }
        uint32_t before = humanize_measured_interval_us();
        humanize_record_arrival(t + 500000); /* 500 ms dropout gap */
        uint32_t after = humanize_measured_interval_us();
        CHECK(labs((long)after - (long)before) <= 5, "dropout gap rejected, EWMA stable");
    }

    /* (K) Seed-poison guard: the FIRST inter-report gap can be huge (device only
     *     sends on movement / NAKs when idle). Pre-seeding from bInterval in
     *     humanize_init must keep the estimate sane so normal reports are NOT all
     *     rejected as "bursts" and the EWMA still converges to the real cadence. */
    humanize_init(1000);                 /* nominal 1 ms → pre-seed */
    humanize_set_level(2);
    {
        uint32_t t = 0;
        humanize_record_arrival(t);      /* first report */
        t += 47000;                      /* 47 ms idle gap before the next one */
        humanize_record_arrival(t);
        for (int i = 0; i < 64; i++) { t += 1000; humanize_record_arrival(t); }
        uint32_t meas = humanize_measured_interval_us();
        CHECK(meas >= 900 && meas <= 1100,
              "seed-poison guard: huge first gap does not lock out the EWMA");
        CHECK(humanize_target_ldval(24000000u) != 0, "target valid after recovery");
    }

    /* (J) Adaptive-noise default OFF: with the envelope compiled out (default),
     *     conservation still holds exactly as in (A) — proves no behavior drift. */
    humanize_init(1000);
    humanize_set_level(2);
    {
        long s = 0; uint32_t t = 0;
        for (int i = 0; i < 3000; i++) {
            humanize_record_arrival(t); t += 1000;  /* feed intervals too */
            int16_t dx = 4, dy = 0; humanize_filter(&dx, &dy); s += dx;
        }
        for (int i = 0; i < 200; i++) { int16_t dx=0,dy=0; humanize_filter(&dx,&dy); s += dx; }
        CHECK(labs(s - 3000L*4) <= 2, "conservation holds with interval feed");
    }

    /* ── injected emit quantization (sub-pixel + clamp carry) ── */
    {
        /* (A) Sub-pixel carry: a stream of 0.4-count steps must accumulate into
         *     whole counts (no per-step truncation-to-zero loss). */
        humanize_init(1000);
        long acc = 0;
        for (int i = 0; i < 100; i++) {
            int16_t ox = 0, oy = 0;
            humanize_inject_emit(0.4f, 0.0f, &ox, &oy);
            acc += ox;
        }
        CHECK(labs(acc - 40L) <= 1, "inject_emit: sub-pixel residual conserves (0.4*100=40)");

        /* (B) Field-clamp carry: a huge single step never teleports, and the
         *     clamped overflow is redelivered over later calls (not dropped). */
        humanize_init(1000);
        int16_t bx = 0, by = 0; long total = 0; int maxframe = 0;
        humanize_inject_emit(30000.0f, 0.0f, &bx, &by);
        total += bx; if (abs(bx) > maxframe) maxframe = abs(bx);
        for (int i = 0; i < 4000; i++) {
            int16_t dx = 0, dy = 0;
            humanize_inject_emit(0.0f, 0.0f, &dx, &dy);
            total += dx; if (abs(dx) > maxframe) maxframe = abs(dx);
        }
        CHECK(maxframe <= 127, "inject_emit: no single frame exceeds human cap");
        CHECK(labs(total - 30000L) <= 4, "inject_emit: clamped overflow redelivered, not lost");

        /* (C) Two-axis independence: a diagonal stream conserves on both axes. */
        humanize_init(1000);
        long ax = 0, ay = 0;
        for (int i = 0; i < 200; i++) {
            int16_t dx = 0, dy = 0;
            humanize_inject_emit(1.5f, -0.75f, &dx, &dy);
            ax += dx; ay += dy;
        }
        CHECK(labs(ax - 300L) <= 1, "inject_emit: x-axis conserves (1.5*200=300)");
        CHECK(labs(ay + 150L) <= 1, "inject_emit: y-axis conserves (-0.75*200=-150)");
    }

    printf(failures ? "\n%d FAILED\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
