#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "gesture.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

/* Warm the library across all length buckets so the selector (Task 6) routes
 * to replay. 6 shapes, 2 per bucket with mirrored curvature for morph variety. */
static void warm_library_all_buckets(void) {
    const int lens[3] = { 40, 200, 800 };
    for (int b = 0; b < 3; b++) {
        for (int rep = 0; rep < 2; rep++) {
            gst_sample_t mv[256];
            int steps = lens[b] / 4; if (steps > 200) steps = 200;
            for (int i = 0; i < steps; i++) {
                mv[i].dx = 4;
                mv[i].dy = (int16_t)((rep ? 1 : -1) * (i & 1));   /* slight, mirrored bow */
                mv[i].t_us = (uint32_t)(i * 1000);
            }
            gst_shape_t s;
            if (gesture_build_shape(mv, (uint16_t)steps, &s)) gesture_library_admit(&s);
        }
    }
}

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

    /* ── Task 3: reconstruction (prepend-origin semantics) ── */
    gesture_init(1000);   /* nominal = 1000 us, used for first-leg timing */
    {
        /* A straight 3-sample move: (2,0),(2,0),(1,0). Cumulative x: 2,4,5.
         * With origin prepended: out[0]=(0,0), out[1]=(2,0), out[2]=(4,0), out[3]=(5,0).
         * Total path length = 2+2+1 = 5. Fractions: 0, 2/5, 4/5, 1.0. */
        gst_sample_t in[3] = {
            { 2, 0, 1000 }, { 2, 0, 2000 }, { 1, 0, 3000 },
        };
        gst_point_t pts[8];
        uint16_t np = gesture_reconstruct(in, 3, pts, 8);
        CHECK(np == 4, "reconstruct returns n+1 (origin + samples)");
        CHECK(pts[0].x == 0.0f && pts[0].y == 0.0f, "pts[0] is the origin (0,0)");
        CHECK(fabsf(pts[1].x - 2.0f) < 1e-4f, "cum x[1]=2");
        CHECK(fabsf(pts[2].x - 4.0f) < 1e-4f, "cum x[2]=4");
        CHECK(fabsf(pts[3].x - 5.0f) < 1e-4f, "cum x[3]=5");
        CHECK(fabsf(pts[0].f - 0.0f) < 1e-4f, "origin fraction is 0.0");
        CHECK(fabsf(pts[3].f - 1.0f) < 1e-4f, "last fraction is 1.0");
        /* timing: origin at 0; out[1] = nominal + (t0-t0) = 1000;
         * out[3] = nominal + (t2-t0) = 1000 + (3000-1000) = 3000 */
        CHECK(pts[0].t_us == 0u,    "origin t_us is 0");
        CHECK(pts[1].t_us == 1000u, "first sample t_us = nominal (1000)");
        CHECK(pts[3].t_us == 3000u, "last sample t_us = nominal + (t2-t0)");
    }
    {
        /* Zero-length input: no NaN, fractions all zero. */
        gst_sample_t in[2] = { {0,0,1000}, {0,0,2000} };
        gst_point_t pts[4];
        uint16_t np = gesture_reconstruct(in, 2, pts, 4);
        CHECK(np == 3, "zero-length still returns n+1 points (origin + 2)");
        CHECK(pts[0].f == 0.0f && pts[1].f == 0.0f && pts[2].f == 0.0f,
              "zero-length fractions are all 0");
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

    /* ── Task 5: spatial normalization ── */
    {
        gst_shape_t sh;
        memset(&sh, 0, sizeof(sh));
        sh.n = GST_KNOTS_MAX;
        /* Build a straight gesture heading up-and-right, endpoint (30,40), |.|=50. */
        for (int i = 0; i < GST_KNOTS_MAX; i++) {
            float t = (float)i / (float)(GST_KNOTS_MAX - 1);
            sh.knots[i].ux = 30.0f * t;
            sh.knots[i].uy = 40.0f * t;
            sh.knots[i].f  = t;
        }
        CHECK(gesture_normalize_spatial(&sh), "normalize succeeds on real gesture");
        CHECK(fabsf(sh.raw_len - 50.0f) < 1e-2f, "raw_len is original magnitude");
        /* endpoint becomes unit +X */
        CHECK(fabsf(sh.knots[GST_KNOTS_MAX-1].ux - 1.0f) < 1e-3f, "endpoint ux=1");
        CHECK(fabsf(sh.knots[GST_KNOTS_MAX-1].uy - 0.0f) < 1e-3f, "endpoint uy=0");
        /* start stays at origin */
        CHECK(fabsf(sh.knots[0].ux) < 1e-3f && fabsf(sh.knots[0].uy) < 1e-3f,
              "start at origin");

        /* degenerate: tiny gesture rejected */
        gst_shape_t tiny;
        memset(&tiny, 0, sizeof(tiny));
        tiny.n = GST_KNOTS_MAX;
        tiny.knots[GST_KNOTS_MAX-1].ux = 0.3f;
        CHECK(!gesture_normalize_spatial(&tiny), "sub-unit gesture rejected");
    }

    /* ── Task 6: temporal normalization ── */
    {
        gesture_init(1000);   /* nominal = 1000 us */
        /* Reconstructed points: evenly 1ms apart over a straight line, 5 pts. */
        gst_point_t pts[5];
        for (int i = 0; i < 5; i++) {
            pts[i].x = (float)(i * 10);
            pts[i].y = 0.0f;
            pts[i].f = (float)i / 4.0f;
            pts[i].t_us = (uint32_t)(i * 1000);
        }
        gst_shape_t sh;
        memset(&sh, 0, sizeof(sh));
        sh.n = GST_KNOTS_MAX;
        for (int k = 0; k < GST_KNOTS_MAX; k++)
            sh.knots[k].f = (float)k / (float)(GST_KNOTS_MAX - 1);

        gesture_normalize_temporal(&sh, pts, 5);
        CHECK(sh.knots[0].dt_q == 0, "knot 0 dt_q is 0");
        /* total time 4000us over 47 inter-knot gaps -> ~85us each -> .8 fixed
         * of nominal(1000): 85/1000*256 ~= 21.8 -> ~22 */
        CHECK(sh.knots[1].dt_q >= 18 && sh.knots[1].dt_q <= 26,
              "per-knot dt_q in .8 fixed of nominal");
        /* total_us: 4000us / 1000 * 256 = 1024 */
        CHECK(sh.total_us >= 1000 && sh.total_us <= 1048, "total_us .8 fixed");
    }

    /* ── Task 7: quality gates + builder ── */
    {
        gesture_init(1000);
        /* A clean medium rightward gesture, 20 samples, ~2px/sample => len ~40. */
        gst_sample_t mv[20];
        for (int i = 0; i < 20; i++) { mv[i].dx = 2; mv[i].dy = 0; mv[i].t_us = (uint32_t)(i*1000); }
        gst_shape_t sh;
        CHECK(gesture_build_shape(mv, 20, &sh), "clean gesture admitted");
        CHECK(sh.n == GST_KNOTS_MAX, "built shape has full knots");
        CHECK(fabsf(sh.knots[GST_KNOTS_MAX-1].ux - 1.0f) < 1e-3f, "built shape normalized");
        CHECK(gesture_length_bucket(sh.raw_len) == 0 || gesture_length_bucket(sh.raw_len) == 1,
              "bucket classifies (short/medium for len~40)");

        /* too few samples rejected */
        CHECK(!gesture_build_shape(mv, 3, &sh), "too-few samples rejected");

        /* a near-still twitch rejected (len below GST_MIN_LEN) */
        gst_sample_t twitch[8] = {{1,0,0},{0,0,1000},{-1,0,2000},{0,0,3000},
                                  {1,0,4000},{0,0,5000},{0,0,6000},{0,0,7000}};
        CHECK(!gesture_build_shape(twitch, 8, &sh), "tiny twitch rejected");

        /* bucket boundaries */
        CHECK(gesture_length_bucket(40.0f)  == 0, "len 40 -> short");
        CHECK(gesture_length_bucket(200.0f) == 1, "len 200 -> medium");
        CHECK(gesture_length_bucket(800.0f) == 2, "len 800 -> long");
    }

    /* Regression (C1): a large first delta must not collapse knots onto pts[0]
     * or zero out the leading dt_q. Spread must survive reconstruction. */
    gesture_init(1000);
    {
        gst_sample_t flick[6] = {
            {50,0,1000},{2,0,2000},{2,0,3000},{2,0,4000},{2,0,5000},{2,0,6000}
        };
        gst_shape_t sh;
        CHECK(gesture_build_shape(flick, 6, &sh), "flick gesture admitted");
        /* Count knots whose ux sits essentially on the endpoint (ux≈1 after
         * normalization). With the C1 bug, most knots collapse there. */
        int near_end = 0;
        for (int k = 0; k < sh.n; k++)
            if (sh.knots[k].ux > 0.98f) near_end++;
        CHECK(near_end < sh.n / 2, "knots are spread, not collapsed at endpoint");
        /* Most inter-knot intervals must carry real (nonzero) time. */
        int nz = 0;
        for (int k = 1; k < sh.n; k++) if (sh.knots[k].dt_q > 0) nz++;
        CHECK(nz > sh.n / 2, "most dt_q are nonzero (no teleport)");
    }

    /* ── Task 8: library + warmth ── */
    {
        gesture_init(1000);
        CHECK(gesture_warmth() == GST_COLD, "empty library is COLD");
        CHECK(gesture_library_count() == 0, "empty library count 0");
        CHECK(gesture_library_select(100.0f) == NULL, "select on empty is NULL");

        /* Admit one shape per bucket by synthesizing gestures of each length. */
        gesture_init(1000);
        const int lens[3] = { 40, 200, 800 };  /* short, medium, long */
        for (int b = 0; b < 3; b++) {
            gst_sample_t mv[64];
            int steps = lens[b] / 4;            /* 4 px/sample */
            if (steps > 64) steps = 64;
            for (int i = 0; i < steps; i++) { mv[i].dx = 4; mv[i].dy = 0; mv[i].t_us = (uint32_t)(i*1000); }
            gst_shape_t sh;
            if (gesture_build_shape(mv, (uint16_t)steps, &sh))
                gesture_library_admit(&sh);
        }
        CHECK(gesture_library_count() == 3, "admitted 3 shapes");
        CHECK(gesture_warmth() == GST_COLD,
              "3 shapes below WARM_MIN threshold stays COLD");

        /* selection picks nearest raw_len */
        const gst_shape_t *sel = gesture_library_select(210.0f);
        CHECK(sel != NULL, "select returns a shape");
        CHECK(gesture_length_bucket(sel->raw_len) == 1, "nearest to 210 is medium bucket");

        /* FIFO eviction: overflow one bucket and confirm count caps. */
        gesture_init(1000);
        for (int i = 0; i < GST_LIB_SHAPES + 10; i++) {
            gst_sample_t mv[20];
            for (int j = 0; j < 20; j++) { mv[j].dx = 2; mv[j].dy = 0; mv[j].t_us = (uint32_t)(j*1000); }
            gst_shape_t sh;
            if (gesture_build_shape(mv, 20, &sh)) gesture_library_admit(&sh);
        }
        CHECK(gesture_library_count() <= GST_LIB_SHAPES, "library count never exceeds cap");
    }

    /* Regression: 1..GST_WARM_MIN-1 shapes must stay COLD (warmth gate). */
    gesture_init(1000);
    {
        gst_sample_t mv[20];
        for (int j = 0; j < 20; j++) { mv[j].dx = 2; mv[j].dy = 0; mv[j].t_us = (uint32_t)(j*1000); }
        gst_shape_t sh;
        int admitted = 0;
        for (int i = 0; i < GST_WARM_MIN - 1 && admitted < GST_WARM_MIN - 1; i++) {
            if (gesture_build_shape(mv, 20, &sh)) { gesture_library_admit(&sh); admitted++; }
        }
        CHECK(admitted == GST_WARM_MIN - 1, "admitted WARM_MIN-1 shapes");
        CHECK(gesture_warmth() == GST_COLD, "below WARM_MIN stays COLD");
    }

    /* ── Plan 2 Task 1: PRNG determinism + range ── */
    {
        gesture_init(1000);
        uint32_t a0 = gesture_rand_u32();
        uint32_t a1 = gesture_rand_u32();
        gesture_init(1000);                     /* same deterministic seed */
        CHECK(gesture_rand_u32() == a0 && gesture_rand_u32() == a1,
              "PRNG is deterministic across re-init");
        CHECK(a0 != a1, "PRNG advances between draws");

        int in_band = 1;
        for (int i = 0; i < 2000; i++) {
            float r = gesture_rand_range(-1.5f, 1.5f);
            if (r < -1.5f || r >= 1.5f) in_band = 0;
        }
        CHECK(in_band, "rand_range stays within [lo, hi)");
    }

    /* ── Plan 2 Task 2: replay transform (no augmentation) ── */
    {
        gesture_init(1000);
        warm_library_all_buckets();             /* routes to replay now and post-Task-6 */

        /* Begin toward (30,40): magnitude 50, angle ~53°. */
        gesture_motion_begin(30, 40, MOTION_MODE_SILENT);
        CHECK(!gesture_motion_done(), "Task2: gesture active after begin");

        float sx = 0.0f, sy = 0.0f, dx, dy; uint16_t dtq = 0; int steps = 0;
        while (gesture_motion_next(&dx, &dy, &dtq)) { sx += dx; sy += dy; if (++steps > 100) break; }
        CHECK(steps == GST_KNOTS_MAX - 1, "Task2: emits n-1 steps");
        CHECK(fabsf(sx - 30.0f) < 1e-2f && fabsf(sy - 40.0f) < 1e-2f,
              "Task2: replay endpoint reaches target");
        CHECK(gesture_motion_done(), "Task2: done after exhaustion");

        /* Zero-magnitude target → neither replay nor synth engages (R<1) → idle.
         * This stays true after Task 5 adds the synth fallback. */
        gesture_init(1000);
        warm_library_all_buckets();
        gesture_motion_begin(0, 0, MOTION_MODE_SILENT);
        CHECK(gesture_motion_done(), "Task2: zero-target begin is idle (no source)");
        CHECK(!gesture_motion_next(&dx, &dy, &dtq), "Task2: next when idle returns false");
    }

    /* ── Plan 2 Task 3: augmentation (jitter + morph + time-warp + trueing) ── */
    {
        gesture_init(1000);
        warm_library_all_buckets();

        /* (A) Endpoint is EXACT despite jitter (trueing). */
        gesture_motion_begin(60, 25, MOTION_MODE_SILENT);
        float sx = 0, sy = 0, dx, dy; uint16_t dtq;
        while (gesture_motion_next(&dx, &dy, &dtq)) { sx += dx; sy += dy; }
        CHECK(fabsf(sx - 60.0f) < 1e-2f && fabsf(sy - 25.0f) < 1e-2f,
              "Task3: endpoint trueing lands exactly on target");

        /* (B) Two begins to the same target differ mid-path (jitter active). */
        float ax[GST_KNOTS_MAX]; int an = 0;
        gesture_motion_begin(60, 25, MOTION_MODE_SILENT);
        while (gesture_motion_next(&dx, &dy, &dtq)) ax[an++] = dx;
        float bx[GST_KNOTS_MAX]; int bn = 0;
        gesture_motion_begin(60, 25, MOTION_MODE_SILENT);
        while (gesture_motion_next(&dx, &dy, &dtq)) bx[bn++] = dx;
        int differ = 0;
        for (int i = 0; i < an && i < bn; i++) if (fabsf(ax[i] - bx[i]) > 1e-4f) differ = 1;
        CHECK(differ, "Task3: jitter makes repeated gestures non-identical");
    }

    /* ── Plan 2 Task 3: morph keeps endpoint, blends micro-structure ── */
    {
        gesture_init(1000);
        warm_library_all_buckets();
        /* Add two distinct medium shapes (same bucket ~len 180, opposite curvature)
         * so the medium bucket has morph partners. */
        gst_sample_t a[60], b[60];
        for (int i = 0; i < 60; i++) {
            a[i].dx = 3; a[i].dy = (int16_t)((i < 30) ?  1 : -1); a[i].t_us = (uint32_t)(i*1000);
            b[i].dx = 3; b[i].dy = (int16_t)((i < 30) ? -1 :  1); b[i].t_us = (uint32_t)(i*1000);
        }
        gst_shape_t sa, sb;
        CHECK(gesture_build_shape(a, 60, &sa), "Task3: shape A built");
        CHECK(gesture_build_shape(b, 60, &sb), "Task3: shape B built");
        gesture_library_admit(&sa);
        gesture_library_admit(&sb);

        gesture_motion_begin(180, 10, MOTION_MODE_SILENT);
        float sx = 0, sy = 0, dx, dy; uint16_t dtq; int steps = 0; uint32_t dtsum = 0;
        while (gesture_motion_next(&dx, &dy, &dtq)) { sx += dx; sy += dy; dtsum += dtq; steps++; }
        CHECK(fabsf(sx - 180.0f) < 1e-2f && fabsf(sy - 10.0f) < 1e-2f,
              "Task3: morphed gesture still lands on target");
        CHECK(steps == GST_KNOTS_MAX - 1, "Task3: morphed gesture full step count");
        CHECK(dtsum > 0, "Task3: time-warped dt_q sequence is non-trivial");
    }

    /* ── resource bound: total engine state stays bounded ── */
    /* lib pool 32*780 ~= 25KB + cap ring 256*8 = 2KB + bookkeeping. */
    printf("INFO sizeof(gst_shape_t)=%zu pool=%zu cap=%zu\n",
           sizeof(gst_shape_t),
           sizeof(gst_shape_t) * GST_LIB_SHAPES,
           sizeof(gst_sample_t) * GST_CAP_RING);
    /* gst_knot_t is 16 B (3 floats + uint16 padded to 4), so shape ~780 B and
     * pool ~25 KB — bounded and well within V5F RAM (the earlier "19 KB" figure
     * miscounted the float trio as 12 B incl. dt_q). */
    CHECK(sizeof(gst_shape_t) * GST_LIB_SHAPES <= 26000, "library pool <= 26KB");

    /* ── Plan 2 Task 4: repetition guard ── */
    {
        gesture_init(1000);
        warm_library_all_buckets();                    /* warm → replay path */
        /* Extra medium-bucket shapes give the guard partners to re-roll among. */
        gst_sample_t a[60], b[60];
        for (int i = 0; i < 60; i++) {
            a[i].dx = 3; a[i].dy = (int16_t)((i < 30) ?  1 : -1); a[i].t_us = (uint32_t)(i*1000);
            b[i].dx = 3; b[i].dy = (int16_t)((i < 30) ? -1 :  1); b[i].t_us = (uint32_t)(i*1000);
        }
        gst_shape_t sa, sb; gesture_build_shape(a, 60, &sa); gesture_build_shape(b, 60, &sb);
        gesture_library_admit(&sa); gesture_library_admit(&sb);

        uint32_t before = gesture_dup_rejected();
        for (int k = 0; k < 50; k++) {                 /* hammer one target */
            gesture_motion_begin(180, 10, MOTION_MODE_SILENT);
            float dx, dy; uint16_t dtq;
            while (gesture_motion_next(&dx, &dy, &dtq)) { }
        }
        CHECK(gesture_dup_rejected() >= before, "Task4: dup counter is monotonic");
        CHECK(gesture_dup_rejected() >  before, "Task4: guard re-rolls on repeated targets");
    }

    if (failures) { printf("%d FAILURES\n", failures); return 1; }
    printf("ALL GESTURE TESTS PASSED\n");
    return 0;
}
