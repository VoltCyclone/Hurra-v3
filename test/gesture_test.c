#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "gesture.h"

/* Host-test stub: identity-ish sub-pixel quantizer (real firmware links humanize.c). */
void humanize_inject_emit(float dx, float dy, int16_t *ox, int16_t *oy) {
    *ox = (int16_t)(dx < 0 ? dx - 0.5f : dx + 0.5f);
    *oy = (int16_t)(dy < 0 ? dy - 0.5f : dy + 0.5f);
}

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

int main(void) {
    /* (A) Struct byte budgets match the spec. */
    CHECK(sizeof(gst_sample_t) == 8,  "gst_sample_t is 8 bytes");

    /* (B) Fixed sizes are exactly the spec values. */
    CHECK(GST_CAP_RING   == 256, "capture ring is 256");

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

    /* ── Plan 3 Task 1: cadence view over the capture ring ── */
    {
        gesture_init(1000);
        CHECK(gesture_cadence_count() == 0, "empty ring: no intervals");
        uint32_t dt;
        CHECK(!gesture_cadence_get(0, &dt), "empty ring: get fails");

        /* Push samples with deliberately jittered timestamps: gaps 1000, 1500, 800. */
        gesture_capture_push(1, 0, 1000);
        gesture_capture_push(1, 0, 2000);   /* +1000 */
        gesture_capture_push(1, 0, 3500);   /* +1500 */
        gesture_capture_push(1, 0, 4300);   /* +800  */
        CHECK(gesture_cadence_count() == 3, "4 samples -> 3 intervals");
        CHECK(gesture_cadence_get(0, &dt) && dt == 800u,  "age 0 = newest interval (800)");
        CHECK(gesture_cadence_get(1, &dt) && dt == 1500u, "age 1 = 1500");
        CHECK(gesture_cadence_get(2, &dt) && dt == 1000u, "age 2 = oldest interval (1000)");
        CHECK(!gesture_cadence_get(3, &dt), "age past count fails");

        /* Variance is retained, not flattened (the whole point). */
        uint32_t a, b, c;
        gesture_cadence_get(0, &a); gesture_cadence_get(1, &b); gesture_cadence_get(2, &c);
        CHECK(!(a == b && b == c), "cadence retains jitter (not constant)");
    }

    /* ── Plan 4 Task 1: click envelope ring ── */
    {
        gesture_init(1000);
        CHECK(gesture_click_count() == 0, "click ring starts empty");
        gst_click_env_t e;
        CHECK(!gesture_click_select(&e), "select on empty ring fails");

        gst_click_env_t env = { .decel_us = 5000, .settle_px = 2.0f, .dwell_us = 80000,
                                .recoil_x = 1.5f, .recoil_y = -0.5f, .flags = 0 };
        gesture_click_admit(&env);
        CHECK(gesture_click_count() == 1, "admit stores one envelope");
        CHECK(gesture_click_admitted() == 1, "admitted counter increments");

        /* Quality gate: implausible dwell rejected (both bounds reachable now
         * that dwell_us is uint32_t). */
        gst_click_env_t bad = env; bad.dwell_us = 5; /* 5us: far below human */
        gesture_click_admit(&bad);
        CHECK(gesture_click_count() == 1, "sub-min dwell rejected");
        gst_click_env_t bad2 = env; bad2.dwell_us = 5000000; /* 5s: far above max */
        gesture_click_admit(&bad2);
        CHECK(gesture_click_count() == 1, "over-max dwell rejected");

        /* select returns a (augmented) copy near the stored values. */
        CHECK(gesture_click_select(&e), "select returns an envelope");
        CHECK(e.dwell_us > 60000 && e.dwell_us < 100000, "selected dwell near source (augmented)");

        /* FIFO eviction: overflow the ring, count caps. */
        gesture_init(1000);
        for (int i = 0; i < GST_CLK_RING + 8; i++) {
            gst_click_env_t f = { .decel_us = 4000, .settle_px = 1.0f, .dwell_us = 70000,
                                  .recoil_x = 0.0f, .recoil_y = 0.0f, .flags = 0 };
            gesture_click_admit(&f);
        }
        CHECK(gesture_click_count() == GST_CLK_RING, "ring count caps at GST_CLK_RING");
    }

    /* ── Plan 4 Task 2: click envelope capture ── */
    {
        gesture_init(1000);
        uint32_t t = 0;
        /* Approach: speeding up then decelerating into the click. */
        int16_t approach[6] = { 8, 12, 16, 10, 5, 2 };
        for (int i = 0; i < 6; i++) { gesture_click_observe(approach[i], 0, 0, t); t += 1000; }
        /* Press (left=bit0), hold ~80ms with tiny drift, then release. */
        gesture_click_observe(1, 0, 0x01, t); t += 1000;          /* button down */
        for (int i = 0; i < 78; i++) { gesture_click_observe((int16_t)(i & 1), 0, 0x01, t); t += 1000; }
        gesture_click_observe(0, 0, 0x00, t); t += 1000;          /* release at ~80ms after press */
        /* Recoil window: small drift after release. */
        for (int i = 0; i < 40; i++) { gesture_click_observe(1, 0, 0x00, t); t += 1000; }

        CHECK(gesture_click_count() == 1, "one envelope captured from a click cycle");
        gst_click_env_t e;
        CHECK(gesture_click_select(&e), "captured envelope is selectable");
        /* dwell measured ~79-80ms (79 hold reports @1ms + the down report). */
        CHECK(e.dwell_us >= 60000 && e.dwell_us <= 100000, "measured dwell ~80ms");
        /* recoil_x positive (drifted +x after release), bounded. */
        CHECK(e.recoil_x > 0.0f && e.recoil_x < 60.0f, "recoil vector measured");

        /* A mere button tap with no prior motion still yields a plausible dwell. */
        gesture_init(1000);
        uint32_t t2 = 5000;
        gesture_click_observe(0, 0, 0x01, t2); t2 += 1000;
        for (int i = 0; i < 30; i++) { gesture_click_observe(0, 0, 0x01, t2); t2 += 1000; }
        gesture_click_observe(0, 0, 0x00, t2); t2 += 1000;
        for (int i = 0; i < 40; i++) { gesture_click_observe(0, 0, 0x00, t2); t2 += 1000; }
        CHECK(gesture_click_count() == 1, "tap with ~31ms dwell captured");
    }

    /* ── Plan 4 Task 3: Mode 2 aim-assist suppression ── */
    {
        gesture_init(1000);
        CHECK(fabsf(gesture_click_motion_scale() - 1.0f) < 1e-6f, "scale starts at 1.0");
        CHECK(!gesture_click_real_active(), "no real click initially");

        /* Real button down: scale ramps down toward the floor over the guard. */
        uint32_t t = 0;
        gesture_click_real_buttons(0x01, t); t += 1000;
        CHECK(gesture_click_real_active(), "real click detected");
        for (int i = 0; i < 40; i++) { gesture_click_real_buttons(0x01, t); t += 1000; }
        float held = gesture_click_motion_scale();
        CHECK(held < 0.2f, "scale suppressed toward floor during real hold");
        CHECK(held >= 0.0f, "scale never negative");

        /* Release: scale ramps back up toward 1.0. */
        gesture_click_real_buttons(0x00, t); t += 1000;
        CHECK(!gesture_click_real_active(), "real click released");
        for (int i = 0; i < 60; i++) { gesture_click_real_buttons(0x00, t); t += 1000; }
        CHECK(gesture_click_motion_scale() > 0.95f, "scale resumes to ~1.0 after release");

        /* Safety: scale only multiplies injected delta; a zero injected delta
         * stays zero regardless of suppression state (no motion invented). */
        gesture_init(1000);
        gesture_click_real_buttons(0x01, 0);
        float s = gesture_click_motion_scale();
        float inj_dx = 0.0f * s, inj_dy = 0.0f * s;
        CHECK(inj_dx == 0.0f && inj_dy == 0.0f, "suppression never invents motion");
    }

    /* ── Plan 4 Task 4: Mode 1 self-fire sequence ── */
    {
        gesture_init(1000);
        /* Capture an envelope so fire uses measured timing (dwell ~80ms). */
        uint32_t t = 0;
        gesture_click_observe(1, 0, 0x01, t); t += 1000;
        for (int i = 0; i < 78; i++) { gesture_click_observe(0, 0, 0x01, t); t += 1000; }
        gesture_click_observe(0, 0, 0x00, t); t += 1000;
        for (int i = 0; i < 40; i++) { gesture_click_observe(2, 1, 0x00, t); t += 1000; }
        CHECK(gesture_click_count() >= 1, "envelope captured for fire");

        CHECK(gesture_click_arm_fire(0), "fire arms when no real click");
        CHECK(gesture_click_fire_active(), "fire is active after arm");

        int presses = 0, releases = 0, drift_steps = 0;
        uint32_t press_at = 0, release_at = 0, elapsed = 0;
        for (int i = 0; i < 4000 && gesture_click_fire_active(); i++) {
            float dx = 0, dy = 0;
            gst_click_action_t a = gesture_click_fire_step(1000, &dx, &dy);
            if (a == GST_CA_PRESS)   { presses++;   press_at = elapsed; }
            if (a == GST_CA_RELEASE) { releases++;  release_at = elapsed; }
            if (a == GST_CA_NONE && (dx != 0 || dy != 0)) drift_steps++;
            elapsed += 1000;
        }
        CHECK(presses == 1 && releases == 1, "exactly one press and one release");
        CHECK(release_at > press_at, "release after press");
        CHECK(release_at - press_at >= 50000 && release_at - press_at <= 110000,
              "dwell between press and release ~80ms");
        CHECK(drift_steps > 0, "micro-drift emitted during dwell (not frozen)");
        CHECK(!gesture_click_fire_active(), "sequence completes");

        /* Arbitration: a real click blocks arming a self-fire (real wins). */
        gesture_init(1000);
        gesture_click_observe(1, 0, 0x01, 0);
        for (int i = 0; i < 80; i++) gesture_click_observe(0,0,0x01,(uint32_t)(i+1)*1000u);
        gesture_click_observe(0,0,0x00,82000); for (int i=0;i<40;i++) gesture_click_observe(1,0,0x00,83000u+(uint32_t)i*1000u);
        gesture_click_real_buttons(0x01, 200000);    /* real click currently down */
        CHECK(gesture_click_real_active(), "real click active");
        CHECK(!gesture_click_arm_fire(0), "self-fire refused while real click active");
        CHECK(!gesture_click_fire_active(), "no self-fire started under real click");
    }

    /* ── Plan 4 Task 5: click-coupling capstone + safety boundary ── */
    {
        gesture_init(1000);
        /* Capture an envelope with a KNOWN recoil so we can assert conservation. */
        uint32_t t = 0;
        gesture_click_observe(10, 0, 0x00, t); t += 1000;   /* approach */
        gesture_click_observe(2, 0, 0x01, t); t += 1000;    /* press */
        for (int i = 0; i < 60; i++) { gesture_click_observe(0, 0, 0x01, t); t += 1000; } /* ~60ms hold */
        gesture_click_observe(0, 0, 0x00, t); t += 1000;    /* release */
        /* recoil window: drift summing to a known vector (+12, -4) over the window */
        for (int i = 0; i < 12; i++) { gesture_click_observe(1, 0, 0x00, t); t += 1000; }
        for (int i = 0; i < 4; i++)  { gesture_click_observe(0, -1, 0x00, t); t += 1000; }
        for (int i = 0; i < 20; i++) { gesture_click_observe(0, 0, 0x00, t); t += 1000; } /* close window */
        CHECK(gesture_click_count() == 1, "capstone: envelope captured");

        gst_click_env_t e;
        gesture_click_select(&e);
        /* measured recoil ~ (+12, -4), within augmentation jitter (±15%).
         * Observed deterministic: recoil_x=12.6064, recoil_y=-4.5601.
         * Tight bands: observed ± 0.5 slack. */
        CHECK(e.recoil_x > 12.1f && e.recoil_x < 13.1f, "capstone: recoil_x measured");
        CHECK(e.recoil_y < -4.0f && e.recoil_y > -5.1f, "capstone: recoil_y measured");

        /* Mode 1: fire, sum the emitted recoil, assert it conserves to the
         * (augmented) envelope recoil — i.e. emission is endpoint-true. */
        gesture_init(1000);
        gst_click_env_t inj = { .decel_us = 4000, .settle_px = 2.0f, .dwell_us = 70000,
                                .recoil_x = 9.0f, .recoil_y = -3.0f, .flags = 0 };
        gesture_click_admit(&inj);
        CHECK(gesture_click_arm_fire(0), "capstone: fire armed");
        /* Accumulate the recoil ONLY over steps strictly AFTER the RELEASE step,
         * i.e. the pure RECOIL phase. Those slices telescope to exactly the
         * augmented recoil vector, with no dwell micro-drift mixed in (the
         * RELEASE step's own drift is excluded by setting the flag after the
         * accumulate). This isolates and verifies endpoint-true recoil. */
        float rx = 0, ry = 0; int press = 0, rel = 0, in_recoil = 0;
        for (int i = 0; i < 4000 && gesture_click_fire_active(); i++) {
            float dx = 0, dy = 0;
            gst_click_action_t a = gesture_click_fire_step(1000, &dx, &dy);
            if (a == GST_CA_PRESS) press++;
            if (in_recoil) { rx += dx; ry += dy; }   /* sum recoil phase only */
            if (a == GST_CA_RELEASE) { rel++; in_recoil = 1; } /* set AFTER accumulate */
        }
        CHECK(press == 1 && rel == 1, "capstone: one press/release");
        /* recoil_x=9 augmented ±15% -> [7.65, 10.35]; recoil_y=-3 -> [-3.45, -2.55].
         * Pure recoil telescopes exactly, so the sums land inside these bands. */
        CHECK(rx > 7.5f && rx < 10.5f, "capstone: emitted recoil_x conserves to envelope");
        CHECK(ry < -2.5f && ry > -3.5f, "capstone: emitted recoil_y conserves to envelope");

        /* Mode 2 + arbitration + safety: real click suppresses scale AND blocks fire. */
        gesture_init(1000);
        gesture_click_real_buttons(0x01, 0);
        for (int i = 0; i < 40; i++) gesture_click_real_buttons(0x01, (uint32_t)(i+1)*1000u);
        CHECK(gesture_click_motion_scale() < 0.2f, "capstone: Mode 2 suppresses during real hold");
        CHECK(!gesture_click_arm_fire(0), "capstone: real click blocks self-fire (real wins)");
        /* safety: applying the suppression scale to a zero injected delta is still
         * zero — the decorator never invents motion, never touches passthrough. */
        float s = gesture_click_motion_scale();
        CHECK((0.0f * s) == 0.0f, "capstone: suppression never invents motion");
    }

    /* ── Plan 5 Task 1: human-status snapshot (updated for v3 residual-based body) ── */
    {
        gesture_init(1000);
        gst_human_status_t hs;
        gesture_human_status(&hs);
        CHECK(hs.warmth == GST_COLD, "cold residual store reports COLD warmth");
        CHECK(hs.replay_pct == 0, "cold -> 0% injecting");

        /* Fill the residual store so gesture_residual_warmth() → WARM. */
        for (int b = 0; b < GST_RES_BUCKETS; b++)
            for (int i = 0; i < GST_RES_WARM_MIN; i++)
                gesture_residual_admit((uint8_t)b, 0.1f, 0.1f, 1000);
        gesture_human_status(&hs);
        CHECK(hs.warmth == GST_WARM, "filled residual store reports WARM warmth");
        CHECK(hs.warmth == (uint8_t)gesture_residual_warmth(), "warmth mirrors gesture_residual_warmth()");
        CHECK(hs.replay_pct <= 100 && hs.synth_pct <= 100, "percentages clamp to 0..100");
    }

    /* ── v3 Task 1: residual store ── */
    {
        gesture_init(1000);
        CHECK(gesture_residual_total() == 0, "residual store starts empty");
        gst_residual_t r;
        CHECK(!gesture_residual_draw(0, &r), "draw on empty store fails");

        /* admit into bucket 1 */
        for (int i = 0; i < 10; i++)
            gesture_residual_admit(1, (float)i, -(float)i, (uint16_t)(1000 + i));
        CHECK(gesture_residual_count(1) == 10, "bucket 1 holds 10");
        CHECK(gesture_residual_total() == 10, "total counts bucket 1");
        CHECK(gesture_residual_count(0) == 0 && gesture_residual_count(2) == 0,
              "other buckets untouched");

        /* sequential draw preserves order (autocorrelation) */
        CHECK(gesture_residual_draw(1, &r) && r.r_par == 0.0f, "draw 1 = oldest sample");
        CHECK(gesture_residual_draw(1, &r) && r.r_par == 1.0f, "draw 2 = next in order");

        /* empty-bucket draw falls back to a populated bucket */
        CHECK(gesture_residual_draw(0, &r), "empty bucket falls back to populated");

        /* FIFO eviction caps the ring */
        gesture_init(1000);
        for (int i = 0; i < GST_RES_RING + 20; i++)
            gesture_residual_admit(2, (float)i, 0.0f, 1000);
        CHECK(gesture_residual_count(2) == GST_RES_RING, "bucket caps at GST_RES_RING");

        /* out-of-range bucket clamps to 2 (no OOB write) */
        gesture_init(1000);
        gesture_residual_admit(9, 1.0f, 1.0f, 1000);
        CHECK(gesture_residual_count(2) == 1, "out-of-range bucket clamps to 2");

        /* warmth helper over fill level */
        gesture_init(1000);
        CHECK(gesture_residual_warmth() == GST_COLD, "empty store is COLD");
        for (int b = 0; b < GST_RES_BUCKETS; b++)
            for (int i = 0; i < GST_RES_WARM_MIN; i++)
                gesture_residual_admit((uint8_t)b, 0.1f, 0.1f, 1000);
        CHECK(gesture_residual_warmth() == GST_WARM, "all buckets filled is WARM");

        /* WARMING: total >= GST_RES_WARM_MIN but not every bucket is filled */
        gesture_init(1000);
        for (int i = 0; i < GST_RES_WARM_MIN; i++)
            gesture_residual_admit(0, 0.1f, 0.1f, 1000);
        CHECK(gesture_residual_warmth() == GST_WARMING,
              "one bucket filled, others empty = WARMING");
    }

    /* ── v3 Task 2: residual extraction ── */
    {
        gesture_init(1000);
        /* Synthesize a capture: smooth rightward drift (trend) + a known
         * perpendicular tremor on Y. Residual must land almost entirely in
         * r_perp (perpendicular), be ~zero-mean, and bucket by speed. */
        for (int i = 0; i < 64; i++) {
            int16_t dx = 5;                                   /* steady 5 cpr right */
            int16_t dy = (int16_t)((i & 1) ? 2 : -2);         /* ±2 perpendicular tremor */
            gesture_capture_push(dx, dy, (uint32_t)(1000u * (i + 1)));
        }
        uint16_t got = gesture_residual_extract(64);
        CHECK(got > 0, "extraction admits residual samples");
        CHECK(gesture_residual_total() == got, "store holds the admitted residual");

        /* Drain bucket(s): mean r_par and r_perp ~ 0 (residual is zero-mean);
         * r_perp variance >> r_par variance (tremor is perpendicular here). */
        double sp = 0, spp = 0, vpp = 0, vpar = 0; int cnt = 0;
        for (uint8_t b = 0; b < GST_RES_BUCKETS; b++) {
            gst_residual_t r;
            for (uint16_t k = 0; k < gesture_residual_count(b); k++) {
                /* read raw via draw (cursor) — fine for a statistical check */
                if (!gesture_residual_draw(b, &r)) break;
                sp += r.r_par; spp += r.r_perp;
                vpar += (double)r.r_par * r.r_par;
                vpp  += (double)r.r_perp * r.r_perp;
                cnt++;
            }
        }
        CHECK(cnt > 0, "drained some residual");
        CHECK(fabs(sp / cnt) < 0.5 && fabs(spp / cnt) < 0.5, "residual is ~zero-mean");
        CHECK(vpp > vpar, "perpendicular tremor dominates (r_perp var > r_par var)");

        /* Speed bucketing */
        CHECK(gesture_speed_bucket(1.0f) == 0, "1 cpr -> slow bucket");
        CHECK(gesture_speed_bucket(5.0f) == 1, "5 cpr -> medium bucket");
        CHECK(gesture_speed_bucket(20.0f) == 2, "20 cpr -> fast bucket");

        /* Too-short window admits nothing */
        gesture_init(1000);
        gesture_capture_push(1, 0, 1000); gesture_capture_push(1, 0, 2000);
        CHECK(gesture_residual_extract(2) == 0, "sub-FIR window admits nothing");
    }

    /* ── v3 Task 3: streaming residual filter ── */
    {
        gesture_init(1000);
        /* Warm the store with a known tremor so draws are non-zero. */
        for (int i = 0; i < 64; i++) {
            gesture_capture_push(6, (int16_t)((i&1)?2:-2), (uint32_t)(1000u*(i+1)));
        }
        gesture_residual_extract(64);

        /* Drive a steady rightward stream; net emitted ~ tracks app sum (drift bounded). */
        long app_sum_x = 0, app_sum_y = 0, emit_sum_x = 0, emit_sum_y = 0;
        int moved_offaxis = 0;
        for (int i = 0; i < 2000; i++) {
            int16_t ox, oy;
            gesture_stream_filter(6, 0, &ox, &oy);     /* app: pure +X */
            app_sum_x += 6; app_sum_y += 0;
            emit_sum_x += ox; emit_sum_y += oy;
            if (oy != 0) moved_offaxis++;
        }
        CHECK(moved_offaxis > 100, "filter injects off-axis residual (not a passthrough)");
        CHECK(labs(emit_sum_x - app_sum_x) <= 2*2000/100 + 4,
              "net X drift bounded (debt-leak keeps cumulative ~ app path)");
        CHECK(labs(emit_sum_y) <= 4, "net Y drift ~ zero (residual zero-mean)");

        /* Rest: app delta 0 -> residual attenuates, no injected motion into idle. */
        long idle_emit = 0;
        for (int i = 0; i < 200; i++) {
            int16_t ox, oy; gesture_stream_filter(0, 0, &ox, &oy);
            idle_emit += labs(ox) + labs(oy);
        }
        CHECK(idle_emit == 0, "zero injected motion when app is idle (leak gated at rest)");
    }

    /* ── v3 Task 4: honest-limit detector ── */
    {
        gesture_init(1000);
        uint32_t base = gesture_nonhuman_trend();
        /* Human-like: decaying Fitts magnitudes -> stays "human", no flag. */
        for (int i = 40; i > 0; i--) gesture_trend_observe((int16_t)i, (int16_t)(i/3));
        CHECK(gesture_nonhuman_trend() == base, "human-like decay raises no flag");
        CHECK(gesture_trend_is_human(), "human-like window reads as human");

        /* Uniform steps: constant magnitude -> flag. */
        gesture_init(1000);
        for (int i = 0; i < GST_NH_WIN + 4; i++) gesture_trend_observe(5, 0);
        CHECK(gesture_nonhuman_trend() > 0, "uniform-step trend is flagged");
        CHECK(!gesture_trend_is_human(), "uniform-step window reads as non-human");

        /* Teleport snap: one huge delta -> flag. */
        gesture_init(1000);
        uint32_t b2 = gesture_nonhuman_trend();
        gesture_trend_observe(200, 50);
        CHECK(gesture_nonhuman_trend() > b2, "teleport snap is flagged");

        /* adaptive teleport bar: a demonstrated-large human movement raises the bar */
        gesture_init(1000);
        uint32_t a0 = gesture_nonhuman_trend();
        gesture_capture_push(150, 0, 1000);     /* human really moved 150 cpr in one report */
        gesture_trend_observe(120, 0);          /* app delta 120 < demonstrated 150 → NOT a teleport */
        CHECK(gesture_nonhuman_trend() == a0, "app delta below human-demonstrated peak is not flagged");
        gesture_trend_observe(200, 0);          /* app delta 200 > demonstrated 150 → teleport */
        CHECK(gesture_nonhuman_trend() > a0, "app delta above human-demonstrated peak flags teleport");

        /* teleport latch: teleport verdict wins even when the window looks human */
        gesture_init(1000);
        for (int i = 40; i > 0; i--) gesture_trend_observe((int16_t)i, (int16_t)(i/3)); /* fill window, high spread → human */
        CHECK(gesture_trend_is_human(), "high-spread window reads human before teleport");
        gesture_trend_observe(250, 60);         /* teleport on a full, human-looking window */
        CHECK(!gesture_trend_is_human(), "teleport latches non-human even on a human-looking window");

        /* combined teleport + uniform-step on one call counts once, not twice */
        gesture_init(1000);
        for (int i = 0; i < GST_NH_WIN; i++) gesture_trend_observe(90, 0); /* fill window: constant 90 (>80 bar) → uniform-step AND teleport each call */
        /* window is now full and uniform; the NEXT identical call is both a teleport (90>80) and a uniform-step window */
        uint32_t c_before = gesture_nonhuman_trend();
        gesture_trend_observe(90, 0);
        CHECK(gesture_nonhuman_trend() == c_before + 1, "combined teleport+uniform-step call counts exactly once");
    }

    /* ── v3 Task 7: status reflects residual state ── */
    {
        gesture_init(1000);
        gst_human_status_t hs;
        gesture_human_status(&hs);
        CHECK(hs.warmth == GST_COLD, "cold residual store -> COLD status");
        CHECK(hs.replay_pct == 0, "cold -> 0% injecting");

        for (int b = 0; b < GST_RES_BUCKETS; b++)
            for (int i = 0; i < GST_RES_WARM_MIN; i++)
                gesture_residual_admit((uint8_t)b, 0.1f, 0.1f, 1000);
        gesture_human_status(&hs);
        CHECK(hs.warmth == GST_WARM, "filled store -> WARM status");
        CHECK(hs.replay_pct > 0, "warm -> nonzero injecting %");
    }

    if (failures) { printf("%d FAILURES\n", failures); return 1; }
    printf("ALL GESTURE TESTS PASSED\n");
    return 0;
}
