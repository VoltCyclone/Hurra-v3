// src/gesture_motion_v5f.c — V5F-only bridge from the act_motion_source_t hook
// to the gesture replay engine. Lives on V5F (host) where TIM9 (1 MHz) and live
// capture exist; never linked on V3F. Registered via act_motion_set_source() at
// host init so km.move/bezier emit gesture-sourced trajectories. Emits integer
// deltas via humanize_inject_emit (quantize + sub-pixel carry, noise=0).
#include "actions.h"
#include "gesture.h"
#include "humanize.h"
#include <stdint.h>
#include <stdbool.h>

extern uint32_t timebase_v5f_us(void);

static uint32_t s_last_us;
static bool     s_running;     /* gesture source still producing steps */

static void gm_begin(int32_t dx, int32_t dy, uint16_t dur_ms, int bezier,
                     int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
    (void)dur_ms; (void)bezier; (void)x1; (void)y1; (void)x2; (void)y2;
    /* Gesture supplies the curve + cadence; only gross (dx,dy) is honored. A
     * self-initiated aim move is SILENT (self-paced), not ride-along. */
    gesture_motion_begin(dx, dy, MOTION_MODE_SILENT);
    s_last_us = timebase_v5f_us();
    s_running = !gesture_motion_done();
}

static int gm_tick(int16_t *out_dx, int16_t *out_dy)
{
    uint32_t now = timebase_v5f_us();
    uint32_t dt  = now - s_last_us;          /* unsigned wrap-safe */
    s_last_us = now;

    float fx = 0.0f, fy = 0.0f;
    if (s_running) {
        gesture_motion_pace_advance(dt);
        float sdx, sdy; uint16_t dq;
        while (gesture_motion_pace_take(&sdx, &sdy, &dq)) { fx += sdx; fy += sdy; }
        if (gesture_motion_done()) s_running = false;
    }

    int16_t qx = 0, qy = 0;
    humanize_inject_emit(fx, fy, &qx, &qy);  /* quantize + carry sub-pixel residual */
    *out_dx = qx; *out_dy = qy;

    /* Stay engaged while the source runs OR the quantizer still owes whole
     * counts to flush. (Sub-pixel residual < 1 count does not keep us alive —
     * it is carried into the next motion. Bench-verify termination.) */
    return s_running || humanize_pending();
}

static void gm_cancel(void) { s_running = false; }

const act_motion_source_t gesture_motion_v5f_source = {
    .begin = gm_begin, .tick = gm_tick, .cancel = gm_cancel,
};
