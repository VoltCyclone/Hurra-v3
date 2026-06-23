#include "gesture.h"
#include <string.h>
#include <math.h>

#ifdef GESTURE_HOSTTEST
#define GST_FASTRUN
#else
#define GST_FASTRUN __attribute__((section(".fastrun")))
#endif

/* All engine state lives in this single static struct (no dynamic alloc). */
static struct {
    uint32_t nominal_us;
    gst_sample_t cap[GST_CAP_RING];
    uint16_t cap_head;   /* index where the NEXT push will write */
    uint16_t cap_count;  /* valid samples, saturates at GST_CAP_RING */
} G;

void gesture_init(uint32_t nominal_interval_us) {
    memset(&G, 0, sizeof(G));
    G.nominal_us = nominal_interval_us ? nominal_interval_us : 1000u;
}

GST_FASTRUN
void gesture_capture_push(int16_t dx, int16_t dy, uint32_t t_us) {
    G.cap[G.cap_head].dx = dx;
    G.cap[G.cap_head].dy = dy;
    G.cap[G.cap_head].t_us = t_us;
    G.cap_head = (uint16_t)((G.cap_head + 1u) % GST_CAP_RING);
    if (G.cap_count < GST_CAP_RING) G.cap_count++;
}

uint16_t gesture_capture_count(void) { return G.cap_count; }

bool gesture_capture_get(uint16_t age, gst_sample_t *out) {
    if (age >= G.cap_count) return false;
    /* head points one past the newest; newest is head-1. */
    uint16_t idx = (uint16_t)((G.cap_head + GST_CAP_RING - 1u - age) % GST_CAP_RING);
    *out = G.cap[idx];
    return true;
}

uint16_t gesture_reconstruct(const gst_sample_t *samples, uint16_t n,
                             gst_point_t *out, uint16_t out_cap) {
    if (n == 0 || out_cap == 0) return 0;
    uint16_t count = (n < out_cap) ? n : out_cap;

    float cx = 0.0f, cy = 0.0f;
    uint32_t t0 = samples[0].t_us;
    /* First pass: cumulative position, time, and cumulative distances. */
    float cum_dist = 0.0f;
    float prev_x = 0.0f, prev_y = 0.0f;
    for (uint16_t i = 0; i < count; i++) {
        cx += (float)samples[i].dx;
        cy += (float)samples[i].dy;
        out[i].x = cx;
        out[i].y = cy;
        out[i].t_us = samples[i].t_us - t0;   /* unsigned, monotonic capture */
        float sdx = cx - prev_x, sdy = cy - prev_y;
        cum_dist += sqrtf(sdx * sdx + sdy * sdy);
        out[i].f = cum_dist;                   /* store cumulative distance; normalize below */
        prev_x = cx; prev_y = cy;
    }
    /* Second pass: normalize fraction by total path length. */
    if (cum_dist > 1e-6f) {
        for (uint16_t i = 0; i < count; i++) out[i].f /= cum_dist;
    } else {
        for (uint16_t i = 0; i < count; i++) out[i].f = 0.0f;
    }
    return count;
}

/* Centripetal Catmull-Rom position at parameter u in [0,1] across p1..p2,
 * with p0/p3 the neighbours. Falls back to linear at the ends. */
static void catmull_rom(float p0x, float p0y, float p1x, float p1y,
                        float p2x, float p2y, float p3x, float p3y,
                        float u, float *ox, float *oy) {
    float u2 = u * u, u3 = u2 * u;
    /* Standard Catmull-Rom basis (tau = 0.5). */
    *ox = 0.5f * ((2.0f*p1x) + (-p0x + p2x)*u +
                  (2.0f*p0x - 5.0f*p1x + 4.0f*p2x - p3x)*u2 +
                  (-p0x + 3.0f*p1x - 3.0f*p2x + p3x)*u3);
    *oy = 0.5f * ((2.0f*p1y) + (-p0y + p2y)*u +
                  (2.0f*p0y - 5.0f*p1y + 4.0f*p2y - p3y)*u2 +
                  (-p0y + 3.0f*p1y - 3.0f*p2y + p3y)*u3);
}

uint16_t gesture_resample(const gst_point_t *pts, uint16_t n, gst_knot_t *out) {
    if (n < 2) return 0;
    for (uint16_t k = 0; k < GST_KNOTS_MAX; k++) {
        float target_f = (float)k / (float)(GST_KNOTS_MAX - 1);
        out[k].f = target_f;
        out[k].dt_q = 0;                       /* filled by gesture_normalize */

        /* Find the segment [i, i+1] whose fraction range contains target_f. */
        uint16_t i = 0;
        while (i < n - 1 && pts[i + 1].f < target_f) i++;
        if (i >= n - 1) {                      /* clamp to endpoint */
            out[k].ux = pts[n - 1].x;
            out[k].uy = pts[n - 1].y;
            continue;
        }
        float f0 = pts[i].f, f1 = pts[i + 1].f;
        float span = f1 - f0;
        float u = (span > 1e-9f) ? (target_f - f0) / span : 0.0f;
        /* Clamp u into [0,1]. gesture_reconstruct counts the implicit origin
         * (0,0)->first-sample arc, so pts[0].f > 0; without this, knots whose
         * target_f < pts[0].f would make Catmull-Rom extrapolate backward. */
        if (u < 0.0f) u = 0.0f;
        if (u > 1.0f) u = 1.0f;

        uint16_t i0 = (i > 0) ? i - 1 : i;
        uint16_t i3 = (i + 2 < n) ? i + 2 : i + 1;
        catmull_rom(pts[i0].x, pts[i0].y, pts[i].x, pts[i].y,
                    pts[i+1].x, pts[i+1].y, pts[i3].x, pts[i3].y,
                    u, &out[k].ux, &out[k].uy);
    }
    /* Force exact endpoints (Catmull-Rom interpolates control points, but the
     * fraction-search clamp above already guarantees this; assert by overwrite). */
    out[0].ux = pts[0].x; out[0].uy = pts[0].y;
    out[GST_KNOTS_MAX-1].ux = pts[n-1].x; out[GST_KNOTS_MAX-1].uy = pts[n-1].y;
    return GST_KNOTS_MAX;
}
