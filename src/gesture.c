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
