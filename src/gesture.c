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
    /* shared-pool FIFO library of shapes */
    gst_shape_t lib[GST_LIB_SHAPES];     /* shared 32-slot pool (~25 KB) */
    uint8_t     lib_bucket[GST_LIB_SHAPES];
    uint8_t     lib_n;                   /* valid shapes 0..GST_LIB_SHAPES */
    uint8_t     lib_head;                /* next write slot (global FIFO)  */
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

bool gesture_normalize_spatial(gst_shape_t *shape) {
    if (shape->n < 2) return false;
    float ex = shape->knots[shape->n - 1].ux;
    float ey = shape->knots[shape->n - 1].uy;
    float len = sqrtf(ex * ex + ey * ey);
    if (len < 1.0f) return false;             /* too short to define a direction */

    float ang = -atan2f(ey, ex);              /* rotate endpoint onto +X */
    float c = cosf(ang), s = sinf(ang);
    float inv = 1.0f / len;
    for (uint16_t i = 0; i < shape->n; i++) {
        float x = shape->knots[i].ux, y = shape->knots[i].uy;
        float rx = (x * c - y * s) * inv;
        float ry = (x * s + y * c) * inv;
        shape->knots[i].ux = rx;
        shape->knots[i].uy = ry;
    }
    shape->raw_len = len;
    return true;
}

/* Interpolate cumulative time (us) at path fraction f from the point array. */
static uint32_t time_at_fraction(const gst_point_t *pts, uint16_t n, float f) {
    if (n == 0) return 0;
    if (f <= pts[0].f) return pts[0].t_us;
    if (f >= pts[n-1].f) return pts[n-1].t_us;
    uint16_t i = 0;
    while (i < n - 1 && pts[i+1].f < f) i++;
    float span = pts[i+1].f - pts[i].f;
    float u = (span > 1e-9f) ? (f - pts[i].f) / span : 0.0f;
    float t = (float)pts[i].t_us + u * (float)(pts[i+1].t_us - pts[i].t_us);
    return (uint32_t)(t + 0.5f);
}

static uint16_t us_to_dtq(uint32_t us, uint32_t nominal) {
    if (nominal == 0) nominal = 1000u;
    float q = (float)us / (float)nominal * 256.0f;
    if (q < 0.0f) q = 0.0f;
    if (q > 65535.0f) q = 65535.0f;
    return (uint16_t)(q + 0.5f);
}

void gesture_normalize_temporal(gst_shape_t *shape, const gst_point_t *pts,
                                uint16_t n) {
    uint32_t prev_t = time_at_fraction(pts, n, shape->knots[0].f);
    shape->knots[0].dt_q = 0;
    for (uint16_t k = 1; k < shape->n; k++) {
        uint32_t t = time_at_fraction(pts, n, shape->knots[k].f);
        uint32_t dt = (t > prev_t) ? (t - prev_t) : 0u;
        shape->knots[k].dt_q = us_to_dtq(dt, G.nominal_us);
        prev_t = t;
    }
    uint32_t total = (n > 0) ? (pts[n-1].t_us - pts[0].t_us) : 0u;
    shape->total_us = us_to_dtq(total, G.nominal_us);
}

uint8_t gesture_length_bucket(float raw_len) {
    if (raw_len < 80.0f)  return 0;
    if (raw_len < 400.0f) return 1;
    return 2;
}

/* Count submovements as velocity-magnitude local minima between local maxima
 * along the reconstructed path (a coarse ballistic+correction proxy). */
static uint8_t count_submovements(const gst_point_t *pts, uint16_t n) {
    if (n < 3) return 1;
    uint8_t subs = 1;
    float prev_v = 0.0f; int rising = 1;
    for (uint16_t i = 1; i < n; i++) {
        float dx = pts[i].x - pts[i-1].x, dy = pts[i].y - pts[i-1].y;
        float v = sqrtf(dx*dx + dy*dy);
        if (rising && v < prev_v * 0.5f) { subs++; rising = 0; }
        else if (!rising && v > prev_v * 2.0f) { rising = 1; }
        prev_v = v;
    }
    return subs;
}

bool gesture_build_shape(const gst_sample_t *samples, uint16_t n,
                         gst_shape_t *out) {
    if (n < 4) return false;
    static gst_point_t pts[GST_CAP_RING];
    uint16_t np = gesture_reconstruct(samples, n, pts, GST_CAP_RING);
    if (np < 2) return false;

    memset(out, 0, sizeof(*out));
    out->n = gesture_resample(pts, np, out->knots);
    if (out->n == 0) return false;

    /* total path length for the min-length gate (pre-normalization). */
    float total_len = 0.0f;
    for (uint16_t i = 1; i < np; i++) {
        float dx = pts[i].x - pts[i-1].x, dy = pts[i].y - pts[i-1].y;
        total_len += sqrtf(dx*dx + dy*dy);
    }
    if (total_len < GST_MIN_LEN) return false;

    if (!gesture_normalize_spatial(out)) return false;
    gesture_normalize_temporal(out, pts, np);

    out->submv = count_submovements(pts, np);
    /* flags bit0: curvature sign from the cross product of first/last segments. */
    float c = (pts[np-1].x - pts[0].x) * (pts[1].y - pts[0].y)
            - (pts[np-1].y - pts[0].y) * (pts[1].x - pts[0].x);
    out->flags = (c < 0.0f) ? 1u : 0u;
    return true;
}

void gesture_library_admit(const gst_shape_t *shape) {
    uint8_t slot = G.lib_head;
    G.lib[slot] = *shape;
    G.lib_bucket[slot] = gesture_length_bucket(shape->raw_len);
    G.lib_head = (uint8_t)((G.lib_head + 1u) % GST_LIB_SHAPES);
    if (G.lib_n < GST_LIB_SHAPES) G.lib_n++;
}

uint8_t gesture_library_count(void) { return G.lib_n; }

gst_warmth_t gesture_warmth(void) {
    if (G.lib_n < GST_WARM_MIN) return GST_COLD;
    bool seen[GST_LEN_BUCKETS] = { false, false, false };
    for (uint8_t i = 0; i < G.lib_n; i++) seen[G.lib_bucket[i]] = true;
    if (G.lib_n >= GST_WARM_MIN && seen[0] && seen[1] && seen[2]) return GST_WARM;
    return GST_WARMING;
}

const gst_shape_t *gesture_library_select(float target_len) {
    if (G.lib_n == 0) return NULL;
    uint8_t want = gesture_length_bucket(target_len);
    const gst_shape_t *best = NULL;
    float best_d = 1e30f;
    /* First pass: same bucket. Second pass: any bucket if none in-bucket. */
    for (int pass = 0; pass < 2 && best == NULL; pass++) {
        for (uint8_t i = 0; i < G.lib_n; i++) {
            if (pass == 0 && G.lib_bucket[i] != want) continue;
            float d = fabsf(G.lib[i].raw_len - target_len);
            if (d < best_d) { best_d = d; best = &G.lib[i]; }
        }
    }
    return best;
}

bool gesture_capture_build_and_admit(uint16_t window) {
    if (window > G.cap_count) window = G.cap_count;
    if (window < 4) return false;
    static gst_sample_t buf[GST_CAP_RING];
    /* Copy newest `window` samples oldest-first into buf. */
    for (uint16_t i = 0; i < window; i++) {
        gst_sample_t s;
        gesture_capture_get((uint16_t)(window - 1u - i), &s);
        buf[i] = s;
    }
    gst_shape_t sh;
    if (!gesture_build_shape(buf, window, &sh)) return false;
    gesture_library_admit(&sh);
    return true;
}
