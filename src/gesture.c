#include "gesture.h"
#include "humanize.h"
#include <string.h>
#include <math.h>

#ifdef GESTURE_HOSTTEST
#define GST_FASTRUN          /* no-op under host test */
#else
/* V5F-only: .fastrun merges into .highcode (RAM_CODE/ITCM) alongside .text.*,
 * so this gives no extra locality on V5F — every function already runs from
 * zero-wait ITCM. Kept for parity with HZ_FASTRUN (humanize.c) / LINK_FASTRUN
 * (spi_link.c), and because link_v5f.ld's .fastrun collection guards against
 * the orphan-section boot bug documented there. */
#define GST_FASTRUN __attribute__((section(".fastrun")))
#endif

/* All engine state lives in this single static struct (no dynamic alloc). */
static struct {
    uint32_t nominal_us;
    gst_sample_t cap[GST_CAP_RING];
    uint16_t cap_head;   /* index where the NEXT push will write */
    uint16_t cap_count;  /* valid samples, saturates at GST_CAP_RING */
    /* ── residual store ── */
    gst_residual_t res[GST_RES_BUCKETS][GST_RES_RING];
    uint8_t  res_head[GST_RES_BUCKETS];   /* next write slot per bucket */
    uint8_t  res_n[GST_RES_BUCKETS];      /* valid samples 0..GST_RES_RING */
    uint8_t  res_read[GST_RES_BUCKETS];   /* sequential read cursor */
    /* ── streaming filter live state ── */
    float sf_hx, sf_hy;    /* EWMA heading vector (counts/report) */
    float sf_speed;        /* EWMA speed magnitude */
    float sf_debt_x, sf_debt_y; /* accumulated injected residual (for leak) */
    uint8_t sf_have;       /* EWMA initialized */
    /* ── honest-limit detector ── */
    float    nh_mag[GST_NH_WIN];   /* recent |app delta| ring */
    uint8_t  nh_head, nh_n;
    uint32_t nh_count;             /* non-human-trend events (diagnostic) */
    uint8_t  nh_human;             /* last verdict: 1 human, 0 non-human */
    float    nh_peak_real;   /* largest real human single-report |delta| this session (adaptive teleport bar) */
} G;

void gesture_init(uint32_t nominal_interval_us) {
    memset(&G, 0, sizeof(G));
    G.nominal_us = nominal_interval_us ? nominal_interval_us : 1000u;
    G.nh_human = 1;                       /* assume human until evidence otherwise (memset zeros it) */
    gesture_stream_reset();
}

GST_FASTRUN
void gesture_capture_push(int16_t dx, int16_t dy, uint32_t t_us) {
    G.cap[G.cap_head].dx = dx;
    G.cap[G.cap_head].dy = dy;
    G.cap[G.cap_head].t_us = t_us;
    G.cap_head = (uint16_t)((G.cap_head + 1u) % GST_CAP_RING);
    if (G.cap_count < GST_CAP_RING) G.cap_count++;
    float rm = sqrtf((float)dx*dx + (float)dy*dy);
    if (rm > G.nh_peak_real) G.nh_peak_real = rm;
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
    /* Output is n+1 points: the implicit origin at t=0 plus one per sample. */
    uint16_t count = (n + 1u < (uint32_t)out_cap) ? (uint16_t)(n + 1u) : out_cap;

    /* out[0] is the origin: cursor sat at (0,0) one report-interval before sample[0]. */
    out[0].x    = 0.0f;
    out[0].y    = 0.0f;
    out[0].t_us = 0u;
    out[0].f    = 0.0f;   /* will be set correctly after normalization */

    uint32_t nominal = G.nominal_us ? G.nominal_us : 1000u;
    uint32_t t0      = samples[0].t_us;

    float cx = 0.0f, cy = 0.0f;
    /* First pass: cumulative position, time, and cumulative distances. */
    float cum_dist = 0.0f;
    float prev_x = 0.0f, prev_y = 0.0f;
    for (uint16_t i = 1; i < count; i++) {
        /* sample index for out[i] is i-1 */
        cx += (float)samples[i-1].dx;
        cy += (float)samples[i-1].dy;
        out[i].x = cx;
        out[i].y = cy;
        /* First leg duration is one nominal interval; later legs use real dt. */
        out[i].t_us = nominal + (samples[i-1].t_us - t0);
        float sdx = cx - prev_x, sdy = cy - prev_y;
        cum_dist += sqrtf(sdx * sdx + sdy * sdy);
        out[i].f = cum_dist;   /* store running length; normalize below */
        prev_x = cx; prev_y = cy;
    }
    /* Second pass: normalize fraction by total path length. */
    if (cum_dist > 1e-6f) {
        float inv = 1.0f / cum_dist;
        for (uint16_t i = 0; i < count; i++) out[i].f *= inv;
        out[count - 1].f = 1.0f;   /* exact 1.0 at endpoint */
    } else {
        for (uint16_t i = 0; i < count; i++) out[i].f = 0.0f;
    }
    return count;
}

/* ── residual store: speed-bucketed real motion residual ──────────────── */

void gesture_residual_admit(uint8_t bucket, float r_par, float r_perp) {
    if (bucket >= GST_RES_BUCKETS) bucket = GST_RES_BUCKETS - 1;
    gst_residual_t *slot = &G.res[bucket][G.res_head[bucket]];
    slot->r_par = r_par; slot->r_perp = r_perp;
    G.res_head[bucket] = (uint8_t)((G.res_head[bucket] + 1u) % GST_RES_RING);
    if (G.res_n[bucket] < GST_RES_RING) G.res_n[bucket]++;
}

uint16_t gesture_residual_count(uint8_t bucket) {
    return (bucket < GST_RES_BUCKETS) ? G.res_n[bucket] : 0u;
}

uint16_t gesture_residual_total(void) {
    uint16_t t = 0;
    for (uint8_t b = 0; b < GST_RES_BUCKETS; b++) t = (uint16_t)(t + G.res_n[b]);
    return t;
}

/* Oldest-first sequential read: cursor walks the populated region in capture
 * order (preserving tremor autocorrelation) and wraps. */
static bool res_draw_one(uint8_t bucket, gst_residual_t *out) {
    uint8_t n = G.res_n[bucket];
    if (n == 0) return false;
    /* oldest sample index = head - n (mod ring); read cursor offsets from oldest */
    uint8_t oldest = (uint8_t)((G.res_head[bucket] + GST_RES_RING - n) % GST_RES_RING);
    uint8_t idx = (uint8_t)((oldest + (G.res_read[bucket] % n)) % GST_RES_RING);
    *out = G.res[bucket][idx];
    G.res_read[bucket] = (uint8_t)((G.res_read[bucket] + 1u) % n);
    return true;
}

bool gesture_residual_draw(uint8_t bucket, gst_residual_t *out) {
    if (bucket >= GST_RES_BUCKETS) bucket = GST_RES_BUCKETS - 1;
    if (res_draw_one(bucket, out)) return true;
    /* fall back to the nearest populated bucket */
    for (uint8_t d = 1; d < GST_RES_BUCKETS; d++) {
        if (bucket >= d && res_draw_one((uint8_t)(bucket - d), out)) return true;
        if (bucket + d < GST_RES_BUCKETS && res_draw_one((uint8_t)(bucket + d), out)) return true;
    }
    return false;
}

gst_warmth_t gesture_residual_warmth(void) {
    if (gesture_residual_total() < GST_RES_WARM_MIN) return GST_COLD;
    uint16_t per = GST_RES_WARM_MIN / GST_RES_BUCKETS;
    for (uint8_t b = 0; b < GST_RES_BUCKETS; b++)
        if (G.res_n[b] < per) return GST_WARMING;
    return GST_WARM;
}

/* ── residual extraction ──────────────────────────────────────────────── */

uint8_t gesture_speed_bucket(float speed_cpr) {
    if (speed_cpr < GST_RES_SLOW_MAX) return 0;
    if (speed_cpr < GST_RES_MED_MAX)  return 1;
    return 2;
}

uint16_t gesture_residual_extract(uint16_t window) {
    if (window < GST_RES_FIR + 2u) return 0;
    if (window > GST_CAP_RING)      window = GST_CAP_RING;

    /* Scratch (buf/pts/vx/vy below) is sized to GST_CAP_RING (256) though
     * production only ever extracts window=64 (~4x over-allocated, ~6KB
     * reclaimable). Intentional: keeps the GST_CAP_RING clamp above as an
     * independent overflow guard and allows a larger offline window without
     * resizing. If DTCM pressure ever rises, right-size these to a
     * GST_RES_EXTRACT_MAX constant and update that clamp to match. */
    /* 1. Pull the newest `window` samples oldest-first into a local buffer. */
    static gst_sample_t buf[GST_CAP_RING];
    uint16_t avail = gesture_capture_count();
    if (window > avail) window = avail;
    if (window < GST_RES_FIR + 2u) return 0;
    for (uint16_t i = 0; i < window; i++)
        if (!gesture_capture_get((uint16_t)(window - 1u - i), &buf[i])) return 0;

    /* 2. Sub-pixel reconstruct → continuous position + cumulative t_us. */
    static gst_point_t pts[GST_CAP_RING + 1];
    uint16_t np = gesture_reconstruct(buf, window, pts, GST_CAP_RING + 1);
    if (np < GST_RES_FIR + 2u) return 0;

    /* 3. Per-report velocity v[i] = (pos[i]-pos[i-1]) / (t[i]-t[i-1]) * nominal,
     *    expressed in counts/report (multiply by nominal so units are per-report,
     *    not per-µs — keeps r_par/r_perp in count scale for the filter). */
    static float vx[GST_CAP_RING + 1], vy[GST_CAP_RING + 1];
    uint16_t nv = (uint16_t)(np - 1u);
    float nominal = (float)(G.nominal_us ? G.nominal_us : 1000u);
    for (uint16_t i = 0; i < nv; i++) {
        float dt = (float)(pts[i+1].t_us - pts[i].t_us);
        if (dt < 1.0f) dt = 1.0f;
        vx[i] = (pts[i+1].x - pts[i].x) / dt * nominal;
        vy[i] = (pts[i+1].y - pts[i].y) / dt * nominal;
    }

    /* 4. FIR low-pass (centered moving average) → trend; residual = v − trend.
     *    5. Rotate residual into trend heading; 6. bucket by |trend|; admit. */
    uint16_t half = GST_RES_FIR / 2u;
    uint16_t admitted = 0;
    for (uint16_t i = half; i + half < nv; i++) {
        float tx = 0.0f, ty = 0.0f;
        for (uint16_t k = i - half; k <= i + half; k++) { tx += vx[k]; ty += vy[k]; }
        tx /= (float)GST_RES_FIR; ty /= (float)GST_RES_FIR;
        float ex = vx[i] - tx, ey = vy[i] - ty;          /* residual */

        float tmag = sqrtf(tx*tx + ty*ty);
        float r_par, r_perp;
        if (tmag > 1e-4f) {
            float ux = tx / tmag, uy = ty / tmag;        /* unit heading */
            r_par  = ex*ux + ey*uy;                      /* along heading */
            r_perp = -ex*uy + ey*ux;                     /* perpendicular */
        } else {
            r_par = ex; r_perp = ey;                     /* heading undefined: pass through */
        }
        uint8_t b = gesture_speed_bucket(tmag);
        gesture_residual_admit(b, r_par, r_perp);
        admitted++;
    }
    return admitted;
}

/* ── honest-limit detector ────────────────────────────────────────────── */

uint32_t gesture_nonhuman_trend(void) { return G.nh_count; }
bool     gesture_trend_is_human(void) { return G.nh_human != 0; }

GST_FASTRUN
void gesture_trend_observe(int16_t in_dx, int16_t in_dy) {
    float m = sqrtf((float)in_dx*in_dx + (float)in_dy*in_dy);
    /* Adaptive teleport bar: "the app commanded a motion larger than this human has
     * ever produced" — a real un-launderable tell — with an 80cpr floor so a fresh
     * session with little capture history doesn't over-flag. */
    float bar = (G.nh_peak_real > GST_TELEPORT_CPR) ? G.nh_peak_real : GST_TELEPORT_CPR;

    int teleport = 0;
    if (m > bar) { G.nh_count++; teleport = 1; }   /* app delta exceeds human-demonstrated peak (adaptive) */

    G.nh_mag[G.nh_head] = m;
    G.nh_head = (uint8_t)((G.nh_head + 1u) % GST_NH_WIN);
    if (G.nh_n < GST_NH_WIN) G.nh_n++;

    if (G.nh_n >= GST_NH_WIN) {
        float mean = 0.0f;
        for (uint8_t i = 0; i < GST_NH_WIN; i++) mean += G.nh_mag[i];
        mean /= (float)GST_NH_WIN;
        float var = 0.0f;
        for (uint8_t i = 0; i < GST_NH_WIN; i++) {
            float d = G.nh_mag[i] - mean; var += d*d;
        }
        var /= (float)GST_NH_WIN;
        /* Uniform-step: nonzero motion with near-zero relative variance.
         * Human aim has Fitts magnitude spread; a flat drag does not. */
        float cv = (mean > 0.5f) ? (var / (mean*mean)) : 1.0f;   /* coeff of variation^2 */
        if (mean > 0.5f && cv < 0.02f) {
            if (!teleport) G.nh_count++;   /* avoid double-count: teleport already bumped it */
            G.nh_human = 0;
        } else {
            G.nh_human = 1;
        }
    } else {
        G.nh_human = 1;   /* not enough data yet → assume human */
    }

    if (teleport) G.nh_human = 0;   /* teleport verdict wins for this call (latched over window) */
}

/* ── streaming residual filter (per-poll) ─────────────────────────────── */

void gesture_stream_reset(void) {
    G.sf_hx = G.sf_hy = G.sf_speed = 0.0f;
    G.sf_debt_x = G.sf_debt_y = 0.0f;
    G.sf_have = 0;
}

GST_FASTRUN
void gesture_stream_filter(int16_t in_dx, int16_t in_dy, int16_t *out_dx, int16_t *out_dy) {
    float ax = (float)in_dx, ay = (float)in_dy;
    float amag = sqrtf(ax*ax + ay*ay);

    /* Update EWMA heading/speed only when the app is actually moving, so a lone
     * tiny delta doesn't spin the heading. Hold last heading at rest. */
    if (amag >= GST_RES_REST_CPR) {
        float a = GST_RES_HEAD_EWMA;
        if (!G.sf_have) { G.sf_hx = ax; G.sf_hy = ay; G.sf_speed = amag; G.sf_have = 1; }
        else {
            G.sf_hx = (1.0f-a)*G.sf_hx + a*ax;
            G.sf_hy = (1.0f-a)*G.sf_hy + a*ay;
            G.sf_speed = (1.0f-a)*G.sf_speed + a*amag;
        }
    }

    /* Rest attenuation: scale residual toward 0 as the app goes idle. */
    float atten = (amag >= GST_RES_REST_CPR) ? 1.0f
                : (amag <= 0.01f) ? 0.0f : (amag / GST_RES_REST_CPR);

    float rx = 0.0f, ry = 0.0f;
    if (atten > 0.0f && gesture_residual_total() > 0) {
        gst_residual_t r;
        if (gesture_residual_draw(gesture_speed_bucket(G.sf_speed), &r)) {
            float hmag = sqrtf(G.sf_hx*G.sf_hx + G.sf_hy*G.sf_hy);
            float ux = (hmag > 1e-4f) ? G.sf_hx/hmag : 1.0f;
            float uy = (hmag > 1e-4f) ? G.sf_hy/hmag : 0.0f;
            /* rotate (r_par along heading, r_perp perpendicular) into world */
            rx = (r.r_par*ux - r.r_perp*uy) * atten;
            ry = (r.r_par*uy + r.r_perp*ux) * atten;
        }
    }

    /* Debt leak: while the app is moving, bleed back a fraction of accumulated
     * injected residual so cumulative injected motion can't drift the cursor off
     * the app's path. At rest (atten==0) decay the debt internally WITHOUT
     * emitting the correction, so an idle app receives exactly zero injected
     * motion (complement-only: no motion the app didn't ask for). */
    if (atten > 0.0f) {
        rx -= GST_RES_DEBT_LEAK * G.sf_debt_x;
        ry -= GST_RES_DEBT_LEAK * G.sf_debt_y;
        G.sf_debt_x += rx; G.sf_debt_y += ry;
    } else {
        G.sf_debt_x *= (1.0f - GST_RES_DEBT_LEAK);
        G.sf_debt_y *= (1.0f - GST_RES_DEBT_LEAK);
        rx = 0.0f; ry = 0.0f;
    }

    /* Emit app delta + residual through the sub-pixel carry quantizer. */
    humanize_inject_emit(ax + rx, ay + ry, out_dx, out_dy);
}

void gesture_human_status(gst_human_status_t *out) {
    if (!out) return;
    out->warmth = (uint8_t)gesture_residual_warmth();
    int human = gesture_trend_is_human();
    if (out->warmth == GST_WARM && human)         out->replay_pct = 100;
    else if (out->warmth == GST_WARMING && human) out->replay_pct = 50;
    else                                          out->replay_pct = 0;
}
