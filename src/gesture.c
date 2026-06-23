#include "gesture.h"
#include <string.h>
#include <math.h>

#ifdef GESTURE_HOSTTEST
#define GST_FASTRUN
#else
#define GST_FASTRUN __attribute__((section(".fastrun")))
#endif

#define GST_SRC_NONE   0u
#define GST_SRC_REPLAY 1u
#define GST_SRC_SYNTH  2u

/* ── replay augmentation tunables ── */
#define GST_AUG_SCALE_JIT  0.04f      /* ± fractional scale jitter        */
#define GST_AUG_ROT_JIT    0.02618f   /* ± rotation jitter, rad (~1.5°)   */
#define GST_AUG_MORPH_LO   0.30f      /* morph weight on A, lower bound   */
#define GST_AUG_MORPH_HI   0.70f      /* morph weight on A, upper bound   */
#define GST_AUG_TWARP      0.08f      /* ± per-knot dt random-walk bound  */
#define GST_DUP_WINDOW     8          /* recent tuple hashes tracked       */
#define GST_CAD_DROP_MULT  4u         /* clamp a captured dt above this*nominal (dropout) */

#define GST_CLK_RECOIL_WIN_US 30000u   /* drift window measured after release */
#define GST_CLK_PEAK_DECAY    0.85f    /* per-report decay of the speed peak  */
#define GST_C2_FLOOR     0.05f     /* injected-motion floor during real hold (micro-drift) */
#define GST_C2_GUARD_US  15000.0f  /* ramp-down time on real button-down  */
#define GST_C2_RESUME_US 25000.0f  /* ramp-up time after real release      */
/* capture state-machine phases */
#define GST_CC_IDLE   0u
#define GST_CC_HOLD   1u
#define GST_CC_RECOIL 2u

#ifdef GESTURE_HOSTTEST
static uint32_t gst_hw_entropy(void) { return 0x12345678u; }   /* deterministic */
#else
extern uint32_t timebase_v5f_us(void);   /* TIM9 1 MHz, started pre-gesture */
static uint32_t gst_hw_entropy(void) {
    uint32_t cyc = timebase_v5f_us();
    uint32_t uid = *(volatile uint32_t *)0x1FFFF704;   /* WCH device-info word */
    return cyc ^ uid;
}
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
    /* ── PRNG (SFC32), seeded in gesture_init ── */
    uint32_t rng_a, rng_b, rng_c, rng_ctr;
    /* ── active motion source (one in-flight gesture) ── */
    uint8_t    src_kind;                  /* GST_SRC_NONE/REPLAY/SYNTH */
    gst_knot_t work[GST_KNOTS_MAX];       /* transformed working copy  */
    uint16_t   work_n;                    /* knots in the working copy */
    uint16_t   work_cursor;               /* next knot index to emit   */
    float      emit_px, emit_py;          /* last emitted absolute pos */
    uint32_t   pace_budget_us;             /* silent-path pacing time budget */
    /* ── repetition guard ── */
    uint32_t dup_ring[8];                 /* GST_DUP_WINDOW recent tuple hashes */
    uint8_t  dup_head;
    uint8_t  dup_n;   /* populated ring slots, saturates at GST_DUP_WINDOW */
    uint32_t dup_rejected;                /* diagnostic */
    /* ── source diagnostics ── */
    uint32_t replay_count, synth_fallback_count, bucket_miss;
    /* ── click envelope ring (Plan 4) ── */
    gst_click_env_t clk[GST_CLK_RING];
    uint8_t  clk_head;                    /* next write slot (FIFO)        */
    uint8_t  clk_n;                       /* valid envelopes 0..GST_CLK_RING */
    uint32_t clk_admitted;                /* diagnostic                    */
    /* ── click capture state machine (Plan 4) ── */
    uint8_t  cc_state;                    /* GST_CC_IDLE/HOLD/RECOIL        */
    uint8_t  cc_prev_buttons;            /* last observed button bits      */
    uint8_t  cc_button;                  /* latched button index this cycle */
    uint32_t cc_press_t;                 /* timestamp of button-down       */
    float    cc_settle_px;               /* accumulated drift during hold  */
    float    cc_recoil_x, cc_recoil_y;   /* accumulated drift after release */
    uint32_t cc_recoil_until;            /* end of the recoil window       */
    float    cc_peak_speed;              /* recent rolling speed peak      */
    uint32_t cc_peak_t;                  /* timestamp of that peak         */
    float    cc_peak_at_press;           /* peak speed latched at press (for vclass) */
    uint32_t cc_decel_us;               /* peak_t→press, latched at press */
    uint32_t cc_dwell_us;               /* press→release, latched at release */
    /* ── Mode 2 aim-assist (Plan 4) ── */
    uint8_t  c2_prev_buttons;            /* last real button bits          */
    bool     c2_real_down;               /* a real button is currently held */
    float    c2_scale;                   /* current injected-motion scale  */
    uint32_t c2_last_t;                  /* last real-report timestamp     */
    bool     c2_have_t;                  /* c2_last_t valid                */
} G;

static inline uint32_t gst_sfc32(void) {
    uint32_t t = G.rng_a + G.rng_b + G.rng_ctr++;
    G.rng_a = G.rng_b ^ (G.rng_b >> 9);
    G.rng_b = G.rng_c + (G.rng_c << 3);
    G.rng_c = ((G.rng_c << 21) | (G.rng_c >> 11)) + t;
    return t;
}

void gesture_init(uint32_t nominal_interval_us) {
    memset(&G, 0, sizeof(G));
    G.nominal_us = nominal_interval_us ? nominal_interval_us : 1000u;
    uint32_t seed = gst_hw_entropy();
    G.rng_a = seed ^ 0xCAFEBABEu; G.rng_b = seed ^ 0xDEADBEEFu;
    G.rng_c = seed ^ 0x8BADF00Du; G.rng_ctr = 1u;
    if (!G.rng_a) G.rng_a = 0xCAFEBABEu;   /* only a is guarded; warm-up restores liveness */
    for (int i = 0; i < 16; i++) (void)gst_sfc32();
    G.c2_scale = 1.0f;                    /* required: memset zeroed it; 0 would suppress all injection */
}

uint32_t gesture_rand_u32(void) { return gst_sfc32(); }

float gesture_rand_range(float lo, float hi) {
    /* top 24 bits → [0,1), then affine to [lo,hi). */
    float u = (float)(gst_sfc32() >> 8) * (1.0f / 16777216.0f);
    return lo + u * (hi - lo);
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

uint16_t gesture_cadence_count(void) {
    uint16_t cc = gesture_capture_count();
    return (cc >= 2u) ? (uint16_t)(cc - 1u) : 0u;
}

bool gesture_cadence_get(uint16_t age, uint32_t *out_dt_us) {
    if (age >= gesture_cadence_count()) return false;
    gst_sample_t newer, older;
    /* interval[age] spans capture samples (age) and (age+1); both valid since
     * age+1 <= capture_count-1. t_us is monotonic over a single ring window. */
    if (!gesture_capture_get(age, &newer))      return false;
    if (!gesture_capture_get((uint16_t)(age + 1u), &older)) return false;
    *out_dt_us = newer.t_us - older.t_us;       /* unsigned, single-wrap safe */
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
        /* Clamp u into [0,1] as defensive safety against fraction rounding at
         * the segment edges (gesture_reconstruct prepends the origin so
         * pts[0].f == 0 and the search normally yields u in range). */
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

/* Real replay interval for a step, from its dt_q (.8 fixed of nominal).
 * interval_us = dt_q * nominal_us / 256. Max 65535*~10000>>8 fits uint32. */
static uint32_t dtq_to_us(uint16_t dt_q) {
    uint32_t nominal = G.nominal_us ? G.nominal_us : 1000u;
    return (uint32_t)(((uint64_t)dt_q * (uint64_t)nominal) >> 8);
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
    {
        uint32_t nominal = G.nominal_us ? G.nominal_us : 1000u;
        float tq = (float)total / (float)nominal * 256.0f;
        if (tq < 0.0f) tq = 0.0f;
        if (tq > 4294967040.0f) tq = 4294967040.0f;   /* < UINT32_MAX, safe cast */
        shape->total_us = (uint32_t)(tq + 0.5f);
    }
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
    static gst_point_t pts[GST_CAP_RING + 1];
    uint16_t np = gesture_reconstruct(samples, n, pts, GST_CAP_RING + 1);
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

void gesture_click_admit(const gst_click_env_t *env) {
    if (env->dwell_us < GST_CLK_DWELL_MIN_US || env->dwell_us > GST_CLK_DWELL_MAX_US)
        return;                           /* quality gate: implausible dwell */
    G.clk[G.clk_head] = *env;
    G.clk_head = (uint8_t)((G.clk_head + 1u) % GST_CLK_RING);
    if (G.clk_n < GST_CLK_RING) G.clk_n++;
    G.clk_admitted++;
}

uint8_t gesture_click_count(void) { return G.clk_n; }

uint32_t gesture_click_admitted(void) { return G.clk_admitted; }

bool gesture_click_select(gst_click_env_t *out) {
    if (G.clk_n == 0) return false;
    uint8_t idx = (uint8_t)(gst_sfc32() % G.clk_n);
    gst_click_env_t e = G.clk[idx];
    /* Augment with bounded jitter so replays are not identical. dwell_us is
     * uint32, so both gate bounds are representable; clamp the jittered value
     * back into the plausible window. */
    float dw = (float)e.dwell_us * (1.0f + gesture_rand_range(-0.10f, 0.10f));
    if (dw < (float)GST_CLK_DWELL_MIN_US) dw = (float)GST_CLK_DWELL_MIN_US;
    if (dw > (float)GST_CLK_DWELL_MAX_US) dw = (float)GST_CLK_DWELL_MAX_US;
    e.dwell_us  = (uint32_t)dw;
    e.settle_px = e.settle_px * (1.0f + gesture_rand_range(-0.10f, 0.10f));
    e.recoil_x  = e.recoil_x  * (1.0f + gesture_rand_range(-0.15f, 0.15f));
    e.recoil_y  = e.recoil_y  * (1.0f + gesture_rand_range(-0.15f, 0.15f));
    *out = e;
    return true;
}

/* Lowest set button bit → index 0..2 (left/right/middle), else 0. */
static uint8_t clk_button_index(uint8_t buttons) {
    if (buttons & 0x01u) return 0u;
    if (buttons & 0x02u) return 1u;
    if (buttons & 0x04u) return 2u;
    return 0u;
}

GST_FASTRUN
void gesture_click_observe(int16_t dx, int16_t dy, uint8_t buttons, uint32_t t_us) {
    float speed = sqrtf((float)dx * (float)dx + (float)dy * (float)dy);

    /* Rolling speed peak (for decel_us): instantaneous attack, decayed memory. */
    if (speed >= G.cc_peak_speed) { G.cc_peak_speed = speed; G.cc_peak_t = t_us; }
    else { G.cc_peak_speed *= GST_CLK_PEAK_DECAY; }

    bool down_now = (buttons != 0u);
    bool was_down = (G.cc_prev_buttons != 0u);

    if (!was_down && down_now) {
        /* press edge */
        G.cc_button = clk_button_index(buttons);
        G.cc_press_t = t_us;
        G.cc_decel_us = (t_us >= G.cc_peak_t) ? (t_us - G.cc_peak_t) : 0u;
        G.cc_peak_at_press = G.cc_peak_speed;   /* latch peak NOW; it decays during the hold */
        G.cc_settle_px = 0.0f;
        G.cc_state = GST_CC_HOLD;
    } else if (was_down && down_now && G.cc_state == GST_CC_HOLD) {
        /* hold: accumulate residual drift */
        G.cc_settle_px += speed;
    } else if (was_down && !down_now && G.cc_state == GST_CC_HOLD) {
        /* release edge: measure dwell, open the recoil window */
        uint32_t dwell = (t_us >= G.cc_press_t) ? (t_us - G.cc_press_t) : 0u;
        if (dwell >= GST_CLK_DWELL_MIN_US && dwell <= GST_CLK_DWELL_MAX_US) {
            G.cc_dwell_us = dwell;
            G.cc_recoil_x = 0.0f; G.cc_recoil_y = 0.0f;
            G.cc_recoil_until = t_us + GST_CLK_RECOIL_WIN_US;
            G.cc_state = GST_CC_RECOIL;
        } else {
            G.cc_state = GST_CC_IDLE;         /* implausible: drop this cycle */
        }
    } else if (G.cc_state == GST_CC_RECOIL) {
        /* accumulate post-release drift until the window closes, then admit */
        if (t_us < G.cc_recoil_until) {
            G.cc_recoil_x += (float)dx;
            G.cc_recoil_y += (float)dy;
        } else {
            /* closing report's own dx/dy intentionally not accumulated:
             * recoil sum covers reports strictly before recoil_until */
            gst_click_env_t env;
            env.decel_us  = G.cc_decel_us;
            env.settle_px = G.cc_settle_px;
            env.dwell_us  = G.cc_dwell_us;    /* uint32: no narrowing needed */
            env.recoil_x  = G.cc_recoil_x;
            env.recoil_y  = G.cc_recoil_y;
            uint8_t vclass = (G.cc_peak_at_press < 5.0f) ? 0u : (G.cc_peak_at_press < 20.0f ? 1u : 2u);
            env.flags = (uint8_t)((vclass & 0x03u) | ((G.cc_button & 0x03u) << 2));
            gesture_click_admit(&env);
            G.cc_state = GST_CC_IDLE;
        }
    }

    G.cc_prev_buttons = buttons;
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

/* Pick a same-bucket shape other than `primary`, uniformly at random (reservoir).
 * NULL when the bucket holds only the primary. */
static const gst_shape_t *library_morph_partner(const gst_shape_t *primary,
                                                float target_len) {
    uint8_t want = gesture_length_bucket(target_len);
    const gst_shape_t *pick = NULL; uint32_t seen = 0;
    for (uint8_t i = 0; i < G.lib_n; i++) {
        if (G.lib_bucket[i] != want) continue;
        if (&G.lib[i] == primary)    continue;
        seen++;
        if ((gst_sfc32() % seen) == 0u) pick = &G.lib[i];
    }
    return pick;
}

/* Force work[n-1] to exactly (tx,ty); fold half the correction into the
 * penultimate knot so the final step is a small corrective submovement.
 * Precondition: G.work_n >= 1 (in practice always GST_KNOTS_MAX, since every
 * source fills a full 48-knot working copy). Callers must not invoke this with
 * G.work_n == 0 — `last` would underflow. Both current callers (replay_begin,
 * synth_begin) guarantee work_n > 0 before calling; a future caller (Plan 3/4)
 * reusing this helper must uphold the same. */
static void endpoint_true(int32_t tx, int32_t ty) {
    uint16_t last = (uint16_t)(G.work_n - 1u);
    float ex = (float)tx - G.work[last].ux;
    float ey = (float)ty - G.work[last].uy;
    if (G.work_n >= 3) {
        G.work[last - 1].ux += ex * 0.5f;
        G.work[last - 1].uy += ey * 0.5f;
    }
    G.work[last].ux = (float)tx;
    G.work[last].uy = (float)ty;
}

/* Quantize (shapes, angle, scale) into a coarse bucket key — close transforms
 * collide so the guard rejects perceptually-similar repeats, not just exact ones. */
static uint32_t replay_tuple_hash(uint8_t a_slot, uint8_t b_slot, float theta, float s) {
    int ti = (int)(theta * (180.0f / 3.14159265f) / 3.0f);   /* 3° buckets */
    int si = (int)(s / 8.0f);                                /* 8-count buckets */
    uint32_t h = (uint32_t)a_slot * 2654435761u;
    h ^= (uint32_t)(b_slot + 1u) * 40503u;
    h ^= (uint32_t)(ti & 0xffff) << 8;
    h ^= (uint32_t)(si & 0xff);
    return h;
}
static bool dup_seen(uint32_t h) {
    for (uint8_t i = 0; i < G.dup_n; i++) if (G.dup_ring[i] == h) return true;
    return false;
}
static void dup_record(uint32_t h) {
    G.dup_ring[G.dup_head] = h;
    G.dup_head = (uint8_t)((G.dup_head + 1u) % GST_DUP_WINDOW);
    if (G.dup_n < GST_DUP_WINDOW) G.dup_n++;
}

uint32_t gesture_dup_rejected(void) { return G.dup_rejected; }

/* Materialize an augmented working copy: optional morph-blend with a same-bucket
 * partner, ±8% per-knot time-warp, scale + rotation jitter, then endpoint trueing
 * so Σ emitted == V exactly. */
static bool replay_begin(int32_t tx, int32_t ty) {
    float Vx = (float)tx, Vy = (float)ty;
    float R = sqrtf(Vx * Vx + Vy * Vy);
    if (R < 1.0f) return false;
    const gst_shape_t *A = gesture_library_select(R);
    if (!A) return false;
    uint8_t a_slot = (uint8_t)(A - G.lib);
    float theta0 = atan2f(Vy, Vx);

    /* Choose augmentation params, re-rolling away from recent near-duplicates. */
    const gst_shape_t *B = NULL;
    float alpha = 1.0f, theta = theta0, s = R;
    uint32_t h = 0;
    for (int attempt = 0; attempt < 4; attempt++) {
        B     = library_morph_partner(A, R);
        alpha = B ? gesture_rand_range(GST_AUG_MORPH_LO, GST_AUG_MORPH_HI) : 1.0f;
        theta = theta0 + gesture_rand_range(-GST_AUG_ROT_JIT, GST_AUG_ROT_JIT);
        s     = R * (1.0f + gesture_rand_range(-GST_AUG_SCALE_JIT, GST_AUG_SCALE_JIT));
        h = replay_tuple_hash(a_slot, B ? (uint8_t)(B - G.lib) : 0xffu, theta, s);
        if (!dup_seen(h)) break;
        G.dup_rejected++;
    }
    dup_record(h);

    float c = cosf(theta), sn = sinf(theta);
    uint16_t n = A->n;
    if (n > GST_KNOTS_MAX) n = GST_KNOTS_MAX;   /* defensive: G.work is GST_KNOTS_MAX */
    float walk = 1.0f;
    for (uint16_t i = 0; i < n; i++) {
        float ux = alpha * A->knots[i].ux + (1.0f - alpha) * (B ? B->knots[i].ux : 0.0f);
        float uy = alpha * A->knots[i].uy + (1.0f - alpha) * (B ? B->knots[i].uy : 0.0f);
        float x = ux * s, y = uy * s;
        G.work[i].ux = x * c - y * sn;
        G.work[i].uy = x * sn + y * c;
        G.work[i].f  = A->knots[i].f;

        walk += gesture_rand_range(-0.02f, 0.02f);
        if (walk < 1.0f - GST_AUG_TWARP) walk = 1.0f - GST_AUG_TWARP;
        if (walk > 1.0f + GST_AUG_TWARP) walk = 1.0f + GST_AUG_TWARP;
        float ad = (float)A->knots[i].dt_q;
        float bd = B ? (float)B->knots[i].dt_q : ad;
        float dq = (alpha * ad + (1.0f - alpha) * bd) * walk;
        if (dq < 0.0f) dq = 0.0f;
        if (dq > 65535.0f) dq = 65535.0f;
        G.work[i].dt_q = (uint16_t)(dq + 0.5f);
    }
    G.work_n = n;
    endpoint_true(tx, ty);
    G.work_cursor = 1;
    G.emit_px = G.work[0].ux;
    G.emit_py = G.work[0].uy;
    return true;
}

/* Normalized min-jerk position, tau in [0,1]: 10t³ − 15t⁴ + 6t⁵
 * (zero velocity and acceleration at both ends). */
static float minjerk(float tau) {
    float t3 = tau * tau * tau;
    return t3 * (10.0f - 15.0f * tau + 6.0f * tau * tau);
}

/* Ballistic primary (undershoots to g1) + corrective submovement to V, with a
 * small lateral bow. The segment junction yields a velocity dip + re-accel. */
static bool synth_begin(int32_t tx, int32_t ty) {
    float Vx = (float)tx, Vy = (float)ty;
    float R = sqrtf(Vx * Vx + Vy * Vy);
    if (R < 1.0f) { G.work_n = 0; return false; }
    float theta = atan2f(Vy, Vx);
    float c = cosf(theta), sn = sinf(theta);

    uint16_t n = GST_KNOTS_MAX;
    uint16_t split = (uint16_t)(n * 7u / 10u);     /* ~70% of knots in the primary */
    if (split < 2)      split = 2;
    if (split > n - 2)  split = (uint16_t)(n - 2);
    float g1      = R * gesture_rand_range(0.80f, 0.90f);          /* primary undershoot:
                                                                      10–20% corrective,
                                                                      always a detectable
                                                                      re-acceleration */
    float lat_amp = R * gesture_rand_range(-0.03f, 0.03f);         /* lateral bow */

    for (uint16_t i = 0; i < n; i++) {
        float fi = (float)i / (float)(n - 1);
        float along;
        if (i <= split) {
            float tau = (float)i / (float)split;
            along = g1 * minjerk(tau);
        } else {
            float tau = (float)(i - split) / (float)(n - 1 - split);
            along = g1 + (R - g1) * minjerk(tau);
        }
        float lateral = lat_amp * sinf(3.14159265f * fi);          /* 0 at both ends */
        float x = along, y = lateral;
        G.work[i].ux   = x * c - y * sn;
        G.work[i].uy   = x * sn + y * c;
        G.work[i].f    = fi;
        /* Cadence: reproduce the device's recent real inter-report jitter in
         * forward order (preserves short-range correlation + coalescing). Knot 0
         * has no predecessor (dt_q 0); fall back to one nominal when no cadence. */
        if (i == 0) {
            G.work[i].dt_q = 0u;
        } else {
            uint16_t cc = gesture_cadence_count();
            if (cc == 0u) {
                G.work[i].dt_q = 256u;               /* cold boot: flat one-nominal */
            } else {
                /* oldest-first mapping so emission walks the captured run forward */
                uint16_t age = (uint16_t)((cc - 1u) - (uint16_t)((i - 1u) % cc));
                uint32_t cad = 0u;
                (void)gesture_cadence_get(age, &cad);
                uint32_t cap = (G.nominal_us ? G.nominal_us : 1000u) * GST_CAD_DROP_MULT;
                if (cad > cap) cad = cap;            /* clamp dropouts, keep coalescing */
                G.work[i].dt_q = us_to_dtq(cad, G.nominal_us);
            }
        }
    }
    G.work_n = n;
    endpoint_true(tx, ty);
    G.work_cursor = 1;
    G.emit_px = G.work[0].ux;
    G.emit_py = G.work[0].uy;
    return true;
}

gst_sel_t gesture_select_source(motion_mode_t mode, float target_len) {
    (void)mode;
    if (gesture_warmth() == GST_COLD) return GST_SEL_SYNTH;
    uint8_t want = gesture_length_bucket(target_len);
    for (uint8_t i = 0; i < G.lib_n; i++)
        if (G.lib_bucket[i] == want) return GST_SEL_REPLAY;
    return GST_SEL_SYNTH;                  /* warm, but no same-bucket shape */
}

uint32_t gesture_replay_count(void)         { return G.replay_count; }
uint32_t gesture_synth_fallback_count(void) { return G.synth_fallback_count; }
uint32_t gesture_bucket_miss(void)          { return G.bucket_miss; }

void gesture_motion_begin(int32_t tx, int32_t ty, motion_mode_t mode) {
    G.pace_budget_us = 0;
    float R = sqrtf((float)tx * (float)tx + (float)ty * (float)ty);
    gst_warmth_t w = gesture_warmth();
    gst_sel_t    sel = gesture_select_source(mode, R);

    /* Warm library but no same-bucket shape → record the miss (selector returned synth). */
    if (sel == GST_SEL_SYNTH && w != GST_COLD) G.bucket_miss++;

    if (sel == GST_SEL_REPLAY && replay_begin(tx, ty)) {
        G.src_kind = GST_SRC_REPLAY; G.replay_count++; return;
    }
    if (synth_begin(tx, ty)) {
        G.src_kind = GST_SRC_SYNTH; G.synth_fallback_count++; return;
    }
    G.src_kind = GST_SRC_NONE;
    G.work_n = 0; G.work_cursor = 0;
}

GST_FASTRUN
bool gesture_motion_next(float *out_dx, float *out_dy, uint16_t *out_dt_q) {
    if (G.src_kind == GST_SRC_NONE) return false;
    if (G.work_cursor >= G.work_n)  return false;      /* exhausted */
    float tx = G.work[G.work_cursor].ux;
    float ty = G.work[G.work_cursor].uy;
    *out_dx = tx - G.emit_px;
    *out_dy = ty - G.emit_py;
    if (out_dt_q) *out_dt_q = G.work[G.work_cursor].dt_q;
    G.emit_px = tx; G.emit_py = ty;
    G.work_cursor++;
    return true;
}

bool gesture_motion_done(void) {
    return G.src_kind == GST_SRC_NONE || G.work_cursor >= G.work_n;
}

void gesture_motion_pace_advance(uint32_t elapsed_us) {
    uint64_t b = (uint64_t)G.pace_budget_us + (uint64_t)elapsed_us;
    if (b > 2000000u) b = 2000000u;        /* bound catch-up after a long stall */
    G.pace_budget_us = (uint32_t)b;
}

GST_FASTRUN
bool gesture_motion_pace_take(float *out_dx, float *out_dy, uint16_t *out_dt_q) {
    if (gesture_motion_done()) return false;        /* no source or exhausted */
    uint32_t need = dtq_to_us(G.work[G.work_cursor].dt_q);  /* next step's interval */
    if (need > G.pace_budget_us) return false;      /* not due yet */
    G.pace_budget_us -= need;
    return gesture_motion_next(out_dx, out_dy, out_dt_q);   /* emit exactly one step */
}

void gesture_click_real_buttons(uint8_t buttons, uint32_t t_us) {
    uint32_t dt = (G.c2_have_t && t_us >= G.c2_last_t) ? (t_us - G.c2_last_t) : 0u;
    G.c2_last_t = t_us; G.c2_have_t = true;

    bool down_now = (buttons != 0u);
    G.c2_real_down = down_now;
    G.c2_prev_buttons = buttons;

    float span = 1.0f - GST_C2_FLOOR;
    if (down_now) {
        /* ramp scale 1.0 → floor over the guard window */
        G.c2_scale -= ((float)dt / GST_C2_GUARD_US) * span;
        if (G.c2_scale < GST_C2_FLOOR) G.c2_scale = GST_C2_FLOOR;
    } else {
        /* ramp scale floor → 1.0 over the resume window */
        G.c2_scale += ((float)dt / GST_C2_RESUME_US) * span;
        if (G.c2_scale > 1.0f) G.c2_scale = 1.0f;
    }
}

GST_FASTRUN
float gesture_click_motion_scale(void) { return G.c2_scale; }

bool gesture_click_real_active(void) { return G.c2_real_down; }
