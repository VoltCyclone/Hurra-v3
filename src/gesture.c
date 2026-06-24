#include "gesture.h"
#include "humanize.h"
#include <string.h>
#include <math.h>

#ifdef GESTURE_HOSTTEST
#define GST_FASTRUN
#else
#define GST_FASTRUN __attribute__((section(".fastrun")))
#endif

#define GST_CLK_RECOIL_WIN_US 30000u   /* drift window measured after release */
#define GST_CLK_PEAK_DECAY    0.85f    /* per-report decay of the speed peak  */
#define GST_C2_FLOOR     0.05f     /* injected-motion floor during real hold (micro-drift) */
#define GST_C2_GUARD_US  15000.0f  /* ramp-down time on real button-down  */
#define GST_C2_RESUME_US 25000.0f  /* ramp-up time after real release      */
/* capture state-machine phases */
#define GST_CC_IDLE   0u
#define GST_CC_HOLD   1u
#define GST_CC_RECOIL 2u
/* ── Mode 1 self-fire constants ── */
#define GST_C1_RECOIL_EMIT_US 30000.0f /* window over which recoil is injected */
#define GST_C1_DEF_DWELL_US   80000u   /* default dwell when library is cold   */
#define GST_C1_DEF_SETTLE     2.0f     /* default hold-drift magnitude (px)    */
/* self-fire phases */
#define GST_C1_IDLE   0u
#define GST_C1_PRESS  1u   /* about to emit PRESS this step */
#define GST_C1_DWELL  2u
#define GST_C1_RECOIL 3u

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
    /* ── PRNG (SFC32), seeded in gesture_init ── */
    uint32_t rng_a, rng_b, rng_c, rng_ctr;
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
    /* ── Mode 1 self-fire (Plan 4) ── */
    uint8_t  c1_state;                    /* GST_C1_IDLE/PRESS/DWELL/RECOIL */
    uint8_t  c1_button;                  /* button index to emit           */
    uint32_t c1_dwell_us;               /* this fire's dwell               */
    uint32_t c1_dwell_el;               /* elapsed in dwell                */
    float    c1_settle;                 /* hold-drift magnitude            */
    float    c1_recoil_x, c1_recoil_y;  /* recoil vector to inject         */
    float    c1_rec_emit_x, c1_rec_emit_y; /* recoil already emitted       */
    uint32_t c1_rec_el;                 /* elapsed in recoil               */
    /* ── residual store (v3) ── */
    gst_residual_t res[GST_RES_BUCKETS][GST_RES_RING];
    uint8_t  res_head[GST_RES_BUCKETS];   /* next write slot per bucket */
    uint8_t  res_n[GST_RES_BUCKETS];      /* valid samples 0..GST_RES_RING */
    uint8_t  res_read[GST_RES_BUCKETS];   /* sequential read cursor */
    /* ── streaming filter live state (v3) ── */
    float sf_hx, sf_hy;    /* EWMA heading vector (counts/report) */
    float sf_speed;        /* EWMA speed magnitude */
    float sf_debt_x, sf_debt_y; /* accumulated injected residual (for leak) */
    uint8_t sf_have;       /* EWMA initialized */
    /* ── honest-limit detector (v3) ── */
    float    nh_mag[GST_NH_WIN];   /* recent |app delta| ring */
    uint8_t  nh_head, nh_n;
    uint32_t nh_count;             /* non-human-trend events (diagnostic) */
    uint8_t  nh_human;             /* last verdict: 1 human, 0 non-human */
    float    nh_peak_real;   /* largest real human single-report |delta| this session (adaptive teleport bar) */
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
    G.nh_human = 1;                       /* assume human until evidence otherwise (memset zeros it) */
    gesture_stream_reset();
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

/* ── residual store (Humanization v3) ─────────────────────────────────── */

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

gst_warmth_t gesture_warmth(void) { return gesture_residual_warmth(); }

/* ── residual extraction (v3) ─────────────────────────────────────────── */

uint8_t gesture_speed_bucket(float speed_cpr) {
    if (speed_cpr < GST_RES_SLOW_MAX) return 0;
    if (speed_cpr < GST_RES_MED_MAX)  return 1;
    return 2;
}

uint16_t gesture_residual_extract(uint16_t window) {
    if (window < GST_RES_FIR + 2u) return 0;
    if (window > GST_CAP_RING)      window = GST_CAP_RING;

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

/* ── honest-limit detector (Humanization v3) ─────────────────────────── */

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

/* ── streaming residual filter (Humanization v3, per-poll) ──────────── */

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

void gesture_human_status(gst_human_status_t *out) {
    if (!out) return;
    out->warmth = (uint8_t)gesture_residual_warmth();
    int human = gesture_trend_is_human();
    if (out->warmth == GST_WARM && human)         out->replay_pct = 100;
    else if (out->warmth == GST_WARMING && human) out->replay_pct = 50;
    else                                          out->replay_pct = 0;
    out->synth_pct = human ? 0 : 100;     /* repurposed: non-human-trend share */
    out->dup = 0;
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

bool gesture_click_arm_fire(uint8_t button) {
    if (G.c2_real_down) return false;        /* arbitration: real click wins */
    if (G.c1_state != GST_C1_IDLE) return false; /* already firing */
    gst_click_env_t e;
    if (gesture_click_select(&e)) {
        G.c1_dwell_us  = e.dwell_us;
        G.c1_settle    = e.settle_px > 0.1f ? e.settle_px : GST_C1_DEF_SETTLE;
        G.c1_recoil_x  = e.recoil_x;
        G.c1_recoil_y  = e.recoil_y;
    } else {                                 /* cold: plausible defaults */
        G.c1_dwell_us  = GST_C1_DEF_DWELL_US;
        G.c1_settle    = GST_C1_DEF_SETTLE;
        G.c1_recoil_x  = 0.0f;
        G.c1_recoil_y  = 0.0f;
    }
    G.c1_button     = button;
    G.c1_dwell_el   = 0u;
    G.c1_rec_el     = 0u;
    G.c1_rec_emit_x = 0.0f; G.c1_rec_emit_y = 0.0f;
    G.c1_state      = GST_C1_PRESS;
    return true;
}

bool gesture_click_fire_active(void) { return G.c1_state != GST_C1_IDLE; }

GST_FASTRUN
gst_click_action_t gesture_click_fire_step(uint32_t dt_us, float *out_dx, float *out_dy) {
    *out_dx = 0.0f; *out_dy = 0.0f;
    switch (G.c1_state) {
    case GST_C1_PRESS:
        G.c1_state = GST_C1_DWELL;
        G.c1_dwell_el = 0u;
        return GST_CA_PRESS;                  /* emit button-down this step */
    case GST_C1_DWELL: {
        G.c1_dwell_el += dt_us;
        /* settle-bounded micro-drift: not frozen, but tiny */
        float d = G.c1_settle * 0.25f;
        *out_dx = gesture_rand_range(-d, d);
        *out_dy = gesture_rand_range(-d, d);
        if (G.c1_dwell_el >= G.c1_dwell_us) {
            G.c1_state = GST_C1_RECOIL;
            G.c1_rec_el = 0u;
            *out_dx = 0.0f; *out_dy = 0.0f;   /* RELEASE is a pure button event: no drift */
            return GST_CA_RELEASE;            /* emit button-up this step */
        }
        return GST_CA_NONE;
    }
    case GST_C1_RECOIL: {
        G.c1_rec_el += dt_us;
        if (G.c1_rec_el >= (uint32_t)GST_C1_RECOIL_EMIT_US) {
            /* final slice: flush the remainder so Σ recoil == envelope recoil */
            *out_dx = G.c1_recoil_x - G.c1_rec_emit_x;
            *out_dy = G.c1_recoil_y - G.c1_rec_emit_y;
            G.c1_rec_el = 0u;                 /* clear timer; don't leave stale state */
            G.c1_state = GST_C1_IDLE;
            return GST_CA_NONE;
        }
        float frac = (float)dt_us / GST_C1_RECOIL_EMIT_US;
        float sx = G.c1_recoil_x * frac, sy = G.c1_recoil_y * frac;
        G.c1_rec_emit_x += sx; G.c1_rec_emit_y += sy;
        *out_dx = sx; *out_dy = sy;
        return GST_CA_NONE;
    }
    default:
        return GST_CA_NONE;                   /* IDLE: nothing */
    }
}
