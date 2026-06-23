#include "humanize.h"
#include <string.h>
#include <math.h>

/* Pin the per-tick hot path to ITCM (.fastrun) for deterministic, cache-miss-free
 * latency, independent of LTO/flag changes. No-op on host tests. */
#ifdef HUMANIZE_HOSTTEST
#define HZ_FASTRUN
#else
#define HZ_FASTRUN __attribute__((section(".fastrun")))
#endif

/* ── tunables ───────────────────────────────────────────────────────── */
#define HZ_DEFAULT_LEVEL   2        /* boot default: on, "normal" */
#define HZ_MAX_PER_FRAME   127      /* human per-frame ceiling (counts) */
#define HZ_IDLE_EPS        0.01f    /* |owed| below this = settled */
#define HZ_TIMING_JITTER   0.12f    /* +/- fraction of base period to jitter */
#define HZ_TIMING_FLOOR    0.80f    /* min multiple of base period */
#define HZ_TIMING_CEIL     1.20f    /* max multiple of base period */

/* ── adaptive feed-rate / measured-interval tunables ───────────────────
 * The measured delivery interval of real mouse reports (µs) is dominated by
 * host/EHCI quantization (125 µs microframe / 1 ms FS frame) plus poll latency,
 * not the mouse's true cadence. It is EWMA-smoothed with outlier rejection. */
#define HZ_MEAS_EWMA_SHIFT 4        /* alpha = 1/16 (>>4) on the measured interval */
#define HZ_MEAS_MIN_COUNT  5        /* reports required before target is trusted */
#define HZ_MEAS_DROP_MULT  4u       /* dt > this*target → dropout (reset baseline) */
#define HZ_MEAS_BURST_DIV  2u       /* dt < target/this → burst/retry (ignore) */
#define HZ_LDVAL_US_MIN    125u     /* must match main.c PIT clamp window */
#define HZ_LDVAL_US_MAX    10000u

/* ── per-axis normalized-speed envelope (peak-with-decay) ──────────────
 * Track a per-axis rolling peak of speed (counts/sec, DPI-independent given the
 * measured interval) and normalize current speed to it. Decay is time-based (per
 * measured dt), not per-frame, so the envelope memory is poll-rate independent;
 * attack is instantaneous. HZ_ADAPTIVE_NOISE gates whether the envelope drives
 * the noise term; default 0 preserves the raw-speed behavior. The envelope state
 * is maintained regardless so diagnostics can observe it. */
#ifndef HZ_ADAPTIVE_NOISE
#define HZ_ADAPTIVE_NOISE  0
#endif
#define HZ_PEAK_TAU_US     200000.0f /* envelope decay time constant (~200 ms) */
#define HZ_PEAK_FLOOR      1.0f       /* min peak (counts/sec) → no div-by-zero */

/* Per-level perpendicular-noise amplitude (fraction of speed → ~constant jitter
 * angle); level 0 = off. Injection is a bounded per-frame perturbation (noise +
 * dither + cap) delivered in-frame, so it never lags the host's motion. */
static const float HZ_NOISE[4] = { 0.0f, 0.05f, 0.10f, 0.18f };

static struct {
    uint8_t  level;
    float    noise_amp;
    float    owed_x, owed_y;        /* undelivered injected motion */
    float    res_x, res_y;          /* sub-pixel residual */
    float    ewma;                  /* noise correlation alpha */
    float    n_perp;                /* correlated perpendicular noise state */
    uint32_t a, b, c, ctr;          /* SFC32 */
    uint32_t timing_lfsr;
    int      idle;
    /* adaptive feed-rate: measured delivery interval (GPT2 µs) */
    uint32_t last_ts_us;            /* timestamp of previous report (0 = none) */
    uint32_t meas_interval_us;      /* EWMA-smoothed delivery interval */
    uint32_t arrival_count;         /* reports recorded since init */
    /* per-axis normalized-speed envelope (counts/sec, time-decayed peak) */
    float    peak_x, peak_y;        /* rolling per-axis speed peak */
} S;

/* ── RNG (SFC32) ────────────────────────────────────────────────────── */
static inline uint32_t sfc32(void) {
    uint32_t t = S.a + S.b + S.ctr++;
    S.a = S.b ^ (S.b >> 9);
    S.b = S.c + (S.c << 3);
    S.c = ((S.c << 21) | (S.c >> 11)) + t;
    return t;
}
static inline float sfc32_uniform(void) {       /* [-1, 1) */
    int32_t bal = (int32_t)(sfc32() >> 8) - 0x800000;
    return (float)bal * (1.0f / 8388608.0f);
}

/* ── seeding ────────────────────────────────────────────────────────── */
#ifdef HUMANIZE_HOSTTEST
static uint32_t hw_entropy(void) { return 0x12345678u; }   /* deterministic */
#else
#include "ch32h417_port.h"
/* Seed entropy from the free-running TIM9 1 MHz counter XOR'd with the WCH
 * device-info word at 0x1FFFF704. The mcycle CSR is avoided: reads of it do not
 * return on the V5F core. Quality is non-critical — this seeds an sfc32 PRNG
 * that is warmed up 16 rounds. */
extern uint32_t timebase_v5f_us(void);   /* TIM9->CNT_32, started pre-humanize */
static uint32_t hw_entropy(void) {
    uint32_t cyc = timebase_v5f_us();
    uint32_t uid = *(volatile uint32_t *)0x1FFFF704;
    return cyc ^ uid;
}
#endif

void humanize_set_level(uint8_t level) {
    if (level > 3) level = 3;
    S.level     = level;
    S.noise_amp = HZ_NOISE[level];
}

uint8_t humanize_get_level(void)
{
    return S.level;
}

void humanize_init(uint32_t interval_us) {
    memset(&S, 0, sizeof(S));
    uint32_t seed = hw_entropy();
    S.a = seed ^ 0xCAFEBABEu; S.b = seed ^ 0xDEADBEEFu;
    S.c = seed ^ 0x8BADF00Du; S.ctr = 1;
    if (!S.a) S.a = 0xCAFEBABEu;
    /* Only S.a is guarded; the 16-iteration warm-up restores liveness even if
     * b/c seed to zero. */
    for (int i = 0; i < 16; i++) sfc32();
    S.timing_lfsr = sfc32() | 1u;
    S.ewma = 0.85f;
    humanize_set_level(HZ_DEFAULT_LEVEL);
    S.noise_amp *= 1.0f + 0.15f * sfc32_uniform();
    /* Pre-seed the measured-interval EWMA with the nominal bInterval period so
     * outlier rejection is sane from the first real report. Otherwise the first
     * inter-report gap (arbitrarily large, since the device only sends on
     * movement) would seed a huge estimate that rejects every normal report as a
     * burst, locking meas_interval_us at the bad value. */
    if (interval_us < HZ_LDVAL_US_MIN) interval_us = HZ_LDVAL_US_MIN;
    if (interval_us > HZ_LDVAL_US_MAX) interval_us = HZ_LDVAL_US_MAX;
    S.meas_interval_us = interval_us;
}

/* ── adaptive feed-rate: measured delivery interval ────────────────────── */
HZ_FASTRUN
void humanize_record_arrival(uint32_t ts_us) {
    if (S.arrival_count == 0) {
        S.last_ts_us = ts_us;
        S.arrival_count = 1;
        return;                                  /* first report: no interval yet */
    }
    uint32_t dt = ts_us - S.last_ts_us;          /* unsigned, single-wrap safe */
    uint32_t cur = S.meas_interval_us;
    if (cur != 0) {
        /* Reject outliers before they pollute the EWMA:
         *  - dropout (far longer than expected): rebase, no update
         *  - burst / double-report / USB retry (far shorter): ignore */
        if (dt > cur * HZ_MEAS_DROP_MULT) { S.last_ts_us = ts_us; return; }
        if (dt < cur / HZ_MEAS_BURST_DIV) { S.last_ts_us = ts_us; return; }
    }
    S.last_ts_us = ts_us;
    if (cur == 0) {
        S.meas_interval_us = dt;                 /* seed EWMA with first interval */
    } else {
        /* EWMA: cur += (dt - cur) * alpha, alpha = 1/2^SHIFT, integer, signed step */
        S.meas_interval_us = (uint32_t)((int32_t)cur +
            (((int32_t)dt - (int32_t)cur) >> HZ_MEAS_EWMA_SHIFT));
    }
    if (S.arrival_count < 0xFFFFFFFFu) S.arrival_count++;
}

uint32_t humanize_measured_interval_us(void) {
    return S.meas_interval_us;
}

uint32_t humanize_target_ldval(uint32_t pit_clk_hz) {
    if (S.arrival_count < HZ_MEAS_MIN_COUNT || S.meas_interval_us == 0)
        return 0;                                /* not yet confident */
    uint32_t us = S.meas_interval_us;
    if (us < HZ_LDVAL_US_MIN) us = HZ_LDVAL_US_MIN;
    if (us > HZ_LDVAL_US_MAX) us = HZ_LDVAL_US_MAX;
    /* pit_clk_hz (PERCLK) is an exact multiple of 1 MHz, so counts =
     * us * (clk/1e6) in pure 32-bit math (avoids a software-loop divide in this
     * per-tick path). Worst case 10000 µs * 24 = 240000, well inside uint32_t. */
    uint32_t counts = us * (pit_clk_hz / 1000000u);
    return counts ? counts - 1u : 0u;
}

HZ_FASTRUN
static int16_t drain_axis(float *owed, float *res, float emit_v, float noise) {
    float want = emit_v + noise + *res;
    float capped = want;
    if (capped >  (float)HZ_MAX_PER_FRAME) capped =  (float)HZ_MAX_PER_FRAME;
    if (capped < -(float)HZ_MAX_PER_FRAME) capped = -(float)HZ_MAX_PER_FRAME;
    float res_in = *res;                     /* residual before this frame's update */
    int16_t out = (int16_t)(capped >= 0 ? (capped + 0.5f) : (capped - 0.5f));
    *res = capped - (float)out;
    /* Drain owed by emit_v unconditionally (the sub-pixel residual lives in *res,
     * and noise is zero-mean so it is not tracked in owed). If the cap cut into
     * the signal portion, add that cut back so owed redelivers it later. */
    float signal_cut = 0.0f;
    float uncapped_signal = emit_v + res_in; /* signal + pre-update dither, no noise */
    if (uncapped_signal > (float)HZ_MAX_PER_FRAME)
        signal_cut = uncapped_signal - (float)HZ_MAX_PER_FRAME;
    else if (uncapped_signal < -(float)HZ_MAX_PER_FRAME)
        signal_cut = uncapped_signal - (-(float)HZ_MAX_PER_FRAME);
    *owed -= emit_v - signal_cut;
    return out;
}

HZ_FASTRUN
void humanize_filter(int16_t *dx, int16_t *dy) {
    if (S.level == 0) return;                 /* off: passthrough */

    S.owed_x += (float)*dx;
    S.owed_y += (float)*dy;

    if (fabsf(S.owed_x) < HZ_IDLE_EPS && fabsf(S.owed_y) < HZ_IDLE_EPS &&
        fabsf(S.res_x) < 0.5f && fabsf(S.res_y) < 0.5f) {
        if (S.idle < 1000) S.idle++;
        *dx = 0; *dy = 0;
        return;
    }
    S.idle = 0;

    /* Deliver all owed motion this frame; cap-with-carry below handles the rare
     * >127/frame flick. No fractional drain, so no added latency or smear. */
    float ex = S.owed_x;
    float ey = S.owed_y;

    float speed = sqrtf(ex*ex + ey*ey);

    float speed_drive = speed;                 /* default: raw speed */
#if HZ_ADAPTIVE_NOISE
    /* Per-axis normalized-speed envelope: meaningful only with a measured
     * delivery interval to convert counts→counts/sec. Attack is instantaneous;
     * decay is time-based over the measured dt, so envelope memory is poll-rate
     * independent. Gated so the default build pays no expf/divide per frame. */
    if (S.meas_interval_us > 0) {
        float inv_dt = 1e6f / (float)S.meas_interval_us;   /* 1/sec */
        float rate_x = fabsf(ex) * inv_dt;
        float rate_y = fabsf(ey) * inv_dt;
        float k = expf(-(float)S.meas_interval_us / HZ_PEAK_TAU_US); /* decay */
        S.peak_x = (rate_x > S.peak_x) ? rate_x : S.peak_x * k;
        S.peak_y = (rate_y > S.peak_y) ? rate_y : S.peak_y * k;
        if (S.peak_x < HZ_PEAK_FLOOR) S.peak_x = HZ_PEAK_FLOOR;
        if (S.peak_y < HZ_PEAK_FLOOR) S.peak_y = HZ_PEAK_FLOOR;
        /* Drive noise off the 0..1 normalized magnitude. Scaling by `speed`
         * keeps the perpendicular jitter a constant angle; the normalized term
         * shapes amplitude by current speed relative to the per-axis peak. */
        float nrm_x = rate_x / S.peak_x;
        float nrm_y = rate_y / S.peak_y;
        float norm  = sqrtf(nrm_x*nrm_x + nrm_y*nrm_y);    /* ~0..~1.41 */
        if (norm > 1.0f) norm = 1.0f;
        speed_drive = speed * norm;
    }
#endif

    S.n_perp = S.ewma * S.n_perp + (1.0f - S.ewma) * sfc32_uniform();
    float nmag = S.n_perp * S.noise_amp * speed_drive;
    float nx = 0.0f, ny = 0.0f;
    if (speed > 1e-3f) { nx = -ey / speed * nmag; ny = ex / speed * nmag; }

    *dx = drain_axis(&S.owed_x, &S.res_x, ex, nx);
    *dy = drain_axis(&S.owed_y, &S.res_y, ey, ny);
}

HZ_FASTRUN
void humanize_inject_emit(float dx, float dy, int16_t *out_dx, int16_t *out_dy) {
    /* Accumulate into the injected owed, then drain through the same cap-with-
     * carry path humanize_filter uses (emit_v = full owed, no perpendicular
     * noise — the engine's shapes already carry real noise). owed retains the
     * field-clamp overflow for redelivery; res retains the sub-pixel remainder. */
    S.owed_x += dx;
    S.owed_y += dy;
    *out_dx = drain_axis(&S.owed_x, &S.res_x, S.owed_x, 0.0f);
    *out_dy = drain_axis(&S.owed_y, &S.res_y, S.owed_y, 0.0f);
}

HZ_FASTRUN
void humanize_return(int16_t dx, int16_t dy) {
    S.owed_x += (float)dx;
    S.owed_y += (float)dy;
}

HZ_FASTRUN
bool humanize_pending(void) {
    return fabsf(S.owed_x) >= HZ_IDLE_EPS || fabsf(S.owed_y) >= HZ_IDLE_EPS;
}

HZ_FASTRUN
uint32_t humanize_timing_next(uint32_t base_ldval) {
    if (S.level == 0) return base_ldval;
    S.timing_lfsr ^= S.timing_lfsr << 13;
    S.timing_lfsr ^= S.timing_lfsr >> 17;
    S.timing_lfsr ^= S.timing_lfsr << 5;
    float u = (float)(S.timing_lfsr >> 8) * (1.0f / 16777216.0f) - 0.5f;
    float r = (float)base_ldval * (1.0f + HZ_TIMING_JITTER * u);
    float lo = (float)base_ldval * HZ_TIMING_FLOOR, hi = (float)base_ldval * HZ_TIMING_CEIL;
    if (r < lo) r = lo;
    if (r > hi) r = hi;
    return (uint32_t)r;
}
