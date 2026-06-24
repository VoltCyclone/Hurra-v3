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
#define HZ_MAX_PER_FRAME   127      /* human per-frame ceiling (counts) */
#define HZ_IDLE_EPS        0.01f    /* |owed| below this = settled */

static struct {
    float    owed_x, owed_y;        /* undelivered injected motion */
    float    res_x, res_y;          /* sub-pixel residual */
} S;

void humanize_init(uint32_t interval_us) {
    (void)interval_us;              /* retained for ABI stability (caller passes nominal) */
    memset(&S, 0, sizeof(S));
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
void humanize_inject_emit(float dx, float dy, int16_t *out_dx, int16_t *out_dy) {
    /* Accumulate into the injected owed, then drain through the cap-with-carry
     * path (emit_v = full owed, no perpendicular noise — the engine's residual
     * shapes already carry real noise). owed retains the field-clamp overflow
     * for redelivery; res retains the sub-pixel remainder. */
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
