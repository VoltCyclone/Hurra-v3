// src/gesture_stream_v5f.c — V5F-only bridge from the act_stream_filter_t hook
// to the gesture residual filter. Adds real captured human residual onto each
// injected km.move delta. Host (V5F) only; never linked on V3F.
#include "actions.h"
#include "gesture.h"
#include <stdint.h>

static void sf_apply(int16_t in_dx, int16_t in_dy, int16_t *out_dx, int16_t *out_dy)
{
    gesture_trend_observe(in_dx, in_dy);                 /* honest-limit detector */
    gesture_stream_filter(in_dx, in_dy, out_dx, out_dy); /* add real residual */
}

const act_stream_filter_t gesture_stream_v5f_filter = { .apply = sf_apply };
