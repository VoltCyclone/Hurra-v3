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
} G;

void gesture_init(uint32_t nominal_interval_us) {
    memset(&G, 0, sizeof(G));
    G.nominal_us = nominal_interval_us ? nominal_interval_us : 1000u;
}
