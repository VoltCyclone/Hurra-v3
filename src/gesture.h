#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── fixed sizes (RAM-only, no dynamic allocation) ───────────────────── */
#define GST_CAP_RING    256   /* raw capture samples retained            */
#define GST_KNOTS_MAX   48    /* knots per normalized shape              */
#define GST_LIB_SHAPES  32    /* real shapes retained in the library     */
#define GST_WARM_MIN    6     /* shapes needed before replay is trusted  */

/* [1] CAPTURE — raw report observation, device-native, timestamped.     */
typedef struct {
    int16_t  dx, dy;          /* device-native report delta              */
    uint32_t t_us;            /* Board B 1 MHz TIM9 timestamp            */
} gst_sample_t;               /* 8 bytes                                 */

/* [2] SHAPE — one normalized resolution-independent gesture.            */
typedef struct {
    float    ux, uy;          /* knot position, sub-pixel, SHAPE space    */
    float    f;               /* path-length fraction [0..1] at this knot */
    uint16_t dt_q;            /* inter-knot interval, .8 fixed x nominal  */
} gst_knot_t;                 /* 16 bytes (float x3 + uint16 padded)      */

typedef struct {
    gst_knot_t knots[GST_KNOTS_MAX];
    uint16_t   n;             /* knots used (<= GST_KNOTS_MAX)            */
    float      raw_len;       /* original path length (counts)           */
    uint16_t   total_us;      /* original duration (x nominal)           */
    uint8_t    submv;         /* submovements detected                   */
    uint8_t    flags;         /* curvature sign, click-adjacent, etc.    */
} gst_shape_t;

/* Initialize/clear all gesture state. nominal_interval_us is the device's
 * measured nominal report interval, used as the dt scaling reference. */
void gesture_init(uint32_t nominal_interval_us);

/* ── capture ring ──────────────────────────────────────────────────────
 * Read-only FIFO of raw report samples. Never mutates the real input path;
 * callers push a copy of each observed delta. */
void     gesture_capture_push(int16_t dx, int16_t dy, uint32_t t_us);
uint16_t gesture_capture_count(void);            /* 0..GST_CAP_RING       */
bool     gesture_capture_get(uint16_t age, gst_sample_t *out); /* age 0=newest */
