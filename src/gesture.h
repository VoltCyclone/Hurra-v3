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

/* ── sub-pixel reconstruction ──────────────────────────────────────────
 * Integrate integer report deltas into a continuous float path. Output is
 * cumulative position (counts), cumulative time from the first sample (us),
 * and path-length fraction in [0,1]. Run before any rotate/scale so the
 * micro-structure is not re-quantized. */
typedef struct {
    float    x, y;     /* cumulative position, counts */
    float    f;        /* path-length fraction [0..1] */
    uint32_t t_us;     /* cumulative time from first sample */
} gst_point_t;

uint16_t gesture_reconstruct(const gst_sample_t *samples, uint16_t n,
                             gst_point_t *out, uint16_t out_cap);

/* ── resample ──────────────────────────────────────────────────────────
 * Resample a reconstructed polyline to exactly GST_KNOTS_MAX knots, evenly
 * spaced in path-length fraction, via centripetal Catmull-Rom. dt_q is left
 * 0 here; gesture_normalize_temporal (Task 6) fills the temporal field. Returns
 * GST_KNOTS_MAX, or 0 when n < 2. */
uint16_t gesture_resample(const gst_point_t *pts, uint16_t n, gst_knot_t *out);

/* ── spatial normalization ─────────────────────────────────────────────
 * Rotate the knot set so its endpoint is +X and scale so |endpoint| == 1.
 * Stores raw_len = original endpoint magnitude for replay rate-conversion.
 * Returns false (knots untouched) for a degenerate sub-unit gesture. */
bool gesture_normalize_spatial(gst_shape_t *shape);

/* ── temporal normalization ────────────────────────────────────────────
 * Fill knot dt_q (.8 fixed-point multiples of the nominal interval) from the
 * reconstructed point times, preserving real captured spacing. Sets total_us
 * (.8 fixed of nominal). Knot 0 dt_q is 0. */
void gesture_normalize_temporal(gst_shape_t *shape, const gst_point_t *pts,
                                uint16_t n);
