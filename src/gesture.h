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
    uint32_t   total_us;      /* original duration, .8 fixed of nominal (no 16-bit cap) */
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

/* ── quality-gated shape builder ───────────────────────────────────────*/
#define GST_LEN_BUCKETS 3
#define GST_MIN_LEN     8.0f   /* counts; shorter gestures are rejected */

uint8_t gesture_length_bucket(float raw_len); /* 0 short / 1 medium / 2 long */
bool    gesture_build_shape(const gst_sample_t *samples, uint16_t n,
                            gst_shape_t *out);

/* ── library + warmth ──────────────────────────────────────────────────*/
typedef enum { GST_COLD = 0, GST_WARMING = 1, GST_WARM = 2 } gst_warmth_t;

void                gesture_library_admit(const gst_shape_t *shape);
uint8_t             gesture_library_count(void);
gst_warmth_t        gesture_warmth(void);
const gst_shape_t  *gesture_library_select(float target_len);
bool                gesture_capture_build_and_admit(uint16_t window);

/* ── PRNG (test hooks) ─────────────────────────────────────────────────
 * SFC32, seeded in gesture_init. Deterministic (0x12345678) under
 * GESTURE_HOSTTEST so augmentation/selection tests are reproducible. */
uint32_t gesture_rand_u32(void);
float    gesture_rand_range(float lo, float hi);   /* uniform [lo, hi) */

/* ── motion source (replay / synth dispatch) ───────────────────────────
 * One in-flight gesture at a time. begin() materializes a transformed working
 * copy; next() streams one per-step float delta per call (Plan-2 geometry +
 * intended timing; Plan-3 adds dt_us pacing and drain_axis quantization). */
typedef enum { MOTION_MODE_AIMED = 0, MOTION_MODE_SILENT = 1 } motion_mode_t;

void gesture_motion_begin(int32_t tx, int32_t ty, motion_mode_t mode);
bool gesture_motion_next(float *out_dx, float *out_dy, uint16_t *out_dt_q);
bool gesture_motion_done(void);

/* Count of replay re-rolls forced by the repetition guard (diagnostic). */
uint32_t gesture_dup_rejected(void);

/* ── source selector + diagnostics ─────────────────────────────────────*/
typedef enum { GST_SEL_SYNTH = 0, GST_SEL_REPLAY = 1 } gst_sel_t;

/* COLD → synth; WARMING/WARM → replay if a same-bucket shape exists, else synth. */
gst_sel_t gesture_select_source(motion_mode_t mode, float target_len);

uint32_t  gesture_replay_count(void);
uint32_t  gesture_synth_fallback_count(void);
uint32_t  gesture_bucket_miss(void);

/* ── cadence view (real device report-rate jitter) ─────────────────────
 * Derived from the capture ring's t_us timestamps — no new clock, no new
 * storage, and humanize's EWMA/PIT path is untouched. Unlike that EWMA, raw
 * intervals are NOT outlier-rejected: the variance and coalescing are the
 * human/hardware signal the silent-path cadence reproduces. */
uint16_t gesture_cadence_count(void);                       /* 0 if < 2 samples */
bool     gesture_cadence_get(uint16_t age, uint32_t *out_dt_us); /* age 0 = newest */
