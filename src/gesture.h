#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── fixed sizes (RAM-only, no dynamic allocation) ───────────────────── */
#define GST_CAP_RING    256   /* raw capture samples retained            */

/* [1] CAPTURE — raw report observation, device-native, timestamped.     */
typedef struct {
    int16_t  dx, dy;          /* device-native report delta              */
    uint32_t t_us;            /* Board B 1 MHz TIM9 timestamp            */
} gst_sample_t;               /* 8 bytes                                 */

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

/* ── library + warmth ──────────────────────────────────────────────────*/
typedef enum { GST_COLD = 0, GST_WARMING = 1, GST_WARM = 2 } gst_warmth_t;

/* ── residual store: speed-bucketed real motion residual ───────────────
 * Real captured high-frequency motion residual (tremor, motor noise,
 * off-axis wobble), speed-bucketed, drawn sequentially to preserve the
 * captured tremor autocorrelation. RAM-only, FIFO eviction. */
#define GST_RES_BUCKETS  3
#define GST_RES_RING     128
#define GST_RES_WARM_MIN 64    /* total samples before residual is trusted */

typedef struct {
    float    r_par;     /* along-heading speed fluctuation (counts/report) */
    float    r_perp;    /* perpendicular wobble (counts/report)            */
} gst_residual_t;       /* 8 bytes                                         */

void     gesture_residual_admit(uint8_t bucket, float r_par, float r_perp);
uint16_t gesture_residual_count(uint8_t bucket);     /* 0..GST_RES_RING */
uint16_t gesture_residual_total(void);
bool     gesture_residual_draw(uint8_t bucket, gst_residual_t *out);
gst_warmth_t gesture_residual_warmth(void);          /* fill-level warmth */

/* ── residual extraction ───────────────────────────────────────────────*/
#define GST_RES_FIR       5      /* centered moving-average taps (~5 Hz @ 1kHz) */
#define GST_RES_SLOW_MAX  2.0f   /* counts/report: slow|medium boundary */
#define GST_RES_MED_MAX   8.0f   /* counts/report: medium|fast boundary */

uint8_t  gesture_speed_bucket(float speed_cpr);
uint16_t gesture_residual_extract(uint16_t window);

/* ── streaming residual filter (v3, per-poll) ──────────────────────────
 * Adds real captured human residual on top of the app's injected delta.
 * Complement-only: zero-mean, debt-leaked to bound drift, attenuated at rest. */
#define GST_RES_HEAD_EWMA  0.30f   /* heading/speed smoothing factor */
#define GST_RES_DEBT_LEAK  0.10f   /* fraction of accumulated residual leaked back */
#define GST_RES_REST_CPR   0.5f    /* |app delta| below this = at rest */

void gesture_stream_filter(int16_t in_dx, int16_t in_dy, int16_t *out_dx, int16_t *out_dy);
void gesture_stream_reset(void);

/* ── honest-limit detector ─────────────────────────────────────────────
 * Flags app trends additive residual cannot launder: uniform fixed-magnitude
 * steps and super-human teleport snaps. Flags, never fakes. */
#define GST_NH_WIN        32
#define GST_TELEPORT_CPR  80.0f   /* |delta| above this in one report = teleport */

void     gesture_trend_observe(int16_t in_dx, int16_t in_dy);
uint32_t gesture_nonhuman_trend(void);
bool     gesture_trend_is_human(void);

/* ── humanization status snapshot ──────────────────────────────────────
 * Pure read of the residual/trend/warmth state for the LED + display.
 * replay_pct + synth_pct == 100 once warm and trending human, else 0. */
typedef struct {
    uint8_t warmth;      /* gst_warmth_t: 0 COLD, 1 WARMING, 2 WARM */
    uint8_t replay_pct;  /* residual injection share, 0..100          */
    uint8_t synth_pct;   /* non-human-trend share, 0..100             */
    uint8_t dup;         /* reserved, always 0                        */
} gst_human_status_t;

void gesture_human_status(gst_human_status_t *out);
