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

gst_warmth_t gesture_warmth(void);

/* ── PRNG (test hooks) ─────────────────────────────────────────────────
 * SFC32, seeded in gesture_init. Deterministic (0x12345678) under
 * GESTURE_HOSTTEST so augmentation/selection tests are reproducible. */
uint32_t gesture_rand_u32(void);
float    gesture_rand_range(float lo, float hi);   /* uniform [lo, hi) */

/* ── cadence view (real device report-rate jitter) ─────────────────────
 * Derived from the capture ring's t_us timestamps — no new clock, no new
 * storage, and humanize's EWMA/PIT path is untouched. Unlike that EWMA, raw
 * intervals are NOT outlier-rejected: the variance and coalescing are the
 * human/hardware signal the silent-path cadence reproduces. */
uint16_t gesture_cadence_count(void);                       /* 0 if < 2 samples */
bool     gesture_cadence_get(uint16_t age, uint32_t *out_dt_us); /* age 0 = newest */

/* ── click coupling (Plan 4) ───────────────────────────────────────────
 * Click kinematics measured from real button events, replayed around
 * injected motion. Decorator modulates ONLY the injected delta; the genuine
 * click/cursor passthrough is never touched. */
typedef struct {
    uint32_t decel_us;        /* decel time before press (uint32: see plan) */
    float    settle_px;       /* residual drift magnitude during hold     */
    uint32_t dwell_us;        /* press→release duration (uint32: >65ms)   */
    float    recoil_x, recoil_y; /* post-release drift vector             */
    uint8_t  flags;           /* bits0-1 pre-click velocity class, 2-3 btn */
} gst_click_env_t;            /* ring of GST_CLK_RING, RAM-only            */

#define GST_CLK_RING          16
#define GST_CLK_DWELL_MIN_US  10000u   /* reject sub-10ms "clicks"        */
#define GST_CLK_DWELL_MAX_US  600000u  /* reject >600ms holds             */

void     gesture_click_admit(const gst_click_env_t *env);
uint8_t  gesture_click_count(void);
bool     gesture_click_select(gst_click_env_t *out);  /* augmented copy; false if empty */
uint32_t gesture_click_admitted(void);                /* diagnostic */

/* Feed one real report (a copy) to the click-capture state machine. On a
 * completed press→release→recoil cycle that passes the quality gate, an
 * envelope is admitted. Never mutates the real report (passthrough sacred). */
void gesture_click_observe(int16_t dx, int16_t dy, uint8_t buttons, uint32_t t_us);

/* ── Mode 2: aim-assist suppression around a real click ────────────────
 * Feed the real button state each report; the injected-motion scale ramps to
 * a micro-drift floor during a real hold and back after release. Multiply the
 * source's injected delta by gesture_click_motion_scale(). Never emits a
 * button; never touches the real report. */
void  gesture_click_real_buttons(uint8_t buttons, uint32_t t_us);
float gesture_click_motion_scale(void);   /* [GST_C2_FLOOR, 1.0]            */
bool  gesture_click_real_active(void);     /* true while a real button held  */

/* ── Mode 1: self-fire (triggerbot) click sequence ─────────────────────
 * arm_fire selects a measured envelope and arms; fire_step drives a timed
 * PRESS → settle-bounded dwell drift → RELEASE → recoil sequence, returning
 * the button action to emit (Plan 5 routes it to act_click/TYPE_INJECT) and
 * the step's injected drift/recoil delta. Arming is refused while a real
 * click is active (arbitration: real wins). */
typedef enum { GST_CA_NONE = 0, GST_CA_PRESS = 1, GST_CA_RELEASE = 2 } gst_click_action_t;

bool               gesture_click_arm_fire(uint8_t button);
gst_click_action_t gesture_click_fire_step(uint32_t dt_us, float *out_dx, float *out_dy);
bool               gesture_click_fire_active(void);

/* ── residual store (Humanization v3) ──────────────────────────────────
 * Real captured high-frequency motion residual (tremor, motor noise,
 * off-axis wobble), speed-bucketed, drawn sequentially to preserve the
 * captured tremor autocorrelation. RAM-only, FIFO eviction. */
#define GST_RES_BUCKETS  3
#define GST_RES_RING     128
#define GST_RES_WARM_MIN 64    /* total samples before residual is trusted */

typedef struct {
    float    r_par;     /* along-heading speed fluctuation (counts/report) */
    float    r_perp;    /* perpendicular wobble (counts/report)            */
    uint16_t dt_us_lo;  /* low 16 bits of the source inter-report interval */
} gst_residual_t;       /* 12 bytes                                        */

void     gesture_residual_admit(uint8_t bucket, float r_par, float r_perp, uint16_t dt);
uint16_t gesture_residual_count(uint8_t bucket);     /* 0..GST_RES_RING */
uint16_t gesture_residual_total(void);
bool     gesture_residual_draw(uint8_t bucket, gst_residual_t *out);
gst_warmth_t gesture_residual_warmth(void);          /* fill-level warmth */

/* ── residual extraction (v3) ──────────────────────────────────────────*/
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

/* ── honest-limit detector (v3) ────────────────────────────────────────
 * Flags app trends additive residual cannot launder: uniform fixed-magnitude
 * steps and super-human teleport snaps. Flags, never fakes. */
#define GST_NH_WIN        32
#define GST_TELEPORT_CPR  80.0f   /* |delta| above this in one report = teleport */

void     gesture_trend_observe(int16_t in_dx, int16_t in_dy);
uint32_t gesture_nonhuman_trend(void);
bool     gesture_trend_is_human(void);

/* ── humanization status snapshot (Plan 5) ─────────────────────────────
 * Pure read of the residual/trend/warmth state for the LED + display.
 * replay_pct + synth_pct == 100 once warm and trending human, else 0. */
typedef struct {
    uint8_t warmth;      /* gst_warmth_t: 0 COLD, 1 WARMING, 2 WARM */
    uint8_t replay_pct;  /* residual injection share, 0..100          */
    uint8_t synth_pct;   /* non-human-trend share, 0..100             */
    uint8_t dup;         /* reserved, always 0                        */
} gst_human_status_t;

void gesture_human_status(gst_human_status_t *out);
