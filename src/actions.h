// src/actions.h — transport-agnostic injection actions
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define ACT_MAX_DISABLED_KEYS 32

// Mouse + keyboard state — transport-agnostic, accessed by ferrum.c.
extern uint8_t  g_buttons;
extern uint8_t  g_kb_modifier;
extern uint8_t  g_kb_keys[6];
extern int32_t  g_pos_x, g_pos_y;
extern uint16_t g_lock_mask;

void    act_init(void);
int8_t  act_button_set(uint8_t mask, uint8_t action);    // action: 0=up, 1=down
void    act_click(uint8_t button_1based, uint8_t count, uint32_t delay_ms);
void    act_move(int16_t dx, int16_t dy);
int8_t  act_kb_down(uint8_t key);
void    act_kb_up(uint8_t key);
void    act_kb_press(uint8_t key, uint32_t delay_ms);
uint8_t act_kb_isdown(uint8_t key);
void    act_kb_init(void);
void    act_kb_mask(uint8_t key, uint8_t mode);

void act_wheel(int8_t ticks);

bool act_get_invert_x(void);
void act_set_invert_x(bool on);
bool act_get_invert_y(void);
void act_set_invert_y(bool on);

bool act_get_swap_xy(void);
void act_set_swap_xy(bool on);

// ── Physical-input masking (`mask` / `unmask_all`) ──────────────────────────
// A masked physical input is suppressed before it reaches the downstream PC,
// while injected input on the same control still passes. Enforced in the merge
// path; these only manage the mask state. The mouse-button/axis bits reuse the
// lock-bit order (ml=0,mr=1,mm=2,ms1=3,ms2=4,mx=5,my=6) but in a dedicated
// bitmap so they do not collide with g_lock_mask's injection gating.
#define PHYS_MASK_ML    0
#define PHYS_MASK_MR    1
#define PHYS_MASK_MM    2
#define PHYS_MASK_MS1   3
#define PHYS_MASK_MS2   4
#define PHYS_MASK_MX    5
#define PHYS_MASK_MY    6
#define PHYS_MASK_WHEEL 7
extern uint16_t g_phys_mask;            // bit i set → suppress physical input i

void act_phys_mask_mouse(uint8_t code, bool enable);  // code 0..7 (see above)
void act_phys_mask_key(uint8_t hid_key, bool enable);
void act_phys_unmask_all(void);
bool act_phys_key_masked(uint8_t hid_key);            // queried in merge path
bool act_phys_kb_mask_active(void);                   // any key currently masked

// ── Motion program (`mouse_move_auto` / `mouse_move_bezier`) ─────────────────
// A time-bounded source of injected delta: each tick emits the increment needed
// to reach the trajectory's position at the current time, via act_move (so
// inverts/swap and humanization apply as for manual moves). Position-based, so
// call cadence affects granularity only, not the total or endpoint. Last-writer
// -wins: a new program or any plain act_move cancels the in-flight one.
void act_motion_move_dur(int16_t dx, int16_t dy, uint16_t dur_ms);
void act_motion_bezier(int16_t dx, int16_t dy, uint16_t dur_ms,
                       int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void act_motion_tick(void);             // step from the poll loop
void act_motion_cancel(void);           // abort in-flight program

/* ── motion-source hook (Plan 5) ───────────────────────────────────────
 * Lets a platform redirect trajectory motion (act_motion_move_dur/_bezier)
 * to an alternate provider — the V5F gesture replay engine — without this
 * shared file depending on it. NULL (default) = the built-in analytic
 * Bézier/smoothstep path. begin() is handed the same args the analytic
 * path would use; tick() emits one integer step per call and returns 0 when
 * the trajectory is complete; cancel() aborts an in-flight source. */
typedef struct {
    void (*begin)(int32_t dx, int32_t dy, uint16_t dur_ms, int bezier,
                  int32_t x1, int32_t y1, int32_t x2, int32_t y2);
    int  (*tick)(int16_t *out_dx, int16_t *out_dy);
    void (*cancel)(void);
} act_motion_source_t;

void act_motion_set_source(const act_motion_source_t *src);  /* NULL = analytic */

/* ── streaming injection filter (Humanization v3) ──────────────────────
 * Transforms each individual injected move delta (the streaming km.move
 * path) before it is emitted. NULL (default) = passthrough. Registered only
 * on the V5F host, where the gesture residual filter lives. */
typedef struct {
    void (*apply)(int16_t in_dx, int16_t in_dy, int16_t *out_dx, int16_t *out_dy);
} act_stream_filter_t;

void act_set_stream_filter(const act_stream_filter_t *f);   /* NULL = passthrough */
