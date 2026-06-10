// src/actions.c — transport-agnostic injection actions (mouse + keyboard state)

#include "actions.h"
#include "kmbox.h"
#include <string.h>

extern uint32_t millis(void);

// Mouse + keyboard state
uint8_t  g_buttons;
uint8_t  g_kb_modifier;
uint8_t  g_kb_keys[6];
int32_t  g_pos_x, g_pos_y;
uint16_t g_lock_mask;
uint16_t g_phys_mask;   // physical-input suppression (see actions.h PHYS_MASK_*)

typedef struct {
	uint8_t  button;
	uint8_t  remaining;
	uint32_t delay_ms;
	uint32_t next_at;
	bool     pressed;
} act_click_sched_t;

static act_click_sched_t g_click_sched;
static uint8_t  g_masked_keys[ACT_MAX_DISABLED_KEYS];
static uint8_t  g_masked_modes[ACT_MAX_DISABLED_KEYS];
static uint8_t  g_masked_count;

static bool s_invert_x = false;
static bool s_invert_y = false;
static bool s_swap_xy  = false;

// ── motion program state (Feature A) ─────────────────────────────────────────
// Endpoint and control points are relative to the program's start. Positions
// are tracked as fixed-point .8 (sub-count precision) so each tick can emit the
// exact integer increment toward curve(t) with no cumulative rounding drift.
typedef enum { MOTION_NONE = 0, MOTION_LINEAR, MOTION_BEZIER } motion_kind_t;
static struct {
	motion_kind_t kind;
	uint32_t start_ms;
	uint32_t dur_ms;
	int32_t  ex, ey;        // endpoint delta (counts)
	int32_t  c1x, c1y;      // bezier control point 1 (counts)
	int32_t  c2x, c2y;      // bezier control point 2 (counts)
	int32_t  emit_x, emit_y; // counts already emitted toward the path
} g_motion;

static uint8_t btn_idx_to_mask(uint8_t idx)
{
	if (idx >= 1 && idx <= 5)
		return 1u << (idx - 1);
	return 0;
}

void act_init(void)
{
	g_buttons = 0;
	g_kb_modifier = 0;
	memset(g_kb_keys, 0, sizeof(g_kb_keys));
	g_pos_x = 0;
	g_pos_y = 0;
	g_lock_mask = 0;
	g_phys_mask = 0;
	memset(&g_click_sched, 0, sizeof(g_click_sched));
	memset(&g_motion, 0, sizeof(g_motion));
	g_masked_count = 0;
}

int8_t act_button_set(uint8_t mask, uint8_t action)
{
	if (action == 0) {
		g_buttons &= ~mask;
		kmbox_inject_mouse(0, 0, g_buttons, 0);
	} else if (action == 1) {
		g_buttons |= mask;
		kmbox_inject_mouse(0, 0, g_buttons, 0);
	} else if (action == 2) {
		g_buttons &= ~mask;
	} else {
		return -1;
	}
	return 0;
}

void act_click(uint8_t button_1based, uint8_t count, uint32_t delay_ms)
{
	uint8_t mask = btn_idx_to_mask(button_1based);
	g_buttons |= mask;
	kmbox_inject_mouse(0, 0, g_buttons, 0);

	if (count == 1) {
		kmbox_schedule_click_release(mask, delay_ms);
	} else {
		g_click_sched.button = button_1based;
		g_click_sched.remaining = count - 1;
		g_click_sched.delay_ms = delay_ms;
		g_click_sched.next_at = millis() + delay_ms;
		g_click_sched.pressed = true;
	}
}

// Core mover: applies swap/invert transforms and injects. Shared by the public
// act_move and the motion-program tick. Does NOT cancel the motion program, so
// the tick can drive it without self-aborting.
static void act_move_raw(int16_t dx, int16_t dy)
{
	if (s_swap_xy)  { int16_t t = dx; dx = dy; dy = t; }
	if (s_invert_x) dx = (int16_t)-dx;
	if (s_invert_y) dy = (int16_t)-dy;
	g_pos_x += dx;
	g_pos_y += dy;
	kmbox_inject_mouse(dx, dy, g_buttons, 0);
}

void act_move(int16_t dx, int16_t dy)
{
	// A manual move overrides any in-flight trajectory (user redirect wins).
	g_motion.kind = MOTION_NONE;
	act_move_raw(dx, dy);
}

int8_t act_kb_down(uint8_t key)
{
	if (key >= 0xE0 && key <= 0xE7) {
		g_kb_modifier |= (1u << (key - 0xE0));
	} else {
		bool added = false;
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == key) { added = true; break; }
			if (g_kb_keys[i] == 0) { g_kb_keys[i] = key; added = true; break; }
		}
		if (!added) return -1;
	}
	kmbox_inject_keyboard(g_kb_modifier, g_kb_keys);
	return 0;
}

void act_kb_up(uint8_t key)
{
	if (key >= 0xE0 && key <= 0xE7) {
		g_kb_modifier &= ~(1u << (key - 0xE0));
	} else {
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == key) { g_kb_keys[i] = 0; break; }
		}
	}
	kmbox_inject_keyboard(g_kb_modifier, g_kb_keys);
}

void act_kb_press(uint8_t key, uint32_t delay_ms)
{
	if (key >= 0xE0 && key <= 0xE7) {
		g_kb_modifier |= (1u << (key - 0xE0));
	} else {
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == key) break;
			if (g_kb_keys[i] == 0) { g_kb_keys[i] = key; break; }
		}
	}
	kmbox_inject_keyboard(g_kb_modifier, g_kb_keys);
	kmbox_schedule_kb_release(key, delay_ms);
}

uint8_t act_kb_isdown(uint8_t key)
{
	if (key >= 0xE0 && key <= 0xE7)
		return (g_kb_modifier & (1u << (key - 0xE0))) ? 1 : 0;
	for (int i = 0; i < 6; i++)
		if (g_kb_keys[i] == key) return 1;
	return 0;
}

void act_kb_init(void)
{
	g_kb_modifier = 0;
	memset(g_kb_keys, 0, sizeof(g_kb_keys));
	kmbox_inject_keyboard(0, g_kb_keys);
}

void act_kb_mask(uint8_t key, uint8_t mode)
{
	if (mode == 0) {
		for (uint8_t i = 0; i < g_masked_count; i++) {
			if (g_masked_keys[i] == key) {
				g_masked_keys[i] = g_masked_keys[--g_masked_count];
				g_masked_modes[i] = g_masked_modes[g_masked_count];
				break;
			}
		}
	} else {
		for (uint8_t i = 0; i < g_masked_count; i++) {
			if (g_masked_keys[i] == key) {
				g_masked_modes[i] = mode;
				return;
			}
		}
		if (g_masked_count < ACT_MAX_DISABLED_KEYS) {
			g_masked_keys[g_masked_count] = key;
			g_masked_modes[g_masked_count] = mode;
			g_masked_count++;
		}
	}
}

void act_wheel(int8_t ticks)
{
	kmbox_inject_mouse(0, 0, g_buttons, ticks);
}

bool act_get_invert_x(void) { return s_invert_x; }
void act_set_invert_x(bool on) { s_invert_x = on; }
bool act_get_invert_y(void) { return s_invert_y; }
void act_set_invert_y(bool on) { s_invert_y = on; }
bool act_get_swap_xy(void)  { return s_swap_xy; }
void act_set_swap_xy(bool on) { s_swap_xy = on; }

// ── physical-input masking ───────────────────────────────────────────────────
void act_phys_mask_mouse(uint8_t code, bool enable)
{
	if (code > PHYS_MASK_WHEEL) return;
	uint16_t bit = (uint16_t)(1u << code);
	if (enable) g_phys_mask |= bit;
	else        g_phys_mask &= (uint16_t)~bit;
}

void act_phys_mask_key(uint8_t hid_key, bool enable)
{
	// Reuse the masked-key table; mode 1 = masked. act_kb_mask owns the table.
	act_kb_mask(hid_key, enable ? 1 : 0);
}

void act_phys_unmask_all(void)
{
	g_phys_mask = 0;
	g_masked_count = 0;
}

bool act_phys_key_masked(uint8_t hid_key)
{
	for (uint8_t i = 0; i < g_masked_count; i++)
		if (g_masked_keys[i] == hid_key && g_masked_modes[i] != 0)
			return true;
	return false;
}

bool act_phys_kb_mask_active(void) { return g_masked_count != 0; }

// ── motion program (Feature A) ───────────────────────────────────────────────
// Cubic Bézier from the origin: B(t) = 3(1-t)^2 t·C1 + 3(1-t) t^2·C2 + t^3·E,
// with the start point at (0,0) folded in (its (1-t)^3 term is zero). Evaluated
// per-axis in fixed-point; only the integer delta from the last emitted point
// is injected, so rounding never accumulates and the final tick lands exactly
// on the endpoint.
static int32_t bezier_axis(int32_t c1, int32_t c2, int32_t e, uint32_t t_q, uint32_t one)
{
	// t_q, one are .16 fixed-point in [0, 65536]. Returns counts (rounded).
	// Work in 64-bit: coefficients are small (≤ ~32767) but t^3 scaling needs room.
	uint64_t t  = t_q;
	uint64_t u  = one - t_q;             // (1 - t)
	uint64_t t2 = (t * t) >> 16;
	uint64_t u2 = (u * u) >> 16;
	uint64_t b1 = (3u * ((u2 * t) >> 16));      // 3(1-t)^2 t   (.16)
	uint64_t b2 = (3u * ((t2 * u) >> 16));      // 3(1-t) t^2   (.16)
	uint64_t b3 = ((t2 * t) >> 16);             // t^3          (.16)
	int64_t acc = (int64_t)b1 * c1 + (int64_t)b2 * c2 + (int64_t)b3 * e;
	// acc is counts<<16; round to nearest.
	return (int32_t)((acc + (acc >= 0 ? (1 << 15) : -(1 << 15))) >> 16);
}

// Smoothstep ease (3t^2 - 2t^3) for automove — a human-like accel/decel profile.
// Returns position fraction in .16 fixed-point for input t_q in .16.
static uint32_t ease_smoothstep(uint32_t t_q, uint32_t one)
{
	uint64_t t  = t_q;
	uint64_t t2 = (t * t) >> 16;
	uint64_t t3 = (t2 * t) >> 16;
	// 3t^2 - 2t^3, all .16
	int64_t s = (int64_t)(3u * t2) - (int64_t)(2u * t3);
	if (s < 0) s = 0;
	if (s > (int64_t)one) s = (int64_t)one;
	return (uint32_t)s;
}

static void motion_start_common(int16_t dx, int16_t dy, uint16_t dur_ms)
{
	g_motion.start_ms = millis();
	g_motion.dur_ms   = dur_ms;
	g_motion.ex = dx;
	g_motion.ey = dy;
	g_motion.emit_x = 0;
	g_motion.emit_y = 0;
}

void act_motion_move_dur(int16_t dx, int16_t dy, uint16_t dur_ms)
{
	if (dur_ms == 0) { act_move(dx, dy); return; }   // immediate
	motion_start_common(dx, dy, dur_ms);
	g_motion.kind = MOTION_LINEAR;
}

void act_motion_bezier(int16_t dx, int16_t dy, uint16_t dur_ms,
                       int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	if (dur_ms == 0) { act_move(dx, dy); return; }
	motion_start_common(dx, dy, dur_ms);
	g_motion.c1x = x1; g_motion.c1y = y1;
	g_motion.c2x = x2; g_motion.c2y = y2;
	g_motion.kind = MOTION_BEZIER;
}

void act_motion_cancel(void) { g_motion.kind = MOTION_NONE; }

void act_motion_tick(void)
{
	if (g_motion.kind == MOTION_NONE) return;

	uint32_t elapsed = millis() - g_motion.start_ms;
	bool last = (elapsed >= g_motion.dur_ms);
	uint32_t t_q;                                   // .16 in [0, 65536]
	if (last) {
		t_q = 65536u;
	} else {
		// t = elapsed / dur, in .16. Kept strictly 32-bit so this maps to the
		// Cortex-M7 hardware UDIV (bounded 2–12 cy, deterministic) instead of the
		// software __aeabi_uldivmod (data-dependent loop). Safe in 32 bits: this
		// branch only runs when elapsed < dur_ms ≤ 65535, so elapsed ≤ 65534 and
		// (elapsed << 16) ≤ 0xFFFE0000 — no overflow of uint32_t.
		t_q = (elapsed << 16) / g_motion.dur_ms;
	}

	int32_t px, py;                                 // target position at t (counts)
	if (g_motion.kind == MOTION_BEZIER) {
		px = bezier_axis(g_motion.c1x, g_motion.c2x, g_motion.ex, t_q, 65536u);
		py = bezier_axis(g_motion.c1y, g_motion.c2y, g_motion.ey, t_q, 65536u);
	} else {
		uint32_t s = ease_smoothstep(t_q, 65536u);  // eased fraction .16
		px = (int32_t)(((int64_t)g_motion.ex * s) >> 16);
		py = (int32_t)(((int64_t)g_motion.ey * s) >> 16);
	}

	int32_t step_x = px - g_motion.emit_x;
	int32_t step_y = py - g_motion.emit_y;

	// Clamp each step to int16 for the injection path. emit_x/y advances only by
	// what is actually emitted, so any clamped residue is carried to the next
	// tick rather than dropped (keeps total/endpoint exact). Huge per-tick steps
	// only occur if the loop stalls for many ms — vanishingly rare.
	if (step_x >  INT16_MAX) step_x =  INT16_MAX;
	if (step_x <  INT16_MIN) step_x =  INT16_MIN;
	if (step_y >  INT16_MAX) step_y =  INT16_MAX;
	if (step_y <  INT16_MIN) step_y =  INT16_MIN;
	g_motion.emit_x += step_x;
	g_motion.emit_y += step_y;

	if (step_x || step_y)
		act_move_raw((int16_t)step_x, (int16_t)step_y);

	// Only finish once the endpoint is fully delivered (clamped residue could
	// otherwise leave the last counts unsent on the final tick).
	if (last && g_motion.emit_x == g_motion.ex && g_motion.emit_y == g_motion.ey)
		g_motion.kind = MOTION_NONE;
}
