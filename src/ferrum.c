// src/ferrum.c — Ferrum ASCII command parser.
//
// Text API:
//   km.<name>(<args>)\r\n      (or trailing \n)
//   m(x, y)\r\n                (alias for km.move)
// Read commands reply value\r\n; write commands reply nothing; junk is dropped.
// No echo, no prompt. Callbacks (km.buttons/axes/keys) emit unsolicited lines
// from the HID merge hot-path; ferrum_notify_* early-out when disabled. No
// printf/sprintf/atoi — internal helpers keep newlib out of the image.

#include "ferrum.h"
#include "actions.h"
#include "kmbox.h"
#include <string.h>

extern uint32_t millis(void);

#define FERRUM_LINE_MAX  128
#define FERRUM_MAX_ARGS  8
// Drop a partial (unterminated) line if the next byte arrives more than this
// many ms later. Prevents a wrong-baud garbage burst from prefixing the first
// valid command after the host reopens at the correct baud.
#define FERRUM_IDLE_GAP_MS 25

// --- transport ---------------------------------------------------------------
static ferrum_tx_fn s_tx;

static inline void emit(const char *s, uint16_t len)
{
	if (s_tx) s_tx((const uint8_t *)s, len);
}

static inline void emit_cstr(const char *s)
{
	emit(s, (uint16_t)strlen(s));
}

// --- line buffer -------------------------------------------------------------
static char     s_line[FERRUM_LINE_MAX];
static uint8_t  s_line_pos;
static bool     s_overflow;
static uint32_t s_last_byte_ms;

// --- callback enable flags ---------------------------------------------------
static uint8_t s_cb_buttons;
static uint8_t s_cb_axes;
static uint8_t s_cb_keys;

// --- catch_xy state ----------------------------------------------------------
static struct {
	bool     active;
	uint32_t deadline;
	int32_t  accum_x;
	int32_t  accum_y;
} s_catch;

// --- last-seen keys for keys callback dedup ----------------------------------
static uint8_t s_last_keys[6];

// --- last-seen buttons bitmap for buttons callback dedup ---------------------
static uint8_t s_last_buttons;

// --- lock name-to-bit mapping ------------------------------------------------
// Bit assignments are local to ferrum.c; only g_lock_mask is shared.
#define LOCK_BIT_ML  0
#define LOCK_BIT_MR  1
#define LOCK_BIT_MM  2
#define LOCK_BIT_MS1 3
#define LOCK_BIT_MS2 4
#define LOCK_BIT_MX  5
#define LOCK_BIT_MY  6

// =============================================================================
// Formatters / parsers
// =============================================================================

// Parse a signed decimal integer from [s, s+len). Returns true on success.
// Whitespace is not tolerated; callers trim first.
static bool parse_int(const char *s, uint8_t len, int32_t *out)
{
	if (len == 0) return false;
	bool neg = false;
	uint8_t i = 0;
	if (s[0] == '-') { neg = true; i = 1; }
	else if (s[0] == '+') { i = 1; }
	if (i >= len) return false;
	uint32_t u = 0;
	for (; i < len; i++) {
		char c = s[i];
		if (c < '0' || c > '9') return false;
		uint32_t d = (uint32_t)(c - '0');
		// Reject values that overflow uint32_t to avoid signed overflow UB.
		if (u > (0xFFFFFFFFu - d) / 10u) return false;
		u = u * 10u + d;
	}
	if (neg) {
		if (u > 0x80000000u) return false;
		*out = -(int32_t)u;
	} else {
		if (u > 0x7FFFFFFFu) return false;
		*out = (int32_t)u;
	}
	return true;
}

// Parse a boolean literal "true" / "false".
static bool parse_bool(const char *s, uint8_t len, bool *out)
{
	if (len == 4 && s[0] == 't' && s[1] == 'r' && s[2] == 'u' && s[3] == 'e') {
		*out = true;
		return true;
	}
	if (len == 5 && s[0] == 'f' && s[1] == 'a' && s[2] == 'l' &&
	    s[3] == 's' && s[4] == 'e') {
		*out = false;
		return true;
	}
	return false;
}

// Format a signed decimal integer into buf. Returns chars written.
// Caller guarantees buf has at least 12 bytes (enough for INT32_MIN).
static uint8_t format_int(int32_t v, char *buf)
{
	char tmp[12];
	uint8_t n = 0;
	bool neg = false;
	uint32_t u;
	if (v < 0) { neg = true; u = (uint32_t)(-(int64_t)v); }
	else       u = (uint32_t)v;
	if (u == 0) tmp[n++] = '0';
	while (u) { tmp[n++] = (char)('0' + (u % 10u)); u /= 10u; }
	uint8_t out = 0;
	if (neg) buf[out++] = '-';
	while (n) buf[out++] = tmp[--n];
	return out;
}

// Emit a single decimal-digit reply ("0\r\n" or "1\r\n").
static void emit_bool(uint8_t v)
{
	char b[3] = { v ? '1' : '0', '\r', '\n' };
	emit(b, 3);
}

// Emit "(x, y)\r\n" with signed decimal ints.
static void emit_pair(int32_t x, int32_t y)
{
	char buf[32];
	uint8_t n = 0;
	buf[n++] = '(';
	n += format_int(x, &buf[n]);
	buf[n++] = ',';
	buf[n++] = ' ';
	n += format_int(y, &buf[n]);
	buf[n++] = ')';
	buf[n++] = '\r';
	buf[n++] = '\n';
	emit(buf, n);
}

// Emit and zero the catch_xy accumulator. Does not change s_catch.active;
// call sites differ on whether the state should remain active.
static void emit_catch_result(void)
{
	emit_pair(s_catch.accum_x, s_catch.accum_y);
	s_catch.accum_x = 0;
	s_catch.accum_y = 0;
}

// =============================================================================
// Argument tokenizer
// =============================================================================

typedef struct {
	const char *p;
	uint8_t     len;
} arg_t;

// Trim whitespace from a (ptr,len) slice in-place.
static void trim(const char **p, uint8_t *len)
{
	const char *s = *p;
	uint8_t n = *len;
	while (n && (*s == ' ' || *s == '\t')) { s++; n--; }
	while (n && (s[n - 1] == ' ' || s[n - 1] == '\t')) n--;
	*p = s; *len = n;
}

// Split arg list "a, b, c" into up to FERRUM_MAX_ARGS entries.
static uint8_t split_args(const char *s, uint8_t len, arg_t *args)
{
	uint8_t n = 0;
	uint8_t start = 0;
	for (uint8_t i = 0; i <= len; i++) {
		if (i == len || s[i] == ',') {
			if (n >= FERRUM_MAX_ARGS) return n;
			const char *p = &s[start];
			uint8_t l = i - start;
			trim(&p, &l);
			args[n].p = p;
			args[n].len = l;
			n++;
			start = i + 1;
		}
	}
	// A single empty slot (e.g. "km.foo()") counts as zero args.
	if (n == 1 && args[0].len == 0) return 0;
	return n;
}

// =============================================================================
// Command handlers
// =============================================================================

static void cmd_version(uint8_t nargs)
{
	(void)nargs;
	emit_cstr("kmbox: Ferrum\r\n");
}

static void cmd_move(arg_t *args, uint8_t nargs)
{
	if (nargs != 2) return;
	int32_t x, y;
	if (!parse_int(args[0].p, args[0].len, &x)) return;
	if (!parse_int(args[1].p, args[1].len, &y)) return;
	// Clamp to int16 instead of relying on implementation-defined narrowing.
	if (x > INT16_MAX) x = INT16_MAX;
	if (x < INT16_MIN) x = INT16_MIN;
	if (y > INT16_MAX) y = INT16_MAX;
	if (y < INT16_MIN) y = INT16_MIN;
	act_move((int16_t)x, (int16_t)y);
}

// Generic button handler. mask is the bit in g_buttons.
static void cmd_button(uint8_t mask, arg_t *args, uint8_t nargs)
{
	if (nargs == 0) {
		emit_bool((g_buttons & mask) ? 1 : 0);
		return;
	}
	if (nargs != 1) return;
	int32_t s;
	if (!parse_int(args[0].p, args[0].len, &s)) return;
	if (s != 0 && s != 1) return;
	act_button_set(mask, (uint8_t)s);
}

static void cmd_click(arg_t *args, uint8_t nargs)
{
	if (nargs != 1) return;
	int32_t b;
	if (!parse_int(args[0].p, args[0].len, &b)) return;
	if (b < 0 || b > 4) return;
	act_click((uint8_t)(b + 1), 1, 0);
}

static void cmd_wheel(arg_t *args, uint8_t nargs)
{
	if (nargs != 1) return;
	int32_t n;
	if (!parse_int(args[0].p, args[0].len, &n)) return;
	if (n > INT8_MAX) n = INT8_MAX;
	if (n < INT8_MIN) n = INT8_MIN;
	kmbox_inject_mouse(0, 0, g_buttons, (int8_t)n);
}

// Generic lock handler. bit is the LOCK_BIT_* index.
static void cmd_lock(uint8_t bit, arg_t *args, uint8_t nargs)
{
	uint16_t mask = (uint16_t)(1u << bit);
	if (nargs == 0) {
		emit_bool((g_lock_mask & mask) ? 1 : 0);
		return;
	}
	if (nargs != 1) return;
	int32_t s;
	if (!parse_int(args[0].p, args[0].len, &s)) return;
	if (s != 0 && s != 1) return;
	if (s) g_lock_mask |= mask;
	else   g_lock_mask &= ~mask;
}

static void cmd_catch_xy(arg_t *args, uint8_t nargs)
{
	if (nargs < 1 || nargs > 2) return;
	int32_t dur;
	if (!parse_int(args[0].p, args[0].len, &dur)) return;
	if (dur < 0) dur = 0;
	if (dur > 1000) dur = 1000;
	if (nargs == 2) {
		bool inc;
		if (!parse_bool(args[1].p, args[1].len, &inc)) return;
		(void)inc; // accepted for syntactic compatibility
	}
	// Re-entrant call: emit the prior request's accumulator so the original
	// caller's read still completes.
	if (s_catch.active) {
		emit_catch_result();
		s_catch.active = false;
	}
	s_catch.accum_x = 0;
	s_catch.accum_y = 0;
	s_catch.deadline = millis() + (uint32_t)dur;
	s_catch.active = true;
}

static void cmd_kb_down(arg_t *args, uint8_t nargs)
{
	if (nargs != 1) return;
	int32_t k;
	if (!parse_int(args[0].p, args[0].len, &k)) return;
	act_kb_down((uint8_t)k);
}

static void cmd_kb_up(arg_t *args, uint8_t nargs)
{
	if (nargs != 1) return;
	int32_t k;
	if (!parse_int(args[0].p, args[0].len, &k)) return;
	act_kb_up((uint8_t)k);
}

// Cheap jitter for HID-style press timing (75..125 ms).
static uint32_t press_delay(void)
{
	return 75u + (millis() % 51u);
}

static void cmd_kb_press(arg_t *args, uint8_t nargs)
{
	if (nargs != 1) return;
	int32_t k;
	if (!parse_int(args[0].p, args[0].len, &k)) return;
	act_kb_press((uint8_t)k, press_delay());
}

static void cmd_kb_multi(arg_t *args, uint8_t nargs, uint8_t op)
{
	// op: 0=down, 1=up, 2=press
	if (nargs == 0 || nargs > 6) return;
	int32_t k[6];
	for (uint8_t i = 0; i < nargs; i++) {
		if (!parse_int(args[i].p, args[i].len, &k[i])) return;
	}
	for (uint8_t i = 0; i < nargs; i++) {
		uint8_t key = (uint8_t)k[i];
		switch (op) {
		case 0: act_kb_down(key); break;
		case 1: act_kb_up(key); break;
		case 2: act_kb_press(key, press_delay()); break;
		}
	}
}

static void cmd_kb_isdown(arg_t *args, uint8_t nargs)
{
	if (nargs != 1) return;
	int32_t k;
	if (!parse_int(args[0].p, args[0].len, &k)) return;
	emit_bool(act_kb_isdown((uint8_t)k));
}

static void cmd_kb_mask(arg_t *args, uint8_t nargs)
{
	if (nargs == 1) {
		// Read variant: no getter for the masked-key table, so report 0.
		int32_t k;
		if (!parse_int(args[0].p, args[0].len, &k)) return;
		(void)k;
		emit_bool(0);
		return;
	}
	if (nargs != 2) return;
	int32_t k, s;
	if (!parse_int(args[0].p, args[0].len, &k)) return;
	if (!parse_int(args[1].p, args[1].len, &s)) return;
	act_kb_mask((uint8_t)k, (uint8_t)s);
}

static void cmd_init(uint8_t nargs)
{
	(void)nargs;
	act_init();
	act_kb_init();  // flush a zero HID keyboard report
}

static void cmd_cb_toggle(uint8_t *flag, arg_t *args, uint8_t nargs)
{
	if (nargs == 0) {
		emit_bool(*flag ? 1 : 0);
		return;
	}
	if (nargs != 1) return;
	int32_t s;
	if (!parse_int(args[0].p, args[0].len, &s)) return;
	if (s != 0 && s != 1) return;
	*flag = (uint8_t)s;
	if (flag == &s_cb_keys) {
		memset(s_last_keys, 0xFF, sizeof(s_last_keys));  // force re-emit
	}
}

static void cmd_baud(arg_t *args, uint8_t nargs)
{
	if (nargs != 1) return;
	int32_t n;
	if (!parse_int(args[0].p, args[0].len, &n)) return;
	if (n <= 0) return;
	kmbox_set_baud((uint32_t)n);
}

// =============================================================================
// Dispatch
// =============================================================================

// Match the command name between "km." and "(". Returns true on match.
static inline bool name_is(const char *s, uint8_t len, const char *kw)
{
	uint8_t kl = (uint8_t)strlen(kw);
	return len == kl && memcmp(s, kw, kl) == 0;
}

static void dispatch(const char *name, uint8_t name_len, arg_t *args, uint8_t nargs)
{
	if (name_is(name, name_len, "version"))      { cmd_version(nargs); return; }
	if (name_is(name, name_len, "move"))         { cmd_move(args, nargs); return; }

	if (name_is(name, name_len, "left"))         { cmd_button(0x01, args, nargs); return; }
	if (name_is(name, name_len, "right"))        { cmd_button(0x02, args, nargs); return; }
	if (name_is(name, name_len, "middle"))       { cmd_button(0x04, args, nargs); return; }
	if (name_is(name, name_len, "side1"))        { cmd_button(0x08, args, nargs); return; }
	if (name_is(name, name_len, "side2"))        { cmd_button(0x10, args, nargs); return; }

	if (name_is(name, name_len, "click"))        { cmd_click(args, nargs); return; }
	if (name_is(name, name_len, "wheel"))        { cmd_wheel(args, nargs); return; }

	if (name_is(name, name_len, "lock_ml"))      { cmd_lock(LOCK_BIT_ML,  args, nargs); return; }
	if (name_is(name, name_len, "lock_mr"))      { cmd_lock(LOCK_BIT_MR,  args, nargs); return; }
	if (name_is(name, name_len, "lock_mm"))      { cmd_lock(LOCK_BIT_MM,  args, nargs); return; }
	if (name_is(name, name_len, "lock_ms1"))     { cmd_lock(LOCK_BIT_MS1, args, nargs); return; }
	if (name_is(name, name_len, "lock_ms2"))     { cmd_lock(LOCK_BIT_MS2, args, nargs); return; }
	if (name_is(name, name_len, "lock_mx"))      { cmd_lock(LOCK_BIT_MX,  args, nargs); return; }
	if (name_is(name, name_len, "lock_my"))      { cmd_lock(LOCK_BIT_MY,  args, nargs); return; }

	if (name_is(name, name_len, "catch_xy"))     { cmd_catch_xy(args, nargs); return; }

	if (name_is(name, name_len, "down"))         { cmd_kb_down(args, nargs); return; }
	if (name_is(name, name_len, "up"))           { cmd_kb_up(args, nargs); return; }
	if (name_is(name, name_len, "press"))        { cmd_kb_press(args, nargs); return; }
	if (name_is(name, name_len, "multidown"))    { cmd_kb_multi(args, nargs, 0); return; }
	if (name_is(name, name_len, "multiup"))      { cmd_kb_multi(args, nargs, 1); return; }
	if (name_is(name, name_len, "multipress"))   { cmd_kb_multi(args, nargs, 2); return; }
	if (name_is(name, name_len, "isdown"))       { cmd_kb_isdown(args, nargs); return; }
	if (name_is(name, name_len, "mask"))         { cmd_kb_mask(args, nargs); return; }

	if (name_is(name, name_len, "init"))         { cmd_init(nargs); return; }

	if (name_is(name, name_len, "buttons"))      { cmd_cb_toggle(&s_cb_buttons, args, nargs); return; }
	if (name_is(name, name_len, "axes"))         { cmd_cb_toggle(&s_cb_axes,    args, nargs); return; }
	if (name_is(name, name_len, "keys"))         { cmd_cb_toggle(&s_cb_keys,    args, nargs); return; }

	if (name_is(name, name_len, "baud"))         { cmd_baud(args, nargs); return; }

	// Unknown — silent drop.
}

// =============================================================================
// Line dispatcher
// =============================================================================

static void dispatch_line(char *line, uint8_t len)
{
	uint8_t i = 0;
	while (i < len && (line[i] == ' ' || line[i] == '\t')) i++;
	if (i >= len) return;

	const char *body = NULL;
	uint8_t     body_len = 0;
	const char *name = NULL;
	uint8_t     name_len = 0;

	// Alias: "m(x, y)" → km.move
	if (line[i] == 'm' && i + 1 < len && line[i + 1] == '(') {
		name = "move";
		name_len = 4;
		body = &line[i + 2];
		body_len = (uint8_t)(len - (i + 2));
	} else if (i + 3 <= len && line[i] == 'k' && line[i + 1] == 'm' &&
	           line[i + 2] == '.') {
		// km.<name>(<args>)
		uint8_t name_start = i + 3;
		uint8_t p = name_start;
		while (p < len && line[p] != '(') p++;
		if (p >= len) return; // no '(' — junk
		name = &line[name_start];
		name_len = (uint8_t)(p - name_start);
		body = &line[p + 1];
		body_len = (uint8_t)(len - (p + 1));
	} else {
		return;
	}

	if (name_len == 0) return;

	// Find trailing ')'.  Garbage between ')' and EOL is tolerated.
	uint8_t b = body_len;
	while (b > 0 && body[b - 1] != ')') b--;
	if (b == 0) return; // no ')' — junk
	b--; // exclude ')'

	arg_t args[FERRUM_MAX_ARGS];
	uint8_t nargs = split_args(body, b, args);

	dispatch(name, name_len, args, nargs);
}

// =============================================================================
// Public API
// =============================================================================

void ferrum_init(void)
{
	s_line_pos = 0;
	s_overflow = false;
	s_cb_buttons = 0;
	s_cb_axes = 0;
	s_cb_keys = 0;
	memset(&s_catch, 0, sizeof(s_catch));
	memset(s_last_keys, 0, sizeof(s_last_keys));
	s_last_buttons = 0;
}

void ferrum_reset(void)
{
	s_line_pos = 0;
	s_overflow = false;
}

void ferrum_set_tx(ferrum_tx_fn tx)
{
	s_tx = tx;
}

void ferrum_tick(void)
{
	// Drives the catch_xy deadline check independent of UART RX, so a host
	// that calls km.catch_xy(dur) then idles still gets its reply.
	if (__builtin_expect(!s_catch.active, 1)) return;
	if (millis() < s_catch.deadline) return;
	emit_catch_result();
	s_catch.active = false;
}

void ferrum_feed_byte(uint8_t b)
{
	uint32_t now = millis();

	// catch_xy: emit immediately when new bytes arrive after the deadline,
	// rather than waiting for the next ferrum_tick().
	if (__builtin_expect(s_catch.active, 0) && now >= s_catch.deadline) {
		emit_catch_result();
		s_catch.active = false;
	}

	// Idle-gap reset (see FERRUM_IDLE_GAP_MS).
	if (s_line_pos > 0 && (now - s_last_byte_ms) > FERRUM_IDLE_GAP_MS) {
		s_line_pos = 0;
		s_overflow = false;
	}
	s_last_byte_ms = now;

	if (b == '\r' || b == '\n') {
		if (s_line_pos > 0 && !s_overflow) {
			s_line[s_line_pos] = '\0';
			dispatch_line(s_line, s_line_pos);
		}
		s_line_pos = 0;
		s_overflow = false;
		return;
	}
	if (s_line_pos < FERRUM_LINE_MAX - 1) {
		s_line[s_line_pos++] = (char)b;
	} else {
		s_overflow = true;
	}
}

// =============================================================================
// Callback hooks (HID merge hot-path)
// =============================================================================

void ferrum_notify_buttons(uint8_t buttons_bitmap)
{
	if (__builtin_expect(s_cb_buttons == 0, 1)) return;
	// Dedup so an unchanging mouse does not flood the TX ring at poll rate.
	if (buttons_bitmap == s_last_buttons) return;
	s_last_buttons = buttons_bitmap;
	// "km.<bitmap_byte>\r\n": a single binary byte after "km."
	uint8_t buf[6];
	buf[0] = 'k';
	buf[1] = 'm';
	buf[2] = '.';
	buf[3] = buttons_bitmap;
	buf[4] = '\r';
	buf[5] = '\n';
	if (s_tx) s_tx(buf, 6);
}

void ferrum_notify_axes(int16_t dx, int16_t dy, int8_t scroll)
{
	// catch_xy accumulator runs even when the callback is disabled.
	if (__builtin_expect(s_catch.active, 0)) {
		s_catch.accum_x += dx;
		s_catch.accum_y += dy;
	}

	if (__builtin_expect(s_cb_axes == 0, 1)) return;
	char buf[40];
	uint8_t n = 0;
	const char *prefix = "Axes(";
	for (; *prefix; prefix++) buf[n++] = *prefix;
	n += format_int(dx, &buf[n]);
	buf[n++] = ','; buf[n++] = ' ';
	n += format_int(dy, &buf[n]);
	buf[n++] = ','; buf[n++] = ' ';
	n += format_int(scroll, &buf[n]);
	buf[n++] = ')';
	buf[n++] = '\r';
	buf[n++] = '\n';
	emit(buf, n);
}

void ferrum_notify_keys(const uint8_t keys[6])
{
	if (__builtin_expect(s_cb_keys == 0, 1)) return;

	// Insertion-sort a copy; empty slots (0) sort to the front and are
	// filtered out when formatting.
	uint8_t sorted[6];
	memcpy(sorted, keys, 6);
	for (uint8_t i = 1; i < 6; i++) {
		uint8_t v = sorted[i];
		int8_t  j = (int8_t)i - 1;
		while (j >= 0 && sorted[j] > v) {
			sorted[j + 1] = sorted[j];
			j--;
		}
		sorted[j + 1] = v;
	}

	if (memcmp(sorted, s_last_keys, 6) == 0) return;
	memcpy(s_last_keys, sorted, 6);

	char buf[48];
	uint8_t n = 0;
	const char *prefix = "Keys(";
	for (; *prefix; prefix++) buf[n++] = *prefix;
	bool first = true;
	for (uint8_t i = 0; i < 6; i++) {
		if (sorted[i] == 0) continue;
		if (!first) { buf[n++] = ','; buf[n++] = ' '; }
		n += format_int(sorted[i], &buf[n]);
		first = false;
	}
	buf[n++] = ')';
	buf[n++] = '\r';
	buf[n++] = '\n';
	emit(buf, n);
}
