// src/usb_merge.c — HID-report merge module (V5F side of the MITM).
//
// Overlays injected input onto the real device's HID reports. The injection
// source is the ICC ring (usb_merge_drain_icc decodes records into the `inject`
// accumulators); the transport to the host PC and the protocol parser live on
// the V3F command core.
//
// proto_* physical-telemetry calls are stubbed to no-ops with proto_phys_enabled()
// constant-false: the telemetry's wire destination (the host UART) lives on V3F.
// The compiler folds the constant-false predicate and eliminates the phys-capture
// branch.
//
// The injection accumulators are also fed by a standalone synth path
// (usb_merge_send_pending) that emits injected motion when the physical mouse is
// silent, capped one-per-ms and gated by merged_this_cycle so the merge and synth
// paths never both emit in one frame. humanize_filter() runs per emitted frame
// inside usb_merge_take_injection(), not per command, to preserve sub-pixel-carry
// behavior.

#include "usb_merge.h"
#include "hid_layout.h"   // mouse_layout_t, hid_layout_parse_mouse/read_field
#include "humanize.h"
#include "actions.h"      // g_phys_mask, act_phys_* helpers, PHYS_MASK_*
#include "icc.h"
#include "usb_device.h"
#include <string.h>
#include <stdbool.h>

extern uint32_t millis(void);

// proto_* physical-telemetry stubs. V5F has no protocol parser or host UART, so
// the notify hooks are no-ops and proto_phys_enabled() is constant-false, which
// compiles out the (g_phys_mask || proto_phys_enabled()) capture branches. The
// mask half (g_phys_mask) still works, driven from actions.c state.
static inline bool proto_phys_enabled(void) { return false; }
static inline void proto_notify_buttons(uint8_t b) { (void)b; }
static inline void proto_notify_axes(int16_t dx, int16_t dy, int8_t w) { (void)dx; (void)dy; (void)w; }
static inline void proto_notify_keys(const uint8_t *keys) { (void)keys; }
static inline void proto_notify_phys_buttons(uint8_t b) { (void)b; }
static inline void proto_notify_phys_axes(int16_t dx, int16_t dy, int8_t w) { (void)dx; (void)dy; (void)w; }
static inline void proto_notify_phys_keys(uint8_t mod, const uint8_t keys[6]) { (void)mod; (void)keys; }

// ── Injection accumulators ───────────────────────────────────────────────────
typedef struct {
	int16_t  mouse_dx;
	int16_t  mouse_dy;
	uint8_t  mouse_buttons;
	int8_t   mouse_wheel;
	bool     mouse_dirty;

	uint8_t  kb_modifier;
	uint8_t  kb_keys[6];
	bool     kb_dirty;

	uint8_t  click_release_mask;
	uint32_t click_release_at; // ms, 0=off

	#define MAX_KB_RELEASES 6
	struct { uint8_t key; uint32_t at; } kb_releases[6];
	uint8_t kb_release_count;
} merge_inject_t;

static merge_inject_t inject;

// ── Cached endpoints and parsed mouse layout ─────────────────────────────────
static uint8_t  cached_mouse_ep;
static uint16_t cached_mouse_maxpkt;
static uint8_t  cached_kb_ep;
static mouse_layout_t mouse_layout;
static uint8_t cached_mouse_report_len; // length captured from the first real report

void usb_merge_init(void)
{
	memset(&inject, 0, sizeof(inject));
	cached_mouse_ep = 0;
	cached_mouse_maxpkt = 0;
	cached_kb_ep = 0;
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;
	cached_mouse_report_len = 0;
}

// Bit-field write helper. The read side and the descriptor parser were factored
// out to hid_layout.c (hid_layout_read_field / hid_layout_parse_mouse) so the
// host-side catch_xy feed shares them; the write side stays here (merge-only).
static void write_report_field(uint8_t *buf, uint16_t buf_len, uint16_t bit_off,
                               uint8_t bit_size, uint8_t data_off, int32_t value)
{
	uint16_t abs_bit = bit_off + (uint16_t)data_off * 8;
	uint16_t byte_idx = abs_bit >> 3;
	uint8_t  bit_idx = abs_bit & 7;

	if (__builtin_expect(bit_idx == 0, 1)) {
		if (bit_size == 16) {
			if (byte_idx + 2 > buf_len) return;
			buf[byte_idx]     = (uint8_t)(value & 0xFF);
			buf[byte_idx + 1] = (uint8_t)((value >> 8) & 0xFF);
			return;
		}
		if (bit_size == 8) {
			if (byte_idx + 1 > buf_len) return;
			buf[byte_idx] = (uint8_t)(int8_t)value;
			return;
		}
	}

	uint32_t mask = ((1u << bit_size) - 1) << bit_idx;
	uint32_t val  = ((uint32_t)value & ((1u << bit_size) - 1)) << bit_idx;
	uint8_t bytes_needed = (bit_idx + bit_size + 7) >> 3;
	if (byte_idx + bytes_needed > buf_len) return;
	for (uint8_t b = 0; b < bytes_needed; b++) {
		uint8_t m = (mask >> (b * 8)) & 0xFF;
		uint8_t v = (val  >> (b * 8)) & 0xFF;
		buf[byte_idx + b] = (buf[byte_idx + b] & ~m) | v;
	}
}

// ── Endpoint/layout cache ─────────────────────────────────────────────────────
void usb_merge_cache_endpoints(const captured_descriptors_t *desc)
{
	cached_mouse_ep = 0;
	cached_kb_ep = 0;
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;
	cached_mouse_report_len = 0;
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		if (desc->ifaces[i].interrupt_in_ep == 0) continue;
		uint8_t ep = desc->ifaces[i].interrupt_in_ep & 0x0F;
		if (desc->ifaces[i].iface_protocol == 2 && !cached_mouse_ep) {
			cached_mouse_ep = ep;
			cached_mouse_maxpkt = desc->ifaces[i].interrupt_in_maxpkt;
			hid_layout_parse_mouse(&mouse_layout,
			                   desc->ifaces[i].hid_report_desc,
			                   desc->ifaces[i].hid_report_desc_len);
		} else if (desc->ifaces[i].iface_protocol == 1 && !cached_kb_ep) {
			cached_kb_ep = ep;
		}
	}
}

// ── Merge implementation ──────────────────────────────────────────────────────
__attribute__((cold, noinline))
static void usb_merge_report_slow(uint8_t *report, uint8_t len,
                                  uint8_t rid, uint8_t doff);
__attribute__((cold, noinline))
static void usb_merge_keyboard(uint8_t *report, uint8_t len);
__attribute__((cold, noinline))
static void usb_merge_phys_mouse(uint8_t *report, uint8_t len);
__attribute__((cold, noinline))
static void usb_merge_phys_keyboard(uint8_t *report, uint8_t len);

// Output cadence tracking.
//   last_merge_ms — when a real mouse report last rode through (injection rides
//                   those reports via the merge path).
//   last_synth_ms — last standalone synth frame (one-per-ms cap).
//   merged_this_cycle — set when a real report was merged this relay-loop
//                   iteration; consumed by usb_merge_send_pending() so the synth
//                   and merge paths never both emit in one frame.
static uint32_t last_merge_ms;
static uint32_t last_synth_ms;
static bool     merged_this_cycle;
#define SYNTH_SILENCE_MS 2   // mouse considered idle after this many ms of no report

/* Pull this frame's injected delta and run it through the humanization filter,
 * which owns conservation (sub-pixel residual and >127 cap-carry). */
static void usb_merge_take_injection(int16_t *out_dx, int16_t *out_dy)
{
	int16_t dx = inject.mouse_dx;
	int16_t dy = inject.mouse_dy;
	inject.mouse_dx = 0;
	inject.mouse_dy = 0;
	humanize_filter(&dx, &dy);
	*out_dx = dx;
	*out_dy = dy;
}

void usb_merge_report(uint8_t iface_protocol, uint8_t * restrict report, uint8_t len)
{
	if (iface_protocol == 2) {
		last_merge_ms = millis();
		if (__builtin_expect(cached_mouse_report_len == 0, 0))
			cached_mouse_report_len = len;

		// Emit physical-only telemetry (pre-merge, pre-mask) and suppress masked
		// physical inputs before any injection is merged in. Both off in the
		// common case, so the predicate is false and this is skipped.
		if (__builtin_expect((g_phys_mask || proto_phys_enabled()) &&
		                     mouse_layout.valid, 0))
			usb_merge_phys_mouse(report, len);

		if (mouse_layout.valid && inject.mouse_dirty && len > mouse_layout.data_off) {
			uint8_t doff = mouse_layout.data_off;
			uint8_t rid = doff ? report[0] : 0;

			// Highest byte index the fast path will touch; a short/malformed
			// report that does not cover it is routed to the bounds-checked
			// slow path rather than indexed out of bounds.
			uint16_t fp_need = doff;
			if (mouse_layout.x_byte + (mouse_layout.x_is16 ? 1u : 0u) > fp_need)
				fp_need = mouse_layout.x_byte + (mouse_layout.x_is16 ? 1u : 0u);
			if (mouse_layout.y_byte + (mouse_layout.y_is16 ? 1u : 0u) > fp_need)
				fp_need = mouse_layout.y_byte + (mouse_layout.y_is16 ? 1u : 0u);
			if (mouse_layout.w_byte != 0xFF &&
			    (uint16_t)(mouse_layout.w_byte + (mouse_layout.w_is16 ? 1u : 0u)) > fp_need)
				fp_need = mouse_layout.w_byte + (mouse_layout.w_is16 ? 1u : 0u);

			if (__builtin_expect(mouse_layout.fast_path && fp_need < len &&
			                     rid == mouse_layout.report_id, 1)) {
				report[doff] |= inject.mouse_buttons;
				proto_notify_buttons(report[doff]);

				int16_t inj_dx, inj_dy;
				usb_merge_take_injection(&inj_dx, &inj_dy);

				// Each axis adds humanized injection onto the mouse's own delta
				// and clamps to the report field as a hard safety bound. The
				// filter's per-frame cap (127) means the clamp rarely fires;
				// conservation is owned by the filter's owed accumulator.
				int32_t done_w = 0;
				int32_t done_dx = 0, done_dy = 0;
				if (mouse_layout.x_is16) {
					int32_t rx = (int16_t)(report[mouse_layout.x_byte] |
					             ((uint16_t)report[mouse_layout.x_byte + 1] << 8));
					int32_t mx = rx + inj_dx;
					if (mx >  mouse_layout.x_max) mx =  mouse_layout.x_max;
					if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
					report[mouse_layout.x_byte]     = (uint8_t)(mx & 0xFF);
					report[mouse_layout.x_byte + 1] = (uint8_t)(mx >> 8);
					done_dx = mx - rx;
				} else {
					int32_t rx = (int8_t)report[mouse_layout.x_byte];
					int32_t mx = rx + inj_dx;
					if (mx >  mouse_layout.x_max) mx =  mouse_layout.x_max;
					if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
					report[mouse_layout.x_byte] = (uint8_t)(int8_t)mx;
					done_dx = mx - rx;
				}

				if (mouse_layout.y_is16) {
					int32_t ry = (int16_t)(report[mouse_layout.y_byte] |
					             ((uint16_t)report[mouse_layout.y_byte + 1] << 8));
					int32_t my = ry + inj_dy;
					if (my >  mouse_layout.y_max) my =  mouse_layout.y_max;
					if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
					report[mouse_layout.y_byte]     = (uint8_t)(my & 0xFF);
					report[mouse_layout.y_byte + 1] = (uint8_t)(my >> 8);
					done_dy = my - ry;
				} else {
					int32_t ry = (int8_t)report[mouse_layout.y_byte];
					int32_t my = ry + inj_dy;
					if (my >  mouse_layout.y_max) my =  mouse_layout.y_max;
					if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
					report[mouse_layout.y_byte] = (uint8_t)(int8_t)my;
					done_dy = my - ry;
				}

				if (mouse_layout.w_byte != 0xFF && inject.mouse_wheel != 0) {
					if (mouse_layout.w_is16) {
						int32_t rw = (int16_t)(report[mouse_layout.w_byte] |
						             ((uint16_t)report[mouse_layout.w_byte + 1] << 8));
						int32_t want = rw + inject.mouse_wheel;
						int32_t mw = want;
						if (mw >  mouse_layout.w_max) mw =  mouse_layout.w_max;
						if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
						report[mouse_layout.w_byte]     = (uint8_t)(mw & 0xFF);
						report[mouse_layout.w_byte + 1] = (uint8_t)(mw >> 8);
						inject.mouse_wheel = (int8_t)(want - mw);
						done_w = mw - rw;
					} else {
						int32_t rw = (int8_t)report[mouse_layout.w_byte];
						int32_t want = rw + inject.mouse_wheel;
						int32_t mw = want;
						if (mw >  mouse_layout.w_max) mw =  mouse_layout.w_max;
						if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
						report[mouse_layout.w_byte] = (uint8_t)(int8_t)mw;
						inject.mouse_wheel = (int8_t)(want - mw);
						done_w = mw - rw;
					}
				}

				// For a wheel on a separate report ID (no field here) the scroll
				// is flushed later by the standalone wheel report, so report the
				// full pending value now to preserve its telemetry cadence.
				int8_t w_tlm = (mouse_layout.w_byte != 0xFF)
				             ? (int8_t)done_w : inject.mouse_wheel;
				proto_notify_axes((int16_t)done_dx, (int16_t)done_dy, w_tlm);
				// Return any field-clamped injected portion so the filter
				// redelivers it next frame; real-mouse motion keeps priority and
				// nothing injected is dropped.
				humanize_return((int16_t)(inj_dx - done_dx),
				                (int16_t)(inj_dy - done_dy));
				inject.mouse_dirty = (inject.mouse_buttons != 0 ||
				                      inject.mouse_wheel != 0 ||
				                      humanize_pending());
			} else {
				usb_merge_report_slow(report, len, rid, doff);
			}
		}
		// A real mouse report rode through this cycle (whether or not injection
		// was applied). The synth path checks this so it never also emits in the
		// same frame. Set unconditionally for protocol==2.
		merged_this_cycle = true;
	} else if (iface_protocol == 1) {
		// Keyboard telemetry and masking run even with nothing injected (the user
		// may be typing on a masked key). Gated so an idle keyboard with no
		// monitoring/mask pays only this branch test.
		if (__builtin_expect((proto_phys_enabled() || act_phys_kb_mask_active()) &&
		                     len >= 8, 0))
			usb_merge_phys_keyboard(report, len);
		if (__builtin_expect(inject.kb_dirty, 0)) {
			usb_merge_keyboard(report, len);
			merged_this_cycle = true;
		}
	}
}

__attribute__((cold, noinline))
static void usb_merge_report_slow(uint8_t *report, uint8_t len,
                                  uint8_t rid, uint8_t doff)
{
	// Pull humanized injection once for this frame. Only the axes whose report ID
	// arrived are applied; if X and Y live on different report IDs the caller
	// re-enters with the other ID and the filter emits again.
	int16_t inj_dx, inj_dy;
	usb_merge_take_injection(&inj_dx, &inj_dy);
	int32_t done_dx = 0, done_dy = 0, done_w = 0;

	if (rid == mouse_layout.report_id) {
		report[doff] |= inject.mouse_buttons;
		proto_notify_buttons(report[doff]);

		int32_t rx = hid_layout_read_field(report, len, mouse_layout.x_bit,
		                               mouse_layout.x_size, doff);
		int32_t mx = rx + inj_dx;
		if (mx > mouse_layout.x_max) mx = mouse_layout.x_max;
		if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
		write_report_field(report, len, mouse_layout.x_bit,
		                   mouse_layout.x_size, doff, mx);
		done_dx = mx - rx;

		if (rid == mouse_layout.y_report_id) {
			int32_t ry = hid_layout_read_field(report, len, mouse_layout.y_bit,
			                               mouse_layout.y_size, doff);
			int32_t my = ry + inj_dy;
			if (my > mouse_layout.y_max) my = mouse_layout.y_max;
			if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
			write_report_field(report, len, mouse_layout.y_bit,
			                   mouse_layout.y_size, doff, my);
			done_dy = my - ry;
		}
	}

	if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0 &&
	    rid == mouse_layout.wheel_report_id) {
		int32_t rw = hid_layout_read_field(report, len, mouse_layout.wheel_bit,
		                               mouse_layout.wheel_size, doff);
		int32_t ww = rw + inject.mouse_wheel;
		int32_t mw = ww;
		if (mw > mouse_layout.w_max) mw = mouse_layout.w_max;
		if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
		write_report_field(report, len, mouse_layout.wheel_bit,
		                   mouse_layout.wheel_size, doff, mw);
		inject.mouse_wheel = (int8_t)(ww - mw);
		done_w = mw - rw;
	}

	proto_notify_axes((int16_t)done_dx, (int16_t)done_dy, (int8_t)done_w);
	// Return any injected motion not applied this frame (field-clamped, or on a
	// split X/Y report-ID layout an axis whose report ID didn't arrive). The
	// filter redelivers it; nothing is dropped.
	humanize_return((int16_t)(inj_dx - done_dx), (int16_t)(inj_dy - done_dy));
	inject.mouse_dirty = (inject.mouse_buttons != 0 ||
	                      inject.mouse_wheel != 0 ||
	                      humanize_pending());
}

__attribute__((cold, noinline))
static void usb_merge_keyboard(uint8_t *report, uint8_t len)
{
	if (len < 8) return;
	report[0] |= inject.kb_modifier;
	for (int i = 0; i < 6; i++) {
		if (inject.kb_keys[i] == 0) continue;
		bool found = false;
		for (int j = 2; j < 8; j++) {
			if (report[j] == inject.kb_keys[i]) {
				found = true;
				break;
			}
		}
		if (!found) {
			for (int j = 2; j < 8; j++) {
				if (report[j] == 0) {
					report[j] = inject.kb_keys[i];
					break;
				}
			}
		}
	}
	proto_notify_keys(&report[2]);
}

// Runs before injection is merged. Reads the physical report fields, pushes them
// as TLM_PHYS_* telemetry (pre-mask), then zeroes any masked physical
// contribution so it never reaches the downstream PC. Injected input is applied
// afterward by the normal merge, so an injected click on a masked button still
// passes. Only the fields whose report ID matches this report are touched.
__attribute__((cold, noinline))
static void usb_merge_phys_mouse(uint8_t *report, uint8_t len)
{
	uint8_t doff = mouse_layout.data_off;
	if (len <= doff) return;          // too short to hold the report-ID/button byte
	uint8_t rid  = doff ? report[0] : 0;

	bool xy_here    = (rid == mouse_layout.report_id);
	bool wheel_here = (mouse_layout.wheel_bit != 0xFFFF &&
	                   rid == mouse_layout.wheel_report_id);

	uint8_t phys_btn = xy_here ? report[doff] : 0;
	int32_t phys_x = 0, phys_y = 0, phys_w = 0;
	if (xy_here) {
		phys_x = hid_layout_read_field(report, len, mouse_layout.x_bit,
		                           mouse_layout.x_size, doff);
		phys_y = hid_layout_read_field(report, len, mouse_layout.y_bit,
		                           mouse_layout.y_size, doff);
	}
	if (wheel_here)
		phys_w = hid_layout_read_field(report, len, mouse_layout.wheel_bit,
		                           mouse_layout.wheel_size, doff);

	// Telemetry: the physical input, before masking.
	if (proto_phys_enabled()) {
		if (xy_here)
			proto_notify_phys_buttons(phys_btn);
		if (xy_here || wheel_here)
			proto_notify_phys_axes((int16_t)phys_x, (int16_t)phys_y,
			                       (int8_t)phys_w);
	}

	// Masking: zero the masked physical contributions in the outgoing report.
	if (g_phys_mask) {
		if (xy_here) {
			uint8_t bmask = (uint8_t)(g_phys_mask & 0x1F); // ml,mr,mm,ms1,ms2
			if (bmask) report[doff] &= (uint8_t)~bmask;
			if (g_phys_mask & (1u << PHYS_MASK_MX))
				write_report_field(report, len, mouse_layout.x_bit,
				                   mouse_layout.x_size, doff, 0);
			if (g_phys_mask & (1u << PHYS_MASK_MY))
				write_report_field(report, len, mouse_layout.y_bit,
				                   mouse_layout.y_size, doff, 0);
		}
		if (wheel_here && (g_phys_mask & (1u << PHYS_MASK_WHEEL)))
			write_report_field(report, len, mouse_layout.wheel_bit,
			                   mouse_layout.wheel_size, doff, 0);
	}
}

// Standard 8-byte boot report (modifier, reserved, 6 keycodes). Pushes the
// physical modifier+keys as TLM_PHYS_KB (pre-mask), then removes any masked
// keycodes from the outgoing report. Modifiers are not individually maskable in
// the KMBox API, so only keycodes are filtered.
__attribute__((cold, noinline))
static void usb_merge_phys_keyboard(uint8_t *report, uint8_t len)
{
	(void)len;  // caller guarantees len >= 8
	if (proto_phys_enabled())
		proto_notify_phys_keys(report[0], &report[2]);

	if (act_phys_kb_mask_active()) {
		for (int j = 2; j < 8; j++) {
			if (report[j] && act_phys_key_masked(report[j]))
				report[j] = 0;
		}
	}
}

// ── Injection accumulation ────────────────────────────────────────────────────
// The V3F command core encodes injection as ICC records; usb_merge_drain_icc
// decodes them into these accumulators.
static void merge_apply_mouse(int16_t dx, int16_t dy, uint8_t buttons,
                              int8_t wheel)
{
	inject.mouse_buttons = buttons;
	inject.mouse_wheel += wheel;
	inject.mouse_dx += dx;
	inject.mouse_dy += dy;
	inject.mouse_dirty = true;
}

static void merge_apply_keyboard(uint8_t modifier, const uint8_t keys[6])
{
	inject.kb_modifier = modifier;
	memcpy(inject.kb_keys, keys, 6);
	inject.kb_dirty = true;
}

static void merge_schedule_click_release(uint8_t button_mask, uint32_t delay_ms)
{
	inject.click_release_mask = button_mask;
	inject.click_release_at = millis() + delay_ms;
}

static void merge_schedule_kb_release(uint8_t key, uint32_t delay_ms)
{
	uint32_t at = millis() + delay_ms;
	for (int i = 0; i < inject.kb_release_count; i++) {
		if (inject.kb_releases[i].key == key) {
			inject.kb_releases[i].at = at;   // replace existing entry for this key
			return;
		}
	}
	if (inject.kb_release_count < MAX_KB_RELEASES) {
		inject.kb_releases[inject.kb_release_count].key = key;
		inject.kb_releases[inject.kb_release_count].at = at;
		inject.kb_release_count++;
	}
}

// Run the scheduled click/key-release bookkeeping. Called once per drain.
static void merge_run_releases(void)
{
	if (__builtin_expect(inject.click_release_at != 0, 0) &&
	    millis() >= inject.click_release_at) {
		inject.mouse_buttons &= ~inject.click_release_mask;
		inject.mouse_dirty = true;
		inject.click_release_mask = 0;
		inject.click_release_at = 0;
	}

	if (__builtin_expect(inject.kb_release_count, 0)) {
		uint32_t now = millis();
		for (int r = 0; r < inject.kb_release_count; ) {
			if (now >= inject.kb_releases[r].at) {
				uint8_t key = inject.kb_releases[r].key;
				for (int i = 0; i < 6; i++) {
					if (inject.kb_keys[i] == key) {
						inject.kb_keys[i] = 0;
						break;
					}
				}
				inject.kb_dirty = true;
				inject.kb_releases[r] = inject.kb_releases[--inject.kb_release_count];
			} else {
				r++;
			}
		}
	}
}

void usb_merge_apply_record(uint8_t tag, const uint8_t *b)
{
	switch (tag) {
	case ICC_TAG_INJECT_MOUSE: {
		// b0..1=dx LE int16, b2..3=dy LE int16, b4=buttons, b5=wheel int8.
		int16_t dx = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
		int16_t dy = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
		merge_apply_mouse(dx, dy, b[4], (int8_t)b[5]);
		break;
	}
	case ICC_TAG_INJECT_KEYBOARD:
		merge_apply_keyboard(b[0], &b[1]);          // b0=modifier, b1..6=keys[6]
		break;
	case ICC_TAG_CLICK_RELEASE: {
		uint32_t delay_ms = (uint32_t)b[1] | ((uint32_t)b[2] << 8) |
		                    ((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 24);
		merge_schedule_click_release(b[0], delay_ms);
		break;
	}
	case ICC_TAG_KB_RELEASE: {
		uint32_t delay_ms = (uint32_t)b[1] | ((uint32_t)b[2] << 8) |
		                    ((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 24);
		merge_schedule_kb_release(b[0], delay_ms);
		break;
	}
	case ICC_TAG_SET_HUMAN_LEVEL:
		humanize_set_level(b[0]);
		break;
	case ICC_TAG_PHYS_MASK:
		// b0=kind (0=mouse,1=key,2=unmask_all), b1=code, b2=enable.
		switch (b[0]) {
		case 0: act_phys_mask_mouse(b[1], b[2] != 0); break;
		case 1: act_phys_mask_key(b[1], b[2] != 0); break;
		case 2: act_phys_unmask_all(); break;
		default: break;
		}
		break;
	case ICC_TAG_SET_BAUD:
		// Baud is owned by the command transport; nothing for the merge to do.
		break;
	default:
		break;
	}
}

void usb_merge_drain_icc(void)
{
	// Reset the per-cycle merge flag at the start of each relay-loop iteration.
	// This is called first each iteration; the EP loop then sets the flag via
	// usb_merge_report(), and usb_merge_send_pending() consumes it afterward.
	merged_this_cycle = false;

	icc_record_t r;
	while (icc_recv_from_v3f(&r)) {
		if (r.tag == ICC_TAG_DEV_TEMP) {
			extern volatile int8_t g_tb_dev_temp_c;  // two_board.c
			g_tb_dev_temp_c = (int8_t)r.b[0];
			continue;
		}
		usb_merge_apply_record(r.tag, r.b);
	}

	// Mailbox drained. Re-arm the IPC doorbell interrupt, which the ISR disables
	// on each fire to stay storm-proof under AutoEN (see IPC_CH0_Handler).
	// Idempotent, safe to call every pass.
	icc_ipc_rearm_v5f();

	merge_run_releases();
}

// ── Standalone synth-injection path ──────────────────────────────────────────
// When V3F injects motion while the physical mouse is silent, the merge path
// (which rides real device reports) emits nothing until the next real report.
// usb_merge_send_pending() synthesizes a standalone mouse report in that case,
// capped to one-per-ms, and flushes an unconsumed wheel on a separate report ID.
// Call once per relay-loop iteration after the per-EP merge/send loop.
//
// merged_this_cycle gate: if a real mouse report was merged this cycle, the
// synth path returns early (after the separate-report-id wheel flush) so the two
// paths never both emit in one frame, which would flood/overwrite at the 1 kHz
// endpoint. The SYNTH_SILENCE_MS test is the time-based guard for the case where
// merged_this_cycle is false but a real report arrived very recently.
//
// cached_mouse_ep is the device-side EP the synth report is sent on, cached by
// usb_merge_cache_endpoints() from the captured descriptors.

__attribute__((cold, noinline))
static void usb_merge_send_wheel_report(void);
__attribute__((cold, noinline))
static void usb_merge_send_keyboard_report(void);

void usb_merge_send_pending(void)
{
	// Flush unconsumed wheel on a separate report ID even when merged.
	if (__builtin_expect(merged_this_cycle && inject.mouse_wheel != 0 &&
	    cached_mouse_ep && mouse_layout.valid &&
	    mouse_layout.wheel_bit != 0xFFFF &&
	    mouse_layout.wheel_report_id != mouse_layout.report_id, 0)) {
		usb_merge_send_wheel_report();
	}

	if (merged_this_cycle) return;
	// Only synthesize a standalone mouse report when the physical mouse has gone
	// silent; otherwise injection rides the next real report via the merge path.
	// Capped to one synth per ms.
	uint32_t ms = millis();
	bool mouse_silent = (uint32_t)(ms - last_merge_ms) >= SYNTH_SILENCE_MS;
	if (inject.mouse_dirty && mouse_silent && ms != last_synth_ms &&
	    cached_mouse_ep && mouse_layout.valid) {
		last_synth_ms = ms;
		uint8_t synth[64];
		memset(synth, 0, sizeof(synth));
		uint8_t doff = mouse_layout.data_off;
		if (doff) synth[0] = mouse_layout.report_id;
		synth[doff] = inject.mouse_buttons;
		int16_t inj_dx, inj_dy;
		usb_merge_take_injection(&inj_dx, &inj_dy);
		int32_t dx = inj_dx;
		int32_t dy = inj_dy;
		if (dx > mouse_layout.x_max) dx = mouse_layout.x_max;
		if (dx < -mouse_layout.x_max) dx = -mouse_layout.x_max;
		if (dy > mouse_layout.y_max) dy = mouse_layout.y_max;
		if (dy < -mouse_layout.y_max) dy = -mouse_layout.y_max;

		write_report_field(synth, sizeof(synth), mouse_layout.x_bit,
		                   mouse_layout.x_size, doff, dx);
		write_report_field(synth, sizeof(synth), mouse_layout.y_bit,
		                   mouse_layout.y_size, doff, dy);

		if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0 &&
		    mouse_layout.wheel_report_id == mouse_layout.report_id) {
			int32_t w = inject.mouse_wheel;
			if (w > mouse_layout.w_max) w = mouse_layout.w_max;
			if (w < -mouse_layout.w_max) w = -mouse_layout.w_max;
			write_report_field(synth, sizeof(synth), mouse_layout.wheel_bit,
			                   mouse_layout.wheel_size, doff, w);
		}
		uint8_t rlen = cached_mouse_report_len;
		if (rlen == 0) rlen = (cached_mouse_maxpkt < 16) ? (uint8_t)cached_mouse_maxpkt : 16;
		if (rlen > sizeof(synth)) rlen = sizeof(synth);
		usb_device_send_report(cached_mouse_ep, synth, rlen);
		inject.mouse_wheel = 0;
		inject.mouse_dirty = (inject.mouse_buttons != 0 ||
		                      humanize_pending());
	}
	if (__builtin_expect(inject.kb_dirty && cached_kb_ep, 0)) {
		usb_merge_send_keyboard_report();
	}
}

__attribute__((cold, noinline))
static void usb_merge_send_wheel_report(void)
{
	uint8_t synth[64];
	memset(synth, 0, sizeof(synth));
	uint8_t doff = mouse_layout.data_off;
	if (doff) synth[0] = mouse_layout.wheel_report_id;
	int32_t w = inject.mouse_wheel;
	if (w > mouse_layout.w_max) w = mouse_layout.w_max;
	if (w < -mouse_layout.w_max) w = -mouse_layout.w_max;
	write_report_field(synth, sizeof(synth), mouse_layout.wheel_bit,
	                   mouse_layout.wheel_size, doff, w);
	uint8_t rlen = cached_mouse_report_len;
	if (rlen == 0) rlen = (cached_mouse_maxpkt < 16) ? (uint8_t)cached_mouse_maxpkt : 16;
	if (rlen > sizeof(synth)) rlen = sizeof(synth);
	usb_device_send_report(cached_mouse_ep, synth, rlen);
	inject.mouse_wheel = 0;
	inject.mouse_dirty = (inject.mouse_buttons != 0);
}

__attribute__((cold, noinline))
static void usb_merge_send_keyboard_report(void)
{
	uint8_t synth[8];
	synth[0] = inject.kb_modifier;
	synth[1] = 0;
	memcpy(&synth[2], inject.kb_keys, 6);
	usb_device_send_report(cached_kb_ep, synth, 8);
	static const uint8_t zeros[6] = {0};
	inject.kb_dirty = (inject.kb_modifier != 0 ||
	                   memcmp(inject.kb_keys, zeros, 6) != 0);
}
