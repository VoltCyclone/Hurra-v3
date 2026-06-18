// src/usb_merge.c — HID-report merge module (V5F side of the MITM).
//
// Ported faithfully from Hurra-v2 src/kmbox.c's merge half. The v2 file mixed
// an i.MX-RT LPUART/DMA transport with a hardware-agnostic, HID-report-
// descriptor-AWARE merge. Only the merge half lives here; the transport half
// (UART link to the host PC + the protocol parser) moved to the V3F command
// core (src/uart.c + src/hurra.c/ferrum.c + src/kmbox_cmd.c).
//
// What changed versus v2:
//   * Renamed the public entry points:
//       kmbox_cache_endpoints -> usb_merge_cache_endpoints
//       kmbox_merge_report    -> usb_merge_report
//   * The injection SOURCE switched from direct kmbox_inject_* calls (same
//     core) to an ICC drain (usb_merge_drain_icc). The bodies of v2's
//     kmbox_inject_mouse/keyboard/schedule_click_release/schedule_kb_release
//     now run inside usb_merge_drain_icc, fed by icc_recv_from_v3f records
//     instead of direct calls. They write the SAME `inject` accumulators.
//   * proto_* physical-telemetry calls are stubbed to no-ops here, with
//     proto_phys_enabled() constant-false (Phase-5: the telemetry's wire
//     destination — the host UART — lives on V3F, not V5F). The compiler folds
//     the constant-false predicate and eliminates the phys-capture branch, so
//     the merge code is byte-for-byte the v2 logic. Forwarding phys-telemetry
//     to V3F over the ICC (TELEM records) is a Phase-7 enhancement.
//
// Everything else (parse_mouse_layout, read/write_report_field, the fast/slow
// merge paths, keyboard merge, physical masking, per-axis 16-bit handling,
// clamping, and wheel carry) is a verbatim port — it is the anti-cheat-correct
// path and must not be simplified.
//
// Task 7.1 additions:
//   * Standalone synth-injection path restored (usb_merge_send_pending +
//     usb_merge_send_wheel_report + usb_merge_send_keyboard_report), ported
//     from v2 kmbox_send_pending. Emits injected motion when the physical mouse
//     is silent, capped one-per-ms, gated by merged_this_cycle so the merge and
//     synth paths never both emit in one frame. cached_mouse_ep (the device-side
//     EP to send on) is the same value as main_v5f's ep_map[].dev_ep_num — both
//     come from (interrupt_in_ep & 0x0F) of the mouse iface — so the existing
//     usb_merge_cache_endpoints() already provides it; no setter is needed.
//   * merged_this_cycle flag restored with v2's exact semantics: cleared at the
//     top of usb_merge_drain_icc() (the first per-loop call, = v2 kmbox_poll_fast),
//     set in usb_merge_report() for every protocol==2 report (+ keyboard when
//     kb_dirty), consumed in usb_merge_send_pending() after the EP loop.
//
//   * humanize split (Gap 3): humanize_filter() stays on V5F, called PER EMITTED
//     FRAME inside usb_merge_take_injection() (both the merge and synth paths).
//     It is intentionally NOT moved to V3F's kmbox_cmd_inject_mouse: the per-
//     frame delta is consumed here, so the filter must run per emitted frame,
//     not per command. Moving it to V3F would filter per-command and change the
//     jitter/sub-pixel-carry behavior. (humanize_set_level still arrives over
//     the ICC SET_HUMAN_LEVEL tag.) This supersedes the plan's "move to V3F".

#include "usb_merge.h"
#include "humanize.h"
#include "actions.h"      // g_phys_mask + act_phys_* query helpers + PHYS_MASK_*
#include "icc.h"
#include "usb_device.h"   // usb_device_send_report (standalone synth path)
#include <string.h>
#include <stdbool.h>

extern uint32_t millis(void);

// ── Phase-5 proto_* physical-telemetry stubs (option a) ──────────────────────
// On V5F there is no protocol parser and no host UART; physical-input telemetry
// has nowhere to go. Stub the proto_* notify hooks to no-ops and force
// proto_phys_enabled() to false so the (g_phys_mask || proto_phys_enabled())
// capture branches compile out. The MASK half (g_phys_mask) still works — it is
// driven from actions.c state. Phase-7: forward phys telemetry to V3F via ICC.
static inline bool proto_phys_enabled(void) { return false; }
static inline void proto_notify_buttons(uint8_t b) { (void)b; }
static inline void proto_notify_axes(int16_t dx, int16_t dy, int8_t w) { (void)dx; (void)dy; (void)w; }
static inline void proto_notify_keys(const uint8_t *keys) { (void)keys; }
static inline void proto_notify_phys_buttons(uint8_t b) { (void)b; }
static inline void proto_notify_phys_axes(int16_t dx, int16_t dy, int8_t w) { (void)dx; (void)dy; (void)w; }
static inline void proto_notify_phys_keys(uint8_t mod, const uint8_t keys[6]) { (void)mod; (void)keys; }

// ── Injection accumulators (verbatim from v2 kmbox.c lines ~112-129) ─────────
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

// ── Cached endpoints + parsed mouse layout (verbatim from v2 ~149-175) ───────
static uint8_t  cached_mouse_ep;
static uint16_t cached_mouse_maxpkt;
static uint8_t  cached_kb_ep;
static struct {
	uint16_t x_bit;
	uint16_t y_bit;
	uint16_t wheel_bit;     // 0xFFFF = none
	uint8_t  x_size;
	uint8_t  y_size;
	uint8_t  wheel_size;
	uint8_t  report_id;
	uint8_t  y_report_id;
	uint8_t  wheel_report_id;
	uint8_t  data_off;
	bool     valid;
	int16_t  x_max;
	int16_t  y_max;
	int16_t  w_max;
	bool     fast_path;
	uint8_t  x_byte;
	uint8_t  y_byte;
	uint8_t  w_byte;        // 0xFF = none
	bool     x_is16;
	bool     y_is16;
	bool     w_is16;
} mouse_layout;
static uint8_t cached_mouse_report_len; // actual report length from first real report

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

// ── HID report-descriptor parser (verbatim from v2 kmbox.c ~392) ─────────────
static void parse_mouse_layout(const uint8_t *rd, uint16_t rdlen)
{
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;

	uint16_t usage_page = 0;
	uint8_t  usages[16];
	uint8_t  num_usages = 0;
	uint16_t usage_min = 0, usage_max = 0;
	uint8_t  report_size = 0;
	uint8_t  report_count = 0;
	uint8_t  current_rid = 0;
	uint16_t bit_pos = 0;

	uint16_t i = 0;
	while (i < rdlen) {
		uint8_t b = rd[i];
		if (b == 0xFE) { // long item — skip
			if (i + 2 < rdlen) i += 3 + rd[i + 1];
			else break;
			continue;
		}

		uint8_t sz = b & 0x03;
		if (sz == 3) sz = 4;
		if (i + 1 + sz > rdlen) break;

		// Read unsigned data
		uint32_t val = 0;
		if (sz >= 1) val = rd[i + 1];
		if (sz >= 2) val |= (uint32_t)rd[i + 2] << 8;
		if (sz >= 4) val |= (uint32_t)rd[i + 3] << 16 | (uint32_t)rd[i + 4] << 24;

		switch (b & 0xFC) {
		case 0x04: usage_page = (uint16_t)val; break;   // Usage Page
		case 0x74: report_size = (uint8_t)val; break;    // Report Size
		case 0x94: report_count = (uint8_t)val; break;   // Report Count
		case 0x84:                                        // Report ID
			current_rid = (uint8_t)val;
			bit_pos = 0;
			break;

		case 0x08: // Usage
			if (num_usages < 16) usages[num_usages++] = (uint8_t)val;
			break;
		case 0x18: usage_min = (uint16_t)val; break;     // Usage Minimum
		case 0x28: usage_max = (uint16_t)val; break;     // Usage Maximum

		case 0x80: { // Input
			if (num_usages == 0 && usage_max >= usage_min) {
				for (uint16_t u = usage_min; u <= usage_max && num_usages < 16; u++)
					usages[num_usages++] = (uint8_t)u;
			}

			for (uint8_t f = 0; f < report_count; f++) {
				uint8_t u = (f < num_usages) ? usages[f] :
				            (num_usages > 0 ? usages[num_usages - 1] : 0);

				if (usage_page == 0x01) { // Generic Desktop
					if (u == 0x30) { // X
						mouse_layout.x_bit = bit_pos;
						mouse_layout.x_size = report_size;
						mouse_layout.report_id = current_rid;
					} else if (u == 0x31) { // Y
						mouse_layout.y_bit = bit_pos;
						mouse_layout.y_size = report_size;
						mouse_layout.y_report_id = current_rid;
					} else if (u == 0x38) { // Wheel
						mouse_layout.wheel_bit = bit_pos;
						mouse_layout.wheel_size = report_size;
						mouse_layout.wheel_report_id = current_rid;
					}
				}
				bit_pos += report_size;
			}
			// Clear local state after Main item
			num_usages = 0;
			usage_min = 0;
			usage_max = 0;
			break;
		}
		case 0xA0: // Collection
			num_usages = 0;
			usage_min = 0;
			usage_max = 0;
			break;
		case 0xC0: // End Collection
			num_usages = 0;
			break;
		}

		i += 1 + sz;
	}

	mouse_layout.data_off = mouse_layout.report_id ? 1 : 0;
	mouse_layout.valid = (mouse_layout.x_size > 0 && mouse_layout.y_size > 0);
	mouse_layout.x_max = mouse_layout.x_size > 0 ? (int16_t)((1 << (mouse_layout.x_size - 1)) - 1) : 0;
	mouse_layout.y_max = mouse_layout.y_size > 0 ? (int16_t)((1 << (mouse_layout.y_size - 1)) - 1) : 0;
	mouse_layout.w_max = mouse_layout.wheel_size > 0 ? (int16_t)((1 << (mouse_layout.wheel_size - 1)) - 1) : 0;

	mouse_layout.fast_path = false;
	mouse_layout.w_byte = 0xFF;

	if (mouse_layout.valid &&
	    (mouse_layout.x_bit & 7) == 0 &&
	    (mouse_layout.y_bit & 7) == 0 &&
	    (mouse_layout.x_size == 8 || mouse_layout.x_size == 16) &&
	    (mouse_layout.y_size == 8 || mouse_layout.y_size == 16) &&
	    mouse_layout.report_id == mouse_layout.y_report_id) {

		mouse_layout.x_byte = (uint8_t)(mouse_layout.x_bit / 8) + mouse_layout.data_off;
		mouse_layout.y_byte = (uint8_t)(mouse_layout.y_bit / 8) + mouse_layout.data_off;
		mouse_layout.x_is16 = (mouse_layout.x_size == 16);
		mouse_layout.y_is16 = (mouse_layout.y_size == 16);

		if (mouse_layout.wheel_bit != 0xFFFF &&
		    (mouse_layout.wheel_bit & 7) == 0 &&
		    (mouse_layout.wheel_size == 8 || mouse_layout.wheel_size == 16) &&
		    mouse_layout.wheel_report_id == mouse_layout.report_id) {
			mouse_layout.w_byte = (uint8_t)(mouse_layout.wheel_bit / 8) + mouse_layout.data_off;
			mouse_layout.w_is16 = (mouse_layout.wheel_size == 16);
		}
		mouse_layout.fast_path = true;
	}
}

// ── Bit-field helpers (verbatim from v2 kmbox.c ~518 / ~548) ──────────────────
static int32_t read_report_field(const uint8_t *buf, uint8_t buf_len,
                                 uint16_t bit_off,
                                 uint8_t bit_size, uint8_t data_off)
{
	uint16_t abs_bit = bit_off + (uint16_t)data_off * 8;
	uint16_t byte_idx = abs_bit >> 3;
	uint8_t  bit_idx = abs_bit & 7;

	if (__builtin_expect(bit_idx == 0, 1)) {
		if (bit_size == 16) {
			if (byte_idx + 2 > buf_len) return 0;
			return (int16_t)(buf[byte_idx] | ((uint16_t)buf[byte_idx + 1] << 8));
		}
		if (bit_size == 8) {
			if (byte_idx + 1 > buf_len) return 0;
			return (int8_t)buf[byte_idx];
		}
	}

	uint32_t raw = 0;
	uint8_t bytes_needed = (bit_idx + bit_size + 7) >> 3;
	if (byte_idx + bytes_needed > buf_len) return 0;
	for (uint8_t b = 0; b < bytes_needed; b++)
		raw |= (uint32_t)buf[byte_idx + b] << (b * 8);
	raw = (raw >> bit_idx) & ((1u << bit_size) - 1);
	if (raw & (1u << (bit_size - 1)))
		raw |= ~((1u << bit_size) - 1); // sign extend
	return (int32_t)raw;
}

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

// ── Endpoint/layout cache (ported from v2 kmbox_cache_endpoints ~580) ────────
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
			parse_mouse_layout(desc->ifaces[i].hid_report_desc,
			                   desc->ifaces[i].hid_report_desc_len);
		} else if (desc->ifaces[i].iface_protocol == 1 && !cached_kb_ep) {
			cached_kb_ep = ep;
		}
	}
}

// ── Merge implementation (verbatim from v2 ~750-1069) ────────────────────────
__attribute__((cold, noinline))
static void usb_merge_report_slow(uint8_t *report, uint8_t len,
                                  uint8_t rid, uint8_t doff);
__attribute__((cold, noinline))
static void usb_merge_keyboard(uint8_t *report, uint8_t len);
__attribute__((cold, noinline))
static void usb_merge_phys_mouse(uint8_t *report, uint8_t len);
__attribute__((cold, noinline))
static void usb_merge_phys_keyboard(uint8_t *report, uint8_t len);

// Output cadence tracking (ported verbatim from v2 kmbox.c ~760-767).
//   last_merge_ms — when a real mouse report last rode through (injection rides
//                   those reports via the merge path).
//   last_synth_ms — last standalone synth frame (one-per-ms cap).
//   merged_this_cycle — set when a real report was merged this relay-loop
//                   iteration; consumed by usb_merge_send_pending() so the synth
//                   path and the merge path never both emit in one frame.
static uint32_t last_merge_ms;
static uint32_t last_synth_ms;
static bool     merged_this_cycle;
#define SYNTH_SILENCE_MS 2   // mouse considered idle after this many ms of no report

/* Pull this frame's injected delta from the pending accumulators and run it
 * through the humanization filter. The filter delivers in-frame and owns
 * conservation (sub-pixel residual + >127 cap-carry), so we just consume. */
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
		last_merge_ms = millis();   // a real mouse report is riding through now
		if (__builtin_expect(cached_mouse_report_len == 0, 0))
			cached_mouse_report_len = len;

		// Feature B/C: emit physical-only telemetry (pre-merge, pre-mask) and
		// suppress masked physical inputs — before any injection is merged in.
		// Both off (the common case) → the predicate is false and we skip it.
		if (__builtin_expect((g_phys_mask || proto_phys_enabled()) &&
		                     mouse_layout.valid, 0))
			usb_merge_phys_mouse(report, len);

		if (mouse_layout.valid && inject.mouse_dirty) {
			uint8_t doff = mouse_layout.data_off;
			uint8_t rid = doff ? report[0] : 0;

			if (__builtin_expect(mouse_layout.fast_path && rid == mouse_layout.report_id, 1)) {
				report[doff] |= inject.mouse_buttons;
				proto_notify_buttons(report[doff]);

				int16_t inj_dx, inj_dy;
				usb_merge_take_injection(&inj_dx, &inj_dy);

				// Each axis adds humanized injection onto the mouse's own delta
				// and clamps to the report field as a hard safety bound only.
				// The filter's per-frame cap (127) means the field clamp rarely
				// fires; conservation is now owned by humanize_filter's internal
				// owed accumulator.
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
				// If the field clamp rejected part of the injected delta (e.g.
				// 8-bit field while the real mouse is also moving), return the
				// unfit injected portion so the filter redelivers it next frame.
				// Real-mouse motion keeps priority; nothing injected is dropped.
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
		// same frame — see usb_merge_send_pending(). Set unconditionally for
		// protocol==2, mirroring v2 kmbox_merge_report (line ~895).
		merged_this_cycle = true;
	} else if (iface_protocol == 1) {
		// Feature B/C (keyboard): telemetry + masking run even with nothing
		// injected (the user may be typing on a masked key). Gated so an idle
		// keyboard with no monitoring/mask pays only this branch test.
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
	// Pull humanized injection once for this frame; conservation is owned by
	// the filter's internal owed accumulator.  Only the axes whose report ID
	// actually arrived are applied — if X and Y live on different report IDs
	// the caller re-enters with the other ID and the filter will emit again.
	int16_t inj_dx, inj_dy;
	usb_merge_take_injection(&inj_dx, &inj_dy);
	int32_t done_dx = 0, done_dy = 0, done_w = 0;

	if (rid == mouse_layout.report_id) {
		report[doff] |= inject.mouse_buttons;
		proto_notify_buttons(report[doff]);

		int32_t rx = read_report_field(report, len, mouse_layout.x_bit,
		                               mouse_layout.x_size, doff);
		int32_t mx = rx + inj_dx;
		if (mx > mouse_layout.x_max) mx = mouse_layout.x_max;
		if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
		write_report_field(report, len, mouse_layout.x_bit,
		                   mouse_layout.x_size, doff, mx);
		done_dx = mx - rx;

		if (rid == mouse_layout.y_report_id) {
			int32_t ry = read_report_field(report, len, mouse_layout.y_bit,
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
		int32_t rw = read_report_field(report, len, mouse_layout.wheel_bit,
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
	// Return any injected motion not applied this frame — either field-clamped,
	// or (on split X/Y report-ID layouts) belonging to an axis whose report ID
	// didn't arrive this call. The filter redelivers it; nothing is dropped.
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

// Feature B/C (mouse): runs before injection is merged. Reads the physical
// report fields, pushes them as TLM_PHYS_* telemetry (the user's TRUE input,
// observed before masking), then zeroes any masked physical contribution so it
// never reaches the downstream PC. Injected input is applied afterward by the
// normal merge, so an injected click on a masked button still passes.
// Only the fields whose report ID matches this report are touched.
__attribute__((cold, noinline))
static void usb_merge_phys_mouse(uint8_t *report, uint8_t len)
{
	uint8_t doff = mouse_layout.data_off;
	uint8_t rid  = doff ? report[0] : 0;

	bool xy_here    = (rid == mouse_layout.report_id);
	bool wheel_here = (mouse_layout.wheel_bit != 0xFFFF &&
	                   rid == mouse_layout.wheel_report_id);

	uint8_t phys_btn = xy_here ? report[doff] : 0;
	int32_t phys_x = 0, phys_y = 0, phys_w = 0;
	if (xy_here) {
		phys_x = read_report_field(report, len, mouse_layout.x_bit,
		                           mouse_layout.x_size, doff);
		phys_y = read_report_field(report, len, mouse_layout.y_bit,
		                           mouse_layout.y_size, doff);
	}
	if (wheel_here)
		phys_w = read_report_field(report, len, mouse_layout.wheel_bit,
		                           mouse_layout.wheel_size, doff);

	// Telemetry: the true physical input, BEFORE masking (spec §5.3).
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

// Feature B/C (keyboard): standard 8-byte boot report (modifier, reserved,
// 6 keycodes). Pushes the physical modifier+keys as TLM_PHYS_KB (pre-mask),
// then removes any masked keycodes from the outgoing report. Modifiers are not
// individually maskable in the KMBox API, so only keycodes are filtered.
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

// ── Injection accumulation (ported from v2 kmbox_inject_* / schedule_*) ───────
// On v2 these were public entry points called directly by actions.c on the SAME
// core. On v3 the V3F command core encodes them as ICC records (kmbox_cmd.c);
// usb_merge_drain_icc decodes them back into these accumulators here.
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
	// Replace existing entry for same key
	for (int i = 0; i < inject.kb_release_count; i++) {
		if (inject.kb_releases[i].key == key) {
			inject.kb_releases[i].at = at;
			return;
		}
	}
	// Add new entry
	if (inject.kb_release_count < MAX_KB_RELEASES) {
		inject.kb_releases[inject.kb_release_count].key = key;
		inject.kb_releases[inject.kb_release_count].at = at;
		inject.kb_release_count++;
	}
}

// Run the scheduled click/key-release bookkeeping (ported from v2
// kmbox_poll_fast's release-deadline blocks). Called once per drain.
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

void usb_merge_drain_icc(void)
{
	// Reset the per-cycle merge flag at the start of each relay-loop iteration.
	// v2 cleared this in kmbox_poll_fast() (the first per-loop call, before the
	// EP merge loop and before kmbox_send_pending()). usb_merge_drain_icc() is
	// the v3 analog: it is called first each iteration (main_v5f.c), the EP loop
	// then sets the flag via usb_merge_report(), and usb_merge_send_pending()
	// consumes it after the EP loop. This preserves v2's exact set/clear order:
	// cleared once per cycle → set in merge → consumed in send_pending.
	merged_this_cycle = false;

	icc_record_t r;
	while (icc_recv_from_v3f(&r)) {
		switch (r.tag) {
		case ICC_TAG_INJECT_MOUSE: {
			// kmbox_cmd.c wire: b0..1=dx LE int16, b2..3=dy LE int16,
			// b4=buttons, b5=wheel int8.
			int16_t dx = (int16_t)((uint16_t)r.b[0] | ((uint16_t)r.b[1] << 8));
			int16_t dy = (int16_t)((uint16_t)r.b[2] | ((uint16_t)r.b[3] << 8));
			uint8_t buttons = r.b[4];
			int8_t  wheel   = (int8_t)r.b[5];
			merge_apply_mouse(dx, dy, buttons, wheel);
			break;
		}
		case ICC_TAG_INJECT_KEYBOARD: {
			// b0=modifier, b1..6=keys[6].
			merge_apply_keyboard(r.b[0], &r.b[1]);
			break;
		}
		case ICC_TAG_CLICK_RELEASE: {
			// b0=button_mask, b1..4=delay_ms LE u32.
			uint32_t delay_ms = (uint32_t)r.b[1] | ((uint32_t)r.b[2] << 8) |
			                    ((uint32_t)r.b[3] << 16) | ((uint32_t)r.b[4] << 24);
			merge_schedule_click_release(r.b[0], delay_ms);
			break;
		}
		case ICC_TAG_KB_RELEASE: {
			// b0=key, b1..4=delay_ms LE u32.
			uint32_t delay_ms = (uint32_t)r.b[1] | ((uint32_t)r.b[2] << 8) |
			                    ((uint32_t)r.b[3] << 16) | ((uint32_t)r.b[4] << 24);
			merge_schedule_kb_release(r.b[0], delay_ms);
			break;
		}
		case ICC_TAG_SET_HUMAN_LEVEL:
			humanize_set_level(r.b[0]);
			break;
		case ICC_TAG_PHYS_MASK:
			// Forward-compatible: V3F currently applies phys-mask on its own
			// g_phys_mask instance (hurra.c l_phys_mask) and does NOT yet
			// forward over the ICC, so this record is not emitted today. When
			// it is (Phase 7), the encoding mirrors actions.c's API:
			//   b0=kind (0=mouse, 1=key, 2=unmask_all), b1=code, b2=enable.
			switch (r.b[0]) {
			case 0: act_phys_mask_mouse(r.b[1], r.b[2] != 0); break;
			case 1: act_phys_mask_key(r.b[1], r.b[2] != 0); break;
			case 2: act_phys_unmask_all(); break;
			default: break;
			}
			break;
		case ICC_TAG_SET_BAUD:
			// V3F owns the host UART; baud changes are applied there. Nothing
			// for the merge to do on V5F. Drained so the ring stays clear.
			break;
		default:
			// PING/PONG and telemetry tags are not the merge's concern; the
			// relay loop handles its own protocol records before draining here.
			break;
		}
	}

	// Mailbox is drained (every pending record consumed + acked). Re-arm the IPC
	// doorbell IT, which the ISR disables on each fire to stay storm-proof under
	// AutoEN (see IPC_CH0_Handler). Idempotent — safe to call every pass.
	icc_ipc_rearm_v5f();

	// Release-deadline bookkeeping (v2 ran this in kmbox_poll_fast each tick).
	merge_run_releases();
}

// ── Standalone synth-injection path (ported verbatim from v2 kmbox.c ~1071) ──
// When V3F injects motion while the physical mouse is SILENT, the merge path
// (which rides REAL device reports) emits nothing until the next real report.
// usb_merge_send_pending() synthesizes a standalone mouse report in that case,
// capped to one-per-ms, and also flushes an unconsumed wheel on a separate
// report ID. Called once per relay-loop iteration AFTER the per-EP merge/send
// loop (main_v5f.c), exactly where v2 main.c called kmbox_send_pending().
//
// merged_this_cycle gate: if a real mouse report was merged this cycle, the
// synth path returns early (after the separate-report-id wheel flush) so the
// two paths never both emit in one frame — which would flood/overwrite at the
// 1 kHz endpoint. The mouse-silent test (SYNTH_SILENCE_MS) is the redundant
// time-based guard for the case where merged_this_cycle is false but a real
// report arrived very recently.
//
// cached_mouse_ep is the DEVICE-side EP number the synth report is sent on. It
// is the same value used as ep_map[].dev_ep_num in main_v5f.c — both are
// derived from `interrupt_in_ep & 0x0F` of the mouse interface. main_v5f.c's
// usb_merge_cache_endpoints(&desc) call already caches it (along with
// cached_mouse_maxpkt and cached_kb_ep) from the captured descriptors, so no
// extra setter is needed: the merge already knows which EP to send on.

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
	// Only synthesize a standalone mouse report when the physical mouse has
	// gone silent — otherwise injection rides the next real report (merge),
	// so the two paths never both emit in the same frame (which would flood /
	// overwrite at the 1 kHz endpoint). Capped to one synth per ms.
	uint32_t ms = millis();
	bool mouse_silent = (uint32_t)(ms - last_merge_ms) >= SYNTH_SILENCE_MS;
	if (inject.mouse_dirty && mouse_silent && ms != last_synth_ms &&
	    cached_mouse_ep && mouse_layout.valid) {
		last_synth_ms = ms;
		uint8_t synth[16];
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
	uint8_t synth[16];
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
