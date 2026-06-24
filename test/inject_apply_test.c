// test/inject_apply_test.c — verifies usb_merge_apply_record decodes each record
// tag into the injection accumulators identically to the legacy ICC drain switch.
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "usb_merge.h"
#include "icc.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } else { printf("ok: %s\n", msg); } } while (0)

// ── Stubs for usb_merge.c's external deps (same set as usb_merge_test.c) ──────
uint16_t g_phys_mask;
volatile int8_t g_tb_dev_temp_c;
static uint32_t fake_ms;
uint32_t millis(void) { return fake_ms; }
void humanize_filter(int16_t *dx, int16_t *dy) { (void)dx; (void)dy; }
bool humanize_pending(void) { return false; }
void humanize_return(int16_t dx, int16_t dy) { (void)dx; (void)dy; }
static uint8_t last_human_level = 0xFF;
void humanize_set_level(uint8_t level) { last_human_level = level; }
/* stub: identity quantizer for noise=0 inject-emit path (task 5) */
void humanize_inject_emit(float dx, float dy, int16_t *ox, int16_t *oy) {
    *ox = (int16_t)dx; *oy = (int16_t)dy;
}
bool act_phys_kb_mask_active(void) { return false; }
bool act_phys_key_masked(uint8_t k) { (void)k; return false; }
static uint8_t masked_mouse_code = 0xFF; static bool masked_mouse_en;
void act_phys_mask_mouse(uint8_t c, bool e) { masked_mouse_code = c; masked_mouse_en = e; }
static uint8_t masked_key_code = 0xFF; static bool masked_key_en;
void act_phys_mask_key(uint8_t k, bool e) { masked_key_code = k; masked_key_en = e; }
static int unmask_all_called = 0;
void act_phys_unmask_all(void) { unmask_all_called++; }
// icc_recv_from_v3f: normally returns false; overridden per-test by setting
// a one-shot record into icc_one_shot and arming icc_one_armed.
static icc_record_t icc_one_shot;
static bool icc_one_armed;
bool icc_recv_from_v3f(icc_record_t *out) {
	if (!icc_one_armed) return false;
	icc_one_armed = false;
	*out = icc_one_shot;
	return true;
}
void icc_ipc_rearm_v5f(void) {}

static uint8_t sent_buf[256]; static uint8_t sent_len, sent_ep; static int sent_count;
void usb_device_send_report(uint8_t ep, const uint8_t *data, uint8_t len) {
	sent_ep = ep; sent_len = len; sent_count++;
	memcpy(sent_buf, data, len);
}

// Standard boot-mouse report descriptor → fast-path layout [btn@0,X@1,Y@2,wheel@3].
static const uint8_t boot_mouse_rd[] = {
	0x05,0x01,0x09,0x02,0xA1,0x01,0x09,0x01,0xA1,0x00,
	0x05,0x09,0x19,0x01,0x29,0x03,0x15,0x00,0x25,0x01,
	0x95,0x03,0x75,0x01,0x81,0x02, 0x95,0x01,0x75,0x05,0x81,0x03,
	0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,0x15,0x81,0x25,0x7F,
	0x75,0x08,0x95,0x03,0x81,0x06, 0xC0,0xC0
};

// Setup a mouse-only device (iface_protocol=2, ep=0x81).
static void setup_mouse(void) {
	captured_descriptors_t d; memset(&d, 0, sizeof d);
	d.valid = true; d.num_ifaces = 1;
	d.ifaces[0].iface_class = 0x03; d.ifaces[0].iface_protocol = 2;
	d.ifaces[0].interrupt_in_ep = 0x81; d.ifaces[0].interrupt_in_maxpkt = 4;
	d.ifaces[0].hid_report_desc_len = sizeof boot_mouse_rd;
	memcpy(d.ifaces[0].hid_report_desc, boot_mouse_rd, sizeof boot_mouse_rd);
	usb_merge_cache_endpoints(&d);
}

// Setup a device with both keyboard (iface_protocol=1, ep=0x82) and mouse
// (iface_protocol=2, ep=0x81) interfaces.
static void setup_mouse_and_keyboard(void) {
	captured_descriptors_t d; memset(&d, 0, sizeof d);
	d.valid = true; d.num_ifaces = 2;
	// Keyboard interface first.
	d.ifaces[0].iface_class = 0x03; d.ifaces[0].iface_protocol = 1;
	d.ifaces[0].interrupt_in_ep = 0x82; d.ifaces[0].interrupt_in_maxpkt = 8;
	// Mouse interface.
	d.ifaces[1].iface_class = 0x03; d.ifaces[1].iface_protocol = 2;
	d.ifaces[1].interrupt_in_ep = 0x81; d.ifaces[1].interrupt_in_maxpkt = 4;
	d.ifaces[1].hid_report_desc_len = sizeof boot_mouse_rd;
	memcpy(d.ifaces[1].hid_report_desc, boot_mouse_rd, sizeof boot_mouse_rd);
	usb_merge_cache_endpoints(&d);
}

int main(void) {
	// ── Test 1: INJECT_MOUSE ─────────────────────────────────────────────────────
	// dx=+5, dy=-3, no buttons.  Silent-path synth should emit a report carrying
	// those deltas once fake_ms has advanced past the SYNTH_SILENCE_MS gate.
	{
		usb_merge_init(); setup_mouse();
		uint8_t b[7] = {0};
		b[0]=5; b[1]=0; b[2]=(uint8_t)(-3); b[3]=0xFF; b[4]=0; b[5]=0;
		usb_merge_apply_record(ICC_TAG_INJECT_MOUSE, b);
		sent_count = 0; fake_ms += 10;
		usb_merge_send_pending();
		CHECK(sent_count == 1, "inject_mouse emits one synth report");
		CHECK((int8_t)sent_buf[1] == 5,  "synth dx == +5");
		CHECK((int8_t)sent_buf[2] == -3, "synth dy == -3");
	}

	// ── Test 2: SET_HUMAN_LEVEL ──────────────────────────────────────────────────
	// The tag routes directly to humanize_set_level(); the stub captures the value.
	{
		usb_merge_init(); setup_mouse();
		uint8_t b[7] = {0}; b[0] = 7;
		usb_merge_apply_record(ICC_TAG_SET_HUMAN_LEVEL, b);
		CHECK(last_human_level == 7, "set_human_level applied");
	}

	// ── Test 3: PHYS_MASK kind=0 (mouse) ────────────────────────────────────────
	{
		usb_merge_init(); setup_mouse();
		uint8_t b[7] = {0}; b[0]=0; b[1]=2; b[2]=1; // kind=mouse, code=2, enable
		usb_merge_apply_record(ICC_TAG_PHYS_MASK, b);
		CHECK(masked_mouse_code == 2 && masked_mouse_en, "phys_mask mouse applied");
	}

	// ── Test 4: INJECT_KEYBOARD ─────────────────────────────────────────────────
	// b[0]=modifier, b[1..6]=keys[6].  After applying, usb_merge_send_pending()
	// should call usb_device_send_report on the cached keyboard EP (0x02 — the low
	// nibble of interrupt_in_ep 0x82) with a standard 8-byte boot-keyboard report:
	//   synth[0]=modifier, synth[1]=0, synth[2..7]=keys[6].
	{
		usb_merge_init(); setup_mouse_and_keyboard();
		uint8_t b[7] = {0};
		b[0] = 0x02;                          // modifier: left shift
		b[1] = 0x04; b[2] = 0x05; b[3] = 0;  // keys: 'a', 'b', rest zero
		usb_merge_apply_record(ICC_TAG_INJECT_KEYBOARD, b);
		sent_count = 0;
		usb_merge_send_pending();
		CHECK(sent_count == 1, "inject_keyboard emits one report");
		CHECK(sent_ep == 0x02, "keyboard report sent on kb ep");
		CHECK(sent_len == 8, "keyboard report is 8 bytes");
		CHECK(sent_buf[0] == 0x02, "keyboard modifier correct");
		CHECK(sent_buf[2] == 0x04, "keyboard key[0] correct");
		CHECK(sent_buf[3] == 0x05, "keyboard key[1] correct");
	}

	// ── Test 5: CLICK_RELEASE ───────────────────────────────────────────────────
	// Apply a click-release record with a 50 ms delay.  The schedule is accepted
	// (no crash, no assert).  We verify the schedule fires via usb_merge_drain_icc,
	// which is the only path that calls merge_run_releases().  Because the
	// icc_recv_from_v3f stub returns false immediately, drain_icc still calls
	// merge_run_releases() after the empty loop, which is sufficient.
	// Before the delay expires the button bit should still be set; after advancing
	// fake_ms and calling drain_icc again the release clears it.
	{
		usb_merge_init(); setup_mouse_and_keyboard();
		fake_ms = 1000;

		// First press the button via INJECT_MOUSE so the accumulator has it.
		uint8_t bm[7] = {0}; bm[4] = 0x01; // button 1 held
		usb_merge_apply_record(ICC_TAG_INJECT_MOUSE, bm);

		// Schedule click-release: button_mask=0x01, delay=50 ms.
		uint8_t br[7] = {0};
		br[0] = 0x01;                      // button_mask
		br[1] = 50; br[2] = 0; br[3] = 0; br[4] = 0; // delay_ms=50 LE
		usb_merge_apply_record(ICC_TAG_CLICK_RELEASE, br);
		// Schedule was accepted — no crash.
		CHECK(1, "click_release apply accepted without crash");

		// Drain at t=1020 (before the 50 ms deadline at t=1050).  Button still live.
		fake_ms = 1020;
		usb_merge_drain_icc(); // merge_run_releases() inside; deadline not yet
		sent_count = 0; fake_ms += 5;
		usb_merge_send_pending();
		// The synth flush should emit the pending button bit (mouse_dirty).
		CHECK(sent_count == 1, "click_release: button still emitted before deadline");
		CHECK(sent_buf[0] & 0x01, "click_release: button bit set before release");

		// Advance past deadline and drain again — release should fire.
		fake_ms = 1060;
		usb_merge_drain_icc(); // merge_run_releases() clears button
		sent_count = 0; fake_ms += 5;
		usb_merge_send_pending();
		// After release the button bit in the synth report must be clear.
		CHECK(sent_buf[0] == 0x00 || sent_count == 0,
		      "click_release: button cleared after deadline");
	}

	// ── Test 6: KB_RELEASE ──────────────────────────────────────────────────────
	// Similar to CLICK_RELEASE but for a keyboard key.  Schedule a release for
	// key 0x04 ('a') with a 30 ms delay, first injecting the key so it is in the
	// kb_keys[] accumulator.
	{
		usb_merge_init(); setup_mouse_and_keyboard();
		fake_ms = 2000;

		// Inject the keyboard state with key 0x04 held.
		uint8_t bk[7] = {0}; bk[0] = 0; bk[1] = 0x04;
		usb_merge_apply_record(ICC_TAG_INJECT_KEYBOARD, bk);

		// Schedule kb-release: key=0x04, delay=30 ms.
		uint8_t bkr[7] = {0};
		bkr[0] = 0x04;                       // key code
		bkr[1] = 30; bkr[2] = 0; bkr[3] = 0; bkr[4] = 0; // delay_ms=30 LE
		usb_merge_apply_record(ICC_TAG_KB_RELEASE, bkr);
		CHECK(1, "kb_release apply accepted without crash");

		// Drain before deadline (t=2015 < t=2030).
		fake_ms = 2015;
		usb_merge_drain_icc();
		sent_count = 0;
		usb_merge_send_pending();
		// Key should still be present in the emitted report.
		bool key_present_before = false;
		if (sent_count > 0) {
			for (int i = 2; i < 8; i++)
				if (sent_buf[i] == 0x04) { key_present_before = true; break; }
		}
		CHECK(key_present_before, "kb_release: key present before deadline");

		// Advance past deadline and drain again.
		fake_ms = 2040;
		usb_merge_drain_icc(); // merge_run_releases() clears the key
		sent_count = 0;
		usb_merge_send_pending();
		bool key_gone_after = true;
		if (sent_count > 0) {
			for (int i = 2; i < 8; i++)
				if (sent_buf[i] == 0x04) { key_gone_after = false; break; }
		}
		CHECK(key_gone_after, "kb_release: key cleared after deadline");
	}

	// ── Test 7: SET_BAUD ────────────────────────────────────────────────────────
	// SET_BAUD is a no-op in apply_record (baud is owned by the command transport).
	// Assert that calling it after a mouse inject does not disturb the accumulator:
	// the previously-injected delta must still appear in the next synth flush.
	{
		usb_merge_init(); setup_mouse();
		fake_ms = 5000;

		// Inject a mouse delta.
		uint8_t bm[7] = {0}; bm[0] = 10; // dx=10
		usb_merge_apply_record(ICC_TAG_INJECT_MOUSE, bm);

		// Apply SET_BAUD — must be a no-op.
		uint8_t bb[7] = {0}; bb[0] = 3; // some baud index
		usb_merge_apply_record(ICC_TAG_SET_BAUD, bb);

		// The delta should still flush cleanly.
		sent_count = 0; fake_ms += 10;
		usb_merge_send_pending();
		CHECK(sent_count == 1, "set_baud: mouse delta still flushes after set_baud");
		CHECK((int8_t)sent_buf[1] == 10, "set_baud: dx=10 preserved after set_baud");
	}

	// ── Test 8: PHYS_MASK kind=1 (key) ──────────────────────────────────────────
	{
		usb_merge_init(); setup_mouse();
		masked_key_code = 0xFF; masked_key_en = false;
		uint8_t b[7] = {0}; b[0]=1; b[1]=0x10; b[2]=1; // kind=key, code=0x10, enable
		usb_merge_apply_record(ICC_TAG_PHYS_MASK, b);
		CHECK(masked_key_code == 0x10 && masked_key_en, "phys_mask key applied");
	}

	// ── Test 9: PHYS_MASK kind=2 (unmask_all) ───────────────────────────────────
	{
		usb_merge_init(); setup_mouse();
		unmask_all_called = 0;
		uint8_t b[7] = {0}; b[0]=2; // kind=unmask_all
		usb_merge_apply_record(ICC_TAG_PHYS_MASK, b);
		CHECK(unmask_all_called == 1, "phys_mask unmask_all called");
	}

	printf(failures ? "\nFAILED (%d)\n" : "\nPASS\n", failures);
	return failures ? 1 : 0;
}
