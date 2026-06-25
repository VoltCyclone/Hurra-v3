// test/usb_merge_test.c — host-native bounds tests for the HID-report merge.
//
// Built with AddressSanitizer against exact-size heap buffers so any
// out-of-bounds access on a short/malformed report (or an oversized synth
// buffer flush) traps precisely instead of silently corrupting adjacent RAM.
//
// The merge module's injection state is fed through the real ICC drain path
// (icc_recv_from_v3f is stubbed to replay a scripted record), and the real
// HID-descriptor parser, so the layout under test is exactly what firmware
// would compute on the bench.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "usb_merge.h"
#include "icc.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } else { \
    printf("ok: %s\n", msg); } } while (0)

// ── Stubs for usb_merge.c's external dependencies ────────────────────────────
uint16_t g_phys_mask;
volatile int8_t g_tb_dev_temp_c;          // referenced by usb_merge_drain_icc

static uint32_t fake_ms;
uint32_t millis(void) { return fake_ms; }

// Humanize stubs so injected deltas arrive verbatim at the report.
bool humanize_pending(void) { return false; }
void humanize_return(int16_t dx, int16_t dy) { (void)dx; (void)dy; }
/* stub: identity quantizer — the noise=0 inject-emit path. usb_merge_take_injection
 * calls this so injected deltas pass through quantize+carry with no added noise. */
void humanize_inject_emit(float dx, float dy, int16_t *ox, int16_t *oy) {
    *ox = (int16_t)dx; *oy = (int16_t)dy;
}

bool act_phys_kb_mask_active(void) { return false; }
bool act_phys_key_masked(uint8_t k) { (void)k; return false; }
void act_phys_mask_mouse(uint8_t c, bool e) { (void)c; (void)e; }
void act_phys_mask_key(uint8_t k, bool e) { (void)k; (void)e; }
void act_phys_unmask_all(void) {}

// Capture the last report the synth path flushed, copying exactly rlen bytes so
// ASan flags a read past the source synth buffer.
static uint8_t  sent_buf[256];
static uint8_t  sent_len;
static uint8_t  sent_ep;
void usb_device_send_report(uint8_t ep, const uint8_t *data, uint8_t len) {
	sent_ep = ep;
	sent_len = len;
	memcpy(sent_buf, data, len);
}

// Controllable device-EP-free fake: drives the ACK-gate in usb_merge_send_pending.
static bool fake_ep_free = true;
bool usb_device_in_ep_free(uint8_t ep) { (void)ep; return fake_ep_free; }

// Scripted ICC record queue (one-shot replay into the drain path).
static icc_record_t icc_queue[8];
static int icc_q_head, icc_q_tail;
bool icc_recv_from_v3f(icc_record_t *out) {
	if (icc_q_head == icc_q_tail) return false;
	*out = icc_queue[icc_q_head++ & 7];
	return true;
}
void icc_ipc_rearm_v5f(void) {}
static void icc_push_inject_mouse(int16_t dx, int16_t dy, uint8_t btn, int8_t wheel) {
	icc_record_t r = {0};
	r.tag  = ICC_TAG_INJECT_MOUSE;
	r.b[0] = (uint8_t)(dx & 0xFF);  r.b[1] = (uint8_t)((dx >> 8) & 0xFF);
	r.b[2] = (uint8_t)(dy & 0xFF);  r.b[3] = (uint8_t)((dy >> 8) & 0xFF);
	r.b[4] = btn;                   r.b[5] = (uint8_t)wheel;
	icc_queue[icc_q_tail++ & 7] = r;
}

// ── Standard boot-mouse report descriptor → fast-path layout ─────────────────
// Layout: [buttons@0, X@1, Y@2, wheel@3], data_off=0, all 8-bit → fast_path.
static const uint8_t boot_mouse_rd[] = {
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
	0x95, 0x03, 0x75, 0x01, 0x81, 0x02,           // 3 button bits
	0x95, 0x01, 0x75, 0x05, 0x81, 0x03,           // 5 const padding
	0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
	0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06, // X,Y,Wheel 8-bit
	0xC0, 0xC0
};

// Report-ID mouse: a leading report-ID byte forces data_off=1. Non-byte-aligned
// is unnecessary; this stays byte-aligned but with a report ID so the merge
// takes the slow path (fast_path requires the same checks but the ID prefix and
// our truncation exercise the report[doff]/field guards).
#define RID_MOUSE 0x01
static const uint8_t rid_mouse_rd[] = {
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
	0x85, RID_MOUSE,                              // Report ID 1
	0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
	0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
	0x95, 0x01, 0x75, 0x05, 0x81, 0x03,
	0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
	0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
	0xC0, 0xC0
};

static captured_descriptors_t desc;
static void cache_report_id_mouse(void) {
	memset(&desc, 0, sizeof(desc));
	desc.num_ifaces = 1;
	desc.ifaces[0].iface_protocol     = 2;
	desc.ifaces[0].interrupt_in_ep    = 0x81;
	desc.ifaces[0].interrupt_in_maxpkt = 8;
	desc.ifaces[0].hid_report_desc_len = sizeof(rid_mouse_rd);
	memcpy(desc.ifaces[0].hid_report_desc, rid_mouse_rd, sizeof(rid_mouse_rd));
	usb_merge_init();
	usb_merge_cache_endpoints(&desc);
}

static void cache_boot_mouse(void) {
	memset(&desc, 0, sizeof(desc));
	desc.num_ifaces = 1;
	desc.ifaces[0].iface_protocol     = 2;        // mouse
	desc.ifaces[0].interrupt_in_ep    = 0x81;
	desc.ifaces[0].interrupt_in_maxpkt = 8;
	desc.ifaces[0].hid_report_desc_len = sizeof(boot_mouse_rd);
	memcpy(desc.ifaces[0].hid_report_desc, boot_mouse_rd, sizeof(boot_mouse_rd));
	usb_merge_init();
	usb_merge_cache_endpoints(&desc);
}

// ACK-gating: a busy IN EP must HOLD injection (no emit, no drain); a free EP
// emits and drains. This proves cadence is paced by EP-free, not millis(), and
// that motion is never drained into a report that doesn't land.
static void test_synth_ack_gating(void) {
    usb_merge_reset_for_test();          // zero injection + cadence state
    cache_boot_mouse();                  // btn@0, X@1, Y@2 layout, EP 0x81

    fake_ms = 1000;                            // advance past SYNTH_SILENCE_MS gate
    icc_push_inject_mouse(10, -5, 0, 0);
    usb_merge_drain_icc();                     // -> inject.mouse_dx=10, dy=-5, dirty

    // EP busy: must NOT emit, must NOT drain.
    fake_ep_free = false;
    sent_len = 0;
    usb_merge_send_pending();
    CHECK(sent_len == 0, "busy EP: no synth report emitted");

    // EP free: emits one report carrying the held delta.
    fake_ep_free = true;
    sent_len = 0;
    usb_merge_send_pending();
    CHECK(sent_len > 0, "free EP: synth report emitted");
    CHECK((int8_t)sent_buf[1] == 10, "free EP: dx carried (x@byte1)");
    CHECK((int8_t)sent_buf[2] == (int8_t)-5, "free EP: dy carried (y@byte2)");

    // Delta consumed: a second free poll with no new injection emits nothing.
    sent_len = 0;
    usb_merge_send_pending();
    CHECK(sent_len == 0, "free EP: drained delta not re-emitted");
}

// With the EP held busy, many injects must accumulate toward a bounded cap and
// never wrap int16 (which would flip a large +x flick into a -x jump on resume).
static void test_synth_accum_saturates(void) {
    usb_merge_reset_for_test();
    cache_boot_mouse();
    fake_ms = 1000;
    fake_ep_free = false;                       // host not consuming

    for (int i = 0; i < 10000; i++) {           // 10000 * 100 = 1,000,000 >> int16
        icc_push_inject_mouse(100, 100, 0, 0);
        usb_merge_drain_icc();
        usb_merge_send_pending();               // held: no emit, accumulates
    }
    int16_t held_x = usb_merge_peek_inject_dx_for_test();
    int16_t held_y = usb_merge_peek_inject_dy_for_test();
    CHECK(held_x > 0 && held_x <= INJECT_ACCUM_CAP, "accum dx bounded, no wrap");
    CHECK(held_y > 0 && held_y <= INJECT_ACCUM_CAP, "accum dy bounded, no wrap");
}

// The wheel accumulator must saturate at the int8 range like dx/dy, not wrap.
// Repeated same-sign scroll injects between flushes (EP held busy) must clamp to
// +127, never roll negative — a wrapped value would reverse the user's scroll.
static void test_synth_wheel_accum_saturates(void) {
    usb_merge_reset_for_test();
    cache_boot_mouse();
    fake_ms = 1000;
    fake_ep_free = false;                       // host not consuming -> accumulate

    for (int i = 0; i < 50; i++) {              // 50 * 100 = 5000 >> int8 range
        icc_push_inject_mouse(0, 0, 0, 100);
        usb_merge_drain_icc();
        usb_merge_send_pending();               // held: no emit, wheel accumulates
    }
    int8_t held_w = usb_merge_peek_inject_wheel_for_test();
    CHECK(held_w == 127, "accum wheel saturates at +127, no int8 wrap");
}

int main(void) {
	// (1) Fast path on a short report must not index past the buffer. A 4-byte
	// boot mouse touches report[3] (wheel); a 2-byte report has only [0],[1].
	// Injecting motion drove an unconditional read/write of report[2] (Y).
	{
		cache_boot_mouse();
		icc_push_inject_mouse(5, 5, 0, 0);
		usb_merge_drain_icc();
		uint8_t *rpt = malloc(2);     // exact size; ASan-poisoned past [1]
		rpt[0] = 0; rpt[1] = 0;
		usb_merge_report(2, rpt, 2);  // must clamp to len, not touch [2]
		free(rpt);
		CHECK(1, "fast-path short report (len=2) stays in bounds");
	}

	// (2) Regression: a full 4-byte boot-mouse report still merges injection.
	// The bounds guard must not suppress the happy path.
	// usb_merge_take_injection calls humanize_inject_emit (noise=0); the stub is
	// an identity quantizer, so the merged delta must equal the injected values
	// exactly — no added component.
	{
		cache_boot_mouse();
		icc_push_inject_mouse(7, -9, 0x01, 0);
		usb_merge_drain_icc();
		uint8_t *rpt = malloc(4);
		rpt[0] = 0; rpt[1] = 0; rpt[2] = 0; rpt[3] = 0;
		usb_merge_report(2, rpt, 4);
		int8_t merged_dx = (int8_t)rpt[1];
		int8_t merged_dy = (int8_t)rpt[2];
		bool ok = (rpt[0] == 0x01) && (merged_dx == 7) && (merged_dy == -9);
		free(rpt);
		CHECK(ok, "fast-path full report (len=4) still applies injection");
		/* inject_emit passthrough: dx and dy must match injected values exactly;
		 * no perpendicular noise component is added (noise=0 quantizer path). */
		CHECK(merged_dx == 7,  "inject_emit: dx magnitude preserved (no perp noise)");
		CHECK(merged_dy == -9, "inject_emit: dy magnitude preserved (no perp noise)");
	}

	// (3) Slow path: a report whose ID byte arrives but body is truncated must
	// not index report[doff] or fields past len. A report-ID layout forces the
	// slow path; len=1 carries only the ID byte.
	{
		cache_report_id_mouse();
		icc_push_inject_mouse(3, 3, 0x02, 0);
		usb_merge_drain_icc();
		uint8_t *rpt = malloc(1);
		rpt[0] = RID_MOUSE;           // matching report ID, nothing after it
		usb_merge_report(2, rpt, 1);  // doff=1 == len; report[1] is OOB
		free(rpt);
		CHECK(1, "slow-path report-ID-only (len=1) stays in bounds");
	}

	// (4) Synth flush: when the physical mouse is silent, the standalone synth
	// path emits a report of cached_mouse_report_len bytes. If that length
	// exceeds the synth stack buffer, send_report reads past it. Drive a large
	// first-report length, then a silent synth cycle.
	// A real device whose report is longer than the synth stack buffer sets
	// cached_mouse_report_len high; the synth flush must clamp the emitted
	// length to the buffer it copies from. (Stack overreads are unreliable
	// under ASan, so assert the behavioral contract: never emit more than the
	// 64-byte synth buffer holds. Old synth[16]+rlen=100 emits 100 → RED.)
	{
		cache_boot_mouse();
		fake_ms = 1000;
		const int RLEN = 100;          // > post-fix synth buffer (64)
		icc_push_inject_mouse(1, 1, 0, 0);
		usb_merge_drain_icc();
		uint8_t *rpt = malloc(RLEN);
		memset(rpt, 0, RLEN);
		usb_merge_report(2, rpt, RLEN); // establishes cached_mouse_report_len=100
		free(rpt);
		// Advance time so the physical mouse counts as silent, then inject and
		// flush through the standalone synth path.
		sent_len = 0;
		fake_ms = 1010;
		icc_push_inject_mouse(2, 2, 0, 0);
		usb_merge_drain_icc();
		usb_merge_send_pending();
		CHECK(sent_len > 0, "synth path actually emitted a report");
		CHECK(sent_len <= 64, "synth flush length clamped to buffer size");
	}

	test_synth_ack_gating();
	test_synth_accum_saturates();
	test_synth_wheel_accum_saturates();

	if (failures == 0) printf("ALL PASS\n");
	else printf("%d FAILURE(S)\n", failures);
	return failures ? 1 : 0;
}
