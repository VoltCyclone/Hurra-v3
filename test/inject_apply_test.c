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
bool act_phys_kb_mask_active(void) { return false; }
bool act_phys_key_masked(uint8_t k) { (void)k; return false; }
static uint8_t masked_mouse_code = 0xFF; static bool masked_mouse_en;
void act_phys_mask_mouse(uint8_t c, bool e) { masked_mouse_code = c; masked_mouse_en = e; }
void act_phys_mask_key(uint8_t k, bool e) { (void)k; (void)e; }
void act_phys_unmask_all(void) {}
// ICC drain is not used in this test; provide empty stubs so usb_merge.c links.
bool icc_recv_from_v3f(icc_record_t *out) { (void)out; return false; }
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

static void setup_mouse(void) {
	captured_descriptors_t d; memset(&d, 0, sizeof d);
	d.valid = true; d.num_ifaces = 1;
	d.ifaces[0].iface_class = 0x03; d.ifaces[0].iface_protocol = 2;
	d.ifaces[0].interrupt_in_ep = 0x81; d.ifaces[0].interrupt_in_maxpkt = 4;
	d.ifaces[0].hid_report_desc_len = sizeof boot_mouse_rd;
	memcpy(d.ifaces[0].hid_report_desc, boot_mouse_rd, sizeof boot_mouse_rd);
	usb_merge_cache_endpoints(&d);
}

int main(void) {
	usb_merge_init();
	setup_mouse();

	// 1. INJECT_MOUSE: dx=+5, dy=-3, no buttons. Silent-path synth should emit a
	//    report carrying those deltas.
	{
		uint8_t b[7] = {0};
		b[0]=5; b[1]=0; b[2]=(uint8_t)(-3); b[3]=0xFF; b[4]=0; b[5]=0; // dx=5, dy=-3 LE
		usb_merge_apply_record(ICC_TAG_INJECT_MOUSE, b);
		sent_count = 0; fake_ms += 10;            // pass SYNTH_SILENCE_MS gate
		usb_merge_send_pending();
		CHECK(sent_count == 1, "inject_mouse emits one synth report");
		CHECK((int8_t)sent_buf[1] == 5,  "synth dx == +5");
		CHECK((int8_t)sent_buf[2] == -3, "synth dy == -3");
	}

	// 2. SET_HUMAN_LEVEL routes to humanize_set_level.
	{
		uint8_t b[7] = {0}; b[0] = 7;
		usb_merge_apply_record(ICC_TAG_SET_HUMAN_LEVEL, b);
		CHECK(last_human_level == 7, "set_human_level applied");
	}

	// 3. PHYS_MASK kind=0 (mouse) routes to act_phys_mask_mouse.
	{
		uint8_t b[7] = {0}; b[0]=0; b[1]=2; b[2]=1; // kind=mouse, code=2, enable
		usb_merge_apply_record(ICC_TAG_PHYS_MASK, b);
		CHECK(masked_mouse_code == 2 && masked_mouse_en, "phys_mask mouse applied");
	}

	printf(failures ? "\nFAILED (%d)\n" : "\nPASS\n", failures);
	return failures ? 1 : 0;
}
