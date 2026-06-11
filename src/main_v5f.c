// main_v5f.c — full MITM relay loop (V5F core).
//
// This is the centerpiece of the man-in-the-middle. On the V5F core:
//   * USBHS host (usb_host.c) captures the real device's HID reports.
//   * usb_merge.c overlays synthetic input — drained from the V3F command core
//     over the ICC — onto those reports (HID-report-descriptor-aware).
//   * USBFS device (usb_device.c) forwards the combined report to the PC.
//
// Ported from Hurra-v2 src/main.c, REMOVING (per Phase-5 spec):
//   * The PIT timer + pit0_isr + all PIT_*: v3 has no PIT. v2 reloaded the PIT
//     from humanize_timing_next() to pace the injection cadence. On v3 that
//     PIT-based reload pacing is INTENTIONALLY REPLACED by the loop-driven
//     synth path: usb_merge_send_pending() runs each relay-loop iteration,
//     gated by SYNTH_SILENCE_MS + a one-per-ms cap (millis()). The adaptive
//     feed-rate INPUT is still fed — humanize_record_arrival(timebase_v5f_us())
//     on each real mouse report (Task 7.1) — so the filter's interval estimate
//     stays accurate; only the PIT reload mechanism is dropped.
//   * tempmon_init/tempmon_read + all OVERTEMP/overclock logic (dropped).
//   * gpt_profile (i.MX µs counter) — replaced on v3 by the free-running 1 MHz
//     TIM9 counter (core/timebase_v5f.c: timebase_v5f_us()).
//
// KEPT structure (ports near-verbatim from v2 main.c):
//   usb_host_init -> wait connect -> port_reset -> device_speed ->
//   capture_descriptors -> SET_PROTOCOL per HID iface -> build ep_map[]/
//   out_map[] -> usb_merge_cache_endpoints -> usb_device_init -> wait
//   configured -> loop { drain ICC; device_poll; for each host IN EP:
//   poll_zerocopy -> merge -> device_send_report; for each OUT EP:
//   device_poll_out -> host_interrupt_out_send }.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "ch32h417_port.h"
#include "usb_host.h"
#include "usb_device.h"
#include "desc_capture.h"
#include "usb_merge.h"
#include "humanize.h"
#include "icc.h"
#include "timebase_v5f.h"
#include "led.h"
#include "debug.h"

// delay() shim for the V5F image. desc_capture.c (host-side enumeration) and
// the init sequence below call `extern void delay(uint32_t msec)`. The vendored
// debug.c only exposes Delay_Ms/Delay_Us, so we bridge here.
void delay(uint32_t msec) { Delay_Ms(msec); }

typedef struct {
	uint8_t  host_slot;
	uint8_t  dev_ep_num;
	uint16_t maxpkt;
	uint8_t  iface_protocol;  // 1=keyboard, 2=mouse (for usb_merge)
} ep_mapping_t;

typedef struct {
	uint8_t  host_slot;       // index into usb_host's intr_out arrays
	uint8_t  dev_ep_num;      // device-side EP number to poll
	uint16_t maxpkt;
} ep_out_mapping_t;

// Static to keep the ~3.6KB descriptor struct off the stack.
static captured_descriptors_t desc;

int main(void)
{
	SystemAndCoreClockUpdate();
	Delay_Init();

	// Millisecond timebase (TIM4) — the merge's release scheduling uses millis().
	timebase_v5f_init(SystemCoreClock);

	// Humanization filter seed (level defaults inside). The interval is a nominal
	// 1 kHz here; Phase-7 wires the measured poll interval (adaptive feed rate).
	humanize_init(1000);

	usb_merge_init();
	led_init();

	// --- ICC rendezvous with the V3F command core ------------------------
	// V3F sets the shared-block magic; we wait for it, then complete the HSEM
	// handshake and enable the V3F->V5F doorbell so injection records wake us.
	icc_init_v5f();
	HSEM_FastTake(HSEM_ID0);
	HSEM_ReleaseOneSem(HSEM_ID0, 0);
	IPC_ITConfig(IPC_CH0, IPC_CH_Sta_Bit0, ENABLE);
	NVIC_EnableIRQ(IPC_CH0_IRQn);

	led_on();

	// --- USBHS host: power port, wait for the real device ----------------
	usb_host_init();
	led_off();
	usb_host_power_on();
	while (!usb_host_device_connected()) {
		usb_host_power_on();
		// Drain any injection the V3F core queues during bring-up so the ICC
		// ring never backs up before the relay starts.
		usb_merge_drain_icc();
		__asm volatile("wfi");
	}

	led_on();
	delay(10);
	usb_host_port_reset();

	uint8_t speed = usb_host_device_speed();
	(void)speed;

	if (!capture_descriptors(&desc)) {
		led_blink_forever(5, 100, 100);
	}

	// capture_descriptors() already sends SET_CONFIG and SET_IDLE.
	// Send SET_PROTOCOL (Report Protocol) for each HID interface.
	usb_setup_t setup;
	int ret;
	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		// SET_PROTOCOL: bmRequestType=0x21, bRequest=0x0B
		setup.bmRequestType = 0x21;
		setup.bRequest = 0x0B; // SET_PROTOCOL
		setup.wValue = 1;      // 1 = Report Protocol (not Boot Protocol)
		setup.wIndex = desc.ifaces[i].iface_num;
		setup.wLength = 0;
		ret = usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&setup, NULL, 2000);
		(void)ret;
	}

	// --- Build the host->device endpoint maps (ports verbatim from v2) ----
	ep_mapping_t ep_map[MAX_INTR_EPS];
	uint8_t num_ep_mappings = 0;
	ep_out_mapping_t out_map[MAX_INTR_OUT_EPS];
	uint8_t num_out_mappings = 0;

	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		if (desc.ifaces[i].interrupt_in_ep == 0) continue;
		if (num_ep_mappings >= MAX_INTR_EPS) break;
		uint8_t slot = num_ep_mappings;
		uint8_t ep = desc.ifaces[i].interrupt_in_ep & 0x0F;

		usb_host_interrupt_init(slot, desc.dev_addr, ep,
			desc.ifaces[i].interrupt_in_maxpkt);

		ep_map[slot].host_slot      = slot;
		ep_map[slot].dev_ep_num     = ep;
		ep_map[slot].maxpkt         = desc.ifaces[i].interrupt_in_maxpkt;
		ep_map[slot].iface_protocol = desc.ifaces[i].iface_protocol;
		num_ep_mappings++;
	}

	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		if (desc.ifaces[i].interrupt_out_ep == 0) continue;
		if (num_out_mappings >= MAX_INTR_OUT_EPS) break;

		uint8_t slot = num_out_mappings;
		uint8_t ep = desc.ifaces[i].interrupt_out_ep & 0x0F;

		usb_host_interrupt_out_init(slot, desc.dev_addr, ep,
			desc.ifaces[i].interrupt_out_maxpkt);

		out_map[slot].host_slot  = slot;
		out_map[slot].dev_ep_num = ep;
		out_map[slot].maxpkt     = desc.ifaces[i].interrupt_out_maxpkt;
		num_out_mappings++;
	}

	// Parse the cloned HID report descriptor into the merge layout.
	usb_merge_cache_endpoints(&desc);

	// --- USBFS device: replay descriptors, wait for the PC to configure --
	if (!usb_device_init(&desc)) {
		led_blink_forever(9, 80, 120);
	}
	led_off();
	uint32_t dev_wait_start = millis();
	uint32_t dev_led_toggle = millis();
	while (!usb_device_is_configured()) {
		usb_device_poll();
		usb_merge_drain_icc(); // keep the ICC ring drained during bring-up
		if ((millis() - dev_led_toggle) >= 250) {
			led_toggle();
			dev_led_toggle = millis();
		}
		if ((millis() - dev_wait_start) > 30000) {
			led_blink_forever(8, 80, 120);
		}
	}
	led_off();
	led_heartbeat_start();

	// --- Relay loop ------------------------------------------------------
	uint32_t loop_count = 0;

	// Telemetry counters (ported from v2 main.c): reports successfully forwarded
	// to the PC vs. dropped (device-side EP busy). Streamed to V3F over the ICC
	// for the LED status ladder.
	uint32_t report_count = 0;
	uint32_t drop_count   = 0;
	uint32_t telem_tick   = millis();   // last time telemetry was emitted

	while (1) {
		bool did_work = false;

		// Drain injection from the V3F command core into the merge accumulators
		// (also runs scheduled click/key-release bookkeeping). Phase-5 cadence:
		// drain + merge + send every iteration, no PIT pacing.
		usb_merge_drain_icc();

		// USB device EP completion (unblock EPs for next send).
		usb_device_poll();

		// Host IN endpoints: capture -> merge -> forward to PC.
		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			uint8_t *rpt_ptr = NULL;
			ret = usb_host_interrupt_poll_zerocopy(ep_map[m].host_slot,
				&rpt_ptr, ep_map[m].maxpkt);
			if (ret > 0 && rpt_ptr) {
				did_work = true;
				// Timestamp real mouse-report arrival from the free-running
				// 1 MHz TIM9 µs counter (single CNT read, monotonic). This is
				// the precise "a report just arrived" point; only the mouse
				// interface drives the adaptive feed rate. Mirrors v2 main.c
				// (humanize_record_arrival(gpt_profile_us()), line ~309).
				if (ep_map[m].iface_protocol == 2)
					humanize_record_arrival(timebase_v5f_us());
				usb_merge_report(ep_map[m].iface_protocol,
					rpt_ptr, (uint8_t)ret);
				// Count forwarded vs. dropped reports for telemetry (ports v2
				// main.c: send_report true -> report_count, false -> drop_count).
				if (usb_device_send_report(
					ep_map[m].dev_ep_num, rpt_ptr, (uint16_t)ret))
					report_count++;
				else
					drop_count++;
			}
		}

		// Device OUT endpoints: PC -> real device (vendor HID++ etc.).
		for (uint8_t m = 0; m < num_out_mappings; m++) {
			uint8_t *out_data = NULL;
			int n = usb_device_poll_out(out_map[m].dev_ep_num, &out_data);
			if (n > 0 && out_data) {
				// Best-effort: if host-side OUT is still busy, drop this packet.
				// Real-world OUT traffic is low rate, so back-pressure via drop
				// is acceptable.
				(void)usb_host_interrupt_out_send(out_map[m].host_slot,
					out_data, (uint16_t)n);
			}
		}

		// Standalone synth-injection: emit injected motion when the physical
		// mouse is silent (the merge path above only fires on real reports).
		// One-per-ms cap + merged_this_cycle gate ensure the synth path and the
		// merge path never both emit in one frame. Mirrors v2 main.c's
		// kmbox_send_pending() call (line ~331), placed after the EP loops.
		usb_merge_send_pending();

		// --- Telemetry to V3F (LED status ladder) ------------------------
		// Cadence: at most once every 50 ms (millis() gate). The relay loop
		// can spin at tens of kHz; an unconditional send would flood the 256-
		// slot ICC ring and burn cycles on the hot path. 50 ms (20 Hz) is far
		// finer than V3F's 100 ms LED sampling, so the LED never sees stale
		// data, yet two cheap icc_send_to_v3f() calls per 50 ms are negligible.
		// V3F polls the ring (no doorbell needed for telemetry).
		uint32_t now = millis();
		if ((now - telem_tick) >= 50) {
			telem_tick = now;
			icc_record_t tr;

			// TELEM_COUNTS: report_count (LE u32) in b[0..3], drop_count in
			// b[4..7]. b[0] of icc_record_t is the tag; payload is b[].
			tr.tag = ICC_TAG_TELEM_COUNTS;
			tr.b[0] = (uint8_t)(report_count      );
			tr.b[1] = (uint8_t)(report_count >>  8);
			tr.b[2] = (uint8_t)(report_count >> 16);
			tr.b[3] = (uint8_t)(report_count >> 24);
			tr.b[4] = (uint8_t)(drop_count        );
			tr.b[5] = (uint8_t)(drop_count   >>  8);
			tr.b[6] = (uint8_t)(drop_count   >> 16);
			tr.b[7] = (uint8_t)(drop_count   >> 24);
			(void)icc_send_to_v3f(&tr);

			// TELEM_STATUS: b[0] = health flags.
			//   bit0 = USB device configured (PC enumerated us)
			//   bit1 = real host device still connected
			tr.tag = ICC_TAG_TELEM_STATUS;
			tr.b[0] = (usb_device_is_configured() ? 0x01 : 0x00) |
			          (usb_host_device_connected() ? 0x02 : 0x00);
			(void)icc_send_to_v3f(&tr);
		}

		if (!did_work)
			__asm volatile("wfi");

		// Liveness check: re-detect a yanked device every ~1024 idle-ish loops.
		if ((++loop_count & 0x3FF) == 0) {
			if (!usb_host_device_connected())
				led_blink_forever(6, 80, 80);
		}
	}
}
