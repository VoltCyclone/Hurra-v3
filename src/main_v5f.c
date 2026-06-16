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
#include "board.h"          // LED_GPIO_* for the SysTick-free trap-blink diag
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
// the init sequence below call `extern void delay(uint32_t msec)`. Route through
// the TIM9-based V5F-local delay, NOT the vendor Delay_Ms — the vendor delay
// spins on the shared SysTick0->ISR register and can be raced to a permanent
// hang by V3F (the "wedge on PC enumerate" bug). TIM9 is V5F-private/race-free.
void delay(uint32_t msec) { timebase_v5f_delay_ms(msec); }

// --- BENCH DIAG (2026-06-12): V5F trap catcher -----------------------------
// Symptom: V5F LED (PC3) sits SOLID-on during USBHS bring-up. The startup's
// weak HardFault_Handler is a bare `1: j 1b` spin that leaves the LED untouched,
// so a CPU trap (mcause=illegal-instr=2, load-fault, etc.) is INDISTINGUISHABLE
// from a plain C hang — both look solid. On QingKe RISC-V every synchronous
// exception funnels through the HardFault vector (slot 3), so a strong override
// here catches any V5F trap and makes it observable.
//
// It blinks the low nibble of mcause out on the LED, SysTick-FREE (raw GPIO +
// NOP loops — SysTick may be dead on V5F, which is why Delay_* can't be trusted):
//   long lead pulse, then N short pulses where N = (mcause & 0xF), then pause.
//   mcause 2 = illegal instruction, 5 = load access fault, 7 = store fault,
//   1 = instr access fault, 0 = instr-addr-misaligned, 11 = ecall-M.
// If the LED starts BLINKING this pattern after flashing, V5F is trapping (not
// hanging in C) and the nibble says why. If it stays SOLID, it is a true hang.
static void v5f_diag_raw_blink(uint8_t pulses, uint32_t on_nop, uint32_t off_nop)
{
	RCC_HB2PeriphClockCmd(LED_RCC_HB2, ENABLE);
	GPIO_InitTypeDef g = {0};
	g.GPIO_Pin = LED_GPIO_PIN; g.GPIO_Speed = GPIO_Speed_Very_High;
	g.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED_GPIO_PORT, &g);
	for (uint8_t i = 0; i < pulses; i++) {
		GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
		for (volatile uint32_t d = 0; d < on_nop;  d++) __asm volatile("nop");
		GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
		for (volatile uint32_t d = 0; d < off_nop; d++) __asm volatile("nop");
	}
}

// BENCH DIAG: catch the OTHER trap vectors too. Any of these not overridden falls
// through to the startup stub `1: j 1b` (infinite spin) — which looks IDENTICAL to
// a pure hang (stage frozen, no witness). On QingKe a clock/PLL fault can raise
// NMI, and a misrouted exception can land in Ecall/Break. Each stamps a distinct
// witness at 0x2017F0E0 so V3F can tell trap-class apart from a real hang.
//   0x40A11ED0 = HardFault, 0x40A11ED1 = NMI, 0x40A11ED2 = Ecall-M,
//   0x40A11ED3 = Ecall-U, 0x40A11ED4 = Break.
void NMI_Handler(void) __attribute__((interrupt("machine")));
void NMI_Handler(void)
{
	*(volatile uint32_t *)0x2017F0E0u = 0x40A11ED1u;
	uint32_t mcause, mepc;
	__asm volatile("csrr %0, mcause" : "=r"(mcause));
	__asm volatile("csrr %0, mepc"   : "=r"(mepc));
	*(volatile uint32_t *)0x2017F0E4u = mcause;
	*(volatile uint32_t *)0x2017F0E8u = mepc;
	for (;;) { __asm volatile("nop"); }
}

void Ecall_M_Mode_Handler(void) __attribute__((interrupt("machine")));
void Ecall_M_Mode_Handler(void)
{
	*(volatile uint32_t *)0x2017F0E0u = 0x40A11ED2u;
	uint32_t mcause = 0, mepc = 0;
	__asm volatile("csrr %0, mcause" : "=r"(mcause));
	__asm volatile("csrr %0, mepc"   : "=r"(mepc));
	*(volatile uint32_t *)0x2017F0E4u = mcause;
	*(volatile uint32_t *)0x2017F0E8u = mepc;
	for (;;) { __asm volatile("nop"); }
}

void Break_Point_Handler(void) __attribute__((interrupt("machine")));
void Break_Point_Handler(void)
{
	*(volatile uint32_t *)0x2017F0E0u = 0x40A11ED4u;
	uint32_t mcause = 0, mepc = 0;
	__asm volatile("csrr %0, mcause" : "=r"(mcause));
	__asm volatile("csrr %0, mepc"   : "=r"(mepc));
	*(volatile uint32_t *)0x2017F0E4u = mcause;
	*(volatile uint32_t *)0x2017F0E8u = mepc;
	for (;;) { __asm volatile("nop"); }
}

void HardFault_Handler(void) __attribute__((interrupt("machine")));
void HardFault_Handler(void)
{
	// BENCH DIAG: write-once trap witness at a DEDICATED address nothing else
	// touches, as the very FIRST thing — so even if the rest of this handler (or
	// dbg_stage) is clobbered/faults, V3F can tell "trap happened" from "pure
	// hang". 0x2017F0E0 = 0xT2A9 magic; 0x2017F0E4 = mcause; 0x2017F0E8 = mepc.
	*(volatile uint32_t *)0x2017F0E0u = 0x40A11ED0u;  /* "TRAPPED" witness */
	uint32_t mcause, mepc;
	__asm volatile("csrr %0, mcause" : "=r"(mcause));
	__asm volatile("csrr %0, mepc"   : "=r"(mepc));
	*(volatile uint32_t *)0x2017F0E4u = mcause;
	*(volatile uint32_t *)0x2017F0E8u = mepc;
	// Stamp the trap into the shared boot-stage marker (0x80 | low mcause nibble)
	// and the faulting PC into the following word, so V3F surfaces a V5F trap as a
	// single UART line (e.g. "V5F=0x82 TRAP mcause=2 mepc=0x200A1234") instead of a
	// PC3 blink that has to be counted by eye. Also keep a slow PC3 blink as a
	// probe-of-last-resort if the UART path is somehow dead.
	dbg_stage(DBG_V5F_TRAP_BASE | (mcause & 0x0F));
	*(volatile uint32_t *)(DBG_STAGE_ADDR + 4) = mcause;
	*(volatile uint32_t *)(DBG_STAGE_ADDR + 8) = mepc;
	uint8_t nib = (uint8_t)(mcause & 0x0F);
	if (nib == 0) nib = 16;  // distinguish "cause 0" from "no pulses"
	for (;;) {
		v5f_diag_raw_blink(1, 48000000u, 12000000u);   // long lead pulse
		v5f_diag_raw_blink(nib, 6000000u, 6000000u);   // N = mcause nibble
		for (volatile uint32_t d = 0; d < 60000000u; d++) __asm volatile("nop");
	}
}

typedef struct {
	uint8_t  host_slot;
	uint8_t  dev_ep_num;
	uint16_t maxpkt;
	uint8_t  iface_protocol;  // 1=keyboard, 2=mouse (for usb_merge)
	// bInterval pacing (matches WCH EVT app_km.c InEndpInterval/InEndpTimeCount):
	// the device only refreshes its IN FIFO once per service interval, so polling
	// faster just floods the bus with NAKs (we saw 1 report in 600k unpaced polls).
	// We pace each EP by its descriptor bInterval using the free-running µs counter.
	uint32_t interval_us;     // poll period in µs derived from bInterval + speed
	uint32_t next_poll_us;    // TIM9 µs timestamp when this EP may next be polled
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
	dbg_stage(DBG_V5F_BOOT);   // bench: see src/icc.h DBG_STAGE_ADDR
	*(volatile uint32_t *)0x2017F0E0u = 0;  // clear trap witness (SRAM is random post-flash)

	// Millisecond timebase (TIM4) — the merge's release scheduling uses millis().
	timebase_v5f_init(SystemCoreClock);
	dbg_stage(DBG_V5F_TIMEBASE);

	// Humanization filter seed (level defaults inside). The interval is a nominal
	// 1 kHz here; Phase-7 wires the measured poll interval (adaptive feed rate).
	humanize_init(1000);
	dbg_stage(DBG_V5F_HUMANIZE);

	usb_merge_init();
	dbg_stage(DBG_V5F_MERGE_INIT);
	led_init();
	dbg_stage(DBG_V5F_LED_INIT);

	// --- ICC rendezvous with the V3F command core ------------------------
	// V3F sets the shared-block magic; we wait for it, then complete the HSEM
	// handshake and enable the V3F->V5F doorbell so injection records wake us.
	icc_init_v5f();
	dbg_stage(DBG_V5F_ICC_MAGIC);
	HSEM_FastTake(HSEM_ID0);
	HSEM_ReleaseOneSem(HSEM_ID0, 0);
	dbg_stage(DBG_V5F_HSEM_DONE);
	IPC_ITConfig(IPC_CH0, IPC_CH_Sta_Bit0, ENABLE);
	NVIC_EnableIRQ(IPC_CH0_IRQn);
	dbg_stage(DBG_V5F_ICC_READY);

	// BENCH DIAG: the pre-USBHS window is now bisected by dbg_stage() markers that
	// V3F reads from shared SRAM and prints over UART — NOT by PC3 blink trains
	// (which overlaid ~3 pulse programs onto one pin and made it unreadable). PC3
	// is held dark here; it only starts its clean ~2 Hz toggle once we reach the
	// host-wait loop below, so "PC3 dark" vs "PC3 2 Hz" vs "trap blink" are the
	// only three PC3 states now.
	led_off();

	// --- USBHS host: power port, wait for the real device ----------------
	// The USBHS D+/D- lines are PB8/PB9, which on this part are ALSO the
	// SWCLK/SWDIO (SWJ) debug pins. The SWJ function must be released or the
	// USB2 PHY can't drive the pads and the host never detects an attached
	// device. Every WCH USBHS example does exactly this in Hardware() before
	// USBHS_Host_Init (EVT USBHS/HOST/Host_KM/Common/hardware.c:42-43).
	// NOTE: this severs SWD on PB8/PB9 — flash via the WCH-Link SDI link (which
	// uses its own pins), and recover debug with a power cycle if needed.
	dbg_stage(DBG_V5F_PRE_AFIO);    // 0x66: about to enable AFIO|GPIOB
	RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
	dbg_stage(DBG_V5F_PRE_SWJ);     // 0x67: AFIO|GPIOB on; about to disable SWJ
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	dbg_stage(DBG_V5F_PRE_HOSTINIT);// 0x68: SWJ disabled; about to call usb_host_init

	usb_host_init();
	dbg_stage(DBG_V5F_HOST_INIT);
	led_off();
	usb_host_power_on();

	// Dynamic attach/capture: wait for a device, try to capture its descriptors,
	// and if capture fails (or the device was yanked / mid-glitch), go back to
	// waiting for a clean attach instead of dead-looping. This lets the board be
	// flashed with nothing plugged in (the only reliable flash state — see the
	// 0x55 SWJ wedge), then have the real mouse hot-plugged afterward and still
	// enumerate. A failed capture re-powers the port and re-arms the wait.
	uint32_t wait_blink   = millis();
	for (;;) {
		// Host-wait. With no device on the port this blinks PC3 at ~2 Hz
		// ("searching"). The wfi wakes each ms from the TIM4 millis IRQ.
		dbg_stage(DBG_V5F_HOST_WAITING);
		while (!usb_host_device_connected()) {
			usb_host_power_on();
			// Drain V3F injection so the ICC mailbox never backs up pre-relay.
			usb_merge_drain_icc();
			uint32_t now = millis();
			if ((now - wait_blink) >= 250) {   // ~2 Hz "searching"
				wait_blink = now;
				led_toggle();
			}
			__asm volatile("wfi");
		}
		dbg_stage(DBG_V5F_DEV_CONNECTED);

		led_on();
		delay(10);
		dbg_stage(0x92);                 // before capture_descriptors
		// capture_descriptors() owns the bus-reset + post-reset stability poll +
		// the full address-assignment retry loop, so the caller does not pre-reset.
		if (capture_descriptors(&desc))
			break;                       // captured — proceed to clone + relay

		// Capture failed: stamp 0x9F (the oracle shows cap_step/cap_ret detail),
		// then loop back to host-wait. If the device is still physically present
		// but mid-glitch, re-powering + waiting for a stable connect gives it a
		// clean re-attach; if it was yanked, we wait for the next plug-in.
		dbg_stage(0x9F);
		led_off();
		delay(200);                      // let a glitching device settle / re-detect
	}
	dbg_stage(DBG_V5F_DESC_OK);

	// capture_descriptors() already sends SET_CONFIG and SET_IDLE.
	// SET_PROTOCOL(Report) for BOOT-SUBCLASS interfaces ONLY (bInterfaceSubClass==1).
	// Root cause of "cursor doesn't move": the Razer Basilisk V3's movement interface
	// (if0, EP0x81) is a BOOT mouse (class=0x03 sub=0x01 prot=0x02, GenericDesktop/
	// Mouse/Pointer w/ 16-bit X/Y — descriptor-verified). A boot-subclass HID device
	// may power up in Boot Protocol; without SET_PROTOCOL(Report) it does not stream
	// its report-protocol data and NAKs every interrupt-IN (bench: s0=0x2A NAK, n=0).
	// The working v2 host (USBHS i.MX) sends this and the cursor moves; v3 had dropped
	// it. We send it ONLY to subclass==1 interfaces (hits if0; spares the subclass-0
	// vendor/consumer interfaces if1..if3 that STALL it — sending to ALL made it worse,
	// which was the basis for the earlier blanket removal). wValue=1 = Report Protocol.
	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		if (desc.ifaces[i].iface_class != 0x03) continue;     // HID only
		if (desc.ifaces[i].iface_subclass != 0x01) continue;  // BOOT subclass only
		usb_setup_t sp;
		sp.bmRequestType = 0x21;   // Host->Device | Class | Interface
		sp.bRequest      = 0x0B;   // HID SET_PROTOCOL
		sp.wValue        = 1;      // 1 = Report Protocol (not Boot)
		sp.wIndex        = desc.ifaces[i].iface_num;
		sp.wLength       = 0;
		// Best-effort: a device may legally STALL; don't fail the relay over it.
		(void)usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&sp, NULL, 2000);
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
		// bInterval -> poll period in µs. For FS/LS, bInterval is in FRAMES (1 ms).
		// For HS it is 2^(bInterval-1) MICROframes (125 µs). The captured device
		// here enumerates full-speed (spd=0), so treat bInterval as milliseconds,
		// which also matches the EVT host (1 ms frame timer * InEndpInterval). Clamp
		// to >=1 ms so a 0/garbage bInterval can't busy-poll. We poll a touch faster
		// than the nominal interval (interval - small slack) so we never MISS the
		// device's fresh report by being a hair late, but we don't flood with NAKs.
		{
			uint32_t iv = desc.ifaces[i].interrupt_in_interval;
			if (iv == 0) iv = 1;
			if (iv > 255) iv = 255;
			uint32_t us = iv * 1000u;          // ms -> µs (full-speed framing)
			if (us > 300u) us -= 250u;         // small lead so we don't poll late
			ep_map[slot].interval_us  = us;
			ep_map[slot].next_poll_us = timebase_v5f_us();   // eligible immediately
		}
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
	dbg_stage(DBG_V5F_DEV_INIT);
	led_off();
	uint32_t dev_led_toggle = millis();
	while (!usb_device_is_configured()) {
		usb_device_poll();
		usb_merge_drain_icc(); // keep the ICC mailbox drained during bring-up
		if ((millis() - dev_led_toggle) >= 250) {
			led_toggle();
			dev_led_toggle = millis();
		}
	}
	// CROSS-CORE BUG FIX: do NOT call led_heartbeat_start() here. That starts
	// TIM2 + NVIC_EnableIRQ(TIM2_IRQn), but TIM2 is V3F's heartbeat timer (it owns
	// it, runs it on the HB1 bus, and TIM2_IRQn is core-allocated to V3F by
	// default). V5F enabling TIM2 collides with V3F and HANGS V5F right here — it
	// was the real blocker that kept V5F from reaching RELAY once Windows actually
	// enumerated the device (stage froze at 0x59, just past the configured loop).
	// V5F owns only its own status pin (PC3): drive it steady-on to mean "relay
	// running". The heartbeat ladder belongs to V3F on PC2.
	led_on();
	dbg_stage(DBG_V5F_RELAY);   // bench: 0x58 here == full relay reached

	// --- Relay loop ------------------------------------------------------
	uint32_t loop_count = 0;
	// QUICK end-to-end report-path probe latches (emitted every poll as code 0x20|…).
	uint8_t s_probe_got = 0, s_probe_fwd = 0, s_probe_drop = 0, s_probe_zerolen = 0;
	uint8_t s_probe_gotmask = 0;
	uint32_t s_probe_ms = millis();
	uint8_t s_probe_phase = 0;   // alternate the two probe codes each emit
	uint32_t hb_led_ms  = millis();   // last PC3 liveness-blink toggle time

	// V5F->V3F relay telemetry uses ONLY the coherent IPC status-bit channel
	// (icc_telem_stage_v5f). It must never write V3F-side SRAM (0x2017xxxx) from
	// this hot loop: such cross-core stores can stall the V5F pipeline on an AHB
	// access that never returns, freezing the core with no trap. ITRC is a
	// per-stage wedge tracer kept as a no-op for production (re-point it to
	// icc_telem_stage_v5f when locating a relay-loop hang); the loop bottom emits
	// the dominant, sampling-stable report-path probe.
	#define ITRC(v) ((void)0)

	while (1) {
		bool did_work = false;

		++loop_count;
		// V5F must NOT write V3F-side SRAM (0x2017xxxx) from this hot loop: those
		// are cross-core stores that can stall the V5F store pipeline on an AHB
		// access that never returns, freezing the core with no trap. All V5F->V3F
		// telemetry goes over the coherent IPC status-bit channel instead, and the
		// only liveness signal here is PC3 (V5F's own GPIO).
		uint32_t now_ms = millis();
		if ((now_ms - hb_led_ms) >= 250u) {   // ~2 Hz "relay running" blink on PC3
			hb_led_ms = now_ms;
			led_toggle();
		}
		ITRC(TLM_RLY_TOP);

		// Drain injection from the V3F command core into the merge accumulators
		// (also runs scheduled click/key-release bookkeeping). Phase-5 cadence:
		// drain + merge + send every iteration, no PIT pacing.
		usb_merge_drain_icc();
		ITRC(TLM_RLY_DRAIN);

		// USB device EP completion (unblock EPs for next send).
		usb_device_poll();
		ITRC(TLM_RLY_DEVPOLL);

		// Host IN endpoints: capture -> merge -> forward to PC.
		// bInterval PACING (matches WCH EVT app_km.c): only issue an IN token for an
		// endpoint once its per-EP service interval has elapsed. Polling flat-out
		// every loop pass floods the bus with NAKs (the device only refreshes its IN
		// FIFO once per interval) — measured 1 report per 600k unpaced polls. Pacing
		// by the captured bInterval lets each poll land in the device's fresh-report
		// window, and collapses host bus/CPU load by ~1000x (which also removes the
		// per-iteration USBHS-register/cross-core storm implicated in the wedge).
		uint32_t now_us = timebase_v5f_us();
		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			// Not yet due? skip this EP this pass. Unsigned wrap-safe compare.
			if ((int32_t)(now_us - ep_map[m].next_poll_us) < 0)
				continue;
			ep_map[m].next_poll_us = now_us + ep_map[m].interval_us;
			uint8_t *rpt_ptr = NULL;
			int ret = usb_host_interrupt_poll_zerocopy(ep_map[m].host_slot,
				&rpt_ptr, ep_map[m].maxpkt);
			// QUICK PROBE (latched, emitted EVERY poll so 1s sampling always sees it):
			// one persistent status byte tracking the whole relay path end-to-end.
			// Code 0x20 | got<<3 | fwd<<2 | drop<<1 | zerolen. got=a poll returned
			// n>0; fwd=a report reached the USBFS device EP; drop=send_report rejected
			// one; zerolen=a SUCCESS poll had n==0 (RX_LEN read as 0 → ret=0 → skips
			// the forward path). This tells us EXACTLY where the report path stops.
			if (ret > 0) {
				s_probe_got = 1u;
				// Latch which slots deliver reports (per-slot bitmask) — purely
				// diagnostic; every slot is forwarded identically (transparent MITM).
				s_probe_gotmask |= (uint8_t)(1u << (m & 0x3));
			} else {
				// distinguish "SUCCESS but zero-length" from a real NAK: oks counts
				// SUCCESS regardless of n, so a SUCCESS with ret==0 is a zero-len RX.
				extern volatile uint8_t usbh_dbg_in_last_s;
				if (usbh_dbg_in_last_s == 0 /*ERR_SUCCESS*/) s_probe_zerolen = 1u;
			}
			// (probe byte is emitted once per iteration at the loop bottom — see there)
			if (ret > 0 && rpt_ptr) {
				did_work = true;
				// Feed the adaptive humanizer with the real mouse-report arrival
				// time (free-running 1 MHz TIM9 µs counter). Only the boot mouse
				// interface (protocol==2) drives the feed rate.
				if (ep_map[m].iface_protocol == 2)
					humanize_record_arrival(timebase_v5f_us());
				ITRC(TLM_RLY_MERGE);
				usb_merge_report(ep_map[m].iface_protocol,
					rpt_ptr, (uint8_t)ret);
				ITRC(TLM_RLY_SEND);
				bool fwd_ok = usb_device_send_report(
					ep_map[m].dev_ep_num, rpt_ptr, (uint16_t)ret);
				if (fwd_ok) s_probe_fwd  = 1u;
				else        s_probe_drop = 1u;
			}
		}
		ITRC(TLM_RLY_OUT);

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

		// EP0 vendor reports: PC -> real device via control endpoint.
		// Razer Synapse / Logitech HID++ push config writes (RGB, DPI, macros)
		// as HID SET_REPORT control transfers, which land on our USBFS device
		// EP0. usb_device captured the setup + payload; replay it verbatim onto
		// the real device's EP0 over USBHS. Same-core hand-off (both run on V5F),
		// so no ICC channel — just drain, forward, release. Best-effort: a STALL
		// or NAK timeout from the real device is dropped silently; we never fail
		// the relay over a vendor control write. The transfer is synchronous and
		// rare, so issuing it inline in the loop is acceptable.
		{
			uint8_t *rpt = NULL;
			uint16_t rpt_val = 0, rpt_idx = 0;
			int rn = usb_device_poll_ep0_report(&rpt, &rpt_val, &rpt_idx);
			if (rn > 0 && rpt) {
				usb_setup_t sp;
				sp.bmRequestType = 0x21;  // Host->Device | Class | Interface
				sp.bRequest      = 0x09;  // HID SET_REPORT
				sp.wValue        = rpt_val;
				sp.wIndex        = rpt_idx;
				sp.wLength       = (uint16_t)rn;
				(void)usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
					&sp, rpt, 2000);
				usb_device_ep0_report_done();
			}
		}

		// Standalone synth-injection: emit injected motion when the physical
		// mouse is silent (the merge path above only fires on real reports).
		// One-per-ms cap + merged_this_cycle gate ensure the synth path and the
		// merge path never both emit in one frame. Mirrors v2 main.c's
		// kmbox_send_pending() call (line ~331), placed after the EP loops.
		usb_merge_send_pending();
		ITRC(TLM_RLY_PENDING);

		// DOMINANT telemetry: emit the latched end-to-end report-path probe at the
		// BOTTOM of every iteration (last-write-wins, so the slow 1s oracle sampling
		// always catches THIS, not a transient stage marker). Throttled to ~200ms to
		// keep IPC traffic light. Code 0x20 | got<<3 | fwd<<2 | drop<<1 | zerolen.
		if ((now_ms - s_probe_ms) >= 200u) {
			s_probe_ms = now_ms;
			s_probe_phase ^= 1u;
			if (s_probe_phase)
				// 0x20 | got<<3 | fwd<<2 | drop<<1 | zerolen  (end-to-end path)
				icc_telem_stage_v5f((uint8_t)(0x20u | (s_probe_got << 3)
					| (s_probe_fwd << 2) | (s_probe_drop << 1) | s_probe_zerolen));
			else
				// 0x10 | gotmask[3:0]  (WHICH host IN slots ever delivered a report;
				// slot order matches ep_map = capture order = if0,if1,if2,if3).
				icc_telem_stage_v5f((uint8_t)(0x10u | (s_probe_gotmask & 0x0F)));
		}
		if (!did_work)
			__asm volatile("wfi");

		// Liveness check: re-detect a yanked device every ~1024 idle-ish loops.
		// A TRANSIENT not-connected read here must NOT hang the relay (the old
		// led_blink_forever froze V5F at 0x58 and poisoned every diagnostic). Only
		// treat a SUSTAINED disconnect (several consecutive checks) as a real
		// removal, and on removal just publish a marker — the loop keeps running so
		// the device can be re-attached without a reflash.
		if ((loop_count & 0x3FF) == 0) {
			static uint8_t disc_run;
			if (!usb_host_device_connected()) {
				if (++disc_run >= 4) {
					// Sustained disconnect: report via the coherent IPC channel
					// (NOT a 0x2017xxxx SRAM store). A distinct code lets V3F show
					// "device removed" without any cross-core SRAM write.
					icc_telem_stage_v5f(0x3F);   // 0x3F = device-removed marker
				}
			} else {
				disc_run = 0;
			}
		}
	}
}
