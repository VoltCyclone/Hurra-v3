// main_v5f.c — full MITM relay loop (V5F core).
//
// On the V5F core:
//   * USBHS host (usb_host.c) captures the real device's HID reports.
//   * usb_merge.c overlays synthetic input — drained from the V3F command core
//     over the ICC — onto those reports (HID-report-descriptor-aware).
//   * USBFS device (usb_device.c) forwards the combined report to the PC.
//
// Injection cadence is loop-driven: usb_merge_send_pending() runs each relay-loop
// iteration, gated by SYNTH_SILENCE_MS and the device IN-EP-free (ACK) gate, which
// self-paces synth output to the cloned device's real poll rate (1 kHz FS, up to
// 8 kHz HS). The free-running 1 MHz TIM9 counter (timebase_v5f_us()) provides µs timing.
//
// Bring-up sequence: usb_host_init -> wait connect -> port_reset -> device_speed
// -> capture_descriptors -> SET_PROTOCOL per HID iface -> build ep_map[]/out_map[]
// -> usb_merge_cache_endpoints -> usb_device_init -> wait configured -> relay loop.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "ch32h417_port.h"
#include "board.h"          // LED_GPIO_* for the trap-blink diag
#include "usb_host.h"
#include "usb_device.h"
#include "desc_capture.h"
#include "usb_merge.h"
#include "humanize.h"
#include "icc.h"
#include "timebase_v5f.h"
#include "led.h"
#include "debug.h"
#include "spi_link_selftest.h"
#include "two_board.h"
#include "report_xfer.h"

// delay() shim for the V5F image. desc_capture.c and the init sequence call
// `extern void delay(uint32_t msec)`. Routed through the TIM9-based V5F-local
// delay, not the vendor Delay_Ms: the vendor delay spins on the shared
// SysTick0->ISR register and can be raced to a permanent hang by V3F. TIM9 is
// V5F-private and race-free.
void delay(uint32_t msec) { timebase_v5f_delay_ms(msec); }

// --- V5F trap catcher -------------------------------------------------------
// The startup weak HardFault_Handler is a bare `1: j 1b` spin, so a CPU trap is
// indistinguishable from a plain C hang (both leave the LED solid). On QingKe
// RISC-V every synchronous exception funnels through the HardFault vector, so this
// strong override catches any V5F trap and makes it observable.
//
// It blinks the low nibble of mcause on the LED, SysTick-free (raw GPIO + NOP
// loops, since SysTick may be dead on V5F): a long lead pulse, then N short pulses
// where N = (mcause & 0xF), then a pause. mcause 2 = illegal instruction, 5 = load
// access fault, 7 = store fault, 1 = instr access fault, 0 = instr-addr-misaligned,
// 11 = ecall-M. Blinking => V5F is trapping and the nibble says why; solid => hang.
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

// Catch the other trap vectors too. Without an override each falls through to the
// startup stub `1: j 1b`, indistinguishable from a pure hang. On QingKe a
// clock/PLL fault can raise NMI and a misrouted exception can land in Ecall/Break.
// Each stamps a distinct witness at 0x2017F0E0 so V3F can tell trap-class from a
// real hang.
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
	// Write-once trap witness at a dedicated address nothing else touches, first,
	// so even if the rest of this handler (or dbg_stage) faults, V3F can tell a trap
	// from a pure hang. 0x2017F0E4 = mcause; 0x2017F0E8 = mepc.
	*(volatile uint32_t *)0x2017F0E0u = 0x40A11ED0u;  /* "TRAPPED" witness */
	uint32_t mcause, mepc;
	__asm volatile("csrr %0, mcause" : "=r"(mcause));
	__asm volatile("csrr %0, mepc"   : "=r"(mepc));
	*(volatile uint32_t *)0x2017F0E4u = mcause;
	*(volatile uint32_t *)0x2017F0E8u = mepc;
	// Stamp the trap into the shared boot-stage marker (0x80 | low mcause nibble)
	// so V3F surfaces a V5F trap as a UART line. mcause/mepc are already in the
	// witness words above (0x2017F0E4/E8); V3F reads them there, so no extra
	// cross-core writes are needed here. Keep a slow PC3 blink as a UART-dead
	// fallback.
	dbg_stage(DBG_V5F_TRAP_BASE | (mcause & 0x0F));
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
	// bInterval pacing: the device refreshes its IN FIFO once per service interval,
	// so polling faster just floods the bus with NAKs. Each EP is paced by its
	// descriptor bInterval using the free-running µs counter.
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
	dbg_stage(DBG_V5F_BOOT);
	*(volatile uint32_t *)0x2017F0E0u = 0;  // clear trap witness (SRAM is random post-flash)

	// Millisecond timebase (TIM4); the merge's release scheduling uses millis().
	timebase_v5f_init(SystemCoreClock);
	dbg_stage(DBG_V5F_TIMEBASE);

	// Humanization filter seed at a nominal 1 kHz; the measured poll interval
	// (adaptive feed rate) is wired in later.
	humanize_init(1000);
	dbg_stage(DBG_V5F_HUMANIZE);

	usb_merge_init();
	dbg_stage(DBG_V5F_MERGE_INIT);
	led_init();
	dbg_stage(DBG_V5F_LED_INIT);

#ifdef SPI_LINK_SELFTEST
	// Bench harness (Makefile SELFTEST=master|slave): run the board-to-board SPI
	// link echo test and never return. Diverts before any USB/ICC bring-up so the
	// test firmware exercises only the SPI link + LED.
	spi_link_selftest_run();
#endif

	// --- ICC rendezvous with the V3F command core ------------------------
	// V3F sets the shared-block magic (waited for in icc_init_v5f); then enable
	// the V3F->V5F doorbell so injection records wake us. HSEM is intentionally
	// unused — V3F never takes HSEM_ID0, so a take/release here was a no-op that
	// only looked like synchronization.
	icc_init_v5f();
	dbg_stage(DBG_V5F_ICC_MAGIC);
	IPC_ITConfig(IPC_CH0, IPC_CH_Sta_Bit0, ENABLE);
	NVIC_EnableIRQ(IPC_CH0_IRQn);
	dbg_stage(DBG_V5F_ICC_READY);

	// Hold PC3 dark through the pre-USBHS window; it starts its ~2 Hz toggle only at
	// the host-wait loop below, so PC3 has three distinct states: dark, 2 Hz, and
	// trap blink.
	led_off();

#if defined(BOARD_ROLE_HOST)
	// Board B = SPI master. Captures the real device on its USBHS host port (PB8/9),
	// which needs SWJ released like the device path — unless built with
	// TWO_BOARD_HOST_SYNTH (synthetic source, no USB host) or TWO_BOARD_HOST_CDC_ECHO
	// (CDC-ACM isolation echo on USBFS PA11/PA12, no USBHS host), where the PB8/9 SWJ
	// pins are unused so SWD stays alive for bring-up. Never returns. See two_board.c.
#  if !defined(TWO_BOARD_HOST_SYNTH) && !defined(TWO_BOARD_HOST_CDC_ECHO)
	dbg_stage(DBG_V5F_PRE_AFIO);    // 0x66: about to enable AFIO|GPIOB
	RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
	dbg_stage(DBG_V5F_PRE_SWJ);     // 0x67: AFIO|GPIOB on; about to disable SWJ
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	dbg_stage(DBG_V5F_PRE_HOSTINIT);// 0x68: SWJ disabled; about to bring up USBHS host
#  endif
	two_board_host_run();
#elif defined(BOARD_ROLE_DEVICE)
	// Board A (Makefile BOARD=device): enumerates a synthetic boot mouse on the
	// USBHSD device port (PB8/9) and replays SPI-received reports to the PC. USBHS
	// device needs the SWJ pins released like the host path. Never returns.
	dbg_stage(DBG_V5F_PRE_AFIO);    // 0x66: about to enable AFIO|GPIOB
	RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
	dbg_stage(DBG_V5F_PRE_SWJ);     // 0x67: AFIO|GPIOB on; about to disable SWJ
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	dbg_stage(DBG_V5F_PRE_HOSTINIT);// 0x68: SWJ disabled; about to bring up USBHSD
	two_board_device_run();
#endif

	// --- USBHS host: power port, wait for the real device ----------------
	// USBHS D+/D- are PB8/PB9, which on this part are also the SWCLK/SWDIO (SWJ)
	// debug pins. SWJ must be released or the USB2 PHY can't drive the pads and the
	// host never detects a device. This severs SWD on PB8/PB9: flash via the
	// WCH-Link SDI link (its own pins) and recover debug with a power cycle.
	dbg_stage(DBG_V5F_PRE_AFIO);    // 0x66: about to enable AFIO|GPIOB
	RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
	dbg_stage(DBG_V5F_PRE_SWJ);     // 0x67: AFIO|GPIOB on; about to disable SWJ
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	dbg_stage(DBG_V5F_PRE_HOSTINIT);// 0x68: SWJ disabled; about to call usb_host_init

	if (!usb_host_init())
		led_blink_forever(3, 80, 120);  // 3 pulses = USBHS 480M PLL never locked
	dbg_stage(DBG_V5F_HOST_INIT);
	led_off();
	usb_host_power_on();

	// Dynamic attach/capture: wait for a device, try to capture its descriptors,
	// and on failure (yanked or mid-glitch) go back to waiting for a clean attach
	// instead of dead-looping. This lets the board be flashed with nothing plugged
	// in (the reliable flash state) and have the real mouse hot-plugged afterward.
	// A failed capture re-powers the port and re-arms the wait.
	display_status_t s_disp = { .state = DISP_STATE_BOOT };
	uint32_t wait_blink   = millis();
	for (;;) {
		// Host-wait: with no device, blink PC3 at ~2 Hz. The wfi wakes each ms from
		// the TIM4 millis IRQ.
		dbg_stage(DBG_V5F_HOST_WAITING);
		s_disp.state = DISP_STATE_WAITING;
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
		s_disp.state = DISP_STATE_CAPTURING;

		led_on();
		delay(10);
		dbg_stage(0x92);                 // before capture_descriptors
		// capture_descriptors() owns the bus-reset + post-reset stability poll +
		// the address-assignment retry loop, so the caller does not pre-reset.
		if (capture_descriptors(&desc))
			break;                       // captured — proceed to clone + relay

		// Capture failed: stamp 0x9F, then loop back to host-wait. Re-powering and
		// waiting for a stable connect gives a glitching device a clean re-attach;
		// a yanked device waits for the next plug-in.
		dbg_stage(0x9F);
		led_off();
		delay(200);                      // let a glitching device settle / re-detect
	}
	dbg_stage(DBG_V5F_DESC_OK);
	s_disp.vid = (uint16_t)(desc.device_desc[8]  | (desc.device_desc[9]  << 8));
	s_disp.pid = (uint16_t)(desc.device_desc[10] | (desc.device_desc[11] << 8));

	// capture_descriptors() already sends SET_CONFIG and SET_IDLE. Send
	// SET_PROTOCOL(Report) to boot-subclass interfaces only (bInterfaceSubClass==1):
	// a boot-subclass HID device may power up in Boot Protocol and then NAKs every
	// interrupt-IN until switched to Report Protocol. Subclass-0 vendor/consumer
	// interfaces may STALL it, so they are skipped. wValue=1 = Report Protocol.
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

	// --- Build the host->device endpoint maps ----------------------------
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
		// bInterval -> poll period in µs. The single-board relay path does not set
		// desc.speed (stays USB_SPEED_FULL), so it operates FS-only by design and
		// the ms framing here is correct for it. The two-board host path decodes
		// speed-aware via bInterval_to_us() (see usb_desc_interval.h); if this path
		// ever gains real HS capture, switch it to that helper too. Clamp >=1 ms so
		// a garbage bInterval can't busy-poll, and poll a touch early to catch the
		// fresh report without flooding NAKs.
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
	// Do not call led_heartbeat_start() here: it starts TIM2 + TIM2_IRQn, which
	// belong to V3F (HB1 bus, core-allocated to V3F). V5F enabling TIM2 collides
	// with V3F and hangs V5F. V5F owns only PC3: drive it steady-on to mean "relay
	// running"; the heartbeat ladder belongs to V3F on PC2.
	led_on();
	dbg_stage(DBG_V5F_RELAY);   // 0x58 = full relay reached
	s_disp.state = DISP_STATE_RELAYING;

	// --- Relay loop ------------------------------------------------------
	uint32_t loop_count = 0;
	// End-to-end report-path probe latches (emitted as code 0x20|…).
	uint8_t s_probe_got = 0, s_probe_fwd = 0, s_probe_drop = 0, s_probe_zerolen = 0;
	uint8_t s_probe_gotmask = 0;
	uint32_t s_probe_ms = millis();
	uint8_t s_probe_phase = 0;   // alternate the two probe codes each emit
	uint32_t hb_led_ms  = millis();   // last PC3 liveness-blink toggle time
	static uint32_t s_rep_count, s_rep_tick;
	static uint32_t s_drop_count;     // cumulative, never reset — clamped at display time

	// V5F->V3F relay telemetry uses only the coherent IPC status-bit channel
	// (icc_telem_stage_v5f); it must never write V3F-side SRAM from this hot loop
	// (cross-core stores can stall the V5F pipeline on an AHB access that never
	// returns, freezing the core with no trap). ITRC is a per-stage wedge tracer
	// kept as a no-op for production (re-point it to icc_telem_stage_v5f when
	// locating a relay-loop hang).
	#define ITRC(v) ((void)0)

	while (1) {
		bool did_work = false;

		++loop_count;
		uint32_t now_ms = millis();
		if ((now_ms - hb_led_ms) >= 250u) {   // ~2 Hz "relay running" blink on PC3
			hb_led_ms = now_ms;
			led_toggle();
		}
		ITRC(TLM_RLY_TOP);

		// Drain injection from the V3F command core into the merge accumulators
		// (also runs scheduled click/key-release bookkeeping). Drain + merge + send
		// every iteration, no PIT pacing.
		usb_merge_drain_icc();
		ITRC(TLM_RLY_DRAIN);

		// USB device EP completion (unblock EPs for next send).
		usb_device_poll();
		ITRC(TLM_RLY_DEVPOLL);

		// Host IN endpoints: capture -> merge -> forward to PC.
		// bInterval pacing: issue an IN token for an endpoint only once its per-EP
		// service interval has elapsed. Polling every pass floods the bus with NAKs
		// (the device refreshes its IN FIFO once per interval); pacing by the
		// captured bInterval lands each poll in the fresh-report window and collapses
		// host bus/CPU load by ~1000x.
		uint32_t now_us = timebase_v5f_us();
		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			// Not yet due? skip this EP this pass. Unsigned wrap-safe compare.
			if ((int32_t)(now_us - ep_map[m].next_poll_us) < 0)
				continue;
			ep_map[m].next_poll_us = now_us + ep_map[m].interval_us;
			uint8_t *rpt_ptr = NULL;
			int ret = usb_host_interrupt_poll_zerocopy(ep_map[m].host_slot,
				&rpt_ptr, ep_map[m].maxpkt);
			// Latched relay-path probe, emitted every poll. Code 0x20 | got<<3 |
			// fwd<<2 | drop<<1 | zerolen. got=poll returned n>0; fwd=report reached the
			// USBFS device EP; drop=send_report rejected one; zerolen=SUCCESS poll with
			// n==0. Names where the report path stops.
			if (ret > 0) {
				s_probe_got = 1u;
				// Latch which slots deliver reports (per-slot bitmask), diagnostic only;
				// every slot is forwarded identically (transparent MITM).
				s_probe_gotmask |= (uint8_t)(1u << (m & 0x3));
			} else {
				// Distinguish a zero-length SUCCESS poll from a real NAK.
				extern volatile uint8_t usbh_dbg_in_last_s;
				if (usbh_dbg_in_last_s == 0 /*ERR_SUCCESS*/) s_probe_zerolen = 1u;
			}
			// (probe byte is emitted once per iteration at the loop bottom)
			if (ret > 0 && rpt_ptr) {
				did_work = true;
				ITRC(TLM_RLY_MERGE);
				// The merge path stores the report length in uint8_t and the
				// transactional forwarder is bounded to the same 64-byte maximum.
				// Reject oversized host packets before narrowing the poll result.
				if (ret > (int)REPORT_XFER_MAX_REPORT) {
					s_probe_drop = 1u;
					s_drop_count++;
					continue;
				}
				bool fwd_ok = usb_merge_forward_report(ep_map[m].dev_ep_num,
					ep_map[m].iface_protocol, rpt_ptr, (uint8_t)ret);
				ITRC(TLM_RLY_SEND);
				if (fwd_ok) { s_probe_fwd  = 1u; s_rep_count++; }
				else        { s_probe_drop = 1u; s_drop_count++; }
			}
		}
		ITRC(TLM_RLY_OUT);

		// Device OUT endpoints: PC -> real device (vendor HID++ etc.).
		for (uint8_t m = 0; m < num_out_mappings; m++) {
			uint8_t *out_data = NULL;
			int n = usb_device_poll_out(out_map[m].dev_ep_num, &out_data);
			if (n > 0 && out_data) {
				// Best-effort: if host-side OUT is busy, drop this packet. OUT traffic is
				// low rate, so drop-based back-pressure is acceptable.
				(void)usb_host_interrupt_out_send(out_map[m].host_slot,
					out_data, (uint16_t)n);
			}
		}

		// EP0 vendor reports: PC -> real device via control endpoint. Vendor config
		// writes (RGB, DPI, macros) arrive as HID SET_REPORT control transfers on our
		// USBFS device EP0; replay the captured setup + payload onto the real device's
		// EP0 over USBHS. Same-core hand-off, so no ICC channel. Best-effort: a STALL
		// or NAK timeout is dropped silently. The transfer is synchronous and rare, so
		// issuing it inline is acceptable.
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

		// Standalone synth-injection: emit injected motion when the physical mouse is
		// silent (the merge path above only fires on real reports). The device IN-EP-free
		// (ACK) gate and the merged_this_cycle gate keep the synth and merge paths from
		// both emitting in one frame.
		usb_merge_send_pending();
		ITRC(TLM_RLY_PENDING);

		// Emit the latched end-to-end report-path probe at the bottom of every
		// iteration (last-write-wins, so the ~1s oracle sampling catches it, not a
		// transient stage marker). Throttled to ~200 ms to keep IPC traffic light.
		if ((now_ms - s_probe_ms) >= 200u) {
			s_probe_ms = now_ms;
			s_probe_phase ^= 1u;
			if (s_probe_phase)
				// 0x20 | got<<3 | fwd<<2 | drop<<1 | zerolen  (end-to-end path)
				icc_telem_stage_v5f((uint8_t)(0x20u | (s_probe_got << 3)
					| (s_probe_fwd << 2) | (s_probe_drop << 1) | s_probe_zerolen));
			else
				// 0x10 | gotmask[3:0]  (which host IN slots delivered a report; slot
				// order matches ep_map = capture order).
				icc_telem_stage_v5f((uint8_t)(0x10u | (s_probe_gotmask & 0x0F)));
		}
		// Status display feed (non-essential, coherent IPC MMIO only, never SRAM).
		if ((now_ms - s_rep_tick) >= 1000u) {
			s_rep_tick = now_ms;
			s_disp.reports_per_sec = (s_rep_count > 1023u) ? 1023u : (uint16_t)s_rep_count;
			s_rep_count = 0;
			// relay-health snapshot for the status display (published via pump)
			s_disp.probe   = (uint8_t)((s_probe_got << 3) | (s_probe_fwd << 2)
			                         | (s_probe_drop << 1) | s_probe_zerolen);
			s_disp.gotmask = (uint8_t)(s_probe_gotmask & 0x0F);
			s_disp.drops   = (uint16_t)(s_drop_count > 1023u ? 1023u : s_drop_count);
		}
		icc_status_pump_v5f(&s_disp);   // rotate-publish one field; cheap IPC MMIO

		if (!did_work)
			__asm volatile("wfi");

		// Liveness check: re-detect a yanked device every ~1024 loops. A transient
		// not-connected read must not hang the relay, so only a sustained disconnect
		// (several consecutive checks) counts as removal, and on removal the loop
		// keeps running so the device can be re-attached without a reflash.
		if ((loop_count & 0x3FF) == 0) {
			static uint8_t disc_run;
			if (!usb_host_device_connected()) {
				if (++disc_run >= 4) {
					// Sustained disconnect: report via the coherent IPC channel. A
					// distinct code lets V3F show "device removed".
					icc_telem_stage_v5f(0x3F);   // 0x3F = device-removed marker
				}
			} else {
				disc_run = 0;
			}
		}
	}
}
