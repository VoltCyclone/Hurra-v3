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

	// BENCH DIAG: publish V5F's clock facts so the oracle can verify delay scaling.
	// The flaky first-SETUP failure (setup_s=0xFE) is consistent with mis-scaled
	// V5F delays (TIM9/TIM4 prescaler from a wrong HCLK). Publish HCLK + the live
	// TIM9 µs counter so we can measure its true rate against the Mac wall clock.
	{
		RCC_ClocksTypeDef ck;
		RCC_GetClocksFreq(&ck);
		*(volatile uint32_t *)0x2017F010u = ck.HCLK_Frequency;   // free slot
		*(volatile uint32_t *)0x2017F014u = SystemCoreClock;
		*(volatile uint32_t *)0x2017F018u = ck.SYSCLK_Frequency;
	}
	// BENCH DIAG: prove TIM9 is actually counting. Sample CNT twice with a short
	// vendor-SysTick delay between (independent of TIM9). If d==0, TIM9 is dead.
	{
		uint32_t c0 = TIM9->CNT_32;
		Delay_Us(50);                 // vendor SysTick delay (NOT TIM9-based)
		uint32_t c1 = TIM9->CNT_32;
		*(volatile uint32_t *)0x2017F01Cu = c0;
		*(volatile uint32_t *)0x2017F020u = c1 - c0;   // expect ~50 if TIM9 @1MHz
	}

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
	uint32_t wait_iter    = 0;
	uint32_t cap_attempts = 0;
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
				dbg_usbhs_regs(USBHSH->CFG, USBHSH->PORT_CFG, USBHSH->PORT_STATUS,
				               USBHSH->PORT_STATUS_CHG, USBHSH->PORT_CTRL, ++wait_iter);
			}
			__asm volatile("wfi");
		}
		dbg_stage(DBG_V5F_DEV_CONNECTED);

		led_on();
		delay(10);
		dbg_stage(0x92);                 // before capture_descriptors
		// capture_descriptors() owns the bus-reset + post-reset stability poll +
		// the full address-assignment retry loop, so the caller does not pre-reset.
		// Publish the attempt count so the oracle shows re-tries (0x2017F0B4).
		*(volatile uint32_t *)0x2017F0B4u = ++cap_attempts;
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

	// BENCH DIAG (descriptor-clone bring-up): publish the captured interface table
	// so the V3F oracle can show, at the relay stage, exactly what we cloned —
	// per-interface class/subclass/protocol, the IN endpoint, and whether the HID
	// report descriptor was actually captured (len>0). This distinguishes "mouse
	// report descriptor STALLed/empty so Windows kept only the keyboard" from a
	// protocol-keying mismatch. Header word: num_ifaces | bDeviceClass<<8 |
	// num_ep_mappings<<16 | num_out_mappings<<24.
	*(volatile uint32_t *)0x2017F080u =
		(uint32_t)desc.num_ifaces
		| ((uint32_t)desc.device_desc[4] << 8)   // bDeviceClass
		| ((uint32_t)num_ep_mappings << 16)
		| ((uint32_t)num_out_mappings << 24);
	for (uint8_t i = 0; i < 4; i++) {
		uint32_t a = 0, b = 0;
		if (i < desc.num_ifaces) {
			const captured_iface_t *f = &desc.ifaces[i];
			a = (uint32_t)f->iface_class
			  | ((uint32_t)f->iface_subclass << 8)
			  | ((uint32_t)f->iface_protocol << 16)
			  | ((uint32_t)f->interrupt_in_ep << 24);
			b = (uint32_t)f->hid_report_desc_len
			  | ((uint32_t)(f->has_hid_desc ? 1u : 0u) << 16)
			  | ((uint32_t)f->interrupt_in_maxpkt << 17);
		}
		*(volatile uint32_t *)(0x2017F084u + (uint32_t)i * 8u)     = a;
		*(volatile uint32_t *)(0x2017F084u + (uint32_t)i * 8u + 4) = b;
	}
	// BENCH DIAG: per-interface GET_REPORT_DESCRIPTOR outcome, one signed byte each
	// (if0..if3 in bytes 0..3). 127 = parser saw report-len 0 so fetch was skipped;
	// negative = fetch transfer failed with that code; >=0 = bytes fetched. Resolves
	// whether if2/if3 rdlen=0 is a parse miss or a fetch failure. Lives in the
	// write-once interface-table region (0x2017F0A4 free slot), which the oracle
	// reads reliably (unlike the per-loop counters subject to cross-core staleness).
	{
		uint32_t fr = 0;
		for (uint8_t i = 0; i < 4; i++) {
			uint8_t v = 0x7F;   // default: not present
			if (i < desc.num_ifaces)
				v = (uint8_t)desc.ifaces[i].report_fetch_ret;
			fr |= (uint32_t)v << (i * 8);
		}
		*(volatile uint32_t *)0x2017F0A4u = fr;
	}

	// --- USBFS device: replay descriptors, wait for the PC to configure --
	if (!usb_device_init(&desc)) {
		led_blink_forever(9, 80, 120);
	}
	dbg_stage(DBG_V5F_DEV_INIT);
	led_off();
	uint32_t dev_wait_start = millis();
	uint32_t dev_led_toggle = millis();
	extern volatile uint32_t usbd_dbg_irq, usbd_dbg_busrst, usbd_dbg_setup, usbd_dbg_lastst;
	extern volatile uint32_t usbd_dbg_alloc_before, usbd_dbg_alloc_after;
	extern volatile uint32_t usbd_dbg_lastsetup, usbd_dbg_lastsetup_len;
	extern volatile uint32_t usbd_dbg_stalls, usbd_dbg_configval;
	while (!usb_device_is_configured()) {
		usb_device_poll();
		usb_merge_drain_icc(); // keep the ICC mailbox drained during bring-up
		// BENCH DIAG: publish USBFS device-side activity so V3F can print it.
		// irq=0 => the device PHY sees NO bus activity (not wired / no pullup);
		// busrst climbing => host is driving the bus; setup climbing => enumerating.
		*(volatile uint32_t *)0x2017F028u = usbd_dbg_irq;
		*(volatile uint32_t *)0x2017F02Cu = usbd_dbg_busrst;
		*(volatile uint32_t *)0x2017F030u = usbd_dbg_setup;
		*(volatile uint32_t *)0x2017F034u = (USBFSD->BASE_CTRL) | (usbd_dbg_lastst << 8)
		                                    | (usbd_dbg_alloc_before << 16)
		                                    | (usbd_dbg_alloc_after << 17);
		// Live PHY/line state: MIS_ST (bit0=DEV_ATTACH bit1=DM_LEVEL bit3=BUS_RESET
		// bit7=SOF_PRES) + UDEV_CTRL (bit5=DP_PIN bit4=DM_PIN live levels). If
		// DEV_ATTACH/line pins are dead, the PHY sees no host; if SOF_PRES toggles,
		// the bus is alive. Packs MIS_ST | UDEV_CTRL<<8 | INT_EN<<16.
		*(volatile uint32_t *)0x2017F054u = (uint8_t)USBFSD->MIS_ST
		                                    | ((uint32_t)(uint8_t)USBFSD->UDEV_CTRL << 8)
		                                    | ((uint32_t)(uint8_t)USBFSD->INT_EN << 16);
		// Last SETUP request + STALL count + config-seen: pinpoints which control
		// request Windows sends that the device chokes on (where enum stalls).
		*(volatile uint32_t *)0x2017F058u = usbd_dbg_lastsetup;
		*(volatile uint32_t *)0x2017F05Cu = (usbd_dbg_lastsetup_len & 0xFFFF)
		                                    | (usbd_dbg_stalls << 16);
		// pack live is_configured into the high byte of the configval slot so we
		// can distinguish "SET_CONFIG was once seen" (low byte) from "configured
		// RIGHT NOW" (high byte) — a bus reset clears the live flag.
		*(volatile uint32_t *)0x2017F060u = usbd_dbg_configval
		                                    | (usb_device_is_configured() ? 0x10000u : 0u);
		if ((millis() - dev_led_toggle) >= 250) {
			led_toggle();
			dev_led_toggle = millis();
		}
		// BENCH: do NOT dead-end on the 30s timeout — keep the loop alive so the
		// register snapshot stays live for the oracle while we debug enumeration.
		(void)dev_wait_start;
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

	// Telemetry counters (ported from v2 main.c): reports successfully forwarded
	// to the PC vs. dropped (device-side EP busy). Streamed to V3F over the ICC
	// for the LED status ladder.
	uint32_t report_count = 0;
	uint32_t drop_count   = 0;
	uint32_t telem_tick   = millis();   // last time telemetry was emitted

	// BENCH DIAG: live relay report-flow counters, published to shared SRAM each
	// loop so the V3F oracle can show whether real IN reports arrive from the
	// device (host_in_reports) and whether they forward to the PC (report_count)
	// vs. drop (device EP busy / not configured). Distinguishes "no reports from
	// the real mouse" from "reports arrive but never reach Windows".
	uint32_t host_in_reports = 0;
	uint8_t  last_in_len     = 0;

	// BENCH DIAG: iteration-1 sub-stage tracer. V5F reaches 0x58 then appears to
	// hang on the first relay iteration (loop heartbeat stuck at 1, publish block
	// never runs). Write a distinct byte before each call so the LAST value at
	// 0x2017F0DC pinpoints which call hangs/faults: 0x10 top, 0x20 after drain,
	// 0x30 after device_poll, 0x40+m before EP poll m, 0x50+m after merge_report m,
	// 0x60 after EP loop, 0x70 after send_pending, 0x80 publish done.
	#define ITRC(v) (*(volatile uint32_t *)0x2017F0DCu = (v))

	while (1) {
		bool did_work = false;

		// BENCH DIAG: raw loop heartbeat at the very TOP of the loop, before any
		// work. If this advances but the publish block below stays frozen, the
		// loop body hangs mid-iteration; if this is frozen too, the loop never
		// iterates. (0x2017F0CC — distinct from the publish-block addresses.)
		*(volatile uint32_t *)0x2017F0CCu = ++loop_count;
		ITRC(0x10);

		// Drain injection from the V3F command core into the merge accumulators
		// (also runs scheduled click/key-release bookkeeping). Phase-5 cadence:
		// drain + merge + send every iteration, no PIT pacing.
		usb_merge_drain_icc();
		ITRC(0x20);

		// USB device EP completion (unblock EPs for next send).
		usb_device_poll();
		ITRC(0x30);

		// Host IN endpoints: capture -> merge -> forward to PC.
		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			ITRC(0x40 | m);
			uint8_t *rpt_ptr = NULL;
			ret = usb_host_interrupt_poll_zerocopy(ep_map[m].host_slot,
				&rpt_ptr, ep_map[m].maxpkt);
			if (ret > 0 && rpt_ptr) {
				did_work = true;
				host_in_reports++;          // bench: a real IN report arrived
				last_in_len = (uint8_t)ret; // bench: its length
				// Timestamp real mouse-report arrival from the free-running
				// 1 MHz TIM9 µs counter (single CNT read, monotonic). This is
				// the precise "a report just arrived" point; only the mouse
				// interface drives the adaptive feed rate. Mirrors v2 main.c
				// (humanize_record_arrival(gpt_profile_us()), line ~309).
				if (ep_map[m].iface_protocol == 2)
					humanize_record_arrival(timebase_v5f_us());
				usb_merge_report(ep_map[m].iface_protocol,
					rpt_ptr, (uint8_t)ret);
				ITRC(0x50 | m);
				// Count forwarded vs. dropped reports for telemetry (ports v2
				// main.c: send_report true -> report_count, false -> drop_count).
				if (usb_device_send_report(
					ep_map[m].dev_ep_num, rpt_ptr, (uint16_t)ret))
					report_count++;
				else
					drop_count++;
			}
		}
		ITRC(0x60);

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
		ITRC(0x70);

		// V5F->V3F telemetry was DROPPED (see icc.c): a lock-free SRAM ring needs
		// coherent cross-core reads of head+tail, which this part does not provide,
		// and the IPC MSG mailbox is fully used by the V3F->V5F injection path.
		// report_count/drop_count are still maintained locally for the optional
		// stage marker below and future use; they're just no longer streamed.
		(void)report_count; (void)drop_count; (void)telem_tick;

		// BENCH DIAG: publish live relay flow so the V3F oracle can show it at the
		// relay stage. host_in = IN reports seen from the real device; fwd =
		// forwarded to PC; drop = device EP busy/unconfigured; last_in_len in the
		// high byte of the fwd word.
		*(volatile uint32_t *)0x2017F0A8u = host_in_reports;
		*(volatile uint32_t *)0x2017F0ACu = report_count | ((uint32_t)last_in_len << 24);
		*(volatile uint32_t *)0x2017F0B0u = drop_count;
		// Host-side IN-poll outcome (why host_in stays 0): total polls, OK count,
		// last transact status + RX len + addr/ep. Packed:
		//   0x2017F0B8 = polls; 0x2017F0BC = oks;
		//   0x2017F0C0 = last_s | last_n<<8 | addr<<16 | ep<<24.
		{
			extern volatile uint32_t usbh_dbg_in_polls, usbh_dbg_in_oks;
			extern volatile uint8_t  usbh_dbg_in_last_s, usbh_dbg_in_last_n;
			extern volatile uint8_t  usbh_dbg_in_addr, usbh_dbg_in_ep;
			*(volatile uint32_t *)0x2017F0B8u = usbh_dbg_in_polls;
			*(volatile uint32_t *)0x2017F0BCu = usbh_dbg_in_oks;
			*(volatile uint32_t *)0x2017F0C0u = (uint32_t)usbh_dbg_in_last_s
				| ((uint32_t)usbh_dbg_in_last_n << 8)
				| ((uint32_t)usbh_dbg_in_addr << 16)
				| ((uint32_t)usbh_dbg_in_ep << 24);
		}
		// Per-slot IN-poll status (slot0/1/2 packed): byte = transact status, so
		// the oracle shows the MOUSE EP separately from the vendor EP. 0x2017F0D4
		// = s0|s1<<8|s2<<16; 0x2017F0D8 = n0|n1<<8|n2<<16 (last RX len per slot).
		{
			extern volatile uint8_t usbh_dbg_slot_s[], usbh_dbg_slot_n[];
			*(volatile uint32_t *)0x2017F0D4u = (uint32_t)usbh_dbg_slot_s[0]
				| ((uint32_t)usbh_dbg_slot_s[1] << 8)
				| ((uint32_t)usbh_dbg_slot_s[2] << 16);
			*(volatile uint32_t *)0x2017F0D8u = (uint32_t)usbh_dbg_slot_n[0]
				| ((uint32_t)usbh_dbg_slot_n[1] << 8)
				| ((uint32_t)usbh_dbg_slot_n[2] << 16);
		}
		// USBFS device ISR activity: irq count + "currently inside ISR" flag. If
		// the count explodes and/or in_isr stays 1 when V5F freezes, the wedge is
		// an interrupt storm / hang inside USBFS_IRQHandler (triggered by the PC
		// enumerating), not the host poll itself. 0x2017F0E0 region is the trap
		// witness; use 0x2017F0DC+? — pack at 0x2017F0CC is loop; use a fresh slot.
		{
			extern volatile uint32_t usbd_dbg_irq;
			extern volatile uint8_t  usbd_dbg_in_isr;
			*(volatile uint32_t *)0x2017F0F0u = usbd_dbg_irq;
			*(volatile uint32_t *)0x2017F0F4u = usbd_dbg_in_isr;
		}
		// V5F's own millis: if frozen, TIM4 isn't waking V5F (the relay-loop wfi
		// never returns and the loop stalls); if ticking, the wake path is alive.
		*(volatile uint32_t *)0x2017F0C4u = millis();
		// Live USBHS port status + detected device speed: resolves whether the real
		// device is LS/FS/HS (PORT_STATUS speed bits) — the root question behind
		// "interrupt-IN NAKs forever". Packs PORT_STATUS | speed<<16 (0=FS 1=LS 2=HS
		// per usb_host_device_speed) | CFG-low<<24.
		*(volatile uint32_t *)0x2017F0C8u = (uint32_t)(USBHSH->PORT_STATUS & 0xFFFF)
			| ((uint32_t)usb_host_device_speed() << 16)
			| ((uint32_t)((uint8_t)USBHSH->CFG) << 24);

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
					*(volatile uint32_t *)0x2017F0D0u = 0xDEAD0000u | disc_run;
				}
			} else {
				disc_run = 0;
			}
		}
	}
}
