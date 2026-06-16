// main_v3f.c — master: clocks, ICC, release V5F, then command/transport loop.
//
// V3F is the master core. Boot order: clocks -> millis timebase -> ICC rings
// (before V5F runs) -> wake V5F -> UART command link + protocol parser. The
// loop drains UART RX into the parser (which calls act_* -> kmbox_cmd_* -> ICC
// records to V5F) and drains V5F telemetry (ignored until Phase 7).
#include "ch32h417_port.h"
#include "board.h"
#include "led.h"
#include "icc.h"
#include "uart.h"
#include "kmbox_cmd.h"
#include "timebase.h"
#include "debug.h"

// ── V5F stage diagnostic (bench bring-up) ───────────────────────────────────
// The running V5F disables SWJ (PB8/PB9) during USB init, so SWD/SDI debug NAKs
// every probe command (`0x55`) while it runs — we cannot read V5F's stage marker
// at 0x2017F000 over wlink. But V3F is healthy, owns USART1 (the WCH-LinkE VCP),
// and 0x2017F000 lives in the shared RAM window mapped in BOTH images. So V3F
// reads V5F's stage byte directly out of shared SRAM and prints it as ASCII over
// the command UART — a probe-free, eyeball-free oracle that mirrors how WCH's own
// reference does bring-up (printf over USART). It replaces counting the PC3
// blink superposition by eye.
//
// Default-ON for this bench build. Build without it (production / when a real
// Hurra host app drives the link, since ASCII lines would corrupt the binary
// protocol) via `make EXTRADEF=-DV5F_STAGE_DIAG_OFF`.
#if !defined(V5F_STAGE_DIAG_OFF)
#define V5F_STAGE_DIAG 1
#endif

#if defined(V5F_STAGE_DIAG)
static void diag_puts(const char *s)
{
    uint16_t n = 0;
    while (s[n]) n++;
    (void)uart_tx_write((const uint8_t *)s, n);
}

static void diag_put_hex8(uint8_t v)
{
    static const char hx[] = "0123456789ABCDEF";
    uint8_t buf[2] = { (uint8_t)hx[(v >> 4) & 0xF], (uint8_t)hx[v & 0xF] };
    (void)uart_tx_write(buf, 2);
}

static void diag_put_u32(uint32_t v)
{
    char tmp[10];
    int i = 0;
    if (v == 0) { diag_puts("0"); return; }
    while (v && i < 10) { tmp[i++] = (char)('0' + (v % 10)); v /= 10; }
    char out[11];
    int j = 0;
    while (i) out[j++] = tmp[--i];
    out[j] = '\0';
    diag_puts(out);
}

// Map a V5F stage byte (icc.h DBG_V5F_*) to a human label. Unknown -> "?".
static const char *diag_stage_name(uint8_t s)
{
    switch (s) {
    case DBG_V5F_BOOT:          return "BOOT(pre-rendezvous)";
    case DBG_V5F_TIMEBASE:      return "TIMEBASE";
    case DBG_V5F_HUMANIZE:      return "HUMANIZE";
    case DBG_V5F_MERGE_INIT:    return "MERGE_INIT";
    case DBG_V5F_LED_INIT:      return "LED_INIT";
    case DBG_V5F_ICC_MAGIC:     return "ICC_MAGIC(saw magic)";
    case DBG_V5F_HSEM_DONE:     return "HSEM_DONE";
    case DBG_V5F_ICC_READY:     return "ICC_READY(IPC armed)";
    case DBG_V5F_HOST_INIT:     return "HOST_INIT(USBHS up)";
    case DBG_V5F_HOST_WAITING:  return "HOST_WAITING(no device)";
    case DBG_V5F_DEV_CONNECTED: return "DEV_CONNECTED";
    case DBG_V5F_DESC_OK:       return "DESC_OK";
    case DBG_V5F_DEV_INIT:      return "DEV_INIT(cloning to PC)";
    case DBG_V5F_RELAY:         return "RELAY(hot path)";
    // usb_host_init() fine-grained sub-stages (usb_host.c 0x70..0x76).
    case 0x70:                  return "HOST_RCC_ENTER";
    case 0x71:                  return "HOST_PLL_WAIT(spinning on USBHS PLL)";
    case 0x72:                  return "HOST_PLL_RDY";
    case 0x73:                  return "HOST_UTMI_ON";
    case 0x74:                  return "HOST_CLK_ON";
    case 0x75:                  return "HOST_MMIO(before USBHSH regs)";
    case 0x76:                  return "HOST_CFG_DONE";
    default:
        if ((s & 0xF0) == DBG_V5F_TRAP_BASE) return "*** V5F TRAP ***";
        return "?";
    }
}

// Read V5F's live stage marker + the ICC magic, print on change or every ~1s.
// Called from the V3F main loop. `*last`/`*hb` are caller-owned persistent state.
static void diag_v5f_stage_poll(uint8_t *last_stage, uint32_t *hb_tick)
{
    uint8_t  stage = *(volatile uint8_t  *)DBG_STAGE_ADDR;
    uint32_t magic = *(volatile uint32_t *)0x20178000u;   // icc_shared_t.magic
    uint32_t now   = millis();
    // Coherent V5F relay telemetry over IPC CH1 status bits (NOT shared SRAM, which
    // V5F can no longer write in the hot path — see icc.c:5-16). tlm = seq<<6|code:
    // the seq half rolls every relay iteration so a CHANGING tlm_hb proves V5F is
    // alive; a FROZEN tlm_hb with a stuck tlm_code names exactly where V5F wedged.
    uint8_t  tlm      = icc_telem_read_v3f();
    uint8_t  tlm_hb   = (uint8_t)(tlm >> 6);          // 2-bit rolling heartbeat
    uint8_t  tlm_code = (uint8_t)(tlm & 0x3F);        // 6-bit relay stage code
    // Snapshot the heartbeat ONCE per ~1s print window and compare ACROSS windows:
    // the seq rolls every relay iteration, so we must NOT trigger a print on every
    // change (that floods the UART). We print on a fixed ~1s cadence (or on a real
    // boot-stage change) and SHOW whether hb advanced since the previous print —
    // that delta is the alive/wedged signal, not a per-change trigger.
    static uint8_t s_prev_hb = 0xFF;
    bool changed   = (stage != *last_stage);
    if (!changed && (now - *hb_tick) < 1000) return;
    bool hb_alive  = (tlm_hb != s_prev_hb);           // advanced since last print
    s_prev_hb = tlm_hb;

    *last_stage = stage;
    *hb_tick    = now;
    diag_puts("[t=");      diag_put_u32(now);  diag_puts("ms] V5F=0x");
    diag_put_hex8(stage);  diag_puts(" ");     diag_puts(diag_stage_name(stage));
    diag_puts(changed ? " <NEW>" : " (still)");
    // Relay telemetry: hb_alive=1 means the V5F loop heartbeat advanced since the
    // last print (V5F running); =0 means FROZEN (V5F wedged) and rly names where.
    diag_puts(" hb=");     diag_puts(hb_alive ? "ALIVE" : "FROZEN");
    diag_puts(" rly=0x");  diag_put_hex8(tlm_code);
    diag_puts(" icc_magic=");
    diag_puts(magic == ICC_MAGIC ? "OK" : "BAD");
    // Trap witness (any vector): if V5F took a trap/NMI/ecall it stamped a witness
    // at 0x2017F0E0 before spinning. Printed UNCONDITIONALLY (a trap can freeze at
    // any stage). 0x40A11EDx → trap class x; mcause/mepc follow. Absent => true hang.
    {
        uint32_t w = *(volatile uint32_t *)0x2017F0E0u;
        if ((w & 0xFFFFFFF0u) == 0x40A11ED0u) {
            diag_puts(" TRAP=");      diag_put_hex8((uint8_t)(w & 0x0F));
            diag_puts(" mcause=");    diag_put_u32(*(volatile uint32_t *)0x2017F0E4u);
            diag_puts(" mepc=0x");
            uint32_t pc = *(volatile uint32_t *)0x2017F0E8u;
            diag_put_hex8((uint8_t)(pc >> 24)); diag_put_hex8((uint8_t)(pc >> 16));
            diag_put_hex8((uint8_t)(pc >> 8));  diag_put_hex8((uint8_t)pc);
        } else if ((w & 0xFFFF0000u) >= 0x40A20000u && (w & 0xFFFF0000u) <= 0x40A40000u) {
            // transact phase tracer: 0x40A2=loop-top 0x40A3=pre-INT-wait 0x40A4=post-wait
            diag_puts(" tx=0x"); diag_put_hex8((uint8_t)(w >> 24)); diag_put_hex8((uint8_t)(w >> 16));
            diag_puts(":"); diag_put_u32(w & 0xFFFF);
        }
    }
    // On a V5F trap (stage 0x8x) the handler also stamped mcause/mepc in the two
    // words after the marker — surface them so the fault is fully self-describing.
    if ((stage & 0xF0) == DBG_V5F_TRAP_BASE) {
        uint32_t mcause = *(volatile uint32_t *)(DBG_STAGE_ADDR + 4);
        uint32_t mepc   = *(volatile uint32_t *)(DBG_STAGE_ADDR + 8);
        diag_puts(" mcause=");  diag_put_u32(mcause);
        diag_puts(" mepc=0x");  diag_put_hex8((uint8_t)(mepc >> 24));
        diag_put_hex8((uint8_t)(mepc >> 16)); diag_put_hex8((uint8_t)(mepc >> 8));
        diag_put_hex8((uint8_t)mepc);
    }
    diag_puts("\r\n");
}
#endif /* V5F_STAGE_DIAG */

int main(void)
{
    SystemInit();
    SystemAndCoreClockUpdate();
    Delay_Init();
    led_init();
    timebase_init(SystemCoreClock);     // start millis() before parsers run

    icc_init_v3f();                     // set up shared rings BEFORE releasing V5F
    NVIC_WakeUp_V5F(Core_V5F_StartAddr);
    // Rendezvous barrier is the ICC magic-spin (V3F sets magic in icc_init_v3f
    // before this wake; V5F spins on it in icc_init_v5f). V3F does NOT block on
    // HSEM here, so do NOT enable the HSEM interrupt: there is no HSEM_Handler
    // that clears the flag (only the weak spin-stub), and V5F's HSEM_ReleaseOneSem
    // would set V3F's HSEM-pending bit — an enabled IRQ would trap V3F in the
    // stub forever. HSEM stays poll-only on the V5F take/release side.

    uart_init(CMD_BAUD_DEFAULT);
    kmbox_cmd_init();                   // act_init + proto_init + bind tx
    led_heartbeat_start();

#if defined(V5F_STAGE_DIAG)
    // One-shot banner so a freshly-opened terminal knows the build + baud and can
    // confirm V3F itself is alive on the link before any V5F line appears.
    diag_puts("\r\n=== Hurra-v3 V5F-stage diag (V3F alive) baud=");
    diag_put_u32(CMD_BAUD_DEFAULT);
    diag_puts(" ===\r\n");
    uint8_t  diag_last_stage = 0xFF;    // force first read to print as <NEW>
    uint32_t diag_hb_tick    = millis();
#endif

    // V5F->V3F bulk telemetry (report/drop counts, status flags) is intentionally
    // NOT carried: it would need a shared-SRAM ring that V3F cannot read coherently
    // (V5F's DTCM reads back stale on V3F — see icc.c), and the 4 IPC MSG registers
    // are fully consumed by the V3F->V5F injection mailbox. V5F liveness instead
    // rides the coherent IPC CH1 stage telemetry (icc_telem_read_v3f, the hb=ALIVE
    // oracle in diag_v5f_stage_poll). So the LED ladder's "locked/reports-flowing"
    // tier is gone; the ladder keys off LOCAL UART activity only.

    // --- LED status ladder snapshots (ported from v2 main.c) -------------
    uint32_t led_status_tick = millis();
    uint32_t led_err_snapshot = 0;      // last-sampled UART error total
    uint32_t led_rx_snapshot  = 0;      // last-sampled UART rx byte count
    uint16_t led_centihz      = 0;      // current heartbeat rate (0 = unset)

    for (;;) {
        kmbox_cmd_poll();               // UART -> proto -> act -> ICC (local FIFO)

        // Pump queued V3F->V5F injection records from the local FIFO into the
        // coherent IPC MSG mailbox (the SRAM ring is not readable by V5F). One
        // record per free mailbox; loop here to drain a burst quickly rather than
        // one-per-main-loop, since the mailbox frees as fast as V5F drains it.
        while (icc_pump_to_v5f()) { /* keep pumping while mailbox frees */ }

        // --- LED status ladder (ports v2 main.c, MINUS the overtemp tier) -
        // Sampled every 100 ms; the heartbeat keeps blinking on its own and we
        // only rewrite its rate (glitch-free) when the tier actually changes.
        uint32_t now = millis();
        if ((now - led_status_tick) >= 100) {
            led_status_tick = now;

            uint32_t rx  = uart_rx_byte_count();
            uint32_t err = uart_overrun() + uart_framing() + uart_noise();
            // The v2 "LOCKED" tier needed V5F report-count telemetry, which is no
            // longer carried (see above). The ladder now reflects LOCAL UART state:
            // ERROR > ACTIVE (rx moving) > IDLE. V5F relay liveness is observable
            // separately via the hb=ALIVE oracle (diag_v5f_stage_poll).
            uint16_t centihz;
            if (err != led_err_snapshot) centihz = 600; // ERROR   ~6 Hz
            else if (rx != led_rx_snapshot) centihz = 200; // ACTIVE ~2 Hz
            else                         centihz = 50;  // IDLE    ~0.5 Hz

            led_err_snapshot = err;
            led_rx_snapshot  = rx;
            if (centihz != led_centihz) {
                led_centihz = centihz;
                led_heartbeat_set_rate(centihz);
            }
        }

#if defined(V5F_STAGE_DIAG)
        // Print V5F's live boot stage (read from shared SRAM) on change or ~1 Hz.
        // This is the probe-free oracle that replaces eyeballing PC3's blink pile.
        diag_v5f_stage_poll(&diag_last_stage, &diag_hb_tick);
#endif
    }
}
