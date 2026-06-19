// main_v3f.c — master: clocks, ICC, release V5F, then command/transport loop.
//
// V3F is the master core. Boot order: clocks -> millis timebase -> ICC rings
// (before V5F runs) -> wake V5F -> UART command link + protocol parser. The loop
// drains UART RX into the parser (act_* -> kmbox_cmd_* -> ICC records to V5F) and
// polls V5F status telemetry.
#include "ch32h417_port.h"
#include "board.h"
#include "led.h"
#include "icc.h"
#include "uart.h"
#include "kmbox_cmd.h"
#include "timebase.h"
#include "debug.h"
#include "display.h"
#include "humanize.h"
#include "temp.h"

// ── V5F stage diagnostic (bench bring-up) ───────────────────────────────────
// The running V5F disables SWJ (PB8/PB9) during USB init, so SWD/SDI debug NAKs
// every probe while it runs and V5F's stage marker at 0x2017F000 is unreadable
// over wlink. V3F owns USART1 (the WCH-LinkE VCP) and 0x2017F000 is in the shared
// RAM window mapped in both images, so V3F reads V5F's stage byte from shared SRAM
// and prints it as ASCII over the command UART.
//
// Default-on for bench builds. Disable for production (where ASCII lines would
// corrupt the binary protocol) via `make EXTRADEF=-DV5F_STAGE_DIAG_OFF`.
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
    // usb_host_init() sub-stages (usb_host.c 0x70..0x76).
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
    // V5F relay telemetry over IPC CH1 status bits (not shared SRAM). tlm =
    // seq<<6|code: the seq half rolls every relay iteration, so a changing tlm_hb
    // proves V5F is alive and a frozen tlm_hb with a stuck tlm_code names where it
    // wedged.
    uint8_t  tlm      = icc_telem_read_v3f();
    uint8_t  tlm_hb   = (uint8_t)(tlm >> 6);          // 2-bit rolling heartbeat
    uint8_t  tlm_code = (uint8_t)(tlm & 0x3F);        // 6-bit relay stage code
    // Print on a fixed ~1s cadence (or a boot-stage change), not per seq change
    // (that would flood the UART). Snapshot the heartbeat once per print window and
    // compare across windows; the delta is the alive/wedged signal.
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
    // hb_alive=1: V5F loop heartbeat advanced since last print (running). =0:
    // frozen (wedged), rly names where.
    diag_puts(" hb=");     diag_puts(hb_alive ? "ALIVE" : "FROZEN");
    diag_puts(" rly=0x");  diag_put_hex8(tlm_code);
    diag_puts(" icc_magic=");
    diag_puts(magic == ICC_MAGIC ? "OK" : "BAD");
    // Trap witness (any vector): a trap/NMI/ecall stamps a witness at 0x2017F0E0
    // before spinning. Printed unconditionally since a trap can freeze at any
    // stage. 0x40A11EDx => trap class x; mcause/mepc follow. Absent => true hang.
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
    // On a V5F trap (stage 0x8x) the handler stamped mcause/mepc in the two words
    // after the marker; surface them so the fault is self-describing.
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
    // Rendezvous is the ICC magic-spin (V3F sets magic in icc_init_v3f before this
    // wake; V5F spins on it in icc_init_v5f). V3F does not block on HSEM, so do not
    // enable the HSEM interrupt: there is no HSEM_Handler to clear the flag (only a
    // weak spin-stub), and V5F's HSEM_ReleaseOneSem would set V3F's HSEM-pending
    // bit, trapping V3F in the stub. HSEM stays poll-only on the V5F side.

    uart_init(CMD_BAUD_DEFAULT);
    kmbox_cmd_init();                   // act_init + proto_init + bind tx
    led_heartbeat_start();

    temp_init();        // ADC1 on HCLK — independent of USBHS PLL (V5F relay-safe)
#ifdef DISPLAY_PRESENT
    display_init();
#endif
    display_status_t g_disp = { .state = DISP_STATE_BOOT };
    uint32_t disp_render_tick = millis();
    (void)disp_render_tick;

#if defined(V5F_STAGE_DIAG)
    // One-shot banner so a fresh terminal sees build + baud and confirms V3F is
    // alive on the link before any V5F line appears.
    diag_puts("\r\n=== Hurra-v3 V5F-stage diag (V3F alive) baud=");
    diag_put_u32(CMD_BAUD_DEFAULT);
    diag_puts(" ===\r\n");
    uint8_t  diag_last_stage = 0xFF;    // force first read to print as <NEW>
    uint32_t diag_hb_tick    = millis();
#endif

    // V5F->V3F bulk telemetry (report/drop counts, status flags) is not carried
    // (see icc.c). V5F liveness rides the coherent IPC CH1 stage telemetry
    // (icc_telem_read_v3f, the hb=ALIVE oracle in diag_v5f_stage_poll), so the LED
    // ladder has no "reports-flowing" tier and keys off local UART activity only.

    // --- LED status ladder snapshots --------------------------------------
    uint32_t led_status_tick = millis();
    uint32_t led_err_snapshot = 0;      // last-sampled UART error total
    uint32_t led_rx_snapshot  = 0;      // last-sampled UART rx byte count
    uint16_t led_centihz      = 0;      // current heartbeat rate (0 = unset)

    for (;;) {
        kmbox_cmd_poll();               // UART -> proto -> act -> ICC (local FIFO)

        // Pump queued injection records from the local FIFO into the IPC MSG
        // mailbox (the SRAM ring is not readable by V5F). Drain a burst here rather
        // than one-per-main-loop, since the mailbox frees as fast as V5F drains it.
        while (icc_pump_to_v5f()) { }

        // Pull V5F status telemetry every iteration (cheap MMIO read); render at
        // ~4 Hz. Display is non-essential and must never gate the relay or command
        // link. Window-based liveness: no heartbeat-seq advance across a render
        // interval shows NOSIGNAL.
        static bool s_seen_advance;
        if (icc_status_poll_v3f(&g_disp)) s_seen_advance = true;
        uint32_t dnow = millis();
        if ((dnow - disp_render_tick) >= 250) {
            disp_render_tick = dnow;
            g_disp.uptime_s = dnow / 1000u;
            // V3F-local stats: fill every render (even during NOSIGNAL) so the
            // bottom display block stays live if the relay core wedges.
            g_disp.cmd_rx    = uart_rx_byte_count();
            uint32_t cerr    = uart_overrun() + uart_framing() + uart_noise();
            g_disp.cmd_err   = (uint16_t)(cerr > 0xFFFFu ? 0xFFFFu : cerr);
            g_disp.human_lvl = humanize_get_level();
            uint32_t im      = kmbox_cmd_inj_mouse_count();
            uint32_t ik      = kmbox_cmd_inj_kbd_count();
            g_disp.inj_m     = (uint16_t)(im > 0xFFFFu ? 0xFFFFu : im);
            g_disp.inj_k     = (uint16_t)(ik > 0xFFFFu ? 0xFFFFu : ik);
            g_disp.temp_c    = temp_read_c();   // single ADC conversion, ~µs, V3F-local
#ifndef DISPLAY_PRESENT
            // Device board (no TFT): ship local temp to the device V5F to fold into
            // the SPI return telemetry the host TFT renders.
            {
                icc_record_t tr = { .tag = ICC_TAG_DEV_TEMP };
                tr.b[0] = (uint8_t)g_disp.temp_c;
                (void)icc_send_to_v5f(&tr);
            }
#endif
            if (!s_seen_advance && g_disp.state != DISP_STATE_BOOT)
                g_disp.state = DISP_STATE_NOSIGNAL;
            s_seen_advance = false;
#ifdef DISPLAY_PRESENT
            display_render(&g_disp);
#endif
        }

        // --- LED status ladder ------------------------------------------
        // Sampled every 100 ms; the heartbeat blinks on its own and we rewrite its
        // rate (glitch-free) only when the tier changes.
        uint32_t now = millis();
        if ((now - led_status_tick) >= 100) {
            led_status_tick = now;

            uint32_t rx  = uart_rx_byte_count();
            uint32_t err = uart_overrun() + uart_framing() + uart_noise();
            // Ladder reflects local UART state: ERROR > ACTIVE (rx moving) > IDLE.
            // V5F relay liveness is observable separately via the hb=ALIVE oracle
            // (diag_v5f_stage_poll).
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
        diag_v5f_stage_poll(&diag_last_stage, &diag_hb_tick);
        // Dump the raw reverse status word + decoded g_disp fields at ~1 Hz so IPC
        // bits [16:31] are visible over UART without a logic analyzer.
        {
            static uint32_t s_sts_tick;
            uint32_t _now = millis();
            if ((_now - s_sts_tick) >= 1000) {
                s_sts_tick = _now;
                uint16_t raw = icc_status_read_raw_v3f();
                diag_puts("STS16=0x");
                diag_put_hex8((uint8_t)(raw >> 8));
                diag_put_hex8((uint8_t)(raw));
                diag_puts(" st=");   diag_put_u32(g_disp.state);
                diag_puts(" vid=");  diag_put_hex8((uint8_t)(g_disp.vid >> 8));
                                     diag_put_hex8((uint8_t)(g_disp.vid));
                diag_puts(" pid=");  diag_put_hex8((uint8_t)(g_disp.pid >> 8));
                                     diag_put_hex8((uint8_t)(g_disp.pid));
                diag_puts(" rps=");  diag_put_u32(g_disp.reports_per_sec);
                diag_puts("\r\n");
            }
        }
#endif
    }
}
