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
    bool changed   = (stage != *last_stage);
    if (!changed && (now - *hb_tick) < 1000) return;

    *last_stage = stage;
    *hb_tick    = now;
    diag_puts("[t=");      diag_put_u32(now);  diag_puts("ms] V5F=0x");
    diag_put_hex8(stage);  diag_puts(" ");     diag_puts(diag_stage_name(stage));
    diag_puts(changed ? " <NEW>" : " (still)");
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
    // At DEV_INIT (0x57, waiting for the PC to enumerate the cloned USBFS device),
    // show USBFS device-side activity. irq=0 => the device PHY sees no bus traffic.
    if (stage == DBG_V5F_DEV_INIT) {
        uint32_t base_st = *(volatile uint32_t *)0x2017F034u;
        diag_puts(" usbd_irq=");  diag_put_u32(*(volatile uint32_t *)0x2017F028u);
        diag_puts(" busrst=");    diag_put_u32(*(volatile uint32_t *)0x2017F02Cu);
        diag_puts(" setup=");     diag_put_u32(*(volatile uint32_t *)0x2017F030u);
        diag_puts(" BASE_CTRL=0x"); diag_put_hex8((uint8_t)base_st);
        diag_puts(" lastst=0x");  diag_put_hex8((uint8_t)(base_st >> 8));
        diag_puts(" alloc(b/a)="); diag_put_u32((base_st >> 16) & 1);
        diag_puts("/");            diag_put_u32((base_st >> 17) & 1);
        // Live USBFS PHY/line state: MIS_ST | UDEV_CTRL<<8 | INT_EN<<16.
        uint32_t phy = *(volatile uint32_t *)0x2017F054u;
        diag_puts(" MIS_ST=0x");   diag_put_hex8((uint8_t)phy);
        diag_puts(" UDEV=0x");     diag_put_hex8((uint8_t)(phy >> 8));
        diag_puts(" INT_EN=0x");   diag_put_hex8((uint8_t)(phy >> 16));
        // Last SETUP (req|type|wValue), wLength, STALL count, SET_CONFIG seen.
        uint32_t ls = *(volatile uint32_t *)0x2017F058u;
        uint32_t lc = *(volatile uint32_t *)0x2017F05Cu;
        diag_puts(" lastSETUP req=0x"); diag_put_hex8((uint8_t)(ls >> 24));
        diag_puts(" type=0x");          diag_put_hex8((uint8_t)(ls >> 16));
        diag_puts(" wVal=0x");          diag_put_hex8((uint8_t)(ls >> 8));
        diag_put_hex8((uint8_t)ls);
        diag_puts(" wLen=");            diag_put_u32(lc & 0xFFFF);
        diag_puts(" stalls=");          diag_put_u32(lc >> 16);
        uint32_t cfgw = *(volatile uint32_t *)0x2017F060u;
        diag_puts(" cfgSeen=0x");        diag_put_hex8((uint8_t)cfgw);
        diag_puts(" cfgLive=");          diag_put_u32((cfgw >> 16) & 1);
    }
    // At RELAY (0x58): show what we cloned + whether reports flow. The captured
    // interface table (0x2017F080..) tells us if the device is composite and
    // whether the mouse interface's HID report descriptor was actually captured
    // (rdlen>0); the flow counters (0x2017F0A8..) tell us if real IN reports
    // arrive and forward to the PC. This pinpoints "named right but shows as a
    // keyboard / no reports".
    if (stage == DBG_V5F_RELAY) {
        uint32_t hdr = *(volatile uint32_t *)0x2017F080u;
        uint8_t nif = (uint8_t)hdr;
        diag_puts(" nif=");      diag_put_u32(nif);
        diag_puts(" devCls=0x"); diag_put_hex8((uint8_t)(hdr >> 8));
        diag_puts(" inEPs=");    diag_put_u32((hdr >> 16) & 0xFF);
        diag_puts(" outEPs=");   diag_put_u32((hdr >> 24) & 0xFF);
        if (nif > 4) nif = 4;
        for (uint8_t i = 0; i < nif; i++) {
            uint32_t a = *(volatile uint32_t *)(0x2017F084u + (uint32_t)i * 8u);
            uint32_t b = *(volatile uint32_t *)(0x2017F084u + (uint32_t)i * 8u + 4);
            diag_puts(" if"); diag_put_u32(i);
            diag_puts("[cls=0x");   diag_put_hex8((uint8_t)a);
            diag_puts(" sub=0x");   diag_put_hex8((uint8_t)(a >> 8));
            diag_puts(" prot=0x");  diag_put_hex8((uint8_t)(a >> 16));
            diag_puts(" inEP=0x");  diag_put_hex8((uint8_t)(a >> 24));
            diag_puts(" rdlen=");   diag_put_u32(b & 0xFFFF);
            diag_puts(" hid=");     diag_put_u32((b >> 16) & 1);
            diag_puts("]");
        }
        diag_puts(" host_in=");  diag_put_u32(*(volatile uint32_t *)0x2017F0A8u);
        uint32_t fwd = *(volatile uint32_t *)0x2017F0ACu;
        diag_puts(" fwd=");      diag_put_u32(fwd & 0xFFFFFF);
        diag_puts(" lastLen=");  diag_put_u32(fwd >> 24);
        diag_puts(" drop=");     diag_put_u32(*(volatile uint32_t *)0x2017F0B0u);
        // Host-side IN-poll outcome: why host_in stays 0.
        diag_puts(" inPolls="); diag_put_u32(*(volatile uint32_t *)0x2017F0B8u);
        diag_puts(" inOks=");   diag_put_u32(*(volatile uint32_t *)0x2017F0BCu);
        uint32_t ip = *(volatile uint32_t *)0x2017F0C0u;
        diag_puts(" inS=0x");   diag_put_hex8((uint8_t)ip);
        diag_puts(" inN=");     diag_put_u32((ip >> 8) & 0xFF);
        diag_puts(" inAddr=");  diag_put_u32((ip >> 16) & 0xFF);
        diag_puts(" inEP=");    diag_put_u32((ip >> 24) & 0xFF);
        diag_puts(" v5ms=");    diag_put_u32(*(volatile uint32_t *)0x2017F0C4u);
        uint32_t ps = *(volatile uint32_t *)0x2017F0C8u;
        diag_puts(" portSt=0x"); diag_put_hex8((uint8_t)(ps >> 8));
        diag_put_hex8((uint8_t)ps);
        diag_puts(" spd=");      diag_put_u32((ps >> 16) & 0xFF);  // 0=FS 1=LS 2=HS
        diag_puts(" cfg=0x");    diag_put_hex8((uint8_t)(ps >> 24));
        diag_puts(" loop=");     diag_put_u32(*(volatile uint32_t *)0x2017F0CCu);
        uint32_t ss = *(volatile uint32_t *)0x2017F0D4u;
        uint32_t sn = *(volatile uint32_t *)0x2017F0D8u;
        diag_puts(" s0=0x"); diag_put_hex8((uint8_t)ss);
        diag_puts("/n"); diag_put_u32((uint8_t)sn);
        diag_puts(" s1=0x"); diag_put_hex8((uint8_t)(ss >> 8));
        diag_puts("/n"); diag_put_u32((uint8_t)(sn >> 8));
        diag_puts(" s2=0x"); diag_put_hex8((uint8_t)(ss >> 16));
        diag_puts("/n"); diag_put_u32((uint8_t)(sn >> 16));
        diag_puts(" itrc=0x"); diag_put_hex8((uint8_t)*(volatile uint32_t *)0x2017F0DCu);
        diag_puts(" usbdIrq="); diag_put_u32(*(volatile uint32_t *)0x2017F0F0u);
        diag_puts(" inISR=");   diag_put_u32(*(volatile uint32_t *)0x2017F0F4u);
    }
    // capture_descriptors() failure detail (stage 0x9F): which control-transfer
    // step failed (p@0x2017F048) and its return code (p@0x2017F04C).
    if (stage == 0x9F || stage == 0x92) {
        diag_puts(" cap_step="); diag_put_u32(*(volatile uint32_t *)0x2017F048u);
        diag_puts(" cap_ret=");  diag_put_u32(*(volatile uint32_t *)0x2017F04Cu);
        // SETUP-stage failure detail: s | INT_FLAG<<8 | INT_ST<<16 | PORT_STATUS<<24.
        // s: 0x20=TRANSFER(NAK/no-resp exhausted) 0xFE=UNKNOWN(transfer-done never
        // set=no response) 0x15=CONNECT 0x16=DISCON.
        uint32_t d = *(volatile uint32_t *)0x2017F050u;
        diag_puts(" setup_s=0x");      diag_put_hex8((uint8_t)d);
        diag_puts(" INT_FLAG=0x");     diag_put_hex8((uint8_t)(d >> 8));
        diag_puts(" INT_ST=0x");       diag_put_hex8((uint8_t)(d >> 16));
        diag_puts(" PORT_STATUS=0x");  diag_put_hex8((uint8_t)(d >> 24));
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
    // While V5F waits in HOST_WAITING, print its live USBHS port-register snapshot
    // so "device attached but CONNECT never sets" becomes visible: PORT_STATUS
    // line-state/speed bits show whether the PHY sees the device on the bus.
    if (stage == DBG_V5F_HOST_WAITING) {
        volatile uint32_t *p = (volatile uint32_t *)DBG_USBHS_REGS_ADDR;
        // Print raw (no magic gate) so the snapshot state itself is ground truth:
        // magic=0 => V5F never wrote it (loop body not running); n frozen => loop
        // not iterating; n climbing => loop alive, read PORT_STATUS for PHY state.
        diag_puts(" | magic=");                  diag_put_u32(p[0] == DBG_USBHS_REGS_MAGIC);
        diag_puts(" CFG=0x");                     diag_put_hex8((uint8_t)p[1]);
        diag_puts(" PORT_CFG=0x");                diag_put_hex8((uint8_t)p[2]);
        diag_puts(" PORT_STATUS=0x");             diag_put_hex8((uint8_t)(p[3] >> 8));
        diag_put_hex8((uint8_t)p[3]);
        diag_puts(" CHG=0x");                     diag_put_hex8((uint8_t)p[4]);
        diag_puts(" CTRL=0x");                    diag_put_hex8((uint8_t)(p[5] >> 8));
        diag_put_hex8((uint8_t)p[5]);
        diag_puts(" n=");                         diag_put_u32(p[6]);
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

    // --- Telemetry state (decoded from V5F over the ICC) -----------------
    uint32_t telem_report_count = 0;    // reports V5F forwarded to the PC
    uint32_t telem_drop_count   = 0;    // reports V5F dropped (EP busy)
    uint8_t  telem_status       = 0;    // bit0 = USB configured, bit1 = host connected

    // --- LED status ladder snapshots (ported from v2 main.c) -------------
    uint32_t led_status_tick = millis();
    uint32_t led_err_snapshot = 0;      // last-sampled UART error total
    uint32_t led_rx_snapshot  = 0;      // last-sampled UART rx byte count
    uint32_t led_rpt_snapshot = 0;      // last-sampled telemetry report_count
    uint16_t led_centihz      = 0;      // current heartbeat rate (0 = unset)

    for (;;) {
        kmbox_cmd_poll();               // UART -> proto -> act -> ICC (local FIFO)

        // Pump queued V3F->V5F injection records from the local FIFO into the
        // coherent IPC MSG mailbox (the SRAM ring is not readable by V5F). One
        // record per free mailbox; loop here to drain a burst quickly rather than
        // one-per-main-loop, since the mailbox frees as fast as V5F drains it.
        while (icc_pump_to_v5f()) { /* keep pumping while mailbox frees */ }

        // Drain V5F telemetry into local state. TELEM_COUNTS carries
        // report_count (LE u32, b[0..3]) and drop_count (b[4..7]);
        // TELEM_STATUS carries health flags in b[0].
        icc_record_t t;
        while (icc_recv_from_v5f(&t)) {
            switch (t.tag) {
            case ICC_TAG_TELEM_COUNTS:
                telem_report_count = (uint32_t)t.b[0]        |
                                     ((uint32_t)t.b[1] <<  8) |
                                     ((uint32_t)t.b[2] << 16) |
                                     ((uint32_t)t.b[3] << 24);
                telem_drop_count   = (uint32_t)t.b[4]        |
                                     ((uint32_t)t.b[5] <<  8) |
                                     ((uint32_t)t.b[6] << 16) |
                                     ((uint32_t)t.b[7] << 24);
                break;
            case ICC_TAG_TELEM_STATUS:
                telem_status = t.b[0];
                break;
            default:
                break;
            }
        }
        (void)telem_drop_count;         // tracked for future use / introspection

        // --- LED status ladder (ports v2 main.c, MINUS the overtemp tier) -
        // Sampled every 100 ms; the heartbeat keeps blinking on its own and we
        // only rewrite its rate (glitch-free) when the tier actually changes.
        uint32_t now = millis();
        if ((now - led_status_tick) >= 100) {
            led_status_tick = now;

            uint32_t rx  = uart_rx_byte_count();
            uint32_t err = uart_overrun() + uart_framing() + uart_noise();
            // LOCKED substitute for v2's pit_locked: v3 has no PIT/adaptive
            // feed-rate convergence to report, so "locked/healthy steady state"
            // is defined as the USB relay being configured (PC enumerated us)
            // AND real reports flowing — report_count advanced since the last
            // 100 ms sample. Reports moving = the MITM is live and healthy.
            bool configured = (telem_status & 0x01) != 0;
            bool reports_flowing = (telem_report_count != led_rpt_snapshot);
            bool locked = configured && reports_flowing;

            uint16_t centihz;
            if (err != led_err_snapshot) centihz = 600; // ERROR     ~6 Hz
            else if (rx != led_rx_snapshot) centihz = 200; // ACTIVE   ~2 Hz
            else if (locked)             centihz = 100; // LOCKED    ~1 Hz
            else                         centihz = 50;  // ACQUIRING ~0.5 Hz

            led_err_snapshot = err;
            led_rx_snapshot  = rx;
            led_rpt_snapshot = telem_report_count;
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
