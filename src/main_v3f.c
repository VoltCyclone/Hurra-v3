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

int main(void)
{
    SystemInit();
    SystemAndCoreClockUpdate();
    Delay_Init();
    led_init();
    timebase_init(SystemCoreClock);     // start millis() before parsers run

    icc_init_v3f();                     // set up shared rings BEFORE releasing V5F
    NVIC_WakeUp_V5F(Core_V5F_StartAddr);
    HSEM_ITConfig(HSEM_ID0, ENABLE);

    uart_init(CMD_BAUD_DEFAULT);
    kmbox_cmd_init();                   // act_init + proto_init + bind tx
    led_heartbeat_start();

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
        kmbox_cmd_poll();               // UART -> proto -> act -> ICC

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
    }
}
