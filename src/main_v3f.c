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

    for (;;) {
        kmbox_cmd_poll();               // UART -> proto -> act -> ICC
        icc_record_t t;
        while (icc_recv_from_v5f(&t)) { /* Phase 7: consume V5F telemetry */ }
    }
}
