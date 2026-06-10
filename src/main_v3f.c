// main_v3f.c — master: clocks, init ICC, release V5F, ping-pong over ICC.
#include "ch32h417_port.h"
#include "board.h"
#include "led.h"
#include "icc.h"
#include "debug.h"

int main(void)
{
    SystemInit();
    SystemAndCoreClockUpdate();
    Delay_Init();
    led_init();

    icc_init_v3f();                         // set up shared rings BEFORE releasing V5F
    NVIC_WakeUp_V5F(Core_V5F_StartAddr);
    HSEM_ITConfig(HSEM_ID0, ENABLE);

    uint32_t counter = 0, pongs = 0;
    for (;;) {
        icc_record_t r = { .tag = ICC_TAG_PING };
        r.b[0] = (uint8_t)(counter      );
        r.b[1] = (uint8_t)(counter >>  8);
        r.b[2] = (uint8_t)(counter >> 16);
        r.b[3] = (uint8_t)(counter >> 24);
        if (icc_send_to_v5f(&r)) { icc_ring_doorbell_v5f(); counter++; }

        icc_record_t got;
        while (icc_recv_from_v5f(&got)) {
            if (got.tag == ICC_TAG_PONG) pongs++;
        }
        if (pongs & 1) led_on(); else led_off();
        Delay_Ms(2);
    }
}
