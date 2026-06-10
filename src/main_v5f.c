// main_v5f.c — V5F slave entry. Phase 1: prove it was released by V3F.
#include "ch32h417_port.h"
#include "board.h"
#include "led.h"
#include "debug.h"

int main(void)
{
    SystemAndCoreClockUpdate();   // V5F does NOT call SystemInit
    Delay_Init();

    HSEM_FastTake(HSEM_ID0);      // rendezvous with V3F
    HSEM_ReleaseOneSem(HSEM_ID0, 0);

    for (;;) {
        led_on();  Delay_Ms(80);
        led_off(); Delay_Ms(80);
    }
}
