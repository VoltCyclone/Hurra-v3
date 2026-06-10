// main_v3f.c — V3F is the boot/master core: clocks, then release V5F.
#include "ch32h417_port.h"
#include "board.h"
#include "led.h"
#include "debug.h"

int main(void)
{
    SystemInit();                 // V3F sets the PLL/clock tree
    SystemAndCoreClockUpdate();
    Delay_Init();

    NVIC_WakeUp_V5F(Core_V5F_StartAddr);   // release V5F (0x00010000)
    HSEM_ITConfig(HSEM_ID0, ENABLE);

    led_init();
    for (;;) {
        led_toggle();
        Delay_Ms(500);            // slow blink = V3F alive
    }
}
