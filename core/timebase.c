// core/timebase.c — 1 kHz millisecond timebase for V3F (provides millis()).
//
// Approach: TIM3 (not SysTick).
//   The vendored Debug/debug.c implements Delay_Us/Delay_Ms by *polling*
//   SysTick0 (set CMP, enable CTLR bit0, busy-wait on ISR bit0, disable).
//   Taking SysTick0 over for a 1 ms interrupt would fight those polled delays
//   (each Delay_Ms reprograms CMP and toggles the enable bit). To keep the
//   vendor delay API intact and avoid that fragile sharing, the millisecond
//   tick lives on a spare general-purpose timer, TIM3 (HB1 bus). TIM3 is unused
//   elsewhere in the firmware and its IRQ vector (TIM3_IRQHandler) is wired in
//   core/startup_v3f.S, so this is a clean, fully-decoupled tick.

#include "ch32h417_port.h"   // device + StdPeriph (RCC/TIM/NVIC) + core_riscv
#include "timebase.h"

static volatile uint32_t g_ms;

void timebase_init(uint32_t core_hz)
{
    // For V3F, Core_Frequency == HCLK_Frequency (see RCC_GetClocksFreq), and
    // TIM3 (HB1) is fed from that domain, so `core_hz` is the timer clock.
    // Prescale to a 1 MHz tick, then count 1000 ticks per 1 ms update event.
    // psc = core_hz/1e6 - 1 ; arr = 1000 - 1  ->  update IRQ at 1 kHz.
    if (core_hz == 0) core_hz = 100000000u;           // safe fallback (V3F 100 MHz)
    uint32_t psc = (core_hz / 1000000u);
    if (psc == 0) psc = 1;                            // guard for very low clocks
    psc -= 1u;

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler         = (uint16_t)psc;         // -> 1 MHz timer tick
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 1000u - 1u;            // 1000 ticks @1MHz = 1 ms
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tb);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);       // drop the init-generated UEV
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM_Cmd(TIM3, ENABLE);
}

uint32_t millis(void) { return g_ms; }

// TIM3 update ISR — handler name matches the V3F vector table in startup_v3f.S.
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        g_ms++;
    }
}
