// core/timebase_v5f.c — 1 kHz millisecond timebase for the V5F core.
//
// Mirrors core/timebase.c (the V3F TIM3 tick) but uses TIM4 so the two cores
// never contend for the same timer instance. The merge module's release
// scheduling (usb_merge_drain_icc -> click/kb release deadlines) needs millis()
// on V5F; this provides it.
//
// Why TIM4 (not TIM3, not SysTick):
//   * TIM3 is the V3F timebase. Although the two cores are separate, keeping a
//     distinct timer per core avoids any shared-register confusion and matches
//     the "TIM4 on V5F" directive.
//   * The vendored Debug/debug.c implements Delay_Us/Delay_Ms by *polling*
//     SysTick (set CMP, busy-wait, disable). Taking SysTick for a 1 ms IRQ
//     would fight those polled delays (desc_capture.c / led_blink_forever use
//     them). A spare general-purpose timer keeps the vendor delay API intact.
//   * TIM4 lives on the HB1 bus and is unused elsewhere in the V5F image. Its
//     IRQ vector (TIM4_IRQHandler) is wired in core/startup_v5f.S.
//
// Timer clock domain (V5F):
//   TIM4 sits on the HB1/HCLK bus. RCC_GetClocksFreq().HCLK_Frequency is that
//   bus clock. We query it directly rather than trusting the passed-in core_hz,
//   because on V5F SystemCoreClock == Core_Frequency == the pre-FPRE AHB clock
//   (system_ch32h417.c: SystemCoreClock = tmp3), whereas the HB1 peripheral
//   clock is HCLK_Frequency = tmp3 >> FPRE. Using HCLK_Frequency makes the 1 ms
//   period exact regardless of the FPRE divider. The core_hz argument is kept
//   for API symmetry with timebase_init() and used only as a fallback.

#include "ch32h417_port.h"   // device + StdPeriph (RCC/TIM/NVIC) + core_riscv
#include "timebase_v5f.h"

static volatile uint32_t g_ms;

void timebase_v5f_init(uint32_t core_hz)
{
    // Determine the HB1 (TIM4) input clock. Prefer the measured HCLK from the
    // RCC driver; fall back to the caller-supplied core_hz, then a safe default.
    uint32_t timer_hz;
    RCC_ClocksTypeDef clk;
    RCC_GetClocksFreq(&clk);
    timer_hz = clk.HCLK_Frequency;
    if (timer_hz == 0) timer_hz = core_hz;
    if (timer_hz == 0) timer_hz = 200000000u;          // safe fallback

    // Prescale to a 1 MHz tick, then count 1000 ticks per 1 ms update event.
    uint32_t psc = (timer_hz / 1000000u);
    if (psc == 0) psc = 1;                             // guard for very low clocks
    psc -= 1u;

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler         = (uint16_t)psc;          // -> 1 MHz timer tick
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 1000u - 1u;             // 1000 ticks @1MHz = 1 ms
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);        // drop the init-generated UEV
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM_Cmd(TIM4, ENABLE);
}

uint32_t millis(void) { return g_ms; }

// TIM4 update ISR — handler name matches the V5F vector table in startup_v5f.S.
void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        g_ms++;
    }
}
