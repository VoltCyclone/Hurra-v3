#include "ch32h417_port.h"
#include "board.h"
#include "led.h"
#include "debug.h"   /* Delay_Ms / Delay_Init */

static volatile uint16_t s_centihz = 100;
static volatile uint32_t s_tim_reload;

void led_init(void)
{
    RCC_HB2PeriphClockCmd(LED_RCC_HB2, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = LED_GPIO_PIN;
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(LED_GPIO_PORT, &g);
}

void led_on(void)     { GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN); }
void led_off(void)    { GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN); }
void led_toggle(void) { GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN,
                          (BitAction)!GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN)); }

void led_blink_forever(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
    for (;;) {
        for (uint8_t i = 0; i < count; i++) {
            led_on();  Delay_Ms(on_ms);
            led_off(); Delay_Ms(off_ms);
        }
        Delay_Ms(400);
    }
}

/* Heartbeat: TIM2 update IRQ toggles the LED. Reload computed from centihz.
   TIM2 on HB1 bus, clocked at V3F HCLK (~100 MHz). */
static void heartbeat_apply_rate(uint16_t centihz)
{
    if (centihz == 0) centihz = 100;
    uint32_t tick_hz = BOARD_V3F_HZ / 10000u;            /* ~10 kHz tick base */
    uint32_t toggles_per_sec = (2u * centihz + 50u) / 100u;
    if (toggles_per_sec == 0) toggles_per_sec = 1;
    s_tim_reload = tick_hz / toggles_per_sec;
    TIM2->ATRLR = (s_tim_reload ? s_tim_reload - 1u : 0u);
}

void led_heartbeat_start(void)
{
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef t = {0};
    t.TIM_Prescaler     = 9999;                 /* 10 kHz tick base */
    t.TIM_CounterMode   = TIM_CounterMode_Up;
    t.TIM_Period        = 4999;                 /* overwritten by apply_rate */
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &t);
    /* ARR preload: ATRLR writes latch on the update event instead of mid-count,
     * so a runtime rate change never produces a partial first period.
     * URS=Regular: only a real counter overflow raises the update interrupt, so
     * a software update event (used to latch a new ARR immediately) reloads the
     * timer without spuriously firing the toggle ISR. */
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);
    heartbeat_apply_rate(s_centihz);
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);  /* latch initial ARR now */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    NVIC_SetPriority(TIM2_IRQn, 16);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void led_heartbeat_set_rate(uint16_t centihz)
{
    s_centihz = centihz;
    heartbeat_apply_rate(centihz);
    /* Latch the new ARR immediately. With URS=Regular this software update does
     * not raise the update interrupt, so no extra masking dance is needed and the
     * heartbeat ISR cannot spuriously fire from the rate change. */
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
}

void TIM2_IRQHandler(void) WCH_IRQ;
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        led_toggle();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
