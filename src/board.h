#pragma once
#include <stdint.h>

// ── Clocks ──────────────────────────────────────────────────────────────────
#define BOARD_HSE_HZ        25000000u   // 25 MHz crystal (EVT ref §2)
#define BOARD_V5F_HZ        400000000u  // V5F core clock in USB profile
#define BOARD_V3F_HZ        100000000u  // V3F core clock in USB profile

// ── LED (status) ────────────────────────────────────────────────────────────
// EVT GPIO_Toggle blinks PB1, and the nanoCH32H417 schematic
// (wuxx/nanoCH32H417 hardware/nanoCH32H417.pdf) shows the PB1 net plus two user
// LEDs (LED0/LED1, each behind a 0R link). PB1 is the best-supported default;
// still trace LED0/LED1 -> exact GPIO on the carrier before trusting on-bench.
#define LED_GPIO_PORT       GPIOB
#define LED_GPIO_PIN        GPIO_Pin_1
#define LED_RCC_HB2         RCC_HB2Periph_GPIOB

// ── Command-link USART (V3F owns it) ────────────────────────────────────────
// USART2 PA2(TX)/PA3(RX) — PA2/PA3 nets are present on the schematic. NOTE the
// EVT debug console default is USART1 TX=PA9(AF7), a *different* port; if you
// repurpose the EVT printf path you'd be on USART1, not this command link.
// Confirm PA2/PA3 routing on the carrier.
#define CMD_USART           USART2
#define CMD_USART_IRQn      USART2_IRQn
#define CMD_USART_RCC_HB1   RCC_HB1Periph_USART2
#define CMD_USART_DATAR     (&USART2->DATAR)
#define CMD_BAUD_DEFAULT    4000000u    // Hurra boots 4 Mbaud (matches bridge)
