#pragma once
#include <stdint.h>

// ── Clocks ──────────────────────────────────────────────────────────────────
#define BOARD_HSE_HZ        25000000u   // 25 MHz crystal (EVT ref §2)
#define BOARD_V5F_HZ        400000000u  // V5F core clock in USB profile
#define BOARD_V3F_HZ        100000000u  // V3F core clock in USB profile

// ── LED (status) ────────────────────────────────────────────────────────────
// Placeholder: EVT GPIO_Toggle blinks PB1. Confirm against board schematic.
#define LED_GPIO_PORT       GPIOB
#define LED_GPIO_PIN        GPIO_Pin_1
#define LED_RCC_HB2         RCC_HB2Periph_GPIOB

// ── Command-link USART (V3F owns it) ────────────────────────────────────────
// USART2 PA2(TX)/PA3(RX) is the common WCH default; confirm on the carrier.
#define CMD_USART           USART2
#define CMD_USART_IRQn      USART2_IRQn
#define CMD_USART_RCC_HB1   RCC_HB1Periph_USART2
#define CMD_USART_DATAR     (&USART2->DATAR)
#define CMD_BAUD_DEFAULT    4000000u    // Hurra boots 4 Mbaud (matches bridge)
