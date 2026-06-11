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
// USART2 = PD5(TX)/PD6(RX), AF7 — this is the mapping EVERY WCH CH32H417 EVT
// example uses (USART/USART_DMA Common/hardware.c:82-93 + main.c:16). The DMA
// request mux must also be programmed (USART2_TX -> DMAMUX req 87, USART2_RX ->
// 88; see uart.c) or DMA never triggers. NOTE: the EVT *debug printf* console
// is a different port (USART1 TX=PA9/AF7) — don't confuse it with this link.
// If your carrier wires the USB-UART bridge to other pins, change the PIN/AF/
// PORT macros below; this is the single place that defines the mapping.
#define CMD_USART           USART2
#define CMD_USART_IRQn      USART2_IRQn
#define CMD_USART_RCC_HB1   RCC_HB1Periph_USART2
#define CMD_USART_DATAR     (&USART2->DATAR)
#define CMD_BAUD_DEFAULT    4000000u    // Hurra boots 4 Mbaud (matches bridge)

// USART2 pin map (AF7) + DMA request-mux source numbers. Change these together
// if the board routes USART2 elsewhere.
#define CMD_USART_GPIO_PORT     GPIOD
#define CMD_USART_GPIO_RCC_HB2  RCC_HB2Periph_GPIOD
#define CMD_USART_TX_PIN        GPIO_Pin_5
#define CMD_USART_TX_PINSRC     GPIO_PinSource5
#define CMD_USART_RX_PIN        GPIO_Pin_6
#define CMD_USART_RX_PINSRC     GPIO_PinSource6
#define CMD_USART_GPIO_AF       GPIO_AF7
#define CMD_USART_DMA_REQ_TX    87u     // USART2_TX DMA request (EVT USART_DMA)
#define CMD_USART_DMA_REQ_RX    88u     // USART2_RX DMA request
