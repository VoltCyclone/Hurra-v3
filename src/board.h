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
// DEFAULT: USART3 = PB10(TX)/PB11(RX), AF7 — wired to the on-board WCH-LinkE's
// virtual COM port through solder bridges SB3/SB4 (schematic: WL_UART3_TX/RX ->
// PB10/PB11). This lets ONE USB-C cable do flash + debug + command link, no
// external USB-UART dongle. The WCH-LinkE VCP caps at 921600 baud (and 115200
// in HID mode), so the Hurra link runs at 921600, not the 4 Mbaud an external
// bridge allows. Mapping verified against EVT USART_DMA (USART3 PB10/PB11 AF7,
// DMA1 Ch2 TX/Ch3 RX, mux req 89/90).
//
// To go back to an external bridge on USART2 PD5/PD6 @ 4 Mbaud, swap this whole
// block to: USART2 / USART2_IRQn / RCC_HB1Periph_USART2 / GPIOD PD5/PD6 /
// DMA1_Channel7 TX (req 87, IRQn 7, IT_TC7) / DMA1_Channel6 RX (req 88), and
// set the Makefile Hurra CMD_BAUD back to 4000000. Everything below is the
// single source of truth for the mapping — uart.c references only these macros,
// including the TX-DMA ISR name, so the ISR can never drift from the vector.
#define CMD_USART           USART3
#define CMD_USART_IRQn      USART3_IRQn
#define CMD_USART_RCC_HB1   RCC_HB1Periph_USART3
#define CMD_USART_DATAR     (&USART3->DATAR)
// Boot baud comes from the Makefile (-DCMD_BAUD): 921600 Hurra / 115200 Ferrum.
#define CMD_BAUD_DEFAULT    ((uint32_t)CMD_BAUD)

// USART3 pin map (AF7) + per-direction DMA channel, request-mux source, and the
// TX transfer-complete plumbing. Change these together to retarget the link.
#define CMD_USART_GPIO_PORT     GPIOB
#define CMD_USART_GPIO_RCC_HB2  RCC_HB2Periph_GPIOB
#define CMD_USART_TX_PIN        GPIO_Pin_10
#define CMD_USART_TX_PINSRC     GPIO_PinSource10
#define CMD_USART_RX_PIN        GPIO_Pin_11
#define CMD_USART_RX_PINSRC     GPIO_PinSource11
#define CMD_USART_GPIO_AF       GPIO_AF7
// USART3: TX = DMA1 Ch2 (mux req 89), RX = DMA1 Ch3 (mux req 90). (EVT USART_DMA)
#define CMD_RX_DMA_CH           DMA1_Channel3
#define CMD_RX_DMA_MUX          DMA_MuxChannel3
#define CMD_USART_DMA_REQ_RX    90u
#define CMD_TX_DMA_CH           DMA1_Channel2
#define CMD_TX_DMA_MUX          DMA_MuxChannel2
#define CMD_USART_DMA_REQ_TX    89u
#define CMD_TX_DMA_IRQn         DMA1_Channel2_IRQn
#define CMD_TX_DMA_IT_TC        DMA1_IT_TC2
#define CMD_TX_DMA_IRQHandler   DMA1_Channel2_IRQHandler
