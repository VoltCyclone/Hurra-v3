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
// DEFAULT: USART1 = PA9(TX)/PA10(RX), AF7 — wired to the on-board WCH-LinkE's
// virtual COM port. Schematic: the WCH-LinkE UART crosses solder bridges
// SB3 (WL_UART3_TX -> target PA10) and SB4 (WL_UART3_RX -> target PA9); on the
// CH32H417, PA9/PA10 = USART1 (this is also the EVT debug.c default console,
// USART1 TX=PA9 AF7). So ONE USB-C cable does flash + debug + command link, no
// external dongle — just close SB3/SB4. The WCH-LinkE VCP caps at 921600 baud
// (115200 in HID mode), so the Hurra link runs at 921600.
//
// NOTE (corrects an earlier mistake): the net is named "WL_UART3" but that is
// the *WCH-Link's* internal UART3 — its target end lands on PA9/PA10 = USART1,
// NOT the target's PB10/PB11 (those are USART3 but route to the SD card here).
//
// Transport is INTERRUPT-DRIVEN (USART1_IRQHandler RXNE/TXE), no DMA: there is
// no confirmed DMAMUX request number for USART1 in the EVT set, and the command
// link's byte rate is modest, so polled/IRQ is the safe, proven choice.
//
// To use an external USB-UART bridge instead, repoint CMD_USART/_IRQn/_RCC/pins
// to that port (e.g. USART2 PD5/PD6 AF7, HB1 clock) and optionally raise the
// Makefile Hurra CMD_BAUD. uart.c references only these macros + CMD_USART_IRQHandler.
#define CMD_USART               USART1
#define CMD_USART_IRQn          USART1_IRQn
#define CMD_USART_IRQHandler    USART1_IRQHandler
#define CMD_USART_RCC_HB2_UART  RCC_HB2Periph_USART1   // USART1 clock is on HB2
#define CMD_USART_DATAR         (&USART1->DATAR)
// Boot baud comes from the Makefile (-DCMD_BAUD): 921600 Hurra / 115200 Ferrum.
#define CMD_BAUD_DEFAULT        ((uint32_t)CMD_BAUD)

// USART1 pin map (AF7). TX=PA9, RX=PA10 on GPIOA (HB2 bus).
#define CMD_USART_GPIO_PORT     GPIOA
#define CMD_USART_GPIO_RCC_HB2  RCC_HB2Periph_GPIOA
#define CMD_USART_TX_PIN        GPIO_Pin_9
#define CMD_USART_TX_PINSRC     GPIO_PinSource9
#define CMD_USART_RX_PIN        GPIO_Pin_10
#define CMD_USART_RX_PINSRC     GPIO_PinSource10
#define CMD_USART_GPIO_AF       GPIO_AF7
