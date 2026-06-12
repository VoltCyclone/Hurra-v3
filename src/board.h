#pragma once
#include <stdint.h>

// ── Clocks ──────────────────────────────────────────────────────────────────
#define BOARD_HSE_HZ        25000000u   // 25 MHz crystal (EVT ref §2)
#define BOARD_V5F_HZ        400000000u  // V5F core clock in USB profile
#define BOARD_V3F_HZ        100000000u  // V3F core clock in USB profile

// ── LED (status) ────────────────────────────────────────────────────────────
// CORRECTS AN EARLIER SCHEMATIC MISREAD (2026-06-12 bench): the board's status
// LEDs are on PC2 / PC3, NOT PB1. Evidence:
//   1. wuxx's own working blink demo — doc/EVT/EXAM/GPIO/GPIO_Toggle/Common/
//      hardware.c — does `GPIO_Init(GPIOC, Pin_2|Pin_3)` then toggles PC2/PC3
//      in a loop. (The example's main.c header comment says "PB1 push-pull";
//      that comment is stale — the code that actually blinks drives PC2/PC3.)
//   2. Schematic (hardware/nanoCH32H417.pdf): PB1 appears only as a bare MCU
//      pin label (PB1/ADC9/OP1_N0/...) with NOTHING attached — no LED, diode,
//      or resistor. The GPIO-driven indicator LEDs (D1/D2) route to PC2/PC3.
// Our old PB1 default toggled a dead pin: firmware booted (the RED power LED D4
// on 3V3 lit, so "the board starts") but the heartbeat never appeared.
// PC2 = LED0; PC3 = LED1 is available as a second indicator if wanted.
#define LED_GPIO_PORT       GPIOC
#define LED_GPIO_PIN        GPIO_Pin_2
#define LED_RCC_HB2         RCC_HB2Periph_GPIOC

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
