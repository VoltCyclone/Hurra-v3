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
// PC2 = LED0 (V3F heartbeat); PC3 = LED1 (V5F relay-stage indicator).
// Split per-core so each core owns a DISTINCT LED. Both cores compile led.c
// separately (-DCore_V3F / -DCore_V5F) and both drive "the" status pin via these
// macros. On a SHARED pin, V3F's TIM2 heartbeat paints over V5F's boot-stage
// blink ladder, so V5F (which owns BOTH USB ports, and is the suspect when the PC
// sees no enumeration) is unobservable. Giving V5F its own PC3 turns its existing
// blink ladder into a probe-less state oracle — readable by eye even though the
// running firmware NAKs all SWD debug (SWJ disabled in main_v5f.c). PC3 is
// otherwise unused (no Pin_3 reference anywhere in src/). Both LEDs are on GPIOC,
// so the existing LED_RCC_HB2 clock enable in led_init() already covers PC3.
#define LED_GPIO_PORT       GPIOC
#define LED_RCC_HB2         RCC_HB2Periph_GPIOC
#if defined(Core_V5F)
#define LED_GPIO_PIN        GPIO_Pin_3   // V5F → LED1 (PC3): relay-core boot stage
#else
#define LED_GPIO_PIN        GPIO_Pin_2   // V3F → LED0 (PC2): command-core heartbeat
#endif

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

// ── Status display (ST7789 240x240 SPI TFT on the 12-pin FPC) ────────────────
// The board ships a 1.54" ST7789 240x240 panel. Driven by V3F over SPI2 in
// 4-wire mode, exactly as the wuxx EVT example doc/EVT/EXAM/SPI/SPI_LCD does.
// SPI2 is on the HB1 bus; the GPIO ports are on HB2. None of these pins collide
// with the relay firmware (V5F touches GPIOB only at PB8/PB9 for SWJ-disable).
//
//   SCK  = PB13 (AF5)      MOSI = PB15 (AF5)      MISO = PB14 (AF5, unused)
//   CS   = PB12 (GPIO)     RES  = PD8  (GPIO)     DC   = PD9  (GPIO)
//
// VIO18 rail must be 3.3V for the panel; display_init() sets it in software
// (PWR_VIO18*). EVT recommends a hardware config for external-device safety.
#define LCD_SPI                 SPI2
#define LCD_SPI_RCC_HB1         RCC_HB1Periph_SPI2
#define LCD_GPIO_RCC_HB2        (RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB | RCC_HB2Periph_GPIOD)

#define LCD_SCK_PORT            GPIOB
#define LCD_SCK_PIN             GPIO_Pin_13
#define LCD_SCK_PINSRC          GPIO_PinSource13
#define LCD_MISO_PORT           GPIOB
#define LCD_MISO_PIN            GPIO_Pin_14
#define LCD_MISO_PINSRC         GPIO_PinSource14
#define LCD_MOSI_PORT           GPIOB
#define LCD_MOSI_PIN            GPIO_Pin_15
#define LCD_MOSI_PINSRC         GPIO_PinSource15
#define LCD_SPI_AF              GPIO_AF5

#define LCD_CS_PORT             GPIOB
#define LCD_CS_PIN              GPIO_Pin_12
#define LCD_RES_PORT            GPIOD
#define LCD_RES_PIN             GPIO_Pin_8
#define LCD_DC_PORT             GPIOD
#define LCD_DC_PIN              GPIO_Pin_9

#define LCD_WIDTH               240
#define LCD_HEIGHT              240
