#pragma once
#include <stdint.h>

// ── Clocks ──────────────────────────────────────────────────────────────────
#define BOARD_HSE_HZ        25000000u   // 25 MHz crystal
#define BOARD_V5F_HZ        400000000u  // V5F core clock in USB profile
#define BOARD_V3F_HZ        100000000u  // V3F core clock in USB profile

// ── LED (status) ────────────────────────────────────────────────────────────
// Status LEDs are on PC2/PC3 (PB1 has no LED attached on this board). Split
// per-core so each core owns a distinct pin: a shared pin would let V3F's TIM2
// heartbeat overwrite V5F's stage blink. V5F's own pin makes its blink ladder a
// state oracle readable by eye while SWD is unavailable (SWJ disabled at runtime).
// Both LEDs are on GPIOC, so the LED_RCC_HB2 clock enable covers either pin.
#define LED_GPIO_PORT       GPIOC
#define LED_RCC_HB2         RCC_HB2Periph_GPIOC
#if defined(Core_V5F)
#define LED_GPIO_PIN        GPIO_Pin_3   // V5F → LED1 (PC3): relay-stage indicator
#else
#define LED_GPIO_PIN        GPIO_Pin_2   // V3F → LED0 (PC2): command-core heartbeat
#endif

// ── Command-link USART (V3F owns it) ────────────────────────────────────────
// USART1 = PA9(TX)/PA10(RX), AF7, wired to the on-board WCH-LinkE virtual COM
// port via solder bridges SB3/SB4, so one USB-C cable does flash + debug +
// command link. The VCP caps at 921600 baud, the link's max.
//
// Transport is interrupt-driven (USART1_IRQHandler RXNE/TXE), no DMA: USART1 has
// no confirmed DMAMUX request number in the EVT set and the byte rate is modest.
//
// To use an external USB-UART bridge, repoint CMD_USART/_IRQn/_RCC/pins to that
// port (e.g. USART2 PD5/PD6 AF7, HB1 clock). uart.c references only these macros
// + CMD_USART_IRQHandler.
#define CMD_USART               USART1
#define CMD_USART_IRQn          USART1_IRQn
#define CMD_USART_IRQHandler    USART1_IRQHandler
#define CMD_USART_RCC_HB2_UART  RCC_HB2Periph_USART1   // USART1 clock is on HB2
#define CMD_USART_DATAR         (&USART1->DATAR)
// Boot baud from the Makefile (-DCMD_BAUD): 921600 Hurra / 115200 Ferrum.
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
// 1.54" ST7789 240x240 panel, driven by V3F over SPI2 in 4-wire mode. SPI2 is on
// HB1; the GPIO ports are on HB2. No collision with the relay firmware (V5F
// touches GPIOB only at PB8/PB9 for SWJ-disable).
//
//   SCK  = PB13 (AF5)      MOSI = PB15 (AF5)      MISO = PB14 (AF5, unused)
//   CS   = PB12 (GPIO)     RES  = PD8  (GPIO)     DC   = PD9  (GPIO)
//
// VIO18 rail must be 3.3V for the panel; display_init() sets it in software.
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

// ── Board-to-board SPI link (two-board end-to-end USB High-Speed) ────────────
// Carries HID descriptors + reports between the two boards: USB-host board (SPI
// master) -> USB-device board (SPI slave) on MOSI; reverse path on MISO. See
// two_board.c / AGENTS.md.
//
// SPI1 on GPIOA, AF5, with hardware NSS. Pins are constrained by what the
// nanoCH32H417 breaks out: SPI1's full four-wire group lands on four adjacent
// bottom-right header pins including hardware NSS (SPI4 lacks a broken-out NSS,
// which would force software-NSS and lose per-frame re-sync). Silkscreen labels
// pins by GPIO name minus the "P" (so "A4" = PA4).
//
//   NSS = PA4 (A4)   SCK = PA5 (A5)   MISO = PA6 (A6)   MOSI = PA7 (A7)
//   DATA_READY = PA3 (A3)   + GND   [all adjacent on the bottom-right header]
// Wire each silkscreen label straight across to the same label on the other
// board: A4<->A4, A5<->A5, A6<->A6, A7<->A7, A3<->A3, GND<->GND.
// No firmware conflict: temp sensor uses the internal ADC channel (no pin),
// USART1=PA9/10, USBFS device=PA11/12. SPI1 is on HB2, same bus as GPIOA.
#define LINK_SPI                 SPI1
#define LINK_SPI_RCC_HB2         RCC_HB2Periph_SPI1
#define LINK_GPIO_RCC_HB2        (RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOA)
#define LINK_SPI_AF              GPIO_AF5

#define LINK_SCK_PORT            GPIOA
#define LINK_SCK_PIN             GPIO_Pin_5
#define LINK_SCK_PINSRC          GPIO_PinSource5
#define LINK_MISO_PORT           GPIOA
#define LINK_MISO_PIN            GPIO_Pin_6
#define LINK_MISO_PINSRC         GPIO_PinSource6
#define LINK_MOSI_PORT           GPIOA
#define LINK_MOSI_PIN            GPIO_Pin_7
#define LINK_MOSI_PINSRC         GPIO_PinSource7
// NSS on PA4 (AF5): SPI1 hardware NSS. Master drives it (HW-NSS output or GPIO);
// slave uses it as the HW-NSS input so the master's CS edge gives per-frame
// select/deselect and bit re-alignment.
#define LINK_NSS_PORT            GPIOA
#define LINK_NSS_PIN             GPIO_Pin_4
#define LINK_NSS_PINSRC          GPIO_PinSource4

// Slave->master reverse-path-data doorbell (GPIO, EXTI line 3 on the master).
#define LINK_DRDY_PORT           GPIOA
#define LINK_DRDY_PIN            GPIO_Pin_3
#define LINK_DRDY_PINSRC         GPIO_PinSource3
