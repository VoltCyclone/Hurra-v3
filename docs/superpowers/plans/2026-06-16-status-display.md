# Status Display Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the board's shipped 1.54" ST7789 240×240 SPI TFT from the V3F core to show live relay status (state, device VID:PID, reports/sec, liveness), fed by a minimal V5F→V3F IPC telemetry trickle.

**Architecture:** V3F owns all SPI + rendering (off the V5F USB hot path). A trimmed ST7789 driver (`st7789.c`) does pixels; `display.c` does status→text layout with per-line dirty tracking; `icc.c` gains a coherent single-writer reverse status channel over the free IPC CH2/CH3 status bits [16:31] (the existing CH1 [8:15] stage telemetry is untouched). V5F publishes status fields on a throttle + on state change; V3F polls and reassembles.

**Tech Stack:** Bare-metal RISC-V (WCH CH32H417 dual-core), WCH StdPeriph SPI2 driver, GCC 14.2 rv32imafc hard-float. Host-compiled C11 unit tests (`cc`) for the pure logic, mirroring the existing `humanize_test`/`motion_test` targets.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `src/board.h` (modify) | LCD pin/peripheral macros (SPI2, PB12/13/15, PD8/9), next to existing LED/USART maps. |
| `src/st7789.c` / `.h` (create) | Pixels only: SPI2 init, `st7789_init()` (verbatim EVT register sequence), `st7789_set_window()`, `st7789_fill_rect()`, `st7789_draw_char()`, `st7789_draw_string()`. No graphics lib, no embedded image. |
| `src/font5x7.h` (create) | One `const` 5×7 ASCII font table (0x20–0x7E) in flash. |
| `src/display.c` / `.h` (create) | Status layer. `display_format_lines()` (PURE: status→text rows + dirty mask, host-testable), `display_init()`, `display_render()` (blits only dirty rows via st7789). |
| `src/icc.c` / `.h` (modify) | `display_status_t`, reverse status channel: pure `icc_status_pack()`/`icc_status_unpack()` + MMIO `icc_status_publish_v5f()`/`icc_status_read_raw_v3f()`, plus `icc_status_pump_v5f()` (V5F rotate/publish) and `icc_status_poll_v3f()` (V3F read+merge). |
| `src/main_v3f.c` (modify) | `display_init()` at boot; per-loop `icc_status_poll_v3f()`; millis-gated render tick (~250 ms). |
| `src/main_v5f.c`, `src/usb_merge.c` (modify) | Maintain a `display_status_t`; call status publish on relay state change + a throttled pump; count reports for the 1 Hz rate. |
| `Makefile` (modify) | Add `src/st7789.c src/display.c` to `V3F_SRC`; add `display_test` to the `test` target. |
| `test/display_test.c` (create) | Host test for `display_format_lines` + `icc_status_pack/unpack` round-trip. |

---

## Conventions (read before any task)

- **Types:** uses `uint8_t`/`uint16_t`/`uint32_t` (`<stdint.h>`). The WCH StdPeriph uses `u8`/`u16`; in our `src/` files use the stdint names and cast at StdPeriph call sites, matching existing `src/` style.
- **Commit style:** Co-Author trailers are stripped from this repo's history — do **not** add them. Conventional-commit prefixes (`feat:`/`docs:`/`build:`) match the existing log.
- **Hardware checkpoints** are explicit manual steps (flash + eyeball the panel). They are not skippable but cannot be unit-tested — the testable logic is isolated into pure functions that ARE unit-tested.
- **Branch:** all work on `feat/status-display` (already created).
- **`make` builds both cores;** `make v3f` builds only V3F (faster feedback for display-only tasks).

---

### Task 1: LCD pin map in board.h

**Files:**
- Modify: `src/board.h` (append before the final content; after the USART block ending at line 75)

- [ ] **Step 1: Add the LCD pin/peripheral macro block**

Append to `src/board.h`:

```c
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
```

- [ ] **Step 2: Verify it compiles into the existing V3F image**

Run: `make v3f 2>&1 | tail -5`
Expected: build succeeds (board.h is header-only; adding macros must not break the existing build). No `v3f.elf` size change is required yet.

- [ ] **Step 3: Commit**

```bash
git add src/board.h
git commit -m "feat(display): add ST7789 SPI2 pin map to board.h"
```

---

### Task 2: ST7789 driver — SPI init, reset, register sequence, fill

**Files:**
- Create: `src/st7789.h`
- Create: `src/st7789.c`
- Modify: `Makefile` (V3F_SRC list at lines 39-41)

This task copies the **load-bearing** parts verbatim from EVT `doc/EVT/EXAM/SPI/SPI_LCD/Common/lcd.c`: the SPI2 config, the byte primitives, and the `lcd_init` power-on register sequence. It drops the EVT graphics library, the 52 KB embedded test image, and the GPIO-bitbang path.

- [ ] **Step 1: Write `src/st7789.h`**

```c
#pragma once
#include <stdint.h>

// ST7789 240x240 SPI TFT driver (V3F-only). Pins/peripheral come from board.h.
// RGB565 colors.
#define ST_BLACK   0x0000u
#define ST_WHITE   0xFFFFu
#define ST_RED     0xF800u
#define ST_GREEN   0x07E0u
#define ST_BLUE    0x001Fu
#define ST_YELLOW  0xFFE0u
#define ST_GRAY    0x8430u

// Bring up SPI2 + GPIO, set VIO18=3.3V, reset and initialize the panel.
// Fire-and-forget (the panel has no readback wired). Safe no-op if absent.
void st7789_init(void);

// Fill the inclusive-exclusive rect [x0,x1) x [y0,y1) with one color.
void st7789_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

// Draw one printable ASCII char at (x,y) top-left, integer-scaled, in the 5x7
// font. `scale`>=1. Background is painted (opaque) so redraws overwrite cleanly.
void st7789_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);

// Draw a NUL-terminated string left-to-right from (x,y). Returns the x past the
// last glyph. No wrapping.
uint16_t st7789_draw_string(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale);
```

- [ ] **Step 2: Write `src/st7789.c`**

```c
// st7789.c — trimmed ST7789 240x240 SPI driver for the V3F status display.
// The SPI2 setup, byte primitives, and power-on register sequence are copied
// verbatim from the wuxx EVT example doc/EVT/EXAM/SPI/SPI_LCD/Common/lcd.c
// (SPI_HW path), with the graphics library, embedded test image, and the
// GPIO-bitbang path removed. The panel is write-only here (no MISO readback).
#include "st7789.h"
#include "board.h"
#include "ch32h417_conf.h"
#include "debug.h"          // Delay_Ms / Delay_Init already used elsewhere
#include "font5x7.h"

#define LCD_W LCD_WIDTH
#define LCD_H LCD_HEIGHT

static inline void cs_clr(void)  { GPIO_ResetBits(LCD_CS_PORT,  LCD_CS_PIN);  }
static inline void cs_set(void)  { GPIO_SetBits(LCD_CS_PORT,   LCD_CS_PIN);  }
static inline void dc_clr(void)  { GPIO_ResetBits(LCD_DC_PORT,  LCD_DC_PIN);  }
static inline void dc_set(void)  { GPIO_SetBits(LCD_DC_PORT,   LCD_DC_PIN);  }
static inline void res_clr(void) { GPIO_ResetBits(LCD_RES_PORT, LCD_RES_PIN); }
static inline void res_set(void) { GPIO_SetBits(LCD_RES_PORT,  LCD_RES_PIN); }

// One byte over SPI2 (full-duplex; read back the dummy to clear RXNE). Verbatim
// from EVT LCD_SendByte (SPI_HW), CS managed by the caller.
static void lcd_send_byte(uint8_t dat)
{
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(LCD_SPI, dat);
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    (void)SPI_I2S_ReceiveData(LCD_SPI);
}

static void wr_reg(uint8_t cmd)   { dc_clr(); cs_clr(); lcd_send_byte(cmd); cs_set(); dc_set(); }
static void wr_data8(uint8_t dat) { cs_clr(); lcd_send_byte(dat); cs_set(); }

// Address window. Matches EVT LCD_Address_Set with USE_HORIZONTAL==0 (no offset).
static void set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    wr_reg(0x2A);
    cs_clr(); lcd_send_byte(x1 >> 8); lcd_send_byte(x1 & 0xFF);
              lcd_send_byte(x2 >> 8); lcd_send_byte(x2 & 0xFF); cs_set();
    wr_reg(0x2B);
    cs_clr(); lcd_send_byte(y1 >> 8); lcd_send_byte(y1 & 0xFF);
              lcd_send_byte(y2 >> 8); lcd_send_byte(y2 & 0xFF); cs_set();
    wr_reg(0x2C);   // memory write
}

static void gpio_spi_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    SPI_InitTypeDef  spi  = {0};

    RCC_HB2PeriphClockCmd(LCD_GPIO_RCC_HB2, ENABLE);
    RCC_HB1PeriphClockCmd(LCD_SPI_RCC_HB1, ENABLE);

    // DC (PD9) + RES (PD8) push-pull out
    gpio.GPIO_Pin = LCD_DC_PIN | LCD_RES_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(LCD_DC_PORT, &gpio);   // LCD_DC_PORT == LCD_RES_PORT == GPIOD

    // CS (PB12) push-pull out
    gpio.GPIO_Pin = LCD_CS_PIN;
    GPIO_Init(LCD_CS_PORT, &gpio);

    // SCK (PB13 AF5)
    GPIO_PinAFConfig(LCD_SCK_PORT, LCD_SCK_PINSRC, LCD_SPI_AF);
    gpio.GPIO_Pin = LCD_SCK_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(LCD_SCK_PORT, &gpio);

    // MISO (PB14 AF5) floating in (unused by the panel)
    GPIO_PinAFConfig(LCD_MISO_PORT, LCD_MISO_PINSRC, LCD_SPI_AF);
    gpio.GPIO_Pin = LCD_MISO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(LCD_MISO_PORT, &gpio);

    // MOSI (PB15 AF5)
    GPIO_PinAFConfig(LCD_MOSI_PORT, LCD_MOSI_PINSRC, LCD_SPI_AF);
    gpio.GPIO_Pin = LCD_MOSI_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(LCD_MOSI_PORT, &gpio);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode0;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(LCD_SPI, &spi);
    SPI_Cmd(LCD_SPI, ENABLE);
}

void st7789_init(void)
{
    // VIO18 rail -> 3.3V (EVT does this in SW; HW config recommended for safety).
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_PWR, ENABLE);
    PWR_VIO18ModeCfg(PWR_VIO18CFGMODE_SW);
    PWR_VIO18LevelCfg(PWR_VIO18Level_MODE3);

    gpio_spi_init();

    res_clr(); Delay_Ms(100);
    res_set(); Delay_Ms(100);
    Delay_Ms(100);

    // ----- Power-on register sequence (verbatim from EVT lcd_init) -----
    wr_reg(0x11); Delay_Ms(120);            // sleep out
    wr_reg(0x36); wr_data8(0x00);           // MADCTL, USE_HORIZONTAL==0
    wr_reg(0x3A); wr_data8(0x05);           // COLMOD = RGB565
    wr_reg(0xB2); wr_data8(0x0C); wr_data8(0x0C); wr_data8(0x00);
                  wr_data8(0x33); wr_data8(0x33);
    wr_reg(0xB7); wr_data8(0x35);
    wr_reg(0xBB); wr_data8(0x32);           // Vcom = 1.35V
    wr_reg(0xC2); wr_data8(0x01);
    wr_reg(0xC3); wr_data8(0x15);           // GVDD = 4.8V
    wr_reg(0xC4); wr_data8(0x20);           // VDV = 0V
    wr_reg(0xC6); wr_data8(0x0F);           // 60 Hz
    wr_reg(0xD0); wr_data8(0xA4); wr_data8(0xA1);
    wr_reg(0xE0); wr_data8(0xD0); wr_data8(0x08); wr_data8(0x0E); wr_data8(0x09);
                  wr_data8(0x09); wr_data8(0x05); wr_data8(0x31); wr_data8(0x33);
                  wr_data8(0x48); wr_data8(0x17); wr_data8(0x14); wr_data8(0x15);
                  wr_data8(0x31); wr_data8(0x34);
    wr_reg(0xE1); wr_data8(0xD0); wr_data8(0x08); wr_data8(0x0E); wr_data8(0x09);
                  wr_data8(0x09); wr_data8(0x15); wr_data8(0x31); wr_data8(0x33);
                  wr_data8(0x48); wr_data8(0x17); wr_data8(0x14); wr_data8(0x15);
                  wr_data8(0x31); wr_data8(0x34);
    wr_reg(0x21);                           // inversion on
    wr_reg(0x29);                           // display on

    st7789_fill_rect(0, 0, LCD_W, LCD_H, ST_BLACK);
}

void st7789_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    if (x1 > LCD_W) x1 = LCD_W;
    if (y1 > LCD_H) y1 = LCD_H;
    if (x0 >= x1 || y0 >= y1) return;
    set_window(x0, y0, (uint16_t)(x1 - 1), (uint16_t)(y1 - 1));
    uint8_t hi = (uint8_t)(color >> 8), lo = (uint8_t)color;
    cs_clr();
    for (uint32_t n = (uint32_t)(x1 - x0) * (uint32_t)(y1 - y0); n; n--) {
        lcd_send_byte(hi);
        lcd_send_byte(lo);
    }
    cs_set();
}

// (st7789_draw_char / st7789_draw_string are added in Task 4.)
```

- [ ] **Step 3: Keep the font include disabled until Task 4**

`font5x7.h` is created in Task 3 and first used by the draw functions added in
Task 4. So st7789.c does not need it yet. In `src/st7789.c`, change the line
`#include "font5x7.h"` to:

```c
// #include "font5x7.h"  // enabled in Task 4 (draw_char/draw_string)
```

This keeps Task 2 self-contained (no forward file dependency).

- [ ] **Step 4: Add st7789.c to the V3F build**

Modify `Makefile` `V3F_SRC` (lines 39-41). Change:

```make
V3F_SRC = src/main_v3f.c src/icc.c src/led.c src/uart.c src/kmbox_cmd.c \
          src/actions.c src/humanize.c core/timebase.c $(PROTO_SRC) \
          core/system_ch32h417.c $(LIBSRC)
```

to add `src/st7789.c`:

```make
V3F_SRC = src/main_v3f.c src/icc.c src/led.c src/uart.c src/kmbox_cmd.c \
          src/actions.c src/humanize.c src/st7789.c core/timebase.c $(PROTO_SRC) \
          core/system_ch32h417.c $(LIBSRC)
```

- [ ] **Step 5: Build the V3F image**

Run: `make v3f 2>&1 | tail -8`
Expected: compiles and links clean. `st7789_init`/`st7789_fill_rect` are not yet called, so they may warn as unused at -O if `-Wunused` is on — that is fine; a clean link is the bar. If StdPeriph names differ (e.g. `PWR_VIO18CFGMODE_SW`), grep the headers to confirm exact spelling: `grep -rn VIO18 vendor/wch/Peripheral/inc/ch32h417_pwr.h`.

- [ ] **Step 6: Commit**

```bash
git add src/st7789.c src/st7789.h Makefile
git commit -m "feat(display): ST7789 SPI2 driver — init sequence + fill_rect"
```

---

### Task 3: Pure status→layout logic + font + host test (TDD)

This is the host-testable core. `display_format_lines()` turns a `display_status_t`
into fixed text rows and a dirty mask vs the previous frame — no hardware. The
`display_status_t` is defined in `icc.h` in Task 5; to keep this task independent,
define it here in `display.h` and Task 5 will move it to `icc.h` and have
`display.h` include it. (We define it once, in `display.h`, and `icc.h` includes
`display.h` for the struct — decided here to avoid duplication.)

**Files:**
- Create: `src/display.h`
- Create: `src/font5x7.h`
- Create: `src/display.c` (format function only this task)
- Create: `test/display_test.c`
- Modify: `Makefile` (`test` target, line ~207)

- [ ] **Step 1: Write `src/display.h`**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

// Relay state shown on the status display (mirrors V5F relay progress).
typedef enum {
    DISP_STATE_BOOT = 0,   // V5F not yet in relay path
    DISP_STATE_WAITING,    // host port powered, no device
    DISP_STATE_CAPTURING,  // device attached, capturing descriptors
    DISP_STATE_RELAYING,   // forwarding reports PC<-device
    DISP_STATE_ERROR,      // relay reported an error/wedge
    DISP_STATE_NOSIGNAL,   // V3F lost the telemetry heartbeat
} disp_state_t;

// Live status the display renders. Populated on V3F from V5F telemetry + local.
typedef struct {
    uint8_t  state;        // disp_state_t
    uint16_t vid;
    uint16_t pid;
    uint16_t reports_per_sec;
    uint32_t uptime_s;     // V3F-local (millis based)
} display_status_t;

// Text grid. 240px / (6px glyph cell * scale 3) = 13 cols; 5 rows * (8*3)=120px.
#define DISP_COLS   13
#define DISP_ROWS    5
#define DISP_SCALE   3

// PURE: render `st` into `rows` (DISP_ROWS NUL-terminated strings, each <=DISP_COLS
// chars). Compares against `prev` and returns a dirty bitmask (bit r set = row r
// changed). `prev` may be NULL (everything dirty). After the call, the caller
// copies rows->prev. Host-testable; no hardware.
uint32_t display_format_lines(const display_status_t *st,
                              char rows[DISP_ROWS][DISP_COLS + 1],
                              const char prev_rows[DISP_ROWS][DISP_COLS + 1]);

// Hardware entry points (implemented across Tasks 4/6).
void display_init(void);
void display_render(const display_status_t *st);
```

- [ ] **Step 2: Write the failing host test `test/display_test.c`**

```c
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "display.h"

static void row_should_contain(const char *row, const char *needle) {
    if (!strstr(row, needle)) { printf("FAIL: '%s' lacks '%s'\n", row, needle); assert(0); }
}

int main(void) {
    char rows[DISP_ROWS][DISP_COLS + 1];

    // 1. NULL prev => all rows dirty.
    display_status_t st = { .state = DISP_STATE_RELAYING, .vid = 0x1A2C,
                            .pid = 0x0094, .reports_per_sec = 980, .uptime_s = 252 };
    uint32_t dirty = display_format_lines(&st, rows, NULL);
    assert(dirty == ((1u << DISP_ROWS) - 1));

    // 2. State row names the state; ids row shows hex VID:PID; rate + uptime shown.
    row_should_contain(rows[0], "RELAY");
    row_should_contain(rows[1], "1A2C");
    row_should_contain(rows[1], "0094");
    row_should_contain(rows[2], "980");
    row_should_contain(rows[3], "4:12");   // 252s -> 4:12

    // 3. No change => zero dirty bits.
    char prev[DISP_ROWS][DISP_COLS + 1];
    memcpy(prev, rows, sizeof rows);
    dirty = display_format_lines(&st, rows, (const char (*)[DISP_COLS+1])prev);
    assert(dirty == 0);

    // 4. Only reports/sec changes => only that row dirty.
    memcpy(prev, rows, sizeof rows);
    st.reports_per_sec = 12;
    dirty = display_format_lines(&st, rows, (const char (*)[DISP_COLS+1])prev);
    assert(dirty == (1u << 2));

    // 5. WAITING state, no device => ids row blanks the VID:PID.
    display_status_t w = { .state = DISP_STATE_WAITING };
    display_format_lines(&w, rows, NULL);
    row_should_contain(rows[0], "WAIT");
    assert(strstr(rows[1], "1A2C") == NULL);

    printf("display_test OK\n");
    return 0;
}
```

- [ ] **Step 3: Run the test to verify it fails (no implementation yet)**

Run: `cc -std=c11 -O2 -Isrc -o /tmp/display_test test/display_test.c src/display.c 2>&1 | tail -5`
Expected: link error — `undefined reference to display_format_lines` (display.c has no body yet). This confirms the test drives the implementation.

- [ ] **Step 4: Write `src/font5x7.h` (full ASCII 0x20–0x7E)**

```c
#pragma once
#include <stdint.h>
// 5x7 ASCII font, 5 column-bytes per glyph (LSB = top pixel), printable 0x20..0x7E.
// Public-domain 5x7 bitmap font (the classic "font5x7"/Adafruit-glcd layout).
static const uint8_t FONT5X7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 0x20 ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '\''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x7F,0x20,0x18,0x20,0x7F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x03,0x04,0x78,0x04,0x03}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x08,0x04,0x08,0x10,0x08}, // '~'
};
```

- [ ] **Step 5: Implement `display_format_lines` in `src/display.c`**

```c
// display.c — status->text layout (pure) + hardware render (Tasks 4/6).
#include "display.h"
#include <stdio.h>
#include <string.h>

static const char *state_name(uint8_t s) {
    switch (s) {
        case DISP_STATE_BOOT:      return "BOOT";
        case DISP_STATE_WAITING:   return "WAITING";
        case DISP_STATE_CAPTURING: return "CAPTURING";
        case DISP_STATE_RELAYING:  return "RELAYING";
        case DISP_STATE_ERROR:     return "ERROR";
        case DISP_STATE_NOSIGNAL:  return "NO SIGNAL";
        default:                   return "?";
    }
}

// Build the 5 fixed rows. Each is clamped to DISP_COLS by snprintf.
static void build_rows(const display_status_t *st, char rows[DISP_ROWS][DISP_COLS + 1]) {
    snprintf(rows[0], DISP_COLS + 1, "%s", state_name(st->state));

    bool have_dev = (st->state == DISP_STATE_RELAYING ||
                     st->state == DISP_STATE_CAPTURING);
    if (have_dev)
        snprintf(rows[1], DISP_COLS + 1, "%04X:%04X", st->vid, st->pid);
    else
        rows[1][0] = '\0';

    snprintf(rows[2], DISP_COLS + 1, "rps %u", (unsigned)st->reports_per_sec);

    unsigned m = (unsigned)(st->uptime_s / 60), s = (unsigned)(st->uptime_s % 60);
    snprintf(rows[3], DISP_COLS + 1, "up %u:%02u", m, s);

    rows[4][0] = '\0';   // spare line (firmware tag/version added in Task 6)
}

uint32_t display_format_lines(const display_status_t *st,
                              char rows[DISP_ROWS][DISP_COLS + 1],
                              const char prev_rows[DISP_ROWS][DISP_COLS + 1]) {
    build_rows(st, rows);
    uint32_t dirty = 0;
    for (int r = 0; r < DISP_ROWS; r++) {
        if (!prev_rows || strncmp(rows[r], prev_rows[r], DISP_COLS + 1) != 0)
            dirty |= (1u << r);
    }
    return dirty;
}
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `cc -std=c11 -O2 -Isrc -o /tmp/display_test test/display_test.c src/display.c && /tmp/display_test`
Expected: prints `display_test OK` and exits 0.

- [ ] **Step 7: Wire the host test into `make test`**

Modify the `test:` target in `Makefile` (after the `motion_test` lines) to append:

```make
	cc -std=c11 -O2 -Isrc -o /tmp/display_test test/display_test.c src/display.c
	/tmp/display_test
```

Run: `make test 2>&1 | tail -4`
Expected: all three tests run; ends with `display_test OK`.

- [ ] **Step 8: Commit**

```bash
git add src/display.h src/display.c src/font5x7.h test/display_test.c Makefile
git commit -m "feat(display): status->text layout with dirty tracking + host test"
```

---

### Task 4: ST7789 text drawing

**Files:**
- Modify: `src/st7789.c` (enable font include; add draw_char/draw_string)

- [ ] **Step 1: Enable the font include**

Edit `src/st7789.c`: change `// #include "font5x7.h"  // enabled in Task 4` back to `#include "font5x7.h"`.

- [ ] **Step 2: Append `st7789_draw_char` / `st7789_draw_string`**

```c
void st7789_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) c = ' ';
    const uint8_t *glyph = FONT5X7[(uint8_t)c - 0x20];
    if (scale < 1) scale = 1;
    // 5 columns + 1 spacing column; 7 rows used of an 8-row cell.
    uint16_t cell_w = (uint16_t)(6 * scale), cell_h = (uint16_t)(8 * scale);
    set_window(x, y, (uint16_t)(x + cell_w - 1), (uint16_t)(y + cell_h - 1));
    uint8_t fhi = fg >> 8, flo = fg, bhi = bg >> 8, blo = bg;
    cs_clr();
    for (uint16_t row = 0; row < 8; row++) {
        for (uint8_t sy = 0; sy < scale; sy++) {
            for (uint16_t col = 0; col < 6; col++) {
                uint8_t on = (col < 5) && (glyph[col] & (1u << row));
                for (uint8_t sx = 0; sx < scale; sx++) {
                    if (on) { lcd_send_byte(fhi); lcd_send_byte(flo); }
                    else    { lcd_send_byte(bhi); lcd_send_byte(blo); }
                }
            }
        }
    }
    cs_set();
}

uint16_t st7789_draw_string(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint16_t cell_w = (uint16_t)(6 * (scale < 1 ? 1 : scale));
    for (; *s; s++) {
        if (x + cell_w > LCD_W) break;
        st7789_draw_char(x, y, *s, fg, bg, scale);
        x = (uint16_t)(x + cell_w);
    }
    return x;
}
```

- [ ] **Step 3: Build the V3F image**

Run: `make v3f 2>&1 | tail -6`
Expected: clean compile/link. Functions still uncalled (display_render wires them in Task 6) — clean link is the bar.

- [ ] **Step 4: Commit**

```bash
git add src/st7789.c
git commit -m "feat(display): ST7789 5x7 text drawing (scaled, opaque)"
```

---

### Task 5: Reverse status telemetry channel (V5F→V3F) + round-trip test (TDD)

Uses the free IPC status bits **[16:31]** (CH2+CH3); the existing CH1 [8:15] stage
telemetry is untouched. 16-bit reverse word `W = (seq<<14) | (sel<<10) | payload10`:
`seq` is a 2-bit rolling heartbeat (bumped every publish → liveness), `sel` is a
4-bit field selector, `payload` is 10 bits. V5F is the **sole writer** (race-free).
V3F polls, decodes whichever field is present, and merges it into its local
`display_status_t` (idempotent — re-reading the same field is harmless).

`display_status_t` and `disp_state_t` already live in `display.h` (Task 3);
`icc.h` includes `display.h` for them. The pack/unpack functions are pure and
host-tested; the publish/read wrappers are thin MMIO.

**Files:**
- Modify: `src/icc.h` (declarations + selector enum)
- Create: `src/icc_status.c` (PURE pack/unpack — host-linkable, no MMIO)
- Modify: `src/icc.c` (MMIO wrappers + pump/poll only)
- Modify: `test/display_test.c` (add round-trip test)
- Modify: `Makefile` (add `src/icc_status.c` to both cores; link it into `display_test`)

**Decomposition decision (resolves the host-link problem up front):** the pure
`icc_status_pack`/`unpack` go in their own file `src/icc_status.c` that includes
**only** `icc.h` (stdint + display.h). The MMIO wrappers (`pump_v5f`/`poll_v3f`)
stay in `icc.c` with the rest of the hardware code. The host test links
`src/icc_status.c` (pure) and never `src/icc.c` (MMIO). No `-DICC_HOSTTEST`
guards needed.

- [ ] **Step 1: Add declarations to `src/icc.h`**

After the existing `icc_telem_read_v3f` declaration (line 87), add:

```c
#include "display.h"   // display_status_t, disp_state_t

// --- V5F->V3F reverse status channel (IPC status bits [16:31], CH2+CH3) -------
// Single-writer (V5F) coherent MMIO status, time-multiplexed. Distinct from the
// CH1 [8:15] stage telemetry above. 16-bit word: [15:14]=seq, [13:10]=field
// selector, [9:0]=payload. V3F polls and reassembles into a display_status_t.
enum {                          // field selectors
    ICC_ST_SEL_STATE = 0,       // payload[2:0] = disp_state_t
    ICC_ST_SEL_VID_HI,          // payload[7:0] = vid >> 8
    ICC_ST_SEL_VID_LO,          // payload[7:0] = vid & 0xFF
    ICC_ST_SEL_PID_HI,          // payload[7:0] = pid >> 8
    ICC_ST_SEL_PID_LO,          // payload[7:0] = pid & 0xFF
    ICC_ST_SEL_RPS,             // payload[9:0] = reports_per_sec (clamped 0..1023)
    ICC_ST_SEL__COUNT
};

// PURE (host-testable): pack one field of `st` into a 16-bit word with seq.
uint16_t icc_status_pack(uint8_t sel, uint8_t seq, const display_status_t *st);
// PURE: decode `word`, merging the carried field into `acc`. Returns the 2-bit seq.
uint8_t  icc_status_unpack(uint16_t word, display_status_t *acc);

// V5F: publish the next field in rotation (call on a throttle); publishes STATE
// immediately when `st->state` differs from the last published state.
void icc_status_pump_v5f(const display_status_t *st);
// V3F: read the current reverse word and merge into `acc`. Returns true if the
// heartbeat seq advanced since the last call (i.e. V5F is alive & publishing).
bool icc_status_poll_v3f(display_status_t *acc);
```

- [ ] **Step 2: Add the failing round-trip test to `test/display_test.c`**

First add `#include "icc.h"` to the top of `test/display_test.c` (with the other
includes). Then, before `printf("display_test OK")`, add:

```c
    // --- icc_status pack/unpack round-trip ---
    display_status_t src = { .state = DISP_STATE_RELAYING, .vid = 0x1A2C,
                             .pid = 0x0094, .reports_per_sec = 980 };
    display_status_t acc = {0};
    for (uint8_t sel = 0; sel < ICC_ST_SEL__COUNT; sel++) {
        uint16_t w = icc_status_pack(sel, sel & 3, &src);
        icc_status_unpack(w, &acc);
    }
    assert(acc.state == DISP_STATE_RELAYING);
    assert(acc.vid == 0x1A2C);
    assert(acc.pid == 0x0094);
    assert(acc.reports_per_sec == 980);
    // seq is the high 2 bits.
    assert(icc_status_unpack(icc_status_pack(ICC_ST_SEL_STATE, 2, &src), &acc) == 2);
```

Move `#include "icc.h"` to the top of the file with the other includes.

- [ ] **Step 3: Run the test to verify it fails**

Run: `cc -std=c11 -O2 -Isrc -o /tmp/display_test test/display_test.c src/display.c src/icc_status.c 2>&1 | tail -8`
Expected: failure — `src/icc_status.c` does not exist yet (Step 4) and `icc_status_pack`/`unpack` are undefined. This confirms the test compiles against the new declarations in icc.h.

- [ ] **Step 4: Create `src/icc_status.c` with the PURE pack/unpack**

```c
// icc_status.c — pure (host-testable) pack/unpack for the V5F->V3F reverse
// status channel. NO MMIO, NO WCH headers — includes only icc.h (stdint +
// display.h). The MMIO wrappers that USE these live in icc.c.
#include "icc.h"

uint16_t icc_status_pack(uint8_t sel, uint8_t seq, const display_status_t *st)
{
    uint16_t payload = 0;
    switch (sel) {
        case ICC_ST_SEL_STATE:  payload = (uint16_t)(st->state & 0x07u); break;
        case ICC_ST_SEL_VID_HI: payload = (uint16_t)((st->vid >> 8) & 0xFFu); break;
        case ICC_ST_SEL_VID_LO: payload = (uint16_t)(st->vid & 0xFFu); break;
        case ICC_ST_SEL_PID_HI: payload = (uint16_t)((st->pid >> 8) & 0xFFu); break;
        case ICC_ST_SEL_PID_LO: payload = (uint16_t)(st->pid & 0xFFu); break;
        case ICC_ST_SEL_RPS:    payload = (st->reports_per_sec > 1023u)
                                          ? 1023u : st->reports_per_sec; break;
        default: break;
    }
    return (uint16_t)(((seq & 0x3u) << 14) | ((sel & 0xFu) << 10) | (payload & 0x3FFu));
}

uint8_t icc_status_unpack(uint16_t word, display_status_t *acc)
{
    uint8_t  seq = (uint8_t)((word >> 14) & 0x3u);
    uint8_t  sel = (uint8_t)((word >> 10) & 0xFu);
    uint16_t payload = (uint16_t)(word & 0x3FFu);
    switch (sel) {
        case ICC_ST_SEL_STATE:  acc->state = (uint8_t)(payload & 0x07u); break;
        case ICC_ST_SEL_VID_HI: acc->vid = (uint16_t)((acc->vid & 0x00FFu) | (payload << 8)); break;
        case ICC_ST_SEL_VID_LO: acc->vid = (uint16_t)((acc->vid & 0xFF00u) | (payload & 0xFFu)); break;
        case ICC_ST_SEL_PID_HI: acc->pid = (uint16_t)((acc->pid & 0x00FFu) | (payload << 8)); break;
        case ICC_ST_SEL_PID_LO: acc->pid = (uint16_t)((acc->pid & 0xFF00u) | (payload & 0xFFu)); break;
        case ICC_ST_SEL_RPS:    acc->reports_per_sec = payload; break;
        default: break;
    }
    return seq;
}
```

- [ ] **Step 5: Add the MMIO wrappers to `src/icc.c`**

These USE the pure functions but touch `IPC->...` MMIO, so they live in `icc.c`
(hardware-only), not `icc_status.c`. Append to `src/icc.c` (after the existing
`icc_telem_read_v3f`):

```c
// --- V5F->V3F reverse status channel (IPC status bits [16:31], CH2+CH3) ------
// Distinct from the CH1 [8:15] stage telemetry above. Single-writer (V5F);
// V3F only reads. Coherent peripheral MMIO — never shared SRAM.
#define ICC_ST_SHIFT   16u
#define ICC_ST_MASK    (0xFFFFu << ICC_ST_SHIFT)

void icc_status_pump_v5f(const display_status_t *st)
{
    static uint8_t s_sel;            // rotation index
    static uint8_t s_seq;            // rolling heartbeat
    static uint8_t s_last_state = 0xFF;
    uint8_t sel;
    if (st->state != s_last_state) { // state changes jump the queue
        s_last_state = st->state;
        sel = ICC_ST_SEL_STATE;
    } else {
        sel = s_sel;
        s_sel = (uint8_t)((s_sel + 1) % ICC_ST_SEL__COUNT);
    }
    uint16_t word = icc_status_pack(sel, ++s_seq, st);
    IPC->CLR = ICC_ST_MASK;
    IPC->SET = ((uint32_t)word << ICC_ST_SHIFT);
}

bool icc_status_poll_v3f(display_status_t *acc)
{
    static uint8_t s_last_seq = 0xFF;
    uint16_t word = (uint16_t)((IPC->STS >> ICC_ST_SHIFT) & 0xFFFFu);
    uint8_t seq = icc_status_unpack(word, acc);
    bool advanced = (seq != s_last_seq);
    s_last_seq = seq;
    return advanced;
}
```

- [ ] **Step 6: Add `icc_status.c` to both cores; wire the host test into `make test`**

In `Makefile`, add `src/icc_status.c` to **both** `V3F_SRC` and `V5F_SRC` (the pure
functions are called by V3F's poll and V5F's pump). Then update the `display_test`
lines added in Task 3 Step 7 to link the pure file (no `icc.c`, no defines):

```make
	cc -std=c11 -O2 -Isrc -o /tmp/display_test test/display_test.c src/display.c src/icc_status.c
	/tmp/display_test
```

- [ ] **Step 7: Run the host test (should pass)**

Run: `cc -std=c11 -O2 -Isrc -o /tmp/display_test test/display_test.c src/display.c src/icc_status.c && /tmp/display_test`
Expected: `display_test OK`.

- [ ] **Step 8: Build both firmware images**

Run: `make 2>&1 | tail -8`
Expected: V3F and V5F both link clean with `src/icc_status.c` added to both.
`icc_status_pump_v5f`/`poll_v3f` are called in Task 6; clean link is the bar.

- [ ] **Step 9: Commit**

```bash
git add src/icc.h src/icc.c src/icc_status.c test/display_test.c Makefile
git commit -m "feat(display): V5F->V3F reverse status channel (IPC bits 16:31) + round-trip test"
```

---

### Task 6: Wire V3F render loop + V5F publish sites + display_render

**Files:**
- Modify: `src/display.c` (implement `display_init` / `display_render`)
- Modify: `src/main_v3f.c` (init + poll + render tick)
- Modify: `src/main_v5f.c` (maintain status, pump, report counter)
- Modify: `src/usb_merge.c` (bump report counter on each forwarded report) — only if the per-report site lives there; otherwise count in main_v5f.

- [ ] **Step 1: Implement `display_init`/`display_render` in `src/display.c`**

Add at the top of `display.c`: `#include "st7789.h"` and `#include "board.h"`, then append:

```c
void display_init(void)
{
    st7789_init();   // includes a black clear
}

void display_render(const display_status_t *st)
{
    static char prev[DISP_ROWS][DISP_COLS + 1];
    static bool have_prev;
    char rows[DISP_ROWS][DISP_COLS + 1];
    uint32_t dirty = display_format_lines(st, rows,
                        have_prev ? (const char (*)[DISP_COLS + 1])prev : 0);
    for (int r = 0; r < DISP_ROWS; r++) {
        if (!(dirty & (1u << r))) continue;
        uint16_t y = (uint16_t)(r * 8 * DISP_SCALE);
        // Clear the whole row band, then draw text (opaque bg also covers it).
        st7789_fill_rect(0, y, LCD_WIDTH, (uint16_t)(y + 8 * DISP_SCALE), ST_BLACK);
        uint16_t fg = (st->state == DISP_STATE_RELAYING) ? ST_GREEN :
                      (st->state == DISP_STATE_ERROR ||
                       st->state == DISP_STATE_NOSIGNAL) ? ST_RED : ST_WHITE;
        st7789_draw_string(0, y, rows[r], fg, ST_BLACK, DISP_SCALE);
    }
    memcpy(prev, rows, sizeof rows);
    have_prev = true;
}
```

Add `#include <string.h>` if not already present (it is, from Task 3).

- [ ] **Step 2: Wire V3F main loop in `src/main_v3f.c`**

Add include near the top with the other `src/` includes: `#include "display.h"`.

After `led_heartbeat_start();` (line 180), add:

```c
    display_init();
    display_status_t g_disp = { .state = DISP_STATE_BOOT };
    uint32_t disp_render_tick = millis();
```

Inside the `for (;;)` loop, after the `while (icc_pump_to_v5f()) {}` block (line 213), add:

```c
        // Pull V5F status telemetry every iteration (cheap MMIO read); render at
        // ~4 Hz. Display is non-essential and must never gate the relay or the
        // command link — it only consumes V3F idle time.
        bool alive = icc_status_poll_v3f(&g_disp);
        uint32_t dnow = millis();
        if ((dnow - disp_render_tick) >= 250) {
            disp_render_tick = dnow;
            g_disp.uptime_s = dnow / 1000u;
            if (!alive && g_disp.state != DISP_STATE_BOOT)
                g_disp.state = DISP_STATE_NOSIGNAL;
            display_render(&g_disp);
        }
```

Note: `icc_status_poll_v3f` returns seq-advanced per call; since we poll far faster
than V5F publishes, treat "no advance across a full render interval" as the stall
signal. Refine: track advance across the 250 ms window rather than per-iteration:

```c
        // (replace the snippet above with this window-based liveness)
        static bool s_seen_advance;
        if (icc_status_poll_v3f(&g_disp)) s_seen_advance = true;
        uint32_t dnow = millis();
        if ((dnow - disp_render_tick) >= 250) {
            disp_render_tick = dnow;
            g_disp.uptime_s = dnow / 1000u;
            if (!s_seen_advance && g_disp.state != DISP_STATE_BOOT)
                g_disp.state = DISP_STATE_NOSIGNAL;
            s_seen_advance = false;
            display_render(&g_disp);
        }
```

- [ ] **Step 3: Maintain + publish status on V5F in `src/main_v5f.c`**

Add `#include "display.h"` near the other includes. Declare a file-scope status and
report counter, and map existing relay stages to `disp_state_t`. At the points that
already call `icc_telem_stage_v5f(...)` / set `dbg_stage(...)`, mirror the state:

- After host-wait begins (DBG_V5F_HOST_WAITING): `s_disp.state = DISP_STATE_WAITING;`
- After device connected / capturing descriptors (DBG_V5F_DEV_CONNECTED / DESC): `s_disp.state = DISP_STATE_CAPTURING;` and set `s_disp.vid`/`s_disp.pid` from the captured device descriptor (use the fields desc_capture stores — grep `idVendor`/`idProduct` in `src/desc_capture.c`/`usb_host.c` for the exact names).
- On reaching the relay loop (DBG_V5F_RELAY): `s_disp.state = DISP_STATE_RELAYING;`

Add near the relay loop body:

```c
    // Status display feed (non-essential). reports/sec computed once per second.
    static uint32_t s_rep_count, s_rep_tick;
    // ... on each successfully forwarded report:
    s_rep_count++;
    // ... once per relay iteration, throttled:
    uint32_t mnow = millis();
    if ((mnow - s_rep_tick) >= 1000) {
        s_rep_tick = mnow;
        s_disp.reports_per_sec = (s_rep_count > 1023u) ? 1023u : (uint16_t)s_rep_count;
        s_rep_count = 0;
    }
    icc_status_pump_v5f(&s_disp);   // rotate-publish one field; cheap MMIO
```

Place the report-count increment at the existing point where a report is confirmed
sent to the device (grep for `usb_device_send_report` in main_v5f.c / usb_merge.c and
increment right after a success). Keep `icc_status_pump_v5f` once per relay iteration —
it is two MMIO stores, no SRAM, consistent with the existing `icc_telem_stage_v5f`.

- [ ] **Step 4: Build both images**

Run: `make 2>&1 | tail -10`
Expected: clean build of both cores. Resolve any field-name mismatches (VID/PID source) by grepping desc_capture as noted.

- [ ] **Step 5: Host test still green**

Run: `make test 2>&1 | tail -3`
Expected: `display_test OK` (pure logic unchanged).

- [ ] **Step 6: Commit**

```bash
git add src/display.c src/main_v3f.c src/main_v5f.c src/usb_merge.c
git commit -m "feat(display): wire V3F render loop + V5F status publish"
```

---

### Task 7: Hardware bring-up + regression

**Files:** none (verification only). Each sub-step is a manual hardware checkpoint.

- [ ] **Step 1: Flash**

Run: `make flash 2>&1 | tail -15`
Expected: merge + program succeeds via the on-board WCH-LinkE. (If a 0x55 SWJ wedge
occurs, follow the power-off-erase recovery in CLAUDE.md — flash with nothing on the
USB host port.)

- [ ] **Step 2: Bring-up ladder — panel + init**

Observe the panel on power-up. Expected: it clears to black (the `st7789_init` clear),
then within ~250 ms shows the BOOT/WAITING state line in white. If the panel stays
dark: re-check FPC seating, then SPI2 pins/AF, then the VIO18 config. If garbled:
suspect SPI mode (CPOL/CPHA) or baud — drop `SPI_BaudRatePrescaler` one notch.

- [ ] **Step 3: Bring-up ladder — live status with a real mouse**

Plug the Razer Basilisk V3 into the host port. Expected on the panel:
WAITING → CAPTURING (shows `1A2C:` VID and the PID) → RELAYING (green), with `rps`
climbing to a nonzero value and `up M:SS` advancing. Pull the mouse: state returns to
WAITING and the VID:PID row blanks.

- [ ] **Step 4: Regression — relay + cursor unaffected**

With the mouse relaying, confirm the PC still sees the cloned mouse and the cursor
moves (the existing hardware-verified path). The display work is V3F-only; V5F behavior
must be unchanged. If the relay or cursor regressed, the `icc_status_pump_v5f` call site
is the only V5F change — verify it is not on a path that can block (it must be a plain
MMIO store, no wait).

- [ ] **Step 5: Liveness check**

Confirm that if telemetry stalls (e.g., immediately after a relay wedge, if one occurs),
the panel shows `NO SIGNAL` in red rather than a frozen stale screen.

- [ ] **Step 6: Final commit / notes**

```bash
git add -A
git commit -m "test(display): hardware bring-up verified (panel, live status, regression)"
```

Update `README.md` Requirements/Disclaimer if the VIO18 software-config caveat or the
optional display should be documented for users.

---

## Self-Review Notes

- **Spec coverage:** architecture (T6), components/files (all), telemetry protocol (T5),
  error handling — telemetry stall→NO SIGNAL (T6 S2, T7 S5), non-essential/non-blocking
  (T6 S3 note + T7 S4), VIO18 (T2), testing host+hardware (T3/T5 host, T7 hardware),
  build sequence (T1–T7 mirror the spec's sequence). Risks: STS bit budget resolved
  (CH2/CH3 [16:31] confirmed free); SPI baud fallback noted (T7 S2).
- **Open implementation lookups (flagged inline, not placeholders):** exact captured
  VID/PID field names in desc_capture/usb_host (T6 S3) and the exact confirmed-send site
  for the report counter (T6 S3) — both are "grep to confirm the existing symbol" steps,
  not undefined new logic.
- **Type consistency:** `display_status_t`/`disp_state_t` defined once in `display.h`,
  included by `icc.h`; selectors `ICC_ST_SEL_*` consistent across icc.h/icc.c/test;
  `display_format_lines` signature identical in header/impl/test.
