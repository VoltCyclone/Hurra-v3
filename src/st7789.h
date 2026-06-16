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
