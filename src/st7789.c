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
static void wr_data8(uint8_t dat) { dc_set(); cs_clr(); lcd_send_byte(dat); cs_set(); }

// Address window. Matches EVT LCD_Address_Set with USE_HORIZONTAL==0 (no offset).
static void set_window(uint16_t xa, uint16_t ya, uint16_t xb, uint16_t yb)
{
    wr_reg(0x2A);
    cs_clr(); lcd_send_byte(xa >> 8); lcd_send_byte(xa & 0xFF);
              lcd_send_byte(xb >> 8); lcd_send_byte(xb & 0xFF); cs_set();
    wr_reg(0x2B);
    cs_clr(); lcd_send_byte(ya >> 8); lcd_send_byte(ya & 0xFF);
              lcd_send_byte(yb >> 8); lcd_send_byte(yb & 0xFF); cs_set();
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
    cs_set();   // idle-deassert CS before the reset pulse

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
