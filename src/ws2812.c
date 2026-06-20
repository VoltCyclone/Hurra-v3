#include "ws2812.h"

/* ── Pure logic (always compiled; no hardware) ───────────────────────────── */

/* Classic 8-bit integer HSV→RGB, then emitted in WS2812 G,R,B order.
 * h,s,v are 0..255. Uses h*6/255 sector mapping so pure-colour anchors
 * (h=0→R, h=85→G, h=170→B) land exactly at sector boundaries with rem=0. */
void ws2812_hsv_to_grb(uint8_t h, uint8_t s, uint8_t v, uint8_t out[3])
{
    uint8_t r, g, b;
    if (s == 0) {
        r = g = b = v;
    } else {
        /* h*6/255 places sector boundaries exactly at h=0(R), 85(G), 170(B). */
        uint16_t h6     = (uint16_t)h * 6;
        uint8_t  region = (uint8_t)(h6 / 255);          /* 0..5 (h<=254) or 6 (h=255, wraps to default) */
        uint8_t  rem    = (uint8_t)(h6 - (uint16_t)region * 255);
        uint8_t p = (uint8_t)((uint16_t)v * (255u - s) / 255u);
        uint8_t q = (uint8_t)((uint16_t)v * (255u - ((uint16_t)s * rem  / 255u)) / 255u);
        uint8_t t = (uint8_t)((uint16_t)v * (255u - ((uint16_t)s * (255u - rem) / 255u)) / 255u);
        switch (region) {
            case 0:  r = v; g = t; b = p; break;
            case 1:  r = q; g = v; b = p; break;
            case 2:  r = p; g = v; b = t; break;
            case 3:  r = p; g = q; b = v; break;
            case 4:  r = t; g = p; b = v; break;
            default: r = v; g = p; b = q; break;   /* region 5 (h=255 -> region 6 also handled here) */
        }
    }
    out[0] = g;   /* WS2812 wire order: green first */
    out[1] = r;
    out[2] = b;
}

int ws2812_should_send(uint8_t prev_hue, uint8_t cur_hue,
                       uint32_t last_ms, uint32_t now_ms)
{
    if (cur_hue == prev_hue) return 0;
    if ((uint32_t)(now_ms - last_ms) < WS2812_MIN_INTERVAL_MS) return 0;
    return 1;
}

ws2812_mode_t ws2812_mode(int relaying, uint32_t now_ms, uint32_t last_report_ms)
{
    if (!relaying) return WS2812_MODE_ERROR;                 /* fault overrides all */
    if ((uint32_t)(now_ms - last_report_ms) >= WS2812_IDLE_MS) return WS2812_MODE_IDLE;
    return WS2812_MODE_ACTIVE;
}

uint8_t ws2812_breathe_v(uint32_t now_ms, uint32_t period_ms,
                         uint8_t min_v, uint8_t max_v)
{
    if (period_ms == 0) return max_v;
    uint32_t half  = period_ms / 2u;
    if (half == 0) return max_v;
    uint32_t phase = now_ms % period_ms;                     /* 0 .. period-1 */
    /* Triangle: rise min→max over the first half, fall max→min over the second. */
    uint32_t up = (phase < half) ? phase : (period_ms - phase);  /* 0 .. half */
    uint32_t span = (uint32_t)(max_v - min_v);
    return (uint8_t)(min_v + (span * up) / half);
}

void ws2812_compose(ws2812_mode_t mode, uint8_t hue, uint32_t now_ms,
                    uint8_t out[3])
{
    switch (mode) {
        case WS2812_MODE_ERROR: {
            uint8_t v = ws2812_breathe_v(now_ms, WS2812_ERROR_PULSE_MS,
                                         WS2812_ERROR_MIN_V, WS2812_ERROR_MAX_V);
            ws2812_hsv_to_grb(0, 255, v, out);               /* hue 0 = red */
            break;
        }
        case WS2812_MODE_IDLE: {
            uint8_t v = ws2812_breathe_v(now_ms, WS2812_IDLE_BREATHE_MS,
                                         WS2812_IDLE_MIN_V, WS2812_BRIGHTNESS);
            ws2812_hsv_to_grb(hue, 255, v, out);
            break;
        }
        case WS2812_MODE_ACTIVE:
        default:
            ws2812_hsv_to_grb(hue, 255, WS2812_BRIGHTNESS, out);
            break;
    }
}

/* ── Hardware glue (PIOC) ─────────────────────────────────────────────────── */
#ifndef WS2812_HOSTTEST
#include "ch32h417.h"
#include <string.h>

/* RGB1W RAM-mode command bits (from the WCH RGB1W example / ch32h417_pioc.h). */
#define WS2812_CMD_RAM   0x80u           /* notice PIOC to send from code RAM     */
/* RAM-mode data buffer. NOTE: the 1152-byte (0x480) program loaded at
 * PIOC_SRAM_BASE overruns past +0x400, so its tail overlaps this buffer. Safe
 * ONLY because a frame is 3 bytes (well under the 0x80 overlap) and matches the
 * vendor demo's layout verbatim. Do NOT lengthen the frame (e.g. multi-pixel)
 * without relocating the buffer — it would clobber live program words. */
#define WS2812_RAM_ADDR  ((uint8_t *)(PIOC_SRAM_BASE + 0x400))

extern const uint8_t  ws2812_pioc_code[];
extern const unsigned ws2812_pioc_code_len;

/* Rainbow state (V5F-local; touched only from the V5F relay loop). */
static volatile uint8_t s_hue;
static uint8_t  s_last_hue_sent_active;  /* last hue pushed in ACTIVE mode        */
static uint32_t s_last_send_ms;      /* last PIOC push time (throttle clock)      */
static uint32_t s_last_report_ms;    /* last forwarded-report time (idle detect)  */
static uint8_t  s_started;           /* 0 until first push, so boot holds dark    */

void ws2812_init(void)
{
    GPIO_InitTypeDef g = {0};

    /* GPIOF + AFIO clock, then the PIOC clock. The RCC->HBPCENR write below is a
     * read-modify-write on a register shared with USBHS/DMA1 enables. It is safe
     * here ONLY because PIOC is enabled on V5F alone, once, before the relay hot
     * loop — V3F never enables PIOC. A second core RMW'ing HBPCENR concurrently
     * could tear the USBHS bit and kill enumeration (see spi_link.c hazard note). */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOF, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_PIOC, ENABLE);

    /* PF13 → PIOC IO1, AF5, push-pull, very-high speed. */
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF5);
    g.GPIO_Pin   = GPIO_Pin_13;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOF, &g);

    /* Reset PIOC, enable both IO switches, load the waveform program. */
    PIOC->D8_SYS_CFG = RB_MST_RESET | RB_MST_IO_EN0 | RB_MST_IO_EN1;
    memcpy((void *)PIOC_SRAM_BASE, ws2812_pioc_code, ws2812_pioc_code_len);

    s_hue = 0; s_last_hue_sent_active = 0;
    s_last_send_ms = 0; s_last_report_ms = 0; s_started = 0;
}

void ws2812_note_report(uint32_t now_ms)
{
    s_hue = (uint8_t)(s_hue + WS2812_HUE_STEP);   /* O(1) hue accumulator */
    s_last_report_ms = now_ms;                    /* reset the idle timer  */
}

/* Non-blocking RAM-mode send of a 3-byte GRB frame on IO1 (the example's
 * mod=1 path). Returns without sending if PIOC has not finished the prior
 * frame — never spins. */
static void ws2812_send_grb(const uint8_t grb[3])
{
    /* If a previous master-written command is still unread by PIOC, drop this
     * frame (bounded, non-spinning wedge guard). The dropped hue is re-sent on
     * the next hue change, not retried for this exact value — fine for a status
     * LED, and at the 16 ms throttle a ~30 µs frame is always long done. */
    if (PIOC->D8_SYS_CFG & RB_DATA_MW_SR) return;

    PIOC->D8_SYS_CFG = RB_MST_RESET | RB_MST_IO_EN0 | RB_MST_IO_EN1;   /* halt   */
    memcpy(WS2812_RAM_ADDR, grb, 3);                                    /* load   */
    PIOC->D8_SYS_CFG = RB_MST_CLK_GATE | RB_MST_IO_EN0 | RB_MST_IO_EN1; /* run    */
    PIOC->D16_DATA_REG0_1 = 3;                                          /* nbytes */
    PIOC->D8_DATA_REG2    = 1;                                          /* IO1    */
    PIOC->D8_CTRL_WR      = (uint8_t)(0x55 | WS2812_CMD_RAM);           /* start  */
}

void ws2812_service(uint32_t now_ms, int relaying)
{
    uint8_t hue = s_hue;
    ws2812_mode_t mode = ws2812_mode(relaying, now_ms, s_last_report_ms);

    /* ACTIVE pushes only when the hue advanced (idle-quiet rainbow). IDLE and
     * ERROR are time-driven brightness animations, so they push at the throttle
     * rate to keep breathing/pulsing. Either way, no more than ~60 Hz. */
    int want;
    if (mode == WS2812_MODE_ACTIVE) {
        want = !s_started ||
               (hue != s_last_hue_sent_active &&
                (uint32_t)(now_ms - s_last_send_ms) >= WS2812_MIN_INTERVAL_MS);
    } else {
        want = !s_started ||
               (uint32_t)(now_ms - s_last_send_ms) >= WS2812_MIN_INTERVAL_MS;
    }

    if (want) {
        uint8_t grb[3];
        ws2812_compose(mode, hue, now_ms, grb);
        ws2812_send_grb(grb);
        if (mode == WS2812_MODE_ACTIVE) s_last_hue_sent_active = hue;
        s_last_send_ms = now_ms;
        s_started      = 1;
    }
}
#endif /* WS2812_HOSTTEST */
