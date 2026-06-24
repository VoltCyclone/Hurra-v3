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
    uint8_t  state;            // disp_state_t
    uint16_t vid;
    uint16_t pid;
    uint16_t reports_per_sec;
    uint32_t uptime_s;         // V3F-local
    // --- relay health (from V5F telemetry) ---
    uint16_t drops;            // forwarded-report drop count
    uint8_t  zerolen;          // 1 if any zero-length poll seen this window
    uint8_t  probe;            // packed: bit3 got, bit2 fwd, bit1 drop, bit0 zerolen
    uint8_t  gotmask;          // which host IN slots delivered (bits 0..3)
    // --- V3F-local stats ---
    uint32_t cmd_rx;           // USART rx byte count
    uint16_t cmd_err;          // overrun+framing+noise total
    uint8_t  human_lvl;        // humanize level 0..3
    uint16_t inj_m;            // mouse injection count
    uint16_t inj_k;            // keyboard injection count
    // --- board temperature (V3F-local ADC) ---
    int8_t   temp_c;           // degrees Celsius (signed)
    // --- two-board: host-side relay link health (host-local) ---
    uint16_t wedge;            // SPI master wedge/recovery count
    uint8_t  cap_speed;        // captured-device speed (USB_SPEED_*)
    // --- two-board: device-board (Board A) status, via SPI return slot ---
    uint8_t  dev_enum;         // 1 = clone configured on the PC
    uint8_t  dev_speed;        // clone->PC speed (USB_SPEED_*)
    int8_t   dev_temp_c;       // device board temperature (deg C)
    uint8_t  dev_link;         // 1 = device telemetry fresh, 0 = stale
    // --- humanization v2 engine status (from V5F over ICC) ---
    uint8_t  human_warmth;     // gst_warmth_t 0 COLD / 1 WARMING / 2 WARM
    uint8_t  human_replay_pct; // replay share of injected motion, 0..100
} display_status_t;

// Text grid. Scale 2 => 12px glyphs; 240px / 12px = 20 cols.
// 13 rows * 16px = 208px; fits the 240px display height.
#define DISP_COLS   20
#define DISP_ROWS   13
#define DISP_SCALE   2

// Row indices. Top block (0..2) = relay state/device-ids/rps (host-local).
// Row 3 = HOST header; 4..5 = host link health + host temp.
// Row 6 = DEVICE header; 7..9 = clone enum/speed, device temp, link-down banner.
// Rows 10..12 render blank.
enum {
    ROW_STATE    = 0,   // relay state name (colored)
    ROW_IDS      = 1,   // dev VID:PID + captured-device speed
    ROW_RPS      = 2,   // reports/s
    ROW_HDR_HOST = 3,   // "--- HOST (B) ---"
    ROW_LINK     = 4,   // "link OK  wedge N"
    ROW_HTEMP    = 5,   // host board temp (colored by value)
    ROW_HDR_DEV  = 6,   // "--- DEVICE (A) ---"
    ROW_PCENUM   = 7,   // "PC: ENUM  HS" / "PC: --"
    ROW_DTEMP    = 8,   // device temp (colored) / "--"
    ROW_DLINK    = 9,   // "LINK DOWN" only when stale, else blank
    ROW_HUMAN    = 10,  // "hum WARM rpl 98%"
};

// Render `st` into `rows` (DISP_ROWS NUL-terminated strings, each <=DISP_COLS
// chars). Returns a dirty bitmask (bit r set = row r differs from `prev_rows`);
// `prev_rows` may be NULL (all rows dirty). Caller copies rows into prev. Pure.
uint32_t display_format_lines(const display_status_t *st,
                              char rows[DISP_ROWS][DISP_COLS + 1],
                              const char prev_rows[DISP_ROWS][DISP_COLS + 1]);

// Hardware entry points.
void display_init(void);
void display_render(const display_status_t *st);
