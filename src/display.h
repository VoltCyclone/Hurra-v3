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
    // --- product name (streamed from V5F, ASCII, <=16 chars) ---
    char     name[17];         // NUL-terminated, uppercased 6-bit charset
    // --- board temperature (V3F-local ADC) ---
    int8_t   temp_c;           // degrees Celsius (signed)
} display_status_t;

// Text grid. Scale 2 => 12px glyphs; 240px / 12px = 20 cols.
// 14 rows * 16px = 224px; fits the 240px display height.
#define DISP_COLS   20
#define DISP_ROWS   14
#define DISP_SCALE   2

// Row indices. Top block (0..6) = V5F relay telemetry; row 7 divider;
// bottom block (8..13) = V3F-local stats. Blank rows render as empty bands.
enum {
    ROW_STATE  = 0,   // relay state name
    ROW_IDS    = 1,   // dev VID:PID (blank if no device)
    ROW_NAME   = 2,   // product name (streamed from V5F)
    ROW_RPS    = 3,   // reports/s (blank if no device)
    ROW_HEALTH = 4,   // drops N  zlen N
    ROW_PATH   = 5,   // decoded probe: GOT/FWD/DROP/ZLEN flags
    ROW_SLOTS  = 6,   // which host IN slots delivered (gotmask)
    ROW_DIV    = 7,   // divider
    ROW_UPTIME = 8,   // uptime M:SS  (V3F-local)
    ROW_CMDRX  = 9,   // cmd rx N B   (V3F-local)
    ROW_CMDERR = 10,  // cmd err N    (V3F-local)
    ROW_HUMAN  = 11,  // human lvl N  (V3F-local)
    ROW_INJ    = 12,  // inj m N  k N (V3F-local)
    ROW_TEMP   = 13,  // board temp (V3F-local ADC)
};

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
