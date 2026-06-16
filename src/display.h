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

// Row indices into the DISP_ROWS text grid.
enum { ROW_STATE = 0, ROW_IDS = 1, ROW_RPS = 2, ROW_UPTIME = 3, ROW_SPARE = 4 };

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
