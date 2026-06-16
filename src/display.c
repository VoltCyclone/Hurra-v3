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
    snprintf(rows[ROW_STATE], DISP_COLS + 1, "%s", state_name(st->state));

    bool have_dev = (st->state == DISP_STATE_RELAYING ||
                     st->state == DISP_STATE_CAPTURING);
    if (have_dev)
        snprintf(rows[ROW_IDS], DISP_COLS + 1, "%04X:%04X", st->vid, st->pid);
    else
        memset(rows[ROW_IDS], 0, DISP_COLS + 1);

    snprintf(rows[ROW_RPS], DISP_COLS + 1, "rps %u", (unsigned)st->reports_per_sec);

    unsigned m = (unsigned)(st->uptime_s / 60), s = (unsigned)(st->uptime_s % 60);
    snprintf(rows[ROW_UPTIME], DISP_COLS + 1, "up %u:%02u", m, s);

    rows[ROW_SPARE][0] = '\0';   // spare line (firmware tag/version added later)
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
