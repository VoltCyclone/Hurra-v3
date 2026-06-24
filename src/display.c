// display.c — status-to-text layout (pure) and hardware render.
#include "display.h"
#include "display_rowpick.h"
#include <stdio.h>
#include <string.h>
// Hardware headers are embedded-only; excluded from host unit tests.
#ifdef __riscv
#include "st7789.h"
#include "board.h"
#endif

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

static const char *speed_name(uint8_t s) {
    return (s == 2 /*USB_SPEED_HIGH*/) ? "HS" : "FS";
}

static void build_rows(const display_status_t *st, char rows[DISP_ROWS][DISP_COLS + 1]) {
    bool have_dev = (st->state == DISP_STATE_RELAYING ||
                     st->state == DISP_STATE_CAPTURING);

    // ROW_STATE (0): state name
    snprintf(rows[ROW_STATE], DISP_COLS + 1, "%s", state_name(st->state));

    // ROW_IDS (1): dev VID:PID + captured-device speed, blank if no device
    if (have_dev)
        snprintf(rows[ROW_IDS], DISP_COLS + 1, "dev %04X:%04X %s",
                 st->vid, st->pid, speed_name(st->cap_speed));
    else
        memset(rows[ROW_IDS], 0, DISP_COLS + 1);

    // ROW_RPS (2): reports/s, blank if no device
    if (have_dev)
        snprintf(rows[ROW_RPS], DISP_COLS + 1, "rps %u",
                 (unsigned)st->reports_per_sec);
    else
        memset(rows[ROW_RPS], 0, DISP_COLS + 1);

    // ROW_HDR_HOST (3)
    snprintf(rows[ROW_HDR_HOST], DISP_COLS + 1, "%s", "--- HOST (B) ---");

    // ROW_LINK (4): SPI master health
    snprintf(rows[ROW_LINK], DISP_COLS + 1, "link %s wedge %u",
             st->wedge ? "WARN" : "OK", (unsigned)st->wedge);

    // ROW_HTEMP (5): host board temp
    snprintf(rows[ROW_HTEMP], DISP_COLS + 1, "temp %d C", (int)st->temp_c);

    // ROW_HDR_DEV (6)
    snprintf(rows[ROW_HDR_DEV], DISP_COLS + 1, "%s", "-- DEVICE (A) --");

    // ROW_PCENUM (7): clone enumerated on PC + clone speed; "--" when link stale
    if (!st->dev_link)
        snprintf(rows[ROW_PCENUM], DISP_COLS + 1, "PC: --");
    else
        snprintf(rows[ROW_PCENUM], DISP_COLS + 1, "PC: %s %s",
                 st->dev_enum ? "ENUM" : "no", speed_name(st->dev_speed));

    // ROW_DTEMP (8): device temp; "--" when link stale
    if (!st->dev_link)
        snprintf(rows[ROW_DTEMP], DISP_COLS + 1, "temp -- C");
    else
        snprintf(rows[ROW_DTEMP], DISP_COLS + 1, "temp %d C", (int)st->dev_temp_c);

    // ROW_DLINK (9): banner only when device telemetry is stale
    if (!st->dev_link)
        snprintf(rows[ROW_DLINK], DISP_COLS + 1, "%s", "LINK DOWN");
    else
        memset(rows[ROW_DLINK], 0, DISP_COLS + 1);

    // ROW_HUMAN (10): humanization residual fill + non-human-source flag
    {
        const char *wn = (st->human_warmth >= 2) ? "WARM"
                       : (st->human_warmth == 1) ? "WARMING" : "COLD";
        if (st->human_replay_pct == 0 && st->human_warmth >= 2)
            snprintf(rows[ROW_HUMAN], DISP_COLS + 1, "hum NONHUMAN SRC");
        else
            snprintf(rows[ROW_HUMAN], DISP_COLS + 1, "hum %s res %u%%",
                     wn, (unsigned)st->human_replay_pct);
    }

    // Rows 11..12 render blank.
    for (int r = 11; r < DISP_ROWS; r++) memset(rows[r], 0, DISP_COLS + 1);
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

#ifdef __riscv
static uint16_t temp_color(int t) {
    if (t > 65) return ST_RED;
    if (t >= 50) return ST_YELLOW;
    return ST_GREEN;
}

void display_init(void)
{
    st7789_init();   // includes a black clear
}

void display_render(const display_status_t *st)
{
    static char prev[DISP_ROWS][DISP_COLS + 1];
    static bool have_prev;
    static uint8_t start_row;        // rotating scan cursor (anti-starvation)
    char rows[DISP_ROWS][DISP_COLS + 1];
    uint32_t dirty = display_format_lines(st, rows,
                        have_prev ? (const char (*)[DISP_COLS + 1])prev : 0);
    have_prev = true;

    // Paint at most one dirty row per call so a multi-row change (e.g. a state
    // transition, or the first full render) can't block the V3F command loop in
    // a single fill_rect+draw_string burst. The rotating cursor guarantees every
    // row eventually repaints; rows not painted this call stay dirty because
    // only the painted row is copied into prev[].
    int r = display_pick_row(dirty, start_row, DISP_ROWS);
    if (r < 0)
        return;                      // nothing changed
    start_row = (uint8_t)((r + 1) % DISP_ROWS);

    uint16_t y = (uint16_t)(r * 8 * DISP_SCALE);
    // Clear the row band before drawing text.
    st7789_fill_rect(0, y, LCD_WIDTH, (uint16_t)(y + 8 * DISP_SCALE), ST_BLACK);
    uint16_t fg = (st->state == DISP_STATE_RELAYING) ? ST_GREEN :
                  (st->state == DISP_STATE_ERROR ||
                   st->state == DISP_STATE_NOSIGNAL) ? ST_RED : ST_WHITE;
    if (r == ROW_HTEMP) fg = temp_color((int)st->temp_c);
    else if (r == ROW_DTEMP) fg = st->dev_link ? temp_color((int)st->dev_temp_c) : ST_WHITE;
    st7789_draw_string(0, y, rows[r], fg, ST_BLACK, DISP_SCALE);

    memcpy(prev[r], rows[r], DISP_COLS + 1);   // only the painted row is now clean
}
#else  /* host build: display is hardware-only, no-op stubs */
void display_init(void) { }
void display_render(const display_status_t *st) { (void)st; }
#endif /* __riscv */
