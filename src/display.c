// display.c — status->text layout (pure) + hardware render (Tasks 4/6).
#include "display.h"
#include <stdio.h>
#include <string.h>
// Hardware headers are RISC-V / embedded only — exclude from host unit tests.
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

// Build all 14 rows. Each buffer is clamped to DISP_COLS by snprintf/memset.
static void build_rows(const display_status_t *st, char rows[DISP_ROWS][DISP_COLS + 1]) {
    // ROW_STATE (0): state name
    snprintf(rows[ROW_STATE], DISP_COLS + 1, "%s", state_name(st->state));

    bool have_dev = (st->state == DISP_STATE_RELAYING ||
                     st->state == DISP_STATE_CAPTURING);

    // ROW_IDS (1): dev VID:PID, blank if no device
    if (have_dev)
        snprintf(rows[ROW_IDS], DISP_COLS + 1, "dev %04X:%04X", st->vid, st->pid);
    else
        memset(rows[ROW_IDS], 0, DISP_COLS + 1);

    // ROW_RPS (2): reports/s, blank if no device
    if (have_dev)
        snprintf(rows[ROW_RPS], DISP_COLS + 1, "reports/s %u",
                 (unsigned)st->reports_per_sec);
    else
        memset(rows[ROW_RPS], 0, DISP_COLS + 1);

    // ROW_HEALTH (4): drops and zero-length count
    snprintf(rows[ROW_HEALTH], DISP_COLS + 1, "drops %u  zlen %u",
             (unsigned)st->drops, (unsigned)st->zerolen);

    // ROW_PATH (5): decode probe bits into flag string
    {
        char tmp[DISP_COLS + 1];
        int pos = 0;
        pos += snprintf(tmp + pos, sizeof(tmp) - (size_t)pos, "path");
        if ((st->probe & 0x8) && pos < DISP_COLS)
            pos += snprintf(tmp + pos, sizeof(tmp) - (size_t)pos, " GOT");
        if ((st->probe & 0x4) && pos < DISP_COLS)
            pos += snprintf(tmp + pos, sizeof(tmp) - (size_t)pos, " FWD");
        if ((st->probe & 0x2) && pos < DISP_COLS)
            pos += snprintf(tmp + pos, sizeof(tmp) - (size_t)pos, " DROP");
        if ((st->probe & 0x1) && pos < DISP_COLS)
            pos += snprintf(tmp + pos, sizeof(tmp) - (size_t)pos, " ZLEN");
        snprintf(rows[ROW_PATH], DISP_COLS + 1, "%s", tmp);
    }

    // ROW_SLOTS (6): which host IN slots delivered
    snprintf(rows[ROW_SLOTS], DISP_COLS + 1, "slots 0x%X",
             (unsigned)(st->gotmask & 0x0F));

    // ROW_DIV (7): divider — 20 dashes (DISP_COLS=20)
    snprintf(rows[ROW_DIV], DISP_COLS + 1, "%s", "--------------------");

    // ROW_UPTIME (8): uptime in M:SS
    {
        unsigned m = (unsigned)(st->uptime_s / 60);
        unsigned s = (unsigned)(st->uptime_s % 60);
        snprintf(rows[ROW_UPTIME], DISP_COLS + 1, "uptime %u:%02u", m, s);
    }

    // ROW_CMDRX (9): USART rx byte count
    snprintf(rows[ROW_CMDRX], DISP_COLS + 1, "cmd rx %u B",
             (unsigned)st->cmd_rx);

    // ROW_CMDERR (10): USART error count
    snprintf(rows[ROW_CMDERR], DISP_COLS + 1, "cmd err %u",
             (unsigned)st->cmd_err);

    // ROW_HUMAN (11): humanize level
    snprintf(rows[ROW_HUMAN], DISP_COLS + 1, "human lvl %u",
             (unsigned)st->human_lvl);

    // ROW_INJ (12): injection counts
    snprintf(rows[ROW_INJ], DISP_COLS + 1, "inj m %u  k %u",
             (unsigned)st->inj_m, (unsigned)st->inj_k);

    // ROW_TEMP (13): board temperature (V3F-local ADC)
    snprintf(rows[ROW_TEMP], DISP_COLS + 1, "temp %d C", (int)st->temp_c);
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
#else  /* !__riscv : host build — display is hardware-only, provide no-op stubs */
void display_init(void) { }
void display_render(const display_status_t *st) { (void)st; }
#endif /* __riscv */
