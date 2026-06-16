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
