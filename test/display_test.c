#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "display.h"
#include "icc.h"

static void row_should_contain(const char *row, const char *needle) {
    if (!strstr(row, needle)) { printf("FAIL: '%s' lacks '%s'\n", row, needle); assert(0); }
}

int main(void) {
    char rows[DISP_ROWS][DISP_COLS + 1];

    // 1. NULL prev => all rows dirty.
    display_status_t st = {
        .state = DISP_STATE_RELAYING, .vid = 0x1A2C, .pid = 0x0094,
        .reports_per_sec = 980, .uptime_s = 252,
        .drops = 3, .zerolen = 1, .probe = 0x0C /*got|fwd*/, .gotmask = 0x3,
        .cmd_rx = 1240, .cmd_err = 0, .human_lvl = 3, .inj_m = 58, .inj_k = 0,
    };
    uint32_t dirty = display_format_lines(&st, rows, NULL);
    assert(dirty == ((1u << DISP_ROWS) - 1));

    // 2. Top block: state, dev IDs, rps, health, path, slots.
    row_should_contain(rows[ROW_STATE], "RELAY");
    row_should_contain(rows[ROW_IDS], "1A2C");
    row_should_contain(rows[ROW_IDS], "0094");
    row_should_contain(rows[ROW_RPS], "980");
    row_should_contain(rows[ROW_HEALTH], "drops 3"); // drops 3
    row_should_contain(rows[ROW_PATH], "GOT");      // probe bit3
    row_should_contain(rows[ROW_PATH], "FWD");      // probe bit2
    row_should_contain(rows[ROW_SLOTS], "0x3");      // gotmask 0x3

    // 3. Bottom block: uptime, cmd rx/err, human, inj.
    row_should_contain(rows[ROW_UPTIME], "4:12");   // 252s
    row_should_contain(rows[ROW_CMDRX], "1240");
    row_should_contain(rows[ROW_CMDERR], "0");
    row_should_contain(rows[ROW_HUMAN], "3");
    row_should_contain(rows[ROW_INJ], "58");

    // 4. No change => zero dirty.
    char prev[DISP_ROWS][DISP_COLS + 1];
    memcpy(prev, rows, sizeof rows);
    dirty = display_format_lines(&st, rows, (const char (*)[DISP_COLS+1])prev);
    assert(dirty == 0);

    // 5. Only rps changes => only ROW_RPS dirty.
    memcpy(prev, rows, sizeof rows);
    st.reports_per_sec = 12;
    dirty = display_format_lines(&st, rows, (const char (*)[DISP_COLS+1])prev);
    assert(dirty == (1u << ROW_RPS));

    // 6. WAITING (no device) => IDs and rps rows blank; bottom block still shows.
    display_status_t w = { .state = DISP_STATE_WAITING, .cmd_rx = 5, .human_lvl = 1 };
    display_format_lines(&w, rows, NULL);
    row_should_contain(rows[ROW_STATE], "WAIT");
    assert(rows[ROW_IDS][0] == '\0');
    assert(rows[ROW_RPS][0] == '\0');
    row_should_contain(rows[ROW_HUMAN], "1");

    // --- icc_status pack/unpack round-trip ---
    display_status_t src = { .state = DISP_STATE_RELAYING, .vid = 0x1A2C,
                             .pid = 0x0094, .reports_per_sec = 980,
                             .drops = 7, .probe = 0x0C, .gotmask = 0x3 };
    display_status_t acc = {0};
    for (uint8_t sel = 0; sel < ICC_ST_SEL__COUNT; sel++) {
        uint16_t w = icc_status_pack(sel, sel & 3, &src);
        icc_status_unpack(w, &acc);
    }
    assert(acc.state == DISP_STATE_RELAYING);
    assert(acc.vid == 0x1A2C);
    assert(acc.pid == 0x0094);
    assert(acc.reports_per_sec == 980);
    assert(acc.drops == 7);
    assert(acc.probe == 0x0C);
    assert(acc.gotmask == 0x3);
    assert(acc.zerolen == 0);   // probe bit0 = 0 here
    // seq is the high 2 bits.
    assert(icc_status_unpack(icc_status_pack(ICC_ST_SEL_STATE, 2, &src), &acc) == 2);

    // zerolen derived from probe bit0
    display_status_t z = { .probe = 0x0D /*got|fwd|zerolen*/, .gotmask = 0x1 };
    display_status_t za = {0};
    icc_status_unpack(icc_status_pack(ICC_ST_SEL_PROBE, 0, &z), &za);
    assert(za.probe == 0x0D);
    assert(za.gotmask == 0x1);
    assert(za.zerolen == 1);   // probe bit0 = 1

    // 7. Transition WAITING->RELAYING: ROW_IDS goes blank->populated (dirty).
    //    `w` is the WAITING status from case 6; `rows` currently holds its render.
    memcpy(prev, rows, sizeof rows);   // prev = WAITING rows (ROW_IDS blank)
    w.state = DISP_STATE_RELAYING; w.vid = 0x046D; w.pid = 0xC08B;
    dirty = display_format_lines(&w, rows, (const char (*)[DISP_COLS+1])prev);
    assert(dirty & (1u << ROW_IDS));
    row_should_contain(rows[ROW_IDS], "046D");
    row_should_contain(rows[ROW_IDS], "C08B");

    printf("display_test OK\n");
    return 0;
}
