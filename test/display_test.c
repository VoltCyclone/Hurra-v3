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
        .state = DISP_STATE_RELAYING, .vid = 0x046D, .pid = 0xC08B,
        .reports_per_sec = 980, .cap_speed = 2 /*HS*/, .temp_c = 41, .wedge = 0,
        .dev_enum = 1, .dev_speed = 2 /*HS*/, .dev_temp_c = 39, .dev_link = 1,
    };
    uint32_t dirty = display_format_lines(&st, rows, NULL);
    assert(dirty == ((1u << DISP_ROWS) - 1));

    // 2. Top block: state, dev IDs + captured speed, rps.
    row_should_contain(rows[ROW_STATE], "RELAY");
    row_should_contain(rows[ROW_IDS], "046D");
    row_should_contain(rows[ROW_IDS], "C08B");
    row_should_contain(rows[ROW_IDS], "HS");      // cap_speed = HIGH
    row_should_contain(rows[ROW_RPS], "980");

    // 3. Host block: header, link+wedge, host temp.
    row_should_contain(rows[ROW_HDR_HOST], "HOST");
    row_should_contain(rows[ROW_LINK], "wedge 0");
    row_should_contain(rows[ROW_HTEMP], "41");

    // 3b. Device block: header, PC enum + clone speed, device temp, no LINK DOWN.
    row_should_contain(rows[ROW_HDR_DEV], "DEVICE");
    row_should_contain(rows[ROW_PCENUM], "ENUM");
    row_should_contain(rows[ROW_PCENUM], "HS");   // dev_speed = HIGH
    row_should_contain(rows[ROW_DTEMP], "39");
    assert(rows[ROW_DLINK][0] == '\0');           // link fresh => banner blank

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

    // 6. Device link stale => ROW_DLINK shows LINK DOWN; PC/temp show "--".
    display_status_t down = { .state = DISP_STATE_RELAYING, .vid = 0x046D,
                              .pid = 0xC08B, .cap_speed = 2, .temp_c = 40,
                              .dev_link = 0 };
    display_format_lines(&down, rows, NULL);
    row_should_contain(rows[ROW_DLINK], "LINK DOWN");
    row_should_contain(rows[ROW_PCENUM], "--");
    row_should_contain(rows[ROW_DTEMP], "--");

    // --- icc_status pack/unpack round-trip ---
    display_status_t src = { .state = DISP_STATE_RELAYING, .vid = 0x1A2C,
                             .pid = 0x0094, .reports_per_sec = 980,
                             .drops = 7, .probe = 0x0C, .gotmask = 0x3 };
    display_status_t acc = {0};
    for (uint8_t sel = 0; sel < ICC_ST_SEL__COUNT; sel++) {
        uint16_t word = icc_status_pack(sel, sel & 3, &src);
        icc_status_unpack(word, &acc);
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

    // --- new two-board selectors round-trip ---
    display_status_t nsrc = { .wedge = 500, .cap_speed = 2 /*HS*/,
                              .dev_enum = 1, .dev_speed = 0 /*FS*/,
                              .dev_temp_c = -5, .dev_link = 1 };
    display_status_t nacc = {0};
    icc_status_unpack(icc_status_pack(ICC_ST_SEL_WEDGE,  0, &nsrc), &nacc);
    icc_status_unpack(icc_status_pack(ICC_ST_SEL_SPEEDS, 1, &nsrc), &nacc);
    icc_status_unpack(icc_status_pack(ICC_ST_SEL_DEV,    2, &nsrc), &nacc);
    assert(nacc.wedge == 500);
    assert(nacc.cap_speed == 2);
    assert(nacc.dev_speed == 0);
    assert(nacc.dev_enum == 1);
    assert(nacc.dev_link == 1);
    assert(nacc.dev_temp_c == -5);

    /* ── HUMAN selector round-trip + row ── */
    {
        display_status_t st; memset(&st, 0, sizeof st);
        st.state = DISP_STATE_RELAYING;
        st.human_warmth = 2;          /* WARM */
        st.human_replay_pct = 98;
        uint16_t w = icc_status_pack(ICC_ST_SEL_HUMAN, 1, &st);
        display_status_t acc; memset(&acc, 0, sizeof acc);
        icc_status_unpack(w, &acc);
        if (!(acc.human_warmth == 2)) { printf("FAIL: HUMAN warmth survives round-trip\n"); assert(0); }
        if (!(acc.human_replay_pct == 98)) { printf("FAIL: HUMAN replay%% survives round-trip\n"); assert(0); }

        /* Row 10 renders warmth + replay%. */
        char rows[DISP_ROWS][DISP_COLS + 1];
        display_format_lines(&st, rows, NULL);
        if (!strstr(rows[ROW_HUMAN], "WARM")) { printf("FAIL: ROW_HUMAN shows warmth name\n"); assert(0); }
        if (!strstr(rows[ROW_HUMAN], "98"))   { printf("FAIL: ROW_HUMAN shows replay%%\n"); assert(0); }
        if (!strstr(rows[ROW_HUMAN], "res"))  { printf("FAIL: ROW_HUMAN shows res keyword\n"); assert(0); }
    }

    printf("display_test OK\n");
    return 0;
}
