# Two-Board TFT Cleanup & Host Telemetry Pass-Through Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the device-board TFT, revive host-side relay telemetry on the host TFT, and pass device-board status (clone-enum, speed, temp) back over the SPI return slot.

**Architecture:** Both boards flash the same V3F image (built per-role via `-DBOARD_ROLE_*`); V3F drives the ST7789. Host-local telemetry rides the existing V5F→V3F IPC reverse status channel; device telemetry rides the SPI MISO return slot as `spi_frame` slots staged by an extended RXNE ISR, is decoded by the host V5F, and merges into the same IPC channel. The host TFT renders a HOST block and a DEVICE block.

**Tech Stack:** C11, RISC-V dual-core (CH32H417 V3F + V5F), WCH StdPeriph, GNU make. Host unit tests compile pure modules with `cc` and run on the dev machine.

## Global Constraints

- **Display geometry:** `DISP_COLS = 20`, `DISP_ROWS = 13`, `DISP_SCALE = 2` (do not change; tied to 240×240 panel and 5×7 font). Each row string is `<= DISP_COLS` chars, NUL-terminated.
- **Pure modules stay pure:** `display.c`, `icc_status.c`, `spi_frame*.c` include stdint/stdbool/string only — NO MMIO, NO WCH headers. Hardware code is `#ifdef __riscv`.
- **Reverse IPC status word:** 16-bit, `[15:14]=seq`, `[13:10]=selector`, `[9:0]=payload`. Selectors are a 4-bit enum (max 16). 8 are used today; new ones must keep total ≤ 16.
- **USB speed encoding:** `USB_SPEED_FULL=0`, `USB_SPEED_LOW=1`, `USB_SPEED_HIGH=2` (from `usb_host.h`). Display shows `FS` for FULL/LOW, `HS` for HIGH.
- **SPI slot:** fixed 32 bytes (`SPI_LINK_SLOT` = `SPI_FRAME_SLOT_SIZE`). Telemetry reuses `spi_frame_pack`/`spi_frame_unpack` (SOF 0x68, CRC16/CCITT-FALSE).
- **Temp color thresholds:** green `< 50 °C`, amber `50..65 °C` inclusive, red `> 65 °C`. Whole temp row colored by value.
- **Commit message trailer:** end every commit body with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- **Test runner:** `make test` builds & runs all host unit tests; the display test line is `cc -std=c11 -O2 -Isrc -o /tmp/display_test test/display_test.c src/display.c src/icc_status.c && /tmp/display_test`.

---

### Task 1: Extend `display_status_t` and add new selectors (pure)

Adds the six new display fields and the new reverse-channel selectors. No layout change yet — this task only widens the data model and the pack/unpack codec so later tasks have types to consume.

**Files:**
- Modify: `src/display.h` (add fields to `display_status_t`)
- Modify: `src/icc.h` (add selectors to the `ICC_ST_SEL_*` enum)
- Modify: `src/icc_status.c` (`icc_status_pack` / `icc_status_unpack` cases)
- Test: `test/display_test.c` (extend the pack/unpack round-trip)

**Interfaces:**
- Consumes: existing `display_status_t`, `icc_status_pack(sel, seq, st)`, `icc_status_unpack(word, acc)`.
- Produces: new fields `wedge` (uint16_t), `cap_speed` (uint8_t), `dev_enum` (uint8_t), `dev_speed` (uint8_t), `dev_temp_c` (int8_t), `dev_link` (uint8_t); new selectors `ICC_ST_SEL_WEDGE`, `ICC_ST_SEL_SPEEDS`, `ICC_ST_SEL_DEV` (still ≤ 16 total).

- [ ] **Step 1: Add fields to `display_status_t`**

In `src/display.h`, inside `typedef struct { ... } display_status_t;`, after the `int8_t temp_c;` line, add:

```c
    // --- two-board: host-side relay link health (host-local) ---
    uint16_t wedge;            // SPI master wedge/recovery count
    uint8_t  cap_speed;        // captured-device speed (USB_SPEED_*)
    // --- two-board: device-board (Board A) status, via SPI return slot ---
    uint8_t  dev_enum;         // 1 = clone configured on the PC
    uint8_t  dev_speed;        // clone->PC speed (USB_SPEED_*)
    int8_t   dev_temp_c;       // device board temperature (deg C)
    uint8_t  dev_link;         // 1 = device telemetry fresh, 0 = stale
```

- [ ] **Step 2: Add selectors to the `ICC_ST_SEL_*` enum**

In `src/icc.h`, in the field-selector enum, add the three new selectors immediately before `ICC_ST_SEL__COUNT`:

```c
    ICC_ST_SEL_WEDGE,           // payload[9:0] = wedge (clamped 0..1023)
    ICC_ST_SEL_SPEEDS,          // payload[3:2]=cap_speed, payload[1:0]=dev_speed
    ICC_ST_SEL_DEV,             // payload[9]=dev_link, payload[8]=dev_enum, payload[7:0]=dev_temp_c
    ICC_ST_SEL__COUNT
```

- [ ] **Step 3: Write the failing pack/unpack test**

In `test/display_test.c`, after the existing `// zerolen derived from probe bit0` block (around line 89, before case 7), insert:

```c
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
```

- [ ] **Step 4: Run the test to verify it fails**

Run: `make test 2>&1 | sed -n '/display_test/,/test OK/p'` (or directly the display_test cc line from Global Constraints).
Expected: FAIL — compile error (unknown `ICC_ST_SEL_WEDGE` / struct fields) or assertion failure.

- [ ] **Step 5: Implement the pack cases**

In `src/icc_status.c`, in `icc_status_pack`'s `switch (sel)`, add before `default:`:

```c
        case ICC_ST_SEL_WEDGE:  payload = (st->wedge > 1023u) ? 1023u : st->wedge; break;
        case ICC_ST_SEL_SPEEDS: payload = (uint16_t)(((st->cap_speed & 0x3u) << 2)
                                                     | (st->dev_speed & 0x3u)); break;
        case ICC_ST_SEL_DEV:    payload = (uint16_t)(((st->dev_link & 0x1u) << 9)
                                                     | ((st->dev_enum & 0x1u) << 8)
                                                     | ((uint8_t)st->dev_temp_c)); break;
```

- [ ] **Step 6: Implement the unpack cases**

In `src/icc_status.c`, in `icc_status_unpack`'s `switch (sel)`, add before `default:`:

```c
        case ICC_ST_SEL_WEDGE:  acc->wedge = payload; break;
        case ICC_ST_SEL_SPEEDS:
            acc->cap_speed = (uint8_t)((payload >> 2) & 0x3u);
            acc->dev_speed = (uint8_t)(payload & 0x3u);
            break;
        case ICC_ST_SEL_DEV:
            acc->dev_link  = (uint8_t)((payload >> 9) & 0x1u);
            acc->dev_enum  = (uint8_t)((payload >> 8) & 0x1u);
            acc->dev_temp_c = (int8_t)(payload & 0xFFu);
            break;
```

- [ ] **Step 7: Run the test to verify it passes**

Run: `make test 2>&1 | grep display_test`
Expected: `display_test OK`

- [ ] **Step 8: Commit**

```bash
git add src/display.h src/icc.h src/icc_status.c test/display_test.c
git commit -m "feat(display): add two-board status fields + reverse-channel selectors

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: New HOST/DEVICE row layout (pure)

Replaces the relay/local row layout with the HOST/DEVICE blocks. This is the largest pure change: new row enum, new `build_rows`, and a rewrite of the existing layout assertions in the test.

**Files:**
- Modify: `src/display.h` (row enum)
- Modify: `src/display.c` (`build_rows`, `state_name` unchanged)
- Test: `test/display_test.c` (replace layout assertions; keep pack/unpack)

**Interfaces:**
- Consumes: `display_status_t` with all fields from Task 1; `DISP_COLS`, `DISP_ROWS`.
- Produces: row enum `ROW_STATE=0, ROW_IDS, ROW_RPS, ROW_HDR_HOST, ROW_LINK, ROW_HTEMP, ROW_HDR_DEV, ROW_PCENUM, ROW_DTEMP, ROW_DLINK` (10 rows; 10..12 blank); helper `static const char *speed_name(uint8_t)` returning `"FS"`/`"HS"`.

- [ ] **Step 1: Replace the row index enum**

In `src/display.h`, replace the entire `enum { ROW_STATE ... ROW_TEMP ... };` block with:

```c
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
};
```

- [ ] **Step 2: Write the failing layout test**

Replace the body of `test/display_test.c` cases 1–3 and case 6–7 (the layout assertions; KEEP the pack/unpack round-trip block from Task 1 and the original round-trip block). Replace lines from the `// 1. NULL prev` comment through the end of case 3 (the `row_should_contain(rows[ROW_SLOTS], "0x3");` line) with:

```c
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
```

Then replace case 6 (the `WAITING` block) entirely with:

```c
    // 6. Device link stale => ROW_DLINK shows LINK DOWN; PC/temp show "--".
    display_status_t down = { .state = DISP_STATE_RELAYING, .vid = 0x046D,
                              .pid = 0xC08B, .cap_speed = 2, .temp_c = 40,
                              .dev_link = 0 };
    display_format_lines(&down, rows, NULL);
    row_should_contain(rows[ROW_DLINK], "LINK DOWN");
    row_should_contain(rows[ROW_PCENUM], "--");
    row_should_contain(rows[ROW_DTEMP], "--");
```

Finally delete case 7 (the WAITING→RELAYING `ROW_IDS` transition block, lines ~91–98) — it referenced the old blanking behavior; the dirty-diff mechanism is already covered by cases 4–5.

- [ ] **Step 3: Run the test to verify it fails**

Run: `make test 2>&1 | grep -A2 display_test`
Expected: FAIL — `ROW_HDR_HOST` / `ROW_PCENUM` undefined, or missing strings.

- [ ] **Step 4: Rewrite `build_rows`**

In `src/display.c`, add a speed helper after `state_name`:

```c
static const char *speed_name(uint8_t s) {
    return (s == 2 /*USB_SPEED_HIGH*/) ? "HS" : "FS";
}
```

Replace the entire `build_rows` function body with:

```c
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

    // Rows 10..12 render blank.
    for (int r = 10; r < DISP_ROWS; r++) memset(rows[r], 0, DISP_COLS + 1);
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `make test 2>&1 | grep display_test`
Expected: `display_test OK`

- [ ] **Step 6: Commit**

```bash
git add src/display.h src/display.c test/display_test.c
git commit -m "feat(display): HOST/DEVICE block layout for the two-board host TFT

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: Temp-row color by value (hardware render)

Colors the two temp rows by temperature. This is in the `#ifdef __riscv` render path (not host-tested); the gate is compile + bench. Keep it tiny.

**Files:**
- Modify: `src/display.c` (`display_render`, inside `#ifdef __riscv`)

**Interfaces:**
- Consumes: `display_status_t.temp_c`, `.dev_temp_c`, `.dev_link`; row enum `ROW_HTEMP`, `ROW_DTEMP`; `ST_GREEN/ST_YELLOW/ST_RED/ST_WHITE` from `st7789.h`.
- Produces: none (leaf render change).

- [ ] **Step 1: Add a temp-color helper**

In `src/display.c`, inside the `#ifdef __riscv` block, above `display_render`, add:

```c
static uint16_t temp_color(int t) {
    if (t > 65) return ST_RED;
    if (t >= 50) return ST_YELLOW;
    return ST_GREEN;
}
```

- [ ] **Step 2: Use it for the temp rows in `display_render`**

In `src/display.c`, in `display_render`, replace the foreground-color computation and draw so temp rows override the state color. Replace this block:

```c
        uint16_t fg = (st->state == DISP_STATE_RELAYING) ? ST_GREEN :
                      (st->state == DISP_STATE_ERROR ||
                       st->state == DISP_STATE_NOSIGNAL) ? ST_RED : ST_WHITE;
        st7789_draw_string(0, y, rows[r], fg, ST_BLACK, DISP_SCALE);
```

with:

```c
        uint16_t fg = (st->state == DISP_STATE_RELAYING) ? ST_GREEN :
                      (st->state == DISP_STATE_ERROR ||
                       st->state == DISP_STATE_NOSIGNAL) ? ST_RED : ST_WHITE;
        if (r == ROW_HTEMP) fg = temp_color((int)st->temp_c);
        else if (r == ROW_DTEMP) fg = st->dev_link ? temp_color((int)st->dev_temp_c) : ST_WHITE;
        st7789_draw_string(0, y, rows[r], fg, ST_BLACK, DISP_SCALE);
```

- [ ] **Step 3: Verify the V3F image still compiles**

Run: `make v3f 2>&1 | tail -5`
Expected: builds to `build/v3f.elf` with a size line, no errors.

- [ ] **Step 4: Commit**

```bash
git add src/display.c
git commit -m "feat(display): color temp rows by value (green/amber/red)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: `DISPLAY_PRESENT` gate — strip the device TFT

Makes the TFT host-only. The device V3F builds with no display init/render; `st7789.c`/`display.c` dead-strip from the device image.

**Files:**
- Modify: `Makefile` (define `-DDISPLAY_PRESENT` for `BOARD=host`)
- Modify: `src/main_v3f.c` (guard display calls)

**Interfaces:**
- Consumes: `BOARD_DEF` mechanism in the Makefile; `display_init()`, `display_render()`, `icc_status_poll_v3f()`.
- Produces: compile-time macro `DISPLAY_PRESENT` (defined only for host role).

- [ ] **Step 1: Define `DISPLAY_PRESENT` for the host role**

In `Makefile`, in the `ifeq ($(BOARD),host)` branch (the `BOARD_DEF` block near line 40), change:

```makefile
ifeq ($(BOARD),host)
  BOARD_DEF = -DBOARD_ROLE_HOST
```

to:

```makefile
ifeq ($(BOARD),host)
  BOARD_DEF = -DBOARD_ROLE_HOST -DDISPLAY_PRESENT
```

Note: the single-board relay build (`make relay`, empty `BOARD`) gets no `DISPLAY_PRESENT` and so would lose its display. To preserve it, also add `DISPLAY_PRESENT` when no role is set. Change the `else ifneq ($(BOARD),)` / `endif` tail to:

```makefile
else ifneq ($(BOARD),)
  $(error BOARD must be 'host' or 'device' (empty = single-board relay))
else
  BOARD_DEF = -DDISPLAY_PRESENT
endif
```

- [ ] **Step 2: Guard the display calls in `main_v3f.c`**

In `src/main_v3f.c`, wrap `display_init()` (line ~186) and its status struct, and the render block. Change:

```c
    display_init();
    display_status_t g_disp = { .state = DISP_STATE_BOOT };
    uint32_t disp_render_tick = millis();
```

to:

```c
#ifdef DISPLAY_PRESENT
    display_init();
#endif
    display_status_t g_disp = { .state = DISP_STATE_BOOT };
    uint32_t disp_render_tick = millis();
    (void)disp_render_tick;
```

(`g_disp` stays unconditional — the `icc_status_poll_v3f` call still consumes it harmlessly on the device, and keeping it avoids threading more guards through the loop.)

Then guard the render. The poll+render block computes local stats and calls `display_render`. Wrap only the *render* portion. Change the final part of that block:

```c
            if (!s_seen_advance && g_disp.state != DISP_STATE_BOOT)
                g_disp.state = DISP_STATE_NOSIGNAL;
            s_seen_advance = false;
            display_render(&g_disp);
        }
```

to:

```c
            if (!s_seen_advance && g_disp.state != DISP_STATE_BOOT)
                g_disp.state = DISP_STATE_NOSIGNAL;
            s_seen_advance = false;
#ifdef DISPLAY_PRESENT
            display_render(&g_disp);
#endif
        }
```

- [ ] **Step 3: Verify host V3F still builds (display present)**

Run: `make v3f BOARD=host 2>&1 | tail -3`
Expected: builds, no errors.

- [ ] **Step 4: Verify device V3F builds and dead-strips the display**

Run:
```bash
make v3f BOARD=device 2>&1 | tail -3
riscv-none-elf-nm build/v3f.elf 2>/dev/null | grep -c st7789_init || echo "st7789 absent"
```
Expected: builds; `st7789_init` count is `0` (or "st7789 absent") — confirming the panel driver dead-stripped. If `nm` isn't on PATH, fall back to `make v3f BOARD=device` succeeding as the gate.

- [ ] **Step 5: Commit**

```bash
git add Makefile src/main_v3f.c
git commit -m "feat(two-board): DISPLAY_PRESENT gate — device board has no TFT

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: SPI slave MISO telemetry staging (driver + ISR)

Extends the IRQ slave so it returns a caller-supplied telemetry slot on MISO. Driver-level, bench-gated. The ISR change is minimal and double-buffered so it never reads a half-written slot.

**Files:**
- Modify: `src/spi_link.h` (declare `spi_link_slave_set_telem`)
- Modify: `src/spi_link.c` (static telem buffer, ISR TXE staging)

**Interfaces:**
- Consumes: existing `SPI1_IRQHandler`, `LINK_SPI`, `SPI_I2S_SendData`, `SPI_I2S_GetFlagStatus`, `SPI_I2S_FLAG_TXE`.
- Produces: `void spi_link_slave_set_telem(const uint8_t slot[SPI_LINK_SLOT]);` — publishes the 32-byte slot the ISR cycles onto MISO (repeats the slot continuously, one byte per clocked byte).

- [ ] **Step 1: Declare the publish API**

In `src/spi_link.h`, after `void spi_link_slave_set_drdy(int asserted);`, add:

```c
// Publish a 32-byte telemetry slot for the IRQ slave to stage on MISO. The RXNE
// ISR cycles these bytes onto the data register (one per clocked byte, repeating
// the slot), so the master sees a continuous stream of this slot on the return
// path. Double-buffered: the copy is atomic w.r.t. the ISR (a publish in progress
// never yields a torn slot to the wire). Pass a slot built with spi_frame_pack.
void spi_link_slave_set_telem(const uint8_t slot[SPI_LINK_SLOT]);
```

- [ ] **Step 2: Add the double-buffered telem state + publish function**

In `src/spi_link.c`, near the RX ring statics (around line 276), add:

```c
// --- Slave -> master telemetry return slot (staged onto MISO by the RXNE ISR) --
// Double-buffered: the foreground writes s_telem[next] then flips s_telem_cur.
// The ISR only ever reads s_telem[s_telem_cur], so it never sees a torn slot.
static volatile uint8_t s_telem[2][SPI_LINK_SLOT];
static volatile uint8_t s_telem_cur;     // which buffer the ISR reads
static volatile uint8_t s_telem_idx;     // next byte to send from the current slot
static volatile uint8_t s_telem_armed;   // 0 until the first publish

void spi_link_slave_set_telem(const uint8_t slot[SPI_LINK_SLOT])
{
    uint8_t next = (uint8_t)(s_telem_cur ^ 1u);
    for (uint32_t i = 0; i < SPI_LINK_SLOT; i++) s_telem[next][i] = slot[i];
    s_telem_cur = next;
    s_telem_armed = 1;
}
```

- [ ] **Step 3: Stage a TX byte in the RXNE ISR**

In `src/spi_link.c`, in `SPI1_IRQHandler`, after the RXNE read/push block (after the closing brace of the `if (... RXNE ...)` block near line 333), add:

```c
    // Stage the next telemetry byte onto MISO for the master's return slot. TXE is
    // up whenever the shift register can accept a byte; we feed the current slot,
    // wrapping so the slot repeats continuously. Cheap: one store per clocked byte.
    if (s_telem_armed &&
        SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_TXE) != RESET) {
        SPI_I2S_SendData(LINK_SPI, s_telem[s_telem_cur][s_telem_idx]);
        s_telem_idx = (uint8_t)((s_telem_idx + 1u) % SPI_LINK_SLOT);
    }
```

- [ ] **Step 4: Verify the V5F image builds (device + host both link the driver)**

Run:
```bash
make v5f BOARD=device 2>&1 | tail -3
make v5f BOARD=host 2>&1 | tail -3
```
Expected: both build, no errors, no unresolved `spi_link_slave_set_telem`.

- [ ] **Step 5: Commit**

```bash
git add src/spi_link.h src/spi_link.c
git commit -m "feat(spi-link): IRQ slave stages a telemetry slot on MISO return path

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: Device temp hop (device V3F → device V5F)

Adds the one new cross-core record so the device V5F knows the device-board temperature (the ADC is on V3F).

**Files:**
- Modify: `src/icc.h` (new `ICC_TAG_DEV_TEMP`)
- Modify: `src/main_v3f.c` (device-side: send temp record each render tick)

**Interfaces:**
- Consumes: `icc_send_to_v5f(const icc_record_t *)`, `icc_record_t { uint8_t tag; uint8_t b[15]; }`, `temp_read_c()`, `DISPLAY_PRESENT` macro (absent on device).
- Produces: record tag `ICC_TAG_DEV_TEMP` with `b[0]` = `(uint8_t)(int8_t)temp_c`.

- [ ] **Step 1: Add the record tag**

In `src/icc.h`, in the record-tag enum (the `// V3F -> V5F (inject commands)` group), add after `ICC_TAG_PHYS_MASK,`:

```c
    ICC_TAG_DEV_TEMP,     // device V3F -> V5F: device-board temp in b[0] (int8)
```

- [ ] **Step 2: Send the temp record on the device V3F (no display)**

In `src/main_v3f.c`, inside the 250 ms render-tick block, the temp is already read into `g_disp.temp_c`. Right after that line (`g_disp.temp_c = temp_read_c();`), add a device-only publish:

```c
            g_disp.temp_c    = temp_read_c();   // single ADC conversion, ~µs, V3F-local
#ifndef DISPLAY_PRESENT
            // Device board: no TFT — ship our local temp to the device V5F so it can
            // fold it into the SPI return telemetry the host TFT renders.
            {
                icc_record_t tr = { .tag = ICC_TAG_DEV_TEMP };
                tr.b[0] = (uint8_t)g_disp.temp_c;
                (void)icc_send_to_v5f(&tr);
            }
#endif
```

(The existing `icc_pump_to_v5f()` drain at the top of the loop carries it into the mailbox.)

- [ ] **Step 3: Verify both V3F roles build**

Run:
```bash
make v3f BOARD=host 2>&1 | tail -2
make v3f BOARD=device 2>&1 | tail -2
```
Expected: both build, no errors.

- [ ] **Step 4: Commit**

```bash
git add src/icc.h src/main_v3f.c
git commit -m "feat(two-board): device V3F ships board temp to V5F for SPI telemetry

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: Device V5F — build & publish the telemetry slot

The device relay loop packs a `TWO_BOARD_TYPE_TELEM` slot (enum, clone speed, device temp) and publishes it via `spi_link_slave_set_telem` so the ISR returns it on MISO.

**Files:**
- Modify: `src/two_board.h` (new frame types)
- Modify: `src/two_board.c` (device loop: consume `ICC_TAG_DEV_TEMP`, pack + publish telem)

**Interfaces:**
- Consumes: `spi_frame_pack`, `spi_link_slave_set_telem`, `usb_device_is_configured()`, `usb_device_active_speed()` (see Step 2 note), `icc_recv_from_v3f` path via `usb_merge_drain_icc` — see note.
- Produces: frame type macros `TWO_BOARD_TYPE_TELEM = 0x04`, `TWO_BOARD_TYPE_TELEM_REQ = 0x05`; telemetry payload schema `[enum, clone_speed, dev_temp]` (3 bytes).

- [ ] **Step 1: Add the frame type macros**

In `src/two_board.h`, after the `TWO_BOARD_TYPE_DESC` define, add:

```c
// SPI frame TYPE for device->host telemetry returned on the MISO slot (step: TFT
// pass-through). Payload: [enum(0/1), clone_speed(USB_SPEED_*), dev_temp(int8)].
#define TWO_BOARD_TYPE_TELEM      0x04u

// SPI frame TYPE for the host's periodic telemetry poll (empty payload). The host
// clocks this when no report has been sent recently so the device's MISO slot keeps
// flowing and the host's freshness heartbeat keeps advancing during idle mouse.
#define TWO_BOARD_TYPE_TELEM_REQ  0x05u
```

- [ ] **Step 2: Determine the device's active clone speed**

The clone speed is whichever USB backend `usb_device_init` selected from `desc.speed`. Check whether an accessor exists:

Run: `grep -n "active_speed\|active_is_hs\|s_active" src/usb_device.c src/usb_device.h`

If `usb_device.h` has no public accessor, add one. In `src/usb_device.h`, after `bool usb_device_is_configured(void);`, add:

```c
// Active clone speed (USB_SPEED_HIGH if the HS backend is live, else USB_SPEED_FULL).
uint8_t usb_device_active_speed(void);
```

In `src/usb_device.c`, after `active_is_hs()` / `active_is_fs()` (near line 30), add:

```c
uint8_t usb_device_active_speed(void) { return s_active; }
```

(`s_active` is the `USB_SPEED_*` value set in `usb_device_init` — confirmed at `usb_device.c:25-47`.)

- [ ] **Step 3: Intercept the temp tag inside the single drain (confirmed path)**

The mailbox is single-consumer: `usb_merge_drain_icc()` (`usb_merge.c:755`) owns the only `while (icc_recv_from_v3f(&r))` loop (line 767) and calls `icc_ipc_rearm_v5f()` at the end (line 828). The device loop already calls this drain every iteration. Intercept the temp tag inside that one drain — no new helper, no separate drain.

In `src/usb_merge.c`, at the **top of the `while (icc_recv_from_v3f(&r))` loop body** (line ~768), add:

```c
        if (r.tag == ICC_TAG_DEV_TEMP) {
            extern volatile int8_t g_tb_dev_temp_c;  // defined in two_board.c
            g_tb_dev_temp_c = (int8_t)r.b[0];
            continue;
        }
```

In `src/two_board.c`, define a global the merge can see (place it after the includes):

```c
volatile int8_t g_tb_dev_temp_c;   // device-board temp from ICC_TAG_DEV_TEMP
```

Use `g_tb_dev_temp_c` as the temp source in Step 4. (This intercept is harmless in the host image too: the host never sends `ICC_TAG_DEV_TEMP`, so the branch is never taken; `g_tb_dev_temp_c` is simply unused there.)

- [ ] **Step 4: Pack & publish the telem slot in the device relay loop**

In `src/two_board.c`, in `two_board_device_run`'s Phase-3 `for (;;)` loop, after the `usb_device_poll();` call and before the heartbeat-LED block, add:

```c
        /* Publish device->host telemetry on the SPI return slot (~every 100 ms).
         * The IRQ slave cycles this slot onto MISO; the host SOF-scans it. */
        static uint32_t tlm_ms;
        static uint8_t  tlm_seq;
        if ((millis() - tlm_ms) >= 100u) {
            tlm_ms = millis();
            uint8_t pay[3] = {
                (uint8_t)(usb_device_is_configured() ? 1u : 0u),
                usb_device_active_speed(),
                (uint8_t)g_tb_dev_temp_c,
            };
            uint8_t slot[SPI_LINK_SLOT];
            if (spi_frame_pack(slot, TWO_BOARD_TYPE_TELEM, tlm_seq++, pay, 3)
                == SPI_FRAME_OK) {
                spi_link_slave_set_telem(slot);
            }
        }
```

`usb_device.h` is already included in `two_board.c`. Ensure `g_tb_dev_temp_c` (Step 3) is declared above this loop.

- [ ] **Step 5: Verify the device V5F builds**

Run: `make v5f BOARD=device 2>&1 | tail -3`
Expected: builds; no unresolved symbols.

- [ ] **Step 6: Commit**

```bash
git add src/two_board.h src/two_board.c src/usb_device.h src/usb_device.c src/usb_merge.c
git commit -m "feat(two-board): device V5F publishes enum/speed/temp on SPI return slot

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 8: Host V5F — revive telemetry pump + decode device return slot

The host relay loop fills its own `display_status_t`, pumps it to V3F (the revival), SOF-scans the MISO return stream for `TWO_BOARD_TYPE_TELEM`, derives `dev_link` freshness, and clocks a periodic poll frame during idle.

**Files:**
- Modify: `src/two_board.c` (`two_board_host_run`, real-host path)

**Interfaces:**
- Consumes: `icc_status_pump_v5f(&disp)`, `spi_frame_stream_t` + `spi_frame_stream_init` + `spi_frame_stream_push`, `spi_frame_unpack`, `spi_link_master_exchange`, `spi_link_master_wedges`, `usb_host_device_speed()`, `TWO_BOARD_TYPE_TELEM`, `TWO_BOARD_TYPE_TELEM_REQ`.
- Produces: live host TFT telemetry (no new exported symbol).

- [ ] **Step 1: Add display + return-stream state to the host loop**

In `src/two_board.c`, in `two_board_host_run`, in the real-host branch (`#else` after `TWO_BOARD_HOST_SYNTH`), after `dbg_stage(DBG_V5F_DESC_OK);` and the `desc.speed = ...` line, add:

```c
    /* Host TFT telemetry: fill our own display_status_t and pump it to V3F over
     * the IPC reverse channel (the two-board loop previously never did this, so the
     * host display sat at NO SIGNAL). */
    display_status_t disp = { .state = DISP_STATE_RELAYING };
    disp.vid = (uint16_t)(desc.device_desc[8]  | (desc.device_desc[9]  << 8));
    disp.pid = (uint16_t)(desc.device_desc[10] | (desc.device_desc[11] << 8));
    disp.cap_speed = desc.speed;
    uint32_t disp_ms = millis();
    uint32_t rep_count = 0, rep_ms = millis();

    /* Return-path (device->host) telemetry decode. */
    static spi_frame_stream_t rxs;
    spi_frame_stream_init(&rxs);
    uint8_t  last_telem_seq = 0;
    bool     have_telem_seq = false;
    uint32_t telem_fresh_ms = millis();   // last time the telem seq advanced
    uint32_t poll_ms = millis();          // last periodic poll-frame clock
```

Add `#include "display.h"` and `#include "icc.h"` at the top of `two_board.c` if not already present (icc.h is included; add display.h).

- [ ] **Step 2: Add a helper to feed the return slot into the decoder**

In `src/two_board.c`, above `two_board_host_run`, add a helper that scans an `rx[]` slot's bytes for a telem frame and updates `disp`:

```c
/* Feed one received SPI return slot through the SOF-scanner; if it completes a
 * TWO_BOARD_TYPE_TELEM frame, fold its fields into *disp and bump *fresh_ms when
 * the frame seq advances (the device->host liveness heartbeat). */
static void host_absorb_return(spi_frame_stream_t *rxs, const uint8_t *rx,
                               display_status_t *disp, uint8_t *last_seq,
                               bool *have_seq, uint32_t *fresh_ms, uint32_t now)
{
    for (uint32_t i = 0; i < SPI_LINK_SLOT; i++) {
        uint8_t slot[SPI_LINK_SLOT];
        if (!spi_frame_stream_push(rxs, rx[i], slot)) continue;
        uint8_t type, seq, len; const uint8_t *pay;
        if (spi_frame_unpack(slot, &type, &seq, &pay, &len) != SPI_FRAME_OK) continue;
        if (type != TWO_BOARD_TYPE_TELEM || len < 3) continue;
        disp->dev_enum   = pay[0] ? 1u : 0u;
        disp->dev_speed  = pay[1];
        disp->dev_temp_c = (int8_t)pay[2];
        if (!*have_seq || seq != *last_seq) { *fresh_ms = now; }
        *last_seq = seq; *have_seq = true;
    }
}
```

- [ ] **Step 3: Capture the return slot on each report exchange**

The report send currently discards `rx`. In `src/two_board.c`, `send_report_frame` passes a local `rx`. To get the return bytes into the decoder without threading state through that helper, change the relay loop to inline-capture. In `two_board_host_run`'s real-host `for (;;)` loop, after the interrupt-IN poll/forward section that calls `send_report_frame(...)`, add the return absorb using a slot the master just clocked. Simplest correct approach: give `send_report_frame` an out param.

Change `send_report_frame`'s signature and body. Replace:

```c
static void send_report_frame(uint8_t dev_ep, uint8_t protocol,
                              const uint8_t *report, uint8_t rlen, uint8_t *seq)
{
    if (rlen > SPI_FRAME_MAX_PAYLOAD - 2) rlen = SPI_FRAME_MAX_PAYLOAD - 2;
    uint8_t payload[SPI_FRAME_MAX_PAYLOAD];
    payload[0] = dev_ep & 0x0F;
    payload[1] = protocol;
    memcpy(&payload[2], report, rlen);

    uint8_t tx[SPI_LINK_SLOT], rx[SPI_LINK_SLOT];
    if (spi_frame_pack(tx, TWO_BOARD_TYPE_MOUSE, (*seq)++, payload,
                       (uint8_t)(2 + rlen)) == SPI_FRAME_OK) {
        spi_link_master_exchange(tx, rx);
    }
}
```

with:

```c
static void send_report_frame(uint8_t dev_ep, uint8_t protocol,
                              const uint8_t *report, uint8_t rlen, uint8_t *seq,
                              uint8_t rx_out[SPI_LINK_SLOT])
{
    if (rlen > SPI_FRAME_MAX_PAYLOAD - 2) rlen = SPI_FRAME_MAX_PAYLOAD - 2;
    uint8_t payload[SPI_FRAME_MAX_PAYLOAD];
    payload[0] = dev_ep & 0x0F;
    payload[1] = protocol;
    memcpy(&payload[2], report, rlen);

    uint8_t tx[SPI_LINK_SLOT];
    if (spi_frame_pack(tx, TWO_BOARD_TYPE_MOUSE, (*seq)++, payload,
                       (uint8_t)(2 + rlen)) == SPI_FRAME_OK) {
        spi_link_master_exchange(tx, rx_out);
    } else if (rx_out) {
        memset(rx_out, 0, SPI_LINK_SLOT);
    }
}
```

Update the **synthetic** call site (in the `TWO_BOARD_HOST_SYNTH` branch) to pass a scratch buffer:

```c
            uint8_t rx_scratch[SPI_LINK_SLOT];
            synth_mouse_next_report(tick++, report);
            send_report_frame(SYNTH_MOUSE_IN_EP, SYNTH_MOUSE_IFACE_PROTO,
                              report, SYNTH_MOUSE_REPORT_LEN, &seq, rx_scratch);
```

- [ ] **Step 4: Absorb return bytes + pump display in the real-host loop**

In `src/two_board.c`, in the real-host `for (;;)` loop, change the interrupt-IN forward call and add absorb + poll + pump. Replace the EP poll block:

```c
            if (ret > 0 && rpt) {
                send_report_frame(ep_map[m].dev_ep_num, ep_map[m].iface_protocol,
                                  rpt, (uint8_t)ret, &seq);
            }
```

with:

```c
            if (ret > 0 && rpt) {
                uint8_t rx[SPI_LINK_SLOT];
                send_report_frame(ep_map[m].dev_ep_num, ep_map[m].iface_protocol,
                                  rpt, (uint8_t)ret, &seq, rx);
                host_absorb_return(&rxs, rx, &disp, &last_telem_seq,
                                   &have_telem_seq, &telem_fresh_ms, now);
                rep_count++;
            }
```

Then, still inside the loop, after the descriptor-resend `if` block and before the LED blink line, add the idle poll + freshness + pump:

```c
        /* Idle poll: if we haven't clocked a report recently, send an empty
         * TELEM_REQ so the device's MISO slot keeps flowing (else a still mouse
         * would falsely read LINK DOWN). */
        if ((now - poll_ms) >= 250u) {
            poll_ms = now;
            uint8_t tx[SPI_LINK_SLOT], rx[SPI_LINK_SLOT];
            if (spi_frame_pack(tx, TWO_BOARD_TYPE_TELEM_REQ, seq++, NULL, 0)
                == SPI_FRAME_OK) {
                spi_link_master_exchange(tx, rx);
                host_absorb_return(&rxs, rx, &disp, &last_telem_seq,
                                   &have_telem_seq, &telem_fresh_ms, now);
            }
        }

        /* Fill host-local fields + freshness, then pump one field to V3F. */
        if ((now - rep_ms) >= 1000u) {
            rep_ms = now;
            disp.reports_per_sec = (rep_count > 1023u) ? 1023u : (uint16_t)rep_count;
            rep_count = 0;
            disp.wedge = (uint16_t)(spi_link_master_wedges > 1023u
                                    ? 1023u : spi_link_master_wedges);
        }
        disp.dev_link = ((now - telem_fresh_ms) < 1000u) ? 1u : 0u;
        if ((now - disp_ms) >= 50u) {   // throttle the rotating field pump
            disp_ms = now;
            icc_status_pump_v5f(&disp);
        }
```

- [ ] **Step 5: Verify the host V5F builds (real + synth)**

Run:
```bash
make v5f BOARD=host 2>&1 | tail -3
make v5f BOARD=host HOST_SYNTH=1 2>&1 | tail -3
```
Expected: both build, no errors, no unresolved symbols.

- [ ] **Step 6: Commit**

```bash
git add src/two_board.c
git commit -m "feat(two-board): host V5F revives TFT telemetry + decodes device return slot

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 9: Full build + host test sweep

Final integration gate: every role image builds and all host unit tests pass.

**Files:** none (verification only).

- [ ] **Step 1: Run all host unit tests**

Run: `make test 2>&1 | tail -20`
Expected: `display_test OK` plus all other suites passing; no compiler errors.

- [ ] **Step 2: Build both board images end to end**

Run: `make all 2>&1 | tail -15`
Expected: `build/BoardB.bin` and `build/BoardA.bin` produced (host + device merged images), with size lines, no errors.

- [ ] **Step 3: Commit any incidental fixes**

If Steps 1–2 required a fix, commit it:

```bash
git add -A
git commit -m "fix(two-board): resolve build/test issues from TFT pass-through integration

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

If nothing changed, skip.

- [ ] **Step 4: Bench verification checklist (manual, hardware)**

Not automated — record results when on the bench:
1. Flash `build/BoardB.bin` (host) and `build/BoardA.bin` (device); follow the clean-boot procedure (flash both → mouse→B → fresh-boot A). See `memory/two-board-resend-load-bearing.md`.
2. Board A (device): TFT stays **dark** (no init).
3. Board B (host): TFT shows state RELAYING, `dev VID:PID HS/FS`, `rps`, `--- HOST (B) ---`, `link OK wedge N`, host `temp` (colored), `-- DEVICE (A) --`, `PC: ENUM HS/FS`, device `temp` (colored).
4. Leave the mouse still ~2 s: device block stays live (poll frame keeps it fresh), NOT `LINK DOWN`.
5. Unplug Board A's USB-to-PC (or reset A): host device block flips to `LINK DOWN` / `PC: --`, then clears on reconnect.
6. Move the mouse: cursor smooth, `rps` rises.

---

## Self-Review

**Spec coverage:**
- Remove device TFT → Task 4 (`DISPLAY_PRESENT` gate). ✓
- Revive host relay telemetry → Task 8 (pump in host loop). ✓
- Device data over SPI return slot (e, g) → Tasks 5 (ISR), 6 (temp hop), 7 (device pack), 8 (host decode). ✓
- FS/HS in two spots → Task 2 (`ROW_IDS` cap_speed, `ROW_PCENUM` dev_speed). ✓
- Temp color by value → Task 3. ✓
- `LINK DOWN` when stale → Task 2 (layout) + Task 8 (freshness). ✓
- rps → Task 2 (`ROW_RPS`). ✓
- Idle-clocking poll frame → Task 8. ✓
- Selectors ≤ 16 → Task 1 (8 existing + 3 new = 11). ✓

**Placeholder scan:** No TBD/TODO. Task 7 Step 2 contains a `grep` to confirm whether `usb_device_active_speed` already exists before adding it — both outcomes have fully-written code, so this is a guarded edit, not a placeholder.

**Type consistency:** `display_status_t` fields (`wedge`, `cap_speed`, `dev_enum`, `dev_speed`, `dev_temp_c`, `dev_link`) defined in Task 1, consumed identically in Tasks 2/3/8. Selectors `ICC_ST_SEL_WEDGE/SPEEDS/DEV` consistent across Task 1 pack/unpack/test. Frame types `TWO_BOARD_TYPE_TELEM`/`_REQ` defined in Task 7, used in Tasks 7/8. `spi_link_slave_set_telem` declared (Task 5) and called (Task 7). `usb_device_active_speed` added (Task 7 Step 2) and called (Task 7 Step 4). `g_tb_dev_temp_c` defined (Task 7 Step 3) and read (Task 7 Step 4). `host_absorb_return` defined and called (Task 8). `send_report_frame` signature change propagated to both call sites (Task 8 Steps 3–4).
