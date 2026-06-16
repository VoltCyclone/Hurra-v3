# Status Display — Design Spec

**Date:** 2026-06-16
**Status:** Approved (design phase)
**Branch:** to be created (`feat/status-display`)

## Summary

Add a small on-board status display to the Hurra-v3 firmware. The MuseLab
nanoCH32H417 ships with a **1.54" ST7789 240×240 SPI TFT** on a 12-pin FPC
connector (the shipped `demo/SPI_LCD.Bin`). This feature drives that panel from
the **V3F (master) core** to show live relay status: connection state, the
captured device's VID:PID, report throughput, and relay liveness.

A minimal **V5F→V3F telemetry trickle** feeds the display, reusing the existing
coherent IPC status-bit mechanism (`icc_telem_*`). No shared-SRAM reverse path
(that direction reads back stale across the DTCM boundary and previously caused
the relay AHB wedge).

## Goals

- A readable status screen on the shipped ST7789 panel.
- Driven entirely by V3F; **zero** added load on the V5F USB hot path beyond a
  couple of MMIO stores on state change.
- Lean: a trimmed ST7789 driver + one small font, not the 3930-line EVT graphics
  library (which embeds a 52 KB test image and Chinese glyph tables).
- Non-essential by construction: if the display or telemetry misbehaves, the
  relay path is unaffected.

## Non-Goals

- No use of the "video" peripherals (LTDC parallel-RGB, DVP camera input, GPHA
  2D accelerator). A small SPI status display does not need them; they cost far
  more pins/RAM and are the wrong tool here.
- No graphics library (lines, circles, image blit, multiple font sizes).
- No touch input, no animation, no framebuffer in RAM.
- No new permanent V5F→V3F bulk channel — only the low-rate status trickle.

## Hardware (verified against wuxx/nanoCH32H417 EVT `SPI/SPI_LCD`)

Panel: 1.54" **ST7789**, **240×240**, RGB565. Driven in `SPI_HW` mode.

| Signal | MCU pin | Mode |
|--------|---------|------|
| SPI2 SCK  | PB13 | AF5 |
| SPI2 MOSI | PB15 | AF5 |
| SPI2 MISO | PB14 | AF5 (unused; panel is write-only) |
| CS  | PB12 | GPIO out |
| RES | PD8  | GPIO out |
| DC  | PD9  | GPIO out |

Clocks: `RCC_HB1Periph_SPI2`, `RCC_HB2Periph_GPIOB | GPIOD | AFIO`.
SPI config (from EVT): Master, 8-bit, CPOL=High, CPHA=2Edge, NSS=Soft,
prescaler Mode0, MSB-first.
Power: the EVT example sets the **VIO18 rail to 3.3V in software**
(`PWR_VIO18ModeCfg(SW)` + `PWR_VIO18LevelCfg(MODE3)`) before driving the panel.
We replicate this in `display_init()`. **Note:** EVT recommends doing this in
hardware for the safety of external devices; document this in the README.

**Pin-conflict check (done 2026-06-16):** none of PB12/13/14/15, PD8, PD9, or
SPI2 are used anywhere in current `src/`. V5F touches GPIOB only at PB8/PB9
(SWJ disable for the USBHS host). No collision.

## Architecture

```
 V5F (relay core)                 IPC MMIO              V3F (master core)
 ───────────────                (0xE000D000)            ─────────────────
 relay loop                                             main loop (millis-gated)
   │ on state change:                                     │ every ~250 ms:
   │  icc_telem_status_v5f(field,val) ─► IPC STS bits ─►  │  read telem → status
   │                                  (single-writer)     │  if changed: render
   │                                                       │  display_render(&st)
                                                           │      │ SPI2 → ST7789
                                                           ▼      ▼
                                                     [ relay state line ]
                                                     [ device VID:PID   ]
                                                     [ reports/s  uptime ]
```

- All SPI + render work and the ~4 Hz redraw run on **V3F**.
- **V5F** only issues a couple of MMIO stores when relay state changes, plus a
  1 Hz report-rate publish — the same single-writer IPC-STS pattern already used
  for stage telemetry (peripheral-bus MMIO, coherent, race-free by single-writer
  construction). No SRAM in the reverse path.

## Components / File Layout

New (V3F-only image, except the telemetry publisher which lives in `icc.c`):

| File | Purpose | ~LOC |
|------|---------|------|
| `src/st7789.c` / `.h` | Trimmed ST7789 driver: SPI2 init, `lcd_init()` (power-on register sequence copied **verbatim** from EVT `lcd.c`), `lcd_set_window()`, `lcd_fill_rect()`. No graphics lib, no embedded image. | ~220 |
| `src/font5x7.h` | One 5×7 ASCII font table, printable range 0x20–0x7E, `const` (flash). | ~100 |
| `src/display.c` / `.h` | Status layer: `display_init()`, `display_render(const display_status_t*)`; owns `draw_char`/`draw_string` (built on `lcd_fill_rect`), layout, and per-line dirty-tracking so only changed lines repaint. | ~200 |

Changed:

| File | Change |
|------|--------|
| `src/icc.c` / `.h` | Add reverse status publisher `icc_telem_status_v5f(field,val)` (V5F) + reader `icc_telem_status_read_v3f(...)` (V3F), extending the existing CH1 STS pattern. Define `display_status_t` and the field-selector enum. |
| `src/main_v3f.c` | `display_init()` after LED init; add a millis-gated `display_tick` (~250 ms) that reads telemetry and calls `display_render`. |
| `src/main_v5f.c`, `src/usb_merge.c` | Call `icc_telem_status_v5f(...)` at existing relay state-transition sites (device connected, descriptors captured, entering relay) + a 1 Hz report-rate publish. No new hot-path branch in the per-report path beyond a cheap counter increment. |
| `Makefile` | Add `src/st7789.c src/display.c` to `V3F_SRC` only. |
| `src/board.h` | Add LCD pin macros (SPI2, PB12/13/15, PD8/9) next to the existing LED/USART maps. |

**Boundaries:** `st7789.c` knows pixels, not status. `display.c` knows
status→layout, not SPI. `icc.c` owns the transport. The font/layout/dirty math
in `display.c` is host-compilable for unit testing.

## Telemetry Protocol (V5F→V3F)

Extends the existing `icc_telem_*` IPC-STS mechanism. V5F is the **sole writer**
of these status bits (race-free by construction).

`display_status_t` fields V3F reconstructs:
- `relay_state` — enum WAITING / CAPTURING / RELAYING / ERROR (mapped from the
  existing `TLM_RLY_*` / `DBG_V5F_*` stage codes).
- `vid`, `pid` — 16-bit each, captured device IDs.
- `reports_per_sec` — V5F maintains a counter and publishes the 1 Hz rate.
- `live_seq` — the existing 2-bit rolling heartbeat; lets V3F distinguish
  "relay alive" from "relay wedged".

**Encoding:** V5F time-multiplexes fields through the spare IPC STS bits — a
small field-selector + payload, one field per publish, cycling. State changes
publish immediately; VID/PID/rate refresh on a slow rotation. Status is
low-rate, so a few fields per second is ample. V3F polls `IPC->STS` once per
display tick and updates its local `display_status_t`.

## Error Handling

- **Telemetry stall:** if `live_seq` stops advancing for N display ticks, V3F
  shows "RELAY: NO SIGNAL" instead of a frozen stale screen. The display never
  blocks waiting on V5F.
- **No SPI readback:** ST7789 is write-only here; `lcd_init()` is fire-and-forget
  (as in EVT). If the panel is absent/unplugged, writes are harmless no-ops and
  firmware behaves exactly as today.
- **Strictly non-essential:** `display_*` is V3F-only and must never gate the
  relay. Slow rendering can only delay V3F housekeeping, never V5F. Render is
  bounded — dirty-line repaint, no full-frame clears after init.
- **VIO18** configured once at init.

## Testing

- **Host-compile** the `display.c` font/layout/dirty-tracking logic via a small
  test (mirroring the existing `humanize_test` / `motion_test` pattern in
  `Makefile`) — no hardware needed.
- **On-hardware bring-up ladder:**
  1. `lcd_init()` + solid-color fill (panel alive, SPI wiring correct).
  2. Static text (font + layout correct).
  3. Live status with a real mouse (Razer Basilisk V3): verify WAITING→RELAYING
     transition and that reports/sec moves.
- **Regression:** confirm the existing hardware-verified relay + cursor path
  still works with the display active — proving V3F display work does not perturb
  V5F. Flash via existing `make flash`.

## Risks / Open Questions

- **IPC STS spare-bit budget:** CH1 currently uses bits [8:15]. Need to confirm
  enough spare STS bits remain for the field-multiplexed status without colliding
  with the forward doorbell/telemetry use. (Verified during implementation; STS
  is a 32-bit register and only [8:15] are currently claimed.)
- **VIO18 software config** is a power-rail change; do it exactly as EVT does and
  document the hardware-config recommendation.
- **SPI2 baud:** EVT uses prescaler Mode0; if the FPC/wiring is marginal at full
  speed, fall back to a slower prescaler (panel writes are not throughput-bound
  for a status screen).
```

## Build Sequence (for the plan)

1. `board.h` LCD pin macros.
2. `st7789.c/.h` — SPI2 + `lcd_init` verbatim from EVT + `fill_rect`/`set_window`;
   bring-up step 1 (color fill).
3. `font5x7.h` + `display.c/.h` text/layout; bring-up step 2 (static text).
   Host-compile test for layout/dirty math.
4. `icc.c/.h` reverse status publisher + reader; `display_status_t`.
5. Wire V3F main loop (`display_tick`) + V5F publish sites.
6. Bring-up step 3 (live status) + regression on relay/cursor.
