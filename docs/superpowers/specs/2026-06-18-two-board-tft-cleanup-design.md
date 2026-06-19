# Two-Board TFT Cleanup & Host Telemetry Pass-Through

**Date:** 2026-06-18
**Status:** Design — pending implementation plan

## Problem

The two-board USB MITM (Board B = host/capture, Board A = device/clone) drives a
ST7789 TFT from each board's V3F core. Two issues:

1. **Both boards have a TFT**, but only the host (Board B) needs a visible one.
   Board A's display is redundant.
2. **The host TFT shows almost nothing useful in two-board mode.** The relay
   telemetry pump (`icc_status_pump_v5f()`) is only called in the *single-board*
   relay loop (`main_v5f.c:603`). The two-board loops (`two_board_host_run()` /
   `two_board_device_run()`) never return and never pump it, so the top half of
   the display is empty and flips to `NO SIGNAL` after a few seconds. Only the
   V3F-local bottom block (uptime, temp, cmd rx/err, injection) is live.

## Goal

- Remove the TFT from Board A (device).
- Revive host-side relay telemetry on Board B's TFT.
- Pass selected device-board (Board A) data back to the host TFT over the
  already-bidirectional SPI link, so the host screen shows both boards' status.
- Show a FS/HS speed hint so the user knows which port to plug into the PC.
- Color-code temperature values by reading.

## Architecture facts (verified during design)

- **Both boards flash the same V3F image**, built per-role: `BOARD=host|device`
  sets `-DBOARD_ROLE_*`, which reaches the V3F compile via `DEFINES → CFLAGS`
  (Makefile:47, 83). Only the *V5F* image differs in role behavior at runtime.
  V3F is the core that drives the ST7789.
- V3F renders `display_status_t` at 4 Hz (`main_v3f.c:230-248`), fed by
  `icc_status_poll_v3f()` over the V5F→V3F IPC reverse status channel
  (IPC STS bits [16:31], 16-bit time-multiplexed word, 4-bit selector).
- The SPI link is **already full-duplex**: `spi_link_master_exchange(tx, rx)`
  clocks 32 bytes each way; Board A's return slot (`rx[]`) is currently
  discarded by all callers in `two_board.c`.
- A DRDY reverse GPIO already exists (`spi_link_master_drdy()` /
  `spi_link_slave_set_drdy()`), used to gate descriptor re-sends.
- Device temp ADC is on the device **V3F** core; the SPI link is driven by the
  device **V5F** core. Only the V3F→V5F IPC mailbox direction is coherent
  (the reverse SRAM ring reads back stale — see `icc.c`).

## What the host TFT shows

Selected during design: capture state + device VID:PID (a), SPI link health (c),
host temp (d) — all host-local; clone-enumerated (e) + device temp (g) passed
back from Board A; plus reports/sec (b) and FS/HS speeds.

There are **two** speeds, intentionally shown in two places:
- **Captured-device speed** — what Board B negotiated with the real device
  (`desc.speed = usb_host_device_speed()`), shown next to its VID:PID.
- **Clone→PC speed** — what Board A re-enumerates to the PC at; this is the
  "which plug to use" hint, shown next to the enumeration status. Sent back
  from Board A.

## Approach (chosen: return-slot piggyback)

Device→host data rides the **wasted SPI return slot**. No new wires, no added
bus traffic. Rejected alternatives: a dedicated CRC'd telemetry frame (more code
and bus slots for non-critical status) and extra GPIO status lines (can't carry
temp/speed).

## Data flow

### Flow A — Host-local (revive existing pump)
Board B's V5F owns: capture state, VID:PID, rps, captured-device speed, SPI
wedge count. It fills its own `display_status_t` and calls the existing
`icc_status_pump_v5f()`; host V3F polls and renders. **Fix:** add the throttled
pump call into `two_board_host_run()`'s relay loop (it is absent there today;
mirror `main_v5f.c:603`). Host temp is read locally by host V3F (unchanged,
already filled in `main_v3f.c`).

### Flow B — Device → Host over SPI return slot (new)
```
device V3F --(IPC tag)--> device V5F --(SPI MISO slot)--> host V5F --(display pump)--> host V3F TFT
  temp ADC               builds telem blob              reads rx[] it             renders DEVICE block
                         into slave TX buf              already clocks
```
- Device V5F maintains a small fixed telem blob written as the slave TX buffer,
  so **every** frame Board B clocks returns current device status. Blob carries:
  `enum_flag` (1 bit), `clone_speed` (1 bit), `device_temp` (int8), `seq`
  (rolling, for freshness).
- Host V5F unpacks `rx[]` (currently discarded), folds the values into its
  `display_status_t` device fields, which ride the **existing** display IPC
  channel to host V3F.
- **Freshness / `LINK DOWN`:** host V5F tracks whether the blob `seq` advanced
  within a window (same heartbeat trick as `icc_status_poll_v3f`). Stale → device
  block renders `LINK DOWN`.

### Flow C — Device temp hop (new, small)
Device temp ADC is on device V3F; SPI is driven by device V5F. Device V3F
publishes temp to device V5F via a new `ICC_TAG_DEV_TEMP` record over the
existing (coherent) V3F→V5F IPC mailbox. Device V5F caches the latest value and
folds it into its telem blob. (`enum_flag` is V5F-native — no hop.)

**Robustness note:** the return blob is intentionally **not CRC-protected**
(Approach 1). A garbled byte shows one wrong temp/speed for ~250 ms and
self-corrects. Acceptable for non-critical status; the mouse-report path keeps
its CRC.

## Display layout & encoding

### New `display_status_t` fields
- `uint16_t wedge` — SPI master wedge/recovery count (host-local)
- `uint8_t  cap_speed` — captured-device speed (0=FS, 1=HS), host-local
- `uint8_t  dev_enum` — clone configured on PC (0/1), from Board A
- `uint8_t  dev_speed` — clone→PC speed (0=FS, 1=HS), from Board A
- `int8_t   dev_temp_c` — device board temp, from Board A
- `uint8_t  dev_link` — device telemetry fresh (1) / stale (0), derived host-side

### Row layout (20 cols × 13 rows; replaces relay/local split)

| Row | Content                       | Source        |
|-----|-------------------------------|---------------|
| 0   | `RELAYING` (state, colored)   | host-local (B)|
| 1   | `dev 046D:C08B  HS`           | host-local (B)|
| 2   | `rps 980`                     | host-local (B)|
| 3   | `--- HOST (B) ---`            | divider       |
| 4   | `link OK  wedge 0`            | host-local (B)|
| 5   | `temp 41 C` (row colored by value) | host V3F |
| 6   | `--- DEVICE (A) ---`          | divider       |
| 7   | `PC: ENUM  HS` / `PC: --`     | Flow B        |
| 8   | `temp 39 C` (row colored by value) / `--` | Flow B/C |
| 9   | `LINK DOWN` (only when stale, red) | derived  |
| 10–12 | blank                       | —             |

### Color rules
- **State row:** green=RELAYING, red=ERROR/NOSIGNAL, white=other (existing).
- **Temp rows:** the *whole row* is colored by the temperature value —
  **green <50 °C, amber 50–65 °C, red >65 °C**. (Per-glyph label/value coloring
  was considered and rejected as unnecessary complexity.)

### Encoding
The new device/host fields slot into spare `ICC_ST_SEL_*` selectors (8 of 16
used today) — no widening of the 16-bit reverse channel. Speeds/enum are 1 bit,
temps are signed 8-bit (fit the 10-bit payload), wedge clamps to 10-bit like
`drops`.

## Files touched

| File | Change |
|---|---|
| `Makefile` | New `-DDISPLAY_PRESENT` define gated on `BOARD=host`; device V3F builds without it. (`st7789.c`/`display.c` stay in `V3F_SRC`; the no-op path dead-strips via existing `-ffunction/-fdata-sections`.) |
| `src/main_v3f.c` | Guard `display_init()`, `display_render()`, and the status-poll/render block behind `#ifdef DISPLAY_PRESENT`. Device V3F skips all display work. |
| `src/two_board.c` | Host loop: add throttled `icc_status_pump_v5f()`; read + unpack Board A return-slot telemetry into the host display fields. Device loop: build the telem blob into the SPI slave TX buffer; consume cached device temp from device V3F. |
| `src/icc.h`, `src/icc_status.c` | New status selectors (wedge, cap_speed, dev_enum, dev_speed, dev_temp, dev_link) + new `ICC_TAG_DEV_TEMP` for device V3F→V5F. |
| `src/display.h`, `src/display.c` | New `display_status_t` fields; new HOST/DEVICE row layout; temp-row color thresholds. |
| `test/display_test.c` | Extend pure-layout tests for the new rows/fields/colors. |

## Testing

- **Pure layout** (`make` host test target → `display_test`): assert the new rows
  render correctly for representative states (no device, capturing, relaying,
  device link up/down, each temp color band). This is the host-testable gate.
- **Bench:** flash Board B (host) + Board A (device). Verify Board A's TFT stays
  dark (no init); Board B's TFT shows live state, VID:PID, both speeds, rps,
  host SPI/temp, and device enum/temp; pull/replug the device to confirm
  `LINK DOWN` appears and clears.

## Out of scope (YAGNI)

- CRC on the return telemetry blob.
- Reports/sec on the device side, injection counts, drop counts (not selected).
- Any change to the mouse-report hot path or its CRC.
- Removing `st7789.c`/`display.c` from the device build (dead-strip handles it;
  a hard removal would fork V3F_SRC per role for no real gain).
