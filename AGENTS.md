# AGENTS.md — Hurra v3

> This file is written for AI coding agents. It assumes you know nothing about the project. Refer to `README.md` for the human-facing overview and to `CLAUDE.md` for day-to-day project memory (notes, gotchas, recent fixes). `docs/ch32h417-evt-reference.md` is the authoritative register/boot reference extracted from the WCH EVT SDK.

## Project overview

**Hurra v3** is bare-metal firmware for the **WCH CH32H417** dual-core RISC-V MCU. It sits between a USB HID device and a host PC:

- Enumerates a real HID mouse/keyboard on the **USBHS High-Speed host** port.
- Clones that device to the PC on the **USBFS Full-Speed device** port.
- Forwards every real HID report through unchanged.
- Injects synthetic mouse/keyboard input on top of the live HID stream over a serial command link.
- Runs injected motion through an always-on humanization filter (sub-pixel jitter, micro-correction, dwell).

The work is split across the two QingKe RISC-V cores:

| Core | Clock | Role | Main file |
|------|-------|------|-----------|
| **V3F** | ~100 MHz | Boot/master core. Clocks, UART command link, protocol parser, humanize-level control, LED heartbeat, status display. | `src/main_v3f.c` |
| **V5F** | 400 MHz | Relay core. USBHS host capture → HID merge → USBFS device forward. | `src/main_v5f.c` |

Two command protocols are supported, selected at compile time:

- **Hurra binary** (default) — TinyFrame framing; driven by `hurra-app` / `hurra-bridge`. Boot baud is 921600 on the on-board WCH-LinkE VCP.
- **Ferrum ASCII** (`make PROTOCOL=ferrum`) — `km.<name>(<args>)\r\n`; default 115200 baud.

This is a port of Hurra-v2 (NXP i.MX RT1062 / Teensy MicroMod). The command protocols, host tooling, and hardware-agnostic logic (parsers, actions, humanize, descriptor capture, tests) are carried over; the HAL (USB stack, UART, clocks, startup, inter-core channel) was rewritten for the CH32H417.

## Hardware target

Only one board is supported:

- **WCH CH32H417QEU6** USB 3.0 development board (QFN128).
- 25 MHz HSE crystal feeding 400 MHz system / 480 MHz USBHS PLL / 48 MHz USBFS clock.
- LEDs on **PC2** (V3F heartbeat) and **PC3** (V5F relay-stage indicator). Earlier code used PB1; that was a stale schematic read.
- Command link: **USART1** = PA9 (TX) / PA10 (RX), AF7. Wired to the on-board WCH-LinkE virtual COM port via solder bridges SB3/SB4, so one USB-C cable carries flash + debug + command link.
- USBHS host D+/D- = **PB8/PB9** (shared with SWCLK/SWDIO). The firmware disables SWJ on V5F to let the USBHS PHY drive the pads; debug then uses the WCH-LinkE SDI link, not SWD.
- Optional status display: 1.54" ST7789 240×240 SPI TFT driven by V3F over **SPI2** (PB12/13/14/15, PD8/9). See `src/board.h` for the pin map and `src/display.c` / `src/st7789.c` for the implementation.

**Critical power/flash notes:**

- The USBHS host port does **not** source 5 V. The attached device must be powered externally or through the board's host VBUS path (SB7/U5 on nanoCH32H417). No device power = no enumeration.
- Some boards have a **device-port VBUS 5 V jumper** that ties PC-side VBUS to the 3.3 V rail. If present, Windows back-powers the rail and `make flash` can fail. Remove that jumper; the USBFS device PHY uses an internal D+ pull-up and does not need to sense VBUS.
- Because running firmware disables SWJ, a running target usually NAKs `wlink` with `protocol error: 0x55`. Use `wlink --chip CH32H41X erase --method power-off` before flashing, or just run `make flash` which does this automatically.

## Repository layout

```
Makefile                      two-image RISC-V build + merge + flash
README.md                     human-facing overview, wiring, commands
CLAUDE.md                     project memory / running notes (not in git)
AGENTS.md                     this file
core/                         startup, linker scripts, clock tree, timebase
  startup_v3f.S / startup_v5f.S   reset vectors + vector tables
  link_v3f.ld / link_v5f.ld       per-core linker scripts
  system_ch32h417.c/h             clock tree (HSE → 400 MHz, USB PLLs)
  timebase.c / timebase_v5f.c     per-core millis() + V5F µs counter
include/                      project-wide config + port header
  ch32h417_port.h               umbrella header + WCH_IRQ macro
  ch32h417_conf.h               StdPeriph header inclusion list
src/                          firmware source
  main_v3f.c / main_v5f.c       core entry points
  icc.c / icc.h                 inter-core channel (IPC MSG mailbox, HSEM, doorbell)
  icc_status.c                  pure pack/unpack for V5F→V3F status telemetry
  usb_host.c / usb_host.h       USBHS host driver (capture real device)
  usb_device.c / usb_device.h   USBFS device driver (clone to PC)
  usb_merge.c / usb_merge.h     HID-report-descriptor-aware merge + injection
  desc_capture.c / desc_capture.h  descriptor + HID report-layout capture
  uart.c / uart.h               USART1 interrupt-driven RX/TX rings
  kmbox_cmd.c / kmbox_cmd.h     V3F injection sinks → ICC records
  kmbox.h                       shim: aliases v2 kmbox_* → kmbox_cmd_* / uart_*
  kmbox_cmd_v5f_stub.c          undefined sink stubs for the V5F link
  hurra.c / hurra.h             Hurra binary parser (TinyFrame)
  ferrum.c / ferrum.h           Ferrum ASCII parser
  proto.h                       compile-time protocol selector
  actions.c / actions.h         transport-agnostic injection helpers
  humanize.c / humanize.h       always-on humanization filter
  led.c / led.h                 LED status ladder / heartbeat
  display.c / display.h         status → text layout + render
  st7789.c / st7789.h           SPI TFT pixel driver
  font5x7.h                     5×7 ASCII font
  temp.c / temp.h               V3F on-die temperature (ADC1 ch16)
  board.h                       board pin/clock/peripheral map
  third_party/TinyFrame/        TinyFrame framing library
vendor/wch/                   vendored WCH CH32H417 EVT SDK
  Core/                         PFIC/HSEM/IPC/SysTick helpers
  Peripheral/inc, Peripheral/src  StdPeriph drivers
  Startup/                      reference startup.S (we derive ours in core/)
  Ld/                           reference linker scripts
  Debug/                        Delay_* helpers
  usb_reference/                unbuilt reference examples (do not compile)
docs/
  ch32h417-evt-reference.md     register / dual-core boot reference
  superpowers/plans/            implementation plans
  superpowers/specs/            design specs
test/                         host-native unit tests
  humanize_test.c
  motion_test.c
  display_test.c
tools/                        host-side Python harnesses
  ferrum_test.py                Ferrum smoke harness
  ferrum_aim_test.py            closed-loop aim test
  ferrum_load_test.py           command-channel load test
  humanization_analyze.py       kinematic trace analyzer
  dump_mouse_desc.py
scripts/
  merge_images.py               merges V3F + V5F binaries into build/Merge.bin
  wch-riscv.cfg                 OpenOCD config for WCH-OpenOCD fork
build/                        build artifacts (ignored by git)
```

## Technology stack

- **Language:** C11, bare-metal, no RTOS.
- **Toolchain:** RISC-V GCC. Default prefix is `riscv-none-elf` (xPack / MounRiver). Homebrew users can override with `make TOOLCHAIN=riscv64-unknown-elf`.
- **Architecture flags:**
  - V3F: `-march=rv32imac_zicsr -mabi=ilp32` (soft-float).
  - V5F: `-march=rv32imafc_zicsr -mabi=ilp32f` (hardware FPU enabled in startup, FP used only in the main loop; ISRs are FP-free).
- **Libraries:** vendored WCH CH32H417 StdPeriph + Core + Debug; TinyFrame for Hurra binary framing.
- **Host tooling:** Python 3 + `pyserial`; optional `pynput` for the aim test.
- **Flashing:** `wlink` (Rust, ch32-rs/wlink, recommended) or the WCH-OpenOCD fork (`wch-openocd`). Mainline OpenOCD does **not** work.

## Build system

The Makefile builds two separate ELF images, converts them to raw binaries, and merges them into one flash image:

- V3F linked at flash offset `0x00000000` (64 KB budget).
- V5F linked at flash offset `0x00010000` (128 KB budget).
- `scripts/merge_images.py` pads with `0xFF` and concatenates.

### Common Make variables

| Variable | Default | Meaning |
|----------|---------|---------|
| `TOOLCHAIN` | `riscv-none-elf` | GCC prefix (use `riscv64-unknown-elf` for the Homebrew toolchain) |
| `PROTOCOL` | `hurra` | `hurra` or `ferrum` |
| `CMD_BAUD` | `921600` (Hurra) / `115200` (Ferrum) | Boot baud |
| `SELFTEST` | empty | `master` or `slave` — builds the board-to-board SPI echo bench harness into the V5F image instead of the relay (diverts in `main_v5f.c`; `src/spi_link_selftest.c`). LED on PC3: slow ~1 Hz = link healthy, fast ~8 Hz = errors/no link. |
| `BOARD` | empty | `host` or `device` — builds the **two-board** USB-over-SPI MITM into the V5F image instead of the single-board relay (`src/two_board.c`). `host` = SPI master + USBHS capture; `device` = SPI slave + USB clone to the PC. |
| `HOST_SYNTH` | unset | With `BOARD=host`, drives Board B from a synthetic mouse (no real USB host) — isolated link test. Without it, `BOARD=host` does the real USBHS capture. |
| `DEVICE_LOCAL` | unset | With `BOARD=device`, drives the clone from a local synthetic mouse with NO SPI — isolates the device USB bring-up. |
| `EXTRADEF` | empty | Extra flags appended to `CFLAGS` for both cores. Intended for `-D` defines, e.g. `EXTRADEF=-DV5F_STAGE_DIAG_OFF`. Note: the `v5f` target always additionally appends `-funroll-loops`, so any conflicting optimization flag passed through `EXTRADEF` will be overridden on V5F. |

The default build (no `SELFTEST`/`BOARD`) is the **single-board relay**: one CH32H417 captures on USBHS host and clones on USBFS device. The **two-board mode** (`BOARD=host`/`device`, two boards linked over SPI1/PA3–PA7) is verified end-to-end with a real composite mouse — capture on Board B → SPI → clone on Board A → PC. Both boards disable SWJ, so their only live diagnostic is the V3F UART stage oracle (`V5F=0x..`; read at 921600 with DTR asserted). `vid/pid/rps/hb` telemetry is only pumped by the single-board relay, not by two-board mode — ignore those fields there.

### Build commands

```sh
make all                       # build both cores + merge -> build/Merge.bin
make v3f                       # build only the V3F image
make v5f                       # build only the V5F image
make PROTOCOL=ferrum all       # build with Ferrum ASCII protocol
make clean                     # remove build/
make test                      # host-native unit tests
```

### Flash commands

```sh
make flash                     # merge + program build/Merge.bin over WCH-LinkE
make flash-v3f                 # flash only V3F image (bring-up aid)
make flash-v5f                 # flash only V5F image (bring-up aid)
make erase                     # full-chip erase
```

`make flash` auto-detects `wlink` first, then `wch-openocd`. Override with `FLASH_TOOL=wlink|openocd`, or set `WLINK=`, `WCH_OPENOCD=`, `WCH_CFG=`, `FLASH_ADDR=`, `WCH_OCD_ADDR=`.

## Runtime architecture

### Dual-core split

- **V3F** boots from flash, calls `SystemInit()`, initializes the ICC, then releases V5F with `NVIC_WakeUp_V5F(Core_V5F_StartAddr)`. It then runs the command loop: UART RX → protocol parser → `act_*` → ICC records to V5F. It also drives the LED status ladder and the status display.
- **V5F** starts after V3F wakes it. It initializes USBHS host, waits for/captures the real device, initializes USBFS device, then runs the relay loop: drain ICC injection → device_poll → host poll → merge → device send → device-OUT relay → synthetic injection → telemetry.

### Inter-core channel (ICC)

Implemented in `src/icc.c` / `src/icc.h`:

- **V3F→V5F injection** uses the **IPC MSG hardware mailbox** (`IPC->MSG[0..3]`), not a shared SRAM ring. Bench-proven fact: each core can write the other core's DTCM, but reading the other core's DTCM returns stale data. Therefore the SRAM ring at `0x20178000` is V3F-local only; `icc_pump_to_v5f()` drains it into the coherent IPC MSG mailbox one record at a time.
- **V5F→V3F telemetry** uses coherent **IPC status bits** (CH1 for a 6-bit relay stage + heartbeat, CH2/CH3 bits [16:31] for the status display fields). V5F must **never** write V3F-side SRAM from the hot loop — such cross-core stores can stall the V5F AHB pipeline.
- **HSEM ID 0** is used only for the one-time startup rendezvous.
- **IPC doorbell CH0 Bit0** wakes V5F from `wfi` when injection is queued.
- ICC record tags include `INJECT_MOUSE`, `INJECT_KEYBOARD`, `CLICK_RELEASE`, `KB_RELEASE`, `SET_BAUD`, `SET_HUMAN_LEVEL`, `PHYS_MASK`.

### Memory / sections

- Code runs from **ITCM** (`RAM_CODE`). Startup copies `.highcode` from flash to ITCM at reset.
- V5F USB DMA buffers live in the **`.usbdma`** section (dedicated uncached SRAM, `NOLOAD` in `link_v5f.ld`). Tag them `__attribute__((section(".usbdma"), aligned(4)))`.
- USBFS DMA buffer pointers need the **+0x20000000** CPU-side offset; USBHS buffers do **not**.
- Shared SRAM block at `0x20178000` (`.shared`) is used for startup magic and V3F-local FIFO only.

## Code organization and key modules

| Module | Responsibility |
|--------|----------------|
| `src/main_v3f.c` | V3F entry: clocks → timebase → ICC → wake V5F → UART command loop + LED/display |
| `src/main_v5f.c` | V5F entry: timebase → ICC rendezvous → USBHS host → capture descriptors → USBFS device → relay loop |
| `src/icc.c/.h`, `src/icc_status.c` | Inter-core channel + V5F→V3F status telemetry |
| `src/usb_host.c/.h` | USBHS host driver: control transfers, interrupt IN/OUT polling |
| `src/usb_device.c/.h` | USBFS device driver: enumeration, HID report forwarding, EP0 SET_REPORT capture |
| `src/usb_merge.c/.h` | HID-report-descriptor-aware overlay of injected input onto real reports |
| `src/desc_capture.c/.h` | Device/configuration/HID/string/BOS/MS-OS descriptor capture |
| `src/uart.c/.h` | USART1 interrupt-driven RX/TX rings (V3F command link) |
| `src/kmbox_cmd.c/.h`, `src/kmbox.h`, `src/kmbox_cmd_v5f_stub.c` | V3F command sinks → ICC; shim to v2 names |
| `src/hurra.c/.h` | Hurra binary protocol parser (TinyFrame) |
| `src/ferrum.c/.h` | Ferrum ASCII protocol parser |
| `src/proto.h` | Compile-time alias of `proto_*` to the selected parser |
| `src/actions.c/.h` | Transport-agnostic injection state and motion programs |
| `src/humanize.c/.h` | Always-on humanization filter (runs per-frame on V5F) |
| `src/led.c/.h` | Status LED heartbeat / ladder |
| `src/display.c/.h`, `src/st7789.c/.h`, `src/font5x7.h` | V3F status display |
| `src/temp.c/.h` | V3F on-die temperature read |
| `src/board.h` | Pin, clock, peripheral macros |
| `include/ch32h417_port.h` | Umbrella header + `WCH_IRQ` ISR attribute |

## Code style guidelines

The project uses plain C with a consistent embedded style:

- **Integer types:** use `<stdint.h>` names (`uint8_t`, `int16_t`, etc.) in `src/` files. The WCH StdPeriph uses `u8`/`u16`; cast at StdPeriph call sites.
- **Headers:** use `#pragma once`. Public APIs live in matching `.h` files.
- **Naming:**
  - Functions/macros: `snake_case` for functions, `SCREAMING_SNAKE_CASE` for macros and enums.
  - Static globals: `s_` prefix.
  - File-scope constants: often `k_` or `S_`, but not strictly enforced.
- **Comments:** extensive `//` comments, often with `── section ──` dividers. Comment *why*, especially hardware gotchas and bench-proven facts.
- **ISR attribute:** use `WCH_IRQ` from `include/ch32h417_port.h`. By default it expands to `__attribute__((interrupt("machine")))` which emits correct `mret` on standard GCC. Do **not** use `interrupt("WCH-Interrupt-fast")` unless `USE_WCH_FAST_IRQ=1` is defined and you build with WCH MounRiver GCC.
- **ISR names:** must exactly match the vector-table entries in `core/startup_v3f.S` / `core/startup_v5f.S` (e.g. `IPC_CH0_Handler`, `USBFS_IRQHandler`, `USART1_IRQHandler`). Renaming one without the other silently breaks the IRQ.
- **Hardware access:** use the vendored WCH register/peripheral definitions under `vendor/wch/`; do not hand-roll register defs.
- **Per-core constraints:**
  - V5F ISRs must be FP-free (no hardware FPU context save).
  - V5F hot loop must not write V3F-side SRAM.
  - USB DMA buffers must be in `.usbdma`.
- **No RTOS:** everything is polled or ISR-driven; the main loops use `wfi` when idle.

## Testing strategy

### Host-native unit tests

Run with `make test` (no hardware required):

- `test/humanize_test.c` — conservation, idle gate, cap, anti-quantization, field-clip carry, adaptive interval, dropout rejection.
- `test/motion_test.c` — `act_motion_*` conservation, exact endpoint, bezier curvature, last-writer-wins cancellation.
- `test/display_test.c` — status→text layout, dirty tracking, `icc_status_pack`/`unpack` round-trip.

The Makefile compiles these with the host `cc`, stubbing the ICC injection sinks as `kmbox_cmd_*` where needed. Tests use a simple `CHECK` macro and return non-zero on failure.

### On-hardware tests

The `tools/` Python harnesses speak the wire protocol:

```sh
pip install pyserial

# Ferrum smoke test (direct for PROTOCOL=ferrum, or through hurra-bridge PTY for Hurra)
tools/ferrum_test.py ~/.hurra-bridge.tty smoke
tools/ferrum_test.py /dev/tty.usbserial-XXXX smoke

# Closed-loop aim test
pip install pyserial pynput
tools/ferrum_aim_test.py /dev/tty.usbserial-XXXX

# Load / latency / integrity test
tools/ferrum_load_test.py ~/.hurra-bridge.tty

# Humanization kinematic analysis
tools/humanization_analyze.py trace.txt --baseline human.txt
```

Hardware bring-up typically follows: flash → verify V3F heartbeat on PC2 → verify V5F stage via V3F UART diag → attach real mouse → verify cursor moves through the MITM.

## Security, safety, and legal considerations

- **Synthetic input is powerful and risky.** Injecting input into software or services you do not own or operate may violate terms of service or local law, including anti-cheat policies. The `README.md` contains a full disclaimer. Do not remove or weaken it.
- **No authentication/encryption** is present on the command link. Anyone with access to the serial port can inject input. Treat the command link as a privileged local interface.
- **Do not casually run `git commit`/`git push`/`git reset`/`git rebase`.** Ask for confirmation before any git mutation.
- **Be careful with `make erase` and `make flash`.** They manipulate the target flash and, via `wlink --method power-off`, can gate the board's power rail. Ensure the device-port VBUS jumper issue above is understood before flashing.
- **Avoid writing V3F-side SRAM from V5F.** This can wedge the V5F AHB pipeline. Use IPC MMIO for cross-core communication in the hot path.

## Deployment / flashing process

1. Build: `make all` → produces `build/Merge.bin`.
2. Ensure WCH-LinkE is in RV mode (`wlink mode-switch --rv` if needed).
3. Run `make flash`. It will:
   - auto-detect `wlink` or `wch-openocd`,
   - perform a power-off erase (to defeat the running-firmware `0x55` SWJ-disabled NAK),
   - flash `build/Merge.bin` at the appropriate alias,
   - reset and resume.
4. For single-core bring-up: `make flash-v3f` or `make flash-v5f`. Note that flashing only V3F and waking V5F onto erased flash is a test artifact that can corrupt the shared bus; always verify with the merged image.

## Where to look before changing things

- Changing USB host/device/merge behavior: read `docs/ch32h417-evt-reference.md` §3–§4 and `CLAUDE.md` GOTCHAS.
- Changing command protocols: keep `src/proto.h` aliases symmetric; update both `hurra.c` and `ferrum.c` only if the wire format changes.
- Changing the display/status feature: read `src/display.c` (formatting, host-tested) and `src/st7789.c` (panel driver); status fields arrive over IPC CH2/CH3.
- Adding host-native tests: mirror the `make test` pattern; keep hardware-impure code out of the host-test path.
- Changing board pins/peripherals: update `src/board.h` and confirm against the schematic. The command link USART1 mapping and LED pins were both corrected mid-project; do not resurrect the old PB1/PB10/PB11 assumptions without hardware proof.

## Summary of conventions

- Build: `make all`, `make test`, `make flash`.
- Protocol: default Hurra binary; opt-in Ferrum ASCII with `make PROTOCOL=ferrum`.
- Style: stdint types, `snake_case` functions, `SCREAMING_SNAKE_CASE` macros, `#pragma once`, `WCH_IRQ` for ISRs.
- Cross-core: use IPC MMIO, not shared SRAM, for V5F→V3F in the hot path.
- USB DMA: `.usbdma` section, USBFS buffers need `+0x20000000`, USBHS buffers do not.
- Tests: host-native `make test` plus Python hardware harnesses in `tools/`.
- Git: do not mutate history or push without explicit confirmation.
