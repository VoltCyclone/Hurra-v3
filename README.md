# Hurra v3 — USB HID man-in-the-middle (CH32H417 dual-core RISC-V)

Bare-metal firmware for the **WCH CH32H417** dual-core RISC-V MCU that sits
between a USB HID device and your computer. It enumerates a real HID device
(mouse, keyboard) on a **USB High-Speed host** port, replays that device to the
PC on a **USB device** port, forwards every real report through unchanged, and
lets you **inject your own mouse and keyboard input** on top of the live HID
stream over a serial command link.

The default build is a **two-board** man-in-the-middle: one board (B) captures
the real device on its USBHS host port and ships the descriptors and reports over
a board-to-board SPI link to a second board (A), which clones the device to the
PC. A **single-board** relay (one board doing both halves) is also available
(`make relay`). Either way, injected motion is passed through an always-on
humanization filter (sub-pixel jitter, micro-correction, and dwell) so synthetic
input blends with the real device stream. The work is split across the two RISC-V
cores: the slow **V3F** core owns boot, clocks, and the command link; the fast
**V5F** core runs the latency-critical USB path.

This is a port of Hurra-v2 (which targeted the NXP i.MX RT1062 / Teensy
MicroMod). The command protocols and host tooling are unchanged; the hardware
abstraction layer (USB stack, UART, clocks, startup, inter-core channel) was
rewritten for the CH32H417.

Two command protocols are supported:

- **Hurra binary** (default) — fast TinyFrame-based protocol, driven by the host app.
- **Ferrum ASCII** (`make PROTOCOL=ferrum`) — text protocol for compatibility with legacy tools.

The host-side companion app is [`hurra-app`](https://github.com/VoltCyclone/hurra-app)
(`hurra-bridge`), which talks the Hurra protocol and also exposes a
Ferrum-compatible virtual COM port for older tooling.


## Requirements

- A **WCH CH32H417** board with both USB ports broken out (this was developed on
  a CH32H417QEU6 dev board). The on-board WCH-LinkE carries flash, debug, and the
  command link over one USB-C cable.
- A **RISC-V GCC toolchain**: `riscv-none-elf-gcc` (MounRiver / xPack, default)
  or `riscv64-unknown-elf-gcc` via `make TOOLCHAIN=riscv64-unknown-elf`.
- A **CH32-aware flasher**: [`wlink`](https://github.com/ch32-rs/wlink)
  (recommended) or the [WCH-OpenOCD fork](https://github.com/openwch/openocd_wch).
  Mainline OpenOCD does **not** support the WCH-LinkE adapter.
- **Python 3 + `pyserial`** for the `tools/` harnesses (optional: `pynput` for
  the aim test).

## How it works

Two-board (default):

```
   USB HID device ──→ Board B USBHS host ──→ SPI link ──→ Board A USB device ──→ PC
                      (capture + forward)    (PA3–PA7)    (clone + merge)
```

Single-board (`make relay`):

```
   USB HID device ──→ CH32H417 USBHS host ──┐
                                            │  (V5F: proxy + merge + humanize)
   PC USB ←──────── CH32H417 USB device ────┘
                          ↑ ICC (shared SRAM @0x20178000 + IPC doorbell)
   PC USB ──→ WCH-Link VCP ──→ CH32H417 USART1 (V3F: parse + command)
```

The real device is captured on the **USBHS** controller (480 Mbps High-Speed
host) and cloned to the PC on a USB device controller (USBFS USB-C for Full/Low
speed, or USBHSD Type-A for High-Speed). In the two-board build, capture and
clone live on separate boards joined by a board-to-board SPI link; in the
single-board build both run on one chip simultaneously. Either way the fast
**V5F** core handles the report path, merging in injected input, while the
**V3F** core takes commands from the PC over the WCH-Link VCP into `USART1`,
parses them (Hurra or Ferrum), applies humanization, and pushes injection records
to V5F across the inter-core channel (ICC). Injection rides real reports when the
mouse is moving (merge) or is emitted as standalone synthetic reports when idle.

## Hardware

- **WCH CH32H417QEU6** (QFN128) USB 3.0 development board.
- **Dual-core RISC-V**, both QingKe cores:
  - **V3F** (~100 MHz) — the **boot / master** core. Sets up clocks, releases
    V5F, and runs the command side: USART command link, protocol parser,
    humanize-level control, LED status.
  - **V5F** (400 MHz) — the **relay** core. Runs the latency-critical USB hot
    path: capture on USBHS host → merge → forward on USBFS device.
- **USB topology** — two on-chip controllers, running at the same time:
  - **USBHS** (480 Mbps High-Speed) = **host** port — captures the real
    mouse/keyboard at High-Speed (up to ~8 kHz mice).
  - **USBFS** (12 Mbps Full-Speed) = **device** port — the clone presented to
    the PC. Full-Speed caps the PC-facing HID at ~1 kHz.
- **Command-link USART** — `USART1` (`PA9` = TX, `PA10` = RX, AF7),
  interrupt-driven, wired to the **on-board WCH-LinkE virtual COM port** (solder
  bridges SB3→PA10 / SB4→PA9), so one USB-C cable carries flash + debug +
  command link with no external dongle. The Hurra build boots it at **921600
  baud** (the WCH-LinkE VCP ceiling). To use an external USB-UART bridge,
  repoint the port in [`src/board.h`](src/board.h) and raise `CMD_BAUD`.
- A 25 MHz HSE crystal feeds the USB PLLs (480 MHz for USBHS, 48 MHz for USBFS).

## Dual-core architecture

The two cores divide the work along the latency boundary:

- **V3F (master / command core)** — boots from flash, runs `SystemInit()`
  (clocks), releases V5F via `NVIC_WakeUp_V5F`, then runs the command loop:
  `USART RX (DMA) → proto_feed (hurra/ferrum) → act_* → ICC inject-command ring`.
  It also drives the LED status ladder from telemetry it consumes back over the
  ICC.
- **V5F (relay / hot path)** — the centerpiece of the man-in-the-middle:
  `USBHS host poll (real device) → merge (drain ICC injection + humanize) →
  USBFS device send (to PC)`. It also emits standalone synthetic reports when
  the physical mouse is idle, and streams forwarded/dropped report telemetry
  back to V3F.

### Inter-core channel (ICC)

The cores talk over a small shared-SRAM mailbox at a fixed address
(`0x20178000`), wired up in [`src/icc.c`](src/icc.c) / [`src/icc.h`](src/icc.h):

- **Two lock-free SPSC ring buffers** of fixed 16-byte records: V3F→V5F carries
  injection commands (`INJECT_MOUSE`, `INJECT_KEYBOARD`, `CLICK_RELEASE`,
  `KB_RELEASE`, `SET_BAUD`, `SET_HUMAN_LEVEL`, `PHYS_MASK`); V5F→V3F carries
  telemetry (`TELEM_COUNTS`, `TELEM_STATUS`). `volatile` head/tail with RISC-V
  fences; no locks on the data path.
- **HSEM** (hardware semaphore) is used only for the one-time startup
  rendezvous — V3F initializes the rings and sets a magic value before releasing
  V5F, and the two cores hand-shake via HSEM ID 0.
- An **IPC doorbell** (IPC channel 0) lets V3F wake V5F from `wfi` when it
  queues injection, so the hot path can sleep when there is no work.

## Command protocols

The wire protocols are **identical to Hurra-v2** — the parsers (`hurra.c`,
`ferrum.c`) were ported essentially verbatim, since they speak the protocol and
touch no hardware.

**Hurra binary (default).** TinyFrame framing — SOF `0x68`, 1-byte ID/LEN/TYPE,
CRC16, little-endian payloads. Driven by `hurra-app` / `hurra-bridge`; see that
repo for the host API. The firmware boots at **921600 baud** (the on-board
WCH-LinkE virtual-COM ceiling for the default USART1 link), so run the bridge
with `--baud 921600`; `km.baud(N)` raises the rate, and the firmware falls back
to the boot default after the link goes idle. On an external bridge (repoint
board.h) built with a higher `CMD_BAUD` you get the full 4 Mbaud / ≥8k cmds/s.

**Ferrum ASCII (`make PROTOCOL=ferrum`).** `\r\n`-terminated text commands at
115200 baud (reset to 115200 on every power cycle). Reference:
<https://ferrumllc.github.io/print.html>.

```
TX: km.version()\r\n
RX: kmbox: Ferrum\r\n
TX: km.move(10, -5)\r\n          # write — no reply
TX: m(2, 0)\r\n                  # alias for km.move
```

The protocol is selected at compile time — [`src/proto.h`](src/proto.h) aliases
`proto_*` to the chosen parser so the command core calls into it with no
`#ifdef`s at the call sites.

## Build & flash

Each flash image is two core images (V3F + V5F) merged by
[`scripts/merge_images.py`](scripts/merge_images.py) (V3F at offset 0, V5F at
`0x10000`). The default build produces **both two-board role images**.

```sh
make all              # build both role images -> build/BoardA.bin + build/BoardB.bin
make relay            # single-board build -> build/Merge.bin
make PROTOCOL=ferrum all   # build with the Ferrum ASCII protocol instead

# Two-board: flash one board at a time over its WCH-LinkE.
make flash-boardb     # Board B (host: capture + SPI master)
make flash-boarda     # Board A (device: SPI slave + clone to PC)

# Single-board:
make flash            # build relay + program over the on-board WCH-LinkE

make erase            # full-chip erase
make clean            # remove the build/ tree
make test             # host-native unit tests (no hardware) — see Test below
```

For the two-board build, wire the **A3–A7 header pins** (SPI1: NSS/SCK/MISO/MOSI
+ DATA_READY) and a common ground across the boards, plug the real device into
Board B's USBHS host port, and plug Board A into the PC.

Toolchain:

- **`riscv-none-elf-gcc`** (MounRiver / xPack) is the default. If you use the
  Homebrew RISC-V toolchain instead, override the prefix:

  ```sh
  make TOOLCHAIN=riscv64-unknown-elf all
  ```

- V3F compiles with **`-march=rv32imac_zicsr -mabi=ilp32`** (soft-float); V5F
  uses the hardware FPU (`rv32imafc` / `ilp32f`).

Flashing goes over the **on-board WCH-LinkE** (same USB-C cable as the command
link):

- The `flash*` targets auto-detect a CLI flasher — preferring
  [`wlink`](https://github.com/ch32-rs/wlink) (Rust; lists CH32H417 support) and
  falling back to the [WCH-OpenOCD fork](https://github.com/openwch/openocd_wch)
  (`wch-openocd`). If neither is installed they print install hints and stop.
  **Mainline `openocd` does not work** — it has no `wlinke` adapter driver.
- Install the recommended tool: `brew install libusb && cargo install --git
  https://github.com/ch32-rs/wlink`. Ensure the WCH-LinkE is in **RV mode**
  (`wlink mode-switch --rv`).
- A running target NAKs `wlink` (it disables SWJ during USB init); the flash
  targets do a power-off erase first, so flashing always works without
  button-tapping.
- Overrides: `FLASH_TOOL=wlink|openocd`, `WLINK=`, `WCH_OPENOCD=`, `WCH_CFG=`.

## Test

The `tools/` scripts speak the **wire protocol**, so they are unchanged from v2.

`tools/ferrum_test.py` speaks Ferrum ASCII. Point it at the serial port of a
`PROTOCOL=ferrum` build, **or** at the `hurra-bridge` PTY symlink
(`~/.hurra-bridge.tty`) when running the default Hurra firmware — not directly
at a Hurra build's port.

```sh
pip install pyserial

# Smoke test via the bridge (default Hurra firmware)
tools/ferrum_test.py ~/.hurra-bridge.tty smoke

# Smoke test direct (PROTOCOL=ferrum build)
tools/ferrum_test.py /dev/tty.usbserial-XXXX smoke
```

The smoke test handshakes `km.version()`, nudges the mouse, exercises the
buttons and wheel, and validates the read forms.

**Closed-loop aim test** — drives the cursor toward on-screen dots:

```sh
pip install pyserial pynput
tools/ferrum_aim_test.py /dev/tty.usbserial-XXXX
```

**Load test** — measures latency, throughput, and integrity of the command
channel under sustained load:

```sh
tools/ferrum_load_test.py ~/.hurra-bridge.tty
```

**Humanization analyzer** — compares a captured motion trace against a real
human baseline to check the kinematic signatures anti-cheat detectors look for:

```sh
tools/humanization_analyze.py trace.txt --baseline human.txt
```

**Host-native unit tests** for the humanization filter and motion helpers run
without any hardware (these pass today):

```sh
make test
```

> The aim/load/smoke tools and `make flash` run against a flashed board; the
> host-native `make test` suite needs no hardware.

## Layout

```
Makefile                      two-image RISC-V build (V3F + V5F) + merge per role
scripts/merge_images.py       merges the two core binaries into one flash image
core/startup_v3f.S            V3F reset vector / vector table (master)
core/startup_v5f.S            V5F reset vector / vector table (relay)
core/link_v3f.ld              V3F linker script (code in RAM_CODE)
core/link_v5f.ld              V5F linker script (code in ITCM, USB DMA in .usbdma)
core/system_ch32h417.c        clock tree: HSE 25 MHz → 400 MHz, USB PLLs
core/timebase.c               V3F millis() (TIM3)
core/timebase_v5f.c           V5F millis() (TIM4) + 1 MHz µs counter (TIM9)
src/main_v3f.c                V3F entry: clocks → release V5F → command loop
src/main_v5f.c                V5F entry: single-board relay; diverts to two_board on BOARD=
src/two_board.c/.h            two-board role loops (host = capture+SPI; device = SPI+clone)
src/spi_link.c/.h             board-to-board SPI1 link (PA3–PA7) driver
src/spi_frame.c / spi_frame_stream.c  SPI slot framing + SOF-scanning reassembly
src/desc_xfer.c/.h            descriptor-blob chunk/reassemble codec over SPI
src/synth_mouse.c/.h          synthetic mouse source (isolated-link bench builds)
src/icc.c/.h                  inter-core channel (SPSC rings + HSEM + IPC doorbell)
src/usb_host.c/.h             USBHS host driver (capture the real device)
src/usb_device.c/.h           device clone dispatch (USBHSD for HS / USBFS for FS-LS)
src/usb_merge.c/.h            HID-aware report merge (real report + injection)
src/desc_capture.c/.h         descriptor + HID report-layout capture (cloning)
src/uart.c/.h                 USART1 PA9/PA10 IRQ-driven RX/TX rings (cmd link, V3F)
src/kmbox_cmd.c/.h            V3F injection sinks → ICC records to V5F
src/kmbox.h                   shim: aliases v2's kmbox_* → kmbox_cmd_* / uart_*
src/hurra.c/.h                Hurra binary parser (TinyFrame) — default protocol
src/ferrum.c/.h               Ferrum ASCII parser (opt-in: PROTOCOL=ferrum)
src/proto.h                   compile-time protocol selector
src/actions.c/.h              transport-agnostic injection helpers (act_*)
src/humanize.c/.h             always-on humanization filter (runs per-frame on V5F)
src/led.c/.h                  LED status ladder / heartbeat (TIM2 on V3F)
src/board.h                   board pin/clock map (LED, USART — confirm vs schematic)
src/third_party/TinyFrame/    TinyFrame framing library (Hurra protocol)
vendor/wch/                   vendored WCH CH32H417 EVT SDK (registers + peripherals)
docs/ch32h417-evt-reference.md  register / dual-core boot reference
tools/ferrum_test.py          protocol smoke harness
tools/ferrum_aim_test.py      closed-loop aim test against on-screen dots
tools/ferrum_load_test.py     command-channel latency/throughput/integrity load test
tools/humanization_analyze.py kinematic trace analyzer vs. a human baseline
```

## Disclaimer

Hurra is published for research, education, and legitimate input-automation and
accessibility work. Injecting synthetic input into software or services you do
not own or operate may violate their terms of service or local law — including
the anti-cheat policies of online games. You are responsible for how you use it.
The software is provided "as is", without warranty of any kind (see
[LICENSE](LICENSE)); the authors accept no liability for misuse or damage.

## License

[MIT](LICENSE) © 2026 Ramsey McGrath.

Third-party components keep their own licenses: the vendored WCH CH32H417 EVT SDK
under `vendor/wch/` and the TinyFrame framing library under
`src/third_party/TinyFrame/`.
