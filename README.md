# Hurra v3 — USB HID man-in-the-middle (CH32H417 dual-core RISC-V)

Bare-metal firmware for the **WCH CH32H417** dual-core RISC-V MCU that sits
between a USB HID device and your computer. It enumerates a real HID device
(mouse, keyboard) on its USB **High-Speed host** port, replays that device to
the host PC on its USB **Full-Speed device** port, forwards every real report
through unchanged, and lets you **inject your own mouse and keyboard input** on
top of the live HID stream over a serial command link.

Injected motion is passed through an always-on humanization filter (sub-pixel
jitter, micro-correction, and dwell) so synthetic input blends with the real
device stream. The work is split across the two RISC-V cores: the slow **V3F**
core owns boot, clocks, and the command link; the fast **V5F** core runs the
latency-critical USB relay.

This is a port of [Hurra-v2](../Hurra-v2) (which targeted the NXP i.MX RT1062 /
Teensy MicroMod). The command protocols and host tooling are unchanged; the
hardware abstraction layer (USB stack, UART, clocks, startup, inter-core
channel) was rewritten for the CH32H417.

Two command protocols are supported:

- **Hurra binary** (default) — fast TinyFrame-based protocol, driven by the host app.
- **Ferrum ASCII** (`make PROTOCOL=ferrum`) — text protocol for compatibility with legacy tools.

The host-side companion app is [`hurra-app`](https://github.com/VoltCyclone/hurra-app)
(`hurra-bridge`), which talks the Hurra protocol and also exposes a
Ferrum-compatible virtual COM port for older tooling.

## Status

**Build-complete; on-device validation pending.** The firmware builds clean
(both `PROTOCOL=hurra` and `PROTOCOL=ferrum`, both core images, merged flash
binary) and the full v2 logic — USB host capture, device replay, HID merge,
humanization, both protocol parsers, the inter-core channel — is ported and
host-tested where it can be (`make test`). What has **not** yet happened is
hardware bring-up: flashing a real board, USB enumeration, and the closed-loop
aim/load tests against live silicon. Some pin assignments (LED, command USART)
are placeholders to confirm against the actual board schematic — see
[`src/board.h`](src/board.h). Nothing in this README should be read as
"hardware-verified behavior" yet.

## How it works

```
   USB HID device ──→ CH32H417 USBHS host ──┐
                                            │  (V5F: proxy + merge + humanize)
   Host PC USB ←── CH32H417 USBFS device ───┘
                          ↑ ICC (shared SRAM @0x20178000 + IPC doorbell)
   Host PC USB ──→ USB-UART ──→ CH32H417 USART2 (V3F: parse + command)
```

The real device is captured on the **USBHS** controller (480 Mbps High-Speed
host) and cloned to the PC on the **USBFS** controller (12 Mbps Full-Speed
device); the two controllers run simultaneously. The fast **V5F** core polls the
host endpoints, merges in any injected input, and forwards the combined report
to the PC. Injected input arrives from the **V3F** core: the PC sends commands
over a USB-UART bridge into `USART2`, where V3F parses them (Hurra or Ferrum),
applies humanization control, and pushes injection records to V5F across the
inter-core channel (ICC). Injection rides real reports when the mouse is moving
(merge) or is emitted as standalone synthetic reports when the mouse is idle.

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
- **Command-link USART** — defaults to `USART3` (`PB10` = TX, `PB11` = RX, AF7),
  wired to the **on-board WCH-LinkE virtual COM port** (solder bridges SB3/SB4),
  so one USB-C cable carries flash + debug + command link with no external
  dongle. The Hurra build boots this link at **921600 baud** (the WCH-LinkE VCP
  ceiling). For an external USB-UART bridge at **4 Mbaud**, retarget to `USART2`
  PD5/PD6 and build `CMD_BAUD=4000000` — see [`src/board.h`](src/board.h).
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
WCH-LinkE virtual-COM ceiling for the default USART3 link), so run the bridge
with `--baud 921600`; `km.baud(N)` raises the rate, and the firmware falls back
to the boot default after the link goes idle. On an external USART2 bridge built
with `CMD_BAUD=4000000` you get the full 4 Mbaud / ≥8k commands/sec profile.

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

The build produces **two images** — one per core (V3F master + V5F slave) —
which are converted to raw binaries and merged into a single flash image
(`build/Merge.bin`) by [`scripts/merge_images.py`](scripts/merge_images.py)
(V3F at offset 0, V5F at `0x10000`).

```sh
make all              # build both cores + merge -> build/Merge.bin (Hurra, default)
make v3f              # build only the V3F (master/command) image
make v5f              # build only the V5F (relay) image
make PROTOCOL=ferrum all   # build with the Ferrum ASCII protocol instead
make flash            # merge + program the board via WCH OpenOCD
make clean            # remove the build/ tree
make test             # host-native unit tests (no hardware) — see Test below
```

Toolchain:

- **`riscv-none-elf-gcc`** (MounRiver / xPack) is the default. If you use the
  Homebrew RISC-V toolchain instead, override the prefix:

  ```sh
  make TOOLCHAIN=riscv64-unknown-elf all
  ```

- Both cores compile with **`-march=rv32imac_zicsr -mabi=ilp32`** (soft-float
  ABI — the humanize float math lives on V3F and works fine soft-float).
- **`make flash`** programs `build/Merge.bin` to `0x08000000` via WCH OpenOCD
  (`wch-riscv.cfg`) over a **WCH-LinkE** probe.

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

> The aim/load/smoke tools and `make flash` are part of **on-device** bring-up,
> which is the remaining validation step (see **Status** above).

## Layout

```
Makefile                      two-image RISC-V build (V3F + V5F) + merge
scripts/merge_images.py       merges the two core binaries into build/Merge.bin
core/startup_v3f.S            V3F reset vector / vector table (master)
core/startup_v5f.S            V5F reset vector / vector table (relay)
core/link_v3f.ld              V3F linker script (code in RAM_CODE)
core/link_v5f.ld              V5F linker script (code in ITCM, USB DMA in .usbdma)
core/system_ch32h417.c        clock tree: HSE 25 MHz → 400 MHz, USB PLLs
core/timebase.c               V3F millis() (TIM3)
core/timebase_v5f.c           V5F millis() (TIM4) + 1 MHz µs counter (TIM9)
src/main_v3f.c                V3F entry: clocks → release V5F → command loop
src/main_v5f.c                V5F entry: USBHS host → merge → USBFS device relay
src/icc.c/.h                  inter-core channel (SPSC rings + HSEM + IPC doorbell)
src/usb_host.c/.h             USBHS host driver (capture the real device)
src/usb_device.c/.h           USBFS device driver (clone presented to the PC)
src/usb_merge.c/.h            HID-aware report merge (real report + injection)
src/desc_capture.c/.h         descriptor + HID report-layout capture (cloning)
src/uart.c/.h                 USART2 + DMA RX/TX ring (command link, V3F)
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
