# Hurra-v3 Project Memory

## Project
Bare-metal USB HID man-in-the-middle firmware for the **WCH CH32H417 dual-core
RISC-V** MCU. Enumerates a real HID device on a USB **host** port, clones it to
the PC on a USB **device** port, forwards every real report unchanged, and
injects synthetic mouse/keyboard input (through an always-on humanization
filter) over a serial command link.

Ported from **Hurra-v2** (NXP i.MX RT1062 / Teensy MicroMod, Cortex-M7). The
command protocols, host tooling, and hardware-agnostic logic (parsers, actions,
humanize, descriptor capture, tests) carry over; the HAL (USB stack, UART,
clocks, startup, linker, **inter-core channel**) was rewritten for the
CH32H417.

**Status:** build-complete; on-device validation pending. Both `make test` and a
full `make all` build are green. Real-hardware flashing + USB enumeration +
aim/load tests have NOT been done yet.

## Board Target
- **WCH CH32H417QEU6** (QFN128) USB 3.0 dev board — only supported board.
- Two QingKe RISC-V cores:
  - **V3F (~100 MHz)** — the **boot / master** core. Boots from flash, sets
    clocks, releases V5F, then runs boot/clocks/UART/humanize-control/LED/
    command-protocol.
  - **V5F (400 MHz)** — the **relay** core. Runs the USB hot path: USBHS host
    capture → merge → USBFS device.
- Clocks: 25 MHz HSE → 400 MHz core (V5F profile), USBHS 480 MHz PLL, USBFS
  48 MHz. (`core/system_ch32h417.c`.)

## USB topology
- **USBHS** (480 Mbps High-Speed) = **HOST** port — captures the real device
  (up to ~8 kHz mice). WCH USBHSH IP (not DWC2/EHCI).
- **USBFS** (12 Mbps Full-Speed) = **DEVICE** port — clone presented to the PC
  (~1 kHz HID). WCH USBFSD IP.
- Both controllers run **simultaneously** on **separate** controllers/PHYs.

## Inter-core channel (ICC) — `src/icc.c` / `src/icc.h`
- Shared SRAM block at a **fixed address `0x20178000`** (the `.shared` section in
  both link scripts).
- **Two lock-free SPSC rings** of fixed 16-byte records: V3F→V5F = injection
  commands; V5F→V3F = telemetry. `volatile` head/tail + RISC-V fences, no locks
  on the data path.
- **HSEM** (hardware semaphore, ID 0) for the **one-time startup rendezvous**
  only — V3F sets the shared magic before releasing V5F; both cores hand-shake.
- **IPC doorbell** (IPC channel 0): V3F rings V5F on enqueue so V5F can `wfi`
  when idle. `IPC_CH0_Handler` (in `icc.c`) clears it.
- Record tags: `INJECT_MOUSE`, `INJECT_KEYBOARD`, `CLICK_RELEASE`, `KB_RELEASE`,
  `SET_BAUD`, `SET_HUMAN_LEVEL`, `PHYS_MASK` (V3F→V5F); `TELEM_COUNTS`,
  `TELEM_STATUS` (V5F→V3F).

## Protocol
- **Default: Hurra binary** — TinyFrame-based (SOF `0x68`, 1-byte ID/LEN/TYPE,
  CRC16), >=8k cmds/sec at 4 Mbps. Built by `make` (no flag). `src/hurra.c` +
  `src/third_party/TinyFrame/`. Host adapter: `hurra-app` (`hurra-bridge`).
  Boots at **4 Mbaud** (`CMD_BAUD` default). `km.baud(N)` bumps; auto-resets to
  the boot default on RX idle.
- **Opt-in: Ferrum ASCII** (`make PROTOCOL=ferrum`) — `km.<name>(<args>)\r\n`,
  alias `m(x,y)`. Default 115200, resets every power cycle. `km.version()` ->
  `kmbox: Ferrum\r\n`. `src/ferrum.c`.
- Wire formats are **unchanged from v2** (parsers ported nearly verbatim).
- Selector: `src/proto.h` aliases `proto_*` → the chosen parser; call sites have
  no `#ifdef`s.

## Build
- `make all` — build both core images + merge → `build/Merge.bin` (Hurra,
  default).
- `make v3f` / `make v5f` — build a single core image.
- `make PROTOCOL=ferrum all` — Ferrum ASCII protocol instead.
- `make flash` — merge + program `build/Merge.bin` @ `0x08000000` via WCH
  OpenOCD (`wch-riscv.cfg`, WCH-LinkE probe).
- `make clean` — remove `build/`.
- `make test` — host-native unit tests (humanize + motion), no hardware.
- Toolchain prefix: `TOOLCHAIN ?= riscv-none-elf`. For Homebrew RISC-V:
  `make TOOLCHAIN=riscv64-unknown-elf all`.
- Flags: **`-march=rv32imac_zicsr -mabi=ilp32`** (soft-float ABI), `-Os`.
- **Two-image build**: V3F and V5F are linked separately, objcopy'd to raw
  binaries, then merged by `scripts/merge_images.py` (V3F at offset 0, V5F at
  `0x10000`).

> Note: the design spec mentions `-march=rv32imafc -mabi=ilp32f`; the AS-BUILT
> Makefile settled on the **soft-float** `rv32imac_zicsr` / `ilp32` (matches the
> EVT examples). Trust the Makefile.

## Key Files
- `src/main_v3f.c` — V3F entry: `SystemInit` → clocks → `timebase_init` → ICC
  init → `NVIC_WakeUp_V5F` → UART command loop + telemetry-driven LED ladder.
- `src/main_v5f.c` — V5F entry: clocks → TIM4 millis → humanize → ICC
  rendezvous → USBHS host enumerate → USBFS device enumerate → relay loop
  (drain ICC → device_poll → host poll_zerocopy → merge → device_send →
  OUT relay → synth-injection → telemetry).
- `src/icc.c/.h` — inter-core channel (see above).
- `src/usb_host.c/.h` — USBHS host driver (capture real device; zero-copy poll).
- `src/usb_device.c/.h` — USBFS device driver (clone to PC, EP0 enum, EP1 IN).
- `src/usb_merge.c/.h` — HID-report-descriptor-aware merge (real + injected),
  ICC-fed; runs the synth/idle-emit path.
- `src/desc_capture.c/.h` — descriptor + HID report-layout capture (cloning
  logic, ported ~verbatim).
- `src/uart.c/.h` — USART2 + DMA RX/TX ring (V3F command link).
- `src/kmbox_cmd.c/.h` — V3F injection sinks; encode ICC records to V5F.
- `src/kmbox.h` — shim: aliases v2's `kmbox_*` → `kmbox_cmd_*` (inject) / `uart_*`
  (link stats/baud). The HID-merge half of v2's `kmbox` lives on V5F as
  `usb_merge.c`.
- `src/kmbox_cmd_v5f_stub.c` — undefined `kmbox_cmd_*` sinks for the V5F image
  (actions.c links there for the physical-mask state but never injects on V5F).
- `src/hurra.c/.h` — Hurra binary parser (TinyFrame), default.
- `src/ferrum.c/.h` — Ferrum ASCII parser (opt-in).
- `src/proto.h` — compile-time protocol selector.
- `src/actions.c/.h` — transport-agnostic `act_*` injection helpers.
- `src/humanize.c/.h` — always-on humanization filter (jitter, micro-correction,
  sub-pixel carry). **Runs per-frame on V5F** inside the merge.
- `src/led.c/.h` — LED status ladder / heartbeat (TIM2 on V3F).
- `src/board.h` — board pin/clock map (LED, USART). **Pins are placeholders;
  confirm vs schematic.**
- `core/startup_v3f.S` / `core/startup_v5f.S` — per-core reset vectors + vector
  tables.
- `core/link_v3f.ld` / `core/link_v5f.ld` — per-core linker scripts (shared
  `.shared` @ `0x20178000`; V5F has the `.usbdma` section).
- `core/system_ch32h417.c` — clock tree (HSE→400 MHz, USB PLLs).
- `core/timebase.c` — V3F `millis()` (TIM3). `core/timebase_v5f.c` — V5F
  `millis()` (TIM4) + 1 MHz µs counter (TIM9).
- `vendor/wch/` — vendored WCH CH32H417 EVT SDK (Core/Peripheral/Debug).
- `docs/ch32h417-evt-reference.md` — **register + dual-core boot reference**;
  consult this for register offsets / boot details.

## GOTCHAS (read before touching the HAL)
- **V3F is the boot/master core.** It boots from `0x00000000`, runs clocks, then
  releases V5F (which boots from `0x00010000`) via `NVIC_WakeUp_V5F(addr)`
  (`core_riscv.h`). Don't assume V5F self-starts. ICC rings MUST be initialized
  by V3F **before** the wake-up.
- **Code runs from ITCM.** The startup `.S` copies the `highcode`/text section
  from flash → ITCM at reset (`_highcode_lma` → `_highcode_vma_*`). The linker
  places hot code in fast RAM; don't expect XIP from flash.
- **USB DMA buffers live in the `.usbdma` section** (dedicated uncached SRAM,
  `NOLOAD`, in `link_v5f.ld`). USB buffers must be tagged
  `__attribute__((section(".usbdma"), aligned(4)))` — never DTCM/ITCM.
- **USBFS needs the `+0x20000000` CPU-side offset on its DMA buffer pointers;
  USBHS does NOT.** `usb_device.c` (`USBFSD_UEP_BUF`) adds `+0x20000000` to the
  raw DMA addr to get the CPU alias. `usb_host.c` reads/writes USBHS buffers
  directly with no translation. Getting this backwards is the classic
  silent-corruption bug.
- **ISRs use `__attribute__((interrupt("WCH-Interrupt-fast")))`** and the
  function name **must match the vector-table entry** in the corresponding
  `startup_*.S` (e.g. `IPC_CH0_Handler`, `USBFS_IRQHandler`,
  `DMA1_Channel7_IRQHandler`, `TIM2_IRQHandler`). Rename one without the other
  and the IRQ silently never fires.
- **Timers:** `millis()` on **V3F = TIM3**, on **V5F = TIM4**; the µs counter on
  **V5F = TIM9** (32-bit, free-running 1 MHz). The LED heartbeat uses **TIM2**
  on V3F. Keep these distinct per core.
- **`humanize_filter` runs per-frame on V5F** (inside the merge), not on V3F.
  V3F only does humanize-level *control* (sets the level over the ICC).
- **The `kmbox.h` shim** aliases v2's `kmbox_*` symbols to `kmbox_cmd_*` /
  `uart_*` so the ported `actions.c` and parsers link unchanged. The merge half
  of v2's `kmbox.c` is now `usb_merge.c` on V5F.
- **Vendored WCH EVT SDK lives under `vendor/wch/`** — registers/peripherals
  come from there; don't hand-roll register defs.
- **Thermal sensor / overclock logic was DROPPED.** v2's i.MX-specific tempmon /
  912 MHz overclock is gone. V5F runs at its rated 400 MHz. Don't re-add a
  temperature tier to the LED ladder or an overclock path.
