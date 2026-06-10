# Hurra-v3 — CH32H417 dual-core RISC-V port (design)

**Status:** approved design, pre-implementation
**Date:** 2026-06-10
**Predecessor:** `Hurra-v2/` (NXP i.MX RT1062 / Teensy MicroMod, Cortex-M7)

## 1. Goal

Reproduce the full functionality of Hurra-v2 — a bare-metal USB HID
man-in-the-middle that enumerates a real HID device on a USB **host** port,
clones it to the PC on a USB **device** port, forwards every real report
unchanged, and injects synthetic mouse/keyboard input (through an always-on
humanization filter) over a serial command link — on the **WCH CH32H417**, a
dual-core RISC-V MCU with on-chip USB High-Speed host+device and a SuperSpeed
controller.

New repo: `Hurra-v3/` (sibling of `Hurra-v2/`, fresh `git init`). Mirrors v2's
tree: `src/ core/ include/ test/ tools/ docs/ Makefile README.md CLAUDE.md`.

## 2. Target hardware (researched; cite SDK headers during impl)

WCH CH32H417 (released ~Dec 2025; reference manual currently Chinese-only).
Authoritative reference repo: <https://github.com/openwch/ch32h417> (EVT
examples + linker scripts + register headers). Datasheet V1.8.

- **Cores:** QingKe **V5F** (RV32IMAFC, single-precision FPU, up to 400 MHz,
  I-cache, 128 KB ITCM @ `0x200A0000` + 256 KB DTCM @ `0x200C0000`) and QingKe
  **V3F** (RV32IMAFC, FPU, ~150 MHz, runs from `RAM_CODE 0x20100000`, data
  `0x20120000`).
- **Inter-core:** shared SRAM (512 KB region; SDK pins `.shared` at
  `0x20178000`, 32 KB), **HSEM** hardware semaphore unit, **IPC** 4-channel
  mailbox/doorbell.
- **USB:** **one** USBHS (480 Mbps, on-chip HS PHY, host+device, WCH USBHSH/
  USBHSD IP — *not* DWC2/EHCI), one USBFS/OTG_FS (12 Mbps, host+device, FS
  PHY), one USBSS (5 Gbps). HS device regs base `0x40030000`; USB DMA uses a
  `+0x20000000` buffer-pointer translation.
- **Memory:** 960 KB flash @ `0x00000000`, 896 KB SRAM total (incl. TCMs).
- **Clocks:** HSE 25 MHz crystal (required clean for USB PLLs); USBHS needs a
  480 MHz PLL off HSE, USBFS 48 MHz.
- **Peripherals used:** USART (DMA-capable) for the command link, GPIO + a GP
  timer for the LED, 2×16-ch DMA, 2× 32-bit SysTick (one per core), PFIC
  interrupt controller, HSEM, IPC.
- **Toolchain:** `riscv-none-elf-gcc` (MounRiver, GCC 12). Match the EVT SDK:
  `-march=rv32imac -mabi=ilp32` (soft-float ABI — the EVT examples all build
  ilp32 even though the FPU is enabled in `mstatus`; humanize float math lives on
  V3F and works fine soft-float). Flash/debug via WCH-LinkE + WCH OpenOCD
  (`wch-dual-core.cfg`); image is the merged `Merge.bin` @ `0x08000000`.
- **Board:** CH32H417QEU6 (QFN128) USB 3.0 dev board.

### Open risks (front-loaded in phasing)
1. **Dual-core boot/release mechanism** — RESOLVED by extracting the EVT SDK
   (see `docs/ch32h417-evt-reference.md` §1). **V3F is the master/boot core**: it
   boots from `0x00000000`, runs `SystemInit()` (clocks), then releases V5F (boots
   from `0x00010000`) via `NVIC_WakeUp_V5F(0x00010000)`; the two cores then
   rendezvous via HSEM. Two separately-built images merged into one flash binary
   (`Merge.bin` @ `0x08000000`). This *refines* the core split: V3F naturally owns
   boot + clocks + UART + LED and releases V5F; V5F owns the USB hot path. Still
   the highest-complexity area, so Phase 2 proves it before features build on it.
2. USB register offsets must be verified against the openwch EVT headers before
   driver code is written (pull the repo locally as the register source of
   truth).
3. Concurrent host(USBHS)+device(USBFS) confirmed by separate controllers/PHYs;
   verify no shared-PLL constraint on the EVT board.

## 3. Decisions (locked)

| Decision | Choice | Rationale |
|---|---|---|
| USB topology | **USBHS host + USBFS device** | HS capture of up to 8 kHz real mice; PC-side HID at FS (~1 kHz). Both have WCH EVT reference examples. Lowest risk. |
| Core split | **Hot path on V5F, offload on V3F** | V5F runs the latency-critical relay; V3F runs UART parse, humanize math, LED, telemetry. |
| USB driver build | **Rewrite behind existing API, ref WCH EVT** | Keep `usb_host.h`/`usb_device.h` surface; reimplement `.c` against WCH USBHS/USBFS regs. `main.c`/`kmbox.c`/`desc_capture` stay intact. |
| Repo | **New `Hurra-v3/` sibling, own git** | Clean history; v2 untouched. |
| Thermal/overclock | **Dropped** | i.MX 912 MHz tempmon logic is platform-specific and unvalidated on CH32. Run V5F at rated 400 MHz. |

## 4. Architecture & layering

The v2 split between hardware-agnostic logic and a hardware abstraction layer is
preserved. Public headers are the stable seam.

```
PORTS ~VERBATIM (pure C, no hardware registers):
  hurra.c / ferrum.c   proto.h   actions.c   humanize.c   desc_capture.c (logic)
  test/*   tools/*
        │ stable API: usb_host.h · usb_device.h · kmbox.h · humanize.h · actions.h
REWRITTEN for CH32H417:
  usb_host.c (USBHS host)   usb_device.c (USBFS device)   kmbox.c (USART+DMA)
  led.c (GPIO/timer)        main.c (2 core entries + ICC)  icc.c/.h (NEW)
NEW platform:
  include/ch32h417.h (lean regs)   core/startup_v5f.S + startup_v3f.S
  core/link_v5f.ld + link_v3f.ld   core/system_ch32h417.c (clocks)
```

**Reused with little/no change:** `hurra.c`, `ferrum.c`, `proto.h`, `actions.c`,
`humanize.c` (algorithm; see §6 for core-split), `desc_capture.c` (descriptor
cloning logic), `test/humanize_test.c`, `test/motion_test.c`, all of `tools/`
(they speak the wire protocol, not hardware).

**Removed:** `include/imxrt.h` (10k lines), `core/imxrt1062_mm.ld`,
`core/bootdata.c`, i.MX `core/startup.c`, EHCI QH/qTD structures, tempmon /
overclock logic in `main.c`.

## 5. Dual-core split & inter-core channel (ICC)

**V5F (400 MHz) — hot path.** Mirrors v2's `main.c` loop:
`USBHS host poll (real device) → kmbox_merge_report → USBFS device send (to PC)`.
Also: drives injection timing (SysTick-V5F), drains inject-commands from the ICC
ring, pushes telemetry to ICC, applies the final integer per-frame delta
clamp/carry.

**V3F (~150 MHz) — offload.**
`USART RX (DMA) → proto_feed (hurra/ferrum) → act_* → ICC inject-command ring`.
Also: humanize jitter/noise/target-interval (float) compute, LED heartbeat,
telemetry consumption, status.

**ICC (`icc.c`/`icc.h`):**
- Two **SPSC lock-free ring buffers** in shared SRAM (`0x20178000`): V3F→V5F
  (inject commands) and V5F→V3F (telemetry). `volatile` head/tail with RISC-V
  `fence` release/acquire. No locks on the data path.
- **HSEM** only for startup rendezvous (both cores agree the ring is
  initialized) and rare control ops.
- **IPC doorbell:** V3F rings V5F's IPC channel on enqueue so V5F can `wfi` when
  idle (mirrors v2's `wfe`/`SEVONPEND`).
- **Fixed 16-byte records** (tag + payload) → power-of-two ring index, no
  variable framing across the boundary.

Record tags (initial): `INJECT_MOUSE`, `INJECT_KEYBOARD`, `CLICK_RELEASE`,
`KB_RELEASE`, `SET_BAUD`, `SET_HUMAN_LEVEL`, `PHYS_MASK` (V3F→V5F);
`TELEM_COUNTS`, `TELEM_STATUS` (V5F→V3F).

## 6. USB stack rewrite

Both `.c` files reimplemented against WCH registers behind the **unchanged**
public APIs. Reference: openwch EVT `USBHS/HOST/Host_KM`,
`USBFS/DEVICE/CompositeKM`/`CompatibilityHID`.

**Host — `usb_host.c` (USBHS host):**
- Delete EHCI QH/qTD/schedules. USBHSH is register-driven: set RX/TX DMA
  pointers, EP/PID token, toggle (RX/TX CTRL), trigger, poll `USB_INT_FG` for
  done. Firmware issues IN tokens at the enumerated `bInterval` from the V5F
  loop, as today.
- Keep public surface: `usb_host_init`, `usb_host_port_reset`,
  `usb_host_device_speed`, `usb_host_control_transfer(_fire)`,
  `usb_host_control_async_busy`, `usb_host_interrupt_init`,
  `usb_host_interrupt_poll[_zerocopy]`, `usb_host_interrupt_out_init/send`.
- **Zero-copy preserved:** `_poll_zerocopy` returns a pointer into the USBHSH RX
  DMA buffer (apply `+0x20000000` translation). One DMA buffer per host EP slot.

**Device — `usb_device.c` (USBFS device):**
- Delete EHCI dQH/dTD. WCH USBFS device model: per-EP `UEPn_DMA` buffers,
  `UEPn_TX_LEN`, `UEPn_TX_CTRL`/`UEPn_RX_CTRL` with DATA0/1 toggle + ACK/NAK.
  USB interrupt signals transfer-complete / SETUP.
- Keep public surface: `usb_device_init(desc)`, `usb_device_poll`,
  `usb_device_send_report`, `usb_device_is_configured`, `usb_device_poll_out`.
- Descriptor bytes still come from `desc_capture` (cloned real-device
  descriptors) — PC sees same VID/PID/report descriptors. FS caps PC-side HID at
  ~1 kHz; host side still captures at HS.
- EP0 enumeration (SETUP, SET_ADDRESS/SET_CONFIG, GET_DESCRIPTOR) rewritten
  against USBFS EP0 regs (ref `CompositeKM`).

**Shared — DMA buffer placement:** all USB ring/EP buffers live in a dedicated
linker section in the lower SRAM alias visible to the USB DMA engine (never
DTCM/ITCM); `+0x20000000` translation applied where the IP expects it. Called
out because it is the easiest thing to get subtly wrong.

## 7. Platform & build

- **Compiler:** `riscv-none-elf-gcc`, `-march=rv32imafc -mabi=ilp32f`. WCH `xw`
  compressed ext left off unless size demands it.
- **`include/ch32h417.h`:** lean register defs (RCC, USBHS, USBFS, USART, GPIO,
  DMA, PFIC, HSEM, IPC, SysTick, timers) derived/trimmed from openwch EVT
  headers.
- **Two-image build (one per core):**
  - V5F: `startup_v5f.S` + `link_v5f.ld` — vectors, code in ITCM `0x200A0000`,
    data in DTCM `0x200C0000`, `.shared` @ `0x20178000`, USB DMA buffers in
    USB-visible SRAM alias.
  - V3F: `startup_v3f.S` + `link_v3f.ld` — code `0x20100000`, data `0x20120000`,
    **same** `.shared` @ `0x20178000`.
  - `system_ch32h417.c`: HSE 25 MHz → 400 MHz core, USBHS 480 MHz PLL, USBFS
    48 MHz. Adapted from SDK (load-bearing, not written blind).
  - Boot: build both ELFs; flashing lays down both images. HSEM rendezvous at
    startup. A core-enable register, if found, is localized here.
- **Makefile:** keeps v2 structure — `PROTOCOL=hurra|ferrum` selector, hot-path
  files `-O2 -ffast-math`, host-native `make test`. Adds `make v5f`, `make v3f`,
  `make all`, `make flash` (WCH-LinkE). `F_CPU=400000000`.

## 8. Implementation phasing (front-loads unknowns)

1. **Bring-up:** toolchain, linker, startup, clocks, LED blink on V5F. Proves
   flash + boot + clock.
2. **Dual-core hello:** start V3F, ICC ring + HSEM rendezvous + IPC doorbell,
   pass a counter across. Proves the core model + the boot risk.
3. **UART transport:** `kmbox.c` USART+DMA on V3F; loopback against `tools/`.
4. **USB device:** USBFS enumerates a cloned HID to the PC (static descriptor
   first, then `desc_capture`).
5. **USB host:** USBHS enumerates + polls the real device.
6. **Full relay:** host→merge→device + inject across ICC; run aim/load tests.
7. **Humanize split, telemetry, polish.**

## 9. Success criteria

- Real HID device enumerates on USBHS host; its descriptors clone to the PC via
  USBFS device; the PC sees the same device.
- Every real report forwards unchanged; injected mouse/keyboard input arrives
  through the humanization filter.
- Both `PROTOCOL=hurra` and `PROTOCOL=ferrum` builds work; `tools/` smoke, aim,
  and load tests pass against the live firmware.
- Host-native `make test` (humanize + motion) passes unchanged.
- V5F hot path never blocks on UART parsing or float math (V3F offload via ICC).
