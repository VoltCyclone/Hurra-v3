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

**Status:** build-complete; **boots on real hardware**, USB relay not yet
end-to-end. `make test` + `make all` green. Validated on a CH32H417QEU via the
on-board WCH-LinkE (2026-06-11): `make flash` programs Merge.bin to 0x08000000
and the firmware **runs** — ICC magic `0x48563343` confirmed at 0x20178000, so
V3F boots, sets clocks, inits the rings, and releases V5F; both cores pass the
rendezvous. Still NOT working: USB enumeration to the PC, because the **USBHS
host port supplies no VBUS** to the attached device (a board power matter — see
GOTCHAS). aim/load tests not started.

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
  CRC16). Built by `make` (no flag). `src/hurra.c` + `src/third_party/TinyFrame/`.
  Host adapter: `hurra-app` (`hurra-bridge --baud 921600`). Boots at **921600
  baud** (`CMD_BAUD` Makefile default — the WCH-LinkE VCP ceiling for the default
  USART1/WCH-Link link). `km.baud(N)` bumps; auto-resets to the boot default on
  RX idle. (On an external bridge you can repoint board.h and build a higher
  `CMD_BAUD` for the old 4 Mbaud / >=8k cmds/sec profile.)
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
- `make flash` — merge + program `build/Merge.bin` over the **on-board
  WCH-LinkE**. Auto-detects a CLI flasher: `wlink` (ch32-rs/wlink, Rust — lists
  CH32H417, programs the `0x08000000` alias) first, else the **WCH-OpenOCD fork**
  `wch-openocd` (`scripts/wch-riscv.cfg`, flash bank `0x00000000`). **Mainline
  `openocd` won't work** (no `wlinke` driver). Prints install hints + exits 127
  if neither is found. Overrides: `FLASH_TOOL=wlink|openocd`, `WLINK=`,
  `WCH_OPENOCD=`, `WCH_CFG=`, `FLASH_ADDR=`/`WCH_OCD_ADDR=`.
- `make flash-v3f` / `make flash-v5f` — flash a single core image (bring-up aid).
- `make erase` — full-chip erase via the detected tool.
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
- `src/uart.c/.h` — USART1 (PA9/PA10) interrupt-driven RX/TX rings (V3F command link).
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
- `src/board.h` — board pin/clock map (LED, USART). **Command link = USART1
  PA9(TX)/PA10(RX) AF7**, wired to the on-board WCH-LinkE virtual COM port via
  solder bridges SB3 (WL_UART3_TX→PA10) / SB4 (WL_UART3_RX→PA9) — one USB-C cable
  does flash + debug + command link. **Interrupt-driven (no DMA):** uart.c uses
  the USART1 RXNE/TXE IRQ; the ISR name (CMD_USART_IRQHandler) lives in board.h
  so it can't drift from the vector table. Boot baud = `CMD_BAUD` from the
  Makefile (**921600** Hurra — WCH-LinkE VCP ceiling — / 115200 Ferrum).
  WATCH: the net is named WL_UART3 but lands on PA9/PA10=USART1; PB10/PB11 (the
  target's USART3) go to the SD card, NOT the WCH-Link — an earlier USART3/
  PB10-11 attempt was wrong. LED still PB1 (confirm LED0/LED1 GPIO vs schematic).
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
- **IPC doorbell direction is BENCH-UNVERIFIED.** `icc_init_v3f` configures IPC
  CH0 (TxCID1/RxCID0/AutoEN) and `icc_ring_doorbell_v5f` sets CH0 Bit0 to wake
  V5F; V5F's `IPC_CH0_Handler` watches/clears Bit0. The TxCID/RxCID cross-core
  *routing* semantics are not documented in the EVT tree (the IPC example is a
  bidirectional ping-pong whose comments say "V3F bit0 RX" — the opposite of our
  use). The doorbell is functionally plausible but UNPROVEN. Bench test: from
  V3F call `icc_ring_doorbell_v5f()` on a timer and toggle an LED inside V5F's
  `IPC_CH0_Handler`; if it never toggles, the Bit0 routing is wrong (V5F still
  works via its own wfi-wakes-on-USB path, so injection latency degrades
  silently rather than failing loudly). The ICC magic-spin + data rings do NOT
  depend on the doorbell — only the ≤1 ms idle-injection wake does.
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
- **USBHS host D+/D- = PB8/PB9 = SWCLK/SWDIO.** The USB2 HS data lines share the
  SWJ debug pins, so V5F MUST call `GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,
  ENABLE)` (after enabling `RCC_HB2Periph_AFIO|GPIOB`) before `usb_host_init()`
  or the PHY can't drive the pads and the host never sees a device. This is in
  `main_v5f.c` and is debug-SAFE: the WCH-LinkE debugs over **SDI single-wire**
  (its own pins), NOT PB8/PB9 SWD — verified `wlink` still attaches after the
  remap. Don't "restore" SWJ thinking it's needed for debugging.
- **USBHS host port needs external VBUS.** The chip does not source 5V; the EVT
  host example drives NO VBUS GPIO — VBUS is hardware-supplied on the board. On
  the nanoCH32H417 there's a `SB7`/`U5` 5V path for the host port; if the device
  on the host port gets no power, it's this (a solder bridge / OTG-PD / external
  power matter), NOT firmware. Confirmed open VBUS = no enumeration on bench
  2026-06-11.
- **Debug transport is SDI, and `wlink` sessions wedge under repeated halts.**
  Reading peripheral regs (esp. USBHS 0x4003xxxx) or many halt/resume cycles can
  leave the WCH-LinkE stuck (USB `0x55` protocol error, or it drops to its IAP
  bootloader `4348:55e0`). RECOVERY: `wlink-iap -q` (cjacker/wlink-iap, built at
  /tmp this session — bundles the CH32V305 LinkE firmware too) cleanly exits IAP
  back to RV mode `1a86:8010`. Do NOT use `wlink mode-switch` casually — an
  interrupted switch is what drops it into `55e0`. Prefer reading firmware state
  via the ICC block at 0x20178000 over live register pokes.
- **Toolchain on this Mac:** `make ... TOOLCHAIN=riscv64-unknown-elf`; `wlink`
  lives at `~/.cargo/bin/wlink` (CH32H417 supported). The on-board WCH-LinkE is
  serial `696B8F06EF62`; its VCP is `/dev/cu.usbmodem696B8F06EF62*`.

## FLASHING (RELIABLE METHOD — no button tapping) — 2026-06-12
The firmware disables SWJ (PB8/PB9) during USB init, so once it's running the
debug module NAKs every wlink command with `protocol error: 0x55
[0x81,0x55,0x01,0x01]`. The probe is healthy (1a86:8010 RV mode); it's the
TARGET rejecting debug. Do NOT reset-tap. Instead, **power-off erase**: the
on-board WCH-LinkE powers the target, so it can gate the rail and attach in the
clean reset window automatically.

PREREQUISITE: ONLY the WCH-LinkE USB-C is plugged in (it supplies target 3V3).
If a second cable powers the board independently, power-off can't gate it.

    wlink --chip CH32H41X erase --method power-off          # clean unbrick, no button
    wlink --chip CH32H41X flash -e --address 0x08000000 build/Merge.bin

After the power-off erase the chip is blank (debug stays alive), so normal
status/regs/dump/flash all work without the race. `make flash` should be updated
to do the power-off erase first. Chip = CH32H41X [CH32H417QEU], ChipID
0x4170052d, 960KB. Fallback if power-off ever fails: BOOT0 + wchisp over the ROM
ISP (USB 4348:55e0), or MounRiver Studio on macOS (first-party H417 support).

## BOOT FIX (2026-06-12): "board starts but never blinks" — THREE stacked bugs
1. **LED on wrong pin.** board.h drove PB1 (no LED). The user LEDs are PC2/PC3 —
   confirmed via wuxx's own EVT hardware.c (toggles GPIOC 2/3) + schematic. Fixed
   board.h -> GPIOC/Pin_2. (The EVT main.c comment saying "PB1" is stale.)
2. **ISRs compiled to `ret`, not `mret`.** All handlers used
   `interrupt("WCH-Interrupt-fast")` — a WCH-MounRiver-GCC-only attribute. Stock
   GCC (xPack riscv-none-elf 15.2 / Homebrew riscv64-unknown-elf) silently
   ignores it (just -Wattributes) and emits a plain-function `ret`. The TIM3
   timebase IRQ fired ~1ms post-boot, "returned" into garbage ra -> illegal-instr
   trap (mcause=2) -> core wedged in the weak HardFault stub with IRQs masked ->
   TIM2 heartbeat never ran. Fixed: `WCH_IRQ` macro in include/ch32h417_port.h =
   standard `interrupt("machine")` (emits mret). `-DUSE_WCH_FAST_IRQ` reverts.
3. **HPE double-stacking.** Startup set intsyscr (CSR 0x804) with HWSTKEN(bit0)=1
   (V3F 0x07, V5F 0x0F) — hardware register stacking meant to pair with
   WCH-Interrupt-fast (which does NO software stacking). With our `"machine"`
   handlers (full software stacking), the two corrupt the saved context ->
   erratic IRQs (millis wandered, LED toggle silently didn't land). Fixed: clear
   bit0 -> V3F 0x06, V5F 0x0E in core/startup_*.S.

TOOLCHAIN: build works with BOTH `riscv-none-elf` (xPack, installed via
`xpm install -g @xpack-dev-tools/riscv-none-elf-gcc`) and Homebrew
`riscv64-unknown-elf` (pass TOOLCHAIN=riscv64-unknown-elf) now that ISRs use the
standard attribute. Neither stock GCC supports WCH-Interrupt-fast; only WCH's
MounRiver GCC does.

VERIFIED on hardware 2026-06-12: merged image (V3F 16300B + V5F 22904B) flashed
via `wlink --chip CH32H41X erase --method power-off` then `flash`; PC2 heartbeat
blinks. Dual-core debugging note: test dual-core bugs with the MERGED image — a
V3F-only flash wakes V5F onto erased flash (0xFF) and the wild V5F corrupts the
shared bus, a test artifact that looks like "waking V5F breaks V3F".

## V5F-HANG FIX (2026-06-13): "PC3 blinks slow/irregular, hard to count" —
## the diagnostic WAS the bug, plus the UART stage oracle that found it
SYMPTOM: PC2 (V3F) blinked clean; PC3 (V5F) blinked slow + irregular. ROOT CAUSE:
V5F was frozen at stage 0x52 (ICC_READY), never reaching usb_host_init (0x70).
The freeze was caused by the **PC3 bench-diagnostic itself** — the three
`v5f_diag_raw_blink()` checkpoint trains in main_v5f.c (the "10 long / 1 / 2"
ladder around the AFIO-remap + SWJ-disable). Those helpers call
`RCC_HB2PeriphClockCmd`+`GPIO_Init` on GPIOC and spin multi-million-NOP loops at
400 MHz right at the most clock/timing-sensitive moment of USBHS bring-up.
DELETING the three blink trains (replaced with passive `dbg_stage()` writes) let
V5F advance cleanly to 0x54 HOST_WAITING and hold there steady (87s, no trap,
icc_magic=OK). Lesson: instrumentation that reconfigures clocks/GPIO from the
fast core perturbs the very window it's trying to observe — prefer passive
shared-memory markers over active blink/UART from V5F during bring-up.

THE ORACLE (keep this — it's how the hang was localized): the running V5F
disables SWJ during USB init, so wlink NAKs all SWD with `0x55` and the
0x2017F000 stage marker is unreadable over the probe. But V3F is healthy, owns
USART1 (the WCH-LinkE VCP, /dev/cu.usbmodem696B8F06EF62*), and 0x2017F000 is in
the shared RAM window mapped in BOTH images. So **V3F reads V5F's stage byte from
shared SRAM and prints it over UART** — a probe-free, eyeball-free oracle that
mirrors WCH's own reference (printf over USART). Impl: `diag_v5f_stage_poll()` in
main_v3f.c (default-on; `-DV5F_STAGE_DIAG_OFF` to disable — DO disable when a real
Hurra host app drives the binary protocol, since ASCII lines would corrupt it).
Read it: `python3` open the VCP at 921600 and print. Stage map in src/icc.h
(0x51..0x58 main path, 0x60..0x68 early/pre-USBHS, 0x70..0x76 usb_host_init,
0x8x = V5F trap with mcause/mepc stamped at 0x2017F004/8). The V5F HardFault
handler also stamps an 0x8x marker so a trap surfaces as a UART line, not a blink.
PC3 now has only 3 states: dark (pre-host) / ~2 Hz (host-waiting) / trap-blink.

NEXT BLOCKER (unchanged, hardware): V5F sits at 0x54 HOST_WAITING because the
USBHS host port sources no VBUS (SB7/U5 5V path — see GOTCHAS). Power a device on
the host port externally and watch for 0x55→0x56→0x57→0x58 RELAY on the VCP.
