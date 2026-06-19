# WCH CH32H417 Best-Practice Fixes — Implementation Plan

> **Goal:** Apply the findings from the 2026-06-17 WCH best-practice review. Fix critical runtime bugs, harden the ICC/USB/peripheral layers, and clean up build/linker fragility while preserving the existing dual-core architecture and command protocols.
>
> **Scope:** `core/`, `src/`, `Makefile`, `AGENTS.md`. Host-native tests and a hardware smoke test are required before the plan is considered complete.

---

## Executive summary

The review found **two critical bugs** that can wedge or corrupt the device, plus a set of high-value warnings and hygiene issues. This plan groups them into five implementation phases. Each phase is self-contained and can be built/tested independently, but they should be applied in order because later phases assume earlier ones are clean.

| Phase | Theme | Priority | Approx. files changed |
|-------|-------|----------|----------------------|
| 1 | Critical runtime fixes | P0 | 3 |
| 2 | USB host/device hardening | P0/P1 | 4 |
| 3 | ICC/inter-core cleanup | P0/P1 | 3 |
| 4 | Peripheral / ISR tuning | P0/P1 | 5 |
| 5 | Build / linker / docs | P1/P2 | 4 |

---

## Phase 1 — Critical runtime fixes

### 1.1 Fix control-transfer NAK timeout bug
**File:** `src/usb_host.c`
**Why P0:** A NAKing control or interrupt-OUT transfer can spin V5F for tens of seconds.

Current code converts `timeout_ms` to a retry count (`timeout_ms * 1000`) and only decrements it when `< 0xFFFF`. Real callers pass `2000`, so the counter never decrements.

**Changes:**
1. Replace the retry-count semantics with a true microsecond deadline.
2. Add `timebase_v5f_us()`-based guard in `usbhs_transact()`.
3. Give interrupt-OUT its own 5 ms budget (`DEF_INTR_OUT_TIMEOUT_US`).
4. Pass remaining time from `usb_host_control_transfer()` to each stage.

Key edits:
- New constants near line 64:
  ```c
  #define DEF_CTRL_TRANS_TIMEOUT_US   2000000u
  #define DEF_INTR_OUT_TIMEOUT_US     5000u
  ```
- Rewrite `usbhs_transact()` to use `uint32_t timeout_us` and a deadline check.
- Add helper `remaining_us()` and use it for SETUP/DATA/STATUS stages.
- Update `usb_host_interrupt_out_send()` to pass `DEF_INTR_OUT_TIMEOUT_US`.

**Verification:**
- `make all` clean.
- Attach real mouse/keyboard; confirm enumeration reaches relay stage.
- V3F heartbeat LED must keep blinking during and after enumeration.
- Send a host-to-device SET_REPORT (keyboard LED); V5F must not freeze.

### 1.2 Harden vector-table alignment
**Files:** `core/startup_v3f.S`, `core/startup_v5f.S`
**Why P1:** `.align 1` gives only 2-byte alignment; `mtvec` vectored mode requires 4-byte alignment.

**Change:**
```asm
    .section    .vector,"ax",@progbits
    .align  2
_vector_base:
```
in both files.

**Verification:**
- Build and inspect map file; `_vector_base` must be 4-byte aligned.
- Confirm interrupts still work (TIM2 heartbeat, USART1, USBHS, USBFS).

---

## Phase 2 — USB host/device hardening

### 2.1 Per-interface HID idle/protocol arrays
**File:** `src/usb_device.c`
**Why P1:** Composite HID clones share a single idle/protocol value across interfaces.

**Changes:**
- Replace globals with arrays:
  ```c
  static volatile uint8_t USBFS_HidIdle[MAX_INTERFACES];
  static volatile uint8_t USBFS_HidProtocol[MAX_INTERFACES];
  ```
- In `SET_IDLE`/`SET_PROTOCOL`/`GET_IDLE`/`GET_PROTOCOL`, validate `wIndex` (interface) and index into arrays.
- Use `USBFSD_UEP_BUF(0)` for GET_IDLE/GET_PROTOCOL payloads (ties to 2.5).

### 2.2 Apply `FORCE_FS` for low-speed root-port devices
**File:** `src/usb_host.c`
**Why P1:** Low-speed devices need `USBHS_UH_FORCE_FS` set in `CFG`.

**Change:** In `usb_host_device_speed()`, set `USBHSH->CFG |= USBHS_UH_FORCE_FS;` only when detected speed is `USB_SPEED_LOW`. Do **not** clear it for FS/HS.

### 2.3 Interrupt-IN max packet size to `uint16_t` / 512
**File:** `src/usb_host.c`
**Why P1:** `intr_slot_t.maxpkt` is `uint8_t` and capped at 255; HS interrupt endpoints can be 512 bytes.

**Changes:**
- Change `maxpkt` fields in `intr_slot_t` and `out_slot_t` to `uint16_t`.
- Remove the 255 cap in `usb_host_interrupt_init()` and `usb_host_interrupt_out_init()`.

### 2.4 Handle `SET_CONFIGURATION(0)` as de-configure
**File:** `src/usb_device.c`
**Why P1:** Currently any `SET_CONFIGURATION` marks the device configured.

**Change:**
```c
case USB_SET_CONFIGURATION:
    USBFS_DevConfig = (uint8_t)(USBFS_SetupReqValue & 0xFF);
    s_configured = (USBFS_DevConfig != 0);
    if (!s_configured) {
        s_in_ep_mask  = 0;
        s_out_ep_mask = 0;
        USBFSD->UEP4_1_MOD = 0;
        USBFSD->UEP2_3_MOD = 0;
        USBFSD->UEP5_6_MOD = 0;
        USBFSD->UEP7_MOD   = 0;
    }
    break;
```

### 2.5 Document EP0 SET_REPORT back-pressure
**File:** `src/usb_device.c`
**Why P1:** Current behavior (drop+ACK while pending) is correct but not obvious.

**Change:** Update the comment in the `HID_SET_REPORT` case to explain the intentional drop.

### 2.6 Add merge length checks and grow synth buffer
**File:** `src/usb_merge.c`
**Why P0:** Short/malformed reports cause out-of-bounds access; synth path can read past a 16-byte stack buffer.

**Changes:**
- In `usb_merge_report()` fast path, verify `len` covers `data_off`, `x_byte`, `y_byte`, and wheel byte before indexing.
- In slow path and physical-mouse path, guard `report[doff]` with `len` checks.
- Grow `synth[16]` to `synth[64]` and clamp `rlen` to `sizeof(synth)`.

### 2.7 Propagate USBHS PLL lock failure
**File:** `src/usb_host.c`, `src/main_v5f.c`
**Why P1:** `usb_host_init()` returns `true` even if the 480 MHz PLL never locks.

**Changes:**
- Change `usbhs_rcc_init()` to return `bool`.
- Return false if `RCC_USBHS_PLLRDY` is not set after the wait loop.
- In `usb_host_init()`, return false on RCC failure.
- In `main_v5f.c`, halt with a fatal blink pattern if `usb_host_init()` fails.

### 2.8 Route all EP0 CPU accesses through `USBFSD_UEP_BUF(0)`
**File:** `src/usb_device.c`
**Why P0:** EP0 currently bypasses the required `+0x20000000` CPU alias; can cause silent DMA corruption.

**Changes:**
- Update macro:
  ```c
  #define pUSBFS_SetupReqPak ((PUSB_SETUP_REQ)USBFSD_UEP_BUF(0))
  ```
- Replace every direct `USBFS_EP0_Buf[...]` CPU access with `USBFSD_UEP_BUF(0)[...]`.
- This includes descriptor copies, SET_REPORT capture, GET_IDLE/GET_PROTOCOL, GET_CONFIGURATION, and GET_STATUS.

**Verification for Phase 2:**
- `make all && make test`.
- Enumeration with real mouse/keyboard.
- Composite mouse+keyboard test if available.
- Host-to-device SET_REPORT (keyboard LEDs, Razer/Logitech config writes).
- Confirm no regression on FS/HS mice after `FORCE_FS` change.

---

## Phase 3 — ICC / inter-core cleanup

### 3.1 Remove vestigial HSEM rendezvous
**Files:** `src/main_v3f.c`, `src/main_v5f.c`
**Why P0:** V3F never takes `HSEM_ID0`, so V5F's take/release is a no-op that looks like synchronization.

**Changes:**
- In `main_v3f.c`, update the comment to state HSEM is intentionally unused.
- In `main_v5f.c`, remove `HSEM_FastTake`/`HSEM_ReleaseOneSem` and `dbg_stage(DBG_V5F_HSEM_DONE)`.
- Leave `DBG_V5F_HSEM_DONE` enum unused; no harm.

### 3.2 Warm-reset-safe IPC status clearing
**File:** `src/icc.c`
**Why P0:** `IPC_DeInit()` writes `STS = 0`, which may not clear sticky bits across warm resets.

**Change:** After `IPC_DeInit()`, add:
```c
IPC->CLR = 0xFFFFFFFFu;
```
with a comment explaining the warm-reset safety rationale.

### 3.3 Verify IPC CH0 doorbell wake
**Files:** `src/icc.c`, `src/main_v5f.c`
**Why P1:** The doorbell path is correct by inspection but was previously bench-unverified.

**Verification (no code change unless it fails):**
1. Build and flash with current code.
2. Run `tools/ferrum_load_test.py` at high rate.
3. Confirm injection latency does not systematically increase by ~1 ms.
4. Optional negative test: temporarily comment out `icc_ring_doorbell_v5f()`; latency should rise by up to 1 ms. Restore after test.
5. If wake fails, adjust `TxCID/RxCID/AutoEN` or the bit index until reliable.

### 3.4 Reduce cross-core SRAM writes in V5F trap handlers
**Files:** `src/main_v5f.c`, `src/main_v3f.c`
**Why P1:** HardFault writes four cross-core words; two are redundant.

**Changes:**
- In `HardFault_Handler`, remove the `DBG_STAGE_ADDR+4` and `DBG_STAGE_ADDR+8` writes.
- In `main_v3f.c`, remove the block that reads those redundant words from `DBG_STAGE_ADDR+4/+8`.
- Keep the `0x2017F0E0/E4/E8` witness writes and the `dbg_stage(0x8x)` marker.

**Verification for Phase 3:**
- `make all && make test`.
- Flash; V3F banner appears, V5F reaches `ICC_READY` then relay loop.
- Induce a V5F illegal-instruction fault; confirm V3F still prints full `mcause`/`mepc` from `0x2017F0E4/E8`.

---

## Phase 4 — Peripheral / ISR tuning

### 4.1 USART ORE/FE/NE handling
**File:** `src/uart.c`
**Why P0:** Hardware overrun/framing/noise errors are not cleared or counted.

**Change:** In `CMD_USART_IRQHandler`, read `CMD_USART->STATR` before `USART_ReceiveData()`, increment `err_or`/`err_fe`/`err_ne`, and drop bytes with `FE`/`NE`.

### 4.2 ADC temp sensor sample time
**File:** `src/temp.c`
**Why P0:** Current sample time undersamples the temp sensor vs. datasheet.

**Changes:**
- Set ADCCLK = HCLK/8 via `RCC_ADCHCLKCLKAsSourceConfig(RCC_PPRE2_DIV0, RCC_HCLK_ADCPRE_DIV8)`.
- Use `ADC_SampleTime_CyclesMode7` for the temp-sensor channel.

### 4.3 ST7789 SPI clock
**File:** `src/st7789.c`
**Why P1:** Current SPI2 SCK ~50 MHz exceeds reliable ST7789 operation.

**Change:** Use `SPI_BaudRatePrescaler_Mode2` (~12.5 MHz) with a comment explaining the margin.

### 4.4 Bound `display_render()` blocking time
**File:** `src/display.c`
**Why P1:** Full-screen refresh can block the V3F command loop for 10–100 ms.

**Change:** Render at most one dirty row per call, rotating the start row so no line starves.

### 4.5 TIM2 auto-reload preload + update event
**File:** `src/led.c`
**Why P1:** Rate changes can produce a partial/wrong first period.

**Changes:**
- Enable ARR preload in `led_heartbeat_start()`.
- Generate a software update event after rate changes.
- Disable/enable the update interrupt around runtime rate changes to avoid spurious ISRs.

**Verification for Phase 4:**
- `make all && make test`.
- Run `tools/ferrum_load_test.py`; command link must not desync.
- Confirm `temp_read_c()` returns sane values.
- Confirm display text is clean; no SPI artifacts.
- Change humanize level and confirm heartbeat LED stays steady.

---

## Phase 5 — Build / linker / docs

### 5.1 Collect `.fastrun` in V3F linker script
**File:** `core/link_v3f.ld`
**Why P1:** V3F script does not collect `.fastrun`; future V3F use of hot-path functions would orphan the section.

**Change:** Add `*(.fastrun)` and `*(.fastrun.*)` to the `.highcode` output section, mirroring `core/link_v5f.ld`.

### 5.2 512-align V5F `.usbdma` section VMA
**File:** `core/link_v5f.ld`
**Why P0:** USBHS DMA buffers require 512-byte alignment; current internal `ALIGN(512)` does not force the section base.

**Change:**
```ld
.usbdma (NOLOAD) : ALIGN(512)
{
    PROVIDE(_usbdma_start = .);
    *(.usbdma)
    *(.usbdma.*)
    . = ALIGN(4);
    PROVIDE(_usbdma_end = .);
} >RAM
```

### 5.3 Document `EXTRADEF` behavior
**Files:** `Makefile`, `AGENTS.md`
**Why P1:** `EXTRADEF` is documented as “extra -D flags” but V5F appends `-funroll-loops` after it.

**Changes:**
- Add a Makefile comment explaining the V5F append.
- Update the `AGENTS.md` table row to describe the actual behavior.

### 5.4 Refactor warning flags
**File:** `Makefile`
**Why P2:** Warning policy is inline and undocumented; vendor header noise drowns project warnings.

**Change:**
```makefile
WARNINGS = -Wall -Wno-unused-variable
CFLAGS   = $(ARCH) $(DEFINES) $(EXTRADEF) $(VPATH_INC) -Os $(WARNINGS) \
           -ffunction-sections -fdata-sections -fsingle-precision-constant $(FP_CFLAGS)
```
with a comment explaining why `-Werror` is not applied globally.

**Verification for Phase 5:**
- `make clean && make all` with no new warnings.
- `make test` still passes.
- Inspect `build/v5f.map`: `_usbdma_start` must be 512-byte aligned.
- Inspect `build/v3f.map`: `.fastrun` is inside `.highcode` and not an orphan.

---

## Cross-phase build & test checklist

Before declaring the plan complete:

- [ ] `make clean`
- [ ] `make all` — both Hurra and Ferrum protocols (`make PROTOCOL=ferrum all`)
- [ ] `make test` — all three host-native test suites pass
- [ ] Map-file checks:
  - [ ] V3F `_vector_base` 4-byte aligned
  - [ ] V3F `.fastrun` inside `.highcode`
  - [ ] V5F `_usbdma_start` 512-byte aligned
- [ ] Hardware smoke test:
  - [ ] Flash `build/Merge.bin`
  - [ ] V3F heartbeat on PC2
  - [ ] V5F stage telemetry advances to relay loop
  - [ ] Real mouse cursor moves through MITM
  - [ ] Injection commands work
  - [ ] Keyboard LED SET_REPORT applies
  - [ ] Display text is clean (if panel attached)
  - [ ] Temp readout is sane (if displayed)

---

## Risk register

| Risk | Mitigation |
|------|------------|
| `FORCE_FS` change regresses FS/HS enumeration | Only set it for `USB_SPEED_LOW`; if issues arise, move the write to `usb_host_port_reset()` after reset-done. |
| NAK deadline is too short for slow devices | `DEF_INTR_OUT_TIMEOUT_US` starts at 5 ms; raise if a specific device needs more. |
| EP0 alias change breaks enumeration | Verify descriptors and SET_REPORT with multiple HID devices. |
| Display one-row-per-call leaves stale rows for >3 s | Acceptable for a non-essential status screen; prioritize `ROW_STATE` later if needed. |
| V3F `.fastrun` collection increases ITCM usage | Verify `size build/v3f.elf`; ITCM is 64 KB. |

---

## Notes for implementers

- Keep V5F ISRs FP-free. The FPU is used only in the main loop.
- Do not introduce V5F→V3F SRAM writes in the relay hot path. Trap-handler writes are the only exception and are already minimized in this plan.
- Preserve the existing command protocols (Hurra/Ferrum) and their public APIs.
- When in doubt, prefer the minimal change that closes the issue. This is a hardening pass, not a refactor.
