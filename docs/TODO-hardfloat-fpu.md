# TODO: Enable the V5F hardware FPU (hard-float ABI)

**Status:** DONE — implemented & hardware-verified 2026-06-16 (commit a93ab66,
merged to master). V5F now builds `-march=rv32imafc_zicsr -mabi=ilp32f` with
`-fno-math-errno`; soft-float libgcc calls eliminated (376→0), `.text`
23012→18844, ITCM pinning intact, V3F unchanged. Bench: hb=ALIVE + cursor smooth.
**Filed:** 2026-06-16. Retained as the implementation record.

## Summary

The CH32H417 QingKe **V5F** core has a single-precision hardware FPU that is
**already enabled in silicon but unused by the build**. All floating-point math
is compiled to soft-float library calls. The float-heavy code (`src/humanize.c`)
is on the latency-critical relay hot path (pinned to ITCM `.fastrun`), so this is
the single biggest remaining CPU-feature win.

## Evidence

- **FPU is ON:** `core/startup_v5f.S:496` sets `mstatus = 0x6088` → FS=3 (Dirty),
  i.e. the FP unit is powered and ready. The startup comment even says "Enable
  floating point." The WCH SDK ships `__get/set_FCSR`, `FFLAGS`, `FRM` accessors
  (`vendor/wch/Core/core_riscv.c`) — CSRs that only exist on an FPU core.
- **But we compile soft-float:** `Makefile` `ARCH = -march=rv32imac_zicsr
  -mabi=ilp32` (no `f`).
- **Cost, measured from `build/v5f.elf`:** 376 emulated FP calls — 97 `__addsf3`,
  93 `__subsf3`, 62 `__mulsf3`, 46 `__divsf3`, 23 `__gesf2`, 28 `__lesf2`, 20
  `__floatsisf`, 7 `__fixsfsi` — plus a software `__ieee754_sqrtf`. All in
  `humanize.c` (`humanize_filter` / `drain_axis` / `humanize_record_arrival`).
- **Trial confirmed:** compiling `humanize.c` with `-march=rv32imafc_zicsr
  -mabi=ilp32f` builds and emits native `fadd.s` / `fmul.s` / `fcvt.s.w` /
  `fsqrt.s`. Mainline `riscv64-unknown-elf-gcc 14.2` accepts the arch string.

## Plan

1. Split `ARCH` per core in the `Makefile`:
   - **V5F:** `-march=rv32imafc_zicsr -mabi=ilp32f` (claim the FPU).
   - **V3F:** keep `-march=rv32imac_zicsr -mabi=ilp32` (no hot-path float; avoids
     paying FP register save/restore on its interrupts). Moving V3F too is
     optional and lower value.
2. **Full clean rebuild is mandatory.** `ilp32f` is an ABI change — every object
   in the V5F image (vendor peripheral libs, `libm`, `libgcc`) must be rebuilt
   with the matching `-mabi`. You cannot link `ilp32` and `ilp32f` objects.
   `make clean` first. The `-lm`/`libgcc` the linker pulls must be the
   `ilp32f` multilib — verify the toolchain ships it (`gcc -print-multi-lib`).
3. **Interrupt context:** with hard-float, ISRs that touch FP (or call code that
   does) must save/restore the FP registers. Audit that no ISR on V5F uses float
   without proper context save. The relay hot path runs in the main loop, not an
   ISR, which helps — but confirm. This is exactly the class of bug that has
   wedged this chip before, so **hardware-verify** with the `hb=ALIVE` oracle.
4. Re-measure: confirm the `__*sf3` libgcc calls are gone from `build/v5f.elf`
   and that `humanize` ITCM pinning still holds (check the `.fastrun` symbols
   land in `RAM_CODE`).

## Out of scope / rejected during the audit

- **WCH `xw` extra-compressed extension:** mainline GCC 14.2 rejects
  `-march=..._xw` ("unsupported non-standard extension"). MounRiver-toolchain
  only. Not usable here.
- **B/bitmanip (`zba_zbb_zbc_zbs`):** toolchain accepts it, but WCH's own KM
  reference compiles plain `rv32i`, so silicon support for this part is
  unconfirmed, and the USB relay is not bit-manipulation-bound. Low payoff —
  defer until profiling shows a hotspot.
- **`-flto`:** accepted by the toolchain, not enabled. Could shrink/speed both
  images but LTO can relocate `.fastrun` symbols out of ITCM — defer until the
  pinning is verified to survive.

## Already optimal (do not touch)

ITCM hot-code mapping (`.highcode`/`.fastrun` → `RAM_CODE @0x200A0000`), prefetch
(`csrw 0xBC0, 0x1237B3E0`), interrupt nesting, HW-stacking intentionally OFF
(`intsyscr 0x0E`, required because mainline GCC lacks WCH-Interrupt-fast),
compressed extension, `--gc-sections`, nano specs. Our `imac` baseline already
exceeds WCH's reference `rv32i`.
