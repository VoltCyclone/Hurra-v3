#pragma once
// Umbrella header: vendored WCH device + StdPeriph + core (PFIC/HSEM/IPC).
#include "ch32h417.h"          // device: IRQn enum, bases, register structs
#include "ch32h417_conf.h"     // StdPeriph: rcc/gpio/usart/dma/tim/usb/hsem/ipc
#include "core_riscv.h"        // PFIC, HSEM, IPC, SysTick, NVIC_WakeUp_V5F

// ── Interrupt handler attribute (TOOLCHAIN-CRITICAL) ────────────────────────
// RISC-V ISRs MUST end in `mret` (restore mepc/mstatus), NOT a plain `ret`.
// The vendor examples tag handlers `interrupt("WCH-Interrupt-fast")`, which is a
// WCH MounRiver-GCC-only extension (adds HPE hardware register stacking). On any
// STANDARD GCC (xPack riscv-none-elf, Homebrew riscv64-unknown-elf) that string
// is UNRECOGNISED: GCC warns `-Wattributes` and SILENTLY compiles the handler as
// an ordinary function that returns with `ret`. The handler then "returns" into a
// garbage `ra`, executes non-code, and traps illegal-instruction (mcause=2) — the
// core wedges in the HardFault spin-stub with interrupts masked. Observed
// 2026-06-12: the TIM3 timebase IRQ fired ~1 ms after boot and hung V3F, so the
// TIM2 LED heartbeat never ran ("LED lit but not blinking").
//
// Fix: use the STANDARD `interrupt("machine")` attribute, which every GCC
// understands and which emits a correct `mret` epilogue. (The only thing lost vs.
// WCH-Interrupt-fast is the optional HPE prologue optimisation — correctness wins.)
// If you ever build with WCH's MounRiver GCC and want HPE, define
// USE_WCH_FAST_IRQ=1 to opt back into the vendor string.
#if defined(USE_WCH_FAST_IRQ)
  #define WCH_IRQ  __attribute__((interrupt("WCH-Interrupt-fast")))
#else
  #define WCH_IRQ  __attribute__((interrupt("machine")))
#endif
