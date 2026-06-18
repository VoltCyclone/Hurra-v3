---
name: spi1-isr-not-broken
description: SPI1 RXNE ISR DOES fire on V5F — the "NVIC routing failure" 0xCF diagnosis was wrong; real gate-4b bug is the master not clocking
metadata:
  type: project
---

The gate-4b "SPI1 ISR never fires on V5F (oracle 0xCF)" diagnosis from sessions 2–3 is **WRONG**. Proven on the bench 2026-06-18 with a dedicated isolation harness (`SELFTEST=slave-irq`, added to spi_link_selftest.c + Makefile): the proven selftest master clocks an IRQ-driven slave, slave LED blinks slow iff `spi_link_isr_entries != 0`.

Result: host board (master) fast = expected (IRQ slave doesn't echo), **device board (slave-irq) SLOW = the RXNE ISR fires and bytes flow**. The reading is self-validating (master-fast and slave-slow are each only explainable one way), so it holds regardless of wlink probe-index↔board mapping.

So the NVIC path was never broken: `SPI1_IRQHandler` is a strong `T` symbol in v5f.elf (overrides the `.weak` vendor stub), vector table (core/startup_v5f.S:60) points at it, NVIC seq is byte-identical to the working USBHS ISR (SetAllocateIRQ→EnableIRQ, IRQn 37>31), RXNEIE set correctly. All verified.

`0xCF` (isr_entries==0) in the full two_board run has a SECOND cause the diagnostic ladder couldn't tell apart: **the master (Board B / host) never clocked.** Already recorded as obs 6698 "Board B Stuck in RELAY with frozen heartbeat, relay counter 0x00". That is the real gate-4b bug.

REAL ROOT CAUSE (2026-06-18, via wlink halt+regs on host board UID f8): the host (synth master) **CRASHES** before its main loop — it is NOT a polite SPI stall. Halted CPU state: oracle stuck 0x58 (stamped at first line of two_board_host_run, pre-loop), PC=0x2010xxxx (OUTSIDE all code — code lives in RAM_CODE 0x200A0000–0x200C0000), **mcause=0x2 = ILLEGAL INSTRUCTION**, mtval=0x10100073, mtvec=0x20100003 (trap vector ALSO points into the bad region so traps never recover → looks frozen), s0=0x58. Confirmed PRE-EXISTING: a pristine build from committed code (git stash) crashes identically. So the crash is in send_descriptor_blob / its callees (desc_xfer_pack_chunk, spi_frame_pack, spi_link_master_exchange), reached right after the 0x58 stamp. NOT MISO mode, NOT NVIC, NOT the wedge — those were wrong leads. The bounded-wait/recovery work (uncommitted, spi_link.c) is harmless+good defensive hygiene but does NOT address this crash. Next: bisect send_descriptor_blob with dbg_stage markers (or addr2line the faulting flow) to find the illegal-instruction site — likely a bad function pointer / stack overrun / misaligned access in the descriptor-chunk path.

To read a crashed board: `wlink -d <i> halt; wlink -d <i> regs` (mcause/mtval/dpc), then `wlink -d <i> resume`. Device board has SWJ disabled (NAK 0x55 on attach) = it's running; host (synth) keeps SWD alive = debuggable. NOTE: halting mid-call distorts oracle reads — sample the oracle WITHOUT halting (repeated `wlink dump 0x2017F000 4`) to see true progress.

CONFIRMED ROOT CAUSE (2026-06-18, single-variable test): the **IPC_CH0 doorbell ISR corrupts the V5F foreground context**. main_v5f.c:217-218 does IPC_ITConfig(IPC_CH0,Bit0,ENABLE)+NVIC_EnableIRQ(IPC_CH0_IRQn). When it fires during send_descriptor_blob it smashes the return address/stack → wild jump into unmapped 0x2010xxxx (RAM ends 0x20100000; the constant mtval=0x10100073 is the open-bus value of that region) → core runs garbage forever (TIM9 still advances, oracle frozen). Deterministic crash at marker 0xA4 (entering first send_descriptor_blob). PROOF: building with the IPC IRQ masked (`EXTRADEF=-DTB_NO_IPC_IRQ`, which does IPC_ITConfig(IPC_CH0,Bit0,DISABLE) at top of two_board_host_run) makes the crash VANISH — host then cycles 0xB5↔0x5A (full blob sent + clean main loop). This is why the selftest master worked: it diverts at main_v5f.c:205 BEFORE icc_init_v5f + IPC IRQ enable.

ACTUAL MECHANISM = IPC_CH0 INTERRUPT STORM (bench-proven 2026-06-18). Added icc_ipc_isr_entries counter to IPC_CH0_Handler: it climbs ~1.5 MILLION entries / 0.5s (~3 MHz). The ISR fires continuously → foreground starved → pinned at 0xA4 (core alive via TIM9, but never progresses). NOT FP, NOT nesting:
- FP ruled out: disasm of IPC_CH0_Handler shows GCC interrupt("machine") DOES save/restore fcsr + all f-regs correctly.
- Nesting ruled out: built with INTSYSCR_NO_NEST (intsyscr=0x00, nesting off) — crash STILL happened. (That gate is now in startup_v5f.S behind #ifdef INTSYSCR_NO_NEST; default unchanged at 0x0E.)
- Storm confirmed cause: building with the IPC IT masked (-DTB_NO_IPC_IRQ → IPC_ITConfig(IPC_CH0,Bit0,DISABLE) at top of two_board_host_run) makes host run cleanly (oracle B5↔5A).

Live IPC regs (0xE000D000) during storm: CTLR=0xF1 (TxCID1|RxCID0|TxIER|RxIER|AutoEN), ISR=0x01 (Bit0 STUCK asserted), ENA=0x01. The handler does IPC->CLR=1<<0 but ISR.Bit0 won't deassert.

ROOT CAUSE: the trivial IPC_CH0_Handler (icc.c) only clears the flag bit; with AutoEN=ENABLE the interrupt re-asserts from the unread MSG-mailbox condition. The RELAY never storms because its foreground constantly calls usb_merge_drain_icc → icc_recv_from_v3f (reads MSG[0..2], writes ack MSG[3]=seq) which satisfies the condition. The two_board HOST foreground gets stuck ~80ms in send_descriptor_blob and STOPS draining → flag never clears → storm → starvation (looks like a crash). EVT example (EXAM/CPU/IPC) convention: "V3F uses CH0 bit0 RX, V5F uses CH0 bit1 TX" — the two cores use DIFFERENT bits and the example ISR DISABLEs the IT bit inside the handler. Our code uses Bit0 for V3F-set AND V5F-clear (icc.c:190 sets Bit0, icc.c:223 clears Bit0).

FIXED + BENCH-VERIFIED (2026-06-18): defense-in-depth, both layers.
1. icc.c IPC_CH0_Handler now does IPC_ClearFlagStatus(Bit0) THEN IPC_ITConfig(Bit0,DISABLE) — self-disables so it can't storm under AutoEN. New icc_ipc_rearm_v5f() (ENABLE Bit0) is called by the foreground after draining.
2. usb_merge.c usb_merge_drain_icc() calls icc_ipc_rearm_v5f() after the drain loop (re-arms every pass).
3. two_board.c send_descriptor_blob() calls usb_merge_drain_icc() each chunk so the ~80ms send keeps acking.
RESULT: host oracle 0x5A stable (was frozen 0xA4), ISR entries ~650/s (was ~3,000,000/s), wedges=0; device board runs device path (SWJ-disabled 0x55 NAK on attach = enumerated + in relay). Host unit tests all pass.
Reverted dead ends: startup_v5f.S intsyscr stays 0x0E (nesting gate removed, didn't help). Kept: bounded master-wait+recovery in spi_link.c (0x5A/0xEA telemetry), SELFTEST=slave-irq harness in Makefile, icc_ipc_isr_entries counter (storm detector). Toolchain gotcha: NVIC_DisableIRQ uses `fence.i` (needs zifencei, absent from V5F march) — don't call it; mask at the peripheral with IPC_ITConfig DISABLE.

SECONDARY (minor, real): master SPI exchange occasionally times out (wedge) — caught+recovered by the new bounded-wait in spi_link.c (oracle shows occasional 0xEA, wedges=1, keeps running). Not the blocker.

Polled path is also proven good (full `SELFTEST=master`+`slave` pair both blink slow). Toolchain: build with `make TOOLCHAIN=riscv64-unknown-elf` (Makefile default `riscv-none-elf` isn't installed). Flash per board: `wlink -d <0|1> --chip CH32H41X flash -e --address 0x08000000 build/Merge.bin`. See [[two-board-hs-HANDOFF]].
