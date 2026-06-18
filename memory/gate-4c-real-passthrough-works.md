---
name: gate-4c-real-passthrough-works
description: Gate 4c PASSED — real composite USB device full two-stack MITM passthrough works end-to-end on the bench
metadata:
  type: project
---

Gate 4c PASSED on hardware 2026-06-18. Full two-board USB MITM with a REAL device:
real composite mouse → Board B USBHS host capture (PB8/9 Type-A) → SPI link → Board A
`desc_xfer` reassembly → USBFS device clone (PA11/12 USB-C) → PC. Cursor moves, all
mouse endpoints (composite) come through. No code changes were needed — the
real-capture path (`two_board.c` real branch, `BOARD=host` without HOST_SYNTH) worked
as written. The only fix this session was the IPC_CH0 interrupt storm (commit f6d720a,
see [[spi1-isr-not-broken]]).

Build: `make merge TOOLCHAIN=riscv64-unknown-elf BOARD=host` (Board B, real capture) and
`BOARD=device` (Board A clone). Flash: Board A = probe #1 / UID 7b (→ PC), Board B =
probe #0 / UID f8 (real device → its host port). Bench sequence: clean-boot BOTH (reflash),
plug device into Board B first (B reaches 0x56→0x58), then Board A into PC (A reaches
0x57→0x58), cursor moves.

KEY BENCH LESSONS:
- The live oracle is the V3F UART, NOT SWD (both boards disable SWJ post-divert → SWD
  NAKs 0x55). Read with pyserial @921600 with **DTR+RTS asserted** (plain cat/stty get
  nothing): Board B = /dev/cu.usbmodem66B08F06ECA72, Board A = /dev/cu.usbmodem696B8F06EF622.
  Only the `V5F=0x..` stage code is real; `vid/pid/rps/hb=FROZEN` are unpumped in
  two-board mode (only main_v5f.c relay pumps them) — ignore them.
- ALWAYS clean-boot (reflash) both boards before trusting an oracle reading. A stale
  0xC1 (bytes-but-no-SOF-lock) after lots of reflash churn was NOT a bug — a fresh boot
  cleared it. 0x57 with no PC attached is normal (device waiting for a host).
- Stage codes (src/icc.h): host 0x54 wait→0x55 connected→0x56 captured→0x58 relay;
  device 0x56 wait-blob→0xC0-CF reassembly ladder→0x57 dev-init→0x58 configured.

NOT YET TESTED: standalone keyboard, HS (480M) device clone on USBHSD, injection/humanize
over the two-board path. Known clone limits (out of scope, basic HID fine): interrupt EPs
>64B, alternate interface settings, bulk/isoch endpoints.
