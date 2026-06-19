---
name: two-board-resend-load-bearing
description: Why the 80ms periodic descriptor re-send on Board B cannot be removed or streamed — it is load-bearing for Board A enumeration
metadata:
  type: project
---

In the two-board USB MITM (`src/two_board.c`), Board B's relay loop re-sends the full
~7KB descriptor blob every 500ms via the BLOCKING `send_descriptor_blob()` (325 chunks
× 250µs ≈ 80ms). This periodic re-send is **load-bearing for Board A enumeration**, not
just reset-recovery as the comment implies.

Proven on hardware 2026-06-18 (two failed fixes, reverted):
- **Removing the re-send** (send blob once up front only) → Board A stuck at stage
  `0x56` forever, never enumerates on PC. The up-front blob alone does NOT land
  reliably; Board A's IRQ slave + SOF-scanner often isn't synced at that first instant
  and depends on the 500ms re-sends to eventually catch one clean pass.
- **Streaming the re-send one chunk per loop iteration** (to avoid the 80ms stall) →
  ALSO stuck at `0x56`. Board A's `desc_xfer_accept` requires a TIGHT, CONTIGUOUS
  250µs-paced burst; chunks spread variably across loop iterations break the slave's
  framing. See the pacing rationale at two_board.c:40-46.

**Discriminator for working vs broken: Board A's oracle STAGE.** `0x58 RELAY` =
enumerated/working; `0x56 DESC_OK` stuck = never got a clean blob pass.

**`hb=FROZEN` is a RED HERRING** — it shows in the WORKING state too. The heartbeat
telemetry pump isn't wired into the two-board hot path (cf. obs 6930), so hb is always
FROZEN here regardless of health. Do NOT read it as a wedge. See [[gate-4c-real-passthrough-works]].

**The original bug** (user report): cursor "stops then shoots forward" every ~500ms —
the 80ms blocking re-send starves the interrupt-IN poll loop; relative-motion deltas
accumulate then flush in a jump.

**FIXED 2026-06-18 (DRDY gate).** Board A asserts the DATA_READY GPIO (PA3, already
wired/jumpered + driven low by spi_link slave init from boot) when it reaches its
Phase-3 relay loop (enumerated). Board B gates its periodic re-send on
`spi_link_master_drdy()`: re-send only while DRDY is LOW. Once A enumerates → DRDY high
→ B stops the 80ms blob blasts → smooth motion. Self-heals: if A resets, PA3 falls, B
resumes re-sending. ~20-line change in src/two_board.c (B: line ~227 gate; A: line ~358
`spi_link_slave_set_drdy(1)`). Verified on hardware: A=0x58, B not re-sending, cursor
smooth. DRDY signal chain proven (diag build showed B reads 0xD0 low pre-enum → 0xD1
high post-enum).

**CRITICAL TEST-PROCEDURE LESSON (cost 3 failed flashes):** a Board A stuck at 0x56 does
NOT resync to an already-running Board B's re-send stream — it needs a CLEAN BOOT while B
is up and streaming. All 3 "fix broke enumeration" failures were actually a STALE Board A
(booted before B, or left stuck across B reflashes), not the code. Correct bench
sequence: flash both → mouse into B (B boots/captures/streams) → THEN fresh-boot A into PC
(unplug/replug). Don't judge a two-board change against a stale A. See
[[gate-4c-real-passthrough-works]] (documents the same clean-boot-both ordering).
