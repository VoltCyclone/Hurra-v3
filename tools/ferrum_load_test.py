#!/usr/bin/env python3
"""Ferrum command-channel load test.

Hammers the Ferrum command surface to measure how much the link can carry and
whether it stays correct under load. Point it at the same endpoint the app uses
— the hurra-app VCOM (hurra-bridge PTY) — so it exercises the real
host → bridge → 4 M Hurra wire → firmware path. (Baud is a no-op on a PTY; the
real wire rate is the bridge's business.)

Phases (run in this order; each is opt-in except throughput):
    latency    (--ping)    km.version() round-trips: host→…→firmware→…→host RTT
    throughput (default)   blast km.move as fast as the link allows; report
                           commands/s, bytes/s, and drops (writes that timed out
                           under backpressure)
    integrity  (--verify)  open a km.catch_xy window, send a known (+1,+1)
                           pattern at a SUSTAINABLE rate (--verify-rate), then
                           read back the firmware's accumulated injected motion
                           and compare — catches commands silently lost or
                           mangled between here and the firmware

The throughput phase alternates +1/-1 moves so the cursor stays put; the
integrity phase uses small fixed (+1,+1) moves that never saturate the HID
delta field, so a clean run should report ~100% delivered. catch_xy is
time-windowed, so the integrity phase must NOT run flat out — saturating the
buffer makes commands arrive after the window closes and reads as false loss.
Overload behaviour is measured by the throughput phase's drop counter instead.

Usage:
    pip install pyserial
    tools/ferrum_load_test.py /dev/ttysNNN                 # 5 s throughput
    tools/ferrum_load_test.py /dev/ttysNNN --duration 10 --ping --verify
    tools/ferrum_load_test.py /dev/ttysNNN --rate 5000     # cap the send rate

NOTE: don't touch the real mouse during --verify; concurrent physical motion
can clip the merged report and skew the accounting.
"""

import argparse
import re
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial required:  pip install pyserial")


PAIR_RE = re.compile(rb"\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\)")


def handshake(ser):
    """Confirm the endpoint speaks Ferrum. No ser.flush() — it hangs on the PTY."""
    ser.reset_input_buffer()
    ser.write(b"km.version()\r\n")
    deadline = time.monotonic() + 0.5
    buf = b""
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if buf.endswith(b"\r\n"):
            break
    ver = buf.decode("ascii", "replace").rstrip("\r\n")
    if ver != "kmbox: Ferrum":
        sys.exit(f"handshake failed: got {ver!r}, expected 'kmbox: Ferrum'")
    return ver


def read_line(ser, timeout_s):
    """Read one CRLF-terminated line, or b'' on timeout."""
    deadline = time.monotonic() + timeout_s
    buf = b""
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if buf.endswith(b"\r\n"):
            return buf
    return b""


# ----- latency -----

def phase_latency(ser, count):
    print(f"\n[latency]  {count} km.version() round-trips")
    rtts = []
    timeouts = 0
    for _ in range(count):
        ser.reset_input_buffer()
        t0 = time.perf_counter()
        ser.write(b"km.version()\r\n")
        line = read_line(ser, 0.5)
        if line.rstrip(b"\r\n") == b"kmbox: Ferrum":
            rtts.append((time.perf_counter() - t0) * 1000.0)
        else:
            timeouts += 1
        time.sleep(0.002)  # don't pipeline; measure one RTT at a time
    if rtts:
        rtts.sort()
        pct = lambda p: rtts[min(len(rtts) - 1, int(p / 100.0 * len(rtts)))]
        print(f"  ok={len(rtts)}  timeouts={timeouts}")
        print(f"  min={rtts[0]:.2f}  median={statistics.median(rtts):.2f}  "
              f"p95={pct(95):.2f}  p99={pct(99):.2f}  max={rtts[-1]:.2f}  (ms)")
    else:
        print(f"  no replies ({timeouts} timeouts)")


# ----- throughput -----

def phase_throughput(ser, duration, rate):
    cap = f"{rate}/s" if rate else "unlimited"
    print(f"\n[throughput]  {duration:.1f}s, rate cap {cap}")
    cmds = (b"km.move(1, 0)\r\n", b"km.move(-1, 0)\r\n")
    period = (1.0 / rate) if rate else 0.0

    sent = bytes_ok = drops = 0
    start = time.perf_counter()
    end = start + duration
    next_report = start + 1.0
    next_t = start
    i = 0
    while True:
        now = time.perf_counter()
        if now >= end:
            break
        if period:
            if now < next_t:
                dt = next_t - now
                if dt > 0.0015:
                    time.sleep(dt - 0.0015)
                continue
            next_t += period
            if next_t < now:
                next_t = now + period

        line = cmds[i & 1]
        i += 1
        try:
            ser.write(line)
            sent += 1
            bytes_ok += len(line)
        except serial.SerialTimeoutException:
            drops += 1
        except serial.SerialException:
            break

        if now >= next_report:
            elapsed = now - start
            print(f"  {elapsed:4.1f}s  {sent/elapsed:8.0f} cmd/s  "
                  f"{bytes_ok/elapsed/1024:7.1f} KiB/s  drops={drops}")
            next_report += 1.0

    elapsed = max(time.perf_counter() - start, 1e-6)
    total = sent + drops
    print(f"  ---")
    print(f"  sent:    {sent} cmds  ({sent/elapsed:.0f}/s)")
    print(f"  bytes:   {bytes_ok/1024:.0f} KiB  ({bytes_ok/elapsed/1024:.1f} KiB/s, "
          f"{bytes_ok*8/elapsed/1e6:.2f} Mbit/s)")
    print(f"  drops:   {drops}  ({100.0*drops/total if total else 0.0:.2f}%)")


# ----- integrity -----

def phase_integrity(ser, rate):
    # catch_xy is a TIME-windowed measurement (firmware-side, clamped to 1000 ms).
    # It only correlates if commands flow through in real time within the window,
    # so we send at a sustainable rate — NOT flat out. Blasting into a full buffer
    # makes commands arrive after the deadline and reads as 100% loss even when
    # nothing was dropped. Send-side overload is what the throughput phase's drop
    # counter is for; this phase checks correctness at a rate the link can carry.
    window_ms = 1000
    blast_ms  = 600              # well inside the window, leaves arrival margin

    # Settle: drain whatever a preceding saturation phase left in flight so the
    # window lines up with our sends.
    time.sleep(0.25)
    for reset in (ser.reset_output_buffer, ser.reset_input_buffer):
        try:
            reset()
        except Exception:
            pass

    print(f"\n[integrity]  catch_xy({window_ms}ms), "
          f"{blast_ms}ms of km.move(1, 1) at {rate}/s")

    ser.write(f"km.catch_xy({window_ms})\r\n".encode("ascii"))
    time.sleep(0.03)  # let the window go active before we start blasting

    cmd = b"km.move(1, 1)\r\n"
    period = 1.0 / rate
    sent = drops = 0
    start = time.perf_counter()
    end = start + blast_ms / 1000.0
    next_t = start
    while True:
        now = time.perf_counter()
        if now >= end:
            break
        if now < next_t:
            dt = next_t - now
            if dt > 0.0015:
                time.sleep(dt - 0.0015)
            continue
        next_t += period
        if next_t < now:
            next_t = now + period
        try:
            ser.write(cmd)
            sent += 1
        except serial.SerialTimeoutException:
            drops += 1
        except serial.SerialException:
            break

    # The result line is emitted when the firmware window deadline expires.
    line = read_line(ser, 1.0)
    m = PAIR_RE.search(line)
    if not m:
        print(f"  no catch_xy result (got {line!r}) — is a real mouse attached "
              "so reports merge?")
        return
    cx, cy = int(m.group(1)), int(m.group(2))
    # Each command injects (1, 1); expected accumulator == commands delivered.
    print(f"  sent:      {sent} cmds  (write drops {drops})")
    print(f"  caught:    x={cx}  y={cy}  (firmware-delivered injected motion)")
    if sent:
        print(f"  delivered: {100.0*cx/sent:.1f}% x, {100.0*cy/sent:.1f}% y")
        lost = sent - cx
        if lost > 0:
            print(f"  LOST/CARRIED: {lost} cmds ({100.0*lost/sent:.1f}%) — "
                  "link loss, or motion still draining at window close")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="hurra-bridge VCOM/PTY, e.g. /dev/ttys003")
    ap.add_argument("--duration", type=float, default=5.0,
                    help="throughput phase duration in seconds (default 5)")
    ap.add_argument("--rate", type=int, default=0,
                    help="cap send rate in cmd/s (default 0 = unlimited / flat out)")
    ap.add_argument("--ping", type=int, nargs="?", const=100, default=0,
                    metavar="N", help="run latency phase with N pings (default 100)")
    ap.add_argument("--verify", action="store_true",
                    help="run the catch_xy integrity phase")
    ap.add_argument("--verify-rate", type=int, default=2000, metavar="HZ",
                    help="send rate for the integrity phase (default 2000); keep "
                         "it well under the measured throughput so commands flow "
                         "in real time within the catch window")
    ap.add_argument("--skip-throughput", action="store_true",
                    help="skip the throughput phase")
    ap.add_argument("--baud", type=int, default=115200,
                    help="ignored on a PTY; only used on a real serial port")
    args = ap.parse_args()

    if args.rate < 0:
        sys.exit("--rate must be >= 0")
    if args.verify_rate < 1:
        sys.exit("--verify-rate must be >= 1")

    # Short write_timeout so a full buffer counts as a drop instead of blocking
    # forever — that's what makes the drop metric meaningful under overload.
    ser = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=0.02)
    try:
        ver = handshake(ser)
        print(f"Connected: {ver}")
        if args.ping:
            phase_latency(ser, args.ping)
        if not args.skip_throughput:
            phase_throughput(ser, args.duration, args.rate)
        if args.verify:
            phase_integrity(ser, args.verify_rate)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
