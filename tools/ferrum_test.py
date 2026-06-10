#!/usr/bin/env python3
"""Ferrum text protocol test harness.

Run against the WCH CH343 USB-UART bridge wired to Teensy D16/D17 (LPUART3),
or against the hurra-bridge PTY symlink (~/.hurra-bridge.tty).

Examples:
    tools/ferrum_test.py /dev/cu.usbserial-XYZ version
    tools/ferrum_test.py /dev/cu.usbserial-XYZ move 50 0
    tools/ferrum_test.py /dev/cu.usbserial-XYZ click 0
    tools/ferrum_test.py /dev/cu.usbserial-XYZ smoke
"""
import argparse, sys, time

try:
    import serial
except ImportError:
    sys.exit("pyserial required:  pip install pyserial")


def send_line(ser, line, *, expect_reply=False, timeout=0.5):
    payload = (line + "\r\n").encode("ascii")
    # No ser.flush(): flush() calls tcdrain(), which blocks indefinitely on a
    # pseudo-terminal (the hurra-bridge PTY) when the reader hasn't drained yet.
    # write() already queues the bytes; the reply read below provides the sync.
    ser.write(payload)
    print(f"TX  {line}")
    if not expect_reply:
        return None
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if buf.endswith(b"\r\n"):
            break
    s = buf.decode("ascii", "replace").rstrip("\r\n")
    print(f"RX  {s!r}")
    return s


def cmd_version(ser):
    r = send_line(ser, "km.version()", expect_reply=True, timeout=0.5)
    assert r == "kmbox: Ferrum", f"got {r!r}, expected 'kmbox: Ferrum'"
    print("    OK")


def cmd_move(ser, dx, dy):
    send_line(ser, f"km.move({dx}, {dy})")


def cmd_click(ser, btn):
    send_line(ser, f"km.click({btn})")


def cmd_button(ser, name, state):
    send_line(ser, f"km.{name}({state})")


def cmd_wheel(ser, n):
    send_line(ser, f"km.wheel({n})")


def cmd_isdown(ser, key):
    return send_line(ser, f"km.isdown({key})", expect_reply=True)


def cmd_smoke(ser):
    print("--- smoke ---")
    cmd_version(ser)
    print("--- nudge mouse ---")
    cmd_move(ser, 5, 0);  time.sleep(0.05)
    cmd_move(ser, -5, 0); time.sleep(0.05)
    cmd_move(ser, 0, 5);  time.sleep(0.05)
    cmd_move(ser, 0, -5)
    print("--- left button down/up ---")
    cmd_button(ser, "left", 1); time.sleep(0.05)
    cmd_button(ser, "left", 0)
    print("--- left state read ---")
    r = send_line(ser, "km.left()", expect_reply=True)
    assert r in ("0", "1"), f"expected 0 or 1, got {r!r}"
    print("--- wheel ---")
    cmd_wheel(ser, 1)
    cmd_wheel(ser, -1)
    print("--- alias m(x,y) ---")
    send_line(ser, "m(1, 1)")
    print("--- done ---")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("version")
    sub.add_parser("smoke")
    p = sub.add_parser("move");  p.add_argument("dx", type=int); p.add_argument("dy", type=int)
    p = sub.add_parser("click"); p.add_argument("button", type=int, choices=range(5))
    p = sub.add_parser("wheel"); p.add_argument("delta", type=int)
    p = sub.add_parser("button")
    p.add_argument("which", choices=["left","right","middle","side1","side2"])
    p.add_argument("state", type=int, choices=[0,1])
    p = sub.add_parser("isdown"); p.add_argument("key", type=int)
    p = sub.add_parser("raw");    p.add_argument("line", help="raw line, e.g. 'km.move(10,10)'")

    args = ap.parse_args()
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    c = args.cmd
    if   c == "version": cmd_version(ser)
    elif c == "smoke":   cmd_smoke(ser)
    elif c == "move":    cmd_move(ser, args.dx, args.dy)
    elif c == "click":   cmd_click(ser, args.button)
    elif c == "wheel":   cmd_wheel(ser, args.delta)
    elif c == "button":  cmd_button(ser, args.which, args.state)
    elif c == "isdown":  cmd_isdown(ser, args.key)
    elif c == "raw":     send_line(ser, args.line, expect_reply=True)


if __name__ == "__main__":
    main()
