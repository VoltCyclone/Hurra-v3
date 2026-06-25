#!/usr/bin/env python3
"""Merge V3F + V5F raw binaries into one flash image.
V3F at offset 0, V5F at `v5f_offset` (default 0x10000), 0xFF pad between."""
import sys

def main():
    v3f, v5f, off_s, out = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    off = int(off_s, 0)
    a = open(v3f, "rb").read()
    b = open(v5f, "rb").read()
    if len(a) > off:
        raise SystemExit(f"V3F image ({len(a)} B) overruns V5F offset {off:#x}")
    img = bytearray(b"\xff" * off)
    img[:len(a)] = a
    img += b
    open(out, "wb").write(img)
    print(f"merged: V3F {len(a)} B + V5F {len(b)} B -> {out} ({len(img)} B)")

if __name__ == "__main__":
    main()
