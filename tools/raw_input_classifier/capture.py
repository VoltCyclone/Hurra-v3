"""Capture a live (or fake) input source into a labeled .jsonl trace.

Supports a serial/VCOM device line source and an in-memory iterator source
(used by tests). The on-device 1 MHz-timestamped exfil frame uses the same
writer and schema; only the source iterator differs.
"""
from . import trace


def _as_report(item):
    if isinstance(item, trace.Report):
        return item
    t_us, dx, dy, bl, br, bm = item
    return trace.Report(int(t_us), int(dx), int(dy), int(bl), int(br), int(bm))


def capture_from_iter(source_iter, label, source, out_path,
                      rate_hz=1000, note="", limit=None):
    """Drain source_iter into a schema-valid trace. Returns the report count."""
    header = trace.make_header(label, source, rate_hz=rate_hz, note=note)  # validates label
    reports = []
    for item in source_iter:
        reports.append(_as_report(item))
        if limit is not None and len(reports) >= limit:
            break
    trace.write(out_path, header, reports)
    return len(reports)


def serial_source(port, baud=115200):
    """Yield (t_us,dx,dy,bl,br,bm) tuples from device lines 't_us dx dy bl br bm'.
    pyserial is imported lazily so this module loads without it."""
    import serial  # noqa: lazy import
    ser = serial.Serial(port, baud, timeout=1.0)
    try:
        for raw in ser:
            parts = raw.decode(errors="replace").split() if isinstance(raw, bytes) else raw.split()
            if len(parts) < 6:
                continue
            try:
                yield tuple(int(p) for p in parts[:6])
            except ValueError:
                continue
    finally:
        ser.close()
