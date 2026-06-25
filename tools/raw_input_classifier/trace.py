"""Trace I/O and the .jsonl schema contract.

One file = one capture session = one label. Line 1 is a JSON header; every
subsequent line is one HID report. Pure stdlib — no numpy, no torch, no device
knowledge. The schema is append-only versioned: new channels bump SCHEMA_VERSION
and readers branch on it, so captured corpora are never silently invalidated.
"""
import json
from typing import Iterable, NamedTuple

SCHEMA_VERSION = 1
VALID_LABELS = ("human", "synthetic")
_REQUIRED_HEADER = ("v", "label", "source")


class Report(NamedTuple):
    t_us: int          # microsecond timestamp (host clock now; device 1 MHz later)
    dx: int
    dy: int
    bl: int            # left button 0/1
    br: int            # right button 0/1
    bm: int            # middle button 0/1


def make_header(label, source, rate_hz=1000, device="hurra-v3", note=""):
    h = {"v": SCHEMA_VERSION, "label": label, "source": source,
         "rate_hz": rate_hz, "device": device, "note": note}
    validate(h)
    return h


def validate(header):
    for k in _REQUIRED_HEADER:
        if k not in header:
            raise ValueError(f"header missing required key: {k!r}")
    if header["v"] != SCHEMA_VERSION:
        raise ValueError(f"unsupported schema version {header['v']} "
                         f"(expected {SCHEMA_VERSION})")
    if header["label"] not in VALID_LABELS:
        raise ValueError(f"invalid label {header['label']!r} "
                         f"(expected one of {VALID_LABELS})")


def write(path, header, reports: Iterable[Report]):
    validate(header)
    with open(path, "w") as f:
        f.write(json.dumps(header) + "\n")
        for r in reports:
            f.write(json.dumps({"t_us": r.t_us, "dx": r.dx, "dy": r.dy,
                                "bl": r.bl, "br": r.br, "bm": r.bm}) + "\n")


def read(path):
    """Return (header, reports, n_skipped). Malformed report lines are skipped
    and counted; an absent/invalid header raises ValueError."""
    header = None
    reports = []
    skipped = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                if header is None:
                    raise ValueError(f"{path}: first line is not valid JSON header")
                skipped += 1
                continue
            if header is None:
                validate(obj)
                header = obj
                continue
            try:
                reports.append(Report(int(obj["t_us"]), int(obj["dx"]), int(obj["dy"]),
                                      int(obj.get("bl", 0)), int(obj.get("br", 0)),
                                      int(obj.get("bm", 0))))
            except (KeyError, ValueError, TypeError):
                skipped += 1
    if header is None:
        raise ValueError(f"{path}: no valid header line")
    return header, reports, skipped
