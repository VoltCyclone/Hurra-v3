#!/usr/bin/env python3
"""Unified, serial-addressed flasher for the two-board Hurra-v3 rig.

Flashes the host (Board B) and/or device (Board A) by WCH-LinkE probe serial,
retrying transient failures and emitting human or --json output. See --help.
"""
import argparse, collections, json, re, subprocess, sys, time

CHIP = "CH32H41X"
FLASH_ADDR = "0x08000000"
ROLE_IMAGE = {"host": "build/BoardB.bin", "device": "build/BoardA.bin"}
ROLE_BOARD = {"host": "host", "device": "device"}
EXIT_OK, EXIT_USAGE, EXIT_PROBE, EXIT_BUILD, EXIT_FLASH, EXIT_NO_TOOL = 0, 2, 3, 4, 5, 6

_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(s):
    """Remove ANSI SGR color codes from a string."""
    return _ANSI_RE.sub("", s)


Probe = collections.namedtuple("Probe", "serial index id")

_INDEX_RE = re.compile(r"^\s*(\d+)\s*:")
_SERIAL_RE = re.compile(r"serial[=:\s]+(\S+)", re.IGNORECASE)
_DEVID_RE = re.compile(r"Probing device\s+(\S+)")


def parse_probe_list(plain, verbose):
    """Parse `wlink list` (and `-v` fallback) into a list of Probe.

    plain:   stdout/stderr of `wlink list`
    verbose: stdout/stderr of `wlink -v list` (device ids), used only when a
             probe has no USB serial.
    """
    plain = strip_ansi(plain)
    verbose = strip_ansi(verbose)
    dev_ids = _DEVID_RE.findall(verbose)
    probes = []
    n = 0
    for line in plain.splitlines():
        m = _INDEX_RE.match(line)
        if not m:
            continue
        index = int(m.group(1))
        sm = _SERIAL_RE.search(line)
        if sm:
            serial = sm.group(1)
            dev_id = serial
        else:
            dev_id = dev_ids[n] if n < len(dev_ids) else str(index)
            serial = dev_id
        probes.append(Probe(serial=serial, index=index, id=dev_id))
        n += 1
    return probes


class ResolveError(Exception):
    """Raised when a requested serial cannot be uniquely resolved to a probe."""


def resolve_serial(probes, requested, allow_any=False):
    """Resolve a requested serial (or prefix) to exactly one live Probe."""
    if requested is None:
        if allow_any and len(probes) == 1:
            return probes[0]
        if not probes:
            raise ResolveError("no probes connected")
        raise ResolveError(
            "no serial given; pass --host-serial/--device-serial "
            "(or --allow-any with exactly one probe connected)")
    exact = [p for p in probes if p.serial == requested]
    if len(exact) == 1:
        return exact[0]
    pref = [p for p in probes if p.serial.startswith(requested)]
    if len(pref) == 1:
        return pref[0]
    if len(pref) > 1:
        cands = ", ".join(p.serial for p in pref)
        raise ResolveError("serial %r is ambiguous; matches: %s" % (requested, cands))
    avail = ", ".join(p.serial for p in probes) or "(none)"
    raise ResolveError("serial %r not found; connected: %s" % (requested, avail))


class RunResult:
    """Outcome of a single subprocess call through the runner seam."""
    def __init__(self, returncode, output, timed_out=False):
        self.returncode = returncode
        self.output = output
        self.timed_out = timed_out


_TRANSIENT_MARKERS = ("0x55", "protocol error", "resource busy",
                      "device busy", "timeout", "timed out")


def is_transient(result):
    """True if a failure looks retryable (NAK after power-off, busy, timeout)."""
    if result.returncode == 0:
        return False
    if result.timed_out:
        return True
    low = result.output.lower()
    return any(m in low for m in _TRANSIENT_MARKERS)
