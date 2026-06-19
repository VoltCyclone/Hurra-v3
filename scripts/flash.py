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
