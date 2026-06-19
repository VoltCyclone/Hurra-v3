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
