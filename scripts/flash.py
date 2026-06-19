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
    return _ANSI_RE.sub("", s)


Probe = collections.namedtuple("Probe", "serial index id")

# A line describes a probe if it names the adapter. wlink's exact `list` line
# format is not pinned across versions (it has appeared as "0:", "Probe 0:",
# and "WCH-Link #0:"), so detection keys off the adapter name and index
# extraction tolerates all those shapes — falling back to enumeration order,
# which is exactly what `-d <index>` selects by anyway.
_PROBE_LINE_RE = re.compile(r"wch[\s-]?link|wchlink|linke", re.IGNORECASE)
# Index markers, tried in order: "#0", "probe/link/device 0", leading "0:".
_INDEX_RES = (
    re.compile(r"#\s*(\d+)"),
    re.compile(r"(?:probe|link|device)\s*#?\s*(\d+)", re.IGNORECASE),
    re.compile(r"^\s*(\d+)\s*[:.)]"),
)
# Serial tokens are alphanumeric; stop at punctuation/whitespace so a trailing
# comma or paren in the source line does not leak into the captured serial.
_SERIAL_RE = re.compile(r"(?:serial|sn)[=:\s]+([0-9A-Za-z]+)", re.IGNORECASE)
_DEVID_RE = re.compile(r"Probing device\s+(\S+)")


def _extract_index(line):
    for rx in _INDEX_RES:
        m = rx.search(line)
        if m:
            return int(m.group(1))
    return None


def parse_probe_list(plain, verbose):
    """Parse `wlink list` (and `-v` fallback) into a list of Probe.

    plain:   stdout/stderr of `wlink list`
    verbose: stdout/stderr of `wlink -v list` (device ids), used only when a
             probe has no USB serial.

    Tolerant of wlink line-format drift: a line is treated as a probe when it
    names the adapter (see _PROBE_LINE_RE). The index is taken from an explicit
    marker when present, else from the probe's position in the list (the order
    `-d <index>` uses). The serial comes from a `serial=`/`sn:` token, else the
    `-v` device id, else the index as a last resort.
    """
    plain = strip_ansi(plain)
    verbose = strip_ansi(verbose)
    dev_ids = _DEVID_RE.findall(verbose)
    probes = []
    n = 0
    for line in plain.splitlines():
        if not _PROBE_LINE_RE.search(line):
            continue
        index = _extract_index(line)
        if index is None:
            index = n
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


class WlinkMissingError(Exception):
    """Raised when wlink is not present in PATH (exit code 127)."""


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


_TRANSIENT_MARKERS = ("0x55", "resource busy",
                      "device busy", "timeout", "timed out")


def is_transient(result):
    """True if a failure looks retryable (NAK after power-off, busy, timeout)."""
    if result.returncode == 0:
        return False
    if result.timed_out:
        return True
    low = result.output.lower()
    return any(m in low for m in _TRANSIENT_MARKERS)


def subprocess_runner(cmd, timeout):
    """Real runner: run cmd, capture combined output, honor a timeout."""
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, timeout=timeout)
        return RunResult(proc.returncode, proc.stdout.decode("utf-8", "replace"))
    except subprocess.TimeoutExpired as e:
        out = e.output.decode("utf-8", "replace") if e.output else ""
        return RunResult(1, out + "\n[timed out]", timed_out=True)
    except FileNotFoundError:
        return RunResult(127, "wlink not found", timed_out=False)


def discover_probes(runner):
    """Enumerate connected probes via `wlink list` (+ `-v` for device ids)."""
    list_result = runner(["wlink", "list"], timeout=15)
    if list_result.returncode == 127:
        raise WlinkMissingError("wlink not found in PATH")
    plain = list_result.output
    verbose = ""
    if "serial" not in plain.lower():
        verbose = runner(["wlink", "-v", "list"], timeout=15).output
    return parse_probe_list(plain, verbose)


def flash_attempt(runner, probe, image, timeout):
    """One erase(power-off)+flash cycle against probe.index. Returns (ok, error)."""
    d = str(probe.index)
    erase = runner(["wlink", "-d", d, "--chip", CHIP,
                   "erase", "--method", "power-off"], timeout=timeout)
    if erase.returncode != 0 and "0x55" not in erase.output:
        # erase itself failed for a non-NAK reason; surface it
        return False, erase.output.strip() or "erase failed"
    res = runner(["wlink", "-d", d, "--chip", CHIP, "flash",
                 "-e", "--address", FLASH_ADDR, image], timeout=timeout)
    if res.returncode == 0:
        return True, None
    return False, res.output.strip() or ("timed out" if res.timed_out else "flash failed")


def build_image(runner, role, timeout):
    """Build the merged image for a role via `make merge BOARD=<role>`."""
    res = runner(["make", "merge", "BOARD=" + ROLE_BOARD[role]], timeout=timeout)
    if res.returncode == 0:
        return True, None
    return False, res.output.strip() or "build failed"


def flash_role(runner, role, requested, *, do_build, retries, timeout,
               allow_any, backoff=0.0, clock=time.monotonic):
    """Build (optional) + flash one role with retries. Returns a role dict."""
    image = ROLE_IMAGE[role]
    out = {"serial": requested, "index": None, "image": image,
           "built": False, "flashed": False, "attempts": 0,
           "duration_s": 0.0, "error": None}
    start = clock()

    if do_build:
        ok, err = build_image(runner, role, timeout)
        out["built"] = ok
        if not ok:
            out["error"] = err
            out["duration_s"] = round(clock() - start, 2)
            return out

    last_err = None
    for attempt in range(1, retries + 2):          # retries=2 -> up to 3 tries
        try:
            probes = discover_probes(runner)
            probe = resolve_serial(probes, requested, allow_any=allow_any)
        except ResolveError as e:
            out["error"] = str(e)
            out["duration_s"] = round(clock() - start, 2)
            return out                              # resolution is fatal, no retry
        out["index"] = probe.index
        out["serial"] = probe.serial
        out["attempts"] = attempt
        ok, err = flash_attempt(runner, probe, image, timeout)
        if ok:
            out["flashed"] = True
            out["error"] = None
            out["duration_s"] = round(clock() - start, 2)
            return out
        last_err = err
        if not is_transient(RunResult(1, err or "")) or attempt == retries + 1:
            break
        if backoff:
            time.sleep(backoff)
    out["error"] = last_err
    out["duration_s"] = round(clock() - start, 2)
    return out


def run_flash(runner, args):
    # Flash each requested role in turn; a per-role failure sets exit_code but
    # does not stop the others unless --fail-fast. Returns the JSON result dict.
    roles = []
    if args.host_serial is not None:
        roles.append(("host", args.host_serial))
    if args.device_serial is not None:
        roles.append(("device", args.device_serial))

    result = {"ok": False, "tool": "wlink", "probes": [],
              "roles": {}, "exit_code": EXIT_OK}

    if not roles:
        result["error"] = ("no role requested; pass --host-serial and/or "
                           "--device-serial (or --list to enumerate)")
        result["exit_code"] = EXIT_USAGE
        return result

    try:
        result["probes"] = [p._asdict() for p in discover_probes(runner)]
    except WlinkMissingError as e:
        result["exit_code"] = EXIT_NO_TOOL
        result["ok"] = False
        result["error"] = str(e)
        return result
    except Exception:
        result["probes"] = []

    exit_code = EXIT_OK
    try:
        for role, serial in roles:
            rr = flash_role(runner, role, serial, do_build=not args.no_build,
                            retries=args.retries, timeout=args.timeout,
                            allow_any=args.allow_any, backoff=1.0)
            result["roles"][role] = rr
            if not rr["flashed"]:
                # classify: build failure vs resolution vs flash
                if not rr["built"] and not args.no_build:
                    code = EXIT_BUILD
                elif rr["index"] is None:
                    code = EXIT_PROBE
                else:
                    code = EXIT_FLASH
                exit_code = code
                if args.fail_fast:
                    break
    except WlinkMissingError as e:
        result["exit_code"] = EXIT_NO_TOOL
        result["ok"] = False
        result["error"] = str(e)
        return result

    result["exit_code"] = exit_code
    result["ok"] = exit_code == EXIT_OK
    return result


def render_human(result):
    """Per-role summary suitable for stderr."""
    lines = []
    for role, r in result["roles"].items():
        if r["flashed"]:
            lines.append("%-6s OK   (idx %s, serial %s, %.1fs)" %
                        (role + ":", r["index"], r["serial"], r["duration_s"]))
        else:
            lines.append("%-6s FAIL (after %d attempt(s)): %s" %
                        (role + ":", r["attempts"], r["error"]))
    lines.append("result: %s (exit %d)" %
                 ("OK" if result["ok"] else "FAILED", result["exit_code"]))
    return "\n".join(lines)


def render_list(probes):
    """Table of connected probes for --list."""
    if not probes:
        return "no probes connected"
    rows = ["idx  serial / id"]
    for p in probes:
        rows.append("%-4d %s" % (p.index, p.serial))
    return "\n".join(rows)


def emit(result, as_json, stream_out, stream_err):
    """Write JSON to stdout (machine) or human text to stderr."""
    if as_json:
        stream_out.write(json.dumps(result) + "\n")
    else:
        stream_err.write(render_human(result) + "\n")


def _build_parser():
    p = argparse.ArgumentParser(
        prog="flash.py",
        description="Flash the host (Board B) and/or device (Board A) by WCH-LinkE "
                    "probe serial, with retries and JSON output.",
        epilog="Examples:\n"
               "  scripts/flash.py --list\n"
               "  scripts/flash.py --host-serial ABC123 --device-serial DEF456\n"
               "  scripts/flash.py --device-serial DEF456 --no-build\n"
               "  scripts/flash.py --host-serial ABC123 --json\n",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host-serial", help="probe serial for the host board (Board B)")
    p.add_argument("--device-serial", help="probe serial for the device board (Board A)")
    p.add_argument("--list", action="store_true", help="list connected probes and exit")
    p.add_argument("--no-build", action="store_true",
                   help="skip `make merge`; flash existing build/*.bin")
    p.add_argument("--json", action="store_true",
                   help="emit one JSON result object to stdout (logs to stderr)")
    p.add_argument("--retries", type=int, default=2,
                   help="transient-failure retries per role (default 2)")
    p.add_argument("--timeout", type=float, default=60,
                   help="per-wlink-call timeout, seconds (default 60)")
    p.add_argument("--fail-fast", action="store_true",
                   help="stop after the first role fails (two-board mode)")
    p.add_argument("--allow-any", action="store_true",
                   help="flash the only connected probe when no serial is given")
    return p


def main(argv=None, runner=None):
    args = _build_parser().parse_args(argv)
    if runner is None:
        runner = subprocess_runner

    if args.list:
        try:
            probes = discover_probes(runner)
        except WlinkMissingError as e:
            sys.stderr.write("error: %s\n" % e)
            return EXIT_NO_TOOL
        if args.json:
            sys.stdout.write(json.dumps([p._asdict() for p in probes]) + "\n")
        else:
            sys.stdout.write(render_list(probes) + "\n")
        return EXIT_OK

    result = run_flash(runner, args)
    emit(result, as_json=args.json, stream_out=sys.stdout, stream_err=sys.stderr)
    return result["exit_code"]


if __name__ == "__main__":
    sys.exit(main())
