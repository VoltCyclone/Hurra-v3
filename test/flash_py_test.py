# test/flash_py_test.py
import importlib.util, os, sys, json, io, unittest

_spec = importlib.util.spec_from_file_location(
    "flash", os.path.join(os.path.dirname(__file__), "..", "scripts", "flash.py"))
flash = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(flash)


class TestStripAnsi(unittest.TestCase):
    def test_removes_color_codes(self):
        s = "\x1b[36m[DEBUG]\x1b[0m probe 0"
        self.assertEqual(flash.strip_ansi(s), "[DEBUG] probe 0")

    def test_plain_text_unchanged(self):
        self.assertEqual(flash.strip_ansi("plain"), "plain")


class TestConstants(unittest.TestCase):
    def test_role_mapping(self):
        self.assertEqual(flash.ROLE_IMAGE["host"], "build/BoardB.bin")
        self.assertEqual(flash.ROLE_IMAGE["device"], "build/BoardA.bin")
        self.assertEqual(flash.FLASH_ADDR, "0x08000000")
        self.assertEqual(flash.CHIP, "CH32H41X")


class TestParseProbeList(unittest.TestCase):
    def test_plain_with_serial(self):
        plain = "0: WCH-LinkE serial=ABC123\n1: WCH-LinkE serial=DEF456\n"
        probes = flash.parse_probe_list(plain, "")
        self.assertEqual([(p.index, p.serial) for p in probes],
                         [(0, "ABC123"), (1, "DEF456")])

    def test_ansi_codes_tolerated(self):
        plain = "\x1b[32m0:\x1b[0m WCH-LinkE serial=ABC123\n"
        probes = flash.parse_probe_list(plain, "")
        self.assertEqual(probes[0].serial, "ABC123")

    def test_fallback_to_device_id_when_no_serial(self):
        plain = "0: WCH-LinkE\n"
        verbose = "[DEBUG] Probing device 100000991\n"
        probes = flash.parse_probe_list(plain, verbose)
        self.assertEqual(probes[0].index, 0)
        self.assertEqual(probes[0].serial, "100000991")  # id used as identifier
        self.assertEqual(probes[0].id, "100000991")

    def test_empty_means_no_probes(self):
        self.assertEqual(flash.parse_probe_list("", ""), [])


class TestResolveSerial(unittest.TestCase):
    def setUp(self):
        self.probes = [flash.Probe("ABC123", 0, "ABC123"),
                       flash.Probe("ABD999", 1, "ABD999")]

    def test_exact_match(self):
        self.assertEqual(flash.resolve_serial(self.probes, "ABD999").index, 1)

    def test_unique_prefix(self):
        self.assertEqual(flash.resolve_serial(self.probes, "ABC").index, 0)

    def test_ambiguous_prefix_raises(self):
        with self.assertRaises(flash.ResolveError):
            flash.resolve_serial(self.probes, "AB")

    def test_not_found_raises(self):
        with self.assertRaises(flash.ResolveError):
            flash.resolve_serial(self.probes, "ZZZ")

    def test_allow_any_single_probe_no_serial(self):
        one = [flash.Probe("ABC123", 0, "ABC123")]
        self.assertEqual(flash.resolve_serial(one, None, allow_any=True).index, 0)

    def test_no_serial_without_allow_any_raises(self):
        with self.assertRaises(flash.ResolveError):
            flash.resolve_serial(self.probes, None)


class TestIsTransient(unittest.TestCase):
    def _r(self, rc=0, out="", timed_out=False):
        return flash.RunResult(returncode=rc, output=out, timed_out=timed_out)

    def test_success_not_transient(self):
        self.assertFalse(flash.is_transient(self._r(rc=0)))

    def test_timeout_is_transient(self):
        self.assertTrue(flash.is_transient(self._r(rc=1, timed_out=True)))

    def test_0x55_protocol_error_is_transient(self):
        self.assertTrue(flash.is_transient(self._r(rc=1, out="protocol error: 0x55")))

    def test_probe_busy_is_transient(self):
        self.assertTrue(flash.is_transient(self._r(rc=1, out="device or resource busy")))

    def test_generic_failure_not_transient(self):
        self.assertFalse(flash.is_transient(self._r(rc=1, out="chip id mismatch")))


class FakeRunner:
    """Records commands; returns queued RunResults (by cmd-substring match)."""
    def __init__(self, rules):
        # rules: list of (substring, RunResult). First match wins per call.
        self.rules = rules
        self.calls = []

    def __call__(self, cmd, timeout):
        self.calls.append(list(cmd))
        joined = " ".join(cmd)
        for sub, res in self.rules:
            if sub in joined:
                return res
        return flash.RunResult(0, "")


class TestDiscoverProbes(unittest.TestCase):
    def test_calls_wlink_list_and_parses(self):
        runner = FakeRunner([("wlink list",
                              flash.RunResult(0, "0: WCH-LinkE serial=ABC123\n"))])
        probes = flash.discover_probes(runner)
        self.assertEqual(probes[0].serial, "ABC123")
        self.assertIn(["wlink", "list"], runner.calls)


class TestFlashAttempt(unittest.TestCase):
    def test_erase_then_flash_uses_index(self):
        probe = flash.Probe("ABC123", 2, "ABC123")
        runner = FakeRunner([("erase", flash.RunResult(0, "")),
                             ("flash", flash.RunResult(0, "done"))])
        ok, err = flash.flash_attempt(runner, probe, "build/BoardB.bin", timeout=60)
        self.assertTrue(ok)
        self.assertIsNone(err)
        # both commands target -d 2
        self.assertTrue(all("-d" in c and "2" in c for c in runner.calls))
        # flash command carries the image + address
        flash_cmd = [c for c in runner.calls if "flash" in c][0]
        self.assertIn("build/BoardB.bin", flash_cmd)
        self.assertIn(flash.FLASH_ADDR, flash_cmd)

    def test_flash_failure_returns_error(self):
        probe = flash.Probe("ABC123", 0, "ABC123")
        runner = FakeRunner([("erase", flash.RunResult(0, "")),
                             ("flash", flash.RunResult(1, "protocol error: 0x55"))])
        ok, err = flash.flash_attempt(runner, probe, "build/BoardB.bin", timeout=60)
        self.assertFalse(ok)
        self.assertIn("0x55", err)


class TestFlashRole(unittest.TestCase):
    def _probes(self, *serials):
        return "".join("%d: WCH-LinkE serial=%s\n" % (i, s)
                       for i, s in enumerate(serials))

    def test_success_no_build(self):
        runner = FakeRunner([("wlink list", flash.RunResult(0, self._probes("ABC"))),
                             ("erase", flash.RunResult(0, "")),
                             ("flash", flash.RunResult(0, "ok"))])
        r = flash.flash_role(runner, "host", "ABC", do_build=False,
                             retries=2, timeout=60, allow_any=False)
        self.assertTrue(r["flashed"])
        self.assertEqual(r["attempts"], 1)
        self.assertEqual(r["image"], "build/BoardB.bin")
        self.assertFalse(r["built"])

    def test_retry_then_success(self):
        # flash fails transiently once, then succeeds.
        seq = {"n": 0, "list_calls": 0}
        def runner(cmd, timeout):
            j = " ".join(cmd)
            if "wlink list" in j:
                seq["list_calls"] += 1
                return flash.RunResult(0, self._probes("ABC"))
            if "erase" in j:
                return flash.RunResult(0, "")
            if "flash" in j:
                seq["n"] += 1
                if seq["n"] == 1:
                    return flash.RunResult(1, "protocol error: 0x55")
                return flash.RunResult(0, "ok")
            return flash.RunResult(0, "")
        r = flash.flash_role(runner, "host", "ABC", do_build=False,
                             retries=2, timeout=60, allow_any=False, backoff=0.0)
        self.assertTrue(r["flashed"])
        self.assertEqual(r["attempts"], 2)
        self.assertEqual(seq["list_calls"], 2)  # re-resolution per attempt

    def test_fatal_error_no_retry(self):
        seq = {"n": 0}
        def runner(cmd, timeout):
            j = " ".join(cmd)
            if "wlink list" in j:
                return flash.RunResult(0, self._probes("ABC"))
            if "erase" in j:
                return flash.RunResult(0, "")
            if "flash" in j:
                seq["n"] += 1
                return flash.RunResult(1, "chip id mismatch")
            return flash.RunResult(0, "")
        r = flash.flash_role(runner, "host", "ABC", do_build=False,
                             retries=2, timeout=60, allow_any=False, backoff=0.0)
        self.assertFalse(r["flashed"])
        self.assertEqual(r["attempts"], 1)       # fatal -> no retry
        self.assertEqual(seq["n"], 1)

    def test_build_failure_skips_flash(self):
        runner = FakeRunner([("make merge", flash.RunResult(1, "compile error"))])
        r = flash.flash_role(runner, "device", "ABC", do_build=True,
                             retries=2, timeout=60, allow_any=False)
        self.assertFalse(r["built"])
        self.assertFalse(r["flashed"])
        self.assertIn("compile error", r["error"])

    def test_serial_not_found(self):
        runner = FakeRunner([("wlink list", flash.RunResult(0, self._probes("XYZ")))])
        r = flash.flash_role(runner, "host", "ABC", do_build=False,
                             retries=2, timeout=60, allow_any=False)
        self.assertFalse(r["flashed"])
        self.assertIn("not found", r["error"])


class Args:
    def __init__(self, **kw):
        self.host_serial = kw.get("host_serial")
        self.device_serial = kw.get("device_serial")
        self.no_build = kw.get("no_build", True)
        self.retries = kw.get("retries", 2)
        self.timeout = kw.get("timeout", 60)
        self.fail_fast = kw.get("fail_fast", False)
        self.allow_any = kw.get("allow_any", False)


class TestRunFlash(unittest.TestCase):
    def _ok_runner(self, *serials):
        listing = "".join("%d: WCH-LinkE serial=%s\n" % (i, s)
                          for i, s in enumerate(serials))
        return FakeRunner([("wlink list", flash.RunResult(0, listing)),
                           ("erase", flash.RunResult(0, "")),
                           ("flash", flash.RunResult(0, "ok"))])

    def test_both_roles_success(self):
        runner = self._ok_runner("HOSTSER", "DEVSER")
        res = flash.run_flash(runner, Args(host_serial="HOSTSER",
                                           device_serial="DEVSER"))
        self.assertTrue(res["ok"])
        self.assertEqual(res["exit_code"], flash.EXIT_OK)
        self.assertTrue(res["roles"]["host"]["flashed"])
        self.assertTrue(res["roles"]["device"]["flashed"])

    def test_one_role_failure_sets_flash_exit(self):
        def runner(cmd, timeout):
            j = " ".join(cmd)
            if "wlink list" in j:
                return flash.RunResult(0, "0: WCH-LinkE serial=HOSTSER\n")
            if "erase" in j:
                return flash.RunResult(0, "")
            if "flash" in j:
                return flash.RunResult(1, "chip id mismatch")
            return flash.RunResult(0, "")
        res = flash.run_flash(runner, Args(host_serial="HOSTSER"))
        self.assertFalse(res["ok"])
        self.assertEqual(res["exit_code"], flash.EXIT_FLASH)

    def test_no_roles_requested_is_usage_error(self):
        runner = self._ok_runner("HOSTSER")
        res = flash.run_flash(runner, Args())
        self.assertEqual(res["exit_code"], flash.EXIT_USAGE)

    def test_build_failure_exit_code(self):
        def runner(cmd, timeout):
            if "make merge" in " ".join(cmd):
                return flash.RunResult(1, "compile error")
            return flash.RunResult(0, "0: WCH-LinkE serial=HOSTSER\n")
        res = flash.run_flash(runner, Args(host_serial="HOSTSER", no_build=False))
        self.assertEqual(res["exit_code"], flash.EXIT_BUILD)

    def test_fail_fast_skips_device(self):
        calls = {"device": 0}
        def runner(cmd, timeout):
            j = " ".join(cmd)
            if "BOARD=device" in j:
                calls["device"] += 1
            if "wlink list" in j:
                return flash.RunResult(0, "0: WCH-LinkE serial=HOSTSER\n")
            if "erase" in j:
                return flash.RunResult(0, "")
            if "flash" in j:
                return flash.RunResult(1, "chip id mismatch")  # host fatal-fails
            return flash.RunResult(0, "")
        res = flash.run_flash(runner, Args(host_serial="HOSTSER",
                                           device_serial="DEVSER",
                                           fail_fast=True))
        self.assertIsNone(res["roles"].get("device"))
        self.assertEqual(res["exit_code"], flash.EXIT_FLASH)
        self.assertEqual(calls["device"], 0)  # device build/flash must be skipped

    def test_wlink_missing_exit_code(self):
        def wlink_missing(cmd, timeout):
            return flash.RunResult(127, "wlink not found")
        res = flash.run_flash(wlink_missing, Args(host_serial="ABC"))
        self.assertEqual(res["exit_code"], flash.EXIT_NO_TOOL)
        self.assertFalse(res["ok"])


class TestRendering(unittest.TestCase):
    def _result(self):
        return {"ok": False, "tool": "wlink",
                "probes": [{"serial": "ABC", "index": 0, "id": "ABC"}],
                "roles": {
                    "host": {"serial": "ABC", "index": 0, "image": "build/BoardB.bin",
                             "built": False, "flashed": True, "attempts": 1,
                             "duration_s": 4.1, "error": None},
                    "device": {"serial": "DEF", "index": 1, "image": "build/BoardA.bin",
                               "built": False, "flashed": False, "attempts": 3,
                               "duration_s": 12.7, "error": "protocol error: 0x55"}},
                "exit_code": 5}

    def test_human_mentions_ok_and_fail(self):
        text = flash.render_human(self._result())
        self.assertIn("host", text)
        self.assertIn("OK", text)
        self.assertIn("device", text)
        self.assertIn("0x55", text)

    def test_list_shows_serial_and_index(self):
        probes = [flash.Probe("ABC123", 0, "ABC123")]
        text = flash.render_list(probes)
        self.assertIn("ABC123", text)
        self.assertIn("0   ", text)  # "%-4d %s" left-pads index to 4 chars

    def test_emit_json_to_stdout_only(self):
        out, err = io.StringIO(), io.StringIO()
        flash.emit(self._result(), as_json=True, stream_out=out, stream_err=err)
        parsed = json.loads(out.getvalue())          # stdout is pure JSON
        self.assertEqual(parsed["exit_code"], 5)
        self.assertEqual(err.getvalue(), "")

    def test_emit_human_to_stderr_only(self):
        out, err = io.StringIO(), io.StringIO()
        flash.emit(self._result(), as_json=False, stream_out=out, stream_err=err)
        self.assertEqual(out.getvalue(), "")
        self.assertIn("device", err.getvalue())


class TestMain(unittest.TestCase):
    def test_list_returns_zero_and_prints(self):
        runner = FakeRunner([("wlink list",
                              flash.RunResult(0, "0: WCH-LinkE serial=ABC123\n"))])
        out = io.StringIO()
        old = sys.stdout
        sys.stdout = out
        try:
            rc = flash.main(["--list"], runner=runner)
        finally:
            sys.stdout = old
        self.assertEqual(rc, flash.EXIT_OK)
        self.assertIn("ABC123", out.getvalue())

    def test_no_args_is_usage_error(self):
        runner = FakeRunner([("wlink list", flash.RunResult(0, ""))])
        old_err = sys.stderr
        sys.stderr = io.StringIO()
        try:
            rc = flash.main([], runner=runner)
        finally:
            sys.stderr = old_err
        self.assertEqual(rc, flash.EXIT_USAGE)

    def test_json_flag_emits_json(self):
        runner = FakeRunner([("wlink list", flash.RunResult(0, "0: WCH-LinkE serial=ABC\n")),
                             ("erase", flash.RunResult(0, "")),
                             ("flash", flash.RunResult(0, "ok"))])
        out = io.StringIO()
        old = sys.stdout
        sys.stdout = out
        try:
            rc = flash.main(["--host-serial", "ABC", "--no-build", "--json"],
                            runner=runner)
        finally:
            sys.stdout = old
        self.assertEqual(rc, flash.EXIT_OK)
        self.assertEqual(json.loads(out.getvalue())["exit_code"], 0)


if __name__ == "__main__":
    unittest.main()
