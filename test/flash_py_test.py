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


if __name__ == "__main__":
    unittest.main()
