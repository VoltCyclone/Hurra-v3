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


if __name__ == "__main__":
    unittest.main()
