import os, sys, tempfile, unittest, io, contextlib
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from raw_input_classifier import capture, trace


class CaptureTest(unittest.TestCase):
    def _tmp(self):
        fd, p = tempfile.mkstemp(suffix=".jsonl"); os.close(fd); return p

    def test_capture_from_iter_writes_valid_trace(self):
        src = [(i * 1000, i % 4, -(i % 2), 1 if i == 3 else 0, 0, 0) for i in range(20)]
        p = self._tmp()
        n = capture.capture_from_iter(iter(src), label="human",
                                      source="passthrough", out_path=p, note="fake")
        self.assertEqual(n, 20)
        hdr, reps, skipped = trace.read(p)
        self.assertEqual(hdr["label"], "human")
        self.assertEqual(hdr["source"], "passthrough")
        self.assertEqual(len(reps), 20)
        self.assertEqual(reps[3].bl, 1)

    def test_limit_caps_report_count(self):
        src = ((i, 1, 1, 0, 0, 0) for i in range(10_000))
        p = self._tmp()
        n = capture.capture_from_iter(src, "synthetic", "aimbot", p, limit=50)
        self.assertEqual(n, 50)

    def test_invalid_label_rejected(self):
        with self.assertRaises(ValueError):
            capture.capture_from_iter(iter([(0, 0, 0, 0, 0, 0)]),
                                      "robot", "x", self._tmp())


class CliTest(unittest.TestCase):
    def test_capture_subcommand_writes_trace(self):
        from raw_input_classifier import __main__ as cli
        import tempfile, os
        # a tiny file source: reuse fake-iter by pointing --from-file at a raw trace
        src = os.path.join(tempfile.mkdtemp(), "src.txt")
        with open(src, "w") as f:
            for i in range(30):
                f.write(f"{i*1000} {i%4} {-(i%2)} 0 0 0\n")
        out = os.path.join(tempfile.mkdtemp(), "cap.jsonl")
        with contextlib.redirect_stdout(io.StringIO()):
            rc = cli.main(["capture", "--from-file", src, "--label", "human",
                           "--source", "passthrough", "--out", out])
        self.assertEqual(rc, 0)
        hdr, reps, _ = trace.read(out)
        self.assertEqual(len(reps), 30)

    def test_eval_without_torch_errs_cleanly(self):
        from raw_input_classifier import __main__ as cli
        from raw_input_classifier import model
        if model._HAVE_TORCH:
            self.skipTest("torch present; clean-error path is for torch-absent env")
        with contextlib.redirect_stdout(io.StringIO()):
            rc = cli.main(["eval", "--ckpt", "x.pt", "--data", "none.jsonl"])
        self.assertEqual(rc, 2)        # clean non-crash exit


if __name__ == "__main__":
    unittest.main()
