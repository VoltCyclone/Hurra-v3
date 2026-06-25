import os, sys, json, tempfile, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from raw_input_classifier import trace


class TraceTest(unittest.TestCase):
    def _tmp(self):
        fd, p = tempfile.mkstemp(suffix=".jsonl"); os.close(fd); return p

    def test_roundtrip(self):
        hdr = trace.make_header("human", "passthrough", note="subj A")
        reps = [trace.Report(0, 0, 0, 0, 0, 0),
                trace.Report(1000, 3, -1, 0, 0, 0),
                trace.Report(2000, 4, -2, 1, 0, 0)]
        p = self._tmp()
        trace.write(p, hdr, reps)
        rhdr, rreps, skipped = trace.read(p)
        self.assertEqual(rhdr["label"], "human")
        self.assertEqual(rhdr["source"], "passthrough")
        self.assertEqual(rreps, reps)
        self.assertEqual(skipped, 0)

    def test_malformed_lines_skipped_and_counted(self):
        p = self._tmp()
        with open(p, "w") as f:
            f.write(json.dumps(trace.make_header("synthetic", "aimbot")) + "\n")
            f.write('{"t_us":0,"dx":1,"dy":1,"bl":0,"br":0,"bm":0}\n')
            f.write("this is not json\n")
            f.write('{"dx":5}\n')                       # missing t_us -> skipped
            f.write('{"t_us":1000,"dx":2,"dy":2,"bl":0,"br":0,"bm":0}\n')
        hdr, reps, skipped = trace.read(p)
        self.assertEqual(len(reps), 2)
        self.assertEqual(skipped, 2)

    def test_validate_rejects_bad_label(self):
        with self.assertRaises(ValueError):
            trace.validate({"v": 1, "label": "robot", "source": "x"})

    def test_validate_rejects_version_mismatch(self):
        with self.assertRaises(ValueError):
            trace.validate({"v": 99, "label": "human", "source": "x"})

    def test_read_missing_header_raises(self):
        p = self._tmp()
        with open(p, "w") as f:
            f.write('{"t_us":0,"dx":1,"dy":1,"bl":0,"br":0,"bm":0}\n')
        with self.assertRaises(ValueError):
            trace.read(p)


if __name__ == "__main__":
    unittest.main()
