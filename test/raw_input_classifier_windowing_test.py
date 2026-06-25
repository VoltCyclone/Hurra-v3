import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from raw_input_classifier import windowing
from raw_input_classifier.trace import Report


class WindowingTest(unittest.TestCase):
    def test_window_count_and_stride(self):
        reps = [Report(i, i, 0, 0, 0, 0) for i in range(1000)]
        ws = windowing.windows(reps, n=256, stride=128)
        # floor((1000-256)/128)+1 = floor(744/128)+1 = 5+1 = 6
        self.assertEqual(len(ws), 6)
        self.assertTrue(all(len(w) == 256 for w in ws))
        self.assertEqual(ws[1][0], reps[128])     # second window starts at stride

    def test_too_short_yields_no_windows(self):
        reps = [Report(i, i, 0, 0, 0, 0) for i in range(100)]
        self.assertEqual(windowing.windows(reps, n=256, stride=128), [])

    def test_group_split_disjoint_and_covers(self):
        groups = [("human", f"h{i}.jsonl") for i in range(6)] + \
                 [("synthetic", f"s{i}.jsonl") for i in range(6)]
        train, val = windowing.group_split(groups, val_frac=0.25, seed=0)
        self.assertEqual(train & val, set())            # disjoint
        self.assertEqual(train | val, set(groups))      # covers all
        self.assertGreaterEqual(len(val), 1)
        self.assertGreaterEqual(len(train), 1)

    def test_leakage_guard_raises(self):
        with self.assertRaises(AssertionError):
            windowing.assert_no_leakage({("human", "a")}, {("human", "a")})

    def test_leakage_guard_passes_when_disjoint(self):
        windowing.assert_no_leakage({("human", "a")}, {("synthetic", "b")})  # no raise


if __name__ == "__main__":
    unittest.main()
