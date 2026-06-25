import os, sys, tempfile, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from raw_input_classifier import train, trace, model

_HAVE = getattr(model, "_HAVE_TORCH", False)


def _write(path, label, source, fn, n=1200):
    reps = [fn(i) for i in range(n)]
    trace.write(path, trace.make_header(label, source), reps)


@unittest.skipUnless(_HAVE, "torch not installed ([torch] extra)")
class TrainTest(unittest.TestCase):
    def test_insufficient_data_gate(self):
        d = tempfile.mkdtemp()
        p = os.path.join(d, "h0.jsonl")
        _write(p, "human", "passthrough", lambda i: trace.Report(i*1000, 1, 0, 0, 0, 0), n=300)
        v = train.train_eval([p], "time", n=256, stride=128, min_windows=500, epochs=1)
        self.assertTrue(v["insufficient"])
        self.assertIn("INSUFFICIENT", train.format_verdict(v))

    def test_separable_corpus_learns(self):
        import math, random
        d = tempfile.mkdtemp()
        paths = []
        # human: tremor + noise;  synthetic: dead-flat uniform steps (a raw aimbot)
        for k in range(3):
            ph = os.path.join(d, f"h{k}.jsonl")
            rnd = random.Random(k)
            _write(ph, "human", "passthrough",
                   lambda i, rnd=rnd: trace.Report(i*1000, int(6 + 2*math.sin(i/5) + rnd.gauss(0,1)),
                                                   int(rnd.gauss(0, 1)), 0, 0, 0), n=1500)
            paths.append(ph)
            ps = os.path.join(d, f"s{k}.jsonl")
            _write(ps, "synthetic", "aimbot",
                   lambda i: trace.Report(i*1000, 6, 0, 0, 0, 0), n=1500)
            paths.append(ps)
        v = train.train_eval(paths, "time", n=256, stride=128, epochs=8,
                             min_windows=10, seed=0)
        self.assertFalse(v["insufficient"])
        self.assertGreater(v["auc"], 0.9)              # trivially separable -> learns
        self.assertIn("dt_us", v["ablation"])

    def test_label_mapping(self):
        d = tempfile.mkdtemp()
        p = os.path.join(d, "s.jsonl")
        _write(p, "synthetic", "aimbot", lambda i: trace.Report(i*1000, 1, 0, 0, 0, 0))
        files = train.load_corpus([p])
        self.assertEqual(files[0]["label"], 1)


if __name__ == "__main__":
    unittest.main()
