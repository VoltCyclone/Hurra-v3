# test/humanization_analyze_test.py
import importlib.util, os, unittest

_spec = importlib.util.spec_from_file_location(
    "humanization_analyze",
    os.path.join(os.path.dirname(__file__), "..", "tools", "humanization_analyze.py"))
humanization_analyze = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(humanization_analyze)
ab_compare = humanization_analyze.ab_compare
passes = humanization_analyze.passes


def synth_human(n=2000):
    # smooth drift + 10 Hz tremor + signal-dependent noise (stand-in for a real trace)
    import math, random
    random.seed(1)
    out = []
    for i in range(n):
        base = 6.0
        tremor = 1.5 * math.sin(2*math.pi*10*i/1000.0)
        noise = random.gauss(0, 0.4*base/6)
        out.append((base, tremor + noise))
    return out


class ABTest(unittest.TestCase):
    def test_identical_streams_pass(self):
        h = synth_human()
        r = ab_compare(h, h)
        self.assertTrue(passes(r))

    def test_flat_uniform_stream_fails(self):
        h = synth_human()
        flat = [(6.0, 0.0)] * len(h)     # no tremor, no noise: a raw aimbot
        r = ab_compare(flat, h)
        self.assertFalse(passes(r))
        self.assertGreater(r["tremor_band_ratio_dev"], 0.0)


if __name__ == "__main__":
    unittest.main()
