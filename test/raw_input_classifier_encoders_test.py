import os, sys, math, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from raw_input_classifier import encoders
from raw_input_classifier.trace import Report


def _ramp(n=64):
    # deterministic non-constant fixture with one click at i==10
    out = []
    for i in range(n):
        out.append(Report(t_us=i * 1000, dx=i % 5, dy=(i % 3) - 1,
                          bl=1 if i == 10 else 0, br=0, bm=0))
    return out


class TimeEncoderTest(unittest.TestCase):
    def test_raw_shape_and_channels(self):
        r = _ramp(64)
        ch = encoders.time_raw(r)
        self.assertEqual(len(ch), 6)
        self.assertTrue(all(len(c) == 64 for c in ch))
        self.assertEqual(encoders.time_shape(64), (6, 64))

    def test_dt_channel_equals_stamped_deltas(self):
        r = _ramp(64)
        dt = encoders.time_raw(r)[3]          # dt_us channel
        self.assertEqual(dt[0], 0.0)          # first report has no predecessor
        self.assertTrue(all(abs(x - 1000.0) < 1e-9 for x in dt[1:]))

    def test_speed_channel(self):
        r = [Report(0, 3, 4, 0, 0, 0), Report(1000, 0, 0, 0, 0, 0)]
        sp = encoders.time_raw(r)[2]
        self.assertAlmostEqual(sp[0], 5.0)    # hypot(3,4)
        self.assertAlmostEqual(sp[1], 0.0)

    def test_click_channel_marks_exact_report(self):
        r = _ramp(64)
        bl = encoders.time_raw(r)[4]
        self.assertEqual(bl[10], 1.0)
        self.assertEqual(sum(bl), 1.0)        # exactly one click

    def test_standardization_zero_mean_unit_var(self):
        r = _ramp(200)
        raw = encoders.time_raw(r)
        stats = encoders.fit_standardizer([raw])
        std = encoders.apply_standardizer(raw, stats)
        for c in range(6):
            col = std[c]
            m = sum(col) / len(col)
            var = sum((x - m) ** 2 for x in col) / len(col)
            self.assertLess(abs(m), 1e-6)
            # constant channels (none here) would have std forced to 1; ramp varies
            self.assertTrue(abs(var - 1.0) < 1e-6 or var == 0.0)


class RasterEncoderTest(unittest.TestCase):
    def test_shape(self):
        from raw_input_classifier.trace import Report
        r = [Report(i * 1000, 1, 0, 0, 0, 0) for i in range(32)]
        img = encoders.raster_encode(r, h=64, w=64)
        self.assertEqual(len(img), 3)
        self.assertEqual(len(img[0]), 64)
        self.assertEqual(len(img[0][0]), 64)
        self.assertEqual(encoders.raster_shape(64, 64), (3, 64, 64))

    def test_l_shaped_path_occupancy(self):
        # move right 16 then up 16: occupancy must hit both arms of the L
        from raw_input_classifier.trace import Report
        reps = [Report(i * 1000, 1, 0, 0, 0, 0) for i in range(16)] + \
               [Report((16 + i) * 1000, 0, -1, 0, 0, 0) for i in range(16)]
        occ = encoders.raster_encode(reps, h=64, w=64)[0]
        lit = sum(1 for row in occ for v in row if v > 0)
        self.assertGreater(lit, 10)            # both arms rendered
        # recency: the last point should be among the brightest occupancy cells
        self.assertAlmostEqual(max(v for row in occ for v in row), 1.0, places=6)

    def test_click_channel_lit_only_with_click(self):
        from raw_input_classifier.trace import Report
        no_click = [Report(i * 1000, 1, 1, 0, 0, 0) for i in range(16)]
        clk0 = encoders.raster_encode(no_click)[2]
        self.assertEqual(sum(v for row in clk0 for v in row), 0.0)
        with_click = [Report(i * 1000, 1, 1, 1 if i == 8 else 0, 0, 0) for i in range(16)]
        clk1 = encoders.raster_encode(with_click)[2]
        self.assertGreater(sum(v for row in clk1 for v in row), 0.0)


if __name__ == "__main__":
    unittest.main()
