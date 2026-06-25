import os, sys, tempfile, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from raw_input_classifier import model

_HAVE = getattr(model, "_HAVE_TORCH", False)


@unittest.skipUnless(_HAVE, "torch not installed ([torch] extra)")
class ModelTest(unittest.TestCase):
    def test_time_forward_shape(self):
        import torch
        m = model.build("time", (6, 256))
        out = m(torch.zeros(4, 6, 256))
        self.assertEqual(tuple(out.shape), (4, 2))

    def test_raster_forward_shape(self):
        import torch
        m = model.build("raster", (3, 64, 64))
        out = m(torch.zeros(2, 3, 64, 64))
        self.assertEqual(tuple(out.shape), (2, 2))

    def test_save_load_roundtrips_weights_and_stats(self):
        import torch
        m = model.build("time", (6, 256))
        stats = {"means": [0.0] * 6, "stds": [1.0] * 6}
        cfg = {"n": 256, "stride": 128}
        p = tempfile.mktemp(suffix=".pt")
        m.save(p, stats=stats, cfg=cfg, meta={"git": "abc"})
        m2, s2, c2, meta2 = model.BaseCNN.load(p)
        x = torch.randn(2, 6, 256)
        self.assertTrue(torch.allclose(m(x), m2(x), atol=1e-6))
        self.assertEqual(s2, stats)
        self.assertEqual(c2["n"], 256)
        self.assertEqual(meta2["git"], "abc")

    def test_one_step_decreases_loss(self):
        import torch
        m = model.build("time", (6, 64))
        opt = torch.optim.Adam(m.parameters(), lr=1e-2)
        x = torch.randn(8, 6, 64)
        y = torch.tensor([0, 1] * 4)
        lossfn = torch.nn.CrossEntropyLoss()
        l0 = lossfn(m(x), y).item()
        for _ in range(5):
            opt.zero_grad(); loss = lossfn(m(x), y); loss.backward(); opt.step()
        self.assertLess(loss.item(), l0)


@unittest.skipUnless(_HAVE, "torch not installed ([torch] extra)")
class DatasetTest(unittest.TestCase):
    def _win(self, n=64):
        from raw_input_classifier.trace import Report
        return [Report(i * 1000, i % 4, (i % 3) - 1, 0, 0, 0) for i in range(n)]

    def test_time_dataset_item_shape(self):
        from raw_input_classifier import dataset
        w = self._win(64)
        stats = dataset.fit_time_stats([w])
        ds = dataset.WindowDataset([(w, 1)], "time", stats)
        x, y = ds[0]
        self.assertEqual(tuple(x.shape), (6, 64))
        self.assertEqual(int(y), 1)

    def test_raster_dataset_item_shape(self):
        from raw_input_classifier import dataset
        w = self._win(64)
        ds = dataset.WindowDataset([(w, 0)], "raster", {})
        x, y = ds[0]
        self.assertEqual(tuple(x.shape), (3, 64, 64))
        self.assertEqual(int(y), 0)


if __name__ == "__main__":
    unittest.main()
