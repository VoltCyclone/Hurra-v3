"""Conv2d adversary models. The only-real-torch unit (with dataset.py, train.py).

Two small, deliberately modest nets so AUC is trustworthy on a limited early
corpus rather than inflated by overfitting. Width/depth come from cfg so the
model scales up later without touching the harness.
"""
try:
    import torch
    import torch.nn as nn
    _HAVE_TORCH = True
except Exception:                       # pragma: no cover - import guard
    _HAVE_TORCH = False


if _HAVE_TORCH:

    class BaseCNN(nn.Module):
        def __init__(self, encoder_name, input_shape, cfg=None):
            super().__init__()
            self.encoder_name = encoder_name
            self.input_shape = tuple(input_shape)
            self.cfg = dict(cfg or {})

        def predict_proba(self, x):
            """P(class==1 == synthetic) per row."""
            self.eval()
            with torch.no_grad():
                return torch.softmax(self.forward(x), dim=1)[:, 1]

        def save(self, path, stats, cfg, meta):
            torch.save({"encoder_name": self.encoder_name,
                        "input_shape": self.input_shape,
                        "model_cfg": self.cfg,
                        "state_dict": self.state_dict(),
                        "stats": stats, "cfg": cfg, "meta": meta}, path)

        @classmethod
        def load(cls, path):
            # weights_only=False: checkpoint carries non-tensor dicts (stats, cfg, meta)
            blob = torch.load(path, map_location="cpu", weights_only=False)
            m = build(blob["encoder_name"], blob["input_shape"], blob["model_cfg"])
            m.load_state_dict(blob["state_dict"])
            m.eval()
            return m, blob["stats"], blob["cfg"], blob["meta"]

    class TimeCNN(BaseCNN):
        def __init__(self, input_shape, cfg=None):
            super().__init__("time", input_shape, cfg)
            c = self.cfg.get("width", 16)
            ch_in = input_shape[0]               # 6 channels
            # treat [C, N] as a 1-row 2D map; conv along time, span channels
            self.net = nn.Sequential(
                nn.Conv1d(ch_in, c, 7, padding=3), nn.BatchNorm1d(c), nn.ReLU(),
                nn.MaxPool1d(2),
                nn.Conv1d(c, c * 2, 5, padding=2), nn.BatchNorm1d(c * 2), nn.ReLU(),
                nn.MaxPool1d(2),
                nn.Conv1d(c * 2, c * 2, 3, padding=1), nn.BatchNorm1d(c * 2), nn.ReLU(),
                nn.AdaptiveAvgPool1d(1))
            self.head = nn.Linear(c * 2, 2)

        def forward(self, x):                    # x: [B, C, N]
            z = self.net(x).squeeze(-1)
            return self.head(z)

    class RasterCNN(BaseCNN):
        def __init__(self, input_shape, cfg=None):
            super().__init__("raster", input_shape, cfg)
            c = self.cfg.get("width", 16)
            ch_in = input_shape[0]               # 3 channels
            self.net = nn.Sequential(
                nn.Conv2d(ch_in, c, 3, padding=1), nn.BatchNorm2d(c), nn.ReLU(),
                nn.MaxPool2d(2),
                nn.Conv2d(c, c * 2, 3, padding=1), nn.BatchNorm2d(c * 2), nn.ReLU(),
                nn.MaxPool2d(2),
                nn.Conv2d(c * 2, c * 2, 3, padding=1), nn.BatchNorm2d(c * 2), nn.ReLU(),
                nn.AdaptiveAvgPool2d(1))
            self.head = nn.Linear(c * 2, 2)

        def forward(self, x):                    # x: [B, C, H, W]
            z = self.net(x).flatten(1)
            return self.head(z)

    def build(encoder_name, input_shape, cfg=None):
        if encoder_name == "time":
            return TimeCNN(input_shape, cfg)
        if encoder_name == "raster":
            return RasterCNN(input_shape, cfg)
        raise ValueError(f"unknown encoder {encoder_name!r}")

else:                                            # pragma: no cover
    def build(*a, **k):
        raise RuntimeError("PyTorch not installed — install the [torch] extra")
