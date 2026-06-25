"""Bridge stdlib encoder output -> torch tensors. Torch-gated.

fit_time_stats pools standardization over the TRAIN windows only; eval/inference
reuse those frozen stats (stored in the checkpoint) so they never peek at their
own distribution.
"""
try:
    import torch
    from torch.utils.data import Dataset
    _HAVE_TORCH = True
except ImportError:                        # pragma: no cover - import guard
    _HAVE_TORCH = False

from . import encoders


def fit_time_stats(train_windows):
    raws = [encoders.time_raw(w) for w in train_windows]
    return encoders.fit_standardizer(raws)


def encode_window(window, encoder_name, stats):
    if encoder_name == "time":
        return encoders.time_encode(window, stats)
    if encoder_name == "raster":
        return encoders.raster_encode(window)
    raise ValueError(f"unknown encoder {encoder_name!r}")


if _HAVE_TORCH:

    class WindowDataset(Dataset):
        def __init__(self, samples, encoder_name, stats):
            # samples: list of (window:list[Report], label:int)
            self.samples = samples
            self.encoder_name = encoder_name
            self.stats = stats

        def __len__(self):
            return len(self.samples)

        def __getitem__(self, i):
            window, label = self.samples[i]
            arr = encode_window(window, self.encoder_name, self.stats)
            x = torch.tensor(arr, dtype=torch.float32)
            return x, torch.tensor(int(label), dtype=torch.long)
