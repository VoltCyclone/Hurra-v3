"""Window extraction and leakage-safe train/val splitting. Pure stdlib.

The split groups by capture identity (source, file): windows from one session
share strong autocorrelation, so any session straddling train/val would leak and
inflate the AUC. assert_no_leakage enforces the invariant downstream callers rely
on.
"""
import random


def windows(reports, n, stride):
    out = []
    i = 0
    last = len(reports) - n
    while i <= last:
        out.append(reports[i:i + n])
        i += stride
    return out


def group_split(groups, val_frac=0.25, seed=0):
    """Split unique group keys into (train, val) sets. Deterministic for a seed.

    With a single unique group no leak-free split exists, so train comes back
    empty; the caller's insufficient-data gate (train_eval) handles that case."""
    uniq = list(dict.fromkeys(groups))          # de-dup, stable order
    rnd = random.Random(seed)
    rnd.shuffle(uniq)
    k = max(1, int(len(uniq) * val_frac))
    val = set(uniq[:k])
    train = set(uniq[k:])
    return train, val


def assert_no_leakage(train_groups, val_groups):
    overlap = set(train_groups) & set(val_groups)
    if overlap:
        raise AssertionError(f"train/val leakage on groups: {sorted(overlap)}")
