"""Train loop + the A/B verdict (AUC / EER / per-channel ablation). Torch-gated.

The verdict is the product: AUC ~0.5 means the CNN cannot separate firmware output
from human (firmware wins); the ablation names which channel leaks most. The
honesty gate refuses to print a number below min_windows.
"""
try:
    import torch
    from torch.utils.data import DataLoader
    _HAVE_TORCH = True
except ImportError:                        # pragma: no cover - import guard
    _HAVE_TORCH = False

from . import trace, windowing, dataset, model, metrics, encoders

_LABEL_INT = {"human": 0, "synthetic": 1}


def load_corpus(paths):
    files = []
    for p in paths:
        hdr, reps, _ = trace.read(p)
        files.append({"groups": (hdr["source"], p),
                      "source": hdr["source"],
                      "label": _LABEL_INT[hdr["label"]],
                      "reports": reps})
    return files


def build_samples(files, n, stride, group_set):
    samples = []
    for f in files:
        if f["groups"] not in group_set:
            continue
        for w in windowing.windows(f["reports"], n, stride):
            samples.append((w, f["label"]))
    return samples


def _shape(encoder_name, n):
    return encoders.time_shape(n) if encoder_name == "time" else encoders.raster_shape()


def _run_eval(m, loader, ablate_ch=None):
    m.eval()
    scores, labels = [], []
    with torch.no_grad():
        for x, y in loader:
            if ablate_ch is not None:
                x = x.clone()
                x[:, ablate_ch] = 0.0
            scores.extend(m.predict_proba(x).tolist())
            labels.extend(y.tolist())
    return scores, labels


def train_eval(paths, encoder_name, n=256, stride=128, epochs=20,
               min_windows=500, seed=0, width=16, batch=64):
    if not _HAVE_TORCH:
        raise RuntimeError("train_eval requires PyTorch — install the [torch] extra")
    torch.manual_seed(seed)
    files = load_corpus(paths)
    groups = [f["groups"] for f in files]
    train_g, val_g = windowing.group_split(groups, val_frac=0.25, seed=seed)
    windowing.assert_no_leakage(train_g, val_g)

    train_s = build_samples(files, n, stride, train_g)
    val_s = build_samples(files, n, stride, val_g)
    total = len(train_s) + len(val_s)
    n_human = sum(1 for _, l in train_s + val_s if l == 0)
    n_synth = total - n_human

    if total < min_windows or not train_s or not val_s:
        return {"encoder": encoder_name, "n_windows": total, "n_human": n_human,
                "n_synth": n_synth, "auc": float("nan"), "eer": float("nan"),
                "acc": float("nan"), "ablation": {}, "insufficient": True}

    stats = dataset.fit_time_stats([w for w, _ in train_s]) if encoder_name == "time" else {}
    tr = DataLoader(dataset.WindowDataset(train_s, encoder_name, stats),
                    batch_size=batch, shuffle=True)
    va = DataLoader(dataset.WindowDataset(val_s, encoder_name, stats),
                    batch_size=batch, shuffle=False)

    m = model.build(encoder_name, _shape(encoder_name, n), {"width": width})
    opt = torch.optim.Adam(m.parameters(), lr=1e-3)
    n_human_tr = sum(1 for _, l in train_s if l == 0)
    n_synth_tr = len(train_s) - n_human_tr
    wpos = max(1.0, n_human_tr / max(1, n_synth_tr))
    lossfn = torch.nn.CrossEntropyLoss(weight=torch.tensor([1.0, wpos]))
    for _ in range(epochs):
        m.train()
        for x, y in tr:
            opt.zero_grad(); loss = lossfn(m(x), y); loss.backward(); opt.step()

    scores, labels = _run_eval(m, va)
    auc = metrics.auc(scores, labels)
    eer = metrics.eer(scores, labels)
    acc = sum(1 for s, l in zip(scores, labels) if (s >= 0.5) == (l == 1)) / len(labels)

    ablation = {}
    if encoder_name == "time":
        for c, name in enumerate(encoders.TIME_CHANNELS):
            s2, l2 = _run_eval(m, va, ablate_ch=c)
            ablation[name] = round(auc - metrics.auc(s2, l2), 4)

    return {"encoder": encoder_name, "n_windows": total, "n_human": n_human,
            "n_synth": n_synth, "auc": auc, "eer": eer, "acc": acc,
            "ablation": ablation, "insufficient": False}


def format_verdict(v):
    if v["insufficient"]:
        return (f"INSUFFICIENT DATA — captured corpus too small for a trustworthy AUC "
                f"({v['n_windows']} windows; raise the corpus or lower --min-windows).")
    lines = ["=== raw-input classifier — A/B verdict ===",
             f"encoder: {v['encoder']}   windows: {v['n_windows']} "
             f"(human {v['n_human']} / synth {v['n_synth']})  split: by-file",
             f"        AUC {v['auc']:.2f}    EER {v['eer']:.2f}    acc {v['acc']:.2f}"]
    if v["auc"] < 0.6:
        lines.append("  → classifier struggles to separate.  GOOD for firmware (target AUC→0.50)")
    elif v["auc"] > 0.8:
        lines.append("  → classifier separates easily.  firmware EXPOSED (target AUC→0.50)")
    else:
        lines.append("  → partial separation.  firmware leaking (target AUC→0.50)")
    if v["ablation"]:
        lines.append("per-channel ablation (AUC drop when channel zeroed):")
        for ch, drop in sorted(v["ablation"].items(), key=lambda kv: -kv[1]):
            lines.append(f"  {ch:<6} {drop:+.3f}")
    return "\n".join(lines)
