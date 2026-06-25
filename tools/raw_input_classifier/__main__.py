"""CLI: rawcls capture | train | eval | compare-encoders.

Invoke as `python -m raw_input_classifier ...` (run from the tools/ dir or with
PYTHONPATH=tools), or as `rawcls ...` after `pip install -e tools/raw_input_classifier`.
"""
import argparse
import glob
import sys

from . import capture, trace


def _file_source(path):
    """Yield (t_us,dx,dy,bl,br,bm) from a whitespace 't_us dx dy bl br bm' file."""
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 6:
                continue
            try:
                yield tuple(int(p) for p in parts[:6])
            except ValueError:
                continue


def _cmd_capture(a):
    if a.from_file:
        src = _file_source(a.from_file)
    elif a.port:
        src = capture.serial_source(a.port, a.baud)
    else:
        print("capture: provide --from-file or --port", file=sys.stderr)
        return 2
    n = capture.capture_from_iter(src, a.label, a.source, a.out,
                                  rate_hz=a.rate_hz, note=a.note, limit=a.limit)
    print(f"captured {n} reports -> {a.out}")
    return 0


def _need_torch():
    from . import model
    if not model._HAVE_TORCH:
        print("this command needs PyTorch — install the [torch] extra "
              "(pip install -e tools/raw_input_classifier[torch])", file=sys.stderr)
        return False
    return True


def _cmd_train(a):
    if not _need_torch():
        return 2
    from . import train
    paths = sorted(p for g in a.data for p in glob.glob(g))
    v = train.train_eval(paths, a.encoder, n=a.n, stride=a.stride, epochs=a.epochs,
                         min_windows=a.min_windows, seed=a.seed, width=a.width)
    print(train.format_verdict(v))
    if a.out and not v["insufficient"]:
        # train_eval returns metrics only; checkpoint persistence is a follow-up
        # (BaseCNN.save/load exists but is not yet wired through the CLI).
        print(f"checkpoint save not yet wired in v1; verdict above is the result")
    return 0


def _cmd_eval(a):
    if getattr(a, "ckpt", None):
        print("eval: --ckpt is not wired in v1; re-fitting on --data instead "
              "(see tools/raw_input_classifier/README.md Limitations).", file=sys.stderr)
    if not _need_torch():
        return 2
    from . import train
    paths = sorted(p for g in a.data for p in glob.glob(g))
    # eval re-fits+evaluates on the provided corpus (no separate frozen ckpt in v1)
    v = train.train_eval(paths, a.encoder, n=a.n, stride=a.stride, epochs=a.epochs,
                         min_windows=a.min_windows, seed=a.seed, width=a.width)
    print(train.format_verdict(v))
    return 0


def _cmd_compare(a):
    if not _need_torch():
        return 2
    from . import train
    paths = sorted(p for g in a.data for p in glob.glob(g))
    print("comparing encoders (time vs raster) — higher AUC = more revealing view\n")
    for enc in ("time", "raster"):
        v = train.train_eval(paths, enc, n=a.n, stride=a.stride, epochs=a.epochs,
                             min_windows=a.min_windows, seed=a.seed, width=a.width)
        print(train.format_verdict(v))
        print()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="rawcls",
                                 description="Raw-input classifier — A/B benchmark")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="capture a source into a labeled .jsonl")
    c.add_argument("--from-file"); c.add_argument("--port"); c.add_argument("--baud", type=int, default=115200)
    c.add_argument("--label", required=True, choices=trace.VALID_LABELS)
    c.add_argument("--source", required=True)
    c.add_argument("--out", required=True)
    c.add_argument("--rate-hz", type=int, default=1000, dest="rate_hz")
    c.add_argument("--note", default=""); c.add_argument("--limit", type=int)
    c.set_defaults(fn=_cmd_capture)

    def _add_train_args(p):
        p.add_argument("--data", nargs="+", required=True, help="trace globs")
        p.add_argument("--encoder", choices=("time", "raster"), default="time")
        p.add_argument("--n", type=int, default=256)
        p.add_argument("--stride", type=int, default=128)
        p.add_argument("--epochs", type=int, default=20)
        p.add_argument("--min-windows", type=int, default=500, dest="min_windows")
        p.add_argument("--seed", type=int, default=0)
        p.add_argument("--width", type=int, default=16)
        p.add_argument("--out")

    t = sub.add_parser("train", help="train + verdict on a corpus"); _add_train_args(t); t.set_defaults(fn=_cmd_train)
    e = sub.add_parser("eval", help="evaluate the verdict on a corpus")
    e.add_argument("--ckpt"); _add_train_args(e); e.set_defaults(fn=_cmd_eval)
    cm = sub.add_parser("compare-encoders", help="run both encoders, compare AUC"); _add_train_args(cm); cm.set_defaults(fn=_cmd_compare)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
