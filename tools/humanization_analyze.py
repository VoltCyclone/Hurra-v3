#!/usr/bin/env python3
"""Tier-1 humanization analyzer.

Reads a motion trace (one "dx dy" pair per line, whitespace-separated; lines
that fail to parse are skipped) and reports the kinematic signatures anti-cheat
Tier-1 detectors look for. Compare a captured device-output trace against a
real human baseline (a passthrough-mouse capture).

Usage:  tools/humanization_analyze.py trace.txt [--baseline human.txt]
"""
import sys, argparse, math, statistics

def load(path):
    xs = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 2: continue
            try: xs.append((float(p[0]), float(p[1])))
            except ValueError: continue
    return xs

def metrics(tr):
    # velocity per frame = the delta itself (1 frame dt)
    v = [math.hypot(dx, dy) for dx, dy in tr]
    a = [v[i] - v[i-1] for i in range(1, len(v))]          # acceleration
    j = [a[i] - a[i-1] for i in range(1, len(a))]          # jerk
    zero_jerk = sum(1 for x in j if abs(x) < 1e-9) / max(len(j), 1)
    # longest identical-value run on dx (quantization tell)
    run = mx = 1
    for i in range(1, len(tr)):
        run = run + 1 if tr[i][0] == tr[i-1][0] else 1
        mx = max(mx, run)
    return {
        "frames": len(tr),
        "zero_jerk_frac": zero_jerk,
        "jerk_rms": (statistics.pstdev(j) if len(j) > 1 else 0.0),
        "max_identical_run": mx,
        "vel_mean": (statistics.mean(v) if v else 0.0),
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--baseline")
    a = ap.parse_args()
    m = metrics(load(a.trace))
    print(f"trace: {a.trace}")
    for k, val in m.items(): print(f"  {k}: {val:.4f}" if isinstance(val,float) else f"  {k}: {val}")
    # Heuristic pass/fail (tune against the human baseline)
    flags = []
    if m["zero_jerk_frac"] > 0.20: flags.append("HIGH zero-jerk fraction (robotic)")
    if m["max_identical_run"] > 200: flags.append("long identical-value run (quantized)")
    if m["jerk_rms"] < 0.05 and m["vel_mean"] > 1.0: flags.append("near-zero jerk variance (too smooth)")
    if a.baseline:
        b = metrics(load(a.baseline))
        print(f"baseline: {a.baseline}")
        for k, val in b.items(): print(f"  {k}: {val:.4f}" if isinstance(val,float) else f"  {k}: {val}")
    print("RESULT:", "FLAGS: " + "; ".join(flags) if flags else "looks human (Tier-1)")
    return 1 if flags else 0

if __name__ == "__main__":
    sys.exit(main())
