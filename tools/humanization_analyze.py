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

def _ks(a, b):
    a = sorted(a); b = sorted(b)
    import bisect
    grid = sorted(set(a) | set(b))
    d = 0.0
    na, nb = len(a), len(b)
    for x in grid:
        fa = bisect.bisect_right(a, x) / na
        fb = bisect.bisect_right(b, x) / nb
        d = max(d, abs(fa - fb))
    return d

def _speeds(stream):
    return [math.hypot(dx, dy) for dx, dy in stream]

def _band_power(stream, lo_hz, hi_hz, fs=1000.0):
    y = [dy for _, dy in stream]
    # Naive DFT is O(N^2); cap to first 512 samples so large traces stay fast.
    # Apply the same cap to both emitted and human streams so the ratio is
    # apples-to-apples.  Use numpy.fft when available for large-N callers.
    CAP = 512
    if len(y) > CAP:
        y = y[:CAP]
    N = len(y)
    p = 0.0
    for k in range(N):
        f = k * fs / N
        if lo_hz <= f <= hi_hz:
            re = sum(y[n]*math.cos(-2*math.pi*k*n/N) for n in range(N))
            im = sum(y[n]*math.sin(-2*math.pi*k*n/N) for n in range(N))
            p += (re*re + im*im)
    return p

def ab_compare(emitted, human):
    # Returns vel/accel/jerk KS distances + 8-12 Hz tremor band-power deviation.
    # cadence_ks (inter-report dt distance) is intentionally DEFERRED: the (dx, dy)
    # stream format carries no per-sample timestamp to derive dt from. When a
    # timestamped trace format lands (e.g. (dx, dy, t_us) from a device capture),
    # add "cadence_ks": _ks(dt_emitted, dt_human) here and to passes().
    se, sh = _speeds(emitted), _speeds(human)
    ae = [se[i+1]-se[i] for i in range(len(se)-1)]
    ah = [sh[i+1]-sh[i] for i in range(len(sh)-1)]
    je = [ae[i+1]-ae[i] for i in range(len(ae)-1)]
    jh = [ah[i+1]-ah[i] for i in range(len(ah)-1)]
    pe = _band_power(emitted, 8, 12)
    ph = _band_power(human, 8, 12) or 1e-9
    return {
        "vel_ks":   _ks(se, sh),
        "accel_ks": _ks(ae, ah),
        "jerk_ks":  _ks(je, jh),
        "tremor_band_ratio_dev": abs(1.0 - pe/ph),
    }

def passes(r):
    # Thresholds: KS < 0.2 for each kinematic order; tremor dev < 0.5.
    # Starting points to tune against real captured traces.
    return (r["vel_ks"] < 0.2 and r["accel_ks"] < 0.2 and r["jerk_ks"] < 0.2
            and r["tremor_band_ratio_dev"] < 0.5)

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
