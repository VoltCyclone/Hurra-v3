"""Window encoders — pure stdlib, firmware-facing math, no numpy/torch.

Two front-ends turn a window of HID reports into a fixed-shape nested list:
  time_encode   -> [6][N]      the "uniform time-series" reading
  raster_encode -> [3][H][W]   the slide's literal trajectory-plot reading
A later torch dataset converts these lists to tensors.
"""
import math

TIME_CHANNELS = ("dx", "dy", "speed", "dt_us", "bl", "br")


def time_shape(n):
    return (len(TIME_CHANNELS), n)


def time_raw(reports):
    """[6][N] raw (pre-standardization) channels: dx, dy, speed, dt_us, bl, br."""
    n = len(reports)
    dx = [float(r.dx) for r in reports]
    dy = [float(r.dy) for r in reports]
    speed = [math.hypot(r.dx, r.dy) for r in reports]
    dt = [0.0] + [float(reports[i].t_us - reports[i - 1].t_us) for i in range(1, n)]
    bl = [float(r.bl) for r in reports]
    br = [float(r.br) for r in reports]
    return [dx, dy, speed, dt, bl, br]


def fit_standardizer(channels_list):
    """Per-channel mean/std pooled across many [6][N] windows (training split only).
    A zero-variance channel gets std=1.0 so standardization is a safe no-op."""
    nch = len(TIME_CHANNELS)
    means = [0.0] * nch
    stds = [1.0] * nch
    for c in range(nch):
        vals = [v for w in channels_list for v in w[c]]
        if not vals:
            continue
        m = sum(vals) / len(vals)
        var = sum((x - m) ** 2 for x in vals) / len(vals)
        means[c] = m
        stds[c] = math.sqrt(var) if var > 0 else 1.0
    return {"means": means, "stds": stds}


def apply_standardizer(channels, stats):
    m, s = stats["means"], stats["stds"]
    return [[(v - m[c]) / s[c] for v in channels[c]] for c in range(len(channels))]


def time_encode(reports, stats):
    return apply_standardizer(time_raw(reports), stats)


def raster_shape(h=64, w=64):
    return (3, h, w)


def raster_encode(reports, h=64, w=64):
    """[3][h][w] image: ch0 occupancy-by-recency, ch1 speed, ch2 click.

    Cumulative-sum the deltas to a trajectory, scale-fit into the grid, and
    rasterize. Discards exact dt (known tradeoff) but exposes spatial shape the
    time view flattens. Empty/degenerate windows render all-zero (safe)."""
    occ = [[0.0] * w for _ in range(h)]
    spd = [[0.0] * w for _ in range(h)]
    clk = [[0.0] * w for _ in range(h)]
    n = len(reports)
    if n == 0:
        return [occ, spd, clk]
    xs = []
    ys = []
    x = y = 0.0
    for r in reports:
        x += r.dx
        y += r.dy
        xs.append(x)
        ys.append(y)
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    spanx = (maxx - minx) or 1.0
    spany = (maxy - miny) or 1.0
    for i, r in enumerate(reports):
        px = int((xs[i] - minx) / spanx * (w - 1))
        py = int((ys[i] - miny) / spany * (h - 1))
        recency = (i + 1) / n
        if recency > occ[py][px]:
            occ[py][px] = recency
        sp = math.hypot(r.dx, r.dy)
        if sp > spd[py][px]:
            spd[py][px] = sp
        if (r.bl or r.br) and clk[py][px] < 1.0:
            clk[py][px] = 1.0
    return [occ, spd, clk]
