"""Surface metrics of the laid print against the slicer's intent.

The outer wall's edge is the nozzle's offset from the commanded wall
line along its outward normal plus half the bead width:
d = n.(p - a) + (w - w_nominal) / 2, w = flow / (speed * layer height).
Per straight wall (corner zones cut off) the nozzle's part n.(p - a) is
resampled along the wall; its detrended oscillation is the ghosting
(amplitude sqrt(2) * rms, dominant spatial frequency from the spectrum).
The width's part is flow error, reported as line-width variation. Per layer, the walls' mean
offsets fit a translation (layer shift) and a uniform outline offset
(dimensional error). Corner bulge and under-fill are read off the laid
layer: material outside the intended outline, and uncovered cells of the
outer bead's band, near each vertex.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

CORNER_ZONE_WIDTHS = 2.0
RESAMPLE_MM = 0.01
CORNER_RADIUS_MM = 1.5
MIN_WALL_MM = 2.0


class SurfaceError(Exception):
    pass


@dataclasses.dataclass
class Wall:
    layer: int
    line: int
    start: np.ndarray
    normal: np.ndarray
    s0: float
    s: np.ndarray
    offset: np.ndarray
    width: np.ndarray
    nominal_width: float
    speed: float

    @property
    def d(self):
        """Edge offset from the intended outline (mm)."""
        return self.offset + 0.5 * (self.width - self.nominal_width)

    def oscillation(self):
        grid = np.arange(self.s[0], self.s[-1], RESAMPLE_MM)
        d = np.interp(grid, self.s, self.offset)
        trend = np.polyval(np.polyfit(grid, d, 1), grid)
        return grid, d - trend

    def ghosting(self):
        grid, osc = self.oscillation()
        amplitude = math.sqrt(2.0) * float(np.sqrt(np.mean(osc**2)))
        window = np.hanning(len(osc))
        spectrum = np.abs(np.fft.rfft(osc * window))
        freqs = np.fft.rfftfreq(len(osc), RESAMPLE_MM)
        i = int(np.argmax(spectrum[1:])) + 1
        if 1 < i < len(spectrum) - 1:
            a, b, c = np.log(spectrum[i - 1 : i + 2] + 1e-300)
            shift = 0.5 * (a - c) / (a - 2.0 * b + c)
        else:
            shift = 0.0
        return amplitude, float(freqs[i] + shift * (freqs[1] - freqs[0]))


def _orientation(points):
    x, y = np.asarray(points).T
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def outer_loops(pairs):
    """Per layer: the outer-wall moves in order."""
    loops = {}
    for move, cap in pairs:
        if move.kind == "WALL-OUTER" and move.layer is not None:
            loops.setdefault(move.layer, []).append((move, cap))
    return loops


def walls(pairs, t, xy, width, nominal_width, support):
    """Wall samples for every straight outer-wall move: t, xy the nozzle,
    width the bead width at each sample. support: how far the shaped
    motion reaches outside a move's commanded time (s)."""
    out = []
    zone = CORNER_ZONE_WIDTHS * nominal_width
    speed = np.hypot(*np.gradient(xy, t, axis=0).T)
    for layer, loop in outer_loops(pairs).items():
        sign = (
            1.0 if _orientation([m.start[:2] for m, _ in loop]) > 0.0 else -1.0
        )
        s0 = 0.0
        for move, cap in loop:
            a = np.array(move.start[:2])
            b = np.array(move.end[:2])
            length = float(np.hypot(*(b - a)))
            e = (b - a) / length
            n = sign * np.array([e[1], -e[0]])
            t0 = cap["print_time"] - support
            t1 = (
                cap["print_time"]
                + cap["accel_t"]
                + cap["cruise_t"]
                + cap["decel_t"]
                + support
            )
            i = np.flatnonzero((t >= t0) & (t <= t1))
            u = (xy[i] - a) @ e
            keep = (u >= zone) & (u <= length - zone)
            if length - 2.0 * zone >= MIN_WALL_MM and keep.sum() > 2:
                i, u = i[keep], u[keep]
                order = np.argsort(u)
                i, u = i[order], u[order]
                out.append(
                    Wall(
                        layer,
                        move.line,
                        a,
                        n,
                        s0,
                        u,
                        (xy[i] - a) @ n,
                        width[i],
                        nominal_width,
                        cap["cruise_v"],
                    )
                )
            s0 += length
    return out


def layer_fit(layer_walls):
    """Least-squares translation (x, y) and outline offset of one layer
    from its walls' mean edge offsets: mean d = n . shift + offset."""
    if len(layer_walls) < 3:
        raise SurfaceError(
            "layer %d has %d walls" % (layer_walls[0].layer, len(layer_walls))
        )
    a = np.array([[w.normal[0], w.normal[1], 1.0] for w in layer_walls])
    m = np.array([float(np.mean(w.d)) for w in layer_walls])
    sol, *_ = np.linalg.lstsq(a, m, rcond=None)
    return sol[:2], float(sol[2])


def _offset_convex(points, distance):
    """Each edge moved outward by distance; new vertices where adjacent
    edges meet. points: a convex loop, counter-clockwise."""
    p = np.asarray(points, dtype=float)
    out = []
    for k in range(len(p)):
        a0, a1 = p[k - 1], p[k]
        b0, b1 = p[k], p[(k + 1) % len(p)]
        da, db = a1 - a0, b1 - b0
        na = np.array([da[1], -da[0]]) / np.hypot(*da)
        nb = np.array([db[1], -db[0]]) / np.hypot(*db)
        pa, pb = a0 + distance * na, b0 + distance * nb
        mat = np.array([da, -db]).T
        s, _ = np.linalg.solve(mat, pb - pa)
        out.append(pa + s * da)
    return np.array(out)


def _inside_convex(poly, x, y):
    inside = np.ones(np.broadcast(x, y).shape, dtype=bool)
    for k in range(len(poly)):
        a, b = poly[k], poly[(k + 1) % len(poly)]
        inside &= (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) >= 0.0
    return inside


def corners(pairs, layers, nominal_width):
    """Per kept layer and outer-wall vertex: volume outside the intended
    outline (bulge, mm^3) and uncovered area of the outer bead's band
    times the layer height (under-fill, mm^3), within CORNER_RADIUS_MM."""
    grid = layers.grid
    xc, yc = grid.centres()
    out = []
    for layer, loop in outer_loops(pairs).items():
        if layer not in layers.fields:
            continue
        pts = [m.start[:2] for m, _ in loop]
        if _orientation(pts) < 0.0:
            pts = pts[::-1]
        outer = _offset_convex(pts, 0.5 * nominal_width)
        inner = _offset_convex(pts, -0.5 * nominal_width)
        field = layers.fields[layer]
        covered = layers.covered(layer)
        for vertex in outer:
            ix = np.flatnonzero(np.abs(xc - vertex[0]) <= CORNER_RADIUS_MM)
            iy = np.flatnonzero(np.abs(yc - vertex[1]) <= CORNER_RADIUS_MM)
            gx, gy = np.meshgrid(xc[ix], yc[iy])
            near = np.hypot(gx - vertex[0], gy - vertex[1]) <= CORNER_RADIUS_MM
            sub = np.ix_(iy, ix)
            in_outer = _inside_convex(outer, gx, gy)
            band = in_outer & ~_inside_convex(inner, gx, gy)
            bulge = float(field[sub][near & ~in_outer].sum()) * grid.dx**2
            under = (
                float((near & band & ~covered[sub]).sum())
                * grid.dx**2
                * layers.layer_height
            )
            out.append(
                {"layer": layer, "bulge_mm3": bulge, "underfill_mm3": under}
            )
    return out


def summarize(wall_list, corner_list, nominal_width):
    if not wall_list:
        raise SurfaceError("no straight outer walls to measure")
    ghosts = [w.ghosting() for w in wall_list]
    amp = np.array([g[0] for g in ghosts]) * 1e3
    freq = np.array([g[1] for g in ghosts])
    speeds = np.array([w.speed for w in wall_list])
    by_speed = {}
    for v in sorted(set(np.round(speeds, 3))):
        sel = np.round(speeds, 3) == v
        by_speed["%g" % v] = {
            "walls": int(sel.sum()),
            "amplitude_um_median": float(np.median(amp[sel])),
            "amplitude_um_max": float(amp[sel].max()),
            "wavelength_mm_median": float(np.median(1.0 / freq[sel])),
        }
    layers = sorted({w.layer for w in wall_list})
    fits = {
        k: layer_fit([w for w in wall_list if w.layer == k]) for k in layers
    }
    base = fits[layers[0]][0]
    shift = max(float(np.hypot(*(fits[k][0] - base))) for k in layers)
    offsets = np.array([fits[k][1] for k in layers])
    widths = np.concatenate([w.width for w in wall_list])
    out = {
        "ghosting": by_speed,
        "layer_shift_mm": shift,
        "outline_offset_mm": float(np.mean(offsets)),
        "size_error_mm": 2.0 * float(np.mean(offsets)),
        "line_width": {
            "nominal_mm": nominal_width,
            "mean_mm": float(widths.mean()),
            "relative_std": float(widths.std() / widths.mean()),
            "p1_mm": float(np.percentile(widths, 1)),
            "p99_mm": float(np.percentile(widths, 99)),
        },
    }
    if corner_list:
        bulge = np.array([c["bulge_mm3"] for c in corner_list])
        under = np.array([c["underfill_mm3"] for c in corner_list])
        out["corners"] = {
            "count": len(corner_list),
            "bulge_mm3_mean": float(bulge.mean()),
            "bulge_mm3_max": float(bulge.max()),
            "underfill_mm3_mean": float(under.mean()),
            "underfill_mm3_max": float(under.max()),
        }
    return out
