#!/usr/bin/env python3
"""Fit nonlinear pressure-advance parameters from a pa_ident CSV.

Reads the load-telemetry capture produced by the PA_LOAD_IDENT command
(klippy/extras/pa_ident.py) and identifies the melt-pressure dynamics by
fitting the full sampled load trace against a simulated pressure ODE:

    dx/dt = v_cmd(t) - a_inv(x)        x = advance (mm of filament)
    u_pred(t) = baseline(v_cmd) + db - x / C

where a_inv is the inverse of the candidate advance curve

    a(v) = linear_advance * v + nonlinear_offset * f(v / linearization_velocity)

with f = tanh or f = u/(1+|u|), C converts StallGuard units to advance
millimeters, and db absorbs a run-to-run friction offset. Both models are
fitted; the reported RMS decides which matches the extruder.

Usage:
    fit_pa_from_load.py capture.csv [--baseline cold.csv] [--plot out.png]

The optional baseline CSV is a second PA_LOAD_IDENT run without filament
(or cold, so the extruder builds no pressure); it removes the
velocity-dependent friction component of the SG signal. Without it, the
least-loaded steady value is used as a flat baseline.
"""

import argparse
import math
import sys

import numpy as np

SG_MASK = 0x3FF
TAIL_FRACTION = 0.5
SIM_SUBSTEP = 0.005
MIN_STEADY_SEGMENT = 1.0
MIN_OVERLAP_SEP = 30.0
AFFINE_GAIN_BOUNDS = (0.4, 2.5)
AFFINE_RESIDUAL_MAX = 15.0

MODEL_SHAPES = {
    "tanh_pressure_advance": np.tanh,
    "recipr_pressure_advance": lambda u: u / (1.0 + np.abs(u)),
}


def parse_capture(path):
    schedule = []
    times = []
    values = []
    meta = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                for token in line[1:].split():
                    if "=" in token:
                        key, _, val = token.partition("=")
                        meta[key] = val
                continue
            kind, rest = line.split(",", 1)
            if kind == "S":
                fields = rest.split(",")
                if len(fields) == 3:
                    t0, t1, vel = (float(x) for x in fields)
                    sgt = meta.get("sgt")
                    sgt = int(sgt) if sgt is not None else None
                elif len(fields) == 4:
                    t0, t1, vel = (float(x) for x in fields[:3])
                    sgt = int(fields[3])
                else:
                    raise ValueError("malformed S row %r in %s" % (line, path))
                schedule.append((t0, t1, vel, sgt))
            elif kind == "D":
                t, raw = rest.split(",")
                times.append(float(t))
                values.append(int(raw) & SG_MASK)
            else:
                raise ValueError("unknown row kind %r in %s" % (kind, path))
    if not schedule or not times:
        raise ValueError("capture %s has no schedule or no samples" % (path,))
    smooth_time = float(meta.get("smooth_time", 0.0))
    return (
        schedule,
        np.asarray(times),
        np.asarray(values, dtype=float),
        smooth_time,
    )


def steady_tails(schedule, times, values):
    by_velocity = {}
    for t0, t1, vel, _ in schedule:
        if t1 - t0 < MIN_STEADY_SEGMENT:
            continue
        tail_start = t0 + TAIL_FRACTION * (t1 - t0)
        mask = (times >= tail_start) & (times <= t1)
        if not mask.any():
            continue
        by_velocity.setdefault(vel, []).append(float(np.median(values[mask])))
    return {v: float(np.mean(tails)) for v, tails in by_velocity.items()}


def sgt_blocks(schedule):
    blocks = []
    for i, (_, _, _, sgt) in enumerate(schedule):
        if blocks and blocks[-1][0] == sgt:
            blocks[-1][1].append(i)
        else:
            blocks.append((sgt, [i]))
    return blocks


def stitch_sgt(schedule, times, values):
    """Map every sample into the first sgt group's reading units.

    Adjacent constant-sgt blocks re-measure each other's boundary
    velocities, so each seam yields >=2 (reading, reading) pairs at
    distinct load levels: enough to solve an affine map per sgt (gain
    AND offset -- changing sgt is not assumed to be a pure shift).
    Fails loudly when a seam cannot identify the map: too few common
    velocities, overlap readings too close together (gain
    unidentifiable), gain outside a plausible family, or a residual
    that says the seam is not affine over its span.
    """
    sgts = [sgt for _, _, _, sgt in schedule]
    if all(sgt is None for sgt in sgts):
        return values, None, False
    if any(sgt is None for sgt in sgts):
        raise ValueError("capture mixes sgt-tagged and untagged segments")
    blocks = sgt_blocks(schedule)
    if len(blocks) <= 1:
        return values, blocks[0][0], False

    def block_tails(indices):
        out = {}
        for i in indices:
            t0, t1, vel, _ = schedule[i]
            mask = (times >= t0 + TAIL_FRACTION * (t1 - t0)) & (times <= t1)
            if mask.any():
                out.setdefault(vel, []).append(float(np.median(values[mask])))
        return {v: float(np.mean(reads)) for v, reads in out.items()}

    maps_by_sgt = {blocks[0][0]: (1.0, 0.0)}
    block_maps = [(1.0, 0.0)]
    for k in range(1, len(blocks)):
        sgt = blocks[k][0]
        if sgt in maps_by_sgt:
            block_maps.append(maps_by_sgt[sgt])
            continue
        prev_tails = block_tails(blocks[k - 1][1])
        cur_tails = block_tails(blocks[k][1])
        common = sorted(set(prev_tails) & set(cur_tails))
        if len(common) < 2:
            raise ValueError(
                "sgt seam %d->%d shares %d velocities; need >=2 overlap "
                "dwells to identify the affine reading map"
                % (blocks[k - 1][0], sgt, len(common))
            )
        r_prev = np.array([prev_tails[v] for v in common])
        r_cur = np.array([cur_tails[v] for v in common])
        if np.ptp(r_cur) < MIN_OVERLAP_SEP or np.ptp(r_prev) < MIN_OVERLAP_SEP:
            raise ValueError(
                "sgt seam %d->%d overlap readings span %.0f/%.0f counts "
                "(need >=%.0f): gain unidentifiable; pick overlap "
                "velocities with more separated loads"
                % (
                    blocks[k - 1][0],
                    sgt,
                    np.ptp(r_prev),
                    np.ptp(r_cur),
                    MIN_OVERLAP_SEP,
                )
            )
        gain, offset = np.polyfit(r_cur, r_prev, 1)
        if len(common) > 2:
            residual = np.max(np.abs(gain * r_cur + offset - r_prev))
            if residual > AFFINE_RESIDUAL_MAX:
                raise ValueError(
                    "sgt seam %d->%d is not affine over its overlap span "
                    "(residual %.1f counts > %.1f); narrow the sgt step"
                    % (blocks[k - 1][0], sgt, residual, AFFINE_RESIDUAL_MAX)
                )
        if not (AFFINE_GAIN_BOUNDS[0] <= gain <= AFFINE_GAIN_BOUNDS[1]):
            raise ValueError(
                "sgt seam %d->%d fitted gain %.3f outside plausible %s"
                % (blocks[k - 1][0], sgt, gain, AFFINE_GAIN_BOUNDS)
            )
        g_prev, o_prev = block_maps[k - 1]
        composed = (g_prev * gain, g_prev * offset + o_prev)
        maps_by_sgt[sgt] = composed
        block_maps.append(composed)

    block_starts = np.array([schedule[indices[0]][0] for _, indices in blocks])
    sample_block = np.clip(
        np.searchsorted(block_starts, times, side="right") - 1, 0, None
    )
    gains = np.array([g for g, _ in block_maps])[sample_block]
    offsets = np.array([o for _, o in block_maps])[sample_block]
    return gains * values + offsets, blocks[0][0], True


def pressure_curve(tails, baseline_tails):
    velocities = np.array(sorted(tails))
    if baseline_tails is not None:
        b_vs = np.array(sorted(baseline_tails))
        if velocities[0] < b_vs[0] or velocities[-1] > b_vs[-1]:
            raise ValueError(
                "capture velocities %s..%s exceed the baseline's range "
                "%s..%s" % (velocities[0], velocities[-1], b_vs[0], b_vs[-1])
            )
        b_us = np.array([baseline_tails[v] for v in b_vs])
        pressure = np.array(
            [float(np.interp(v, b_vs, b_us)) - tails[v] for v in velocities]
        )
    else:
        flat = max(tails.values())
        pressure = np.array([flat - tails[v] for v in velocities])
    return velocities, pressure


def baseline_of_velocity(tails, baseline_tails):
    if baseline_tails is not None:
        vs = np.array(sorted(baseline_tails))
        us = np.array([baseline_tails[v] for v in vs])
        return lambda v: np.interp(v, vs, us)
    flat = max(tails.values())
    return lambda v: np.full_like(np.asarray(v, dtype=float), flat)


def triangle_cdf(x, smooth_time):
    h = 0.5 * smooth_time
    x = np.clip(x, -h, h)
    rising = 0.5 * (x + h) ** 2 / (h * h)
    falling = 1.0 - 0.5 * (h - x) ** 2 / (h * h)
    return np.where(x < 0.0, rising, falling)


def command_velocity(schedule, times, smooth_time):
    if smooth_time <= 0.0:
        starts = np.array([seg[0] for seg in schedule])
        vels = np.array([seg[2] for seg in schedule])
        idx = np.clip(np.searchsorted(starts, times, side="right") - 1, 0, None)
        return vels[idx]
    v = np.zeros(len(times))
    for t0, t1, vel, _ in schedule:
        v += vel * (
            triangle_cdf(times - t0, smooth_time)
            - triangle_cdf(times - t1, smooth_time)
        )
    return v


def simulate_pressure(u_dense, v_dense, c, times, v_cmd):
    p = 0.0
    out = np.empty(len(times))
    prev_t = times[0]
    for i in range(len(times)):
        dt = times[i] - prev_t
        if dt > 0.0:
            steps = max(1, int(math.ceil(dt / SIM_SUBSTEP)))
            h = dt / steps
            v = v_cmd[i]
            for _ in range(steps):
                outflow = np.interp(p, u_dense, v_dense)
                p += h * (v - outflow) / c
                if p < 0.0:
                    p = 0.0
        out[i] = p
        prev_t = times[i]
    return out


def fit_steady_shape(shape, velocities, pressure):
    v_lin_grid = np.logspace(
        math.log10(velocities[0] / 4.0),
        math.log10(velocities[-1] * 4.0),
        200,
    )
    ones = np.ones(len(velocities))
    best = None
    for v_lin in v_lin_grid:
        basis = np.column_stack([velocities, shape(velocities / v_lin), ones])
        coeffs, _, _, _ = np.linalg.lstsq(basis, pressure, rcond=None)
        coeffs[:2] = np.maximum(coeffs[:2], 0.0)
        rms = float(np.sqrt(np.mean((basis @ coeffs - pressure) ** 2)))
        if best is None or rms < best[0]:
            best = (
                rms,
                float(coeffs[0]),
                float(coeffs[1]),
                float(v_lin),
                float(coeffs[2]),
            )
    assert best is not None
    return best


def fit_scale(
    shape, shape_units, times, values, v_cmd, base_u, v_max, c0, fit_mask
):
    from scipy.optimize import least_squares

    la_u, off_u, v_lin = shape_units
    v_dense = np.linspace(0.0, 1.5 * v_max, 2000)
    u_dense = la_u * v_dense + off_u * shape(v_dense / v_lin)

    def residuals(theta):
        c = math.exp(np.clip(theta[0], -30.0, 10.0))
        db = theta[1]
        p = simulate_pressure(u_dense, v_dense, c, times, v_cmd)
        return ((base_u + db - p) - values)[fit_mask]

    result = least_squares(
        residuals, np.array([math.log(c0), 0.0]), method="lm"
    )
    c = math.exp(np.clip(result.x[0], -30.0, 10.0))
    rms = float(np.sqrt(np.mean(result.fun**2)))
    return c, float(result.x[1]), rms


def fit_scale_fixed_c(
    shape, shape_units, times, values, v_cmd, base_u, v_max, c, fit_mask
):
    la_u, off_u, v_lin = shape_units
    v_dense = np.linspace(0.0, 1.5 * v_max, 2000)
    u_dense = la_u * v_dense + off_u * shape(v_dense / v_lin)
    p = simulate_pressure(u_dense, v_dense, c, times, v_cmd)
    residual = (base_u - p) - values
    db = -float(np.mean(residual[fit_mask]))
    rms = float(np.sqrt(np.mean((residual[fit_mask] + db) ** 2)))
    return db, rms


def rectify_schedule(schedule, times, values, baseline_tails):
    """Correct the analytic schedule against observed transition times.

    The commanded schedule assumes each segment lasts exactly E/v; the
    planner's real timing drifts by a few ms per segment. Boundaries
    whose BASELINE level change is large respond to actual motion (the
    SG velocity term), not melt pressure, so their observed crossing
    times give the true transition times without eating the pressure
    dynamics; a linear time map absorbs drift and constant response lag.
    """
    if baseline_tails is None:
        return schedule, 0.0, 0.0
    levels = []
    for t0, t1, vel, _ in schedule:
        mask = (times >= t0 + 0.5 * (t1 - t0)) & (times <= t1)
        levels.append(float(np.median(values[mask])) if mask.any() else None)
    observations = []
    for i in range(1, len(schedule)):
        lv_a, lv_b = levels[i - 1], levels[i]
        v_a, v_b = schedule[i - 1][2], schedule[i][2]
        if lv_a is None or lv_b is None or abs(lv_b - lv_a) < 30.0:
            continue
        b_vs = sorted(baseline_tails)
        b_us = [baseline_tails[v] for v in b_vs]
        base_a = float(np.interp(v_a, b_vs, b_us))
        base_b = float(np.interp(v_b, b_vs, b_us))
        if abs(base_b - base_a) < 3.0 * abs(lv_b - lv_a - (base_b - base_a)):
            continue
        t0 = schedule[i][0]
        mask = (times >= t0 - 0.5) & (times <= t0 + 1.5)
        t_win, u_win = times[mask], values[mask]
        mid = 0.5 * (lv_a + lv_b)
        sign = 1.0 if lv_b > lv_a else -1.0
        for k in range(len(t_win) - 1):
            if (
                sign * (u_win[k] - mid) > 0.0
                and sign * (u_win[k + 1] - mid) > 0.0
            ):
                observations.append((t0, t_win[k] - t0))
                break
    if len(observations) < 3:
        return schedule, 0.0, 0.0
    t_obs = np.array([t for t, _ in observations])
    lags = np.array([lag for _, lag in observations])
    t_ref = schedule[0][0]
    coeffs = np.polyfit(t_obs - t_ref, lags, 1)
    drift, lag0 = float(coeffs[0]), float(coeffs[1])
    rectified = [
        (
            t0 + lag0 + drift * (t0 - t_ref),
            t1 + lag0 + drift * (t1 - t_ref),
            vel,
            sgt,
        )
        for t0, t1, vel, sgt in schedule
    ]
    return rectified, lag0, drift


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture")
    parser.add_argument("--baseline")
    parser.add_argument(
        "--baseline-sgt",
        type=int,
        help="sgt the baseline capture was recorded at, for baselines "
        "predating the sgt column (e.g. 8 for the archived v1 baseline)",
    )
    parser.add_argument("--plot")
    parser.add_argument(
        "--anchor",
        help="pin the scale from a trusted linear PA value, as "
        "'<velocity>:<k>' (advance k*v mm at that filament velocity)",
    )
    args = parser.parse_args()

    schedule, times, values, smooth_time = parse_capture(args.capture)
    values, ref_sgt, stitched = stitch_sgt(schedule, times, values)
    if stitched:
        print(
            "sgt-stitched capture: %d blocks mapped into sgt=%d units"
            % (len(sgt_blocks(schedule)), ref_sgt)
        )
    baseline_tails = None
    if args.baseline:
        b_schedule, b_times, b_values, _ = parse_capture(args.baseline)
        b_sgts = {sgt for _, _, _, sgt in b_schedule}
        if len(b_sgts) != 1:
            sys.exit("baseline capture must use a single sgt")
        b_sgt = b_sgts.pop()
        if b_sgt is None:
            b_sgt = args.baseline_sgt
        if ref_sgt is not None:
            if b_sgt is None:
                sys.exit(
                    "capture is sgt-tagged but the baseline's sgt is "
                    "unknown; pass --baseline-sgt"
                )
            if b_sgt != ref_sgt:
                sys.exit(
                    "baseline sgt=%d does not match the capture's "
                    "reference group sgt=%d; capture the first velocity "
                    "group at the baseline's sgt so both share reading "
                    "units" % (b_sgt, ref_sgt)
                )
        baseline_tails = steady_tails(b_schedule, b_times, b_values)
    if stitched:
        rectified = []
        for _, indices in sgt_blocks(schedule):
            block = [schedule[i] for i in indices]
            block, lag0, drift = rectify_schedule(
                block, times, values, baseline_tails
            )
            rectified += block
            if lag0 or drift:
                print(
                    "block at t=%.1fs rectified: lag %.3fs, drift "
                    "%.2f ms/s" % (block[0][0], lag0, drift * 1000.0)
                )
        schedule = rectified
    else:
        schedule, lag0, drift = rectify_schedule(
            schedule, times, values, baseline_tails
        )
        if lag0 or drift:
            print(
                "schedule rectified: lag %.3fs, drift %.2f ms/s"
                % (lag0, drift * 1000.0)
            )
    tails = steady_tails(schedule, times, values)
    if len(tails) < 3:
        sys.exit("need steady tails at >=3 distinct velocities")
    velocities, pressure = pressure_curve(tails, baseline_tails)
    if np.max(pressure) <= 0.0:
        sys.exit(
            "no load signal above baseline; StallGuard SNR insufficient "
            "or baseline mismatched"
        )

    v_cmd = command_velocity(schedule, times, smooth_time)
    moving = v_cmd >= 0.7 * float(velocities[0])
    times, values, v_cmd = times[moving], values[moving], v_cmd[moving]
    base_u = np.asarray(baseline_of_velocity(tails, baseline_tails)(v_cmd))
    v_max = float(velocities[-1])
    fit_mask = np.ones(len(times), dtype=bool)
    half_window = 0.5 * smooth_time + 0.03
    for seg in schedule[1:]:
        fit_mask &= np.abs(times - seg[0]) > half_window

    results = {}
    for name, shape in MODEL_SHAPES.items():
        shape_rms, la_u, off_u, v_lin, drift = fit_steady_shape(
            shape, velocities, pressure
        )
        anchored = None
        if args.anchor:
            v_a, _, k_a = args.anchor.partition(":")
            v_a, k_a = float(v_a), float(k_a)
            u_at = la_u * v_a + off_u * shape(v_a / v_lin)
            if u_at <= 0.0:
                sys.exit("anchor velocity has no measured pressure")
            anchored = k_a * v_a / u_at
        best = None
        for c0 in (0.001, 0.003, 0.01, 0.03, 0.1):
            if anchored is not None:
                c = anchored
                db, rms = fit_scale_fixed_c(
                    shape,
                    (la_u, off_u, v_lin),
                    times,
                    values,
                    v_cmd,
                    base_u,
                    v_max,
                    c,
                    fit_mask,
                )
            else:
                c, db, rms = fit_scale(
                    shape,
                    (la_u, off_u, v_lin),
                    times,
                    values,
                    v_cmd,
                    base_u,
                    v_max,
                    c0,
                    fit_mask,
                )
            if best is None or rms < best["rms"]:
                best = {
                    "linear_advance": c * la_u,
                    "nonlinear_offset": c * off_u,
                    "linearization_velocity": v_lin,
                    "compliance": c,
                    "baseline_offset": db,
                    "rms": rms,
                    "shape_rms": shape_rms,
                    "drift": drift,
                }
            if anchored is not None:
                break
        results[name] = best
        print(
            "\n[%s]  trace rms %.3f SG units\n"
            "  steady-shape rms %.3f units, baseline drift %.2f units, "
            "compliance %.5g mm/unit, offset %.2f\n"
            "linear_advance: %.5f\n"
            "nonlinear_offset: %.5f\n"
            "linearization_velocity: %.3f"
            % (
                name,
                best["rms"],
                best["shape_rms"],
                best["drift"],
                best["compliance"],
                best["baseline_offset"],
                best["linear_advance"],
                best["nonlinear_offset"],
                best["linearization_velocity"],
            )
        )
    winner = min(results, key=lambda k: results[k]["rms"])
    print("\nbest fit: %s" % (winner,))

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax_raw, ax_fit) = plt.subplots(2, 1, figsize=(10, 8))
        ax_raw.plot(times, values, ".", markersize=2, label="measured")
        for name in results:
            fit = results[name]
            shape = MODEL_SHAPES[name]
            la_u = fit["linear_advance"] / fit["compliance"]
            off_u = fit["nonlinear_offset"] / fit["compliance"]
            v_dense = np.linspace(0.0, 1.5 * v_max, 2000)
            u_dense = la_u * v_dense + off_u * shape(
                v_dense / fit["linearization_velocity"]
            )
            p = simulate_pressure(
                u_dense, v_dense, fit["compliance"], times, v_cmd
            )
            pred = base_u + fit["baseline_offset"] - p
            ax_raw.plot(times, pred, linewidth=0.8, label=name)
        ax_raw.set_xlabel("print time (s)")
        ax_raw.set_ylabel("SG load (raw)")
        ax_raw.legend(fontsize=7)

        for name in results:
            fit = results[name]
            shape = MODEL_SHAPES[name]
            v_dense = np.linspace(0.0, v_max * 1.1, 200)
            ax_fit.plot(
                v_dense,
                fit["linear_advance"] * v_dense
                + fit["nonlinear_offset"]
                * shape(v_dense / fit["linearization_velocity"]),
                label="%s (rms %.2f)" % (name, fit["rms"]),
            )
            ax_fit.plot(
                velocities,
                fit["compliance"] * pressure,
                "o",
                markersize=4,
                alpha=0.5,
            )
        ax_fit.set_xlabel("extrusion velocity (mm/s filament)")
        ax_fit.set_ylabel("advance (mm)")
        ax_fit.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=150)
        print("plot written to %s" % (args.plot,))


if __name__ == "__main__":
    main()
