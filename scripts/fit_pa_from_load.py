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
from scipy.optimize import least_squares

SG_MASK = 0x3FF
TAIL_FRACTION = 0.5
SIM_SUBSTEP = 0.005

MODEL_SHAPES = {
    "tanh_pressure_advance": np.tanh,
    "recipr_pressure_advance": lambda u: u / (1.0 + np.abs(u)),
}


def parse_capture(path):
    schedule = []
    times = []
    values = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            kind, rest = line.split(",", 1)
            if kind == "S":
                t0, t1, vel = (float(x) for x in rest.split(","))
                schedule.append((t0, t1, vel))
            elif kind == "D":
                t, raw = rest.split(",")
                times.append(float(t))
                values.append(int(raw) & SG_MASK)
            else:
                raise ValueError("unknown row kind %r in %s" % (kind, path))
    if not schedule or not times:
        raise ValueError("capture %s has no schedule or no samples" % (path,))
    return schedule, np.asarray(times), np.asarray(values, dtype=float)


def steady_tails(schedule, times, values):
    by_velocity = {}
    for t0, t1, vel in schedule:
        tail_start = t0 + TAIL_FRACTION * (t1 - t0)
        mask = (times >= tail_start) & (times <= t1)
        if not mask.any():
            continue
        by_velocity.setdefault(vel, []).append(float(np.median(values[mask])))
    return {v: float(np.mean(tails)) for v, tails in by_velocity.items()}


def pressure_curve(tails, baseline_tails):
    velocities = np.array(sorted(tails))
    if baseline_tails is not None:
        missing = [v for v in velocities if v not in baseline_tails]
        if missing:
            raise ValueError(
                "baseline capture lacks velocities %s" % (missing,)
            )
        pressure = np.array([baseline_tails[v] - tails[v] for v in velocities])
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


def command_velocity(schedule, times):
    starts = np.array([t0 for t0, _, _ in schedule])
    vels = np.array([vel for _, _, vel in schedule])
    idx = np.clip(np.searchsorted(starts, times, side="right") - 1, 0, None)
    return vels[idx]


def simulate_advance(shape, params, times, v_cmd, v_max):
    la, off, v_lin = params
    v_dense = np.linspace(0.0, 1.5 * v_max, 2000)
    a_dense = la * v_dense + off * shape(v_dense / v_lin)
    x = 0.0
    out = np.empty(len(times))
    prev_t = times[0]
    for i in range(len(times)):
        dt = times[i] - prev_t
        if dt > 0.0:
            steps = max(1, int(math.ceil(dt / SIM_SUBSTEP)))
            h = dt / steps
            v = v_cmd[i]
            for _ in range(steps):
                outflow = np.interp(x, a_dense, v_dense)
                x += h * (v - outflow)
                if x < 0.0:
                    x = 0.0
        out[i] = x
        prev_t = times[i]
    return out


def fit_ode(shape, times, values, v_cmd, base_u, v_max, init):
    la0, off0, v_lin0, c0 = init

    def unpack(theta):
        return np.exp(theta[:4]), theta[4]

    def residuals(theta):
        (la, off, v_lin, c), db = unpack(theta)
        x = simulate_advance(shape, (la, off, v_lin), times, v_cmd, v_max)
        return (base_u + db - x / c) - values

    theta0 = np.array(
        [math.log(la0), math.log(off0), math.log(v_lin0), math.log(c0), 0.0]
    )
    result = least_squares(residuals, theta0, method="lm")
    (la, off, v_lin, c), db = unpack(result.x)
    rms = float(np.sqrt(np.mean(result.fun**2)))
    return {
        "linear_advance": la,
        "nonlinear_offset": off,
        "linearization_velocity": v_lin,
        "compliance": c,
        "baseline_offset": db,
        "rms": rms,
        "theta": result.x,
    }


def initial_guess(shape, velocities, pressure):
    v_lin_grid = np.logspace(
        math.log10(velocities[0] / 4.0),
        math.log10(velocities[-1] * 4.0),
        100,
    )
    best = None
    for v_lin in v_lin_grid:
        basis = np.column_stack([velocities, shape(velocities / v_lin)])
        coeffs, _, _, _ = np.linalg.lstsq(basis, pressure, rcond=None)
        coeffs = np.maximum(coeffs, 1e-9)
        rms = float(np.sqrt(np.mean((basis @ coeffs - pressure) ** 2)))
        if best is None or rms < best[0]:
            best = (rms, float(coeffs[0]), float(coeffs[1]), float(v_lin))
    assert best is not None
    _, la_units, off_units, v_lin = best
    return la_units, off_units, v_lin


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture")
    parser.add_argument("--baseline")
    parser.add_argument("--plot")
    args = parser.parse_args()

    schedule, times, values = parse_capture(args.capture)
    tails = steady_tails(schedule, times, values)
    if len(tails) < 3:
        sys.exit("need steady tails at >=3 distinct velocities")
    baseline_tails = None
    if args.baseline:
        baseline_tails = steady_tails(*parse_capture(args.baseline))
    velocities, pressure = pressure_curve(tails, baseline_tails)
    if np.max(pressure) <= 0.0:
        sys.exit(
            "no load signal above baseline; StallGuard SNR insufficient "
            "or baseline mismatched"
        )

    v_cmd = command_velocity(schedule, times)
    base_u = np.asarray(baseline_of_velocity(tails, baseline_tails)(v_cmd))
    v_max = float(velocities[-1])

    results = {}
    for name, shape in MODEL_SHAPES.items():
        la_u, off_u, v_lin0 = initial_guess(shape, velocities, pressure)
        best = None
        for c0 in (0.001, 0.003, 0.01, 0.03):
            init = (max(la_u * c0, 1e-6), max(off_u * c0, 1e-6), v_lin0, c0)
            fit = fit_ode(shape, times, values, v_cmd, base_u, v_max, init)
            if best is None or fit["rms"] < best["rms"]:
                best = fit
        results[name] = best
        print(
            "\n[%s]  rms %.3f SG units  (compliance %.5g mm/unit, "
            "baseline offset %.2f)\n"
            "linear_advance: %.5f\n"
            "nonlinear_offset: %.5f\n"
            "linearization_velocity: %.3f"
            % (
                name,
                best["rms"],
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
            x = simulate_advance(
                shape,
                (
                    fit["linear_advance"],
                    fit["nonlinear_offset"],
                    fit["linearization_velocity"],
                ),
                times,
                v_cmd,
                v_max,
            )
            pred = base_u + fit["baseline_offset"] - x / fit["compliance"]
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
