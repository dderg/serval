"""Phase-1 end-to-end: printer_test gcodes through klippy, then the
virtual printer, per parts set: nozzle residual vibration and lost steps.

  PYTHONPATH=. uv run python -m tools.printer_sim.report [--files ...]
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import json
import math
import pathlib

import numpy as np

from tools.motion_eval import accel_residual

from . import drive as drive_mod
from . import mechanics, motor, parts, sim

PRINTER_TEST = (
    pathlib.Path(accel_residual.__file__).resolve().parent / "printer_test"
)
FILES = (
    "corners_scv50",
    "corners_bounded81000",
    "stress_scv50",
    "stress_floor",
    "stress_bounded",
    "stress_bounded81000",
)
FINAL_WINDOW_S = 0.05
UM = 1e6
MM = 1e3


@dataclasses.dataclass(frozen=True)
class Case:
    reading: str
    parts: str
    mechanics_from: str | None = None


CASES = (
    Case("A", "voron_48v_a"),
    Case("A", "voron_stock_24v_a", mechanics_from="voron_48v_a"),
    Case("B", "voron_48v_b"),
    Case("B", "voron_stock_24v_b"),
)


def _load(name):
    return parts.load(
        parts.PARTS_DIR / (name + ".cfg"), accel_residual.DEFAULT_CONFIG
    )


def build_case(case: Case):
    """(model, notes): the case's own calibration, or, where its parts
    cannot reach the measured resonances, mechanics_from's calibrated
    mechanics driven by this case's motors and driver."""
    p = _load(case.parts)
    e = motor.Electrical.from_parts(p)
    notes = {}
    try:
        cal = mechanics.calibrate(p, e.magnetic_stiffness)
        mech_parts = p
    except mechanics.CalibrationError as err:
        if case.mechanics_from is None:
            raise
        notes["own_calibration"] = str(err)
        mech_parts = _load(case.mechanics_from)
        cal = mechanics.calibrate(
            mech_parts,
            motor.Electrical.from_parts(mech_parts).magnetic_stiffness,
        )
        notes["mechanics_from"] = case.mechanics_from
    system = mechanics.build(mech_parts, cal, e.magnetic_stiffness)
    notes["calibrated"] = dataclasses.asdict(cal)
    notes["modes_hz"] = [
        (float(f), mechanics.damping_ratio(system, f))
        for f in mechanics.modes(system)
    ]
    notes["response_peaks"] = {
        axis: mechanics.response_peaks(system, axis) for axis in "xy"
    }
    return sim.Model(system, e), notes


def metrics(model: sim.Model, d: drive_mod.Drive, res: sim.Result):
    err = res.nozzle * MM - d.reference
    t = res.t
    window_amp = []
    for start, end in d.windows:
        inside = (t >= start) & (t <= end)
        if inside.any():
            window_amp.append(np.abs(err[inside]).max(axis=0))
    window_amp = np.array(window_amp) * (UM / MM)
    final = err[t >= t[-1] - FINAL_WINDOW_S].mean(axis=0)
    e = model.electrical
    return {
        "residual_um": {
            axis: {
                "p50": float(np.percentile(window_amp[:, i], 50)),
                "p99": float(np.percentile(window_amp[:, i], 99)),
                "max": float(window_amp[:, i].max()),
            }
            for i, axis in enumerate("xy")
        },
        "windows": len(window_amp),
        "max_error_um": {
            axis: float(np.abs(err[:, i]).max() * UM / MM)
            for i, axis in enumerate("xy")
        },
        "final_offset_mm": {
            axis: float(final[i]) for i, axis in enumerate("xy")
        },
        "motors": {
            name: {
                "max_load_angle_deg": float(
                    math.degrees(res.delta_absmax[:, m].max())
                ),
                "pole_slips": int(sim.pole_slips(res.delta[:, m : m + 1])[0]),
                "peak_torque_fraction": float(
                    np.nanmax(np.abs(res.torque[:, m])) / e.peak_torque
                ),
                "min_current_fraction": float(
                    np.nanmin(res.current[:, m]) / e.current_amplitude
                ),
            }
            for m, name in enumerate(("a", "b"))
        },
    }


def _drive(name, pole_pairs):
    text = (PRINTER_TEST / (name + ".gcode")).read_text()
    return name, drive_mod.from_gcode(
        text, accel_residual.DEFAULT_CONFIG, pole_pairs
    )


def _simulate(args):
    case, d, continuous = args
    model, _ = build_case(case)
    command = d.continuous_command if continuous else d.step_command
    res = sim.run(model, d.t0, drive_mod.DT, d.n, command, d.command_rate)
    return metrics(model, d, res)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--files", nargs="*", default=list(FILES))
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument(
        "--continuous",
        action="store_true",
        help="drive the motors with the executed motion instead of the step stream",
    )
    ap.add_argument(
        "--out",
        type=pathlib.Path,
        default=parts.HERE / ".build" / "report.json",
    )
    args = ap.parse_args(argv)
    pole_pairs = {_load(c.parts).motor.pole_pairs for c in CASES}
    if len(pole_pairs) != 1:
        raise SystemExit("cases mix motor step angles")
    pp = pole_pairs.pop()
    with concurrent.futures.ProcessPoolExecutor(args.jobs) as pool:
        drives = dict(pool.map(_drive, args.files, [pp] * len(args.files)))
        jobs = [
            (c, drives[f], args.continuous) for c in CASES for f in args.files
        ]
        results = list(pool.map(_simulate, jobs))
    out = {
        "cases": {
            c.parts: {"reading": c.reading, **build_case(c)[1]} for c in CASES
        },
        "runs": [
            {"parts": c.parts, "file": f, **r}
            for (c, _, _), f, r in zip(
                jobs, [f for _ in CASES for f in args.files], results
            )
        ],
    }
    args.out.parent.mkdir(exist_ok=True)
    args.out.write_text(json.dumps(out, indent=1))
    print(table(out))


def table(out):
    lines = [
        "| reading | parts | file | residual X p50/p99/max um | residual Y p50/p99/max um"
        " | max error X/Y um | load angle A/B deg | slips A/B | final offset X/Y mm |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in out["runs"]:
        c = out["cases"][r["parts"]]
        rx, ry = r["residual_um"]["x"], r["residual_um"]["y"]
        ma, mb = r["motors"]["a"], r["motors"]["b"]
        lines.append(
            "| %s | %s | %s | %.1f / %.1f / %.1f | %.1f / %.1f / %.1f | %.0f / %.0f"
            " | %.0f / %.0f | %d / %d | %.3f / %.3f |"
            % (
                c["reading"],
                r["parts"],
                r["file"],
                rx["p50"],
                rx["p99"],
                rx["max"],
                ry["p50"],
                ry["p99"],
                ry["max"],
                r["max_error_um"]["x"],
                r["max_error_um"]["y"],
                ma["max_load_angle_deg"],
                mb["max_load_angle_deg"],
                ma["pole_slips"],
                mb["pole_slips"],
                r["final_offset_mm"]["x"],
                r["final_offset_mm"]["y"],
            )
        )
    return "\n".join(lines)


if __name__ == "__main__":
    main()
