"""Owner operating point: smooth_mzv at X 200 Hz / Y 130 Hz, 50 000 mm/s^2.

    python3 ~/Developer/serval/scripts/slots.py run -- \
        uv run python -m tools.motion_eval.owner_point [--jobs 12]

Runs the corpus at square_corner_velocity SCVS and with [corner_deviation]
in floor mode, on owner.cfg (max_velocity 800) and its 500 / 1200 mm/s
variants, and reports print time, the executed (shaped) peak acceleration
and jerk, corner deviation against the shaper floor, and the residual
vibration of damped oscillators tuned around and above the shaper
frequencies. Writes DIR/results.jsonl and DIR/tables.md.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import json
import pathlib
import sys
import tempfile

import numpy as np

from . import evaluate, oscillator, run, sweep

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_OUT = HERE / ".build" / "owner_point"
CONFIGS = {
    "owner": HERE / "configs" / "owner.cfg",
    "owner_v500": HERE / "configs" / "owner_v500.cfg",
    "owner_v1200": HERE / "configs" / "owner_v1200.cfg",
}
MAIN_CONFIG = "owner"
FAMILY = "smooth_mzv"
FREQ = {"x": 200.0, "y": 130.0}
MCR = 0.5
SCVS = (5.0, 10.0, 20.0, 35.0, 50.0, 80.0)
REFERENCE_SCV = 50.0
MODE_RATIOS = (0.8, 0.9, 1.0, 1.1, 1.2, 2.5, 4.0)
ZETAS = (0.05, 0.1)
DT = 25e-6


@dataclasses.dataclass(frozen=True)
class Run:
    case: str
    config: str
    scv: float | None
    shaped: bool = True

    @property
    def setting(self):
        if self.scv is None:
            return "corner_deviation"
        return "scv %g" % self.scv + ("" if self.shaped else " unshaped")


def plan(cases):
    runs = [
        Run(case, config, scv)
        for config in CONFIGS
        for case in cases
        for scv in SCVS + (None,)
    ]
    runs += [Run(case, MAIN_CONFIG, REFERENCE_SCV, False) for case in cases]
    return runs


def _config(run_, tmpdir):
    base = CONFIGS[run_.config]
    if run_.scv is not None:
        return base
    path = pathlib.Path(tmpdir) / "printer.cfg"
    path.write_text("[include %s]\n\n[corner_deviation]\n" % (base,))
    return path


def _window_peaks(values, t0, dt, centers, reach):
    """Largest |value| within reach of each center."""
    n = len(values)
    lo = np.clip(np.floor((centers - reach - t0) / dt).astype(int), 0, n - 1)
    hi = np.clip(np.ceil((centers + reach - t0) / dt).astype(int) + 1, 1, n)
    edges = np.stack([lo, hi], axis=-1).ravel()
    return np.maximum.reduceat(np.abs(values), edges)[::2]


def _summary(values):
    if not len(values):
        return {"count": 0, "p50": 0.0, "p99": 0.0, "max": 0.0}
    return {
        "count": int(len(values)),
        "p50": float(np.percentile(values, 50)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def execute(run_):
    text = (sweep.CORPUS / (run_.case + ".gcode")).read_text()
    limits = "MINIMUM_CRUISE_RATIO=%r" % (MCR,)
    if run_.scv is not None:
        limits += " SQUARE_CORNER_VELOCITY=%r" % (run_.scv,)
    freq = FREQ if run_.shaped else {"x": 0.0, "y": 0.0}
    prefix = (
        "SET_INPUT_SHAPER SHAPER_TYPE=%s SHAPER_FREQ_X=%r SHAPER_FREQ_Y=%r\n"
        "SET_VELOCITY_LIMIT %s\n" % (FAMILY, freq["x"], freq["y"], limits)
    )
    with tempfile.TemporaryDirectory(prefix="motion_eval_owner_") as tmp:
        raw = run.run_klippy(
            _config(run_, tmp), prefix + text, sweep.case_start(text)
        )
    cap = raw.capture
    std = evaluate.analyze(raw, dt=DT, sample_dt=None)
    seg, kernels, t0, n, _, motion = evaluate.executed_motion(cap, DT)
    max_accel = cap["settings"]["max_accel"]
    vertices = seg.vertices()["t_arrive"]
    windows = oscillator.constant_velocity_windows(
        seg, evaluate.kernel_support(kernels)
    )
    axes = {}
    for i, axis in enumerate("xy"):
        kernel = kernels[axis]
        reach = kernel.half_width + abs(kernel.t_offs)
        acc = motion[axis].acc
        jerk = np.diff(acc) / DT
        variance = kernel.second_moment()
        residual = {}
        for ratio in MODE_RATIOS:
            freq = ratio * FREQ[axis]
            for zeta in ZETAS:
                e = oscillator.lag(motion[axis].pos, DT, freq, zeta)
                residual["%g/%g" % (ratio, zeta)] = _summary(
                    oscillator.residuals(e, t0, DT, windows, freq)
                )
        axes[axis] = {
            "floor_mm": 0.5 * variance * max_accel,
            "smooth_time_s": 2.0 * kernel.half_width,
            "peak_accel": float(np.abs(acc).max()),
            "corner_peak_accel": _summary(
                _window_peaks(acc, t0, DT, vertices, reach)
            ),
            "peak_jerk": float(np.abs(jerk).max()),
            "corner_peak_jerk": _summary(
                _window_peaks(jerk, t0 + 0.5 * DT, DT, vertices, reach)
            ),
            "residual": residual,
            "commanded_max_velocity_jump": std["commanded"]["per_axis"][axis][
                "max_velocity_jump_mm_s"
            ],
            "executed_max_velocity_jump": std["executed"]["per_axis"][axis][
                "max_velocity_jump_mm_s"
            ],
        }
    return dict(
        dataclasses.asdict(run_),
        setting=run_.setting,
        print_time_s=std["print_time_s"],
        max_accel=max_accel,
        corners=std["executed"]["corners"],
        windows=len(windows),
        axes=axes,
        max_step_model_error_share=max(
            s["model_vs_steps"]["max_error_mm"]
            / s["model_vs_steps"]["max_allowed_mm"]
            for s in std["steppers"].values()
            if s["model_vs_steps"]["steps"]
        ),
    )


def run_all(runs, jobs):
    records = []
    with concurrent.futures.ProcessPoolExecutor(jobs) as pool:
        futures = {pool.submit(execute, r): r for r in runs}
        for i, fut in enumerate(concurrent.futures.as_completed(futures), 1):
            try:
                records.append(fut.result())
            except Exception as e:
                raise RuntimeError("run failed: %r" % (futures[fut],)) from e
            if i % 25 == 0 or i == len(runs):
                print("%d/%d runs" % (i, len(runs)), file=sys.stderr)
    return records


def _settings(config=MAIN_CONFIG):
    names = ["scv %g" % s for s in SCVS] + ["corner_deviation"]
    if config == MAIN_CONFIG:
        names.append("scv %g unshaped" % REFERENCE_SCV)
    return names


def _by(records, config):
    out = {}
    for r in records:
        if r["config"] == config:
            out.setdefault(r["setting"], {})[r["case"]] = r
    return out


def _pct(reference, value):
    return "%+.1f%%" % (100.0 * (value - reference) / reference)


def summary_table(records, cases, config):
    groups = _by(records, config)
    ref = sum(
        groups["scv %g" % REFERENCE_SCV][c]["print_time_s"] for c in cases
    )
    lines = [
        "| setting | corpus s | vs scv %g | peak accel / max_accel (x / y) |"
        " corner peak accel p99 / max_accel | peak jerk km/s^3 (x / y) |"
        " corner dev p99 / max mm | max / floor |" % REFERENCE_SCV,
        "|" + "---|" * 8,
    ]
    for name in _settings():
        g = groups[name]
        total = sum(g[c]["print_time_s"] for c in cases)
        rs = [g[c] for c in cases]
        ma = rs[0]["max_accel"]
        floor = max(rs[0]["axes"][a]["floor_mm"] for a in "xy")
        peak = {
            a: max(r["axes"][a]["peak_accel"] for r in rs) / ma for a in "xy"
        }
        corner_p99 = max(
            r["axes"][a]["corner_peak_accel"]["p99"] for r in rs for a in "xy"
        )
        jerk = {
            a: max(r["axes"][a]["peak_jerk"] for r in rs) / 1e6 for a in "xy"
        }
        dev_p99 = max(r["corners"]["p99_mm"] for r in rs)
        dev_max = max(r["corners"]["max_mm"] for r in rs)
        lines.append(
            "| %s | %.1f | %s | %.3f / %.3f | %.3f | %.0f / %.0f | %.3f / %.3f |"
            " %s |"
            % (
                name,
                total,
                _pct(ref, total),
                peak["x"],
                peak["y"],
                corner_p99 / ma,
                jerk["x"],
                jerk["y"],
                dev_p99,
                dev_max,
                "%.2f" % (dev_max / floor) if floor else "no shaper",
            )
        )
    return "\n".join(lines)


def residual_table(records, cases, config, zeta, stat):
    groups = _by(records, config)
    lines = [
        "| setting | "
        + " | ".join("%g x f" % r for r in MODE_RATIOS)
        + " |  (worst axis, um)",
        "|" + "---|" * (len(MODE_RATIOS) + 1),
    ]
    for name in _settings():
        rs = [groups[name][c] for c in cases]
        cells = []
        for ratio in MODE_RATIOS:
            key = "%g/%g" % (ratio, zeta)
            if stat == "max":
                value = max(
                    r["axes"][a]["residual"][key]["max"]
                    for r in rs
                    for a in "xy"
                )
            else:
                value = max(
                    r["axes"][a]["residual"][key][stat]
                    for r in rs
                    for a in "xy"
                )
            cells.append("%.2f" % (1000.0 * value))
        lines.append("| %s | %s |" % (name, " | ".join(cells)))
    return "\n".join(lines)


def per_case_table(records, cases, config):
    groups = _by(records, config)
    names = _settings()
    lines = [
        "| case | " + " | ".join("%s s (peak a)" % n for n in names) + " |",
        "|" + "---|" * (len(names) + 1),
    ]
    for case in cases:
        cells = []
        for name in names:
            r = groups[name][case]
            peak = (
                max(r["axes"][a]["peak_accel"] for a in "xy") / r["max_accel"]
            )
            cells.append("%.2f (%.2f)" % (r["print_time_s"], peak))
        lines.append("| %s | %s |" % (case, " | ".join(cells)))
    return "\n".join(lines)


def velocity_table(records, cases):
    names = _settings(config=None)
    lines = [
        "| config | " + " | ".join(names) + " |",
        "|" + "---|" * (len(names) + 1),
    ]
    for config in CONFIGS:
        groups = _by(records, config)
        ref = sum(
            groups["scv %g" % REFERENCE_SCV][c]["print_time_s"] for c in cases
        )
        cells = []
        for name in names:
            g = groups[name]
            total = sum(g[c]["print_time_s"] for c in cases)
            peak = max(
                g[c]["axes"][a]["peak_accel"] / g[c]["max_accel"]
                for c in cases
                for a in "xy"
            )
            cells.append("%s (%.2f)" % (_pct(ref, total), peak))
        lines.append("| %s | %s |" % (config, " | ".join(cells)))
    return "\n".join(lines)


def tables(records, cases):
    out = [
        "## Summary, %s (smooth_mzv 200/130 Hz, 50 000 mm/s^2, mcr 0.5)"
        % MAIN_CONFIG,
        "",
        summary_table(records, cases, MAIN_CONFIG),
    ]
    for zeta in ZETAS:
        for stat in ("max", "p99"):
            out += [
                "",
                "## Residual vibration after corners, zeta %g, %s over"
                " constant-velocity windows (um)" % (zeta, stat),
                "",
                residual_table(records, cases, MAIN_CONFIG, zeta, stat),
            ]
    out += [
        "",
        "## Per case: print time s (peak accel / max_accel)",
        "",
        per_case_table(records, cases, MAIN_CONFIG),
        "",
        "## max_velocity 500 / 800 / 1200: corpus time vs scv %g"
        " (peak accel / max_accel)" % REFERENCE_SCV,
        "",
        velocity_table(records, cases),
    ]
    return "\n".join(out) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.motion_eval.owner_point")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    ap.add_argument("--tables-only", action="store_true")
    args = ap.parse_args(argv)
    cases = sweep.corpus_cases()
    args.out.mkdir(parents=True, exist_ok=True)
    results = args.out / "results.jsonl"
    if args.tables_only:
        records = [
            json.loads(line) for line in results.read_text().splitlines()
        ]
    else:
        records = run_all(plan(cases), args.jobs)
        results.write_text("".join(json.dumps(r) + "\n" for r in records))
    text = tables(records, cases)
    (args.out / "tables.md").write_text(text)
    sys.stdout.write(text)


if __name__ == "__main__":
    main()
