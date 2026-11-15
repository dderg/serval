"""Timing, per-feature split, judge and health of the cube session.

    uv run python -m tools.kv2_bench.cube_report plans PLANS_DIR
    uv run python -m tools.kv2_bench.cube_report session SESSION_DIR PLANS_DIR

The span is the real-printer comparison's: from the end of the line before
the first extrusion of layer 1 to the end of original-file line 39601
(layer 86, Z 17.2), mapped to the `_cd` file through the common line index
(voron0/ab/print_compare_common.py). Each run is timed four ways:
- plan: the offline batch plan of the same setting (cube.py plan);
- stats: print_compare.py's method on the bench klippy.log (1 Hz Stats,
  sd_pos mapped to file lines, plan-shaped between samples), print_time and
  wall clock;
- trapq: the live toolhead trapq (motion_report tap) mapped to file lines;
- capture: F407 timestamps of the first and last XY motor step of the span.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from . import compare, firmware, replay
from .cube import AB, SETTINGS, STOCK
from .headroom import fatal_lines
from .reference import Reference, Steps
from .report import peak_rates
from .trapq_diff import live_phases

sys.path.insert(0, str(AB))
import print_compare as pc  # noqa: E402
from print_compare_common import (  # noqa: E402
    common_index,
    parse_gcode,
    parse_stats_line,
    plan_lines,
)

FEATURES = ("outer wall", "inner wall", "infill", "travel", "other")
END_LINE = 39601
END_TEXT = "G1 X56.447 Y74.37 E.27151"
FIRST_TRAVEL = "G1 X59.854 Y68.599 F18000"
SMOOTHING_HALF_S = 0.006
XY_IDLE_BEFORE_START_S = 0.010
XY_MOTORS = ("stepper_x", "stepper_y")
PRINT_START_END = (60.0, 0.3, 0.28)
AXES_TOL = 1e-6
POS_TOL = 1e-5
MAX_SKIPPED_LINES = 6
REVERSAL_CONTEXT = 50
ERROR_MARKERS = ("Traceback", "Internal error", "!! ", "Unknown command")
REAL_PLANS = AB / "print-compare-plans"
REAL = {
    "print 1 (stock, old host)": (
        "stock",
        REAL_PLANS / "plan_stock_oldhost.json",
        [(pc.LOG1, 9676)],
    ),
    "print 2 (_cd, corner 21000)": (
        "cd",
        REAL_PLANS / "plan_cd21000.json",
        [(pc.LOG2A, 20602), (pc.LOG2B, 1299)],
    ),
}


class Gcode:
    def __init__(self, path: Path, stock: Gcode | None = None):
        self.lines = parse_gcode(path)
        self.ends = np.array([r["end"] for r in self.lines])
        self.first = next(
            i for i, r in enumerate(self.lines) if r["text"] == FIRST_TRAVEL
        )
        self.feature = np.array([feature_index(r) for r in self.lines])
        if stock is None:
            first_extrusion = next(
                i
                for i in range(self.first, len(self.lines))
                if self.lines[i]["words"].get("E", 0) > 0
                and "X" in self.lines[i]["words"]
            )
            self.start, self.end = first_extrusion - 1, END_LINE - 1
            if self.lines[self.end]["text"] != END_TEXT:
                raise ValueError(
                    f"line {END_LINE} is {self.lines[self.end]['text']!r}"
                )
        else:
            mine = {
                c: i
                for i, c in enumerate(common_index(self.lines))
                if c is not None
            }
            theirs = common_index(stock.lines)
            self.start, self.end = (
                mine[theirs[stock.start]],
                mine[theirs[stock.end]],
            )
            if self.lines[self.end]["text"] != END_TEXT:
                raise ValueError("end marker maps to another line")


def feature_index(rec: dict) -> int:
    group = rec.get("group")
    return (
        FEATURES.index(group)
        if group in FEATURES[:4]
        else FEATURES.index("other")
    )


def split(g: Gcode, durations: np.ndarray) -> dict:
    sl = slice(g.start + 1, g.end + 1)
    by = np.bincount(
        g.feature[sl], weights=durations[sl], minlength=len(FEATURES)
    )
    return {name: float(v) for name, v in zip(FEATURES, by)}


def phases_of_entries(entries: list) -> list[dict]:
    """trapq_append records (trapq_runner) as the accel/cruise/decel phases
    the live motion_report tap delivers."""
    out = []
    for pt, at, ct, dt, sx, sy, sz, rx, ry, rz, sv, cv, a in entries:
        d_accel = sv * at + 0.5 * a * at * at
        d_cruise = cv * ct
        t, dist = pt, 0.0
        for dur, v0, acc, d in (
            (at, sv, a, d_accel),
            (ct, cv, 0.0, d_cruise),
            (dt, cv, -a, 0.0),
        ):
            if dur > 1e-12:
                out.append(
                    {
                        "t": t,
                        "dur": dur,
                        "v0": v0,
                        "a": acc,
                        "start": [
                            sx + rx * dist,
                            sy + ry * dist,
                            sz + rz * dist,
                        ],
                        "axes": [rx, ry, rz],
                    }
                )
            t += dur
            dist += d
    return out


def segments(g: Gcode) -> list[tuple[int, np.ndarray, np.ndarray, float]]:
    """(line, start, unit direction, length) of every XYZ line from the
    first travel on that moves the toolhead."""
    pos = np.array(PRINT_START_END)
    out = []
    for i in range(g.first, len(g.lines)):
        rec = g.lines[i]
        if rec["kind"] != "xyz":
            continue
        new = np.array(
            [rec["words"].get(axis, pos[n]) for n, axis in enumerate("XYZ")]
        )
        length = float(np.linalg.norm(new - pos))
        if length > 1e-9:
            out.append((i, pos, (new - pos) / length, length))
        pos = new
    return out


def on_segment(p: dict, seg: tuple) -> bool:
    _, start, direction, length = seg
    if np.abs(np.array(p["axes"]) - direction).max() > AXES_TOL:
        return False
    rel = np.array(p["start"]) - start
    along = float(rel @ direction)
    return (
        -POS_TOL <= along < length - 1e-9
        and np.abs(rel - along * direction).max() <= POS_TOL
    )


def line_durations(g: Gcode, phases: list[dict]) -> tuple[np.ndarray, float]:
    """Per file line: the time of the trapq phases lying on its segment, plus
    the idle gap before them split over the E-only/dwell lines in between (as
    print_compare_common.plan_lines does with whole moves)."""
    segs = segments(g)
    k = next(k for k, p in enumerate(phases) if on_segment(p, segs[0]))
    origin = phases[k]["t"]
    durations = np.zeros(len(g.lines))
    s, matched_s, prev_line, prev_t1 = 0, 0, None, None
    for p in phases[k:]:
        while s < len(segs) and not on_segment(p, segs[s]):
            s += 1
            if s - matched_s > MAX_SKIPPED_LINES and s < len(segs):
                raise ValueError(
                    f"trapq phase at {p['t']:.6f} lies on no line after {segs[matched_s][0] + 1}"
                )
        if s == len(segs):
            break
        line = segs[s][0]
        gap = p["t"] - prev_t1 if prev_t1 is not None else 0.0
        if gap > 1e-9:
            mids = [
                i
                for i in range(prev_line + 1, line)
                if g.lines[i]["kind"] in ("e", "dwell")
            ] or [line]
            for i in mids:
                durations[i] += gap / len(mids)
        durations[line] += p["dur"]
        matched_s, prev_line, prev_t1 = s, line, p["t"] + p["dur"]
    return durations, origin


def timing_of_durations(g: Gcode, durations: np.ndarray) -> dict:
    return {
        "span_s": float(durations[g.start + 1 : g.end + 1].sum()),
        "features": split(g, durations),
    }


def plan_timing(g: Gcode, plan: dict) -> dict:
    lines = plan_lines(g.lines, plan, g.first)
    th = np.array(plan["toolhead"])
    ends = th[:, 0] + th[:, 1] + th[:, 2] + th[:, 3]
    return {
        **timing_of_durations(g, lines["D"]),
        "whole_s": float(ends.max() - th[:, 0].min()),
        "unmatched_moves": len(lines["unmatched"]),
        "_lines": lines,
    }


def regress(g: Gcode, rows: list, samples, sample_lines, durations) -> dict:
    """print_compare.regress with travel as its own group."""
    pt = np.array([pc.find(rows[i], "print_time") for i in samples])
    x, y = [], []
    for k in range(len(samples) - 1):
        a, b = sample_lines[k], sample_lines[k + 1]
        if b <= a or a < g.start or b > g.end:
            continue
        x.append(
            np.bincount(
                g.feature[a + 1 : b + 1],
                weights=durations[a + 1 : b + 1],
                minlength=len(FEATURES),
            )
        )
        y.append(pt[k + 1] - pt[k])
    x, y = np.array(x), np.array(y)
    beta = np.linalg.lstsq(x, y, rcond=None)[0]
    r = y - x @ beta
    cov = (r @ r / (len(y) - len(FEATURES))) * np.linalg.inv(x.T @ x)
    planned = split(g, durations)
    se = np.sqrt(np.diag(cov))
    return {
        "windows": len(y),
        "window_rms_s": float(np.sqrt(r @ r / len(y))),
        "features": {n: float(b * planned[n]) for n, b in zip(FEATURES, beta)},
        "features_se": {n: float(s * planned[n]) for n, s in zip(FEATURES, se)},
    }


def stats_rows(parts: list[tuple[Path, int]]) -> list[dict]:
    rows = []
    for path, skip in parts:
        for line in Path(path).read_text().split("\n")[skip:]:
            row = parse_stats_line(line)
            if row:
                rows.append(row)
    return rows


def stats_timing(g: Gcode, rows: list[dict], plan_lines_: dict) -> dict:
    samples, sample_lines = pc.printing_samples(rows, g.lines, g.ends)
    out = {"samples": len(samples)}
    for clock in ("pt", "wall"):
        f = pc.timeline(rows, samples, sample_lines, plan_lines_, clock)
        out[f"span_{clock}_s"] = float(f(g.end) - f(g.start))
    out["regression"] = regress(
        g, rows, samples, sample_lines, plan_lines_["D"]
    )
    return out


def capture_timing(
    g: Gcode,
    run_dir: Path,
    t_start: float,
    t_end: float,
    whole: tuple[float, float],
) -> dict:
    """F407 time between the first XY motor step of the span (XY idle for
    XY_IDLE_BEFORE_START_S before it: travel, Z hop and unretract) and the
    last XY step at or before the end marker. Steps pair by index with the
    replay of the live trapq (the judge checks the sequences are equal)."""
    ref = Reference.load(run_dir / "replay")
    cap = compare.load_capture(run_dir)
    freq = ref.clock_freq
    firsts, lasts = [], []
    for name in XY_MOTORS:
        clocks = ref.steppers[name].clocks
        i = int(np.searchsorted(clocks, (t_start - SMOOTHING_HALF_S) * freq))
        if i and clocks[i] - clocks[i - 1] < XY_IDLE_BEFORE_START_S * freq:
            raise ValueError(
                f"{name}: no XY idle before the start marker step {i}"
            )
        firsts.append((int(clocks[i]), name, i))
        j = int(np.searchsorted(clocks, t_end * freq, side="right")) - 1
        lasts.append((int(clocks[j]), name, j))
    (c0, n0, i0), (c1, n1, i1) = min(firsts), max(lasts)
    ticks = compare.CAPTURE_TICK_HZ
    all_cap = [s.clocks for s in cap.values() if len(s.clocks)]
    all_ref = [s.clocks for s in ref.steppers.values() if len(s.clocks)]
    return {
        "span_s": float((cap[n1].clocks[i1] - cap[n0].clocks[i0]) / ticks),
        "span_replay_s": float((c1 - c0) / freq),
        "first_step_after_marker_ms": float((c0 / freq - t_start) * 1e3),
        "last_step_before_marker_ms": float((t_end - c1 / freq) * 1e3),
        "whole_s": float(
            (max(c[-1] for c in all_cap) - min(c[0] for c in all_cap)) / ticks
        ),
        "whole_replay_s": float(
            (max(c[-1] for c in all_ref) - min(c[0] for c in all_ref)) / freq
        ),
        "whole_trapq_s": whole[1] - whole[0],
    }


def health(g: Gcode, run_dir: Path, rows: list[dict]) -> dict:
    """Errors over the whole log; Stats over the span (first extrusion to
    line 39601), as the real-printer comparison reports them."""
    log = (run_dir / "klippy.log").read_text()
    printing = [r for r in rows if pc.find(r, "sd_pos") is not None]

    def peak(key: str, fn=max) -> float:
        return fn(pc.find(r, key) for r in span)

    run = json.loads((run_dir / "run.json").read_text())
    lines = log.splitlines()
    return {
        "print_state": run.get("print_state"),
        "failure": run.get("failure"),
        "errors": fatal_lines(log)
        + [line for line in lines if any(k in line for k in ERROR_MARKERS)],
        "scv_ignored_warnings": log.count("SQUARE_CORNER_VELOCITY is ignored"),
        "accel_to_decel_warnings": log.count("ACCEL_TO_DECEL is ignored"),
        "toolhead_limit_lines": sum(
            1
            for line in log.splitlines()
            if line.startswith("toolhead: max_velocity")
        ),
        "span_samples": len(span),
        "buffer_time_min_s": peak("buffer_time", min),
        "print_stall_max": peak("print_stall"),
        "mcu_awake_max": peak("mcu_awake"),
        "mcu_task_avg_max": peak("mcu_task_avg"),
        "corner_table": [
            line
            for line in log.splitlines()
            if "corner_deviation: table ready" in line
        ],
    }


def judge(run_dir: Path, kv2: str, dictionary: Path) -> dict:
    out = run_dir / "replay" / "replay_judgement.json"
    if not out.exists():
        firmware.checkout(firmware.FW_WORKTREE, kv2)
        reference = replay.build(
            run_dir, firmware.FW_WORKTREE, dictionary, run_dir / "replay"
        )
        out.write_text(json.dumps(replay.judge(run_dir, reference), indent=2))
    return json.loads(out.read_text())


def gcodes() -> dict[str, Gcode]:
    stock = Gcode(STOCK)
    return {
        str(STOCK): stock,
        str(SETTINGS["D"].gcode): Gcode(SETTINGS["D"].gcode, stock),
    }


def plan_diff(a: dict, b: dict) -> dict:
    out = {}
    for key in ("toolhead", "extruder"):
        x, y = np.array(a[key]), np.array(b[key])
        if x.shape != y.shape:
            out[key] = {"entries": [len(x), len(y)], "identical": False}
            continue
        diff = np.abs(x - y).max(axis=1)
        bad = np.nonzero(diff)[0]
        out[key] = {
            "entries": len(x),
            "identical_entries": int(len(x) - len(bad)),
            "differing": [
                {"index": int(i), "print_time": float(x[i, 0]), "max_abs_diff": float(diff[i]),
                 "fields": [int(f) for f in np.nonzero(x[i] != y[i])[0]]}
                for i in bad
            ],
        }  # fmt: skip
    return out


def plans_report(plans: Path) -> dict:
    gs = gcodes()
    out: dict = {"settings": {}, "real": {}}
    for name, setting in SETTINGS.items():
        g = gs[str(setting.gcode)]
        bench_plan = json.loads(
            (plans / name / "bench" / "plan.json").read_text()
        )
        motion_plan = json.loads(
            (plans / name / "motion" / "plan.json").read_text()
        )
        timing = plan_timing(g, bench_plan)
        replayed, _ = line_durations(
            g, phases_of_entries(bench_plan["toolhead"])
        )
        span = slice(g.start + 1, g.end + 1)
        mapping_error = np.abs(replayed - timing.pop("_lines")["D"])[span].max()
        log = (plans / name / "bench" / "klippy.log").read_text()
        out["settings"][name] = {
            **timing,
            "phase_mapping_max_line_error_in_span_s": float(mapping_error),
            "bench_vs_motion_cfg": plan_diff(bench_plan, motion_plan),
            "batch_scv_ignored_warnings": log.count(
                "SQUARE_CORNER_VELOCITY is ignored"
            ),
            "batch_accel_to_decel_warnings": log.count(
                "ACCEL_TO_DECEL is ignored"
            ),
        }
    for label, (kind, plan_path, parts) in REAL.items():
        g = gs[str(STOCK if kind == "stock" else SETTINGS["D"].gcode)]
        plan = json.loads(plan_path.read_text())
        timing = plan_timing(g, plan)
        lines = timing.pop("_lines")
        out["real"][label] = {
            "plan": timing,
            "measured": stats_timing(g, stats_rows(parts), lines),
        }
    return out


def session_report(session: Path, plans: Path) -> dict:
    record = json.loads((session / "session.json").read_text())
    dictionary = firmware.FW_WORKTREE / firmware.OUT / "klipper.dict"
    gs = gcodes()
    out = {}
    for entry in record["runs"]:
        name = entry["run"]
        if "run_dir" not in entry:
            out[name] = {"error": entry.get("error")}
            continue
        run_dir = Path(entry["run_dir"])
        g = gs[entry["gcode"]]
        plan = json.loads((plans / name / "bench" / "plan.json").read_text())
        planned = plan_timing(g, plan)
        lines = planned.pop("_lines")
        rows = stats_rows([(run_dir / "klippy.log", 0)])
        phases = live_phases(run_dir, "toolhead")
        durations, t_first = line_durations(g, phases)
        cumulative = t_first + np.cumsum(durations)
        t_start, t_end = float(cumulative[g.start]), float(cumulative[g.end])
        whole = (phases[0]["t"], max(p["t"] + p["dur"] for p in phases))
        pa_script = pressure_advance_script(g, phases)
        verdict = judge(run_dir, record["kv2"], dictionary, [pa_script])
        result = {
            "run_dir": str(run_dir),
            "plan": planned,
            "stats": stats_timing(g, rows, lines),
            "trapq": {
                **timing_of_durations(g, durations),
                "whole_s": whole[1] - whole[0],
            },
            "judge": verdict,
            "health": health(g, run_dir, rows),
            "pressure_advance_script": pa_script,
        }
        xy_paired = not any(verdict["reversal_pairs"][m] for m in XY_MOTORS)
        if verdict["checks"]["capture_equals_tap_sequence"] and xy_paired:
            result["capture"] = capture_timing(
                g, run_dir, t_start, t_end, whole
            )
            result["peak_steps_per_s"] = peak_rates(run_dir / "replay")
        out[name] = result
        print(name, "done", flush=True)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("plans", "session", "tables"))
    parser.add_argument("dirs", type=Path, nargs="+")
    args = parser.parse_args()
    if args.action == "tables":
        report = json.loads((args.dirs[0] / "cube_report.json").read_text())
        plans = json.loads((args.dirs[1] / "plans_report.json").read_text())
        print(tables(report, plans))
        return
    if args.action == "plans":
        result = plans_report(args.dirs[0])
        path = args.dirs[0] / "plans_report.json"
    else:
        result = session_report(args.dirs[0], args.dirs[1])
        path = args.dirs[0] / "cube_report.json"
    path.write_text(json.dumps(result, indent=2))
    print(path)


if __name__ == "__main__":
    main()
