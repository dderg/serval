"""Timing, per-feature split, judge and health of the cube session.

    uv run python -m tools.kv2_bench.cube_report plans PLANS_DIR
    uv run python -m tools.kv2_bench.cube_report session SESSION_DIR PLANS_DIR
    uv run python -m tools.kv2_bench.cube_report tables SESSION_DIR PLANS_DIR
    uv run python -m tools.kv2_bench.cube_report stripped SESSION_DIR

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
    samples, sample_lines = pc.printing_samples(rows, g.lines, g.ends)
    span = [
        rows[i] for i, n in zip(samples, sample_lines) if g.start <= n <= g.end
    ]

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
            line.startswith("toolhead: max_velocity") for line in lines
        ),
        "span_samples": len(span),
        "buffer_time_min_s": peak("buffer_time", min),
        "print_stall_max": peak("print_stall"),
        "mcu_awake_max": peak("mcu_awake"),
        "mcu_task_avg_max": peak("mcu_task_avg"),
        "corner_table": [
            line for line in lines if "corner_deviation: table ready" in line
        ],
    }


def pressure_advance_script(g: Gcode, phases: list[dict]) -> tuple[float, str]:
    """The program's one SET_PRESSURE_ADVANCE comes after PRINT_START and
    before the first travel: live, its lookahead callback ran at the end of
    PRINT_START's last move, the toolhead phase just before the first travel."""
    commands = [
        i
        for i, r in enumerate(g.lines)
        if r["text"].startswith("SET_PRESSURE_ADVANCE")
    ]
    print_start = next(
        i for i, r in enumerate(g.lines) if r["text"].startswith("PRINT_START")
    )
    if len(commands) != 1 or not print_start < commands[0] < g.first:
        raise ValueError(
            f"SET_PRESSURE_ADVANCE lines {commands}, expected one before line {g.first + 1}"
        )
    first_travel = next(
        k for k, p in enumerate(phases) if on_segment(p, segments(g)[0])
    )
    before = phases[first_travel - 1]
    return before["t"] + before["dur"], g.lines[commands[0]]["text"].split(";")[
        0
    ].strip()


def reversal_pair_at(
    signs: np.ndarray, i: int, other: np.ndarray, j: int
) -> bool:
    """signs[i:i+2] is a step and its undo, and after it the two sequences
    agree again for REVERSAL_CONTEXT steps (or to their ends)."""
    if i + 1 >= len(signs) or signs[i] != -signs[i + 1]:
        return False
    after = signs[i + 2 : i + 2 + REVERSAL_CONTEXT]
    return np.array_equal(after, other[j : j + len(after)])


def align_reversal_pairs(
    a: Steps, b: Steps, freq: float
) -> tuple[Steps, Steps, list[dict]]:
    """a and b with the ±1 reversal pairs one side emitted and the other did
    not removed; any other difference raises."""
    keep_a, keep_b = np.ones(len(a.signs), bool), np.ones(len(b.signs), bool)
    events = []
    i = j = 0
    while True:
        d = compare.first_divergence(a.signs[i:], b.signs[j:])
        if d is None:
            break
        i, j = i + d, j + d
        if reversal_pair_at(b.signs, j, a.signs, i):
            keep_b[j : j + 2] = False
            events.append({"extra_in": "live tap", "index": j, "t": b.clocks[j] / freq,
                           "pair_spacing_us": (b.clocks[j + 1] - b.clocks[j]) / freq * 1e6})  # fmt: skip
            j += 2
        elif reversal_pair_at(a.signs, i, b.signs, j):
            keep_a[i : i + 2] = False
            events.append({"extra_in": "replay", "index": i, "t": a.clocks[i] / freq,
                           "pair_spacing_us": (a.clocks[i + 1] - a.clocks[i]) / freq * 1e6})  # fmt: skip
            i += 2
        else:
            raise ValueError(
                f"replay and tap differ by more than a reversal pair at replay step {i}"
            )
    return (
        Steps(a.clocks[keep_a], a.signs[keep_a]),
        Steps(b.clocks[keep_b], b.signs[keep_b]),
        [
            {k: float(v) if not isinstance(v, str) else v for k, v in e.items()}
            for e in events
        ],
    )


def judge_run(run_dir: Path, reference: Reference) -> dict:
    """Capture vs live tap: the MCU executed every commanded step (sequences
    equal) and its lateness. Replay vs tap: the live trapq regenerates the
    commanded steps, up to ±1 reversal pairs (stepcompress flush boundaries,
    RESULTS.md "Live-trapq replay reference"), listed. Capture vs replay with
    those pairs removed: timing within 25 µs + 0.1 µs + this run's lateness."""
    freq = reference.clock_freq
    tap = compare.load_tap(run_dir)
    cap = compare.load_capture(run_dir)
    names = list(reference.steppers)
    executed = compare.sequences(
        {n: tap[n] for n in names}, {n: cap[n] for n in names}
    )
    _, _, residual = compare.timeline(
        {n: tap[n] for n in names}, {n: cap[n] for n in names}, freq, None, True
    )
    lateness = compare.mcu_lateness(residual)
    planned, commanded, captured, pairs = {}, {}, {}, {}
    for name in names:
        planned[name], commanded[name], pairs[name] = align_reversal_pairs(
            reference.steppers[name], tap[name], freq
        )
        _, captured[name], _ = align_reversal_pairs(
            reference.steppers[name], cap[name], freq
        )
    identity = compare.schedule_identity(planned, commanded, freq)
    timing, _, _ = compare.timeline(planned, captured, freq, None, True)
    bound_us = (
        compare.MAX_STEPCOMPRESS_ERROR_S + compare.INSTRUMENT_SLACK_S
    ) * 1e6 + lateness["max_us"]
    checks = {
        "net_steps_match": all(
            reference.steppers[n].net == cap[n].net == tap[n].net for n in names
        ),
        "capture_equals_tap_sequence": executed["all_steps_paired"],
        "replay_equals_tap_up_to_reversal_pairs": compare.sequences(
            planned, commanded
        )["all_steps_paired"],
        "replay_within_max_error": identity["abs_max_us"]
        <= compare.MAX_STEPCOMPRESS_ERROR_S * 1e6
        and identity["epochs"] == 1,
        "capture_within_bound": timing["abs_max_us"] <= bound_us,
    }
    return {
        "checks": checks,
        "ok": all(checks.values()),
        "capture_vs_tap": {"sequences": executed, "mcu_lateness": lateness},
        "reversal_pairs": pairs,
        "replay_vs_tap": identity,
        "capture_vs_replay": {**timing, "bound_us": bound_us},
    }


def judge(
    run_dir: Path, kv2: str, dictionary: Path, timed_scripts: list
) -> dict:
    out = run_dir / "replay" / "cube_judgement.json"
    if not out.exists():
        if (run_dir / "replay" / "reference.json").exists():
            reference = Reference.load(run_dir / "replay")
        else:
            firmware.checkout(firmware.FW_WORKTREE, kv2)
            reference = replay.build(
                run_dir,
                firmware.FW_WORKTREE,
                dictionary,
                run_dir / "replay",
                timed_scripts,
            )
        out.write_text(json.dumps(judge_run(run_dir, reference), indent=2))
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


def cube_phases(g: Gcode, run_dir: Path) -> list[dict]:
    phases = live_phases(run_dir, "toolhead")
    first = next(
        k for k, p in enumerate(phases) if on_segment(p, segments(g)[0])
    )
    return phases[first:]


def live_identity(a: list[dict], b: list[dict]) -> dict:
    """Two live toolhead trapqs from the first travel on: phases with equal
    geometry, accel, speed and duration, and where their start times,
    relative to the first travel, step apart."""
    same = sum(
        p["start"] == q["start"] and p["axes"] == q["axes"] and p["a"] == q["a"]
        and p["v0"] == q["v0"] and p["dur"] == q["dur"]
        for p, q in zip(a, b)
    )  # fmt: skip
    offset = np.array(
        [(p["t"] - a[0]["t"]) - (q["t"] - b[0]["t"]) for p, q in zip(a, b)]
    )
    steps = np.nonzero(np.abs(np.diff(offset)) > 1e-9)[0] + 1
    return {
        "phases": [len(a), len(b)],
        "identical_phases": int(same),
        "start_time_steps": [
            {"phase": int(k), "t_from_first_travel_s": float(a[k]["t"] - a[0]["t"]),
             "step_ms": float((offset[k] - offset[k - 1]) * 1e3)}
            for k in steps
        ],
    }  # fmt: skip


def stock_vs_stripped(session: Path) -> dict:
    """Run C (unedited file) against run D (`_cd` file), same config."""
    record = json.loads((session / "session.json").read_text())
    runs = {e["run"]: e for e in record["runs"] if "run_dir" in e}
    gs = gcodes()
    phases = [
        cube_phases(gs[runs[n]["gcode"]], Path(runs[n]["run_dir"]))
        for n in ("C", "D")
    ]
    return live_identity(*phases)


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
                "max_cumulative_diff_to_plan_s": float(
                    np.abs(
                        np.cumsum(
                            (durations - lines["D"])[g.start + 1 : g.end + 1]
                        )
                    ).max()
                ),
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


SETTING_LABEL = {
    "A": "stock file, mcr 0.2",
    "B": "stock file, mcr 0.1",
    "C": "unedited file, cap 21000",
    "D": "`_cd` file, cap 21000",
    "E": "unedited file, cap 24000",
    "F": "unedited file, cap 28000",
    "G": "unedited file, no cap",
}
REAL_OF = {
    "A": "print 1 (stock, old host)",
    "C": "print 2 (_cd, corner 21000)",
    "D": "print 2 (_cd, corner 21000)",
}


def tables(report: dict, plans: dict) -> str:
    out = []
    base = report["A"]["trapq"]["span_s"]
    out += [
        "| run | setting | span: Stats print_time | Stats wall | live trapq | capture | offline plan | real printer | Δ vs A (trapq) | whole file: trapq | capture | plan |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for name, r in report.items():
        real = (
            plans["real"]
            .get(REAL_OF.get(name, ""), {})
            .get("measured", {})
            .get("span_pt_s")
        )
        cap = r.get("capture", {})
        out.append(
            f"| {name} | {SETTING_LABEL[name]} | {r['stats']['span_pt_s']:.2f} | {r['stats']['span_wall_s']:.2f} | "
            f"{r['trapq']['span_s']:.2f} | {cap.get('span_s', float('nan')):.2f} | {r['plan']['span_s']:.2f} | "
            f"{'' if real is None else f'{real:.2f}'} | {100 * (r['trapq']['span_s'] / base - 1):+.2f}% | "
            f"{r['trapq']['whole_s']:.2f} | {cap.get('whole_s', float('nan')):.2f} | {r['plan']['whole_s']:.2f} |"
        )
    out += [
        "",
        "| run | outer wall | inner wall | infill | travel | other | source |",
        "|---|---|---|---|---|---|---|",
    ]
    for name, r in report.items():
        rows = [
            ("live trapq", r["trapq"]["features"], None),
            (
                "Stats regression",
                r["stats"]["regression"]["features"],
                r["stats"]["regression"]["features_se"],
            ),
            ("offline plan", r["plan"]["features"], None),
        ]
        real = plans["real"].get(REAL_OF.get(name, ""))
        if real:
            m = real["measured"]["regression"]
            rows.append(
                (
                    "real printer, Stats regression",
                    m["features"],
                    m["features_se"],
                )
            )
        for source, values, se in rows:
            cells = [
                f"{values[f]:.1f}" + ("" if se is None else f" ± {se[f]:.1f}")
                for f in FEATURES
            ]
            out.append(f"| {name} | " + " | ".join(cells) + f" | {source} |")
    out += [
        "",
        "| run | net steps exact | capture = live tap (every step, in order) | replay = tap (± reversal pairs) / within 25 µs | reversal pairs X/Y/Z/E | capture−replay max / bound µs | MCU lateness max µs | F407/F401 ppm | peak steps/s X / Y / E / aggregate |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for name, r in report.items():
        j = r["judge"]
        c = j["checks"]
        timing = j["capture_vs_replay"]
        peak = r.get("peak_steps_per_s", {})
        pairs = "/".join(
            str(len(j["reversal_pairs"][m])) for m in compare.CHANNELS
        )
        out.append(
            f"| {name} | {c['net_steps_match']} | {c['capture_equals_tap_sequence']} | "
            f"{c['replay_equals_tap_up_to_reversal_pairs']} / {c['replay_within_max_error']} | {pairs} | "
            f"{timing['abs_max_us']:.1f} / {timing['bound_us']:.1f} | "
            f"{j['capture_vs_tap']['mcu_lateness']['max_us']:.1f} | {timing['slope_ppm']:.1f} | "
            + " / ".join(
                f"{peak[k] / 1e3:.1f} k"
                for k in ("stepper_x", "stepper_y", "extruder", "aggregate")
            )
            + " |"
        )
    out += [
        "",
        "| run | print | error lines | SCV warnings | ACCEL_TO_DECEL warnings | toolhead limit lines | min buffer_time s | print_stall | mcu_awake max | mcu_task_avg max µs |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for name, r in report.items():
        h = r["health"]
        out.append(
            f"| {name} | {h['print_state']} | {len(h['errors'])} | {h['scv_ignored_warnings']} | {h['accel_to_decel_warnings']} | "
            f"{h['toolhead_limit_lines']} | {h['buffer_time_min_s']:.2f} | {h['print_stall_max']:.0f} | "
            f"{h['mcu_awake_max']:.3f} | {1e6 * h['mcu_task_avg_max']:.0f} |"
        )
    return "\n".join(out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action", choices=("plans", "session", "tables", "stripped")
    )
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
    elif args.action == "stripped":
        result = stock_vs_stripped(args.dirs[0])
        path = args.dirs[0] / "unedited_vs_stripped.json"
    else:
        result = session_report(args.dirs[0], args.dirs[1])
        path = args.dirs[0] / "cube_report.json"
    path.write_text(json.dumps(result, indent=2))
    print(path)


if __name__ == "__main__":
    main()
