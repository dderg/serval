"""SCV / minimum_cruise_ratio / shaper / corner-budget sweep over the corpus.

    python3 ~/Developer/serval/scripts/slots.py run -- \
        uv run python -m tools.motion_eval.sweep [--jobs 8] [--out DIR]

Writes DIR/results.jsonl (one record per run) and DIR/tables.md, and prints
the tables. Every run goes through the full harness, so each one is also a
check of the shaper model against the decoded steps.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import json
import math
import pathlib
import sys

import numpy as np

from . import evaluate

HERE = pathlib.Path(__file__).resolve().parent
CORPUS = HERE / "corpus"
DEFAULT_OUT = HERE / ".build" / "sweep"
CONFIGS = {
    "bench": {"cfg": HERE / "configs" / "bench.cfg", "freq": (50, 44)},
    "fast": {"cfg": HERE / "configs" / "fast.cfg", "freq": (70, 70)},
}
DEFAULT_FAMILY = {"bench": "mzv", "fast": "smooth_mzv"}
FAMILIES = ("mzv", "smooth_mzv")
SCV_VALUES = (1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 35.0)
MCR_VALUES = (0.0, 0.25, 0.5)
BUDGETS_MM = (
    0.03,
    0.05,
    0.07,
    0.08,
    0.09,
    0.1,
    0.12,
    0.15,
    0.2,
    0.25,
    0.3,
    0.4,
)
BASE_SCV = 5.0
BASE_MCR = 0.5
UNLIMITED = 1e4
SWEEP_DT = 50e-6


@dataclasses.dataclass(frozen=True)
class Run:
    kind: str
    case: str
    config: str
    family: str
    scv: float
    mcr: float
    budget: float | None = None
    unlimited_extruder_corner: bool = False

    def key(self):
        return json.dumps(dataclasses.asdict(self), sort_keys=True)


def corpus_cases():
    return sorted(p.stem for p in CORPUS.glob("*.gcode"))


def case_start(text):
    first = text.splitlines()[0]
    if not first.startswith("; motion_eval start="):
        raise ValueError("corpus file lacks the start header: %r" % first)
    return [float(v) for v in first.split("=", 1)[1].split(",")]


def plan(cases, configs):
    runs = []
    for config in configs:
        for case in cases:
            for family in FAMILIES:
                for scv in SCV_VALUES:
                    for mcr in MCR_VALUES:
                        runs.append(Run("scv", case, config, family, scv, mcr))
                for budget in BUDGETS_MM:
                    for mcr in MCR_VALUES:
                        runs.append(
                            Run(
                                "budget",
                                case,
                                config,
                                family,
                                BASE_SCV,
                                mcr,
                                budget,
                            )
                        )
            family = DEFAULT_FAMILY[config]
            for scv, mcr, icv in (
                (UNLIMITED, BASE_MCR, False),
                (BASE_SCV, 0.0, False),
                (UNLIMITED, 0.0, False),
                (UNLIMITED, 0.0, True),
            ):
                runs.append(
                    Run("decomp", case, config, family, scv, mcr, None, icv)
                )
    return runs


def _config_path(run, tmpdir):
    base = CONFIGS[run.config]["cfg"]
    if not run.unlimited_extruder_corner:
        return base
    path = pathlib.Path(tmpdir) / ("%s_icv.cfg" % run.config)
    path.write_text(
        "[include %s]\n\n[extruder]\ninstantaneous_corner_velocity: %r\n"
        % (base, UNLIMITED)
    )
    return path


def _move_distance(m):
    end_v = m["cruise_v"] - m["accel"] * m["decel_t"]
    return (
        0.5 * (m["start_v"] + m["cruise_v"]) * m["accel_t"]
        + m["cruise_v"] * m["cruise_t"]
        + 0.5 * (m["cruise_v"] + end_v) * m["decel_t"]
    )


def execute(run, tmpdir):
    text = (CORPUS / (run.case + ".gcode")).read_text()
    fx, fy = CONFIGS[run.config]["freq"]
    prefix = (
        "SET_INPUT_SHAPER SHAPER_TYPE=%s SHAPER_FREQ_X=%r SHAPER_FREQ_Y=%r\n"
        "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=%r MINIMUM_CRUISE_RATIO=%r\n"
        % (run.family, fx, fy, run.scv, run.mcr)
    )
    r = evaluate.evaluate(
        _config_path(run, tmpdir),
        prefix + text,
        case_start(text),
        dt=SWEEP_DT,
        sample_dt=None,
        corner_budget_mm=run.budget,
    )
    ex = r["executed"]["per_axis"]
    cmd = r["commanded"]["per_axis"]
    moves = r["commanded"]["moves"]
    return dict(
        dataclasses.asdict(run),
        print_time_s=r["print_time_s"],
        feed_time_s=sum(_move_distance(m) / m["max_cruise_v"] for m in moves),
        phase_time=r["phase_time"],
        corners=r["executed"]["corners"],
        path_deviation_mm=r["executed"]["xy_path_deviation_model"]["max_mm"],
        max_jump_mm_s=max(ex[a]["max_velocity_jump_mm_s"] for a in "xy"),
        p99_jump_mm_s=max(ex[a]["p99_velocity_jump_mm_s"] for a in "xy"),
        peak_accel=max(ex[a]["peak_accel_mm_s2"] for a in "xy"),
        cmd_peak_accel=max(cmd[a]["peak_accel_mm_s2"] for a in "xy"),
        max_step_model_error_share=max(
            s["model_vs_steps"]["max_error_mm"]
            / s["model_vs_steps"]["max_allowed_mm"]
            for s in r["steppers"].values()
            if s["model_vs_steps"]["steps"]
        ),
    )


def _execute_with_tmp(run):
    import tempfile

    with tempfile.TemporaryDirectory(prefix="motion_eval_sweep_") as tmp:
        return execute(run, tmp)


def run_all(runs, jobs):
    records = []
    with concurrent.futures.ProcessPoolExecutor(jobs) as pool:
        futures = {pool.submit(_execute_with_tmp, run): run for run in runs}
        for i, fut in enumerate(concurrent.futures.as_completed(futures), 1):
            try:
                records.append(fut.result())
            except Exception as e:
                raise RuntimeError("run failed: %s" % futures[fut].key()) from e
            if i % 50 == 0 or i == len(runs):
                print("%d/%d runs" % (i, len(runs)), file=sys.stderr)
    return records


def _index(records):
    return {
        (
            r["kind"],
            r["case"],
            r["config"],
            r["family"],
            r["scv"],
            r["mcr"],
            r["budget"],
            r["unlimited_extruder_corner"],
        ): r
        for r in records
    }


def _baseline(idx, case, config, family):
    return idx[("scv", case, config, family, BASE_SCV, BASE_MCR, None, False)]


def decomposition_table(records, cases, configs):
    idx = _index(records)
    lines = [
        "| config | case | base s | corner limit (scv) | min_cruise_ratio |"
        " both | extruder corner | accel ramps | idle | below cruise |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for config in configs:
        family = DEFAULT_FAMILY[config]
        totals = np.zeros(7)
        for case in cases:
            base = _baseline(idx, case, config, family)

            def t(scv, mcr, icv=False):
                return idx[
                    ("decomp", case, config, family, scv, mcr, None, icv)
                ]["print_time_s"]

            tb = base["print_time_s"]
            no_scv, mcr0 = t(UNLIMITED, BASE_MCR), t(BASE_SCV, 0.0)
            both, free = t(UNLIMITED, 0.0), t(UNLIMITED, 0.0, True)
            idle = base["phase_time"]["idle_s"]
            ramps = free - base["feed_time_s"] - idle
            below = (
                base["phase_time"]["accel_s"] + base["phase_time"]["decel_s"]
            )
            row = np.array(
                [
                    tb,
                    tb - no_scv,
                    tb - mcr0,
                    tb - both,
                    both - free,
                    ramps,
                    idle,
                ]
            )
            totals += row
            lines.append(_decomp_row(config, case, row, below / tb))
        lines.append(_decomp_row(config, "**sum**", totals, None))
    return "\n".join(lines)


def _decomp_row(config, case, row, below_share):
    tb = row[0]
    cells = ["%.2f" % tb] + [
        "%.2f (%.0f%%)" % (v, 100.0 * v / tb) for v in row[1:]
    ]
    below = "" if below_share is None else "%.0f%%" % (100.0 * below_share)
    return "| %s | %s | %s | %s |" % (config, case, " | ".join(cells), below)


def _pareto_mask(times, devs):
    keep = []
    for i in range(len(times)):
        dominated = any(
            times[j] <= times[i]
            and devs[j] <= devs[i]
            and (times[j] < times[i] or devs[j] < devs[i])
            for j in range(len(times))
        )
        keep.append(not dominated)
    return keep


def aggregate(records, cases, config, family, kind, knob):
    idx = _index(records)
    rows = []
    for value in sorted({r[knob] for r in records if r["kind"] == kind}):
        for mcr in MCR_VALUES:
            per_case = []
            for case in cases:
                key = (
                    kind,
                    case,
                    config,
                    family,
                    value if knob == "scv" else BASE_SCV,
                    mcr,
                    value if knob == "budget" else None,
                    False,
                )
                if key in idx:
                    per_case.append((case, idx[key]))
            if len(per_case) != len(cases):
                continue
            rel = [
                r["print_time_s"]
                / _baseline(idx, case, config, DEFAULT_FAMILY[config])[
                    "print_time_s"
                ]
                for case, r in per_case
            ]
            rows.append(
                {
                    knob: value,
                    "mcr": mcr,
                    "rel_time": float(np.mean(rel)),
                    "total_time": sum(r["print_time_s"] for _, r in per_case),
                    "corner_p99": max(
                        r["corners"]["p99_mm"] for _, r in per_case
                    ),
                    "corner_max": max(
                        r["corners"]["max_mm"] for _, r in per_case
                    ),
                    "max_jump": max(r["max_jump_mm_s"] for _, r in per_case),
                    "peak_accel": max(r["peak_accel"] for _, r in per_case),
                }
            )
    return rows


def pareto_table(rows, knob):
    rows = sorted(rows, key=lambda r: r["corner_p99"])
    mask = _pareto_mask(
        [r["rel_time"] for r in rows], [r["corner_p99"] for r in rows]
    )
    lines = [
        "| %s | mcr | time vs base | corpus s | corner p99 mm | corner max mm |"
        " max jump mm/s | peak accel | pareto |" % knob,
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r, front in zip(rows, mask):
        lines.append(
            "| %g | %g | %.3f | %.1f | %.3f | %.3f | %.1f | %.0f | %s |"
            % (
                r[knob],
                r["mcr"],
                r["rel_time"],
                r["total_time"],
                r["corner_p99"],
                r["corner_max"],
                r["max_jump"],
                r["peak_accel"],
                "*" if front else "",
            )
        )
    return "\n".join(lines)


def _setting(r):
    knob = (
        "scv %g" % r["scv"] if r["kind"] == "scv" else "budget %g" % r["budget"]
    )
    return "%s, mcr %g" % (knob, r["mcr"])


def _runs_by_setting(records, kind, config, family):
    grouped = {}
    for r in records:
        if (
            r["kind"] == kind
            and r["config"] == config
            and r["family"] == family
        ):
            grouped.setdefault(_setting(r), {})[r["case"]] = r
    return grouped


def equal_deviation_tables(records, cases, configs):
    idx = _index(records)
    per_case = [
        "| config | shaper | case | default s | default p99 mm |"
        " best SCV setting | s | best budget setting | s |"
        " budget vs default | budget vs best SCV |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    single = [
        "| config | shaper | default corpus s | SCV setting | s | vs default |"
        " budget setting | s | vs default | budget vs SCV |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for config in configs:
        for family in FAMILIES:
            default = {
                case: _baseline(idx, case, config, family) for case in cases
            }
            limit = {c: default[c]["corners"]["p99_mm"] for c in cases}
            best = {}
            for kind in ("scv", "budget"):
                groups = _runs_by_setting(records, kind, config, family)
                for case in cases:
                    ok = [
                        g[case]
                        for g in groups.values()
                        if g[case]["corners"]["p99_mm"] <= limit[case]
                    ]
                    best[kind, case] = min(
                        ok, key=lambda r: r["print_time_s"], default=None
                    )
                feasible = {
                    name: sum(g[c]["print_time_s"] for c in cases)
                    for name, g in groups.items()
                    if all(g[c]["corners"]["p99_mm"] <= limit[c] for c in cases)
                }
                best[kind] = min(
                    feasible.items(),
                    key=lambda kv: kv[1],
                    default=("none", math.nan),
                )
            for case in cases:
                td = default[case]["print_time_s"]
                s, b = best["scv", case], best["budget", case]
                per_case.append(
                    "| %s | %s | %s | %.2f | %.3f | %s | %.2f | %s | %s | %s | %s |"
                    % (
                        config,
                        family,
                        case,
                        td,
                        limit[case],
                        _setting(s),
                        s["print_time_s"],
                        _setting(b) if b else "none",
                        "%.2f" % b["print_time_s"] if b else "",
                        _pct(td, b["print_time_s"]) if b else "",
                        _pct(s["print_time_s"], b["print_time_s"]) if b else "",
                    )
                )
            total = sum(default[c]["print_time_s"] for c in cases)
            (s_name, s_time), (b_name, b_time) = best["scv"], best["budget"]
            single.append(
                "| %s | %s | %.1f | %s | %.1f | %s | %s | %.1f | %s | %s |"
                % (
                    config,
                    family,
                    total,
                    s_name,
                    s_time,
                    _pct(total, s_time),
                    b_name,
                    b_time,
                    _pct(total, b_time),
                    _pct(s_time, b_time),
                )
            )
    return "\n".join(single), "\n".join(per_case)


def _pct(reference, value):
    return "%+.1f%%" % (100.0 * (value - reference) / reference)


def tables(records, cases, configs):
    out = ["## Time decomposition (default shaper, scv 5, mcr 0.5)", ""]
    out.append(decomposition_table(records, cases, configs))
    for config in configs:
        for family in FAMILIES:
            for kind, knob in (("scv", "scv"), ("budget", "budget")):
                out += [
                    "",
                    "## %s, %s, %s sweep" % (config, family, knob),
                    "",
                    pareto_table(
                        aggregate(records, cases, config, family, kind, knob),
                        knob,
                    ),
                ]
    single, per_case = equal_deviation_tables(records, cases, configs)
    out += [
        "",
        "## Fastest single setting with corner p99 <= default (scv 5, mcr 0.5)"
        " in every case",
        "",
        single,
        "",
        "## Fastest setting per case with corner p99 <= default",
        "",
        per_case,
    ]
    return "\n".join(out) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.motion_eval.sweep")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    ap.add_argument("--cases", help="comma separated subset of the corpus")
    ap.add_argument("--configs", default=",".join(CONFIGS))
    ap.add_argument(
        "--tables-only",
        action="store_true",
        help="rebuild tables.md from an existing results.jsonl",
    )
    args = ap.parse_args(argv)
    cases = args.cases.split(",") if args.cases else corpus_cases()
    configs = args.configs.split(",")
    args.out.mkdir(parents=True, exist_ok=True)
    results = args.out / "results.jsonl"
    if args.tables_only:
        records = [
            json.loads(line) for line in results.read_text().splitlines()
        ]
    else:
        records = run_all(plan(cases, configs), args.jobs)
        results.write_text("".join(json.dumps(r) + "\n" for r in records))
    text = tables(records, cases, configs)
    (args.out / "tables.md").write_text(text)
    sys.stdout.write(text)


if __name__ == "__main__":
    main()
