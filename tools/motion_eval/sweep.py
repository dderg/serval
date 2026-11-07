"""SCV / minimum_cruise_ratio / shaper / [corner_deviation] sweep over the corpus.

    python3 ~/Developer/serval/scripts/slots.py run -- \
        uv run python -m tools.motion_eval.sweep [--jobs 8] [--out DIR]

Writes DIR/results.jsonl (one record per run) and DIR/tables.md, and prints
the tables. Every run goes through the full harness, so each one is also a
check of the shaper model against the decoded steps.
"""

from __future__ import annotations

import argparse
import collections
import concurrent.futures
import dataclasses
import json
import math
import pathlib
import sys

import numpy as np

from . import analysis, evaluate

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
CAPS_MM = (
    0.05,
    0.07,
    0.09,
    0.1,
    0.11,
    0.12,
    0.13,
    0.14,
    0.15,
    0.16,
    0.18,
    0.2,
    0.22,
    0.25,
)
PARETO_CASES = (
    "voron_top_layers",
    "circles",
    "zigzag_infill",
    "corners_turn90",
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
    max_deviation: float | None = None
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
                runs.append(
                    Run("floor", case, config, family, BASE_SCV, BASE_MCR)
                )
                for cap in CAPS_MM:
                    runs.append(
                        Run(
                            "cap", case, config, family, BASE_SCV, BASE_MCR, cap
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
    sections = []
    if run.unlimited_extruder_corner:
        sections.append(
            "[extruder]\ninstantaneous_corner_velocity: %r\n" % (UNLIMITED,)
        )
    if run.kind in ("floor", "cap"):
        section = "[corner_deviation]\n"
        if run.max_deviation is not None:
            section += "max_deviation: %r\n" % (run.max_deviation,)
        sections.append(section)
    if not sections:
        return base
    path = pathlib.Path(tmpdir) / "printer.cfg"
    path.write_text("[include %s]\n\n%s" % (base, "\n".join(sections)))
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
    limits = "MINIMUM_CRUISE_RATIO=%r" % (run.mcr,)
    if run.kind in ("scv", "decomp"):
        limits += " SQUARE_CORNER_VELOCITY=%r" % (run.scv,)
    prefix = (
        "SET_INPUT_SHAPER SHAPER_TYPE=%s SHAPER_FREQ_X=%r SHAPER_FREQ_Y=%r\n"
        "SET_VELOCITY_LIMIT %s\n" % (run.family, fx, fy, limits)
    )
    r = evaluate.evaluate(
        _config_path(run, tmpdir),
        prefix + text,
        case_start(text),
        dt=SWEEP_DT,
        sample_dt=None,
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
        moves_planned=r["moves_planned"],
        lookahead_us_per_move=1e6
        * r["timing"]["lookahead_s"]
        / r["moves_planned"],
        limiter_us_per_move=1e6
        * r["timing"]["junction_limiter_s"]
        / r["moves_planned"],
        limiter_stats=r["corner_deviation"],
        max_accel=r["settings"]["max_accel"],
        shaper_floor_mm=_shaper_floor(r),
        move_accels=dict(
            collections.Counter("%.6g" % m["accel"] for m in moves)
        ),
    )


def _shaper_floor(r):
    """sigma_x^2 * a_x and sigma_y^2 * a_y over 2 at max_accel on both axes:
    no acceleration profile within max_accel rounds the shaped path more."""
    variances = [
        analysis.Kernel.from_shaper(r["shapers"][0][axis]).second_moment()
        for axis in "xy"
    ]
    return 0.5 * r["settings"]["max_accel"] * math.hypot(*variances)


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
            r["max_deviation"],
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
                    value if knob == "max_deviation" else None,
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
    if r["kind"] == "scv":
        knob = "scv %g" % r["scv"]
    elif r["kind"] == "floor":
        knob = "floor"
    else:
        knob = "floor+cap %g" % r["max_deviation"]
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


def _fastest_within(groups, case, metric, limit):
    ok = [
        g[case] for g in groups.values() if g[case]["corners"][metric] <= limit
    ]
    return min(ok, key=lambda r: r["print_time_s"], default=None)


def _time_cell(reference, r):
    if r is None:
        return "none | | "
    return "%s | %.2f | %s" % (
        _setting(r).split(",")[0],
        r["print_time_s"],
        _pct(reference, r["print_time_s"]),
    )


def floor_tables(records, cases, configs):
    """Default (scv 5, mcr 0.5) against floor mode and floor+cap, the
    setting chosen as the fastest of floor and every cap whose corner max
    (or p99) is within the default's in that case."""
    idx = _index(records)
    per_case = [
        "| config | shaper | case | default s | max mm | p99 mm | floor s |"
        " max mm | p99 mm | floor vs default | equal max: cap | s |"
        " vs default | equal p99: cap | s | vs default | max step mm/s"
        " default / floor | peak accel default / floor |",
        "|" + "---|" * 18,
    ]
    corpus = [
        "| config | shaper | default s | floor s | floor vs default |"
        " cases where floor max <= default | single cap, equal max | s |"
        " vs default | single cap, equal p99 | s | vs default |"
        " per-case cap, equal max: s | vs default |",
        "|" + "---|" * 13,
    ]
    for config in configs:
        for family in FAMILIES:
            default = {
                case: _baseline(idx, case, config, family) for case in cases
            }
            floor = {
                case: idx[
                    ("floor", case, config, family, BASE_SCV, BASE_MCR)
                    + (None, False)
                ]
                for case in cases
            }
            caps = _runs_by_setting(records, "cap", config, family)
            caps["floor, mcr %g" % BASE_MCR] = floor
            best = {}
            for metric in ("max_mm", "p99_mm"):
                for case in cases:
                    best[metric, case] = _fastest_within(
                        caps, case, metric, default[case]["corners"][metric]
                    )
                feasible = {
                    name: sum(g[c]["print_time_s"] for c in cases)
                    for name, g in caps.items()
                    if all(
                        g[c]["corners"][metric] <= default[c]["corners"][metric]
                        for c in cases
                    )
                }
                best[metric] = min(
                    feasible.items(),
                    key=lambda kv: kv[1],
                    default=("none", math.nan),
                )
            for case in cases:
                d, f = default[case], floor[case]
                td = d["print_time_s"]
                per_case.append(
                    "| %s | %s | %s | %.2f | %.3f | %.3f | %.2f | %.3f | %.3f"
                    " | %s | %s | %s | %.1f / %.1f | %.0f / %.0f |"
                    % (
                        config,
                        family,
                        case,
                        td,
                        d["corners"]["max_mm"],
                        d["corners"]["p99_mm"],
                        f["print_time_s"],
                        f["corners"]["max_mm"],
                        f["corners"]["p99_mm"],
                        _pct(td, f["print_time_s"]),
                        _time_cell(td, best["max_mm", case]),
                        _time_cell(td, best["p99_mm", case]),
                        d["max_jump_mm_s"],
                        f["max_jump_mm_s"],
                        d["peak_accel"],
                        f["peak_accel"],
                    )
                )
            total = sum(default[c]["print_time_s"] for c in cases)
            floor_total = sum(floor[c]["print_time_s"] for c in cases)
            within = sum(
                floor[c]["corners"]["max_mm"] <= default[c]["corners"]["max_mm"]
                for c in cases
            )
            per_case_total = sum(
                (best["max_mm", c] or default[c])["print_time_s"] for c in cases
            )
            (m_name, m_time), (p_name, p_time) = best["max_mm"], best["p99_mm"]
            corpus.append(
                "| %s | %s | %.1f | %.1f | %s | %d of %d | %s | %.1f | %s | %s"
                " | %.1f | %s | %.1f | %s |"
                % (
                    config,
                    family,
                    total,
                    floor_total,
                    _pct(total, floor_total),
                    within,
                    len(cases),
                    m_name.split(",")[0],
                    m_time,
                    _pct(total, m_time),
                    p_name.split(",")[0],
                    p_time,
                    _pct(total, p_time),
                    per_case_total,
                    _pct(total, per_case_total),
                )
            )
    return "\n".join(corpus), "\n".join(per_case)


PI5_SLOWDOWN = 2.5


def cpu_table(records, configs):
    """Planner cost of the limiter on the densest case (most planned moves
    per second of print time), as a share of one core of this host and of
    a Pi 5 assumed PI5_SLOWDOWN times slower."""
    lines = [
        "| config | densest case | moves/s | lookahead us/move scv 5 |"
        " floor | of which limiter | limiter, % of one core here |"
        " % of a Pi 5 core |",
        "|" + "---|" * 8,
    ]
    for config in configs:
        base = [
            r for r in records if r["kind"] == "scv" and r["config"] == config
        ]
        rate = {
            r["case"]: r["moves_planned"] / r["print_time_s"]
            for r in base
            if r["scv"] == BASE_SCV and r["mcr"] == BASE_MCR
        }
        case = max(rate, key=rate.get)
        on_case = [
            r for r in records if r["config"] == config and r["case"] == case
        ]
        base_us = float(
            np.median(
                [
                    r["lookahead_us_per_move"]
                    for r in on_case
                    if r["kind"] == "scv"
                ]
            )
        )
        floor_runs = [r for r in on_case if r["kind"] in ("floor", "cap")]
        floor_us = float(
            np.median([r["lookahead_us_per_move"] for r in floor_runs])
        )
        limiter_us = float(
            np.median([r["limiter_us_per_move"] for r in floor_runs])
        )
        share = 100.0 * rate[case] * limiter_us * 1e-6
        lines.append(
            "| %s | %s | %.0f | %.1f | %.1f | %.1f | %.2f%% | %.2f%% |"
            % (
                config,
                case,
                rate[case],
                base_us,
                floor_us,
                limiter_us,
                share,
                share * PI5_SLOWDOWN,
            )
        )
    return "\n".join(lines)


def _pct(reference, value):
    return "%+.1f%%" % (100.0 * (value - reference) / reference)


FRONT_STEP = 0.005


def case_pareto_table(records, case, config, family):
    rows = sorted(
        (
            r
            for r in records
            if r["case"] == case
            and r["config"] == config
            and r["family"] == family
            and r["kind"] in ("scv", "floor", "cap")
        ),
        key=lambda r: r["corners"]["max_mm"],
    )
    mask = _pareto_mask(
        [r["print_time_s"] for r in rows],
        [r["corners"]["max_mm"] for r in rows],
    )
    lines = [
        "| setting | s | corner p50 mm | p99 mm | max mm | max jump mm/s |"
        " peak accel |",
        "|---|---|---|---|---|---|---|",
    ]
    kept_time = math.inf
    for r, front in zip(rows, mask):
        if not front or r["print_time_s"] > kept_time * (1 - FRONT_STEP):
            continue
        kept_time = r["print_time_s"]
        c = r["corners"]
        lines.append(
            "| %s | %.2f | %.3f | %.3f | %.3f | %.1f | %.0f |"
            % (
                _setting(r),
                r["print_time_s"],
                c["p50_mm"],
                c["p99_mm"],
                c["max_mm"],
                r["max_jump_mm_s"],
                r["peak_accel"],
            )
        )
    return "\n".join(lines)


def tables(records, cases, configs):
    out = ["## Time decomposition (default shaper, scv 5, mcr 0.5)", ""]
    out.append(decomposition_table(records, cases, configs))
    for config in configs:
        for family in FAMILIES:
            for kind, knob in (("scv", "scv"), ("cap", "max_deviation")):
                out += [
                    "",
                    "## %s, %s, %s sweep" % (config, family, knob),
                    "",
                    pareto_table(
                        aggregate(records, cases, config, family, kind, knob),
                        knob,
                    ),
                ]
    corpus, per_case = floor_tables(records, cases, configs)
    out += [
        "",
        "## Planner CPU (densest case, median over its runs)",
        "",
        cpu_table(records, configs),
        "",
        "## Corpus: default (scv 5, mcr 0.5) vs [corner_deviation] (mcr 0.5)",
        "",
        corpus,
        "",
        "## Per case: default vs floor vs floor+cap at equal max and equal p99",
        "",
        per_case,
    ]
    for config in configs:
        for case in PARETO_CASES:
            if case not in cases:
                continue
            family = DEFAULT_FAMILY[config]
            out += [
                "",
                "## Pareto front (time vs corner max), %s, %s, %s, SCV and"
                " [corner_deviation] runs" % (case, config, family),
                "",
                case_pareto_table(records, case, config, family),
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
