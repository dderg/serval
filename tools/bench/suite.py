"""Run the bench regression matrix under one lock hold.

uv run python tools/bench/suite.py [--quick | --group {matrix,session}]
    [--expect-fail NAME ...]

Syncs the tree, runs every row, prints a table and writes
~/bench/suites/<timestamp>/summary.json. Matrix rows are one print each
(run.py + analyze.py); session rows are one klippy session each
(session.py + session_check.py). The full suite runs both groups. Exit
status is non-zero when any row fails (unless named in --expect-fail) or
when an --expect-fail row passes.
"""

from __future__ import annotations

import argparse
import enum
import json
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import lock
import pi
import session_check
from session_scenarios import SCENARIOS, Scenario

BENCH = Path(__file__).resolve().parent
REPO = BENCH.parents[1]
SUITES_DIR = Path("~/bench/suites").expanduser()
PRINTER_CFG = BENCH / "printer.cfg"
FAST_CFG = BENCH / "configs" / "fast.cfg"
REST_LADDER_CFG = BENCH / "configs" / "rest_ladder.cfg"
CAPACITY_CFG = BENCH / "configs" / "capacity.cfg"
SIM_GCODE = REPO / "tools" / "sim" / "gcode"
RUN_TIMEOUT_S = 2400.0
ANALYZE_TIMEOUT_S = 1200.0
ENGINE_BUILD_TIMEOUT_S = 3600
ENGINE_BUILD = ["make", "-f", "Makefile.rust", "motion-engine-fast"]
THROTTLE_ARGS = ["--host-cpus", "1", "--host-max-mhz", "1500"]
SINGLE_MOTOR_CEILING_STEPS_PER_S = 172_000
BENCHMARK_BAND = (0.85, 1.10)


class SuiteError(RuntimeError):
    pass


def step_benchmark_verdict(mcu_status: dict) -> dict:
    benchmark = mcu_status.get("step_benchmark")
    if benchmark is None:
        raise SuiteError(f"mcu status has no step_benchmark: {mcu_status}")
    if benchmark["simulated"]:
        raise SuiteError(
            f"the bench MCU reports a simulated benchmark: {benchmark}"
        )
    clock = float(mcu_status["mcu_constants"]["CLOCK_FREQ"])
    capacity = clock / benchmark["pulsed_ticks_per_step"]
    ratio = capacity / SINGLE_MOTOR_CEILING_STEPS_PER_S
    return {
        **benchmark,
        "pulsed_capacity_steps_per_s": capacity,
        "single_motor_ceiling_steps_per_s": SINGLE_MOTOR_CEILING_STEPS_PER_S,
        "ratio_to_single_motor_ceiling": ratio,
        "band": list(BENCHMARK_BAND),
        "within_band": BENCHMARK_BAND[0] <= ratio <= BENCHMARK_BAND[1],
    }


def query_step_benchmark() -> dict:
    status = pi.http("GET", "/printer/objects/query?mcu")["status"]["mcu"]
    return step_benchmark_verdict(status)


@dataclass(frozen=True)
class Row:
    name: str
    gcode: Path
    config: Path = PRINTER_CFG
    throttle: list[str] = field(default_factory=list)
    quick: bool = False
    run_args: list[str] = field(default_factory=list)
    analyze_args: list[str] = field(default_factory=list)


MATRIX = [
    Row("first_layers", SIM_GCODE / "voron_first_layers.gcode", quick=True),
    Row("speed_ladder", BENCH / "gcode" / "speed_ladder.gcode", quick=True),
    Row("speed_ladder_fast", BENCH / "gcode" / "speed_ladder.gcode", FAST_CFG),
    Row("extrude_ladder", BENCH / "gcode" / "extrude_ladder.gcode"),
    Row("epochs", BENCH / "gcode" / "epochs.gcode", quick=True),
    Row("rest_ladder", BENCH / "gcode" / "rest_ladder.gcode", REST_LADDER_CFG),
    Row("dense_top", SIM_GCODE / "voron_dense_top_layers.gcode"),
    Row(
        "dense_top_fast_throttled",
        SIM_GCODE / "voron_dense_top_layers.gcode",
        FAST_CFG,
        THROTTLE_ARGS,
    ),
    Row(
        "corner_flow",
        BENCH / "gcode" / "corner_flow.gcode",
        run_args=["--events"],
        analyze_args=["--corner-flow", "--corner-budget"],
    ),
    Row(
        "corner_budget_default",
        BENCH / "gcode" / "corner_flow.gcode",
        BENCH / "configs" / "printer_default_corner.cfg",
        run_args=["--events"],
        analyze_args=["--corner-budget"],
    ),
    Row(
        "single_motor_cap",
        BENCH / "gcode" / "single_motor_cap.gcode",
        CAPACITY_CFG,
        quick=True,
        run_args=["--events", "--step-tap"],
    ),
]


@dataclass(frozen=True)
class SessionRow:
    scenario: Scenario

    @property
    def name(self) -> str:
        return self.scenario.name


class Scope(enum.Enum):
    ALL = "all"
    QUICK = "quick"
    MATRIX = "matrix"
    SESSION = "session"


GROUPS = (Scope.MATRIX, Scope.SESSION)
SESSION_ROWS = [SessionRow(scenario) for scenario in SCENARIOS]
ROWS = [*MATRIX, *SESSION_ROWS]


def selected_rows(scope: Scope) -> list:
    if scope is Scope.QUICK:
        return [row for row in MATRIX if row.quick]
    rows = {Scope.ALL: ROWS, Scope.MATRIX: MATRIX, Scope.SESSION: SESSION_ROWS}
    return rows[scope]


def python(
    script: str, *args: str, timeout_s: float
) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(BENCH / script), *args],
        capture_output=True,
        text=True,
        timeout=timeout_s,
        pass_fds=lock.inherited_fds(),
    )


def build_local_engine() -> None:
    build = subprocess.run(
        ENGINE_BUILD,
        cwd=REPO,
        capture_output=True,
        text=True,
        timeout=ENGINE_BUILD_TIMEOUT_S,
    )
    if build.returncode != 0:
        raise SuiteError(
            f"`{' '.join(ENGINE_BUILD)}` in {REPO} exited "
            f"{build.returncode}; analyzing with the stale engine would hold "
            f"every row against another planner:\n{build.stdout}{build.stderr}"
        )


def last_lines(text: str, count: int = 3) -> str:
    return " | ".join(line for line in text.strip().splitlines()[-count:])


def read_json(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


def row_identity(row: Row | SessionRow) -> dict:
    if isinstance(row, SessionRow):
        return {
            "gcode": row.scenario.gcode.describe(),
            "config": row.scenario.config.describe(),
            "throttle": [],
        }
    return {
        "gcode": str(row.gcode.relative_to(REPO)),
        "config": row.config.name,
        "throttle": row.throttle,
    }


def runner_args(
    row: Row | SessionRow, runs_dir: Path, lock_timeout_s: float
) -> list[str]:
    common = [
        "--runs-dir",
        str(runs_dir),
        "--lock-timeout",
        str(lock_timeout_s),
    ]
    if isinstance(row, SessionRow):
        return ["session.py", row.name, *common]
    return [
        "run.py",
        str(row.gcode),
        "--config",
        str(row.config),
        *common,
        *row.throttle,
        *row.run_args,
    ]


def run_row(
    row: Row | SessionRow, suite_dir: Path, lock_timeout_s: float
) -> dict:
    runs_dir = suite_dir / row.name
    result: dict = {
        "name": row.name,
        **row_identity(row),
        "run_dir": None,
        "error": None,
    }
    started = time.monotonic()
    runner = runner_args(row, runs_dir, lock_timeout_s)
    run = python(*runner, timeout_s=RUN_TIMEOUT_S)
    run_dirs = sorted(runs_dir.glob("*")) if runs_dir.exists() else []
    if run_dirs:
        result["run_dir"] = str(run_dirs[-1])
    if run.returncode != 0:
        result["error"] = f"{runner[0]}: {last_lines(run.stderr)}"
    if run.returncode not in (0, pi.DRIVER_SIDE_EXIT):
        result["recovered_with_reset_target"] = True
        recovery = python("deploy.py", "reset-target", timeout_s=600)
        if recovery.returncode != 0:
            result["error"] += f"; reset-target: {last_lines(recovery.stderr)}"
    if result["run_dir"]:
        run_dir = Path(result["run_dir"])
        info = read_json(run_dir / "run.json")
        result["print_state"] = info.get("print_state")
        result["print_message"] = info.get("print_message")
        result["duration_s"] = info.get("duration_s")
        result["deployed"] = info.get("deployed")
        result["host_throttle"] = info.get("host_throttle")
        if isinstance(row, SessionRow):
            judge_session(run_dir, result)
        else:
            judge_capture(row, run_dir, result)
    result["wall_s"] = time.monotonic() - started
    result["passed"] = row_passed(row, result)
    return result


def add_error(result: dict, error: str) -> None:
    result["error"] = "; ".join(filter(None, [result["error"], error]))


def judge_capture(row: Row, run_dir: Path, result: dict) -> None:
    if (run_dir / "capture.bin").exists() or (run_dir / "events.bin").exists():
        analysis = python(
            "analyze.py",
            str(run_dir),
            *row.analyze_args,
            timeout_s=ANALYZE_TIMEOUT_S,
        )
        if analysis.returncode != 0:
            add_error(result, f"analyze.py: {last_lines(analysis.stderr)}")
    verdict = read_json(run_dir / "analysis.json")
    if not verdict:
        return
    result["analyzer_ok"] = verdict["ok"]
    result["failed_checks"] = [
        name for name, ok in verdict["checks"].items() if not ok
    ]
    result["stalls"] = verdict["stalls"]["count"]
    result["abs_max_us"] = {
        role: (motor["timing_error_capture_minus_reference_us"] or {}).get(
            "abs_max"
        )
        for role, motor in verdict["motors"].items()
    }
    if "corner_flow" in verdict:
        result["corner_flow_max_abs"] = verdict["corner_flow"]["max_abs"]
    if "corner_budget" in verdict:
        result["corner_budget_worst"] = verdict["corner_budget"]["worst"]


def judge_session(run_dir: Path, result: dict) -> None:
    check = python(
        "session_check.py", str(run_dir), timeout_s=ANALYZE_TIMEOUT_S
    )
    verdict = read_json(run_dir / session_check.CHECKS_FILE)
    if not verdict:
        add_error(result, f"session_check.py: {last_lines(check.stderr)}")
        return
    result["session_ok"] = verdict["ok"]
    result["guards"] = verdict["guards"]
    result["analyzer"] = f"not run: {verdict['not_analyzed']}"
    result["failed_checks"] = [
        f"{c['name']}: {c['detail']}" for c in verdict["checks"] if not c["ok"]
    ]


def row_passed(row: Row | SessionRow, result: dict) -> bool:
    if result["error"] is not None:
        return False
    if isinstance(row, SessionRow):
        return bool(result.get("session_ok"))
    return result.get("print_state") == "complete" and bool(
        result.get("analyzer_ok")
    )


def fmt_abs_max(row: dict) -> str:
    values = [
        v for v in (row.get("abs_max_us") or {}).values() if v is not None
    ]
    return f"{max(values):.0f}" if values else "-"


def print_table(rows: list[dict]) -> None:
    header = f"{'row':<26}{'result':<8}{'print':<11}{'ok':<6}{'stalls':<8}{'abs_max_us':<12}{'dur_s':<8}failed / error"
    print(header)
    for row in rows:
        verdict = (
            "xfail"
            if row["expected_fail"] and not row["passed"]
            else "XPASS"
            if row["expected_fail"]
            else "PASS"
            if row["passed"]
            else "FAIL"
        )
        detail = ",".join(row.get("failed_checks") or []) or (
            row["error"] or ""
        )
        print(
            f"{row['name']:<26}{verdict:<8}{str(row.get('print_state')):<11}"
            f"{str(row.get('analyzer_ok', row.get('session_ok'))):<6}"
            f"{str(row.get('stalls')):<8}"
            f"{fmt_abs_max(row):<12}{(row.get('duration_s') or 0):<8.1f}{detail}"
        )


def exit_status(rows: list[dict]) -> int:
    bad = [row for row in rows if row["passed"] == row["expected_fail"]]
    return 1 if bad else 0


def scope_of(args: argparse.Namespace) -> Scope:
    if args.quick:
        return Scope.QUICK
    return Scope.ALL if args.group is None else Scope(args.group)


def main() -> None:
    names = [row.name for row in ROWS]
    parser = argparse.ArgumentParser(description=__doc__)
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument("--quick", action="store_true")
    scope.add_argument("--group", choices=[group.value for group in GROUPS])
    parser.add_argument(
        "--expect-fail", action="append", default=[], choices=names
    )
    parser.add_argument(
        "--tree",
        type=Path,
        help="worktree to sync to the Pi instead of this one",
    )
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    selected = selected_rows(scope_of(args))
    unselected = sorted(set(args.expect_fail) - {row.name for row in selected})
    if unselected:
        parser.error(f"--expect-fail rows not in this suite: {unselected}")
    suite_dir = SUITES_DIR / datetime.now().strftime("%Y%m%d-%H%M%S")
    build_local_engine()
    suite_dir.mkdir(parents=True)
    with lock.bench_lock(args.lock_timeout, command=" ".join(sys.argv)):
        tree = [] if args.tree is None else ["--tree", str(args.tree)]
        sync = python("deploy.py", "sync", *tree, timeout_s=1800)
        if sync.returncode != 0:
            sys.exit(f"sync failed: {last_lines(sync.stderr, 8)}")
        synced = pi.read_deployed()
        benchmark = query_step_benchmark()
        rows = []
        aborted = None
        for row in selected:
            aborted = pi.foreign_deploy(synced)
            if aborted is not None:
                break
            print(f"running {row.name} ...", flush=True)
            result = run_row(row, suite_dir, args.lock_timeout)
            result["expected_fail"] = row.name in args.expect_fail
            rows.append(result)
        aborted = aborted or pi.foreign_deploy(synced)
    summary = {
        "quick": args.quick,
        "group": args.group,
        "expect_fail": args.expect_fail,
        "suite_dir": str(suite_dir),
        "rows": rows,
        "step_benchmark": benchmark,
        "aborted": aborted,
    }
    (suite_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print_table(rows)
    print(
        f"step benchmark: {benchmark['pulsed_capacity_steps_per_s']:.0f} "
        f"steps/s pulsed = {benchmark['ratio_to_single_motor_ceiling']:.2f} x "
        f"the single-motor ceiling, band {BENCHMARK_BAND}: "
        + ("ok" if benchmark["within_band"] else "OUT OF BAND")
    )
    print(f"summary: {suite_dir / 'summary.json'}")
    if aborted is not None:
        sys.exit(f"suite aborted, its rows are void: {aborted}")
    sys.exit(exit_status(rows) or int(not benchmark["within_band"]))


if __name__ == "__main__":
    main()
