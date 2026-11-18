"""Run the bench regression matrix as a sequence of bench holds.

uv run python tools/bench/suite.py [--quick | --group {matrix,session,soak}
    | --row NAME ...] [--expect-fail NAME ...] [--tree DIR]

Plans the rows into holds that fit lock.MAX_HOLD_S (hold_plan.py) and runs
each hold in a child process (--hold) that takes the bench lock, runs its
rows' hardware part and releases the lock, so other holders interleave
between holds. A hold starts by checking the Pi still runs this suite's
sync of the tree (deploy record) and the boards the firmware the first
hold found; it re-syncs the tree, or reflashes the F401 from the tree, when
another holder changed them, and records that under `restored`. It then
settles the bench (deploy.settle: no bench drop-in, no cpufreq cap, no
orphaned Pi job, printer.cfg loaded, klippy ready and idle), and settles it
again after its last row, recorded under `left`, so the next holder finds
printer.cfg however the last row's config differed. A row starts
only when its expected seconds (hold_plan.EXPECTED_ROW_S) and the margin
fit the hold's remaining time; the rest move to the next hold. Each row is
judged after its hold, outside the lock: matrix rows are one print each
(run.py + analyze.py), session rows one klippy session each (session.py +
session_check.py). The full suite runs both groups. The soak group
(qualification's, never in the full suite) runs the generated soak prints
(soak_gcode.py): a mesh session with a pause and a second print, and the
longest print a hold fits on a throttled host, judged by the analyzer, both
also by the run-wide checks (session_check.py). --row runs named rows of
any group; --tree syncs DIR and analyzes with DIR's analyzer and engine.
Writes ~/bench/suites/<timestamp>/summary.json (rows, and holds with their
lock wait and held seconds). Exit status is non-zero when any row fails
(unless named in --expect-fail) or when an --expect-fail row passes.
"""

from __future__ import annotations

import argparse
import enum
import json
import os
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import capture
import crawl_gcode
import deploy
import firmware_build_id
import hold_plan
import lock
import pi
import session_check
import soak_gcode
from session_scenarios import SCENARIOS, SOAK_SCENARIOS, Scenario

BENCH = Path(__file__).resolve().parent
REPO = BENCH.parents[1]
SUITES_DIR = Path("~/bench/suites").expanduser()
PRINTER_CFG = BENCH / "printer.cfg"
FAST_CFG = BENCH / "configs" / "fast.cfg"
REST_LADDER_CFG = BENCH / "configs" / "rest_ladder.cfg"
CAPACITY_CFG = BENCH / "configs" / "capacity.cfg"
SIM_GCODE = REPO / "tools" / "sim" / "gcode"
RUN_TIMEOUT_S = 2400.0
RUN_SETUP_S = 600.0
ANALYZE_TIMEOUT_S = 1200.0
ENGINE_BUILD_TIMEOUT_S = 3600
ENGINE_BUILD = ["make", "-f", "Makefile.rust", "motion-engine-fast"]
THROTTLE_ARGS = ["--host-cpus", "1", "--host-max-mhz", "1500"]
SOAK_THROTTLE_ARGS = ["--host-cpus", "2", "--host-max-mhz", "1500"]
SOAK_PRINT_TIMEOUT_S = 2 * soak_gcode.LONG_PRINT.planned_print_s
SYNC_TIMEOUT_S = 1800
FLASH_TIMEOUT_S = 1800
RECOVERY_TIMEOUT_S = 600
HOLDS_DIR = "holds"
STALLED_HOLDS = 2
SOAK_ANALYZE_TIMEOUT_S = 3600.0
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
    run_wide_checks: bool = False
    timeout_s: float = RUN_TIMEOUT_S
    analyze_timeout_s: float = ANALYZE_TIMEOUT_S
    fails_until: str = ""


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
SOAK_MATRIX = [
    Row(
        "soak_throttled",
        soak_gcode.LONG_PRINT.path,
        throttle=SOAK_THROTTLE_ARGS,
        run_args=["--timeout", str(SOAK_PRINT_TIMEOUT_S)],
        run_wide_checks=True,
        timeout_s=SOAK_PRINT_TIMEOUT_S + RUN_SETUP_S,
        analyze_timeout_s=SOAK_ANALYZE_TIMEOUT_S,
    ),
]
CRAWL_FAILS_UNTIL = {"crawl_s20": "#1003", "mid_crawl_s5_300s": "#1007"}
CRAWL_MATRIX = [
    Row(
        spec.name,
        spec.path,
        run_wide_checks=True,
        fails_until=CRAWL_FAILS_UNTIL.get(spec.name, ""),
    )
    for spec in (*crawl_gcode.CRAWL_PRINTS, *crawl_gcode.MID_PRINTS)
]


@dataclass(frozen=True)
class SessionRow:
    scenario: Scenario

    @property
    def name(self) -> str:
        return self.scenario.name

    @property
    def timeout_s(self) -> float:
        return self.scenario.timeline_timeout_s + RUN_SETUP_S


class Scope(enum.Enum):
    ALL = "all"
    QUICK = "quick"
    MATRIX = "matrix"
    SESSION = "session"
    SOAK = "soak"
    CRAWL = "crawl"


GROUPS = (Scope.MATRIX, Scope.SESSION, Scope.SOAK, Scope.CRAWL)
SESSION_ROWS = [SessionRow(scenario) for scenario in SCENARIOS]
SOAK_ROWS = [
    *(SessionRow(scenario) for scenario in SOAK_SCENARIOS),
    *SOAK_MATRIX,
]
ROWS = [*MATRIX, *SESSION_ROWS]
ROW_BY_NAME = {row.name: row for row in [*ROWS, *SOAK_ROWS, *CRAWL_MATRIX]}


def selected_rows(scope: Scope) -> list:
    if scope is Scope.QUICK:
        return [row for row in MATRIX if row.quick]
    rows = {
        Scope.ALL: ROWS,
        Scope.MATRIX: MATRIX,
        Scope.SESSION: SESSION_ROWS,
        Scope.SOAK: SOAK_ROWS,
        Scope.CRAWL: CRAWL_MATRIX,
    }
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


def build_local_engine(tree: Path) -> None:
    build = subprocess.run(
        ENGINE_BUILD,
        cwd=tree,
        capture_output=True,
        text=True,
        timeout=ENGINE_BUILD_TIMEOUT_S,
    )
    if build.returncode != 0:
        raise SuiteError(
            f"`{' '.join(ENGINE_BUILD)}` in {tree} exited "
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


def run_hardware(
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
    run = python(*runner, timeout_s=row.timeout_s)
    run_dirs = sorted(runs_dir.glob("*")) if runs_dir.exists() else []
    if run_dirs:
        result["run_dir"] = str(run_dirs[-1])
    if run.returncode != 0:
        result["error"] = f"{runner[0]}: {last_lines(run.stderr)}"
    if run.returncode not in (0, pi.DRIVER_SIDE_EXIT):
        result["recovered_with_reset_target"] = True
        recovery = python(
            "deploy.py", "reset-target", timeout_s=RECOVERY_TIMEOUT_S
        )
        if recovery.returncode != 0:
            result["error"] += f"; reset-target: {last_lines(recovery.stderr)}"
    if result["run_dir"]:
        info = read_json(Path(result["run_dir"]) / "run.json")
        result["print_state"] = info.get("print_state")
        result["print_message"] = info.get("print_message")
        result["duration_s"] = info.get("duration_s")
        result["deployed"] = info.get("deployed")
        result["host_throttle"] = info.get("host_throttle")
    result["held_s"] = time.monotonic() - started
    return result


def row_fails_until(row: Row | SessionRow) -> str:
    return row.fails_until if isinstance(row, Row) else ""


def judge_row(row: Row | SessionRow, result: dict, analyzer: Path) -> dict:
    started = time.monotonic()
    if result["run_dir"] and result.get("killed_at_cap") is None:
        run_dir = Path(result["run_dir"])
        if isinstance(row, SessionRow):
            judge_session(run_dir, result)
        else:
            judge_capture(row, run_dir, result, analyzer)
        if isinstance(row, Row) and row.run_wide_checks:
            judge_run_wide(run_dir, result)
    result["judge_s"] = time.monotonic() - started
    result["wall_s"] = result["held_s"] + result["judge_s"]
    result["passed"] = row_passed(row, result)
    return result


def add_error(result: dict, error: str) -> None:
    result["error"] = "; ".join(filter(None, [result["error"], error]))


def judge_capture(
    row: Row, run_dir: Path, result: dict, analyzer: Path
) -> None:
    if (run_dir / "capture.bin").exists() or (run_dir / "events.bin").exists():
        analysis = python(
            str(analyzer / "analyze.py"),
            str(run_dir),
            *row.analyze_args,
            timeout_s=row.analyze_timeout_s,
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
    result["checks"] = verdict["checks"]


def judge_run_wide(run_dir: Path, result: dict) -> None:
    check = python(
        "session_check.py",
        "--run-wide",
        str(run_dir),
        timeout_s=ANALYZE_TIMEOUT_S,
    )
    verdict = read_json(run_dir / session_check.CHECKS_FILE)
    if not verdict:
        add_error(result, f"session_check.py: {last_lines(check.stderr)}")
        return
    result["run_wide_ok"] = verdict["ok"]
    result["run_wide_checks"] = verdict["checks"]
    result["failed_checks"] = [
        *result.get("failed_checks", []),
        *(
            f"{c['name']}: {c['detail']}"
            for c in verdict["checks"]
            if not c["ok"]
        ),
    ]


def row_passed(row: Row | SessionRow, result: dict) -> bool:
    if result["error"] is not None:
        return False
    if isinstance(row, SessionRow):
        return bool(result.get("session_ok"))
    run_wide_ok = not row.run_wide_checks or bool(result.get("run_wide_ok"))
    return (
        result.get("print_state") == "complete"
        and bool(result.get("analyzer_ok"))
        and run_wide_ok
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
        if verdict == "XPASS":
            reason = row["fails_until"] or "--expect-fail"
            detail = f"passes: drop its expected failure ({reason})"
        print(
            f"{row['name']:<26}{verdict:<8}{str(row.get('print_state')):<11}"
            f"{str(row.get('analyzer_ok', row.get('session_ok'))):<6}"
            f"{str(row.get('stalls')):<8}"
            f"{fmt_abs_max(row):<12}{(row.get('duration_s') or 0):<8.1f}{detail}"
        )


def print_holds(holds: list[dict]) -> None:
    for hold in holds:
        undone = [*hold["restored"], *hold["settled"]]
        print(
            f"hold {hold['index']}: wait {hold['wait_s']} s, held "
            f"{hold['held_s']} s, exit {hold['exit']}, rows "
            f"{', '.join(hold['rows']) or '-'}"
            + (
                f"; deferred {', '.join(hold['deferred'])}"
                if hold["deferred"]
                else ""
            )
            + (f"; restored: {' | '.join(undone)}" if undone else "")
        )


def print_benchmark(benchmark: dict | None) -> None:
    if benchmark is None:
        print("step benchmark: not measured")
        return
    print(
        f"step benchmark: {benchmark['pulsed_capacity_steps_per_s']:.0f} "
        f"steps/s pulsed = {benchmark['ratio_to_single_motor_ceiling']:.2f} x "
        f"the single-motor ceiling, band {BENCHMARK_BAND}: "
        + ("ok" if benchmark["within_band"] else "OUT OF BAND")
    )


def restore_tree(tree: Path, deployed: dict | None) -> tuple[dict, list[str]]:
    foreign = None if deployed is None else pi.foreign_deploy(deployed)
    if deployed is not None and foreign is None:
        return deployed, []
    sync = python(
        "deploy.py", "sync", "--tree", str(tree), timeout_s=SYNC_TIMEOUT_S
    )
    if sync.returncode != 0:
        raise SuiteError(f"sync failed: {last_lines(sync.stderr, 8)}")
    restored = [] if foreign is None else [f"re-synced {tree}: {foreign}"]
    return pi.read_deployed(), restored


def board_firmware() -> dict:
    return {"target": pi.mcu_version(), "capture": capture.identity().build_id}


def check_restorable(tree: Path, ran_on: dict, found: dict) -> None:
    if found["capture"] != ran_on["capture"]:
        raise SuiteError(
            f"the capture board reports firmware {found['capture']}, not "
            f"{ran_on['capture']} this suite measured with: another holder "
            "reflashed the instrument"
        )
    build_id = firmware_build_id.target_build_id(
        tree, tree / deploy.TARGET_CONFIG
    )
    if not ran_on["target"].endswith(build_id):
        raise SuiteError(
            f"the F401 reports {found['target']!r}, not {ran_on['target']!r} "
            f"this suite ran on, which is no build of {tree}: it cannot be "
            "restored"
        )


def restore_target(tree: Path, ran_on: dict, found: dict) -> str:
    flash = python(
        "deploy.py",
        "ensure-target",
        "--tree",
        str(tree),
        timeout_s=FLASH_TIMEOUT_S,
    )
    if flash.returncode != 0:
        raise SuiteError(f"ensure-target failed: {last_lines(flash.stderr, 8)}")
    return f"F401 reflashed from {found['target']!r} to {ran_on['target']!r}"


Save = Callable[[], None]


def held(plan: dict, result: dict, save: Save) -> None:
    tree = Path(plan["tree"])
    result["deployed"], result["restored"] = restore_tree(
        tree, plan["deployed"]
    )
    save()
    found = board_firmware()
    ran_on = plan["firmware"] or found
    result["firmware"] = ran_on
    if found != ran_on:
        check_restorable(tree, ran_on, found)
        if not plan["target_built"]:
            result["needs_target_build"] = True
            result["deferred"] = plan["rows"]
            return
        result["restored"].append(restore_target(tree, ran_on, found))
    result["settled"] = deploy.settle(PRINTER_CFG)
    result["step_benchmark"] = query_step_benchmark()
    save()
    run_rows(plan, result, save)
    result["left"] = deploy.settle(PRINTER_CFG)


def run_rows(plan: dict, result: dict, save: Save) -> None:
    rows = plan["rows"]
    for position, name in enumerate(rows):
        if not hold_plan.fits(name, lock.remaining_s()):
            result["deferred"] = rows[position:]
            return
        result["aborted"] = pi.foreign_deploy(result["deployed"])
        if result["aborted"] is not None:
            return
        result.update(running=name, running_since=time.time())
        save()
        print(f"running {name} ...", flush=True)
        result["rows"].append(
            run_hardware(
                ROW_BY_NAME[name], Path(plan["suite_dir"]), plan["lock_timeout"]
            )
        )
        result["running"] = None
        save()
    result["aborted"] = pi.foreign_deploy(result["deployed"])


def hold_main(plan_path: Path) -> None:
    plan = read_json(plan_path)
    result_path = Path(plan["result"])
    result: dict = {
        "index": plan["index"],
        "asked": time.time(),
        "acquired": None,
        "released": None,
        "deployed": plan["deployed"],
        "firmware": plan["firmware"],
        "restored": [],
        "settled": [],
        "left": [],
        "step_benchmark": None,
        "rows": [],
        "running": None,
        "deferred": [],
        "needs_target_build": False,
        "aborted": None,
    }

    def save() -> None:
        result_path.write_text(json.dumps(result, indent=2))

    save()
    try:
        with lock.bench_lock(plan["lock_timeout"], command=" ".join(sys.argv)):
            result["acquired"] = time.time()
            save()
            held(plan, result, save)
            result["released"] = time.time()
    except (SuiteError, pi.BenchError) as exc:
        result["aborted"] = str(exc)
    save()
    sys.exit(1 if result["aborted"] else 0)


def hold_record(result: dict, planned: list[str]) -> dict:
    acquired = result.get("acquired")
    released = result.get("released") or result["ended"]
    running = [result["running"]] if result.get("running") else []
    return {
        "index": result["index"],
        "planned": planned,
        "rows": [row["name"] for row in result.get("rows", [])] + running,
        "deferred": result.get("deferred", []),
        "wait_s": None
        if acquired is None
        else round(acquired - result["asked"], 1),
        "held_s": 0.0 if acquired is None else round(released - acquired, 1),
        "restored": result.get("restored", []),
        "settled": result.get("settled", []),
        "left": result.get("left", []),
        "deployed_head": (result.get("deployed") or {}).get("head"),
        "firmware": result.get("firmware"),
        "exit": result["exit"],
        "aborted": result.get("aborted"),
    }


class SuiteRun:
    """The holds of one suite, taken one after another; between them the
    lock is free and the finished rows are judged."""

    def __init__(self, tree: Path, suite_dir: Path, args: argparse.Namespace):
        self.tree = tree
        self.suite_dir = suite_dir
        self.analyzer = tree / BENCH.relative_to(REPO)
        self.lock_timeout_s = args.lock_timeout
        self.expect_fail = set(args.expect_fail)
        self.deployed: dict | None = None
        self.firmware: dict | None = None
        self.target_built = False
        self.holds: list[dict] = []
        self.rows: list[dict] = []
        self.benchmarks: list[dict] = []
        self.aborted: str | None = None

    def run(self, plan: list[list[str]]) -> None:
        pending = [list(rows) for rows in plan]
        stalled = 0
        while pending and self.aborted is None:
            planned = pending.pop(0)
            result = self.take_hold(planned)
            left = self.absorb(result, planned)
            if result.get("needs_target_build"):
                self.build_target()
            progressed = result.get("rows") or result.get("needs_target_build")
            stalled = 0 if progressed else stalled + 1
            if stalled >= STALLED_HOLDS:
                self.aborted = f"{stalled} holds in a row ran no row of {left}"
            if left:
                pending.insert(0, left)

    def take_hold(self, rows: list[str]) -> dict:
        index = len(self.holds) + 1
        holds_dir = self.suite_dir / HOLDS_DIR
        holds_dir.mkdir(exist_ok=True)
        plan_path = holds_dir / f"{index}-plan.json"
        result_path = holds_dir / f"{index}.json"
        plan = {
            "index": index,
            "rows": rows,
            "tree": str(self.tree),
            "suite_dir": str(self.suite_dir),
            "lock_timeout": self.lock_timeout_s,
            "deployed": self.deployed,
            "firmware": self.firmware,
            "target_built": self.target_built,
            "result": str(result_path),
        }
        plan_path.write_text(json.dumps(plan, indent=2))
        print(f"hold {index}: {', '.join(rows)}", flush=True)
        child = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--hold",
                str(plan_path),
            ]
        )
        return {
            "index": index,
            **read_json(result_path),
            "exit": child.returncode,
            "ended": time.time(),
        }

    def absorb(self, result: dict, planned: list[str]) -> list[str]:
        """Judge the hold's rows, outside the lock; the rows it left."""
        self.deployed = result.get("deployed") or self.deployed
        self.firmware = result.get("firmware") or self.firmware
        if result.get("step_benchmark"):
            self.benchmarks.append(result["step_benchmark"])
        for row in result.get("rows", []):
            self.rows.append(self.judged(row))
        killed = result["exit"] == lock.HOLD_EXPIRED_EXIT
        if killed and result.get("running"):
            self.rows.append(self.judged(killed_row(result, self.suite_dir)))
        if result.get("aborted") or result["exit"] not in (
            0,
            lock.HOLD_EXPIRED_EXIT,
        ):
            self.aborted = result.get("aborted") or (
                f"hold {result['index']} exited {result['exit']}"
            )
        self.holds.append(hold_record(result, planned))
        done = set(self.holds[-1]["rows"])
        return [name for name in planned if name not in done]

    def judged(self, row: dict) -> dict:
        definition = ROW_BY_NAME[row["name"]]
        judge_row(definition, row, self.analyzer)
        marked = row_fails_until(definition)
        row["fails_until"] = marked
        row["expected_fail"] = bool(marked) or row["name"] in self.expect_fail
        return row

    def build_target(self) -> None:
        build = python(
            "deploy.py",
            "build-target",
            "--tree",
            str(self.tree),
            timeout_s=ENGINE_BUILD_TIMEOUT_S,
        )
        if build.returncode != 0:
            self.aborted = f"build-target failed: {last_lines(build.stderr, 8)}"
        self.target_built = True

    def benchmark(self) -> dict | None:
        out_of_band = [b for b in self.benchmarks if not b["within_band"]]
        return next(iter(out_of_band or self.benchmarks), None)


def killed_row(result: dict, suite_dir: Path) -> dict:
    name = result["running"]
    runs_dir = suite_dir / name
    run_dirs = sorted(runs_dir.glob("*")) if runs_dir.exists() else []
    return {
        "name": name,
        **row_identity(ROW_BY_NAME[name]),
        "run_dir": str(run_dirs[-1]) if run_dirs else None,
        "error": (
            f"the hold reached lock.MAX_HOLD_S = {lock.MAX_HOLD_S:.0f} s "
            "during this row and was killed"
        ),
        "killed_at_cap": True,
        "held_s": result["ended"] - result["running_since"],
    }


def exit_status(rows: list[dict]) -> int:
    bad = [row for row in rows if row["passed"] == row["expected_fail"]]
    return 1 if bad else 0


def scope_of(args: argparse.Namespace) -> Scope:
    if args.quick:
        return Scope.QUICK
    return Scope.ALL if args.group is None else Scope(args.group)


def parse_args() -> argparse.Namespace:
    names = list(ROW_BY_NAME)
    parser = argparse.ArgumentParser(description=__doc__)
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument("--quick", action="store_true")
    scope.add_argument("--group", choices=[group.value for group in GROUPS])
    scope.add_argument("--row", action="append", choices=names)
    parser.add_argument(
        "--expect-fail", action="append", default=[], choices=names
    )
    parser.add_argument(
        "--tree",
        type=Path,
        help="worktree to sync to the Pi and analyze with instead of this one",
    )
    parser.add_argument("--hold", type=Path, help=argparse.SUPPRESS)
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    if args.hold is not None:
        return args
    selected = (
        [ROW_BY_NAME[name] for name in args.row]
        if args.row
        else selected_rows(scope_of(args))
    )
    unselected = sorted(set(args.expect_fail) - {row.name for row in selected})
    if unselected:
        parser.error(f"--expect-fail rows not in this suite: {unselected}")
    args.selected = selected
    return args


def main() -> None:
    args = parse_args()
    if args.hold is not None:
        hold_main(args.hold)
        return
    if lock.TOKEN_ENV in os.environ:
        sys.exit(
            "suite.py takes and releases the bench lock once per hold; it "
            "cannot run inside lock.py hold or another holder's tree"
        )
    plan = hold_plan.plan([row.name for row in args.selected])
    tree = REPO if args.tree is None else args.tree.resolve()
    if any(row in SOAK_ROWS for row in args.selected):
        soak_gcode.ensure_all()
    if any(row in CRAWL_MATRIX for row in args.selected):
        crawl_gcode.ensure_all()
    suite_dir = SUITES_DIR / datetime.now().strftime("%Y%m%d-%H%M%S")
    build_local_engine(tree)
    suite_dir.mkdir(parents=True)
    print(
        f"plan: {len(plan)} hold(s) of at most {hold_plan.HOLD_ROWS_S:.0f} s "
        "of rows each: "
        + " | ".join(
            f"{', '.join(rows)} ({hold_plan.planned_s(rows):.0f} s)"
            for rows in plan
        ),
        flush=True,
    )
    suite = SuiteRun(tree, suite_dir, args)
    suite.run(plan)
    benchmark = suite.benchmark()
    summary = {
        "quick": args.quick,
        "group": args.group,
        "expect_fail": args.expect_fail,
        "suite_dir": str(suite_dir),
        "rows": suite.rows,
        "step_benchmark": benchmark,
        "aborted": suite.aborted,
        "plan": plan,
        "holds": suite.holds,
    }
    (suite_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print_table(suite.rows)
    print_holds(suite.holds)
    print_benchmark(benchmark)
    print(f"summary: {suite_dir / 'summary.json'}")
    if suite.aborted is not None:
        sys.exit(f"suite aborted, its rows are void: {suite.aborted}")
    in_band = benchmark is not None and benchmark["within_band"]
    sys.exit(exit_status(suite.rows) or int(not in_band))


if __name__ == "__main__":
    main()
