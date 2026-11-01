"""Run the bench regression matrix under one lock hold.

uv run python tools/bench/suite.py [--quick] [--expect-fail NAME ...]

Syncs the tree, runs every row with run.py + analyze.py, prints a table and
writes ~/bench/suites/<timestamp>/summary.json. Exit status is non-zero when
any row fails (unless named in --expect-fail) or when an --expect-fail row
passes.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import lock

BENCH = Path(__file__).resolve().parent
REPO = BENCH.parents[1]
SUITES_DIR = Path("~/bench/suites").expanduser()
PRINTER_CFG = BENCH / "printer.cfg"
FAST_CFG = BENCH / "configs" / "fast.cfg"
SIM_GCODE = REPO / "tools" / "sim" / "gcode"
RUN_TIMEOUT_S = 2400.0
ANALYZE_TIMEOUT_S = 1200.0
THROTTLE_ARGS = ["--host-cpus", "1", "--host-max-mhz", "1500"]


@dataclass(frozen=True)
class Row:
    name: str
    gcode: Path
    config: Path = PRINTER_CFG
    throttle: list[str] = field(default_factory=list)
    quick: bool = False


MATRIX = [
    Row("first_layers", SIM_GCODE / "voron_first_layers.gcode", quick=True),
    Row("speed_ladder", BENCH / "gcode" / "speed_ladder.gcode", quick=True),
    Row("speed_ladder_fast", BENCH / "gcode" / "speed_ladder.gcode", FAST_CFG),
    Row("extrude_ladder", BENCH / "gcode" / "extrude_ladder.gcode"),
    Row("epochs", BENCH / "gcode" / "epochs.gcode", quick=True),
    Row("dense_top", SIM_GCODE / "voron_dense_top_layers.gcode"),
    Row(
        "dense_top_fast_throttled",
        SIM_GCODE / "voron_dense_top_layers.gcode",
        FAST_CFG,
        THROTTLE_ARGS,
    ),
]


def python(
    script: str, *args: str, timeout_s: float
) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(BENCH / script), *args],
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )


def last_lines(text: str, count: int = 3) -> str:
    return " | ".join(line for line in text.strip().splitlines()[-count:])


def read_json(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


def run_row(row: Row, suite_dir: Path, lock_timeout_s: float) -> dict:
    runs_dir = suite_dir / row.name
    result: dict = {
        "name": row.name,
        "gcode": str(row.gcode.relative_to(REPO)),
        "config": row.config.name,
        "throttle": row.throttle,
        "run_dir": None,
        "error": None,
    }
    started = time.monotonic()
    run = python(
        "run.py",
        str(row.gcode),
        "--config",
        str(row.config),
        "--runs-dir",
        str(runs_dir),
        "--lock-timeout",
        str(lock_timeout_s),
        *row.throttle,
        timeout_s=RUN_TIMEOUT_S,
    )
    run_dirs = sorted(runs_dir.glob("*")) if runs_dir.exists() else []
    if run_dirs:
        result["run_dir"] = str(run_dirs[-1])
    if run.returncode != 0:
        result["error"] = f"run.py: {last_lines(run.stderr)}"
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
        if (run_dir / "capture.bin").exists():
            analysis = python(
                "analyze.py", str(run_dir), timeout_s=ANALYZE_TIMEOUT_S
            )
            if analysis.returncode != 0:
                joined = f"analyze.py: {last_lines(analysis.stderr)}"
                result["error"] = "; ".join(
                    filter(None, [result["error"], joined])
                )
        verdict = read_json(run_dir / "analysis.json")
        if verdict:
            result["analyzer_ok"] = verdict["ok"]
            result["failed_checks"] = [
                name for name, ok in verdict["checks"].items() if not ok
            ]
            result["stalls"] = verdict["stalls"]["count"]
            result["abs_max_us"] = {
                role: (
                    motor["timing_error_capture_minus_reference_us"] or {}
                ).get("abs_max")
                for role, motor in verdict["motors"].items()
            }
    result["wall_s"] = time.monotonic() - started
    result["passed"] = bool(
        result["error"] is None
        and result.get("print_state") == "complete"
        and result.get("analyzer_ok")
    )
    return result


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
            f"{str(row.get('analyzer_ok')):<6}{str(row.get('stalls')):<8}"
            f"{fmt_abs_max(row):<12}{(row.get('duration_s') or 0):<8.1f}{detail}"
        )


def exit_status(rows: list[dict]) -> int:
    bad = [row for row in rows if row["passed"] == row["expected_fail"]]
    return 1 if bad else 0


def main() -> None:
    names = [row.name for row in MATRIX]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument(
        "--expect-fail", action="append", default=[], choices=names
    )
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    selected = [row for row in MATRIX if row.quick or not args.quick]
    unselected = sorted(set(args.expect_fail) - {row.name for row in selected})
    if unselected:
        parser.error(f"--expect-fail rows not in this suite: {unselected}")
    suite_dir = SUITES_DIR / datetime.now().strftime("%Y%m%d-%H%M%S")
    suite_dir.mkdir(parents=True)
    with lock.bench_lock(args.lock_timeout, command=" ".join(sys.argv)):
        os.environ[lock.TOKEN_ENV] = str(os.getpid())
        sync = python("deploy.py", "sync", timeout_s=1800)
        if sync.returncode != 0:
            sys.exit(f"sync failed: {last_lines(sync.stderr, 8)}")
        rows = []
        for row in selected:
            print(f"running {row.name} ...", flush=True)
            result = run_row(row, suite_dir, args.lock_timeout)
            result["expected_fail"] = row.name in args.expect_fail
            rows.append(result)
    summary = {
        "quick": args.quick,
        "expect_fail": args.expect_fail,
        "suite_dir": str(suite_dir),
        "rows": rows,
    }
    (suite_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print_table(rows)
    print(f"summary: {suite_dir / 'summary.json'}")
    sys.exit(exit_status(rows))


if __name__ == "__main__":
    main()
