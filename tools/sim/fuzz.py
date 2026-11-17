#!/usr/bin/env python3
"""Seeded full-stack scenario fuzzer (tools/sim/fuzzing).

Run through tools/sim/run.sh, which mounts .sim-fuzz/ for the results:

    tools/sim/run.sh fuzz --seeds 1000..1999 [--jobs 8] [--budget-s 3600] [--minimize]
    tools/sim/run.sh fuzz --seed 1234                 # replay one seed
    tools/sim/run.sh fuzz --replay .sim-fuzz/<run>/seed-1234/minimized

One line per scenario (PASS, or FAIL and the violated check). A failure
keeps its directory: the scenario (scenario.json, timeline.txt and the
rendered printer.cfg / print_*.gcode), the world's logs, result.json and
repro.txt.
"""

from __future__ import annotations

import argparse
import collections
import concurrent.futures
import json
import os
import pathlib
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from tools.sim.fuzzing import execute, minimize, scenario  # noqa: E402
from tools.sim.fuzzing.checks import ROOT_CAUSES, Check  # noqa: E402
from tools.sim.fuzzing.execute import FAIL, PASS  # noqa: E402

DEFAULT_JOBS = 8
WORKER_TIMEOUT_S = 900
STUCK_GRACE_S = 120
RESULT_FILE = "result.json"
REPRO_FILE = "repro.txt"
LEDGER_FILE = "ledger.jsonl"
SUMMARY_FILE = "summary.txt"
MINIMIZED_DIR = "minimized"


def _seed_range(text: str) -> range:
    low, _, high = text.partition("..")
    return range(int(low), int(high) + 1)


def _parse(argv) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    which = parser.add_mutually_exclusive_group(required=True)
    which.add_argument("--seeds", type=_seed_range, help="inclusive range A..B")
    which.add_argument(
        "--seed",
        type=lambda text: [int(s) for s in text.split(",")],
        help="one seed or a comma list",
    )
    which.add_argument(
        "--replay", type=pathlib.Path, help="a saved scenario directory"
    )
    which.add_argument("--worker", type=pathlib.Path, help=argparse.SUPPRESS)
    parser.add_argument(
        "--jobs",
        type=int,
        default=int(os.environ.get("SIM_FUZZ_JOBS", DEFAULT_JOBS)),
    )
    parser.add_argument("--budget-s", type=float, default=None)
    parser.add_argument(
        "--repeat", type=int, default=1, help="run each scenario N times"
    )
    parser.add_argument("--minimize", action="store_true")
    parser.add_argument("--keep-passing", action="store_true")
    parser.add_argument(
        "--run-name",
        help="fixed run directory under --out-root; a rerun skips the "
        "scenarios its ledger already records",
    )
    parser.add_argument(
        "--step-tap",
        action="store_true",
        help="record every step frame klippy sends (SERVAL_STEP_TAP_DIR)",
    )
    parser.add_argument(
        "--out-root", type=pathlib.Path, default=REPO_ROOT / ".sim-fuzz"
    )
    return parser.parse_args(argv)


def _stuck(signum, frame) -> None:
    raise execute.Stuck(f"worker over its {WORKER_TIMEOUT_S} s wall budget")


def run_worker(directory: pathlib.Path) -> dict:
    signal.signal(signal.SIGTERM, _stuck)
    chosen = scenario.load(directory)
    with tempfile.TemporaryDirectory(prefix=f"fz{chosen.seed}-") as tmp:
        return execute.run_scenario(chosen, pathlib.Path(tmp), directory)


# ------------------------------------------------------------- parent


def _spawn(directory: pathlib.Path) -> dict:
    command = [
        sys.executable,
        str(pathlib.Path(__file__).resolve()),
        "--worker",
        str(directory),
    ]
    log = (directory / "worker.log").open("wb")
    process = subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
    )
    try:
        process.wait(timeout=WORKER_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=STUCK_GRACE_S)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    log.close()
    result_path = directory / RESULT_FILE
    if result_path.exists():
        return json.loads(result_path.read_text())
    tail = (directory / "worker.log").read_text(errors="replace")[-2000:]
    return {
        "seed": scenario.load(directory).seed,
        "status": FAIL,
        "check": Check.HARNESS.value,
        "signature": f"{Check.HARNESS.value}: worker died (rc={process.returncode})",
        "evidence": tail,
        "violations": [],
        "notes": [],
        "real_s": WORKER_TIMEOUT_S,
        "virtual_s": 0.0,
        "summary": "",
    }


def _line(result: dict) -> str:
    head = f"seed {result['seed']} {result['status']}"
    timing = f"{result['real_s']:.1f}s {result['virtual_s']:.1f}v"
    if result["status"] == PASS:
        return f"{head} {timing} {result['summary']}"
    return f"{head} {result['check']} {timing} {result['signature']}"


def _repro(
    directory: pathlib.Path, out_root: pathlib.Path, seed_mode: bool, seed: int
) -> str:
    if seed_mode:
        return f"tools/sim/run.sh fuzz --seed {seed}"
    relative = directory.resolve().relative_to(out_root.resolve().parent)
    return f"tools/sim/run.sh fuzz --replay {relative}"


class Runner:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        stamp = time.strftime("%Y%m%d-%H%M%S")
        name = args.run_name or f"run-{stamp}-{os.getpid()}"
        self.run_dir = args.out_root / name
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.ledger = self.run_dir / LEDGER_FILE
        self.ledger_lock = threading.Lock()
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs)

    def recorded(self) -> dict[str, dict]:
        """Results already in this run's ledger, by scenario directory: a
        resumed run skips them."""
        if not self.ledger.exists():
            return {}
        lines = self.ledger.read_text().splitlines()
        return {r["dir"]: r for r in map(json.loads, lines) if r.get("dir")}

    def _record(self, result: dict) -> None:
        with self.ledger_lock, self.ledger.open("a") as stream:
            stream.write(json.dumps(result) + "\n")

    def evaluate(self, directory: pathlib.Path, seed_mode: bool) -> dict:
        result = _spawn(directory)
        result["dir"] = str(directory.relative_to(self.run_dir))
        (directory / RESULT_FILE).write_text(
            json.dumps(result, indent=1) + "\n"
        )
        if result["status"] == FAIL:
            repro = _repro(
                directory, self.args.out_root, seed_mode, result["seed"]
            )
            (directory / REPRO_FILE).write_text(repro + "\n")
        elif not (
            self.args.keep_passing
            or self.args.seed is not None
            or self.args.replay
        ):
            shutil.rmtree(directory, ignore_errors=True)
        return result

    def _prepare(self, chosen: scenario.Scenario, name: str) -> pathlib.Path:
        directory = self.run_dir / name
        scenario.save(chosen, directory)
        return directory

    def run_seeds(self, seeds) -> list[dict]:
        started = time.monotonic()
        done = self.recorded()
        results = []
        futures = []
        for seed in seeds:
            for attempt in range(self.args.repeat):
                name = self._name(f"seed-{seed}", attempt)
                if name in done:
                    results.append(done[name])
                    continue
                if (
                    self.args.budget_s
                    and time.monotonic() - started > self.args.budget_s
                ):
                    return results + [future.result() for future in futures]
                futures.append(self.pool.submit(self._seed, seed, name))
                while sum(not f.done() for f in futures) >= self.args.jobs:
                    time.sleep(0.2)
        return results + [future.result() for future in futures]

    def _name(self, stem: str, attempt: int) -> str:
        return stem if self.args.repeat == 1 else f"{stem}-r{attempt}"

    def _seed(self, seed: int, name: str) -> dict:
        directory = self._prepare(scenario.generate(seed), name)
        result = self.evaluate(directory, seed_mode=True)
        self._record(result)
        print(_line(result), flush=True)
        return result

    def replay(self, source: pathlib.Path) -> list[dict]:
        chosen = scenario.load(source)

        def one(attempt: int) -> dict:
            name = self._name(f"replay-{chosen.seed}", attempt)
            directory = self._prepare(chosen, name)
            result = self.evaluate(directory, seed_mode=False)
            print(_line(result), flush=True)
            return result

        return list(self.pool.map(one, range(self.args.repeat)))

    def minimize(self, result: dict) -> dict:
        source = result["dir"]
        original = scenario.load(self.run_dir / source)
        counter = iter(range(1_000_000))

        def reproduces(options) -> list[bool]:
            dirs = [
                self._prepare(o, f"{source}/try-{next(counter)}")
                for o in options
            ]
            outcomes = list(
                self.pool.map(lambda d: self.evaluate(d, seed_mode=False), dirs)
            )
            for d in dirs:
                shutil.rmtree(d, ignore_errors=True)
            return [_same_failure(result, outcome) for outcome in outcomes]

        smallest = minimize.minimize(original, reproduces, self.args.jobs)
        target = self._prepare(smallest, f"{source}/{MINIMIZED_DIR}")
        final = self.evaluate(target, seed_mode=False)
        print(f"minimized {source}: {_line(final)}", flush=True)
        return final


def _same_failure(reference: dict, outcome: dict) -> bool:
    """The same check; for a panic, fatal event or MCU shutdown also the
    same message, so the minimizer cannot slide onto another root cause."""
    if outcome["status"] != FAIL or outcome["check"] != reference["check"]:
        return False
    if Check(reference["check"]) in ROOT_CAUSES:
        return outcome["signature"] == reference["signature"]
    return True


def _summary(results: list[dict], wall_s: float) -> str:
    failures = [r for r in results if r["status"] == FAIL]
    by_signature = collections.defaultdict(list)
    for result in failures:
        by_signature[result["signature"]].append(result["seed"])
    real = [r["real_s"] for r in results]
    lines = [
        f"{len(results)} scenarios: {len(results) - len(failures)} pass, {len(failures)} fail",
        f"real s per scenario: median {statistics.median(real):.1f}, "
        f"p90 {sorted(real)[int(0.9 * (len(real) - 1))]:.1f}; wall {wall_s:.0f} s, "
        f"{3600 * len(results) / max(wall_s, 1e-9):.0f} scenarios/hour",
    ]
    for signature, seeds in sorted(
        by_signature.items(), key=lambda kv: -len(kv[1])
    ):
        lines.append(f"  {len(seeds):4d}  {signature}  seeds {seeds[:12]}")
    return "\n".join(lines)


def main(argv=None) -> int:
    args = _parse(argv)
    if args.worker:
        result = run_worker(args.worker)
        (args.worker / RESULT_FILE).write_text(
            json.dumps(result, indent=1) + "\n"
        )
        return 0
    if args.step_tap:
        os.environ[execute.STEP_TAP_SWITCH] = "1"
    runner = Runner(args)
    started = time.monotonic()
    if args.replay:
        results = runner.replay(args.replay)
    else:
        seeds = args.seed if args.seed is not None else args.seeds
        results = runner.run_seeds(seeds)
    if args.minimize:
        first_failures = {}
        for result in results:
            if result["status"] == FAIL:
                first_failures.setdefault(result["seed"], result)
        for result in first_failures.values():
            runner.minimize(result)
    summary = _summary(results, time.monotonic() - started)
    (runner.run_dir / SUMMARY_FILE).write_text(summary + "\n")
    print(summary)
    print(f"results: {runner.run_dir}")
    return 1 if any(r["status"] == FAIL for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
