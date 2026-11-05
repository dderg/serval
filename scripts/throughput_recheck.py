#!/usr/bin/env python3
"""Re-measure the Pi and refit the throughput gate's Pi worst-window models.

    python3 scripts/throughput_recheck.py [--dispatches N] [--stage STAGE]

The gate (scripts/throughput_gate.py) predicts the Pi 5 2-core worst 5 s
window of the dense print from what it measures on its own machine. Both of
its predictions come from fits over paired points this script measures: the
variants of the code in pi_calibration.variants of
tools/bench/throughput_baseline.json (commits, optionally patched for
spread) plus the baseline's reference commit. Per variant and config:

- Pi: the full dense print under `taskset -c 0,1` on the bench Pi (ssh
  PI_HOST, under tools/bench/lock.py), the lowest worst window of PI_RUNS
  runs, every variant built in one directory there so only its code differs;
- Ir: the instructions and stream seconds of the gate's slice, counted the
  way the gate counts them, in the gate's image;
- live: the live 2-core wall per stream second of the full print on every
  CPU model the gate runs on. That is the GitHub runners: a throwaway branch,
  whose every CI job replays all variants, is dispatched DISPATCHES times,
  with the binaries built in the gate's image at the runner's paths, so they
  are the binaries the gate builds. It is also this machine, which accepts
  run on. A job takes each variant's cost as the mean of its LIVE_FASTEST
  fastest runs, then as a log against the job's mean over variants. A CPU
  model's value is the mean over its jobs.

Per config it fits, through the reference the way the gate predicts a PR
from it, ln(W / W_ref) = -beta ln(Ir per stream second / the reference's)
and, per CPU model, ln(W / W_ref) = -alpha ln(live cost / the reference's).
For each fit it records each point's error and leave-one-out error (the
point predicted by the fit without it), in % of the measured window. The
gate predicts the window from Ir as the reference's window times the Ir
rate ratio^-beta, and from the live ratio as W_reference times
live_ratio^-alpha. Fitting makes the reference's window the one measured
here and stamps when and at which commit the Pi was measured, which
restarts the gate's PI_CALIBRATION_STALE_DAYS clock.

Stages: build (trees, binaries, Ir), dispatch (the runner branches), pi,
local, collect (wait for the runners, read their logs, delete the
branches), fit. `all` runs them in that order. Each stage saves its data
under WORK/data, so a failed stage reruns alone with --stage. Collect adds
its rows to the runner rows already collected for the same variants, so a
further dispatch + collect widens the CPU coverage before the fit.
"""

from __future__ import annotations

import argparse
import collections
import datetime
import hashlib
import json
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import throughput_gate as gate
from throughput_gate import GateError

WORK = gate.DOCKER_CACHE / "target" / "throughput-recheck"
IMAGE_WORK = Path("rust") / "target" / "throughput-recheck"
REFERENCE_LABEL = "reference"
REPO = "dderg/serval"
WORKFLOW = Path(".github") / "workflows" / "ci-rust-runtime.yaml"
PROBE_DIR = "recheck-probe"
PROBE_MARGIN_SECS = 300
PROBE_MIN_BUDGET_SECS = 300
BRANCH_PREFIX = "throughput-recheck-"
DISPATCHES = 2
POLL_SECS = 60
RETRIES = 4
PI_HOST = "bench-pi"
PI_DIR = "throughput-recheck"
PI_RUNS = 2
PI_COOLDOWN_SECS = 60
PI_LOCK_TIMEOUT_SECS = 3500
IMAGE_TAG = "klipper-build-throughput-recheck"
WORST_WINDOW = re.compile(
    r"worst stream windows[^\n]*\n\s*[0-9.]+\.\.\s*[0-9.]+\s+([0-9.]+)x"
)
PROBE_ROW = re.compile(r"RECHECK (\{.*\})\s*$")
STAGES = ("build", "dispatch", "pi", "local", "collect", "fit")


def data_path(name: str) -> Path:
    return WORK / "data" / f"{name}.json"


def save(name: str, payload: dict) -> None:
    data_path(name).parent.mkdir(parents=True, exist_ok=True)
    data_path(name).write_text(json.dumps(payload, indent=2) + "\n")


def load(name: str, keys: dict[str, str]) -> dict:
    path = data_path(name)
    if not path.exists():
        raise GateError(f"{path} is missing: run --stage {name}")
    payload = json.loads(path.read_text())
    if payload["variants"] != keys:
        raise GateError(
            f"{path} measured other variants ({payload['variants']}), "
            f"not {keys}: rerun --stage {name}"
        )
    return payload


def retried(command: list[str], cwd: Path | None = None) -> str:
    for attempt in range(RETRIES):
        try:
            return gate.run(command, cwd=cwd)
        except GateError:
            if attempt == RETRIES - 1:
                raise
            time.sleep(15 * (attempt + 1))
    raise AssertionError("unreachable")


def stream(command: list[str], cwd: Path | None = None) -> None:
    if subprocess.run(command, cwd=cwd, check=False).returncode != 0:
        raise GateError(f"failed: {' '.join(command)}")


def variants(baseline: dict) -> list[dict]:
    listed = baseline.get("pi_calibration", {}).get("variants")
    if not listed:
        raise GateError("the baseline lists no pi_calibration.variants")
    labels = [variant["label"] for variant in listed]
    if REFERENCE_LABEL in labels or len(set(labels)) != len(labels):
        raise GateError(
            f"variant labels must be unique and not {REFERENCE_LABEL!r}: {labels}"
        )
    return [*listed, {"label": REFERENCE_LABEL, "commit": baseline["commit"]}]


def variant_key(variant: dict) -> str:
    patch = variant.get("patch")
    if not patch:
        return variant["commit"]
    digest = hashlib.sha256((gate.ROOT / patch).read_bytes()).hexdigest()
    return f"{variant['commit']}+{Path(patch).name}@{digest[:12]}"


def extract_tree(variant: dict, tree: Path) -> None:
    marker = tree / gate.EXTRACTED_MARKER
    key = variant_key(variant)
    if marker.exists() and marker.read_text().strip() == key:
        return
    staging = gate.reference_tree(WORK / "staging", variant["commit"])
    gate.prepare_reference(variant["commit"], WORK / "staging")
    if tree.exists():
        shutil.rmtree(tree)
    shutil.copytree(staging, tree)
    if variant.get("patch"):
        gate.run(
            [
                "patch",
                "--batch",
                "--forward",
                "-p1",
                "-d",
                str(tree),
                "-i",
                str(gate.ROOT / variant["patch"]),
            ]
        )
    marker.write_text(key + "\n")


def image() -> str:
    named = os.environ.get("THROUGHPUT_GATE_IMAGE")
    if named:
        return named
    gate.run(
        [
            "docker",
            "build",
            "--quiet",
            "-f",
            str(gate.ROOT / "scripts" / "Dockerfile-build"),
            "-t",
            IMAGE_TAG,
            str(gate.ROOT),
        ]
    )
    return IMAGE_TAG


def build_stage(listed: list[dict], keys: dict[str, str]) -> None:
    for variant in listed:
        extract_tree(variant, WORK / "trees" / variant["label"])
    (WORK / "manifest.json").write_text(
        json.dumps({"variants": keys, "configs": list(gate.CONFIGS)}, indent=2)
    )
    stream(
        [
            sys.executable,
            str(gate.ROOT / "scripts" / "slots.py"),
            "run",
            "--",
            sys.executable,
            str(Path(__file__).resolve()),
            "--stage",
            "build-in-image",
            "--image",
            image(),
        ]
    )


def build_in_image(work: Path) -> None:
    """Inside the gate's image: every variant's bench, built in the gate's A/B
    lab, and its slice's Ir per config."""
    manifest = json.loads((work / "manifest.json").read_text())
    labels = list(manifest["variants"])
    bins = work / "bins"
    bins.mkdir(exist_ok=True)
    for label in labels:
        print(f"building {label}", file=sys.stderr, flush=True)
        bench = gate.build_in_lab(work / "trees" / label / "rust", "recheck")
        shutil.copy2(bench, bins / label)
    counted = gate.measure_instructions(
        {
            (label, config): (bins / label, work / "trees" / label, config)
            for label in labels
            for config in manifest["configs"]
        },
        gate.gate_cores(),
    )
    ir = collections.defaultdict(dict)
    for (label, config), (instructions, stream) in counted.items():
        ir[label][config] = {"ir": instructions, "stream_secs": stream}
    (work / "data").mkdir(exist_ok=True)
    (work / "data" / "build.json").write_text(
        json.dumps({"variants": manifest["variants"], "ir": ir}, indent=2)
    )


def probe_live(
    benches: dict[str, Path],
    trees: dict[str, Path],
    configs: list[str],
    job: str,
    budget_secs: float,
) -> list[dict]:
    """Replay every variant live within the budget: first LIVE_FASTEST reps of
    each config in turn, so a short job still completes whole configs, then
    further reps of every config up to gate.LIVE_REPS. A rep replays each
    variant once, the order alternating between reps."""
    cpu = gate.cpu_model()
    gate.gate_cores()
    labels = list(benches)
    deadline = time.monotonic() + budget_secs
    rows = []
    rep_secs = 0.0
    schedule = [
        *(
            (config, rep)
            for config in configs
            for rep in range(gate.LIVE_FASTEST)
        ),
        *(
            (config, rep)
            for rep in range(gate.LIVE_FASTEST, gate.LIVE_REPS)
            for config in configs
        ),
    ]
    for config, rep in schedule:
        if time.monotonic() + rep_secs > deadline:
            break
        started = time.monotonic()
        for label in labels if rep % 2 == 0 else labels[::-1]:
            cost, wall = gate.live_secs_per_stream_sec(
                benches[label], trees[label], config
            )
            row = {
                "job": job,
                "cpu": cpu,
                "rep": rep,
                "label": label,
                "config": config,
                "cost": cost,
                "wall": wall,
            }
            print("RECHECK " + json.dumps(row), flush=True)
            rows.append(row)
        rep_secs = max(rep_secs, time.monotonic() - started)
    return rows


def job_timeouts(workflow: str) -> dict[str, int]:
    """Timeout minutes per `./scripts/ci.sh <job>` the workflow runs."""
    timeouts = {}
    timeout = None
    for line in workflow.splitlines():
        if re.match(r"^  [A-Za-z0-9_-]+:\s*$", line):
            timeout = None
        elif match := re.match(r"^\s+timeout-minutes:\s*(\d+)", line):
            timeout = int(match.group(1))
        elif (
            match := re.search(r"\./scripts/ci\.sh ([a-z0-9-]+)", line)
        ) and timeout:
            timeouts[match.group(1)] = timeout
    return timeouts


def probe_job(probe: Path, job: str) -> int:
    """A CI job of the runner branch: replay all variants within its timeout."""
    timeout = job_timeouts((gate.ROOT / WORKFLOW).read_text()).get(job)
    budget = timeout * 60 - PROBE_MARGIN_SECS if timeout else 0
    if budget < PROBE_MIN_BUDGET_SECS:
        print(f"recheck probe: {job} has no time for a rep ({timeout} min)")
        return 0
    manifest = json.loads((probe / "manifest.json").read_text())
    bins = Path(tempfile.mkdtemp(prefix="recheck-bins-"))
    benches = {}
    for label in manifest["variants"]:
        benches[label] = bins / label
        with open(benches[label], "wb") as out:
            subprocess.run(
                ["xz", "-dc", str(probe / "bins" / f"{label}.xz")],
                stdout=out,
                check=True,
            )
        benches[label].chmod(0o755)
    trees = {label: probe / "trees" / label for label in manifest["variants"]}
    probe_live(benches, trees, manifest["configs"], job, budget)
    return 0


CI_PROBE_HOOK = (
    'RUST="$ROOT/rust"\n'
    f'if [ -f "$ROOT/{PROBE_DIR}/manifest.json" ]; then\n'
    f'    exec python3 "$ROOT/scripts/throughput_recheck.py" --stage probe '
    f'--probe "$ROOT/{PROBE_DIR}" --job "${{1:-}}"\n'
    "fi\n"
)


def dispatch_stage(keys: dict[str, str], dispatches: int) -> None:
    manifest = json.loads((WORK / "manifest.json").read_text())
    if manifest["variants"] != keys:
        raise GateError(
            "the built variants are not the listed ones: rerun build"
        )
    checkout = WORK / "probe-branch"
    if checkout.exists():
        gate.run(
            ["git", "worktree", "remove", "--force", str(checkout)],
            cwd=gate.ROOT,
        )
    gate.run(
        ["git", "worktree", "add", "--detach", str(checkout), "HEAD"],
        cwd=gate.ROOT,
    )
    try:
        for script in ("throughput_gate.py", "throughput_recheck.py"):
            shutil.copy2(
                gate.ROOT / "scripts" / script, checkout / "scripts" / script
            )
        ci = checkout / "scripts" / "ci.sh"
        text = ci.read_text()
        if text.count('RUST="$ROOT/rust"\n') != 1:
            raise GateError(
                "scripts/ci.sh no longer sets RUST once: move the hook"
            )
        ci.write_text(text.replace('RUST="$ROOT/rust"\n', CI_PROBE_HOOK))
        probe = checkout / PROBE_DIR
        (probe / "bins").mkdir(parents=True)
        for label in keys:
            with open(probe / "bins" / f"{label}.xz", "wb") as out:
                subprocess.run(
                    ["xz", "-T0", "-6", "-c", str(WORK / "bins" / label)],
                    stdout=out,
                    check=True,
                )
            for needed in ("tools/bench/configs", "tools/sim/gcode"):
                shutil.copytree(
                    WORK / "trees" / label / needed,
                    probe / "trees" / label / needed,
                )
        shutil.copy2(WORK / "manifest.json", probe / "manifest.json")
        gate.run(["git", "add", "-A", "-f", "."], cwd=checkout)
        gate.run(
            [
                "git",
                "commit",
                "--quiet",
                "--no-verify",
                "-m",
                "Throwaway: Pi calibration re-check probe",
            ],
            cwd=checkout,
        )
        stamp = int(time.time())
        branches, runs = [], []
        for index in range(dispatches):
            branch = f"{BRANCH_PREFIX}{stamp}-{index}"
            retried(
                [
                    "git",
                    "push",
                    "--no-verify",
                    "--quiet",
                    "origin",
                    f"HEAD:refs/heads/{branch}",
                ],
                cwd=checkout,
            )
            branches.append(branch)
            retried(
                [
                    "gh",
                    "-R",
                    REPO,
                    "workflow",
                    "run",
                    WORKFLOW.name,
                    "--ref",
                    branch,
                ]
            )
            runs.append(run_id(branch))
            print(
                f"dispatched run {runs[-1]} on {branch}",
                file=sys.stderr,
                flush=True,
            )
    finally:
        gate.run(
            ["git", "worktree", "remove", "--force", str(checkout)],
            cwd=gate.ROOT,
        )
    save("dispatch", {"variants": keys, "branches": branches, "runs": runs})


def run_id(branch: str) -> int:
    for _ in range(30):
        listed = json.loads(
            retried(
                [
                    "gh",
                    "-R",
                    REPO,
                    "run",
                    "list",
                    "--workflow",
                    WORKFLOW.name,
                    "--branch",
                    branch,
                    "--json",
                    "databaseId",
                ]
            )
        )
        if listed:
            return listed[0]["databaseId"]
        time.sleep(10)
    raise GateError(f"no workflow run appeared for {branch}")


def collected_rows(keys: dict[str, str]) -> list[dict]:
    path = data_path("collect")
    if not path.exists():
        return []
    payload = json.loads(path.read_text())
    return payload["rows"] if payload["variants"] == keys else []


def collect_stage(keys: dict[str, str]) -> None:
    """Wait for the dispatched runs, add their rows to those already collected
    for these variants and delete their branches."""
    dispatched = load("dispatch", keys)
    runs = {f"{run}/" for run in dispatched["runs"]}
    rows = [
        row
        for row in collected_rows(keys)
        if not any(row["job"].startswith(run) for run in runs)
    ]
    for run in dispatched["runs"]:
        while True:
            status = json.loads(
                retried(
                    [
                        "gh",
                        "-R",
                        REPO,
                        "run",
                        "view",
                        str(run),
                        "--json",
                        "status",
                    ]
                )
            )["status"]
            if status == "completed":
                break
            time.sleep(POLL_SECS)
        log = retried(["gh", "-R", REPO, "run", "view", str(run), "--log"])
        rows.extend(probe_rows(log, run))
    for branch in dispatched["branches"]:
        retried(
            [
                "git",
                "push",
                "--no-verify",
                "--quiet",
                "origin",
                "--delete",
                branch,
            ]
        )
    save("collect", {"variants": keys, "rows": rows})


def probe_rows(log: str, run: int) -> list[dict]:
    rows = []
    for line in log.splitlines():
        match = PROBE_ROW.search(line)
        if match:
            row = json.loads(match.group(1))
            row["job"] = f"{run}/{line.split(chr(9), 1)[0]}"
            rows.append(row)
    return rows


def local_stage(keys: dict[str, str]) -> None:
    stream(
        [
            sys.executable,
            str(gate.ROOT / "scripts" / "slots.py"),
            "run",
            "--exclusive",
            "--",
            sys.executable,
            str(Path(__file__).resolve()),
            "--stage",
            "local-held",
        ]
    )
    load("local", keys)


def local_held(keys: dict[str, str]) -> None:
    rows = probe_live(
        {label: WORK / "bins" / label for label in keys},
        {label: WORK / "trees" / label for label in keys},
        list(gate.CONFIGS),
        "local",
        math.inf,
    )
    save("local", {"variants": keys, "rows": rows})


def pi_stage(keys: dict[str, str]) -> None:
    stream(
        [
            "uv",
            "run",
            "python",
            str(gate.ROOT / "tools" / "bench" / "lock.py"),
            "hold",
            "--lock-timeout",
            str(PI_LOCK_TIMEOUT_SECS),
            "--",
            sys.executable,
            str(Path(__file__).resolve()),
            "--stage",
            "pi-held",
        ],
        cwd=gate.ROOT,
    )
    load("pi", keys)


def ssh(command: str) -> str:
    return gate.run(["ssh", PI_HOST, command])


def pi_throttled_now() -> int:
    reply = ssh("vcgencmd get_throttled").strip()
    return int(reply.partition("=")[2], 16) & 0xF


def pi_held(keys: dict[str, str]) -> None:
    """Under the bench lock: build every variant in one directory on the Pi,
    then replay each config PI_RUNS times per variant, alternating order."""
    trees = " ".join(f"{PI_DIR}/trees/{label}" for label in keys)
    ssh(f"mkdir -p {PI_DIR}/lab {PI_DIR}/bins {trees}")
    for label in keys:
        tree = WORK / "trees" / label
        gate.run(
            [
                "rsync",
                "-rlp",
                "--checksum",
                "--delete",
                "--exclude",
                "target",
                f"{tree / 'rust'}/",
                f"{PI_HOST}:{PI_DIR}/lab/rust/",
            ]
        )
        gate.run(
            [
                "rsync",
                "-a",
                "--delete",
                f"{tree / 'tools'}/",
                f"{PI_HOST}:{PI_DIR}/trees/{label}/tools/",
            ]
        )
        print(f"building {label} on the Pi", file=sys.stderr, flush=True)
        ssh(
            f"cd {PI_DIR}/lab/rust && . ~/.cargo/env && "
            "CARGO_TARGET_DIR=../target cargo build --release --locked -q "
            "-p motion-core --example gcode_replay_bench && "
            f"cp ../target/release/examples/gcode_replay_bench ../../bins/{label}"
        )
    time.sleep(PI_COOLDOWN_SECS)
    if pi_throttled_now():
        raise GateError("the Pi is throttling before the runs")
    measured_at = datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="seconds"
    )
    labels = list(keys)
    worst = {label: {config: [] for config in gate.CONFIGS} for label in labels}
    for run in range(PI_RUNS):
        order = labels if run % 2 == 0 else labels[::-1]
        for config in gate.CONFIGS:
            for label in order:
                out = ssh(
                    f"cd {PI_DIR} && taskset -c {gate.LIVE_CORES} bins/{label} "
                    f"trees/{label}/{gate.GCODE} --config trees/{label}/{config}"
                )
                match = WORST_WINDOW.search(out)
                if not match:
                    raise GateError(
                        f"no worst window in the Pi replay of {label} {config}"
                    )
                worst[label][config].append(float(match.group(1)))
                print(
                    f"pi run {run + 1}/{PI_RUNS} {label} {Path(config).stem}: "
                    f"{match.group(1)}x",
                    file=sys.stderr,
                    flush=True,
                )
    if pi_throttled_now():
        raise GateError(
            "the Pi throttled during the runs: their windows are void"
        )
    save("pi", {"variants": keys, "measured_at": measured_at, "worst_x": worst})


def factor_through_origin(points: list[tuple[float, float]]) -> float:
    """Least-squares factor of y = -factor * x."""
    spread = sum(x * x for x, _ in points)
    if spread == 0.0:
        raise GateError(f"points {points} all sit at x = 0: no factor")
    return -sum(x * y for x, y in points) / spread


def error_pct(predicted_log: float, measured_log: float) -> float:
    return 100.0 * (math.exp(predicted_log - measured_log) - 1.0)


def fit(
    xs: dict[str, float], ys: dict[str, float], anchor: str = REFERENCE_LABEL
) -> dict:
    """ln(W / W_anchor) = -factor * (x - x_anchor), the way the gate predicts
    a PR from the reference, with each other point's residual and
    leave-one-out residual in % of its measured window."""
    others = sorted(k for k in xs if k != anchor)
    if len(others) < 2:
        raise GateError(f"a fit wants 2 points besides {anchor}, got {others}")
    paired = {k: (xs[k] - xs[anchor], ys[k] - ys[anchor]) for k in others}
    factor = factor_through_origin(list(paired.values()))
    residuals = {
        k: round(error_pct(-factor * x, y), 2) for k, (x, y) in paired.items()
    }
    loo = {}
    for k, (x, y) in paired.items():
        rest = [paired[m] for m in others if m != k]
        loo[k] = round(error_pct(-factor_through_origin(rest) * x, y), 2)
    return {
        "factor": factor,
        "intercept": ys[anchor] + factor * xs[anchor],
        "max_residual_pct": max(abs(r) for r in residuals.values()),
        "max_loo_residual_pct": max(abs(r) for r in loo.values()),
        "residual_pct": residuals,
        "loo_residual_pct": loo,
    }


def live_log_costs(
    rows: list[dict], labels: list[str], configs: list[str]
) -> dict[str, dict[str, dict]]:
    """Per config and CPU model: each variant's log live cost against the
    job's mean over variants, averaged over the jobs that ran every variant
    at least LIVE_FASTEST times, and how many jobs that was."""
    costs = collections.defaultdict(list)
    cpus = {}
    for row in rows:
        costs[row["job"], row["config"], row["label"]].append(row["cost"])
        cpus[row["job"]] = row["cpu"]
    centred = collections.defaultdict(list)
    for job, cpu in cpus.items():
        for config in configs:
            samples = [costs[job, config, label] for label in labels]
            if min(len(s) for s in samples) < gate.LIVE_FASTEST:
                continue
            logs = [math.log(gate.fastest_mean(s)) for s in samples]
            mean = statistics.fmean(logs)
            centred[config, cpu].append(
                {label: value - mean for label, value in zip(labels, logs)}
            )
    result = collections.defaultdict(dict)
    for (config, cpu), jobs in centred.items():
        result[config][cpu] = {
            "jobs": len(jobs),
            "x": {
                label: statistics.fmean(j[label] for j in jobs)
                for label in labels
            },
        }
    return result


def fit_stage(baseline: dict, listed: list[dict], keys: dict[str, str]) -> dict:
    labels = list(keys)
    built = load("build", keys)["ir"]
    pi = load("pi", keys)
    rows = load("collect", keys)["rows"] + load("local", keys)["rows"]
    live = live_log_costs(rows, labels, list(gate.CONFIGS))
    models = {}
    for config in gate.CONFIGS:
        worst = {label: min(pi["worst_x"][label][config]) for label in labels}
        ys = {label: math.log(worst[label]) for label in labels}
        ir_x = {
            label: math.log(
                built[label][config]["ir"] / built[label][config]["stream_secs"]
            )
            for label in labels
        }
        live_fits = {}
        for cpu, measured in sorted(live[config].items()):
            live_fits[cpu] = {
                "jobs": measured["jobs"],
                **fit(measured["x"], ys),
            }
        models[config] = {
            "ir": fit(ir_x, ys),
            "live": live_fits,
            "points": {
                label: {
                    "pi_worst_x": pi["worst_x"][label][config],
                    "ir": built[label][config]["ir"],
                    "stream_secs": built[label][config]["stream_secs"],
                    "live_log_cost": {
                        cpu: round(measured["x"][label], 5)
                        for cpu, measured in sorted(live[config].items())
                    },
                }
                for label in labels
            },
        }
        baseline["configs"][config]["pi_2core"] = {
            "worst_x_reference": worst[REFERENCE_LABEL],
            "worst_x": pi["worst_x"][REFERENCE_LABEL][config],
            "commit": baseline["commit"],
        }
    baseline["pi_calibration"] = {
        "measured_at": pi["measured_at"],
        "commit": baseline["commit"],
        "variants": listed[:-1],
        "configs": models,
    }
    return baseline


def short_cpu(cpu: str) -> str:
    noise = {
        "AMD",
        "Intel",
        "INTEL",
        "CPU",
        "Processor",
        "Platinum",
        "PLATINUM",
    }
    words = cpu.replace("(R)", "").replace("XEON", "Xeon").split()
    return " ".join(
        w
        for w in words
        if w not in noise and not w.endswith(("-Core", "GHz")) and w != "@"
    )


def report(calibration: dict) -> None:
    """Each point against the reference (Pi worst window, Ir per stream
    second and live cost per CPU), then each fit's factor and errors."""
    print(
        f"Pi calibration measured {calibration['measured_at']} at "
        f"{calibration['commit']}"
    )
    for config, model in calibration["configs"].items():
        print(f"\n{config}")
        points = model["points"]
        cpus = list(model["live"])
        print(
            f"  {'variant':<34} {'Pi worst':>8} {'Ir/s':>7}"
            + "".join(f" {short_cpu(cpu)[:13]:>13}" for cpu in cpus)
        )
        reference = points[REFERENCE_LABEL]
        reference_rate = reference["ir"] / reference["stream_secs"]
        for label, point in points.items():
            rate = point["ir"] / point["stream_secs"]
            live = "".join(
                f" {100 * math.expm1(point['live_log_cost'][cpu] - reference['live_log_cost'][cpu]):>+12.1f}%"
                for cpu in cpus
            )
            print(
                f"  {label:<34} {min(point['pi_worst_x']):>7.2f}x "
                f"{100 * (rate / reference_rate - 1):>+6.1f}%{live}"
            )
        fits = [("Ir", model["ir"])] + [
            (f"live on {short_cpu(cpu)} ({fitted['jobs']} jobs)", fitted)
            for cpu, fitted in model["live"].items()
        ]
        for name, fitted in fits:
            print(
                f"  {name}: factor {fitted['factor']:.2f}, error max "
                f"{fitted['max_residual_pct']:.1f} %, leave-one-out max "
                f"{fitted['max_loo_residual_pct']:.1f} % "
                f"({len(fitted['residual_pct'])} pairs with the reference)"
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--stage",
        default="all",
        choices=(
            "all",
            *STAGES,
            "build-in-image",
            "local-held",
            "pi-held",
            "probe",
        ),
    )
    parser.add_argument("--dispatches", type=int, default=DISPATCHES)
    parser.add_argument("--image", help=argparse.SUPPRESS)
    parser.add_argument("--probe", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--job", default="", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.stage == "probe":
        return probe_job(args.probe, args.job)
    if args.stage == "build-in-image" and args.image is None:
        build_in_image(gate.ROOT / IMAGE_WORK)
        return 0
    if args.stage == "build-in-image":
        gate.run_in_image(
            args.image,
            "scripts/throughput_recheck.py",
            ["--stage", "build-in-image"],
        )
        return 0

    baseline = gate.load_baseline()
    listed = variants(baseline)
    keys = {variant["label"]: variant_key(variant) for variant in listed}
    stages = STAGES if args.stage == "all" else (args.stage,)
    for stage in stages:
        print(f"recheck stage {stage}", file=sys.stderr, flush=True)
        if stage == "build":
            build_stage(listed, keys)
        elif stage == "dispatch":
            dispatch_stage(keys, args.dispatches)
        elif stage == "pi":
            pi_stage(keys)
        elif stage == "pi-held":
            pi_held(keys)
        elif stage == "local":
            local_stage(keys)
        elif stage == "local-held":
            local_held(keys)
        elif stage == "collect":
            collect_stage(keys)
        elif stage == "fit":
            gate.write_baseline(fit_stage(baseline, listed, keys))
            print(f"wrote {gate.BASELINE.relative_to(gate.ROOT)}")
            report(gate.load_baseline()["pi_calibration"])
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except GateError as exc:
        sys.exit(f"throughput recheck: {exc}")
