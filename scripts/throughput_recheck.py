#!/usr/bin/env python3
"""Re-measure the Pi and refit the throughput gate's Pi worst-window models.

    python3 scripts/throughput_recheck.py [--dispatches N] [--stage STAGE]
        [--calibration-commit REV]

The gate (scripts/throughput_gate.py) predicts the Pi 5 2-core worst 5 s
window of the dense print from what it measures on its own machine. Both of
its predictions come from fits over paired points this script measures: the
variants of the code in pi_calibration.variants of
tools/bench/throughput_baseline.json (commits, optionally patched for
spread) plus the calibration commit the gate's live replay is pinned to:
the baseline's reference commit unless --calibration-commit names another.
Pinning it at the reference puts the gate's yardstick at the code it judges
next, where its prediction is the Pi's own measurement on every runner CPU;
the fit's CPU-dependent error grows with the head's distance from it.
Between re-checks the pin holds still. Per variant and config:

- Pi: the full dense print under `taskset -c 0,1` on the bench Pi (ssh
  PI_HOST, under tools/bench/lock.py), the lowest worst window of PI_RUNS
  runs, every variant built in one directory there so only its code differs,
  and for the calibration commit also its fastest wall per stream second;
- Ir: the instructions and stream seconds of the gate's slice, counted the
  way the gate counts them, in the gate's image;
- live: the live 2-core wall per stream second of the full print on as many
  runner CPU models as GitHub hands out. A throwaway branch, whose every CI
  job replays all variants, is dispatched DISPATCHES times, with the
  binaries built in the gate's image at the runner's paths, so they are the
  binaries the gate builds. This machine, which accepts run on, replays them
  too. A job takes each variant's cost as the mean of its LIVE_FASTEST
  fastest runs, as a log against the calibration commit's in the same job. A
  CPU model's value is the mean over its jobs. Runner rows are kept per
  variant key (commit and patch digest), so rows collected before for an
  unchanged variant and calibration commit still count after the variant
  list grows.

Per config it fits, through the calibration commit the way the gate
predicts, ln(W / W_cal) = -beta ln(Ir per stream second / the
calibration's) and ln(W / W_cal) = -alpha x, x the log live cost over the
calibration's. alpha is one factor over every (CPU model, variant) point, so
each CPU model counts once however many jobs landed on it. Each fit records
each point's error and leave-one-out error (the point predicted by the fit
without its variant), in % of the measured window; the live fit, per CPU
model, also the error with that CPU model left out of the fit, which is how
well it predicts a CPU model it never saw. It also records the Pi's speed
on the calibration commit and the range of each job's speed on it as a
multiple of the Pi's, which the gate holds its runner to. Fitting stamps
when the Pi was measured, which restarts the gate's
PI_CALIBRATION_STALE_DAYS clock.

Stages: build (trees, binaries, Ir), dispatch (the runner branches), pi,
local, collect (wait for the runners, read their logs, delete the
branches), fit. `all` runs them in that order. Each stage saves its data
under WORK/data, so a failed stage reruns alone with --stage. Collect adds
its rows to the runner rows already collected, so a further dispatch +
collect widens the CPU coverage before the fit.
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
CALIBRATION_LABEL = "calibration"
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
PI_STAMP = re.compile(r"\bpi_(started|ended)_ns=(\d+)")
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


def calibration_commit(baseline: dict, pinned: str | None) -> str:
    """The commit to pin the gate's calibration replay to: --calibration-commit,
    or else the baseline's reference, so the yardstick sits at the code the
    gate judges next."""
    chosen = pinned or baseline.get("commit")
    if not chosen:
        raise GateError(
            "the baseline names no reference commit: pass --calibration-commit"
        )
    return gate.run(
        ["git", "rev-parse", "--verify", f"{chosen}^{{commit}}"],
        cwd=gate.ROOT,
    ).strip()


def variants(baseline: dict, calibration: str) -> list[dict]:
    listed = baseline.get("pi_calibration", {}).get("variants")
    if not listed:
        raise GateError("the baseline lists no pi_calibration.variants")
    labels = [variant["label"] for variant in listed]
    if CALIBRATION_LABEL in labels or len(set(labels)) != len(labels):
        raise GateError(
            f"variant labels must be unique and not {CALIBRATION_LABEL!r}: {labels}"
        )
    duplicates = [
        variant["label"]
        for variant in listed
        if variant["commit"] == calibration and not variant.get("patch")
    ]
    if duplicates:
        raise GateError(
            f"variants {duplicates} are the calibration commit {calibration} "
            "itself: drop them from pi_calibration.variants"
        )
    return [*listed, {"label": CALIBRATION_LABEL, "commit": calibration}]


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
    staging = gate.commit_tree(WORK / "staging", variant["commit"])
    gate.extract_commit(variant["commit"], WORK / "staging")
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


def stored_rows() -> list[dict]:
    path = data_path("collect")
    return json.loads(path.read_text())["rows"] if path.exists() else []


def collected_rows(keys: dict[str, str]) -> list[dict]:
    """Runner rows collected so far whose variant key is still listed,
    under that variant's current label."""
    labels = {key: label for label, key in keys.items()}
    return [
        {**row, "label": labels[row["key"]]}
        for row in stored_rows()
        if row["key"] in labels
    ]


def collect_stage(keys: dict[str, str]) -> None:
    """Wait for the dispatched runs, add their rows to those already collected
    and delete their branches."""
    dispatched = load("dispatch", keys)
    runs = {f"{run}/" for run in dispatched["runs"]}
    rows = [
        row
        for row in stored_rows()
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
        rows.extend(probe_rows(log, run, dispatched["variants"]))
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
    save("collect", {"rows": rows})


def probe_rows(log: str, run: int, keys: dict[str, str]) -> list[dict]:
    rows = []
    for line in log.splitlines():
        match = PROBE_ROW.search(line)
        if match:
            row = json.loads(match.group(1))
            row["job"] = f"{run}/{line.split(chr(9), 1)[0]}"
            row["key"] = keys[row["label"]]
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
            "--calibration-commit",
            keys[CALIBRATION_LABEL],
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
            "--calibration-commit",
            keys[CALIBRATION_LABEL],
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
    cost = {label: {config: [] for config in gate.CONFIGS} for label in labels}
    for run in range(PI_RUNS):
        order = labels if run % 2 == 0 else labels[::-1]
        for config in gate.CONFIGS:
            for label in order:
                out = ssh(
                    f"cd {PI_DIR} && date +pi_started_ns=%s%N && "
                    f"taskset -c {gate.LIVE_CORES} bins/{label} "
                    f"trees/{label}/{gate.GCODE} --config trees/{label}/{config} "
                    "&& date +pi_ended_ns=%s%N"
                )
                match = WORST_WINDOW.search(out)
                if not match:
                    raise GateError(
                        f"no worst window in the Pi replay of {label} {config}"
                    )
                worst[label][config].append(float(match.group(1)))
                cost[label][config].append(
                    pi_wall_secs(out) / gate.stream_secs(out)
                )
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
    save(
        "pi",
        {
            "variants": keys,
            "measured_at": measured_at,
            "worst_x": worst,
            "cost": cost,
        },
    )


def pi_wall_secs(out: str) -> float:
    """The Pi replay's wall seconds, process start to exit as the gate times
    its own replays, from the `date` stamps around it."""
    stamps = dict(PI_STAMP.findall(out))
    if set(stamps) != {"started", "ended"}:
        raise GateError(
            f"the Pi replay printed no start and end stamps:\n{out}"
        )
    return (int(stamps["ended"]) - int(stamps["started"])) * 1e-9


def factor_through_origin(points: list[tuple[float, float]]) -> float:
    """Least-squares factor of y = -factor * x."""
    spread = sum(x * x for x, _ in points)
    if spread == 0.0:
        raise GateError(f"points {points} all sit at x = 0: no factor")
    return -sum(x * y for x, y in points) / spread


def error_pct(predicted_log: float, measured_log: float) -> float:
    return 100.0 * (math.exp(predicted_log - measured_log) - 1.0)


def fit(
    xs: dict[str, float], ys: dict[str, float], anchor: str = CALIBRATION_LABEL
) -> dict:
    """ln(W / W_anchor) = -factor * (x - x_anchor), the way the gate predicts
    a PR from the calibration commit, with each other point's residual and
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


def pooled_fit(
    xs: dict[str, dict[str, float]], ys: dict[str, float], jobs: dict[str, int]
) -> dict:
    """ln(W / W_cal) = -factor * x over every (CPU model, variant) point, x
    the variant's log live cost over the calibration commit's on that CPU
    model. Per CPU model: each point's residual, its leave-one-out residual
    (the fit without that variant on any CPU model) and its residual from the
    fit without that CPU model, in % of the measured window."""
    if len(xs) < 2:
        raise GateError(
            f"the live fit wants 2 CPU models to be told apart, got {sorted(xs)}"
        )
    others = sorted(label for label in ys if label != CALIBRATION_LABEL)
    points = {
        (cpu, label): (measured[label], ys[label] - ys[CALIBRATION_LABEL])
        for cpu, measured in xs.items()
        for label in others
        if label in measured
    }
    fitted_labels = sorted({label for _, label in points})
    if len(fitted_labels) < 2:
        raise GateError(
            f"the live fit wants 2 variants besides {CALIBRATION_LABEL}"
        )

    def factor_without(drop: int, value: str | None) -> float:
        return factor_through_origin(
            [point for key, point in points.items() if key[drop] != value]
        )

    factor = factor_without(0, None)
    loo_factor = {label: factor_without(1, label) for label in fitted_labels}
    by_cpu = {}
    for cpu in sorted(xs):
        mine = {
            label: point
            for (owner, label), point in points.items()
            if owner == cpu
        }
        cpu_out = factor_without(0, cpu)
        slopes = {
            "residual_pct": {label: factor for label in mine},
            "loo_residual_pct": loo_factor,
            "cpu_out_residual_pct": {label: cpu_out for label in mine},
        }
        errors = {
            name: {
                label: round(error_pct(-slope[label] * x, y), 2)
                for label, (x, y) in mine.items()
            }
            for name, slope in slopes.items()
        }
        by_cpu[cpu] = {
            "jobs": jobs[cpu],
            **{
                f"max_{name}": max(abs(e) for e in values.values())
                for name, values in errors.items()
            },
            **errors,
        }
    return {
        "factor": factor,
        "intercept": ys[CALIBRATION_LABEL],
        **{
            name: max(fitted[name] for fitted in by_cpu.values())
            for name in (
                "max_residual_pct",
                "max_loo_residual_pct",
                "max_cpu_out_residual_pct",
            )
        },
        "by_cpu": by_cpu,
    }


def live_costs(
    rows: list[dict], labels: list[str], configs: list[str]
) -> dict[str, dict]:
    """Per config: per CPU model, each variant's log live cost over the
    calibration commit's in the same job, averaged over the jobs that ran
    both at least LIVE_FASTEST times, and how many jobs ran the calibration
    commit; per such job, the calibration commit's cost and how far apart
    its fastest runs were."""
    costs = collections.defaultdict(list)
    cpus = {}
    for row in rows:
        costs[row["job"], row["config"], row["label"]].append(row["cost"])
        cpus[row["job"]] = row["cpu"]
    result = {
        config: {"cpus": {}, "calibration_costs": [], "spreads_pct": []}
        for config in configs
    }
    for config in configs:
        logs = collections.defaultdict(lambda: collections.defaultdict(list))
        jobs = collections.Counter()
        for job, cpu in cpus.items():
            calibration = costs[job, config, CALIBRATION_LABEL]
            if len(calibration) < gate.LIVE_FASTEST:
                continue
            yardstick = gate.fastest_mean(calibration)
            jobs[cpu] += 1
            result[config]["calibration_costs"].append(yardstick)
            result[config]["spreads_pct"].append(
                gate.fastest_spread_pct(calibration)
            )
            for label in labels:
                samples = costs[job, config, label]
                if len(samples) >= gate.LIVE_FASTEST:
                    logs[cpu][label].append(
                        math.log(gate.fastest_mean(samples) / yardstick)
                    )
        result[config]["cpus"] = {
            cpu: {
                "jobs": jobs[cpu],
                "x": {
                    label: statistics.fmean(values)
                    for label, values in by_label.items()
                },
            }
            for cpu, by_label in sorted(logs.items())
        }
    return result


def fit_stage(baseline: dict, listed: list[dict], keys: dict[str, str]) -> dict:
    labels = list(keys)
    built = load("build", keys)["ir"]
    pi = load("pi", keys)
    rows = collected_rows(keys) + load("local", keys)["rows"]
    live = live_costs(rows, labels, list(gate.CONFIGS))
    models = {}
    for config in gate.CONFIGS:
        ys = {
            label: math.log(min(pi["worst_x"][label][config]))
            for label in labels
        }
        ir_x = {
            label: math.log(
                built[label][config]["ir"] / built[label][config]["stream_secs"]
            )
            for label in labels
        }
        measured = live[config]
        pi_cost = min(pi["cost"][CALIBRATION_LABEL][config])
        speeds = [pi_cost / cost for cost in measured["calibration_costs"]]
        models[config] = {
            "ir": fit(ir_x, ys),
            "live": {
                **pooled_fit(
                    {cpu: m["x"] for cpu, m in measured["cpus"].items()},
                    ys,
                    {cpu: m["jobs"] for cpu, m in measured["cpus"].items()},
                ),
                "pi_cost": pi_cost,
                "speed_x": {"min": min(speeds), "max": max(speeds)},
                "max_calibration_spread_pct": max(measured["spreads_pct"]),
            },
            "points": {
                label: {
                    "pi_worst_x": pi["worst_x"][label][config],
                    "ir": built[label][config]["ir"],
                    "stream_secs": built[label][config]["stream_secs"],
                    "live_log_cost": {
                        cpu: round(m["x"][label], 5)
                        for cpu, m in measured["cpus"].items()
                        if label in m["x"]
                    },
                }
                for label in labels
            },
        }
    baseline["pi_calibration"] = {
        "measured_at": pi["measured_at"],
        "commit": listed[-1]["commit"],
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
    """Each point against the calibration commit (Pi worst window, Ir per
    stream second and live cost per CPU model), then each fit's factor and
    errors, the live fit's per CPU model."""
    print(
        f"Pi calibration at {calibration['commit']}, measured "
        f"{calibration['measured_at']}"
    )
    for config, model in calibration["configs"].items():
        print(f"\n{config}")
        points = model["points"]
        live = model["live"]
        cpus = list(live["by_cpu"])
        print(
            f"  {'variant':<34} {'Pi worst':>8} {'Ir/s':>7}"
            + "".join(f" {short_cpu(cpu)[:13]:>13}" for cpu in cpus)
        )
        anchor = points[CALIBRATION_LABEL]
        anchor_rate = anchor["ir"] / anchor["stream_secs"]
        for label, point in points.items():
            rate = point["ir"] / point["stream_secs"]
            costs = point["live_log_cost"]
            cells = "".join(
                f" {100 * math.expm1(costs[cpu]):>+12.1f}%"
                if cpu in costs
                else f" {'-':>13}"
                for cpu in cpus
            )
            print(
                f"  {label:<34} {min(point['pi_worst_x']):>7.2f}x "
                f"{100 * (rate / anchor_rate - 1):>+6.1f}%{cells}"
            )
        ir = model["ir"]
        print(
            f"  Ir: factor {ir['factor']:.2f}, error max "
            f"{ir['max_residual_pct']:.1f} %, leave-one-out max "
            f"{ir['max_loo_residual_pct']:.1f} %"
        )
        print(
            f"  live: factor {live['factor']:.2f} over {len(cpus)} CPU models, "
            f"error max {live['max_residual_pct']:.1f} %, leave-one-out max "
            f"{live['max_loo_residual_pct']:.1f} %, CPU model left out max "
            f"{live['max_cpu_out_residual_pct']:.1f} %"
        )
        for cpu, fitted in live["by_cpu"].items():
            print(
                f"    {short_cpu(cpu):<24} {fitted['jobs']:>3} jobs: error max "
                f"{fitted['max_residual_pct']:>4.1f} %, leave-one-out "
                f"{fitted['max_loo_residual_pct']:>4.1f} %, left out "
                f"{fitted['max_cpu_out_residual_pct']:>4.1f} %"
            )
        print(
            f"  calibration replay: the Pi's {live['pi_cost']:.3f} s per stream "
            f"second, jobs {live['speed_x']['min']:.2f}-"
            f"{live['speed_x']['max']:.2f}x its speed, fastest "
            f"{gate.LIVE_FASTEST} runs up to "
            f"{live['max_calibration_spread_pct']:.2f} % apart"
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
    parser.add_argument("--calibration-commit")
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
    listed = variants(
        baseline, calibration_commit(baseline, args.calibration_commit)
    )
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
