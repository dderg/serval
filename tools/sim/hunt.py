#!/usr/bin/env python3
"""Long scenario-fuzzer runs that share the box: seeds in batches sized to
hold a machine-wide slot (scripts/slots.py) for about --batch-minutes each,
released between batches so other agents' jobs interleave.

    python3 tools/sim/hunt.py --seeds 1000..2999 --run-name hunt-1000

Each batch is `slots.py run -- tools/sim/run.sh fuzz --seed ... --run-name`,
whose ledger (.sim-fuzz/<run-name>/ledger.jsonl) records every seed as it
finishes; a rerun of this command resumes after the recorded ones. The next
batch's size comes from the last batch's seeds per minute of slot time.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import tempfile
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
SLOTS = REPO_ROOT / "scripts" / "slots.py"
RUN_SH = REPO_ROOT / "tools" / "sim" / "run.sh"
FUZZ_ROOT = REPO_ROOT / ".sim-fuzz"
LEDGER_FILE = "ledger.jsonl"
FIRST_BATCH = 40
MIN_BATCH = 8
MAX_BATCH = 400
SECONDS_PER_MINUTE = 60


def _seed_range(text: str) -> range:
    low, _, high = text.partition("..")
    return range(int(low), int(high) + 1)


def recorded_seeds(run_name: str) -> set[int]:
    ledger = FUZZ_ROOT / run_name / LEDGER_FILE
    if not ledger.exists():
        return set()
    lines = ledger.read_text().splitlines()
    return {json.loads(line)["seed"] for line in lines if line.strip()}


def next_size(done: int, slot_s: float, target_s: float) -> int:
    """Seeds that fill `target_s` of slot time at the last batch's rate."""
    if done == 0 or slot_s <= 0:
        return MIN_BATCH
    return max(MIN_BATCH, min(MAX_BATCH, round(done * target_s / slot_s)))


def _run_batch(seeds: list[int], run_name: str, extra: list[str]) -> float:
    """Seconds the batch held its slot (from acquisition to release)."""
    fuzz = [str(RUN_SH), "fuzz", "--seed", ",".join(map(str, seeds))]
    fuzz += ["--run-name", run_name, *extra]
    with tempfile.NamedTemporaryFile("r", prefix="hunt-slot-") as stamp:
        inner = f'date +%s.%N > "{stamp.name}"; exec "$@"'
        command = [sys.executable, str(SLOTS), "run", "--"]
        command += ["bash", "-c", inner, "_", *fuzz]
        subprocess.run(command, cwd=REPO_ROOT, check=False)
        acquired = stamp.read().strip()
    started = float(acquired) if acquired else time.time()
    return time.time() - started


def hunt(
    seeds: range, run_name: str, target_s: float, extra: list[str]
) -> None:
    size = FIRST_BATCH
    while True:
        done = recorded_seeds(run_name)
        pending = [seed for seed in seeds if seed not in done]
        if not pending:
            print(
                f"hunt {run_name}: all {len(seeds)} seeds recorded", flush=True
            )
            return
        batch = pending[:size]
        print(
            f"hunt {run_name}: batch {batch[0]}..{batch[-1]} ({len(batch)} seeds, "
            f"{len(pending)} pending)",
            flush=True,
        )
        slot_s = _run_batch(batch, run_name, extra)
        finished = len(recorded_seeds(run_name) & set(batch))
        if finished == 0:
            print(
                f"hunt {run_name}: batch recorded nothing; stopping", flush=True
            )
            sys.exit(1)
        size = next_size(finished, slot_s, target_s)
        print(
            f"hunt {run_name}: {finished} seeds in {slot_s:.0f} s of slot; "
            f"next batch {size}",
            flush=True,
        )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=_seed_range, required=True)
    parser.add_argument("--run-name", required=True)
    parser.add_argument("--batch-minutes", type=float, default=10.0)
    args, extra = parser.parse_known_args(argv)
    hunt(
        args.seeds,
        args.run_name,
        args.batch_minutes * SECONDS_PER_MINUTE,
        extra,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
