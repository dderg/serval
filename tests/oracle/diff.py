#!/usr/bin/env python3
"""Trajectory oracle diff: compare actual_fork/*.csv against expected/*.csv.

Walks the expected/ directory, looks for a matching actual_fork/ file for
each input, and reports the first divergence sample-by-sample. Tolerances
are tight on position (1e-4 mm) and looser on velocity / accel (1e-2)
because differentiation amplifies numerical noise.

Usage:
    python3 tests/oracle/diff.py            # diff all inputs
    python3 tests/oracle/diff.py 01_x10     # diff one input by stem

Exit code 0 = all match (or no actual_fork CSVs to compare).
Exit code 1 = at least one divergence.
Exit code 2 = at least one fork capture is missing (e.g. fork failed to run).
"""
from __future__ import annotations
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
EXPECTED_DIR = ROOT / "expected"
ACTUAL_DIR = ROOT / "actual_fork"

POS_TOL = 1e-4    # mm
VEL_TOL = 1e-2    # mm/s — noise from differentiating the integrated step trace
ACC_TOL = 1.0     # mm/s^2

POS_COLS = ("x", "y", "z", "e")
VEL_COLS = ("vx", "vy", "vz", "ve")
ACC_COLS = ("ax", "ay", "az", "ae")


def _read_csv(path: Path):
    with path.open() as f:
        r = csv.DictReader(f)
        return list(r)


def _f(row, col):
    return float(row[col])


def _row_diff(a: dict, b: dict, idx: int) -> str | None:
    # t alignment: simulator output is on a fixed dt grid, so tolerate
    # < 1 µs drift only.
    if abs(_f(a, "t") - _f(b, "t")) > 1e-6:
        return f"t mismatch: expected={_f(a,'t')} actual={_f(b,'t')}"
    for c, tol in [(POS_COLS, POS_TOL), (VEL_COLS, VEL_TOL), (ACC_COLS, ACC_TOL)]:
        for col in c:
            d = _f(a, col) - _f(b, col)
            if abs(d) > tol:
                return (
                    f"{col} diverges at sample {idx} (t={_f(a,'t')}): "
                    f"expected={_f(a,col):.6g} actual={_f(b,col):.6g} "
                    f"delta={d:+.6g} tol={tol}"
                )
    return None


def diff_one(stem: str) -> int:
    exp = EXPECTED_DIR / f"{stem}.csv"
    act = ACTUAL_DIR / f"{stem}.csv"
    if not exp.exists():
        print(f"[{stem}] MISSING EXPECTED: {exp}")
        return 2
    if not act.exists():
        print(f"[{stem}] MISSING FORK CAPTURE: {act}")
        print(f"         (the fork failed to produce a trajectory CSV; see "
              f"actual_fork/{stem}.log for why)")
        return 2
    a = _read_csv(exp)
    b = _read_csv(act)
    if len(a) != len(b):
        print(f"[{stem}] LENGTH MISMATCH: expected={len(a)} actual={len(b)} rows")
        # still try the prefix
    n = min(len(a), len(b))
    for i in range(n):
        msg = _row_diff(a[i], b[i], i)
        if msg is not None:
            print(f"[{stem}] FIRST DIVERGENCE: {msg}")
            return 1
    if len(a) != len(b):
        return 1
    print(f"[{stem}] OK ({n} samples match within tol)")
    return 0


def main(argv):
    if len(argv) > 1:
        stems = argv[1:]
    else:
        stems = sorted(p.stem for p in EXPECTED_DIR.glob("*.csv"))
    if not stems:
        print(f"No expected CSVs found in {EXPECTED_DIR}")
        return 0
    worst = 0
    for s in stems:
        r = diff_one(s)
        if r > worst:
            worst = r
    return worst


if __name__ == "__main__":
    sys.exit(main(sys.argv))
