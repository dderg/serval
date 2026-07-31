#!/usr/bin/env python3
"""Characterise a planner change against the committed snapshot baselines.

`run.py` answers "did anything change?" — a gate. This answers "what exactly
changed, and by how much?" — a measuring instrument, for the human who has to
decide whether a planner change is an improvement before regenerating the 50
committed baselines (only the user does that).

Per case it reports traversal time before/after, per-axis piece counts before/
after, the worst per-axis position/velocity/acceleration deviation between the
two trajectories on a shared time grid, and whether the difference is inside
`harness.FLOAT_ATOL`/`FLOAT_RTOL` (so `run.py` would still call it EXACT) or a
real shape change. Cases are listed worst-first, with a totals line.

Deviations are measured, not inferred from the baseline diff: the two runs are
sampled against each other over the union of both piece breakpoints, so a case
whose piece *count* changed is still comparable.

Exit 0 unless a case could not be characterised (missing baseline, malformed
case, cdylib not built) — a shape change is a finding, not a failure.
"""

from __future__ import annotations

import argparse
import bisect
import concurrent.futures
import enum
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import harness  # noqa: E402

AXES = ("x", "y", "z", "e")

SAMPLES_PER_SPAN = 3


class Verdict(enum.Enum):
    IDENTICAL = "identical"
    WITHIN_TOL = "within-tol"
    SHAPE = "SHAPE"
    NEW = "NEW"


@dataclass(frozen=True)
class AxisDelta:
    axis: str
    pieces_before: int
    pieces_after: int
    max_dp: float
    max_dv: float
    max_da: float

    @property
    def pieces_change(self) -> int:
        return self.pieces_after - self.pieces_before


@dataclass(frozen=True)
class CaseDelta:
    name: str
    verdict: Verdict
    time_before: float
    time_after: float
    axes: list[AxisDelta] = field(default_factory=list)
    drift_rel: float = 0.0
    drift_rel_at: str = ""
    drift_abs: float = 0.0
    drift_abs_at: str = ""
    samples: int = 0

    @property
    def time_change(self) -> float:
        return self.time_after - self.time_before

    @property
    def time_change_rel(self) -> float:
        if self.time_before == 0.0:
            return 0.0
        return self.time_change / self.time_before

    @property
    def pieces_before(self) -> int:
        return sum(a.pieces_before for a in self.axes)

    @property
    def pieces_after(self) -> int:
        return sum(a.pieces_after for a in self.axes)

    @property
    def max_dp(self) -> float:
        return max((a.max_dp for a in self.axes), default=0.0)

    @property
    def max_dv(self) -> float:
        return max((a.max_dv for a in self.axes), default=0.0)

    @property
    def max_da(self) -> float:
        return max((a.max_da for a in self.axes), default=0.0)

    @property
    def rank(self) -> tuple:
        return (
            self.verdict is Verdict.SHAPE,
            abs(self.time_change_rel),
            self.drift_rel,
            self.drift_abs,
        )


def piece_state_at(
    piece: list[float], tau: float
) -> tuple[float, float, float]:
    """Mirror of `pipeline_snapshot::piece_state_at`: a piece row is
    `[t_start, t_end, c0, c1, ...]` and `tau` is time since `t_start`."""
    coeffs = piece[2:]
    pos = 0.0
    vel = 0.0
    acc = 0.0
    for k in range(len(coeffs) - 1, -1, -1):
        ck = coeffs[k]
        pos = pos * tau + ck
        if k >= 1:
            vel = vel * tau + k * ck
        if k >= 2:
            acc = acc * tau + k * (k - 1) * ck
    return pos, vel, acc


class AxisTrack:
    """Piecewise-polynomial trajectory for one axis, evaluable at any time in
    its span. Pieces are contiguous and ordered, as the snapshot emits them."""

    def __init__(self, axis: str, pieces: list[list[float]]) -> None:
        for row in pieces:
            if len(row) < 3:
                raise ValueError(
                    f"axis {axis}: piece row of length {len(row)}, "
                    "expected [t_start, t_end, c0, ...]"
                )
        for prev, nxt in zip(pieces, pieces[1:]):
            if abs(prev[1] - nxt[0]) > 1e-12:
                raise ValueError(
                    f"axis {axis}: piece gap {prev[1]!r} -> {nxt[0]!r}"
                )
        self.axis = axis
        self.pieces = pieces
        self.starts = [row[0] for row in pieces]

    @property
    def t_end(self) -> float:
        return self.pieces[-1][1] if self.pieces else 0.0

    def breakpoints(self) -> list[float]:
        if not self.pieces:
            return []
        return self.starts + [self.pieces[-1][1]]

    def state_at(self, t: float) -> tuple[float, float, float]:
        index = bisect.bisect_right(self.starts, t) - 1
        if index < 0:
            index = 0
        piece = self.pieces[index]
        return piece_state_at(piece, t - piece[0])


def _sample_times(before: AxisTrack, after: AxisTrack) -> list[float]:
    """Interior samples of every span delimited by either run's breakpoints,
    plus both ends. Interior points keep an acceleration step at a shared
    breakpoint from being read as a deviation between the two runs."""
    overlap = min(before.t_end, after.t_end)
    edges = sorted(
        {0.0, overlap}
        | {t for t in before.breakpoints() if 0.0 <= t <= overlap}
        | {t for t in after.breakpoints() if 0.0 <= t <= overlap}
    )
    times = [0.0]
    for lo, hi in zip(edges, edges[1:]):
        span = hi - lo
        if span <= 0.0:
            continue
        for i in range(1, SAMPLES_PER_SPAN + 1):
            times.append(lo + span * i / (SAMPLES_PER_SPAN + 1))
    times.append(overlap)
    return times


def axis_delta(axis: str, before: AxisTrack, after: AxisTrack) -> AxisDelta:
    max_dp = 0.0
    max_dv = 0.0
    max_da = 0.0
    if before.pieces and after.pieces:
        for t in _sample_times(before, after):
            p0, v0, a0 = before.state_at(t)
            p1, v1, a1 = after.state_at(t)
            max_dp = max(max_dp, abs(p1 - p0))
            max_dv = max(max_dv, abs(v1 - v0))
            max_da = max(max_da, abs(a1 - a0))
    return AxisDelta(
        axis=axis,
        pieces_before=len(before.pieces),
        pieces_after=len(after.pieces),
        max_dp=max_dp,
        max_dv=max_dv,
        max_da=max_da,
    )


def _tracks(snapshot: dict) -> dict[str, AxisTrack]:
    return {
        axis: AxisTrack(axis, snapshot[f"traj_{axis}_pieces"]) for axis in AXES
    }


def _verdict(baseline: dict, snapshot: dict) -> Verdict:
    if harness.canonical_json(baseline) == harness.canonical_json(snapshot):
        return Verdict.IDENTICAL
    if harness.snapshots_match(baseline, snapshot):
        return Verdict.WITHIN_TOL
    return Verdict.SHAPE


def characterise_case(case: harness.Case) -> CaseDelta:
    snapshot = harness.run_case(case)
    baseline = harness.baseline_snapshot(case)
    if baseline is None:
        return CaseDelta(
            name=case.name,
            verdict=Verdict.NEW,
            time_before=0.0,
            time_after=snapshot["traversal_time_s"],
        )
    before = _tracks(baseline)
    after = _tracks(snapshot)
    axes = [axis_delta(a, before[a], after[a]) for a in AXES]
    drift = harness.drift_envelope(baseline, snapshot)
    return CaseDelta(
        name=case.name,
        verdict=_verdict(baseline, snapshot),
        time_before=baseline["traversal_time_s"],
        time_after=snapshot["traversal_time_s"],
        axes=axes,
        drift_rel=drift["rel"],
        drift_rel_at=drift["rel_at"],
        drift_abs=drift["abs"],
        drift_abs_at=drift["abs_at"],
        samples=sum(
            len(_sample_times(before[a], after[a]))
            for a in AXES
            if before[a].pieces and after[a].pieces
        ),
    )


def characterise_parallel(
    cases: list[harness.Case], max_workers: int | None = None
):
    """Yields `CaseDelta` in completion order. Each worker samples its own case
    so only the small delta crosses the process boundary, not two full
    trajectories."""
    if len(cases) == 1:
        yield characterise_case(cases[0])
        return
    if max_workers is None:
        max_workers = min(len(cases), os.cpu_count() or 1)
    with concurrent.futures.ProcessPoolExecutor(
        max_workers=max_workers
    ) as pool:
        futures = [pool.submit(characterise_case, c) for c in cases]
        for fut in concurrent.futures.as_completed(futures):
            yield fut.result()


NAME_WIDTH = 34

HEADER = (
    f"{'case':<{NAME_WIDTH}} {'verdict':<10} {'t_before':>10} "
    f"{'t_after':>10} {'dt':>11} {'dt%':>9} {'pieces':>13} "
    f"{'max|dp|':>10} {'max|dv|':>10} {'max|da|':>10}"
)


def _pieces_cell(delta: CaseDelta) -> str:
    change = delta.pieces_after - delta.pieces_before
    if change == 0:
        return f"{delta.pieces_before}"
    return f"{delta.pieces_before}->{delta.pieces_after}"


def format_row(delta: CaseDelta) -> str:
    name = delta.name
    if len(name) > NAME_WIDTH:
        name = "…" + name[-(NAME_WIDTH - 1) :]
    return (
        f"{name:<{NAME_WIDTH}} {delta.verdict.value:<10} "
        f"{delta.time_before:>10.6f} {delta.time_after:>10.6f} "
        f"{delta.time_change:>+11.6f} {delta.time_change_rel * 100:>+8.4f}% "
        f"{_pieces_cell(delta):>13} "
        f"{delta.max_dp:>10.2e} {delta.max_dv:>10.2e} {delta.max_da:>10.2e}"
    )


def format_totals(deltas: list[CaseDelta]) -> str:
    time_before = sum(d.time_before for d in deltas)
    time_after = sum(d.time_after for d in deltas)
    pieces_before = sum(d.pieces_before for d in deltas)
    pieces_after = sum(d.pieces_after for d in deltas)
    rel = (time_after - time_before) / time_before if time_before else 0.0
    counts = {v: 0 for v in Verdict}
    for d in deltas:
        counts[d.verdict] += 1
    return (
        f"TOTALS {len(deltas)} cases: "
        f"{counts[Verdict.IDENTICAL]} identical, "
        f"{counts[Verdict.WITHIN_TOL]} within-tol, "
        f"{counts[Verdict.SHAPE]} shape-changed, "
        f"{counts[Verdict.NEW]} new | traversal "
        f"{time_before:.6f}s -> {time_after:.6f}s "
        f"({time_after - time_before:+.6f}s, {rel * 100:+.4f}%) | pieces "
        f"{pieces_before} -> {pieces_after} "
        f"({pieces_after - pieces_before:+d}) | worst "
        f"dp={max((d.max_dp for d in deltas), default=0.0):.3e}mm "
        f"dv={max((d.max_dv for d in deltas), default=0.0):.3e}mm/s "
        f"da={max((d.max_da for d in deltas), default=0.0):.3e}mm/s^2"
    )


def format_detail(delta: CaseDelta) -> list[str]:
    lines = [f"  {delta.name} [{delta.verdict.value}]"]
    for axis in delta.axes:
        lines.append(
            f"    {axis.axis}: pieces {axis.pieces_before}"
            f" -> {axis.pieces_after} ({axis.pieces_change:+d})"
            f"  max|dp|={axis.max_dp:.3e}"
            f"  max|dv|={axis.max_dv:.3e}"
            f"  max|da|={axis.max_da:.3e}"
        )
    if delta.drift_rel_at:
        lines.append(
            f"    worst relative field drift {delta.drift_rel:.3e} at "
            f"{delta.drift_rel_at}"
        )
    if delta.drift_abs_at:
        lines.append(
            f"    worst near-zero field drift {delta.drift_abs:.3e} at "
            f"{delta.drift_abs_at}"
        )
    lines.append(f"    samples compared: {delta.samples}")
    return lines


def as_json(delta: CaseDelta) -> dict:
    return {
        "case": delta.name,
        "verdict": delta.verdict.value,
        "traversal_time_s": {
            "before": delta.time_before,
            "after": delta.time_after,
            "change": delta.time_change,
            "change_rel": delta.time_change_rel,
        },
        "pieces": {
            "before": delta.pieces_before,
            "after": delta.pieces_after,
        },
        "max_deviation": {
            "position_mm": delta.max_dp,
            "velocity_mm_s": delta.max_dv,
            "accel_mm_s2": delta.max_da,
        },
        "axes": [
            {
                "axis": a.axis,
                "pieces_before": a.pieces_before,
                "pieces_after": a.pieces_after,
                "max_dp": a.max_dp,
                "max_dv": a.max_dv,
                "max_da": a.max_da,
            }
            for a in delta.axes
        ],
        "field_drift": {
            "rel": delta.drift_rel,
            "rel_at": delta.drift_rel_at,
            "abs": delta.drift_abs,
            "abs_at": delta.drift_abs_at,
        },
        "samples": delta.samples,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-k", dest="filter", help="only run cases whose name contains this"
    )
    parser.add_argument(
        "--detail",
        action="store_true",
        help="per-axis breakdown and worst-drifting field for every case that "
        "is not identical",
    )
    parser.add_argument(
        "--json",
        type=Path,
        help="also write the full structured delta for every case here",
    )
    args = parser.parse_args()

    try:
        cases = harness.discover_cases()
    except ValueError as exc:
        print(f"  ERROR   {exc}")
        return 2
    if args.filter:
        cases = [c for c in cases if args.filter in c.name]
    if not cases:
        print("no snapshot cases found under cases/")
        return 2

    started = time.monotonic()
    try:
        deltas = list(characterise_parallel(cases))
    except (ImportError, ValueError) as exc:
        print(f"  ERROR   {exc}")
        return 2
    elapsed = time.monotonic() - started

    deltas.sort(key=lambda d: d.rank, reverse=True)

    print(HEADER)
    print("-" * len(HEADER))
    for delta in deltas:
        print(format_row(delta))
    print()
    print(format_totals(deltas))
    print(
        f"tolerance: atol={harness.FLOAT_ATOL:g} rtol={harness.FLOAT_RTOL:g}"
        f" — 'identical'/'within-tol' both compare EXACT in run.py"
    )
    print(f"characterised {len(deltas)} cases in {elapsed:.1f}s")

    if args.detail:
        print()
        for delta in deltas:
            if delta.verdict is Verdict.IDENTICAL:
                continue
            for line in format_detail(delta):
                print(line)

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps([as_json(d) for d in deltas], indent=2) + "\n"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
