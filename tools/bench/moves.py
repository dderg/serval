"""Per-move timing error table for one analyzed bench run.

uv run python tools/bench/moves.py <run_dir> [--epochs 1-3]

Writes run_dir/moves.tsv: one row per (epoch, gcode move, motor) with the
number of matched steps and the median/abs-max capture-minus-reference
timing error in microseconds.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import analyze
import numpy as np


def move_spans(job: analyze.Job, waypoints: list[tuple]) -> list[dict]:
    engine = analyze.harness._import_engine()
    snapshot, _ = engine.pipeline_snapshot_axis_samples(
        waypoints, job.config.config_text, [0], 2
    )
    return json.loads(snapshot)["trajectory"]["spans"]


def span_index(spans: list[dict], t_block_s: np.ndarray) -> np.ndarray:
    starts = np.array([s["t_start"] for s in spans])
    return np.clip(np.searchsorted(starts, t_block_s, "right") - 1, 0, None)


def move_rows(run_dir: Path, epochs: range) -> list[tuple]:
    inputs = analyze.load_run(run_dir)
    reference = analyze.build_reference(inputs)
    start, waypoint_blocks = analyze.split_gcode(inputs.gcode_text)
    _, _, waypoints = analyze.plan_waypoints(inputs)
    block_moves = [
        [ln for ln in text.splitlines() if ln.startswith("G1")]
        for text in waypoint_blocks
    ]
    capture_events = {
        m.role: analyze.capture_steps(
            inputs.capture, m, reference.start_counts[m.role]
        )
        for m in inputs.motors
    }
    results = analyze.align_all(capture_events, reference, 0.15)
    rows = []
    for index in epochs:
        result = results[index]
        block = reference.blocks[index]
        spans = move_spans(inputs, waypoints[index])
        for role, match in result.matches.items():
            if not match.late_s.size:
                continue
            stream_s = result.alignment.to_stream(
                match.capture_time_s - match.late_s
            )
            owner = span_index(spans, stream_s - block.stream_offset_s)
            move = np.array([spans[i]["source_start_line"] for i in owner])
            for line in np.unique(move):
                late_us = match.late_s[move == line] * 1e6
                rows.append(
                    (
                        index,
                        int(line),
                        block_moves[index][line],
                        role,
                        int(late_us.size),
                        float(np.median(late_us)),
                        float(np.abs(late_us).max()),
                    )
                )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--first-epoch", type=int, default=0)
    parser.add_argument("--last-epoch", type=int, default=2)
    args = parser.parse_args()
    rows = move_rows(args.run_dir, range(args.first_epoch, args.last_epoch + 1))
    header = "epoch\tmove\tline\tmotor\tn\tmedian_us\tabs_max_us"
    body = [
        f"{e}\t{s}\t{line}\t{role}\t{n}\t{med:.1f}\t{mx:.1f}"
        for e, s, line, role, n, med, mx in rows
    ]
    (args.run_dir / "moves.tsv").write_text("\n".join([header, *body]) + "\n")
    print("\n".join([header, *body]))


if __name__ == "__main__":
    main()
