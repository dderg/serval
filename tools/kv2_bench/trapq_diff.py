"""First divergence between the live toolhead trapq and the batch one.

    uv run python -m tools.kv2_bench.trapq_diff RUN_DIR REFERENCE_DIR

Live: motion_report/dump_trapq messages in RUN_DIR/tap.jsonl (pi_agent
--trapqs toolhead). Batch: the trapq_append calls batch_capture.py recorded
(REFERENCE_DIR/batch_capture.json), split into the accel/cruise/decel
phases trapq_append stores. Phases are compared in order on duration, start
velocity, acceleration and direction, and on start time relative to the
first phase. A divergence is mapped back to the toolhead move and the gcode
line that made it.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

REL_TOL = 1e-9
ABS_TOL = 1e-9
TIME_TOL_S = 1e-6
AXIS = re.compile(r"([XYZ])(-?[\d.]+)", re.IGNORECASE)


def live_phases(run_dir: Path) -> list[dict]:
    seen = {}
    for line in (run_dir / "tap.jsonl").read_text().splitlines():
        message = json.loads(line)
        if "trapq" not in message:
            continue
        for pt, move_t, start_v, accel, start, axes in message["params"][
            "data"
        ]:
            if move_t > 0.0:
                seen[pt] = {
                    "t": pt,
                    "dur": move_t,
                    "v0": start_v,
                    "a": accel,
                    "start": start,
                    "axes": axes,
                }
    return [seen[k] for k in sorted(seen)]


def batch_phases(ref_dir: Path) -> list[dict]:
    moves = json.loads((ref_dir / "batch_capture.json").read_text())["moves"]
    phases = []
    for index, m in enumerate(moves):
        t, d = m["print_time"], 0.0
        axes = [m["axes_r_x"], m["axes_r_y"], m["axes_r_z"]]
        start = [m["start_x"], m["start_y"], m["start_z"]]
        for dur, v0, a in (
            (m["accel_t"], m["start_v"], m["accel"]),
            (m["cruise_t"], m["cruise_v"], 0.0),
            (m["decel_t"], m["cruise_v"], -m["accel"]),
        ):
            if dur > 0.0:
                phases.append(
                    {
                        "t": t,
                        "dur": dur,
                        "v0": v0,
                        "a": a,
                        "start": [s + r * d for s, r in zip(start, axes)],
                        "axes": axes,
                        "move": index,
                        "cruise_v": m["cruise_v"],
                        "max_cruise_v": m["max_cruise_v"],
                    }
                )
                t += dur
                d += (v0 + 0.5 * a * dur) * dur
    return phases


def close(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=REL_TOL, abs_tol=ABS_TOL)


def differs(live: dict, batch: dict, offset: float) -> list[str]:
    fields = [k for k in ("dur", "v0", "a") if not close(live[k], batch[k])]
    if not all(close(x, y) for x, y in zip(live["axes"], batch["axes"])):
        fields.append("axes")
    if abs(live["t"] - offset - batch["t"]) > TIME_TOL_S:
        fields.append("start_time")
    return fields


def gcode_lines_of_moves(program: str) -> list[tuple[int, str]]:
    """(line number, text) of every gcode line that made a toolhead move."""
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    absolute = True
    out = []
    for number, line in enumerate(program.splitlines(), 1):
        words = line.split()
        if not words:
            continue
        command = words[0].upper()
        if command == "SET_KINEMATIC_POSITION":
            for word in words[1:]:
                key, _, value = word.partition("=")
                pos[key.upper()] = float(value)
        elif command == "G90":
            absolute = True
        elif command == "G91":
            absolute = False
        elif command in ("G0", "G1"):
            target = dict(pos)
            for axis, value in AXIS.findall(line):
                axis = axis.upper()
                target[axis] = (
                    float(value) if absolute else pos[axis] + float(value)
                )
            if any(abs(target[a] - pos[a]) > 1e-9 for a in pos):
                out.append((number, line))
            pos = target
    return out


def stats_around(log: str, print_time: float, span: float = 3.0) -> list[str]:
    rows = []
    for line in log.splitlines():
        match = re.search(r"print_time=([\d.]+) buffer_time=([\d.]+)", line)
        if line.startswith("Stats") and match:
            if abs(float(match.group(1)) - print_time) <= span:
                rows.append(
                    re.sub(r" mcu: .*?(print_time)", r" ... \1", line)[:200]
                )
    return rows


def diff(run_dir: Path, ref_dir: Path) -> dict:
    live, batch = live_phases(run_dir), batch_phases(ref_dir)
    offset = live[0]["t"] - batch[0]["t"]
    first = None
    for i, (lp, bp) in enumerate(zip(live, batch)):
        fields = differs(lp, bp, offset)
        if fields:
            first = i
            break
    result: dict = {
        "live_phases": len(live),
        "batch_phases": len(batch),
        "offset_s": offset,
        "first_divergence": first,
    }
    if first is None:
        result["identical"] = len(live) == len(batch)
        return result
    move = batch[first]["move"]
    lines = gcode_lines_of_moves((ref_dir / "program.gcode").read_text())
    context = range(max(0, first - 3), min(len(batch), first + 4))
    result.update(
        fields=differs(live[first], batch[first], offset),
        batch_move=move,
        batch_time_s=batch[first]["t"],
        live_print_time=live[first]["t"],
        gcode=[
            {"move": m, "line": lines[m][0], "text": lines[m][1]}
            for m in range(max(0, move - 3), min(len(lines), move + 3))
        ],
        phases=[
            {
                "index": i,
                "live": {k: live[i][k] for k in ("t", "dur", "v0", "a")}
                | {"t": live[i]["t"] - offset},
                "batch": {
                    k: batch[i][k] for k in ("t", "dur", "v0", "a", "move")
                },
            }
            for i in context
        ],
        klippy_stats=stats_around(
            (run_dir / "klippy.log").read_text(), live[first]["t"]
        ),
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("reference_dir", type=Path)
    args = parser.parse_args()
    result = diff(args.run_dir, args.reference_dir)
    (args.run_dir / "trapq_diff.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
