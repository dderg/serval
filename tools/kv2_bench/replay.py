"""Reference from the live trapq: regenerate the step stream a live run's
planned motion should have produced, and compare it with the live tap.

    uv run python -m tools.kv2_bench.replay RUN_DIR --tree KLIPPY_TREE --dictionary DICT

Batch mode cannot reproduce a long live print step-for-step, because the
live lookahead flushes on real time (see RESULTS.md, x15 divergence). This
reference starts after the planner. It takes the toolhead (and extruder)
trapq the live host recorded and regenerates steps offline through the
same chelper itersolve + input shaper + stepcompress path
(replay_capture.py), with the run's printer.cfg. So it judges step
generation, transport and MCU execution. trapq_diff.py judges the planner
separately (live trapq vs batch trapq).

Every queue_step lies in [ideal - 25 us, ideal] for the live host and the
replay alike, and both run on the live print_time axis. So replay and live
tap must have identical step sequences and agree within
MAX_STEPCOMPRESS_ERROR. Only the step-generation batch boundaries differ.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from . import compare, trapq_diff
from .reference import Reference, batch_config, decode, run_batch

KINEMATIC = ("stepper_x", "stepper_y", "stepper_z")


def replayed_steppers(trapqs: list[str]) -> tuple[str, ...]:
    if "toolhead" not in trapqs:
        raise ValueError("the run has no toolhead trapq tap")
    return KINEMATIC + tuple(n for n in trapqs if n != "toolhead")


def replay_phases(
    phases: dict[str, list[dict]],
    config_text: str,
    tree: Path,
    dictionary: Path,
    out_dir: Path,
    meta: dict,
) -> Reference:
    out_dir.mkdir(parents=True, exist_ok=True)
    phases = {name: p for name, p in phases.items() if p}
    phases_path = out_dir / "phases.json"
    phases_path.write_text(json.dumps(phases))
    config_path = out_dir / "batch.cfg"
    config_path.write_text(batch_config(config_text))
    gcode_path = out_dir / "replay.gcode"
    gcode_path.write_text("REPLAY_LIVE_TRAPQ\n")
    capture, output_path = run_batch(
        "tools.kv2_bench.replay_capture",
        tree,
        config_path,
        gcode_path,
        dictionary,
        out_dir,
        (str(phases_path),),
    )
    steppers, freq = decode(capture, output_path, dictionary)
    wanted = replayed_steppers(sorted(phases))
    reference = Reference(
        {name: steppers[name] for name in wanted},
        freq,
        {
            **meta,
            "klippy_tree": str(tree),
            "trapqs": {name: len(p) for name, p in phases.items()},
            "input_shaper": capture["input_shaper"],
        },
    )
    reference.save(out_dir)
    return reference


def build(
    run_dir: Path, tree: Path, dictionary: Path, out_dir: Path
) -> Reference:
    trapqs = trapq_diff.trapq_names(run_dir)
    return replay_phases(
        {name: trapq_diff.live_phases(run_dir, name) for name in trapqs},
        (run_dir / "printer.cfg").read_text(),
        tree,
        dictionary,
        out_dir,
        {"run_dir": str(run_dir)},
    )


def self_check(ref_dir: Path, tree: Path, dictionary: Path) -> dict:
    """Replay the trapqs a batch reference recorded and require the batch's
    own step stream back: exercises the replay, extruder included, offline."""
    capture = json.loads((ref_dir / "batch_capture.json").read_text())
    replayed = replay_phases(
        capture["trapqs"],
        (ref_dir / "batch.cfg").read_text(),
        tree,
        dictionary,
        ref_dir / "self_check",
        {"reference": str(ref_dir)},
    )
    batch = Reference.load(ref_dir)
    executed = {n: batch.steppers[n] for n in replayed.steppers}
    seq = compare.sequences(replayed.steppers, executed)
    result = {"sequences": seq}
    if seq["all_steps_paired"]:
        result["identity"] = compare.schedule_identity(
            replayed.steppers, executed, replayed.clock_freq
        )
    return result


def judge(run_dir: Path, reference: Reference) -> dict:
    """Replay vs live tap (step generation), and capture vs replay with the
    same bound as compare.py: 25 us + this run's MCU lateness + 0.1 us."""
    freq = reference.clock_freq
    tap = compare.load_tap(run_dir)
    planned = dict(reference.steppers)
    executed = {n: tap[n] for n in planned}
    seq = compare.sequences(planned, executed)
    result: dict = {"replay_vs_tap": {"sequences": seq}}
    checks = {"replay_equals_tap_sequence": seq["all_steps_paired"]}
    if seq["all_steps_paired"]:
        identity = compare.schedule_identity(planned, executed, freq)
        result["replay_vs_tap"]["identity"] = identity
        checks["replay_within_max_error"] = (
            identity["abs_max_us"] <= compare.MAX_STEPCOMPRESS_ERROR_S * 1e6
            and identity["epochs"] == 1
        )
    if (run_dir / "events.bin").exists():
        captured = compare.load_capture(run_dir)
        cap = {n: captured[n] for n in planned}
        cap_seq = compare.sequences(planned, cap)
        result["capture_vs_replay"] = {"sequences": cap_seq}
        checks["capture_equals_replay_sequence"] = cap_seq["all_steps_paired"]
        tap_seq = compare.sequences(executed, cap)
        if cap_seq["all_steps_paired"] and tap_seq["all_steps_paired"]:
            _, _, residual = compare.timeline(executed, cap, freq, None, True)
            lateness = compare.mcu_lateness(residual)
            stats, _, _ = compare.timeline(planned, cap, freq, None, True)
            bound_us = (
                compare.MAX_STEPCOMPRESS_ERROR_S + compare.INSTRUMENT_SLACK_S
            ) * 1e6 + lateness["max_us"]
            stats.update(mcu_lateness=lateness, bound_us=bound_us)
            result["capture_vs_replay"]["timing"] = stats
            checks["capture_within_bound"] = stats["abs_max_us"] <= bound_us
    result["checks"] = checks
    result["ok"] = all(checks.values())
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--tree", type=Path, required=True)
    parser.add_argument("--dictionary", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    out = args.out or args.run_dir / "replay"
    reference = build(args.run_dir, args.tree, args.dictionary, out)
    result = judge(args.run_dir, reference)
    (out / "replay_judgement.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
