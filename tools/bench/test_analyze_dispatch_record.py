from __future__ import annotations

import json
import shutil
from dataclasses import replace
from pathlib import Path

import analyze
import dispatch_record
import numpy as np
import pytest

BENCH_DIR = Path(__file__).resolve().parent
TICK_HZ = 40_000.0
INJECTED_PPM = 37.0
JITTER_US = 0.5e6 / TICK_HZ
HOST_ORIGIN_S = 5_000.0
RESUME_AFTER_S = 1.5
CAPTURE_LEAD_S = 0.4
BRAKE_AT_WAYPOINT = 14
STRAYED_WAYPOINT = 4
DENSE_SAMPLE_PERIOD_S = 4e-6


def _zigzag_gcode() -> str:
    lines = [
        "SET_KINEMATIC_POSITION X=0 Y=0 Z=0",
        "G92 E0",
        "M83",
        "G1 X5 Y5 F6000",
    ]
    for row in range(12):
        y = 5 + 2 * row
        x = 25 if row % 2 == 0 else 5
        lines.append(f"G1 X{x} Y{y} E0.6 F4800")
        lines.append(f"G1 X{x} Y{y + 2} E0.06")
    return "\n".join(lines) + "\n"


def _encode(record: dispatch_record.DispatchRecord) -> bytes:
    out = bytearray(
        dispatch_record.HEADER.pack(
            dispatch_record.MAGIC, record.axes, record.sample_interval_s
        )
    )
    drains = list(record.drains)
    for index in range(len(record) + 1):
        while drains and drains[0].after_segments == index:
            d = drains.pop(0)
            out += bytes([dispatch_record.KIND_DRAIN])
            out += dispatch_record.DRAIN.pack(d.cause, d.moves_sent)
        if index == len(record):
            break
        samples = record.segment_samples(index)
        out += bytes([dispatch_record.KIND_SEGMENT])
        out += dispatch_record.SEGMENT.pack(
            record.t0[index],
            record.t_start[index],
            record.t_end[index],
            int(record.source_line[index]),
            int(record.flags[index]),
            len(samples),
        )
        rows = np.column_stack(
            [record.segment_times(index), samples.reshape(len(samples), -1)]
        )
        out += np.ascontiguousarray(rows, "<f8").tobytes()
    return bytes(out)


def _plan_record(
    job: analyze.Job, waypoints: list[dict]
) -> dispatch_record.DispatchRecord:
    return analyze.planned_dispatch_record(
        job, replace(analyze.plan_waypoints(job), waypoints=waypoints)
    )


def _live_record_braked_at(
    job: analyze.Job, waypoint: int
) -> dispatch_record.DispatchRecord:
    """What the host dispatches when the pacer brakes before `waypoint`: the
    plan with a stop at rest there, resumed RESUME_AFTER_S later on a fresh
    anchor."""
    waypoints = [dict(w) for w in analyze.plan_waypoints(job).waypoints]
    plain = _plan_record(job, waypoints)
    waypoints[waypoint]["dwell_before_s"] = 0.0
    braked = _plan_record(job, waypoints)
    planned_stops = {d.moves_sent for d in plain.drains}
    (brake,) = [d for d in braked.drains if d.moves_sent not in planned_stops]
    drains = [
        replace(d, cause=dispatch_record.CAUSE_PACER_BRAKE) if d == brake else d
        for d in braked.drains
    ]
    resumed = np.arange(len(braked)) >= brake.after_segments
    flags = braked.flags.copy()
    flags[brake.after_segments] |= dispatch_record.FLAG_FRESH_EPOCH
    return replace(
        braked,
        t0=HOST_ORIGIN_S + np.where(resumed, RESUME_AFTER_S, 0.0),
        flags=flags,
        drains=drains,
    )


def _write_run(
    tmp_path: Path, record: dispatch_record.DispatchRecord | None
) -> Path:
    shutil.copy(BENCH_DIR / "printer.cfg", tmp_path / "printer.cfg")
    (tmp_path / "print.gcode").write_text(_zigzag_gcode())
    if record is not None:
        (tmp_path / "dispatch_record_p1_s0.bin").write_bytes(_encode(record))
    return tmp_path


def _synthesize_capture(job: analyze.Job, reference: analyze.Reference) -> None:
    rng = np.random.default_rng(11)
    scale = 1.0 + INJECTED_PPM * 1e-6
    first = min(ev.time.min() for ev in reference.events.values() if len(ev))
    times = {
        m.role: CAPTURE_LEAD_S
        + scale * (reference.events[m.role].time - first)
        + rng.uniform(-JITTER_US, JITTER_US, len(reference.events[m.role]))
        * 1e-6
        for m in job.motors
    }
    last = max(t.max() for t in times.values() if t.size)
    deltas = np.zeros((int((last + 0.5) * TICK_HZ), 4), np.int64)
    for motor in job.motors:
        ticks = np.floor(times[motor.role] * TICK_HZ).astype(np.int64)
        signed = reference.events[motor.role].direction * motor.dir_sign
        np.add.at(deltas[:, motor.channel], ticks, signed)
    deltas.astype(np.int8).tofile(job.run_dir / "capture.bin")
    (job.run_dir / "capture.json").write_text(
        json.dumps({"tick_hz": TICK_HZ, "channel_count": 4})
    )


@pytest.fixture(scope="module")
def braked_run(tmp_path_factory):
    run_dir = _write_run(tmp_path_factory.mktemp("braked"), None)
    record = _live_record_braked_at(
        analyze.load_job(run_dir), BRAKE_AT_WAYPOINT
    )
    (run_dir / "dispatch_record_p1_s0.bin").write_bytes(_encode(record))
    job = analyze.load_job(run_dir)
    reference = analyze.build_reference(job)
    _synthesize_capture(job, reference)
    verdict, _ = analyze.analyze(analyze.load_run(run_dir))
    return job, reference, verdict


def test_a_braked_run_is_judged_against_what_the_host_dispatched(braked_run):
    _, reference, verdict = braked_run
    assert reference.source == analyze.REFERENCE_DISPATCHED
    assert verdict["reference_source"] == analyze.REFERENCE_DISPATCHED
    assert len(verdict["epochs"]) == 2
    for check in (
        "net_steps_match",
        "no_stalls",
        "all_steps_matched",
        analyze.TIMING_CHECK,
        "pace_matches_reference",
        "dispatch_matches_plan_outside_brakes",
    ):
        assert verdict["checks"][check], check


def test_a_braked_run_fails_on_its_pacer_brakes(braked_run):
    _, _, verdict = braked_run
    assert not verdict["checks"]["no_pacer_brakes"]
    assert not verdict["ok"]
    dispatched = verdict["dispatch_record"]
    assert dispatched["pacer_brakes"] == 1
    (window,) = dispatched["brake_windows"]
    assert window["pacer_brakes"] == 1
    assert 0.0 < window["motion_s"] < 1.0
    assert dispatched["matched_segments"] > dispatched["segments"] // 2


def test_the_reference_holds_the_rest_the_brake_inserted(braked_run):
    _, reference, _ = braked_run
    first, second = reference.blocks
    gap_s = second.stream_offset_s - (first.stream_offset_s + first.duration_s)
    assert gap_s == pytest.approx(RESUME_AFTER_S, abs=1e-9)


def test_a_dispatched_segment_off_the_plan_away_from_brakes_is_a_divergence(
    tmp_path,
):
    job = analyze.load_job(_write_run(tmp_path, None))
    record = _live_record_braked_at(job, BRAKE_AT_WAYPOINT)
    strayed_line = int(np.unique(record.source_line)[STRAYED_WAYPOINT])
    index = int(np.flatnonzero(record.source_line == strayed_line)[0])
    samples = record.samples.copy()
    rows = slice(record.sample_start[index], record.sample_start[index + 1])
    samples[rows, 0, 0] += 0.1
    report = analyze.compare_dispatch_with_plan(
        job, replace(record, samples=samples)
    )
    assert not report["matches_plan_outside_brakes"]
    (divergence,) = report["first_divergences"]
    assert divergence["source_lines"][0] == strayed_line
    assert divergence["max_deviation_steps"] == pytest.approx(
        0.1 / job.motors[0].step_mm, rel=1e-6
    )
    assert len(report["brake_windows"]) == 1


@pytest.mark.parametrize(
    ("waypoint", "judged"),
    [(STRAYED_WAYPOINT, True), (BRAKE_AT_WAYPOINT + 6, False)],
)
def test_the_extruder_is_judged_only_until_the_first_brake(
    tmp_path, waypoint, judged
):
    job = analyze.load_job(_write_run(tmp_path, None))
    record = _live_record_braked_at(job, BRAKE_AT_WAYPOINT)
    line = int(np.unique(record.source_line)[waypoint])
    index = int(np.flatnonzero(record.source_line == line)[0])
    samples = record.samples.copy()
    rows = slice(record.sample_start[index], record.sample_start[index + 1])
    samples[rows, analyze.EXTRUDER_AXIS, 0] += np.linspace(
        0.0, 0.01, rows.stop - rows.start
    )
    report = analyze.compare_dispatch_with_plan(
        job, replace(record, samples=samples)
    )
    assert report["matches_plan_outside_brakes"] is not judged
    replanned = report["max_extruder_deviation_after_brakes_steps"]
    assert (
        replanned > analyze.POSITION_EQUIVALENT_TOLERANCE_STEPS
    ) is not judged


def test_a_record_of_the_plan_matches_it_and_keeps_the_plan_as_reference(
    tmp_path,
):
    job = analyze.load_job(_write_run(tmp_path, None))
    plan = _plan_record(job, analyze.plan_waypoints(job).waypoints)
    run_dir = _write_run(tmp_path, replace(plan, t0=np.full(len(plan), 7.0)))
    job = analyze.load_job(run_dir)
    assert analyze.build_reference(job).source == analyze.REFERENCE_PLANNED
    report = analyze.compare_dispatch_with_plan(job, job.dispatch_record)
    assert report["matches_plan_outside_brakes"]
    assert report["matched_segments"] == report["segments"] == len(plan)
    assert report["brake_windows"] == []


def _densely_sampled_steps(
    job: analyze.Job, planned: analyze.PlannedPrint, motor: analyze.Motor
) -> analyze.StepEvents:
    ((t_start, t_end, rows),) = (
        analyze.harness._import_engine().pipeline_snapshot_epoch_samples(
            planned.waypoints,
            job.config.config_text,
            [0, 1, 2, 3],
            DENSE_SAMPLE_PERIOD_S,
            job.step_benchmarks,
        )
    )
    positions = np.asarray(rows)[:, :, 0]
    steps = np.asarray(motor.axis_weights) @ positions / motor.step_mm
    times = np.linspace(0.0, t_end - t_start, steps.size)
    state = analyze.PlayState.resting_at(steps[0])
    return analyze.lattice_steps(times, steps, state)


def test_the_reference_steps_where_the_plan_crosses_each_step(tmp_path):
    job = analyze.load_job(_write_run(tmp_path, None))
    planned = analyze.plan_waypoints(job)
    reference = analyze.planned_blocks(job, planned)
    for motor in job.motors:
        steps, _, _ = analyze._motor_steps(motor, reference)
        truth = _densely_sampled_steps(job, planned, motor)
        assert steps.level.tolist() == truth.level.tolist(), motor.role
        shift_steps = np.abs(steps.time - truth.time) * truth.rate_hz
        assert shift_steps.max(initial=0.0) < 1e-3, motor.role


def _moving_segment() -> bytes:
    span = 0.5e-3
    rows = np.zeros((2, 1 + 4 * 3))
    rows[1, 0] = span
    return (
        bytes([dispatch_record.KIND_SEGMENT])
        + dispatch_record.SEGMENT.pack(1.0, 0.0, span, 5, 1, 2)
        + rows.astype("<f8").tobytes()
    )


def test_a_record_with_a_nudge_is_refused(tmp_path):
    (tmp_path / "dispatch_record_p1_s0.bin").write_bytes(
        dispatch_record.HEADER.pack(dispatch_record.MAGIC, 4, 0.5e-3)
        + bytes([dispatch_record.KIND_NUDGE])
        + dispatch_record.NUDGE.pack(1.0, 0.0, 0.1, 0, 2)
        + _moving_segment()
    )
    with pytest.raises(analyze.AnalysisError, match="1 nudges bypassed"):
        analyze.load_dispatch_record(tmp_path)


def test_only_one_record_may_hold_the_print(tmp_path):
    empty = dispatch_record.HEADER.pack(dispatch_record.MAGIC, 4, 0.5e-3)
    moving = empty + _moving_segment()
    (tmp_path / "dispatch_record_p1_s0.bin").write_bytes(empty)
    (tmp_path / "dispatch_record_p1_s1.bin").write_bytes(moving)
    assert len(analyze.load_dispatch_record(tmp_path)) == 1
    (tmp_path / "dispatch_record_p1_s2.bin").write_bytes(moving)
    with pytest.raises(
        analyze.AnalysisError, match="expected one dispatch record"
    ):
        analyze.load_dispatch_record(tmp_path)
