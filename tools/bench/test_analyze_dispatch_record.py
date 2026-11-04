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


def _require_snapshot_engine() -> None:
    try:
        analyze.harness._import_engine()
    except ImportError as exc:
        pytest.skip(str(exc))


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
        out += np.ascontiguousarray(samples, "<f8").tobytes()
    return bytes(out)


def _plan_record(
    job: analyze.Job, waypoints: list[dict]
) -> dispatch_record.DispatchRecord:
    engine = analyze.harness._import_engine()
    return dispatch_record.parse(
        bytes(
            engine.pipeline_snapshot_dispatch_record(
                waypoints, job.config.config_text, job.step_benchmarks
            )
        )
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
    _require_snapshot_engine()
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
    _require_snapshot_engine()
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


def test_a_record_of_the_plan_matches_it_and_keeps_the_plan_as_reference(
    tmp_path,
):
    _require_snapshot_engine()
    job = analyze.load_job(_write_run(tmp_path, None))
    plan = _plan_record(job, analyze.plan_waypoints(job).waypoints)
    run_dir = _write_run(tmp_path, replace(plan, t0=np.full(len(plan), 7.0)))
    job = analyze.load_job(run_dir)
    assert analyze.build_reference(job).source == analyze.REFERENCE_PLANNED
    report = analyze.compare_dispatch_with_plan(job, job.dispatch_record)
    assert report["matches_plan_outside_brakes"]
    assert report["matched_segments"] == report["segments"] == len(plan)
    assert report["brake_windows"] == []


def test_the_plans_own_record_steps_like_the_plan(tmp_path):
    _require_snapshot_engine()
    job = analyze.load_job(_write_run(tmp_path, None))
    plan = _plan_record(job, analyze.plan_waypoints(job).waypoints)
    planned = analyze.planned_blocks(job, analyze.plan_waypoints(job))
    recorded = analyze.dispatched_blocks(job, plan)
    assert [b.duration_s for b in recorded] == pytest.approx(
        [b.duration_s for b in planned], abs=1e-9
    )
    for motor in job.motors:
        p, _, _ = analyze._motor_steps(motor, planned)
        r, _, _ = analyze._motor_steps(motor, recorded)
        assert r.level.tolist() == p.level.tolist()
        shift_steps = np.abs(r.time - p.time) * np.maximum(r.rate_hz, p.rate_hz)
        assert np.all(shift_steps < 1e-3)


def test_a_record_with_a_nudge_is_refused(tmp_path):
    _write_run(tmp_path, None)
    (tmp_path / "dispatch_record_p1_s0.bin").write_bytes(
        dispatch_record.HEADER.pack(dispatch_record.MAGIC, 4, 0.5e-3)
        + bytes([dispatch_record.KIND_NUDGE])
        + dispatch_record.NUDGE.pack(1.0, 0.0, 0.1, 0, 2)
        + bytes([dispatch_record.KIND_SEGMENT])
        + dispatch_record.SEGMENT.pack(1.0, 0.0, 0.1, 5, 1, 2)
        + bytes(2 * 4 * 3 * 8)
    )
    with pytest.raises(analyze.AnalysisError, match="1 nudges bypassed"):
        analyze.load_job(tmp_path)


def test_only_one_record_may_hold_the_print(tmp_path):
    _write_run(tmp_path, None)
    empty = dispatch_record.HEADER.pack(dispatch_record.MAGIC, 4, 0.5e-3)
    moving = (
        empty
        + bytes([dispatch_record.KIND_SEGMENT])
        + dispatch_record.SEGMENT.pack(1.0, 0.0, 0.1, 5, 1, 2)
        + bytes(2 * 4 * 3 * 8)
    )
    (tmp_path / "dispatch_record_p1_s0.bin").write_bytes(empty)
    (tmp_path / "dispatch_record_p1_s1.bin").write_bytes(moving)
    assert len(analyze.load_job(tmp_path).dispatch_record) == 1
    (tmp_path / "dispatch_record_p1_s2.bin").write_bytes(moving)
    with pytest.raises(
        analyze.AnalysisError, match="expected one dispatch record"
    ):
        analyze.load_job(tmp_path)
