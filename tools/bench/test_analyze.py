from __future__ import annotations

import json
import shutil
from pathlib import Path

import analyze
import numpy as np
import pytest

BENCH_DIR = Path(__file__).resolve().parent
TICK_HZ = 40_000.0
INJECTED_PPM = 37.0
FIRST_EPOCH_CAPTURE_START_S = 0.4
INTER_EPOCH_IDLE_S = 0.8
JITTER_US = 0.5e6 / TICK_HZ
STALLED_Z_STEPS = slice(150, 210)
STALL_S = 15e-3
GCODE = """\
SET_KINEMATIC_POSITION X=0 Y=0 Z=0
G92 E0
M83
G1 X20 Y5 F3000
G1 X40 Y20 E1
G1 Z1 F600
G1 X0 Y0 E0.5 F3000
M400
G1 X10 E0.4 F3000
"""


def _block_alignments(
    reference: analyze.Reference, idles_s: list[float] | None = None
) -> list[analyze.Alignment]:
    alignments = []
    capture_start = FIRST_EPOCH_CAPTURE_START_S
    idles = idles_s or [INTER_EPOCH_IDLE_S] * len(reference.blocks)
    for block, idle_s in zip(reference.blocks, idles):
        first_step = min(
            ev.time[ev.time >= block.stream_offset_s].min()
            for ev in reference.events.values()
            if np.any(ev.time >= block.stream_offset_s)
        )
        scale = 1.0 + INJECTED_PPM * 1e-6
        alignment = analyze.Alignment(
            capture_start - scale * first_step, INJECTED_PPM, True
        )
        alignments.append(alignment)
        capture_start = (
            alignment.to_capture(block.stream_offset_s + block.duration_s)
            + idle_s
        )
    return alignments


def _true_capture_times(
    reference: analyze.Reference,
    alignments: list[analyze.Alignment],
    events: analyze.StepEvents,
) -> np.ndarray:
    block_starts = np.array([b.stream_offset_s for b in reference.blocks])
    block = np.searchsorted(block_starts, events.time, "right") - 1
    offsets = np.array([a.offset_s for a in alignments])
    return offsets[block] + (1.0 + INJECTED_PPM * 1e-6) * events.time


def _synthesize_capture(
    job: analyze.Job,
    reference: analyze.Reference,
    alignments: list[analyze.Alignment],
    stalled: bool = True,
) -> np.ndarray:
    rng = np.random.default_rng(7)
    times = {}
    for motor in job.motors:
        events = reference.events[motor.role]
        t = _true_capture_times(reference, alignments, events)
        t = t + rng.uniform(-JITTER_US, JITTER_US, t.size) * 1e-6
        if motor.role == "z" and stalled:
            t[STALLED_Z_STEPS] += STALL_S
        times[motor.role] = t
    last = max(t.max() for t in times.values())
    deltas = np.zeros((int((last + 0.5) * TICK_HZ), 4), np.int64)
    for motor in job.motors:
        ticks = np.floor(times[motor.role] * TICK_HZ).astype(np.int64)
        signed = reference.events[motor.role].direction * motor.dir_sign
        np.add.at(deltas[:, motor.channel], ticks, signed)
    assert np.abs(deltas).max() <= 127
    return deltas.astype(np.int8)


def _require_snapshot_engine() -> None:
    try:
        analyze.harness._import_engine()
    except ImportError as exc:
        pytest.skip(str(exc))


@pytest.fixture(scope="module")
def synthesized(tmp_path_factory):
    _require_snapshot_engine()
    run_dir = tmp_path_factory.mktemp("run")
    shutil.copy(BENCH_DIR / "printer.cfg", run_dir / "printer.cfg")
    (run_dir / "print.gcode").write_text(GCODE)
    job = analyze.load_job(run_dir)
    reference = analyze.build_reference(job)
    alignments = _block_alignments(reference)
    _synthesize_capture(job, reference, alignments).tofile(
        run_dir / "capture.bin"
    )
    (run_dir / "capture.json").write_text(
        json.dumps({"tick_hz": TICK_HZ, "channel_count": 4})
    )
    verdict, _ = analyze.analyze(analyze.load_run(run_dir))
    stalled = reference.events["z"].time[STALLED_Z_STEPS]
    stall_window = (
        alignments[0].to_capture(stalled[0]),
        alignments[0].to_capture(stalled[-1]),
    )
    return verdict, alignments, stall_window


def test_recovers_per_epoch_offset_and_scale(synthesized):
    verdict, alignments, _ = synthesized
    epochs = verdict["epochs"]
    assert len(epochs) == len(alignments) == 2
    assert epochs[0]["scale_fitted"]
    assert epochs[0]["ppm"] == pytest.approx(INJECTED_PPM, abs=1.0)
    assert not epochs[1]["scale_fitted"]
    assert epochs[1]["ppm"] == pytest.approx(epochs[0]["ppm"])
    for epoch, truth in zip(epochs, alignments):
        recovered = analyze.Alignment(epoch["offset_s"], epoch["ppm"], True)
        start = epoch["stream_start_s"]
        assert recovered.to_capture(start) == pytest.approx(
            truth.to_capture(start), abs=3e-6
        )


def test_recovers_injected_timing_errors(synthesized):
    verdict, _, _ = synthesized
    quantization_us = 0.5e6 / TICK_HZ
    for role, motor in verdict["motors"].items():
        assert motor["net_steps_capture_minus_expected"] == 0
        assert motor["unmatched_capture_steps"] == 0
        assert motor["unmatched_reference_steps"] == 0
        stats = motor["timing_error_capture_minus_reference_us"]
        if role == "z":
            assert stats["max_late"] == pytest.approx(STALL_S * 1e6, rel=0.1)
            assert motor["timing_violation_steps"] > 0
            assert motor["timing_beyond_time_bound_worst_position"][
                "timing_error_us"
            ] == pytest.approx(STALL_S * 1e6, rel=0.1)
        else:
            assert stats["abs_max"] <= JITTER_US + quantization_us + 2.0
            assert abs(stats["p50"]) <= 2.0
            assert motor["timing_violation_steps"] == 0
    assert not verdict["checks"][analyze.TIMING_CHECK]


TURNAROUND_NEIGHBOUR_RATE_HZ = 2900.0
TURNAROUND_RATE_HZ = 227.0


def _timing_report_with(late_s: float, rate_hz: float) -> dict:
    neighbours = 20
    late = np.full(2 * neighbours + 1, 1e-6)
    rate = np.full(late.size, TURNAROUND_NEIGHBOUR_RATE_HZ)
    late[neighbours] = late_s
    rate[neighbours] = rate_hz
    capture_time = np.arange(late.size) / TURNAROUND_NEIGHBOUR_RATE_HZ
    return analyze.timing_report(late, capture_time, rate)


def test_a_turnaround_step_far_off_in_time_but_close_in_position_passes():
    report = _timing_report_with(-57.3e-6, TURNAROUND_RATE_HZ)
    assert report["timing_error_capture_minus_reference_us"][
        "abs_max"
    ] == pytest.approx(57.3)
    assert report["timing_beyond_time_bound_steps"] == 1
    assert report["timing_violation_steps"] == 0
    excused = report["timing_beyond_time_bound_worst_position"]
    assert excused["position_equivalent_steps"] == pytest.approx(
        0.013, abs=1e-3
    )
    assert excused["reference_rate_hz"] == TURNAROUND_RATE_HZ
    assert report["position_equivalent_error_abs_max"] == excused


def test_a_low_rate_step_misplaced_by_a_large_step_fraction_fails():
    report = _timing_report_with(-1e-3, TURNAROUND_RATE_HZ)
    assert report["timing_violation_steps"] == 1
    worst = report["timing_beyond_time_bound_worst_position"]
    assert worst["timing_error_us"] == pytest.approx(-1000.0)
    assert worst["position_equivalent_steps"] == pytest.approx(0.227)


@pytest.mark.parametrize(
    "rate_hz",
    [analyze.POSITION_BOUND_RELAXES_BELOW_HZ * 1.01, 20e3, 140e3],
)
def test_a_step_beyond_the_time_bound_fails_at_every_rate_the_time_bound_owns(
    rate_hz,
):
    late_s = (analyze.TIMING_TOLERANCE_US + 1.0) * 1e-6
    report = _timing_report_with(late_s, rate_hz)
    assert report["timing_violation_steps"] == 1
    worst = report["timing_beyond_time_bound_worst_position"]
    assert worst["timing_error_us"] == pytest.approx(
        analyze.TIMING_TOLERANCE_US + 1.0
    )


def test_reports_injected_stall(synthesized):
    verdict, _, (stall_from, stall_to) = synthesized
    stalls = verdict["stalls"]["items"]
    z_idle = [
        s
        for s in stalls
        if s["role"] == "z" and s["kind"] == "capture_idle_reference_moving"
    ]
    assert len(z_idle) == 1
    assert stall_from - 0.01 <= z_idle[0]["capture_start_s"] <= stall_to
    assert z_idle[0]["duration_ms"] >= 0.5 * STALL_S * 1e3
    assert all(s["role"] == "z" for s in stalls)


def test_a_capture_on_the_plans_pace_passes_the_pace_check(synthesized):
    verdict, _, _ = synthesized
    assert verdict["checks"]["pace_matches_reference"]
    for epoch in verdict["epochs"]:
        assert epoch["pace_within_tolerance"]
        assert epoch["pace_max_abs_lag_ms"] <= STALL_S * 1e3 + 1.0


def _uniform_steps(times: np.ndarray) -> analyze.StepEvents:
    return analyze.StepEvents(
        times,
        np.arange(1, times.size + 1, dtype=np.int64),
        np.ones(times.size, np.int8),
        np.full(times.size, np.nan),
    )


def _paced(capture_times: np.ndarray, reference_times: np.ndarray) -> dict:
    return analyze.epoch_pace(
        analyze.EpochSteps(
            capture={"a": _uniform_steps(capture_times)},
            reference={"a": _uniform_steps(reference_times)},
        )
    )


PACE_REFERENCE_S = np.arange(0.0, 20.0, 1e-3)


def test_a_capture_one_percent_faster_than_the_plan_fails_the_pace_check():
    pace = _paced(16.6 + 0.99 * PACE_REFERENCE_S, PACE_REFERENCE_S)
    assert not pace["pace_within_tolerance"]
    assert pace["pace_capture_over_stream"] == pytest.approx(0.99, abs=1e-4)
    assert pace["pace_worst_lag_ms"] == pytest.approx(-200.0, abs=1.0)
    assert pace["pace_worst_at_stream_s"] == pytest.approx(20.0, abs=1e-2)


def test_a_local_burst_that_runs_ahead_and_resyncs_fails_the_pace_check():
    capture = PACE_REFERENCE_S.copy()
    ahead = (PACE_REFERENCE_S > 5.0) & (PACE_REFERENCE_S < 10.0)
    capture[ahead] -= 0.1 * (PACE_REFERENCE_S[ahead] - 5.0)
    capture[PACE_REFERENCE_S >= 10.0] -= 0.5
    capture[PACE_REFERENCE_S >= 12.0] += 0.5
    pace = _paced(capture, PACE_REFERENCE_S)
    assert not pace["pace_within_tolerance"]
    assert pace["pace_capture_over_stream"] == pytest.approx(1.0)


def test_crystal_ppm_and_a_short_stall_stay_within_the_pace_tolerance():
    capture = (1.0 + 150e-6) * PACE_REFERENCE_S
    capture[(PACE_REFERENCE_S > 3.0) & (PACE_REFERENCE_S < 3.06)] += STALL_S
    pace = _paced(capture, PACE_REFERENCE_S)
    assert pace["pace_within_tolerance"]
    assert pace["pace_max_abs_lag_ms"] == pytest.approx(STALL_S * 1e3, abs=0.5)


TIE_GCODE = """\
SET_KINEMATIC_POSITION X=0 Y=0 Z=0
G92 E0
M83
G1 X20 E0.5 F3000
G1 Z1 F600
G1 X0 F3000
G1 Z2 F600
G1 X10 F3000
"""
TIE_CAPTURE_START_S = 0.4


@pytest.fixture(scope="module")
def tie_run(tmp_path_factory):
    _require_snapshot_engine()
    template = tmp_path_factory.mktemp("tie")
    shutil.copy(BENCH_DIR / "printer.cfg", template / "printer.cfg")
    (template / "print.gcode").write_text(TIE_GCODE)
    job = analyze.load_job(template)
    reference = analyze.build_reference(job)
    first_step = min(
        ev.time.min() for ev in reference.events.values() if len(ev)
    )

    def verdict_with_z_step_at(index: int, stream_time_s: float) -> dict:
        run_dir = tmp_path_factory.mktemp("tie_run")
        for name in ("printer.cfg", "print.gcode"):
            shutil.copy(template / name, run_dir / name)
        times = {
            m.role: reference.events[m.role].time.copy() for m in job.motors
        }
        times["z"][index] = stream_time_s
        last = max(t.max() for t in times.values() if t.size) - first_step
        deltas = np.zeros(
            (int((last + TIE_CAPTURE_START_S + 0.5) * TICK_HZ), 4), np.int64
        )
        for motor in job.motors:
            capture_t = times[motor.role] - first_step + TIE_CAPTURE_START_S
            ticks = np.floor(capture_t * TICK_HZ).astype(np.int64)
            signed = reference.events[motor.role].direction * motor.dir_sign
            np.add.at(deltas[:, motor.channel], ticks, signed)
        deltas.astype(np.int8).tofile(run_dir / "capture.bin")
        (run_dir / "capture.json").write_text(
            json.dumps({"tick_hz": TICK_HZ, "channel_count": 4})
        )
        verdict, _ = analyze.analyze(analyze.load_run(run_dir))
        return verdict

    z_motor = next(m for m in job.motors if m.role == "z")
    z = reference.events["z"]
    landing = int(
        np.flatnonzero(
            (z.level == round(1.0 / z_motor.step_mm)) & (z.direction == 1)
        )[0]
    )
    return verdict_with_z_step_at, z, landing


def test_step_on_a_lattice_line_the_axis_rests_on_is_an_edge_tie(tie_run):
    verdict_with_z_step_at, z, landing = tie_run
    rest_s = z.time[landing + 1] - z.time[landing]
    assert rest_s > 2 * analyze.REPORT_MATCH_WINDOW_S
    verdict = verdict_with_z_step_at(landing, z.time[landing] + 0.5 * rest_s)
    motor = verdict["motors"]["z"]
    assert motor["unmatched_capture_steps"] == 0
    assert motor["unmatched_reference_steps"] == 0
    assert motor["edge_tie_steps"] == 2
    assert verdict["checks"]["all_steps_matched"]


def test_a_late_step_mid_move_is_still_unmatched(tie_run):
    verdict_with_z_step_at, z, landing = tie_run
    mid_move = landing // 2
    assert z.rate_hz[mid_move] > 100.0
    window_s = analyze.REPORT_MATCH_WINDOW_S
    late_s = z.time[landing] + 1.5 * window_s
    assert late_s + 1.5 * window_s < z.time[landing + 1]
    verdict = verdict_with_z_step_at(mid_move, late_s)
    motor = verdict["motors"]["z"]
    assert motor["unmatched_capture_steps"] > 0
    assert motor["unmatched_reference_steps"] > 0
    assert not verdict["checks"]["all_steps_matched"]


def _events(times, levels, directions) -> analyze.StepEvents:
    return analyze.StepEvents(
        np.asarray(times, np.float64),
        np.asarray(levels, np.int64),
        np.asarray(directions, np.int8),
        np.full(len(times), np.nan),
    )


def _apex_positions(apex_steps: float):
    def positions(start_s: float, end_s: float) -> np.ndarray:
        t = np.linspace(start_s, end_s, 200)
        return apex_steps - 4.0 * (t - 1.1) ** 2

    return positions


@pytest.mark.parametrize(
    ("apex_steps", "tie"), [(10.000002, True), (10.6, False)]
)
def test_reversal_apex_step_the_capture_lacks(apex_steps, tie):
    reference = _events([0.8, 1.0, 1.2, 1.3], [9, 10, 9, 8], [1, 1, -1, -1])
    capture = _events([0.8, 1.3], [9, 8], [1, -1])
    capture_tie, reference_tie, _ = analyze.edge_ties(
        capture,
        capture.time,
        np.array([False, False]),
        reference,
        np.array([False, True, False, False]),
        _apex_positions(apex_steps),
        None,
    )
    assert reference_tie.tolist() == [False, tie, False, False]
    assert not capture_tie.any()


@pytest.mark.parametrize(("apex_steps", "tie"), [(9.9999, True), (9.4, False)])
def test_reversal_apex_step_the_reference_lacks(apex_steps, tie):
    reference = _events([0.8, 1.3], [9, 8], [1, -1])
    capture = _events([0.8, 1.0, 1.2, 1.3], [9, 10, 9, 8], [1, 1, -1, -1])
    capture_tie, reference_tie, _ = analyze.edge_ties(
        capture,
        capture.time,
        np.array([False, True, False, False]),
        reference,
        np.array([False, False]),
        _apex_positions(apex_steps),
        None,
    )
    assert capture_tie.tolist() == [False, tie, False, False]
    assert not reference_tie.any()


class _RippleReference:
    def trajectory_span_s(self, t_stream: float):
        return None

    def positions(self, role: str, start_s: float, end_s: float):
        t = np.linspace(start_s, end_s, 50)
        return 9.9 + 0.1 * np.cos(2 * np.pi * t / 0.05)


def test_ripple_bursts_beyond_the_pair_window_are_not_timing_errors():
    reference = _events(
        [1.000, 1.001, 1.050, 1.051], [10, 10, 10, 10], [1, -1, 1, -1]
    )
    capture = _events(
        [1.025, 1.026, 1.075, 1.076], [10, 10, 10, 10], [1, -1, 1, -1]
    )
    match = analyze.measure_epoch(
        analyze.EpochSteps({"a": capture}, {"a": reference}),
        analyze.Alignment(0.0, 0.0, True),
        _RippleReference(),
    )["a"]
    assert match.late_s.size == 0
    assert match.unmatched_capture + match.edge_ties >= len(capture)


def _resting_positions(rest_steps: float):
    def positions(start_s: float, end_s: float) -> np.ndarray:
        return np.full(2, rest_steps)

    return positions


@pytest.mark.parametrize(
    ("final_steps", "tie"), [(10.0, True), (10.0000001, True), (10.001, False)]
)
def test_final_step_the_capture_lacks_is_a_tie_only_on_the_lattice_line(
    final_steps, tie
):
    reference = _events([0.8, 1.0], [9, 10], [1, 1])
    capture = _events([0.8], [9], [1])
    capture_tie, reference_tie, rest_ties = analyze.edge_ties(
        capture,
        capture.time,
        np.array([False]),
        reference,
        np.array([False, True]),
        _resting_positions(final_steps),
        (0.0, 1.2),
    )
    assert reference_tie.tolist() == [False, tie]
    assert rest_ties == (1 if tie else 0)
    assert not capture_tie.any()


class _RestingReference:
    def __init__(self, rest_steps: float):
        self.positions_at_rest = _resting_positions(rest_steps)

    def trajectory_span_s(self, t_stream: float):
        return 0.9, 1.2

    def positions(self, role: str, start_s: float, end_s: float):
        return self.positions_at_rest(start_s, end_s)


@pytest.mark.parametrize(
    ("start_steps", "tie"), [(10.0, True), (10.001, False)]
)
def test_step_withheld_at_rest_and_taken_on_resume_is_a_tie(start_steps, tie):
    reference = _events([1.0, 1.1], [9, 8], [-1, -1])
    capture = _events([0.95, 1.0, 1.1], [10, 9, 8], [1, -1, -1])
    match = analyze.measure_epoch(
        analyze.EpochSteps({"a": capture}, {"a": reference}),
        analyze.Alignment(0.0, 0.0, True),
        _RestingReference(start_steps),
    )["a"]
    assert match.late_s.tolist() == [0.0, 0.0]
    assert match.unmatched_reference == 0
    assert match.unmatched_capture == (0 if tie else 1)
    assert match.rest_ties == (1 if tie else 0)


@pytest.mark.parametrize(
    ("start_steps", "tie"), [(10.0, True), (10.001, False)]
)
def test_step_withheld_at_rest_and_cancelled_by_reversal_is_a_tie(
    start_steps, tie
):
    reference = _events([1.0, 1.1], [9, 8], [-1, -1])
    capture = _events([1.1], [8], [-1])
    capture_tie, reference_tie, rest_ties = analyze.edge_ties(
        capture,
        capture.time,
        np.array([False]),
        reference,
        np.array([True, False]),
        _resting_positions(start_steps),
        (0.9, 1.2),
    )
    assert reference_tie.tolist() == [tie, False]
    assert rest_ties == (1 if tie else 0)
    assert not capture_tie.any()


END_GCODE = """\
SET_KINEMATIC_POSITION X=0 Y=0 Z=0
G92 E0
M83
G1 X{x} F3000
"""


def _verdict_dropping_last_step(tmp_path_factory, x_mm: str) -> dict:
    _require_snapshot_engine()
    run_dir = tmp_path_factory.mktemp("end")
    shutil.copy(BENCH_DIR / "printer.cfg", run_dir / "printer.cfg")
    (run_dir / "print.gcode").write_text(END_GCODE.format(x=x_mm))
    job = analyze.load_job(run_dir)
    reference = analyze.build_reference(job)
    first_step = min(
        ev.time.min() for ev in reference.events.values() if len(ev)
    )
    last = max(ev.time.max() for ev in reference.events.values() if len(ev))
    deltas = np.zeros(
        (int((last - first_step + TIE_CAPTURE_START_S + 0.5) * TICK_HZ), 4),
        np.int64,
    )
    for motor in job.motors:
        events = reference.events[motor.role]
        keep = np.ones(len(events), bool)
        if motor.role == "a":
            keep[-1] = False
        capture_t = events.time[keep] - first_step + TIE_CAPTURE_START_S
        ticks = np.floor(capture_t * TICK_HZ).astype(np.int64)
        signed = events.direction[keep] * motor.dir_sign
        np.add.at(deltas[:, motor.channel], ticks, signed)
    deltas.astype(np.int8).tofile(run_dir / "capture.bin")
    (run_dir / "capture.json").write_text(
        json.dumps({"tick_hz": TICK_HZ, "channel_count": 4})
    )
    verdict, _ = analyze.analyze(analyze.load_run(run_dir))
    return verdict


def test_dropped_final_step_on_the_endpoint_line_is_an_end_tie(
    tmp_path_factory,
):
    verdict = _verdict_dropping_last_step(tmp_path_factory, "10")
    motor = verdict["motors"]["a"]
    assert motor["net_steps_capture_minus_expected"] == -1
    assert motor["edge_tie_steps"] == 1
    assert motor["unmatched_reference_steps"] == 0
    assert verdict["checks"]["net_steps_match"]
    assert verdict["checks"]["all_steps_matched"]


def test_dropped_final_step_past_the_line_still_fails(tmp_path_factory):
    verdict = _verdict_dropping_last_step(tmp_path_factory, "10.004")
    motor = verdict["motors"]["a"]
    assert motor["net_steps_capture_minus_expected"] == -1
    assert motor["unmatched_reference_steps"] == 1
    assert not verdict["checks"]["net_steps_match"]
    assert not verdict["checks"]["all_steps_matched"]


def test_blocks_split_by_drain_commands_stay_separate_epochs_across_short_rests(
    tmp_path,
):
    _require_snapshot_engine()
    gcode = (BENCH_DIR / "gcode" / "epochs.gcode").read_text()
    shutil.copy(BENCH_DIR / "printer.cfg", tmp_path / "printer.cfg")
    (tmp_path / "print.gcode").write_text(
        "SET_KINEMATIC_POSITION X=0 Y=0 Z=0\nG92 E0\n" + gcode
    )
    job = analyze.load_job(tmp_path)
    reference = analyze.build_reference(job)
    assert len(reference.blocks) == 6
    rests_s = [0.32, 1.52, 0.02, 0.03, 0.12, 0.02]
    assert min(rests_s[2:5]) < analyze.DEFAULT_EPOCH_GAP_S
    alignments = _block_alignments(reference, rests_s)
    _synthesize_capture(job, reference, alignments, stalled=False).tofile(
        tmp_path / "capture.bin"
    )
    (tmp_path / "capture.json").write_text(
        json.dumps({"tick_hz": TICK_HZ, "channel_count": 4})
    )
    verdict, _ = analyze.analyze(analyze.load_run(tmp_path))
    assert verdict["ok"], verdict["checks"]
    assert len(verdict["epochs"]) == 6
    for epoch, truth in zip(verdict["epochs"], alignments):
        recovered = analyze.Alignment(epoch["offset_s"], epoch["ppm"], True)
        start = epoch["stream_start_s"]
        assert recovered.to_capture(start) == pytest.approx(
            truth.to_capture(start), abs=20e-6
        )


DRAIN_GCODE = """\
SET_KINEMATIC_POSITION X=0 Y=0 Z=0
G92 E0
M83
G1 X20 Y5 E0.501 F3000
M400
G1 X40 Y20 E0.5 F3000
"""


def _e_steps_at(reference: analyze.Reference, t_stream: float) -> float:
    return float(reference.positions("e", t_stream, t_stream)[-1])


def test_extruder_lattice_carries_its_fraction_across_a_drain(tmp_path):
    _require_snapshot_engine()
    shutil.copy(BENCH_DIR / "printer.cfg", tmp_path / "printer.cfg")
    (tmp_path / "print.gcode").write_text(DRAIN_GCODE)
    reference = analyze.build_reference(analyze.load_job(tmp_path))
    first, second = reference.blocks
    parked = _e_steps_at(reference, first.stream_offset_s + first.duration_s)
    assert 0.2 < parked % 1.0 < 0.8
    assert _e_steps_at(reference, second.stream_offset_s) == pytest.approx(
        parked, abs=1e-5
    )
    e = reference.events["e"]
    resumed = int(np.flatnonzero(e.time >= second.stream_offset_s)[0])
    assert _e_steps_at(reference, e.time[resumed]) == pytest.approx(
        e.level[resumed], abs=1e-3
    )


def test_merged_epoch_is_cut_at_the_rest_nearest_the_reference_count():
    def burst(start_s: float, steps: int, length_s: float):
        return np.linspace(start_s, start_s + length_s, steps)

    def events(times: np.ndarray) -> analyze.StepEvents:
        return _events(times, np.arange(1, times.size + 1), np.ones(times.size))

    reference = {
        "a": events(
            np.concatenate(
                [
                    burst(0.0, 2000, 1.0),
                    burst(100.0, 2000, 1.0),
                    burst(200.0, 2000, 1.0),
                ]
            )
        )
    }
    capture = {
        "a": events(
            np.concatenate(
                [
                    burst(0.0, 2001, 1.0),
                    burst(5.0, 1999, 0.1),
                    burst(5.12, 2000, 0.1),
                ]
            )
        )
    }
    capture_epochs = analyze.find_epochs(capture, 0.15)
    reference_epochs = analyze.find_epochs(reference, 0.15)
    assert len(capture_epochs) == 2
    split = analyze.split_epochs_by_reference_counts(
        capture, capture_epochs, reference, reference_epochs
    )
    assert [e.start_s for e in split] == pytest.approx([0.0, 5.0, 5.12])


LIVE_SEMANTICS_GCODE = """\
SET_KINEMATIC_POSITION X=0 Y=0 Z=0
G92 E0
G1 X10 Y5 E1
G92 E0
G1 X20 E0.5 F3000
G1 Z1
G4 P200
M83
G1 X0 Y0 E1 F9000
G92 E0
G1 X5 E0.2
G91
M82
G1 X5 E0.3
M400
G90
G1 X30 Y30 Z0.4 E2 F1200
SET_VELOCITY_LIMIT ACCEL=2000
G1 X40 Y30 F120000
M400
"""


def _live_epochs(gcode: str, cfg: Path) -> list[list[tuple]]:
    import live_replay

    steps = live_replay.replay(gcode, cfg)
    anchor = next(
        i
        for i, s in enumerate(steps)
        if isinstance(s, live_replay.LivePosition)
    )
    epochs: list[list[tuple]] = [[]]
    for step in steps[anchor + 1 :]:
        if isinstance(step, live_replay.LiveRest) and epochs[-1]:
            epochs.append([])
        elif isinstance(step, live_replay.LiveMove):
            epochs[-1].append((step.delta, step.feedrate, step.limits))
    return [epoch for epoch in epochs if epoch]


def _reference_epochs(job: analyze.Job) -> list[list[tuple]]:
    engine = analyze.harness._import_engine()
    _, _, blocks = analyze.plan_waypoints(job)
    epochs = []
    for block in blocks:
        moves = engine.snapshot_moves(block, job.config.config_text)
        assert all(m["dwell_before_s"] is None for m in moves)
        epochs.append(
            [
                (
                    tuple(e - s for s, e in zip(m["start"], m["end"])),
                    m["feedrate"],
                    (m["velocity"], m["accel"], m["corner_deviation"]),
                )
                for m in moves
            ]
        )
    return epochs


def test_reference_epochs_plan_exactly_the_moves_live_submits(tmp_path):
    _require_snapshot_engine()
    shutil.copy(BENCH_DIR / "printer.cfg", tmp_path / "printer.cfg")
    (tmp_path / "print.gcode").write_text(LIVE_SEMANTICS_GCODE)
    job = analyze.load_job(tmp_path)
    reference = _reference_epochs(job)
    assert reference == _live_epochs(
        LIVE_SEMANTICS_GCODE, tmp_path / "printer.cfg"
    )
    assert [len(epoch) for epoch in reference] == [3, 3, 2]
    _, final_xyze, _ = analyze.plan_waypoints(job)
    assert final_xyze == pytest.approx((40.0, 30.0, 0.4, 4.5))


def test_a_command_live_cannot_plan_offline_fails_the_analysis(tmp_path):
    _require_snapshot_engine()
    shutil.copy(BENCH_DIR / "printer.cfg", tmp_path / "printer.cfg")
    (tmp_path / "print.gcode").write_text(
        "SET_KINEMATIC_POSITION X=0 Y=0 Z=0\nG1 X10 F3000\nG2 X20 I5 J0\n"
    )
    with pytest.raises(analyze.AnalysisError, match="line 3: G2"):
        analyze.plan_waypoints(analyze.load_job(tmp_path))


def test_same_key_matching_stays_exact_over_millions_of_levels_and_long_runs():
    ref_key = np.array([0, 5_000_000, 5_000_000], np.int64)
    ref_t = np.array([0.0, 90.0, 90.000_000_05], np.float64)
    query_key = np.array([5_000_000, 0], np.int64)
    query_t = np.array([90.000_000_04, 0.000_000_02], np.float64)
    nearest, delta = analyze.nearest_same_key(
        query_key, query_t, ref_key, ref_t
    )
    assert nearest.tolist() == [2, 0]
    assert delta == pytest.approx([-1e-8, 2e-8], abs=1e-9)
    counts = analyze.same_key_count(query_key, query_t, ref_key, ref_t)
    assert counts.tolist() == [2, 1]


def test_a_short_epoch_runs_at_the_latest_fitted_rate_not_the_median():
    fitted = [-47.0, -46.6, -45.5, -4.3, None, None]
    assert analyze.nearest_fitted_ppm(fitted, 4) == -4.3
    assert analyze.nearest_fitted_ppm(fitted, 5) == -4.3


def test_a_short_first_epoch_runs_at_the_next_fitted_rate():
    assert analyze.nearest_fitted_ppm([None, 12.0, -3.0], 0) == 12.0


def test_a_run_with_no_fitted_epoch_runs_at_the_nominal_rate():
    assert analyze.nearest_fitted_ppm([None, None], 1) == 0.0
