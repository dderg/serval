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
JITTER_US = 5.0
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
        else:
            assert stats["abs_max"] <= JITTER_US + quantization_us + 2.0
            assert abs(stats["p50"]) <= 2.0


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
    delay_s = 0.5 * (z.time[landing + 1] - z.time[landing])
    verdict = verdict_with_z_step_at(mid_move, z.time[mid_move] + delay_s)
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
    def trajectory_end_s(self, t_stream: float):
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


def _end_positions(final_steps: float):
    def positions(start_s: float, end_s: float) -> np.ndarray:
        return np.full(2, final_steps)

    return positions


@pytest.mark.parametrize(
    ("final_steps", "tie"), [(10.0, True), (10.0000001, True), (10.001, False)]
)
def test_final_step_the_capture_lacks_is_a_tie_only_on_the_lattice_line(
    final_steps, tie
):
    reference = _events([0.8, 1.0], [9, 10], [1, 1])
    capture = _events([0.8], [9], [1])
    capture_tie, reference_tie, end_ties = analyze.edge_ties(
        capture,
        capture.time,
        np.array([False]),
        reference,
        np.array([False, True]),
        _end_positions(final_steps),
        1.2,
    )
    assert reference_tie.tolist() == [False, tie]
    assert end_ties == (1 if tie else 0)
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
