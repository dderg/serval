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
    reference: analyze.Reference,
) -> list[analyze.Alignment]:
    alignments = []
    capture_start = FIRST_EPOCH_CAPTURE_START_S
    for block in reference.blocks:
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
            + INTER_EPOCH_IDLE_S
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
) -> np.ndarray:
    rng = np.random.default_rng(7)
    times = {}
    for motor in job.motors:
        events = reference.events[motor.role]
        t = _true_capture_times(reference, alignments, events)
        t = t + rng.uniform(-JITTER_US, JITTER_US, t.size) * 1e-6
        if motor.role == "z":
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
