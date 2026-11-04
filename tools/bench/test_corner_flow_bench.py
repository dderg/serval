from __future__ import annotations

import math
import sys
from pathlib import Path

import analyze
import corner_flow_bench
import numpy as np
import pytest

from tools import corner_flow

BENCH_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BENCH_DIR / "gcode"))
import gen_corner_flow  # noqa: E402

INJECTED_PPM = 41.0
CAPTURE_GAP_S = 0.7
SPEED_MM_S = 150.0
BEAD = (0.4, 0.2, 1.75)


def _waypoints(path: list[tuple[float, float]], speed: float) -> list[dict]:
    position = [*path[0], 0.0, 0.0]
    waypoints = [{"position": list(position), "feedrate": speed}]
    for previous, point in zip(path, path[1:]):
        position[3] += gen_corner_flow.RATIO * math.dist(previous, point)
        waypoints.append(
            {"position": [*point, 0.0, position[3]], "feedrate": speed}
        )
    return waypoints


@pytest.mark.parametrize("interior", gen_corner_flow.CORNER_ANGLES_DEG)
def test_a_corner_is_one_feature_turning_its_supplement(interior):
    path = gen_corner_flow.corner(interior)
    (feature,) = corner_flow_bench.features(_waypoints(path, 100.0))
    assert feature.kind == "corner"
    assert feature.turn_deg == pytest.approx(180.0 - interior)
    assert feature.apex_xy == pytest.approx(path[1])
    assert feature.window_mm == pytest.approx(15.0)
    assert feature.ratio == pytest.approx(gen_corner_flow.RATIO)


def test_a_short_segment_arc_is_one_curve_feature():
    (feature,) = corner_flow_bench.features(
        _waypoints(gen_corner_flow.arc(), 100.0)
    )
    assert feature.kind == "curve"
    assert feature.turn_deg == pytest.approx(180.0, abs=4.0)
    assert feature.window_mm == pytest.approx(
        math.pi * gen_corner_flow.ARC_RADIUS_MM / 2.0, rel=0.01
    )


def test_zigzag_corners_window_half_a_leg():
    feats = corner_flow_bench.features(
        _waypoints(gen_corner_flow.zigzag(), 100.0)
    )
    assert len(feats) == gen_corner_flow.ZIGZAG_TURNS + 1
    assert all(f.kind == "corner" for f in feats)
    assert [f.window_mm for f in feats[1:-1]] == pytest.approx(
        [gen_corner_flow.ZIGZAG_LEG_MM / 2.0] * (len(feats) - 2)
    )


def test_a_dry_move_between_extrusions_is_no_feature():
    waypoints = _waypoints([(0.0, 0.0), (20.0, 0.0)], 100.0)
    waypoints.append(
        {
            "position": [20.0, 20.0, 0.0, waypoints[-1]["position"][3]],
            "feedrate": 100.0,
        }
    )
    assert corner_flow_bench.features(waypoints) == []


def _exact_capture(job, reference) -> analyze.Capture:
    """Every reference step as an edge the capture timestamps exactly, each
    block shifted by its own offset and the whole clock scaled by
    INJECTED_PPM, as a real capture's crystal would."""
    starts = np.array([b.stream_offset_s for b in reference.blocks])
    scale = 1.0 + INJECTED_PPM * 1e-6
    exact = {}
    for motor in job.motors:
        events = reference.events[motor.role]
        block = np.searchsorted(starts, events.time, "right") - 1
        local = events.time - starts[block]
        offsets = np.cumsum(
            [CAPTURE_GAP_S]
            + [b.duration_s * scale + CAPTURE_GAP_S for b in reference.blocks]
        )
        exact[motor.channel] = (
            offsets[block] + scale * local,
            (events.direction * motor.dir_sign).astype(np.int8),
        )
    last = max(times.max() for times, _ in exact.values() if times.size)
    return analyze.Capture(
        tick_hz=84e6,
        deltas=np.zeros((int(last * 1000) + 1000, 4), np.int8),
        exact_steps=[exact[channel] for channel in range(4)],
    )


def _smoothed_extruder_config() -> str:
    """The bench printer with the `smooth_triangle` 0.04 override after its
    advance. The bounds below are calibrated on that smooth extruder signal:
    the kernel-free chain `printer.cfg` ships lays its extruder steps sharply
    enough that step quantization alone moves the apex by several points
    (the README's "Corner flow" captures), and puts the reference's peak near
    the 5 % blob threshold, where blob amplifies any timing residual."""
    config = (BENCH_DIR / "printer.cfg").read_text()
    chain = "post_processors: pa\n"
    shaper = "[post_processor shaper_x]\n"
    assert config.count(chain) == 1 and config.count(shaper) == 1
    kernel = "[post_processor st]\ntype: smooth_triangle\nsmooth_time: 0.04\n\n"
    return config.replace(chain, "post_processors: pa, st\n").replace(
        shaper, kernel + shaper
    )


@pytest.fixture(scope="module")
def corner_run(tmp_path_factory):
    try:
        analyze.harness._import_engine()
    except ImportError as exc:
        pytest.skip(str(exc))
    run_dir = tmp_path_factory.mktemp("corner")
    (run_dir / "printer.cfg").write_text(_smoothed_extruder_config())
    geometry = corner_flow.corner(90)
    ratio = BEAD[0] * BEAD[1] / (math.pi * (BEAD[2] / 2.0) ** 2)
    (run_dir / "print.gcode").write_text(
        "SET_KINEMATIC_POSITION X=0 Y=0 Z=0\nG92 E0\n"
        + corner_flow.gcode(geometry, SPEED_MM_S, ratio)
    )
    job = analyze.load_job(run_dir)
    inputs = analyze.RunInputs(
        **vars(job), capture=_exact_capture(job, analyze.build_reference(job))
    )
    _, detail = analyze.analyze(inputs)
    report = corner_flow_bench.report(
        inputs,
        detail,
        analyze.plan_waypoints(inputs)[2],
        analyze.ROLE_AXIS_WEIGHTS,
        analyze.harness._import_engine(),
    )
    return job, geometry, report


def test_an_exact_capture_measures_what_its_reference_steps_command(corner_run):
    _, _, report = corner_run
    (row,) = report["features"]
    for metric in (*corner_flow_bench.METRICS, "ve_max", "ve_min", "ae_max"):
        assert row["capture"][metric] == pytest.approx(
            row["reference"][metric], rel=1e-4, abs=1e-4
        ), metric


def test_the_offline_source_is_corner_flow_measure(corner_run):
    job, geometry, report = corner_run
    (row,) = report["features"]
    config = corner_flow.PrinterConfig.parse(job.config.config_text)
    expected = corner_flow.measure(
        geometry, SPEED_MM_S, config, config.advance_k(), BEAD
    )
    assert not expected.error
    assert row["offline"]["peak"] * 100.0 == pytest.approx(
        expected.peak, abs=0.05
    )
    assert row["offline"]["dip"] * 100.0 == pytest.approx(
        expected.dip, abs=0.05
    )
    assert row["offline"]["blob_nl"] == pytest.approx(
        expected.blob_nl, abs=0.05
    )


def test_step_quantization_of_a_90_degree_corner_stays_within_its_bound(
    corner_run,
):
    _, _, report = corner_run
    (row,) = report["features"]
    for metric in ("apex", "peak", "dip"):
        assert abs(row["reference"][metric] - row["offline"][metric]) < 0.01, (
            metric
        )
