# test/test_blendshaper.py
import math

import pytest

from klippy import blendshaper


def test_axis_shaper_snapshot_fields():
    snap = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=150.0,
        damping_ratio=0.1,
        A_axis=87685.6,
    )
    assert snap.axis == "x"
    assert snap.shaper_type == "zv"
    assert snap.shaper_freq == 150.0
    assert snap.damping_ratio == 0.1
    assert snap.A_axis == 87685.6


def test_shaper_bounds_fields():
    bounds = blendshaper.ShaperBounds(
        j_eff=3.97e6,
        v_step_cap=132.8,
    )
    assert bounds.j_eff == 3.97e6
    assert bounds.v_step_cap == 132.8


def test_shaper_span_zv():
    # t_d = 1/(f·sqrt(1-zeta^2)); T_span = 0.5 * t_d
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("zv", f, zeta) == pytest.approx(
        0.5 * t_d, rel=1e-12
    )


def test_shaper_span_mzv():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("mzv", f, zeta) == pytest.approx(
        0.75 * t_d, rel=1e-12
    )


def test_shaper_span_zvd():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("zvd", f, zeta) == pytest.approx(
        1.0 * t_d, rel=1e-12
    )


def test_shaper_span_ei():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("ei", f, zeta) == pytest.approx(
        1.0 * t_d, rel=1e-12
    )


def test_shaper_span_2hump_ei():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("2hump_ei", f, zeta) == pytest.approx(
        1.5 * t_d, rel=1e-12
    )


def test_shaper_span_3hump_ei():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("3hump_ei", f, zeta) == pytest.approx(
        2.0 * t_d, rel=1e-12
    )


def test_shaper_span_damping_effect():
    # Higher damping ratio stretches t_d.
    f = 100.0
    span_low = blendshaper.shaper_span("zv", f, 0.05)
    span_high = blendshaper.shaper_span("zv", f, 0.2)
    assert span_high > span_low


def test_shaper_span_unknown_raises():
    with pytest.raises(ValueError):
        blendshaper.shaper_span("not_a_shaper", 100.0, 0.1)


def test_axis_projections_unit_x():
    projs = blendshaper.axis_projections((1.0, 0.0, 0.0))
    assert projs["x"] == pytest.approx(1.0, abs=1e-12)
    assert projs["y"] == pytest.approx(0.0, abs=1e-12)
    assert projs["z"] == pytest.approx(0.0, abs=1e-12)


def test_axis_projections_45_deg_xy():
    s = 1.0 / math.sqrt(2.0)
    projs = blendshaper.axis_projections((s, s, 0.0))
    assert projs["x"] == pytest.approx(s, abs=1e-12)
    assert projs["y"] == pytest.approx(s, abs=1e-12)
    assert projs["z"] == pytest.approx(0.0, abs=1e-12)


def test_axis_projections_negative_components_return_abs():
    projs = blendshaper.axis_projections((-0.6, 0.8, 0.0))
    assert projs["x"] == pytest.approx(0.6, abs=1e-12)
    assert projs["y"] == pytest.approx(0.8, abs=1e-12)
    assert projs["z"] == pytest.approx(0.0, abs=1e-12)


def test_axis_in_plane_xy_plane():
    # Arc plane normal along +Z: x and y lie fully in the plane.
    in_plane = blendshaper.axis_in_plane((0.0, 0.0, 1.0))
    assert in_plane["x"] == pytest.approx(1.0, abs=1e-12)
    assert in_plane["y"] == pytest.approx(1.0, abs=1e-12)
    assert in_plane["z"] == pytest.approx(0.0, abs=1e-12)


def test_axis_in_plane_yz_plane():
    # Arc plane normal along +X: y and z lie fully in the plane.
    in_plane = blendshaper.axis_in_plane((1.0, 0.0, 0.0))
    assert in_plane["x"] == pytest.approx(0.0, abs=1e-12)
    assert in_plane["y"] == pytest.approx(1.0, abs=1e-12)
    assert in_plane["z"] == pytest.approx(1.0, abs=1e-12)


def test_axis_in_plane_tilted():
    # Plane normal at 45° between X and Z: x and z partially in-plane.
    s = 1.0 / math.sqrt(2.0)
    in_plane = blendshaper.axis_in_plane((s, 0.0, s))
    # sqrt(1 - (1/sqrt(2))^2) = sqrt(1 - 0.5) = sqrt(0.5) = 1/sqrt(2)
    assert in_plane["x"] == pytest.approx(s, abs=1e-12)
    assert in_plane["y"] == pytest.approx(1.0, abs=1e-12)  # perpendicular to normal
    assert in_plane["z"] == pytest.approx(s, abs=1e-12)


def test_compute_shaper_bounds_step_single_axis_x_projection():
    # Contrived n̂ with |n̂·x̂|=1/√2 and |n̂·ŷ|=1/√2 so the single shaped axis
    # (X) contributes to Bound (b). Unit test of the formula; n̂ here is a
    # direct input, not derived from a corner.
    # v_step_cap = √(A_x · R / (1/√2)) = √(A_x · R · √2)
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=100.0,
        damping_ratio=0.1,
        A_axis=10000.0,
    )
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x],
        R=0.5,
        n_hat=(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    expected_v_step = math.sqrt(10000.0 * 0.5 * math.sqrt(2.0))
    assert bounds.v_step_cap == pytest.approx(expected_v_step, rel=1e-9)


def test_compute_shaper_bounds_zero_A_axis_skipped():
    # A shaper with freq > 0 but A_axis = 0 is a malformed snapshot;
    # the function must skip it instead of returning v_step_cap = 0.
    snap_bad = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=100.0,
        damping_ratio=0.1,
        A_axis=0.0,
    )
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_bad],
        R=0.5,
        n_hat=(1.0, 0.0, 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.v_step_cap == float("inf")


def test_compute_shaper_bounds_jerk_single_axis_in_plane():
    # Single shaped axis X, arc in XY plane → axis_in_plane_x = 1.
    # j_eff = A_x / T_x.
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=100.0,
        damping_ratio=0.1,
        A_axis=10000.0,
    )
    T_x = blendshaper.shaper_span("zv", 100.0, 0.1)
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x],
        R=0.5,
        n_hat=(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.j_eff == pytest.approx(10000.0 / T_x, rel=1e-9)


def test_compute_shaper_bounds_jerk_axis_partially_in_plane():
    # Single shaped axis X, arc plane normal at 45° between X and Z:
    # axis_in_plane_x = sqrt(1 - 0.5) = 1/sqrt(2).
    # j_x_effective = A_x / (T_x · (1/sqrt(2))) = A_x · sqrt(2) / T_x.
    # n_hat must be perpendicular to p_hat in real arc geometry; we use +Y
    # which is perpendicular to any plane with a normal in the XZ plane.
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=100.0,
        damping_ratio=0.1,
        A_axis=10000.0,
    )
    T_x = blendshaper.shaper_span("zv", 100.0, 0.1)
    s = 1.0 / math.sqrt(2.0)
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x],
        R=0.5,
        n_hat=(0.0, 1.0, 0.0),   # perpendicular to p_hat
        p_hat=(s, 0.0, s),       # plane normal at 45° in XZ
    )
    expected_j = 10000.0 / (T_x * s)
    assert bounds.j_eff == pytest.approx(expected_j, rel=1e-9)
