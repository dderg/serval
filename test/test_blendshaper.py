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


def test_compute_shaper_bounds_y_binds_over_x():
    # X at 150Hz, Y at 80Hz; Y has smaller A/T → Y binds on Bound (c).
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x", shaper_type="zv", shaper_freq=150.0,
        damping_ratio=0.1, A_axis=87000.0,
    )
    snap_y = blendshaper.AxisShaperSnapshot(
        axis="y", shaper_type="zv", shaper_freq=80.0,
        damping_ratio=0.1, A_axis=25000.0,
    )
    T_x = blendshaper.shaper_span("zv", 150.0, 0.1)
    T_y = blendshaper.shaper_span("zv", 80.0, 0.1)
    assert 25000.0 / T_y < 87000.0 / T_x  # Y is stricter for jerk

    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x, snap_y],
        R=0.5,
        n_hat=(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.j_eff == pytest.approx(25000.0 / T_y, rel=1e-9)
    # Both axes project equally (n̂ at 45°), so v_step_cap binds on the
    # tighter (Y) axis: sqrt(A_y · R / proj) < sqrt(A_x · R / proj).
    proj = 1.0 / math.sqrt(2.0)
    expected_v_step = math.sqrt(25000.0 * 0.5 / proj)
    assert bounds.v_step_cap == pytest.approx(expected_v_step, rel=1e-9)


def test_compute_shaper_bounds_no_shapers_returns_infinity():
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[],
        R=0.5,
        n_hat=(1.0, 0.0, 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.j_eff == float("inf")
    assert bounds.v_step_cap == float("inf")


def test_compute_shaper_bounds_unshaped_axis_contributes_nothing():
    # freq=0 means no shaper — axis is skipped.
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x", shaper_type=None, shaper_freq=0.0,
        damping_ratio=0.1, A_axis=0.0,
    )
    snap_y = blendshaper.AxisShaperSnapshot(
        axis="y", shaper_type="zv", shaper_freq=80.0,
        damping_ratio=0.1, A_axis=25000.0,
    )
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x, snap_y],
        R=0.5,
        n_hat=(0.0, 1.0, 0.0),   # n̂ along +y
        p_hat=(0.0, 0.0, 1.0),
    )
    # Only Y contributes.
    T_y = blendshaper.shaper_span("zv", 80.0, 0.1)
    assert bounds.j_eff == pytest.approx(25000.0 / T_y, rel=1e-9)
    assert bounds.v_step_cap == pytest.approx(
        math.sqrt(25000.0 * 0.5 / 1.0), rel=1e-9
    )


def test_compute_shaper_bounds_out_of_plane_shaper_contributes_nothing():
    # XY arc, only Z shaped: axis_in_plane_z = 0, |n̂·ẑ| = 0.
    # Z contributes to neither bound → both return infinity.
    snap_z = blendshaper.AxisShaperSnapshot(
        axis="z", shaper_type="zv", shaper_freq=50.0,
        damping_ratio=0.1, A_axis=5000.0,
    )
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_z],
        R=0.5,
        n_hat=(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.j_eff == float("inf")
    assert bounds.v_step_cap == float("inf")


def test_compute_shaper_bounds_small_projection_axis_skipped_for_step():
    # X shaped, but n̂ is (0, 1, 0) — no X projection for step bound.
    # Bound (b) contributes nothing from X; Bound (c) still does
    # (axis_in_plane_x = 1).
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x", shaper_type="zv", shaper_freq=100.0,
        damping_ratio=0.1, A_axis=10000.0,
    )
    T_x = blendshaper.shaper_span("zv", 100.0, 0.1)
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x],
        R=0.5,
        n_hat=(0.0, 1.0, 0.0),   # no X component
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.v_step_cap == float("inf")  # no X-projected step
    assert bounds.j_eff == pytest.approx(10000.0 / T_x, rel=1e-9)  # X still in plane


def _zv_A(f, zeta=0.1):
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    from klippy.extras import shaper_defs
    sc = ShaperCalibrate(printer=None)
    shaper = shaper_defs.get_zv_shaper(f, zeta)
    return sc.find_shaper_max_accel(shaper)


def test_numeric_sanity_user_regime_90deg_corner():
    """Matches docs/superpowers/specs/2026-04-17-j-eff-derivation-design.md §Testing point 5.

    Setup: X=ZV@150Hz, Y=ZV@80Hz, ζ=0.1. 90° +X→+Y corner at R=0.5mm.
    Real n̂ at entry for this corner is (0, 1, 0) — pure Y direction
    (centripetal accel appears entirely on Y as the toolhead starts
    turning from +X into +Y). So X's entry-step is not triggered;
    only Y contributes to Bound (b). Bound (c) binds.
    """
    f_x, f_y = 150.0, 80.0
    zeta = 0.1
    A_x = _zv_A(f_x, zeta)
    A_y = _zv_A(f_y, zeta)
    T_x = blendshaper.shaper_span("zv", f_x, zeta)
    T_y = blendshaper.shaper_span("zv", f_y, zeta)

    snaps = [
        blendshaper.AxisShaperSnapshot("x", "zv", f_x, zeta, A_x),
        blendshaper.AxisShaperSnapshot("y", "zv", f_y, zeta, A_y),
    ]
    R = 0.5
    n_hat = (0.0, 1.0, 0.0)
    p_hat = (0.0, 0.0, 1.0)
    bounds = blendshaper.compute_shaper_bounds(snaps, R, n_hat, p_hat)

    # j_eff expected to bind on Y: j_y = A_y / T_y.
    assert bounds.j_eff == pytest.approx(A_y / T_y, rel=1e-9)

    # v_step_cap expected on Y only (X has |n̂·x̂|=0): √(A_y · R).
    expected_v_step = math.sqrt(A_y * R)
    assert bounds.v_step_cap == pytest.approx(expected_v_step, rel=1e-9)

    # End-to-end v_jerk from j_eff and this R.
    v_jerk = (R * R * bounds.j_eff) ** (1.0 / 3.0)
    # Centripetal cap.
    a_max = 50000.0
    v_centripetal = math.sqrt((math.sqrt(3) / 2) * a_max * R)
    # Rotation jerk should bind: v_jerk < others.
    assert v_jerk < v_centripetal
    assert v_jerk < expected_v_step
    # Cross-check: ~99.8 mm/s per the spec sanity section.
    assert v_jerk == pytest.approx(99.8, rel=0.05)


@pytest.mark.parametrize("f", [50.0, 80.0, 100.0, 150.0, 200.0])
def test_j_eff_monotone_in_frequency(f):
    """Higher shaper frequency → higher j_eff. All else equal."""
    zeta = 0.1
    A = 10000.0  # hold A constant so we isolate the T dependence
    T_f = blendshaper.shaper_span("zv", f, zeta)
    j_f = A / T_f
    # A higher frequency gives a shorter T and thus larger j.
    T_higher = blendshaper.shaper_span("zv", f * 1.5, zeta)
    j_higher = A / T_higher
    assert j_higher > j_f


def test_j_eff_monotone_in_damping():
    """Higher damping ratio → larger t_d → smaller j_eff."""
    f = 100.0
    T_low = blendshaper.shaper_span("zv", f, 0.05)
    T_high = blendshaper.shaper_span("zv", f, 0.2)
    # With A constant:
    A = 10000.0
    assert A / T_high < A / T_low


def test_j_eff_monotone_in_shaper_type():
    """ZV has shortest T → largest j_eff for given f; 3HUMP_EI longest T → smallest."""
    f = 100.0
    zeta = 0.1
    A = 10000.0
    t_zv = blendshaper.shaper_span("zv", f, zeta)
    t_zvd = blendshaper.shaper_span("zvd", f, zeta)
    t_3hump = blendshaper.shaper_span("3hump_ei", f, zeta)
    assert A / t_zv > A / t_zvd > A / t_3hump


@pytest.mark.parametrize("f_low,f_high", [(50.0, 100.0), (80.0, 150.0)])
def test_compute_shaper_bounds_j_eff_monotone_in_frequency(f_low, f_high):
    """Verifies monotonicity through the real compute_shaper_bounds path, not just the formula."""
    A = 10000.0
    zeta = 0.1
    make = lambda f: blendshaper.AxisShaperSnapshot(
        axis="x", shaper_type="zv", shaper_freq=f,
        damping_ratio=zeta, A_axis=A,
    )
    b_low = blendshaper.compute_shaper_bounds(
        shapers=[make(f_low)],
        R=0.5,
        n_hat=(0.0, 1.0, 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    b_high = blendshaper.compute_shaper_bounds(
        shapers=[make(f_high)],
        R=0.5,
        n_hat=(0.0, 1.0, 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert b_high.j_eff > b_low.j_eff
