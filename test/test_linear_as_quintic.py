import pytest
from klippy.chelper import get_ffi
from klippy.chelper.linear_quintic import linear_as_quintic_coeffs

# Chunk 3 stride: per-phase MOVE_QUINTIC_POLY_COEFFS (15) * 4 axes = 60 doubles.
# axes are {0=x, 1=y, 2=z, 3=e}; .e is left zero by linear_as_quintic_coeffs
# and populated downstream by linear_pa_compose.
PHASE_STRIDE = 60
AXIS_STRIDE = 4
COEFFS_PER_PHASE = 15


def test_degenerate_quintic_matches_linear_at_sample_times():
    # Linear motion: accel from v0=10 mm/s over accel_t=0.05s at a=200 mm/s^2,
    # cruise for 0.1s, decel over 0.05s. axes_r = (1,0,0) — pure X.
    ffi, lib = get_ffi()
    accel_t, cruise_t, decel_t = 0.05, 0.1, 0.05
    start_v, accel = 10.0, 200.0
    cruise_v = start_v + accel * accel_t
    start_pos = (0.0, 0.0, 0.0)
    axes_r = (1.0, 0.0, 0.0)

    coeffs = linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r, start_pos,
    )
    assert len(coeffs) == 180

    # Degenerate quintic: phase 0 (accel), c[0]=start_pos_x=0, c[1]=start_v,
    # c[2]=half_accel=100, c[3..14]=0.
    # Buffer index: phase * 60 + coeff * 4 + axis.
    assert coeffs[0 * PHASE_STRIDE + 0 * AXIS_STRIDE + 0] == pytest.approx(0.0)    # x0
    assert coeffs[0 * PHASE_STRIDE + 1 * AXIS_STRIDE + 0] == pytest.approx(10.0)   # v0
    assert coeffs[0 * PHASE_STRIDE + 2 * AXIS_STRIDE + 0] == pytest.approx(100.0)  # half_a
    for i in range(3, COEFFS_PER_PHASE):
        assert coeffs[0 * PHASE_STRIDE + i * AXIS_STRIDE + 0] == 0.0


def test_degenerate_quintic_pure_cruise():
    # accel_t=0, cruise_t=0.1, decel_t=0: constant velocity segment only.
    coeffs = linear_as_quintic_coeffs(
        0.0, 0.1, 0.0,
        50.0, 50.0, 0.0,
        (1.0, 0.0, 0.0), (5.0, 0.0, 0.0),
    )
    # Cruise phase (phase 1): c[0]=5 (x0 at start of cruise), c[1]=50, c[2..14]=0.
    assert coeffs[1 * PHASE_STRIDE + 0 * AXIS_STRIDE + 0] == pytest.approx(5.0)
    assert coeffs[1 * PHASE_STRIDE + 1 * AXIS_STRIDE + 0] == pytest.approx(50.0)
    assert coeffs[1 * PHASE_STRIDE + 2 * AXIS_STRIDE + 0] == 0.0


def test_decel_phase_position_and_accel_sign():
    # Same trapezoid as the first test, but verify decel phase explicitly.
    accel_t, cruise_t, decel_t = 0.05, 0.1, 0.05
    start_v, accel = 10.0, 200.0
    cruise_v = start_v + accel * accel_t  # 20
    start_pos = (0.0, 0.0, 0.0)
    axes_r = (1.0, 0.0, 0.0)

    coeffs = linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r, start_pos,
    )

    # Cruise start: x = start_v*accel_t + 0.5*accel*accel_t^2
    #             = 10*0.05 + 100*0.0025 = 0.5 + 0.25 = 0.75
    # Decel start: cruise_start_x + cruise_v*cruise_t
    #            = 0.75 + 20*0.1 = 2.75
    decel_phase = 2
    assert coeffs[decel_phase * PHASE_STRIDE + 0 * AXIS_STRIDE + 0] == pytest.approx(2.75)  # x0
    assert coeffs[decel_phase * PHASE_STRIDE + 1 * AXIS_STRIDE + 0] == pytest.approx(20.0)  # v0=cruise_v
    # Decel accel is negated: c[2] = axes_r_x * (-accel/2) = -100.
    assert coeffs[decel_phase * PHASE_STRIDE + 2 * AXIS_STRIDE + 0] == pytest.approx(-100.0)
    for i in range(3, COEFFS_PER_PHASE):
        assert coeffs[decel_phase * PHASE_STRIDE + i * AXIS_STRIDE + 0] == 0.0


def test_pure_y_axis_no_xz_transposition():
    # axes_r=(0,1,0) must write per-axis velocity to Y slot, not X.
    coeffs = linear_as_quintic_coeffs(
        0.05, 0.0, 0.0,
        10.0, 20.0, 200.0,
        (0.0, 1.0, 0.0), (7.0, 8.0, 9.0),
    )
    # Accel phase:
    # c[0] on X/Y/Z reads start_pos_x, start_pos_y, start_pos_z.
    assert coeffs[0 * PHASE_STRIDE + 0 * AXIS_STRIDE + 0] == pytest.approx(7.0)
    assert coeffs[0 * PHASE_STRIDE + 0 * AXIS_STRIDE + 1] == pytest.approx(8.0)
    assert coeffs[0 * PHASE_STRIDE + 0 * AXIS_STRIDE + 2] == pytest.approx(9.0)
    # c[1] = axes_r * start_v.  X=0, Y=10, Z=0.
    assert coeffs[0 * PHASE_STRIDE + 1 * AXIS_STRIDE + 0] == pytest.approx(0.0)
    assert coeffs[0 * PHASE_STRIDE + 1 * AXIS_STRIDE + 1] == pytest.approx(10.0)
    assert coeffs[0 * PHASE_STRIDE + 1 * AXIS_STRIDE + 2] == pytest.approx(0.0)
    # c[2] = axes_r * (accel/2).  X=0, Y=100, Z=0.
    assert coeffs[0 * PHASE_STRIDE + 2 * AXIS_STRIDE + 0] == pytest.approx(0.0)
    assert coeffs[0 * PHASE_STRIDE + 2 * AXIS_STRIDE + 1] == pytest.approx(100.0)
    assert coeffs[0 * PHASE_STRIDE + 2 * AXIS_STRIDE + 2] == pytest.approx(0.0)


def test_negative_axes_r_sign_propagation():
    # axes_r=(-1,0,0): reverse X-direction move. c[1] and c[2] should negate.
    coeffs = linear_as_quintic_coeffs(
        0.05, 0.0, 0.0,
        10.0, 20.0, 200.0,
        (-1.0, 0.0, 0.0), (0.0, 0.0, 0.0),
    )
    assert coeffs[0 * PHASE_STRIDE + 1 * AXIS_STRIDE + 0] == pytest.approx(-10.0)
    assert coeffs[0 * PHASE_STRIDE + 2 * AXIS_STRIDE + 0] == pytest.approx(-100.0)
