import pytest
from klippy.chelper import get_ffi
from klippy.chelper.linear_quintic import linear_as_quintic_coeffs


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
    assert len(coeffs) == 99

    # Degenerate quintic: phase 0 (accel), c[0]=start_pos_x=0, c[1]=start_v,
    # c[2]=half_accel=100, c[3..10]=0.
    # Buffer index: phase * 33 + coeff * 3 + axis.
    assert coeffs[0 * 33 + 0 * 3 + 0] == pytest.approx(0.0)    # x0
    assert coeffs[0 * 33 + 1 * 3 + 0] == pytest.approx(10.0)   # v0
    assert coeffs[0 * 33 + 2 * 3 + 0] == pytest.approx(100.0)  # half_a
    for i in range(3, 11):
        assert coeffs[0 * 33 + i * 3 + 0] == 0.0


def test_degenerate_quintic_pure_cruise():
    # accel_t=0, cruise_t=0.1, decel_t=0: constant velocity segment only.
    coeffs = linear_as_quintic_coeffs(
        0.0, 0.1, 0.0,
        50.0, 50.0, 0.0,
        (1.0, 0.0, 0.0), (5.0, 0.0, 0.0),
    )
    # Cruise phase (phase 1): c[0]=5 (x0 at start of cruise), c[1]=50, c[2..10]=0.
    assert coeffs[1 * 33 + 0 * 3 + 0] == pytest.approx(5.0)
    assert coeffs[1 * 33 + 1 * 3 + 0] == pytest.approx(50.0)
    assert coeffs[1 * 33 + 2 * 3 + 0] == 0.0
