import math
import pytest


def _setup_linear(pa=0.04):
    from klippy.kinematics.extruder import PALinearModel
    m = PALinearModel.__new__(PALinearModel)
    m.pressure_advance = pa
    return m


def _setup_tanh(la=0.0, no=0.04, lv=100.0):
    from klippy.kinematics.extruder import PATanhModel
    m = PATanhModel.__new__(PATanhModel)
    m.linear_advance = la
    m.nonlinear_offset = no
    m.linearization_velocity = lv
    return m


def _setup_recipr(la=0.0, no=0.04, lv=100.0):
    from klippy.kinematics.extruder import PAReciprModel
    m = PAReciprModel.__new__(PAReciprModel)
    m.linear_advance = la
    m.nonlinear_offset = no
    m.linearization_velocity = lv
    return m


def test_linear_f_prime_is_constant_pa():
    m = _setup_linear(pa=0.04)
    assert m.f_prime(0.0) == 0.04
    assert m.f_prime(100.0) == 0.04
    assert m.f_prime(600.0) == 0.04


def test_linear_f_double_prime_is_zero():
    m = _setup_linear(pa=0.04)
    assert m.f_double_prime(0.0) == 0.0
    assert m.f_double_prime(100.0) == 0.0


def test_tanh_f_prime_at_zero_is_max():
    m = _setup_tanh(la=0.01, no=0.04, lv=100.0)
    fp0 = m.f_prime(0.0)
    fp100 = m.f_prime(100.0)
    fp500 = m.f_prime(500.0)
    assert fp0 == pytest.approx(0.01 + 0.04 / 100.0, rel=1e-9)
    assert fp0 > fp100 > fp500
    assert fp500 > 0.01 - 1e-6


def test_tanh_f_double_prime_is_negative():
    m = _setup_tanh(la=0.0, no=0.04, lv=100.0)
    assert m.f_double_prime(0.0) == 0.0
    assert m.f_double_prime(50.0) < 0.0
    assert m.f_double_prime(200.0) < 0.0


def test_tanh_f_prime_numerical_check():
    m = _setup_tanh(la=0.005, no=0.04, lv=100.0)
    def f(v):
        return m.linear_advance * v + m.nonlinear_offset * math.tanh(v / m.linearization_velocity)
    h = 1e-4
    for v in (0.5, 50.0, 200.0, 400.0):
        fd = (f(v + h) - f(v - h)) / (2 * h)
        assert m.f_prime(v) == pytest.approx(fd, rel=1e-6)


def test_recipr_f_prime_at_zero_is_max():
    m = _setup_recipr(la=0.01, no=0.04, lv=100.0)
    fp0 = m.f_prime(0.0)
    fp100 = m.f_prime(100.0)
    fp500 = m.f_prime(500.0)
    assert fp0 == pytest.approx(0.01 + 0.04 / 100.0, rel=1e-9)
    assert fp0 > fp100 > fp500


def test_recipr_f_double_prime_is_negative():
    m = _setup_recipr(la=0.0, no=0.04, lv=100.0)
    assert m.f_double_prime(0.0) < 0.0
    assert m.f_double_prime(100.0) < 0.0


def test_recipr_f_prime_numerical_check():
    m = _setup_recipr(la=0.005, no=0.04, lv=100.0)
    def f(v):
        r = v / m.linearization_velocity
        return m.linear_advance * v + m.nonlinear_offset * (1.0 - 1.0 / (1.0 + r))
    h = 1e-4
    for v in (0.5, 50.0, 200.0, 400.0):
        fd = (f(v + h) - f(v - h)) / (2 * h)
        assert m.f_prime(v) == pytest.approx(fd, rel=1e-6)
