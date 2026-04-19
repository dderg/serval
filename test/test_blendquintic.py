# test/test_blendquintic.py
import math

import pytest

from klippy import blendquintic


def test_module_imports():
    assert blendquintic is not None


def test_quintic_eval_at_endpoints_returns_Q0_and_Q5():
    Q = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (3.0, 1.0, 0.0),
        (4.0, 2.0, 0.0),
        (5.0, 3.0, 0.0),
    ]
    p0 = blendquintic._quintic_eval(Q, 0.0)
    p1 = blendquintic._quintic_eval(Q, 1.0)
    assert p0 == pytest.approx(Q[0])
    assert p1 == pytest.approx(Q[5])


def test_quintic_eval_mid_matches_bernstein_direct():
    Q = [
        (0.0, 0.0, 0.0),
        (1.0, 2.0, 0.0),
        (2.0, 4.0, 0.0),
        (3.0, 4.0, 1.0),
        (4.0, 2.0, 1.0),
        (5.0, 0.0, 1.0),
    ]
    t = 0.5
    # Binomial(5, i) = 1, 5, 10, 10, 5, 1
    coeffs = [1, 5, 10, 10, 5, 1]
    omt = 1.0 - t
    expected = (0.0, 0.0, 0.0)
    for i, (c, q) in enumerate(zip(coeffs, Q)):
        w = c * (omt ** (5 - i)) * (t ** i)
        expected = (
            expected[0] + w * q[0],
            expected[1] + w * q[1],
            expected[2] + w * q[2],
        )
    got = blendquintic._quintic_eval(Q, t)
    assert got == pytest.approx(expected, abs=1e-12)


def test_quintic_eval_random_t_matches_bernstein_direct():
    # Randomish but deterministic set of t values.
    Q = [
        (0.1, -0.2, 0.3),
        (1.4, 2.5, -0.6),
        (2.7, 4.8, 0.9),
        (3.0, 3.1, 1.2),
        (4.3, 1.4, 1.5),
        (5.6, -0.7, 1.8),
    ]
    coeffs = [1, 5, 10, 10, 5, 1]
    for t in (0.1, 0.25, 0.37, 0.6, 0.8, 0.95):
        omt = 1.0 - t
        expected = (0.0, 0.0, 0.0)
        for i, (c, q) in enumerate(zip(coeffs, Q)):
            w = c * (omt ** (5 - i)) * (t ** i)
            expected = (
                expected[0] + w * q[0],
                expected[1] + w * q[1],
                expected[2] + w * q[2],
            )
        got = blendquintic._quintic_eval(Q, t)
        assert got == pytest.approx(expected, abs=1e-12)
