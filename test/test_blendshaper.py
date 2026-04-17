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
