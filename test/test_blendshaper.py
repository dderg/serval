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
