# test/test_blendmath.py
import math

import pytest

from klippy import blendmath


def test_vec_dot():
    assert blendmath.vdot((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) == 0.0
    assert blendmath.vdot((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == 32.0


def test_vec_cross():
    assert blendmath.vcross((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) == (0.0, 0.0, 1.0)
    assert blendmath.vcross((0.0, 1.0, 0.0), (1.0, 0.0, 0.0)) == (0.0, 0.0, -1.0)


def test_vec_norm():
    assert blendmath.vnorm((3.0, 4.0, 0.0)) == 5.0
    assert blendmath.vnorm((0.0, 0.0, 0.0)) == 0.0


def test_vec_scale():
    assert blendmath.vscale((1.0, 2.0, 3.0), 2.0) == (2.0, 4.0, 6.0)


def test_vec_add_sub():
    assert blendmath.vadd((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == (5.0, 7.0, 9.0)
    assert blendmath.vsub((4.0, 5.0, 6.0), (1.0, 2.0, 3.0)) == (3.0, 3.0, 3.0)


def test_vec_normalize():
    n = blendmath.vnormalize((3.0, 4.0, 0.0))
    assert n == pytest.approx((0.6, 0.8, 0.0))

    with pytest.raises(ValueError):
        blendmath.vnormalize((0.0, 0.0, 0.0))






class _FakeMove:
    """Minimal duck-typed stand-in for Kalico's Move class."""

    def __init__(self, axes_r, move_d, accel, max_cruise_v2, is_kinematic_move=True):
        # Kalico's Move.axes_r is a 4-vector [x, y, z, e]; only [:3] is used here.
        self.axes_r = axes_r
        self.move_d = move_d
        self.accel = accel
        self.max_cruise_v2 = max_cruise_v2
        self.is_kinematic_move = is_kinematic_move




def test_interpolate_extruder_through_arc():
    # Setup: a blend arc polyline, plus E-axis consumption rates per mm
    # for prev and next moves. The adapter helper should produce a list of
    # (x, y, z, e) points whose E increases monotonically from 0 to the
    # total E consumption across the blend arc length.
    polyline = [
        (-1.0, 0.0, 0.0),
        (-0.9, 0.1, 0.0),
        (-0.5, 0.5, 0.0),
        (-0.1, 0.9, 0.0),
        (0.0, 1.0, 0.0),
    ]
    # Suppose e_per_mm_prev = 0.05, e_per_mm_next = 0.04, and the arc
    # consumes d=1.0 from each side.
    e_per_mm_prev = 0.05
    e_per_mm_next = 0.04
    d_consumed = 1.0

    points_xyze = blendmath.interpolate_extruder(
        polyline,
        d_consumed=d_consumed,
        e_per_mm_prev=e_per_mm_prev,
        e_per_mm_next=e_per_mm_next,
    )

    # First point has E=0 (start of the blend).
    assert points_xyze[0][3] == pytest.approx(0.0, abs=1e-12)
    # Last point has total E = d_consumed * (prev_rate + next_rate) consumed over
    # the two halves of the blend. The blend replaces the final d_consumed mm of
    # the prev move (consuming d_consumed * e_per_mm_prev) plus the first
    # d_consumed mm of the next move (consuming d_consumed * e_per_mm_next).
    expected_total_e = d_consumed * (e_per_mm_prev + e_per_mm_next)
    assert points_xyze[-1][3] == pytest.approx(expected_total_e, rel=1e-9)
    # Monotonic non-decreasing.
    for p0, p1 in zip(points_xyze, points_xyze[1:]):
        assert p1[3] >= p0[3] - 1e-12
    # Length of output matches polyline.
    assert len(points_xyze) == len(polyline)




class _FakeAxisInputShaper:
    def __init__(self, axis, shaper_type, freq, damping_ratio=0.1):
        self.axis = axis
        self._type = shaper_type
        self._freq = freq
        self._damping = damping_ratio

    class _Params:
        def __init__(self, outer):
            self.shaper_type = outer._type
            self.shaper_freq = outer._freq
            self.damping_ratio = outer._damping

    @property
    def params(self):
        return self._Params(self)


class _FakeInputShaper:
    def __init__(self, shapers):
        self._shapers = shapers

    def get_shapers(self):
        return list(self._shapers)


class _FakePrinterObject:
    def __init__(self, input_shaper):
        self._is = input_shaper

    def lookup_object(self, name, default=None):
        if name == "input_shaper":
            return self._is
        return default


class _FakeToolheadWithShapers:
    def __init__(self, input_shaper):
        self.printer = _FakePrinterObject(input_shaper)


def test_extract_shapers_two_axes():
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    snaps = blendmath._extract_shapers(toolhead)
    snaps_by_axis = {s.axis: s for s in snaps}
    assert snaps_by_axis["x"].shaper_freq == 150.0
    assert snaps_by_axis["x"].shaper_type == "zv"
    assert snaps_by_axis["y"].shaper_freq == 80.0
    # A_axis is populated from find_shaper_max_accel — positive for shaped axes.
    assert snaps_by_axis["x"].A_axis > 0.0
    assert snaps_by_axis["y"].A_axis > 0.0
    # X should have larger A_axis (higher frequency, more accel budget).
    assert snaps_by_axis["x"].A_axis > snaps_by_axis["y"].A_axis


def test_extract_shapers_none_toolhead_returns_empty():
    assert blendmath._extract_shapers(None) == []


def test_extract_shapers_no_input_shaper_module_returns_empty():
    class _FakePrinterObjectNoIS:
        def lookup_object(self, name, default=None):
            return default

    class _FakeToolhead:
        printer = _FakePrinterObjectNoIS()

    assert blendmath._extract_shapers(_FakeToolhead()) == []


def test_extract_shapers_unshaped_axis_has_zero_A():
    # Axis with shaper_freq=0 is unshaped → snapshot carries A_axis=0.
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 0.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    snaps = blendmath._extract_shapers(toolhead)
    snaps_by_axis = {s.axis: s for s in snaps}
    assert snaps_by_axis["x"].shaper_freq == 0.0
    assert snaps_by_axis["x"].A_axis == 0.0
    # shaper_type is mirrored from params regardless of freq
    assert snaps_by_axis["x"].shaper_type == "zv"


def test_extract_shapers_zero_target_smoothing_returns_empty():
    # target_smoothing=0 is the sentinel to disable the shaper-derived
    # velocity cap. _extract_shapers must return [] so compute_shaper_bounds
    # produces (inf, inf) bounds — identical to "no input_shaper loaded".
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    is_obj.target_smoothing = 0.0
    toolhead = _FakeToolheadWithShapers(is_obj)
    assert blendmath._extract_shapers(toolhead) == []


