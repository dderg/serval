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
    """Mirrors the API of klippy.extras.input_shaper.AxisInputShaper.

    The real class exposes axis access via ``get_axis()``, not a direct
    ``.axis`` attribute — regression: test/test_blendmath.py used to
    expose ``.axis`` directly and masked a blendmath bug on real hardware
    (see commit adding this comment).
    """

    def __init__(self, axis, shaper_type, freq, damping_ratio=0.1):
        self._axis = axis
        self._type = shaper_type
        self._freq = freq
        self._damping = damping_ratio

    def get_axis(self):
        return self._axis

    def get_type(self):
        return self._type

    class _Params:
        def __init__(self, outer):
            self.axis = outer._axis
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


def test_extract_shapers_uses_real_axis_input_shaper_api():
    """Regression: route through the real AxisInputShaper class (post
    BE-v2 smooth-shapers port) and confirm blendmath._extract_shapers
    uses get_axis() instead of a direct .axis attribute. The pre-port
    API exposed .axis directly; the port removed it and the fake had
    masked the mismatch — breaking TEST_RESONANCES on real hardware.
    """
    from klippy.extras import input_shaper as _is_mod
    params = _is_mod.TypedInputShaperParams("x", "zv", None)
    params.shaper_freq = 50.0
    params.damping_ratio = 0.1
    real_axis_shaper = _is_mod.AxisInputShaper(params)
    assert not hasattr(real_axis_shaper, "axis"), (
        "AxisInputShaper is not expected to expose .axis directly; "
        "blendmath must call get_axis()."
    )
    assert real_axis_shaper.get_axis() == "x"

    is_obj = _FakeInputShaper([real_axis_shaper])
    toolhead = _FakeToolheadWithShapers(is_obj)
    snaps = blendmath._extract_shapers(toolhead)
    assert len(snaps) == 1
    assert snaps[0].axis == "x"
    assert snaps[0].shaper_type == "zv"
    assert snaps[0].shaper_freq == 50.0
    assert snaps[0].A_axis > 0.0


def test_extract_shapers_smooth_family_axis_has_zero_A():
    """Smooth-family axes carry TypedInputSmootherParams (no shaper_freq
    / shaper_type / damping_ratio fields). _extract_shapers must not
    crash on them — it records A_axis=0.0 because the arc-blending
    velocity cap only consumes the impulse family today.
    """
    from klippy.extras import input_shaper as _is_mod
    params = _is_mod.TypedInputSmootherParams("x", "smooth_mzv", None)
    params.smoother_freq = 40.0
    real_axis_smoother = _is_mod.AxisInputSmoother(params)
    assert real_axis_smoother.get_axis() == "x"

    is_obj = _FakeInputShaper([real_axis_smoother])
    toolhead = _FakeToolheadWithShapers(is_obj)
    snaps = blendmath._extract_shapers(toolhead)
    assert len(snaps) == 1
    assert snaps[0].axis == "x"
    assert snaps[0].A_axis == 0.0


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


# --- Task 2: suppressed_junction_v + _scv_equivalent_junction_v ---

def test_scv_equivalent_junction_v_collinear_returns_inf():
    """Collinear corner (sin_half=0) -> no cap derivable -> +inf."""
    v = blendmath._scv_equivalent_junction_v(
        cos_half=1.0, sin_half=0.0,
        corner_deviation=0.1, sigma_T_max=0.015, a_max=50000.0,
    )
    assert math.isinf(v)


def test_scv_equivalent_junction_v_reversal_returns_near_zero():
    """Near-reversal (cos_half=0) -> R_scv=0 -> v_j=0."""
    v = blendmath._scv_equivalent_junction_v(
        cos_half=1e-5, sin_half=1.0,
        corner_deviation=0.1, sigma_T_max=0.015, a_max=50000.0,
    )
    assert v >= 0.0 and v < 1.0  # sub-1 mm/s


def test_scv_equivalent_junction_v_right_angle_is_finite():
    """90deg corner (cos_half = sin_half = 1/sqrt(2)) -> finite positive cap."""
    import math as _m
    h = _m.sqrt(2.0) / 2.0
    v = blendmath._scv_equivalent_junction_v(
        cos_half=h, sin_half=h,
        corner_deviation=0.1, sigma_T_max=0.015, a_max=50000.0,
    )
    assert math.isfinite(v) and v > 0.0


def test_scv_equivalent_junction_v_zero_sigma_returns_inf():
    """sigma_T_max=0 -> no cap derivable -> +inf."""
    v = blendmath._scv_equivalent_junction_v(
        cos_half=0.7, sin_half=0.7,
        corner_deviation=0.1, sigma_T_max=0.0, a_max=50000.0,
    )
    assert math.isinf(v)


def test_suppressed_junction_v_none_without_shaper():
    """Toolhead with no input_shaper -> no cap derivable -> return None."""
    class _TH:
        printer = None
    prev = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=10.0, accel=50000.0, max_cruise_v2=1000.0)
    nxt  = _FakeMove(axes_r=(0.0, 1.0, 0.0), move_d=10.0, accel=50000.0, max_cruise_v2=1000.0)
    assert blendmath.suppressed_junction_v(prev, nxt, 0.1, _TH()) is None


def test_suppressed_junction_v_collinear_returns_none():
    """Collinear (sin_half < COLLINEAR_EPS) -> None (no cap needed)."""
    prev = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=10.0, accel=50000.0, max_cruise_v2=1000.0)
    nxt  = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=10.0, accel=50000.0, max_cruise_v2=1000.0)
    th = _FakeToolheadWithShapers(_FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 50.0),
        _FakeAxisInputShaper("y", "zv", 50.0),
    ]))
    assert blendmath.suppressed_junction_v(prev, nxt, 0.1, th) is None


def test_suppressed_junction_v_right_angle_returns_finite():
    """90deg corner with shaper loaded -> finite positive cap."""
    prev = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=10.0, accel=50000.0, max_cruise_v2=1000.0)
    nxt  = _FakeMove(axes_r=(0.0, 1.0, 0.0), move_d=10.0, accel=50000.0, max_cruise_v2=1000.0)
    th = _FakeToolheadWithShapers(_FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 50.0),
        _FakeAxisInputShaper("y", "zv", 50.0),
    ]))
    v = blendmath.suppressed_junction_v(prev, nxt, 0.1, th)
    assert v is not None and math.isfinite(v) and v > 0.0


# ---------------------------------------------------------------------------
# _compute_A_axis_smooth_is tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("shaper_type,expected_A_axis", [
    ("smooth_zv",        5732.9),
    ("smooth_mzv",       4548.5),
    ("smooth_ei",        4023.8),
    ("smooth_2hump_ei",  3844.3),
    ("smooth_zvd_ei",    2609.4),
    ("smooth_si",        3819.4),
])
def test_compute_A_axis_smooth_is_expected_values(shaper_type, expected_A_axis):
    """A_axis for each SIS kernel at f_sh=40, ts=0.12 matches derivation.

    Derivation: plan4-derivations/A_axis_smooth_is.md.
    Tolerance: rel=1e-3 — the 5% digit is noise vs the underlying
    closed form's 1e-10 precision, but test tolerance is set to catch
    gross implementation errors (typos in a coefficient, wrong
    target_smoothing default).
    """
    A = blendmath._compute_A_axis_smooth_is(shaper_type, 40.0, 0.1,
                                             target_smoothing=0.12)
    assert A == pytest.approx(expected_A_axis, rel=1e-3)
    assert A > 0.0
    assert math.isfinite(A)


def test_compute_A_axis_smooth_is_scales_with_freq_squared():
    """A_axis proportional to f_sh^2 — doubling frequency quadruples A_axis."""
    A_40 = blendmath._compute_A_axis_smooth_is("smooth_mzv", 40.0, 0.1,
                                                target_smoothing=0.12)
    A_80 = blendmath._compute_A_axis_smooth_is("smooth_mzv", 80.0, 0.1,
                                                target_smoothing=0.12)
    ratio = A_80 / A_40
    assert ratio == pytest.approx(4.0, rel=1e-6)


def test_compute_A_axis_smooth_is_damping_independent():
    """SIS kernels are fixed-shape — damping_ratio argument is accepted
    for signature parity with FIR but has no effect on A_axis.
    """
    A_low = blendmath._compute_A_axis_smooth_is("smooth_mzv", 40.0, 0.0,
                                                 target_smoothing=0.12)
    A_high = blendmath._compute_A_axis_smooth_is("smooth_mzv", 40.0, 0.5,
                                                  target_smoothing=0.12)
    assert A_low == pytest.approx(A_high, rel=1e-9)


def test_compute_A_axis_smooth_is_unknown_returns_zero():
    """Unknown SIS name returns 0.0 rather than raising.
    Contract: _extract_shapers uses 0.0 as the sentinel for 'axis has
    no effective shaper contribution' — matches the behavior of the
    existing non-FIR-non-SIS fallthrough in _extract_shapers.
    """
    A = blendmath._compute_A_axis_smooth_is("smooth_nonexistent", 40.0, 0.1,
                                             target_smoothing=0.12)
    assert A == 0.0


def test_compute_A_axis_smooth_is_zero_freq_returns_zero():
    """shaper_freq <= 0 returns 0.0 (no shaper -> no cap contribution)."""
    A = blendmath._compute_A_axis_smooth_is("smooth_mzv", 0.0, 0.1,
                                             target_smoothing=0.12)
    assert A == 0.0


def test_extract_shapers_smooth_is_produces_nonzero_A_axis():
    """After D1, SIS axes must carry a finite positive A_axis, not 0.0.

    This is the regression test for the P0 silent no-op that Plan 4
    D1 fixes.
    """
    class MockShaperParams:
        shaper_type = "smooth_mzv"
        shaper_freq = 40.0
        damping_ratio = 0.1

    class MockAxisShaper:
        def __init__(self, axis):
            self._axis = axis
            self.params = MockShaperParams()
        def get_axis(self):
            return self._axis

    class MockInputShaper:
        target_smoothing = None  # default
        def get_shapers(self):
            return [MockAxisShaper("x"), MockAxisShaper("y")]

    class MockPrinter:
        def lookup_object(self, name, default=None):
            if name == "input_shaper":
                return MockInputShaper()
            return default

    class MockToolhead:
        printer = MockPrinter()

    snaps = blendmath._extract_shapers(MockToolhead())
    assert len(snaps) == 2
    for s in snaps:
        assert s.shaper_type == "smooth_mzv"
        assert s.A_axis > 0.0
        import math
        assert math.isfinite(s.A_axis)


def test_extract_shapers_fir_unchanged():
    """FIR path must still produce A_axis via ShaperCalibrate.find_shaper_max_accel."""
    class MockShaperParams:
        shaper_type = "mzv"
        shaper_freq = 40.0
        damping_ratio = 0.1

    class MockAxisShaper:
        def __init__(self, axis):
            self._axis = axis
            self.params = MockShaperParams()
        def get_axis(self):
            return self._axis

    class MockInputShaper:
        target_smoothing = None
        def get_shapers(self):
            return [MockAxisShaper("x")]

    class MockPrinter:
        def lookup_object(self, name, default=None):
            return MockInputShaper() if name == "input_shaper" else default

    class MockToolhead:
        printer = MockPrinter()

    snaps = blendmath._extract_shapers(MockToolhead())
    assert len(snaps) == 1
    assert snaps[0].shaper_type == "mzv"
    assert snaps[0].A_axis > 0.0
