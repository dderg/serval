# test/test_unified_v_cap.py
#
# Plan 5 D7: unified v_cap_fn(s) composition + blendextruder.cap_move
# retirement. The former per-move blendextruder.cap_move is preserved for
# linear trunc edges but absorbed into v_cap_fn(s) as the v_extr(s)
# branch for blend emission.

import math

import pytest

from klippy import blendextruder, blendquintic, blendshape


class _LinearMove:
    def __init__(self, start_xy, end_xy, flow_k=0.04):
        dx = end_xy[0] - start_xy[0]
        dy = end_xy[1] - start_xy[1]
        length = math.hypot(dx, dy)
        self.axes_d = (dx, dy, 0.0, flow_k * length)
        self.move_d = length
        self.axes_r = (dx / length, dy / length, 0.0, flow_k)
        self.start_pos = (start_xy[0], start_xy[1], 0.0, 0.0)
        self.end_pos = (end_xy[0], end_xy[1], 0.0, flow_k * length)


def _build_90deg_shape(corner_deviation=0.05, extruder_caps=None):
    m_prev = _LinearMove((0.0, 0.0), (10.0, 0.0), flow_k=0.04)
    m_next = _LinearMove((10.0, 0.0), (10.0, 10.0), flow_k=0.04)
    limits = blendshape.KinematicLimits(
        a_max=5000.0, v_max=500.0, jerk_max=None,
        extruder_caps=extruder_caps, shapers=[],
    )
    shape = blendquintic.QuinticShape.from_moves(
        m_prev, m_next, corner_deviation, limits,
    )
    return shape


def test_v_cap_fn_includes_extruder_cap():
    """Plan 5 D7: v_cap_fn(s) composes the per-s extruder flow cap when
    extruder_caps + pa_model + flow ratios are supplied. With a
    deliberately-tight v_E_max the extruder cap should bind at the
    midpoint, producing a lower v_cap than without the flow cap."""
    # Same 90 deg shape, but with an extruder cap that forces a tight
    # flow ceiling.
    tight_caps = blendshape.ExtruderLimits(
        a_E_max=5000.0, v_E_max=0.5,  # very tight -> v_cap dominated by flow
        smooth_time=0.04,
    )
    no_caps = None

    shape_tight = _build_90deg_shape(extruder_caps=tight_caps)
    shape_open = _build_90deg_shape(extruder_caps=no_caps)
    assert shape_tight is not None and shape_open is not None

    pa_snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.04,))
    s_mid = shape_tight.arc_length / 2.0
    v_with_extr = shape_tight.v_cap_fn(
        s_mid, prev_flow_k=0.04, nxt_flow_k=0.04, pa_model=pa_snap,
    )
    v_no_extr = shape_open.v_cap_fn(
        s_mid, prev_flow_k=0.04, nxt_flow_k=0.04, pa_model=pa_snap,
    )
    # With tight v_E_max=0.5 and k=0.04 the flow cap should dominate:
    # v_extr = 0.5 / 0.04 = 12.5 mm/s (approx).
    assert v_with_extr < v_no_extr
    assert v_with_extr <= 15.0


def test_v_cap_fn_flow_cap_skipped_for_travel():
    """k <= 0 (pure travel): flow cap returns +inf regardless of pa_model."""
    shape = _build_90deg_shape(
        extruder_caps=blendshape.ExtruderLimits(
            a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04,
        ),
    )
    pa_snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.04,))
    s_mid = shape.arc_length / 2.0
    v_travel = shape.v_cap_fn(
        s_mid, prev_flow_k=0.0, nxt_flow_k=0.0, pa_model=pa_snap,
    )
    v_no_flow = shape.v_cap_fn(
        s_mid, prev_flow_k=None, nxt_flow_k=None, pa_model=None,
    )
    # Travel and "no flow plumbing" should produce the same cap.
    assert v_travel == pytest.approx(v_no_flow, rel=1e-12)


def test_blendextruder_cap_move_retired_from_quintic_emit_path():
    """Plan 5 D7: QuinticBlendMove does NOT have a separate cap_move call
    layered on its velocity. The quintic emit's cap comes solely from
    TOPP's composition of v_cap_fn(s). Inspect the blendplanner source
    to verify no cap_move call exists in _emit_blend.
    """
    import inspect
    from klippy import blendplanner

    src = inspect.getsource(blendplanner.CornerBlender._emit_blend)
    # The retired path does not mention cap_move inside _emit_blend.
    assert "cap_move" not in src, (
        "_emit_blend must not call blendextruder.cap_move — "
        "the per-s flow cap is absorbed into v_cap_fn(s) as v_extr(s)."
    )
    # It must still route through TOPP for the trapezoid-in-s profile.
    assert "topp" in src.lower() or "topp_trapezoid" in src
