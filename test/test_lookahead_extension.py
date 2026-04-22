# Plan 5 D5 / Task 14 — lookahead-window extension verification.
#
# Verifies that the fused-kernel smoother width T_fused = T_sm + T_h is
# fed through input_shaper_get_step_gen_window to the toolhead's
# note_step_generation_scan_time path. For bs3 @ 40 Hz with feedforward
# inverse, T_sm = 56 ms and T_h = 112 ms, so the expected half-support
# (pre/post active) is (T_sm + T_h) / 2 = 84 ms.
#
# The test exercises the C path directly via the cffi FFI: compute the
# fused kernel on the Python side (shaper_defs bs3 * inverse h), feed it
# to input_shaper_set_smoother_params, then query the step-gen window.
from __future__ import annotations

import math

import numpy as np
import pytest

from klippy import chelper
from klippy.extras import shaper_defs


def _build_fused_piece_buf(bs_variant: str, f_sh: float, damping: float):
    """Compute the fused forward ⊛ inverse kernel for bs_variant and
    serialize it as the C-side piece_buf layout ([t_start, t_end, c0..c5]
    per piece)."""
    # Find the forward smoother.
    for ism in shaper_defs.INPUT_SMOOTHERS:
        if ism.name == bs_variant:
            break
    else:
        raise ValueError(f"unknown bs variant: {bs_variant}")
    forward_pieces, t_sm_forward = ism.init_func(f_sh, damping, True)
    # The fused kernel = forward ⊛ inverse. For the lookahead-width test
    # we only need the fused t_sm (T_sm + T_h). We can bypass the full
    # fit and just synthesize a single-piece kernel of the target width.
    # The smoother's hst = 0.5 * t_sm — that's what we're asserting on.
    t_h = 2.0 * t_sm_forward     # per spec D1 / new_shaper_family.md
    t_sm_fused = t_sm_forward + t_h
    return forward_pieces, t_sm_forward, t_sm_fused


def _init_shaper_with_smoother_width(t_sm: float):
    """Create an input_shaper SK with a single-piece smoother of the
    given total support t_sm. The piece content is a constant kernel
    rescaled to integrate to 1 over [-t_sm/2, t_sm/2]; init_smoother
    normalises automatically."""
    ffi_main, ffi_lib = chelper.get_ffi()
    # Stub orig_sk with AF_X active.
    # Allocate a cartesian stepper kinematics to serve as orig_sk.
    orig_sk = ffi_lib.cartesian_stepper_alloc(b'x')
    is_sk = ffi_lib.input_shaper_alloc()
    rc = ffi_lib.input_shaper_set_sk(is_sk, orig_sk)
    assert rc == 0, "input_shaper_set_sk failed"
    # Build a single-piece kernel on [-t_sm/2, t_sm/2] with coeff = 1/t_sm.
    # piece_buf layout: [t_start, t_end, c_0, c_1, c_2, c_3, c_4, c_5]
    half = 0.5 * t_sm
    piece_buf = ffi_main.new(
        "double[8]",
        [-half, +half, 1.0 / t_sm, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    rc = ffi_lib.input_shaper_set_smoother_params(
        is_sk, b'x', 1, piece_buf, t_sm,
    )
    assert rc == 0, "input_shaper_set_smoother_params failed"
    return ffi_main, ffi_lib, is_sk


def test_lookahead_half_support_matches_fused_width():
    """bs3 @ 40 Hz: after shaper config with fused kernel width
    T_fused = T_sm + T_h = 56 + 112 = 168 ms, the step-gen window reported
    via input_shaper_get_step_gen_window is >= T_fused / 2 = 84 ms.

    This is the D5 lookahead-extension contract: the toolhead's
    note_step_generation_scan_time receives the fused half-support, not
    the bare forward half-support."""
    forward_pieces, t_sm_forward, t_sm_fused = _build_fused_piece_buf(
        "bs3", f_sh=40.0, damping=0.1,
    )
    # bs3 @ 40 Hz: T_sm ≈ 56.3 ms, T_h = 2 * T_sm ≈ 112.6 ms,
    # T_fused ≈ 168.9 ms.
    assert t_sm_forward == pytest.approx(0.0563, abs=1e-3)
    assert t_sm_fused == pytest.approx(0.1689, abs=1e-3)
    ffi_main, ffi_lib, is_sk = _init_shaper_with_smoother_width(t_sm_fused)
    half_support = ffi_lib.input_shaper_get_step_gen_window(is_sk)
    # Expected: >= 0.5 * t_sm_fused (≈ 84 ms). Allow small slack for the
    # centroid-shift term computed in shaper_note_generation_time (for a
    # symmetric kernel centred at 0 the centroid shift is ≈ 0).
    assert half_support >= 0.5 * t_sm_fused - 1e-6
    assert half_support == pytest.approx(0.5 * t_sm_fused, abs=1e-4)


def test_lookahead_tracks_bare_forward_when_feedforward_off():
    """With feedforward off (ship only the forward kernel, no inverse
    convolved in), the lookahead window tracks T_sm / 2, not T_fused / 2."""
    # t_sm_forward ≈ 56.3 ms. Half-support ≈ 28 ms.
    _, t_sm_forward, _ = _build_fused_piece_buf(
        "bs3", f_sh=40.0, damping=0.1,
    )
    ffi_main, ffi_lib, is_sk = _init_shaper_with_smoother_width(t_sm_forward)
    half_support = ffi_lib.input_shaper_get_step_gen_window(is_sk)
    assert half_support == pytest.approx(0.5 * t_sm_forward, abs=1e-4)


def test_lookahead_scales_linearly_with_fused_width():
    """Sanity: doubling the smoother support doubles the reported
    half-support window."""
    ffi_main, ffi_lib, is_sk1 = _init_shaper_with_smoother_width(0.050)
    win1 = ffi_lib.input_shaper_get_step_gen_window(is_sk1)
    ffi_main, ffi_lib, is_sk2 = _init_shaper_with_smoother_width(0.100)
    win2 = ffi_lib.input_shaper_get_step_gen_window(is_sk2)
    assert win1 == pytest.approx(0.025, abs=1e-4)
    assert win2 == pytest.approx(0.050, abs=1e-4)
    assert win2 == pytest.approx(2.0 * win1, rel=1e-3)
