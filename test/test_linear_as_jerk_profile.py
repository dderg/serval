"""Tests for klippy/chelper/linear_quintic.c::build_jerk_profile_as_quintic_coeffs.

Plan 9 Phase A2a — emitter that translates a 1-D jerk_profile_result into the
multi-axis quintic-trapq slot layout (phases × 15-coeff × 4-axis).
"""
from __future__ import annotations

import math

import pytest

from klippy.chelper import get_ffi, jerk_profile as jp
from klippy.chelper.linear_quintic import (
    build_jerk_profile_as_quintic_coeffs,
)


# ---- helpers --------------------------------------------------------------

def _eval_phase(coeff_buf, phase_idx, axis, t):
    """Horner-eval the 15-coeff polynomial for (phase, axis) at phase-local t."""
    acc = 0.0
    for k in range(14, -1, -1):
        c = coeff_buf[(phase_idx * 15 + k) * 4 + axis]
        acc = acc * t + c
    return acc


def _eval_phase_deriv(coeff_buf, phase_idx, axis, t, order):
    if order == 0:
        return _eval_phase(coeff_buf, phase_idx, axis, t)
    derived = []
    for k in range(1, 15):
        c = coeff_buf[(phase_idx * 15 + k) * 4 + axis]
        derived.append(c * k)
    acc = 0.0
    for k in range(len(derived) - 1, -1, -1):
        acc = acc * t + derived[k]
    if order == 1:
        return acc
    raise NotImplementedError("order > 1 not needed in these tests")


def _make_single_axis_move():
    """Simple X-only move: v0=0, v1=0, v_peak=200, a_max=5000, j_max=100000, L=50."""
    prof = jp.compute_profile(0.0, 0.0, 200.0, 5000.0, 100000.0, 50.0)
    assert prof.status == jp.JP_OK
    return prof


# ---- tests ----------------------------------------------------------------

def test_emitter_populates_phase_count():
    """The emitter should populate n_phases phases, each with coeffs filled per axis."""
    prof = _make_single_axis_move()
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=(1.0, 0.0, 0.0),         # pure +X motion
        start_pos=(0.0, 0.0, 0.0),
    )
    # Number of phases equals nonzero-T segments in the profile.
    expected = sum(1 for s in prof.segments if s.T > 1e-12)
    assert n_phases == expected, f"n_phases {n_phases} != expected {expected}"
    assert len(phase_t_ends) == n_phases
    assert len(coeff_buf) == 32 * 15 * 4  # MOVE_MAX_PIECES * coeffs * axes


def test_emitter_reproduces_position_on_x_axis():
    """X-axis polynomial evaluated at each phase boundary matches jerk_profile position."""
    prof = _make_single_axis_move()
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=(1.0, 0.0, 0.0),
        start_pos=(0.0, 0.0, 0.0),
    )
    running_x = 0.0
    seg_iter = iter(s for s in prof.segments if s.T > 1e-12)
    for phase_idx in range(n_phases):
        seg = next(seg_iter)
        x_end = _eval_phase(coeff_buf, phase_idx, 0, seg.T)
        seg_local_end = 0.0
        for k in range(len(seg.coeffs) - 1, -1, -1):
            seg_local_end = seg_local_end * seg.T + seg.coeffs[k]
        assert x_end == pytest.approx(seg_local_end, abs=1e-9, rel=1e-9)
        running_x = x_end


def test_emitter_projects_onto_3d_direction():
    """A 3D direction (rx, ry, rz) with |r|=1 produces per-axis polys = r_axis * p(t)."""
    prof = _make_single_axis_move()
    r = (0.6, 0.8, 0.0)
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=r,
        start_pos=(0.0, 0.0, 0.0),
    )
    for phase_idx in range(n_phases):
        t = phase_t_ends[phase_idx] - (phase_t_ends[phase_idx - 1] if phase_idx else 0.0)
        tm = t * 0.5
        px = _eval_phase(coeff_buf, phase_idx, 0, tm)
        py = _eval_phase(coeff_buf, phase_idx, 1, tm)
        pz = _eval_phase(coeff_buf, phase_idx, 2, tm)
        assert px * r[1] == pytest.approx(py * r[0], abs=1e-9, rel=1e-9)
        assert pz == pytest.approx(0.0, abs=1e-12)


def test_emitter_applies_start_position_offset():
    """Nonzero start_pos shifts each axis's c0 on phase 0."""
    prof = _make_single_axis_move()
    start_pos = (10.0, 20.0, -5.0)
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=(1.0, 0.0, 0.0),
        start_pos=start_pos,
    )
    assert _eval_phase(coeff_buf, 0, 0, 0.0) == pytest.approx(start_pos[0], abs=1e-12)
    assert _eval_phase(coeff_buf, 0, 1, 0.0) == pytest.approx(start_pos[1], abs=1e-12)
    assert _eval_phase(coeff_buf, 0, 2, 0.0) == pytest.approx(start_pos[2], abs=1e-12)


def test_emitter_rejects_bad_profile():
    """A profile with status != JP_OK should raise."""
    bad = jp.compute_profile(0.0, 0.0, 0.0, 5000.0, 100000.0, 10.0)
    assert bad.status == jp.JP_BAD_INPUT
    with pytest.raises(ValueError):
        build_jerk_profile_as_quintic_coeffs(
            profile=bad, axes_r=(1.0, 0.0, 0.0), start_pos=(0.0, 0.0, 0.0))


def test_roundtrip_eval_matches_profile_sum():
    """Sample positions from coeff_buf at key times; must match jerk_profile's
    own polynomial evaluation + start_pos offset."""
    prof = jp.compute_profile(0.0, 0.0, 200.0, 5000.0, 100000.0, 50.0)
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=(1.0, 0.0, 0.0),
        start_pos=(100.0, 0.0, 0.0),
    )
    segs_nonzero = [s for s in prof.segments if s.T > 1e-12]
    for phase_idx, seg in enumerate(segs_nonzero):
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            local_t = frac * seg.T
            # Direct eval on the segment's 1-D polynomial.
            p_1d = 0.0
            for c in reversed(seg.coeffs):
                p_1d = p_1d * local_t + c
            x_expected = 100.0 + p_1d
            x_from_buf = _eval_phase(coeff_buf, phase_idx, 0, local_t)
            assert x_from_buf == pytest.approx(x_expected, abs=1e-9, rel=1e-9)
