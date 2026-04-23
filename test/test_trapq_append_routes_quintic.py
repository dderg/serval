# Plan 8 Chunk 1 Task 2 — verify trapq_append dispatches through the
# quintic path. After Chunk 1 Task 8 flattens struct move, every trapq entry
# IS a quintic polynomial by construction (no tagged union any more). The
# tests assert that trapq_append produces history entries that (a) exist,
# (b) cover the right total move time, and (c) reconstruct the expected
# chord displacement — which the chord-projection in trapq_extract_old
# encodes via (start_v, x_r) on the pull_move.
#
# Walk-style mirrors test_plan5_integration.py and test_trapq_quintic.py:
# finalize moves into history, then use trapq_extract_old to project out
# the move metadata.

from __future__ import annotations

import pytest

from klippy import chelper
from klippy.chelper.linear_quintic import append_trapezoid_as_quintic


def _finalize_and_extract(ffi_main, ffi_lib, tq, t_print, duration, nmax=8):
    ffi_lib.trapq_finalize_moves(tq, t_print + duration + 1.0, 0.0)
    pm = ffi_main.new("struct pull_move[%d]" % nmax)
    n = ffi_lib.trapq_extract_old(
        tq, pm, nmax, t_print - 0.5, t_print + duration + 1.0,
    )
    return pm, n


def _find_motion_entry(pm, n):
    """Pick the pull_move entry that actually carries motion (start_v > 0)."""
    for i in range(n):
        if pm[i].start_v > 0.0:
            return pm[i]
    return None


def test_trapq_append_emits_single_quintic_entry():
    ffi_main, ffi_lib = chelper.get_ffi()
    tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    accel_t, cruise_t, decel_t = 0.05, 0.10, 0.05
    start_v, cruise_v, accel = 10.0, 20.0, 200.0
    total_t = accel_t + cruise_t + decel_t
    t_print = 1.0
    append_trapezoid_as_quintic(
        tq, t_print,
        accel_t, cruise_t, decel_t,
        0.0, 0.0, 0.0,   # start_pos
        1.0, 0.0, 0.0,   # axes_r — pure +X
        start_v, cruise_v, accel,
    )
    pm, n = _finalize_and_extract(ffi_main, ffi_lib, tq, t_print, total_t)
    # ONE motion-carrying entry spanning the full trapezoid.
    motion = _find_motion_entry(pm, n)
    assert motion is not None, (
        "expected at least one motion-carrying trapq history entry"
    )
    assert motion.move_t == pytest.approx(total_t, rel=1e-12)
    # Chord length for a pure +X trapezoid = accel_d + cruise_d + decel_d.
    accel_d = start_v * accel_t + 0.5 * accel * accel_t * accel_t
    cruise_d = cruise_v * cruise_t
    decel_d = cruise_v * decel_t - 0.5 * accel * decel_t * decel_t
    chord = accel_d + cruise_d + decel_d
    # trapq_extract_old projects quintic moves to average velocity along
    # the chord (start_v = chord / move_t).
    assert motion.start_v == pytest.approx(chord / total_t, rel=1e-9)
    assert motion.x_r == pytest.approx(1.0, abs=1e-12)
    assert motion.y_r == pytest.approx(0.0, abs=1e-12)


def test_trapq_append_pure_cruise_emits_quintic():
    # Edge case: accel_t = decel_t = 0, pure-cruise segment.
    ffi_main, ffi_lib = chelper.get_ffi()
    tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    accel_t, cruise_t, decel_t = 0.0, 0.10, 0.0
    start_v = cruise_v = 50.0
    accel = 0.0
    total_t = accel_t + cruise_t + decel_t
    t_print = 0.5
    append_trapezoid_as_quintic(
        tq, t_print,
        accel_t, cruise_t, decel_t,
        5.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        start_v, cruise_v, accel,
    )
    pm, n = _finalize_and_extract(ffi_main, ffi_lib, tq, t_print, total_t)
    motion = _find_motion_entry(pm, n)
    assert motion is not None, (
        "pure-cruise: expected a motion-carrying history entry"
    )
    assert motion.move_t == pytest.approx(total_t, rel=1e-12)
    # chord = cruise_v * cruise_t; average velocity = cruise_v.
    assert motion.start_v == pytest.approx(cruise_v, rel=1e-9)


def test_trapq_append_move_t_matches_sum_of_phase_times():
    # The single quintic entry must carry move_t = accel_t+cruise_t+decel_t.
    ffi_main, ffi_lib = chelper.get_ffi()
    tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    accel_t, cruise_t, decel_t = 0.03, 0.07, 0.04
    start_v, cruise_v, accel = 5.0, 15.0, 333.333333
    total_t = accel_t + cruise_t + decel_t
    t_print = 2.0
    append_trapezoid_as_quintic(
        tq, t_print,
        accel_t, cruise_t, decel_t,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        start_v, cruise_v, accel,
    )
    pm, n = _finalize_and_extract(ffi_main, ffi_lib, tq, t_print, total_t)
    motion = _find_motion_entry(pm, n)
    assert motion is not None
    assert motion.move_t == pytest.approx(total_t, rel=1e-12)
    assert motion.print_time == pytest.approx(t_print, rel=1e-12)
