# Plan 8 Chunk 1 Task 2 — verify trapq_append dispatches through the
# quintic path. After the rewrite, every emitted move must carry
# kind == MOVE_QUINTIC_POLY_T (=1), not the legacy MOVE_LINEAR (=0).
#
# Walk-style mirrors test_plan5_integration.py and test_trapq_quintic.py:
# finalize moves into history, then use trapq_extract_old to project out
# the move metadata (including kind).

from __future__ import annotations

import pytest

from klippy import chelper


def _finalize_and_extract(ffi_main, ffi_lib, tq, t_print, duration, nmax=8):
    ffi_lib.trapq_finalize_moves(tq, t_print + duration + 1.0, 0.0)
    pm = ffi_main.new("struct pull_move[%d]" % nmax)
    n = ffi_lib.trapq_extract_old(
        tq, pm, nmax, t_print - 0.5, t_print + duration + 1.0,
    )
    return pm, n


def test_trapq_append_emits_single_quintic_entry():
    ffi_main, ffi_lib = chelper.get_ffi()
    tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    accel_t, cruise_t, decel_t = 0.05, 0.10, 0.05
    start_v, cruise_v, accel = 10.0, 20.0, 200.0
    t_print = 1.0
    ffi_lib.trapq_append(
        tq, t_print,
        accel_t, cruise_t, decel_t,
        0.0, 0.0, 0.0,   # start_pos
        1.0, 0.0, 0.0,   # axes_r — pure +X
        start_v, cruise_v, accel,
    )
    pm, n = _finalize_and_extract(
        ffi_main, ffi_lib, tq, t_print,
        accel_t + cruise_t + decel_t,
    )
    # ONE quintic entry, not three linear trapq entries.
    kinds = [pm[i].kind for i in range(n)]
    # Depending on finalize semantics there may be boundary null-moves; what
    # matters is (a) at least one MOVE_QUINTIC_POLY_T entry exists, and
    # (b) no MOVE_LINEAR (kind=0) entry carrying motion exists.
    assert 1 in kinds, (
        "expected at least one MOVE_QUINTIC_POLY_T (kind=1) entry, got "
        "kinds=%r" % kinds
    )
    for i in range(n):
        if pm[i].kind == 0:
            # A kind=0 history entry with non-zero motion would mean
            # trapq_append still constructs MOVE_LINEAR structs.
            has_motion = (abs(pm[i].start_v) > 0.0 or abs(pm[i].accel) > 0.0)
            assert not has_motion, (
                "stray MOVE_LINEAR entry with motion: "
                "start_v=%r accel=%r" % (pm[i].start_v, pm[i].accel)
            )


def test_trapq_append_pure_cruise_emits_quintic():
    # Edge case: accel_t = decel_t = 0, pure-cruise segment.
    ffi_main, ffi_lib = chelper.get_ffi()
    tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    accel_t, cruise_t, decel_t = 0.0, 0.10, 0.0
    start_v = cruise_v = 50.0
    accel = 0.0
    t_print = 0.5
    ffi_lib.trapq_append(
        tq, t_print,
        accel_t, cruise_t, decel_t,
        5.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        start_v, cruise_v, accel,
    )
    pm, n = _finalize_and_extract(
        ffi_main, ffi_lib, tq, t_print,
        accel_t + cruise_t + decel_t,
    )
    kinds = [pm[i].kind for i in range(n)]
    assert 1 in kinds, (
        "pure-cruise: expected at least one MOVE_QUINTIC_POLY_T entry, got "
        "kinds=%r" % kinds
    )


def test_trapq_append_move_t_matches_sum_of_phase_times():
    # The single quintic entry must carry move_t = accel_t+cruise_t+decel_t.
    ffi_main, ffi_lib = chelper.get_ffi()
    tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    accel_t, cruise_t, decel_t = 0.03, 0.07, 0.04
    start_v, cruise_v, accel = 5.0, 15.0, 333.333333
    total_t = accel_t + cruise_t + decel_t
    t_print = 2.0
    ffi_lib.trapq_append(
        tq, t_print,
        accel_t, cruise_t, decel_t,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        start_v, cruise_v, accel,
    )
    pm, n = _finalize_and_extract(ffi_main, ffi_lib, tq, t_print, total_t)
    # Locate the quintic entry and assert its move_t.
    quintic = None
    for i in range(n):
        if pm[i].kind == 1:
            quintic = pm[i]
            break
    assert quintic is not None
    assert quintic.move_t == pytest.approx(total_t, rel=1e-12)
    assert quintic.print_time == pytest.approx(t_print, rel=1e-12)
