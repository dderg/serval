"""servo_sample.c against independent models of the shaped trajectory.

Pulse shapers are checked against tools/motion_eval's exec(t) = sum_j
w_j * cmd(t + s_j). Smoothers are checked against an exact piecewise
polynomial convolution built here from the same kernel density, with the
acceleration's velocity-jump terms added analytically.
"""

from __future__ import annotations

import numpy as np
import pytest
from ethercat_support import (
    apply_shaper,
    build_trapq,
    corner_path_moves,
    isclose_to_any,
    shaper_state,
)
from numpy.polynomial import Polynomial

from klippy import chelper
from klippy.extras import shaper_defs
from klippy.plugins.ethercat.servo_stepper import (
    NonlinearKinematics,
    probe_linear_map,
)
from tools.motion_eval.analysis import Kernel, Segments

EPS = np.finfo(float).eps
ACCEL = 5000.0
CRUISE_V = 150.0
SHAPER_FREQ = 45.0
INTERVAL = 250e-6
# A sample at the exact instant of a velocity jump may take either one-sided
# limit; samples that close to a jump are compared on position only.
JUMP_GUARD_S = 1e-9
# Rounding headroom: the C and numpy evaluations sum a handful of terms of
# the trajectory's magnitude, each correct to a few ulps.
ULPS = 1e4


def generate(sk, tq, start, interval, count):
    """(return code, pos, vel, acc, error record, row, offset)"""
    ffi_main, ffi_lib = chelper.get_ffi()
    ffi_lib.itersolve_set_trapq(sk, tq, 1.0)
    offset, row = probe_linear_map(sk)
    sampler = ffi_main.gc(
        ffi_lib.servo_sampler_alloc(), ffi_lib.servo_sampler_free
    )
    ffi_lib.servo_sampler_set_kinematics(sampler, sk, offset, *row)
    out = [ffi_main.new("double[]", count) for _ in range(3)]
    err = ffi_main.new("struct servo_sample_error *")
    ret = ffi_lib.servo_sampler_generate(
        sampler, start, interval, count, *out, err
    )
    arrays = [
        np.frombuffer(ffi_main.buffer(a), dtype=float).copy() for a in out
    ]
    return ret, *arrays, err, np.array(row), offset


def sample(sk, tq, start, interval, count):
    ret, pos, vel, acc, err, row, offset = generate(
        sk, tq, start, interval, count
    )
    assert ret == 0, (ret, err.time, err.position, err.linear_position)
    return pos, vel, acc, row, offset


def shaped_stepper(alloc, param, shapers):
    ffi_main, ffi_lib = chelper.get_ffi()
    orig = ffi_main.gc(getattr(ffi_lib, alloc)(param), ffi_lib.free)
    sk = ffi_main.gc(ffi_lib.input_shaper_alloc(), ffi_lib.free)
    assert ffi_lib.input_shaper_set_sk(sk, orig) == 0
    for axis, state in shapers.items():
        apply_shaper(sk, axis, state)
    return sk, orig


def pulse_model(seg, kernel, t):
    out = np.zeros((3, len(t), 3))
    for w, s in zip(kernel.weights, kernel.shifts):
        out += w * np.array(seg.eval(t + s))
    return out


def trajectory_pieces(seg, axis):
    """(start, end, x(u)) pieces covering all time, gaps included."""
    pieces = []
    start_pos = seg.start[:, axis] + seg.axes_r[:, axis] * seg.d0
    pieces.append((-np.inf, seg.t0[0], Polynomial([start_pos[0]])))
    for k in range(len(seg.t0)):
        r = seg.axes_r[k, axis]
        tau = Polynomial([-seg.t0[k], 1.0])
        x = start_pos[k] + r * (seg.v0[k] * tau + 0.5 * seg.accel[k] * tau**2)
        pieces.append((seg.t0[k], seg.end[k], x))
        gap_end = seg.t0[k + 1] if k + 1 < len(seg.t0) else np.inf
        if gap_end > seg.end[k]:
            pieces.append((seg.end[k], gap_end, Polynomial([x(seg.end[k])])))
    return pieces


def smoother_model(seg, kernel, axis, t):
    """Exact X(t) = integral w(s) x(t + t_offs - s) ds and derivatives."""
    pieces = trajectory_pieces(seg, axis)
    jump_times, jumps = seg.velocity_jumps()
    h, w = kernel.half_width, kernel.density
    out = np.zeros((3, len(t)))
    for i, ti in enumerate(t):
        T = ti + kernel.t_offs
        reflect = Polynomial([T, -1.0])
        for lo, hi, x in pieces:
            s_lo, s_hi = max(-h, T - hi), min(h, T - lo)
            if s_lo >= s_hi:
                continue
            for order, f in enumerate((x, x.deriv(1), x.deriv(2))):
                integral = (w * f(reflect)).integ()
                out[order, i] += integral(s_hi) - integral(s_lo)
        inside = np.abs(T - jump_times) < h
        out[2, i] += np.sum(jumps[inside, axis] * w(T - jump_times[inside]))
    return out


PULSE_SHAPERS = [s.name for s in shaper_defs.INPUT_SHAPERS]
SMOOTHERS = [s.name for s in shaper_defs.INPUT_SMOOTHERS]


@pytest.fixture(scope="module")
def path():
    rng = np.random.default_rng(20261106)
    moves = corner_path_moves(
        rng, (100.0, 100.0, 5.0), 6, ACCEL, CRUISE_V, dwell_after=2
    )
    return moves, Segments.from_moves(moves)


def grid_for(seg, kernel, interval=INTERVAL):
    lo, hi = kernel.support
    start = seg.t0[0] + lo - 0.01
    count = int((seg.end[-1] + hi + 0.01 - start) / interval) + 1
    return start, count


def jump_instants(seg, kernel):
    jump_times, _ = seg.velocity_jumps()
    if kernel.kind == "pulses":
        return (jump_times[:, None] - kernel.shifts[None, :]).ravel()
    edges = np.array([-kernel.half_width, 0.0, kernel.half_width])
    return (jump_times[:, None] - kernel.t_offs + edges[None, :]).ravel()


def tolerance(values):
    return ULPS * EPS * max(1.0, float(np.max(np.abs(values))))


@pytest.mark.parametrize("shaper_type", PULSE_SHAPERS + SMOOTHERS)
@pytest.mark.parametrize(
    "alloc,param",
    [("cartesian_stepper_alloc", b"x"), ("corexy_stepper_alloc", b"-")],
)
def test_samples_equal_shaped_trajectory(path, shaper_type, alloc, param):
    moves, seg = path
    state = shaper_state(shaper_type, SHAPER_FREQ)
    kernel = Kernel.from_shaper(state)
    sk, _ = shaped_stepper(alloc, param, {"x": state, "y": state})
    tq = build_trapq(moves)
    # The exact smoother reference integrates polynomials per sample in
    # numpy; a 1 ms grid still puts ~20 samples inside every kernel window.
    interval = INTERVAL if kernel.kind == "pulses" else 4 * INTERVAL
    start, count = grid_for(seg, kernel, interval)
    t = start + interval * np.arange(count)
    pos, vel, acc, row, offset = sample(sk, tq, start, interval, count)

    if kernel.kind == "pulses":
        model = pulse_model(seg, kernel, t)
        xyz = [model[order] for order in range(3)]
    else:
        xy = [smoother_model(seg, kernel, a, t) for a in (0, 1)]
        z = np.array(seg.eval(t))[:, :, 2]
        xyz = [
            np.stack([xy[0][order], xy[1][order], z[order]], axis=-1)
            for order in range(3)
        ]
    expected = [xyz[0] @ row + offset, xyz[1] @ row, xyz[2] @ row]
    smooth = ~isclose_to_any(t, jump_instants(seg, kernel), JUMP_GUARD_S)
    for name, got, want, mask in (
        ("position", pos, expected[0], np.ones(count, dtype=bool)),
        ("velocity", vel, expected[1], smooth),
        ("acceleration", acc, expected[2], smooth),
    ):
        err = np.abs(got - want)[mask]
        assert err.max() <= tolerance(want), (
            name,
            float(err.max()),
            float(t[mask][np.argmax(err)]),
        )


@pytest.mark.parametrize("shaper_type", ["mzv", "smooth_mzv"])
def test_velocity_integrates_to_position(path, shaper_type):
    moves, seg = path
    state = shaper_state(shaper_type, SHAPER_FREQ)
    kernel = Kernel.from_shaper(state)
    sk, _ = shaped_stepper("cartesian_stepper_alloc", b"x", {"x": state})
    start, count = grid_for(seg, kernel)
    interval = INTERVAL / 16
    pos, vel, acc, _, _ = sample(
        sk, build_trapq(moves), start, interval, count * 16
    )
    # Trapezoid rule: per step error <= dt^3/12 * |a'| away from jumps, and
    # at most dt * |dv| across a velocity jump.
    moved = np.diff(pos)
    trapezoid = 0.5 * (vel[1:] + vel[:-1]) * interval
    dv = np.abs(np.diff(vel))
    bound = dv * interval + np.abs(np.diff(acc)) * interval**2 + tolerance(pos)
    assert np.all(np.abs(moved - trapezoid) <= bound)


def test_rest_before_motion_holds_start_position(path):
    moves, seg = path
    sk, _ = shaped_stepper(
        "cartesian_stepper_alloc", b"x", {"x": shaper_state("mzv", SHAPER_FREQ)}
    )
    pos, vel, acc, _, _ = sample(
        sk, build_trapq(moves), seg.t0[0] - 3.0, INTERVAL, 100
    )
    assert np.all(pos == moves[0]["start_x"])
    assert np.all(vel == 0.0) and np.all(acc == 0.0)


def mzv_stepper():
    sk, _ = shaped_stepper(
        "cartesian_stepper_alloc", b"x", {"x": shaper_state("mzv", SHAPER_FREQ)}
    )
    ffi_main, ffi_lib = chelper.get_ffi()
    pre = ffi_lib.itersolve_get_gen_steps_pre_active(sk)
    post = ffi_lib.itersolve_get_gen_steps_post_active(sk)
    return sk, pre, post


def test_sampling_before_retained_moves_fails(path):
    moves, _ = path
    ffi_main, ffi_lib = chelper.get_ffi()
    sk, pre, _ = mzv_stepper()
    tq = build_trapq(moves)
    # Drop the leading null move and the first move, as motion_queuing does
    # once they are older than the step generation window.
    second = moves[1]["print_time"]
    ffi_lib.trapq_finalize_moves(tq, second, 0.0)
    t = second + 0.5 * pre
    ret, *_, err, _, _ = generate(sk, tq, t, INTERVAL, 1)
    assert ret == -2
    assert err.time == t


def test_window_reaching_finalized_motion_fails(path):
    moves, _ = path
    ffi_main, ffi_lib = chelper.get_ffi()
    sk, pre, post = mzv_stepper()
    tq = build_trapq(moves)
    stop = moves[2]
    gap_start = (
        stop["print_time"]
        + stop["accel_t"]
        + stop["cruise_t"]
        + stop["decel_t"]
    )
    gap_end = moves[3]["print_time"]
    # Finalizing up to the dwell leaves its stationary null move first in
    # the trapq, while the shaper window still reaches the motion before it.
    ffi_lib.trapq_finalize_moves(tq, gap_start, 0.0)
    ret, *_ = generate(sk, tq, gap_start + 0.5 * pre, INTERVAL, 1)
    assert ret == -2
    clear = gap_start + pre + INTERVAL
    assert clear + post < gap_end
    ret, pos, vel, acc, *_ = generate(sk, tq, clear, INTERVAL, 1)
    assert ret == 0
    assert pos[0] == pytest.approx(moves[3]["start_x"], abs=tolerance(pos))
    assert vel[0] == 0.0 and acc[0] == 0.0


def test_nonlinear_kinematics_rejected():
    ffi_main, ffi_lib = chelper.get_ffi()
    sk = ffi_main.gc(
        ffi_lib.delta_stepper_alloc(250.0**2, 0.0, 140.0), ffi_lib.free
    )
    with pytest.raises(NonlinearKinematics):
        probe_linear_map(sk)
