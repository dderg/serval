"""Python wrappers around the linear→degenerate-quintic helpers."""
import math
from klippy.chelper import get_ffi


def append_trapezoid_e_only_as_quintic(
    tq, print_time, accel_t, cruise_t, decel_t,
    e_start,
    start_v, cruise_v, accel,
):
    """Emit an extruder-only trapezoid onto `tq` carrying the filament
    position polynomial in the .e slot of a 3-phase degenerate quintic.

    Plan 8 Chunk 3 Task 8: the extruder stepper reads .e directly via
    move_get_coord(m, t).e (no convolution, no smoother). For moves
    WITHOUT a corresponding XY blend payload (pure-E retracts, hops,
    idle extrusions), this helper provides the straight filament-
    trapezoid emit path with .e filled and x/y/z = 0.

    Signature mirrors the accel/cruise/decel pattern of
    append_trapezoid_as_quintic so callers can swap between them by
    trivial keyword adjustment.
    """
    ffi, lib = get_ffi()
    n_phases = 3
    stride = 15 * 4
    buf = ffi.new(f"double[{n_phases * stride}]")
    # Phase 0: accel, E(tau) = e_start + start_v*tau + 0.5*accel*tau^2
    buf[0 * stride + 0 * 4 + 3] = e_start
    buf[0 * stride + 1 * 4 + 3] = start_v
    buf[0 * stride + 2 * 4 + 3] = 0.5 * accel
    # Phase 1: cruise, starting at E after accel.
    accel_d = start_v * accel_t + 0.5 * accel * accel_t * accel_t
    cruise_e_start = e_start + accel_d
    buf[1 * stride + 0 * 4 + 3] = cruise_e_start
    buf[1 * stride + 1 * 4 + 3] = cruise_v
    # c[2] = 0 (cruise has no accel), already zero from ffi.new.
    # Phase 2: decel, starting after cruise.
    cruise_d = cruise_v * cruise_t
    decel_e_start = cruise_e_start + cruise_d
    buf[2 * stride + 0 * 4 + 3] = decel_e_start
    buf[2 * stride + 1 * 4 + 3] = cruise_v
    buf[2 * stride + 2 * 4 + 3] = -0.5 * accel
    move_t = accel_t + cruise_t + decel_t
    decel_d = cruise_v * decel_t - 0.5 * accel * decel_t * decel_t
    arc_length = abs(accel_d + cruise_d + decel_d)
    decel_end_v = cruise_v - accel * decel_t
    v_cap_min = min(start_v, cruise_v, decel_end_v)
    if v_cap_min < 0.0:
        v_cap_min = 0.0
    phase_t_ends = ffi.new("double[3]", [
        accel_t,
        accel_t + cruise_t,
        move_t,
    ])
    lib.trapq_append_quintic(
        tq, print_time,
        n_phases, phase_t_ends,
        move_t, arc_length, v_cap_min,
        1,  # shape_disabled: pure-E has no XY shaper to inherit
        e_start, 0.0, 0.0,
        buf,
    )


def linear_as_quintic_coeffs(
    accel_t, cruise_t, decel_t,
    start_v, cruise_v, accel,
    axes_r, start_pos,
):
    """Return a 180-double list representing a linear accel/cruise/decel
    motion as a degenerate quintic coefficient buffer (3 phases × 15 × 4).

    axes_r, start_pos: 3-tuples (x, y, z). The .e slot of every coefficient
    is left zero; the linear-PA composer populates it at plan time.
    """
    ffi, lib = get_ffi()
    buf = ffi.new("double[180]")
    lib.build_linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r[0], axes_r[1], axes_r[2],
        start_pos[0], start_pos[1], start_pos[2],
        buf,
    )
    return [buf[i] for i in range(180)]


def append_trapezoid_as_quintic(
    tq, print_time, accel_t, cruise_t, decel_t,
    start_pos_x, start_pos_y, start_pos_z,
    axes_r_x, axes_r_y, axes_r_z,
    start_v, cruise_v, accel,
    shape_disabled=False,
):
    """Emit an accel/cruise/decel trapezoid onto trapq as a single degenerate-
    quintic move. Signature mirrors the legacy trapq_append FFI call so
    callers migrate with no parameter reshuffling.

    ``shape_disabled`` (kwarg, default False) stamps the emitted move with
    the Plan 8 shape-disabled flag. Must-be-unshaped emit sites (force_move,
    manual_stepper, drip-homed moves, pure-E) pass ``True`` so the
    planner's shaper-bake step skips baking (Chunk 2 Task 11 threading).

    Plan 8 Chunk 3: the underlying coeff_buf is now 4-axis (x, y, z, e);
    .e is left zero by the C-side builder. Pure-E emit goes via
    append_extruder_only_as_quintic which fills .e directly.
    """
    ffi, lib = get_ffi()
    buf = ffi.new("double[180]")
    lib.build_linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r_x, axes_r_y, axes_r_z,
        start_pos_x, start_pos_y, start_pos_z,
        buf,
    )
    move_t = accel_t + cruise_t + decel_t
    accel_d = start_v * accel_t + 0.5 * accel * accel_t * accel_t
    cruise_d = cruise_v * cruise_t
    decel_d = cruise_v * decel_t - 0.5 * accel * decel_t * decel_t
    axes_r_mag = math.sqrt(
        axes_r_x * axes_r_x + axes_r_y * axes_r_y + axes_r_z * axes_r_z)
    arc_length = (accel_d + cruise_d + decel_d) * axes_r_mag
    decel_end_v = cruise_v - accel * decel_t
    v_cap_min = min(start_v, cruise_v, decel_end_v)
    if v_cap_min < 0.0:
        v_cap_min = 0.0
    # 3-phase layout: t_ends = [accel_t, accel_t+cruise_t, move_t].
    phase_t_ends = ffi.new("double[3]", [
        accel_t,
        accel_t + cruise_t,
        move_t,
    ])
    lib.trapq_append_quintic(
        tq, print_time,
        3, phase_t_ends,
        move_t, arc_length, v_cap_min,
        1 if shape_disabled else 0,
        start_pos_x, start_pos_y, start_pos_z,
        buf,
    )
