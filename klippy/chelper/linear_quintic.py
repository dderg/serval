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


MOVE_MAX_PIECES = 32
QUINTIC_SLOT_COEFFS = 15
QUINTIC_AXES = 4


def build_jerk_profile_as_quintic_coeffs(profile, axes_r, start_pos):
    """Translate a jerk_profile.Profile into the quintic-trapq slot layout.

    Parameters
    ----------
    profile : klippy.chelper.jerk_profile.Profile
        Result of jerk_profile.compute_profile(); must have status == JP_OK.
    axes_r : tuple of 3 floats
        Move direction ratios (rx, ry, rz). For a unit-norm vector |r| == 1.
    start_pos : tuple of 3 floats
        Start position (sx, sy, sz). Axis E (index 3) is always 0 here.

    Returns
    -------
    (n_phases, phase_t_ends, coeff_buf)
        n_phases: int in [1, 7].
        phase_t_ends: list of n_phases absolute (cumulative) phase end times.
        coeff_buf: list of MOVE_MAX_PIECES * QUINTIC_SLOT_COEFFS * QUINTIC_AXES
            doubles, ready to feed to trapq_append_quintic. Unused phases are
            zero-filled.

    Raises
    ------
    ValueError: if profile.status != JP_OK or axes_r / start_pos are wrong shape.
    """
    from klippy.chelper import jerk_profile as jp_mod
    if profile.status != jp_mod.JP_OK:
        raise ValueError(f"profile status {profile.status} is not JP_OK")
    if len(axes_r) != 3 or len(start_pos) != 3:
        raise ValueError("axes_r and start_pos must be 3-tuples")
    ffi, lib = get_ffi()
    result_c = ffi.new("struct jerk_profile_result *")
    result_c.status = profile.status
    result_c.n_segments = len(profile.segments)
    result_c.a_acc = profile.a_acc
    result_c.a_dec = profile.a_dec
    result_c.v_hat = profile.v_hat
    for i, seg in enumerate(profile.segments):
        type_int = {"J+": 1, "A+": 2, "J-": 3, "C": 4,
                    "J-d": 5, "A-": 6, "J+d": 7}.get(seg.type, 0)
        result_c.segments[i].type = type_int
        result_c.segments[i].T = seg.T
        for k in range(6):
            result_c.segments[i].coeffs[k] = seg.coeffs[k]
        result_c.segments[i].p0 = seg.p0
        result_c.segments[i].v0 = seg.v0
        result_c.segments[i].a0 = seg.a0
        result_c.segments[i].j = seg.j
    phase_t_ends = ffi.new(f"double[{MOVE_MAX_PIECES}]")
    coeff_buf = ffi.new(
        f"double[{MOVE_MAX_PIECES * QUINTIC_SLOT_COEFFS * QUINTIC_AXES}]")
    rx, ry, rz = axes_r
    sx, sy, sz = start_pos
    n_phases = lib.build_jerk_profile_as_quintic_coeffs(
        result_c, rx, ry, rz, sx, sy, sz, phase_t_ends, coeff_buf)
    if n_phases < 0:
        raise RuntimeError("build_jerk_profile_as_quintic_coeffs failed")
    return (n_phases,
            [phase_t_ends[i] for i in range(n_phases)],
            [coeff_buf[i] for i in
             range(MOVE_MAX_PIECES * QUINTIC_SLOT_COEFFS * QUINTIC_AXES)])
