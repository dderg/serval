"""Python wrappers around the linear→degenerate-quintic helpers."""
import math
from klippy.chelper import get_ffi


def linear_as_quintic_coeffs(
    accel_t, cruise_t, decel_t,
    start_v, cruise_v, accel,
    axes_r, start_pos,
):
    """Return a 99-double list representing a linear accel/cruise/decel
    motion as a degenerate quintic coefficient buffer.

    axes_r, start_pos: 3-tuples (x, y, z)."""
    ffi, lib = get_ffi()
    buf = ffi.new("double[99]")
    lib.build_linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r[0], axes_r[1], axes_r[2],
        start_pos[0], start_pos[1], start_pos[2],
        buf,
    )
    return [buf[i] for i in range(99)]


def append_trapezoid_as_quintic(
    tq, print_time, accel_t, cruise_t, decel_t,
    start_pos_x, start_pos_y, start_pos_z,
    axes_r_x, axes_r_y, axes_r_z,
    start_v, cruise_v, accel,
):
    """Emit an accel/cruise/decel trapezoid onto trapq as a single degenerate-
    quintic move. Signature mirrors the legacy trapq_append FFI call so
    callers migrate with no parameter reshuffling."""
    ffi, lib = get_ffi()
    buf = ffi.new("double[99]")
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
    lib.trapq_append_quintic(
        tq, print_time,
        accel_t,                    # t_accel_end
        accel_t + cruise_t,         # t_decel_start
        move_t, arc_length, v_cap_min,
        start_pos_x, start_pos_y, start_pos_z,
        buf,
    )
