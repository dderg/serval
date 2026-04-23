"""Python wrapper around build_linear_as_quintic_coeffs C helper."""
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
