"""Python wrapper around klippy/chelper/jerk_profile.c.

Plan 9 Phase A1: jerk-limited polynomial profile generator.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from klippy.chelper import get_ffi

JP_MAX_SEGMENTS = 7
JP_MAX_COEFFS = 6

# Match enum jerk_profile_seg_type in jerk_profile.h.
SEG_TYPE_NAMES = {
    1: "J+",
    2: "A+",
    3: "J-",
    4: "C",
    5: "J-d",
    6: "A-",
    7: "J+d",
}

JP_OK = 0
JP_INFEASIBLE = 1
JP_BAD_INPUT = 2


@dataclass
class Segment:
    type: str
    T: float
    coeffs: List[float] = field(default_factory=list)
    p0: float = 0.0
    v0: float = 0.0
    a0: float = 0.0
    j: float = 0.0


@dataclass
class Profile:
    status: int
    segments: List[Segment] = field(default_factory=list)
    a_acc: float = 0.0
    a_dec: float = 0.0
    v_hat: float = 0.0


def accel_side_timings(v_start: float, v_end: float, a_max: float, j_max: float):
    """Call the C accel_side_timings primitive. Returns (t_j, t_a, a_peak, dist)."""
    ffi, lib = get_ffi()
    out_t_j = ffi.new("double[1]")
    out_t_a = ffi.new("double[1]")
    out_a_peak = ffi.new("double[1]")
    out_dist = ffi.new("double[1]")
    lib.jerk_profile_accel_side_timings(
        v_start, v_end, a_max, j_max,
        out_t_j, out_t_a, out_a_peak, out_dist)
    return out_t_j[0], out_t_a[0], out_a_peak[0], out_dist[0]


def find_v_hat(v0: float, v1: float, v_peak: float,
               a_max: float, j_max: float, L: float) -> float:
    """Call the C find_v_hat Newton-Raphson for reduced peak velocity."""
    _, lib = get_ffi()
    return lib.jerk_profile_find_v_hat(v0, v1, v_peak, a_max, j_max, L)


def compute_profile(v0: float, v1: float, v_peak: float,
                    a_max: float, j_max: float, L: float) -> Profile:
    """Compute the full jerk-limited profile. Returns a Profile dataclass."""
    ffi, lib = get_ffi()
    result = ffi.new("struct jerk_profile_result *")
    status = lib.jerk_profile_compute(v0, v1, v_peak, a_max, j_max, L, result)
    prof = Profile(status=status,
                   a_acc=result.a_acc,
                   a_dec=result.a_dec,
                   v_hat=result.v_hat)
    for i in range(result.n_segments):
        c_seg = result.segments[i]
        seg = Segment(
            type=SEG_TYPE_NAMES.get(c_seg.type, "?"),
            T=c_seg.T,
            coeffs=[c_seg.coeffs[k] for k in range(JP_MAX_COEFFS)],
            p0=c_seg.p0, v0=c_seg.v0, a0=c_seg.a0, j=c_seg.j,
        )
        prof.segments.append(seg)
    return prof
