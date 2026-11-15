"""The C max_corner_accel bound (chelper/corner_deviation.c) and its
Python reference (corner_bound_ref.py) plan the same junction speeds over
random junction sequences, smoothers and kinematic rows."""

import math
import random

import pytest

from klippy.extras import corner_deviation as cd
from klippy.extras import shaper_defs
from tools.motion_eval import corner_bound_ref, fuzz

JUNCTIONS = 400
RELATIVE_TOLERANCE = 1e-9
SMOOTHERS = (
    (shaper_defs.get_mzv_smoother, 112.8, 90.2),
    (shaper_defs.get_ei_smoother, 200.0, 130.0),
    (shaper_defs.get_zv_smoother, 70.0, 70.0),
)
CAPS = {
    "axes": cd.CornerCaps(30000.0, None, cd.COREXY_ROWS),
    "corexy": cd.CornerCaps(None, 30000.0, cd.COREXY_ROWS),
    "both": cd.CornerCaps(30000.0, 45000.0, cd.COREXY_ROWS),
}


def _kernels(smoother, fx, fy):
    specs = tuple(
        ("smoother",) + tuple(smoother(f, normalize_coeffs=False))
        for f in (fx, fy)
    )
    kx, ky = (cd.kernel_from_spec(s) for s in specs)
    return specs, kx, ky


def _direction(rng, heading):
    z = rng.choice((0.0, 0.0, 0.0, rng.uniform(-0.3, 0.3)))
    xy = math.sqrt(1.0 - z * z)
    return (xy * math.cos(heading), xy * math.sin(heading))


@pytest.mark.parametrize("setting", sorted(CAPS))
@pytest.mark.parametrize(
    "smoother", SMOOTHERS, ids=lambda s: "%s-%g" % (s[0].__name__, s[1])
)
def test_c_bound_matches_the_reference(smoother, setting):
    specs, kx, ky = _kernels(*smoother)
    arrays = cd.table_arrays(specs, kx, ky, fuzz.TABLE_CACHE_DIR)
    cap, rows = CAPS[setting].limiter()
    bounded = cd.JunctionLimit(arrays, kx, ky, cap, rows)
    free = cd.JunctionLimit(arrays, kx, ky, None, rows)
    reference = corner_bound_ref.ReferenceLimit(kx, ky, cap, rows)
    rng = random.Random("%g-%s" % (smoother[1], setting))
    heading = rng.uniform(-math.pi, math.pi)
    up = _direction(rng, heading)
    prev_d, prev_accel, prev_max_v2, prev_start_v2 = 1.0, 20000.0, 9e4, 0.0
    compared = 0
    for _ in range(JUNCTIONS):
        heading += math.radians(
            rng.choice((rng.uniform(2.0, 30.0), rng.uniform(30.0, 179.0)))
            * rng.choice((-1.0, 1.0))
        )
        um = _direction(rng, heading)
        move_d = rng.choice((rng.uniform(0.05, 2.0), rng.uniform(2.0, 60.0)))
        move_accel = rng.choice((5000.0, 12000.0, 20000.0, 40000.0))
        move_max_v2 = rng.uniform(50.0, 600.0) ** 2
        cap_v2 = min(
            move_max_v2,
            prev_max_v2,
            prev_start_v2 + 2.0 * prev_accel * prev_d,
        )
        args = (
            up[0],
            up[1],
            um[0],
            um[1],
            prev_d,
            move_d,
            prev_accel,
            move_accel,
            prev_max_v2,
            move_max_v2,
            prev_start_v2,
            cap_v2,
        )
        table_v2 = free.junction_v2(free.c, *args)
        c_v2 = bounded.junction_v2(bounded.c, *args)
        ref_v2 = reference.junction_v2(
            up,
            um,
            prev_d,
            move_d,
            prev_accel,
            move_accel,
            prev_max_v2,
            move_max_v2,
            prev_start_v2,
            cap_v2,
            math.sqrt(table_v2),
        )
        assert c_v2 == pytest.approx(ref_v2, rel=RELATIVE_TOLERANCE, abs=1e-9)
        compared += c_v2 < table_v2
        up, prev_d, prev_accel = um, move_d, move_accel
        prev_max_v2, prev_start_v2 = move_max_v2, c_v2
    assert compared > JUNCTIONS // 10
