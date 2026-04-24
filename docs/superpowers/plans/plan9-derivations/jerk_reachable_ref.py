"""
Reference implementation of the jerk-aware reachable-velocity inverse
(Phase A2b of the Plan 9 / magnum-opus motion-pipeline rewrite).

Problem:
  Given v_start >= 0, a_max > 0, j_max > 0, L > 0, find v_end > v_start such
  that an accel-side jerk-limited profile from v_start to v_end covers
  EXACTLY distance L under those limits.

This is the inverse of accel_side_distance() from jerk_profile_ref.py.

We split on two regimes:
  - Regime A (triangular): peak acceleration stays below a_max.
  - Regime B (trapezoidal): peak acceleration saturates at a_max.

Regime B is a pure quadratic in dv = v_end - v_start.  Regime A is a cubic in
a_peak (equivalently sqrt(dv)); we solve it either in closed form (depressed
cubic) or via a couple of Newton iterations from a good seed.

See 2026-04-24-plan9-phaseA2b-derivation.md for the derivation.
"""
import math
import sys
import os

# Allow the companion module to be imported regardless of cwd.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from jerk_profile_ref import accel_side_distance  # noqa: E402


# -----------------------------------------------------------------------------
# Regime boundary and forward helpers
# -----------------------------------------------------------------------------
def regime_boundary_dv(a_max, j_max):
    """dv at which the triangular jerk ramp just saturates a_max."""
    return a_max * a_max / j_max


def regime_boundary_distance(v_start, a_max, j_max):
    """
    Distance of the accel-side group AT the regime boundary.  Using
      dv_b = a_max^2 / j_max,  T_b = 2 * a_max / j_max,  v_end = v_start + dv_b,
    so L_b = 0.5*(v_start + v_end)*T_b = (2*v_start + dv_b) * (a_max/j_max).
    """
    dv_b = regime_boundary_dv(a_max, j_max)
    return (2.0 * v_start + dv_b) * (a_max / j_max)


# -----------------------------------------------------------------------------
# Regime B: trapezoidal-acceleration closed form.
# -----------------------------------------------------------------------------
def _reachable_v_end_trap(v_start, a_max, j_max, L):
    """
    Trapezoidal regime solver.  From the derivation:

        2*L = (v_end^2 - v_start^2)/a_max + a_max*(v_end + v_start)/j_max

    Let dv = v_end - v_start.  Let C = a_max^2 / j_max.  Then

        dv^2 + dv*(2*v_start + C) + (2*v_start*C - 2*L*a_max) = 0.

    Pick the positive root.
    """
    C = a_max * a_max / j_max
    b = 2.0 * v_start + C
    c = 2.0 * v_start * C - 2.0 * L * a_max
    disc = b * b - 4.0 * c
    # disc is always >= 0 in this regime for L > 0 and v_start >= 0:
    # disc = (2*v_start + C)^2 - 4*(2*v_start*C - 2*L*a_max)
    #      = (2*v_start - C)^2 + 8*L*a_max   >= 0.
    if disc < 0.0:
        # Numerical safety net.
        disc = 0.0
    dv = 0.5 * (-b + math.sqrt(disc))
    return v_start + dv


# -----------------------------------------------------------------------------
# Regime A: triangular-acceleration closed form.
# -----------------------------------------------------------------------------
def _reachable_v_end_tri(v_start, a_max, j_max, L):
    """
    Triangular regime solver.  From the derivation:

        L = (2*v_start + dv) * sqrt(dv / j_max)

    Substitute u = sqrt(dv).  Then dv = u^2 and

        L * sqrt(j_max) = (2*v_start + u^2) * u  =  u^3 + 2*v_start * u.

    So u satisfies the depressed cubic

        u^3 + 2*v_start*u - L*sqrt(j_max) = 0.        (*)

    For v_start >= 0 the coefficients are p = 2*v_start >= 0 and q = -L*sqrt(j_max) <= 0.
    Cardano's discriminant D = (q/2)^2 + (p/3)^3 >= 0 always, so (*) has exactly
    one real root given by

        u = cbrt(-q/2 + sqrt(D)) + cbrt(-q/2 - sqrt(D)).

    Both cbrt arguments are real; the second may be negative when D > (q/2)^2
    which happens for v_start > 0.  Use signed cube-root.
    """
    if v_start <= 0.0:
        # Pure triangular from rest: L = u^3 / sqrt(j_max)  ->  u = (L*sqrt(j))**(1/3)
        u = (L * math.sqrt(j_max)) ** (1.0 / 3.0)
        dv = u * u
        return v_start + dv

    p = 2.0 * v_start
    q = -L * math.sqrt(j_max)
    half_q = 0.5 * q
    D = half_q * half_q + (p / 3.0) ** 3
    sqrt_D = math.sqrt(D)
    t1 = -half_q + sqrt_D
    t2 = -half_q - sqrt_D

    def cbrt(x):
        return math.copysign(abs(x) ** (1.0 / 3.0), x)

    u = cbrt(t1) + cbrt(t2)
    dv = u * u
    return v_start + dv


# -----------------------------------------------------------------------------
# Public API.
# -----------------------------------------------------------------------------
def reachable_v_end(v_start, a_max, j_max, L):
    """
    Return the largest v_end such that a jerk-limited accel-side group from
    v_start to v_end covers distance L under (a_max, j_max).

    v_start >= 0, a_max > 0, j_max > 0, L >= 0.
    Returns v_end >= v_start.
    """
    if not (math.isfinite(v_start) and math.isfinite(a_max)
            and math.isfinite(j_max) and math.isfinite(L)):
        return float('nan')
    if a_max <= 0.0 or j_max <= 0.0:
        return float('nan')
    if v_start < 0.0 or L < 0.0:
        return float('nan')
    if L == 0.0:
        return v_start

    L_boundary = regime_boundary_distance(v_start, a_max, j_max)
    if L <= L_boundary:
        return _reachable_v_end_tri(v_start, a_max, j_max, L)
    return _reachable_v_end_trap(v_start, a_max, j_max, L)


def reachable_v_end_diag(v_start, a_max, j_max, L):
    """
    Diagnostic variant: returns (v_end, regime) where regime is 'triangular'
    or 'trapezoidal' or 'degenerate' (L == 0).
    """
    v = reachable_v_end(v_start, a_max, j_max, L)
    if L == 0.0:
        return v, 'degenerate'
    L_boundary = regime_boundary_distance(v_start, a_max, j_max)
    return v, ('triangular' if L <= L_boundary else 'trapezoidal')


# -----------------------------------------------------------------------------
# Verification.
# -----------------------------------------------------------------------------
def verify_reachable_v_end(v_start, a_max, j_max, L, rtol=1e-9, atol=1e-9):
    """
    Returns (ok, v_end, L_check, abs_err, rel_err, regime).  ok is True iff
    |accel_side_distance(v_start, v_end, a_max, j_max) - L| <= atol + rtol*|L|.
    """
    v_end, regime = reachable_v_end_diag(v_start, a_max, j_max, L)
    L_check = accel_side_distance(v_start, v_end, a_max, j_max)
    abs_err = abs(L_check - L)
    rel_err = abs_err / max(1.0, abs(L))
    ok = abs_err <= (atol + rtol * max(1.0, abs(L)))
    return ok, v_end, L_check, abs_err, rel_err, regime


def sweep():
    import itertools, collections
    v_starts = [0.0, 50.0, 200.0, 500.0]
    a_maxes = [2500.0, 5000.0, 10000.0]
    j_maxes = [50000.0, 100000.0, 500000.0]
    Ls = [0.1, 1.0, 10.0, 100.0, 1000.0]

    cases = list(itertools.product(v_starts, a_maxes, j_maxes, Ls))
    fail = []
    regime_count = collections.Counter()
    max_abs = 0.0
    max_rel = 0.0
    per_regime_max_rel = collections.defaultdict(float)

    for v0, a, j, L in cases:
        ok, v_end, L_chk, ae, re_, reg = verify_reachable_v_end(v0, a, j, L)
        regime_count[reg] += 1
        max_abs = max(max_abs, ae)
        max_rel = max(max_rel, re_)
        per_regime_max_rel[reg] = max(per_regime_max_rel[reg], re_)
        if not ok:
            fail.append((v0, a, j, L, v_end, L_chk, ae, re_, reg))

    print("=== phase A2b reachable_v_end sweep ===")
    print(f"total cases: {len(cases)}")
    for reg, n in regime_count.most_common():
        print(f"  regime {reg}: {n} ({100.0 * n / len(cases):.1f}%) "
              f"max_rel_err={per_regime_max_rel[reg]:.2e}")
    print(f"max abs err: {max_abs:.3e}")
    print(f"max rel err: {max_rel:.3e}")
    if fail:
        print(f"FAIL: {len(fail)} cases")
        for f in fail[:10]:
            print(" ", f)
    else:
        print("OK: all cases pass at rtol=1e-9.")

    # A couple of explicit sanity checks against the constant-accel limit.
    # When j_max -> infty the trapezoidal regime recovers v_end = sqrt(v0^2 + 2*L*a).
    print()
    print("=== classical-limit check (j_max very large) ===")
    for v0, a, L in [(0.0, 5000.0, 10.0), (100.0, 5000.0, 50.0), (300.0, 10000.0, 200.0)]:
        j = 1e12  # effectively infinite jerk
        v_end = reachable_v_end(v0, a, j, L)
        v_classical = math.sqrt(v0 * v0 + 2.0 * L * a)
        print(f"  v0={v0:7.1f} a={a:7.0f} L={L:7.1f}  "
              f"jerk-ware v_end={v_end:.6f}  classical={v_classical:.6f}  "
              f"diff={v_end - v_classical:.3e}")

    # Monotonicity check: v_end must be strictly increasing in L.
    print()
    print("=== monotonicity check ===")
    mono_fail = 0
    for v0 in v_starts:
        for a in a_maxes:
            for j in j_maxes:
                prev = -1.0
                for L in sorted(Ls):
                    v_end = reachable_v_end(v0, a, j, L)
                    if v_end < prev - 1e-9:
                        mono_fail += 1
                    prev = v_end
    print(f"monotonicity violations: {mono_fail}")


if __name__ == "__main__":
    sweep()
