"""
Reference implementation of 7-phase jerk-limited motion profile generator.
Used to numerically verify the derivation in plan9-phaseA1-derivation.md.

Convention:
  - Signed distance L: we handle the unsigned case and the caller flips sign.
  - v0, v1, v_peak, a_max, j_max >= 0, L > 0, v0 <= v_peak, v1 <= v_peak.
  - Output: list of segments. Each segment is a dict:
      { 'type': str, 'T': duration, 'coeffs': tuple of c0..ck (lowest -> highest) }
    position p(t) = c0 + c1*t + c2*t^2 + ... for t in [0, T].

We also return the phase diagnostic (t1..t7, a_acc, a_dec, v_hat).
"""
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

EPS = 1e-12


@dataclass
class Segment:
    type: str            # 'J+', 'A+', 'J-', 'C', 'J-d', 'A-', 'J+d'
    T: float
    coeffs: Tuple[float, ...]  # ascending order: c0, c1, c2, c3[, c4, c5]
    # Diagnostics at segment start:
    p0: float = 0.0
    v0: float = 0.0
    a0: float = 0.0
    j:  float = 0.0


@dataclass
class Profile:
    segments: List[Segment] = field(default_factory=list)
    a_acc: float = 0.0
    a_dec: float = 0.0
    v_hat: float = 0.0
    t_phase: Tuple[float, ...] = (0,)*7
    feasible: bool = True
    note: str = ''


# -----------------------------------------------------------------------------
# Helpers for one-side (accel or decel) triangle/trapezoid timings.
# -----------------------------------------------------------------------------
def accel_side_timings(v_start, v_end, a_max, j_max):
    """
    Compute (t_j, t_a, a_peak, dist) for a one-sided speed change from v_start
    to v_end with jerk limit j_max and accel limit a_max. Direction (accel or
    decel) is handled by taking |dv|: durations, peak-|a|, and the distance are
    direction-agnostic because the velocity profile mean is still (v_start+v_end)/2.
    Returns:
      t_j  : duration of each jerk phase
      t_a  : duration of the const-|a| phase (0 if triangular)
      a_p  : peak |a| reached (<= a_max)
      d    : distance covered during this group (>=0)
    """
    dv = abs(v_end - v_start)
    if dv < EPS:
        return 0.0, 0.0, 0.0, 0.0
    # Try trapezoidal accel profile (phase 2 non-zero): reach a_max.
    # Required dv to fill both jerk ramps at a_max: 2 * (0.5 * a_max * t_j) = a_max*t_j,
    # where t_j = a_max / j_max. So min dv for trapezoid is a_max^2 / j_max.
    dv_tri = a_max * a_max / j_max
    if dv >= dv_tri:
        # Trapezoidal
        t_j = a_max / j_max
        t_a = (dv - dv_tri) / a_max
        a_p = a_max
    else:
        # Triangular: peak a = sqrt(j_max * dv); t_j = a_p / j_max; t_a = 0.
        a_p = math.sqrt(j_max * dv)
        t_j = a_p / j_max
        t_a = 0.0
    # Distance: integrate velocity profile over this group.
    # v(t) during phase 1: v_start + 0.5*j_max*t^2
    # v(t) during phase 2: v_start + 0.5*a_p*t_j + a_p*(t - t_j)  -> simpler to use symmetry
    # Total duration T = 2*t_j + t_a. The velocity profile is symmetric about the mid.
    # Mean velocity = (v_start + v_end)/2. So distance = mean * T.
    T = 2.0 * t_j + t_a
    d = 0.5 * (v_start + v_end) * T
    return t_j, t_a, a_p, d


def accel_side_distance(v_start, v_end, a_max, j_max):
    """Distance for the accel (or decel) group from v_start to v_end."""
    _, _, _, d = accel_side_timings(v_start, v_end, a_max, j_max)
    return d


# -----------------------------------------------------------------------------
# Reduce v_peak when cruise collapses.
# -----------------------------------------------------------------------------
def find_v_hat(v0, v1, a_max, j_max, L):
    """
    Given that a full-peak-v accel + full-peak-v decel already overshoots L,
    find v_hat in [max(v0,v1), v_peak] such that
       d_acc(v0 -> v_hat) + d_dec(v_hat -> v1) == L
    d_acc and d_dec are each (v0+v_hat)/2 * T_acc and (v_hat+v1)/2 * T_dec,
    where T depends on whether the jerk triangle saturates or not.

    We use Newton-Raphson on f(v_hat) = d_acc + d_dec - L.
    Initial guess: midpoint. f is monotonic increasing in v_hat for v_hat > max(v0,v1).
    """
    v_lo = max(v0, v1)
    # Upper bracket: use v_peak-like quantity; caller ensures L too small for their cap.
    # Start v_hi by extrapolating: assume trapezoidal, v_hi ~ v_lo + 0.5 * a_max * sqrt(L/a_max).
    # Simpler: do bisection first to get a safe bracket, then Newton.
    # f(v_lo) = d(v0->v_lo) + d(v_lo->v1). If v_lo == max(v0,v1), the opposite side has d=0.
    def f(v):
        return accel_side_distance(v0, v, a_max, j_max) + accel_side_distance(v, v1, a_max, j_max) - L
    # f(v_lo) < 0 in our degeneracy branch (otherwise we wouldn't be here).
    # Find v_hi by doubling.
    v_hi = max(v_lo, 1.0)
    while f(v_hi) < 0:
        v_hi *= 2.0
        if v_hi > 1e9:
            raise RuntimeError("find_v_hat failed to bracket root")
    # Bisection to tight bracket, then Newton.
    for _ in range(80):
        vm = 0.5 * (v_lo + v_hi)
        if f(vm) < 0:
            v_lo = vm
        else:
            v_hi = vm
        if v_hi - v_lo < 1e-12 * max(1.0, v_hi):
            break
    return 0.5 * (v_lo + v_hi)


# -----------------------------------------------------------------------------
# Build position polynomial for each phase in segment-local time.
# Phase position polynomial has at most degree 3 (const jerk), degree 2 (const a),
# or degree 1 (cruise).
# -----------------------------------------------------------------------------
def poly_const_jerk(p0, v0, a0, j, T):
    """
    p(t) = p0 + v0*t + (1/2)*a0*t^2 + (1/6)*j*t^3
    return (c0, c1, c2, c3).
    """
    return (p0, v0, 0.5 * a0, j / 6.0)


def poly_const_accel(p0, v0, a0, T):
    """p(t) = p0 + v0*t + (1/2)*a0*t^2; degree 2."""
    return (p0, v0, 0.5 * a0)


def poly_linear(p0, v, T):
    """p(t) = p0 + v*t; degree 1."""
    return (p0, v)


def eval_poly(coeffs, t):
    """Horner."""
    s = 0.0
    for c in reversed(coeffs):
        s = s * t + c
    return s


def eval_poly_deriv(coeffs, t, order=1):
    """Evaluate derivative of order 'order' of poly at t."""
    # Differentiate symbolically then eval.
    c = list(coeffs)
    for _ in range(order):
        c = [k * c[k] for k in range(1, len(c))]
        if not c:
            return 0.0
    return eval_poly(tuple(c), t)


# -----------------------------------------------------------------------------
# Top-level profile generator.
# -----------------------------------------------------------------------------
def compute_profile(v0, v1, v_peak, a_max, j_max, L):
    """
    Generate the (up to) 7-phase jerk-limited profile for a 1D move from
    position 0 to position L with start velocity v0, end velocity v1.
    """
    prof = Profile()

    # Clamp inputs
    assert L > 0
    assert v0 >= 0 and v1 >= 0 and v_peak > 0 and a_max > 0 and j_max > 0
    assert v0 <= v_peak + 1e-9 and v1 <= v_peak + 1e-9

    # Feasibility floor: minimum distance to transition v0 -> v1 monotonically.
    # (Take v_hat = max(v0, v1); the slower side is a zero-length group.)
    d_floor = accel_side_distance(v0, max(v0, v1), a_max, j_max) + \
              accel_side_distance(max(v0, v1), v1, a_max, j_max)
    if L + 1e-12 < d_floor:
        prof.feasible = False
        prof.note = (f"infeasible: L={L} below min distance d_floor={d_floor} "
                     f"to transition v0={v0}->v1={v1} within (a_max,j_max).")
        return prof

    # Step 1: compute accel + decel distances at full v_peak.
    tj_a, ta_a, a_acc, d_acc = accel_side_timings(v0, v_peak, a_max, j_max)
    tj_d, ta_d, a_dec, d_dec = accel_side_timings(v1, v_peak, a_max, j_max)  # symmetric

    if d_acc + d_dec <= L + EPS:
        v_hat = v_peak
        d_cruise = L - d_acc - d_dec
        t_cruise = d_cruise / v_hat if v_hat > EPS else 0.0
    else:
        # Cruise collapses; find v_hat.
        v_hat = find_v_hat(v0, v1, a_max, j_max, L)
        tj_a, ta_a, a_acc, d_acc = accel_side_timings(v0, v_hat, a_max, j_max)
        tj_d, ta_d, a_dec, d_dec = accel_side_timings(v1, v_hat, a_max, j_max)
        t_cruise = 0.0
        d_cruise = 0.0

    prof.a_acc = a_acc
    prof.a_dec = a_dec
    prof.v_hat = v_hat
    prof.t_phase = (tj_a, ta_a, tj_a, t_cruise, tj_d, ta_d, tj_d)

    # Step 2: build segments.
    p = 0.0
    v = v0
    a = 0.0

    def emit_jerk_phase(tag, j_sign_times_jmax, T):
        nonlocal p, v, a
        if T <= EPS:
            return
        j = j_sign_times_jmax
        coeffs = poly_const_jerk(p, v, a, j, T)
        seg = Segment(tag, T, coeffs, p, v, a, j)
        prof.segments.append(seg)
        # Advance state to end of segment.
        p = eval_poly(coeffs, T)
        v = eval_poly_deriv(coeffs, T, 1)
        a = eval_poly_deriv(coeffs, T, 2)

    def emit_const_accel_phase(tag, T):
        nonlocal p, v, a
        if T <= EPS:
            return
        coeffs = poly_const_accel(p, v, a, T)
        seg = Segment(tag, T, coeffs, p, v, a, 0.0)
        prof.segments.append(seg)
        p = eval_poly(coeffs, T)
        v = eval_poly_deriv(coeffs, T, 1)
        # a unchanged

    def emit_cruise(T):
        nonlocal p, v, a
        if T <= EPS:
            return
        coeffs = poly_linear(p, v, T)
        seg = Segment('C', T, coeffs, p, v, a, 0.0)
        prof.segments.append(seg)
        p = p + v * T
        # v, a unchanged (a should be ~0)

    # Accel group
    emit_jerk_phase('J+',  +j_max, tj_a)
    emit_const_accel_phase('A+', ta_a)
    emit_jerk_phase('J-',  -j_max, tj_a)
    # Cruise
    emit_cruise(t_cruise)
    # Decel group
    emit_jerk_phase('J-d', -j_max, tj_d)
    emit_const_accel_phase('A-', ta_d)
    emit_jerk_phase('J+d', +j_max, tj_d)

    return prof


# -----------------------------------------------------------------------------
# Verification.
# -----------------------------------------------------------------------------
def verify_profile(prof: Profile, v0, v1, v_peak, a_max, j_max, L, tol=1e-8):
    """
    Check:
      - durations >= 0
      - C2 continuity across boundaries
      - total distance matches L
      - peak accel <= a_max (within tol)
      - peak jerk <= j_max
      - start velocity == v0, end velocity == v1
    Returns (ok, messages).
    """
    msgs = []
    ok = True
    if not prof.feasible:
        return False, ["infeasible"]

    # durations
    for i, seg in enumerate(prof.segments):
        if seg.T < -tol:
            ok = False
            msgs.append(f"neg duration seg{i} {seg.type} T={seg.T}")

    # Walk and check C2.
    p = 0.0
    v = v0
    a = 0.0
    for i, seg in enumerate(prof.segments):
        # start checks
        p_start = seg.coeffs[0]
        v_start = seg.coeffs[1] if len(seg.coeffs) > 1 else 0.0
        a_start = 2.0 * seg.coeffs[2] if len(seg.coeffs) > 2 else 0.0
        rel = max(1.0, abs(p), abs(v), abs(a))
        if abs(p_start - p) > tol * rel:
            ok = False
            msgs.append(f"C0 break at seg{i} {seg.type}: p_start={p_start} expected {p}")
        if abs(v_start - v) > tol * rel:
            ok = False
            msgs.append(f"C1 break at seg{i} {seg.type}: v_start={v_start} expected {v}")
        if abs(a_start - a) > tol * rel:
            ok = False
            msgs.append(f"C2 break at seg{i} {seg.type}: a_start={a_start} expected {a}")

        # End state
        T = seg.T
        p = eval_poly(seg.coeffs, T)
        v = eval_poly_deriv(seg.coeffs, T, 1)
        a = eval_poly_deriv(seg.coeffs, T, 2)

        # Peak check: peak accel in a jerk segment is at one of the endpoints (linear a(t)).
        a_end = a
        a_peak = max(abs(a_start), abs(a_end))
        if a_peak > a_max + 1e-6 * a_max:
            ok = False
            msgs.append(f"a exceeds a_max in seg{i}: {a_peak} > {a_max}")
        # jerk peak check
        j_peak = abs(seg.j) if seg.type.startswith('J') else 0.0
        if j_peak > j_max + 1e-6 * j_max:
            ok = False
            msgs.append(f"j exceeds j_max in seg{i}: {j_peak} > {j_max}")

    # Total distance
    if abs(p - L) > 1e-8 * max(1.0, L):
        ok = False
        msgs.append(f"distance mismatch: p_end={p} L={L}")
    if abs(v - v1) > 1e-8 * max(1.0, v1):
        ok = False
        msgs.append(f"v_end mismatch: v={v} v1={v1}")
    if abs(a) > 1e-6:
        ok = False
        msgs.append(f"a_end nonzero: a={a}")
    return ok, msgs


# -----------------------------------------------------------------------------
# Test sweep.
# -----------------------------------------------------------------------------
def sweep():
    import itertools, collections
    v_peak = 500.0
    a_max = 5000.0
    j_max = 100000.0
    cases = list(itertools.product([0, 50, 200], [0, 50, 200], [1, 10, 100, 1000]))
    stats = collections.Counter()
    worst = []
    infeasible = []
    for v0, v1, L in cases:
        prof = compute_profile(float(v0), float(v1), v_peak, a_max, j_max, float(L))
        if not prof.feasible:
            infeasible.append((v0, v1, L, prof.note))
            stats['INFEASIBLE'] += 1
            continue
        ok, msgs = verify_profile(prof, v0, v1, v_peak, a_max, j_max, L)
        tj_a, ta_a, _, t_cruise, tj_d, ta_d, _ = prof.t_phase
        # classify
        labels = []
        if t_cruise <= EPS: labels.append('cruise-collapse')
        if ta_a <= EPS: labels.append('A+collapse')
        if ta_d <= EPS: labels.append('A-collapse')
        if tj_a <= EPS and ta_a <= EPS: labels.append('no-accel')
        if tj_d <= EPS and ta_d <= EPS: labels.append('no-decel')
        if not labels: labels.append('full-7-phase')
        key = '|'.join(labels)
        stats[key] += 1
        if not ok:
            worst.append((v0, v1, L, msgs))
        # record distance error + peak values
    print("=== sweep stats ===")
    for k, n in stats.most_common():
        print(f"  {k}: {n}")
    if worst:
        print("=== FAILURES ===")
        for w in worst:
            print(w)
    else:
        print(f"All {sum(stats.values()) - stats['INFEASIBLE']} feasible cases pass verification (C2, distance, bounds).")
    if infeasible:
        print("=== Infeasible (expected: L too small to bridge v0->v1) ===")
        for w in infeasible:
            print(w)

    # Precision probe: report max position-error in any feasible case at t=T_total.
    max_abs = 0.0
    max_rel = 0.0
    for v0, v1, L in cases:
        prof = compute_profile(float(v0), float(v1), v_peak, a_max, j_max, float(L))
        if not prof.feasible: continue
        p_end = eval_poly(prof.segments[-1].coeffs, prof.segments[-1].T) if prof.segments else 0.0
        err = abs(p_end - L)
        max_abs = max(max_abs, err)
        max_rel = max(max_rel, err / max(1.0, L))
    print(f"\nPrecision probe: max |p_end - L| = {max_abs:.3e}, max relative = {max_rel:.3e}")

    # Detail table
    print()
    print(f"{'v0':>5}{'v1':>5}{'L':>6}   {'v_hat':>8}  {'a_acc':>8}  {'a_dec':>8}  {'phases':>40}")
    for v0, v1, L in cases:
        prof = compute_profile(float(v0), float(v1), v_peak, a_max, j_max, float(L))
        ps = ','.join(f"{seg.type}:{seg.T:.4g}" for seg in prof.segments)
        print(f"{v0:>5}{v1:>5}{L:>6}   {prof.v_hat:8.2f}  {prof.a_acc:8.2f}  {prof.a_dec:8.2f}  {ps}")


if __name__ == "__main__":
    sweep()
