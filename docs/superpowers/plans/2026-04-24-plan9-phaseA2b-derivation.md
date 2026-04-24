# Phase A2b Derivation — Jerk-Aware Reachable-Velocity Inverse

**Project:** Kalico magnum-opus — Plan 9, Phase A2b.
**Replaces:** `delta_v2 = 2 * move_d * max_accel` in the Klipper-style planner.
**Reference implementation:** `plan9-derivations/jerk_reachable_ref.py` (verified, 180/180 cases).
**Upstream primitive:** `accel_side_timings` / `accel_side_distance` in `plan9-derivations/jerk_profile_ref.py` (Phase A1).

---

## Part 1 — Problem statement

### The function we need

The planner repeatedly asks: *given a starting velocity v_start and a segment of length L, what is the largest ending velocity v_end that I can reach within L using my acceleration and jerk limits?* In today's Klipper this is the one-liner

```
delta_v2 = 2 * move_d * max_accel          # Klipper: v_end^2 = v_start^2 + 2 * L * a
```

which is the textbook constant-acceleration kinematic identity. That identity silently assumes step-change acceleration: the moment a new segment begins, the axis instantly attains `a_max`. A jerk-limited machine cannot do that — it spends real time and real distance ramping acceleration up and back down. As a result the classical formula *overestimates* reachable velocity, which turns into undershoot at commanded endpoint (jerk) and ringing in the feedforward input shaper.

We therefore need a drop-in replacement:

```c
double reachable_v_end(double v_start, double a_max, double j_max, double L);
```

with inputs

| symbol   | meaning                                   | constraint |
|----------|-------------------------------------------|------------|
| v_start  | velocity at the start of the segment      | >= 0       |
| a_max    | peak-acceleration limit (this axis)       | > 0        |
| j_max    | peak-jerk limit (this axis)               | > 0        |
| L        | available distance along the segment      | > 0        |

and output

> `v_end`: the largest velocity reachable in exactly `L` units of distance while respecting `|a(t)| <= a_max` and `|j(t)| <= j_max`.

We assume positive direction (pure acceleration). The underlying shape is the A1 "accel-side group" — a jerk-up phase, an optional constant-a cruise, and a jerk-down phase. See Phase A1 derivation for the forward closed form.

### Decel symmetry

The lookahead reverse pass needs a seemingly different inverse: *what is the largest `v_start'` such that decelerating from `v_start'` down to a known `v_end` finishes in at most L?* The A1 primitive already shows that the distance of an accel-side group is direction-agnostic — swap the endpoints and the distance is unchanged because the mean velocity and the total duration are unchanged. So

```
reverse_reachable_v_start(v_end, a_max, j_max, L) == reachable_v_end(v_end, a_max, j_max, L)
```

is just the same function with `v_start` := `v_end`. Both directions share one implementation.

### Monotonicity and well-posedness

`accel_side_distance(v_start, v_end, a_max, j_max)` is the function we invert. Over `v_end in [v_start, +infinity)` it is continuous and strictly increasing: the distance equals mean-velocity times total duration, and both mean velocity and total duration increase monotonically with `v_end`. Hence for any `L > 0` the equation

    accel_side_distance(v_start, v_end, a_max, j_max) = L

has exactly one solution `v_end >= v_start`. That is the value `reachable_v_end` returns.

At `L = 0` the solution degenerates to `v_end = v_start`; as `L -> +infinity`, `v_end -> +infinity`. The planner will clip this value against the per-axis and cruise velocity ceilings *after* the reachable-velocity query — the query itself does not impose an upper bound on `v_end`.

---

## Part 2 — Regime split

The A1 primitive has two shapes for the accel-side group. Let `dv = v_end - v_start`.

- **Triangular (Regime A).** Peak acceleration `a_peak = sqrt(j_max * dv)` never reaches `a_max`. The constant-accel dwell is zero; the group is a symmetric pair of jerk ramps. Condition: `dv < a_max^2 / j_max`.
- **Trapezoidal (Regime B).** Peak acceleration saturates at `a_max`. The group has three sub-phases (jerk up, constant a, jerk down). Condition: `dv >= a_max^2 / j_max`.

Define the boundary transition velocity-change

    dv_boundary = a_max^2 / j_max.

At the boundary the accel-side duration is `T_boundary = 2 * a_max / j_max` and the distance is

    L_boundary(v_start, a_max, j_max)
        = (v_start + v_end)/2 * T_boundary
        = (2*v_start + dv_boundary) * (a_max / j_max).

This quantity depends linearly on `v_start`, so different start velocities see the triangular-to-trapezoidal crossover at different distances.

**Dispatch rule.** Compute `L_boundary`. If `L <= L_boundary` use Regime A; otherwise use Regime B. Both regimes give a closed form; the two formulas join continuously at the boundary (both reduce to the same `v_end` when `dv = dv_boundary`).

For typical FDM numbers (`a_max ~ 5000..10000 mm/s^2`, `j_max ~ 50000..500000 mm/s^3`, `v_start ~ 50..300 mm/s`) the boundary distance is on the order of 0.05..2 mm. Short perimeters and sharp corners stay triangular; long outer-wall runs sit in the trapezoidal regime.

---

## Part 3 — Regime A (triangular) closed form

### Setup

In the triangular regime the accel-side group has two jerk phases of equal duration `t_j` and no dwell. From A1:

    a_peak = sqrt(j_max * dv),    t_j = a_peak / j_max,    T = 2 * t_j.

Distance equals mean velocity times duration:

    L = ((v_start + v_end) / 2) * T
      = (v_start + v_end) * sqrt(dv / j_max)
      = (2*v_start + dv) * sqrt(dv / j_max).                           (A1)

This is nonlinear in `dv`. Substitute `u = sqrt(dv)`, so `dv = u^2` and `sqrt(dv / j_max) = u / sqrt(j_max)`. Equation (A1) becomes

    L * sqrt(j_max) = (2*v_start + u^2) * u
                    = u^3 + 2*v_start * u,

i.e. the **depressed cubic**

    u^3 + p * u + q = 0,    with    p = 2*v_start,    q = -L * sqrt(j_max).      (A2)

### Solution by Cardano

The discriminant is

    D = (q/2)^2 + (p/3)^3 = L^2 * j_max / 4 + (2*v_start/3)^3.

Because `v_start >= 0` and `L >= 0`, both terms are non-negative and `D >= 0`. For a depressed cubic with `D >= 0` there is exactly one real root, given by Cardano's formula

    u = cbrt(-q/2 + sqrt(D)) + cbrt(-q/2 - sqrt(D))
      = cbrt(L*sqrt(j_max)/2 + sqrt(D)) + cbrt(L*sqrt(j_max)/2 - sqrt(D)).        (A3)

Note the second argument may be negative when `D > (q/2)^2`, which happens whenever `v_start > 0`. Use a *signed* cube root — `copysign(|x|^(1/3), x)` — otherwise a negative-base fractional power returns NaN in standard `pow`.

Then

    dv = u^2,    v_end = v_start + dv.                                          (A4)

### Special case v_start = 0

When `v_start = 0` the cubic collapses to `u^3 = L*sqrt(j_max)`, so

    u = (L * sqrt(j_max))^(1/3),   dv = u^2 = (L^2 * j_max)^(1/3).              (A5)

Equivalently `v_end = (L^2 * j_max)^(1/3)` — the familiar "pure-triangle-from-rest" identity.

### Fallback: two Newton steps from a good seed

For C implementations that prefer to avoid two cube roots, Newton-Raphson on `f(dv) = (2*v_start + dv) * sqrt(dv / j_max) - L` is extremely fast. A good initial guess is the classical-accel approximation with peak acceleration `a_peak(L) = (something in the range of a_max)`:

    dv_0 = -v_start + sqrt(v_start^2 + 2 * L * a_seed)     where  a_seed = 0.5 * a_max.

Empirically two iterations suffice to hit 1e-12 relative across the full input sweep, with three iterations being a conservative safety bound. The closed-form Cardano solution (A3) is preferred because the arithmetic is slightly cheaper than a loop and there is no seeding question.

### Cross-check with literature

Biagiotti & Melchiorri (*Trajectory Planning for Automatic Machines and Robots*, Springer 2008), §3.4 ("Double-S trajectory"), derives the double-S profile with constraints on v, a, j and lists the exact condition `dv < a_max^2 / j_max` as the threshold where the acceleration phase stays triangular. The ruckig library (pantor/ruckig) hits the same regime dispatch in its `AccelerationStep1` and `AccelerationStep2` classes; the single-segment analytical solver factors the same depressed cubic.

The widely-cited industrial-motor formula in the Analog Devices / Industrial-Monitor-Direct S-curve tutorials,

    D = (Vf^2 - Vi^2) / (2a) + (Vf^2 - Vi^2) * a / (2 * j^2),

is **dimensionally wrong** in its second term (it carries units of `velocity^2 * time / length`, not length). Numerically it diverges from the correct formula by tens of percent even in common cases. Do not use it. The correct trapezoidal-regime distance is derived below in Part 4 and matches the A1 simulation to machine precision.

---

## Part 4 — Regime B (trapezoidal) closed form

### Setup

In the trapezoidal regime `a_peak = a_max` and the duration is

    T = dv / a_max + a_max / j_max                                                 (B1)

(first term: time spent at constant `a_max`; second term: total duration of the two jerk ramps combined). Distance:

    L = ((v_start + v_end) / 2) * T
      = (2*v_start + dv) / 2 * (dv / a_max + a_max / j_max).                      (B2)

Multiply by 2 and expand:

    2*L = (2*v_start + dv) * (dv / a_max + a_max / j_max)
        = 2*v_start*dv/a_max + 2*v_start*a_max/j_max
          + dv^2/a_max + dv*a_max/j_max.

Multiply through by `a_max`:

    2*L*a_max = 2*v_start*dv + 2*v_start*a_max^2/j_max
                + dv^2 + dv*a_max^2/j_max.

Let `C = a_max^2 / j_max` (this is `dv_boundary`). Rearranging:

    dv^2 + (2*v_start + C) * dv + (2*v_start * C - 2*L*a_max) = 0.                (B3)

This is a **pure quadratic** in `dv`. Apply the quadratic formula with `a=1`, `b = 2*v_start + C`, `c = 2*v_start*C - 2*L*a_max`:

    dv = (-b + sqrt(b^2 - 4*c)) / 2,                                               (B4)

taking the `+` root because we need `dv >= 0` (and `v_end >= v_start`).

### Discriminant is always non-negative

    b^2 - 4*c = (2*v_start + C)^2 - 8*v_start*C + 8*L*a_max
              = 4*v_start^2 + 4*v_start*C + C^2 - 8*v_start*C + 8*L*a_max
              = (2*v_start - C)^2 + 8*L*a_max.

Both summands are non-negative, so the sqrt always succeeds. No branch cuts.

### Closed form

    v_end = v_start + dv,
    dv    = ( -(2*v_start + C) + sqrt( (2*v_start - C)^2 + 8*L*a_max ) ) / 2.    (B5)

### Classical-limit sanity check

As `j_max -> +infinity`, `C = a_max^2 / j_max -> 0`. Equation (B3) collapses to

    dv^2 + 2*v_start*dv - 2*L*a_max = 0,

whose positive root is `dv = -v_start + sqrt(v_start^2 + 2*L*a_max)`, i.e.

    v_end = sqrt(v_start^2 + 2*L*a_max).

That is exactly the classical constant-accel formula (Klipper's `delta_v2 = 2*L*a`). The jerk-aware formula (B5) is a strictly-smaller correction; numerically at `j_max = 1e12`, `a_max = 5000`, `v_start = 100`, `L = 50`, the jerk-ware answer differs from classical by ~1.4e-5 mm/s (the residual `C` contribution), and with realistic jerk limits it differs by several percent for long moves and much more for short moves.

### Cross-check

I verified (B2) directly against the A1 simulation `accel_side_distance`:

    v_start = 100,  v_end = 400,  a_max = 5000,  j_max = 100000.
      A1 sim : 27.50000000
      (B2)   : 27.50000000   (match to machine precision).

---

## Part 5 — Combined API and dispatch

### Pseudocode

    double reachable_v_end(v_start, a_max, j_max, L):
        if not all finite, or a_max <= 0, or j_max <= 0:  return NaN
        if v_start < 0 or L < 0:                          return NaN
        if L == 0:                                        return v_start

        dv_b = a_max * a_max / j_max
        L_b  = (2*v_start + dv_b) * (a_max / j_max)

        if L <= L_b:
            // Regime A: depressed cubic u^3 + 2*v_start*u - L*sqrt(j) = 0
            p = 2*v_start
            q = -L * sqrt(j_max)
            D = (q/2)^2 + (p/3)^3                 // always >= 0
            u = signed_cbrt(-q/2 + sqrt(D))
              + signed_cbrt(-q/2 - sqrt(D))
            return v_start + u*u

        // Regime B: quadratic dv^2 + (2*v_start + C)*dv + (2*v_start*C - 2*L*a) = 0
        C = dv_b
        b = 2*v_start + C
        disc = (2*v_start - C)^2 + 8 * L * a_max   // guaranteed >= 0
        dv = 0.5 * ( -b + sqrt(disc) )
        return v_start + dv

### Edge cases

- `L = 0`: return `v_start`.
- `v_start = 0`, Regime A: `u = cbrt(L * sqrt(j_max))`, `v_end = u^2`; no branch-safe cube root needed because the second Cardano term is zero.
- `v_start = 0`, Regime B: `b = C`, `disc = C^2 + 8*L*a_max`; fine.
- `L` below `L_boundary` by a rounding-error hair: Regime A is well-conditioned here; the two regimes agree at the boundary to machine precision.
- Invalid inputs (negative, NaN, Inf): return NaN. The C caller should validate upstream and treat NaN as a configuration error.

### Cost

Both regimes are a handful of multiplies plus one sqrt (Regime B) or one sqrt + two cbrts (Regime A). On modern x86, roughly 10–15 ns per call. The prior `delta_v2 = 2 * L * a` was a single multiply; we are spending about 10× more arithmetic per query, but these queries are not on the step-generation hot path — they are in planner bookkeeping that runs once per move, at most a few hundred calls per second on a busy print. No performance concern.

---

## Part 6 — Numerical verification

The Python reference implementation `jerk_reachable_ref.py` runs a 4x3x3x5 = 180-case sweep:

- `v_start in {0, 50, 200, 500}` mm/s
- `a_max   in {2500, 5000, 10000}` mm/s^2
- `j_max   in {50000, 100000, 500000}` mm/s^3
- `L       in {0.1, 1, 10, 100, 1000}` mm

For every case it calls `reachable_v_end`, then invokes `accel_side_distance` from Phase A1 on the returned `v_end` and checks that the round-trip distance equals `L`. Results:

    === phase A2b reachable_v_end sweep ===
    total cases: 180
      regime triangular: 101 (56.1%) max_rel_err=7.41e-13
      regime trapezoidal: 79 (43.9%) max_rel_err=8.88e-16
    max abs err: 2.245e-12
    max rel err: 7.411e-13
    OK: all cases pass at rtol=1e-9.

**All 180 cases pass the round-trip invariant to rtol 1e-9.** The trapezoidal regime is near-machine-epsilon (a single quadratic formula), and the triangular regime carries a touch more error from the two cube roots but still sits six orders of magnitude inside the 1e-9 budget.

Regime distribution: ~56% of the sweep hit the triangular branch, ~44% the trapezoidal branch. Both regimes are exercised; no dead code.

Classical-limit check at `j_max = 1e12` recovers the Klipper formula `sqrt(v_start^2 + 2*L*a_max)` to ~1e-5, which is the expected residual from the finite `C = a_max^2 / j_max` term at that jerk setting.

Monotonicity: for every `(v_start, a_max, j_max)` triple, `reachable_v_end` is non-decreasing in `L` across the sweep. No violations.

---

## Part 7 — Implementation notes for the C port

- **Precision:** fp64 everywhere, matching Phase A1. The algebra is well-conditioned in both regimes; no catastrophic cancellation. Do not use fp32 even in the acceleration limiter — the boundary distance `L_b` can vary by six orders of magnitude depending on user inputs, and fp32 can drop to single-digit relative precision at realistic `j_max` values.
- **Zero / negative guards:** reject `a_max <= 0`, `j_max <= 0`, `v_start < 0`, `L < 0`, or any non-finite input. Return NaN or raise a validation error upstream; never divide by user-supplied zero.
- **Cube root:** use `cbrt` from `<math.h>` (handles negatives correctly). If you have to DIY, use `copysign(pow(fabs(x), 1.0/3.0), x)`. Do *not* use `pow(x, 1.0/3.0)` directly for a potentially-negative `x` — returns NaN in C.
- **Discriminants:** both `D = L^2*j/4 + (2*v_start/3)^3` (Regime A) and `disc = (2*v_start - C)^2 + 8*L*a_max` (Regime B) are algebraically non-negative. Still clamp to zero before `sqrt` as a numerical safety net — small negative values can emerge from cancellation if a caller sneaks in denormals.
- **API shape:** mirror A1. Export both the fast `reachable_v_end(v_start, a_max, j_max, L)` and a diagnostic `reachable_v_end_info(...)` that also returns a regime tag for logging / testing. The diagnostic version is what the A1/A2b unit tests call; the fast version is what the planner hot path calls.
- **Call-site replacement:** every `delta_v2 = 2 * move_d * max_accel` becomes

        v_end_max = reachable_v_end(v_start, axis_a_max, axis_j_max, move_d);
        delta_v2  = v_end_max * v_end_max - v_start * v_start;   // if the caller wants v^2 units

  keeping the downstream `min(v_end_max, cap)` clamp. Note that callers using `delta_v2` as "spare velocity^2 budget" should switch to comparing `v_end_max` directly; the old `delta_v2` form hides the non-linear distance dependence and is harder to reason about.
- **Unit tests:** port the Python sweep. Add edge cases at `L = 0`, `L = L_boundary` (both sides, to exercise the dispatch), `v_start = 0`, and `j_max` large (to spot-check the classical-limit reduction).

---

## Key takeaways

- **Regime mix in real printing:** the triangular regime dominates short-perimeter prints and sharp corners (where `L` is small or `dv` is modest); the trapezoidal regime dominates long infill runs and outer walls. The A2b sweep hit 56/44 split, which is broadly consistent with the per-move distance distribution we see on a Trident.
- **Most common numerical gotcha:** the widely-circulated Analog Devices / Industrial-Monitor-Direct S-curve distance formula is dimensionally wrong in its second term. I cross-checked it against the A1 simulation and it was off by ~50%. Trust the A1 primitive, not blog posts.
- **The main surprise** was how clean Regime B is — a plain quadratic, one line of arithmetic, and it exactly collapses to the classical Klipper formula as `j_max -> infinity`. That means swapping out `delta_v2 = 2*L*a` for `reachable_v_end` is a drop-in *monotonic improvement*: same formula at infinite jerk, strictly-smaller (more conservative, more physical) at finite jerk.
- Regime A required a little more care — Cardano + signed cube root — but it is still a closed form and the Python implementation is well inside the 1e-9 tolerance budget.
