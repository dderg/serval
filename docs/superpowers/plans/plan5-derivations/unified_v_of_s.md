# Unified v(s) for the direct-quintic corner primitive (Plan 5 Pillar 2b)

**Status:** research / design memo. Not approved. Written 2026-04-22 on
branch `magnum-opus`.

> **⚠️ Revision notes (see adversarial reviews + verifier memos):**
> - **§3.1 v_sat formula outdated.** Correct form is
>   `v_sat(s) = sqrt(a_max / (G_worst(s) · κ(s)))` with
>   `G_worst(s) = max_axes G_axis · (|proj_t(s)|+|proj_n(s)|)` —
>   sum-of-projections, no √2. See
>   `per_axis_saturation_derivation.md` for the derivation.
> - **§8 worked example uses G=1 (pre-D1 baseline), not G=2.003
>   (bs3 post-D1).** All T_opt / T_safe / throughput-gain numbers
>   need regeneration. Qualitative conclusion holds (TOPP > unsafe
>   baseline). Regenerated numbers are orientation-dependent
>   (see per_axis_saturation_derivation.md).
> - **§4.3 claim "~12% worse than TOPP" is wrong.** Reviewer's
>   faithful trapezoid-in-s fit gives 9.04 ms vs TOPP's 9.18 ms —
>   **only 1.4% off**. This strengthens the representation choice.
> - **§7 retract/re-plan architecture superseded by Option Z.**
>   Spec D7 now uses upstream `v_cap_min` junction-cap feed
>   instead of 2-pass retract. No `LookAheadQueue` changes needed.
> - **§3.4 monotonicity claim is false for v_step** (axis
>   projection varies non-monotonically). Other four caps are
>   monotone; v_step is not.
> - **§1.2 "5.3× centripetal overshoot" conflates v_step with
>   v_cent.** Actual centripetal overshoot at the shoulders is ~4×,
>   not 5.3×. Qualitative framing ("baseline is unsafe") holds.
> - **Struct size in §4 was 24 doubles; actual implementation
>   uses Option B (per-phase polynomial-in-t, ~840 B per move).**
>   See spec D2b for canonical struct layout.

**Companion reading:**
- `docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md`
  (current D2c; single-velocity quintic emit — the baseline this memo
  replaces).
- `docs/superpowers/plans/plan5-derivations/saturation_feedback.md`
  (Pillar 1 inverse-saturation cap, feeds into the unified `v_cap_fn`).
- `docs/superpowers/plans/plan5-derivations/direct_quintic_architecture.md`
  (Option A tagged-union `struct move`, the trapq container this memo
  extends).
- `klippy/blendquintic.py:429-595` (`QuinticShape` and `v_cap_fn`).
- `klippy/blendextruder.py:91-165` (`cap_move`, per-move extruder cap).
- `klippy/chelper/trapq.h:15-21` (`struct move`, the tagged-union
  target).

## 1. Problem formulation

Given a quintic corner blend of arc length `L` with position `p(s)` and
scalar velocity limit `v_cap(s)`, boundary conditions

- `v(0) = v_in`, set by the cruise velocity of the prev move at its
  truncated end,
- `v(L) = v_out`, set by the cruise velocity of the next move at its
  truncated head,

and a magnitude bound on tangential acceleration `|dv/dt| ≤ a_max`
along the curve, the planner must find the minimum-time velocity
profile

```
    T_blend = min  ∫₀^L ds / v(s)
             v(·)
    s.t.    0 < v(s) ≤ v_cap(s)           for all s ∈ [0, L],
            v(0) = v_in,  v(L) = v_out,
            |v · dv/ds| ≤ a_max.                         (1)
```

The acceleration bound is the relevant one because `a_t = dv/dt =
v · dv/ds` is what the toolhead experiences tangentially. Centripetal
accel `v²·κ(s)` is already folded into `v_cap(s)`.

### 1.1 What "time-optimal" means here

In a vacuum — `v_in = v_out = 0`, no outer moves — the solution is the
classical bang-bang profile: accelerate at `+a_max` from `v_in`, ride
`v_cap(s)` wherever it binds, decelerate at `-a_max` to `v_out`.
Standard single-axis path-parameterised trajectory planning. The extra
structure here is (a) `v_cap(s)` composed from five sources (Pillar 1,
Pillar 2 centripetal, rotation-jerk, shaper bandwidth, and Plan 3
extruder), and (b) the blend sits inside a lookahead window so `v_in`
and `v_out` are themselves negotiable — they push back on prev/nxt's
cruise speeds if the blend cannot absorb them.

### 1.2 What today's code does (baseline for comparison)

`CornerBlender._emit_blend` (`klippy/blendplanner.py:210-220`) collapses
`v(s)` to a single scalar `v = min(prev.cruise, nxt.cruise, v_cap(L/2))`
applied as a flat cap on every polyline sub-move. Direct-quintic D2c
preserves this single-scalar behaviour — the quintic trapq entry uses
`T = L / v_cap(L/2)` as its `move_t`.

**This is mathematically unsafe.** The v_cap profile has its minimum at
the curvature-peak (`t ≈ 0.18` and `t ≈ 0.82`, symmetric), not at the
midpoint, so `v_cap(L/2) > min_s v_cap(s)` in general. At `cd = 0.05`,
`θ = π/2`, `a_max = 5000`: `v_cap(L/2) = 34.75 mm/s` but the shoulder
minimum is `15.06 mm/s`. The polyline sub-moves at the shoulder are
capped at 34.75 mm/s by the aggregator cap — a 2.3× overshoot of the
local constraint. Centripetal accel there exceeds `a_max` by a factor of
`(34.75 / 15.06)² ≈ 5.3`. This is shippable today only because the
existing shaper cap and the `corner_deviation` default (~0.05 mm) keep
the absolute violation small and because the sub-move timeslice through
the shoulder is brief. It is not safe at looser tolerances.

Pillar 2b must simultaneously (i) make the cap honest at every point
in s, and (ii) recover the throughput lost to a min-over-s constant-v
bound by letting v(s) rise in the corners-of-corners where κ drops.

## 2. TOPP vs closed-form analysis

### 2.1 Closed form attempt

`v_cap_centripetal(s) = √(a_max / (G · κ(s)))`. For a quintic Bezier
`κ(s)` is a ratio of polynomials in `s` (via the `s ↦ t` remap), with
the curvature closed-form in `t` being `|B' × B''| / |B'|³`. Squaring
`v_cap` gives `v_cap² = a_max / (G · κ)`, a ratio of a polynomial to a
square-root-of-polynomial in `t`. The other three caps each layer in
additional branches (`min` of square roots, cube roots, piecewise
shaper-bandwidth curves). **There is no closed-form analytic inverse
of v_cap(s)** — the `min` of five algebraic branches is at best
piecewise-algebraic, with break-points that depend on `G`, the active
shaper variant, the extruder flow ratio, and the jerk ceiling. Finding
those break-points symbolically is not worth the engineering cost.

### 2.2 What a closed form would need

Even if `v_cap` collapsed to a single algebraic branch (centripetal
only), the bang-bang-with-cap profile still requires a 1D root-find
in s for `v_in² + 2·a_max·s_fwd = v_cap(s_fwd)²` (and symmetric for
v_out). That's a numerical method dressed up as closed-form, and the
win over TOPP is marginal: both are O(N). TOPP is simpler to implement
and generalises trivially as new cap sources are added.

### 2.3 TOPP and why it is the right tool

Pham 2014's TOPP (*time-optimal path parameterisation*) algorithm
solves exactly (1) with an arbitrary pointwise `v_cap(s)`. The algorithm
is two O(N) passes over a uniform discretisation `s_i = i · L/N`:

```
    # Forward pass (accel-limited from v_in):
    v_fwd[0] = v_in
    for i in 0 … N-1:
        v_fwd[i+1] = min( √(v_fwd[i]² + 2·a_max·Δs), v_cap(s_{i+1}) )

    # Backward pass (decel-limited to v_out):
    v_back[N] = v_out
    for i in N, N-1, … 1:
        v_back[i-1] = min( √(v_back[i]² + 2·a_max·Δs), v_cap(s_{i-1}) )

    # Profile:
    v_opt[i] = min(v_fwd[i], v_back[i])                      (2)
```

The forward pass enforces `v² ≤ v_in² + 2·a_max·s` (max reachable from
start). The backward pass enforces `v² ≤ v_out² + 2·a_max·(L-s)` (max
from which stopping at `v_out` is possible). Their pointwise `min`
intersected with `v_cap` is the optimum. Pham's theorem 3.1 proves
this is time-optimal under an arbitrary — possibly discontinuous —
`v_cap(s)`.

**Why TOPP fits here.** We already pay O(N) in emit-time work — the
quintic is sampled at N points for the s-to-t table (`_build_s_to_t_map`
at `klippy/blendquintic.py:346-377`, currently `n_subintervals = 20`;
bump to ~100-200 for TOPP). The forward-backward passes add O(N) for a
few dozen arithmetic ops per step; cheaper than the curvature eval
itself. No convex solver, no root-find.

**Why not RRT-flavoured / iterative convex methods.** The Pestana-Shiller
convex-concave algorithm and Bobrow's phase-plane method iterate on v
and v̇ simultaneously. They matter when the constraint set is not
separable into `v_cap(s)` plus `|v̇| ≤ a_max`. We have separable
constraints (Pillar 2's centripetal is pure v, saturation is pure v,
shaper bandwidth is pure v; tangential-accel is pure v̇ via the
`a_max` bound). TOPP is the right fit for separable problems.

**Robustness.** Pham's algorithm is guaranteed to converge in two
passes for well-posed inputs. Ill-posed cases (`v_in > v_cap(0)` or
`v_out > v_cap(L)`) must be caught upstream — §7.

### 2.4 Recommendation

Ship TOPP-on-a-dense-grid, `N ≈ 128-256`. Refine to a curvature-adapted
non-uniform grid only if profiling shows the emit-time O(N) dominates.

## 3. Composition of caps

All five cap sources compose as a pointwise `min`:

```
    v_cap(s) = min(
        v_max,                                      [user / toolhead]
        v_sat(s)    = √( a_max / (G · κ(s)) ),      [Pillar 1 sat., saturation_feedback.md]
        v_jerk(s)   = (j_eff / κ(s)²)^{1/3},        [Pillar 2 rotation jerk]
        v_step(s)   = √( A_axis · R(s) / |n̂ · ê| ), [Pillar 2 shaper-rejection band]
        v_extr(s)   = cap_k( k(s), ... )            [Plan 3 extruder flow cap]
    )                                                        (3)
```

where `κ(s)` is quintic curvature, `R(s) = 1/κ(s)`, `n̂(s)` is the
path normal projected onto each axis, and `k(s)` is the linearly
interpolated flow ratio.

### 3.1 Why v_sat subsumes Pillar 2 centripetal

Pillar 2's original centripetal cap was `v_cent = √(a_max / κ)`. Pillar
1 replaces `a_max` with `a_max / G`, i.e. `v_sat ≤ v_cent` always
(`G ≥ 1`). When `G = 1` (no inverse, classic FIR path) they coincide;
when `G > 1` (post-D1), `v_sat` binds tighter. The `min` collapses
(saturation_feedback.md §3).

### 3.2 Shaper bandwidth cap — where it comes from

Plan 4's `v_step_cap` (`klippy/blendshaper.py:120-125`) was derived for
a *sub-move polyline* bounding the centripetal step the shaper sees.
With direct-quintic it applies pointwise: at each s the blend has a
local radius `R(s) = 1/κ(s)`, and the shaper's entry-step velocity bound
is `√(A_axis · R(s) / |proj|)`. The per-s form drops in cleanly —
`v_step(s)` is what `v_step_cap` already computes, just evaluated per s
instead of once per sub-move. No new derivation required.

### 3.3 Extruder cap — see §6

### 3.4 Monotonicity claim

All four κ-dependent caps are strictly increasing in `R(s) = 1/κ(s)`:
as the blend straightens out (κ → 0 at endpoints), every cap goes to
`+∞` and only `v_max` binds. The `v_cap(s)` envelope is therefore
concave-down around the curvature peak and monotone elsewhere.
**Consequence for TOPP:** the forward pass can accelerate unboundedly
in the straight regions, which is why `v_in` and `v_out` get enforced
only at the endpoints and the accel-ramps-in from the outer moves
cross into the blend. This is the mechanism that lets the planner
exploit the straightening-out region to speed up — see §7.

## 4. Representation choice

### 4.1 Options recap

(a) **Trapezoid in s** — `(accel_end_s, cruise_v, decel_start_s,
    end_v)`, 4 scalars, `s(t)` piecewise-quadratic.
(b) **Piecewise-polynomial v(s)** — N pieces each with polynomial
    coefficients; `s(t)` via numerical integration at query time or
    precomputed.
(c) **Multiple trapq entries per quintic** — N linear sub-moves each
    with its own `(start_v, half_accel)`. Defeats single-quintic goal.
(d) **Direct v(t) polynomial** — store v(t) coefficients; s(t) is
    antiderivative, position is quintic ∘ s(t).

### 4.2 Analysis

**(c) is out.** It reintroduces the polyline in a different disguise
and defeats D2c's architectural premise. Skipped.

**(a) trapezoid-in-s.** Simple and cheap. Fits four scalars into the
tagged union alongside quintic coefficients with no structural
churn. Three phases:

- Accel: `v² = v_in² + 2·a_max·s` for `s ∈ [0, accel_end_s]`.
- Cruise: `v = cruise_v` for `s ∈ [accel_end_s, decel_start_s]`.
- Decel: `v² = v_out² + 2·a_max·(L-s)` for `s ∈ [decel_start_s, L]`.

`t(s)` for each phase has closed form (`t = (v - v_in)/a_max` in accel,
linear in cruise, similar in decel). Inverting for `s(t)` at query
time is two ops plus one `sqrt`. **Cost per step-gen query: ~10 flops
plus the quintic position eval.**

The bad case: `v_cap(s)` has multiple interior minima (two shoulders
symmetric around the midpoint) separated by a local maximum. A single
trapezoid cannot ride a cap that dips twice. For a symmetric blend
the two shoulders have the same `v_cap` value and a single cruise
segment hits both — works if the central bump in `v_cap(s)` is above
the shoulders' minimum, which it is for Pillar 2's centripetal alone
(central `R` is largest). Adding shaper-bandwidth or extruder caps
can change this shape; §4.3 does numerical verification on the
worked example.

**(b) piecewise-polynomial v(s).** Most flexible. Can track a
multi-minimum `v_cap(s)` exactly. But: `s(t)` requires numerical
integration unless each piece is pinned to a simple form. Emit-time
cost is higher; query-time cost multiplies by the piece count. For
typical blends (`L ~ 0.1-2 mm`) the shape of `v_cap(s)` is smooth
enough that (a) is within 1-3% of (b). The engineering cost of (b)
is significant: per-piece polynomial coefficients inflate the tagged
union from ~40 bytes to ~200 bytes, and the query-layer dispatch in
`itersolve` gets more complex. Defer (b) to a follow-up.

**(d) direct v(t).** Elegant. Position query is quintic ∘ s(t) where
s(t) is an antiderivative of a polynomial v(t) — a polynomial. The
composition polynomial-in-polynomial is exactly what D2a's `struct
move` with Bernstein coefficients stores. But: fitting a v(t)
polynomial to the TOPP grid requires least-squares over the
(irregular) break-points between forward-pass, cap-ridden, and
backward-pass regions. Fitting across a corner where `v_opt(s)` has
a kink (the cap-to-ramp transition) with a single low-degree
polynomial introduces ringing. You'd need piecewise (d), which is
structurally (b) with less expressive pieces. Skipped.

### 4.3 Numerical check: does (a) suffice?

Worked example, `cd = 0.05`, `a_max = 5000`, `v_in = v_out = 30`,
shaper = bs3 at 40 Hz, `target_smoothing = 0.12` (A_axis = 3635):

TOPP on N = 400 grid gives T_opt = 9.177 ms (sampled from live code,
see §8 for full numbers).

Fit (a) to TOPP: `accel_end_s = s_fwd`, `cruise_v = min_s v_opt(s)`,
`decel_start_s = s_back`, `end_v = v_out`. Emit the single-cruise
trapezoid and recompute T:

For the symmetric 90° / cd=0.05 case, `min_s v_opt = 17.57 mm/s` at
both shoulders, `v_opt(L/2) = 27.58 mm/s`. A single cruise at 17.57
gives T = 10.29 ms. A two-phase emit (two cruise pieces at the two
shoulders, connected by a mid-blend ramp) recovers T = 9.31 ms —
within 1.5% of TOPP.

**So (a) is ~12% worse than TOPP on this geometry** — the single
cruise has to sit at the shoulder minimum rather than riding the
central lift. (b) with 3 pieces closes the gap.

**Engineering verdict: trapezoid-in-s (option a) as the ship
target.** Document the residual ~10% gap; upgrade to piecewise (b)
as a Plan 6 item if HW shows it matters. The unsafe baseline we're
replacing loses the same ~20% (T_opt = 9.18 ms vs T_safe_const =
12.01 ms) — so even (a) captures most of the win.

### 4.4 Trapezoid-in-s storage layout

Extend D2b's tagged-union `struct move` with a third variant:

```c
enum move_kind {
    MOVE_LINEAR = 0,
    MOVE_QUINTIC_CONST_V = 1,   /* current D2c: single velocity */
    MOVE_QUINTIC_TRAPEZOID_S = 2  /* Plan 5 Pillar 2b: trapezoid-in-s */
};

struct move_quintic_trap {
    /* Geometry: same as MOVE_QUINTIC_CONST_V */
    struct coord c1, c2, c3, c4, c5;  /* c0 = start_pos */
    double arc_length;                /* L */
    /* Velocity profile trapezoid-in-s: */
    double v_in, v_out;               /* endpoint speeds */
    double cruise_v;                  /* interior constant-speed level */
    double accel_end_s;               /* s where accel ramp hits cruise */
    double decel_start_s;             /* s where decel ramp starts */
    double a_max;                     /* tangential accel magnitude */
};
```

Total per-move storage: 5×3 (quintic) + 7 scalars + `arc_length` =
**24 doubles ≈ 192 bytes**. Up from D2c's 5×3 + 1 scalar. Comfortably
fits in the `union` slot of `struct move`.

## 5. Query-time cost

Per `calc_position_cb` invocation:

1. **Phase dispatch on `move_time`:** 2 compares against precomputed
   `t_accel_end` and `t_decel_start`.
2. **`t → s` within phase:** accel is `v_in·t + 0.5·a_max·t²`,
   cruise is linear, decel is quadratic. ~3 fma each.
3. **`s → u`:** precomputed `s_tab` / `t_tab` (stored C-side inside
   the `struct move`, `n_subintervals = 40`) plus linear interp.
   Skip the Newton refinement — at n=40 the linear-interp error is
   ≤ 1e-4 mm, adequate for stepper quantisation. ~6 compares + 2
   fma.
4. **Quintic eval at u:** Horner on pre-converted monomial coeffs,
   5 fma per axis = 15 fma.

**Total:** ~40-50 flops per query + 6-compare bisect. The current
linear-move path is ~3 flops, so quintic-trapezoid is ~15× slower
per query — but corners are a small fraction of total trapq
traversal (typical: 0.2 mm blend vs 200 mm straights → linear
dominates 99% of step-gen). Projected sysload impact ≤ 2%.

## 6. Plan 3 extruder cap absorption

### 6.1 Current Plan 3 (per-trapq-entry)

`blendextruder.cap_move` (`klippy/blendextruder.py:91-165`) computes
`(v_cap, a_cap)` once per move from a scalar flow ratio `k =
move.axes_r[3]`. For travel moves `k ≤ 0` returns `(∞, ∞)` (no cap).
For linear PA the cap is closed-form; for tanh / recipr it is a 1D
bisection over `v_xy`.

### 6.2 Per-s cap for the quintic

Inside a blend the planar tangent direction rotates from prev's `e1`
to next's `e2`, but the **flow ratio `k(s)` is a property of the E
axis's interpolated extrusion**, not the tangent. Plan 3 today
prorates `axes_r[3]` by arc-length in `interpolate_extruder`
(`klippy/blendmath.py`, called from `blendplanner._emit_blend`). The
clean per-s form is:

```
    k(s) = k_prev + (k_next - k_prev) · (s / L)                     (4)
```

assuming linear interpolation of the E delta across the blend (same
as current polyline behaviour; exact because E moves linearly in s).

The extruder cap per s is `cap_move(k(s), ...)`:

- **Linear PA** (`cap_k_linear`): `v_extr(s) = (v_E_max - pa·a_E_cap)
  / k(s)` and `a_extr(s) = a_E_cap / k(s)`. Both are closed-form in
  `k(s)`, so in `s`. **Cheap to evaluate in TOPP's grid.**
- **Non-linear PA** (tanh/recipr): bisection. Need ~30 iters × N
  grid points = 30N evals per blend. Expensive but amortises across
  emit time (tens of micro-seconds per blend — acceptable).

### 6.3 Absorption into v_cap_fn

Add `v_extr(s)` to the `min` in (3). One new branch in the per-s
cap function; no new pass or iteration. The accel cap `a_extr(s)`
is a separate concern — it caps `a_max`, not `v`, so it feeds into
TOPP's `a_max` ceiling per step:

```
    a_max_eff(s) = min(a_max, a_extr(s))                            (5)
```

`a_max_eff` varies with `s`. This is still TOPP-compatible: replace
`2·a_max·Δs` in (2) with `2·a_max_eff(s_i)·Δs`. Pham's theorem
still holds for state-dependent accel bounds.

### 6.4 Where Plan 3's `move.limit_speed` plugs in

Plan 3 today calls `move.limit_speed(v_cap, a_cap)` on the trapq
move. With per-s caps this is replaced by: the TOPP forward pass
inside the blend naturally incorporates `v_extr(s)` and `a_extr(s)`.
On the truncated prev and next moves (straight segments, still
MOVE_LINEAR kind), Plan 3's existing scalar cap applies unchanged.
**No breaking change to `blendextruder.cap_move`'s signature.** The
call-site for blends migrates from `move.limit_speed` to TOPP
integration.

## 7. Boundary matching and lookahead interaction

### 7.1 Feasibility conditions on v_in / v_out

`v_in > v_cap(0)` and `v_out > v_cap(L)` are both fine — the endpoints
have `κ(0) = κ(L) = 0` so every κ-dependent cap is `+∞`. `v_max` is
the only binding cap at endpoints. If `v_in > v_max` or `v_in ≤ 0`
the toolhead is broken elsewhere (flag it; assert; do not silently
clip).

What can break the blend is if `v_in` cannot ramp down to `v_cap(s)`
within the blend length:

```
    Ramp-down distance from v_in to v_cap(s_peak):
        d_fwd_min(v_in) = (v_in² - v_cap(s_peak)²) / (2·a_max)

    Feasibility:  d_fwd_min(v_in) ≤ s_peak.                          (6)
```

Violation means the forward pass cannot drop fast enough to meet
the cap. Symmetric condition for `v_out` and `L - s_peak` for the
backward pass.

### 7.2 Upstream feedback (three cases)

Let `v_cap_peak = min_s v_cap(s)` (the tightest point along the
blend). Three cases:

**Case (i): v_in ≤ v_cap_peak AND v_out ≤ v_cap_peak.** Both endpoints
are compatible with the cap. TOPP produces a valid profile with no
upstream feedback. Emit the trapezoid, done.

**Case (ii): v_in > v_cap_peak, but (6) holds.** Forward pass starts
at v_in, decelerates at `-a_max`, hits the cap before s_peak, rides
the cap through s_peak, then the backward pass takes over. Same for
the (iii-symmetric) case.

**Case (iii): (6) violated.** The blend cannot absorb v_in with the
accel budget alone. **Feed back:** reduce prev's `max_cruise_v2` so
that v_in ≤ max_feasible. The max feasible is:

```
    v_in_max = √( v_cap(s_peak)² + 2·a_max·s_peak )                  (7)
```

which is the velocity at s=0 of a backward ramp starting at the cap
at s_peak. Same logic for v_out_max at the other end.

In code terms, this replaces today's `prev.limit_next_junction_speed`
call in `_suppress_and_advance` with a more precise cap: instead of
`suppressed_junction_v` (which assumes sharp-V geometry), the blend
feeds `v_in_max` back to prev. The `LookAheadQueue`'s next pass
(`klippy/toolhead.py:157-219`) then propagates it.

### 7.3 Iteration question

Is there a risk of planner oscillation? `v_in_max` depends on the
blend geometry, which depends on `corner_deviation` (static config),
not on prev's cruise_v. The feedback is one-directional: blend →
prev. No iteration. **One pass of `CornerBlender.feed` produces a
fixed-point feasible profile.**

The only subtlety is when two adjacent blends share a move (very
short straight segment between two corners). Then the blend between
A-B and the blend between B-C both constrain B's cruise_v. The
LookAheadQueue's existing reachable-velocity propagation handles
this by taking `min(v_in_max_fromBC, max_cruise_v2_B)` — the same
machinery that handles junction-velocity caps today.

### 7.4 When v_in / v_out must stay equal to prev / next cruise_v

Today's `_copy_caller_state` (`klippy/blendplanner.py:43-67`)
preserves `max_cruise_v2` across truncation so the prev/nxt continue
at their full cruise. Pillar 2b needs the same: the blend's `v_in`
= `sqrt(prev_trunc.max_cruise_v2)` unless (iii) fires, in which case
the blend requests a lower prev-cruise. That's a `limit_speed`-style
downstream cap, not a re-planning of prev.

## 8. Worked example

Setup: 90° corner, `cd = 0.05 mm`, `a_max = 5000 mm/s²`, shaper =
bs3 at 40 Hz, `target_smoothing = 0.12`, so `A_axis = 3635 mm/s²`
(per spec table 2026-04-22-plan5 §D1). `v_in = v_out = 300 mm/s`
nominally, but (§7) this is infeasible for the cd=0.05 blend —
blend can only absorb up to `v_in_max = √(v_cap_peak² + 2·a_max·L) =
√(15² + 2·5000·0.18) = √(225 + 1808) = 45 mm/s`. So the realistic
scenario is `v_in = v_out = 30 mm/s` with prev/nxt having
decelerated to 30 via the upstream feedback loop — representative
of sharp-corner slow-zone behaviour.

### 8.1 v_cap at 5 sample points

```
 s/L    s [mm]   κ [1/mm]   v_sat [mm/s]   v_jerk    v_step    v_cap
 0.00   0.0000     0.000      ∞              ∞         ∞         500
 0.25   0.0452    13.101      19.5           ∞         17.7       17.7
 0.50   0.0904     4.141      34.7           ∞         69.5       34.7
 0.75   0.1356    13.101      19.5           ∞         17.7       17.7
 1.00   0.1808     0.000      ∞              ∞         ∞         500
```

(Computed live with `klippy/blendquintic.py` at branch head. `v_max`
clip at 500 = `limits.v_max`. Jerk cap inactive because
`jerk_max=None` in current `CornerBlender.feed`.)

**The shaper bandwidth (`v_step`) binds tighter than saturation
(`v_sat`) at the shoulders.** `v_cap_peak = 17.7 mm/s` at both
`s/L = 0.25` and `s/L = 0.75`. Interior cap rises to 34.7 at the
midpoint because κ drops there.

### 8.2 TOPP profile (N = 400 grid)

```
 s/L    v_cap   v_fwd   v_back   v_opt
 0.00   500     30      22.45    22.45
 0.10   19.2    19.2    20.0     19.2
 0.18   15.1    15.1    15.1     15.1          ← shoulder minimum
 0.25   17.7    17.6    17.7     17.6
 0.50   34.7    27.6    27.6     27.6          ← central rise
 0.75   17.7    17.7    17.6     17.6
 0.82   15.1    15.1    15.1     15.1          ← shoulder minimum
 0.90   19.2    20.0    19.2     19.2
 1.00   500     22.45   30       22.45
```

T_opt (integrated) = **9.18 ms**.

### 8.3 Comparisons

```
 Method                              v [mm/s]         T [ms]     delta
 Naive v = v_cap(L/2) (current D2c)  34.75 (unsafe!)  5.20       -43%
 Safe constant v = min_s v_cap       15.06            12.01      +31%
 TOPP trapezoid-in-s (Pillar 2b)     per above         9.18       baseline
 Trapezoid-in-s fit (option (a))     single cruise    10.29      +12% over TOPP
 Piecewise-3 fit (option (b))        two cruises       9.31      +1.4% over TOPP
```

**Throughput gain TOPP vs safe-constant: 23.6% time saved.**
**Throughput gain trap-in-s (a) vs safe-constant: 14.3% time
saved.**

TOPP also makes the current D2c scheme honest: D2c's 5.20 ms is
not a "throughput win" — it's a safety violation. Comparing D2c
to Pillar 2b is comparing a broken profile to a correct one.

### 8.4 Sensitivity check

Repeated at `cd = 0.2` (`L = 0.72 mm`): T_safe_const = 24.01 ms,
T_opt = 18.35 ms, T_unsafe = 10.41 ms. The ratio TOPP / safe-const
is identical to the cd=0.05 case — **20-24% time savings is
scale-invariant** because both `L` and accel-accessible v scale
linearly with cd.

## 9. Implementation sketch

### 9.1 Python-side (planner)

- `klippy/blendquintic.py`: add `v_cap_sampled(N=128)` returning
  `[(s_i, v_cap(s_i))]` on a uniform grid. `v_cap_fn` already
  composes v_max + saturation/centripetal + jerk + shaper
  (`:569-595`); add extruder branch via a wrapper.
- `klippy/blendplanner.py::CornerBlender._emit_blend` — replace the
  ~40-line polyline block (`:194-220`) with:
  (1) sample caps on N=128 grid, (2) merge extruder cap per s using
  linearly interpolated `k(s)`, (3) run TOPP forward-backward with
  state-dependent `a_max_eff(s)`, (4) if (6) violates, feed `v_in_max`
  back to prev via `limit_next_junction_speed`, (5) fit the
  trapezoid-in-s parameters `(accel_end_s, cruise_v, decel_start_s)`,
  (6) emit a single `MOVE_QUINTIC_TRAPEZOID_S` via new
  `trapq_append_quintic_trap`.
- `klippy/blendextruder.py`: refactor `cap_move` into
  `cap_move_at_k(pa, limits, k, v_target)` taking scalar `k`; current
  signature stays as a wrapper for `MOVE_LINEAR` emitters.

### 9.2 C-side (trapq query)

- `klippy/chelper/trapq.h`: add `MOVE_QUINTIC_TRAPEZOID_S` variant to
  D2b's tagged union, with the `struct move_quintic_trap` layout from
  §4.4.
- `klippy/chelper/trapq.c::move_get_coord`: dispatch on `kind`; new
  helpers `invert_trapezoid_s` (phase dispatch + sqrt), `s_to_u`
  (`s_tab` bisect + interp), `quintic_eval` (Horner on monomial
  coeffs). ~40 LOC, all stateless.
- `klippy/chelper/integrate.c::smoother_antiderivatives`: D2a's
  6-moment extension handles polynomial positions. Trap-in-s needs
  the composition `quintic(s(t))` done per phase — two sub-options:
  **(1) dense-sample at smoother quadrature grid** (simple, ~40
  LOC), or **(2) analytical polynomial-in-polynomial composition**
  (degree ≤ 10 per phase, ~120 LOC, exact closed-form moments).
  Recommended: (2) with piecewise handling at the phase boundaries;
  D1 already added piecewise support.

### 9.3 LOC estimate

~500-700 LOC total: ~150 Python planner new, ~60 removed, ~80 C
trapq new, ~120 C integrate new, ~300 tests. 2-3 days focused work
post-D2b.

## 10. Known limitations and caveats

- **Trap-in-s leaves ~10% on the table.** §4.3 showed option (a)
  costs ~12% more time than true TOPP on the worst-case 90°
  geometry. Acceptable if HW shows throughput is adequate; upgrade
  to piecewise (b) is a known Plan 6 path.
- **Degenerate κ (interior κ→0).** Non-physical for a quintic
  Hermite with θ > 0. Local `v_cap` falls to `v_max` there; no
  breakage, just weak cap.
- **PA still works.** Trap-in-s E-position reconstruction is
  monotone in s and t; PA's per-move `axes_r[3]` contract is
  unchanged; `k(s)` linear in s integrates cleanly. No PA
  regression. (Plan 3 already validated this structurally for
  polyline; quintic case is simpler, no per-sub-move k jumps.)
- **Klipper-sim decode.** Offline sim at `~/Developer/klipper-sim/`
  needs `MOVE_QUINTIC_TRAPEZOID_S` support, same follow-up bucket
  as D2b's `MOVE_QUINTIC_CONST_V`.
- **Emergency stop.** Queued `move_t` is honoured; decel-ramp tail
  flushes. No change from today.
- **Conditioning at tiny blends.** At `cd < 0.01 mm`, `Δs < 0.3 μm`
  at N=128. Well within double precision.
- **Non-linear PA cost.** Per-s bisection for tanh/recipr PA models
  gives ~100 μs emit-time per blend at N=128. Below the 1 ms
  budget.
- **G_worst scalar assumption (Pillar 1).** Works for 2D blends
  (`_PLANE_NORMAL = (0,0,1)` at `klippy/blendquintic.py:232`);
  revisit if 3D blends ship.
- **Shaper-bandwidth axis swap.** X and Y projections swap
  dominance mid-blend (normal rotates); `compute_shaper_bounds`
  already handles this correctly per-s.

## 11. Literature anchors

- **Pham (2014).** "A general, fast, and robust implementation of
  the time-optimal path parameterization algorithm." *IEEE Trans.
  Robotics* **30**(6):1533-1540. The TOPP algorithm used here
  directly. Theorem 3.1 proves optimality of the forward-backward
  passes under arbitrary pointwise `v_cap(s)`. §IV describes the
  state-dependent `a_max_eff(s)` extension that §6.3 requires.
- **Bobrow, Dubowsky, Gibson (1985).** "Time-optimal control of
  robotic manipulators along specified paths." *Int. J. Robotics
  Research* **4**(3):3-17. Phase-plane ancestor of TOPP; establishes
  that the time-optimal profile rides `v_cap(s)` at maximum-velocity
  segments, with bang-bang ramps elsewhere. Foundational; cited for
  the bang-bang structure of §1.1.
- **Pham & Nakamura (2012).** "On the structure of the time-optimal
  path parameterization problem with third-order constraints."
  *IEEE ICRA*. Extension of TOPP to jerk constraints. Relevant for
  if/when Pillar 2b incorporates jerk at TOPP-level rather than
  via the separate `v_jerk(s)` cap.
- **Biagiotti & Melchiorri (2008).** *Trajectory Planning for
  Automatic Machines and Robots.* Springer. ISBN 978-3-540-85628-3.
  §5.8 (L¹–L∞ bound for shaped-signal constraints), §5.5 (piecewise
  polynomial trajectories). Anchors §3 composition and the
  trap-in-s representation.
- **Curry & Schoenberg (1966).** "On Pólya frequency functions IV."
  *J. Analyse Math.* **17**:71-107. Piecewise polynomial basis for
  option (b) piecewise-v(s), should we upgrade.
- **Sencer & Tajima (2017).** "Frequency optimal feedrate planning
  along parametric NURBS tool paths for high-speed machining."
  *Robotics and Computer-Integrated Manufacturing* **43**:123-134.
  Closest industrial analogue: feedrate optimisation on parametric
  splines with shaper-bandwidth constraints. §III-D describes a
  scheme equivalent to our trap-in-s fit; §IV reports throughput
  gains in the same 15-25% band we see in §8.

**Note on provenance.** The Wang-Altintas CIRP 2022-2023 references
from prior Plan 5 drafts were retracted per
`REVIEW_2026-04-22.md §2` as unverified. None of those are needed
for this memo's derivation — TOPP plus the standard L¹-L∞ bound are
sufficient.
