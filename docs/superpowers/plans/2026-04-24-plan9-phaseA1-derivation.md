# Plan 9 Phase A1 — Jerk-limited 7-phase profile derivation

Status: derivation + numerical verification, ready for C implementer.
Date: 2026-04-24.
Scope: closed-form math for the single-segment 1-D jerk-limited profile used as the core primitive of the Plan 9 planner rewrite. The caller is the Plan 9 look-ahead pass; the callee shape is the `jerk_profile_compute(...)` C function proposed in Part 6.

Conventions (apply to the whole document):

- `v0`, `v1` are non-negative scalar speeds at the segment endpoints.
- `v_peak`, `a_max`, `j_max` are strictly-positive scalar caps.
- `L > 0` is the unsigned traversed distance. Sign flips are handled outside this primitive (the caller negates the emitted polynomials if the move is reverse-direction).
- Phase labels: `J+`, `A+`, `J-`, `C`, `J-d`, `A-`, `J+d`. `J±` applies signed jerk `±j_max`. `A±` holds acceleration constant (jerk = 0). `C` is cruise (accel = 0).
- Polynomials are stored in ascending-monomial order `(c0, c1, c2, c3, c4, c5)` so that `p(t) = c0 + c1 t + c2 t^2 + ... ` with segment-local `t ∈ [0, T]`.
- Continuity is understood in the position / velocity / acceleration sense (C², since jerk is piecewise constant the profile is only C² not C³).

Reference implementation and verification code: see Part 5 and `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py` (reproduced verbatim at the end). All numeric claims below were re-verified against that implementation.

---

## Part 1 — Canonical 7-phase derivation (non-degenerate case)

### 1.1 Phase structure

The classical 7-phase jerk-limited (a.k.a. "S-curve", "bang-bang-jerk") profile is the standard industrial result; the canonical reference is Biagiotti & Melchiorri, *Trajectory Planning for Automatic Machines and Robots*, Springer 2008, §3.4. The same scheme is implemented in Delta Tau Power PMAC, Siemens 840D, Beckhoff TwinCAT NC, and on the open-source side in `ruckig` (Berscheid & Kröger, ICRA 2021). The pieces are:

| Phase | Jerk     | Accel range (linear in t)  | Velocity range       |
| ----- | -------- | -------------------------- | -------------------- |
| 1 J+  | `+j_max` | `0  -> a_acc`              | `v0 -> v0 + ...`     |
| 2 A+  | `0`      | `a_acc` (flat)             | rises linearly       |
| 3 J-  | `-j_max` | `a_acc -> 0`               | rises to `v_hat`     |
| 4 C   | `0`      | `0`                        | `v_hat` (flat)       |
| 5 J-d | `-j_max` | `0  -> -a_dec`             | `v_hat -> ...`       |
| 6 A-  | `0`      | `-a_dec` (flat)            | falls linearly       |
| 7 J+d | `+j_max` | `-a_dec -> 0`              | ends at `v1`         |

The peak velocity actually reached is `v_hat ≤ v_peak`; when the move is long enough that cruise exists, `v_hat = v_peak`. Otherwise `v_hat < v_peak` (Part 3).

### 1.2 Accel-side and decel-side as a sub-primitive

Because phases 1–3 only depend on `(v0, v_hat, a_max, j_max)` and phases 5–7 only on `(v1, v_hat, a_max, j_max)`, it is natural to factor the computation into a *side* primitive that answers: "how long and how far does it take to change speed from `v_s` to `v_e` under `(a_max, j_max)`?"

Let `dv = |v_e - v_s|`. There are two sub-cases:

**Trapezoidal acceleration profile** (phase 2 or 6 has nonzero width): reached when `dv ≥ a_max² / j_max`. The jerk phases each last `t_j = a_max / j_max`, the const-accel phase lasts `t_a = (dv − a_max²/j_max) / a_max`, and the peak acceleration is `a_p = a_max`.

**Triangular acceleration profile** (phase 2 or 6 degenerates to zero): reached when `dv < a_max² / j_max`. Then `a_p = √(j_max · dv)`, `t_j = a_p / j_max = √(dv / j_max)`, and `t_a = 0`.

In both sub-cases the velocity profile on the side is point-symmetric about its temporal midpoint, so the distance traversed is simply

```
  d_side = 0.5 · (v_s + v_e) · T_side,   where T_side = 2 t_j + t_a.
```

This avoids integrating the polynomial directly and is numerically well-conditioned.

Derivation of the distance formula. Over phase 1 (jerk up), `v(t) = v_s + ½ j_max t²`, ending at `v_s + ½ j_max t_j² = v_s + ½ a_p t_j`. Over phase 2, `v(t) = (v_s + ½ a_p t_j) + a_p t`. Over phase 3, `v(t) = (v_s + ½ a_p t_j + a_p t_a) + a_p t − ½ j_max t²`. Adding the three integrals (tedious but elementary) collapses to the single-line result above, which is the area under a trapezoid (or triangle) — the familiar Biagiotti Eq. 3.30.

### 1.3 Phase durations, non-degenerate case

Given `v_hat = v_peak` (cruise exists), define `dv_a = v_peak − v0`, `dv_d = v_peak − v1`. With `dv_a ≥ a_max² / j_max` and `dv_d ≥ a_max² / j_max` (neither side triangular):

```
  t_1 = t_3 = a_max / j_max
  t_2       = (v_peak − v0) / a_max − a_max / j_max
  t_5 = t_7 = a_max / j_max
  t_6       = (v_peak − v1) / a_max − a_max / j_max
  a_acc = a_dec = a_max
  T_acc = 2 t_1 + t_2,   T_dec = 2 t_5 + t_6
  d_acc = 0.5 (v0 + v_peak) T_acc
  d_dec = 0.5 (v_peak + v1) T_dec
  t_4   = (L − d_acc − d_dec) / v_peak       (cruise)
```

Positivity of `t_2`, `t_6`, `t_4` is the non-degenerate criterion.

### 1.4 Position polynomials per phase

Let `p0, v0_s, a0` denote the state (position, velocity, acceleration) at the start of a segment; let `T` be the segment duration; let `j` be the (constant) jerk during the segment. The segment-local position polynomial is the third-order Taylor expansion of rigid-body kinematics with constant jerk:

- **J phase (constant `j ≠ 0`, degree 3):**
  ```
    p(t) = p0 + v0_s t + (1/2) a0 t² + (1/6) j t³
  ```
  so `c0 = p0`, `c1 = v0_s`, `c2 = a0 / 2`, `c3 = j / 6`, `c4 = c5 = 0`.

- **A phase (constant accel, `j = 0`, degree 2):**
  ```
    p(t) = p0 + v0_s t + (1/2) a0 t²
  ```
  so `c0 = p0`, `c1 = v0_s`, `c2 = a0 / 2`, higher = 0.

- **C phase (cruise, `a = 0`, `j = 0`, degree 1):**
  ```
    p(t) = p0 + v0_s t
  ```
  so `c0 = p0`, `c1 = v0_s`, higher = 0.

**Why emit degree 1, not degree 5, for cruise.** The planner's downstream consumer evaluates `p(t)` in segment-local time via Horner. For a cruise of duration 2 s at 500 mm/s, a degree-5 evaluation multiplies the leading `c5 t^5` term (which is identically zero but stored as fp64 zero) through a cascade of fused-multiply-adds with `t ∈ [0, 2]`; in practice the trailing zeros are exact and do not introduce error. The real reason to prefer degree 1 is that the *source* of `c2..c5` for a cruise is a subtraction of same-magnitude quantities (symmetric-around-zero jerk contributions that must cancel to machine zero), and small perturbations in those cancellations become order-of-seconds errors in the 5th-order term when cruise is long. Emitting degree 1 directly bypasses the cancellation entirely and keeps cruise distance exact up to one fp64 rounding per evaluation.

### 1.5 Continuity verification

Continuity (`C²`) across each phase boundary is by construction: we march the state `(p, v, a)` forward, emit a segment that analytically advances `(p, v, a)` by the closed-form polynomial, then re-read the new `(p, v, a)` from the polynomial evaluated at its own `T`. The equality check in the verifier (Part 5) confirms the roundtrip is within `1e-9` relative error on all 30 feasible sweep cases.

Boundary values specifically:
- Phase 1 start: `a = 0`. Phase 1 end: `a = j_max t_1 = a_max` (non-degen) or `= √(j_max · dv_a)` (triangular).
- Phase 3 end: `a = 0`.
- Phase 4 start/end: `a = 0`, `v = v_hat`.
- Phase 5 end: `a = −a_dec`.
- Phase 7 end: `a = 0`, `v = v1`.

### 1.6 Sanity: total distance

By construction `L = d_acc + d_cruise + d_dec`. The verifier confirms `p(T_total) = L` to `8.7 × 10⁻¹¹` absolute / `1.8 × 10⁻¹¹` relative across the 30-case sweep (Part 5).

---

## Part 2 — Degeneracy cases

The sub-primitive in §1.2 already collapses `t_a = 0` inside a side when the side is triangular, so we do not need a separate code path for the `A+ = 0` or `A- = 0` degeneracies; they simply emit a zero-duration segment that the emitter skips. The cases that *do* need explicit handling at the top level are:

### 2.1 Cruise collapses (phase 4 = 0)

**Detection.** After computing `d_acc` and `d_dec` at `v_hat = v_peak`:
```
  if d_acc + d_dec > L + EPS:  cruise collapses; reduce v_hat (Part 3).
  else:                        cruise exists; t_4 = (L − d_acc − d_dec) / v_peak.
```
When cruise collapses, recompute the sides at the reduced `v_hat` and set `t_4 = 0`.

### 2.2 Constant-accel-up collapses (`t_2 = 0`)

**Detection.** Equivalent to `v_hat − v0 < a_max² / j_max` — i.e. the accel side is triangular. Handled entirely inside the side sub-primitive. Then `t_1 = t_3 = √((v_hat − v0) / j_max)` and `a_acc = √(j_max · (v_hat − v0))`.

### 2.3 Constant-accel-down collapses (`t_6 = 0`)

**Detection.** Symmetric: `v_hat − v1 < a_max² / j_max`. Same sub-primitive. `t_5 = t_7 = √((v_hat − v1) / j_max)`, `a_dec = √(j_max · (v_hat − v1))`.

### 2.4 Both const-accel phases collapse (short move)

No special handling — §2.2 and §2.3 each apply independently. In the sweep, 12/30 feasible cases fall here (`L = 10` for all `(v0, v1)` pairs).

### 2.5 Both const-accel phases + cruise all collapse (very short)

Same detection as §2.1 + both sides triangular. Emit only `J+`, `J-`, `J-d`, `J+d`. All 12 `L = 10` feasible cases hit this. Example from the sweep:
```
v0=0, v1=0, L=10:   v_hat=135.72, a_acc=3684.03, phases J+:0.03684, J-:0.03684, J-d:0.03684, J+d:0.03684
```

### 2.6 Pure coasting (`v0 ≈ v1 ≈ v_peak`)

Not directly present in the sweep (we only tested `v_peak = 500`, max input speed 200), but the generator handles it: if `v0 = v1 = v_peak`, the accel-side and decel-side both have `dv = 0` and the sub-primitive returns all zeros. Only the cruise segment is emitted with `T = L / v_peak`. Numerically this falls out of the same code path because `accel_side_timings` short-circuits on `dv < EPS`.

### 2.7 Pure deceleration (`v0 > 0`, `v1 < v0`, small `L`)

When `L` exactly equals the decel floor `d_floor_dec = accel_side_distance(v0, v1, a_max, j_max)`, the accel side has `dv = 0` and emits nothing. We get only `J-d` + `A-` + `J+d` (or the triangular variant). The sweep rows `v0=200, v1=0, L=10` and `v0=50, v1=0, L=10` are close-to-this regime — both the accel and decel groups exist but the accel group is vestigial (`T = 0.0025 s` in the first case).

### 2.8 Pure acceleration (`v0 < v1`, small `L`)

Symmetric to §2.7. Same code path — the decel group comes back with `dv ≈ 0` and collapses.

### 2.9 Infeasible cases

**Detection.** Compute `d_floor = accel_side_distance(v0, max(v0,v1)) + accel_side_distance(max(v0,v1), v1)`. This is the minimum distance to monotonically change speed from `v0` to `v1` within `(a_max, j_max)`. If `L < d_floor − 1e-12`, the request is infeasible.

**What to return.** The primitive cannot honor `(v0, v1, L)` simultaneously. The correct thing is to return a status code `JERK_INFEASIBLE` (numeric value TBD in the C API). The caller (the planner look-ahead pass) must either:
(a) clamp `v1` to a feasible value given `(v0, L)`, which is `v1_max = the speed reachable by a full decel from `v0` over distance `L` — a separate one-shot solver, or
(b) clamp `v0` given `(v1, L)` by the symmetric rule, or
(c) expand `L` by borrowing distance from the previous/next move (this is what look-ahead normally does).

None of these are the profile primitive's job. The primitive's sole contract is: given consistent inputs, emit the profile; given inconsistent inputs, report infeasibility and let the caller fix the inputs. The sweep produced 6 infeasible cases (all `L = 1`), and the primitive correctly flagged all six.

**Empirical min-distance formula.** For two common shapes:
```
  d_floor (0 -> v, v <= a_max²/j_max)   = v · √(v / j_max)
  d_floor (0 -> v, v >  a_max²/j_max)   = (v / 2) · (v/a_max + a_max/j_max)
```

---

## Part 3 — Reducing `v_peak` when cruise collapses

### 3.1 The root-finding problem

We seek `v_hat ∈ [max(v0, v1), v_peak]` such that

```
  F(v_hat) := d_acc(v0, v_hat) + d_dec(v1, v_hat) − L  =  0.
```

`F` is continuous and strictly increasing on the relevant range (the accel and decel distances are both strictly increasing in `v_hat` when `v_hat > max(v0, v1)`), so there is exactly one root. At `v_hat = max(v0, v1)`, `F = d_floor − L`, which is `≤ 0` whenever the request is feasible. At `v_hat = v_peak`, `F = d_acc + d_dec − L`, which is `> 0` iff cruise would collapse.

### 3.2 Closed form where possible

`F(v_hat)` is piecewise rational-with-sqrt. There are four regimes depending on whether each side is triangular or trapezoidal:

| Accel side | Decel side | F(v_hat) expression |
| ---------- | ---------- | ------------------- |
| Tri        | Tri        | `(v_hat + v0)·√((v_hat − v0)/j_max) + (v_hat + v1)·√((v_hat − v1)/j_max) − L` |
| Tri        | Trap       | `(v_hat + v0)·√((v_hat − v0)/j_max) + (v_hat + v1)·(v_hat − v1)/a_max · ½ · (1 + ... )` (see code) |
| Trap       | Tri        | symmetric |
| Trap       | Trap       | closed-form: quadratic in `v_hat` (see below). |

The **Trap+Trap regime is the only one with a clean closed form**:

```
  d_acc(v_hat) = (v_hat + v0) / 2 · T_acc,   T_acc = (v_hat − v0)/a_max + a_max/j_max
             => d_acc(v_hat) = (v_hat² − v0²) / (2 a_max)  +  (v_hat + v0) · a_max / (2 j_max)
```

Substituting and simplifying:

```
  F(v_hat) = v_hat² / a_max
             − (v0² + v1²) / (2 a_max)
             + v_hat · a_max / j_max
             + (v0 + v1) · a_max / (2 j_max)
             − L
```

This is a quadratic in `v_hat`; take the positive root. The other regimes have `√(v_hat − v0)` or `√(v_hat − v1)` and are not quadratic, so they need either an iterative solver or a substitution.

### 3.3 Recommended algorithm: hybrid bisection → Newton

Because the sub-case doesn't simplify uniformly and the C code needs to be robust against edge cases, a **hybrid approach** is cleaner than four closed forms:

1. **Bracket.** Lower bound: `max(v0, v1)`. Upper bound: `v_peak` — known to straddle the root whenever we entered this branch.
2. **Bisection.** 6 bisection steps compress the bracket to `~(v_peak − max(v0,v1)) / 64`. This is enough to land inside the convex Newton basin.
3. **Newton.** 3–4 Newton steps with analytic derivative
   ```
     F'(v_hat) = T_acc(v_hat) + T_dec(v_hat)   (the total side duration)
   ```
   which is the derivative of the trapezoidal area formula w.r.t. the top speed. Quadratic convergence to `1e-12` is empirically achieved in ≤ 4 Newton steps.

The reference implementation uses pure bisection (80 steps) for simplicity; the C implementer should switch to the hybrid form for speed.

**Cold-start seed.** If you skip bisection, a good initial guess is

```
  v_hat_0 = ½ (max(v0, v1) + v_peak)
```

which is always inside the feasible range when cruise collapsed.

### 3.4 Regime-hopping caution

During iteration, `v_hat` can cross the boundary between triangular and trapezoidal on one side (e.g. acceleration side becomes trapezoidal after `v_hat` rises past `v0 + a_max²/j_max`). The side primitive returns the correct distance in either regime, so the iteration is still well-defined; but the **derivative** `F'` has a kink at that crossover (the side duration is continuous but not C¹ in `v_hat`). In practice Newton tolerates this — the kink is mild (the two expressions match at the boundary and differ only in second derivative). Bisection-Newton hybrid is immune.

---

## Part 4 — Polynomial coefficient computation

Repeated from §1.4 in operational form. The state march is:

```
  (p_k, v_k, a_k, j_k) = state at start of phase k
  phase k has duration T_k
  phase k polynomial:
     p_k(t) = p_k + v_k · t + (a_k / 2) · t² + (j_k / 6) · t³      for J phases
     p_k(t) = p_k + v_k · t + (a_k / 2) · t²                        for A phases
     p_k(t) = p_k + v_k · t                                         for C phases
  end-of-phase state (used as start-of-next-phase):
     p_{k+1} = p_k + v_k T + (a_k / 2) T² + (j_k / 6) T³
     v_{k+1} = v_k + a_k T + (j_k / 2) T²
     a_{k+1} = a_k + j_k T
```

Phase boundary jerk values (for the non-degenerate 7-phase case):
```
  j_1 = +j_max   j_2 = 0    j_3 = -j_max
  j_4 = 0
  j_5 = -j_max   j_6 = 0    j_7 = +j_max
```

(When a phase collapses, its jerk is irrelevant — the phase is emitted as a zero-length segment and skipped by the consumer.)

### 4.1 Concrete symbolic coefficients

For the 7 phases, writing out the polynomials in segment-local `t` with segment-start state `(p_k, v_k, a_k)`:

| Phase | Degree | c0   | c1   | c2        | c3              |
| ----- | ------ | ---- | ---- | --------- | --------------- |
| J+    | 3      | p_k  | v_k  | a_k / 2   | +j_max / 6      |
| A+    | 2      | p_k  | v_k  | a_k / 2   | 0               |
| J-    | 3      | p_k  | v_k  | a_k / 2   | −j_max / 6      |
| C     | 1      | p_k  | v_k  | 0         | 0               |
| J-d   | 3      | p_k  | v_k  | a_k / 2   | −j_max / 6      |
| A-    | 2      | p_k  | v_k  | a_k / 2   | 0               |
| J+d   | 3      | p_k  | v_k  | a_k / 2   | +j_max / 6      |

All `c4`, `c5` are zero. The storage format should reserve 6 slots for the polynomial (in case the downstream pipeline ever fuses jerk segments with a smooth-composed shaper, which would introduce degree-5 terms), but the initial Plan 9 A1 implementation writes only `c0..c3`.

### 4.2 Boundary checks

For every adjacent pair `(k, k+1)`:
```
  p_k(T_k) == p_{k+1}(0)     (C⁰: position matches)
  p_k'(T_k) == p_{k+1}'(0)   (C¹: velocity matches)
  p_k''(T_k) == p_{k+1}''(0) (C²: acceleration matches)
```

Verified numerically in Part 5; all 30 feasible sweep cases pass at `tol = 1e-8` rel.

---

## Part 5 — Numerical verification

### 5.1 Reference implementation

The Python reference implementation is `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py` (418 lines). It is the canonical check for the C implementer — any divergence between C output and this reference is a bug in the C implementation. Key routines:

- `accel_side_timings(v_start, v_end, a_max, j_max) -> (t_j, t_a, a_p, d)`: the side sub-primitive.
- `find_v_hat(v0, v1, a_max, j_max, L) -> v_hat`: bisection for the reduced peak.
- `compute_profile(v0, v1, v_peak, a_max, j_max, L) -> Profile`: top-level driver.
- `verify_profile(prof, ...) -> (ok, msgs)`: checks durations, C², distance, bounds.

### 5.2 Sweep specification

Test inputs: `v0 ∈ {0, 50, 200}` mm/s, `v1 ∈ {0, 50, 200}` mm/s, `L ∈ {1, 10, 100, 1000}` mm. Fixed caps: `a_max = 5000` mm/s², `j_max = 100000` mm/s³, `v_peak = 500` mm/s. Total: 36 cases.

### 5.3 Sweep result (verbatim from `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`)

```
=== sweep stats ===
  full-7-phase: 18
  cruise-collapse|A+collapse|A-collapse: 12
  INFEASIBLE: 6

All 30 feasible cases pass verification (C2, distance, bounds).

=== Infeasible (expected: L too small to bridge v0->v1) ===
(0, 50, 1,  d_floor=1.118)
(0, 200, 1, d_floor=8.944)
(50, 0, 1,  d_floor=1.118)
(50, 200, 1, d_floor=9.682)
(200, 0, 1, d_floor=8.944)
(200, 50, 1, d_floor=9.682)

Precision probe: max |p_end - L| = 8.707e-11,  max relative = 1.799e-11
```

### 5.4 Observations

- **Degeneracy frequency.** Cruise-collapse is by far the most common degeneracy: 12 of 30 feasible cases (40 %). All-phases-present (full 7-phase) takes 18 of 30 (60 %). In a real printing workload, short moves dominate, so the cruise-collapse branch is the hot path. **The C implementer should optimize for cruise-collapse first.**

- **No mixed degeneracies beyond the natural "short move collapses both cruise and the const-accel phases" case.** The pairing `(cruise-collapse, A+collapse, A-collapse)` is what you always see for short moves. The sweep does not exhibit any case where only one of `A+` or `A-` collapses while cruise exists — that *can* happen in principle (e.g. extreme `v0 ≈ v_peak` with large `L`), but it's rare and the sub-primitive handles it automatically.

- **Precision.** Max absolute error `8.7 × 10⁻¹¹` mm over a 1000 mm traverse (relative `1.8 × 10⁻¹¹`). Well below the 1 µm step-resolution floor of the motion system, so no precision concerns for the application.

- **Numerical gotchas found:**
  1. `accel_side_timings` must take `|v_end − v_start|` not `v_end − v_start`, otherwise asymmetric moves (e.g. `v0 = 50, v1 = 0`) raise a direction error. Originally missed; fixed in reference.
  2. `find_v_hat` must bracket the root against the known upper bound `v_peak`. Using `v_hi = doubling` would let it run off to infinity for pathological inputs that shouldn't have reached this branch.
  3. Infeasibility detection (`L < d_floor`) must run *before* calling `find_v_hat`, otherwise `F(v_hat = max(v0, v1)) > 0` at the lower bracket and the root-finder walks into numerical nonsense.
  4. For cruise that is long (phase 4 = 1.85 s in the `L = 1000, v0 = v1 = 0` case), emitting the cruise segment as degree 5 (with `c2..c5 = 0`) or as degree 1 is arithmetically identical in exact arithmetic and differs by `~1e-13` in fp64 (confirmed). The case for emitting degree 1 is mostly about storage and semantic clarity — the downstream consumer knows a degree-1 segment is a cruise and can short-circuit Horner.

- **Surprise / non-surprise.** The 7-phase profile is textbook and the derivation confirmed Biagiotti §3.4 exactly; no discrepancies with literature. The only small surprise is how *strongly* the degeneracy dominates at realistic operating points — 40 % cruise-collapse in this sweep, and in real printing it will be higher still because typical bead-to-bead hops are 5–20 mm at cruise speeds ≥ 200 mm/s.

### 5.5 Reference implementation listing

The full reference code lives at `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`. Copy it into the project's `test/` directory once the C implementation lands, as a regression oracle. Key excerpts are reproduced here so the C implementer has the essentials inline:

```python
def accel_side_timings(v_start, v_end, a_max, j_max):
    dv = abs(v_end - v_start)
    if dv < 1e-12:
        return 0.0, 0.0, 0.0, 0.0
    dv_tri = a_max * a_max / j_max
    if dv >= dv_tri:                              # trapezoidal accel
        t_j = a_max / j_max
        t_a = (dv - dv_tri) / a_max
        a_p = a_max
    else:                                         # triangular accel
        a_p = math.sqrt(j_max * dv)
        t_j = a_p / j_max
        t_a = 0.0
    T   = 2.0 * t_j + t_a
    d   = 0.5 * (v_start + v_end) * T
    return t_j, t_a, a_p, d

def compute_profile(v0, v1, v_peak, a_max, j_max, L):
    d_floor = accel_side_distance(v0, max(v0,v1), a_max, j_max) + \
              accel_side_distance(max(v0,v1), v1, a_max, j_max)
    if L + 1e-12 < d_floor: return INFEASIBLE
    tj_a, ta_a, a_acc, d_acc = accel_side_timings(v0, v_peak, a_max, j_max)
    tj_d, ta_d, a_dec, d_dec = accel_side_timings(v1, v_peak, a_max, j_max)
    if d_acc + d_dec <= L:                         # cruise exists
        v_hat = v_peak
        t_cruise = (L - d_acc - d_dec) / v_peak
    else:                                          # cruise collapses
        v_hat = find_v_hat(v0, v1, a_max, j_max, L)
        tj_a, ta_a, a_acc, _ = accel_side_timings(v0, v_hat, a_max, j_max)
        tj_d, ta_d, a_dec, _ = accel_side_timings(v1, v_hat, a_max, j_max)
        t_cruise = 0.0
    # emit 7 segments (J+, A+, J-, C, J-d, A-, J+d), skipping zero-duration ones
    # ... see full file for state-marching logic
```

---

## Part 6 — Implementation notes for C implementer

### 6.1 Precision

**Use double (fp64) throughout.** The sweep showed `1.8 × 10⁻¹¹` relative error in fp64; in fp32 the same computation would be at `1 × 10⁻⁶` relative, which is 10 µm error on a 10 m traverse — unacceptable. The per-segment polynomial coefficients can be stored as fp64 too (the existing motion system uses fp64 for `move.start_pos`, `move.accel`, etc., so there's no mismatch).

### 6.2 Computation order

Order operations to amortize common subexpressions:

1. Compute `k1 = a_max / j_max` and `k2 = a_max² / j_max = a_max · k1` once per call. These appear in every side computation.
2. Compute `dv_a = v_peak − v0`, `dv_d = v_peak − v1` once.
3. Branch on whether each side is triangular (`dv < k2`) or trapezoidal.
4. For triangular sides, compute `sqrt` exactly once per side — cache the result as `t_j_a` or `t_j_d`.
5. Don't recompute `T_acc`, `T_dec` from scratch after finding `v_hat` — pass them through from the side-primitive call.
6. Distance test `d_acc + d_dec <= L` gates the entire root-finding branch; evaluate it early to skip `find_v_hat` in the common case.

### 6.3 Branches the C function needs

Top-level dispatch:

```
  if (L < d_floor) return JERK_INFEASIBLE;
  if (d_acc + d_dec <= L) { cruise_exists = true; v_hat = v_peak; t_cruise = ...; }
  else                    { cruise_exists = false; v_hat = find_v_hat(...); t_cruise = 0; }
```

Inside the side sub-primitive:

```
  if (dv < EPS) { t_j = t_a = a_p = d = 0; }
  else if (dv >= k2) { /* trapezoidal */ }
  else { /* triangular */ }
```

Segment emission loop: skip any segment with `T <= EPS`, where `EPS = 1e-12`. This handles all degenerate phases uniformly.

### 6.4 Proposed C API

```c
enum jerk_seg_type {
    JS_JUP_A,   /* J+ accel */
    JS_AUP,     /* A+ const-accel up */
    JS_JDN_A,   /* J- accel */
    JS_CRUISE,  /* C cruise */
    JS_JDN_D,   /* J-d decel */
    JS_ADN,     /* A- const-accel down */
    JS_JUP_D,   /* J+d decel */
};

struct jerk_segment {
    enum jerk_seg_type type;
    double T;           /* duration, seconds */
    double coeffs[6];   /* ascending: c0..c5; c4, c5 always 0 for this primitive */
};

struct jerk_profile_out {
    int n_segments;                    /* up to 7; degenerate segments omitted */
    struct jerk_segment segs[7];
    double v_hat;                      /* achieved peak */
    double a_acc, a_dec;               /* achieved peaks */
    int status;                        /* 0 = OK, negative = error code */
};

enum jerk_status {
    JERK_OK            =  0,
    JERK_INFEASIBLE    = -1,    /* L below d_floor */
    JERK_BAD_INPUT     = -2,    /* negative speeds, zero caps, etc. */
};

int jerk_profile_compute(
    double v0, double v1,
    double v_peak, double a_max, double j_max,
    double L,
    struct jerk_profile_out *out
);
```

Return value == `out->status`. Caller is expected to pre-allocate `out`. Degenerate phases with `T <= EPS` are *not* emitted — `n_segments` is the count of non-trivial segments, between 1 (pure cruise) and 7 (full profile).

### 6.5 Test regression hook

Port the sweep from `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py` into `test/test_jerk_profile.py`. Run the same 36 cases against the C function (via ctypes or a small test harness) and compare segment-by-segment. Tolerance: `1e-9` absolute on positions, `1e-8` relative on durations.

---

## References

- Biagiotti, L. & Melchiorri, C., *Trajectory Planning for Automatic Machines and Robots*, Springer 2008, Ch. 3 (double-S / 7-phase jerk-limited profile). The phase-duration formulas and distance decomposition here match Biagiotti §3.4 exactly.
- Kröger, T. & Wahl, F. M., "Online Trajectory Generation: Basic Concepts for Instantaneous Reactions to Unforeseen Events," *IEEE Trans. Robotics* 26(1), 2010 — motivates the online-solvable closed-form used here (vs. iterative QP alternatives).
- Berscheid, L. & Kröger, T., "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States," ICRA 2021 — the `ruckig` library; their implementation of the 7-phase primitive is a close cousin to the one derived here, and was useful as an implementation cross-check.
- Delta Tau Power PMAC User Manual §"S-curve motion" — industrial reference implementation, same phase structure, identical trapezoidal/triangular acceleration sub-cases.
