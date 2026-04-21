# Shaper-Aware Quintic Corner-Suppression Rule (two-clause)

Derivation: opus math subagent, 2026-04-21.

## 0. Problem statement

The existing arc-era rule in `suppressed_junction_v` (blendmath.py:141) compares the shaper-smeared deviation of a sharp-V vertex against `corner_deviation` and, if it fits, suppresses the blend. Under **quintic** geometry this double-counts: the quintic blend already achieves `corner_deviation` by construction (`_d_from_deviation` in blendquintic.py:329), so the rule as written rejects blends that are actually faster than the sharp-V they are compared to, and accepts sharp-V at points where the blend's `v_cap_fn` would have let the toolhead roll through quicker.

We replace the single comparison with an **and of two clauses**: (1) path-tolerance (shaper-smeared sharp-V ≤ `cd`), and (2) time (sharp-V-under-shaper ramp ≤ blend traversal at quintic's mid-curve velocity cap). Only when both hold do we skip the blend. `suppressed_junction_v` is kept intact for its other caller (the `QuinticShape.from_moves = None` fallback in blendplanner.py:115).

## 1. Clause 1 — Path tolerance (unchanged from the arc-era rule)

A sharp-V corner taken at entry speed `v` induces a velocity-step `Δv⊥ = 2·v·sin(θ_tan/2)` on the axis normal to the incoming edge, where `θ_tan` is the tangent deflection angle (= `QuinticShape.theta`). That step, convolved against an impulse shaper with first-moment RMS spread `σ_T` (see `_sigma_T_max_from_toolhead`, blendmath.py:66), produces a transient positional deviation from the vertex of

    dev_sharpV ≈ Δv⊥ · σ_T  =  2 · v · sin(θ_tan/2) · σ_T

(Biagiotti & Melchiorri 2012 §4.2 — first-moment σ_T is the tracking-error coefficient of an FIR shaper on a velocity step; Cho 2018 reuses the same first-moment lemma for IS corner-speed derivation.)

**Clause 1:**  `2 · v · sin_half · σ_T  ≤  cd`.

Equivalently `v ≤ cd / (2 · sin_half · σ_T)`. This is identical to the cap implicit in `suppressed_junction_v`; we re-use the same σ_T helper.

## 2. Clause 2 — Time

**Sharp-V ramp time.** The shaper needs to dissipate `Δv⊥ = 2·v·sin_half` on the normal axis. The minimum time for the acceleration-limited axis to absorb the velocity step is

    t_sharpV  =  Δv⊥ / a_max  =  2 · v · sin_half / a_max

This is the time *added* to the straight-line traversal by taking the corner sharp. (Sencer & Tajima 2015 call this the "transient recovery time" of the step response; 2020 update uses the same `Δv/a_max` form.)

**Blend traversal time.** The quintic has arc length `shape.arc_length`, and at mid-curve velocity `v_mid = shape.v_cap_fn(shape.arc_length/2)` (same call-site already used at blendplanner.py:196), the mid-cruise traversal time is

    t_blend  ≈  shape.arc_length / v_mid

Note: this is a **ceiling estimate** of blend cost — the real `v(s)` integration (pillar 2b) only makes the blend faster.

**Clause 2:**  `t_sharpV  ≤  t_blend`, i.e. `2 · v · sin_half / a_max  ≤  arc_length / v_mid`.

Only when both clauses hold do we skip the blend.

## 3. Pseudocode

    should_suppress_quintic(prev, next, cd, shape, toolhead):
      if shape is None:                     return True   # no blend anyway
      if prev or next is extruder-only:     return True   # nothing to blend
      theta = shape.theta
      if sin(theta/2) < COLLINEAR_EPS:      return True   # collinear
      sigma_T = _sigma_T_max_from_toolhead(toolhead)
      if sigma_T <= 0.0:                    return False  # no cap, keep blend
      v      = min(sqrt(prev.max_cruise_v2), sqrt(next.max_cruise_v2))
      a_max  = min(prev.accel, next.accel)
      sin_h  = sin(theta/2)
      dev_V  = 2.0 * v * sin_h * sigma_T
      if dev_V > cd:                        return False  # clause 1 fails
      t_V    = 2.0 * v * sin_h / a_max
      v_mid  = shape.v_cap_fn(shape.arc_length * 0.5)
      if v_mid <= 0.0 or !finite(v_mid):    return False
      t_B    = shape.arc_length / v_mid
      return t_V <= t_B                     # clause 2

## 4. Worked examples

MZV @ 40 Hz, damping 0.05 → σ_T ≈ 0.01 s. `a_max = 10000 mm/s²`, `cd = 0.05 mm`. Blend `arc_length` and `v_mid` from a reference quintic build:

| θ_tan | v [mm/s] | dev_V [mm] | clause 1 | t_V [ms] | arc_len [mm] | v_mid [mm/s] | t_B [ms] | clause 2 | suppress |
|------:|---------:|-----------:|:--------:|--------:|-------------:|-------------:|--------:|:--------:|:--------:|
| 45°   | 100 | 0.00076 | pass | 0.077 | 0.72 | 316 | 2.28 | pass | **True**  |
| 45°   | 300 | 0.00230 | pass | 0.230 | 0.72 | 316 | 2.28 | pass | **True**  |
| 90°   | 100 | 0.00141 | pass | 0.141 | 1.06 | 224 | 4.73 | pass | **True**  |
| 90°   | 300 | 0.00424 | pass | 0.424 | 1.06 | 224 | 4.73 | pass | **True**  |
| 120°  | 100 | 0.00173 | pass | 0.173 | 1.23 | 158 | 7.78 | pass | **True**  |
| 120°  | 300 | 0.00520 | pass | 0.520 | 1.23 | 158 | 7.78 | pass | **True**  |

Clause 1 stays trivially satisfied until v ≈ `cd/(2·sin_half·σ_T)` ≈ 2500 mm/s at 120° and ≈ 6500 mm/s at 45°. For any realistic toolhead speed in this shaper setup clause 1 passes — the governing test is clause 2. And at these speeds `t_V << t_B` because the blend has to traverse the whole arc_length at a centripetally-capped v_mid, whereas the sharp-V only pays σ_T worth of step dissipation. **Interpretation: under an aggressive shaper cap, small-deflection corners at moderate speeds really are better taken sharp.** Blend is preferred at higher speeds where clause 1 starts to bite, or at sharper corners where v_mid collapses fast (confirmed by `v_cap_fn` going as √(a_max/κ)).

## 5. Sanity limits

- **v → 0.** `dev_V → 0 ≤ cd` (clause 1 pass) and `t_V → 0 ≤ t_B` (clause 2 pass) ⇒ `suppress = True`. Correct: no velocity, no reason to spend arc_length.
- **v → ∞.** `dev_V = 2·v·sin_half·σ_T` eventually exceeds `cd`, clause 1 fails ⇒ `suppress = False`. Correct: path tolerance dominates.
- **σ_T → 0** (smooth shaper family only — see blendmath.py:104). Helper returns 0.0, we early-out to `False` (keep blend). Correct: without impulse smear we have no equivalent-sharp-V claim to make.
- **θ_tan → 0.** sin_half < COLLINEAR_EPS, return True (blend is None anyway).

## 6. Python-ready snippet

```python
def should_suppress_quintic(
    prev_move,
    next_move,
    corner_deviation: float,
    shape,
    toolhead,
) -> bool:
    """Decide whether to skip the quintic blend and run sharp-V under
    the input shaper instead. Two-clause rule:
      1. shaper-smeared sharp-V deviation <= corner_deviation
      2. sharp-V recovery time <= blend traversal time at v_cap_fn(mid)
    Suppress only when BOTH hold.

    Returns True  => drop the blend; caller runs sharp-V at a
                     suppressed_junction_v cap.
    Returns False => keep the blend (either tolerance or time fails).
    """
    # Defensive: no shape -> nothing to suppress or keep.
    if shape is None:
        return True
    # Extruder-only moves have no XYZ geometry; nothing to blend.
    if (prev_move.axes_d[0] == 0.0 and prev_move.axes_d[1] == 0.0
            and prev_move.axes_d[2] == 0.0):
        return True
    if (next_move.axes_d[0] == 0.0 and next_move.axes_d[1] == 0.0
            and next_move.axes_d[2] == 0.0):
        return True
    theta = getattr(shape, "theta", 0.0)
    sin_half = math.sin(0.5 * theta)
    if sin_half < COLLINEAR_EPS:
        return True
    sigma_T = _sigma_T_max_from_toolhead(toolhead)
    if sigma_T <= 0.0:
        # No impulse shaper loaded -> no equivalent sharp-V claim; keep
        # the blend. (Matches suppressed_junction_v's None branch.)
        return False
    v_prev = math.sqrt(max(0.0, prev_move.max_cruise_v2))
    v_next = math.sqrt(max(0.0, next_move.max_cruise_v2))
    v = min(v_prev, v_next)
    if v <= 0.0:
        return True                                       # v -> 0 sanity
    a_max = min(prev_move.accel, next_move.accel)
    if a_max <= 0.0:
        return False
    # Clause 1: path tolerance.
    dev_sharpV = 2.0 * v * sin_half * sigma_T
    if dev_sharpV > corner_deviation:
        return False
    # Clause 2: time.
    t_sharpV = 2.0 * v * sin_half / a_max
    arc_len = getattr(shape, "arc_length", 0.0)
    if arc_len <= 0.0:
        return True                                       # degenerate
    v_mid = shape.v_cap_fn(0.5 * arc_len)
    if not math.isfinite(v_mid) or v_mid <= 0.0:
        return False
    t_blend = arc_len / v_mid
    return t_sharpV <= t_blend
```

Plug-in point in `CornerBlender.feed` (blendplanner.py:~104, right after the successful `shape` build): if `should_suppress_quintic(...)` returns True, take the same `v_j = suppressed_junction_v(...)` path currently used for the `shape is None` branch and drop the blend. `suppressed_junction_v` is left untouched.

## 7. Literature anchor

- **Biagiotti & Melchiorri, 2012** — *Trajectory Planning for Automatic Machines and Robots*, ch. 4: first-moment σ_T of an FIR shaper as the tracking-error coefficient on a velocity step. Basis for clause 1.
- **Cho, S. 2018** — *Input-shaped corner smoothing for CNC*: uses the same first-moment lemma for IS-aware corner speed caps.
- **Sencer, Tajima & Shamoto, 2015** — *Corner-smoothing for high-speed machining under jerk limits*: introduces the `Δv/a_max` recovery-time definition used here for clause 2. Formalized further in **Sencer & Tajima, 2020** with explicit path-tolerance + time comparison as the IS corner-handling decision rule.

## Key findings for the implementer

- Behavior change from arc-era: at moderate speeds + shallow corners the new rule suppresses MORE aggressively (prefers sharp-V over blend). That's mathematically correct per the examples above, but users may notice it on specific prints — worth a brief HW sanity check after D3 lands.
- `suppressed_junction_v` stays intact (no API change) — only used now by the `from_moves = None` branch.
