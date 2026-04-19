# Sub-spec 6d — G² Quintic Hermite Geometry Module

**Date:** 2026-04-19
**Status:** pending
**Depends on:** existing `klippy/blendmath.py` and `klippy/blendshaper.py` (no code change required in those for 6d)
**Unlocks:** 6e (per-corner shape selection), 6g (inverse-shaper benefits from smoother input)

---

## What this sub-spec does

Add a standalone pure-math module `klippy/blendquintic.py` that computes a **G² symmetric quintic Hermite** corner blend — analogous in scope and boundary conventions to the existing `blendmath.py` (which handles G¹ arcs). No integration with the CornerBlender yet; that's 6e.

**Why quintic, not something else:** our 2026-04-19 shape experiments showed quintic Hermite has ~25% lower post-shaper lateral deviation than arcs at matched commanded chord deviation, across every tested corner angle. It achieves G² continuity (curvature = 0 at both endpoints, matching the linear segments it joins), so the inverse-shaper filter in 6g has no acceleration step to compensate for. Cubic Bézier was tested and dropped (worse post-shaper quality despite faster traversal).

Evidence files:
- `klipper-sim/examples/shape_ceiling.py` — numerical comparison across shapes
- `docs/superpowers/specs/2026-04-19-ultimate-corner-blending-research.md` — overarching rationale (optional deeper read)

---

## Measured evidence from today's experiments

At α=90°, ε_cmd=0.2mm, a_max=45k, MZV 120Hz shaper applied:

| Shape | Traversal time | Commanded deviation | Post-shaper deviation |
|---|---|---|---|
| Arc (G¹, current) | 5.15 ms | 0.139 mm | 0.288 mm |
| Cubic Bézier (G²) | 4.57 ms | 0.139 mm | 0.321 mm |
| **Quintic Hermite (G²)** | **5.20 ms** | **0.141 mm** | **0.242 mm** |

Across angles (30°–135°), quintic's post-shaper deviation stays in 0.20–0.25 mm range vs arc's 0.23–0.31 mm. Time penalty is 0–29%: zero at α≥135°, small (1–5%) at α≥60°, substantial (29%) at α=20°. This is why 6e (per-corner shape selection) picks arc at shallow angles and quintic in the middle band.

---

## Geometry — the six control points

A symmetric quintic Bézier with 6 control points, parameterized by a single scalar `d` (tangent length along each ray):

```
Q0 = V − d · e1
Q1 = V − (4d/5) · e1
Q2 = V − (4d/5) · e1        ← Q2 coincident with Q1 ⇒ κ = 0 at entry
Q3 = V + (4d/5) · e2        ← Q3 coincident with Q4 ⇒ κ = 0 at exit
Q4 = V + (4d/5) · e2
Q5 = V + d · e2
```

where `V` is the corner vertex, `e1` is the entry tangent unit vector (pointing from upstream into V), and `e2` is the exit tangent unit vector (pointing from V downstream).

**Evaluation** (Bernstein form at parameter `t ∈ [0, 1]`):
```
B(t) = Σ C(5, i) · (1−t)^(5−i) · t^i · Q_i
     = (1−t)^5·Q0 + 5(1−t)^4·t·Q1 + 10(1−t)^3·t²·Q2
       + 10(1−t)^2·t³·Q3 + 5(1−t)·t^4·Q4 + t^5·Q5
```

---

## Closed-form chord-deviation relationship

At `t = 0.5`, the midpoint of the quintic is:
```
B(0.5) = (1/32)·[1·Q0 + 5·Q1 + 10·Q2 + 10·Q3 + 5·Q4 + 1·Q5]
       = V + (13d/32) · (e2 − e1)
```

Chord deviation from the corner vertex:
```
ε = |B(0.5) − V| = (13d/32) · |e2 − e1| = (13d/16) · sin(α/2)
```

**Inverse** (given target ε, derive d):
```
d = 16·ε / (13·sin(α/2))
```

Cross-check: α=90°, ε=0.2mm → d = 16·0.2 / (13·0.707) = 0.348mm. Matches the bisection result in `shape_ceiling.py` (d ≈ 0.35).

---

## Peak curvature

Curvature at the quintic midpoint is the binding constraint for the centripetal velocity cap. The formula follows from computing `B'(0.5)` and `B''(0.5)` analytically:

```
B'(0.5) ∝ (e1 + e2)     (tangent direction at mid, magnitude 15d/16 · cos(α/2))
B''(0.5) ∝ (e2 − e1)    (normal direction at mid)
```

The two are perpendicular for planar corners, so `κ_max = |B''| / |B'|². Full derivation lives in the implementation; TODO during 6d is to produce the closed-form expression `κ_max = f(α, ε)` and verify against numerical evaluation in `shape_ceiling.py`.

**Rough behaviour from numerical experiment** (used as sanity anchor for the analytical derivation):
| α (°) | κ_max·ε |
|---|---|
| 30 | 0.18 |
| 90 | 0.55 |
| 135 | 0.83 |

These are lower than the arc's equivalent `κ·ε = 1 − cos(α/2)` at α=90° (0.29 for arc vs 0.55 for quintic), meaning the quintic has *higher* peak curvature than the arc at matched ε. But the curvature is 0 at the endpoints, so average curvature is lower and post-shaper behaviour is much better.

---

## Velocity cap derivation

Mirror `blendmath.blend_geometry`'s structure but with quintic peak-curvature instead of arc `1/R`:

```
v_cent     = √(a_max / κ_max)                          (centripetal budget, post-Option-A full a_max)
v_step_cap = √(A_axis_per_axis / (κ_max · proj))      (shaper bound (b))
v_jerk     = (√(j_eff / κ_max) · (something))        (shaper rotation-jerk, derive analogous to arc)
v_cap      = min(v_cent, v_step_cap, v_jerk, max_velocity)
```

The `v_jerk` formula needs care because the quintic's curvature varies along the path; the derivation should use the *maximum* κ along the blend.

For `blend_from_moves_quintic` (analogous to current `blend_from_moves`): compute d from (α, ε) using the closed-form above, then κ_max, then v_cap. Return a `QuinticBlend` dataclass mirroring `BlendArc`.

---

## Polyline sampling

Unlike arcs (which have `segment_arc` with a clean `R·(1−cos(Δθ/2))` chord error), the quintic needs **adaptive De Casteljau subdivision**:

```
subdivide(Q0..Q5):
  if flatness(Q0..Q5) < max_chord_err:
    emit segments
  else:
    split at t=0.5, recurse on each half
```

The flatness test for a quintic is the max perpendicular distance of `Q1..Q4` from the line `Q0–Q5`. Standard algorithm; ~3–5 recursion depths at printer tolerances.

Expected output: 6–16 polyline points per corner (vs 2–5 for arcs). This is the 4–6× lookahead-queue pressure mentioned in the subagent research. Acceptable, but something to measure.

---

## E-axis parameterization

The extruder E-coordinate must track physical arc length along the quintic. For arcs, `s` is linear in the angular parameter, so E distributes uniformly. For quintics, `s(t)` requires numerical integration of `|B'(t)|` — Gauss-Legendre 5-point on each polyline sub-segment is adequate.

Implementation note: reuse `blendmath.interpolate_extruder` with a per-segment `s` array instead of uniform spacing.

---

## File layout

```
klippy/blendquintic.py          (new, ~250 LOC est.)
  - QuinticBlend dataclass
  - quintic_geometry(prev_dir, next_dir, L_prev, L_next, ε, a_max, j_eff) → QuinticBlend
  - blend_from_moves_quintic(prev_move, next_move, ε, toolhead=None) → QuinticBlend
  - segment_quintic(q, max_chord_err) → polyline
  - interpolate_extruder_quintic(polyline, ...)

test/test_blendquintic.py       (new, ~500 LOC est.)
  - test_chord_deviation_matches_eps
  - test_curvature_zero_at_endpoints
  - test_tangent_matches_entry_exit
  - test_v_cap_property_across_alpha
  - test_degenerate_near_collinear      (should return None like arc)
  - test_degenerate_near_u_turn         (R→0 equivalent; fall back to stop)
  - test_random_corners_property         (50-run property sweep, mirror test_blendmath.py)
```

---

## Test plan

1. **Numeric**: cross-check closed-form `d(α, ε)` and `κ_max(α, ε)` against the Bernstein evaluator on 50 random corners. Tolerance `1e-9`.
2. **Geometric property tests**: chord deviation ≤ ε, tangent match at endpoints, κ(0) = κ(1) = 0.
3. **Velocity cap**: `v_cap² ≤ min(a_max·R_eff, ...)` where `R_eff = 1/κ_max`.
4. **Polyline fidelity**: max chord error of emitted polyline ≤ `max_chord_err`.
5. **E-axis**: sum of E deltas along polyline equals `ε · (e_per_mm_prev + e_per_mm_next)` within 1e-6 (conservation).
6. **Simulator comparison**: extend `klipper-sim/examples/shape_ceiling.py` to assert quintic post-shaper deviation ≤ arc post-shaper deviation across α in {30, 60, 90, 120}.

---

## Open questions to resolve during implementation

1. **Closed-form `κ_max`** — derive fully, don't leave numerical. Verify matches Bernstein-eval to 1e-9.
2. **Choice of the 4/5 ratio in Q1, Q2, Q3, Q4** — is this optimal for minimum post-shaper deviation, or is there a free parameter we should tune per corner? The shape-ceiling experiment uses 4/5 empirically; derive whether it's provably the shape-parameter optimum.
3. **Velocity-cap shaper bound at quintic midpoint** — the existing `blendshaper.compute_shaper_bounds` handles arcs; adapt for curved paths where κ varies along the blend. The `in_plane` and `n_hat` projections need to be at the κ-peak point.

---

## Scope and exit criteria

**Estimated effort:** 2–3 weeks. Roughly:
- 3 days for math derivation and unit tests
- 4 days for integration with existing `blendshaper` path (`blend_from_moves_quintic`)
- 2 days for polyline sampling (De Casteljau)
- 2 days for E-axis parameterization
- 2 days for property tests and simulator comparison
- 3 days for polish, documentation, commit stack

**Done when:**
- All 6 test categories green
- `test_blendquintic.py` reaches test coverage parity with `test_blendmath.py` (at least 50 tests)
- Running `examples/shape_ceiling.py` with quintic enabled reproduces the measured 25% post-shaper deviation improvement
- No integration with `CornerBlender` yet — that's 6e
- No production code path uses this yet — purely additive module

---

## After this sub-spec

Hand off to 6e (per-corner shape selection). Quintic lives alongside the arc module; CornerBlender learns to pick between them. 6g can start in parallel — its inverse-filter work doesn't depend on shape choice, just benefits from smoother input.
