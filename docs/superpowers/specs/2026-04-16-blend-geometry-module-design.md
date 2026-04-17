# Blend Geometry Module — Design Spec

**Date:** 2026-04-16
**Scope:** Stage 1 sub-spec #1 of the corner-blending fork (see `2026-04-16-phase0-research/00-summary.md`).
**Status:** Design approved, ready for implementation planning.

---

## Purpose

Provide the pure-math primitive that, given two adjacent linear moves and a chord-tolerance parameter, returns:

1. The geometry of a G¹ tangent circular arc that smooths the corner.
2. The maximum velocity that arc can be traversed at, consistent with the machine's acceleration budget and the input shaper's implicit jerk ceiling.
3. A fine-segmented polyline approximation of that arc, ready to be fed through the existing `trapq` pipeline as ordinary linear moves.

The module is the foundation of the fork's replacement for `square_corner_velocity` / `junction_deviation`. Every other sub-spec in Stage 1 consumes its output.

## Non-goals

Not this spec:

- Deriving the shaper-imposed jerk ceiling `j_eff`. It is an input parameter. (Separate sub-spec, must land before planner integration.)
- Collapsing collinear slicer segments prior to blending. (Sub-spec #2, the naive-CAM prepass.)
- Wiring into `toolhead.py` / `LookAheadQueue`. (Sub-spec #3.)
- Removing SCV / `junction_deviation` config. (Sub-spec #4.)
- Updating `find_shaper_max_accel`'s `offset_90` term. (Sub-spec #5.)
- Final user-facing parameter name. (Sub-spec #6.)
- G² Bézier blends. Stage 3, gated on measurement.

## Module layout

File: `klippy/blendmath.py`. Python 3. No Kalico imports in the core functions; `math` and `dataclasses` only.

```
klippy/blendmath.py
├─ BlendArc  (dataclass)                    # computed arc parameters
│    R             float    arc radius (mm)
│    theta         float    deflection angle (rad); 0=collinear, π=U-turn
│    d_consumed    float    length consumed on each adjacent segment (mm)
│    v_cap         float    max traversal velocity (mm/s)
│    center        Vec3     arc center
│    entry_pt      Vec3     tangent point on incoming segment
│    exit_pt       Vec3     tangent point on outgoing segment
│    entry_tangent Vec3     unit tangent at entry (= prev_dir)
│    exit_tangent  Vec3     unit tangent at exit  (= next_dir)
│    plane_normal  Vec3     unit normal to the arc plane
├─ blend_geometry(prev_dir, next_dir, L_prev, L_next,
│                 corner_deviation, a_max, j_eff) -> BlendArc | None
├─ segment_arc(arc: BlendArc, max_chord_err: float) -> list[Vec3]
└─ blend_from_moves(prev_move, next_move, corner_deviation,
                    j_eff) -> BlendArc | None
```

`BlendArc` is the canonical intermediate representation. `None` means no blend applies (collinear or otherwise degenerate — caller emits the two straight moves unchanged).

`blend_geometry` and `segment_arc` are the pure-math core, testable in isolation. `blend_from_moves` is a thin adapter that reads `axes_r`, `move_d`, `accel`, and related fields off a pair of `Move` objects and calls the core.

`Vec3` is a 3-tuple of floats throughout this module — no custom vector class.

## Algorithm

### Conventions

`θ = deflection angle` between incoming and outgoing tangent vectors:

- `θ = 0` when the segments are collinear.
- `θ = π` at a U-turn reversal.

With head-to-tail direction vectors `prev_dir, next_dir` (the direction the toolhead is traveling), the deflection is:

```
cos(θ)     = -(prev_dir · next_dir)
cos(θ/2)   = √((1 + prev_dir·next_dir) / 2)   [mirror of toolhead.py:94-100]
sin(θ/2)   = √((1 − prev_dir·next_dir) / 2)
```

LinuxCNC's `blendmath.c` uses a complement convention `θ_LCNC = π/2 − θ/2`. Cross-reference mapping when reading their source:

```
sin(θ_LCNC) = cos(θ/2)
cos(θ_LCNC) = sin(θ/2)
tan(θ_LCNC) = cot(θ/2)
```

### Arc geometry

Two radius constraints, final radius is the smaller:

**1. Tolerance-driven radius** — chord deviation from corner vertex to arc ≤ `corner_deviation`:

```
R_tol = corner_deviation · cos(θ/2) / (1 − cos(θ/2))
```

**2. Midpoint cap** — tangent points must sit inside their respective segments:

```
R_mid = min(L_prev, L_next) / tan(θ/2)
```

No additional fractional safety factor here. Inter-corner overlap (when two adjacent corners each want a piece of the shared segment) is resolved by the downstream look-ahead, not by shrinking locally.

```
R = min(R_tol, R_mid)
```

Consumed length along each adjacent segment:

```
d_consumed = R · tan(θ/2)
```

### Velocity cap

Three bounds; take the minimum.

**Centripetal** — LinuxCNC's Pythagorean acceleration split:

```
a_t_max = 0.5 · a_max
a_n_max = (√3 / 2) · a_max ≈ 0.866 · a_max
v_centripetal = √(a_n_max · R)
```

The fixed 50% tangential allocation is a design choice; the 86.6% normal falls out of `a_t² + a_n² ≤ a_max²` vector closure. Ensures the machine's total acceleration magnitude never exceeds `a_max` during corner traversal.

**Jerk floor** — protects the shaper's spectral design assumptions when entering a G¹ arc (where normal acceleration steps from 0 to v²/R):

```
v_jerk = (R · √j_eff)^(2/3)
```

Equivalently: `R ≥ v^(3/2) / √j_eff`. `j_eff` is derived externally from shaper properties (separate sub-spec); this module takes it as an input.

**Midpoint-implied cap** — redundant with the `R_mid` geometry cap but useful as a sanity bound; falls out of the chosen `R` and the centripetal formula.

```
v_cap = min(v_centripetal, v_jerk)
```

### Polyline segmentation (`segment_arc`)

Walk the arc in equal-angle steps `Δφ` such that the chord error between the polyline and the true arc stays within `max_chord_err`:

```
Δφ ≤ 2 · arccos(1 − max_chord_err / R)
```

Default `max_chord_err = 10 µm`, derived from Phase 0 Bucket E's analysis (≈0.2 mm segments at R=0.5 mm yields 10 µm chord error). Output is a list of 3D points starting at `entry_pt` and ending at `exit_pt`, inclusive.

## Degenerate cases

1. **Collinear** (θ near 0): return `None`. Threshold: `sin(θ/2) < 1e-6`. Caller emits the two straight moves unchanged.
2. **U-turn / reversal** (θ near π): return `BlendArc` with `R=0, v_cap=0`. Caller must stop at the junction. Threshold: `cos(θ/2) < 1e-6`.
3. **Short adjacent segments**: `R_mid` drives `R` toward zero. No fallback — emit the small arc; `v_cap` collapses accordingly; look-ahead propagation handles the resulting low-speed junction.
4. **Non-kinematic moves** (E-only, no XYZ motion): caller's responsibility to skip the blend call entirely, preserving the existing `if not self.is_kinematic_move` guard at `toolhead.py:80`.
5. **Extruder axis (E) through a blend**: handled in the `blend_from_moves` adapter, not in the pure-math core. The adapter interpolates E proportionally to arc length and attaches it to each polyline point. `segment_arc` itself remains 3D.

## Testing

`test/test_blendmath.py`.

1. **Unit tests** — analytic fixtures at representative corners (30°, 60°, 90°, 120°, 150° deflection; symmetric and asymmetric segment lengths). Compare computed `R`, `v_cap`, `d_consumed` against hand-derived values within 1e-9 relative tolerance.
2. **Property tests** — random valid corners:
   - Chord deviation of every polyline segment ≤ `corner_deviation`.
   - `v_cap` ≤ every individual cap (centripetal, jerk).
   - Polyline endpoints lie on the incoming and outgoing segment lines.
   - Polyline points all lie on the arc within chord tolerance.
3. **Regression fixtures** — each degenerate case from the section above as an explicit test.

Out of scope for this spec: golden-file cross-check against a C harness running LinuxCNC's `blendmath.c` on matching inputs. Useful for independent verification; requires pulling LinuxCNC's TP module; deferred.

## Dependencies

**Must land before this module is wired into the planner:**

- `j_eff` derivation sub-spec. Until then, a conservative constant can be passed in for unit testing, but real use requires the derived value.

**This spec does not depend on:**

- Any other Stage 1 sub-spec. The module is self-contained and can be built and tested in isolation.

## Validation gate before shipping (Stage 1 wrap)

Measure corner residuals on real hardware with G¹ tangent arcs engaged end-to-end. Per Prunt's reported experience, even G² blends can produce audible ringing on some hardware; G¹ is strictly less smooth. If residuals exceed the shaper's error envelope, the Stage 3 G² upgrade must be pre-empted into Stage 1. This gate applies to the integrated system, not to this module in isolation.
