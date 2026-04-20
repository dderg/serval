# Plan 1 — Quintic Revival + Shape-Pluggable Primitive (design)

**Date:** 2026-04-20
**Branch:** `magnum-opus`
**Status:** design, awaiting user review
**Parent design:** `docs/Magnum_Opus_Design.md` (MO pillars overview)
**Unblocks:** plans 4 (pillar 3), 5 (pillar 2 unified `v(s)`), 6 (pillar 1 inverse shaper)
**Does not depend on:** any parallel porting work (smooth-IS, HP-stepcompress, non-linear PA all independent)

---

## Goal

Replace `blend-arc`'s circular-arc corner primitive with a curvature-continuous quintic Hermite Bezier shape on the `magnum-opus` branch, behind a shape-pluggable interface (`SmoothShape`). Establish the interface so future candidates — Pythagorean-Hodograph (PH) spline, Euler-spiral clothoid — slot in without touching the planner. Fix the one math bug the audit identified in the archive.

Plan 1 is a *foundation*. It does not try to beat mainline on print time. It leaves the planner working, emitting smooth-curve blends instead of arcs, with the same v_cap machinery as today. Pillar 2 (plan 5) layers unified `v(s)` on top; pillar 3 (plan 4) layers extruder constraints; pillar 1 (plan 6) layers inverse shaping.

## Non-goals

- No unified `v(s)` integration along the curve — scalar/piecewise v_cap per archive is fine for plan 1; pillar 2 changes this.
- No suppression-rule rewrite. Plan 1 skips suppression entirely; planner always calls the blend factory; degenerate corners fall back to sharp-V via `from_moves(...) -> None`.
- No extruder-constraint enforcement (plan 4).
- No inverse-shaper integration (plan 6).
- No shape research or candidate swap — those run in parallel subagents; if PH or clothoid emerge as winners, they slot in later via the same protocol.
- No hardware test — plan 7 (integrated MO test) is the first HW touchpoint.

## The `SmoothShape` protocol

Validated against published corner-smoothing literature (Erkorkmaz-Altintas 2001, Sencer-Tajima 2016/2018, Shi-Huang 2021, Farouki 2008 PH, Beudaert 2012, Manni-Sestini PH series, Biagiotti-Melchiorri 2019). Arc-length parameterisation, callable velocity bound, and opaque-to-implementation surface all match modern convention.

```python
# klippy/blendshape.py

from dataclasses import dataclass
from typing import Optional, Protocol

Vec3 = tuple[float, float, float]


@dataclass
class ExtruderLimits:
    """Stub for pillar 3. Plan 1 leaves this as None everywhere."""
    accel_max: float       # mm/s² on filament
    rpm_max: float         # drive pulley angular velocity


@dataclass
class KinematicLimits:
    """Flat dataclass passed into shape factories. Replaces handing
    the whole toolhead object in. Built once per planner run."""
    a_max: float
    v_max: float
    jerk_max: Optional[float]           # j_eff; None disables rotation-jerk cap
    shaper_sigma_T: float               # from IS impulse pattern
    extruder_caps: Optional[ExtruderLimits]   # None for plan 1


class SmoothShape(Protocol):
    """Curvature-continuous corner blend between two adjacent moves.
    Arc-length parameterised; s ∈ [0, arc_length]. Protocol is
    implementation-opaque — consumers see only this surface."""

    # Static properties
    d_consumed: float      # tangent length consumed per incoming edge [mm]
    theta: float           # deflection angle [rad]
    arc_length: float      # total length of the blend [mm]

    # Geometric queries along arc-length s
    def position_at(self, s: float) -> Vec3: ...
    def tangent_at(self, s: float) -> Vec3: ...
    def curvature_at(self, s: float) -> float: ...
    def dkappa_ds(self, s: float) -> float: ...     # for pillar 2 jerk bound

    # Velocity limit curve V_lim(s) — centripetal + shaper + (optional) jerk.
    # Pillar 3 wraps this with extruder cap in a separate stage, not here.
    def v_cap_fn(self, s: float) -> float: ...

    # Dense polyline for trapq emission.
    def polyline(self, chord_tol: float) -> list[Vec3]: ...
```

Per-shape factory sits on the concrete class, not the protocol:

```python
# klippy/blendquintic.py

class QuinticShape:
    @classmethod
    def from_moves(
        cls,
        prev_move,
        next_move,
        corner_deviation: float,
        limits: KinematicLimits,
    ) -> Optional["QuinticShape"]:
        """Construct a quintic blend for the corner between
        `prev_move` and `next_move`. Returns None for degenerate
        corners (collinear, near-reversal, chord budget infeasible) —
        caller falls back to sharp-V."""
        ...
```

Planner calls `blendquintic.QuinticShape.from_moves(...)` directly. Swapping shapes later = change that one import + factory call. No runtime dispatch, no config flag.

### What the protocol does NOT cover

- Extruder-constraint cap (pillar 3 adds as a wrapper stage).
- Suppression rules (planner's concern, not shape's).
- Shaper-type awareness — the shape pulls whatever `blendshaper.compute_shaper_bounds(...)` hands it; that module handles FIR vs polynomial.
- Second-derivative / normal vector / torsion — 2D, derivable on demand, not load-bearing.

## File layout

### New files

| File | Purpose | ~LOC |
|---|---|---|
| `klippy/blendshape.py` | Protocol + `KinematicLimits` + `ExtruderLimits` stub. Shared type home. | 50 |
| `klippy/blendquintic.py` | `QuinticShape` implementation. Ports ~300 LOC of clean archive math; fresh scaffolding. | 450 |
| `test/test_blendshape.py` | Protocol-conformance harness — any implementation passes basic property tests. | 80 |
| `test/test_blendquintic.py` | Port/adapt from archive. Inverted 3-pt shaper assertion. | 700 |

### Modified files

**`klippy/blendmath.py`.** Delete arc-specific pieces; keep shape-agnostic math:

- **Delete:** `BlendArc` dataclass (L70), `blend_geometry` (L95), `segment_arc` (L214), `blend_from_moves` (L438). ~230 LOC removed.
- **Keep:** vector utilities (`vdot`, `vnorm`, `vcross`, `vscale`, `vadd`, `vsub`, `vnormalize`, `_rotate`), `_sigma_T_max_from_toolhead`, `_scv_equivalent_junction_v`, `suppressed_junction_v` (kept as dead code for now — plan 5 decides fate), `_extract_shapers`, `interpolate_extruder` (already shape-agnostic).

**`klippy/blendplanner.py`.** Replace the arc factory call:

- Line 66 (`blendmath.blend_from_moves(...)`) → `blendquintic.QuinticShape.from_moves(...)`.
- Update the `arc is None` short-circuit to `shape is None`.
- Return type change: `BlendArc` → `QuinticShape` (both have `d_consumed`, `v_cap` at the SmoothShape-surface level).
- Call `shape.polyline(chord_tol)` for emission (was `segment_arc(arc, chord_tol)`).
- v_cap access: currently `arc.v_cap` scalar → `shape.v_cap_fn(s)` evaluated at `s=arc_length/2` for plan 1's scalar-equivalent behaviour (pillar 2 replaces with full integration).

**`test/test_blendmath.py`.** Delete arc-specific tests; keep shared-utility tests.

**`test/test_blendplanner.py`.** Adapt fixtures to `QuinticShape` return type.

### Unchanged files

- `klippy/blendshaper.py` — `compute_shaper_bounds` stays as-is.
- `klippy/blendprepass.py` — `CollinearCollapser` unaffected.
- `klippy/toolhead.py` — no `shape_switchover_*` knobs to remove (archive-only).

## Port items

Per the audit in the archive-review subagent's report, five pieces port **verbatim** after renaming into `QuinticShape`:

| Archive symbol | New home in `QuinticShape` | Audit verdict |
|---|---|---|
| `_quintic_eval`, `_quintic_first_deriv`, `_quintic_split`, `_quintic_flatness` + De Casteljau helpers | `_eval`, `_deriv`, `_split`, `_flatness` (private methods) | correct |
| `_curvature_at`, `_point_frame`, `_peak_curvature` | `curvature_at`, `_point_frame`, `_peak_curvature` | correct |
| `_deviation_coeff`, `_deviation_closed_form`, `_d_from_deviation` | internal helpers of `from_moves` | correct |
| `r(θ)` quadratic fit + clamp `[0.50, 0.86]` | `_r_of_theta` | correct |
| `segment_quintic`, `segment_quintic_with_t` | `polyline`, `_polyline_with_t` | correct |
| rotation-jerk bound `v ≤ (R·√j_eff)^(2/3)` | inside `v_cap_fn` composition | correct |

### Three deltas on top of the verbatim port

**1. Replace the 3-point shaper cap (~40 LOC).** Archive samples `compute_shaper_bounds(...).v_step_cap` at `t ∈ {0.25, 0.5, 0.75}` and returns `min`. Audit measured ~15% overshoot at `(θ=122°, rotation=164°)` — silent violation of the shaper entry-step budget.

Replacement: dense sampling at 50 uniform `t` values. Cost ~50 µs per corner on CPython; negligible against the `from_moves` call rate (100 Hz-ish at lookahead).

```python
# QuinticShape internal
_SHAPER_SAMPLE_N = 50

def _shaper_velocity_cap(self, limits: KinematicLimits) -> float:
    """Dense min of v_step_cap over the blend. Replaces archive's
    3-point sampler (which under-tightened by up to ~15%)."""
    worst = float("inf")
    for i in range(self._SHAPER_SAMPLE_N + 1):
        t = i / self._SHAPER_SAMPLE_N
        pos, tan, nrm = self._point_frame(t)
        k = self._curvature_at_t(t)
        if k <= 0.0:
            continue
        R = 1.0 / k
        bounds = blendshaper.compute_shaper_bounds(
            R=R, n_hat=nrm, p_hat=tan, sigma_T=limits.shaper_sigma_T,
        )
        worst = min(worst, bounds.v_step_cap)
    return worst
```

**2. Add `dkappa_ds(s)` (~30 LOC).** Required by pillar 2's jerk-bounded velocity integration. The analytical form:

```
κ(t) = (B'(t) × B''(t)) · ẑ / |B'(t)|³          (2D planar)
dκ/dt = [(B'' × B'' + B' × B''') · ẑ] / |B'|³
      − 3 (B'(t) × B''(t)) · ẑ · (B'·B'') / |B'|⁵
    = (B' × B''') · ẑ / |B'|³ − 3 κ (B'·B'') / |B'|²
dκ/ds = (dκ/dt) / |B'(t)|
```

Implement `_dkappa_dt_at_t`, then `dkappa_ds(s)` maps `s → t` via the cached s→t table and divides by `|B'(t)|`. No finite differences.

**3. Use 8 Gauss-Legendre nodes for s→t inversion.** Archive uses fewer (need to verify); 8 nodes give sub-µm arc-length accuracy over a 5 mm blend. Build the s→t cache in `from_moves`; store on the instance; reuse for all subsequent position/tangent/curvature queries.

### What does NOT get ported

- `blendemit.py` (isinstance dispatch) — doesn't exist on magnum-opus; archive-only.
- `QuinticBlend` dataclass — folded into `QuinticShape` class with methods.
- Archive's `blend_from_moves_quintic` free function — reshaped as the `from_moves` classmethod.
- `_three_point_shaper_cap` — replaced per delta 1.
- Shape-switchover config / angle selector / arc fallback — doesn't exist on magnum-opus.

## Test strategy

### Test files

| File | Responsibility | Source |
|---|---|---|
| `test/test_blendshape.py` (NEW) | Protocol conformance: any `SmoothShape` passes basic property checks (arc_length > 0, d_consumed > 0, tangent matches move dirs at endpoints, κ(0)=κ(L)=0, tangent ≈ d/ds position). | Fresh |
| `test/test_blendquintic.py` (NEW) | Quintic math (closed-form deviation, curvature, `r(θ)` anchors, rotation-jerk, adaptive subdivision, property sweeps). | ~80% port from archive, adjusted for class API |
| `test/test_blendmath.py` (modified) | Shared-utility tests kept. Arc-specific tests deleted. | Trim |
| `test/test_blendplanner.py` (modified) | Planner integration fixtures updated for `QuinticShape` return. | Adapt |

### New tests (not in archive)

1. **3-pt cap regression.** At `(θ=122°, rotation=164°)` the audit identified, assert the new dense-sampled cap is **tighter** than the archive's 3-pt formula. Also check `(θ=150°, rotation=45°)` from the archive's own pathological case.
2. **`dkappa_ds` correctness.** Analytical vs 5-point finite difference of `curvature_at` within `1e-6` over a sweep of `s`. Document the value at endpoints (not guaranteed zero — derive from the math).
3. **s→t map sub-µm accuracy.** Build the 8-GL cache, invert `position_at(s)` for 100 random s, compare against 20001-sample reference. Assert max position error < 1 µm.
4. **`v_cap_fn(s)` structural property.** On a clean quintic, `v_cap_fn(s)` has its minimum at the peak-curvature s. No spurious local minima.
5. **`from_moves` returns None for degenerate corners.** Collinear (θ ≈ 0), near-reversal (θ ≈ π), infeasible chord budget all return None without exceptions.

### Property tests ported from archive

- Random corner sweep (θ × rotation × r) across 200+ synthetic corners: chord-deviation budget, endpoint G² continuity, curvature non-negativity, `v_cap_fn` bounded.
- Symmetry (mirrored corners produce mirrored blends).
- Tiny-deflection and tiny-tangent robustness.

### Sim-level smoke test

Added to CI:

- One batch-sim run with the Voron cube gcode + `sim_main.cfg` analog.
- Assert: no crashes, total time within ±5% of current mainline SCV, step-queue health (`buffer_time` min > 1.0 s).

No runtime regression gate. Plan 1 explicitly does not optimise time; pillar-2 plan sets the first real performance target.

## Dependencies and ordering

Plan 1 has no prerequisites. It does not depend on:

- Smooth-IS port (your parallel session) — `QuinticShape` uses whatever shaper-bound machinery exists today; pillar-1 plan later swaps both.
- HP-stepcompress port — independent, downstream of the planner.
- Non-linear PA port — pillar 3 prerequisite, not plan 1's.
- Shape-research subagent — if PH or clothoid emerges as winner, they plug into the same protocol.

Plan 1 unblocks:

- Plan 4 (pillar 3, extruder first-class) — needs the protocol + `KinematicLimits.extruder_caps` slot.
- Plan 5 (pillar 2, unified `v(s)`) — needs `v_cap_fn(s)`, `dkappa_ds`, and the protocol to hang integration on.
- Plan 6 (pillar 1, inverse shaper) — needs the smooth-curve path the inverse-shaper operates on.

## Open questions

1. **Does `dkappa_ds(0)` equal zero at G² endpoints?** For the symmetric quintic with zero endpoint curvature, `dκ/dt(0) ≠ 0` in general — curvature ramps in from zero. Plan 1 derives and documents the value; test 2 asserts whatever the derivation gives.
2. **Chord-tolerance default for `polyline()` in plan 1.** Archive defaulted to `1e-2 mm` (10 µm). Plan 5 (pillar 2) replaces this with shaper-bandwidth-derived auto-tuning. In the meantime, tighten the default to `1e-3 mm` (1 µm) to reduce the "4 segments per blend" artefact. Revisit if trapq load balloons in the smoke test.
3. **Name:** protocol is `SmoothShape`, implementation is `QuinticShape`. Alternative: `CornerBlend` / `QuinticBlend`. Leaving as `SmoothShape` — matches the MO design doc's "smooth-accel corner primitive" language.
