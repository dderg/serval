# Sub-spec 6e — CornerBlender Per-Corner Shape Selection

**Date:** 2026-04-19
**Status:** pending
**Depends on:** 6d (quintic geometry module — complete, `klippy/blendquintic.py`)
**Unlocks:** shape-aware corner emission; gives 6g's inverse-shaper pre-compensation a G²-continuous commanded trajectory to work against across the bulk of the angle range

---

## What this sub-spec does

Teach `klippy/blendplanner.CornerBlender` to pick between the existing G¹ tangent arc (`blendmath.BlendArc`) and the new G² quintic Bézier (`blendquintic.QuinticBlend`) per corner, based on the corner's deflection angle. The planner's downstream emission path (truncated-prev + polyline + truncated-next) becomes shape-agnostic — it stops hard-coding `BlendArc`.

No runtime flag. The quintic replaces the arc in its angle band unconditionally. This is the Kalico blend-arc fork; the fork itself is the opt-in gate.

---

## Selection rule

```
α < 35°         → arc
35° ≤ α ≤ 150°  → quintic
α > 150°        → arc
```

Prose: deflection is the angle the tool direction changes by through the corner; 0° is straight-through, 180° is a U-turn. Arc handles both shallow and near-U-turn corners; quintic handles the meaty middle band where its G² endpoint curvature pays off.

### Why these thresholds

Refreshed per-angle sweep against the *as-built* `blendquintic` module (shaper+rotation-jerk caps active, r-fit from 6d's quadratic, MZV 120 Hz / 0.1 ζ shaper, ε = 0.2 mm, v_cruise = 600 mm/s, a_max = 45 000 mm/s²):

| α (°) | arc t | quintic t | Δ time | arc post-dev | quintic post-dev | Δ quality |
|---:|---:|---:|---:|---:|---:|---:|
| 10  | 15.23 ms | 14.27 ms | **−6.3%** | 0.218 mm | 0.211 mm | −3% |
| 20  | 7.54 ms | 7.59 ms | +0.6% | 0.271 mm | 0.259 mm | −4% |
| 25  | 5.99 ms | 8.35 ms | **+39%** | 0.313 mm | 0.243 mm | −22% |
| 35  | 5.85 ms | 8.16 ms | +40% | 0.312 mm | 0.240 mm | −23% |
| 45  | 5.77 ms | 7.84 ms | +36% | 0.310 mm | 0.236 mm | −24% |
| 60  | 5.61 ms | 7.36 ms | +31% | 0.299 mm | 0.227 mm | −24% |
| 90  | 5.15 ms | 5.43 ms | **+5.5%** | 0.288 mm | 0.258 mm | −10% |
| 130 | 4.09 ms | 4.23 ms | +3.4% | 0.246 mm | 0.230 mm | −7% |
| 150 | 3.26 ms | 3.74 ms | +14.6% | 0.188 mm | 0.164 mm | −13% |
| 160 | 2.70 ms | 3.13 ms | +16.1% | 0.141 mm | 0.127 mm | −10% |
| 170 | 1.93 ms | 2.39 ms | +24% | 0.081 mm | 0.077 mm | −5% |
| 178 | 0.87 ms | 2.28 ms | **+162%** | 0.019 mm | 0.059 mm | **+205%** |

Three regimes:

- **α < ~20°**: quintic is cheap *and* better quality. But we still pick arc — corners this shallow are rare in slicer output, the quality win is marginal, and keeping the arc path active for them simplifies the selector (two thresholds beat three). Revisit if a real slice is dominated by sub-20° corners.
- **α ∈ [20°, ~35°]**: arc time advantage is steep (≥+35% time cost for quintic). Use arc. The quality delta (−23%) does not pay for a 40% slowdown here because the post-deviation is still inside ε anyway.
- **α ∈ [35°, 150°]**: quintic wins or ties on both axes in the middle of the range; modest time cost near the edges. This is the payoff band.
- **α > 150°**: quintic's curvature-concentration near the center gets extreme; traversal time diverges and quality inverts. Arc (and at α≈180° the degenerate R=0 junction stop) takes over.

The 35° and 150° cutoffs are sharp transitions in the data, not smooth. A single hard switch with no cross-fade is the right simplification.

### Threshold confidence

Defaults chosen from one sweep at the parameters above. Confidence: medium — enough to ship, not enough to call final. What would nail them down:

1. Test sweep at a few (ε, shaper) combinations — 0.1 mm with 60 Hz shaper behaves differently from 0.2 mm with 120 Hz.
2. A representative slicer G-code pass (e.g. the Voron cube slice the rest of the fork is validated against) with per-corner logging of α and shape choice, confirming the band covers ≥80% of the corners that dominate the time budget.

Shipping the cutoffs as user-tunable config keys (see below) lets operators shift them if their geometry is atypical; they are not tuning knobs in the normal sense.

---

## Prior art

Brief scan of neighbouring open-source motion planners to confirm the per-corner shape choice is unusual:

- **Classic Klipper / Smoothieware / grbl**: *no shape*. Junction deviation / SCV picks a scalar velocity cap at the vertex, and the tool follows two straight lines meeting at a sharp point — the physical arc is produced by the motion *profile* decelerating into a sharp corner under jerk/accel limits, not by a commanded curve. Single rule, no angle branching.
- **LinuxCNC G64**: similar — `G64 P<tol>` and the "naive cam" collapse collinear segments under a chord tolerance. The corner geometry is a side-effect of accel limits, not a planned blend shape.
- **Duet RepRapFirmware**: grbl-style junction deviation since 3.x; DAA (dynamic acceleration adjustment) is an orthogonal resonance fix, not a corner-shape choice.
- **Prunt**: single-shape, degree-15 Bézier on every corner with user-tunable deviation. The planner has a 4th-derivative rectangular-wave velocity profile to match the curve's smoothness. No per-corner shape selection — they just pay the shape cost everywhere.
- **This fork**: arc + quintic with a per-angle switch. The motivation is precisely what the table above shows: no single fixed shape dominates both axes (time AND post-shaper quality) across the full angle range, so picking the right shape per corner is the only way to claim "ultimate speed AND quality."

We are not aware of any other open-source FDM planner that does angle-based shape selection. Closest analog is high-end CNC literature on spline-vs-arc hybrid blending (Tajima & Sencer 2020; Zhao et al. 2013), both referenced in the 6d research doc.

---

## Dispatch pattern

**Chosen: duck-typed module-level `segment(blend, max_chord_err)` and `interpolate_extruder(polyline, ...)` in a new small helper module, plus the planner branching on shape at selection time only.**

### Why not methods on the dataclass

Klipper's convention across `blendmath`, `blendquintic`, and `blendshaper`: the dataclasses (`BlendArc`, `QuinticBlend`, `AxisShaperSnapshot`, `ShaperBounds`) are frozen, data-only. All behaviour is module-level free functions. Adding a `segment()` method to the dataclasses would be the first instance of behaviour-on-data in the blend stack and would spread: someone will then ask for `blend.world_points(vertex)`, `blend.interpolate_e(...)`, and so on.

### Why not `isinstance` dispatch in the planner

Works, but smears shape knowledge across the caller. The planner would need to import `blendmath.BlendArc` AND `blendquintic.QuinticBlend` just to branch, and every new shape (clothoid in 6f) means editing the planner's isinstance ladder.

### Chosen shape

A small new module `klippy/blendemit.py` that holds the shape-agnostic emit helpers. Each shape module publishes its own `segment_*` (already exists: `blendmath.segment_arc`, `blendquintic.segment_quintic`). The planner calls into a single `blendemit.segment(blend, max_chord_err)` that duck-types on the type name (or dict dispatch):

```python
# klippy/blendemit.py
from . import blendmath, blendquintic

def segment(blend, max_chord_err):
    if isinstance(blend, blendquintic.QuinticBlend):
        return blendquintic.segment_quintic(blend, max_chord_err)
    return blendmath.segment_arc(blend, max_chord_err)
```

Yes, this *is* isinstance dispatch — but it lives in one place behind a named seam (`blendemit.segment`). The planner just calls `blendemit.segment(blend, err)`. Future shapes add one line here; the planner is untouched.

E-axis interpolation stays as-is: `blendmath.interpolate_extruder` is shape-agnostic (operates on the polyline arc-length, not on the blend object). `blendquintic.interpolate_extruder_quintic` is a byte-for-byte duplicate and gets deleted in this sub-spec.

---

## Integration points

### `CornerBlender.feed(move)` — select the shape

Current:

```python
arc = blendmath.blend_from_moves(self._prev, move,
                                 self._toolhead.corner_deviation,
                                 toolhead=self._toolhead)
if arc is None: ...
if arc.R == 0.0 or arc.v_cap == 0.0: ...
trunc_prev, arc_moves, trunc_next_head = self._emit_arc(...)
```

New:

```python
blend = self._select_blend(self._prev, move)
if blend is None: ...
if blend.d_consumed == 0.0 or blend.v_cap == 0.0: ...
trunc_prev, arc_moves, trunc_next_head = self._emit_blend(self._prev, move, blend)
```

`_select_blend` computes α from `prev.axes_r` and `next.axes_r`, picks the shape per the rule, and calls either `blendmath.blend_from_moves` or `blendquintic.blend_from_moves_quintic`. Both adapters take the same signature and return a frozen dataclass, so the call-site stays clean.

Degenerate-corner detection switches from `arc.R == 0.0` to `blend.d_consumed == 0.0` — this check is already correct for both shapes (arc: `d_consumed = R·tan(θ/2) = 0` when `R = 0`; quintic: explicitly zero in the U-turn branch). The `v_cap == 0.0` clause remains as a belt-and-braces.

### `_emit_arc` → `_emit_blend`

Rename and generalise. The emission logic uses only these attributes of the blend: `d_consumed`, `v_cap`, and implicitly the polyline from `blendemit.segment(blend, err)`. Both `BlendArc` and `QuinticBlend` already expose `d_consumed` and `v_cap`. No `entry_pt` / `exit_pt` is actually needed — the planner reconstructs world-frame tangent points from `vertex ± d_consumed · dir`, which it already does today. No other dataclass attribute touches.

Shape-aware polyline generation is a one-line substitution:

```python
# old
polyline_local = blendmath.segment_arc(arc, chord_err)
# new
polyline_local = blendemit.segment(blend, chord_err)
```

The E-axis call (`blendmath.interpolate_extruder`) is unchanged; it takes a polyline, not a blend object.

### Everything upstream — unchanged

`BlendPipelineLookAheadQueue` (`klippy/blendprepass.py`) and the prepass `CollinearCollapser` see Move objects in and out. Shape choice is invisible at that level.

---

## E-axis handling

The existing `blendmath.interpolate_extruder(polyline, d_consumed, e_per_mm_prev, e_per_mm_next)` distributes E linearly over the polyline's piecewise-linear arc length. That's geometrically correct for both shapes, provided the polyline is sampled finely enough that the piecewise sum tracks the true arc length within the chord tolerance — which it is, by construction of `segment_arc` and `segment_quintic`.

`blendquintic.interpolate_extruder_quintic` exists but is a byte-for-byte duplicate of `blendmath.interpolate_extruder`. Delete it in this sub-spec; the planner uses the `blendmath` version for both shapes.

A possible future refinement is Gauss-Legendre integration of `s(t)` for per-segment proportional E on quintics at long arc lengths (>5 mm). Not in scope here — measure first on a real slice, don't pre-optimize.

---

## Degenerate-corner handling

- **Collinear (α ≈ 0)**: the prepass (`CollinearCollapser`) collapses these upstream; blend-selection is not reached. Both `blend_from_moves` and `blend_from_moves_quintic` also return `None` defensively.
- **U-turn (α ≈ π)**: both shapes return a degenerate blend with `d_consumed = 0.0` and `v_cap = 0.0`. The planner forces a junction stop (`prev.limit_next_junction_speed(0.0)`) and emits `prev` unaltered. This path is unchanged except the check key.

No new degenerate handling in 6e.

---

## Configuration

Two new knobs. Placement: `[printer]` section (not a `[blend_arc]` section — none exists; `corner_deviation` currently lives under `[printer]`).

```
[printer]
corner_deviation: 0.2
shape_switchover_low: 35       # degrees; α strictly below this → arc
shape_switchover_high: 150     # degrees; α strictly above this → arc
```

Defaults as shown. Range sanity: `shape_switchover_low > 0`, `shape_switchover_low < shape_switchover_high`, `shape_switchover_high < 180`. The knobs are advanced tuning — they do not turn quintic on or off. Document them in the blend-arc user reference alongside `corner_deviation`.

If the sweep in "Threshold confidence" above promotes the defaults to final, the knobs stay for diagnostic tuning but should carry a comment flagging them as rarely-changed.

---

## File layout

```
klippy/blendplanner.py     (modify CornerBlender.feed, _emit_arc → _emit_blend, add _select_blend)
klippy/blendemit.py        (NEW — holds segment() dispatch)
klippy/blendquintic.py     (delete interpolate_extruder_quintic duplicate)
klippy/toolhead.py         (parse shape_switchover_low / _high from [printer])
test/test_blendplanner.py  (add shape-selection integration tests)
```

One new file (`blendemit.py`). Everything else is edits to existing files.

---

## Test plan

1. **Regression.** All existing tests (`test_blendmath`, `test_blendquintic`, `test_blendplanner`) green. The `_emit_arc → _emit_blend` refactor is behaviour-preserving when all corners fall in the arc band; assert that by running the arc-only fixture set.
2. **Selection unit tests.** `test_blendplanner.py::test_shape_selection_by_angle` feeds corner pairs at α = {15, 25, 34, 36, 45, 90, 135, 149, 151, 170, 179}, asserts that the emitted polyline has the fingerprint of the expected shape (arc: near-uniform curvature along the polyline; quintic: zero curvature at endpoints, peak in the middle).
3. **Degenerate branches.** U-turn (α = π) still forces a junction stop under both shape bands (edge case: `_select_blend` may still try to build a quintic first; the zero-`d_consumed` check catches it).
4. **Simulator integration.** Run the existing `slice_24layers.gcode` fixture through the planner with the selector active. Compare vs arc-only baseline:
   - Total print time: should be ≤ arc-only by the same margin the per-angle sweep predicts (~3–5% on a typical slice).
   - Post-shaper Y excursions: should be ≤ arc-only in the angle range dominated by quintic.
5. **Threshold sanity.** Sweep `shape_switchover_low` ∈ {30, 35, 40} and `shape_switchover_high` ∈ {140, 150, 160} on the same slice. Expect a broad plateau around the defaults.

---

## Scope and exit criteria

**Estimated effort:** roughly

- 0.5 day — `blendemit.py` scaffold + `segment()` dispatch + delete the duplicate.
- 1 day — `_emit_arc → _emit_blend` refactor and `_select_blend` selector.
- 0.5 day — `toolhead.py` config wiring for the two knobs.
- 1 day — new integration tests + threshold sanity sweep.
- 1 day — simulator run against `slice_24layers.gcode`.
- 0.5 day — polish, documentation, commit stack.

Total: ~1 week.

**Done when:**

- All existing blend tests green.
- Shape-selection tests assert the selector matches the rule at the listed angles.
- Simulator run on `slice_24layers.gcode` shows total time ≤ arc-only and post-shaper excursions ≤ arc-only in the target band.
- Config knobs parsed, range-checked, and documented.
- Hardware test on V0 or Trident shows no regression in visible print quality vs current arc blender.

---

## After this sub-spec

- **6g** (inverse-shaper pre-compensation) — can start in parallel; 6e provides a G²-continuous commanded trajectory that makes 6g's deconvolution well-posed across the bulk of the angle range.
- **6f** (G³ clothoid) — speculative; defer until 6e's hardware test reveals residual ringing at the quintic's peak-curvature region. If 6d+6e+6g nails the target, 6f is unneeded.
