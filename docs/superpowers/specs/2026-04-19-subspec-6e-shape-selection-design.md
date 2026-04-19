# Sub-spec 6e — CornerBlender Per-Corner Shape Selection

**Date:** 2026-04-19
**Status:** pending
**Depends on:** 6d (quintic Hermite module must exist)
**Unlocks:** full shape-aware blender; makes 6g's smoother input available across the typical-geometry range

---

## What this sub-spec does

Extend `klippy/blendplanner.CornerBlender` with a **shape selector**: for each corner, pick one of {arc, quintic} based on the deflection angle α and a small set of rules. Don't remove the arc code path — it remains the fallback for shallow and near-U-turn corners. Additive change.

---

## The selection rule

```
if α < 35°  → arc
if 35° ≤ α ≤ 150° → quintic
if α > 150° → arc (R → 0 fallback)
```

**Justification from measured data** (`klipper-sim/examples/shape_ceiling.py`):

| α (°) | arc time | quintic time | quintic post-deviation gain |
|---|---|---|---|
| 30 | 5.88 ms | **7.61 ms (+29%)** | −35% (but quintic loses 29% time) |
| 45 | 5.77 ms | 6.81 ms (+18%) | −33% (break-even regime) |
| 60 | 5.61 ms | 6.19 ms (+10%) | −30% (quintic starts winning) |
| 90 | 5.15 ms | **5.20 ms (+1%)** | −16% (quintic clearly wins) |
| 135 | 3.91 ms | **3.93 ms (0%)** | −1% (shapes converge) |

The 35° threshold is where time penalty equals quality gain (roughly). 150° is where the quintic degenerates numerically (`d → ∞` since `sin(α/2) → 1` but geometry near U-turn is pathological). Both are cliffs, not smooth transitions — a hard switch with no "blend between blends" is the simplest.

Thresholds are likely user-tunable (`[blend] shape_switchover_low`, `[blend] shape_switchover_high`) with the above defaults. Revisit after hardware testing.

---

## Integration points

### `CornerBlender.feed(move)` (existing, `klippy/blendplanner.py`)

Currently calls `blendmath.blend_from_moves(prev, move, cd, toolhead=self._toolhead)` to produce a `BlendArc`. Change to:

```python
deflection = compute_deflection(prev, move)
if SHAPE_LOW <= deflection <= SHAPE_HIGH:
    blend = blendquintic.blend_from_moves_quintic(prev, move, cd, toolhead=self._toolhead)
else:
    blend = blendmath.blend_from_moves(prev, move, cd, toolhead=self._toolhead)
```

The downstream `_emit_arc` path needs renaming/generalization — it currently builds polyline moves from a `BlendArc`. Extract the "truncated prev + polyline + truncated next" emission into a shape-agnostic helper that takes either `BlendArc` or `QuinticBlend`, via duck-typed attributes `(d_consumed, v_cap, entry_pt, exit_pt)` and a `segment(...)` method for polyline generation.

### `_emit_arc` → `_emit_blend`

Refactor: abstract over the geometry object. Both `BlendArc.segment()` (existing `blendmath.segment_arc`) and `QuinticBlend.segment()` (from 6d) return `[(x, y, z), ...]`. The rest of the emission logic (E-axis interpolation, truncated-prev/next construction) becomes shape-agnostic.

### `BlendPipelineLookAheadQueue` — no change.

The filter-chain adapter (`klippy/blendprepass.py`) sees Move objects in and out; internal shape choice is invisible.

---

## E-axis handling in the emission

`blendmath.interpolate_extruder(polyline, d_consumed, e_per_mm_prev, e_per_mm_next)` currently assumes linear E distribution over the polyline. This still works for quintic output *as long as* the polyline is sampled by arc length, not by parameter `t`. 6d's polyline sampler (De Casteljau) must emit samples at roughly-uniform `ds`, which the flatness-controlled subdivision naturally produces to within a factor of 2–3×. Acceptable.

If the E-axis error turns out to be measurable on long blends (>5mm arc length), switch to per-segment proportional E based on `s(t)` from Gauss-Legendre integration. Flag — measure first, don't preemptively over-engineer.

---

## Degenerate-corner handling

Current code already handles:
- Collinear (α ≈ 0): `blend_from_moves` returns None, no blend emitted
- U-turn (α ≈ π): `BlendArc` with R=0, caller forces stop

Both remain on the arc code path because of the selection rule (α<35° and α>150° use arc). No quintic-specific degenerate handling needed in this sub-spec.

---

## File layout

```
klippy/blendplanner.py          (modify CornerBlender.feed, _emit_arc → _emit_blend)
klippy/blendquintic.py          (from 6d — imported here)
test/test_blendplanner.py       (add integration tests for shape selection + hybrid corners)
```

No new files in this sub-spec. The config keys (if added) go in `config/sample-blend-arc.cfg`.

---

## Test plan

1. **Regression**: all existing 419 tests green after the `_emit_blend` refactor. Arc code path behavior unchanged.
2. **Integration**: `test_blendplanner.py::test_shape_selection_by_angle` feeds corners at α = {20, 30, 40, 90, 140, 160, 180} and asserts:
   - α ∈ {20, 30, 160, 180} → emits arc polyline (identifiable by uniform curvature)
   - α ∈ {40, 90, 140} → emits quintic polyline (κ=0 at endpoints, peak in middle)
3. **Simulator**: run `slice_24layers.gcode` with shape selection enabled. Compare:
   - vs arc-only (current baseline): print time should be ≤ arc, Y excursions ≤ arc
   - vs quintic-only: print time should be ≤ quintic (quintic alone loses at shallow corners)
4. **Threshold sweep**: run at SHAPE_LOW ∈ {25, 30, 35, 40, 45} to confirm the chosen default is near-optimal on the Voron cube slice.

---

## Configuration

Add to `[blend_arc]` config section (or wherever `corner_deviation` lives):

```
[blend_arc]
corner_deviation: 0.2
shape_switchover_low: 35      # degrees; below this, use arc
shape_switchover_high: 150    # degrees; above this, use arc
```

Defaults chosen from measured sweep. Document both knobs in the user-facing config reference.

---

## Scope and exit criteria

**Estimated effort:** 1–2 weeks. Roughly:
- 1 day `_emit_arc` → `_emit_blend` refactor
- 1 day shape selector + config parsing
- 2 days integration tests + threshold sweep
- 2 days simulator comparison on real G-code
- 3 days polish, documentation, commit stack

**Done when:**
- All existing blend tests green
- Integration tests assert shape selection matches selection rule
- Simulator run on `slice_24layers.gcode` shows net win: ≤ arc's time AND ≤ quintic-only's time, with fewer post-shaper excursions than arc-only
- Config knobs documented
- Hardware test on V0 or Trident shows no regression vs current arc blender

---

## After this sub-spec

Two parallel paths open:
- **6g** (inverse-shaper pre-compensation) — can start any time; benefits from 6e's smoother commanded trajectory but doesn't require it
- **6f** (G³ clothoid) — only if 6e's hardware testing reveals residual ringing at quintic curvature-ramp regions

Don't start 6f speculatively. The 6d+6e+6g combination may already be sufficient for ultimate performance + quality.
