# Naive-CAM Collinearity Prepass — Design Spec

**Date:** 2026-04-17
**Scope:** Stage 1 sub-spec #3 of the corner-blending fork (see `2026-04-16-phase0-research/00-summary.md`).
**Status:** Design approved, ready for implementation planning.

---

## Purpose

Slicers emit curves as chains of many short collinear line segments. When a corner occurs between two such chains, the downstream blend-geometry module sees only the **last** micro-segment on each side as `L_prev` / `L_next`, which collapses the radius cap `R_mid = min(L_prev, L_next) / tan(θ/2)` and, transitively, the cornering velocity.

The prepass consolidates adjacent XYZ-collinear moves (within a geometric tolerance and with stable extrusion / speed parameters) into single longer moves **before** they reach the lookahead queue. Effect: the blender sees realistic segment lengths on both sides of a corner, and the lookahead / trapq process fewer Move objects.

This is the Kalico analogue of LinuxCNC's `emccanon.cc::linkable()` naive-CAM consolidator (pre-TP stage) and Siemens's COMPCAD block compressor. Phase 0 research identifies it as table stakes for arc blending to deliver its promised velocities.

## Non-goals

Not this spec:

- The blend geometry itself. (`blendmath.py`, sub-spec #1, landed.)
- Effective jerk `j_eff` derivation. (`blendshaper.py`, sub-spec #2, landed.)
- Planner integration (wiring `blend_geometry` into `toolhead.py`). (Sub-spec #4.)
- Removing SCV / `square_corner_velocity` / `_calc_junction_deviation`. (Sub-spec #5.)
- Pressure-advance synchronization with input shaping. (Separate initiative; see **Prior art & future compatibility** below.)
- User-tunable prepass tolerance. The tolerance is a module-level constant for v1; add a `danger_options` override later if empirical tuning proves necessary.

## Module layout

File: `klippy/blendprepass.py`. Pure Python, standard library only. Independently unit-testable without a running toolhead.

```
klippy/blendprepass.py
├─ NAIVE_CAM_TOLERANCE   = 25e-3 mm    # perpendicular deviation cap (25 µm)
├─ NAIVE_CAM_MAX_CHAIN   = 100         # matches LinuxCNC
├─ NAIVE_CAM_EPM_REL     = 1e-2        # E-per-XYZ-mm relative tolerance (1%)
├─ NAIVE_CAM_F_REL       = 1e-6        # cruise-velocity equality (1 ppm)
├─ NAIVE_CAM_MIN_SEG_LEN = 1e-9 mm     # XYZ-length floor to avoid div-by-zero
│
└─ class CollinearCollapser:
      def __init__(self, toolhead)
      def feed(self, move: Move) -> list[Move]
      def flush(self)             -> list[Move]
```

`toolhead` is accepted so the collapser can construct replacement `Move` objects via the standard `Move(toolhead, start, end, speed)` constructor — it does not read mutable toolhead state beyond what `Move.__init__` reads.

## Algorithm

### State

`self._chain: list[Move]` — buffered moves that have passed the merge gate but not yet been emitted. `self._chain[0].start_pos` is the chain anchor.

### `feed(move)`

```
1. If move.move_d < NAIVE_CAM_MIN_SEG_LEN:
       return [move]                                    # zero-length: pass through
2. If not move.is_kinematic_move:
       return self._flush_chain() + [move]              # E-only / special: breaks chain
3. If self._chain is empty:
       self._chain = [move]
       return []
4. If len(self._chain) >= NAIVE_CAM_MAX_CHAIN:
       emitted = self._flush_chain()
       self._chain = [move]
       return emitted
5. If not self._merge_gate_passes(move):
       emitted = self._flush_chain()
       self._chain = [move]
       return emitted
6. Else:
       self._chain.append(move)
       return []
```

### `flush()`

Drains any buffered chain. Returns 0 or 1 Move objects. Called when the caller knows no more moves will arrive in the current stream (end of gcode command, dwell, homing, etc.).

```
if self._chain is empty:
    return []
return self._flush_chain()
```

### `_merge_gate_passes(candidate)` — strict gate

All four conditions must hold:

**(a) Speed equality** — same commanded cruise velocity:

```
|candidate.max_cruise_v2 − self._chain[0].max_cruise_v2|
    <= NAIVE_CAM_F_REL · max(candidate.max_cruise_v2, self._chain[0].max_cruise_v2)
```

**(b) Extrusion-ratio equality** — same E-per-XYZ-mm:

```
|candidate.axes_r[3] − self._chain[0].axes_r[3]|
    <= NAIVE_CAM_EPM_REL · max(|candidate.axes_r[3]|, |self._chain[0].axes_r[3]|, 1e-9)
```

**(c) Geometric collinearity** — every existing intermediate endpoint `P_k = self._chain[k].end_pos` (for `k < len(self._chain)`) lies within tolerance of the line from anchor `A = self._chain[0].start_pos` to the proposed new end `B = candidate.end_pos`. Using 3D cross product:

```
AB = B - A                                # new proposed chord
if |AB| < NAIVE_CAM_MIN_SEG_LEN:          # candidate ends at anchor (U-turn closure)
    return False                           # reject; pathological
for each P_k:
    AP = P_k - A
    perp_dist = |AP × AB| / |AB|
    if perp_dist > NAIVE_CAM_TOLERANCE:
        return False
```

**(d) Segment-projection bounds** — each intermediate endpoint must project onto the chord *interior*, not behind `A` or past `B`. This guards against U-turns where perpendicular distance is zero but the motion reverses along the chord.

```
for each P_k:
    AP = P_k - A
    t_k = (AP · AB) / (AB · AB)
    if not (0.0 <= t_k <= 1.0):
        return False
```

The projection check also enforces monotonic progress along the chord, which is what "collinear" physically means.

### `_flush_chain()`

```
if len(self._chain) == 1:
    result = self._chain
else:
    result = [self._build_merged_move(self._chain)]
self._chain = []
return result
```

### `_build_merged_move(chain)`

```
start_pos = chain[0].start_pos
end_pos   = chain[-1].end_pos
# Use the shared cruise velocity (validated equal in gate (a))
cruise_v  = math.sqrt(chain[0].max_cruise_v2)
merged    = Move(self._toolhead, start_pos, end_pos, cruise_v)
# Preserve the narrowest accel seen across the chain; defensively
# same as merged.accel unless a limit_speed() fired upstream.
merged_accel = min(m.accel for m in chain)
if merged_accel < merged.accel:
    merged.limit_speed(cruise_v, merged_accel)
return merged
```

`Move.__init__` recomputes `axes_d`, `axes_r`, `move_d`, `min_move_t`, `delta_v2`, `smooth_delta_v2`, `max_start_v2 = 0.0`, `next_junction_v2 = 999999999.9`. None of those are copies from any source move — they're recomputed from `start_pos`, `end_pos`, and `cruise_v`. The resulting merged Move is indistinguishable from one the caller would have produced natively.

`merged.junction_deviation` comes from the toolhead, matching what any other Move sees.

**Material conservation check (testing only, not runtime):** `sum(m.axes_d[3] for m in chain)` must equal `merged.axes_d[3]` within float precision. Same for XYZ components.

## Integration with `ToolHead`

Minimal-diff integration in `klippy/toolhead.py`:

**Constructor** (near existing `self.lookahead = LookAheadQueue(self)`):

```python
from . import blendprepass
...
self.prepass = blendprepass.CollinearCollapser(self)
```

**`move(newpos, speed)`** — existing body:

```python
def move(self, newpos, speed):
    move = Move(self, self.commanded_pos, newpos, speed)
    if not move.move_d:
        return
    if move.is_kinematic_move:
        self.kin.check_move(move)
    if move.axes_d[3]:
        self.extruder.check_move(move)
    self.commanded_pos[:] = move.end_pos
    for m in self.prepass.feed(move):
        self.lookahead.add_move(m)
    if self.print_time > self.need_check_pause:
        self._check_pause()
```

Only change: `self.lookahead.add_move(move)` → `for m in self.prepass.feed(move): self.lookahead.add_move(m)`.

**Flush sites** — every place that currently calls `self.lookahead.flush(...)` or `self._flush_lookahead(...)` must first drain the prepass:

```python
for m in self.prepass.flush():
    self.lookahead.add_move(m)
self.lookahead.flush(...)
```

Grep `toolhead.py` for `lookahead.flush(` and `_flush_lookahead`; each call site gets the two-line prefix. Expected call sites (to be verified at implementation): `_flush_lookahead`, `wait_moves`, `dwell`, `drip_move`, `set_position` / homing transitions. Each is a local change.

**Why `check_move` runs pre-merge, not post-merge:** kinematics checks (position envelope, extruder extrude-only-accel) are invariant under consolidation of collinear moves: if every constituent satisfies the limits, the merged move satisfies them. The merged move's `move_d` is the sum of constituent `move_d`s, `axes_d` is the vector sum (preserved because the constituents are XYZ-collinear and XYZ-monotonic, validated by gate (d)), and `max_cruise_v2` / `accel` are inherited from the constituents. No new check is required on the merged move.

## Edge cases & degeneracies

1. **Zero-length move** (`move_d < 1e-9`): short-circuit in step 1; emit untouched, chain unchanged.
2. **E-only move** (step 2): flushes chain, emits chain result + the E-only move unchanged. Classic PA `calc_junction` still fires; behavior identical to current code from the moment the E-only move reaches the lookahead.
3. **Z-hop or Z-only travel**: not collinear with adjacent XY moves (unless the chain itself was Z-monotonic, which is rare — vase mode is the exception). The collinearity / projection gate naturally rejects. Chain flushes.
4. **Arachne variable-width walls**: E-per-mm changes per segment; gate (b) rejects every candidate after the first. Each Arachne sub-segment flows through the lookahead individually, preserving the original extrusion profile. This is the intended behavior (see **Rationale** below).
5. **Scarf seam ramps**: same mechanism as Arachne — gate (b) rejects.
6. **Bridge / dynamic-overhang speed steps**: F changes per segment; gate (a) rejects.
7. **U-turn disguised as collinear**: perpendicular distance can be 0 on a reversal; gate (d) catches it via projection `t_k ∉ [0, 1]`.
8. **Chain starting with zero-E motion, later gaining E**: `axes_r[3]` jumps from 0 to non-zero; gate (b) rejects.
9. **Candidate ending at anchor** (degenerate `|AB| < 1e-9`): rejected inside the gate to avoid div-by-zero.
10. **Chain cap hit at 100 moves**: flushes 100, starts fresh chain with the 101st move. The 100-move chunk is emitted; no work is lost.
11. **Float-precision accumulation**: all checks are computed against the anchor `self._chain[0].start_pos`, so consecutive merges do not compound float error. 25 µm tolerance is three orders of magnitude above float64 noise on mm-scale coordinates.

## Testing (`test/test_blendprepass.py`)

All tests use pytest, follow the existing `test_blendmath.py` / `test_blendshaper.py` conventions, and avoid loading the toolhead C module.

### Fakes required

- `_FakeToolhead` with `max_velocity`, `max_accel`, `max_accel_to_decel`, `junction_deviation`, `extruder` stubs — enough for `Move.__init__` to run without a real printer.
- `_make_move(start, end, speed, toolhead)` factory that calls the real `Move` constructor.

### Unit tests

1. **Single-move passthrough** — `feed(move)` on an empty collapser returns `[]`, `flush()` then returns `[move]`.
2. **100 collinear-constant-flow moves** merge into 1 on a subsequent `flush()`. Verify `axes_d` sum, `move_d` sum, preserved `max_cruise_v2` and `accel`.
3. **101st move** triggers chain cap: returns 1 merged move from `feed` of the 101st, buffered chain restarts.
4. **Collinearity break** — 50 collinear + 1 move with 50 µm perpendicular offset (> 25 µm). Gate rejects; returns the 50-merged chain; new chain begins with the offset move.
5. **Collinear within tolerance** — 10 moves with 20 µm perpendicular offsets each. Must merge (every `P_k` is within 25 µm of the current anchor-to-new-end line). Verify all 10 still merge into 1.
6. **Speed change** — chain of 5 at F=10000 mm/min, 6th at F=10001 mm/min (> 1 ppm). Gate (a) rejects.
7. **Flow change** — chain of 5 at E-ratio 0.5, 6th at E-ratio 0.505 (> 1% rel). Gate (b) rejects.
8. **U-turn** — move A: X=0→10, move B: X=10→0. Perpendicular distance is 0; gate (d) rejects via projection.
9. **E-only move mid-chain** — flushes chain, emits chain + the E-only move.
10. **Z-hop mid-chain** — flushes chain (perpendicular distance exceeds tolerance).
11. **Zero-length move** — passthrough regardless of chain state; chain unchanged.
12. **Float-precision sanity** — chain of 100 moves of length 0.001 mm each; post-merge `move_d` equals 100 · 0.001 within 1e-10 mm.
13. **Preservation** — post-merge `axes_d[i]` for i=0..3 equals `sum(m.axes_d[i] for m in chain)` within float precision.
14. **3D collinearity** — vase-mode chain with small Z per move; merges if Z increments are monotonic and within perpendicular tolerance.

### Property tests (hypothesis)

15. **Random collinear chains with per-step offset < tolerance** always merge. Generator: anchor, direction unit vector, N ∈ [2, 100] segments with lengths ∈ [0.01, 10] mm and perpendicular noise ∈ [-20 µm, 20 µm]. Assert: chain of N returns 1 merged move.
16. **Random chains with one offset-violating step** always split at the violation. Assert: exactly two output moves (the chain up to violation, then the rest starting at the offending move).
17. **Total displacement preserved** across random arbitrary valid chains: `sum(merged.axes_d[i]) == sum(all original axes_d[i])` within float precision.

### Regression fixtures

Each edge case from the section above as an explicit test with inline numeric values. Don't rely on randomness for regression tests.

## Dependencies

- Depends on nothing new. `Move` class already exists in `toolhead.py`.
- No C changes.
- No dependency on `blendmath.py` or `blendshaper.py` — the prepass runs strictly upstream of them.
- No dependency on the planner-integration sub-spec (#4). When sub-spec #4 wires blend geometry into the lookahead, the prepass output is the input to that wiring; no contract change.

## Rationale

### Why strict gate (not loose)

Three research threads informed this choice (see `2026-04-16-phase0-research/` and follow-up analyses):

1. **Slicer behavior** — modern slicers (PrusaSlicer, OrcaSlicer, SuperSlicer, Cura, Bambu) emit variable E-per-XYZ-mm *only* for intentional features:
   - Arachne variable-width walls (PrusaSlicer 2.5+ / Orca)
   - Scarf seam ramps
   - Small-Area Flow Compensation (Orca)
   - Z-sloped scarf / conical vase mode
  Source: `PrusaSlicer` `PerimeterGenerator.cpp:152-154` explicit comment *"this value determines granularity of adaptive width, as G-code does not allow variable extrusion within a single move"*. The slicer subdivides the straight wall into collinear paths *specifically so* each sub-path can carry its own `mm3_per_mm`. Merging them erases the feature.
   Otherwise — >95% of collinear chains have constant E-per-mm, arising from STL-facet polyline approximation of a single `ExtrusionPath` with a single `mm3_per_mm`.

2. **Pressure-advance interaction** — classic Klipper PA is `e_cmd(t) = e_nominal + PA · v_e(t)` convolved with a tent kernel of width `smooth_time`. Total extruded material is conserved by any merge strategy (tent kernel integrates to 1). Kalico's `extruder.calc_junction` caps XYZ junction speed at `v_cap = instant_corner_v / |Δaxis_r[3]|` — typically 1.0 mm/s ÷ small flow change ≈ 5 mm/s. This fires on every Arachne/Scarf flow step today; strict merging preserves those junctions because they are the slicer's explicit request. Sources: `klippy/chelper/kin_extruder.c:15-131` (PA transform + tent kernel), `klippy/kinematics/extruder.py:328-332` (`extruder_v2 = (instant_corner_v / |diff_r|)²`).

3. **Future PA-sync compatibility** — dmbutyugin's `bleeding-edge-v2` PA-synchronization-with-input-shaping work (not on this branch) multiplies pre-shaper XY jerk into extruder acceleration. At a direction discontinuity this demands impulse-level extruder accel (`a_E ≈ PA · ΔV / T_shaper²`) that direct-drive extruders can't deliver. The blend-arc work this fork is building turns sharp corners into circular arcs at constant `|v_xy_cmd|`; under any LTI shaper, a constant-magnitude circular input produces a constant-magnitude circular output (derivation: for shaper pulses `Σ a_i δ(t − τ_i)`, `|v_xy_shaped|² = v_arc² · |H(jω)|²` with `ω = v_arc / R` — a constant). So blend-arc dissolves the PA-sync accel spike: extruder accel demand through a blended corner is zero. The prepass is **neutral** to this: it emits fewer junctions, each with shorter or unchanged velocity transitions, which strictly helps any future PA-sync merge.

### Why 25 µm tolerance

- PrusaSlicer / Orca default `gcode_resolution = 0.0125 mm = 12.5 µm` ([source](https://github.com/prusa3d/PrusaSlicer/blob/master/src/libslic3r/PrintConfig.cpp)).
- Other slicers emit similar or coarser quantization (Cura ~0.05 mm).
- 25 µm is 2× PrusaSlicer's quantization floor — captures slicer-quantization noise and STL-facet drift without risking merges across intentional geometry changes.
- One order of magnitude below the Phase 0 default blend-arc chord-error target (10 µm per segment at R=0.5 mm, arising from ~0.2 mm polyline segment length — Bucket E analysis).

### Why 1% E-per-mm, 1 ppm F

- 1% E: captures float-precision and slicer-quantization noise (e.g. small rounding in `e_per_mm3 * mm3_per_mm` within one ExtrusionPath) while rejecting intentional Arachne width variation (typically 2–40% per sub-path).
- 1 ppm F: F values are emitted as integer `mm/min` in gcode; two successive moves at the same commanded F arrive as identical float values. 1 ppm is effectively exact-equality with a safety margin.

### Why chain cap 100

- Matches LinuxCNC's `emccanon.cc::linkable()`.
- Prevents pathological inputs (malformed gcode, infinite-loop macros) from unbounded memory growth.
- At 100 collinear segments per chunk, downstream trapq and lookahead see "real" segment lengths anyway; incremental chunk flushing is fine.

## Prior art & future compatibility

- **LinuxCNC `emccanon.cc::linkable()`** — direct algorithmic reference; our implementation matches its perpendicular-distance-from-anchor-to-new-end test, adds the projection-bound check for U-turn safety, and adds the strict gate on F and E-per-mm that CNC doesn't have.
- **Siemens COMPCAD / COMPCURV** — NC-block compressor; higher-order (polynomial spline fit) variant of the same concept.
- **Prunt** — runs a dedicated preprocessing stage before its Bézier corner blender. Same architectural placement.
- **Bleeding-edge-v2 PA sync** — future compatibility is by construction: the prepass emits fewer junctions and shorter post-shaper transients, strictly helping the junction-spike behavior (`a_E ≈ PA · ΔV / T_shaper²`). No coupling in the other direction.

## Validation gate before shipping (Stage 1 wrap)

Measure on real hardware once blend-arc and the prepass are wired together (sub-spec #4):

1. **Merge rate per print** — on a representative Arachne-on print (single-wall curved model), fraction of moves consolidated by the prepass. Expected: 50–90% of moves on curve-dominated prints, <5% on orthogonal-geometry prints.
2. **Corner velocity improvement** — A/B prints of a shape that historically collapses cornering velocity (e.g. a 50 mm radius arc approximated as 500 polyline segments). Measure wall-time and commanded velocity at the critical corner with prepass off vs on; expected: 5–20× velocity increase on the curve-dominated case.
3. **Extrusion profile integrity** — A/B visual comparison of Arachne variable-width walls and Scarf seams. Expected: indistinguishable to the eye; E-per-mm varies per sub-segment as slicer emitted.

These are integrated-system gates, not unit tests — they confirm the prepass delivers the blend-arc velocity benefit without regressing print quality.

## References

- Phase 0 research: `docs/superpowers/specs/2026-04-16-phase0-research/`
  - Summary convergence on naive-CAM prepass as Stage 1 table stakes: `00-summary.md:47, 107-110`
  - LinuxCNC source reference: `A-industrial-cnc.md:27-29`
- Blend-geometry sub-spec (landed): `docs/superpowers/specs/2026-04-16-blend-geometry-module-design.md`
- j_eff sub-spec (landed): `docs/superpowers/specs/2026-04-17-j-eff-derivation-design.md`
- Kalico extruder model: `klippy/kinematics/extruder.py:290-363`, `klippy/chelper/kin_extruder.c:15-131`
- Slicer source (Arachne rationale): https://github.com/prusa3d/PrusaSlicer/blob/master/src/libslic3r/PerimeterGenerator.cpp (lines 152–154)
- dmbutyugin PA-sync (out-of-scope future work): `KalicoCrew/kalico` branch `bleeding-edge-v2`, `klippy/chelper/kin_extruder.c`
- Discourse: Extruder PA synchronization with input shaping — https://klipper.discourse.group/t/extruder-pa-synchronization-with-input-shaping/3843
- Duet RRF 3.5 Input Shaping docs — https://docs.duet3d.com/User_manual/Tuning/Input_shaping
