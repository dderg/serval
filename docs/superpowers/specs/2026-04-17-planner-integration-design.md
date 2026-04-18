# Planner Integration — Design Spec

**Date:** 2026-04-17
**Scope:** Stage 1 sub-spec #4 of the corner-blending fork (see `2026-04-16-phase0-research/00-summary.md`).
**Status:** Draft, awaiting review.

---

## Purpose

Wire the three standalone feeder modules (`blendmath`, `blendshaper`, `blendprepass`) into the main motion pipeline so fine-segmented circular-arc blends replace the existing `square_corner_velocity` / junction-deviation cornering between every pair of non-collinear moves. Output: each original sharp corner is replaced by a tangent arc, emitted as a chain of small linear `Move` objects that flow through the existing `LookAheadQueue` → `trapq` → `itersolve` pipeline unchanged.

## Non-goals

Not this spec:

- The blend geometry itself (`blendmath.py`, #1 — landed).
- Jerk ceiling derivation from input shaper (`blendshaper.py`, #2 — landed).
- Collinear chain consolidation (`blendprepass.py`, #3 — landed).
- Removing `square_corner_velocity`, `_calc_junction_deviation`, related `SET_VELOCITY_LIMIT` args, or the `junction_deviation` field on `Move`. (Sub-spec #5.)
- Updating `find_shaper_max_accel`'s `offset_90` term for the new kinematic model. (Sub-spec #6.)
- Final user-facing parameter name. (Sub-spec #7.) This spec uses `corner_deviation` as a placeholder.
- Docs / example configs. (Sub-spec #7.)
- G² Bézier upgrade. (Stage 3.)

## Module layout

New file: `klippy/blendplanner.py`. Python 3. Depends on `klippy.blendmath` only (which pulls `blendshaper`). No direct Kalico imports in the core class; `Move` is injected via the `move_cls` constructor kwarg, matching the pattern established in `blendprepass.py`.

```
klippy/blendplanner.py
├─ CornerBlender (class)                       # stateful feed/flush/reset filter
│    .tolerance              float    corner_deviation (mm), required, set by toolhead
│    .max_chord_err          float    polyline-chord tolerance (mm), default 10e-3
│    .min_straight_len       float    micro-straight floor (mm), default 0.0 (= emit all)
│    .micro_straight_count   int      instrumentation counter
│    .feed(move) -> list[Move]
│    .flush() -> list[Move]
│    .reset() -> None
└─ BlendPipelineLookAheadQueue(filters, lookahead)  # generic N-stage adapter
     # Replaces PrepassLookAheadQueue. Accepts any ordered list of filter
     # objects each exposing feed/flush/reset, and composes them before the
     # inner LookAheadQueue. Empty filters list is valid (passthrough).
```

`CornerBlender` exposes exactly the filter protocol used by `CollinearCollapser`: `feed(move) -> list[Move]`, `flush() -> list[Move]`, `reset() -> None`. This lets a single adapter compose both in order without special-casing either.

`BlendPipelineLookAheadQueue` is a rename / generalization of `PrepassLookAheadQueue` (sub-spec #3's adapter). It accepts a list of filters and pipes each incoming `Move` through all of them in series before handing the survivors to the inner `LookAheadQueue`. The `PrepassLookAheadQueue` type alias is kept as a no-op compatibility name during the refactor and deleted in the same sub-spec once all callers switch.

## Pipeline

```
ToolHead.move(newpos, speed)
    ├─ Move(…) instantiation
    ├─ kin.check_move(move)
    ├─ extruder.check_move(move)
    └─ self.lookahead.add_move(move)
            │
            ▼
   BlendPipelineLookAheadQueue.add_move
            │  pipes through each filter in order:
            ├─ CollinearCollapser.feed(move)  → emits 0 or more consolidated Moves
            ├─ CornerBlender.feed(move)       → emits 0 or more (truncated-prev + arc polyline) Moves
            └─ LookAheadQueue.add_move(each)  → existing planner; no change
```

Flush cascades through the same chain. `get_last` drains both filters before consulting the inner queue. The `queue` property reports buffered-filter contents concatenated with inner-queue contents so `check_busy` and similar emptiness probes see a faithful backlog.

## Algorithm

### State

`CornerBlender` buffers exactly one move — the candidate `prev` for the next blend. It has no other internal state beyond tunables and the instrumentation counter.

### `feed(move)`

```
1. If move.is_kinematic_move is False:
       return flush() + [move]              # E-only / special: break the chain
2. If no buffered prev:
       self._prev = move
       return []
3. arc = blend_from_moves(prev, move, corner_deviation, toolhead=toolhead)
   where prev = self._prev
4. If arc is None:                          # collinear; prepass should have caught
       emitted = [self._prev]
       self._prev = move
       return emitted
5. If arc.R == 0.0 or arc.v_cap == 0.0:     # U-turn / degenerate
       self._prev.limit_next_junction_speed(0.0)
       emitted = [self._prev]
       self._prev = move
       return emitted
6. prev_trunc, arc_moves, next_trunc_head = _emit_arc(self._prev, move, arc)
7. self._prev = next_trunc_head
8. return [prev_trunc] + arc_moves
```

### `flush()`

```
1. If no buffered prev:
       return []
2. emitted = [self._prev]
3. self._prev = None
4. return emitted
```

### `reset()`

```
self._prev = None
```

### `_emit_arc(prev, next, arc)`

Constructs three pieces:

1. **Truncated prev.** Start at `prev.start_pos`, end at `prev.end_pos - arc.d_consumed · prev_dir`. Same speed / accel as original `prev`. `Move.__init__` recomputes `move_d`, `axes_r`, junction fields correctly from the new endpoints. E-per-mm is preserved by carrying the original 3-axis `axes_r[3]` ratio: `end_e = start_e + (new_move_d / prev.move_d) · prev.axes_d[3]`.

2. **Arc polyline.** Call `blendmath.segment_arc(arc, max_chord_err)` → list of 3D points in **local corner frame** (vertex at origin). Translate each by the shared corner vertex (= `prev.end_pos[:3]`) to get world coordinates. Attach E coordinates via `blendmath.interpolate_extruder(points, arc.d_consumed, prev.axes_r[3], next.axes_r[3])`. For each consecutive point pair, construct `Move(toolhead, p_i, p_{i+1}, sqrt(min(prev.max_cruise_v2, next.max_cruise_v2, arc.v_cap²)))`. Set `accel = min(prev.accel, next.accel)` via `limit_speed`.

3. **Truncated next head.** Start at `next.start_pos + arc.d_consumed · next_dir`, end at `next.end_pos`. Same speed / accel as original `next`. E-per-mm preserved analogously. Carries `next.next_junction_v2` and `next.timing_callbacks` forward (those attach to the TAIL of the original move, which is still in the truncated-next-head version since it hasn't been truncated at its tail yet).

### `_emit_arc`: micro-straight policy (option i)

The truncated-prev and truncated-next-head pieces can have very small `move_d` (< 0.01 mm) when segments are short relative to blend consumption. Per-decision (see **Prior art**): emit them anyway. **Instrumentation:** `self.micro_straight_count += 1` for each piece with `move_d < self.min_straight_len` (default 0.0, so counter increments on every sub-threshold piece). Exposed as an integer attribute for the stats layer to read.

## Geometry change: half-segment rule

Independent math review (subagent, `2026-04-17`) flagged that `blendmath.blend_geometry`'s midpoint cap

```
R_mid = min(L_prev, L_next) / tan(θ/2)        # current
```

allows each of two neighboring corners to unilaterally claim up to the full shared segment length. Non-overlap then depends on the processing order of the downstream look-ahead — first corner is greedy, last is squeezed. LinuxCNC's `blendmath.c` (line 1031) and every reviewed industrial reference use the **half-segment rule**:

```
R_mid = 0.5 · min(L_prev, L_next) / tan(θ/2)  # revised
d_consumed = R · tan(θ/2) ≤ 0.5 · min(L_prev, L_next)
```

Each blend claims at most half the adjacent segment from its side. Neighboring blends meet at most at the midpoint → guaranteed non-overlap, fairness independent of pass direction, no recursive bookkeeping.

**Change required in this sub-spec:**

- `klippy/blendmath.py:145`: multiply by `0.5`.
- `test/test_blendmath.py`: update analytic fixtures that hit the midpoint cap (affects symmetric short-segment cases). Fixtures computing `R_tol` only are unaffected.

This is the narrowest possible update. `BlendArc` struct and caller API are unchanged.

**Note for Zhao 2022 upgrade:** the proportional-share formulation from Zhao et al. (J. Manuf. Proc. 2022) is optimal on short-segmented paths — it shares the gap proportional to each corner's tolerance-unconstrained demand rather than blindly halving. Deferred to Stage 2 if/when measurements show the half-segment rule is conservatively capping throughput.

## Corner-deviation parameter

New config entry in the `[printer]` section: `corner_deviation` (placeholder name; sub-spec #7 may rename). Required. No default. Parsed with `config.getfloat("corner_deviation", above=0.0)`. Read once at `ToolHead.__init__` and passed to `CornerBlender`.

Rationale for "required, no default": this is a blend-arc fork. The parameter replaces `square_corner_velocity` semantically; users migrating a config must make an explicit choice. Picking a default now without measurement risks picking a bad one. Sub-spec #7 may introduce a measured-sensible default once Stage 1 validation has data.

If `corner_deviation` is missing from config, `ToolHead.__init__` raises `config.error` with a message naming the new parameter and pointing to docs (sub-spec #7 populates).

## Config interactions during the transitional period

`square_corner_velocity` / `junction_deviation` / `_calc_junction_deviation` remain in place (sub-spec #5 deletes them). During the interim:

- `Move.__init__` continues to snapshot `toolhead.junction_deviation` into every constructed move, including the emitted arc-polyline and truncated pieces. Harmless: subagent-verified math confirms the JD check is never binding at near-tangent polyline junctions (`v² ≤ 8·JD·a·R²/s²`, loose by orders of magnitude in our operating envelope).
- `calc_junction`'s centripetal term `v² ≤ a·R·(α/2)cot(α/2)` yields `v² ≤ a·R` at polyline-internal junctions, slightly looser than the arc's own cap `v² ≤ 0.866·a·R`. Binding is always on the arc cap first.
- At truncated-prev → first arc polyline junction: exactly tangent by construction. `cos(θ/2) = 0` in code's convention, so the entire JD/centripetal block short-circuits. No over-cap.
- At last arc polyline → truncated-next-head: same tangent-by-construction behavior.

Net: no planner change required. The existing `calc_junction` silently passes the arc's `max_cruise_v2` through.

## Degenerate cases

1. **Collinear junction (`blend_from_moves` returns `None`).** `CollinearCollapser` should have merged; reaching `CornerBlender` means the corner slipped past one of its gates (rare). Emit `prev` unchanged, buffer `next`. No velocity constraint imposed — the tangent is exact.
2. **U-turn / near-reversal (`v_cap == 0` or `R == 0`).** Emit `prev` with `limit_next_junction_speed(0)`. Buffer `next` unchanged. Toolhead stops at the vertex, as today.
3. **First move of a session / after `reset()`.** No buffered prev → buffer it, emit nothing.
4. **Non-kinematic (E-only) move.** Flush buffered prev, emit the E-only move unchanged. Cannot participate in a corner blend.
5. **Arc consumption exceeds `next.move_d`.** Half-segment rule guarantees `d_consumed ≤ 0.5 · next.move_d` by construction. `next_trunc_head.move_d ≥ 0.5 · next.move_d > 0`. No degeneracy path.
6. **Arc consumption exceeds prev.move_d when prev was already head-truncated by the previous blend.** Cannot happen. The gap-invariant proof (below) shows the straight between two adjacent arcs is always ≥ `0.25 · L_shared`. The blender never emits a zero-length truncated-prev unless the upstream segment itself was zero-length, which the prepass filters.
7. **Corner inside a drip move.** Drip mode flushes lookahead aggressively (`toolhead.py:686–709`). Drip-mode moves still pass through the blender; the flush at the end of `drip_move` drains any buffered prev. No special handling.
8. **Kinematic / extruder `check_move` on emitted arc polyline.** Each emitted Move already passes through `check_move` when added to the inner `LookAheadQueue`? **NO — `check_move` is called in `ToolHead.move`, before `lookahead.add_move`.** The blender emits additional Moves directly into the inner queue that bypass `check_move`. The truncated pieces inherit their validity from the originals, which already passed. The arc polyline has no extrude-only nor invalid-position concerns by construction (points sit on a tangent arc between two validated endpoints, all three axes monotone within the vertex locality). Cover this with tests; if a pathological case emerges, add a post-emit `kin.check_move` loop analogous to `blendprepass.py:131-134`'s aggregate-safety re-check.

## Instrumentation

- `CornerBlender.micro_straight_count` (int, cumulative). Logged at stats interval via a small patch in `ToolHead.stats` — appended to the existing stats string as `micro_straights=%d`. Plan for removal once monitoring data shows it's quiet.
- Emitted polyline length per blend is NOT logged per-move (too chatty). A separate counter `CornerBlender.polyline_moves_emitted` accumulates and is dumped alongside `micro_straight_count`. Cheap integer ops on the fast path.

These counters fulfill the monitoring requirement agreed during brainstorming: on Stage 1 validation prints, compare `micro_straight_count` / `polyline_moves_emitted` / `print_stall` against tight-corner gcode. Rising `print_stall` correlated with rising polyline count signals step-rate pressure → upgrade to the absorb-micro-straights policy (option ii, deferred).

## Testing

`test/test_blendplanner.py`. Mirrors `test_blendprepass.py` — uses a `_FakeMove` faithful reimplementation (no `pyserial` dependency) and a `_FakeToolhead` exposing only what `Move.__init__` reads.

1. **Unit: single corner.** 90° corner between two 10 mm moves, corner_deviation = 50 µm. Verify emitted sequence: `[trunc_prev, arc[0], …, arc[N-1], trunc_next_head]` (where `next_trunc_head` is returned on the NEXT `feed`). Verify all points lie on the arc within `max_chord_err`. Verify `sum(arc_move.move_d) ≈ R · θ` within tolerance. Verify E conservation: sum of E across truncated-prev + polyline + truncated-next-head equals original prev.axes_d[3] + next.axes_d[3].
2. **Unit: 60° corner with asymmetric segment lengths (2 mm + 10 mm).** Half-segment rule caps consumption at 1 mm. Verify `d_consumed == 1.0` within tolerance.
3. **Unit: U-turn (170° deflection).** Verify emitted sequence is `[prev]` with `next_junction_v2 == 0`, next buffered unchanged.
4. **Unit: near-collinear (sub-1° deflection).** Verify emitted is `[prev]`, next buffered (None return from `blend_from_moves`).
5. **Unit: E-only move breaks chain.** Verify prev is flushed, E-only passed through.
6. **Unit: flush drains buffered prev.** Emit two normal moves, call flush, verify both ripple out.
7. **Unit: reset drops state.** Same sequence but reset instead of flush — buffered prev discarded, next feed starts fresh.
8. **Pipeline: prepass + blender composition.** Build chain of 10 short collinear moves followed by a 90° turn into another 10 shorts. Verify prepass merges both sides into long moves, blender then blends the one resulting corner.
9. **Pipeline: BlendPipelineLookAheadQueue generic.** Feed an ordered sequence through a 2-filter stack, verify `get_last`, `queue`, `flush`, `reset`, `set_flush_time` all behave transparently.
10. **Regression: speed continuity.** Adjacent arc polyline segments have `max_cruise_v2` within 1 ppm of each other (they share the same cap). Important because the `LookAheadQueue` uses this to plan joint velocity.
11. **Regression: half-segment rule in `blendmath`.** Short-segment fixture where the midpoint cap was binding. Expected `R` drops by factor 2 vs. pre-revision.
12. **Property: random corners.** Seed-parameterized (`@pytest.mark.parametrize("seed", range(50))` as in `test_blendprepass.py`) random 3D corners with random segment lengths. Invariants: non-overlap of consumed segments, emitted polyline chord error within `max_chord_err`, E conservation within 1 ppm.
13. **Integration smoke.** Instantiate a real `ToolHead` with a mocked config, feed three moves, verify the emitted trapq calls include the expected number of kinematic moves (= 2 truncated + polyline count). Use the existing Kalico test harness pattern; if it's too heavy, defer to Stage 1 validation.

## Dependencies

**Must land before wiring:**
- `blendmath` half-segment fix (included in this sub-spec).
- `blendmath.blend_from_moves(…, toolhead=…)` 2-pass adapter (sub-spec #1, landed).
- `CollinearCollapser` prepass (sub-spec #3, landed).

**This sub-spec does not depend on:**
- Sub-spec #5 (SCV/JD removal).
- Sub-spec #6 (Shake&Tune rework).
- Sub-spec #7 (docs/config final).

## Validation gate before shipping (Stage 1 wrap)

The blend-arc model is end-to-end live after this sub-spec lands. Before merging the feature branch to the fork's main, run the Stage 1 validation from `00-summary.md`:

- Real-hardware corner residuals measurement with G¹ arcs engaged.
- Compare `print_stall`, MCU step-rate shutdowns, and `micro_straight_count` on stress prints (dense-corner cubes, spiral vases, text-on-surface) against a pre-blend baseline.
- If residuals exceed the shaper's error envelope → Stage 3 G² upgrade is pulled forward.
- If `print_stall` rises or MCU shutdowns occur → implement option (ii), the absorb-short-straights policy.

## Prior art

- **LinuxCNC `src/emc/tp/blendmath.c`** — half-segment rule at `:1031`, `blendCheckConsume()` short-segment absorption at `:1176-1199`. The single direct industrial reference.
- **Zhao et al., J. Manuf. Proc. 2022** — proportional-share overlap elimination, optimal on short-segmented paths. Deferred to Stage 2.
- **Bi et al. 2015 / 2019** — cubic Bézier (G²) with half-segment cap. Same overlap treatment. Relevant for Stage 3.
- **`klippy/blendprepass.py`** — internal precedent for the `feed / flush / reset` filter protocol and `_FakeMove` test harness pattern.

## Gap invariant between adjacent blends

With the half-segment rule AND this sub-spec's recursive truncation (each blend sees the already-head-truncated prev), the straight between two adjacent arcs on a shared segment of length `L` is always ≥ `0.25 · L`:

```
d_B ≤ 0.5 · min(L_AB, L_BC)          (first blend's half-segment cap)
truncated_BC = L_BC − d_B ≥ 0.5 · L_BC  (when L_AB ≥ L_BC; tighter otherwise)
d_C ≤ 0.5 · truncated_BC ≤ 0.25 · L_BC
gap  = truncated_BC − d_C ≥ 0.25 · L_BC
```

No zero-length truncated straight unless `L_BC` itself was zero (which the prepass already filters). The degenerate-case #6 (zero-length truncated-prev) cannot occur. Removing it from the degenerate list below is fine; kept above for readers tracing the logic.

## Open questions

1. **Drip-mode ordering.** Does the blender behave correctly inside `drip_move` when the single move is fed, then flushed by `lookahead.flush()` immediately? Expected: `feed` buffers prev, `flush` drains it — works. Verify in integration test.
