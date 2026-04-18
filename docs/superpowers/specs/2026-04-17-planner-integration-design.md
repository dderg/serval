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
├─ CornerBlender                                # stateful feed/flush/reset filter
│    __init__(toolhead, *, move_cls, max_chord_err=None)
│        # toolhead     : Kalico toolhead (read access to .corner_deviation, .kin,
│        #                .extruder, .max_accel_to_decel, etc.)
│        # move_cls     : Move class, injected for testability (same as blendprepass)
│        # max_chord_err: override polyline chord tolerance. None → auto-compute
│        #                max(20e-3, 0.2 * toolhead.corner_deviation) per feed.
│    .max_chord_err          float | None         # fixed override or None for auto
│    .polyline_moves_emitted int                  # instrumentation counter
│    .blends_emitted         int                  # instrumentation counter
│    .feed(move)             -> list[Move]
│    .flush()                -> list[Move]
│    .reset()                -> None
│    .peek_buffered()        -> list[Move]
└─ BlendPipelineLookAheadQueue(filters, lookahead)
     # Replaces PrepassLookAheadQueue. Accepts any ordered list of filter
     # objects conforming to the filter protocol and composes them before
     # the inner LookAheadQueue. Empty filters list is valid (passthrough).
```

**`corner_deviation` storage.** Single source of truth: `ToolHead.corner_deviation`. `CornerBlender` reads `self._toolhead.corner_deviation` per `feed()` call (not cached at construction), so a future `SET_VELOCITY_LIMIT CORNER_DEVIATION=` command that mutates `toolhead.corner_deviation` takes effect on the next corner without cross-object sync. The previous revision of this spec had the value mirrored onto `CornerBlender.tolerance`; lazy reading eliminates the anti-pattern.

### Filter protocol (extended from sub-spec #3)

Every filter object plugged into `BlendPipelineLookAheadQueue` exposes:

| Method               | Returns                 | Semantics |
|----------------------|-------------------------|-----------|
| `feed(move)`         | `list[Move]`            | Accept an incoming Move, return 0+ Moves to pass downstream. |
| `flush()`            | `list[Move]`            | End-of-stream drain. Return everything still buffered. State is `None` / empty afterward. |
| `reset()`            | `None`                  | Drop buffered state without emitting. |
| `peek_buffered()`    | `list[Move]`            | **New** — read-only view of currently buffered state. Must not mutate the filter. |

The new `peek_buffered()` method is what `BlendPipelineLookAheadQueue.queue` and `.get_last()` call to inspect filter contents **without** forcing a flush. `CollinearCollapser` (sub-spec #3) grows this method as part of this sub-spec: `return list(self._chain)`. `CornerBlender.peek_buffered()` returns `[self._prev]` when buffered, else `[]`.

### `BlendPipelineLookAheadQueue` behavior

`add_move(move)`
```
acc = [move]
for f in self._filters:
    acc = [out for m in acc for out in f.feed(m)]
for m in acc:
    self._lookahead.add_move(m)
```

`flush(lazy=False)` — two-pass drain:
```
acc = []
for f in self._filters:                             # pass 1: pipe each filter's flush through later filters
    acc = [out for m in acc for out in f.feed(m)]
    acc += f.flush()
for m in acc:
    self._lookahead.add_move(m)
self._lookahead.flush(lazy=lazy)
```

`reset()` — cascades to all filters then the inner queue; order does not matter.

`set_flush_time(t)` — passthrough to inner queue only.

`get_last()` — **does NOT drain filters** (this is the key semantic difference vs. sub-spec #3's prepass adapter, whose flush was side-effect-free). Returns the last move visible across the stack:

```
for f in reversed(self._filters):
    buf = f.peek_buffered()
    if buf:
        return buf[-1]
return self._lookahead.get_last()
```

Rationale: callers (`register_lookahead_callback`, `limit_next_junction_speed`) mutate the returned Move. If `get_last()` flushed the blender, the buffered `_prev` would be emitted *unblended* (no corner arrived yet), permanently forfeiting the blend. The emit-on-blend path (`_emit_arc`) transfers all mutable state set by callers on `_prev` onto the emitted `trunc_prev`, so the callback / junction-limit ends up on a real queued Move.

`queue` property — reports buffered filter contents concatenated with inner queue:
```
result = []
for f in self._filters:
    result += f.peek_buffered()
result += list(self._lookahead.queue)
return result
```

The `PrepassLookAheadQueue` class is renamed to `BlendPipelineLookAheadQueue` in `blendprepass.py`. Its prior internal access of `_prepass._chain` (prepass.py:176) is replaced by the new `peek_buffered()` call. Sub-spec #3's tests update to the new name.

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

Flush cascades through the same chain via the two-pass drain described below. `get_last` does NOT flush filters — it peeks via `peek_buffered()` so callers that mutate the returned Move don't force an unwanted emission. The `queue` property reports buffered-filter contents concatenated with inner-queue contents so `check_busy` and similar emptiness probes see a faithful backlog.

**Migration from sub-spec #3 `get_last` semantics.** The prepass adapter in sub-spec #3 flushed the prepass on `get_last` (blendprepass.py:164-170). This sub-spec changes the semantic to no-flush-on-get-last (peek instead) for both the prepass and the new blender. The sub-spec #3 test `test_adapter_get_last_flushes_prepass_first` (or equivalent) is renamed / rewritten to verify the new peek semantic: callers attaching `timing_callbacks` to a `get_last()` return value receive those callbacks back via `_copy_caller_state` at emit time (through `_build_merged_move` in prepass, `_emit_arc` in blender). Both mechanisms already preserve `timing_callbacks`; the migration is a test-semantic change, not a runtime behavior regression.

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

Construct three pieces, preserving caller-mutated state.

**Shared helper** — `_copy_caller_state(src, dst)`: copies mutable fields that an upstream caller may have set via `get_last()` mutation between buffer-time and emit-time, and recomputes length-derived fields from the new (shorter) `move_d`:

```
# Caller-intent fields: pinned verbatim from parent.
dst.timing_callbacks   = list(src.timing_callbacks)
dst.next_junction_v2   = src.next_junction_v2
dst.max_cruise_v2      = src.max_cruise_v2       # SET_VELOCITY_LIMIT leak guard
dst.junction_deviation = src.junction_deviation
dst.accel              = src.accel               # M204-lowers-accel leak guard

# Length-derived fields: recompute from NEW move_d and pinned accel.
dst.delta_v2           = 2.0 * dst.move_d * dst.accel
ratio = src.smooth_delta_v2 / src.delta_v2 if src.delta_v2 > 0.0 else 1.0
dst.smooth_delta_v2    = min(dst.delta_v2, 2.0 * dst.move_d * dst.accel * ratio)
dst.min_move_t         = dst.move_d / sqrt(dst.max_cruise_v2)
```

The `accel` pin is a DIRECT assignment rather than a `limit_speed(...)` call because `limit_speed` does `min(self.accel, accel)` (toolhead.py:67); if an intervening `M204` had lowered `toolhead.max_accel` between parent construction and emit, `Move.__init__`'s snapshot of the new (lower) value would win over `src.accel`. Direct pin avoids the leak.

`smooth_delta_v2` preserves the parent's ratio of smoothed to full delta (equal to `max_accel_to_decel / accel` when unclamped, or tighter if a kinematic's `check_move` narrowed it via `limit_speed`). This keeps the truncated move's smoothing budget consistent with the parent's, scaled to the shorter length.

**1. Truncated prev.** Start at `prev.start_pos`, end at `prev.end_pos - arc.d_consumed · prev_dir`. Constructed via `self._move_cls(toolhead, start, end, sqrt(prev.max_cruise_v2))`. After construction, call `_copy_caller_state(prev, trunc_prev)`. E is preserved by carrying the original per-mm rate: `end_pos[3] = start_pos[3] + (trunc_prev.move_d / prev.move_d) * prev.axes_d[3]`.

**2. Arc polyline.** Call `blendmath.segment_arc(arc, self.max_chord_err)` → list of 3D points in the local corner frame (vertex at origin). Translate each by the shared corner vertex (`prev.end_pos[:3]`) to world coordinates. Attach E via `blendmath.interpolate_extruder(points, arc.d_consumed, prev.axes_r[3], next.axes_r[3])`. For each consecutive point pair, construct `arc_move_k = Move(toolhead, p_k, p_{k+1}, sqrt(min(prev.max_cruise_v2, next.max_cruise_v2, arc.v_cap**2)))`. Then for each:

```
arc_move_k.max_cruise_v2      = min(prev.max_cruise_v2, next.max_cruise_v2, arc.v_cap ** 2)
arc_move_k.junction_deviation = min(prev.junction_deviation, next.junction_deviation)
arc_move_k.smooth_delta_v2    = arc_move_k.delta_v2      # treat the arc as cruise for
                                                         # look-ahead smoothing; otherwise
                                                         # tiny polyline segments produce
                                                         # artificial gentle ramps around
                                                         # the arc boundary
arc_move_k.min_move_t         = arc_move_k.move_d / sqrt(arc_move_k.max_cruise_v2)
arc_move_k.limit_speed(sqrt(arc_move_k.max_cruise_v2), min(prev.accel, next.accel))
```

Aggregate-safety re-check — per the sub-spec #3 precedent (`blendprepass.py:131-134`): run `kin.check_move(arc_move_k)` and, if it extrudes, `extruder.check_move(arc_move_k)` on **at least one** representative arc move per blend (the first is sufficient — all arc moves share accel, v_cap, and per-mm E rate; spatially the polyline is localized near the corner vertex so envelope checks evaluate at roughly the same coordinates across all points). This catches extruder-max-velocity violations when the arc's per-mm E rate exceeds the extruder's throughput, which is bypassed otherwise (emitted moves skip `ToolHead.move`'s validation).

**3. Truncated next head.** Start at `next.start_pos + arc.d_consumed · next_dir`, end at `next.end_pos`. E-per-mm preserved: `end_pos[3] = start_pos[3] + (trunc_next_head.move_d / next.move_d) * next.axes_d[3]`. Constructed and state-copied analogously: `_copy_caller_state(next, trunc_next_head)`. Carries `next.next_junction_v2` (the tail of the original next) forward — `trunc_next_head` still terminates at the original `next.end_pos`, so end-of-move hooks fire at the correct physical point.

### Extruder cap at arc boundaries

`Move.calc_junction` at `toolhead.py:83` invokes `self.toolhead.extruder.calc_junction(prev_move, self)`, which returns `(instant_corner_v / abs(diff_r)) ** 2` when `diff_r = move.axes_r[3] - prev_move.axes_r[3] ≠ 0` (extruder.py:328-332). Within the arc polyline, `interpolate_extruder` distributes E uniformly by arc-length, so every polyline segment shares the same `axes_r[3] = total_e / total_len`. **Internal polyline junctions have `diff_r = 0` and are unaffected.** Only the two boundary junctions see a jump:

- `trunc_prev → arc[0]`: `axes_r[3]` steps from `e_per_mm_prev` to `(tan(θ/2)/θ) · (e_per_mm_prev + e_per_mm_next)`.
- `arc[-1] → trunc_next_head`: symmetrically, to `e_per_mm_next`.

For equal-flow corners (`e_per_mm_prev ≈ e_per_mm_next`), the jump is `|e_per_mm − (2·tan(θ/2)/θ) · e_per_mm|`, which is ≈ 0 for small θ and rises to `0.27 · e_per_mm` at 90°. Combined with Klipper's default `instant_corner_v = 1 mm/s`, this yields a velocity cap of `(1/diff_r)²` that binds below `arc.v_cap` only on high-flow corners (`e_per_mm > 0.1`) with large deflections. Acceptable: correct extruder physics; the cap is genuinely the extruder-jerk budget.

For variable-flow corners (Arachne segment boundaries), the jump can be larger and the extruder cap tighter. The prepass does not merge across flow changes (gate (b)), so these corners reach the blender intact. The resulting cap is again correct: physical extruder response, not a model artifact.

### Auto-scaling chord tolerance

`CornerBlender.max_chord_err` defaults to `max(20e-3, 0.2 * self.tolerance)` — **floor** at 20 µm, rising to 20% of `corner_deviation` for loose-tolerance users. Rationale:

- Pure `0.2 * tolerance` with no floor collapses to 10 µm at the common `tolerance = 50 µm` case (no improvement over a fixed 10 µm default).
- Pure fixed 10 µm inflates trapq traffic at loose tolerance (200 µm corner_deviation should not demand 10 µm chord fidelity).
- `max(20 µm, 0.2·tolerance)` gives the common 50 µm user 20 µm chord (2× fewer polyline segments than pure 10 µm) while still scaling for users who pick large tolerances.

Concrete scaling at representative (R, tolerance) — all 90° corner segment counts:
- tol=50 µm → chord=20 µm. R=1 mm → ~4 segments. R=100 mm → ~40 segments.
- tol=200 µm → chord=40 µm. R=1 mm → ~3 segments. R=100 mm → ~28 segments.
- tol=20 µm (tight) → chord=20 µm (floor binds). R=1 mm → ~4 segments. Same as tol=50 µm — we don't go below 20 µm.

Override via `CornerBlender(toolhead, move_cls=Move, max_chord_err=…)` constructor kwarg for future tuning. Tests pass an explicit value to pin behavior.

### Micro-straight policy (option i) + instrumentation

The gap invariant (§"Gap invariant between adjacent blends") guarantees truncated pieces have `move_d ≥ 0.25 · min(L_shared, ...)`. Non-degenerate input cannot produce zero-length truncated straights; the instrumentation exists to catch drift from this assumption in production. Instead of a separate `micro_straight_count` counter, track:

- `self.polyline_moves_emitted` (int, cumulative): incremented per arc-polyline Move emitted. Total trapq pressure attributable to the blender.
- `self.blends_emitted` (int, cumulative): incremented once per blend (`_emit_arc` call). Ratio `polyline_moves_emitted / blends_emitted` gives mean polyline length per corner.

Both counters are read by `ToolHead.stats` and appended to the stats string as `blend_moves=<polyline_moves_emitted> blend_corners=<blends_emitted>`. Cheap integer increments on the fast path. Sub-spec #7 decides whether to keep, remove, or expose them as a Kalico status object.

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

- `klippy/blendmath.py:145`: multiply by `0.5`. Update comment at `:144` to explicitly state the half-segment rule.
- `klippy/blendmath.py:99-100` (the algorithm doc in module docstring / design section): re-state `R_mid = 0.5 · min(L_prev, L_next) · cos(θ/2)/sin(θ/2)` and `d_consumed ≤ 0.5 · min(L_prev, L_next)`.
- `test/test_blendmath.py`:
  - `test_blend_geometry_midpoint_cap_binds_on_short_segment` — expected `R` halves.
  - `test_blend_from_moves_shaper_bounds_binding` — the R_tol branch still binds, existing `R == 0.5` assertion stays green; update its inline comment only.
  - Property test `test_blend_geometry_property_random_corners` — invariant `d_consumed ≤ L_prev` still holds; tighten to `d_consumed ≤ 0.5 · min(L_prev, L_next) + eps`.
  - Any fixture that computes `expected_R_mid = L_short · cot(θ/2)` → multiply by 0.5.

This is the narrowest possible update. `BlendArc` struct, `blend_from_moves` API, `segment_arc`, and `interpolate_extruder` are unchanged. `d = R * tan(θ/2)` (blendmath.py:167) stays correct because it's derived from the already-capped `R`.

**Note for Zhao 2022 upgrade:** the proportional-share formulation from Zhao et al. (J. Manuf. Proc. 2022) is optimal on short-segmented paths — it shares the gap proportional to each corner's tolerance-unconstrained demand rather than blindly halving. Deferred to Stage 2 if/when measurements show the half-segment rule is conservatively capping throughput.

## Corner-deviation parameter

New config entry in the `[printer]` section: `corner_deviation` (placeholder name; sub-spec #7 may rename). Required. No default. Parsed with `config.getfloat("corner_deviation", above=0.0)` — omitting the `default` kwarg makes the option required; `configfile.py:97-106` raises `config.error("Option 'corner_deviation' in section 'printer' must be specified")` automatically. No explicit `raise` needed.

**Access path.** `ToolHead.corner_deviation` is the single source of truth. `CornerBlender` reads it lazily per `feed()` call via `self._toolhead.corner_deviation`. No mirroring onto the blender.

Rationale for "required, no default": this is a blend-arc fork. The parameter replaces `square_corner_velocity` semantically; users migrating a config must make an explicit choice. Picking a default now without measurement risks picking a bad one. Sub-spec #7 may introduce a measured-sensible default once Stage 1 validation has data; at that point the docs-pointing error message gets wrapped around the `getfloat` call.

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
8. **Kinematic / extruder `check_move` on emitted arc polyline.** `check_move` runs in `ToolHead.move` *before* `lookahead.add_move`, so Moves that the blender emits directly into the inner queue normally bypass it. The truncated pieces inherit their validity from the originals, which already passed. The arc polyline's per-mm E rate is `e_per_mm_prev + e_per_mm_next` (the *sum*), which can exceed extruder limits on high-flow corners. `_emit_arc` therefore runs an eager aggregate `kin.check_move` and (if extruding) `extruder.check_move` on one representative arc move per blend, analogous to `blendprepass.py:131-134`.

## Instrumentation

Counters live on `CornerBlender` (defined earlier in §"Micro-straight policy (option i) + instrumentation"):

- `polyline_moves_emitted` — trapq pressure attributable to the blender.
- `blends_emitted` — total corner blends. Mean polyline length per corner = `polyline_moves_emitted / blends_emitted`.

Both read by `ToolHead.stats` and appended to the stats string as `blend_moves=<polyline_moves_emitted> blend_corners=<blends_emitted>`. On Stage 1 validation prints, compare these against `print_stall` on tight-corner gcode. Rising `print_stall` correlated with rising polyline count signals step-rate pressure → upgrade to the absorb-micro-straights policy (option ii, deferred).

## Testing

`test/test_blendplanner.py`. Mirrors `test_blendprepass.py` — uses a `_FakeMove` faithful reimplementation (no `pyserial` dependency) and a `_FakeToolhead` exposing only what `Move.__init__` reads.

1. **Unit: single corner.** 90° corner between two 10 mm moves, corner_deviation = 50 µm. Verify emitted sequence on the SECOND feed is `[trunc_prev, arc[0], …, arc[N-1]]`, third feed (or flush) emits `[trunc_next_head]`. All polyline points lie on the arc within `max_chord_err`. `sum(arc_move.move_d) ≈ R · θ`. E conservation: prev.axes_d[3] + next.axes_d[3] = sum across all emitted Moves.
2. **Unit: 60° corner with asymmetric segment lengths (2 mm + 10 mm).** Half-segment rule caps consumption at 1 mm (= 0.5 · L_short). Verify `d_consumed == 1.0` within tolerance.
3. **Unit: U-turn (170° deflection).** Emitted sequence is `[prev]` with `prev.next_junction_v2 == 0`; next buffered unchanged; subsequent feed treats next as the new chain head.
4. **Unit: near-collinear (sub-1° deflection).** Emitted `[prev]`, next buffered (None return from `blend_from_moves`). Prepass-emit path separately covered by test #9.
5. **Unit: E-only move breaks chain.** Flushes buffered prev, emits E-only unchanged.
6. **Unit: flush drains buffered prev.** Feed two normal moves (first buffers, second emits split), call flush, verify `trunc_next_head` ripples out.
7. **Unit: reset drops state.** Same as #6 but call reset — buffered prev discarded, next feed starts fresh.
8. **Unit: `peek_buffered`.** Feed one move; `peek_buffered() == [move]`; state unchanged after peek (can continue feeding normally).
9. **Pipeline: prepass + blender composition.** Build 10 short collinear moves → 90° turn → 10 more collinear. Prepass merges both sides; blender blends the one resulting corner.
10. **Pipeline: `BlendPipelineLookAheadQueue` adapter passthroughs.** Two-filter stack, verify `get_last`, `queue`, `flush`, `reset`, `set_flush_time` each behave transparently. Specifically for `get_last`: returns blender's `_prev` without flushing it (new semantic vs. sub-spec #3's prepass adapter).
11. **Pipeline: `get_last` does not forfeit blend.** Feed move X (buffers in blender). Call `get_last()` → returns X. Attach a `timing_callback` and a `limit_next_junction_speed(100)` to the returned Move. Feed move Y that would normally blend with X. Verify: the emitted `trunc_prev` inherits the callback AND the next-junction cap (100); NO unblended X is in the queue.
12. **Regression: caller-state transfer on emit.** `get_last()`, mutate `_prev.timing_callbacks`, feed triggering blend, verify callbacks fire at `trunc_prev`'s end time (via `ToolHead._process_moves` callback dispatch).
13. **Regression: `SET_VELOCITY_LIMIT` mid-blend.** Feed prev, mutate `toolhead.max_accel` / `toolhead.junction_deviation`, feed next. Verify emitted Moves use prev's state (via `_copy_caller_state` pin), NOT the leaked new toolhead state.
14. **Regression: `extruder.check_move` on arc polyline.** Construct a high-extrusion pair where `e_per_mm_prev + e_per_mm_next` would saturate the extruder. Verify the aggregate check fires a `config.error` / `printer.command_error`.
15. **Regression: `smooth_delta_v2` pinned on arc.** Feed a blend; verify each arc polyline Move has `smooth_delta_v2 == delta_v2` (cruise-through-arc behavior, not look-ahead ramped).
16. **Regression: speed continuity.** Adjacent arc polyline segments share `max_cruise_v2` within 1 ppm.
17. **Regression: half-segment rule in `blendmath`.** Short-segment fixture where the midpoint cap was binding. Expected `R` drops by factor 2 vs. pre-revision.
18. **Property: random corners.** Seed-parameterized (`@pytest.mark.parametrize("seed", range(50))` as in `test_blendprepass.py`) random 3D corners with random segment lengths. Invariants: non-overlap of consumed segments, emitted polyline chord error within `max_chord_err`, E conservation within 1 ppm, gap invariant (≥ 0.25 · L_shared) on chains of 3+ corners.
19. **Drip-mode.** Single-move drip (matching `toolhead.py:696`): feed move, flush, verify the single move emits unchanged with no blend attempt.
20. **Integration smoke — DEFERRED to Stage 1 validation.** Real `ToolHead` instantiation requires ~100 LOC of Kalico test-harness setup (printer, reactor, MCUs, `chelper.get_ffi`, kinematics-specific rails). Out of scope for this sub-spec's test file; the behavior is covered end-to-end by Stage 1 validation on real hardware.

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
- Compare `print_stall`, MCU step-rate shutdowns, and `polyline_moves_emitted` / `blends_emitted` on stress prints (dense-corner cubes, spiral vases, text-on-surface) against a pre-blend baseline.
- If residuals exceed the shaper's error envelope → Stage 3 G² upgrade is pulled forward.
- If `print_stall` rises or MCU shutdowns occur → implement option (ii), the absorb-short-straights policy.

## Prior art

- **LinuxCNC `src/emc/tp/blendmath.c`** — half-segment rule at `:1031`, `blendCheckConsume()` short-segment absorption at `:1176-1199`. The single direct industrial reference.
- **Zhao et al., J. Manuf. Proc. 2022** — proportional-share overlap elimination, optimal on short-segmented paths. Deferred to Stage 2.
- **Bi et al. 2015 / 2019** — cubic Bézier (G²) with half-segment cap. Same overlap treatment. Relevant for Stage 3.
- **`klippy/blendprepass.py`** — internal precedent for the `feed / flush / reset` filter protocol and `_FakeMove` test harness pattern.

## Gap invariant between adjacent blends

With the half-segment rule AND this sub-spec's recursive truncation (each blend sees the already-head-truncated prev), the straight between two adjacent arcs on a shared segment of length `L_BC` is always ≥ `0.25 · L_BC`.

Proof. The first blend at corner B consumes `d_B = R_B · tan(θ_B/2) ≤ 0.5 · min(L_AB, L_BC)` from each adjacent segment (half-segment cap on the tolerance-or-midpoint-driven `R`). Therefore:

```
truncated_BC = L_BC − d_B ≥ 0.5 · L_BC          (d_B ≤ 0.5·L_BC)
```

The second blend at corner C uses `truncated_BC` as its `L_prev` input to `blend_from_moves`, so `d_C = R_C · tan(θ_C/2) ≤ 0.5 · truncated_BC`. The straight piece between the two emitted arc polylines on segment BC is:

```
gap = truncated_BC − d_C
    ≥ truncated_BC − 0.5 · truncated_BC
    = 0.5 · truncated_BC
    ≥ 0.5 · (0.5 · L_BC)
    = 0.25 · L_BC.                              ∎
```

Two edge cases confirm the bound:

- **Tolerance-binding `R`.** If `R_B = R_tol < R_mid`, then `d_B < 0.5 · min(L_AB, L_BC)` — strictly tighter than the cap. Proof chain survives.
- **U-turn at either corner.** `blend_geometry` returns `R = 0, d_consumed = 0`, and `_emit_arc` is never called (feed() step 5 short-circuits). The segment is untouched by the U-turn corner; full `L_BC` preserved. The invariant trivially holds.

No zero-length truncated straight unless `L_BC` itself was zero, which the prepass already filters via its `min_seg_len` gate.

## Implementation chunking hints

For the plan author. Two "must-land-together" groups that cannot be split across tasks without leaving the repo red:

**Pair A — half-segment rule + its test fixtures.** `blendmath.py:145` factor-of-0.5 change breaks `test_blend_geometry_midpoint_cap_binds_on_short_segment` and any property test asserting `d_consumed ≤ L_prev` (tightens to `≤ 0.5 · L_prev`). One atomic task.

**Pair B — `PrepassLookAheadQueue` rename + constructor signature change + `toolhead.py` call-site update + `blendprepass.py` tests.** Old signature: `PrepassLookAheadQueue(prepass, lookahead)`. New signature: `BlendPipelineLookAheadQueue(filters, lookahead)` with `filters` an ordered list. Both the class name and the constructor shape change; plus the `_chain` direct-read replaced by `peek_buffered()`. Plus `get_last` changes from "flush prepass first" to "peek without flushing." All sub-spec #3 tests referencing `PrepassLookAheadQueue` or `test_adapter_get_last_flushes_prepass_first` need updating in the same commit. One atomic task.

Beyond these, the remaining work decomposes cleanly: scaffolding, feed state-machine branches (one per `feed` step), `_emit_arc` body, pipeline-composition tests, instrumentation, `ToolHead` wiring, stats patch, property/randomized tests. Target 15–18 tasks total, matching sub-spec #3's density.

## Forward compatibility (sub-spec #5 prep)

This sub-spec adds NO new consumers of `junction_deviation` or `square_corner_velocity`. `Move.junction_deviation` is still snapshotted by `Move.__init__` into emitted Moves, and `_copy_caller_state` pins it to preserve mid-session consistency — both behaviors disappear cleanly when #5 removes the field. Grep check for #5's author: the strings `junction_deviation` and `square_corner_velocity` appear in this sub-spec only inside the `_copy_caller_state` helper and the transitional-period analysis; both are removed wholesale when the fields go away.

## Open questions

None remaining after the revision round.
