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
- User-tunable prepass tolerance. Tolerances are `CollinearCollapser` instance attributes (set in `__init__`) for v1; add a `danger_options` override later if empirical tuning proves necessary.

## Module layout

File: `klippy/blendprepass.py`. Pure Python, standard library only. Independently unit-testable without a running toolhead.

```python
# klippy/blendprepass.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

# Naive-CAM collinearity prepass. See docs/superpowers/specs/
# 2026-04-17-naive-cam-prepass-design.md for rationale.

class CollinearCollapser:
    def __init__(self, toolhead):
        self._toolhead = toolhead
        self._chain: list[Move] = []
        # Instance-level tunables, consistent with LookAheadQueue's init
        # style (toolhead.py:147-150). A future
        # danger_options.naive_cam_tolerance override becomes a one-line
        # read here; no class-attribute indirection needed.
        self.tolerance   = 25e-3   # perpendicular deviation cap, mm (25 µm)
        self.max_chain   = 100     # matches LinuxCNC
        self.epm_rel     = 1e-2    # E-per-XYZ-mm relative tolerance (1%)
        self.f_rel       = 1e-6    # cruise-velocity equality (1 ppm)
        self.min_seg_len = 1e-9    # XYZ-length floor, mm
        self.t_eps       = 1e-9    # float-noise eps for gate (d) projection

    def feed(self, move)  -> list[Move]: ...
    def flush(self)       -> list[Move]: ...
    def reset(self)       -> None:       ...   # discard buffered chain
```

`toolhead` is accepted so the collapser can construct replacement `Move` objects via the standard `Move(toolhead, start, end, speed)` constructor — it does not read mutable toolhead state beyond what `Move.__init__` reads.

## Algorithm

### State

`self._chain: list[Move]` — buffered moves that have passed the merge gate but not yet been emitted. `self._chain[0].start_pos` is the chain anchor.

### `feed(move)`

```
1. If move.move_d < self.min_seg_len:
       return [move]                                    # zero-length: pass through
2. If not move.is_kinematic_move:
       return self._flush_chain() + [move]              # E-only / special: breaks chain
3. If self._chain is empty:
       self._chain = [move]
       return []
4. If len(self._chain) >= self.max_chain:
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
    <= self.f_rel · max(candidate.max_cruise_v2, self._chain[0].max_cruise_v2)
```

**(b) Extrusion-ratio equality** — same E-per-XYZ-mm:

```
|candidate.axes_r[3] − self._chain[0].axes_r[3]|
    <= self.epm_rel · max(|candidate.axes_r[3]|, |self._chain[0].axes_r[3]|, 1e-9)
```

`axes_r[3]` is signed (negative on retract, positive on extrude). A retract-to-extrude reversal has `|diff| = |retract| + |extrude|`, which exceeds `epm_rel · max(...)` for any realistic flow → gate correctly rejects. Two consecutive same-sign moves of equal magnitude pass. The 1% relative tolerance is deliberate: it absorbs slicer float-quantization noise within one `ExtrusionPath` but splits on any intentional flow change (Arachne sub-paths differ by 2–40%).

**(c) Geometric collinearity** — every existing intermediate endpoint `P_k = self._chain[k].end_pos` (for `k < len(self._chain)`) lies within tolerance of the line from anchor `A = self._chain[0].start_pos` to the proposed new end `B = candidate.end_pos`. Using 3D cross product:

```
AB = B - A                                # new proposed chord
if |AB| < self.min_seg_len:          # candidate ends at anchor (U-turn closure)
    return False                           # reject; pathological
for each P_k:
    AP = P_k - A
    perp_dist = |AP × AB| / |AB|
    if perp_dist > self.tolerance:
        return False
```

**(d) Segment-projection bounds** — each intermediate endpoint must project onto the chord *interior*, not behind `A` or past `B`. This guards against U-turns where perpendicular distance is zero but the motion reverses along the chord.

```
for each P_k:
    AP = P_k - A
    t_k = (AP · AB) / (AB · AB)
    if not (-self.t_eps <= t_k <= 1.0 + self.t_eps):
        return False
```

The projection check also enforces monotonic progress along the chord, which is what "collinear" physically means. `t_eps = 1e-9` guards against subtractive-cancellation noise on legitimate chord-end extensions (e.g. a candidate endpoint numerically equal to the previous constituent's endpoint). Without the eps, a valid `t_k = 1 + 1·ulp` would falsely reject.

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

```python
def _build_merged_move(self, chain):
    start_pos = chain[0].start_pos
    end_pos   = chain[-1].end_pos
    cruise_v  = math.sqrt(chain[0].max_cruise_v2)
    merged    = Move(self._toolhead, start_pos, end_pos, cruise_v)

    # Pin fields that Move.__init__ would otherwise re-snapshot from a
    # possibly-drifted toolhead state. SET_VELOCITY_LIMIT, M204, or
    # SET_PRESSURE_ADVANCE could have fired between chain[0]'s
    # construction and this call; we want the chain's head values.
    merged.max_cruise_v2     = chain[0].max_cruise_v2
    merged.junction_deviation = chain[0].junction_deviation

    # Preserve the narrowest accel observed across the chain — a
    # constituent kinematic check_move may have applied limit_speed
    # (Z-ratio, slow-XY zones) which must propagate through the merge.
    merged.limit_speed(cruise_v, min(m.accel for m in chain))

    # Preserve the chain tail's next-junction cap; limit_next_junction_speed
    # attaches to the caller-visible last move, which is chain[-1] because
    # the adapter's get_last() flushes before exposing it (see Integration).
    merged.next_junction_v2 = chain[-1].next_junction_v2

    # Concatenate timing_callbacks across the chain. Under flush-on-get_last
    # these should only ever be on chain[-1]; the list comprehension is
    # defense in depth against future callers that bypass the adapter.
    merged.timing_callbacks = [
        cb for m in chain for cb in m.timing_callbacks
    ]

    # Re-run the kinematics and extruder check_move on the merged result.
    # Per-constituent checks already passed, and axes_r is preserved to
    # within TOLERANCE/min_seg_len (~25% in the worst case of 0.1 mm
    # segments), so limits should continue to pass — but aggregate checks
    # like max_extrude_only_distance would be bypassed otherwise. Safety
    # net, not expected to fire in normal operation.
    if merged.is_kinematic_move:
        self._toolhead.kin.check_move(merged)
    if merged.axes_d[3]:
        self._toolhead.extruder.check_move(merged)
    return merged
```

`Move.__init__` recomputes `axes_d`, `axes_r`, `move_d`, `min_move_t`, `delta_v2`, `smooth_delta_v2`, `max_start_v2 = 0.0`. Fields explicitly pinned above override post-init snapshots: `max_cruise_v2`, `junction_deviation`, `accel` (via `limit_speed`), `next_junction_v2`, `timing_callbacks`. `is_kinematic_move` is True because gate step 2 flushed any non-kinematic input before chain building.

**Material conservation:** `sum(m.axes_d[i] for m in chain) == merged.axes_d[i]` for i ∈ {0,1,2,3} within 1–2 ulp of coordinate magnitude (~2e-13 mm on mm-scale coords), by telescoping through `chain[k].start_pos == chain[k-1].end_pos`.

### Exception safety

`_flush_chain` wraps the build in `try/finally` that unconditionally clears `self._chain` before returning or propagating:

```python
def _flush_chain(self):
    try:
        if len(self._chain) == 1:
            result = self._chain
        else:
            result = [self._build_merged_move(self._chain)]
    except Exception:
        logging.warning(
            "blendprepass: chain cleared after build error (len=%d)",
            len(self._chain))
        raise
    finally:
        self._chain = []
    return result
```

An exception during `Move.__init__` or `check_move` clears the chain and propagates the error; subsequent `feed()` calls start with a clean buffer. The `logging.warning` makes field diagnosis possible without interfering with normal operation.

## Integration with `ToolHead`

### Wrapper adapter

To avoid scattering "drain prepass before each lookahead.flush()" ceremony across every call site, introduce a thin adapter class in `klippy/blendprepass.py`. The adapter must mirror **every** `LookAheadQueue` surface accessed from `toolhead.py` and external callers — verified by grepping `klippy/` for `lookahead.` and `.lookahead.`:

| `LookAheadQueue` surface | Consumed at | Adapter behavior |
|---|---|---|
| `add_move(move)` | `toolhead.py:624` | Feed prepass; push emitted moves to inner queue |
| `flush(lazy=...)` | `toolhead.py:498, 514, 681, 698` | Drain prepass into inner queue, then inner flush |
| `reset()` | `toolhead.py:700, 749` | Discard prepass chain, then inner reset |
| `set_flush_time(t)` | `toolhead.py:150, 268, 445` | Pass through |
| `get_last()` | `toolhead.py:611, 770`, `extras/trad_rack.py:1489` | **Flush prepass first**, then inner get_last (see below) |
| `queue` (attribute) | `toolhead.py:725` | `@property` — return inner queue concatenated with buffered chain so truthy/length checks see buffered moves as "pending" |

```python
class PrepassLookAheadQueue:
    """Wraps a LookAheadQueue; drains a CollinearCollapser transparently.

    Mirrors the full LookAheadQueue surface used elsewhere in Kalico so
    ToolHead and extras don't need to know the prepass exists beyond the
    add_move entry point.
    """
    def __init__(self, prepass, lookahead):
        self._prepass = prepass
        self._lookahead = lookahead

    def add_move(self, move):
        for m in self._prepass.feed(move):
            self._lookahead.add_move(m)

    def flush(self, lazy=False):
        for m in self._prepass.flush():
            self._lookahead.add_move(m)
        self._lookahead.flush(lazy=lazy)

    def reset(self):
        self._prepass.reset()
        self._lookahead.reset()

    def set_flush_time(self, flush_time):
        self._lookahead.set_flush_time(flush_time)

    def get_last(self):
        # Callers of get_last attach lookahead-critical state to the
        # returned Move: register_lookahead_callback (toolhead.py:770)
        # attaches timing_callbacks, limit_next_junction_speed
        # (toolhead.py:611) attaches a speed cap. If we returned an
        # unflushed chain tail, that state would land on a Move that
        # _build_merged_move would then replace. Flushing first ensures
        # callers see the actual queued Move.
        for m in self._prepass.flush():
            self._lookahead.add_move(m)
        return self._lookahead.get_last()

    @property
    def queue(self):
        # check_busy and similar callers test emptiness/length; buffered
        # chain counts as pending. Return a concatenation so list indexing
        # remains consistent with arrival order.
        if self._prepass._chain:
            return list(self._prepass._chain) + list(self._lookahead.queue)
        return self._lookahead.queue
```

**Why flush-on-`get_last`:** `register_lookahead_callback` is invoked occasionally (per M106, per heater set, per SET_PIN — not per-move), and `limit_next_junction_speed` is rarer still (resonance_tester, tool-change macros). Flushing incomplete chains at these boundaries costs little; returning a stale tail would corrupt timing-callback semantics and silently drop junction caps. Belt-and-suspenders, `_build_merged_move` also preserves `timing_callbacks` and `next_junction_v2` from `chain[-1]` in case a future caller bypasses the adapter.

### Changes to `ToolHead`

Only the constructor changes; every other `self.lookahead.*` call site in `toolhead.py` continues to work unmodified:

```python
from . import blendprepass
...
inner_queue    = LookAheadQueue(self)
self.prepass   = blendprepass.CollinearCollapser(self)
self.lookahead = blendprepass.PrepassLookAheadQueue(self.prepass, inner_queue)
self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
```

The four concrete `lookahead.flush()` / `lookahead.reset()` sites in the current `toolhead.py` (`_flush_lookahead` at line 498, `lookahead.flush()` at line 514 inside `get_last_move_time`, `lookahead.flush()` at lines 681 and 698 in `drip_move`, and `lookahead.reset()` at lines 700 and 749 in `drip_move` error path and `_handle_shutdown`) all go through the adapter — no per-site change needed.

### `check_move` runs both pre- and post-merge

**Pre-merge (unchanged):** `ToolHead.move` continues to call `kin.check_move(move)` and `extruder.check_move(move)` on each raw constituent before feeding it to the prepass. This enforces the user's gcode envelope and catches any per-segment violation at its real line number.

**Post-merge (added):** `_build_merged_move` re-runs `kin.check_move(merged)` and `extruder.check_move(merged)` on the merged result before returning it. Per-constituent checks already guarantee the per-segment limits; the post-merge re-run catches **aggregate** limits like `max_extrude_only_distance` that the strict gate's per-move checks cannot see (even though gate step 2 flushes on non-kinematic moves and should prevent pure-E merges reaching this limit, re-running is a cheap safety net against future gate changes).

**`axes_r` drift under merging:** within the prepass tolerance, each constituent's `axes_r` can differ from the merged `axes_r` (which equals the true chord direction by telescoping) by up to `tolerance / min_constituent_d` — e.g. 25 µm / 0.1 mm = 25% in the worst case of 100 µm slicer chords. Kinematic limits keyed to `axes_r` (Z-ratio caps, CoreXY belt limits, deltesian slow-X zones) were applied to the constituents' noisier `axes_r`; `_build_merged_move` propagates the strictest via `merged.limit_speed(cruise_v, min(m.accel for m in chain))`. The post-merge `check_move` re-evaluates against the canonical merged direction, which may tighten or loosen limits slightly. This is acceptable because (a) merged `axes_r` is the "intended" direction the slicer meant to express, and (b) the constituent-derived `accel` floor is already a conservative bound. No case where the merged move would pass post-merge `check_move` but violate a physical limit.

### Pipeline invariants (downstream contract)

Documented because downstream code relies on them:

- **Input shaper** (`kin_shaper.c`) is stateless across moves; a merged constant-velocity chain produces position output identical to the unmerged chain. Verified.
- **Trapq** (`trapq.c`) is an append-only linked list with no per-move state beyond print_time bookkeeping. Verified.
- **Pressure advance** (`kin_extruder.c:86-112`, `pa_range_integrate`): convolves over trapq moves; for a constant-velocity constant-flow chain the integrand is identical merged vs unmerged. Verified.
- **`per_move_pressure_advance`** (`extruder.py:26-28, 342-346`): stores PA in each trapq move's `axes_r.y` slot. `SET_PRESSURE_ADVANCE` routes through `flush_step_generation` → `_flush_lookahead` → adapter `flush()` → prepass drain, so PA changes always break chains. Verified structurally; no test required.
- **Extruder `calc_junction`** (`extruder.py:328-332`): uses `axes_r[3]` diff against previous move; gate (b) guarantees merged `axes_r[3]` matches constituents within 1%, so junction behavior between merged and surrounding moves is identical to unmerged behavior. The 1% interior flow drift is absorbed into a single averaged extrusion ratio — intentional (Rationale 1).

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
12. **Shutdown / emergency stop**: `ToolHead._handle_shutdown` currently calls `self.lookahead.reset()`, which under the wrapper adapter also calls `self._prepass.reset()` — buffered chain is discarded, not flushed. Correct behavior: an aborted print should not re-emit stale moves into a halted lookahead.
13. **Drip-move reset path**: `drip_move` at line 700 of `toolhead.py` calls `lookahead.reset()` on its error branch; same discard semantics apply via the adapter.
14. **Retract-wipe-retract patterns**: two consecutive E-only moves (retract + wipe-retract) each hit gate step 2 (non-kinematic), flushing any chain and passing through. No special handling needed.
15. **Exception during Move construction**: if a kinematics error fires inside the merged `Move.__init__`, the try/finally wrapper in `_flush_chain` clears `self._chain` before propagating the exception — subsequent feeds start with a clean buffer.

## Testing (`test/test_blendprepass.py`)

All tests use pytest, follow the existing `test_blendmath.py` / `test_blendshaper.py` conventions, and avoid loading the toolhead C module. **No `hypothesis` dependency** — Kalico does not ship it (`scripts/requirements_dev.txt` only lists `pytest` + `pytest-xdist`). Randomized tests use the repo's established pattern: `@pytest.mark.parametrize("seed", range(N))` plus `random.Random(seed)` (seen at `test_blendmath.py:319, 442`).

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
15. **Fresh merged-Move invariants** — merged Move's `next_junction_v2 == 999999999.9`, `max_start_v2 == 0.0`, `max_smoothed_v2 == 0.0` (constructor defaults), confirming no lookahead state leaks from constituents through the merge.
16. **junction_deviation pinned to chain[0]** — if `toolhead.junction_deviation` is mutated between a chain's first and last move, the merged Move still carries `chain[0].junction_deviation`, not the current toolhead value.
17. **Adapter transparency** — `PrepassLookAheadQueue.flush()` drains the collapser, then calls the inner `lookahead.flush(lazy=...)`; `reset()` discards the chain without emitting. Test via a mock inner queue.
18. **Exception safety** — injecting a `Move.__init__` failure during `_build_merged_move` leaves `self._chain` empty on the next `feed()` call; a `logging.warning` is emitted before propagation.
19. **Timing callbacks preserved** — attach a callback to `chain[-1]` via `limit_next_junction_speed` surrogate; after merge, callback is on the merged Move's `timing_callbacks` list.
20. **`next_junction_v2` preserved from chain[-1]** — set `chain[-1].next_junction_v2 = 12345.0` pre-flush; merged Move carries `12345.0`, not the `999999999.9` sentinel.
21. **Adapter `get_last` flushes** — when the collapser has a buffered chain, `PrepassLookAheadQueue.get_last()` first drains the prepass; subsequent inner-queue tail is the merged Move, not the pre-chain Move.
22. **Adapter `queue` property reports buffered state** — `len(adapter.queue) == len(prepass._chain) + len(inner.queue)`; truthy when chain is buffered even if inner queue is empty.
23. **Post-merge `check_move` re-runs** — mock kinematics that raises on moves over `max_extrude_only_distance`; a chain that individually passes but aggregates over the limit raises on merge (verifies aggregate-limit safety net).
24. **`t_k` eps tolerance** — constituent endpoint numerically at `1 + 1·ulp` along chord (via subtractive cancellation) does NOT reject; one at `1 + 100·ulp` does.
25. **Chain-cap flush produces collinear junction** — two successive 100-chain flushes should produce a `junction_cos_theta ≈ -1` junction (collinear passthrough), not a cornering penalty. Test by checking `calc_junction` output between the two merged moves.

### Randomized tests (seed-parameterized)

```python
@pytest.mark.parametrize("seed", range(50))
def test_random_collinear_chain_merges(seed):
    rng = random.Random(seed)
    ...
```

26. **Random collinear chains with per-step offset < tolerance always merge.** Generator: anchor, direction unit vector, N ∈ [2, 100] segments with lengths ∈ [0.01, 10] mm and perpendicular noise ∈ [-20 µm, 20 µm]. Assert: chain of N returns 1 merged move.
27. **Random chains with one offset-violating step always split at the violation.** Assert: exactly two output moves (the chain up to violation, then the rest starting at the offending move).
28. **Total displacement preserved** across random arbitrary valid chains: `sum(merged.axes_d[i]) == sum(all original axes_d[i])` within float precision.

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

- **LinuxCNC `emccanon.cc::linkable()`** — direct algorithmic reference. LinuxCNC computes a clamped projection `t ∈ [0, 1]` and measures distance to the *nearest point on the segment* (perpendicular when the foot falls inside the chord, endpoint distance otherwise). Our gate (d) is deliberately **stricter**: it rejects the merge whenever the projection falls outside `[0, 1]`, which catches U-turns and "overshoot-then-retrace" patterns that LinuxCNC would quietly accept if the endpoint distance stayed under tolerance. On FDM this stricter behavior is appropriate — an Arachne wall-end overshoot is a real motion event that must not collapse into the preceding chain. We accept slightly fewer merges in exchange for unambiguous chain semantics. Additionally, we add the strict gate on F and E-per-mm that CNC doesn't have.
- **Siemens COMPCAD / COMPCURV** — NC-block compressor; higher-order (polynomial spline fit) variant of the same concept.
- **Prunt** — runs a dedicated preprocessing stage before its Bézier corner blender. Same architectural placement.
- **Bleeding-edge-v2 PA sync** — future compatibility is by construction: the prepass emits fewer junctions and shorter post-shaper transients, strictly helping the junction-spike behavior (`a_E ≈ PA · ΔV / T_shaper²`). No coupling in the other direction.

## Validation

Hardware A/B benchmarking (merge-rate telemetry, corner-velocity improvement, Arachne/Scarf visual regression) requires the prepass to be wired to blend-arc geometry end-to-end, which is sub-spec #4's scope. This spec delivers the module and its unit tests; integrated validation belongs to the planner-integration sub-spec.

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
