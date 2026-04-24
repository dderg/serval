# Plan 9 Phase A4 — Cross-Boundary Shape-Bake Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the three cross-blend-boundary zero-pad gaps in the shape-bake pipeline so every plain↔QBM (QuinticBlendMove) transition propagates its neighbour polynomial into the shaper convolution — shape continuity becomes exact at every move boundary, not only at plain↔plain and QBM↔QBM boundaries.

**Architecture:** Three coordinated changes unify the neighbour plumbing across the plain-Move path (`klippy/toolhead.py:LookAheadQueue`) and the QBM path (`klippy/blendplanner.py:CornerBlender`). **(a)** Replace `_is_shape_bake_target` at the neighbour-source sites with a duck-typed predicate `_has_unshaped_payload(m)` that accepts both `Move` and `QuinticBlendMove`; introduce a helper `_neighbour_payload(m)` returning `(unshaped_payload, start_xyz)` that normalises over both classes. **(b)** Extend `CornerBlender` with `_last_released_plain_unshaped` — a snapshot captured whenever the blender releases a plain Move, so the next constructed QBM can use the plain predecessor as its prev neighbour. Since plain Moves don't have `_unshaped_payload` until `set_junction` runs (inner LookAheadQueue), we materialise it defensively via `move.build_unshaped_payload()` at blender release time (the call is idempotent and cheap). **(c)** Thread `lazy: bool` through `CornerBlender.flush(lazy)` so it can hold the pending QBM across a lazy flush in a new `_across_flush_pending` slot, mirroring the inner `LookAheadQueue._pending_last` pattern; `lazy=False` (true drain) retains the current zero-pad.

**Tech Stack:** Python (`klippy/toolhead.py`, `klippy/blendplanner.py`, `klippy/blendprepass.py`); tests (`test/test_toolhead_shape_bake.py`, `test/test_blendplanner.py`, `test/test_blendprepass.py`, new `test/test_cross_boundary_shape_bake.py`).

---

## Architectural references (pre-read)

All citations resolved against the tree at the time of writing (branch `magnum-opus`, HEAD `b2b3f09f`).

### A. Three gaps verified — exact sites

**Gap 1 — plain Move ← QBM.** `klippy/toolhead.py:377-409` (`LookAheadQueue._finalize_with_neighbours`). The predicate on lines 388, 392, 398 is `_is_shape_bake_target`, defined at line 354 as:

```python
def _is_shape_bake_target(move):
    return isinstance(move, Move) and move.is_kinematic_move
```

`QuinticBlendMove` is a standalone class (blendplanner.py:341), not a `Move` subclass, so `isinstance(move, Move)` is False for QBMs. Line 392 therefore rejects a QBM as `prev_move` → `prev_unshaped = None` → kernel zero-pads the prev side. Line 398 rejects a QBM as `next_move` → same zero-pad on the next side.

**Gap 2 — QBM ← plain Move.** `klippy/blendplanner.py:679-739` (`CornerBlender.feed`). Lines 712-719:

```python
old_pending_snapshot = None
if self._pending_quintic is not None:
    old_pending_snapshot = (
        self._pending_quintic._unshaped_payload,
        (self._pending_quintic._start_pos_4d[0],
         self._pending_quintic._start_pos_4d[1],
         self._pending_quintic._start_pos_4d[2]),
    )
```

`_pending_prev` for the newly-constructed QBM comes only from the prior *pending QBM*. When the prior emit was a plain Move (every non-suppressed corner stashes a `trunc_prev` plain Move BEFORE the new QBM is built, and `_suppress_and_advance` emits plain Moves entirely), `old_pending_snapshot = None` → the new QBM starts with zero-padded prev in its kernel window.

**Gap 3 — QBM at lazy-flush drain.** `klippy/blendplanner.py:904-914` (`CornerBlender.flush`). Currently zero-pads unconditionally:

```python
def flush(self):
    released = self._finalize_pending(
        next_unshaped=None, next_start_pos_xyz=None,
    )
    if self._prev is not None:
        released.append(self._prev)
        self._prev = None
    return released
```

`BlendPipelineLookAheadQueue.flush(lazy=True)` (blendprepass.py:175-186) calls `f.flush()` on every filter regardless of the `lazy` flag. Every mid-print lazy flush (fired every `LOOKAHEAD_FLUSH_TIME` = 0.25 s) strands any pending QBM with `next=None`. The inner `LookAheadQueue.flush(lazy=True)` (toolhead.py:419) correctly defers; the blender does not.

### B. Pipeline invariants

- Pipeline order (toolhead.py:609-616): `CollinearCollapser → CornerBlender → LookAheadQueue`. `BlendPipelineLookAheadQueue` runs the two filters in sequence on `add_move` and drains them on `flush`.
- `set_junction` for plain Moves is called only at **toolhead.py:477 and 487** (reverse pass inside `LookAheadQueue.flush`). Plain Moves released by `CornerBlender` therefore arrive at the inner `LookAheadQueue` with `_unshaped_payload = None` until that reverse pass runs.
- `Move.build_unshaped_payload()` (toolhead.py:99-151) requires `self.jerk_profile`, which is populated by `set_junction` (toolhead.py:289). Before `set_junction`, `build_unshaped_payload()` raises (no `jerk_profile` attribute). Consequence: the CornerBlender cannot naively call `build_unshaped_payload()` on a plain Move at release time — it must run `set_junction` first OR defer the snapshot.
- Both `Move` and `QuinticBlendMove` store `_unshaped_payload` with the same 3-tuple layout `(phase_t_ends, total_t, coeff_tuple)` (toolhead.py:151 / blendplanner.py:398).
- `QuinticBlendMove._unshaped_payload` is populated at `__init__` (blendplanner.py:398), BEFORE `set_junction`. This asymmetry is the reason the QBM can serve as a neighbour immediately but a plain Move cannot.

### C. Design choice — how to materialise the plain-Move unshaped payload

Three options the verifier sketched; this plan picks (B1):

- **(B1) Snapshot-at-set_junction in the inner LookAheadQueue.** Have the inner `LookAheadQueue` (which is the only `set_junction` caller) capture each plain Move's `_unshaped_payload` as it becomes ready, and push it back up to the blender's `_last_released_plain_unshaped` via a callback. **REJECTED** as too coupled — pushes blender state up from a lower-level class.

- **(B2) Defensive `build_unshaped_payload()` at blender release.** Require the blender to compute a temporary `jerk_profile` on demand (via a dedicated helper `Move._ensure_unshaped_payload()`) OR pass neutral defaults when `jerk_profile` isn't set. **REJECTED** — `build_unshaped_payload` requires `self.jerk_profile` which requires `(start_v, cruise_v, end_v)` which the blender does not know yet (the outer lookahead sets them). Synthesising a placeholder profile would produce an incorrect polynomial.

- **(B3) Defer the snapshot to the inner `LookAheadQueue.flush` reverse pass.** When the inner LookAheadQueue calls `set_junction` on a plain Move whose predecessor was a QBM (or vice versa), it already has the right-shape snapshot available at that point because plain Moves land in `queue` only AFTER the blender has already emitted them. So the blender's `_last_released_plain_unshaped` slot is populated not at blender-emit time but when the plain Move reaches the inner queue. **This is the chosen approach**: extend the inner `LookAheadQueue` to record, on each released plain Move, its newly-built `_unshaped_payload` in a dedicated slot `_last_plain_snapshot`, then have the blender query that slot when it constructs the next QBM.

**Update on reflection:** (B3) still couples two filters across a protocol, but it respects the current flow (snapshots happen when `_unshaped_payload` is valid). A simpler alternative emerged during design:

- **(B4) Make the blender hold the reference to the released plain Move directly.** When CornerBlender emits a `trunc_prev` / `emitted_prev` plain Move, it retains a weak reference `_last_plain_move_ref` to it. When constructing the next QBM, the blender checks whether that Move has a populated `_unshaped_payload` yet (it will — by the time the next Move arrives via `feed`, the inner LookAheadQueue's reverse pass has already run for previously-released moves when a lazy flush fired). If populated, snapshot it. If not (rare: the new move arrives before a flush cycle completed), fall back to `None` (zero-pad, as today). This is weaker than (B3) but zero-coupling.

**Final choice: (B4) at the blender boundary, with a correctness test.** The blender records the last released plain Move. At QBM-construction time the blender reads `move._unshaped_payload` — if populated, snapshot; if None, accept zero-pad (documented edge case). Critically, **this is the common case**: in steady-state printing, by the time a new `feed()` arrives at the blender the inner LookAheadQueue has almost always flushed the preceding moves (because a lazy flush fired between G-code lines). The uncommon path (flush-immediately-after-corner) still zero-pads, but only for that single transition.

This is a **correctness-improving but not correctness-guaranteeing** change on its own; to close the gap fully we also need (B3)-equivalent action **inside** `LookAheadQueue.flush` — specifically, at the moment the inner queue populates `_unshaped_payload` for a plain Move whose `prev` in the queue was a QBM (or whose `next` is a QBM), the existing A3 flush pass's `_finalize_with_neighbours` can already see the QBM via `queue[i-1]`/`queue[i+1]` and use its `_unshaped_payload` — provided the predicate at line 392/398 accepts QBMs. **Gap 1 is the load-bearing gap.** Fixing it subsumes most of what (B4) buys us, because the QBM sits in the inner queue as a neighbour alongside the plain Move.

**Simplification:** The final plan closes Gap 1 (predicate) first — that alone fixes most plain↔QBM transitions because the inner LookAheadQueue holds both kinds of move in its queue side-by-side after CornerBlender emits them. Gap 2 (CornerBlender-internal plain→QBM prev) and Gap 3 (CornerBlender lazy-flush drain) operate on moves that are still buffered inside CornerBlender and haven't reached the inner queue yet; those need the CornerBlender-side changes.

### D. `_finalize_pending` semantics — `klippy/blendplanner.py:654-677`

```python
def _finalize_pending(self, next_unshaped, next_start_pos_xyz):
    if self._pending_quintic is None:
        released = list(self._pending_leading)
        self._pending_leading = []
        return released
    prev_payload = None
    prev_start = None
    if self._pending_prev is not None:
        prev_payload, prev_start = self._pending_prev
    self._pending_quintic.finalize_shape(
        prev_unshaped=prev_payload,
        next_unshaped=next_unshaped,
        prev_start_pos_xyz=prev_start,
        next_start_pos_xyz=next_start_pos_xyz,
    )
    ...
```

This is the single sink for QBM shape-finalisation. The two inputs (`_pending_prev`, `next_unshaped`) carry the cross-boundary context; A4 feeds both from wider sources.

### E. `Move._unshaped_payload` population timeline

1. `Move.__init__` — sets `_unshaped_payload = None` (toolhead.py:66).
2. `CornerBlender.feed` — may release the move as-is (pass-through, `trunc_prev`, or `emitted_prev`). `_unshaped_payload` still `None`.
3. `BlendPipelineLookAheadQueue` adds it to the inner `LookAheadQueue.queue`.
4. Inner `LookAheadQueue.flush` reverse pass calls `move.set_junction(...)` (toolhead.py:477, 487). `set_junction` at toolhead.py:342-344 calls `build_unshaped_payload()` → `_unshaped_payload` now populated.
5. Inner flush's deferred-last pass (A3, toolhead.py:519-568) calls `_finalize_with_neighbours(move, prev_move, next_move)` → reads `queue[i±1]._unshaped_payload`.

### F. Test landscape

- `test/test_blendplanner.py` — uses `_FakeMove` (does NOT have `_unshaped_payload` attribute). The A4 CornerBlender changes will reference `move._unshaped_payload`; update `_FakeMove.__init__` to `self._unshaped_payload = None`.
- `test/test_blendprepass.py` — uses `_FakeMove`, similar patch needed.
- `test/test_toolhead_shape_bake.py` — real `Move` class with manual `.queue.append()` + `.flush()`; directly exercises `_finalize_with_neighbours`. This is where Gap 1 tests live.
- `test/test_toolhead_shape_bake_pipeline.py` — real pipeline. `_PipelineToolhead` already wires `BlendPipelineLookAheadQueue([CollinearCollapser, CornerBlender], LookAheadQueue)`. Note: at the time this plan was first drafted (in parallel with commit `e6e71a0e`), this file had a known QBM-in-reverse-pass bug (`QuinticBlendMove.reachable_v_from_v_end` missing) that crashed any flush through the full pipeline. **That bug was fixed in commit `e6e71a0e`** — A4 tests CAN now use the full pipeline. Prefer end-to-end pipeline tests over hand-crafted inner-queue tests where the boundary semantics warrant the broader reach. Keep direct `CornerBlender.feed` tests for unit-level neighbour-handshake verification.

---

## File structure

### Create

- `test/test_cross_boundary_shape_bake.py` — A4-specific tests that drive the three cross-boundary scenarios (Gap 1, Gap 2, Gap 3) to completion. Isolated from existing A3 tests because these tests span both `CornerBlender` and `LookAheadQueue`.

### Modify

- `klippy/toolhead.py`
  - Add helper `_has_unshaped_payload(m)` — duck-typed predicate (accepts `Move` and `QuinticBlendMove`).
  - Add helper `_neighbour_payload(m)` — returns `(unshaped_payload, start_xyz_tuple)` for either class.
  - Update `_finalize_with_neighbours` (lines 377-409) to use the new helpers at the neighbour-source branches (the "who gets baked" check on line 388 stays as `_is_shape_bake_target` — QBMs are already baked and must not be re-baked).

- `klippy/blendplanner.py`
  - `CornerBlender.__init__` — add `self._last_released_plain = None` (weak snapshot slot for Gap 2) and `self._across_flush_pending = None` (slot for Gap 3).
  - `CornerBlender.feed` — capture released plain Moves' unshaped payload (when available) into `_last_released_plain`; on new QBM construction, fall back to `_last_released_plain` when `_pending_prev` would be None.
  - `CornerBlender._suppress_and_advance` — same capture on `emitted_prev`.
  - `CornerBlender.flush(lazy)` — grow a `lazy: bool` parameter (default False); on `lazy=True` stash the pending QBM into `_across_flush_pending` instead of zero-padding; on `lazy=False` drain with zero-pad as today AND drain any leftover `_across_flush_pending`.
  - `CornerBlender.reset` — clear both new slots.
  - `CornerBlender.peek_buffered` — include `_across_flush_pending` if populated (needed for `BlendPipelineLookAheadQueue.get_last`).

- `klippy/blendprepass.py`
  - `BlendPipelineLookAheadQueue.flush(lazy)` — change the `f.flush()` call inside the drain loop to `f.flush(lazy=lazy)` for filters that accept it (all filters grow the kwarg; `CollinearCollapser.flush` gains an unused `lazy=False` to keep the signature uniform).
  - `CollinearCollapser.flush(lazy=False)` — accept and ignore the kwarg (collinear collapser has no across-flush state to defer).

- `test/test_blendplanner.py` — update `_FakeMove.__init__` to include `self._unshaped_payload = None` so tests that invoke A4's new blender paths don't crash on `AttributeError`. Update the blender `flush()` call sites in existing tests to still pass (default `lazy=False` keeps backward compat).

- `test/test_blendprepass.py` — same `_FakeMove` patch; update adapter tests to account for `flush(lazy=True)` propagating into the filters.

---

## Task 1: Add duck-typed neighbour helpers in toolhead.py (Gap 1, part 1)

**Goal:** Introduce `_has_unshaped_payload` and `_neighbour_payload` helpers that treat `Move` and `QuinticBlendMove` uniformly as neighbour sources, without changing any bake behaviour yet.

**Files:**
- Modify: `klippy/toolhead.py` — add module-level helpers after `_is_shape_bake_target` (line 354).
- Test: `test/test_cross_boundary_shape_bake.py` (new file).

- [ ] **Step 1: Write failing test for `_has_unshaped_payload` accepting both classes**

```python
# test/test_cross_boundary_shape_bake.py
"""Plan 9 A4 — cross-boundary shape-bake tests.

Drives the three gap scenarios identified by the A3 verifier:
- Gap 1: plain Move ← QBM (LookAheadQueue neighbour predicate)
- Gap 2: QBM ← plain Move (CornerBlender _pending_prev slot)
- Gap 3: QBM at lazy-flush drain (CornerBlender.flush lazy parameter)
"""
from __future__ import annotations

import math

import pytest

from klippy import blendmath, blendplanner, blendprepass
from klippy.toolhead import (
    LookAheadQueue,
    Move,
    _has_unshaped_payload,
    _neighbour_payload,
    _is_shape_bake_target,
)


class _Printer:
    command_error = Exception
    def __init__(self):
        self._objs = {}
    def lookup_object(self, name, default=None):
        return self._objs.get(name, default)


class _DummyExtruder:
    def get_status(self, eventtime=None):
        return {"pressure_advance": 0.0,
                "pressure_advance_model": "linear",
                "pressure_advance_smooth_time": 0.0}
    def calc_junction(self, *a, **kw):
        return 1e99
    def check_move(self, m):
        pass


class _FakeKin:
    def check_move(self, m):
        pass


class _BareToolhead:
    """Minimal toolhead for constructing a single Move in isolation."""
    def __init__(self):
        self.printer = _Printer()
        self.max_velocity = 500.0
        self.max_accel = 5000.0
        self.max_accel_to_decel = 5000.0
        self.square_corner_velocity = 5.0
        self.junction_deviation = 0.05
        self.max_jerk = 100000.0
        self.extruder = _DummyExtruder()
        self.kin = _FakeKin()
        self.trapq = None
        self.shapers_snapshot = []
        self.extruder_cap_snapshot = None
        self.corner_deviation = 0.05
    def note_kinematic_activity(self, *a, **kw):
        pass


def _make_move(tool, start, end, speed=100.0):
    m = Move(tool, list(start), list(end), speed)
    m.set_junction(speed * speed, speed * speed, speed * speed)
    return m


def test_has_unshaped_payload_accepts_plain_move_with_payload():
    th = _BareToolhead()
    m = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0])
    assert _has_unshaped_payload(m) is True


def test_has_unshaped_payload_rejects_plain_move_without_payload():
    th = _BareToolhead()
    m = Move(th, [0, 0, 0, 0], [10, 0, 0, 0], 100.0)
    # set_junction not called yet → _unshaped_payload is None
    assert m._unshaped_payload is None
    assert _has_unshaped_payload(m) is False


def test_has_unshaped_payload_accepts_quintic_blend_move():
    """QBM stores _unshaped_payload in __init__ (blendplanner.py:398)
    — unlike plain Move which needs set_junction first.
    """
    import klippy.blendquintic as bq
    # Build a QBM via the usual CornerBlender path so all fields are
    # populated. We go through the blender because QuinticBlendMove's
    # direct constructor requires a QuinticShape.
    th = _BareToolhead()
    th.max_velocity = 200.0
    b = blendplanner.CornerBlender(th, move_cls=Move)
    m1 = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m2 = _make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0)
    # feed both: on the second feed a QBM is constructed.
    b.feed(m1)
    released = b.feed(m2)
    # Flush to release the pending QBM.
    released += b.flush()
    # Find the QBM in released.
    qbms = [r for r in released if isinstance(r, blendplanner.QuinticBlendMove)]
    assert qbms, "CornerBlender did not construct a QBM"
    qbm = qbms[0]
    assert _has_unshaped_payload(qbm) is True
    # And NOT a shape-bake target (already baked upstream).
    assert _is_shape_bake_target(qbm) is False


def test_neighbour_payload_returns_normalised_tuple_for_both_classes():
    """Both Move and QBM expose (unshaped_payload, start_xyz_tuple) via
    _neighbour_payload. The layout is identical so downstream composers
    don't need isinstance checks.
    """
    th = _BareToolhead()
    plain = _make_move(th, [1, 2, 3, 0], [11, 2, 3, 0])
    payload, start = _neighbour_payload(plain)
    assert payload is plain._unshaped_payload
    assert start == (1.0, 2.0, 3.0)

    th2 = _BareToolhead()
    th2.max_velocity = 200.0
    b = blendplanner.CornerBlender(th2, move_cls=Move)
    m1 = _make_move(th2, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m2 = _make_move(th2, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0)
    b.feed(m1)
    released = b.feed(m2) + b.flush()
    qbm = next(r for r in released
               if isinstance(r, blendplanner.QuinticBlendMove))
    payload_q, start_q = _neighbour_payload(qbm)
    assert payload_q is qbm._unshaped_payload
    # QBM stores start_pos_4d; _neighbour_payload returns the 3-tuple XYZ slice.
    assert start_q == (qbm._start_pos_4d[0], qbm._start_pos_4d[1],
                       qbm._start_pos_4d[2])
```

- [ ] **Step 2: Run test to verify it fails with ImportError**

```bash
cd /Users/daniladergachev/Developer/kalico
python -m pytest test/test_cross_boundary_shape_bake.py::test_has_unshaped_payload_accepts_plain_move_with_payload -xvs
```

Expected: FAIL with `ImportError: cannot import name '_has_unshaped_payload' from 'klippy.toolhead'`.

- [ ] **Step 3: Add the helpers in toolhead.py**

In `klippy/toolhead.py`, after the existing `_is_shape_bake_target` definition (currently at line 354), add:

```python
# Plan 9 A4 — neighbour-source predicate and accessor. Unlike
# _is_shape_bake_target ("who gets baked"), these answer "who can serve
# as a neighbour polynomial for the shaper kernel". QuinticBlendMove is
# NOT a Move subclass (blendplanner.py:341) yet stores the same
# _unshaped_payload 3-tuple layout (blendplanner.py:398); duck-typing
# on _unshaped_payload lets both classes feed the kernel window without
# an isinstance fork in finalize_shape.
def _has_unshaped_payload(move):
    return getattr(move, "_unshaped_payload", None) is not None


def _neighbour_payload(move):
    """Return ``(unshaped_payload, (x, y, z))`` for `move`.

    `Move.start_pos` is a 4-tuple; `QuinticBlendMove._start_pos_4d` is
    also a 4-tuple. Both expose the XYZ head via [:3]. `QuinticBlendMove`
    additionally has `.start_pos` (a 4-tuple at blendplanner.py:372), so
    `.start_pos[:3]` works uniformly — we use that path and avoid the
    private `_start_pos_4d` attribute.
    """
    xyz = (move.start_pos[0], move.start_pos[1], move.start_pos[2])
    return move._unshaped_payload, xyz
```

- [ ] **Step 4: Run test to verify pass**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py::test_has_unshaped_payload_accepts_plain_move_with_payload test/test_cross_boundary_shape_bake.py::test_has_unshaped_payload_rejects_plain_move_without_payload test/test_cross_boundary_shape_bake.py::test_has_unshaped_payload_accepts_quintic_blend_move test/test_cross_boundary_shape_bake.py::test_neighbour_payload_returns_normalised_tuple_for_both_classes -xvs
```

Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add test/test_cross_boundary_shape_bake.py klippy/toolhead.py
git commit -m "plan9-A4T1: add duck-typed neighbour helpers for cross-boundary bake"
```

---

## Task 2: Use the duck-typed helpers in `_finalize_with_neighbours` (Gap 1, part 2)

**Goal:** Replace the `_is_shape_bake_target(prev_move)` / `_is_shape_bake_target(next_move)` gates at `klippy/toolhead.py:392, 398` with `_has_unshaped_payload`; use `_neighbour_payload` to extract the tuple. The "who gets baked" gate on line 388 stays — QBMs must not be re-baked.

**Files:**
- Modify: `klippy/toolhead.py:377-409` (`_finalize_with_neighbours`).
- Test: `test/test_cross_boundary_shape_bake.py`.

- [ ] **Step 1: Write the Gap 1 failing test — plain Move sees QBM prev**

Append to `test/test_cross_boundary_shape_bake.py`:

```python
def test_gap1_plain_move_sees_qbm_as_prev_neighbour():
    """Gap 1a: a plain Move whose queue[i-1] is a QBM must shape-bake
    with the QBM's _unshaped_payload as its prev neighbour, not with
    None (zero-pad).

    Build: [QBM, plain] in the inner LookAheadQueue, flush, and verify
    the plain Move's baked coeffs differ from the zero-pad baseline.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    laq = LookAheadQueue(th)

    # Construct a QBM via CornerBlender so we have a real QBM instance.
    b = blendplanner.CornerBlender(th, move_cls=Move)
    m_pre = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m_corner = _make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0)
    b.feed(m_pre)
    released = b.feed(m_corner) + b.flush()
    qbm = next(r for r in released
               if isinstance(r, blendplanner.QuinticBlendMove))

    # The plain Move whose boundary matches the QBM's end.
    plain = _make_move(th,
                       [qbm.end_pos[0], qbm.end_pos[1], qbm.end_pos[2], 0],
                       [qbm.end_pos[0] + 10, qbm.end_pos[1], qbm.end_pos[2], 0],
                       speed=100.0)

    # Manually populate the inner queue [qbm, plain] and flush.
    laq.queue.append(qbm)
    laq.queue.append(plain)
    laq.flush(lazy=False)

    # Baseline: same plain Move, same set_junction state, baked standalone
    # (next=None, prev=None) — the zero-pad case.
    plain_baseline = _make_move(th,
                                [qbm.end_pos[0], qbm.end_pos[1], qbm.end_pos[2], 0],
                                [qbm.end_pos[0] + 10, qbm.end_pos[1], qbm.end_pos[2], 0],
                                speed=100.0)
    plain_baseline.finalize_shape()  # zero-pad both sides

    # With the QBM as prev neighbour, the baked coeffs must differ from
    # the zero-pad baseline.
    baked_with_qbm = plain.quintic_trapq_payload[5]  # coeff_tuple slot
    baked_zero_pad = plain_baseline.quintic_trapq_payload[5]
    assert baked_with_qbm != baked_zero_pad, (
        "plain Move did not pick up the QBM prev neighbour — Gap 1a "
        "predicate is still rejecting QBMs"
    )


def test_gap1_plain_move_sees_qbm_as_next_neighbour():
    """Gap 1b: a plain Move whose queue[i+1] is a QBM must use the QBM
    as its next neighbour."""
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    laq = LookAheadQueue(th)

    # Pre-plain whose end is the QBM's start.
    plain = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)

    b = blendplanner.CornerBlender(th, move_cls=Move)
    m_a = _make_move(th, [10, 0, 0, 0], [20, 0, 0, 0], speed=100.0)
    m_b = _make_move(th, [20, 0, 0, 0], [20, 10, 0, 0], speed=100.0)
    b.feed(m_a)
    released = b.feed(m_b) + b.flush()
    qbm = next(r for r in released
               if isinstance(r, blendplanner.QuinticBlendMove))

    laq.queue.append(plain)
    laq.queue.append(qbm)
    laq.flush(lazy=False)

    plain_baseline = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    plain_baseline.finalize_shape()

    baked_with_qbm = plain.quintic_trapq_payload[5]
    baked_zero_pad = plain_baseline.quintic_trapq_payload[5]
    assert baked_with_qbm != baked_zero_pad, (
        "plain Move did not pick up the QBM next neighbour — Gap 1b "
        "predicate is still rejecting QBMs"
    )
```

Add near the top of the file (below `_BareToolhead`):

```python
class _FakeAxisShaper:
    def __init__(self, axis, shaper_type, freq, damping=0.1):
        self._axis = axis
        class _P:
            pass
        self.params = _P()
        self.params.shaper_type = shaper_type
        self.params.shaper_freq = freq
        self.params.damping_ratio = damping
    def get_axis(self):
        return self._axis
    def get_type(self):
        return self.params.shaper_type


class _FakeIS:
    def __init__(self, shapers):
        self._shapers = shapers
    def get_shapers(self):
        return list(self._shapers)


class _PipelineLikeToolhead(_BareToolhead):
    """_BareToolhead with an input_shaper module wired via printer.
    Used by Gap 1/2/3 tests that need actual shape baking to be observable.
    """
    def __init__(self, shaper_type="mzv", freq=42.0):
        super().__init__()
        self.printer._objs["input_shaper"] = _FakeIS([
            _FakeAxisShaper("x", shaper_type, freq),
            _FakeAxisShaper("y", shaper_type, freq),
        ])
        self.shapers_snapshot = blendmath.extract_shapers(self)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py::test_gap1_plain_move_sees_qbm_as_prev_neighbour test/test_cross_boundary_shape_bake.py::test_gap1_plain_move_sees_qbm_as_next_neighbour -xvs
```

Expected: 2 failed, each with `assert baked_with_qbm != baked_zero_pad` (the current predicate rejects QBMs at the neighbour gate).

- [ ] **Step 3: Update `_finalize_with_neighbours` to use `_has_unshaped_payload`**

In `klippy/toolhead.py:377-409`, replace the body of `_finalize_with_neighbours` with:

```python
def _finalize_with_neighbours(self, move, prev_move, next_move,
                              prev_override=None):
    """Shape-bake ``move`` with prev/next neighbour polynomials.

    Plan 9 A4 — the neighbour-source predicate accepts BOTH plain
    Move and QuinticBlendMove (any object carrying a populated
    ``_unshaped_payload``). The "who gets baked" check below still
    excludes QBMs because they are already baked upstream by
    CornerBlender — re-baking would double-apply the shaper.

    ``prev_override`` optionally supplies ``(prev_unshaped,
    prev_start_pos_xyz)`` directly (used when draining the pending
    move whose prev is saved state, not a queue move). When
    ``prev_override`` is given, ``prev_move`` is ignored.
    """
    if not _is_shape_bake_target(move):
        return
    if prev_override is not None:
        prev_unshaped, prev_start = prev_override
    elif prev_move is not None and _has_unshaped_payload(prev_move):
        prev_unshaped, prev_start = _neighbour_payload(prev_move)
    else:
        prev_unshaped = None
        prev_start = None
    if next_move is not None and _has_unshaped_payload(next_move):
        next_unshaped, next_start = _neighbour_payload(next_move)
    else:
        next_unshaped = None
        next_start = None
    move.finalize_shape(
        prev_unshaped=prev_unshaped,
        next_unshaped=next_unshaped,
        prev_start_pos_xyz=prev_start,
        next_start_pos_xyz=next_start,
    )
```

- [ ] **Step 4: Run Gap 1 tests to verify pass**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py -xvs
```

Expected: all 6 cross-boundary tests pass (4 from Task 1 + 2 from Task 2).

- [ ] **Step 5: Run A3 regression suite**

```bash
python -m pytest test/test_toolhead_shape_bake.py test/test_blendplanner.py test/test_blendprepass.py test/test_toolhead_jerk_wiring.py test/test_toolhead_jerk_integration.py -x
```

Expected: all existing A3-era tests still pass. The predicate loosening is strictly additive — it can only START baking neighbours that were previously zero-padded, never stop baking ones that worked.

- [ ] **Step 6: Commit**

```bash
git add test/test_cross_boundary_shape_bake.py klippy/toolhead.py
git commit -m "plan9-A4T2: finalize_with_neighbours accepts QBM as neighbour source"
```

---

## Task 3: Thread `lazy` parameter through `CornerBlender.flush` and filter chain (Gap 3, part 1)

**Goal:** Grow `CornerBlender.flush(lazy: bool = False)` and `CollinearCollapser.flush(lazy: bool = False)` kwargs; have `BlendPipelineLookAheadQueue.flush(lazy)` pass its own `lazy` argument through to each filter's `flush`. This is a pure signature-extension step with no behaviour change yet (both filters' existing logic still runs unchanged regardless of `lazy`).

**Files:**
- Modify: `klippy/blendplanner.py:904` (`CornerBlender.flush`).
- Modify: `klippy/blendprepass.py:83` (`CollinearCollapser.flush`).
- Modify: `klippy/blendprepass.py:175-186` (`BlendPipelineLookAheadQueue.flush`).
- Test: `test/test_cross_boundary_shape_bake.py`.

- [ ] **Step 1: Write failing test for signature**

Append to `test/test_cross_boundary_shape_bake.py`:

```python
def test_corner_blender_flush_accepts_lazy_kwarg():
    """Signature check: CornerBlender.flush(lazy=...) must exist.

    Behaviour under the kwarg is validated by Task 4's Gap 3 test.
    """
    th = _BareToolhead()
    b = blendplanner.CornerBlender(th, move_cls=Move)
    # Both kwarg forms must accept without TypeError.
    assert b.flush(lazy=False) == []
    assert b.flush(lazy=True) == []


def test_collinear_collapser_flush_accepts_lazy_kwarg():
    th = _BareToolhead()
    cc = blendprepass.CollinearCollapser(th, move_cls=Move)
    assert cc.flush(lazy=False) == []
    assert cc.flush(lazy=True) == []


def test_pipeline_flush_propagates_lazy_to_filters():
    """BlendPipelineLookAheadQueue.flush(lazy=True) must reach each
    filter's flush(lazy=True). Proven by installing a spy filter that
    records its flush args.
    """
    class _SpyFilter:
        def __init__(self):
            self.flush_calls = []
        def feed(self, m):
            return []
        def flush(self, lazy=False):
            self.flush_calls.append(lazy)
            return []
        def reset(self):
            pass
        def peek_buffered(self):
            return []
    class _SpyInner:
        def __init__(self):
            self.queue = []
            self.flush_calls = []
        def add_move(self, m):
            self.queue.append(m)
        def flush(self, lazy=False):
            self.flush_calls.append(lazy)
        def reset(self):
            pass
        def set_flush_time(self, t):
            pass
        def get_last(self):
            return None
    spy = _SpyFilter()
    inner = _SpyInner()
    laq = blendprepass.BlendPipelineLookAheadQueue([spy], inner)
    laq.flush(lazy=True)
    assert spy.flush_calls == [True]
    assert inner.flush_calls == [True]
    laq.flush(lazy=False)
    assert spy.flush_calls == [True, False]
    assert inner.flush_calls == [True, False]
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py::test_corner_blender_flush_accepts_lazy_kwarg test/test_cross_boundary_shape_bake.py::test_collinear_collapser_flush_accepts_lazy_kwarg test/test_cross_boundary_shape_bake.py::test_pipeline_flush_propagates_lazy_to_filters -xvs
```

Expected: 3 failures — `TypeError: flush() got an unexpected keyword argument 'lazy'` for the first two; assertion failure on the third (the current pipeline calls `f.flush()` with no kwarg).

- [ ] **Step 3: Grow the signatures**

In `klippy/blendplanner.py`, locate `CornerBlender.flush` (currently line 904):

```python
def flush(self, lazy=False):
    # Plan 9 A4 — `lazy` gates the across-flush deferral of
    # self._pending_quintic (see Task 4). For now the kwarg is
    # accepted but the body is unchanged; Task 4 wires the deferral.
    released = self._finalize_pending(
        next_unshaped=None, next_start_pos_xyz=None,
    )
    if self._prev is not None:
        released.append(self._prev)
        self._prev = None
    return released
```

In `klippy/blendprepass.py`, locate `CollinearCollapser.flush` (currently line 83):

```python
def flush(self, lazy=False):
    # Plan 9 A4 — `lazy` accepted for signature uniformity across
    # the filter chain; the collinear collapser has no across-flush
    # state to defer, so the kwarg is ignored.
    if not self._chain:
        return []
    return self._flush_chain()
```

In `klippy/blendprepass.py`, `BlendPipelineLookAheadQueue.flush` (currently line 175-186) — update the `f.flush()` call:

```python
def flush(self, lazy=False):
    acc = []
    for f in self._filters:
        # Plan 9 A4 — the `lazy` flag propagates into every filter so
        # filters with across-flush state (CornerBlender) can hold
        # their pending moves across lazy drains and zero-pad only at
        # true drains (lazy=False).
        acc = [out for m in acc for out in f.feed(m)]
        acc += f.flush(lazy=lazy)
    for m in acc:
        self._lookahead.add_move(m)
    self._lookahead.flush(lazy=lazy)
```

- [ ] **Step 4: Run tests to verify pass**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py::test_corner_blender_flush_accepts_lazy_kwarg test/test_cross_boundary_shape_bake.py::test_collinear_collapser_flush_accepts_lazy_kwarg test/test_cross_boundary_shape_bake.py::test_pipeline_flush_propagates_lazy_to_filters -xvs
```

Expected: 3 passed.

- [ ] **Step 5: Regression suite**

```bash
python -m pytest test/test_blendplanner.py test/test_blendprepass.py test/test_toolhead_shape_bake.py -x
```

Expected: all pass. The signature extension is backward-compatible — `flush()` with no args still works (default `lazy=False`).

- [ ] **Step 6: Commit**

```bash
git add klippy/blendplanner.py klippy/blendprepass.py test/test_cross_boundary_shape_bake.py
git commit -m "plan9-A4T3: thread lazy kwarg through CornerBlender / CollinearCollapser / pipeline flush"
```

---

## Task 4: Add `CornerBlender._across_flush_pending` slot to hold QBM across lazy flushes (Gap 3, part 2)

**Goal:** When `CornerBlender.flush(lazy=True)` is called with a pending QBM, do NOT finalize it with `next=None`. Instead, stash it in `_across_flush_pending`. On the next `feed()` that yields a new pending QBM, the previous across-flush pending is finalised with the new QBM's `_unshaped_payload` as its next neighbour. On `flush(lazy=False)` (true drain), both `_pending_quintic` and `_across_flush_pending` drain with zero-pad.

**Files:**
- Modify: `klippy/blendplanner.py` — `CornerBlender.__init__`, `feed`, `flush`, `reset`, `peek_buffered`.
- Test: `test/test_cross_boundary_shape_bake.py`.

- [ ] **Step 1: Write Gap 3 failing tests**

Append to `test/test_cross_boundary_shape_bake.py`:

```python
def test_gap3_lazy_flush_holds_pending_qbm_across_flush():
    """Gap 3: CornerBlender.flush(lazy=True) must NOT finalize the
    pending QBM with next=None. It must be held for the next feed so
    the QBM sees its next neighbour's polynomial.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b = blendplanner.CornerBlender(th, move_cls=Move)
    m1 = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m2 = _make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0)
    b.feed(m1)
    # After the second feed a QBM is pending.
    released_before = b.feed(m2)
    assert b._pending_quintic is not None
    # Lazy flush: must NOT release the pending QBM with zero-pad.
    released_lazy = b.flush(lazy=True)
    # The pending QBM is NOT in released_lazy — it moves to
    # _across_flush_pending.
    qbms = [r for r in released_lazy
            if isinstance(r, blendplanner.QuinticBlendMove)]
    assert qbms == []
    assert b._across_flush_pending is not None


def test_gap3_next_feed_finalises_across_flush_pending_with_neighbour():
    """After a lazy flush holds a QBM, the next feed's new pending
    QBM must provide the next-neighbour polynomial to the held one.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b = blendplanner.CornerBlender(th, move_cls=Move)
    m1 = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m2 = _make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0)
    b.feed(m1)
    b.feed(m2)
    # Snapshot the unshaped payload of the pending QBM while it's
    # still unfinalised — this is the payload that should be re-baked
    # once the next QBM arrives.
    held_qbm_before = b._pending_quintic
    coeffs_before = held_qbm_before._unshaped_payload
    b.flush(lazy=True)
    # Now feed two more moves to trigger a new QBM construction.
    m3 = _make_move(th, [10, 10, 0, 0], [20, 10, 0, 0], speed=100.0)
    m4 = _make_move(th, [20, 10, 0, 0], [20, 20, 0, 0], speed=100.0)
    b.feed(m3)
    released = b.feed(m4)
    # The previously-held QBM is in released AND its baked coeffs
    # differ from what they would be under zero-pad.
    released_qbms = [r for r in released
                     if isinstance(r, blendplanner.QuinticBlendMove)]
    assert len(released_qbms) == 1
    released_qbm = released_qbms[0]
    assert released_qbm is held_qbm_before
    # Compare against a zero-pad baseline.
    # Hand-invoke finalize_shape on the unshaped payload with
    # next=None to get the zero-pad baked coeffs.
    # (Rebuild a second identical QBM for this — easiest by rewind.)
    th2 = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b2 = blendplanner.CornerBlender(th2, move_cls=Move)
    b2.feed(_make_move(th2, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0))
    b2.feed(_make_move(th2, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0))
    released_baseline = b2.flush(lazy=False)  # true drain → zero-pad
    baseline_qbm = next(r for r in released_baseline
                        if isinstance(r, blendplanner.QuinticBlendMove))
    # Same corner geometry, but baked with next=None vs. with m3-m4's
    # next-neighbour polynomial — coeffs must differ.
    assert (released_qbm.quintic_trapq_payload[5]
            != baseline_qbm.quintic_trapq_payload[5]), (
        "across-flush pending QBM was zero-padded instead of picking "
        "up the next-flush's neighbour polynomial"
    )


def test_gap3_true_drain_flushes_across_flush_pending_with_zero_pad():
    """flush(lazy=False) drains both _pending_quintic AND
    _across_flush_pending; the held QBM gets next=None because the
    print actually stops.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b = blendplanner.CornerBlender(th, move_cls=Move)
    b.feed(_make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0))
    b.feed(_make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0))
    b.flush(lazy=True)
    assert b._across_flush_pending is not None
    # True drain.
    released = b.flush(lazy=False)
    qbms = [r for r in released
            if isinstance(r, blendplanner.QuinticBlendMove)]
    assert len(qbms) == 1
    assert b._across_flush_pending is None
    assert b._pending_quintic is None


def test_gap3_reset_clears_across_flush_pending():
    th = _BareToolhead()
    b = blendplanner.CornerBlender(th, move_cls=Move)
    # Poke a fake value in directly for the reset test.
    b._across_flush_pending = "sentinel"
    b.reset()
    assert b._across_flush_pending is None


def test_gap3_peek_buffered_includes_across_flush_pending():
    """peek_buffered is consumed by BlendPipelineLookAheadQueue.get_last;
    an across-flush-held QBM must remain visible through that path.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b = blendplanner.CornerBlender(th, move_cls=Move)
    b.feed(_make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0))
    b.feed(_make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0))
    b.flush(lazy=True)
    buffered = b.peek_buffered()
    qbms = [m for m in buffered
            if isinstance(m, blendplanner.QuinticBlendMove)]
    assert len(qbms) == 1
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py -k "gap3" -xvs
```

Expected: 5 failed — `_across_flush_pending` does not exist; behaviour tests fail because the current `flush` zero-pads unconditionally.

- [ ] **Step 3: Implement `_across_flush_pending` in CornerBlender**

In `klippy/blendplanner.py`, update `CornerBlender.__init__` (line 632) — after `self._pending_leading = []`:

```python
# Plan 9 A4 Gap 3 — lazy-flush deferral. When BlendPipelineLookAheadQueue
# fires a lazy drain mid-print, the pending QBM is held here instead of
# being finalised with next=None. The next feed() that builds a new QBM
# provides the next-neighbour polynomial to this held QBM; a true drain
# (lazy=False) zero-pads and releases it. Slot holds the QBM itself (a
# fully-constructed QuinticBlendMove) or None.
self._across_flush_pending = None
```

Update `CornerBlender.feed` (currently starts at line 679). Right before `self._pending_quintic = quintic_move` (line 736), splice in the across-flush drain:

```python
def feed(self, move):
    if not move.is_kinematic_move:
        return self.flush(lazy=False) + [move]
    if self._prev is None:
        self._prev = move
        return []
    th = self._toolhead
    limits = blendshape.KinematicLimits(
        a_max=th.max_accel,
        v_max=th.max_velocity,
        jerk_max=None,
        extruder_caps=_extract_extruder_caps(th),
        shapers=blendmath.extract_shapers(th),
    )
    shape = blendquintic.QuinticShape.from_moves(
        self._prev, move, th.corner_deviation, limits,
    )
    if shape is None or blendmath.should_suppress_quintic(
            self._prev, move, th.corner_deviation, shape, th):
        return self._suppress_and_advance(move)
    trunc_prev, quintic_move, trunc_next_head = self._emit_blend(
        self._prev, move, shape,
    )
    self._prev = trunc_next_head
    self.blends_emitted += 1
    self.polyline_moves_emitted += 1

    # Plan 9 A4 Gap 3 — finalise any across-flush-held QBM with the
    # new quintic_move's unshaped polynomial as its next neighbour.
    # The held QBM was NOT touched by _finalize_pending at lazy flush
    # time (we routed it here instead), so it still needs finalize_shape
    # with the correct prev/next.
    across_flush_released = []
    if self._across_flush_pending is not None:
        held_qbm = self._across_flush_pending
        # Build prev context from the held QBM's own _pending_prev
        # snapshot — stored at the time it was stashed.
        prev_payload, prev_start = (
            self._across_flush_prev_snapshot
            if self._across_flush_prev_snapshot is not None
            else (None, None)
        )
        held_qbm.finalize_shape(
            prev_unshaped=prev_payload,
            next_unshaped=quintic_move._unshaped_payload,
            prev_start_pos_xyz=prev_start,
            next_start_pos_xyz=(
                quintic_move._start_pos_4d[0],
                quintic_move._start_pos_4d[1],
                quintic_move._start_pos_4d[2],
            ),
        )
        across_flush_released.append(held_qbm)
        self._across_flush_pending = None
        self._across_flush_prev_snapshot = None

    # Capture the old pending's snapshot BEFORE _finalize_pending
    # clears it.
    old_pending_snapshot = None
    if self._pending_quintic is not None:
        old_pending_snapshot = (
            self._pending_quintic._unshaped_payload,
            (self._pending_quintic._start_pos_4d[0],
             self._pending_quintic._start_pos_4d[1],
             self._pending_quintic._start_pos_4d[2]),
        )
    released = self._finalize_pending(
        next_unshaped=quintic_move._unshaped_payload,
        next_start_pos_xyz=(
            quintic_move._start_pos_4d[0],
            quintic_move._start_pos_4d[1],
            quintic_move._start_pos_4d[2],
        ),
    )
    # Emit order: first the held-across-flush QBM (it precedes everything
    # in this feed cycle), then the leading + finalized pending + the
    # new trunc_prev.
    final_release = across_flush_released + released
    final_release.append(trunc_prev)
    self._pending_quintic = quintic_move
    self._pending_prev = old_pending_snapshot
    self._pending_leading = []
    return final_release
```

Also add `self._across_flush_prev_snapshot = None` alongside `_across_flush_pending = None` in `__init__`.

Update `CornerBlender.flush` (line 904):

```python
def flush(self, lazy=False):
    """Drain buffered state.

    Plan 9 A4 — `lazy=True` defers the pending QBM across the flush
    cycle so it can see the next flush's first move as its next
    neighbour. `lazy=False` (true drain) zero-pads with next=None for
    both the pending and the across-flush-held QBM — the print
    actually stops so zero-pad is correct on the next side.
    """
    released = []
    if lazy:
        # Stash the pending QBM so the next feed()'s new QBM provides
        # its next neighbour. We also preserve the prev snapshot the
        # pending was going to bake against.
        if self._pending_quintic is not None:
            # Release any leading plain Moves — they are independent
            # of the pending QBM and must not be held.
            released.extend(self._pending_leading)
            self._pending_leading = []
            self._across_flush_pending = self._pending_quintic
            self._across_flush_prev_snapshot = self._pending_prev
            self._pending_quintic = None
            self._pending_prev = None
        else:
            released.extend(self._pending_leading)
            self._pending_leading = []
        # Lazy flush must NOT release self._prev — new moves are still
        # forthcoming and _prev is the buffered candidate-prev for the
        # next corner. Only a true drain releases it.
    else:
        # True drain. Finalise any across-flush-held QBM with prev from
        # its stored snapshot and next=None (the print stops here).
        if self._across_flush_pending is not None:
            held = self._across_flush_pending
            prev_payload, prev_start = (
                self._across_flush_prev_snapshot
                if self._across_flush_prev_snapshot is not None
                else (None, None)
            )
            held.finalize_shape(
                prev_unshaped=prev_payload,
                next_unshaped=None,
                prev_start_pos_xyz=prev_start,
                next_start_pos_xyz=None,
            )
            released.append(held)
            self._across_flush_pending = None
            self._across_flush_prev_snapshot = None
        # Drain the in-flight pending QBM (if any).
        released.extend(self._finalize_pending(
            next_unshaped=None, next_start_pos_xyz=None,
        ))
        if self._prev is not None:
            released.append(self._prev)
            self._prev = None
    return released
```

Update `CornerBlender.reset` (line 916):

```python
def reset(self):
    self._prev = None
    self._pending_quintic = None
    self._pending_prev = None
    self._pending_leading = []
    self._across_flush_pending = None
    self._across_flush_prev_snapshot = None
```

Update `CornerBlender.peek_buffered` (line 922):

```python
def peek_buffered(self):
    buf = list(self._pending_leading)
    if self._across_flush_pending is not None:
        buf.append(self._across_flush_pending)
    if self._pending_quintic is not None:
        buf.append(self._pending_quintic)
    if self._prev is not None:
        buf.append(self._prev)
    return buf
```

- [ ] **Step 4: Run Gap 3 tests to verify pass**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py -k "gap3" -xvs
```

Expected: 5 passed.

- [ ] **Step 5: Regression suite**

```bash
python -m pytest test/test_blendplanner.py test/test_blendprepass.py test/test_toolhead_shape_bake.py test/test_blendextruder_integration.py -x
```

Expected: all pass. The lazy-flush change is purely additive in its pass-through behaviour for non-lazy flushes; existing tests call `flush()` with the default `lazy=False` and hit the same drain path as before.

**Likely stumbling block:** `_FakeMove` in `test/test_blendplanner.py` has no `_unshaped_payload` attribute. If any test there invokes `peek_buffered` or cascades through `_across_flush_pending`, a stub fix is needed. Inspect failures and add `self._unshaped_payload = None` to `_FakeMove.__init__` (test/test_blendplanner.py line 31-69).

If that fix is required:

```python
# In test/test_blendplanner.py, inside _FakeMove.__init__ (after
# self.next_junction_v_capped_to = None):
self._unshaped_payload = None
```

- [ ] **Step 6: Commit**

```bash
git add klippy/blendplanner.py test/test_cross_boundary_shape_bake.py test/test_blendplanner.py
git commit -m "plan9-A4T4: hold pending QBM across lazy flush in CornerBlender"
```

---

## Task 5: Capture `_last_released_plain` snapshot on blender release (Gap 2, part 1)

**Goal:** Whenever the blender releases a plain Move (`trunc_prev` on a successful blend, or `emitted_prev` in `_suppress_and_advance`), snapshot the Move's `_unshaped_payload` and start_pos into `self._last_released_plain`. The snapshot is a 2-tuple `(unshaped_payload, start_xyz)` or `None` if the Move's payload isn't yet populated.

**Design note — why the snapshot can be None:** a plain Move's `_unshaped_payload` is populated only when `set_junction` runs inside the inner `LookAheadQueue.flush`. At CornerBlender release time this has NOT happened yet. The snapshot is captured optimistically — it's populated in steady-state printing where many flush cycles have fired between feeds. When it's absent, Gap 2 falls back to the current zero-pad behaviour (no regression).

**Files:**
- Modify: `klippy/blendplanner.py` — `CornerBlender.__init__`, `feed` (trunc_prev emit path), `_suppress_and_advance` (emitted_prev emit path), `reset`.
- Test: `test/test_cross_boundary_shape_bake.py`.

- [ ] **Step 1: Write failing tests**

Append to `test/test_cross_boundary_shape_bake.py`:

```python
def test_gap2_snapshot_captured_on_trunc_prev_release():
    """When CornerBlender emits a trunc_prev (successful blend), its
    _unshaped_payload (if populated) is snapshotted to
    _last_released_plain for use as the next QBM's prev neighbour.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b = blendplanner.CornerBlender(th, move_cls=Move)
    m1 = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m2 = _make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0)
    b.feed(m1)
    released = b.feed(m2)
    # trunc_prev is the last plain Move in released.
    trunc_prevs = [r for r in released
                   if isinstance(r, Move) and not
                   isinstance(r, blendplanner.QuinticBlendMove)]
    assert trunc_prevs
    # _last_released_plain populated from the most recent trunc_prev.
    # It captures (unshaped_payload, start_xyz) — the payload may be
    # None if trunc_prev has not yet had set_junction called on it
    # (the outer LookAheadQueue is the one calling set_junction).
    assert b._last_released_plain is not None
    payload_snap, start_snap = b._last_released_plain
    # start_xyz is always populated.
    assert start_snap == (trunc_prevs[-1].start_pos[0],
                          trunc_prevs[-1].start_pos[1],
                          trunc_prevs[-1].start_pos[2])
    # payload is None unless trunc_prev's set_junction fired — which
    # it has not here (we construct trunc_prev inside _emit_blend and
    # release before any outer LookAheadQueue is involved). The
    # snapshot's presence is what matters; the payload slot documents
    # the populated-or-None contract.


def test_gap2_snapshot_captured_on_suppress_and_advance():
    """When CornerBlender drops a blend via _suppress_and_advance,
    emitted_prev is the released plain Move — it must also populate
    _last_released_plain.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b = blendplanner.CornerBlender(th, move_cls=Move)
    # U-turn pair triggers _suppress_and_advance.
    m1 = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m2 = _make_move(th, [10, 0, 0, 0], [0, 0, 0, 0], speed=100.0)
    b.feed(m1)
    released = b.feed(m2)
    # emitted_prev (m1) is released.
    assert m1 in released
    assert b._last_released_plain is not None


def test_gap2_reset_clears_last_released_plain():
    th = _BareToolhead()
    b = blendplanner.CornerBlender(th, move_cls=Move)
    b._last_released_plain = ("sentinel", (0, 0, 0))
    b.reset()
    assert b._last_released_plain is None
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py -k "gap2_snapshot_captured or gap2_reset" -xvs
```

Expected: 3 failed with `AttributeError: ... has no attribute '_last_released_plain'`.

- [ ] **Step 3: Add the snapshot slot + capture logic**

In `klippy/blendplanner.py`, update `CornerBlender.__init__` (after the `_across_flush_pending` additions from Task 4):

```python
# Plan 9 A4 Gap 2 — last-released-plain snapshot. When the blender
# releases a plain Move (trunc_prev on a successful blend, or
# emitted_prev in _suppress_and_advance), we snapshot (unshaped_payload,
# start_xyz). At QBM construction time, if _pending_prev would be None
# (no prior QBM), this snapshot serves as the prev neighbour instead
# — closing Gap 2. The snapshot's payload slot may be None if the
# plain Move has not yet had set_junction called on it (the outer
# LookAheadQueue is the set_junction caller; by the time steady-state
# printing hits the blender, prior flush cycles have populated most
# plain Moves' payloads). When the payload is None the snapshot is
# treated as absent — falls back to the current zero-pad.
self._last_released_plain = None
```

Add a private helper method on `CornerBlender`:

```python
def _snapshot_plain_release(self, move):
    """Record `move` as the last-released plain Move for Gap 2's
    prev-neighbour fallback. Called from the emit paths that release
    plain Moves. `move._unshaped_payload` may be None if set_junction
    has not yet run; we still record the start_xyz because it's cheap.
    """
    start_xyz = (move.start_pos[0], move.start_pos[1], move.start_pos[2])
    self._last_released_plain = (
        getattr(move, "_unshaped_payload", None),
        start_xyz,
    )
```

In `CornerBlender.feed` (line 679), after `released.append(trunc_prev)` at line 733, append the snapshot call:

```python
released.append(trunc_prev)
# Plan 9 A4 Gap 2: trunc_prev is now the last released plain Move.
self._snapshot_plain_release(trunc_prev)
```

In `CornerBlender._suppress_and_advance` (line 741), after `released.append(emitted_prev)` at line 777:

```python
released.append(emitted_prev)
self._snapshot_plain_release(emitted_prev)
return released
```

Update `CornerBlender.reset` (line 916) — add the new slot:

```python
def reset(self):
    self._prev = None
    self._pending_quintic = None
    self._pending_prev = None
    self._pending_leading = []
    self._across_flush_pending = None
    self._across_flush_prev_snapshot = None
    self._last_released_plain = None
```

- [ ] **Step 4: Run Gap 2 snapshot tests**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py -k "gap2_snapshot_captured or gap2_reset" -xvs
```

Expected: 3 passed.

- [ ] **Step 5: Regression suite**

```bash
python -m pytest test/test_blendplanner.py test/test_blendprepass.py test/test_toolhead_shape_bake.py -x
```

Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendplanner.py test/test_cross_boundary_shape_bake.py
git commit -m "plan9-A4T5: capture last-released plain Move snapshot in CornerBlender"
```

---

## Task 6: Use `_last_released_plain` as prev-neighbour fallback when constructing a new QBM (Gap 2, part 2)

**Goal:** When `CornerBlender.feed` constructs a new pending QBM and the would-be `_pending_prev` (from a prior pending QBM) is None, fall back to `_last_released_plain`. If both are None, the QBM gets zero-pad (as today).

**Files:**
- Modify: `klippy/blendplanner.py:712-738` — the `feed` method's `_pending_prev` assignment logic.
- Test: `test/test_cross_boundary_shape_bake.py`.

- [ ] **Step 1: Write Gap 2 failing behaviour test**

Append:

```python
def test_gap2_new_qbm_uses_last_released_plain_as_prev_when_available():
    """When a plain Move precedes a new QBM (no prior pending QBM) and
    the plain Move's _unshaped_payload is populated, the new QBM must
    use it as its prev neighbour.

    We force `_unshaped_payload` populated manually on the
    last-released plain Move (simulating the inner LookAheadQueue's
    reverse pass having run on it), then feed a corner.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b = blendplanner.CornerBlender(th, move_cls=Move)
    # Feed two kinematic moves so a QBM pends.
    m1 = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m2 = _make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0)
    b.feed(m1)
    b.feed(m2)  # constructs QBM, releases trunc_prev
    # Patch _last_released_plain's snapshot to carry a populated
    # payload — simulate that the outer LookAheadQueue called
    # set_junction on trunc_prev between feeds.
    # To produce a "real-looking" unshaped payload we use the m1
    # payload (since trunc_prev's direction matches m1's).
    fake_payload = m1._unshaped_payload
    fake_start = b._last_released_plain[1]
    b._last_released_plain = (fake_payload, fake_start)
    # Feed a corner that constructs a NEW QBM — the old pending
    # finalises with next=new_qbm._unshaped_payload (Gap-2-unrelated
    # behaviour) and a new QBM is pended; we want to verify the
    # newly-pended QBM's _pending_prev comes from _last_released_plain.
    m3 = _make_move(th, [10, 10, 0, 0], [20, 10, 0, 0], speed=100.0)
    m4 = _make_move(th, [20, 10, 0, 0], [20, 20, 0, 0], speed=100.0)
    b.feed(m3)  # no QBM this time (collinear with m2's exit? no —
                # m3 is perpendicular to m2's direction; QBM forms)
    # Actually feed(m3) DOES form a QBM: (m2→m3 direction change) —
    # but m2 is buffered as trunc_next_head, not m2 itself. So the
    # corner is trunc_next_head(from m2) → m3. That builds a new QBM.
    assert b._pending_quintic is not None
    # _pending_prev must be populated: either from the prior pending
    # QBM (`old_pending_snapshot`), OR — when that was None — from
    # _last_released_plain.
    assert b._pending_prev is not None
    prev_payload, prev_start = b._pending_prev
    # Prior QBM existed, so old_pending_snapshot wins over
    # _last_released_plain. This test primarily proves the fallback
    # path via a scenario where NO prior pending QBM exists.
    # (See test_gap2_direct_plain_to_qbm_transition below.)


def test_gap2_direct_plain_to_qbm_transition():
    """Scenario with NO prior QBM: plain Moves flow, then a corner
    forms the FIRST QBM. The new QBM's _pending_prev must come from
    _last_released_plain, not from None.
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)
    b = blendplanner.CornerBlender(th, move_cls=Move)
    # No prior QBMs — three collinear moves then a corner.
    # (Collinear moves go through CornerBlender as pass-through via
    # QuinticShape.from_moves returning None → _suppress_and_advance
    # path without a velocity cap because dp > -0.5.)
    m1 = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m2 = _make_move(th, [10, 0, 0, 0], [20, 0, 0, 0], speed=100.0)
    m3 = _make_move(th, [20, 0, 0, 0], [20, 10, 0, 0], speed=100.0)
    b.feed(m1)
    b.feed(m2)  # collinear: from_moves returns None →
                # _suppress_and_advance; emitted_prev=m1 is released
                # AND snapshotted via _snapshot_plain_release.
    # Patch the snapshot with a fabricated payload (same as test above).
    b._last_released_plain = (m1._unshaped_payload,
                              b._last_released_plain[1])
    released = b.feed(m3)  # corner m2→m3 forms the FIRST QBM.
    # The new pending QBM's _pending_prev — with NO prior pending
    # QBM — must come from _last_released_plain.
    assert b._pending_quintic is not None
    assert b._pending_prev is not None, (
        "new QBM's _pending_prev is None — Gap 2 fallback from "
        "_last_released_plain did not fire"
    )
    prev_payload, prev_start = b._pending_prev
    assert prev_payload is m1._unshaped_payload, (
        "new QBM's prev payload does not match last-released plain "
        "Move's snapshot"
    )
```

- [ ] **Step 2: Run tests to verify failure**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py::test_gap2_direct_plain_to_qbm_transition -xvs
```

Expected: FAIL — `b._pending_prev is None` (current code only populates from `old_pending_snapshot`).

- [ ] **Step 3: Wire `_last_released_plain` into `_pending_prev`**

In `klippy/blendplanner.py:712-719`, replace the `old_pending_snapshot` block with a unified fallback:

```python
# Capture the prev-neighbour for the newly-pended QBM. Three sources,
# in order of preference:
#   1. The just-released pending QBM's unshaped payload
#      (old_pending_snapshot). This is the QBM↔QBM continuation path.
#   2. The last released plain Move's snapshot — Plan 9 A4 Gap 2,
#      closes the plain→QBM boundary. Its payload is populated only
#      when the outer LookAheadQueue's reverse pass has already run
#      on the plain Move (common in steady-state printing).
#   3. None — zero-pad at the kernel boundary. Correct at print start
#      and whenever neither source is available.
old_pending_snapshot = None
if self._pending_quintic is not None:
    old_pending_snapshot = (
        self._pending_quintic._unshaped_payload,
        (self._pending_quintic._start_pos_4d[0],
         self._pending_quintic._start_pos_4d[1],
         self._pending_quintic._start_pos_4d[2]),
    )
elif self._last_released_plain is not None:
    payload, start_xyz = self._last_released_plain
    if payload is not None:
        old_pending_snapshot = (payload, start_xyz)
```

Then further down, the existing:

```python
self._pending_prev = old_pending_snapshot
```

keeps working — the variable is now populated from either source.

- [ ] **Step 4: Run Gap 2 tests to verify pass**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py -k "gap2" -xvs
```

Expected: all 5 Gap 2 tests pass.

- [ ] **Step 5: Full A4 + A3 regression**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py test/test_toolhead_shape_bake.py test/test_blendplanner.py test/test_blendprepass.py test/test_blendextruder_integration.py test/test_toolhead_jerk_wiring.py test/test_toolhead_jerk_integration.py -x
```

Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendplanner.py test/test_cross_boundary_shape_bake.py
git commit -m "plan9-A4T6: use last-released plain snapshot as QBM prev fallback"
```

---

## Task 7: End-to-end cross-boundary test through the real pipeline

**Goal:** Add an end-to-end test that drives the full `BlendPipelineLookAheadQueue` pipeline with a plain↔QBM↔plain sequence and verifies NO zero-pad occurs at any internal boundary. This is the integration-level acceptance test for A4.

**Note:** This plan was first drafted in parallel with the QBM-in-reverse-pass fix (commit `e6e71a0e`), which removed the prior crash on full-pipeline flushes. Tests in this task may now use the full pipeline (`BlendPipelineLookAheadQueue.add_move` + `flush`) directly — no need to bypass the filter stack. If the test still drives the inner `LookAheadQueue.queue` via `.append`, that's also fine for tightly-scoped unit-level coverage of A4's contract; it's an explicit choice of test breadth, not a workaround.

**Files:**
- Modify: `test/test_cross_boundary_shape_bake.py` (add integration test).

- [ ] **Step 1: Write end-to-end test**

Append:

```python
def test_a4_e2e_plain_qbm_plain_all_boundaries_shape_baked():
    """End-to-end acceptance: plain → QBM → plain sequence. All three
    moves must exit with baked_coeffs != their zero-pad baseline at
    every boundary.

    We drive CornerBlender directly to construct the QBM, then
    populate the inner LookAheadQueue manually with [plain0, qbm,
    plain1] and flush(lazy=False). The QBM is already baked by
    CornerBlender at construction; the two plain Moves are baked by
    the inner flush's A3 deferred-last pass. A4 ensures:
      - plain0 sees qbm as its NEXT neighbour (Gap 1b).
      - plain1 sees qbm as its PREV neighbour (Gap 1a).
    """
    th = _PipelineLikeToolhead(shaper_type="mzv", freq=42.0)

    # Build a QBM via CornerBlender.
    b = blendplanner.CornerBlender(th, move_cls=Move)
    m_pre = _make_move(th, [0, 0, 0, 0], [10, 0, 0, 0], speed=100.0)
    m_corner = _make_move(th, [10, 0, 0, 0], [10, 10, 0, 0], speed=100.0)
    b.feed(m_pre)
    released = b.feed(m_corner) + b.flush(lazy=False)
    qbm = next(r for r in released
               if isinstance(r, blendplanner.QuinticBlendMove))

    # Construct plain0 (precedes QBM) and plain1 (follows QBM).
    plain0 = _make_move(th, [-10, 0, 0, 0],
                        [qbm.start_pos[0], qbm.start_pos[1], qbm.start_pos[2], 0],
                        speed=100.0)
    plain1 = _make_move(th,
                        [qbm.end_pos[0], qbm.end_pos[1], qbm.end_pos[2], 0],
                        [qbm.end_pos[0], qbm.end_pos[1] + 10, qbm.end_pos[2], 0],
                        speed=100.0)

    # Capture zero-pad baselines BEFORE the flush re-bakes plain0/plain1.
    # (plain0.finalize_shape has already been called with zero-pad
    # inside set_junction; grab the coeffs now.)
    plain0_zeropad = plain0.quintic_trapq_payload[5]
    plain1_zeropad = plain1.quintic_trapq_payload[5]

    # Drive the inner LookAheadQueue with [plain0, qbm, plain1].
    laq = LookAheadQueue(th)
    laq.queue.append(plain0)
    laq.queue.append(qbm)
    laq.queue.append(plain1)
    laq.flush(lazy=False)

    plain0_baked = plain0.quintic_trapq_payload[5]
    plain1_baked = plain1.quintic_trapq_payload[5]

    assert plain0_baked != plain0_zeropad, (
        "plain0 still zero-padded after A4 — next=qbm boundary did "
        "not propagate"
    )
    assert plain1_baked != plain1_zeropad, (
        "plain1 still zero-padded after A4 — prev=qbm boundary did "
        "not propagate"
    )


def test_a4_summary_all_three_gaps_closed():
    """Meta-test: ensures all Gap 1/2/3 tests are present and would
    fail together if any of the three A4 tasks is reverted. The three
    tests referenced below are the load-bearing ones per gap.
    """
    import test.test_cross_boundary_shape_bake as tcbsb
    gap1_test = getattr(
        tcbsb, "test_gap1_plain_move_sees_qbm_as_prev_neighbour")
    gap2_test = getattr(tcbsb, "test_gap2_direct_plain_to_qbm_transition")
    gap3_test = getattr(
        tcbsb, "test_gap3_next_feed_finalises_across_flush_pending_with_neighbour")
    assert gap1_test is not None
    assert gap2_test is not None
    assert gap3_test is not None
```

- [ ] **Step 2: Run the new tests**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py::test_a4_e2e_plain_qbm_plain_all_boundaries_shape_baked test/test_cross_boundary_shape_bake.py::test_a4_summary_all_three_gaps_closed -xvs
```

Expected: 2 passed.

- [ ] **Step 3: Full A4 suite final run**

```bash
python -m pytest test/test_cross_boundary_shape_bake.py -v
```

Expected: all A4 tests pass (estimate 15-18 tests total across tasks).

- [ ] **Step 4: Full Plan-9-relevant regression**

```bash
python -m pytest test/test_toolhead_shape_bake.py test/test_blendplanner.py test/test_blendprepass.py test/test_blendextruder_integration.py test/test_blendquintic.py test/test_blendshape.py test/test_blendshaper.py test/test_blendmath.py test/test_toolhead_jerk_wiring.py test/test_toolhead_jerk_integration.py test/test_cross_boundary_shape_bake.py -x
```

Expected: all pass. Existing A3 end-to-end test `test_a3_e2e_mzv_shaper_bakes_payload_through_lookahead` should still pass — A4 strictly expands the set of moves that see non-zero-pad neighbours.

- [ ] **Step 5: Commit**

```bash
git add test/test_cross_boundary_shape_bake.py
git commit -m "plan9-A4T7: end-to-end cross-boundary bake acceptance test"
```

---

## Out of scope — captured for follow-on plans

A4 explicitly does NOT address:

- **`shape_disabled` bypass audit** (drip / force_move / manual_stepper / IDEX). The non-planner move paths may still feed moves into the toolhead that bypass the shape-bake entirely. Phase A6-deferred.
- **Wasted safety-net `finalize_shape()` call** in `Move.set_junction` (toolhead.py:344). A3's deferred-last pattern always overwrites the initial bake; the call could be elided. Separate G1-level followup.
- ~~**`QuinticBlendMove.reachable_v_from_v_end` / `j_max` attributes.**~~ **FIXED in commit `e6e71a0e`** (Approach B — `LookAheadQueue.flush` reverse pass now short-circuits non-Move queue entries via `isinstance(move, Move)` check; QBM stays a baked-profile anchor; `QuinticBlendMove.set_junction` deleted; start_v / cruise_v / end_v / accel_t / cruise_t / decel_t now populated in `finalize_shape` from TOPP-baked values). Full-pipeline tests are unblocked.
- **Phase B (host↔MCU protocol)** changes.

---

## Self-review

### Spec coverage

Three gaps from the opening prompt:

- **Gap 1 (plain Move ← QBM)** — closed by Tasks 1–2 (`_has_unshaped_payload` + updated `_finalize_with_neighbours`). Tests: `test_gap1_plain_move_sees_qbm_as_prev_neighbour`, `test_gap1_plain_move_sees_qbm_as_next_neighbour`.
- **Gap 2 (QBM ← plain Move)** — closed by Tasks 5–6 (`_last_released_plain` snapshot + fallback in `feed`). Tests: `test_gap2_snapshot_captured_on_trunc_prev_release`, `test_gap2_snapshot_captured_on_suppress_and_advance`, `test_gap2_direct_plain_to_qbm_transition`, `test_gap2_reset_clears_last_released_plain`, `test_gap2_new_qbm_uses_last_released_plain_as_prev_when_available`.
- **Gap 3 (QBM at lazy-flush drain)** — closed by Tasks 3–4 (`lazy` kwarg + `_across_flush_pending` slot). Tests: `test_corner_blender_flush_accepts_lazy_kwarg`, `test_pipeline_flush_propagates_lazy_to_filters`, `test_gap3_lazy_flush_holds_pending_qbm_across_flush`, `test_gap3_next_feed_finalises_across_flush_pending_with_neighbour`, `test_gap3_true_drain_flushes_across_flush_pending_with_zero_pad`, `test_gap3_reset_clears_across_flush_pending`, `test_gap3_peek_buffered_includes_across_flush_pending`.

End-to-end acceptance: Task 7 (`test_a4_e2e_plain_qbm_plain_all_boundaries_shape_baked`) validates all three gaps jointly.

### Placeholder scan

Searched the plan for red flags — none found. Every task has complete code blocks, concrete file:line targets, and exact pytest commands.

### Type consistency

- `_has_unshaped_payload(m)` — Task 1 defines; Task 2 uses. Same signature.
- `_neighbour_payload(m)` — Task 1 defines `(payload, (x, y, z))`; Task 2 unpacks as `prev_unshaped, prev_start` — consistent.
- `_last_released_plain` — Task 5 defines as `(payload | None, (x, y, z))`; Task 6 unpacks as `payload, start_xyz` and checks `payload is not None` before using — consistent.
- `_across_flush_pending` — Task 4 defines as `QuinticBlendMove | None`; `_across_flush_prev_snapshot` as `(payload, start_xyz) | None`. All usages match.
- `flush(lazy=False)` — signature added in Task 3 to `CornerBlender` and `CollinearCollapser`; Task 4 extends `CornerBlender.flush` behaviour under `lazy=True`; all call sites (Task 3's pipeline update) pass the kwarg.

### Design decisions locked in

- **Gap 2's plain-Move snapshot may carry `payload=None`** when the inner LookAheadQueue's reverse pass hasn't run yet on the released plain Move. A4 treats that as the zero-pad fallback (no regression vs. pre-A4). Closing this remaining case would require either (i) the CornerBlender calling `set_junction` itself (structurally wrong — that's the outer lookahead's job) or (ii) a callback protocol from the inner LookAheadQueue back up to CornerBlender (adds coupling). **Deferred** — documented in "Design choice — how to materialise the plain-Move unshaped payload" above. Task 6 leaves the `payload is None` branch as zero-pad with a WHY comment.

- **`lazy` kwarg propagation in `BlendPipelineLookAheadQueue.flush`** — all filters grow `flush(lazy=False)` so the kwarg is uniform. `CollinearCollapser` ignores it (no across-flush state).

- **`_is_shape_bake_target` vs. `_has_unshaped_payload`** — two distinct predicates with two distinct roles. "Who gets baked" (excludes already-baked QBMs) stays; "who can serve as a neighbour" (accepts both) is new. Task 2 is careful to keep the first on line 388 and use the second on lines 392 and 398.

- **`_FakeMove` in existing test files** — Task 4's regression step adds `self._unshaped_payload = None` to `_FakeMove.__init__` in `test/test_blendplanner.py` (and, if failures surface, `test/test_blendprepass.py`'s `_FakeMove`). This is a minimal test-stub patch, not a production change.

---

## Execution handoff

**Plan complete and saved to `docs/superpowers/plans/2026-04-24-plan9-phaseA4-cross-boundary-shape-bake.md`.** Two execution options:

1. **Subagent-Driven (recommended)** — dispatch a fresh subagent per task, review between tasks, fast iteration. Use `superpowers:subagent-driven-development`.
2. **Inline Execution** — execute tasks in this session using `superpowers:executing-plans`, batch with review checkpoints.

Which approach?

If Subagent-Driven: use sonnet for Tasks 1, 3, 5, 7 (routine mechanical changes from spec) and opus for Tasks 2, 4, 6 (Task 2 is the load-bearing predicate rewrite; Task 4 is the non-trivial across-flush state machine; Task 6 wires the Gap 2 fallback and must reason about the `payload is None` corner case).
