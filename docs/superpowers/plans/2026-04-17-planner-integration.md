# Planner Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `klippy/blendplanner.py` — the `CornerBlender` filter that consumes the prepass output and emits fine-segmented tangent-arc polylines, plus a generalized `BlendPipelineLookAheadQueue` adapter replacing the old `PrepassLookAheadQueue`, plus the `ToolHead` wiring that reads `corner_deviation` from config and appends both filters to the pipeline.

**Architecture:** `CornerBlender` buffers one `Move` (candidate prev). When the next move arrives, it calls `blendmath.blend_from_moves(prev, next, corner_deviation, toolhead=toolhead)`, emits `[trunc_prev, arc_polyline_moves...]`, and buffers `next_trunc_head` as the new candidate prev. `_copy_caller_state(src, dst)` transfers all caller-mutable Move fields (`timing_callbacks`, `next_junction_v2`, `max_cruise_v2`, `junction_deviation`, `accel`) through the truncation so an upstream caller's `get_last().mutate()` ends up on the real queued move. `BlendPipelineLookAheadQueue(filters, lookahead)` is a generic ordered filter-chain adapter; `get_last` peeks via `peek_buffered()` instead of flushing, so buffered blend candidates are not forfeited.

**Tech Stack:** Python 3, pytest, Kalico motion pipeline (`klippy/toolhead.py`, `klippy/blendmath.py`, `klippy/blendprepass.py`).

**Spec:** `docs/superpowers/specs/2026-04-17-planner-integration-design.md`.

---

## File structure

- `klippy/blendmath.py` — MODIFY — half-segment rule factor at `:145`; comment + docstring updates (Task 1).
- `klippy/blendprepass.py` — MODIFY — add `CollinearCollapser.peek_buffered()` (Task 2); rename `PrepassLookAheadQueue` → `BlendPipelineLookAheadQueue` with generic filter-chain constructor + `peek`-based `get_last` / `queue` (Task 13).
- `klippy/blendplanner.py` — NEW — `CornerBlender` class (Tasks 3–12).
- `klippy/toolhead.py` — MODIFY — parse `corner_deviation` from config, instantiate `CornerBlender`, wire into `BlendPipelineLookAheadQueue([prepass, blender], inner)`, append stats counters (Task 13).
- `test/test_blendmath.py` — MODIFY — fixtures that assert `R_mid` values now halve (Task 1).
- `test/test_blendprepass.py` — MODIFY — add `peek_buffered()` tests (Task 2); rename + rewrite adapter tests for new signature and no-flush `get_last` semantic (Task 13).
- `test/test_blendplanner.py` — NEW — all blender unit + property tests (Tasks 3–12).

**Move-class injection:** `CornerBlender.__init__` takes a `move_cls` callable alongside `toolhead`. Tests pass `_FakeMove` (faithful `Move.__init__` reimplementation, copied from `test_blendprepass.py` for isolation — no shared fixture module). Production: `ToolHead.__init__` passes `Move` explicitly.

**Shared-helper note:** `_copy_caller_state` is a **private module-level function** (not a method) inside `klippy/blendplanner.py`, so tests can exercise it without constructing a full `CornerBlender`.

---

## Task 1: Half-segment rule in `blendmath` + test fixture updates (atomic; Pair A)

**Files:**
- Modify: `klippy/blendmath.py:141-145`
- Modify: `test/test_blendmath.py:181-204` and `:780-808`

- [ ] **Step 1: Update the failing fixtures first (TDD doesn't strictly apply — this is a geometry constant change driven by the spec review)**

In `test/test_blendmath.py`, update `test_blend_geometry_midpoint_cap_binds_on_short_segment` (around line 181). Replace the body with:

```python
def test_blend_geometry_midpoint_cap_binds_on_short_segment():
    # 90 deg corner, but one adjacent segment is short.
    # Half-segment rule: R_mid = 0.5 * min(L_prev, L_next) * cot(theta/2)
    #                         = 0.5 * 0.5 * 1.0 = 0.25 mm
    # R_tol should be much larger given the tolerance; verify R_mid wins.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 5.0  # absurdly loose tolerance so R_tol is the larger value
    L_short = 0.5
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=L_short,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,
        j_eff=1e30,
    )
    assert result is not None
    cos_half = math.sqrt(2) / 2
    sin_half = math.sqrt(2) / 2
    expected_R_mid = 0.5 * L_short * cos_half / sin_half  # = 0.25 (half-segment rule)
    assert result.R == pytest.approx(expected_R_mid, rel=1e-9)
    # d_consumed = R * tan(theta/2) = R for 90 deg. At R=0.25, d=0.25 (= L_short/2).
    assert result.d_consumed == pytest.approx(L_short * 0.5, rel=1e-9)
```

Around line 793 in `test_blend_from_moves_shaper_bounds_binding`, update the inline comment only (the R=0.5 assertion remains green because R_tol still binds there):

```python
    # corner_deviation is loose enough that R_tol is large; half-segment R_mid
    # caps at 0.5·min(L)·cot(45°) = 25 mm, so R_tol binds (R_tol << R_mid).
    # We still expect R ≈ 0.5mm if we set corner_deviation to produce that.
    # R_tol = corner_deviation · cos(45°)/(1-cos(45°)) = corner_dev · 2.414
    # Solving corner_deviation = 0.5/2.414 ≈ 0.207 mm:
    corner_dev = 0.5 / (math.sqrt(2)/2 / (1 - math.sqrt(2)/2))
```

- [ ] **Step 2: Run the fixtures to verify they fail**

Run: `pytest test/test_blendmath.py::test_blend_geometry_midpoint_cap_binds_on_short_segment -v`
Expected: FAIL — current `blendmath.py:145` still returns `R = 0.5`, test now expects `R = 0.25`.

- [ ] **Step 3: Apply the half-segment rule in `blendmath.py`**

In `klippy/blendmath.py`, around line 144–145, replace:

```python
    # Midpoint / adjacent-segment cap. cot(theta/2) = cos_half / sin_half.
    R_mid = min(L_prev, L_next) * cos_half / sin_half
```

with:

```python
    # Midpoint / adjacent-segment cap (half-segment rule): each blend claims
    # at most half the adjacent segment so two neighbouring corners meet at
    # most at the segment midpoint. cot(theta/2) = cos_half / sin_half.
    # d_consumed = R * tan(theta/2) <= 0.5 * min(L_prev, L_next) by construction.
    R_mid = 0.5 * min(L_prev, L_next) * cos_half / sin_half
```

- [ ] **Step 4: Update the algorithm docstring block at `blendmath.py` around line 100**

Find the comment block near the top of `blend_geometry` describing R_mid (currently reads `R_mid = min(L_prev, L_next) / tan(θ/2)` in the comments). Replace any such comment with `R_mid = 0.5 · min(L_prev, L_next) · cos(θ/2)/sin(θ/2)`. If the comment block doesn't exist verbatim (only the inline comment at :144 does), this sub-step is a no-op — the inline comment is authoritative.

- [ ] **Step 5: Run the full `test_blendmath.py` suite**

Run: `pytest test/test_blendmath.py -v`
Expected: all tests PASS. Any property test asserting `d_consumed <= L_prev + eps` still passes; the new bound `d_consumed <= 0.5 * min(L_prev, L_next)` is strictly tighter and implies the old one.

- [ ] **Step 6: Run the full `test_blendprepass.py` suite**

Run: `pytest test/test_blendprepass.py -v`
Expected: all 186 tests PASS (the prepass does not call `blend_geometry`; no impact).

- [ ] **Step 7: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: adopt LinuxCNC half-segment rule (R_mid *= 0.5)

R_mid = 0.5 * min(L_prev, L_next) / tan(theta/2). Each blend claims at
most half the adjacent segment so two neighbouring corners meet at
most at the segment midpoint. Removes the order-dependent greediness
of the prior full-length cap without changing tolerance-driven R_tol.

Required precondition for the gap invariant proof in
docs/superpowers/specs/2026-04-17-planner-integration-design.md.
References LinuxCNC src/emc/tp/blendmath.c:1031."
```

---

## Task 2: Add `CollinearCollapser.peek_buffered()`

**Files:**
- Modify: `klippy/blendprepass.py` (new `peek_buffered` method)
- Modify: `test/test_blendprepass.py` (new test for the method)

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendprepass.py`:

```python
def test_peek_buffered_returns_chain_copy():
    c = _collapser()
    th = c._toolhead
    assert c.peek_buffered() == []
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    buf = c.peek_buffered()
    assert buf == [m1, m2]
    # Mutation of the returned list must not affect internal state.
    buf.append("garbage")
    assert c._chain == [m1, m2]
    # Subsequent feed must still work.
    m3 = _FakeMove(th, (20, 0, 0, 1.0), (30, 0, 0, 1.5), speed=100.0)
    assert c.feed(m3) == []
    assert c._chain == [m1, m2, m3]
```

- [ ] **Step 2: Run the test to see it fail**

Run: `pytest test/test_blendprepass.py::test_peek_buffered_returns_chain_copy -v`
Expected: FAIL — `peek_buffered` is not defined.

- [ ] **Step 3: Implement `peek_buffered` on `CollinearCollapser`**

In `klippy/blendprepass.py`, inside the `CollinearCollapser` class (any place after `reset`, before `_flush_chain`), add:

```python
    def peek_buffered(self):
        """Read-only view of the currently buffered chain.

        Returns a fresh list copy so callers that mutate the result do not
        corrupt internal state. Part of the filter protocol consumed by
        BlendPipelineLookAheadQueue (sub-spec #4).
        """
        return list(self._chain)
```

- [ ] **Step 4: Run the test**

Run: `pytest test/test_blendprepass.py::test_peek_buffered_returns_chain_copy -v`
Expected: PASS.

- [ ] **Step 5: Run the full `test_blendprepass.py`**

Run: `pytest test/test_blendprepass.py -v`
Expected: all 187 tests PASS (186 pre-existing + 1 new).

- [ ] **Step 6: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: add CollinearCollapser.peek_buffered()

Read-only view of buffered chain for the filter protocol consumed by
the upcoming BlendPipelineLookAheadQueue. Returns a fresh list so
callers cannot corrupt internal state. Additive; existing adapter
remains on _chain direct access until Task 13."
```

---

## Task 3: `CornerBlender` scaffolding + empty test module

**Files:**
- Create: `klippy/blendplanner.py`
- Create: `test/test_blendplanner.py`

- [ ] **Step 1: Write `test/test_blendplanner.py` with scaffolding and a construction smoke test**

Create `test/test_blendplanner.py`:

```python
# test/test_blendplanner.py
import math
import random

import pytest

from klippy import blendplanner


class _FakeCheckMove:
    def __init__(self, exc=None):
        self.calls = []
        self._exc = exc

    def check_move(self, move):
        self.calls.append(move)
        if self._exc is not None:
            raise self._exc


class _FakeToolhead:
    def __init__(self, **overrides):
        self.max_velocity = overrides.get("max_velocity", 500.0)
        self.max_accel = overrides.get("max_accel", 10000.0)
        self.max_accel_to_decel = overrides.get("max_accel_to_decel", 10000.0)
        self.junction_deviation = overrides.get("junction_deviation", 0.01)
        self.corner_deviation = overrides.get("corner_deviation", 50e-3)
        self.kin = _FakeCheckMove()
        self.extruder = _FakeCheckMove()


class _FakeMove:
    """Reimplements klippy.toolhead.Move.__init__ without pulling pyserial."""

    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.junction_deviation = toolhead.junction_deviation
        self.timing_callbacks = []
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True
        axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.axes_d = axes_d
        move_d = math.sqrt(sum(d * d for d in axes_d[:3]))
        if move_d < 0.000000001:
            self.end_pos = (
                start_pos[0],
                start_pos[1],
                start_pos[2],
                end_pos[3],
            )
            axes_d[0] = axes_d[1] = axes_d[2] = 0.0
            move_d = abs(axes_d[3])
            inv_move_d = 1.0 / move_d if move_d else 0.0
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1.0 / move_d
        self.move_d = move_d
        self.axes_r = [d * inv_move_d for d in axes_d]
        self.min_move_t = move_d / velocity if velocity else 0.0
        self.max_start_v2 = 0.0
        self.max_cruise_v2 = velocity ** 2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.0
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel
        self.next_junction_v2 = 999999999.9

    def limit_speed(self, speed, accel):
        speed2 = speed ** 2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed if speed else 0.0
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def limit_next_junction_speed(self, speed):
        self.next_junction_v2 = min(self.next_junction_v2, speed ** 2)


def _blender(toolhead=None, max_chord_err=None):
    th = toolhead or _FakeToolhead()
    return blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=max_chord_err
    )


def test_construct_and_flush_empty():
    b = _blender()
    assert b.flush() == []
    assert b.peek_buffered() == []
    assert b.polyline_moves_emitted == 0
    assert b.blends_emitted == 0
```

- [ ] **Step 2: Run the test to see it fail**

Run: `pytest test/test_blendplanner.py::test_construct_and_flush_empty -v`
Expected: FAIL — `klippy.blendplanner` does not exist.

- [ ] **Step 3: Create `klippy/blendplanner.py` with a minimal skeleton**

Create `klippy/blendplanner.py`:

```python
# klippy/blendplanner.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Corner-blending planner integration.
# See docs/superpowers/specs/2026-04-17-planner-integration-design.md
from __future__ import annotations

import math

from . import blendmath


class CornerBlender:
    """Second filter stage in the blend pipeline.

    Buffers one move; on the next arriving move computes a tangent-arc
    blend and emits [trunc_prev, arc_polyline_moves...] while buffering
    the truncated-next-head as the new candidate prev.
    """

    def __init__(self, toolhead, *, move_cls, max_chord_err=None):
        self._toolhead = toolhead
        self._move_cls = move_cls
        self.max_chord_err = max_chord_err
        self._prev = None
        self.polyline_moves_emitted = 0
        self.blends_emitted = 0

    def feed(self, move):
        return []

    def flush(self):
        return []

    def reset(self):
        self._prev = None

    def peek_buffered(self):
        return [self._prev] if self._prev is not None else []
```

- [ ] **Step 4: Run the test**

Run: `pytest test/test_blendplanner.py::test_construct_and_flush_empty -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: module + test scaffolding"
```

---

## Task 4: `feed` steps 1 (non-kinematic breaks) + 2 (buffer first move); `flush`; `reset`

**Files:**
- Modify: `klippy/blendplanner.py`
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendplanner.py`:

```python
def test_feed_first_move_buffers():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    out = b.feed(m)
    assert out == []
    assert b._prev is m
    assert b.peek_buffered() == [m]


def test_flush_drains_buffered_prev():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m)
    out = b.flush()
    assert out == [m]
    assert b._prev is None


def test_reset_drops_buffered_prev():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m)
    b.reset()
    assert b._prev is None
    assert b.flush() == []


def test_feed_non_kinematic_flushes_and_passes():
    b = _blender()
    th = b._toolhead
    m_kin = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m_kin)
    # E-only: XYZ identical, E delta present
    eonly = _FakeMove(th, (10, 0, 0, 0.5), (10, 0, 0, 1.5), speed=100.0)
    assert eonly.is_kinematic_move is False
    out = b.feed(eonly)
    assert out == [m_kin, eonly]
    assert b._prev is None
```

- [ ] **Step 2: Run the tests to see them fail**

Run: `pytest test/test_blendplanner.py -v -k "first_move or flush_drains or reset_drops or non_kinematic"`
Expected: 4 FAIL — current `feed` returns `[]`; current `flush` returns `[]`.

- [ ] **Step 3: Implement `feed` steps 1 and 2 + `flush`**

In `klippy/blendplanner.py`, replace the bodies of `feed` and `flush`:

```python
    def feed(self, move):
        if not move.is_kinematic_move:
            return self.flush() + [move]
        if self._prev is None:
            self._prev = move
            return []
        # Blend steps 3–8 come in later tasks. For now, treat any second
        # kinematic move as a temporary passthrough so downstream tasks can
        # introduce gates one by one.
        emitted = [self._prev]
        self._prev = move
        return emitted

    def flush(self):
        if self._prev is None:
            return []
        emitted = [self._prev]
        self._prev = None
        return emitted
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendplanner.py -v`
Expected: all 5 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: feed step 1/2 + flush + reset + non-kin break"
```

---

## Task 5: `feed` step 4 — collinear passthrough (None return from `blend_from_moves`)

**Files:**
- Modify: `klippy/blendplanner.py`
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendplanner.py`:

```python
def test_feed_collinear_pair_passes_through_with_rebuffer():
    b = _blender()
    th = b._toolhead
    # Two exactly collinear moves along +X: blend_from_moves returns None.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    assert b.feed(m1) == []
    out = b.feed(m2)
    # Collinear: emit prev unchanged, buffer next. No velocity cap imposed.
    assert out == [m1]
    assert b._prev is m2
    assert m1.next_junction_v2 == 999999999.9  # unchanged
```

- [ ] **Step 2: Run the test**

Run: `pytest test/test_blendplanner.py::test_feed_collinear_pair_passes_through_with_rebuffer -v`
Expected: PASS — the current passthrough from Task 4 already gives the right behavior for this case. We include the test to pin the behavior before we introduce the U-turn branch (which partially overlaps this code path).

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: pin collinear-pair passthrough behavior"
```

---

## Task 6: `feed` step 5 — U-turn / degenerate (`v_cap == 0` or `R == 0`)

**Files:**
- Modify: `klippy/blendplanner.py`
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendplanner.py`:

```python
def test_feed_uturn_emits_prev_with_zero_next_junction():
    b = _blender()
    th = b._toolhead
    # 180° reversal: +X then -X
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (0, 0, 0, 1.0), speed=100.0)
    assert b.feed(m1) == []
    out = b.feed(m2)
    # U-turn: emit prev with limit_next_junction_speed(0); buffer next.
    assert out == [m1]
    assert m1.next_junction_v2 == 0.0
    assert b._prev is m2
```

- [ ] **Step 2: Run the test to see it fail**

Run: `pytest test/test_blendplanner.py::test_feed_uturn_emits_prev_with_zero_next_junction -v`
Expected: FAIL — current passthrough does not set `next_junction_v2 = 0`; assertion on m1.next_junction_v2 fails.

- [ ] **Step 3: Implement `feed` steps 3–5 (call `blend_from_moves`, handle None, handle v_cap=0 / R=0)**

In `klippy/blendplanner.py`, replace the `feed` body:

```python
    def feed(self, move):
        if not move.is_kinematic_move:
            return self.flush() + [move]
        if self._prev is None:
            self._prev = move
            return []
        arc = blendmath.blend_from_moves(
            self._prev, move,
            self._toolhead.corner_deviation,
            toolhead=self._toolhead,
        )
        if arc is None:
            # Collinear: prepass should have caught. Emit prev, buffer next.
            emitted = [self._prev]
            self._prev = move
            return emitted
        if arc.R == 0.0 or arc.v_cap == 0.0:
            # U-turn / degenerate: force a stop at the junction.
            self._prev.limit_next_junction_speed(0.0)
            emitted = [self._prev]
            self._prev = move
            return emitted
        # Blend steps 6–8 (arc emission) come in Task 8.
        emitted = [self._prev]
        self._prev = move
        return emitted
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendplanner.py -v`
Expected: all tests PASS. Specifically `test_feed_collinear_pair_passes_through_with_rebuffer` and `test_feed_uturn_emits_prev_with_zero_next_junction` both pass.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: U-turn handling (limit_next_junction_speed(0))"
```

---

## Task 7: `_copy_caller_state` helper

**Files:**
- Modify: `klippy/blendplanner.py`
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write failing tests for the helper**

Append to `test/test_blendplanner.py`:

```python
def _state_src_dst_pair():
    """Build a (src, dst) pair where src is a 'full-length' parent and dst a
    truncated child constructed via the Move ctor against the same toolhead."""
    th = _FakeToolhead()
    src = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 1.0), speed=200.0)
    # Simulate caller mutations on src:
    src.timing_callbacks.append(lambda t: None)
    src.next_junction_v2 = 42.0
    src.max_cruise_v2 = 150.0 ** 2
    src.junction_deviation = 0.05
    src.accel = 5000.0
    src.delta_v2 = 2.0 * src.move_d * src.accel
    src.smooth_delta_v2 = min(src.delta_v2, 2.0 * src.move_d * 2500.0)
    dst = _FakeMove(th, (0, 0, 0, 0), (4, 0, 0, 0.4), speed=200.0)
    return th, src, dst


def test_copy_caller_state_transfers_caller_intent_fields():
    th, src, dst = _state_src_dst_pair()
    blendplanner._copy_caller_state(src, dst)
    # Caller-intent fields pinned verbatim.
    assert dst.timing_callbacks == src.timing_callbacks
    assert dst.timing_callbacks is not src.timing_callbacks  # copy, not alias
    assert dst.next_junction_v2 == 42.0
    assert dst.max_cruise_v2 == 150.0 ** 2
    assert dst.junction_deviation == 0.05
    assert dst.accel == 5000.0


def test_copy_caller_state_recomputes_length_derived_fields():
    th, src, dst = _state_src_dst_pair()
    blendplanner._copy_caller_state(src, dst)
    # delta_v2 recomputed from NEW move_d (4 mm) and pinned accel (5000).
    assert dst.delta_v2 == pytest.approx(2.0 * 4.0 * 5000.0)
    # smooth_delta_v2 preserves the parent ratio; src had smooth/delta = 0.5
    # (max_accel_to_decel/accel = 2500/5000), so dst should follow.
    ratio = src.smooth_delta_v2 / src.delta_v2
    assert dst.smooth_delta_v2 == pytest.approx(
        min(dst.delta_v2, 2.0 * 4.0 * dst.accel * ratio)
    )
    # min_move_t = move_d / sqrt(max_cruise_v2) = 4 / 150 = 0.02667
    assert dst.min_move_t == pytest.approx(4.0 / 150.0)


def test_copy_caller_state_handles_zero_delta_v2():
    th, src, dst = _state_src_dst_pair()
    src.delta_v2 = 0.0
    src.smooth_delta_v2 = 0.0
    blendplanner._copy_caller_state(src, dst)
    # Falls back to ratio=1.0 when src.delta_v2 is zero; dst.smooth_delta_v2
    # collapses to dst.delta_v2 via the min().
    assert dst.smooth_delta_v2 == pytest.approx(dst.delta_v2)
```

- [ ] **Step 2: Run the tests to see them fail**

Run: `pytest test/test_blendplanner.py -v -k "copy_caller_state"`
Expected: 3 FAIL — `blendplanner._copy_caller_state` is not defined.

- [ ] **Step 3: Implement `_copy_caller_state`**

In `klippy/blendplanner.py`, add at module level before the `CornerBlender` class:

```python
def _copy_caller_state(src, dst):
    """Transfer caller-mutable Move state from src to the truncated dst.

    Pins caller-intent fields verbatim (timing_callbacks, next_junction_v2,
    max_cruise_v2, junction_deviation, accel) so that M204 / SET_VELOCITY_LIMIT
    / register_lookahead_callback mutations applied upstream to src survive
    the emit-time construction of dst. Recomputes length-derived fields
    (delta_v2, smooth_delta_v2, min_move_t) from dst's NEW move_d and the
    pinned accel.

    The accel pin is a direct assignment (not via dst.limit_speed) because
    limit_speed takes min(self.accel, accel); if an intervening M204 had
    lowered toolhead.max_accel between src construction and emit, Move.__init__'s
    snapshot of the new (lower) value would win over src.accel.
    """
    dst.timing_callbacks = list(src.timing_callbacks)
    dst.next_junction_v2 = src.next_junction_v2
    dst.max_cruise_v2 = src.max_cruise_v2
    dst.junction_deviation = src.junction_deviation
    dst.accel = src.accel
    dst.delta_v2 = 2.0 * dst.move_d * dst.accel
    ratio = src.smooth_delta_v2 / src.delta_v2 if src.delta_v2 > 0.0 else 1.0
    dst.smooth_delta_v2 = min(
        dst.delta_v2, 2.0 * dst.move_d * dst.accel * ratio
    )
    dst.min_move_t = dst.move_d / math.sqrt(dst.max_cruise_v2)
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendplanner.py -v -k "copy_caller_state"`
Expected: all 3 PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: _copy_caller_state helper

Pins caller-intent Move fields (timing_callbacks, next_junction_v2,
max_cruise_v2, junction_deviation, accel) from src to dst. Recomputes
length-derived fields (delta_v2, smooth_delta_v2, min_move_t) from
dst's new move_d and the pinned accel. Direct accel assignment avoids
the limit_speed min() leak that would let an M204-lowered
toolhead.max_accel override src.accel."
```

---

## Task 8: `_emit_arc` + `feed` steps 6–8 (core blend emission)

**Files:**
- Modify: `klippy/blendplanner.py`
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write failing test**

Append to `test/test_blendplanner.py`:

```python
def test_90deg_corner_emits_trunc_prev_plus_arc_polyline_and_buffers_next_head():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # Two 10mm moves meeting at a 90° corner at (10,0,0).
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    assert b.feed(m_prev) == []
    out = b.feed(m_next)
    # Emission: [trunc_prev, arc[0], ..., arc[N-1]]
    assert len(out) >= 2
    trunc_prev = out[0]
    arc_moves = out[1:]
    # trunc_prev shares start_pos with m_prev.
    assert trunc_prev.start_pos[:3] == m_prev.start_pos[:3]
    # trunc_prev ends before the vertex by arc.d_consumed along +X.
    # R_mid = 0.5 * min(10,10) * cot(45°) = 5. R_tol binds much smaller:
    # R_tol = 50e-3 * cos(45°)/(1-cos(45°)) ≈ 0.1207. So R = R_tol ≈ 0.1207,
    # d = R * tan(45°) ≈ 0.1207.
    d_expected = 50e-3 * (math.sqrt(2)/2) / (1 - math.sqrt(2)/2)
    assert trunc_prev.end_pos[0] == pytest.approx(10.0 - d_expected, rel=1e-6)
    assert trunc_prev.end_pos[1] == pytest.approx(0.0, abs=1e-9)
    # buffered next_trunc_head starts where the arc ends.
    assert b._prev is not None
    assert b._prev is not m_next
    nxt_head = b._prev
    assert nxt_head.start_pos[0] == pytest.approx(10.0, abs=1e-9)
    assert nxt_head.start_pos[1] == pytest.approx(d_expected, rel=1e-6)
    assert nxt_head.end_pos[:3] == (10.0, 10.0, 0.0)
    # Polyline points all lie on the arc within max_chord_err.
    # Arc center: m_prev.end_pos + R*n_hat where n_hat bisects inward.
    # At a 90° corner +X to +Y, center = vertex + R*(-1/sqrt2, 1/sqrt2) rotated;
    # simpler check: every arc_move endpoint must be within R + chord_err of center.
    # We compute center from arc.entry_pt + R in the direction (next-prev)/|...|.
    # For simplicity just verify that arc spans from near (10-d,0) to (10,d).
    first_pt = arc_moves[0].start_pos[:3]
    last_pt = arc_moves[-1].end_pos[:3]
    assert first_pt[0] == pytest.approx(10.0 - d_expected, rel=1e-6)
    assert last_pt[1] == pytest.approx(d_expected, rel=1e-6)
    # All arc moves share the same max_cruise_v2 (arc.v_cap^2 in this case).
    v_caps = [am.max_cruise_v2 for am in arc_moves]
    assert max(v_caps) - min(v_caps) < 1e-6
    # Instrumentation.
    assert b.blends_emitted == 1
    assert b.polyline_moves_emitted == len(arc_moves)
```

- [ ] **Step 2: Run the test to see it fail**

Run: `pytest test/test_blendplanner.py::test_90deg_corner_emits_trunc_prev_plus_arc_polyline_and_buffers_next_head -v`
Expected: FAIL — current `feed` step 6–8 stub just does the passthrough emission.

- [ ] **Step 3: Implement `_emit_arc` and rewire `feed` step 6–8**

In `klippy/blendplanner.py`, inside `CornerBlender`, add `_resolve_chord_err` and `_emit_arc`, and replace the stub at the end of `feed`:

```python
    def _resolve_chord_err(self):
        """Return the polyline chord tolerance to use for the current blend.

        If self.max_chord_err was set at construction time, that value wins.
        Otherwise auto-scale as max(20e-3, 0.2 * toolhead.corner_deviation).
        """
        if self.max_chord_err is not None:
            return self.max_chord_err
        return max(20e-3, 0.2 * self._toolhead.corner_deviation)

    def _emit_arc(self, prev, nxt, arc):
        """Construct [trunc_prev, arc_moves...] and the trunc_next_head.

        Returns (trunc_prev, arc_moves_list, trunc_next_head).
        """
        th = self._toolhead
        move_cls = self._move_cls

        prev_dir = prev.axes_r[:3]
        next_dir = nxt.axes_r[:3]
        vertex = prev.end_pos[:3]

        # --- 1. Truncated prev ---
        prev_cruise_v = math.sqrt(prev.max_cruise_v2)
        trunc_prev_end_xyz = tuple(
            vertex[i] - arc.d_consumed * prev_dir[i] for i in range(3)
        )
        # E carried proportional to the truncated fraction of prev.move_d.
        frac_prev = 1.0 - arc.d_consumed / prev.move_d
        trunc_prev_end_e = prev.start_pos[3] + frac_prev * prev.axes_d[3]
        trunc_prev_end = (
            trunc_prev_end_xyz[0], trunc_prev_end_xyz[1],
            trunc_prev_end_xyz[2], trunc_prev_end_e,
        )
        trunc_prev = move_cls(th, prev.start_pos, trunc_prev_end, prev_cruise_v)
        _copy_caller_state(prev, trunc_prev)

        # --- 2. Arc polyline ---
        chord_err = self._resolve_chord_err()
        polyline_local = blendmath.segment_arc(arc, chord_err)
        polyline_world = [
            (p[0] + vertex[0], p[1] + vertex[1], p[2] + vertex[2])
            for p in polyline_local
        ]
        points_4d = blendmath.interpolate_extruder(
            polyline_world, arc.d_consumed,
            prev.axes_r[3], nxt.axes_r[3],
        )
        # Offset the interpolate_extruder E (starts at 0) by trunc_prev_end_e
        # so each polyline point's absolute E continues the global count.
        points_4d = [
            (p[0], p[1], p[2], p[3] + trunc_prev_end_e) for p in points_4d
        ]
        arc_cap_v2 = min(prev.max_cruise_v2, nxt.max_cruise_v2, arc.v_cap ** 2)
        arc_cap_v = math.sqrt(arc_cap_v2)
        arc_accel = min(prev.accel, nxt.accel)
        arc_jd = min(prev.junction_deviation, nxt.junction_deviation)
        arc_moves = []
        for p0, p1 in zip(points_4d, points_4d[1:]):
            am = move_cls(th, p0, p1, arc_cap_v)
            am.max_cruise_v2 = arc_cap_v2
            am.junction_deviation = arc_jd
            am.limit_speed(arc_cap_v, arc_accel)
            # Cruise-through-arc: pin smooth_delta_v2 to delta_v2 so look-ahead
            # smoothing does not ramp gently at the arc boundaries.
            am.smooth_delta_v2 = am.delta_v2
            am.min_move_t = am.move_d / arc_cap_v
            arc_moves.append(am)

        # --- 3. Truncated next head ---
        trunc_next_head_start_xyz = tuple(
            vertex[i] + arc.d_consumed * next_dir[i] for i in range(3)
        )
        # E carry for the truncated-next-head: fraction of next.move_d after
        # the head is consumed.
        frac_next = 1.0 - arc.d_consumed / nxt.move_d
        trunc_next_head_start_e = nxt.end_pos[3] - frac_next * nxt.axes_d[3]
        trunc_next_head_start = (
            trunc_next_head_start_xyz[0], trunc_next_head_start_xyz[1],
            trunc_next_head_start_xyz[2], trunc_next_head_start_e,
        )
        next_cruise_v = math.sqrt(nxt.max_cruise_v2)
        trunc_next_head = move_cls(
            th, trunc_next_head_start, nxt.end_pos, next_cruise_v
        )
        _copy_caller_state(nxt, trunc_next_head)

        return trunc_prev, arc_moves, trunc_next_head
```

Then replace the trailing stub in `feed` (the `# Blend steps 6–8 ...` region) with:

```python
        trunc_prev, arc_moves, trunc_next_head = self._emit_arc(
            self._prev, move, arc
        )
        self._prev = trunc_next_head
        self.blends_emitted += 1
        self.polyline_moves_emitted += len(arc_moves)
        return [trunc_prev] + arc_moves
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendplanner.py -v`
Expected: all tests PASS including the new 90° emission test.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: _emit_arc + feed step 6-8 core blend emission

Splits a corner into [trunc_prev, arc_polyline_moves...] and buffers
trunc_next_head for the next feed. Uses half-segment-capped blend
geometry; auto-scales chord tolerance to max(20e-3, 0.2*corner_deviation);
pins arc moves' smooth_delta_v2 = delta_v2 (cruise-through-arc); carries
E through trunc_prev and trunc_next_head proportional to truncated
lengths and via interpolate_extruder across the polyline."
```

---

## Task 9: E conservation + half-segment consumption regression tests

**Files:**
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendplanner.py`:

```python
def test_e_conservation_through_blend():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    # Drain buffered trunc_next_head.
    out += b.flush()
    total_e = sum(am.axes_d[3] for am in out)
    expected = m_prev.axes_d[3] + m_next.axes_d[3]
    assert total_e == pytest.approx(expected, rel=1e-9, abs=1e-12)


def test_asymmetric_segments_half_segment_rule_caps_consumption():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # 60° corner. Short segment = 2mm, long = 10mm. With LOOSE tolerance so
    # R_tol >> R_mid and the midpoint cap binds.
    # R_mid = 0.5 * min(2, 10) * cot(30°) = 0.5 * 2 * sqrt(3) = sqrt(3)
    # d = R * tan(30°) = sqrt(3) * (1/sqrt(3)) = 1.0 (= L_short / 2)
    th.corner_deviation = 10.0  # absurdly loose so R_tol does not bind
    angle = math.radians(60.0)
    m_prev = _FakeMove(th, (0, 0, 0, 0), (2, 0, 0, 0.1), speed=100.0)
    # Rotate next direction by 60° from +X.
    next_end = (
        2 + 10 * math.cos(angle),
        0 + 10 * math.sin(angle),
        0, 0.6,
    )
    m_next = _FakeMove(th, (2, 0, 0, 0.1), next_end, speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    trunc_prev = out[0]
    # trunc_prev.move_d should equal 2 - 1 = 1 mm (half-segment consumption).
    assert trunc_prev.move_d == pytest.approx(1.0, rel=1e-6)
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendplanner.py -v -k "e_conservation or half_segment_rule_caps"`
Expected: both PASS (the `_emit_arc` implementation is correct; this task pins the behavior).

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: regression tests for E conservation and half-segment cap"
```

---

## Task 10: Aggregate `kin.check_move` and `extruder.check_move` on arc polyline

**Files:**
- Modify: `klippy/blendplanner.py` (add eager aggregate check)
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendplanner.py`:

```python
def test_aggregate_kin_check_move_fires_on_representative_arc_move():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    # The representative arc move was passed to kin.check_move exactly once.
    arc_moves = out[1:]
    assert len(th.kin.calls) == 1
    assert th.kin.calls[0] is arc_moves[0]


def test_aggregate_extruder_check_move_fires_when_extruding():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    b.feed(m_next)
    # Extruder check_move called once on the representative arc move (E delta
    # is non-zero because both prev and next extrude).
    assert len(th.extruder.calls) == 1


def test_aggregate_extruder_check_move_skipped_when_not_extruding():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # E coordinate identical across prev and next (travel moves).
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0), (10, 10, 0, 0), speed=100.0)
    b.feed(m_prev)
    b.feed(m_next)
    assert len(th.extruder.calls) == 0
```

- [ ] **Step 2: Run the tests to see them fail**

Run: `pytest test/test_blendplanner.py -v -k "aggregate"`
Expected: 3 FAIL — `_emit_arc` does not currently call `kin.check_move` or `extruder.check_move`.

- [ ] **Step 3: Add the aggregate check to `_emit_arc`**

In `klippy/blendplanner.py`, inside `_emit_arc`, just before the `return trunc_prev, arc_moves, trunc_next_head` line:

```python
        # Aggregate-safety re-check. check_move runs before lookahead.add_move
        # in ToolHead.move, so emitted arc-polyline Moves bypass it otherwise.
        # One representative is sufficient: all arc moves share accel, v_cap,
        # and per-mm E rate; spatially the polyline is localized near the
        # corner vertex so envelope checks evaluate at roughly the same
        # coordinates across all points.
        if arc_moves:
            representative = arc_moves[0]
            th.kin.check_move(representative)
            if representative.axes_d[3]:
                th.extruder.check_move(representative)
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendplanner.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: eager aggregate kin/extruder check_move on arc

One representative arc-polyline move is passed to kin.check_move, and
(if extruding) extruder.check_move. Catches extruder-max-velocity
violations on high-flow corners and envelope violations on tight
clearances that would otherwise bypass ToolHead.move validation
(which only fires before lookahead.add_move, not on synthesized moves).

Follows the blendprepass.py:131-134 precedent."
```

---

## Task 11: `smooth_delta_v2 == delta_v2` pin regression + speed continuity

**Files:**
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write tests**

Append to `test/test_blendplanner.py`:

```python
def test_arc_polyline_smooth_delta_v2_equals_delta_v2():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    arc_moves = out[1:]
    for am in arc_moves:
        assert am.smooth_delta_v2 == pytest.approx(am.delta_v2, rel=1e-12)


def test_arc_polyline_speed_continuity_1ppm():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    arc_moves = out[1:]
    v2s = [am.max_cruise_v2 for am in arc_moves]
    # All arc moves share the same cap to 1 ppm.
    assert (max(v2s) - min(v2s)) / max(v2s) < 1e-6
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendplanner.py -v -k "smooth_delta_v2 or speed_continuity"`
Expected: both PASS — the `_emit_arc` already pins `am.smooth_delta_v2 = am.delta_v2` and sets a shared `arc_cap_v2`.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: pin smooth_delta_v2 and speed-continuity regressions"
```

---

## Task 12: Seed-parameterized random property tests

**Files:**
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write the property test**

Append to `test/test_blendplanner.py`:

```python
def _random_unit_3d(rng):
    while True:
        v = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1))
        n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
        if n > 0.1:
            return (v[0] / n, v[1] / n, v[2] / n)


@pytest.mark.parametrize("seed", range(50))
def test_property_random_3d_corners(seed):
    rng = random.Random(seed)
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    th.corner_deviation = rng.uniform(20e-3, 200e-3)
    d_prev = _random_unit_3d(rng)
    d_next = _random_unit_3d(rng)
    # Skip pathological near-collinear / near-reversal samples so the test
    # exercises a real blend.
    dot = d_prev[0] * d_next[0] + d_prev[1] * d_next[1] + d_prev[2] * d_next[2]
    if abs(dot) > 0.95:
        pytest.skip("near-collinear or near-reversal")
    L_prev = rng.uniform(1.0, 20.0)
    L_next = rng.uniform(1.0, 20.0)
    vertex = (
        rng.uniform(10, 90),
        rng.uniform(10, 90),
        rng.uniform(5, 20),
    )
    start = tuple(vertex[i] - L_prev * d_prev[i] for i in range(3))
    end = tuple(vertex[i] + L_next * d_next[i] for i in range(3))
    # E coordinates picked so prev and next share approximate flow so the
    # extruder-boundary cap does not dominate the test.
    prev_e_start = 0.0
    prev_e_end = 0.05 * L_prev
    next_e_end = prev_e_end + 0.05 * L_next
    m_prev = _FakeMove(
        th, (start[0], start[1], start[2], prev_e_start),
        (vertex[0], vertex[1], vertex[2], prev_e_end),
        speed=100.0,
    )
    m_next = _FakeMove(
        th, (vertex[0], vertex[1], vertex[2], prev_e_end),
        (end[0], end[1], end[2], next_e_end),
        speed=100.0,
    )
    b.feed(m_prev)
    out = b.feed(m_next) + b.flush()
    # Invariant 1: E conservation.
    total_e = sum(am.axes_d[3] for am in out)
    expected_e = m_prev.axes_d[3] + m_next.axes_d[3]
    assert total_e == pytest.approx(expected_e, rel=1e-9, abs=1e-12)
    # Invariant 2: non-negative move_d on every emitted piece.
    for am in out:
        assert am.move_d >= -1e-12
    # Invariant 3: first emitted piece starts at m_prev.start_pos.
    assert out[0].start_pos[:3] == m_prev.start_pos[:3]
    # Invariant 4: last emitted piece ends at m_next.end_pos (within float noise).
    assert out[-1].end_pos[0] == pytest.approx(m_next.end_pos[0], abs=1e-9)
    assert out[-1].end_pos[1] == pytest.approx(m_next.end_pos[1], abs=1e-9)
    assert out[-1].end_pos[2] == pytest.approx(m_next.end_pos[2], abs=1e-9)
```

- [ ] **Step 2: Run the test**

Run: `pytest test/test_blendplanner.py::test_property_random_3d_corners -v`
Expected: 50 parametrized cases PASS (some may skip for near-collinear samples; that's fine).

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: seed-parameterized random 3D corner property tests

50 random corners per run. Invariants: E conservation (1 ppm), non-
negative move_d on every emitted piece, start/end position continuity
across the split. Uses random.Random(seed) for Kalico-idiomatic
determinism (no hypothesis)."
```

---

## Task 13: Rename `PrepassLookAheadQueue` → `BlendPipelineLookAheadQueue`, wire `CornerBlender`, stats patch, config parse (atomic; Pair B)

**Files:**
- Modify: `klippy/blendprepass.py` (rename class, change constructor, no-flush `get_last`, rebuild `flush`/`add_move`/`queue`)
- Modify: `klippy/toolhead.py:267-273` (construct blender, use new adapter signature, parse `corner_deviation`, stats patch)
- Modify: `test/test_blendprepass.py` (rename + rewrite adapter tests)
- Modify: `test/test_blendplanner.py` (add adapter-composition tests using the new class)

- [ ] **Step 1: Rewrite the adapter in `blendprepass.py`**

In `klippy/blendprepass.py`, replace the entire `PrepassLookAheadQueue` class (currently around lines 138-178) with:

```python
class BlendPipelineLookAheadQueue:
    """Generic ordered filter-chain adapter in front of a LookAheadQueue.

    Each filter exposes feed(move) -> list[Move], flush() -> list[Move],
    reset() -> None, peek_buffered() -> list[Move]. On add_move, the
    incoming Move is piped through every filter in order; the survivors
    reach the inner LookAheadQueue. On flush, a two-pass drain flows
    each filter's flush() output through later filters' feed() before
    delivering to the inner queue, then flush()es the inner queue.

    get_last() does NOT drain filters — it peeks via peek_buffered() so
    that callers mutating the returned Move (timing_callbacks,
    limit_next_junction_speed) do not force a premature un-blended
    emission. The emit-time path (_build_merged_move in the prepass,
    _emit_arc in the blender) transfers caller-mutated state onto the
    actually-queued Move so the mutation survives.
    """

    def __init__(self, filters, lookahead):
        self._filters = list(filters)
        self._lookahead = lookahead

    def add_move(self, move):
        acc = [move]
        for f in self._filters:
            acc = [out for m in acc for out in f.feed(m)]
        for m in acc:
            self._lookahead.add_move(m)

    def flush(self, lazy=False):
        acc = []
        for f in self._filters:
            # Pipe any already-drained moves from earlier filters through
            # this filter's feed, then append this filter's own flush.
            acc = [out for m in acc for out in f.feed(m)]
            acc += f.flush()
        for m in acc:
            self._lookahead.add_move(m)
        self._lookahead.flush(lazy=lazy)

    def reset(self):
        for f in self._filters:
            f.reset()
        self._lookahead.reset()

    def set_flush_time(self, flush_time):
        self._lookahead.set_flush_time(flush_time)

    def get_last(self):
        for f in reversed(self._filters):
            buf = f.peek_buffered()
            if buf:
                return buf[-1]
        return self._lookahead.get_last()

    @property
    def queue(self):
        result = []
        for f in self._filters:
            result += f.peek_buffered()
        result += list(self._lookahead.queue)
        return result
```

- [ ] **Step 2: Update `toolhead.py` to use the new adapter and parse `corner_deviation`**

In `klippy/toolhead.py`, around lines 267-273, replace the current prepass wiring:

```python
        from . import blendprepass
        inner_queue = LookAheadQueue(self)
        self.prepass = blendprepass.CollinearCollapser(self, move_cls=Move)
        self.lookahead = blendprepass.PrepassLookAheadQueue(
            self.prepass, inner_queue
        )
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
```

with:

```python
        from . import blendprepass, blendplanner
        inner_queue = LookAheadQueue(self)
        self.prepass = blendprepass.CollinearCollapser(self, move_cls=Move)
        self.blender = blendplanner.CornerBlender(self, move_cls=Move)
        self.lookahead = blendprepass.BlendPipelineLookAheadQueue(
            [self.prepass, self.blender], inner_queue
        )
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
```

In the same file, after the existing `max_accel` / `min_cruise_ratio` parsing (around line 277-290), add `corner_deviation` parse. Find the block that looks like:

```python
        self.max_velocity = config.getfloat("max_velocity", above=0.0)
        self.max_accel = config.getfloat("max_accel", above=0.0)
```

and add after the existing `max_accel` line:

```python
        self.corner_deviation = config.getfloat("corner_deviation", above=0.0)
```

Also append the new field to `orig_cfg` — find the existing block:

```python
        self.orig_cfg["max_velocity"] = self.max_velocity
        self.orig_cfg["max_accel"] = self.max_accel
```

and add:

```python
        self.orig_cfg["corner_deviation"] = self.corner_deviation
```

Then patch `ToolHead.stats` to append the instrumentation counters. Find the existing method (around line 712-726):

```python
    def stats(self, eventtime):
        max_queue_time = max(self.print_time, self.last_flush_time)
        for m in self.all_mcus:
            m.check_active(max_queue_time, eventtime)
        est_print_time = self.mcu.estimated_print_time(eventtime)
        self.clear_history_time = est_print_time - MOVE_HISTORY_EXPIRE
        buffer_time = self.print_time - est_print_time
        is_active = buffer_time > -60.0 or not self.special_queuing_state
        if self.special_queuing_state == "Drip":
            buffer_time = 0.0
        return is_active, "print_time=%.3f buffer_time=%.3f print_stall=%d" % (
            self.print_time,
            max(buffer_time, 0.0),
            self.print_stall,
        )
```

Replace the `return` statement with:

```python
        return is_active, (
            "print_time=%.3f buffer_time=%.3f print_stall=%d "
            "blend_moves=%d blend_corners=%d"
            % (
                self.print_time,
                max(buffer_time, 0.0),
                self.print_stall,
                self.blender.polyline_moves_emitted,
                self.blender.blends_emitted,
            )
        )
```

- [ ] **Step 3: Rewrite the adapter tests in `test_blendprepass.py`**

In `test/test_blendprepass.py`, replace all six `blendprepass.PrepassLookAheadQueue(c, inner)` call sites (lines 484, 499, 516, 529, 539, 558) with `blendprepass.BlendPipelineLookAheadQueue([c], inner)`. Then find `test_adapter_get_last_flushes_prepass_first` (around line 535) and replace its body to reflect the new no-flush semantic:

```python
def test_adapter_get_last_peeks_without_flushing_prepass():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue([c], inner)

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    # Before get_last, chain is buffered, inner queue empty.
    assert inner.queue == []
    last = adapter.get_last()
    # get_last peeks; the prepass chain is STILL buffered (not flushed).
    assert inner.queue == []
    assert c._chain == [m1, m2]
    # Returned move is the tail of the buffered chain.
    assert last is m2
```

- [ ] **Step 4: Add pipeline-composition test in `test_blendplanner.py`**

Append to `test/test_blendplanner.py`:

```python
class _FakeInnerQueue:
    def __init__(self):
        self.queue = []
        self.flush_calls = []
        self.reset_calls = 0
        self.set_flush_time_calls = []

    def add_move(self, move):
        self.queue.append(move)

    def flush(self, lazy=False):
        self.flush_calls.append(lazy)

    def reset(self):
        self.reset_calls += 1
        self.queue = []

    def set_flush_time(self, t):
        self.set_flush_time_calls.append(t)

    def get_last(self):
        return self.queue[-1] if self.queue else None


def test_pipeline_composition_prepass_then_blender():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    # 10 short collinear +X moves, then a 90° turn into 10 short +Y moves.
    pos = (0.0, 0.0, 0.0, 0.0)
    for i in range(10):
        nxt = (pos[0] + 1.0, pos[1], pos[2], pos[3] + 0.05)
        adapter.add_move(_FakeMove(th, pos, nxt, speed=100.0))
        pos = nxt
    for i in range(10):
        nxt = (pos[0], pos[1] + 1.0, pos[2], pos[3] + 0.05)
        adapter.add_move(_FakeMove(th, pos, nxt, speed=100.0))
        pos = nxt
    adapter.flush()
    # Prepass merged each side into a long move; blender produced one blend.
    assert blender.blends_emitted == 1
    assert blender.polyline_moves_emitted >= 2


def test_pipeline_adapter_get_last_returns_blender_prev_when_buffered():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    # Feed one move. Prepass buffers it; get_last returns from prepass.
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m)
    assert adapter.get_last() is m
    # Inner queue still empty — no flush side effect.
    assert inner.queue == []
```

- [ ] **Step 5: Run the full test suite**

Run: `pytest test/test_blendprepass.py test/test_blendplanner.py test/test_blendmath.py -v`
Expected: all tests PASS. Specifically the renamed `test_adapter_get_last_peeks_without_flushing_prepass` verifies the new semantic.

- [ ] **Step 6: Run the full repo test suite**

Run: `pytest test/ -v`
Expected: all tests PASS (any other test that imported `PrepassLookAheadQueue` directly would fail; the only in-tree import was in `toolhead.py`, now updated).

- [ ] **Step 7: Commit**

```bash
git add klippy/blendprepass.py klippy/toolhead.py test/test_blendprepass.py test/test_blendplanner.py
git commit -m "blendplanner: wire CornerBlender via BlendPipelineLookAheadQueue

Atomic rename + constructor change + toolhead wiring + test migration:
- klippy/blendprepass.py: rename PrepassLookAheadQueue to
  BlendPipelineLookAheadQueue. Constructor takes an ordered list of
  filters instead of a single prepass. get_last() peeks via
  peek_buffered() instead of flushing (no-forfeit semantic).
  Two-pass flush drains each filter through downstream filters.
- klippy/toolhead.py: parse corner_deviation from [printer] section
  (required; config.getfloat(above=0.0) without default raises on
  missing). Instantiate CornerBlender and wire via
  BlendPipelineLookAheadQueue([prepass, blender], inner). Append
  blend_moves / blend_corners counters to ToolHead.stats output.
- test/test_blendprepass.py: update six adapter test call sites to the
  new constructor form. Rewrite test_adapter_get_last_flushes_prepass_first
  (renamed _peeks_without_flushing_prepass) to verify the new semantic.
- test/test_blendplanner.py: add pipeline composition test and adapter
  get_last test that exercise the prepass+blender chain end-to-end."
```

---

## Task 14: `get_last` no-forfeit integration test + `SET_VELOCITY_LIMIT` mid-blend leak test

**Files:**
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendplanner.py`:

```python
def test_get_last_no_forfeit_callback_transfers_to_trunc_prev():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m_prev)
    # Flush prepass so it lands in the blender's buffered _prev.
    # (In production this happens when the prepass's chain breaks via gate.)
    for m in prepass.flush():
        blender.feed(m)
    # Now the blender holds _prev; caller attaches a callback + junction cap.
    last = adapter.get_last()
    assert last is m_prev  # blender._prev returned, no flush side-effect
    marker = []
    last.timing_callbacks.append(lambda t: marker.append(t))
    last.limit_next_junction_speed(50.0)
    # Feed the next move — should trigger a blend and transfer callback state.
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    adapter.add_move(m_next)
    # Inner queue now has [trunc_prev, arc[0], ..., arc[N-1]] from the blend.
    assert len(inner.queue) >= 2
    trunc_prev = inner.queue[0]
    assert trunc_prev is not m_prev  # new Move, not the original
    # Callback transferred onto trunc_prev.
    assert trunc_prev.timing_callbacks != []
    # limit_next_junction_speed was applied to m_prev (→ m_prev.next_junction_v2 = 50^2)
    # and transferred onto trunc_prev via _copy_caller_state.
    assert trunc_prev.next_junction_v2 == 50.0 ** 2


def test_set_velocity_limit_mid_blend_does_not_leak_lowered_accel():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    # Original accel snapshotted on m_prev: 10000 (th.max_accel at ctor).
    assert m_prev.accel == 10000.0
    b.feed(m_prev)
    # User issues an M204 that lowers accel.
    th.max_accel = 3000.0
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    assert m_next.accel == 3000.0
    out = b.feed(m_next)
    trunc_prev = out[0]
    # trunc_prev must pin parent's accel (10000), NOT the lowered toolhead
    # value. This is the critical anti-leak assertion — _copy_caller_state
    # uses direct assignment, not limit_speed.
    assert trunc_prev.accel == m_prev.accel  # 10000, not min(10000, 3000)
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendplanner.py -v -k "get_last_no_forfeit or set_velocity_limit_mid_blend"`
Expected: both PASS — the `_copy_caller_state` helper and the no-flush `get_last` adapter behavior together deliver this.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: regression tests for get_last no-forfeit + M204 leak"
```

---

## Task 15: Drip-mode regression test

**Files:**
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write the test**

Append to `test/test_blendplanner.py`:

```python
def test_drip_mode_single_move_emits_unchanged():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    # Mimic drip_move: one move arrives, then flush.
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m)
    adapter.flush()
    # Single move exits unblended (no corner to blend against).
    assert inner.queue == [m]
    assert blender.blends_emitted == 0
```

- [ ] **Step 2: Run the test**

Run: `pytest test/test_blendplanner.py::test_drip_mode_single_move_emits_unchanged -v`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: drip-mode single-move passthrough regression"
```

---

## Task 16: `peek_buffered` on CornerBlender + adapter `queue` property test

**Files:**
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write the tests**

Append to `test/test_blendplanner.py`:

```python
def test_blender_peek_buffered():
    b = _blender()
    th = b._toolhead
    assert b.peek_buffered() == []
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m)
    assert b.peek_buffered() == [m]
    # Peek must not mutate state.
    b.peek_buffered()
    assert b._prev is m


def test_adapter_queue_reports_blender_buffered_move():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m)
    # Prepass buffered the move; adapter.queue must reflect it.
    assert adapter.queue == [m]
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendplanner.py -v -k "peek_buffered or queue_reports"`
Expected: both PASS — `CornerBlender.peek_buffered` was implemented in Task 3; the adapter queue property was implemented in Task 13.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: peek_buffered and adapter queue visibility tests"
```

---

## Task 17: Full repo test sweep + self-review

**Files:**
- None (verification task)

- [ ] **Step 1: Run the full repo test suite**

Run: `pytest test/ -v`
Expected: all tests green. If any test references `PrepassLookAheadQueue` by name outside of the ones already updated in Task 13, fix the reference and re-run.

- [ ] **Step 2: Grep for any lingering references to the old class**

Run: `grep -rn "PrepassLookAheadQueue" klippy/ test/ docs/`
Expected: only `docs/superpowers/specs/2026-04-17-planner-integration-design.md` and `docs/superpowers/plans/` mention the old name in their historical-context paragraphs. No production or test code references remain.

- [ ] **Step 3: Verify the stats string renders as expected**

In a REPL or a small script, exercise `ToolHead.stats` manually by loading a minimal config. If manual verification is heavyweight (requires printer harness), skip and rely on the test that the stats formatting string has the correct percent-format placeholders.

- [ ] **Step 4: If all clean, commit a final sweep marker (no code changes)**

If `git status` reports any uncommitted changes (e.g. you fixed lingering references), commit them:

```bash
git add -A
git commit -m "blendplanner: post-integration cleanup (grep sweep)"
```

Otherwise skip the commit.

---

## Summary

17 tasks. Total estimated new code: ~350 LOC in `blendplanner.py`, ~600 LOC in `test_blendplanner.py`, ~50 LOC changes in `blendprepass.py`, ~15 LOC changes in `toolhead.py`, ~20 LOC updates in `test_blendprepass.py`, ~15 LOC change in `blendmath.py`, ~5 LOC updates in `test_blendmath.py`.

Post-Task-17 state: `CornerBlender` is live in the main `ToolHead` pipeline. Every non-collinear corner becomes a tangent arc; SCV and JD remain in place but are never binding at polyline junctions (tangent-by-construction short-circuit + per-vertex caps from sub-specs #1-#3). Stage 1 is functionally complete and ready for on-hardware validation.

Sub-spec #5 (SCV/JD removal) can then proceed cleanly: grep `square_corner_velocity` and `junction_deviation` and delete every reference; the blender is the only active cornering path.
