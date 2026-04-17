# Naive-CAM Collinearity Prepass Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `klippy/blendprepass.py` — a stream processor that consolidates XYZ-collinear short slicer moves into single long moves before they reach the lookahead queue, plus a transparent `PrepassLookAheadQueue` wrapper that preserves all existing `LookAheadQueue` call-site semantics.

**Architecture:** Pure-Python `CollinearCollapser` class buffers a chain of `Move` objects; gates reject on F / E-per-mm / perpendicular-deviation / projection-bounds violations; `_build_merged_move` constructs a single replacement Move preserving `timing_callbacks`, `next_junction_v2`, `max_cruise_v2`, `junction_deviation`, and re-running `check_move` post-merge. Integration via a wrapper queue so `ToolHead` constructor is the only non-module file touched.

**Tech Stack:** Python 3, pytest, Kalico motion pipeline (`klippy/toolhead.py`, `klippy/blendmath.py` for reused vector helpers).

**Spec:** `docs/superpowers/specs/2026-04-17-naive-cam-prepass-design.md`.

---

## File structure

- `klippy/blendprepass.py` — NEW — `CollinearCollapser` class, `PrepassLookAheadQueue` adapter, module constants.
- `test/test_blendprepass.py` — NEW — unit tests, `_FakeToolhead`, `_FakeMove` (reimplements `Move.__init__` / `Move.limit_speed` logic so tests run without importing `klippy.toolhead`, which pulls `pyserial`).
- `klippy/toolhead.py` — MODIFY — one-location change in `ToolHead.__init__` to instantiate the collapser and wrap the lookahead queue.

**Move-class injection:** `CollinearCollapser.__init__` takes a `move_cls` callable alongside `toolhead`, defaulting to toolhead-supplied. This lets tests pass `_FakeMove` (which replicates `Move.__init__` / `limit_speed` behavior) without importing `klippy.toolhead`. Production path: `ToolHead.__init__` passes `Move` explicitly.

---

## Task 1: Module skeleton + test scaffolding

**Files:**
- Create: `klippy/blendprepass.py`
- Create: `test/test_blendprepass.py`

- [ ] **Step 1: Write `test_blendprepass.py` scaffolding with a single import-and-construct test**

```python
# test/test_blendprepass.py
import math
import random

import pytest

from klippy import blendprepass


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


def _collapser(toolhead=None):
    th = toolhead or _FakeToolhead()
    return blendprepass.CollinearCollapser(th, move_cls=_FakeMove)


def test_construct_and_flush_empty():
    c = _collapser()
    assert c.flush() == []
```

- [ ] **Step 2: Write `blendprepass.py` skeleton**

```python
# klippy/blendprepass.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import math

from . import blendmath


class CollinearCollapser:
    """Naive-CAM collinearity prepass. See
    docs/superpowers/specs/2026-04-17-naive-cam-prepass-design.md for rationale.
    """

    def __init__(self, toolhead, move_cls):
        self._toolhead = toolhead
        self._move_cls = move_cls
        self._chain = []
        self.tolerance = 25e-3
        self.max_chain = 100
        self.epm_rel = 1e-2
        self.f_rel = 1e-6
        self.min_seg_len = 1e-9
        self.t_eps = 1e-9

    def feed(self, move):
        return []

    def flush(self):
        return []

    def reset(self):
        self._chain = []
```

- [ ] **Step 3: Run the test**

Run: `pytest test/test_blendprepass.py::test_construct_and_flush_empty -v`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: module + test scaffolding"
```

---

## Task 2: `feed()` passthroughs — zero-length and non-kinematic

**Files:**
- Modify: `klippy/blendprepass.py` (implement `feed` steps 1 and 2)
- Modify: `test/test_blendprepass.py` (add tests)

- [ ] **Step 1: Write failing tests for zero-length and non-kinematic passthrough**

Append to `test/test_blendprepass.py`:

```python
def test_feed_zero_length_move_passes_through():
    c = _collapser()
    th = c._toolhead
    # Construct a zero-length move directly; Move.__init__ flags it non-kinematic
    # but also gives it move_d=0 which is the step-1 branch we want to exercise.
    zero = _FakeMove(th, (0, 0, 0, 0), (0, 0, 0, 0), speed=100.0)
    assert zero.move_d == 0.0
    out = c.feed(zero)
    assert out == [zero]
    assert c._chain == []


def test_feed_non_kinematic_flushes_and_passes():
    c = _collapser()
    th = c._toolhead
    # Build a non-empty chain first
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m1)
    assert c._chain == [m1]
    # E-only move: XYZ identical, E delta present => is_kinematic_move=False
    eonly = _FakeMove(th, (10, 0, 0, 0.5), (10, 0, 0, 1.5), speed=100.0)
    assert eonly.is_kinematic_move is False
    out = c.feed(eonly)
    assert out == [m1, eonly]
    assert c._chain == []
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `pytest test/test_blendprepass.py -v -k "zero_length or non_kinematic"`
Expected: FAIL (both return `[]`).

- [ ] **Step 3: Implement `feed` steps 1 and 2**

Replace `feed` and add `_flush_chain` in `klippy/blendprepass.py`:

```python
    def feed(self, move):
        if move.move_d < self.min_seg_len:
            return [move]
        if not move.is_kinematic_move:
            return self._flush_chain() + [move]
        if not self._chain:
            self._chain = [move]
            return []
        # (gates / chain-cap come in later tasks)
        self._chain = [move]
        return []

    def flush(self):
        if not self._chain:
            return []
        return self._flush_chain()

    def _flush_chain(self):
        try:
            if len(self._chain) == 1:
                result = self._chain
            else:
                result = [self._build_merged_move(self._chain)]
        except Exception:
            logging.warning(
                "blendprepass: chain cleared after build error (len=%d)",
                len(self._chain),
            )
            raise
        finally:
            self._chain = []
        return result

    def _build_merged_move(self, chain):
        # Real implementation arrives in Task 5; placeholder raises so any
        # unexpected multi-move chain in earlier tasks is visible.
        raise NotImplementedError("merged move construction not yet implemented")
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all 3 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: feed passthroughs for zero-length and non-kinematic moves"
```

---

## Task 3: Chain bootstrap + singleton flush

**Files:**
- Modify: `test/test_blendprepass.py` (add tests)

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendprepass.py`:

```python
def test_first_kinematic_move_starts_chain():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    out = c.feed(m)
    assert out == []
    assert c._chain == [m]


def test_flush_singleton_returns_move_unchanged():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m)
    out = c.flush()
    assert out == [m]  # single-element chain: identity, not a built merge
    assert c._chain == []


def test_reset_discards_chain():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m)
    c.reset()
    assert c._chain == []
    assert c.flush() == []
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendprepass.py -v -k "first_kinematic or singleton or reset_discards"`
Expected: all 3 PASS (Task 2's `feed` already handles bootstrap; `_flush_chain` handles singletons; `reset` was written in Task 1).

- [ ] **Step 3: Commit**

```bash
git add test/test_blendprepass.py
git commit -m "blendprepass: tests for chain bootstrap, singleton flush, reset"
```

---

## Task 4: Gates (a) speed and (b) extrusion ratio

**Files:**
- Modify: `klippy/blendprepass.py` (implement `_merge_gate_passes` with gates a/b)
- Modify: `test/test_blendprepass.py` (add tests)

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendprepass.py`:

```python
def test_speed_change_breaks_chain():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    # Speed differs by 1% (> f_rel=1e-6)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=101.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    # Gate (a) rejects; chain flushes as singleton, m2 starts new chain.
    assert out == [m1]
    assert c._chain == [m2]


def test_flow_change_breaks_chain():
    c = _collapser()
    th = c._toolhead
    # Same speed, same direction; different E-per-mm (> epm_rel=1%)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.1), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_flow_within_tolerance_does_not_break():
    c = _collapser()
    th = c._toolhead
    # 0.5 mm E vs 0.5005 mm E over same 10 mm XYZ -> 0.1% diff (< epm_rel)
    # Collinearity is also satisfied (perfectly straight along +X)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0005), speed=100.0)
    # With gates a/b only (no gate c yet), this should accumulate.
    # After gate (c) lands (Task 5) this remains the behavior.
    out = c.feed(m1)
    assert out == []
    # Gate (a) passes: speeds equal. Gate (b) passes: 0.1% < 1%. No gate c yet
    # so the chain should still accept m2. We implement the acceptance in step 3.
    out = c.feed(m2)
    assert out == []
    assert c._chain == [m1, m2]
```

- [ ] **Step 2: Run the tests to see them fail**

Run: `pytest test/test_blendprepass.py -v -k "speed_change or flow_change or flow_within"`
Expected: the first two FAIL (current feed discards chain on every 2nd move), the third FAIL for the same reason.

- [ ] **Step 3: Implement gates (a) and (b) plus chain-accept branch**

Replace the `# (gates / chain-cap come in later tasks)` block plus the discard line in `feed`:

```python
    def feed(self, move):
        if move.move_d < self.min_seg_len:
            return [move]
        if not move.is_kinematic_move:
            return self._flush_chain() + [move]
        if not self._chain:
            self._chain = [move]
            return []
        if not self._merge_gate_passes(move):
            emitted = self._flush_chain()
            self._chain = [move]
            return emitted
        self._chain.append(move)
        return []

    def _merge_gate_passes(self, candidate):
        anchor = self._chain[0]
        # Gate (a): cruise velocity equality
        max_cv2 = max(candidate.max_cruise_v2, anchor.max_cruise_v2)
        if abs(candidate.max_cruise_v2 - anchor.max_cruise_v2) > self.f_rel * max_cv2:
            return False
        # Gate (b): E-per-XYZ-mm equality (signed; retract<->extrude reversal fails)
        ae = candidate.axes_r[3]
        be = anchor.axes_r[3]
        if abs(ae - be) > self.epm_rel * max(abs(ae), abs(be), 1e-9):
            return False
        # Gates (c) and (d) come in later tasks.
        return True
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: gates (a) speed and (b) extrusion-ratio equality"
```

---

## Task 5: Gate (c) perpendicular collinearity + merged-Move construction

**Files:**
- Modify: `klippy/blendprepass.py` (gate c + `_build_merged_move` basic)
- Modify: `test/test_blendprepass.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendprepass.py`:

```python
def test_two_collinear_moves_merge():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    out = c.flush()
    assert len(out) == 1
    merged = out[0]
    assert merged is not m1 and merged is not m2
    assert merged.start_pos == (0, 0, 0, 0)
    # Move ctor clamps E-component of end_pos; check x/y/z and E separately
    assert merged.end_pos[:3] == (20.0, 0.0, 0.0)
    assert merged.axes_d[3] == pytest.approx(1.0, abs=1e-12)
    assert merged.move_d == pytest.approx(20.0, abs=1e-12)


def test_non_collinear_moves_do_not_merge():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    # 1 mm perpendicular offset: well beyond 25 µm tolerance
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 1.0, 0, 1.0), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_within_tolerance_offset_merges():
    c = _collapser()
    th = c._toolhead
    # 20 µm perpendicular offset from the A-to-C chord — within 25 µm tolerance.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 20e-3, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 20e-3, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Chord A(0,0,0)->C(20,0,0). Midpoint B=(10, 20e-3, 0). Perpendicular
    # distance from B to chord = 20 µm.
    assert c.feed(m1) == []
    assert c.feed(m2) == []
    assert c._chain == [m1, m2]
```

- [ ] **Step 2: Run the tests to see failures**

Run: `pytest test/test_blendprepass.py -v -k "collinear or non_collinear or within_tolerance"`
Expected: all 3 FAIL (gate c not implemented; `_build_merged_move` still raises NotImplementedError).

- [ ] **Step 3: Implement gate (c) and a basic `_build_merged_move`**

In `klippy/blendprepass.py`, extend `_merge_gate_passes`:

```python
    def _merge_gate_passes(self, candidate):
        anchor = self._chain[0]
        # Gate (a): cruise velocity equality
        max_cv2 = max(candidate.max_cruise_v2, anchor.max_cruise_v2)
        if abs(candidate.max_cruise_v2 - anchor.max_cruise_v2) > self.f_rel * max_cv2:
            return False
        # Gate (b): E-per-XYZ-mm equality (signed; retract<->extrude reversal fails)
        ae = candidate.axes_r[3]
        be = anchor.axes_r[3]
        if abs(ae - be) > self.epm_rel * max(abs(ae), abs(be), 1e-9):
            return False
        # Gate (c): perpendicular deviation of every buffered intermediate endpoint
        # from the anchor-to-candidate chord stays within tolerance.
        A = anchor.start_pos[:3]
        B = candidate.end_pos[:3]
        AB = blendmath.vsub(B, A)
        ab_len = blendmath.vnorm(AB)
        if ab_len < self.min_seg_len:
            return False
        for p_move in self._chain:
            P = p_move.end_pos[:3]
            AP = blendmath.vsub(P, A)
            perp_dist = blendmath.vnorm(blendmath.vcross(AP, AB)) / ab_len
            if perp_dist > self.tolerance:
                return False
        return True
```

Replace the `_build_merged_move` placeholder with:

```python
    def _build_merged_move(self, chain):
        start_pos = chain[0].start_pos
        end_pos = chain[-1].end_pos
        cruise_v = math.sqrt(chain[0].max_cruise_v2)
        merged = self._move_cls(self._toolhead, start_pos, end_pos, cruise_v)
        # Further preservation (junction_deviation, next_junction_v2,
        # timing_callbacks, post-merge check_move) lands in Tasks 8-10.
        return merged
```

- [ ] **Step 4: Run tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: gate (c) perpendicular collinearity + basic merged-Move build"
```

---

## Task 6: Gate (d) projection bounds + eps tolerance

**Files:**
- Modify: `klippy/blendprepass.py`
- Modify: `test/test_blendprepass.py`

- [ ] **Step 1: Write failing tests**

Append:

```python
def test_uturn_rejected_by_projection_bounds():
    c = _collapser()
    th = c._toolhead
    # A=(0,0,0) -> B=(10,0,0), candidate ends at (0,0,0): AB length 0 -> rejected.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (0, 0, 0, 1.0), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_overshoot_retrace_rejected():
    c = _collapser()
    th = c._toolhead
    # Anchor A=(0,0,0); chain moves to B=(12,0,0) then candidate to C=(10,0,0).
    # Projection t of B onto AC chord = 12/10 = 1.2 -> out of [0,1], reject.
    m1 = _FakeMove(th, (0, 0, 0, 0), (12, 0, 0, 0.6), speed=100.0)
    m2 = _FakeMove(th, (12, 0, 0, 0.6), (10, 0, 0, 0.5), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_legitimate_extension_passes_projection():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    assert c.feed(m1) == []
    assert c.feed(m2) == []
    # Both buffered; gate (d) allowed the extension.
    assert len(c._chain) == 2
```

- [ ] **Step 2: Run the tests to see failures**

Run: `pytest test/test_blendprepass.py -v -k "uturn or overshoot or legitimate_extension"`
Expected: `uturn` passes (gate c already rejects |AB|=0); `overshoot` FAILS (gate c alone accepts, because B is on the chord line); `legitimate_extension` PASSES.

- [ ] **Step 3: Extend `_merge_gate_passes` with gate (d)**

Append inside `_merge_gate_passes`, just before `return True`:

```python
        # Gate (d): projection bounds — every intermediate endpoint must lie
        # on the AB segment interior (0 <= t <= 1, with eps slack for float noise).
        ab_dot_ab = blendmath.vdot(AB, AB)
        for p_move in self._chain:
            P = p_move.end_pos[:3]
            AP = blendmath.vsub(P, A)
            t = blendmath.vdot(AP, AB) / ab_dot_ab
            if not (-self.t_eps <= t <= 1.0 + self.t_eps):
                return False
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: gate (d) projection bounds for U-turn / overshoot rejection"
```

---

## Task 7: Chain cap at 100

**Files:**
- Modify: `klippy/blendprepass.py`
- Modify: `test/test_blendprepass.py`

- [ ] **Step 1: Write failing tests**

Append:

```python
def _build_collinear_chain(toolhead, n, seg_len=1.0, e_per_mm=0.05, speed=100.0):
    moves = []
    for i in range(n):
        start = (i * seg_len, 0, 0, i * seg_len * e_per_mm)
        end = ((i + 1) * seg_len, 0, 0, (i + 1) * seg_len * e_per_mm)
        moves.append(_FakeMove(toolhead, start, end, speed=speed))
    return moves


def test_chain_cap_flushes_at_max():
    c = _collapser()
    th = c._toolhead
    moves = _build_collinear_chain(th, c.max_chain + 1)
    for m in moves[:-1]:
        assert c.feed(m) == []
    assert len(c._chain) == c.max_chain
    out = c.feed(moves[-1])
    assert len(out) == 1  # merged chain[:100] emitted
    merged = out[0]
    assert merged.start_pos == (0, 0, 0, 0)
    # end_pos x equals 100 * seg_len = 100.0
    assert merged.end_pos[:3] == pytest.approx((100.0, 0.0, 0.0), abs=1e-9)
    # New chain started with the 101st move:
    assert c._chain == [moves[-1]]
```

- [ ] **Step 2: Run the test to see it fail**

Run: `pytest test/test_blendprepass.py -v -k "chain_cap"`
Expected: FAIL (chain grows unbounded; no cap check in `feed`).

- [ ] **Step 3: Add cap enforcement to `feed`**

Replace `feed`:

```python
    def feed(self, move):
        if move.move_d < self.min_seg_len:
            return [move]
        if not move.is_kinematic_move:
            return self._flush_chain() + [move]
        if not self._chain:
            self._chain = [move]
            return []
        if len(self._chain) >= self.max_chain:
            emitted = self._flush_chain()
            self._chain = [move]
            return emitted
        if not self._merge_gate_passes(move):
            emitted = self._flush_chain()
            self._chain = [move]
            return emitted
        self._chain.append(move)
        return []
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: chain cap flush at 100 moves"
```

---

## Task 8: Merged-Move preservation — junction_deviation, max_cruise_v2, accel-floor

**Files:**
- Modify: `klippy/blendprepass.py` (`_build_merged_move` preservation)
- Modify: `test/test_blendprepass.py`

- [ ] **Step 1: Write failing tests**

Append:

```python
def test_merged_pins_max_cruise_v2_to_chain_head():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1); c.feed(m2)
    # Simulate SET_VELOCITY_LIMIT: toolhead raises max_velocity mid-chain.
    th.max_velocity = 1000.0
    out = c.flush()
    merged = out[0]
    # Without pinning, Move.__init__ would clamp to min(100, 1000)=100, v2=1e4;
    # with pinning, we keep chain[0].max_cruise_v2 (= 100**2 = 10000). Verify the
    # pin doesn't drift even when toolhead.max_velocity changed.
    assert merged.max_cruise_v2 == pytest.approx(m1.max_cruise_v2, rel=1e-12)


def test_merged_pins_junction_deviation_to_chain_head():
    c = _collapser()
    th = c._toolhead
    th.junction_deviation = 0.005
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    th.junction_deviation = 0.02  # SET_VELOCITY_LIMIT between moves
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1); c.feed(m2)
    th.junction_deviation = 0.05  # change again before merge
    out = c.flush()
    merged = out[0]
    assert merged.junction_deviation == pytest.approx(0.005, rel=1e-12)


def test_merged_preserves_minimum_accel_across_chain():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Simulate kinematics having applied limit_speed to m2.
    m2.limit_speed(100.0, 3000.0)
    assert m2.accel == 3000.0
    c.feed(m1); c.feed(m2)
    out = c.flush()
    merged = out[0]
    assert merged.accel == pytest.approx(3000.0, rel=1e-12)
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendprepass.py -v -k "pins_max_cruise or pins_junction or minimum_accel"`
Expected: FAIL (basic `_build_merged_move` doesn't pin any of these).

- [ ] **Step 3: Extend `_build_merged_move`**

Replace `_build_merged_move`:

```python
    def _build_merged_move(self, chain):
        start_pos = chain[0].start_pos
        end_pos = chain[-1].end_pos
        cruise_v = math.sqrt(chain[0].max_cruise_v2)
        merged = self._move_cls(self._toolhead, start_pos, end_pos, cruise_v)
        # Pin head-of-chain values so SET_VELOCITY_LIMIT / M204 mid-chain does
        # not leak into the merged Move via Move.__init__'s toolhead snapshot.
        merged.max_cruise_v2 = chain[0].max_cruise_v2
        merged.junction_deviation = chain[0].junction_deviation
        # Narrowest accel observed (may have been lowered by a constituent's
        # kin.check_move via limit_speed).
        merged.limit_speed(cruise_v, min(m.accel for m in chain))
        return merged
```

- [ ] **Step 4: Run tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: pin junction_deviation, max_cruise_v2, accel floor on merge"
```

---

## Task 9: Merged-Move preservation — `next_junction_v2` and `timing_callbacks`

**Files:**
- Modify: `klippy/blendprepass.py`
- Modify: `test/test_blendprepass.py`

- [ ] **Step 1: Write failing tests**

Append:

```python
def test_merged_preserves_next_junction_v2_from_chain_tail():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    m2.next_junction_v2 = 12345.0  # limit_next_junction_speed was called on tail
    c.feed(m1); c.feed(m2)
    out = c.flush()
    assert out[0].next_junction_v2 == pytest.approx(12345.0, rel=1e-12)


def test_merged_concatenates_timing_callbacks():
    c = _collapser()
    th = c._toolhead
    cb1 = lambda t: None
    cb2 = lambda t: None
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Under flush-on-get_last, callbacks should only ever land on chain[-1].
    # Defense in depth: also preserve if an earlier constituent carried them.
    m1.timing_callbacks.append(cb1)
    m2.timing_callbacks.append(cb2)
    c.feed(m1); c.feed(m2)
    out = c.flush()
    assert out[0].timing_callbacks == [cb1, cb2]
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendprepass.py -v -k "next_junction_v2 or timing_callbacks"`
Expected: FAIL.

- [ ] **Step 3: Extend `_build_merged_move`**

Replace `_build_merged_move`:

```python
    def _build_merged_move(self, chain):
        start_pos = chain[0].start_pos
        end_pos = chain[-1].end_pos
        cruise_v = math.sqrt(chain[0].max_cruise_v2)
        merged = self._move_cls(self._toolhead, start_pos, end_pos, cruise_v)
        merged.max_cruise_v2 = chain[0].max_cruise_v2
        merged.junction_deviation = chain[0].junction_deviation
        merged.limit_speed(cruise_v, min(m.accel for m in chain))
        # Preserve chain tail's next-junction cap and all constituent callbacks.
        merged.next_junction_v2 = chain[-1].next_junction_v2
        merged.timing_callbacks = [
            cb for m in chain for cb in m.timing_callbacks
        ]
        return merged
```

- [ ] **Step 4: Run tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: preserve next_junction_v2 and timing_callbacks through merge"
```

---

## Task 10: Post-merge `check_move` re-run

**Files:**
- Modify: `klippy/blendprepass.py`
- Modify: `test/test_blendprepass.py`

- [ ] **Step 1: Write failing tests**

Append:

```python
def test_post_merge_kin_check_move_runs():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1); c.feed(m2)
    out = c.flush()
    # Exactly one post-merge check: on the merged Move itself.
    assert len(th.kin.calls) == 1
    assert th.kin.calls[0] is out[0]


def test_post_merge_extruder_check_runs_only_when_e_delta_nonzero():
    c = _collapser()
    th = c._toolhead
    # Pure-travel chain: axes_d[3] == 0 on both, merged axes_d[3] == 0
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0), (20, 0, 0, 0), speed=100.0)
    c.feed(m1); c.feed(m2)
    c.flush()
    assert th.extruder.calls == []

    # With extrusion:
    th2 = _FakeToolhead()
    c2 = blendprepass.CollinearCollapser(th2, move_cls=_FakeMove)
    m3 = _FakeMove(th2, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m4 = _FakeMove(th2, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c2.feed(m3); c2.feed(m4)
    c2.flush()
    assert len(th2.extruder.calls) == 1


def test_post_merge_check_skipped_for_singleton_chain():
    # Singletons skip _build_merged_move entirely (pass through identity),
    # so no post-merge check fires. This preserves per-move check_move behavior.
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m)
    c.flush()
    assert th.kin.calls == []
    assert th.extruder.calls == []
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendprepass.py -v -k "post_merge"`
Expected: FAIL.

- [ ] **Step 3: Extend `_build_merged_move`**

Replace `_build_merged_move`:

```python
    def _build_merged_move(self, chain):
        start_pos = chain[0].start_pos
        end_pos = chain[-1].end_pos
        cruise_v = math.sqrt(chain[0].max_cruise_v2)
        merged = self._move_cls(self._toolhead, start_pos, end_pos, cruise_v)
        merged.max_cruise_v2 = chain[0].max_cruise_v2
        merged.junction_deviation = chain[0].junction_deviation
        merged.limit_speed(cruise_v, min(m.accel for m in chain))
        merged.next_junction_v2 = chain[-1].next_junction_v2
        merged.timing_callbacks = [
            cb for m in chain for cb in m.timing_callbacks
        ]
        # Aggregate-safety re-check. Per-constituent kin.check_move already
        # validated each segment; this catches aggregate limits such as
        # max_extrude_only_distance that can only be evaluated on the merge.
        if merged.is_kinematic_move:
            self._toolhead.kin.check_move(merged)
        if merged.axes_d[3]:
            self._toolhead.extruder.check_move(merged)
        return merged
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: re-run kin/extruder check_move on merged Move"
```

---

## Task 11: Exception safety

**Files:**
- Modify: `test/test_blendprepass.py`

(Implementation of `try/finally` already shipped in Task 2 — this task adds coverage.)

- [ ] **Step 1: Write failing tests**

Append:

```python
class _RaisingKin:
    def check_move(self, move):
        raise RuntimeError("kin limit violation")


def test_exception_in_merged_check_clears_chain(caplog):
    th = _FakeToolhead()
    th.kin = _RaisingKin()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1); c.feed(m2)
    with caplog.at_level("WARNING"):
        with pytest.raises(RuntimeError, match="kin limit violation"):
            c.flush()
    assert c._chain == []
    assert any("blendprepass: chain cleared" in r.message for r in caplog.records)


def test_feed_after_exception_starts_clean():
    th = _FakeToolhead()
    th.kin = _RaisingKin()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1); c.feed(m2)
    with pytest.raises(RuntimeError):
        c.flush()
    # After exception, chain is empty; next feed starts a fresh chain of size 1.
    m3 = _FakeMove(th, (20, 0, 0, 1.0), (30, 0, 0, 1.5), speed=100.0)
    assert c.feed(m3) == []
    assert c._chain == [m3]
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendprepass.py -v -k "exception"`
Expected: PASS (try/finally was written in Task 2).

- [ ] **Step 3: Commit**

```bash
git add test/test_blendprepass.py
git commit -m "blendprepass: coverage for exception-path chain reset"
```

---

## Task 12: `PrepassLookAheadQueue` adapter

**Files:**
- Modify: `klippy/blendprepass.py` (append adapter class)
- Modify: `test/test_blendprepass.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendprepass.py`:

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


def test_adapter_add_move_routes_through_prepass():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.PrepassLookAheadQueue(c, inner)

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    # Buffered in prepass; inner queue still empty.
    assert inner.queue == []
    assert c._chain == [m1, m2]


def test_adapter_flush_drains_and_forwards_lazy_flag():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.PrepassLookAheadQueue(c, inner)

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    adapter.flush(lazy=True)
    # Chain emitted into inner queue, then inner flushed with lazy=True.
    assert len(inner.queue) == 1
    assert inner.flush_calls == [True]
    assert c._chain == []


def test_adapter_reset_discards_chain_and_resets_inner():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.PrepassLookAheadQueue(c, inner)

    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m)
    adapter.reset()
    assert c._chain == []
    assert inner.reset_calls == 1


def test_adapter_set_flush_time_passes_through():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.PrepassLookAheadQueue(c, inner)

    adapter.set_flush_time(2.0)
    assert inner.set_flush_time_calls == [2.0]


def test_adapter_get_last_flushes_prepass_first():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.PrepassLookAheadQueue(c, inner)

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    # Before get_last, chain is buffered, inner queue empty.
    assert inner.queue == []
    last = adapter.get_last()
    # get_last drained the prepass; the inner queue now has the merged move.
    assert len(inner.queue) == 1
    assert last is inner.queue[0]
    assert c._chain == []


def test_adapter_queue_property_reports_buffered_moves():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.PrepassLookAheadQueue(c, inner)

    # Empty state: queue is empty.
    assert not adapter.queue

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    # Buffered in prepass but inner still empty — queue property reflects
    # buffered pending work.
    assert adapter.queue
    assert len(adapter.queue) == 2
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendprepass.py -v -k "adapter"`
Expected: FAIL (no adapter class yet).

- [ ] **Step 3: Implement `PrepassLookAheadQueue`**

Append to `klippy/blendprepass.py`:

```python
class PrepassLookAheadQueue:
    """Transparent wrapper that drains a CollinearCollapser on every flush,
    get_last, or queue access. ToolHead uses this in place of a raw
    LookAheadQueue so no per-call-site prepass handling is required.
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
        # Drain prepass first so callers attaching timing_callbacks /
        # next_junction_v2 via the returned Move land on the canonical
        # queued move, not a transient chain constituent.
        for m in self._prepass.flush():
            self._lookahead.add_move(m)
        return self._lookahead.get_last()

    @property
    def queue(self):
        # check_busy and similar callers test emptiness/length; buffered
        # chain counts as pending.
        if self._prepass._chain:
            return list(self._prepass._chain) + list(self._lookahead.queue)
        return self._lookahead.queue
```

- [ ] **Step 4: Run the tests**

Run: `pytest test/test_blendprepass.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendprepass.py test/test_blendprepass.py
git commit -m "blendprepass: PrepassLookAheadQueue transparent adapter"
```

---

## Task 13: Randomized seed-parameterized tests

**Files:**
- Modify: `test/test_blendprepass.py`

- [ ] **Step 1: Write tests**

Append:

```python
@pytest.mark.parametrize("seed", range(50))
def test_random_collinear_chain_merges(seed):
    rng = random.Random(seed)
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    # Random unit direction in a plane (z=0 for simplicity of the noise model)
    theta = rng.uniform(0.0, 2 * math.pi)
    ux, uy = math.cos(theta), math.sin(theta)
    # Perpendicular direction in the plane
    px, py = -uy, ux
    n = rng.randint(2, 100)
    anchor = (0.0, 0.0, 0.0, 0.0)
    cursor = anchor
    for _ in range(n):
        seg_len = rng.uniform(0.01, 10.0)
        noise = rng.uniform(-20e-6, 20e-6)  # 20 µm well under 25 µm tolerance
        nx = cursor[0] + ux * seg_len + px * noise
        ny = cursor[1] + uy * seg_len + py * noise
        e_delta = seg_len * 0.05
        end = (nx, ny, 0.0, cursor[3] + e_delta)
        m = _FakeMove(th, cursor, end, speed=100.0)
        c.feed(m)
        cursor = end
    out = c.flush()
    assert len(out) == 1, f"seed {seed}: expected 1 merged move, got {len(out)}"


@pytest.mark.parametrize("seed", range(50))
def test_random_chain_splits_at_violation(seed):
    rng = random.Random(seed)
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    n_before = rng.randint(2, 50)
    moves = _build_collinear_chain(th, n_before)
    for m in moves:
        c.feed(m)
    # Now an offset-violating move: 50 µm perpendicular offset > 25 µm tolerance.
    last_end = moves[-1].end_pos
    violator_end = (last_end[0] + 1.0, last_end[1] + 50e-3, 0.0, last_end[3] + 0.05)
    violator = _FakeMove(th, last_end, violator_end, speed=100.0)
    out = c.feed(violator)
    # First output: the merged prior chain.
    assert len(out) == 1
    # Violator started a fresh chain.
    assert c._chain == [violator]


@pytest.mark.parametrize("seed", range(50))
def test_total_displacement_preserved(seed):
    rng = random.Random(seed)
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    n = rng.randint(2, 50)
    moves = _build_collinear_chain(th, n)
    for m in moves:
        c.feed(m)
    out = c.flush()
    merged = out[0]
    for i in range(4):
        expected = sum(m.axes_d[i] for m in moves)
        assert merged.axes_d[i] == pytest.approx(expected, abs=1e-9)
```

- [ ] **Step 2: Run the tests**

Run: `pytest test/test_blendprepass.py -v -k "random"`
Expected: all 150 PASS.

- [ ] **Step 3: Run full suite**

Run: `pytest test/test_blendprepass.py -v`
Expected: all PASS.

- [ ] **Step 4: Commit**

```bash
git add test/test_blendprepass.py
git commit -m "blendprepass: seed-parameterized randomized collinear-chain tests"
```

---

## Task 14: ToolHead integration

**Files:**
- Modify: `klippy/toolhead.py`

- [ ] **Step 1: Inspect current `ToolHead.__init__` around the lookahead wiring**

Run: `grep -n "self.lookahead\|LookAheadQueue" klippy/toolhead.py`
Expected output includes:
```
147:    def __init__(self, toolhead):
267:        self.lookahead = LookAheadQueue(self)
268:        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
```
(Line numbers may differ slightly; anchor by content.)

- [ ] **Step 2: Apply the change**

In `klippy/toolhead.py`, find the block:

```python
        self.lookahead = LookAheadQueue(self)
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
```

Replace with:

```python
        from . import blendprepass
        inner_queue = LookAheadQueue(self)
        self.prepass = blendprepass.CollinearCollapser(self, move_cls=Move)
        self.lookahead = blendprepass.PrepassLookAheadQueue(
            self.prepass, inner_queue)
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
```

- [ ] **Step 3: Syntax-check the modified file**

Run: `python3 -c "import ast; ast.parse(open('klippy/toolhead.py').read())"`
Expected: no output (valid Python).

- [ ] **Step 4: Run the full Kalico test suite**

Run: `pytest test/ -v`
Expected: all tests PASS, including the existing `test_blendmath.py`, `test_blendshaper.py`, and the new `test_blendprepass.py`. No regressions.

- [ ] **Step 5: Commit**

```bash
git add klippy/toolhead.py
git commit -m "toolhead: wire CollinearCollapser + PrepassLookAheadQueue into ToolHead.__init__"
```

---

## Notes for the implementer

- Kalico tests run from repo root: `pytest test/test_blendprepass.py -v`.
- Do **not** add `hypothesis` to deps — use `@pytest.mark.parametrize("seed", range(N))` + `random.Random(seed)` (matches `test_blendmath.py`).
- `blendmath.vsub`, `vdot`, `vcross`, `vnorm` are reused for gate math; `from . import blendmath` at the top of `blendprepass.py`.
- `_FakeMove` must faithfully replicate `Move.__init__` and `limit_speed` because Kalico's toolhead module pulls `pyserial` and can't be imported in a pure-python test env. The `move_cls` injection keeps production and tests symmetric.
- The spec is authoritative; if a test conflicts with the spec, fix the test. If the spec is wrong, say so — don't silently deviate.
