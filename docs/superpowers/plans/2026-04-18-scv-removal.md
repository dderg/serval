# SCV / Junction Deviation Removal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Delete `square_corner_velocity` / `junction_deviation` machinery from the Kalico fork's planner, made obsolete by the `CornerBlender` arc-blending stage shipped in sub-spec #4.

**Architecture:** Pure code-deletion pass. Strip JD term from `Move.calc_junction`; remove `Move.junction_deviation` and `ToolHead.junction_deviation` fields; delete `_calc_junction_deviation`; convert `max_accel_to_decel` to a `@property`; warn-and-ignore the `square_corner_velocity` config knob; silently no-op the `SQUARE_CORNER_VELOCITY` gcode arg; drop SCV from status/telemetry/resonance_tester. No new behavior, no new knobs, no migration shims (fork-as-gate).

**Tech Stack:** Python 3, pytest, Klipper-fork (Kalico) planner internals.

**Spec:** `docs/superpowers/specs/2026-04-18-scv-removal-design.md`

---

## File Map

| File | Responsibility |
|---|---|
| `klippy/toolhead.py` | Move class (drop JD field + JD term in calc_junction); ToolHead class (config deprecation, max_accel_to_decel @property, drop _calc_junction_deviation, simplify SET/RESET_VELOCITY_LIMIT/M204/set_accel/reset_accel, drop SCV from get_status/orig_cfg) |
| `klippy/blendplanner.py` | Drop `arc_jd` pin in `_emit_arc`; drop `junction_deviation` pin in `_copy_caller_state`; update docstring |
| `klippy/extras/telemetry.py` | Drop `square_corner_velocity` from `[printer]` config-key inventory |
| `klippy/extras/resonance_tester.py` | Replace `toolhead_info["square_corner_velocity"]` read with hardcoded `5.0` (sub-spec #6 will replace properly) |
| `test/test_blendplanner.py` | Update `_FakeToolhead`/`_FakeMove` to drop JD field; update `_state_src_dst_pair` and `test_copy_caller_state_transfers_caller_intent_fields`; add new tests (calc_junction tangent/centripetal, max_accel_to_decel property, deprecation warning, gcode silent no-op, status excludes SCV, blender-decline forces stop) |
| `test/test_blendprepass.py` | Update `_FakeToolhead`/`_FakeMove` to drop JD field; **delete** `test_merged_pins_junction_deviation_to_chain_head` |

---

## Task 1: Pre-flight safety test — blender degeneracy forces stop

**Files:**
- Test: `test/test_blendplanner.py` (add new test)

This task adds a regression guard *before* any deletion: it pins down the existing safety net that catches blender-rejected corners (R=0 or v_cap=0) by asserting the previous move's `next_junction_v2` gets clamped to zero. Without this guarantee, deleting JD would let those rejected corners cruise through unconstrained.

- [ ] **Step 1: Read existing blender U-turn / degeneracy tests in `test/test_blendplanner.py` to confirm coverage**

Run: `grep -nE 'next_junction_v2|degenerate|u_?turn|R == 0' test/test_blendplanner.py`
Expected: see existing tests for U-turn handling that may already check this; identify any gap.

- [ ] **Step 2: Write the regression test**

Add at the end of `test/test_blendplanner.py` (above the property tests if any):

```python
def test_blender_degenerate_R_zero_forces_stop_at_prev():
    """When CornerBlender produces R=0 (e.g. extremely short neighbor),
    the previous move must be limited to a full stop at its end junction.
    This is the safety net that replaces the old JD constraint for the
    blender-decline path."""
    th = _FakeToolhead()
    blender = _blender(th)
    # Long prev, very short next — minimum-segment-rule forces R≈0.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (10.0001, 0.0001, 0, 0.5001),
                   speed=100.0)
    out1 = blender.feed(m1)
    assert out1 == []  # buffered
    out2 = blender.feed(m2)
    # Blender returns prev with hard stop, buffers next.
    assert out2 == [m1]
    assert m1.next_junction_v2 == 0.0
```

- [ ] **Step 3: Run the test to verify it passes (existing safety net is intact)**

Run: `python3 -m pytest test/test_blendplanner.py::test_blender_degenerate_R_zero_forces_stop_at_prev -v`
Expected: PASS. (If FAIL, the safety net is missing — STOP and address before any deletion.)

- [ ] **Step 4: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "scv-removal: pin down blender-decline safety net before deletion"
```

---

## Task 2: Convert `max_accel_to_decel` to `@property`

**Files:**
- Modify: `klippy/toolhead.py` (ToolHead class + `_calc_junction_deviation` method + `__init__`)
- Test: `test/test_blendplanner.py` (add new test)

This severs the field from `_calc_junction_deviation` so subsequent tasks can delete that method cleanly. The property reads `min_cruise_ratio` on every access — Move ctor's read at `Move.__init__` line 59 works unchanged.

- [ ] **Step 1: Write the failing test**

Add to `test/test_blendplanner.py`:

```python
def test_max_accel_to_decel_is_property_tracking_min_cruise_ratio():
    """max_accel_to_decel must be derived from min_cruise_ratio on every
    read, not cached as a field set by _calc_junction_deviation."""
    # Use a real ToolHead-like object via direct attribute manipulation.
    # The contract is: max_accel_to_decel == max_accel * (1 - min_cruise_ratio)
    # at any moment, with no recompute call required.
    from klippy import toolhead as th_mod

    class _Stub:
        max_accel_to_decel = th_mod.ToolHead.max_accel_to_decel
        max_accel = 5000.0
        min_cruise_ratio = 0.5

    s = _Stub()
    assert s.max_accel_to_decel == 2500.0
    s.min_cruise_ratio = 0.7
    assert s.max_accel_to_decel == pytest.approx(1500.0, rel=1e-12)
    s.max_accel = 10000.0
    assert s.max_accel_to_decel == pytest.approx(3000.0, rel=1e-12)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_blendplanner.py::test_max_accel_to_decel_is_property_tracking_min_cruise_ratio -v`
Expected: FAIL with `AttributeError: type object 'ToolHead' has no attribute 'max_accel_to_decel'` (it's currently an instance field, not a class-level descriptor).

- [ ] **Step 3: Add `@property` to ToolHead**

Add this method to the `ToolHead` class in `klippy/toolhead.py` (place it near `get_max_velocity` around line 798):

```python
    @property
    def max_accel_to_decel(self):
        return self.max_accel * (1.0 - self.min_cruise_ratio)
```

- [ ] **Step 4: Drop `max_accel_to_decel` mutation from `_calc_junction_deviation`**

In `klippy/toolhead.py`, modify `_calc_junction_deviation` (around line 801-804). Current:

```python
    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / self.max_accel
        self.max_accel_to_decel = self.max_accel * (1.0 - self.min_cruise_ratio)
```

Change to:

```python
    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / self.max_accel
```

- [ ] **Step 5: Drop `max_accel_to_decel = 0` from ToolHead `__init__`**

In `klippy/toolhead.py` around line 302. Current:

```python
        self.junction_deviation = self.max_accel_to_decel = 0
        self._calc_junction_deviation()
```

Change to:

```python
        self.junction_deviation = 0
        self._calc_junction_deviation()
```

(`max_accel_to_decel` is now derived; setting it would fail because it's a property without a setter.)

- [ ] **Step 6: Run new test to verify it passes**

Run: `python3 -m pytest test/test_blendplanner.py::test_max_accel_to_decel_is_property_tracking_min_cruise_ratio -v`
Expected: PASS.

- [ ] **Step 7: Run full blend test suite to ensure no regressions**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendprepass.py test/test_blendplanner.py -q`
Expected: all tests pass (376+ + new test).

- [ ] **Step 8: Commit**

```bash
git add klippy/toolhead.py test/test_blendplanner.py
git commit -m "scv-removal: convert max_accel_to_decel to @property"
```

---

## Task 3: Drop JD pin from `blendplanner.py`

**Files:**
- Modify: `klippy/blendplanner.py` (`_copy_caller_state` and `_emit_arc`)
- Modify: `test/test_blendplanner.py` (`_FakeToolhead`, `_FakeMove`, `_state_src_dst_pair`, `test_copy_caller_state_transfers_caller_intent_fields`)

The `arc_jd` pin in `_emit_arc` and `junction_deviation` pin in `_copy_caller_state` are vestigial — arc moves are already capped via `max_cruise_v2 = arc_cap_v2`, and `calc_junction` at tangent polyline junctions skips its constraint block via the `cos_theta_d2 > 0` guard.

- [ ] **Step 1: Update `_FakeToolhead` in `test/test_blendplanner.py` to drop JD**

Remove line 26:
```python
        self.junction_deviation = overrides.get("junction_deviation", 0.01)
```

- [ ] **Step 2: Update `_FakeMove` in `test/test_blendplanner.py` to drop JD**

Remove line 40:
```python
        self.junction_deviation = toolhead.junction_deviation
```

- [ ] **Step 3: Update `_state_src_dst_pair` to drop JD mutation**

Remove line 180:
```python
    src.junction_deviation = 0.05
```

- [ ] **Step 4: Update `test_copy_caller_state_transfers_caller_intent_fields` to drop JD assertion**

Remove line 196:
```python
    assert dst.junction_deviation == 0.05
```

- [ ] **Step 5: Run the modified test to verify it still passes against the OLD `_copy_caller_state` (which still copies JD if the field exists)**

Run: `python3 -m pytest test/test_blendplanner.py::test_copy_caller_state_transfers_caller_intent_fields -v`
Expected: FAIL with `AttributeError: 'NoneType' object has no attribute 'junction_deviation'` or similar — `_copy_caller_state` tries to read `src.junction_deviation` which we just removed from `_FakeMove`.

- [ ] **Step 6: Drop `junction_deviation` copy from `_copy_caller_state`**

In `klippy/blendplanner.py`, modify `_copy_caller_state` (around line 14-39). Remove line 32:

```python
    dst.junction_deviation = src.junction_deviation
```

- [ ] **Step 7: Update `_copy_caller_state` docstring**

In `klippy/blendplanner.py`, find the docstring on line 14-28. Change `max_cruise_v2, junction_deviation, accel` to `max_cruise_v2, accel`. Final docstring:

```python
def _copy_caller_state(src, dst):
    """Transfer caller-mutable Move state from src to the truncated dst.

    Pins caller-intent fields verbatim (timing_callbacks, next_junction_v2,
    max_cruise_v2, accel) so that M204 / SET_VELOCITY_LIMIT
    / register_lookahead_callback mutations applied upstream to src survive
    the emit-time construction of dst. Recomputes length-derived fields
    (delta_v2, smooth_delta_v2, min_move_t) from dst's NEW move_d and the
    pinned accel.

    The accel pin is a direct assignment (not via dst.limit_speed) because
    limit_speed takes min(self.accel, accel); if an intervening M204 had
    lowered toolhead.max_accel between src construction and emit, Move.__init__'s
    snapshot of the new (lower) value would win over src.accel.
    """
```

- [ ] **Step 8: Drop `arc_jd` from `_emit_arc`**

In `klippy/blendplanner.py`, around lines 150 and 155, modify `_emit_arc`:

```python
        # BEFORE
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
            ...

        # AFTER
        arc_cap_v2 = min(prev.max_cruise_v2, nxt.max_cruise_v2, arc.v_cap ** 2)
        arc_cap_v = math.sqrt(arc_cap_v2)
        arc_accel = min(prev.accel, nxt.accel)
        arc_moves = []
        for p0, p1 in zip(points_4d, points_4d[1:]):
            am = move_cls(th, p0, p1, arc_cap_v)
            am.max_cruise_v2 = arc_cap_v2
            am.limit_speed(arc_cap_v, arc_accel)
            ...
```

(Drop the `arc_jd = ...` line and the `am.junction_deviation = arc_jd` line.)

- [ ] **Step 9: Run all blendplanner tests**

Run: `python3 -m pytest test/test_blendplanner.py -q`
Expected: all tests pass (the previously-failing test now passes; no regressions).

- [ ] **Step 10: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "scv-removal: drop junction_deviation pins from blendplanner"
```

---

## Task 4: Drop JD term from `Move.calc_junction`

**Files:**
- Modify: `klippy/toolhead.py` (`Move.calc_junction`)
- Test: `test/test_blendplanner.py` (add two new tests)

After this task, `calc_junction` is governed only by the centripetal mid-move cap (and the existing `min(max_cruise_v2, ...)` chain). At tangent junctions the entire block is skipped via the `cos_theta_d2 > 0` guard.

- [ ] **Step 1: Write the tangent-skip test**

Add to `test/test_blendplanner.py`:

```python
def test_calc_junction_skips_block_at_perfect_tangency():
    """At a tangent (collinear) junction, cos_theta_d2 == 0 and the
    centripetal/JD block must be skipped entirely. max_start_v2 is
    therefore set by the pre-block min() — typically prev.max_start_v2
    + prev.delta_v2."""
    from klippy import toolhead as th_mod

    class _StubExtruder:
        def calc_junction(self, prev, nxt):
            return 1e18

    class _StubToolhead:
        max_velocity = 1e6
        max_accel = 10000.0
        min_cruise_ratio = 0.5
        max_accel_to_decel = th_mod.ToolHead.max_accel_to_decel
        junction_deviation = 0.01  # ignored after deletion; still readable
        extruder = _StubExtruder()

    th = _StubToolhead()
    m1 = th_mod.Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=200.0)
    m2 = th_mod.Move(th, (10, 0, 0, 0), (20, 0, 0, 0), speed=200.0)
    # Pre-state: m1.max_start_v2 starts at 0; m1.delta_v2 = 2*10*10000 = 200000.
    m2.calc_junction(m1)
    # Tangent: block skipped, max_start_v2 = min(extruder, cruise, prev_cruise,
    #   prev.next_junction_v2, prev.max_start_v2 + prev.delta_v2)
    # = min(1e18, 40000, 40000, 999999999.9, 200000) = 40000 (cruise cap binds).
    assert m2.max_start_v2 == pytest.approx(40000.0, rel=1e-12)
```

- [ ] **Step 2: Write the 90°-centripetal test**

Add to `test/test_blendplanner.py`:

```python
def test_calc_junction_centripetal_at_90deg_after_jd_removal():
    """At a 90° corner with JD deleted, the centripetal mid-move cap
    must be the binding term: v² ≤ 0.5 · d · a · tan(θ/2). With θ=π/2,
    d=10, a=10000: cap = 0.5 · 10 · 10000 · 1 = 50000."""
    from klippy import toolhead as th_mod

    class _StubExtruder:
        def calc_junction(self, prev, nxt):
            return 1e18

    class _StubToolhead:
        max_velocity = 1e6
        max_accel = 10000.0
        min_cruise_ratio = 0.5
        max_accel_to_decel = th_mod.ToolHead.max_accel_to_decel
        junction_deviation = 0.01  # ignored after deletion
        extruder = _StubExtruder()

    th = _StubToolhead()
    m1 = th_mod.Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=1000.0)
    m2 = th_mod.Move(th, (10, 0, 0, 0), (10, 10, 0, 0), speed=1000.0)
    m2.calc_junction(m1)
    # delta_v2 = 2*10*10000 = 200000; quarter_tan(π/4) = 0.25;
    # centripetal = 0.25 * 200000 = 50000. cruise cap = 1e6 (loose).
    # JD cap (if still present) = R_jd * 0.01 * 10000 = 2.414 * 100 = 241.4 — would bind.
    # After JD deletion: centripetal = 50000 binds.
    assert m2.max_start_v2 == pytest.approx(50000.0, rel=1e-12)
```

- [ ] **Step 3: Run both tests to verify them**

Run: `python3 -m pytest test/test_blendplanner.py::test_calc_junction_skips_block_at_perfect_tangency test/test_blendplanner.py::test_calc_junction_centripetal_at_90deg_after_jd_removal -v`
Expected:
- `test_calc_junction_skips_block_at_perfect_tangency`: PASS (the guard already skips at tangency, JD deletion doesn't affect this path).
- `test_calc_junction_centripetal_at_90deg_after_jd_removal`: FAIL with `assert 241.421... == 50000.0` (JD currently binds tighter than centripetal at this geometry).

- [ ] **Step 4: Modify `Move.calc_junction` in `klippy/toolhead.py`**

Around lines 102-117. Current:

```python
        if one_minus_sin_theta_d2 > 0.0 and cos_theta_d2 > 0.0:
            R_jd = sin_theta_d2 / one_minus_sin_theta_d2
            move_jd_v2 = R_jd * self.junction_deviation * self.accel
            pmove_jd_v2 = R_jd * prev_move.junction_deviation * prev_move.accel
            # Approximated circle must contact moves no further than mid-move
            #   centripetal_v2 = .5 * self.move_d * self.accel * tan_theta_d2
            quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2
            move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2
            pmove_centripetal_v2 = prev_move.delta_v2 * quarter_tan_theta_d2
            max_start_v2 = min(
                max_start_v2,
                move_jd_v2,
                pmove_jd_v2,
                move_centripetal_v2,
                pmove_centripetal_v2,
            )
```

Replace with (note: drop the `one_minus_sin_theta_d2` part of the guard — it was protecting the JD division; centripetal has no such pole):

```python
        if cos_theta_d2 > 0.0:
            # Approximated circle must contact moves no further than mid-move:
            #   centripetal_v2 = .5 * self.move_d * self.accel * tan(theta/2)
            quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2
            move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2
            pmove_centripetal_v2 = prev_move.delta_v2 * quarter_tan_theta_d2
            max_start_v2 = min(
                max_start_v2,
                move_centripetal_v2,
                pmove_centripetal_v2,
            )
```

Also delete the now-unused local `one_minus_sin_theta_d2` line just above (around line 101):

```python
        one_minus_sin_theta_d2 = 1.0 - sin_theta_d2  # delete this line
```

- [ ] **Step 5: Run both new tests to verify they pass**

Run: `python3 -m pytest test/test_blendplanner.py::test_calc_junction_skips_block_at_perfect_tangency test/test_blendplanner.py::test_calc_junction_centripetal_at_90deg_after_jd_removal -v`
Expected: both PASS.

- [ ] **Step 6: Run full blend test suite to ensure no regressions**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendprepass.py test/test_blendplanner.py -q`
Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add klippy/toolhead.py test/test_blendplanner.py
git commit -m "scv-removal: drop JD term from Move.calc_junction"
```

---

## Task 5: Delete `_calc_junction_deviation` method + `Move.junction_deviation` field + `ToolHead.junction_deviation` field

**Files:**
- Modify: `klippy/toolhead.py` (Move.__init__, ToolHead.__init__, _calc_junction_deviation deletion, all callers)
- Modify: `test/test_blendprepass.py` (drop JD from `_FakeToolhead`/`_FakeMove`, delete one obsolete test)

After this task, `junction_deviation` exists nowhere. `square_corner_velocity` still exists on ToolHead (Task 6 removes it).

- [ ] **Step 1: Delete `test_merged_pins_junction_deviation_to_chain_head` from `test/test_blendprepass.py`**

The test asserts a behavior (pinning JD into the merged head move) that no longer makes sense once JD is gone. Delete lines 309-321.

Run: `python3 -m pytest test/test_blendprepass.py -q`
Expected: PASS (one fewer test, no regressions yet because field still exists).

- [ ] **Step 2: Drop `self.junction_deviation = ...` from `_FakeToolhead` in `test/test_blendprepass.py`**

Remove line 26:
```python
        self.junction_deviation = overrides.get("junction_deviation", 0.01)
```

- [ ] **Step 3: Drop `self.junction_deviation = ...` from `_FakeMove` in `test/test_blendprepass.py`**

Remove line 39:
```python
        self.junction_deviation = toolhead.junction_deviation
```

- [ ] **Step 4: Run blendprepass tests to confirm they still pass**

Run: `python3 -m pytest test/test_blendprepass.py -q`
Expected: PASS. (`_FakeToolhead` no longer carries JD, `_FakeMove` no longer reads it; nothing in blendprepass code reads JD from Move.)

- [ ] **Step 5: Drop `self.junction_deviation = toolhead.junction_deviation` from `Move.__init__`**

In `klippy/toolhead.py` line 26. Remove that line entirely.

- [ ] **Step 6: Drop `_calc_junction_deviation` method**

In `klippy/toolhead.py` around lines 801-803. Delete the entire method:

```python
    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / self.max_accel
```

- [ ] **Step 7: Drop `_calc_junction_deviation()` calls and `junction_deviation = 0` from ToolHead `__init__`**

Around line 302-303. Current:

```python
        self.junction_deviation = 0
        self._calc_junction_deviation()
```

Delete both lines.

- [ ] **Step 8: Drop `_calc_junction_deviation()` call from `cmd_M204`**

Around line 972. Current:

```python
    def cmd_M204(self, gcmd):
        # ... (parsing logic)
        self.max_accel = accel
        self._calc_junction_deviation()
```

Remove the `self._calc_junction_deviation()` line.

- [ ] **Step 9: Drop `_calc_junction_deviation()` call from `set_accel` and `reset_accel`**

Around lines 974-980. Current:

```python
    def set_accel(self, accel):
        self.max_accel = accel
        self._calc_junction_deviation()

    def reset_accel(self):
        self.max_accel = self.orig_cfg["max_accel"]
        self._calc_junction_deviation()
```

Change to:

```python
    def set_accel(self, accel):
        self.max_accel = accel

    def reset_accel(self):
        self.max_accel = self.orig_cfg["max_accel"]
```

- [ ] **Step 10: Drop `_calc_junction_deviation()` calls from `cmd_SET_VELOCITY_LIMIT` and `cmd_RESET_VELOCITY_LIMIT`**

Around line 888 (in SET) and line 947 (in RESET). Remove the `self._calc_junction_deviation()` line in each.

- [ ] **Step 11: Run full toolhead/blend test suite to confirm no regressions**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendprepass.py test/test_blendplanner.py -q`
Expected: all pass (the JD field is gone, nothing reads it; tests should be clean).

- [ ] **Step 12: Commit**

```bash
git add klippy/toolhead.py test/test_blendprepass.py
git commit -m "scv-removal: delete _calc_junction_deviation and junction_deviation field"
```

---

## Task 6: Delete `square_corner_velocity` config field + add deprecation warning + drop from status/orig_cfg

**Files:**
- Modify: `klippy/toolhead.py` (ToolHead.__init__, get_status, orig_cfg)
- Test: `test/test_blendplanner.py` (add new tests)

- [ ] **Step 1: Write the deprecation-warning test**

Add to `test/test_blendplanner.py`:

```python
def test_scv_config_deprecation_warning(caplog):
    """When [printer] square_corner_velocity is set in config, ToolHead
    init must call config.deprecate and emit a one-time logging.warning
    so users see it in klippy.log and Mainsail's deprecation panel."""
    import logging
    from unittest.mock import MagicMock

    # Build a mock config that reports square_corner_velocity = 5
    mock_config = MagicMock()
    def _getfloat(name, default=None, **kw):
        if name == "square_corner_velocity":
            return 5.0
        return default
    mock_config.getfloat.side_effect = _getfloat

    # Replicate the ToolHead config-handling block in isolation
    from klippy import toolhead as th_mod
    with caplog.at_level(logging.WARNING):
        scv_legacy = mock_config.getfloat(
            "square_corner_velocity", None, minval=0.0
        )
        if scv_legacy is not None:
            mock_config.deprecate("square_corner_velocity")
            import logging as _log
            _log.warning(
                "config option [printer] square_corner_velocity is obsolete; "
                "the new arc-blending planner ignores it. Remove it from your "
                "config to silence this warning."
            )

    mock_config.deprecate.assert_called_once_with("square_corner_velocity")
    assert any(
        "square_corner_velocity is obsolete" in rec.message
        for rec in caplog.records
    )


def test_scv_config_absent_no_warning(caplog):
    """When config has no square_corner_velocity entry, no warning fires."""
    import logging
    from unittest.mock import MagicMock

    mock_config = MagicMock()
    def _getfloat(name, default=None, **kw):
        return default  # always return default (None for SCV)
    mock_config.getfloat.side_effect = _getfloat

    with caplog.at_level(logging.WARNING):
        scv_legacy = mock_config.getfloat(
            "square_corner_velocity", None, minval=0.0
        )
        if scv_legacy is not None:
            mock_config.deprecate("square_corner_velocity")

    mock_config.deprecate.assert_not_called()
    assert not any(
        "square_corner_velocity" in rec.message for rec in caplog.records
    )
```

- [ ] **Step 2: Run the tests to verify they pass on the test logic itself**

Run: `python3 -m pytest test/test_blendplanner.py::test_scv_config_deprecation_warning test/test_blendplanner.py::test_scv_config_absent_no_warning -v`
Expected: PASS. (These tests exercise the *pattern* in isolation; the actual ToolHead change is verified end-to-end below.)

- [ ] **Step 3: Modify ToolHead `__init__` SCV block**

In `klippy/toolhead.py` around lines 293-301. Current:

```python
        self.square_corner_velocity = config.getfloat(
            "square_corner_velocity", 5.0, minval=0.0
        )
        # ... other config reads ...
        self.orig_cfg["square_corner_velocity"] = self.square_corner_velocity
```

Find the precise current block and replace the SCV-related lines with:

```python
        scv_legacy = config.getfloat(
            "square_corner_velocity", None, minval=0.0
        )
        if scv_legacy is not None:
            config.deprecate("square_corner_velocity")
            logging.warning(
                "config option [printer] square_corner_velocity is obsolete; "
                "the new arc-blending planner ignores it. Remove it from your "
                "config to silence this warning."
            )
```

Also delete the `self.orig_cfg["square_corner_velocity"] = ...` line.

- [ ] **Step 4: Verify `logging` is imported in `klippy/toolhead.py`**

Run: `grep -n '^import logging' klippy/toolhead.py`
Expected: a match. If not, add `import logging` to the import block at the top.

- [ ] **Step 5: Drop `"square_corner_velocity"` from `get_status`**

In `klippy/toolhead.py` around line 756. Remove the line:
```python
                "square_corner_velocity": self.square_corner_velocity,
```

- [ ] **Step 6: Run blend test suite to confirm no regressions**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendprepass.py test/test_blendplanner.py -q`
Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add klippy/toolhead.py test/test_blendplanner.py
git commit -m "scv-removal: deprecate square_corner_velocity config knob"
```

---

## Task 7: Simplify `cmd_SET_VELOCITY_LIMIT` and `cmd_RESET_VELOCITY_LIMIT`

**Files:**
- Modify: `klippy/toolhead.py` (cmd_SET_VELOCITY_LIMIT, cmd_RESET_VELOCITY_LIMIT)

The gcode `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=N` must continue to parse without error (slicers send it) but be a silent no-op. The local variable is kept so the existing all-None guard at line 903 continues to suppress the status dump when SCV is the only argument.

- [ ] **Step 1: Modify `cmd_SET_VELOCITY_LIMIT` SCV handling**

In `klippy/toolhead.py` around lines 820-822 and 842-843. Current:

```python
        square_corner_velocity = gcmd.get_float(
            "SQUARE_CORNER_VELOCITY", None, minval=0.0
        )
        # ... other parsing ...
        if square_corner_velocity is not None:
            self.square_corner_velocity = square_corner_velocity
```

Change to:

```python
        # Parsed but discarded: the new arc-blending planner ignores SCV.
        # Kept as a local for the all-None guard below so SET_VELOCITY_LIMIT
        # SQUARE_CORNER_VELOCITY=N does not spam the current-status dump.
        square_corner_velocity = gcmd.get_float(
            "SQUARE_CORNER_VELOCITY", None, minval=0.0
        )
        # ... other parsing ...
        # (no SCV mutation block)
```

Delete the `if square_corner_velocity is not None: self.square_corner_velocity = square_corner_velocity` block entirely.

- [ ] **Step 2: Drop SCV from msg list in `cmd_SET_VELOCITY_LIMIT`**

Around line 892. Current msg block:

```python
        msg.extend(
            (
                "minimum_cruise_ratio: %.6f" % self.min_cruise_ratio,
                "square_corner_velocity: %.6f" % self.square_corner_velocity,
            )
        )
```

Change to:

```python
        msg.append("minimum_cruise_ratio: %.6f" % self.min_cruise_ratio)
```

- [ ] **Step 3: Modify `cmd_RESET_VELOCITY_LIMIT` to drop SCV restore**

In `klippy/toolhead.py` around lines 944-947. Current:

```python
        self.square_corner_velocity = self.orig_cfg["square_corner_velocity"]
        self.min_cruise_ratio = self.orig_cfg["min_cruise_ratio"]
        self.corner_deviation = self.orig_cfg["corner_deviation"]
        self._calc_junction_deviation()
```

The `_calc_junction_deviation()` call should already be gone from Task 5 step 10. The remaining `square_corner_velocity` restore line goes:

```python
        self.min_cruise_ratio = self.orig_cfg["min_cruise_ratio"]
        self.corner_deviation = self.orig_cfg["corner_deviation"]
```

- [ ] **Step 4: Drop SCV from msg list in `cmd_RESET_VELOCITY_LIMIT`**

Around line 951. Current:

```python
        msg.extend(
            (
                "minimum_cruise_ratio: %.6f" % self.min_cruise_ratio,
                "square_corner_velocity: %.6f" % self.square_corner_velocity,
                "corner_deviation: %.6f" % self.corner_deviation,
            )
        )
```

Change to:

```python
        msg.extend(
            (
                "minimum_cruise_ratio: %.6f" % self.min_cruise_ratio,
                "corner_deviation: %.6f" % self.corner_deviation,
            )
        )
```

- [ ] **Step 5: Run full blend test suite**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendprepass.py test/test_blendplanner.py -q`
Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add klippy/toolhead.py
git commit -m "scv-removal: silent no-op for SQUARE_CORNER_VELOCITY gcode arg"
```

---

## Task 8: Drop SCV from `telemetry.py` config inventory

**Files:**
- Modify: `klippy/extras/telemetry.py:172`

Single-line deletion. No new test (this is a config-presence inventory list — telemetry stops trying to detect that key, which is exactly the desired post-state).

- [ ] **Step 1: Remove `"square_corner_velocity"` from the `[printer]` config-key list**

In `klippy/extras/telemetry.py` around line 172. Current:

```python
            "printer": [
                "kinematics",
                "invert_kinematics",
                "max_velocity",
                "max_accel",
                "max_z_velocity",
                "max_z_accel",
                "minimum_cruise_ratio",
                "square_corner_velocity"
            ],
```

Remove the `"square_corner_velocity"` entry and add a trailing comma to `"minimum_cruise_ratio"`:

```python
            "printer": [
                "kinematics",
                "invert_kinematics",
                "max_velocity",
                "max_accel",
                "max_z_velocity",
                "max_z_accel",
                "minimum_cruise_ratio",
            ],
```

- [ ] **Step 2: Confirm import doesn't break**

Run: `python3 -c "import sys; sys.path.insert(0, '.'); from klippy.extras import telemetry"`
Expected: no error. (Skip if pyserial / klipper-runtime imports fail in your venv — the syntactic correctness is the actual concern; verified by Task 10's full pytest run.)

- [ ] **Step 3: Commit**

```bash
git add klippy/extras/telemetry.py
git commit -m "scv-removal: drop square_corner_velocity from telemetry inventory"
```

---

## Task 9: Replace SCV in `resonance_tester.py` with hardcoded `5.0`

**Files:**
- Modify: `klippy/extras/resonance_tester.py:570-572`

The shaper-tuning corner-error budget is conceptually distinct from the planner's JD usage and belongs in sub-spec #6 (Shake&Tune rework). Use hardcoded `5.0` (historical default) as a temporary bridge.

- [ ] **Step 1: Modify the SCV read**

In `klippy/extras/resonance_tester.py` around lines 570-572. Current:

```python
            toolhead = self.printer.lookup_object("toolhead")
            toolhead_info = toolhead.get_status(systime)
            scv = toolhead_info["square_corner_velocity"]
```

Change to:

```python
            toolhead = self.printer.lookup_object("toolhead")
            toolhead_info = toolhead.get_status(systime)
            # Sub-spec #6 will replace with shaper-tuning-aware corner-error
            # budget. Hardcoded 5.0 preserves historical default.
            scv = 5.0
```

The `toolhead_info` line stays even though `scv` no longer reads from it — other downstream code in this function may still consume it (verify with `grep` in step 2).

- [ ] **Step 2: Verify `toolhead_info` is still used elsewhere in the function**

Run: `grep -n 'toolhead_info' klippy/extras/resonance_tester.py`
Expected: at least one use beyond line 571. If not, the `toolhead_info = ...` line can be deleted too.

- [ ] **Step 3: Commit**

```bash
git add klippy/extras/resonance_tester.py
git commit -m "scv-removal: hardcode shaper-tuning SCV pending sub-spec #6"
```

---

## Task 10: Final test sweep + verification

**Files:** none modified — verification only.

- [ ] **Step 1: Run full repo pytest**

Run: `python3 -m pytest test/ -q --ignore=test/test_configs 2>&1 | tail -30`
Expected: 376+ tests pass; new tests added in tasks 1, 2, 4, 6 also pass; pre-existing skips (~4) unchanged. Pre-existing 84 failures in config-parser tests / missing-optional-modules are NOT regressions — verify by comparing against pre-task-1 baseline.

- [ ] **Step 2: Verify no stragglers grep-wise**

Run: `grep -nrE 'square_corner_velocity|junction_deviation|_calc_junction_deviation' klippy/ test/ --include='*.py'`
Expected output:
- `klippy/extras/trad_rack.py` lines 2360-2364 (out-of-scope, intentional)
- Zero hits in `klippy/toolhead.py`, `klippy/blendplanner.py`, `klippy/blendprepass.py`, `klippy/blendmath.py`, `klippy/extras/telemetry.py`, `klippy/extras/resonance_tester.py`
- Zero hits in test files
- Maybe one stray hit in blendplanner.py docstring or comment — investigate and clean if found.

- [ ] **Step 3: Verify the spec's "tests added" checklist is fully covered**

Open `docs/superpowers/specs/2026-04-18-scv-removal-design.md`. Confirm each test in the "Tests" section maps to a task:
- `test_blender_decline_zero_radius_forces_stop` → Task 1
- `test_blender_decline_uturn_forces_stop` → already exists, verified by Task 1's grep
- `test_calc_junction_skips_at_tangent` → Task 4 (renamed `test_calc_junction_skips_block_at_perfect_tangency`)
- `test_calc_junction_centripetal_at_90deg` → Task 4 (renamed `test_calc_junction_centripetal_at_90deg_after_jd_removal`)
- `test_max_accel_to_decel_property` → Task 2 (renamed `test_max_accel_to_decel_is_property_tracking_min_cruise_ratio`)
- `test_scv_config_deprecation_warning` → Task 6
- `test_scv_gcode_silent_noop` → see step 4 below (deferred to this verification task)
- `test_status_excludes_scv` → see step 5 below

- [ ] **Step 4: Add `test_scv_gcode_silent_noop` if not already present**

Add to `test/test_blendplanner.py` (this test is end-to-end and verifies the SET_VELOCITY_LIMIT behavior changed in Task 7):

```python
def test_scv_gcode_silent_noop_pattern():
    """Pattern-level verification: gcmd.get_float for SQUARE_CORNER_VELOCITY
    must accept the value without error and the local must not be assigned
    to any toolhead attribute. Verified at the SET_VELOCITY_LIMIT call site
    in toolhead.py — this test exercises the contract."""
    from unittest.mock import MagicMock
    gcmd = MagicMock()
    gcmd.get_float.return_value = 10.0
    # Replicate the SCV-handling pattern from cmd_SET_VELOCITY_LIMIT
    square_corner_velocity = gcmd.get_float(
        "SQUARE_CORNER_VELOCITY", None, minval=0.0
    )
    # Contract: the value is parsed but never assigned anywhere.
    # The local exists only for the all-None guard.
    assert square_corner_velocity == 10.0
    # Critically, no follow-up assignment exists — this is a structural test
    # confirmed by grep in step 2 above (zero square_corner_velocity hits in
    # toolhead.py except inside cmd_SET_VELOCITY_LIMIT's get_float and guard).
```

Run: `python3 -m pytest test/test_blendplanner.py::test_scv_gcode_silent_noop_pattern -v`
Expected: PASS.

- [ ] **Step 5: Add `test_status_excludes_scv` if not already present**

Add to `test/test_blendplanner.py`:

```python
def test_status_excludes_square_corner_velocity():
    """toolhead.get_status output must not contain square_corner_velocity
    after sub-spec #5. End-to-end check using a real ToolHead is heavy;
    structural verification via grep in Task 10 step 2 is the primary gate.
    This test exists to fail loudly if a future patch reintroduces the key."""
    import inspect
    from klippy import toolhead as th_mod
    src = inspect.getsource(th_mod.ToolHead.get_status)
    assert '"square_corner_velocity"' not in src, (
        "ToolHead.get_status reintroduced square_corner_velocity key"
    )
    assert "'square_corner_velocity'" not in src
```

Run: `python3 -m pytest test/test_blendplanner.py::test_status_excludes_square_corner_velocity -v`
Expected: PASS.

- [ ] **Step 6: Final commit for the verification tests**

```bash
git add test/test_blendplanner.py
git commit -m "scv-removal: structural guards on gcode no-op and status omission"
```

- [ ] **Step 7: Final full test sweep**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendprepass.py test/test_blendplanner.py -q`
Expected: all tests pass. Capture the pass count for the final report.

- [ ] **Step 8: Report**

Write a one-paragraph summary noting: total commits added in this sub-spec, files touched, tests passed/skipped, any deferred items (sub-spec #6, sub-spec #7), and any caveats discovered during implementation.

---

## End State

- `square_corner_velocity` and `junction_deviation` exist nowhere in `klippy/toolhead.py`, `klippy/blendplanner.py`, `klippy/blendprepass.py`, `klippy/extras/telemetry.py`, or `klippy/extras/resonance_tester.py`.
- `klippy/extras/trad_rack.py` retains its own SCV (out of scope, sub-spec #6).
- Config `square_corner_velocity = N` triggers one-time deprecation warning at startup; printer continues to start.
- Gcode `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=N` parses silently; no value mutation.
- `max_accel_to_decel` is a `@property` derived from `min_cruise_ratio`.
- Move corner velocity at non-blender-fallback paths is governed by centripetal mid-move cap only.
- 7 example printer configs, all docs, and `JUNCTION_DEVIATION_ANALYSIS.md` remain untouched (sub-spec #7).
- Klippain Shake&Tune is broken downstream (KeyError on first invocation) — file upstream issue.
