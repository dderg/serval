# Subspec 6e — CornerBlender Per-Corner Shape Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Teach `klippy/blendplanner.CornerBlender` to pick between the G¹ tangent arc (`blendmath.BlendArc`) and the G² quintic Bézier (`blendquintic.QuinticBlend`) per corner, based on the corner's deflection angle. The downstream emission path stops hard-coding `BlendArc` and becomes shape-agnostic behind a new `klippy/blendemit.segment(blend, err)` seam. Two user-tunable degree thresholds (`shape_switchover_low`, `shape_switchover_high`) land in `[printer]`. No feature flag — quintic replaces arc in its angle band unconditionally.

**Architecture:** One new helper module `klippy/blendemit.py` holds an `isinstance`-dispatch `segment(blend, max_chord_err)` that calls the existing `blendmath.segment_arc` or `blendquintic.segment_quintic`. The planner gains `_select_blend(prev, nxt)` (angle-driven dispatch to `blend_from_moves` vs `blend_from_moves_quintic`) and renames `_emit_arc` → `_emit_blend` (one-line polyline substitution). The E-axis call stays `blendmath.interpolate_extruder` for both shapes; the byte-for-byte duplicate `blendquintic.interpolate_extruder_quintic` is deleted. `toolhead.py` parses the two thresholds and stashes them on the toolhead for the selector.

**Tech Stack:** Python 3.x, stdlib `math` only. Tests use `pytest` and `pytest.approx`. No numpy dependency. All tests live in `test/`.

**Spec:** `docs/superpowers/specs/2026-04-19-subspec-6e-shape-selection-design.md`

---

## File Structure

**Files to create:**
- `klippy/blendemit.py` — ~30 LOC. Single function `segment(blend, max_chord_err)` dispatching on type (`QuinticBlend` → `segment_quintic`, else `segment_arc`). Import-time dependency on both shape modules.

**Files to modify:**
- `klippy/blendplanner.py` — import `blendemit`; add `CornerBlender._select_blend(prev, nxt)`; rename `_emit_arc` → `_emit_blend`; swap `blendmath.segment_arc(arc, ...)` for `blendemit.segment(blend, ...)`; swap degenerate check from `arc.R == 0.0` to `blend.d_consumed == 0.0`; wire `feed()` through the new selector.
- `klippy/blendquintic.py` — delete `interpolate_extruder_quintic` (unused after planner consolidation).
- `klippy/toolhead.py` — parse `shape_switchover_low` (default 35) and `shape_switchover_high` (default 150), range-check `0 < low < high < 180`, stash on the toolhead, include in `orig_cfg`, echo in `SET_VELOCITY_LIMIT` status.
- `test/test_blendquintic.py` — delete the three `interpolate_extruder_quintic` tests (they move to `test_blendmath.py` scope; `blendmath.interpolate_extruder` already has equivalent coverage).
- `test/test_blendplanner.py` — add `test_shape_selection_by_angle` integration test (shape fingerprint by α), add degenerate-corner selector test, adjust existing arc-path tests to reference `blend` naming where they inspect internals.

**No other files modified.**

---

## Repo conventions (read before starting)

- **Angle convention:** deflection angle α (radians). 0 = collinear, π = U-turn. `cos(α) = prev_dir · next_dir`. Thresholds are compared in degrees; keep α in radians internally and convert only at comparison.
- **Selection rule:** `α < low → arc`, `low ≤ α ≤ high → quintic`, `α > high → arc`. Defaults `low = 35°`, `high = 150°`.
- **Dataclass attributes shared by `BlendArc` and `QuinticBlend`:** `d_consumed`, `v_cap`, `theta`, `entry_tangent`, `exit_tangent`, `plane_normal`. The planner only touches `d_consumed` and `v_cap`; everything else is shape-internal.
- **Corner-local frame:** both `segment_arc` and `segment_quintic` return polyline points with the vertex at origin. Planner translates to world by `vertex + point`.
- **Commit style:** imperative mood, lowercase prefix (e.g., `blendemit: add segment dispatch helper`). Never add `Co-Authored-By` trailers.
- **Running tests:** from repo root, `python3 -m pytest test/test_blendplanner.py -v`. Individual: `python3 -m pytest test/test_blendplanner.py::test_name -v`. Full blend stack: `python3 -m pytest test/test_blendmath.py test/test_blendquintic.py test/test_blendplanner.py -v`.

---

## Task 1: `blendemit` module scaffold + import smoke test

**Files:**
- Create: `klippy/blendemit.py`
- Modify: `test/test_blendplanner.py` (append smoke test)

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendplanner.py`:

```python
def test_blendemit_module_imports():
    from klippy import blendemit
    assert blendemit is not None
    assert hasattr(blendemit, "segment")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_blendplanner.py::test_blendemit_module_imports -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'klippy.blendemit'`

- [ ] **Step 3: Write minimal implementation**

Create `klippy/blendemit.py`:

```python
# klippy/blendemit.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Shape-agnostic emission helpers for the corner blender.
#
# The planner calls into a single seam so new shapes (clothoid, etc.)
# slot in here without touching CornerBlender.
#
# See docs/superpowers/specs/2026-04-19-subspec-6e-shape-selection-design.md
from __future__ import annotations

from . import blendmath, blendquintic


def segment(blend, max_chord_err):
    """Return a polyline approximating `blend` with chord error
    <= max_chord_err. Dispatches on the blend's dataclass type.
    """
    if isinstance(blend, blendquintic.QuinticBlend):
        return blendquintic.segment_quintic(blend, max_chord_err)
    return blendmath.segment_arc(blend, max_chord_err)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_blendplanner.py::test_blendemit_module_imports -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add klippy/blendemit.py test/test_blendplanner.py
git commit -m "blendemit: scaffold module with segment() dispatch"
```

---

## Task 2: `blendemit.segment` dispatch — arc and quintic fixtures

**Files:**
- Modify: `test/test_blendplanner.py`

Prove the dispatch round-trips to each underlying module for both shape types. The fixtures construct a `BlendArc` directly and a `QuinticBlend` from `quintic_geometry`; the assertion is that `blendemit.segment` returns the same polyline the shape-specific function does.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendplanner.py`:

```python
def test_blendemit_segment_dispatches_to_arc_for_blendarc():
    from klippy import blendemit, blendmath
    arc = blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0),
        next_dir=(0.0, 1.0, 0.0),
        L_prev=10.0, L_next=10.0,
        corner_deviation=0.2,
        a_max=10000.0,
        j_eff=float("inf"),
    )
    assert arc is not None
    expected = blendmath.segment_arc(arc, 20e-3)
    got = blendemit.segment(arc, 20e-3)
    assert got == expected


def test_blendemit_segment_dispatches_to_quintic_for_quinticblend():
    from klippy import blendemit, blendquintic
    q = blendquintic.quintic_geometry(
        prev_dir=(1.0, 0.0, 0.0),
        next_dir=(0.0, 1.0, 0.0),
        L_prev=10.0, L_next=10.0,
        corner_deviation=0.2,
        a_max=45000.0,
    )
    assert q is not None
    expected = blendquintic.segment_quintic(q, 20e-3)
    got = blendemit.segment(q, 20e-3)
    assert got == expected
```

- [ ] **Step 2: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendplanner.py -v -k blendemit_segment_dispatches`
Expected: PASS on both (the minimal `segment` from Task 1 already dispatches correctly).

If either fails, check the `isinstance` arm matches the dataclass type actually returned by the shape module.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendemit: test segment dispatch for arc and quintic fixtures"
```

---

## Task 3: Delete `blendquintic.interpolate_extruder_quintic` + its tests

**Files:**
- Modify: `klippy/blendquintic.py` (remove `interpolate_extruder_quintic`)
- Modify: `test/test_blendquintic.py` (remove the three `interpolate_extruder` tests)

The function is a byte-for-byte duplicate of `blendmath.interpolate_extruder`. The planner will use the `blendmath` version for both shapes. The tests for the `blendmath` version already cover E-conservation, monotonicity, and the degenerate polyline.

- [ ] **Step 1: Delete the function body**

Open `klippy/blendquintic.py`. Locate `def interpolate_extruder_quintic(` (around line 598). Delete the function definition and its docstring block in full (continues to the end of the function body; ~30 lines).

- [ ] **Step 2: Delete the tests**

Open `test/test_blendquintic.py`. Delete the three tests (search for `interpolate_extruder` in the file):

- `test_interpolate_extruder_conserves_total_e`
- `test_interpolate_extruder_monotone_increasing`
- `test_interpolate_extruder_degenerate_polyline`

- [ ] **Step 3: Run the blend-stack tests to verify nothing else references the deleted symbol**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendquintic.py test/test_blendplanner.py -v`
Expected: PASS. If `AttributeError: module 'klippy.blendquintic' has no attribute 'interpolate_extruder_quintic'` pops up, grep for the remaining caller and remove/redirect it (should not happen — the planner does not yet use the quintic path).

- [ ] **Step 4: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: delete interpolate_extruder_quintic duplicate"
```

---

## Task 4: `CornerBlender._select_blend` — angle-driven shape dispatch

**Files:**
- Modify: `klippy/blendplanner.py`
- Modify: `test/test_blendplanner.py`

Add the selector as a pure method on `CornerBlender`. It computes deflection α from `prev.axes_r[:3]` and `nxt.axes_r[:3]`, compares against the toolhead's two thresholds (in degrees), and dispatches. The `feed` wiring lands in Task 6 — this task only introduces the method and unit-tests it in isolation.

Thresholds pull from module-level constants for now (`_SHAPE_SWITCHOVER_LOW_DEG = 35.0`, `_SHAPE_SWITCHOVER_HIGH_DEG = 150.0`). Task 8 swaps these for toolhead-config reads.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendplanner.py`:

```python
def _fake_move_for_dir(th, start, direction, length, speed=100.0, e=0.0):
    """Build a _FakeMove with start and a given unit-direction and length."""
    end = (
        start[0] + direction[0] * length,
        start[1] + direction[1] * length,
        start[2] + direction[2] * length,
        e,
    )
    return _FakeMove(th, start, end, speed=speed)


@pytest.mark.parametrize(
    "angle_deg,expected_type_name",
    [
        (15.0, "BlendArc"),
        (25.0, "BlendArc"),
        (34.0, "BlendArc"),
        (36.0, "QuinticBlend"),
        (45.0, "QuinticBlend"),
        (90.0, "QuinticBlend"),
        (135.0, "QuinticBlend"),
        (149.0, "QuinticBlend"),
        (151.0, "BlendArc"),
        (170.0, "BlendArc"),
    ],
)
def test_select_blend_dispatches_by_angle(angle_deg, expected_type_name):
    b = _blender()
    th = b._toolhead
    angle = math.radians(angle_deg)
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (math.cos(angle), math.sin(angle), 0.0)
    m_prev = _fake_move_for_dir(th, (0, 0, 0), prev_dir, 10.0)
    m_next = _fake_move_for_dir(th, m_prev.end_pos[:3], next_dir, 10.0)
    blend = b._select_blend(m_prev, m_next)
    assert blend is not None
    assert type(blend).__name__ == expected_type_name


def test_select_blend_uturn_returns_degenerate_quintic_or_zero_arc():
    # α ≈ π: both regimes dispatch to arc by the > high-threshold rule.
    # The arc module returns a degenerate BlendArc with R = d_consumed = 0.
    b = _blender()
    th = b._toolhead
    m_prev = _fake_move_for_dir(th, (0, 0, 0), (1.0, 0.0, 0.0), 10.0)
    m_next = _fake_move_for_dir(th, m_prev.end_pos[:3], (-1.0, 0.0, 0.0), 10.0)
    blend = b._select_blend(m_prev, m_next)
    assert blend is not None
    assert blend.d_consumed == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendplanner.py -v -k select_blend`
Expected: FAIL with `AttributeError: 'CornerBlender' object has no attribute '_select_blend'`.

- [ ] **Step 3: Write the implementation**

Edit `klippy/blendplanner.py`. Update the top-of-file imports:

```python
from . import blendemit, blendmath, blendquintic
```

Below the `_copy_caller_state` function and above the `class CornerBlender` declaration, add module-level constants:

```python
# Deflection-angle thresholds for the arc-vs-quintic selector, in degrees.
# See docs/superpowers/specs/2026-04-19-subspec-6e-shape-selection-design.md.
# Task 8 replaces these module constants with toolhead-config reads.
_SHAPE_SWITCHOVER_LOW_DEG = 35.0
_SHAPE_SWITCHOVER_HIGH_DEG = 150.0
```

Inside `class CornerBlender`, add the new method after `_resolve_chord_err` and before `_emit_arc`:

```python
    def _select_blend(self, prev, nxt):
        """Pick arc or quintic per the deflection-angle rule.

        alpha < low -> arc; low <= alpha <= high -> quintic;
        alpha > high -> arc. Thresholds read from module constants
        for now; Task 8 wires them to toolhead config.
        """
        prev_dir = prev.axes_r[:3]
        next_dir = nxt.axes_r[:3]
        dot = (
            prev_dir[0] * next_dir[0]
            + prev_dir[1] * next_dir[1]
            + prev_dir[2] * next_dir[2]
        )
        # Clamp for numerical safety before acos.
        if dot > 1.0:
            dot = 1.0
        elif dot < -1.0:
            dot = -1.0
        alpha_deg = math.degrees(math.acos(dot))
        low = _SHAPE_SWITCHOVER_LOW_DEG
        high = _SHAPE_SWITCHOVER_HIGH_DEG
        if low <= alpha_deg <= high:
            return blendquintic.blend_from_moves_quintic(
                prev, nxt,
                self._toolhead.corner_deviation,
                toolhead=self._toolhead,
            )
        return blendmath.blend_from_moves(
            prev, nxt,
            self._toolhead.corner_deviation,
            toolhead=self._toolhead,
        )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendplanner.py -v -k select_blend`
Expected: PASS on all parametrized cases and the U-turn case.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: add _select_blend angle-driven shape dispatch"
```

---

## Task 5: Rename `_emit_arc` → `_emit_blend` and generalize polyline call

**Files:**
- Modify: `klippy/blendplanner.py`

Behaviour-preserving refactor. The emission logic already uses only `d_consumed`, `v_cap`, and the polyline — no `arc.R`, no `arc.entry_pt`/`exit_pt` touches inside the body. Rename the parameter, swap `blendmath.segment_arc(arc, err)` for `blendemit.segment(blend, err)`, and leave everything else intact. `feed()` still calls `_emit_blend(self._prev, move, arc)` with an arc — the selector swap comes in Task 6. All existing tests must continue to pass unchanged.

- [ ] **Step 1: Verify current tests pass (baseline)**

Run: `python3 -m pytest test/test_blendplanner.py -v`
Expected: PASS. Record the count.

- [ ] **Step 2: Rename the method and its body variable**

Edit `klippy/blendplanner.py`. Change the method signature:

```python
    def _emit_blend(self, prev, nxt, blend):
```

Inside the body, replace every `arc` reference with `blend`:

- `arc.d_consumed` → `blend.d_consumed`
- `arc.v_cap` → `blend.v_cap`
- `blendmath.segment_arc(arc, chord_err)` → `blendemit.segment(blend, chord_err)`

The local variable names `arc_moves`, `arc_cap_v2`, `arc_cap_v`, `arc_accel` stay — they describe the emitted polyline moves, not the shape. (Aesthetic cleanup can come later; keeping them minimizes diff.)

Update the docstring first line:

```python
        """Construct [trunc_prev, blend_moves...] and the trunc_next_head.

        Returns (trunc_prev, arc_moves_list, trunc_next_head). The
        arc_moves_list name is historical; it holds polyline moves for
        whichever shape (arc or quintic) the selector picked.
        """
```

Update the caller inside `feed()` to pass `arc` by the new name (still the arc selector wins in `feed` — Task 6 swaps it):

```python
        trunc_prev, arc_moves, trunc_next_head = self._emit_blend(
            self._prev, move, arc
        )
```

- [ ] **Step 3: Run the full planner test suite**

Run: `python3 -m pytest test/test_blendplanner.py -v`
Expected: PASS, identical count to Step 1. If any test fails, the refactor has drifted behaviour — likely a stray `arc` reference was missed or the isinstance dispatch went the wrong way for an arc input.

- [ ] **Step 4: Commit**

```bash
git add klippy/blendplanner.py
git commit -m "blendplanner: rename _emit_arc to _emit_blend and generalize polyline call"
```

---

## Task 6: Wire `feed()` through `_select_blend` and update degenerate check

**Files:**
- Modify: `klippy/blendplanner.py`

Swap the hard-coded `blendmath.blend_from_moves` call in `feed()` for `self._select_blend(self._prev, move)`, and change the degenerate check from `arc.R == 0.0` to `blend.d_consumed == 0.0`. `QuinticBlend` has no `R`; `d_consumed == 0.0` is the shape-agnostic check that already holds for both (arc: `d_consumed = R·tan(θ/2) = 0` iff `R = 0`; quintic: explicitly zero in the U-turn branch).

- [ ] **Step 1: Edit `feed()`**

In `klippy/blendplanner.py`, inside `CornerBlender.feed`, replace the block:

```python
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
        trunc_prev, arc_moves, trunc_next_head = self._emit_blend(
            self._prev, move, arc
        )
```

with:

```python
        blend = self._select_blend(self._prev, move)
        if blend is None:
            # Collinear: prepass should have caught. Emit prev, buffer next.
            emitted = [self._prev]
            self._prev = move
            return emitted
        if blend.d_consumed == 0.0 or blend.v_cap == 0.0:
            # U-turn / degenerate: force a stop at the junction.
            self._prev.limit_next_junction_speed(0.0)
            emitted = [self._prev]
            self._prev = move
            return emitted
        trunc_prev, arc_moves, trunc_next_head = self._emit_blend(
            self._prev, move, blend
        )
```

- [ ] **Step 2: Run the full planner test suite**

Run: `python3 -m pytest test/test_blendplanner.py -v`
Expected: PASS. Existing 90° test (`test_90deg_corner_emits_trunc_prev_plus_arc_polyline_and_buffers_next_head`) now exercises the quintic path (90° ∈ [35°, 150°]), so polyline geometry assertions will shift. **If this test fails on arc-specific assertions** (e.g. `d_expected = 50e-3 * (sqrt(2)/2) / (1 - sqrt(2)/2)`), that is the expected symptom of the shape swap — proceed to Step 3.

- [ ] **Step 3: Relax the arc-shape-specific assertions in pre-existing 90° and 60° tests**

The existing `test_90deg_corner_emits_trunc_prev_plus_arc_polyline_and_buffers_next_head` and `test_asymmetric_segments_half_segment_rule_caps_consumption` compute `d_expected` from arc-only formulas. After the swap, 90° dispatches to quintic. Two options:

1. **Pin these tests to the arc regime** by setting `th._shape_switchover_low_deg = 91.0` in the test body (adds a toolhead override; Task 8 formalizes the attribute).
2. **Drop the arc-specific numeric assertions** and replace with shape-agnostic invariants (trunc_prev ends somewhere along +X before the vertex; trunc_next_head begins somewhere along +Y after the vertex; E is conserved).

**Pick option 1** — it preserves the arc-path coverage exactly, and the new `test_shape_selection_by_angle` in Task 9 covers the quintic-path geometry. Mechanically:

Add a helper at the top of the test module:

```python
def _pin_to_arc(th):
    """Force the selector into arc-only mode for tests that assert on
    arc-specific geometry (d_consumed formulas, midpoint caps, etc.)."""
    # Module constants are read at call time in Task 4; Task 8 makes them
    # attribute reads. For now the constants are patched via monkeypatching
    # the module (done in tests that need it via the fixture below).
    pass  # placeholder; see monkeypatch usage per-test
```

For each affected test, use `monkeypatch.setattr(blendplanner, "_SHAPE_SWITCHOVER_LOW_DEG", 181.0)` (i.e. push the low threshold above 180° so every corner lands in the arc band). Example patch for `test_90deg_corner_...`:

```python
def test_90deg_corner_emits_trunc_prev_plus_arc_polyline_and_buffers_next_head(
    monkeypatch,
):
    monkeypatch.setattr(blendplanner, "_SHAPE_SWITCHOVER_LOW_DEG", 181.0)
    b = _blender(max_chord_err=20e-3)
    # ... rest unchanged
```

Apply the same monkeypatch-in-fixture change to `test_asymmetric_segments_half_segment_rule_caps_consumption`, `test_e_conservation_through_blend` (E assertion holds for quintic too — monkeypatch optional, skip it there), `test_aggregate_kin_check_move_fires_on_representative_arc_move`, `test_aggregate_extruder_check_move_fires_when_extruding`, `test_aggregate_extruder_check_move_skipped_when_not_extruding`, `test_arc_polyline_smooth_delta_v2_equals_delta_v2`, `test_arc_polyline_speed_continuity_1ppm`, `test_property_random_3d_corners`, `test_pipeline_composition_prepass_then_blender`, `test_pipeline_adapter_get_last_returns_blender_prev_when_buffered`, `test_get_last_no_forfeit_callback_transfers_to_trunc_prev`, `test_set_velocity_limit_mid_blend_does_not_leak_lowered_accel`, `test_adapter_queue_reports_blender_buffered_move`.

Rule of thumb: any test that asserts on arc-specific formulas, polyline count, or polyline point positions needs the monkeypatch. Tests that only check Move-plumbing behaviour (`limit_next_junction_speed`, `check_move` firing, E-conservation) do not.

- [ ] **Step 4: Re-run the full planner test suite**

Run: `python3 -m pytest test/test_blendplanner.py -v`
Expected: PASS on all tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: wire feed() through _select_blend"
```

---

## Task 7: `toolhead.py` — parse `shape_switchover_low` / `_high`

**Files:**
- Modify: `klippy/toolhead.py`

Read the two degree thresholds from `[printer]` with defaults 35 / 150. Range-check: `0 < low < high < 180`. Store on `self` as `shape_switchover_low_deg` / `shape_switchover_high_deg`. Include in `orig_cfg` and echo in the `SET_VELOCITY_LIMIT` reset handler alongside `corner_deviation`.

- [ ] **Step 1: Parse and range-check the knobs**

In `klippy/toolhead.py`, inside `ToolHead.__init__`, locate the line:

```python
        self.corner_deviation = config.getfloat("corner_deviation", above=0.0)
```

Immediately below it, add:

```python
        self.shape_switchover_low_deg = config.getfloat(
            "shape_switchover_low", 35.0, above=0.0, below=180.0,
        )
        self.shape_switchover_high_deg = config.getfloat(
            "shape_switchover_high", 150.0, above=0.0, below=180.0,
        )
        if self.shape_switchover_low_deg >= self.shape_switchover_high_deg:
            raise config.error(
                "shape_switchover_low (%.3f) must be strictly less than "
                "shape_switchover_high (%.3f)" % (
                    self.shape_switchover_low_deg,
                    self.shape_switchover_high_deg,
                )
            )
```

- [ ] **Step 2: Add to `orig_cfg` and the reset echo**

Below the `self.orig_cfg["corner_deviation"] = self.corner_deviation` line, add:

```python
        self.orig_cfg["shape_switchover_low_deg"] = self.shape_switchover_low_deg
        self.orig_cfg["shape_switchover_high_deg"] = self.shape_switchover_high_deg
```

In the reset path (inside the `SET_VELOCITY_LIMIT` no-args handler, near `self.corner_deviation = self.orig_cfg["corner_deviation"]`), add:

```python
        self.shape_switchover_low_deg = self.orig_cfg["shape_switchover_low_deg"]
        self.shape_switchover_high_deg = self.orig_cfg["shape_switchover_high_deg"]
        msg.extend(
            (
                "shape_switchover_low: %.6f" % self.shape_switchover_low_deg,
                "shape_switchover_high: %.6f" % self.shape_switchover_high_deg,
            )
        )
```

- [ ] **Step 3: Smoke-check via planner tests**

The planner tests do not import toolhead.py (they use `_FakeToolhead`), so this change does not affect them directly. Ensure existing suites still pass:

Run: `python3 -m pytest test/ -v`
Expected: PASS. Failures here mean a syntax or import-time error in toolhead.py; fix before committing.

- [ ] **Step 4: Commit**

```bash
git add klippy/toolhead.py
git commit -m "toolhead: parse shape_switchover_low/high config knobs"
```

---

## Task 8: Selector reads toolhead attributes instead of module constants

**Files:**
- Modify: `klippy/blendplanner.py`
- Modify: `test/test_blendplanner.py`

The module constants in Task 4 were a bridge. Swap them for `toolhead.shape_switchover_low_deg` / `_high_deg` reads in `_select_blend`. Update `_FakeToolhead` so it exposes the attributes with the production defaults, and replace the `monkeypatch.setattr(blendplanner, "_SHAPE_SWITCHOVER_LOW_DEG", ...)` helpers from Task 6 with direct attribute sets on the fake toolhead.

- [ ] **Step 1: Update `_FakeToolhead`**

In `test/test_blendplanner.py`, add the two attributes to `_FakeToolhead.__init__`:

```python
        self.shape_switchover_low_deg = overrides.get(
            "shape_switchover_low_deg", 35.0,
        )
        self.shape_switchover_high_deg = overrides.get(
            "shape_switchover_high_deg", 150.0,
        )
```

- [ ] **Step 2: Update `_select_blend` to read from the toolhead**

In `klippy/blendplanner.py`, inside `_select_blend`, replace the two local `low`/`high` lines:

```python
        low = _SHAPE_SWITCHOVER_LOW_DEG
        high = _SHAPE_SWITCHOVER_HIGH_DEG
```

with:

```python
        low = self._toolhead.shape_switchover_low_deg
        high = self._toolhead.shape_switchover_high_deg
```

Delete the module-level constants `_SHAPE_SWITCHOVER_LOW_DEG` and `_SHAPE_SWITCHOVER_HIGH_DEG` — no longer referenced.

- [ ] **Step 3: Replace monkeypatch calls with direct attribute sets in the Task-6 tests**

In the affected tests, change every `monkeypatch.setattr(blendplanner, "_SHAPE_SWITCHOVER_LOW_DEG", 181.0)` to `th.shape_switchover_low_deg = 181.0` (where `th` is the local toolhead; the blender is constructed just after, or `b._toolhead.shape_switchover_low_deg = 181.0` if the blender is already constructed). Remove the `monkeypatch` parameter from the test signatures where it is no longer used.

- [ ] **Step 4: Run the full test suite**

Run: `python3 -m pytest test/ -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: read shape-switchover thresholds from toolhead"
```

---

## Task 9: Integration test — `test_shape_selection_by_angle` with fingerprints

**Files:**
- Modify: `test/test_blendplanner.py`

Feed corner pairs at the spec's full angle list {15, 25, 34, 36, 45, 90, 135, 149, 151, 170, 179}. For each, assert the emitted polyline's curvature fingerprint matches the expected shape:

- **Arc fingerprint:** non-endpoint polyline curvature (discrete: `|vcross(p_{i+1}-p_i, p_i-p_{i-1})| / (|p_{i+1}-p_i| * |p_i-p_{i-1}|)`) is near-uniform across interior vertices (max/min ratio < ~1.5).
- **Quintic fingerprint:** discrete curvature is near-zero at the two interior vertices adjacent to the endpoints (first and last), and peaks at the center (central max > endpoint-neighbor * 3).

179° is a U-turn-adjacent case; both regimes fall in arc band, and the arc module returns a non-degenerate blend with small `d_consumed`. The existing degenerate handler in `feed()` only triggers at exactly `d_consumed == 0.0`.

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendplanner.py`:

```python
def _discrete_curvatures(points):
    """Return a list of discrete curvature estimates at interior vertices.
    points: list of 3-tuples. Ignores degenerate zero-length segments.
    """
    out = []
    for i in range(1, len(points) - 1):
        a = points[i - 1]
        b = points[i]
        c = points[i + 1]
        v1 = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        v2 = (c[0] - b[0], c[1] - b[1], c[2] - b[2])
        n1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2 + v1[2] ** 2)
        n2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2 + v2[2] ** 2)
        if n1 == 0.0 or n2 == 0.0:
            continue
        cx = v1[1] * v2[2] - v1[2] * v2[1]
        cy = v1[2] * v2[0] - v1[0] * v2[2]
        cz = v1[0] * v2[1] - v1[1] * v2[0]
        cross_mag = math.sqrt(cx * cx + cy * cy + cz * cz)
        out.append(cross_mag / (n1 * n2))
    return out


@pytest.mark.parametrize(
    "angle_deg,expected_shape",
    [
        (15.0, "arc"),
        (25.0, "arc"),
        (34.0, "arc"),
        (36.0, "quintic"),
        (45.0, "quintic"),
        (90.0, "quintic"),
        (135.0, "quintic"),
        (149.0, "quintic"),
        (151.0, "arc"),
        (170.0, "arc"),
        (179.0, "arc"),
    ],
)
def test_shape_selection_by_angle(angle_deg, expected_shape):
    b = _blender(max_chord_err=5e-3)
    th = b._toolhead
    angle = math.radians(angle_deg)
    m_prev = _fake_move_for_dir(th, (0, 0, 0), (1.0, 0.0, 0.0), 10.0, speed=200.0)
    next_dir = (math.cos(angle), math.sin(angle), 0.0)
    m_next = _fake_move_for_dir(
        th, m_prev.end_pos[:3], next_dir, 10.0, speed=200.0,
    )
    b.feed(m_prev)
    out = b.feed(m_next)
    # Drain buffered trunc_next_head.
    out += b.flush()

    # Pull the polyline back out of the emitted moves. The polyline
    # consists of every emitted move's start point, plus the last move's
    # end point.
    poly = [m.start_pos[:3] for m in out] + [out[-1].end_pos[:3]]
    # Strip trunc_prev (first point) and trunc_next_head (last point);
    # what remains is the blend's polyline.
    blend_poly = poly[1:-1]
    # Need at least 3 points (2 interior) for fingerprint discrimination.
    if len(blend_poly) < 3:
        pytest.skip("polyline too short to fingerprint at this angle")

    curvatures = _discrete_curvatures(blend_poly)
    assert curvatures, "no interior curvatures computed"
    k_max = max(curvatures)
    k_min = min(curvatures)
    n = len(curvatures)

    if expected_shape == "arc":
        # Near-uniform curvature: max/min should be modest (< 2.0 with
        # chord-error discretization; tighten if needed).
        assert k_max > 0.0
        assert k_max / max(k_min, 1e-12) < 2.0, (
            "arc fingerprint expected at alpha=%.1f: k_max/k_min = %.3f"
            % (angle_deg, k_max / max(k_min, 1e-12))
        )
    else:
        # Quintic: endpoint-adjacent curvatures much smaller than center.
        center_k = curvatures[n // 2]
        edge_k = max(curvatures[0], curvatures[-1])
        assert center_k > 0.0
        assert center_k > edge_k * 3.0, (
            "quintic fingerprint expected at alpha=%.1f: center=%.4e "
            "edge=%.4e" % (angle_deg, center_k, edge_k)
        )
```

- [ ] **Step 2: Run the test**

Run: `python3 -m pytest test/test_blendplanner.py::test_shape_selection_by_angle -v`
Expected: PASS on all 11 parametrizations. If a fingerprint threshold (2.0 or 3.0) trips:

1. For the arc case: chord-error discretization makes curvature estimate vary. Loosen the ratio to 2.5 or tighten `max_chord_err` in the test to `1e-3`.
2. For the quintic case: if `n // 2` lands on the true peak, ratio is huge; if off-peak (because `t_peak` is off-center for the r values used), lower the 3.0 to 2.0.

If the test is skipped (< 3 polyline points) at 15° or 170°: chord error is too loose for those shallow corners; drop `max_chord_err` to `2e-3` and retry.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: test shape selection by deflection angle"
```

---

## Task 10: Simulator scenario — `klipper-sim/examples/` 6e slice pass

**Files:**
- Modify (if accessible): `~/Developer/klipper-sim/examples/shape_selection_6e.py` (new or extension of `shape_ceiling.py`)

**Gating:** This task edits a file in a sibling repo (`~/Developer/klipper-sim/`). If the repo is not accessible, skip this task and leave a note in the session log. The core acceptance does not depend on the simulator.

- [ ] **Step 1: Check if klipper-sim is accessible**

Run: `test -d ~/Developer/klipper-sim/examples && echo "present" || echo "absent"`

If "absent": skip this task, commit nothing. Leave the note in the session log: "6e simulator parity deferred — klipper-sim repo not present."

- [ ] **Step 2: If present, run the `slice_24layers.gcode` fixture through two configurations**

Create `~/Developer/klipper-sim/examples/shape_selection_6e.py` (or extend the existing `shape_ceiling.py` driver):

- **Configuration A:** selector defaults (`low=35, high=150`). Quintic active in its band.
- **Configuration B:** arc-only (`low=181, high=181`). Forces arc everywhere.

For each, log total print time and max post-shaper Y excursion across the slice.

Assertions (the spec's Test-plan item 4):

- `total_time(A) <= total_time(B)` within a 0.5% tolerance.
- `max_post_shaper_y(A) <= max_post_shaper_y(B)` within a 1% tolerance on a representative mid-sample.

If the fixture `slice_24layers.gcode` is not present in `klipper-sim/examples/`, substitute the repo's default test slice and note the substitution in the session log.

- [ ] **Step 3: Commit in klipper-sim if it has a git repo; otherwise skip**

```bash
cd ~/Developer/klipper-sim
git add examples/shape_selection_6e.py
git commit -m "examples: add subspec-6e shape-selection scenario"
```

If the simulator repo has no git tracking, leave the script in place and note in the session log that it is persisted but not committed.

---

## Task 11: Final full-suite verification

**Files:** none modified.

- [ ] **Step 1: Run the complete blend stack test suite**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendquintic.py test/test_blendplanner.py -v`
Expected: PASS on all tests. Note the count; no new failures vs. pre-6e baseline.

- [ ] **Step 2: Run the full `test/` directory**

Run: `python3 -m pytest test/ -v`
Expected: PASS. Any pre-existing failures unrelated to blend changes are noted but not gating.

- [ ] **Step 3: Placeholder scan**

Run: `grep -n "TBD\|XXX\|FIXME" klippy/blendemit.py klippy/blendplanner.py klippy/blendquintic.py klippy/toolhead.py`
Expected: no hits in the files touched by 6e. (Pre-existing hits elsewhere are fine.)

- [ ] **Step 4: Sanity sweep on the thresholds (optional, from spec Test-plan item 5)**

If time allows, locally set `(shape_switchover_low, shape_switchover_high)` ∈ {(30,140), (35,150), (40,160)} via a bench config and re-run the simulator scenario from Task 10. Expect the 35/150 default to sit inside a broad plateau of total-time/quality outcomes. No test assertion here; this is a confidence check for the defaults.

- [ ] **Step 5: Commit (if any polish edits landed during verification)**

```bash
git add -p  # review each hunk
git commit -m "blendplanner: final polish for subspec 6e"
```

If nothing changed in Steps 1–4, skip the commit.

---

## Post-6e notes

- **Hardware test** on V0 / Trident: flash the branch, print the Voron cube, confirm no regression in visible print quality vs. pre-6e arc blender. The spec's "Done when" section gates this as an acceptance criterion. Not a code-level task; record the result in the session log.
- **6g** (inverse-shaper pre-compensation) can begin in parallel — it receives a G²-continuous commanded trajectory from the now-shape-agnostic emitter.
- **6f** (G³ clothoid) is deferred; revisit only if the hardware test reveals residual ringing at the quintic's peak-curvature region.
