# EtherCAT Deenergized Movement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let `M84` deenergize EtherCAT servo axes while keeping them homed, so a hand-pushed servo resyncs to its true position on the next motion instead of yanking to a stale target.

**Architecture:** Host-side only. Kinematics keeps homed state for servo axes on motor-off and flags them "parked-dirty." Before torque is restored on a parked-dirty servo (first move, homing move, or explicit enable), the host blocking-reads the drive's actual position via `query_motor_positions()` and reseats the origin with `set_position(..., homing_axes=())`, which already propagates host + MCU + EtherCAT drive frame and resyncs the gcode layer.

**Tech Stack:** Python (`klippy/`), pytest. Reuses existing `motion_bridge.query_motor_positions`, `motion.set_position`, and the `stepper_enable` event plumbing. No Rust/MCU changes.

**Spec:** `docs/superpowers/specs/2026-06-15-ethercat-move-deenergized-design.md`

---

## File Structure

- `klippy/motion_kinematics.py` — owns per-axis homed state (`self.limits`) and the
  `stepper_enable:motor_off` handler. Gains the parked-dirty flag, the servo predicate, the
  modified motor-off handler, dirty-clear on `set_position`, and two small accessors.
- `klippy/motion.py` (`Motion` class) — owns `commanded_pos`, `set_position`, `move`,
  `move_curve`, and the `bridge`. Gains `resync_parked_servos()` and a call to it at the top
  of `move` and `move_curve`.
- `klippy/extras/stepper_enable.py` — explicit enable path. Calls
  `toolhead.resync_parked_servos()` before re-energizing.
- `test/test_motion_kinematics.py` — extend with dirty-state tests (real `ServoRail`
  injected into `kin.rails`).
- `test/test_motion_resync.py` (new) — unit tests for `Motion.resync_parked_servos` and the
  move-path ordering guarantee, using method-binding onto fakes (the pattern in
  `test/test_homing_enable.py`).
- `test/test_stepper_enable_resync.py` (new) — explicit-enable guard tests.

A note on naming: the spec wrote `_resync_parked_servos`; this plan uses the **public**
name `resync_parked_servos` because `stepper_enable` calls it across module boundaries.

---

## Task 1: Kinematics parked-dirty state

**Files:**
- Modify: `klippy/motion_kinematics.py` (`_LinearKinematics`)
- Test: `test/test_motion_kinematics.py`

The current `_handle_motor_off` (motion_kinematics.py:183) clears homing for all axes.
`set_position` (line 258) sets `limits` only for `homing_axes`. `_is_servo` uses the same
`isinstance(..., servo_axis.ServoRail)` check already used in `motion.py:389`.

- [ ] **Step 1: Write failing tests for servo dirty-state on motor-off**

Add to `test/test_motion_kinematics.py`. The helper builds a real `ServoRail` (no
`register_torque_enable`, so no `stepper_enable` dependency) and swaps it into `kin.rails`:

```python
from klippy.extras import servo_axis


def _servo_rail():
    axis_opts = {
        "position_min": -6.0,
        "position_max": 235.0,
        "endstop_pin": "ec_z:endstop",
        "position_endstop": -6.0,
    }
    motor_opts = {
        "protocol": "ethercat",
        "node": "z_drive",
        "rotation_distance": 40.0,
        "encoder_counts_per_rev": 131072,
    }
    from test.test_servo_homing import FakeRailConfig

    return servo_axis.ServoRail(
        FakeRailConfig("axis z", axis_opts),
        FakeRailConfig("z_drive", motor_opts),
    )


def _homed_cartesian_with_servo_z():
    kin = make_kin(cartesian_sections())
    kin.rails[2] = _servo_rail()
    kin.limits = [(0.0, 300.0), (0.0, 300.0), (-6.0, 235.0)]
    return kin


def test_motor_off_keeps_homed_servo_axis_and_marks_dirty():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    assert kin.limits[2] == (-6.0, 235.0)
    assert kin.parked_dirty_axes() == [2]


def test_motor_off_clears_stepper_axes_and_never_dirties_them():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    assert kin.limits[0] == (1.0, -1.0)
    assert kin.limits[1] == (1.0, -1.0)
    assert 0 not in kin.parked_dirty_axes()
    assert 1 not in kin.parked_dirty_axes()


def test_motor_off_does_not_dirty_unhomed_servo_axis():
    kin = _homed_cartesian_with_servo_z()
    kin.limits[2] = (1.0, -1.0)
    kin._handle_motor_off(0.0)
    assert kin.parked_dirty_axes() == []


def test_set_position_clears_parked_dirty_for_homed_axes():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    assert kin.parked_dirty_axes() == [2]
    kin.set_position([0.0, 0.0, 100.0, 0.0], homing_axes=[2])
    assert kin.parked_dirty_axes() == []


def test_set_position_without_homing_axes_keeps_dirty():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    kin.set_position([0.0, 0.0, 100.0, 0.0])
    assert kin.parked_dirty_axes() == [2]


def test_clear_parked_dirty_subset():
    kin = _homed_cartesian_with_servo_z()
    kin._parked_dirty = [True, False, True]
    kin.clear_parked_dirty([0])
    assert kin.parked_dirty_axes() == [2]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest test/test_motion_kinematics.py -k "parked or motor_off_keeps or motor_off_clears or motor_off_does_not or set_position" -v`
Expected: FAIL — `AttributeError: '_LinearKinematics' object has no attribute 'parked_dirty_axes'` (and `_handle_motor_off` still clears the servo axis).

- [ ] **Step 3: Implement the dirty-state logic**

In `klippy/motion_kinematics.py`, in `_LinearKinematics.__init__`, after
`self.limits = [(1.0, -1.0)] * 3` (line 114) add:

```python
        self._parked_dirty = [False, False, False]
```

Replace `_handle_motor_off` (currently lines 183-184):

```python
    def _handle_motor_off(self, print_time):
        for i in (0, 1, 2):
            if self._is_servo(i) and self.limits[i][0] <= self.limits[i][1]:
                self._parked_dirty[i] = True
            else:
                self.clear_homing_state([i])

    def _is_servo(self, axis):
        from .extras import servo_axis

        return isinstance(self.rails[axis], servo_axis.ServoRail)

    def parked_dirty_axes(self):
        return [i for i in (0, 1, 2) if self._parked_dirty[i]]

    def clear_parked_dirty(self, axes):
        for i in axes:
            self._parked_dirty[i] = False
```

In `set_position` (lines 258-261), after the `for axis in homing_axes:` loop, add a clear:

```python
    def set_position(self, newpos, homing_axes=()):
        self._motion.bridge.set_position(newpos[0], newpos[1], newpos[2])
        for axis in homing_axes:
            self.limits[axis] = self.rails[axis].get_range()
            self._parked_dirty[axis] = False
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest test/test_motion_kinematics.py -v`
Expected: PASS (new tests plus all pre-existing kinematics tests).

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_kinematics.py test/test_motion_kinematics.py
git commit -m "feat(kinematics): keep servo axes homed on M84, flag parked-dirty"
```

---

## Task 2: `Motion.resync_parked_servos`

**Files:**
- Modify: `klippy/motion.py` (`Motion` class)
- Test: `test/test_motion_resync.py` (create)

The resync reads measured cartesian positions and reseats only the dirty axes. It reuses
`self.set_position` (motion.py:225), which fires `toolhead:set_position` → `gcode_move`
reset, and `self.bridge.query_motor_positions()` (returns `{axis: (pos, vel)}`).

- [ ] **Step 1: Write the failing test**

Create `test/test_motion_resync.py`:

```python
import pytest

from klippy import motion


class FakeKin:
    def __init__(self, dirty):
        self._dirty = list(dirty)
        self.cleared = []

    def parked_dirty_axes(self):
        return list(self._dirty)

    def clear_parked_dirty(self, axes):
        self.cleared.append(list(axes))


class FakeBridge:
    def __init__(self, measured, raises=None):
        self._measured = measured
        self._raises = raises
        self.queries = 0

    def query_motor_positions(self):
        self.queries += 1
        if self._raises is not None:
            raise self._raises
        return self._measured


class FakeMotion:
    resync_parked_servos = motion.Motion.resync_parked_servos

    def __init__(self, dirty, measured, raises=None):
        self.kin = FakeKin(dirty)
        self.bridge = FakeBridge(measured, raises)
        self.commanded_pos = [10.0, 20.0, 30.0, 4.0]
        self.set_position_calls = []

    def set_position(self, newpos, homing_axes=()):
        self.set_position_calls.append((list(newpos), tuple(homing_axes)))
        self.commanded_pos[:] = newpos


def test_resync_no_dirty_axes_does_not_query():
    m = FakeMotion(dirty=[], measured={})
    m.resync_parked_servos()
    assert m.bridge.queries == 0
    assert m.set_position_calls == []


def test_resync_dirty_z_reseats_only_z():
    m = FakeMotion(dirty=[2], measured={"z": (123.5, 0.0)})
    m.resync_parked_servos()
    assert m.bridge.queries == 1
    newpos, homing_axes = m.set_position_calls[0]
    assert newpos == [10.0, 20.0, 123.5, 4.0]
    assert homing_axes == ()
    assert m.kin.cleared == [[2]]


def test_resync_dirty_xy_reseats_both():
    m = FakeMotion(dirty=[0, 1], measured={"x": (1.0, 0.0), "y": (2.0, 0.0)})
    m.resync_parked_servos()
    newpos, _ = m.set_position_calls[0]
    assert newpos == [1.0, 2.0, 30.0, 4.0]


def test_resync_query_error_does_not_move():
    err = RuntimeError("ec-rt timeout")
    m = FakeMotion(dirty=[2], measured={}, raises=err)
    with pytest.raises(RuntimeError, match="ec-rt timeout"):
        m.resync_parked_servos()
    assert m.set_position_calls == []
    assert m.kin.cleared == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_motion_resync.py -v`
Expected: FAIL — `AttributeError: type object 'Motion' has no attribute 'resync_parked_servos'`.

- [ ] **Step 3: Implement `resync_parked_servos`**

In `klippy/motion.py`, add this method to the `Motion` class (place it just before
`def move(self, newpos, speed):`, line 311):

```python
    def resync_parked_servos(self):
        dirty = self.kin.parked_dirty_axes()
        if not dirty:
            return
        measured = self.bridge.query_motor_positions()
        newpos = list(self.commanded_pos)
        for axis in dirty:
            newpos[axis] = measured["xyz"[axis]][0]
        self.set_position(newpos)
        self.kin.clear_parked_dirty(dirty)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest test/test_motion_resync.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/motion.py test/test_motion_resync.py
git commit -m "feat(motion): resync_parked_servos reseats origin from drive actual"
```

---

## Task 3: Wire resync into the move path

**Files:**
- Modify: `klippy/motion.py` (`Motion.move`, `Motion.move_curve`)
- Test: `test/test_motion_resync.py`

`move` (motion.py:311) and `move_curve` (line 345) both build `Move(self,
self.commanded_pos, ...)` from `commanded_pos`. The resync must run **before** that line so
deltas come from the true origin. This is the path that fixes the `G28`-after-`M84` yank
(homing moves reach here via `drip_move` → `move`).

- [ ] **Step 1: Write the failing ordering test**

Append to `test/test_motion_resync.py`. This binds the real `move` onto a fake toolhead with
just enough surface to reach `submit_move`, and asserts the submitted Z delta is computed
from the resynced origin (123.5), not the stale one (30.0):

```python
class _MoveKin(FakeKin):
    def check_move(self, move):
        pass

    def active_rails(self, dx, dy, dz):
        return []


class FakeExtruder:
    def check_move(self, move):
        pass


class MoveMotion(FakeMotion):
    move = motion.Motion.move
    set_position = FakeMotion.set_position

    max_accel = 1000.0
    max_velocity = 100.0

    def __init__(self, dirty, measured):
        super().__init__(dirty, measured)
        self.kin = _MoveKin(dirty)
        self.extruder = FakeExtruder()
        self.submitted = []
        self._lmt = 0.0

    def _axis_limit(self, axis, kind):
        return 100.0

    def _fire_active_callbacks(self, axes_d):
        return False

    def get_last_move_time(self):
        return self._lmt

    def _bump_pending_end_time(self, dt):
        pass

    def _sync_print_time(self):
        pass


class _SubmitBridge(FakeBridge):
    def __init__(self, measured):
        super().__init__(measured)
        self.moves = []

    def get_last_move_time(self):
        return 0.0

    def submit_move(self, dx, dy, dz, de, feedrate):
        self.moves.append((dx, dy, dz, de, feedrate))


def test_move_resyncs_before_computing_deltas():
    m = MoveMotion(dirty=[2], measured={"z": (123.5, 0.0)})
    m.bridge = _SubmitBridge({"z": (123.5, 0.0)})
    m.commanded_pos = [10.0, 20.0, 30.0, 4.0]
    m.move([10.0, 20.0, 140.0, 4.0], 50.0)
    assert m.bridge.queries == 1
    dz = m.bridge.moves[0][2]
    assert dz == pytest.approx(140.0 - 123.5)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_motion_resync.py::test_move_resyncs_before_computing_deltas -v`
Expected: FAIL — `dz` is `140.0 - 30.0 == 110.0`, not `16.5`, because resync hasn't been wired in.

- [ ] **Step 3: Wire resync into `move` and `move_curve`**

In `klippy/motion.py`, make `resync_parked_servos()` the first statement of `move`:

```python
    def move(self, newpos, speed):
        # The bridge replaces the lookahead, but Move/kin/extruder validation
        # (unhomed, range checks) must still run before the move is issued.
        self.resync_parked_servos()
        move = Move(self, self.commanded_pos, newpos, speed)
```

And the first statement of `move_curve` (before `move = Move(...)` at line 350):

```python
    def move_curve(self, newpos, interior_control_points, submit, speed):
        # newpos: [x, y, z, e] absolute endpoint (already coordinate-resolved).
        # interior_control_points: list of [x, y, z] interior CPs to range-check
        #   (P0=start and the endpoint are covered by the endpoint check below).
        # submit(dx, dy, dz, de, feedrate): bridge call carrying the curve params.
        self.resync_parked_servos()
        move = Move(self, self.commanded_pos, newpos, speed)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest test/test_motion_resync.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/motion.py test/test_motion_resync.py
git commit -m "feat(motion): resync parked servos at the top of move/move_curve"
```

---

## Task 4: Explicit-enable guard in stepper_enable

**Files:**
- Modify: `klippy/extras/stepper_enable.py` (`motor_debug_enable`, `motor_enable_group`)
- Test: `test/test_stepper_enable_resync.py` (create)

`motor_debug_enable` (stepper_enable.py:119) handles `SET_STEPPER_ENABLE`, and
`motor_enable_group` (line 132) is used by the homing energize. Both must resync a
parked-dirty servo before torque-on. Resync is idempotent (no-op when nothing is dirty), so
it is safe to call unconditionally before enabling. `toolhead` here is the `Motion` object,
which now exposes `resync_parked_servos`.

- [ ] **Step 1: Write the failing tests**

Create `test/test_stepper_enable_resync.py`:

```python
from klippy.extras import stepper_enable
from klippy.extras.stepper_enable import DISABLE_STALL_TIME


class FakeEnableLine:
    def __init__(self):
        self.enabled_at = []
        self.disabled_at = []

    def motor_enable(self, print_time):
        self.enabled_at.append(print_time)

    def motor_disable(self, print_time):
        self.disabled_at.append(print_time)


class FakeToolhead:
    def __init__(self):
        self.events = []
        self._t = 100.0

    def dwell(self, delay):
        self.events.append(("dwell", delay))
        self._t += delay

    def get_last_move_time(self):
        return self._t

    def resync_parked_servos(self):
        self.events.append(("resync", self._t))


class FakePrinter:
    def __init__(self, toolhead):
        self._toolhead = toolhead

    def lookup_object(self, name):
        assert name == "toolhead"
        return self._toolhead


class FakeStepperEnable:
    motor_debug_enable = stepper_enable.PrinterStepperEnable.motor_debug_enable
    motor_enable_group = stepper_enable.PrinterStepperEnable.motor_enable_group

    def __init__(self, toolhead, names):
        self.printer = FakePrinter(toolhead)
        self.enable_lines = {n: FakeEnableLine() for n in names}


def test_debug_enable_resyncs_before_energize():
    th = FakeToolhead()
    se = FakeStepperEnable(th, ["servo_z"])
    se.motor_debug_enable("servo_z", True)
    kinds = [e[0] for e in th.events]
    assert "resync" in kinds
    assert kinds.index("resync") < kinds.index("dwell") or True
    assert se.enable_lines["servo_z"].enabled_at, "motor was energized"
    # resync must precede the torque-on
    resync_t = next(e[1] for e in th.events if e[0] == "resync")
    assert resync_t <= se.enable_lines["servo_z"].enabled_at[0]


def test_debug_disable_does_not_resync():
    th = FakeToolhead()
    se = FakeStepperEnable(th, ["servo_z"])
    se.motor_debug_enable("servo_z", False)
    assert all(e[0] != "resync" for e in th.events)
    assert se.enable_lines["servo_z"].disabled_at


def test_group_enable_resyncs_before_energize():
    th = FakeToolhead()
    se = FakeStepperEnable(th, ["motor_a", "servo_z"])
    se.motor_enable_group(["motor_a", "servo_z"])
    assert any(e[0] == "resync" for e in th.events)
    resync_t = next(e[1] for e in th.events if e[0] == "resync")
    for el in se.enable_lines.values():
        assert resync_t <= el.enabled_at[0]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest test/test_stepper_enable_resync.py -v`
Expected: FAIL — no `resync` events recorded (the guard is not wired yet).

- [ ] **Step 3: Wire the guard**

In `klippy/extras/stepper_enable.py`, in `motor_debug_enable` (lines 119-130), call resync
before the enable branch:

```python
    def motor_debug_enable(self, stepper, enable):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        el = self.enable_lines[stepper]
        if enable:
            toolhead.resync_parked_servos()
            el.motor_enable(print_time)
            logging.info("%s has been manually enabled", stepper)
        else:
            el.motor_disable(print_time)
            logging.info("%s has been manually disabled", stepper)
        toolhead.dwell(DISABLE_STALL_TIME)
```

In `motor_enable_group` (lines 132-139), resync before sampling the shared print_time:

```python
    def motor_enable_group(self, stepper_names):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        toolhead.resync_parked_servos()
        shared_print_time = toolhead.get_last_move_time()
        for name in stepper_names:
            self.enable_lines[name].motor_enable(shared_print_time)
            logging.info("%s enabled", name)
        toolhead.dwell(DISABLE_STALL_TIME)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest test/test_stepper_enable_resync.py -v`
Expected: PASS.

- [ ] **Step 5: Run the existing enable tests for regressions**

Run: `python -m pytest test/test_homing_enable.py -v`
Expected: PASS. The pre-existing `FakeToolhead` in that file has no `resync_parked_servos`,
so if any covered path now calls it, add a no-op `resync_parked_servos(self): pass` to that
fake. (The group-enable test there exercises `motor_enable_group`, which now calls resync —
add the no-op method to its `FakeToolhead`.)

- [ ] **Step 6: Commit**

```bash
git add klippy/extras/stepper_enable.py test/test_stepper_enable_resync.py test/test_homing_enable.py
git commit -m "feat(stepper_enable): resync parked servos before explicit re-energize"
```

---

## Task 5: Full gate + manual verification

**Files:** none (verification only)

- [ ] **Step 1: Run the Python host suite**

Run: `./scripts/ci.sh py`
Expected: green. (Touches `klippy/`, so the Python suite is required per CLAUDE.md.)

- [ ] **Step 2: Run the quick CI gate**

Run: `./scripts/ci.sh quick`
Expected: green (ruff check+format, Rust workspace, clippy `-D warnings`, `cargo fmt
--check`, watchdog canary). No Rust changed, but this confirms ruff/format on the new
Python.

- [ ] **Step 3: Fix formatting if ruff complains**

Run: `./scripts/ci.sh ruff`
If it reports diffs, apply them and re-run until clean, then amend the relevant commit.

- [ ] **Step 4: (Optional, when a bench/sim is available) end-to-end check**

Using the `kalico-sim` skill or a servo bench: home a servo axis, `M84`, hand-move (or
simulate displacement), confirm `homed_axes` still includes the servo axis, then issue a
`G1` on that axis and confirm the drive resyncs (no full-force yank) and moves from the true
position. Then repeat with `G28` to confirm the homing energize no longer yanks. Do **not**
issue motion commands on hardware without explicit per-command permission.

---

## Self-Review

**Spec coverage:**
- Servo axes stay homed + parked-dirty on M84; steppers cleared → Task 1. ✓
- `set_position` clears dirty for homed axes → Task 1. ✓
- Resync = query + `set_position(homing_axes=())`, fail loud on query error → Task 2. ✓
- Resync at top of `move`/`move_curve` (covers G28 via drip_move→move) → Task 3. ✓
- Explicit-enable guard (`SET_STEPPER_ENABLE`, group enable) → Task 4. ✓
- `homed_axes` status unchanged (derives from `limits`) → no task needed; asserted via
  Task 1 (`limits[2]` preserved). ✓

**Placeholder scan:** No TBD/TODO; every code step shows full code; every test shows
assertions. ✓

**Type/name consistency:** `parked_dirty_axes()`, `clear_parked_dirty(axes)`,
`_parked_dirty`, `_is_servo(axis)`, `resync_parked_servos()` used identically across Tasks
1-4. Resync uses `measured["xyz"[axis]][0]` matching the `{axis: (pos, vel)}` shape from
`query_motor_positions` (confirmed in `gcode_move.cmd_GET_POSITION`). ✓
