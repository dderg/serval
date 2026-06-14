# Vendor motors-sync as a First-Party Addon — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring motors-sync in-tree as `klippy/extras/motors_sync.py` and adapt its machine-geometry discovery to the fork's config schema, so it loads and runs on the Kalico fork.

**Architecture:** Vendor the plugin file (which already carries the bridge-backed move backend) into `klippy/extras`, preserving GPLv3 attribution. Then reroute the four mainline-schema config reads to the fork's sections — kinematics type from `[kinematics] type`, the per-axis motor list from `[kinematics] *_motors`, rail center from `[axis <name>] position_min/max`, and `rotation_distance`/`full_steps_per_rotation` from `[motor <name>]` — all at config time, with stepper *objects* matched by name at connect via `get_kinematics().get_steppers()`.

**Tech Stack:** Python (klippy host addon), the Rust motion bridge (`submit_correction_sequence`, already built), pytest host tests, kalico-sim + Trident bench for integration.

---

## Why config reads, not the kin object

`BaseKinematics.__init__` (motors_sync.py:23-31) calls `_init_axes` at **config time** but only sets `self.toolhead_kin = toolhead.get_kinematics()` in `_handle_connect`, a **connect** task. So anything `_init_axes` needs (rail center, the stepper config section for `rotation_distance`) must come from config sections, which are fully parsed when `[motors_sync]` loads. Stepper *objects* are matched later, at connect, in `_init_axes_steppers`. This is why the discovery reroutes to fork config sections rather than the runtime `kin` object, refining the spec's intent (fork-native sources) to respect load order.

## Fork config schema (verified on the bench)

```
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: motor_a, motor_a1      # corexy lane 0 (plugin axis x)
b_motors: motor_b, motor_b1      # corexy lane 1 (plugin axis y)
z_motors: motor_z, motor_z1, motor_z2

[axis x]                         # position_min/position_max live here
[motor motor_a]                  # rotation_distance, microsteps, drive
  rotation_distance: 40
[tmc5160 motor_a]                # TMC lookup already works: "tmc5160 " + get_name()
```

For `type: cartesian` the motor-list keys are `x_motors` / `y_motors` instead of `a_motors` / `b_motors`. The plugin's existing two classes already split on this: `CoreXYKinematics` uses `a_motors`/`b_motors`, `CartesianKinematics` uses `x_motors`/`y_motors`.

## File Structure

| File | Responsibility |
|------|----------------|
| `klippy/extras/motors_sync.py` | The vendored addon (new file). Discovery layer adapted; move backend already present. |
| `test/test_motors_sync_discovery.py` | Unit tests for the pure discovery helpers (new). |
| `docs/kalico-rewrite/credits.md` | Attribution entry crediting upstream (new or appended). |

---

## Task 1: Vendor the file with attribution

**Files:**
- Create: `klippy/extras/motors_sync.py` (copied from the plugin fork)
- Create/append: `docs/kalico-rewrite/credits.md`

Context: the plugin fork at `/Users/daniladergachev/Developer/motors-sync` is at commit `24cb3e0`, which already contains the bridge-backed `StepperManualMove` (the move backend). That exact file is what we vendor. It is GPLv3 (same as Kalico), so adoption only requires preserving notices, stating the change with a date, and crediting upstream. The discovery layer in this copy still reads the mainline schema — that is fixed in Tasks 2-3; this task only vendors and proves it imports.

- [ ] **Step 1: Copy the file in**

Run: `cp /Users/daniladergachev/Developer/motors-sync/motors_sync.py klippy/extras/motors_sync.py`

- [ ] **Step 2: Add the GPLv3 modification + provenance note to the header**

The file header currently reads:
```python
# Motors synchronization script
#
# Copyright (C) 2024-2026  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
```
Insert, immediately after the GPLv3 line, these lines (keep the original three lines verbatim — copyright and license must remain):
```python
#
# Vendored into Kalico and adapted to the new motion engine, 2026-06.
# Based on motors-sync by Maksim Bolgov (https://github.com/MRX8024/motors-sync),
# vendored via dderg/motors-sync @ 24cb3e0. This module intentionally couples
# to the motion engine's internals: it is a first-party addon and moves with it.
```

- [ ] **Step 3: Add the attribution doc**

Create `docs/kalico-rewrite/credits.md` (or append if it exists) with:
```markdown
# Third-party credits

## motors-sync

`klippy/extras/motors_sync.py` is vendored from
[motors-sync](https://github.com/MRX8024/motors-sync) by Maksim Bolgov,
distributed under GPLv3 (the same license as Kalico). It was vendored via the
`dderg/motors-sync` fork at commit `24cb3e0` and adapted to Kalico's motion
engine (single-motor moves via the correction bridge; geometry discovery via
the fork's config schema).
```

- [ ] **Step 4: Verify it imports**

Run: `python3 -m py_compile klippy/extras/motors_sync.py && echo COMPILE_OK`
Expected: `COMPILE_OK`. (Module-level imports are `numpy`, `from .z_tilt import ZAdjustStatus` — both present on the fork — so the file compiles. Instantiation still fails on schema, fixed next.)

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/motors_sync.py docs/kalico-rewrite/credits.md
git commit -m "vendor: bring motors-sync in-tree with GPLv3 attribution"
```

---

## Task 2: Kinematics type from the [kinematics] section

**Files:**
- Modify: `klippy/extras/motors_sync.py` — `KinematicsParser.get_kinematics` (the `class KinematicsParser` block, ~lines 315-327 in the original)

Context: `KinematicsParser.get_kinematics` picks the `CartesianKinematics` vs `CoreXYKinematics` helper. It currently reads `config.getsection('printer').get('kinematics')`, which the fork rejects — the fork declares `[kinematics] type: corexy|cartesian`. The `kin_map` is built from the helper class names (`'cartesian'`, `'corexy'`, plus a `'limitedcorexy'` alias), so the fork's `type` value maps directly.

- [ ] **Step 1: Rewrite the section read**

Replace the body of `KinematicsParser.get_kinematics`:
```python
class KinematicsParser:
    @staticmethod
    def get_kinematics(config, sync):
        kin_map = {cls.__name__.replace('Kinematics', '').lower(): cls
                   for cls in BaseKinematics.__subclasses__()}
        kin_map.update({'limitedcorexy': CoreXYKinematics})
        if not config.has_section('kinematics'):
            raise config.error("motors_sync: [kinematics] section is required")
        conf_kin = config.getsection('kinematics').get('type')
        kin_helper = kin_map.get("".join(conf_kin.split("_")))
        if kin_helper is None:
            raise config.error(f"motors_sync: Not supported "
                               f"'{conf_kin}' kinematics")
        return kin_helper(config, sync)
```

- [ ] **Step 2: Verify it compiles**

Run: `python3 -m py_compile klippy/extras/motors_sync.py && echo COMPILE_OK`
Expected: `COMPILE_OK`.

- [ ] **Step 3: Commit**

```bash
git add klippy/extras/motors_sync.py
git commit -m "feat(motors_sync): read kinematics type from [kinematics] section"
```

---

## Task 3: Per-axis discovery from fork config + connect-time steppers

**Files:**
- Modify: `klippy/extras/motors_sync.py` — `CartesianKinematics` (`get_axes_rails_center`, `_init_axes`, `_init_axes_steppers`) and `CoreXYKinematics` (`_init_axes`, `_init_axes_steppers`)
- Test: `test/test_motors_sync_discovery.py` (create)

Context: four things must move off the mainline schema. (1) Rail center: the old code reads `[stepper_<ax>] position_min/max`; the fork keeps the travel range in `[axis <name>] position_min/position_max`. (2) The stepper config section passed to `MotionAxis` (for `rotation_distance`/`full_steps_per_rotation`): old `'stepper_'+ax`, new `'motor '+<first motor name>` (the fork's `[motor motor_a]` carries those same option names, default `full_steps_per_rotation` 200). `MotionAxis` itself is unchanged — it just receives the fork section name. (3) The per-axis motor name list: from `[kinematics] a_motors/b_motors` (corexy) or `x_motors/y_motors` (cartesian). (4) Belt stepper objects: matched at connect by name from `get_kinematics().get_steppers()` (names are `motor_a`, `motor_a1`, …), replacing the `'stepper_'+name in s.get_name()` filter.

The axis-letter → fork-axis-name mapping uses `[kinematics] axis_x/axis_y` (e.g. `axis_x: x`). Both classes sync axes `x`,`y` (their `valid_axes`); plugin axis `x` ↔ `axis_x` role, `y` ↔ `axis_y` role.

- [ ] **Step 1: Write failing unit tests for the pure helpers**

Create `test/test_motors_sync_discovery.py`:
```python
from klippy.extras.motors_sync import rail_center


def test_rail_center_is_midpoint():
    assert rail_center(0.0, 350.0) == 175.0
    assert rail_center(-10.0, 10.0) == 0.0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python3 -m pytest test/test_motors_sync_discovery.py -v`
Expected: FAIL with ImportError (`rail_center` not defined).

- [ ] **Step 3: Add the module-level helper**

In `klippy/extras/motors_sync.py`, just after the module constants near the top (after `SETTLE_PAD = 0.050 ...`), add:
```python
def rail_center(position_min, position_max):
    return (position_min + position_max) / 2.0
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest test/test_motors_sync_discovery.py -v`
Expected: PASS (2 assertions in 1 test).

- [ ] **Step 5: Add shared discovery methods to `BaseKinematics`**

Add these to `BaseKinematics` (it holds `self.toolhead_kin` after connect and `self.motion_axes`). `MOTOR_LIST_KEYS` is supplied by each subclass (next steps):
```python
    def get_axis_motor_names(self, config, axis):
        kin = config.getsection('kinematics')
        names = kin.getlist(self.MOTOR_LIST_KEYS[axis], [])
        if len(names) != 2:
            raise config.error(
                f"motors_sync: axis '{axis}' needs exactly 2 motors in "
                f"[kinematics] {self.MOTOR_LIST_KEYS[axis]}, got {len(names)}")
        return names

    def get_axes_rails_center(self, config, axes):
        kin = config.getsection('kinematics')
        positions = {}
        for axis in axes:
            axis_name = kin.get('axis_' + axis)
            ax_section = config.getsection('axis ' + axis_name)
            positions[axis] = rail_center(
                ax_section.getfloat('position_min', 0.),
                ax_section.getfloat('position_max'))
        return [positions.get(a, None) for a in ['x', 'y', 'z']]

    def discover_belt_steppers(self, config, axis_name):
        names = self.get_axis_motor_names(config, axis_name)
        belt = [s for s in self.toolhead_kin.get_steppers()
                if s.get_name() in names]
        if len(belt) != 2:
            raise config.error(
                f"motors_sync: found {len(belt)} of 2 motors for axis "
                f"'{axis_name}' ({names})")
        return belt
```

- [ ] **Step 6: Rewrite `CartesianKinematics`**

Remove its old static `get_axes_rails_center`, `_init_axes`, and `_init_axes_steppers`; replace with (the shared methods now live on `BaseKinematics`, so only the key map + the two `_init_*` overrides remain):
```python
    MOTOR_LIST_KEYS = {'x': 'x_motors', 'y': 'y_motors'}

    def _init_axes(self, config, sync):
        valid_axes = ['x', 'y']
        axes = sorted([a.lower() for a in config.getlist('axes')])
        if any(a not in valid_axes for a in axes):
            raise config.error(f"motors_sync: Invalid axes '{axes}'")
        sync_pos = self.get_axes_rails_center(config, axes)
        ph_offs = self.stats_helper.get_axes_phase_offsets(axes)
        self.motion_axes.update(
            {ax: MotionAxis(config, sync, ax, axes, ph_offs.get(ax),
                'motor ' + self.get_axis_motor_names(config, ax)[0],
                sync_pos, False) for ax in axes})

    def _init_axes_steppers(self, config):
        for axis in self.motion_axes.values():
            belt = self.discover_belt_steppers(config, axis.name)
            axis.add_steppers(*belt, belt[1], None)
```

- [ ] **Step 7: Rewrite `CoreXYKinematics`**

Keep `check_common_attr` / `axes_sync` / `check_axis_drift` unchanged. Replace `_init_axes` and `_init_axes_steppers` and add the key map. The shared methods are inherited from `BaseKinematics`; only `MOTOR_LIST_KEYS` differs (`a_motors`/`b_motors`):
```python
    MOTOR_LIST_KEYS = {'x': 'a_motors', 'y': 'b_motors'}

    def _init_axes(self, config, sync):
        valid_axes = ['x', 'y']
        axes = sorted([a.lower() for a in config.getlist(
            'axes', count=len(valid_axes), default=valid_axes)])
        if any(a not in valid_axes for a in axes):
            raise config.error(f"motors_sync: Invalid axes '{axes}'")
        sync_pos = self.get_axes_rails_center(config, axes)
        ph_offs = self.stats_helper.get_axes_phase_offsets(axes)
        self.motion_axes.update(
            {ax: MotionAxis(config, sync, ax, axes, ph_offs.get(ax),
                'motor ' + self.get_axis_motor_names(config, ax)[0],
                sync_pos, False) for ax in axes})
        attr = ['microsteps', 'model_name', 'model_coeffs',
                'max_step_size', 'axes_steps_diff']
        self.check_common_attr(config, self.motion_axes.values(), attr)

    def _init_axes_steppers(self, config):
        belts = {a.name: (a, self.discover_belt_steppers(config, a.name))
                 for a in self.motion_axes.values()}
        dx = belts['x']
        dy = belts['y']
        dx[0].add_steppers(dx[1][0], dx[1][1], dx[1][1], [dy[1][0]])
        dy[0].add_steppers(dy[1][0], dy[1][1], dy[1][1], [dx[1][0]])
```

- [ ] **Step 8: Verify compile + helper tests**

Run: `python3 -m pytest test/test_motors_sync_discovery.py -v && python3 -m py_compile klippy/extras/motors_sync.py && echo OK`
Expected: tests PASS, then `OK`.

- [ ] **Step 9: Verify no mainline-schema reads remain**

Run: `grep -n "getsection('printer')\|getsection(\"printer\")\|'stepper_'\|\"stepper_\"\|position_min\|position_max" klippy/extras/motors_sync.py`
Expected: the only `position_min`/`position_max` hits are inside `get_axes_rails_center` (reading `[axis <name>]`); NO `getsection('printer')` and NO `'stepper_'+` name construction remain. If any other hit appears, it is an un-migrated read — fix it.

- [ ] **Step 10: Commit**

```bash
git add klippy/extras/motors_sync.py test/test_motors_sync_discovery.py
git commit -m "feat(motors_sync): discover axes/motors/limits from fork config schema"
```

---

## Task 4: Bench integration (deploy in-tree + user smoke test)

**Files:** none — deploy and live verification only.

Context: the addon now lives in-tree, so the Pi's symlink (`~/klipper/klippy/extras/motors_sync.py -> ~/motors-sync/motors_sync.py`) must be removed or it shadows the vendored file. This task is **not** TDD: it deploys, confirms the addon loads, then hands off the live `SYNC_MOTORS` run, which drives motion and requires explicit per-command user permission (hard rule — do not issue it autonomously).

- [ ] **Step 1: Push the fork branch**

Run: `git push`

- [ ] **Step 2: Remove the Pi symlink so the in-tree file is used**

Run: `ssh dderg@trident.local 'rm -f ~/klipper/klippy/extras/motors_sync.py && echo SYMLINK_REMOVED'`
Expected: `SYMLINK_REMOVED`. (The vendored file arrives via the branch pull in the next step.)

- [ ] **Step 3: Deploy the branch (host scope) to the Trident**

Run: `~/.claude/skills/trident-bench/scripts/flash-trident.sh motors-sync host`
Expected: success gate passes (`klipper` active). Run in background/subagent (multi-minute). This pulls `origin/motors-sync` (bringing `klippy/extras/motors_sync.py`), rebuilds the `.so`, restarts klippy.

- [ ] **Step 4: Confirm the addon loads (no motion)**

Use the query-logs skill against the Trident (VL on the Pi) to check the latest session for `level:error` and for `Traceback` / `Unknown command` / config errors mentioning `motors_sync` or `kinematics`. Confirm no load error and that `[motors_sync]` instantiated.
Expected: no errors; the prior `[printer] kinematics` / schema failures are gone.

- [ ] **Step 5: Hand off the live test to the user**

STOP and ask the user to run (motion — their go-ahead required):
```
G28
SYNC_MOTORS
```
Watch for: it runs (no schema/Unknown-command error); the buzz is one continuous shake on one belt motor (no inter-swing pause); the partner motor stays put; axis position unchanged; the magnitude converges across repeats. Do not issue `SYNC_MOTORS` autonomously.

---

## Self-Review

**Spec coverage:**
- Vendor + GPLv3 attribution (keep notices, state change+date, credit) → Task 1. ✓
- Disabled by default (no `[motors_sync]` = not loaded) → inherent; no code. ✓
- Discovery reads fork-native sources, no frozen public API → Tasks 2-3 (config sections + connect-time `get_steppers()`); the "intentionally couples to internals" note → Task 1 Step 2. ✓
- Move backend unchanged (bridge-backed `StepperManualMove`) → vendored as-is in Task 1; not modified. ✓
- Fail-loud on an axis with no matching motors/lane → Task 3 (`config.error` in `get_axis_motor_names` / `_init_axes_steppers`). ✓
- Testing: host unit (helpers) → Task 3; bench → Task 4. ✓
- Retire `dderg/motors-sync` to provenance → recorded in the Task 1 header note + credits doc. ✓

**Placeholder scan:** no TBD/TODO; every code step shows full code; every run step has a command + expected output.

**Type/signature consistency:** `MotionAxis(config, sync, ax, axes, ph_off, <stepper-section-name>, sync_pos, multi)` — the 6th arg is now `'motor '+name` in both kinematics classes (was `'stepper_'+ax`), matching `MotionAxis.__init__`'s `config.getsection(main_stepper)`. `rail_center(min, max)` is the only module helper, defined in Task 3 Step 3 and used by `get_axes_rails_center`. `get_axis_motor_names` / `get_axes_rails_center` / `discover_belt_steppers` live on `BaseKinematics`; `MOTOR_LIST_KEYS` is defined per subclass and read via `self` in those shared methods. `add_steppers(enable, step, buzz, conflict)` call shapes match the original signature.

**Refinement vs spec:** the spec's discovery table named the runtime `kin` object (`kind`/`rails`/`get_status`); the plan reads the equivalent fork **config sections** at config time because `_init_axes` runs before `toolhead_kin` is set. Stepper objects are still sourced from `get_kinematics().get_steppers()` at connect. Same fork-native intent, load-order-correct.
