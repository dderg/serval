# motors-sync Plugin Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the motors-sync plugin run on the Kalico fork by driving motors through the new correction primitive instead of the deleted host-stepping path.

**Architecture:** Two coordinated pieces in two repos. (A) In the Kalico fork, implement the stubbed `force_move.manual_move` as a thin scalar seam over `submit_correction_sequence` — the general single-stepper API any plugin can call. (B) In the motors-sync fork, rewrite `StepperManualMove` to call `submit_correction_sequence` directly (single nudges and the gapless buzz both flow through it) and delete the now-dead trapq/compat machinery.

**Tech Stack:** Python (klippy host + the plugin), the Rust motion bridge (`submit_correction_sequence`, already built), pytest host tests, kalico-sim and the Trident bench for integration.

---

## Repos & paths

- **Kalico fork (this worktree):** `/Users/daniladergachev/Developer/kalico/.worktrees/motors-sync`
- **motors-sync fork:** `/Users/daniladergachev/Developer/motors-sync`

Commits land in the repo whose files the task touches. The fork uses no
`Co-Authored-By`/Claude trailer in commit messages.

## File Structure

| File | Repo | Responsibility |
|------|------|----------------|
| `klippy/extras/force_move.py` | fork | Implement `manual_move` (scalar) over the bridge. |
| `test/test_force_move_manual_move.py` | fork (new) | Unit-test the seam with fakes. |
| `motors_sync.py` | plugin | Rewrite `StepperManualMove`; delete `DummyPrinterMotionQueuing`, `chelper`, `calc_move_time`. |

The seam (A) and the plugin (B) are independent code-wise — the plugin calls
the bridge directly, not the seam — so they can be built in either order. The
seam is built first because it is cleanly unit-testable and answers the
"support other plugins" goal on its own.

---

## Task 1: `force_move.manual_move` seam (Kalico fork)

**Files:**
- Modify: `klippy/extras/force_move.py:41-42` (the `PHASE5_GATE` stub)
- Test: `test/test_force_move_manual_move.py` (create)

Context: `ForceMove` lives at `klippy/extras/force_move.py`. The `Motion`
toolhead shim exposes `get_motor_binding(name) -> (mcu_id, axis_idx,
motor_idx)` (`klippy/motion.py:252`), `get_bridge()` (`klippy/motion.py:249`),
and `get_max_axis_accel(axis_idx)` (`klippy/motion.py:261`). `get_bridge()`
returns the bridge wrapper whose `submit_correction_sequence(mcu_id, axis_idx,
motor_idx, segments, speed, accel) -> duration_secs` is at
`klippy/motion_bridge.py:417`. The working reference for the call pattern is
`klippy/extras/motor_adjust.py:35-44`.

- [ ] **Step 1: Write the failing test**

Create `test/test_force_move_manual_move.py`:

```python
from klippy.extras.force_move import ForceMove


class FakeBridge:
    def __init__(self):
        self.calls = []

    def submit_correction_sequence(
        self, mcu_id, axis_idx, motor_idx, segments, speed, accel
    ):
        self.calls.append(
            (mcu_id, axis_idx, motor_idx, list(segments), speed, accel)
        )
        return 0.25


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self, short=False):
        return self._name


class FakeToolhead:
    def __init__(self, bridge, binding, max_accel):
        self._bridge = bridge
        self._binding = binding
        self._max_accel = max_accel

    def get_motor_binding(self, name):
        return self._binding

    def get_bridge(self):
        return self._bridge

    def get_max_axis_accel(self, axis_idx):
        return self._max_accel


class FakePrinter:
    def __init__(self, toolhead):
        self._toolhead = toolhead

    def lookup_object(self, name, default=None):
        return {"toolhead": self._toolhead}.get(name, default)


def make_force_move(bridge, binding=(7, 1, 0), max_accel=3000.0):
    fm = ForceMove.__new__(ForceMove)
    fm.printer = FakePrinter(FakeToolhead(bridge, binding, max_accel))
    return fm


def test_manual_move_calls_bridge_with_single_segment():
    bridge = FakeBridge()
    fm = make_force_move(bridge, binding=(7, 1, 0))
    dur = fm.manual_move(FakeStepper("stepper_x1"), 0.4, 12.0, 800.0)
    assert dur == 0.25
    assert bridge.calls == [(7, 1, 0, [0.4], 12.0, 800.0)]


def test_manual_move_substitutes_machine_accel_when_unset():
    bridge = FakeBridge()
    fm = make_force_move(bridge, binding=(7, 1, 0), max_accel=3000.0)
    fm.manual_move(FakeStepper("stepper_x1"), 0.4, 12.0)
    assert bridge.calls[0][5] == 3000.0


def test_manual_move_accepts_stepper_name_string():
    bridge = FakeBridge()
    fm = make_force_move(bridge)
    fm.manual_move("stepper_x1", 0.4, 12.0, 800.0)
    assert bridge.calls[0][:3] == (7, 1, 0)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python -m pytest test/test_force_move_manual_move.py -v`
Expected: FAIL — `manual_move` raises `command_error` (`PHASE5_GATE`), so
`test_manual_move_calls_bridge_with_single_segment` errors instead of asserting.

- [ ] **Step 3: Implement the seam**

Replace the stub body at `klippy/extras/force_move.py:41-42`:

```python
    def manual_move(self, stepper, dist, speed, accel=0.0):
        toolhead = self.printer.lookup_object("toolhead")
        name = stepper.get_name() if hasattr(stepper, "get_name") else stepper
        mcu_id, axis_idx, motor_idx = toolhead.get_motor_binding(name)
        if accel <= 0.0:
            accel = toolhead.get_max_axis_accel(axis_idx)
        bridge = toolhead.get_bridge()
        return bridge.submit_correction_sequence(
            mcu_id, axis_idx, motor_idx, [dist], speed, accel
        )
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `python -m pytest test/test_force_move_manual_move.py -v`
Expected: PASS — all three tests green.

- [ ] **Step 5: Run the broader host gate**

Run: `./scripts/ci.sh py`
Expected: PASS (no regression in the host suite).

- [ ] **Step 6: Commit (Kalico fork)**

```bash
git add klippy/extras/force_move.py test/test_force_move_manual_move.py
git commit -m "feat(force_move): implement manual_move on the correction bridge"
```

---

## Task 2: Rewrite `StepperManualMove` onto the bridge (motors-sync fork)

**Files:**
- Modify: `/Users/daniladergachev/Developer/motors-sync/motors_sync.py`
  - Remove `import chelper` (line 10)
  - Delete `DummyPrinterMotionQueuing` (lines ~330-359)
  - Rewrite `StepperManualMove` (lines ~362-424)
  - Add a `SETTLE_PAD` constant near the other module constants (~line 13-15)

Context: every single-motor move in the plugin funnels through
`StepperManualMove.manual_move(mcu_stepper, moves)` — `step_move` passes
`[dist]`, the phase-restore nudges pass `[dist]`, and `buzz_move` passes the
full ~50-element fading-oscillation list. The new body resolves the motor
binding once and hands the whole `moves` list to `submit_correction_sequence`,
so single nudges and the gapless buzz both flow through one call. The plugin's
`steppers_enable` is unchanged (it already uses
`stepper_enable.lookup_enable(name).motor_enable(print_time)`, which the fork
keeps). `self.toolhead.max_velocity` / `max_accel` are attributes on the fork's
`Motion` shim (`klippy/motion.py` `get_status`). The reference wait pattern is
`klippy/extras/motor_adjust.py:35-47`.

This repo has no unit-test harness (the file list is `motors_sync.py`,
`install.sh`, wiki); the plugin needs a full klippy host to load. The automated
gate here is therefore a syntax compile plus a dead-reference grep; the
functional gate is the Trident bench in Task 3.

- [ ] **Step 1: Add the settle-pad constant**

After `MOTOR_STALL_TIME = 0.100` (motors_sync.py ~line 15), add:

```python
SETTLE_PAD = 0.050              # Wall-clock pad after a correction completes
```

- [ ] **Step 2: Remove the dead `chelper` import**

Delete line 10 (`import chelper`). Leave the rest of the import block intact:

```python
import os, logging, traceback, itertools, csv, ast, importlib.util
import threading, multiprocessing
from datetime import datetime
import numpy as np
from .z_tilt import ZAdjustStatus
```

- [ ] **Step 3: Delete `DummyPrinterMotionQueuing`**

Remove the entire class (the block beginning with the comment
`# Plug class for the Klipper version before add "motion_queuing"` through the
end of `wipe_trapq`). It has no callers once `StepperManualMove` is rewritten.

- [ ] **Step 4: Rewrite `StepperManualMove`**

Replace the whole class (from `class StepperManualMove:` through the end of
`manual_move`) with:

```python
class StepperManualMove:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.stepper_en = printer.load_object(config, 'stepper_enable')
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = min(self.toolhead.max_velocity, 100)
        self.travel_accel = min(self.toolhead.max_accel, 5000)

    def steppers_enable(self, mcu_steppers, mode):
        ptime = self.toolhead.get_last_move_time()
        did_change = False
        for mcu_stepper in mcu_steppers:
            el = self.stepper_en.lookup_enable(mcu_stepper.get_name())
            if el.is_motor_enabled() == mode:
                continue
            if mode:
                el.motor_enable(ptime)
            else:
                el.motor_disable(ptime)
            did_change = True
        return did_change

    def manual_move(self, mcu_stepper, moves):
        segments = [m for m in moves if abs(m) >= 0.00001]
        if not segments:
            return
        name = mcu_stepper.get_name()
        mcu_id, axis_idx, motor_idx = self.toolhead.get_motor_binding(name)
        bridge = self.toolhead.get_bridge()
        reactor = self.printer.get_reactor()
        start = reactor.monotonic()
        duration = bridge.submit_correction_sequence(
            mcu_id, axis_idx, motor_idx, segments,
            self.travel_speed, self.travel_accel)
        deadline = start + duration + SETTLE_PAD
        while reactor.monotonic() < deadline:
            reactor.pause(reactor.monotonic() + 0.01)
```

- [ ] **Step 5: Verify it compiles**

Run: `cd /Users/daniladergachev/Developer/motors-sync && python -m py_compile motors_sync.py && echo COMPILE_OK`
Expected: prints `COMPILE_OK` (no syntax error).

- [ ] **Step 6: Verify no dead references remain**

Run: `cd /Users/daniladergachev/Developer/motors-sync && grep -n "calc_move_time\|DummyPrinterMotionQueuing\|motion_queuing\|trapq\|generate_steps\|chelper\|stepper_kin" motors_sync.py || echo NONE_LEFT`
Expected: prints `NONE_LEFT` (every removed symbol is gone).

- [ ] **Step 7: Commit (motors-sync fork)**

```bash
cd /Users/daniladergachev/Developer/motors-sync
git add motors_sync.py
git commit -m "Port StepperManualMove to the Kalico correction bridge"
```

---

## Task 3: Bench integration (deploy + user-driven smoke test)

**Files:** none — deploy and live verification only.

Context: the plugin loads only inside a full klippy host with an
accelerometer, so the functional gate is the Trident, which already has
motors-sync installed and an accel chip. This task is **not** TDD: it deploys
the two changes, then hands off to the user for the live `SYNC_MOTORS` run,
because `SYNC_MOTORS` drives motion and motion commands require explicit
per-command user permission (hard rule — do not issue it autonomously).

- [ ] **Step 1: Push both repos**

```bash
# Kalico fork (seam) — push the feature branch
cd /Users/daniladergachev/Developer/kalico/.worktrees/motors-sync && git push
# motors-sync fork (adapter)
cd /Users/daniladergachev/Developer/motors-sync && git push
```

- [ ] **Step 2: Deploy the fork seam to the Trident**

The `force_move.py` change is pure host Python (no MCU reflash needed). Use the
trident-bench flash script `host` scope, which pulls the branch on the Pi and
restarts klippy:

Run: `~/.claude/skills/trident-bench/scripts/flash-trident.sh motors-sync host`
Expected: success gate passes (`klipper` active). Run it in the background or a
subagent (multi-minute build output).

- [ ] **Step 3: Update the plugin on the Pi**

The plugin is installed from its own repo checkout on the Pi. Pull the
rewritten `motors_sync.py` there (the plugin repo's remote is `dderg/motors-sync`):

```bash
ssh dderg@trident.local 'cd ~/motors-sync 2>/dev/null && git pull --ff-only || echo PLUGIN_REPO_PATH_UNKNOWN'
```

If the path differs, locate the installed copy: `ssh dderg@trident.local 'ls -l ~/klipper/klippy/extras/motors_sync.py'` (it is typically a symlink into the plugin repo). Restart klippy after updating: `ssh dderg@trident.local 'sudo systemctl restart klipper'`.

- [ ] **Step 4: Confirm the plugin loads (no motion)**

Use the query-logs skill to confirm klippy started cleanly and `[motors_sync]`
instantiated without the old `force_move` / `calc_move_time` `AttributeError`.
This reads logs only — no motion.

Expected: no `calc_move_time` / `AttributeError` at load; `SYNC_MOTORS` is a
registered command.

- [ ] **Step 5: Hand off to the user for the live smoke test**

STOP and ask the user to run the live test (motion — requires their explicit
go-ahead). Present exactly what to run and what to watch:

- Run `SYNC_MOTORS` on the bench.
- Watch for: the plugin loads; the buzz is one continuous shake on one belt
  motor with no inter-swing pause; the partner motor stays put; the reported
  axis position is unchanged; and the sync converges (magnitude falls across
  repeats rather than growing).

Do not issue `SYNC_MOTORS` autonomously.

---

## Self-Review

**Spec coverage:**
- Spec "A. Fork seam" → Task 1. ✓
- Spec "B. Plugin adapter" (delete `DummyPrinterMotionQueuing`,
  `chelper`/trapq, `calc_move_time`; rewrite `manual_move`; keep
  `steppers_enable`) → Task 2, Steps 2-4. ✓
- Spec timeline contract (reactor-wait anchored at `start` before the call,
  not `toolhead.dwell`) → Task 2, Step 4 `manual_move`. ✓
- Spec validation: sub-epsilon `moves` → no-op (Task 2 `manual_move` filter);
  unknown stepper → `config_error` (inherited from `get_motor_binding`, no
  extra code). ✓
- Spec testing: seam unit test → Task 1; bench buzz/convergence → Task 3. ✓
- Spec out-of-scope (`FORCE_MOVE`/`STEPPER_BUZZ` G-codes, sensor paths) →
  correctly untouched. ✓

**Placeholder scan:** No TBD/TODO; every code step shows full code; every run
step shows the command and expected output.

**Type/signature consistency:** `submit_correction_sequence(mcu_id, axis_idx,
motor_idx, segments, speed, accel) -> duration` used identically in the seam
(Task 1 Step 3) and the adapter (Task 2 Step 4). `get_motor_binding(name) ->
(mcu_id, axis_idx, motor_idx)` and `get_bridge()` used consistently. `SETTLE_PAD`
defined in Task 2 Step 1, used in Step 4.
