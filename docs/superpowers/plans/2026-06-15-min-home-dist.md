# `min_home_dist` Safety Rehome Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `G28` consult the per-axis `min_home_dist`: if the first endstop trigger comes too soon, back off by `min_home_dist`, re-approach over a bounded distance, and fail loudly if the second trigger is still too short.

**Architecture:** All logic is host-side in `klippy/extras/homing.py`. The Rust bridge/MCU are untouched — the bridge already returns `trip_pos`/`final_pos` from `trip_move`. We extract three pure-ish seams (`_trigger_too_early`, `_run_homing_attempts`, `_commit_and_seed`) so the semantics are unit-testable with injected trigger distances and fake toolheads. The rehome works in the **real motion frame** (back off relative to `trip_pos`, declare the configured `trigger_height` only after a validated trip). The EtherCAT seed stays after the final retract.

**Tech Stack:** Python 3.x, pytest (`test/` for host unit tests, `tools/sim_klippy/tests/` for ELF integration), `structured_log.event` for logging, `danger_options` for the tolerance.

**Spec:** `docs/superpowers/specs/2026-06-15-min-home-dist-design.md`

---

## File Structure

- **Modify** `klippy/extras/homing.py`:
  - Add import `from klippy.extras.danger_options import get_danger_options`.
  - Add module function `_trigger_too_early(traveled, min_home_dist, tolerance)`.
  - Add module function `_run_homing_attempts(...)` (first approach + optional one-shot rehome).
  - Add module function `_commit_and_seed(...)` (set homed position, retract, EtherCAT seed).
  - Add method `Homing._guarded_approach(...)` (wraps `_run_servo_guarded_trip` + `trip_move`).
  - Rewrite the body of `Homing._home_axis` to call the three seams. Remove the now-inlined approach/commit/retract/seed code and the obsolete TODO at the end of `trip_move`.
- **Create** `test/test_homing_min_dist.py`: unit tests for the decision predicate, the rehome orchestration, and the seed ordering.
- **Modify** `tools/sim_klippy/tests/test_homing_lag_repro.py`: split overrides so the timing test uses `min_home_dist=0`; repurpose the rehome test to the deterministic held-high path.

---

## Task 1: Decision predicate `_trigger_too_early`

**Files:**
- Modify: `klippy/extras/homing.py` (add module function near the other `_`-helpers, e.g. after `_homed_axis_position`, ~line 78)
- Test: `test/test_homing_min_dist.py` (create)

- [ ] **Step 1: Write the failing test**

Create `test/test_homing_min_dist.py`:

```python
import pytest

from klippy.extras import homing as homing_mod


def test_trigger_too_early_short_with_margin():
    assert homing_mod._trigger_too_early(2.0, 15.0, 0.5) is True


def test_trigger_too_early_at_tolerance_edge_is_early():
    # 15 - 14.5 = 0.5 >= 0.5 -> early
    assert homing_mod._trigger_too_early(14.5, 15.0, 0.5) is True


def test_trigger_too_early_within_tolerance_band_not_early():
    # 15 - 14.6 = 0.4 < 0.5 -> not early
    assert homing_mod._trigger_too_early(14.6, 15.0, 0.5) is False


def test_trigger_too_early_beyond_min_not_early():
    assert homing_mod._trigger_too_early(100.0, 15.0, 0.5) is False


def test_trigger_too_early_disabled_when_min_zero():
    assert homing_mod._trigger_too_early(0.0, 0.0, 0.5) is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_homing_min_dist.py -v`
Expected: FAIL — `AttributeError: module 'klippy.extras.homing' has no attribute '_trigger_too_early'`

- [ ] **Step 3: Write minimal implementation**

In `klippy/extras/homing.py`, after `_homed_axis_position` (around line 78), add:

```python
def _trigger_too_early(traveled, min_home_dist, tolerance):
    if min_home_dist <= 0.0:
        return False
    return traveled < min_home_dist and (min_home_dist - traveled) >= tolerance
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest test/test_homing_min_dist.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/homing.py test/test_homing_min_dist.py
git commit -m "feat(homing): add min_home_dist early-trigger predicate"
```

---

## Task 2: Extract `_commit_and_seed` tail + lock seed ordering

This extracts the existing post-trip commit/retract/seed code into a module function with no behavior change, then adds a test that pins the EtherCAT seed to the **post-final-retract** position.

**Files:**
- Modify: `klippy/extras/homing.py` (add `_commit_and_seed`; call it from `_home_axis`)
- Test: `test/test_homing_min_dist.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_homing_min_dist.py`:

```python
class FakeToolhead:
    def __init__(self, pos):
        self.pos = list(pos)
        self.events = []

    def get_position(self):
        return list(self.pos)

    def set_position(self, newpos, homing_axes=None):
        self.pos = list(newpos)
        self.events.append(("set_position", list(newpos)))

    def move(self, newpos, speed):
        self.pos = list(newpos)
        self.events.append(("move", list(newpos), speed))

    def wait_moves(self):
        self.events.append(("wait_moves",))


class FakeBridge:
    def __init__(self):
        self.finalize_calls = []

    def finalize_homed_axis(self, handle, axis, pos):
        self.finalize_calls.append((handle, axis, pos))


def _hi(min_home_dist=15.0, speed=50.0, retract_speed=25.0, retract_dist=5.0):
    from klippy.rail import HomingInfo

    return HomingInfo(
        speed=speed,
        position_endstop=20.0,
        retract_speed=retract_speed,
        retract_dist=retract_dist,
        positive_dir=True,
        second_homing_speed=speed,
        use_sensorless_homing=False,
        min_home_dist=min_home_dist,
        accel=None,
    )


def test_commit_and_seed_seeds_post_retract_position():
    axis = 0
    toolhead = FakeToolhead([0.0, 0.0, 0.0])
    bridge = FakeBridge()
    hi = _hi(retract_dist=5.0)
    homing_mod._commit_and_seed(
        toolhead, bridge, axis, 1.0, hi,
        trip_pos=[20.0, 0.0, 0.0], final_pos=[20.0, 0.0, 0.0],
        trigger_height=20.0, provider=None, servo_handle="h",
    )
    # homed set to trigger_height (20) then retracted by 5 -> 15
    assert toolhead.get_position()[axis] == 15.0
    # seed fired once, carrying the POST-retract coordinate (15), not 20
    assert bridge.finalize_calls == [("h", 0, 15.0)]


def test_commit_and_seed_no_servo_does_not_seed():
    toolhead = FakeToolhead([0.0, 0.0, 0.0])
    bridge = FakeBridge()
    homing_mod._commit_and_seed(
        toolhead, bridge, 0, 1.0, _hi(),
        trip_pos=[20.0, 0.0, 0.0], final_pos=[20.0, 0.0, 0.0],
        trigger_height=20.0, provider=None, servo_handle=None,
    )
    assert bridge.finalize_calls == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_homing_min_dist.py -k commit_and_seed -v`
Expected: FAIL — `AttributeError: ... has no attribute '_commit_and_seed'`

- [ ] **Step 3: Write the implementation**

In `klippy/extras/homing.py`, add this module function (near `_trigger_too_early`):

```python
def _commit_and_seed(
    toolhead, bridge, axis, direction, hi, trip_pos, final_pos,
    trigger_height, provider, servo_handle,
):
    overshoot = final_pos[axis] - trip_pos[axis]
    newpos = list(toolhead.get_position())
    newpos[axis] = _homed_axis_position(
        provider, axis, trip_pos, final_pos, trigger_height
    )
    toolhead.set_position(newpos, homing_axes=[axis])
    structured_log.event(
        "homing",
        "axis_homed",
        msg="homing: %s trigger=%.4f overshoot=%+.4f set %s=%.4f"
        % ("XYZ"[axis], trigger_height, overshoot, "XYZ"[axis], newpos[axis]),
        axis="XYZ"[axis],
        trigger_height=trigger_height,
        overshoot=overshoot,
        homed_position=newpos[axis],
    )
    if hi.retract_dist:
        retractpos = list(toolhead.get_position())
        retractpos[axis] -= direction * hi.retract_dist + overshoot
        toolhead.move(retractpos, hi.retract_speed)
        toolhead.wait_moves()
    if servo_handle is not None:
        bridge.finalize_homed_axis(
            servo_handle, axis, toolhead.get_position()[axis]
        )
```

- [ ] **Step 4: Wire it into `_home_axis` (replace the inlined tail)**

In `klippy/extras/homing.py` `_home_axis`, replace the block that currently runs from `overshoot = final_pos[axis] - trip_pos[axis]` (≈line 300) through the `bridge.finalize_homed_axis(...)` call (≈line 330) with a single call. The `try:` body becomes:

```python
        self._set_homing_current(toolhead, rail, pre_homing=True)
        try:
            provider = entry["provider"]
            trip_pos, final_pos = _run_servo_guarded_trip(
                gcmd,
                bridge,
                axis,
                stepper_enable,
                rail,
                servo_handle,
                servo_limits,
                lambda: self.trip_move(
                    gcmd, toolhead, bridge, axis, direction, speed,
                    max_travel, entry,
                ),
            )
            _commit_and_seed(
                toolhead, bridge, axis, direction, hi, trip_pos, final_pos,
                trigger_height, provider, servo_handle,
            )
            _check_servo_drive_fault(gcmd, bridge, axis, servo_handle)
        except BaseException:
```

(Leave the `except BaseException:` / `else:` current-restore blocks unchanged.)

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m pytest test/test_homing_min_dist.py -v`
Expected: PASS (7 passed)

Run: `python -m pytest test/test_servo_homing.py test/test_rail.py -v`
Expected: PASS (no regression in existing homing-adjacent tests)

- [ ] **Step 6: Commit**

```bash
git add klippy/extras/homing.py test/test_homing_min_dist.py
git commit -m "refactor(homing): extract _commit_and_seed; lock seed after final retract"
```

---

## Task 3: Rehome orchestration `_run_homing_attempts` + `_guarded_approach`

**Files:**
- Modify: `klippy/extras/homing.py` (add `_guarded_approach` method, `_run_homing_attempts` module function; rewire `_home_axis`; add the danger_options import; drop the obsolete TODO in `trip_move`)
- Test: `test/test_homing_min_dist.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_homing_min_dist.py`:

```python
class FakeGcmd:
    error = RuntimeError


def _approach_script(toolhead, axis, traveled_per_call, overshoot=0.0):
    state = {"i": 0}
    calls = []

    def approach(speed, max_travel):
        i = state["i"]
        state["i"] += 1
        calls.append((speed, max_travel))
        cur = toolhead.get_position()
        trip = list(cur)
        trip[axis] = cur[axis] + traveled_per_call[i]
        final = list(trip)
        final[axis] = trip[axis] + overshoot
        return trip, final

    return approach, calls


def test_no_rehome_when_first_travel_exceeds_min():
    axis = 0
    toolhead = FakeToolhead([0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [100.0])
    trip, final = homing_mod._run_homing_attempts(
        FakeGcmd(), toolhead, axis, 1.0, _hi(min_home_dist=15.0),
        trigger_height=20.0, provider=None, first_max_travel=200.0,
        tolerance=0.5, approach=approach,
    )
    assert len(calls) == 1
    assert trip[axis] == 100.0


def test_rehome_then_legit_returns_second_trip():
    axis = 0
    toolhead = FakeToolhead([0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [2.0, 20.0])
    trip, final = homing_mod._run_homing_attempts(
        FakeGcmd(), toolhead, axis, 1.0, _hi(min_home_dist=15.0),
        trigger_height=20.0, provider=None, first_max_travel=200.0,
        tolerance=0.5, approach=approach,
    )
    assert len(calls) == 2
    # re-approach bound is 2 * min_home_dist
    assert calls[1][1] == 30.0
    # back-off is in the real motion frame, relative to trip_pos (2.0):
    # 2.0 - direction(1) * 15 = -13.0, at retract_speed (25)
    assert ("move", [-13.0, 0.0, 0.0], 25.0) in toolhead.events
    # second trip travelled from the back-off point (-13) by 20 -> -13 + 20 = 7
    assert trip[axis] == 7.0


def test_rehome_then_still_early_raises():
    axis = 0
    toolhead = FakeToolhead([0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [2.0, 1.0])
    with pytest.raises(RuntimeError, match="early homing trigger"):
        homing_mod._run_homing_attempts(
            FakeGcmd(), toolhead, axis, 1.0, _hi(min_home_dist=15.0),
            trigger_height=20.0, provider=None, first_max_travel=200.0,
            tolerance=0.5, approach=approach,
        )
    assert len(calls) == 2


def test_min_home_dist_zero_never_rehomes():
    axis = 0
    toolhead = FakeToolhead([0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [0.0])
    homing_mod._run_homing_attempts(
        FakeGcmd(), toolhead, axis, 1.0, _hi(min_home_dist=0.0),
        trigger_height=20.0, provider=None, first_max_travel=200.0,
        tolerance=0.5, approach=approach,
    )
    assert len(calls) == 1
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest test/test_homing_min_dist.py -k "rehome or zero_never" -v`
Expected: FAIL — `AttributeError: ... has no attribute '_run_homing_attempts'`

- [ ] **Step 3: Add `_run_homing_attempts` module function**

In `klippy/extras/homing.py` (near `_commit_and_seed`):

```python
def _run_homing_attempts(
    gcmd, toolhead, axis, direction, hi, trigger_height, provider,
    first_max_travel, tolerance, approach,
):
    start_pos = toolhead.get_position()
    trip_pos, final_pos = approach(hi.speed, first_max_travel)
    traveled = abs(trip_pos[axis] - start_pos[axis])
    needs_rehome = _trigger_too_early(traveled, hi.min_home_dist, tolerance)
    structured_log.event(
        "homing",
        "needs_rehome",
        msg="homing: %s needs rehome: %s (traveled=%.4f min_home_dist=%.4f)"
        % ("XYZ"[axis], needs_rehome, traveled, hi.min_home_dist),
        axis="XYZ"[axis],
        needs_rehome=needs_rehome,
        traveled=traveled,
        min_home_dist=hi.min_home_dist,
    )
    if not needs_rehome:
        return trip_pos, final_pos
    # Real motion frame: the suspect trip is NOT the validated endstop, so
    # inform the toolhead of its actual halt position and back off relative
    # to the trigger. trigger_height is declared only after a valid trip.
    haltpos = list(toolhead.get_position())
    haltpos[axis] = final_pos[axis]
    toolhead.set_position(haltpos, homing_axes=[axis])
    backoff = list(toolhead.get_position())
    backoff[axis] = trip_pos[axis] - direction * hi.min_home_dist
    toolhead.move(backoff, hi.retract_speed)
    toolhead.wait_moves()
    start_pos = toolhead.get_position()
    trip_pos, final_pos = approach(hi.speed, 2.0 * hi.min_home_dist)
    traveled = abs(trip_pos[axis] - start_pos[axis])
    if _trigger_too_early(traveled, hi.min_home_dist, tolerance):
        raise gcmd.error(
            "%s early homing trigger: endstop tripped after only %.2fmm on "
            "re-approach (min_home_dist %.2fmm) — false trigger or "
            "stuck/miswired endstop"
            % ("XYZ"[axis], traveled, hi.min_home_dist)
        )
    return trip_pos, final_pos
```

- [ ] **Step 4: Run the orchestration tests to verify they pass**

Run: `python -m pytest test/test_homing_min_dist.py -v`
Expected: PASS (all, now 11)

- [ ] **Step 5: Add the danger_options import and `_guarded_approach`, rewire `_home_axis`**

At the top of `klippy/extras/homing.py`, add the import next to the existing ones:

```python
from klippy.extras.danger_options import get_danger_options
```

Add the method to the `Homing` class (e.g. just before `_home_axis`):

```python
    def _guarded_approach(
        self, gcmd, toolhead, bridge, axis, direction, speed, max_travel,
        entry, stepper_enable, rail, servo_handle, servo_limits,
    ):
        return _run_servo_guarded_trip(
            gcmd, bridge, axis, stepper_enable, rail, servo_handle,
            servo_limits,
            lambda: self.trip_move(
                gcmd, toolhead, bridge, axis, direction, speed, max_travel,
                entry,
            ),
        )
```

Replace the `try:` body of `_home_axis` (the version from Task 2 Step 4) with:

```python
        self._set_homing_current(toolhead, rail, pre_homing=True)
        try:
            provider = entry["provider"]
            tolerance = get_danger_options().homing_elapsed_distance_tolerance

            def approach(spd, mt):
                return self._guarded_approach(
                    gcmd, toolhead, bridge, axis, direction, spd, mt, entry,
                    stepper_enable, rail, servo_handle, servo_limits,
                )

            trip_pos, final_pos = _run_homing_attempts(
                gcmd, toolhead, axis, direction, hi, trigger_height, provider,
                max_travel, tolerance, approach,
            )
            _commit_and_seed(
                toolhead, bridge, axis, direction, hi, trip_pos, final_pos,
                trigger_height, provider, servo_handle,
            )
            _check_servo_drive_fault(gcmd, bridge, axis, servo_handle)
        except BaseException:
```

(The `except BaseException:` / `else:` current-restore blocks stay unchanged.)

- [ ] **Step 6: Drop the obsolete TODO in `trip_move`**

In `klippy/extras/homing.py` `trip_move`, remove the now-resolved comment (the early-trigger guard now lives in `_run_homing_attempts`):

```python
        trip_pos, final_pos, trip_clock = result
        _verify_latched_trip(gcmd, axis, endstop, trip_clock)
        return trip_pos, final_pos
```

- [ ] **Step 7: Run unit + adjacent tests + lint**

Run: `python -m pytest test/test_homing_min_dist.py test/test_servo_homing.py test/test_rail.py -v`
Expected: PASS

Run: `./scripts/ci.sh ruff`
Expected: PASS (clean)

- [ ] **Step 8: Commit**

```bash
git add klippy/extras/homing.py test/test_homing_min_dist.py
git commit -m "feat(homing): min_home_dist safety rehome with bounded re-approach"
```

---

## Task 4: Rework the ELF sim integration tests

**Files:**
- Modify: `tools/sim_klippy/tests/test_homing_lag_repro.py`

- [ ] **Step 1: Split the overrides so the timing test disables `min_home_dist`**

In `tools/sim_klippy/tests/test_homing_lag_repro.py`, keep the existing `SIM_OVERRIDES` (with `stepper_x` `min_home_dist=15`) for the rehome test, and add a second dict for the timing test. Replace the single `SIM_OVERRIDES` definition with:

```python
SIM_OVERRIDES = {
    "stepper_x.config_set": {
        "endstop_pin": "^gpiochip0/gpio200",
        "use_sensorless_homing": "False",
        "homing_retract_dist": "5",
        "min_home_dist": "15",
        "position_endstop": "20",
        "position_max": "20",
    },
    "stepper_y.config_set": {
        "endstop_pin": "^gpiochip0/gpio201",
        "use_sensorless_homing": "False",
        "homing_retract_dist": "0",
        "min_home_dist": "0",
        "position_endstop": "20",
        "position_max": "20",
    },
}

TIMING_OVERRIDES = {
    "stepper_x.config_set": {
        "endstop_pin": "^gpiochip0/gpio200",
        "use_sensorless_homing": "False",
        "homing_retract_dist": "5",
        "min_home_dist": "0",
        "position_endstop": "20",
        "position_max": "20",
    },
    "stepper_y.config_set": dict(SIM_OVERRIDES["stepper_y.config_set"]),
}
```

- [ ] **Step 2: Point the timing test at `TIMING_OVERRIDES`**

Change the decorator on `test_homing_retract_timing`:

```python
@pytest.mark.parametrize("sim_extra_overrides", [TIMING_OVERRIDES], indirect=True)
def test_homing_retract_timing(sim):
```

(Body unchanged — with `min_home_dist=0` the held-high pin insta-trips and homing completes via the normal retract, exactly as before this feature.)

- [ ] **Step 3: Repurpose the rehome test to the deterministic held-high path**

Replace the body of `test_homing_retract_and_rehome` with a version that holds the pin high so the path runs and then fails on the re-approach:

```python
@pytest.mark.parametrize("sim_extra_overrides", [SIM_OVERRIDES], indirect=True)
def test_homing_rehome_path_runs_and_fails_on_held_trigger(sim):
    _wait_ready(sim)

    # Endstop forced high for the whole homing op: the first approach trips
    # short (< min_home_dist), so the rehome path runs; the held pin makes the
    # re-approach insta-trip too, so G28 must fail loudly — fast, no hang.
    _set_pin(sim, X_ENDSTOP_LINE, 1)

    t0 = time.time()
    r = sim.gcode("G28 X", timeout=30.0)
    elapsed = time.time() - t0

    print(f"\n[G28 X held-trigger] elapsed={elapsed:.2f}s result={r}")

    log_text = sim.klippy_log.read_text()
    assert "needs rehome: True" in log_text, (
        "Expected 'needs rehome: True' — the short first trip should have "
        "requested a rehome"
    )

    if "No trigger on x after full movement" in log_text:
        pytest.fail(
            "Second homing attempt failed with 'No trigger after full "
            "movement' — the retract move did not complete before re-approach"
        )

    err = (r.get("error") or {}).get("message", "")
    assert "early homing trigger" in err, (
        f"Expected an 'early homing trigger' failure, got: {err!r}"
    )

    # Guard the original lag/deadlock bug: must fail fast, not hang.
    assert elapsed < 10.0, f"G28 X took {elapsed:.1f}s — likely ghost-time delay"
```

- [ ] **Step 4: Run the sim tests (requires ELF — runs in CI / on the bench)**

Run: `./scripts/ci.sh sim` (or, with the ELF built, `uv run py.test tools/sim_klippy/tests/test_homing_lag_repro.py -v`)
Expected: both tests PASS. If the runner here lacks the ELF (`needs_elf` skip), note it and rely on CI/bench to execute them.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/tests/test_homing_lag_repro.py
git commit -m "test(homing): deterministic ELF sim coverage for min_home_dist rehome"
```

---

## Task 5: Full verification

**Files:** none (verification only)

- [ ] **Step 1: Host unit tests**

Run: `python -m pytest test/test_homing_min_dist.py test/test_servo_homing.py test/test_rail.py test/test_active_rails.py -v`
Expected: PASS

- [ ] **Step 2: Ruff (format + check) over the repo**

Run: `./scripts/ci.sh ruff`
Expected: PASS (clean)

- [ ] **Step 3: Python host suite**

Run: `./scripts/ci.sh py`
Expected: PASS

- [ ] **Step 4: Quick gate (rust + clippy + fmt + canary)**

Run: `./scripts/ci.sh quick`
Expected: PASS (this change is host-only, but the gate must stay green before a PR)

- [ ] **Step 5: Final commit (if any lint/format fixups were needed)**

```bash
git add -A
git commit -m "chore(homing): min_home_dist verification fixups"
```

---

## Self-Review Notes (for the implementer)

- **Spec coverage:** Task 1 = decision predicate (spec §2). Task 3 = rehome flow, real-frame back-off, bounded re-approach, fail-loud (spec §3, scope). Task 2 = seed-after-final-retract (spec §4). Task 4 = sim strategy; unit tests across Tasks 1–3 = decision/orchestration/seed-ordering (spec Testing).
- **`gcmd.error`:** in production `gcmd.error(...)` returns a `command_error` instance to raise; the unit-test `FakeGcmd.error = RuntimeError` makes `raise gcmd.error(msg)` raise `RuntimeError(msg)` — same call shape.
- **`provider`:** `entry["provider"]` (was referenced inline as `entry["provider"]` in the old `_home_axis`); now bound once and threaded into both seams.
- **Frame correctness:** `_run_homing_attempts` only ever calls `set_position`/`move`; the configured `trigger_height` is applied solely in `_commit_and_seed`, after a validated trip.
- **No Rust/MCU edits** — `cargo` jobs in Task 5 are regression guards, not because anything Rust changed.
