# Correction Stream on the Move Timeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the per-motor correction "buzz" obey the same scheduling timeline as regular motion, so stepper enable/disable orders deterministically around it, and collapse the four duplicated copies of the 0.25 s motion lead into one Rust-owned constant.

**Architecture:** The buzz currently anchors at a private `host_now + 0.15` and never touches the toolhead's move-time bookkeeping, so it lands earlier than the enable (which rides `get_last_move_time()` at the shared 0.25 lead) and the skew grows with each `dwell()`. We make `submit_correction_sequence` accept an explicit `start_host_secs` computed from the live toolhead timeline, advance `_mcu_pending_end_time` past the buzz, and route motors_sync through a new `Motion.submit_correction` so the enable/disable bracketing it already uses becomes correct. One lead constant (`anchor::DEFAULT_LEAD_SECS`) is exposed to Python via a bridge getter.

**Tech Stack:** Rust (PyO3 `motion_bridge_native`, `cargo nextest`), Python (klippy host, `pytest`).

**Reference spec:** `docs/superpowers/specs/2026-06-15-correction-stream-on-move-timeline-design.md`

**Testing commands:**
- Rust: `cd rust && cargo nextest run -p motion-bridge` (full suite: `cargo nextest run`)
- Python: `python -m pytest test/test_toolhead_shim.py -v` (full host: `./scripts/ci.sh py`)
- Pre-PR gate: `./scripts/ci.sh quick` then `./scripts/ci.sh py`

**Execution note:** The Rust correction signature (Task 2) changes before the Python wrapper that calls it (Task 4). Each task runs only its own tests; the end-to-end Python path is exercised in Tasks 5-6 and the full gate in Task 7. Do not flash a bench mid-plan.

---

## File Structure

- `rust/motion-bridge/src/anchor.rs` — owns `DEFAULT_LEAD_SECS`; make it `pub`. Single source of the motion lead.
- `rust/motion-bridge/src/planner.rs` — `LEAD` becomes an alias of `anchor::DEFAULT_LEAD_SECS`; delete the keep-in-sync comment.
- `rust/motion-bridge/src/bridge.rs` — `submit_correction_sequence` / `stream_correction_entries` take `start_host_secs`; remove `CORRECTION_LEAD_SECS`; return `total_duration`; add `motion_lead_secs()` getter.
- `rust/motion-bridge/src/correction/tests.rs` — add explicit-start anchoring test.
- `klippy/motion_bridge.py` — `MotionBridgeWrapper.submit_correction_sequence` gains `start_host_secs`; add `motion_lead_secs()` delegation.
- `klippy/motion.py` — fetch `motion_lead` at init; replace `BUFFER_TIME_START`; add `Motion.submit_correction`.
- `klippy/extras/motors_sync.py` — `StepperManualMove.manual_move` calls `toolhead.submit_correction`.
- `test/test_toolhead_shim.py` — add tests for `motion_lead`-driven `get_last_move_time` and `submit_correction`.

---

## Task 1: Unify the Rust motion-lead constant

**Files:**
- Modify: `rust/motion-bridge/src/anchor.rs:2`
- Modify: `rust/motion-bridge/src/planner.rs:18-19`
- Test: `rust/motion-bridge/src/anchor.rs` (inline `#[cfg(test)]` already has `mod tests;` → add to `rust/motion-bridge/src/anchor/tests.rs`? No — anchor.rs uses `mod tests;` at line 68. Put the test there.)

- [ ] **Step 1: Write the failing test**

In `rust/motion-bridge/src/anchor/tests.rs`, append:

```rust
#[test]
fn default_lead_is_quarter_second_and_shared_with_planner() {
    assert_eq!(super::DEFAULT_LEAD_SECS, 0.25);
    assert_eq!(crate::planner::lead_secs(), super::DEFAULT_LEAD_SECS);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(default_lead_is_quarter_second)'`
Expected: FAIL — `DEFAULT_LEAD_SECS` is private (`pub` not yet added) and `crate::planner::lead_secs` does not exist (compile error).

- [ ] **Step 3: Make the constant public and alias it in the planner**

In `rust/motion-bridge/src/anchor.rs:2`, change:

```rust
const DEFAULT_LEAD_SECS: f64 = 0.25;
```

to:

```rust
pub const DEFAULT_LEAD_SECS: f64 = 0.25;
```

In `rust/motion-bridge/src/planner.rs:18-19`, replace:

```rust
/// Must equal `anchor::DEFAULT_LEAD_SECS`. Keep in sync with anchor.rs.
const LEAD: f64 = 0.25;
```

with:

```rust
const LEAD: f64 = crate::anchor::DEFAULT_LEAD_SECS;

#[cfg(test)]
pub(crate) fn lead_secs() -> f64 {
    LEAD
}
```

Note: `LEAD` stays a `const`, so `REPLAN_WARN_BUDGET_US` (planner.rs:23) still const-evaluates. `DEFAULT_LEAD_SECS` is referenced inside `anchor.rs` as a bare `DEFAULT_LEAD_SECS` (anchor.rs:59) — making it `pub` does not change in-module use.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(default_lead_is_quarter_second)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/anchor.rs rust/motion-bridge/src/anchor/tests.rs rust/motion-bridge/src/planner.rs
git commit -m "refactor(bridge): single source for motion lead (anchor::DEFAULT_LEAD_SECS)"
```

---

## Task 2: Thread explicit `start_host_secs` through the correction path

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:2169-2182` (`submit_correction_sequence`)
- Modify: `rust/motion-bridge/src/bridge.rs:4161-4246` (`stream_correction_entries`)
- Test: `rust/motion-bridge/src/correction/tests.rs`

- [ ] **Step 1: Write the failing test**

In `rust/motion-bridge/src/correction/tests.rs`, append a test pinning the contract the bridge now depends on — the first piece anchors exactly at `start_host_secs` and pieces advance by their durations:

```rust
#[test]
fn piece_entries_anchor_at_explicit_start() {
    let pieces = vec![
        ProfilePiece { coeffs: [0.0, 1.0, 2.0, 3.0], duration: 0.4 },
        ProfilePiece { coeffs: [3.0, 3.0, 3.0, 3.0], duration: 0.6 },
    ];
    // project: scale host-seconds to integer "clock" 1:1 (rounded) for assertion.
    let entries = to_piece_entries(&pieces, |secs| (secs * 1000.0).round() as u64, 12.5);
    assert_eq!(entries[0].start_time, 12_500); // 12.5 s
    assert_eq!(entries[1].start_time, 12_900); // 12.5 + 0.4 s
}
```

If `ProfilePiece` has additional fields, construct it the way the other tests in this file already do (match the existing constructor pattern at the top of `correction/tests.rs`).

- [ ] **Step 2: Run test to verify it fails or passes**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(piece_entries_anchor_at_explicit_start)'`
Expected: PASS if `to_piece_entries` already honors the start (it does, correction.rs:149-174). This test is a **regression guard** locking the contract before we remove the private lead. If it fails, fix the test's `ProfilePiece` construction to match the file's pattern — do not change `to_piece_entries`.

- [ ] **Step 3: Add the parameter to `submit_correction_sequence`**

In `rust/motion-bridge/src/bridge.rs:2169`, change the signature and the call to `stream_correction_entries`:

```rust
    fn submit_correction_sequence(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        axis_idx: u8,
        motor_idx: u8,
        segments: Vec<f64>,
        speed: f64,
        accel: f64,
        start_host_secs: f64,
    ) -> PyResult<f64> {
        let pieces = crate::correction::plan_correction_sequence(&segments, speed, accel)
            .map_err(PyRuntimeError::new_err)?;
        self.stream_correction_entries(
            py, mcu_handle, axis_idx, motor_idx, &pieces, start_host_secs,
        )
    }
```

- [ ] **Step 4: Use the explicit start in `stream_correction_entries` and return the buzz duration**

In `rust/motion-bridge/src/bridge.rs:4161-4246`, change the signature, delete the private lead, anchor at `start_host_secs`, and return `total_duration`:

Replace the signature/header (4161-4181):

```rust
    fn stream_correction_entries(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        axis_idx: u8,
        motor_idx: u8,
        pieces: &[crate::correction::ProfilePiece],
        start_host_secs: f64,
    ) -> PyResult<f64> {
        let ring_depth = runtime::stepping_state::CORRECTION_RING_DEPTH as u32;
        let handle = mcu_handle_from_raw(mcu_handle);

        let entries = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            crate::correction::to_piece_entries(
                pieces,
                |secs| router.host_time_to_mcu_clock(handle, secs).unwrap_or(0),
                start_host_secs,
            )
        };
```

The `if entries.iter().any(|e| e.start_time == 0)` clock-unsynced check (4182-4186), the chunking, the IO lookup, and the `wait_room`/`add_sent` feedback loop are unchanged.

Replace the final return (bridge.rs:4245) from:

```rust
        Ok(CORRECTION_LEAD_SECS + total_duration)
```

to:

```rust
        Ok(total_duration)
```

Delete the `const CORRECTION_LEAD_SECS: f64 = 0.15;` line (was bridge.rs:4169). `total_duration` is still computed at bridge.rs:4187 (`pieces.iter().map(|p| p.duration).sum()`).

- [ ] **Step 5: Run the correction tests + clippy**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(correction) + test(piece_entries_anchor_at_explicit_start)' && cargo clippy -p motion-bridge -- -D warnings`
Expected: PASS, no warnings. (`CORRECTION_LEAD_SECS` removed → no dead-code/unused warning.)

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs rust/motion-bridge/src/correction/tests.rs
git commit -m "feat(bridge): correction stream anchors at caller-provided start_host_secs"
```

---

## Task 3: Expose the motion lead to Python via a bridge getter

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (add a method to the `#[pymethods] impl PyMotionBridge` block — place it next to `get_last_move_time`, bridge.rs:3585)

- [ ] **Step 1: Add the getter**

Immediately after the `get_last_move_time` method (ends bridge.rs:3595), add inside the same `#[pymethods]` impl:

```rust
    fn motion_lead_secs(&self) -> f64 {
        crate::anchor::DEFAULT_LEAD_SECS
    }
```

- [ ] **Step 2: Build the cdylib to confirm the PyO3 surface compiles**

Run: `cd rust && cargo build -p motion-bridge`
Expected: builds clean.

- [ ] **Step 3: Verify the method is exported (smoke check via the workspace test build)**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(default_lead_is_quarter_second)'`
Expected: PASS (confirms the crate still compiles with the new method).

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(bridge): expose motion_lead_secs() to the host"
```

---

## Task 4: Thread the parameter and getter through the Python wrapper

**Files:**
- Modify: `klippy/motion_bridge.py:431-440` (`submit_correction_sequence`)
- Modify: `klippy/motion_bridge.py` (add `motion_lead_secs` near `get_last_move_time`, ~line 463)

- [ ] **Step 1: Add `start_host_secs` to the wrapper**

In `klippy/motion_bridge.py:431-440`, change:

```python
    def submit_correction_sequence(
        self, mcu_id, axis_idx, motor_idx, segments, speed, accel, start_host_secs
    ):
        return self._bridge.submit_correction_sequence(
            mcu_id,
            axis_idx,
            motor_idx,
            [float(s) for s in segments],
            speed,
            accel,
            float(start_host_secs),
        )
```

- [ ] **Step 2: Add the `motion_lead_secs` delegation**

In `klippy/motion_bridge.py`, immediately after the `get_last_move_time` method (the one delegating to `self._bridge.get_last_move_time()`), add:

```python
    def motion_lead_secs(self):
        return self._bridge.motion_lead_secs()
```

- [ ] **Step 3: Verify import still loads**

Run: `python -c "import klippy.motion_bridge"`
Expected: no error.

- [ ] **Step 4: Commit**

```bash
git add klippy/motion_bridge.py
git commit -m "feat(host): thread start_host_secs + motion_lead_secs through the bridge wrapper"
```

---

## Task 5: Drive `Motion` off the shared lead and add `submit_correction`

**Files:**
- Modify: `klippy/motion.py:12` (delete `BUFFER_TIME_START` constant)
- Modify: `klippy/motion.py:106-123` (`__init__` — fetch `motion_lead`)
- Modify: `klippy/motion.py:470-488` (`get_last_move_time`, `_ground_pending_end_time_after_bridge_drain` — use `self.motion_lead`)
- Modify: `klippy/motion.py` (add `submit_correction` method near `manual_move`, ~line 237)
- Test: `test/test_toolhead_shim.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_toolhead_shim.py`:

```python
class _RecordingBridge:
    def __init__(self, duration):
        self._duration = duration
        self.last_call = None

    def motion_lead_secs(self):
        return 0.25

    def submit_correction_sequence(
        self, mcu_id, axis_idx, motor_idx, segments, speed, accel, start_host_secs
    ):
        self.last_call = dict(
            mcu_id=mcu_id, axis_idx=axis_idx, motor_idx=motor_idx,
            segments=list(segments), speed=speed, accel=accel,
            start_host_secs=start_host_secs,
        )
        return self._duration


class _FixedReactor:
    def monotonic(self):
        return 100.0


def _make_correction_toolhead(duration):
    th = Motion.__new__(Motion)
    th.mcu = FakeMcu()                 # estimated_print_time(t) = t + 1.0
    th.reactor = _FixedReactor()
    th.bridge = _RecordingBridge(duration)
    th.motion_lead = 0.25
    th._mcu_pending_end_time = 0.0
    return th


def test_get_last_move_time_uses_motion_lead():
    th = _make_correction_toolhead(0.0)
    th.motion_lead = 0.5
    # est = 100 + 1 = 101; floor = est + 0.5 = 101.5; pending 0 < est -> floor
    assert th.get_last_move_time() == pytest.approx(101.5)


def test_submit_correction_anchors_on_timeline_and_advances_pending():
    th = _make_correction_toolhead(0.6)
    # idle: glmt = est(101.0) + lead(0.25) = 101.25
    # start_host = now + (glmt - est) = 100 + 0.25 = 100.25
    wait_s = th.submit_correction(7, 1, 0, [0.3, -0.3], 80.0, 5000.0)
    call = th.bridge.last_call
    assert call["mcu_id"] == 7 and call["axis_idx"] == 1 and call["motor_idx"] == 0
    assert call["start_host_secs"] == pytest.approx(100.25)
    # pending advanced past the buzz: glmt + duration = 101.25 + 0.6 = 101.85
    assert th._mcu_pending_end_time == pytest.approx(101.85)
    # caller wait = (start_host - now) + duration = 0.25 + 0.6 = 0.85
    assert wait_s == pytest.approx(0.85)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest test/test_toolhead_shim.py::test_submit_correction_anchors_on_timeline_and_advances_pending test/test_toolhead_shim.py::test_get_last_move_time_uses_motion_lead -v`
Expected: FAIL — `Motion` has no `submit_correction`, and `get_last_move_time` uses the module constant `BUFFER_TIME_START`, not `self.motion_lead`.

- [ ] **Step 3: Fetch the lead at init and delete the module constant**

In `klippy/motion.py:12`, delete:

```python
BUFFER_TIME_START = 0.250
```

In `klippy/motion.py:115` (right after `self._mcu_pending_end_time = 0.0`), add:

```python
        self.motion_lead = self.bridge.motion_lead_secs()
        if self.motion_lead is None:
            self.motion_lead = 0.25
```

(The `or`-default covers `_StubBridge`, whose `__getattr__` returns a no-op → `None` when the native engine is absent; no real motion runs under the stub, so this boot-only fallback is not a second copy of the production lead.)

- [ ] **Step 4: Use `self.motion_lead` in the two floor sites**

In `klippy/motion.py:474`, change `floor = est + BUFFER_TIME_START` to:

```python
        floor = est + self.motion_lead
```

In `klippy/motion.py:486`, change `command_time = est + BUFFER_TIME_START` to:

```python
        command_time = est + self.motion_lead
```

- [ ] **Step 5: Add `Motion.submit_correction`**

In `klippy/motion.py`, after `manual_move` (ends line 237), add:

```python
    def submit_correction(self, mcu_id, axis_idx, motor_idx, segments, speed, accel):
        now = self.reactor.monotonic()
        glmt = self.get_last_move_time()
        est = self.mcu.estimated_print_time(now)
        start_host_secs = now + (glmt - est)
        duration = self.bridge.submit_correction_sequence(
            mcu_id, axis_idx, motor_idx, segments, speed, accel, start_host_secs
        )
        buzz_end_print_time = glmt + duration
        if buzz_end_print_time > self._mcu_pending_end_time:
            self._mcu_pending_end_time = buzz_end_print_time
        return (start_host_secs - now) + duration
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `python -m pytest test/test_toolhead_shim.py -v`
Expected: PASS (new tests green; existing shim tests unaffected).

- [ ] **Step 7: Confirm no stray `BUFFER_TIME_START` references remain**

Run: `grep -rn "BUFFER_TIME_START" klippy/`
Expected: no output.

- [ ] **Step 8: Commit**

```bash
git add klippy/motion.py test/test_toolhead_shim.py
git commit -m "feat(host): Motion.submit_correction anchors the buzz on the move timeline"
```

---

## Task 6: Route motors_sync through the toolhead

**Files:**
- Modify: `klippy/extras/motors_sync.py:380-394` (`StepperManualMove.manual_move`)

- [ ] **Step 1: Replace the raw-bridge call with the toolhead method**

In `klippy/extras/motors_sync.py:380-394`, change the body of `manual_move`:

```python
    def manual_move(self, mcu_stepper, moves):
        segments = [m for m in moves if abs(m) >= 0.00001]
        if not segments:
            return
        name = mcu_stepper.get_name()
        mcu_id, axis_idx, motor_idx = self.toolhead.get_motor_binding(name)
        reactor = self.printer.get_reactor()
        start = reactor.monotonic()
        duration = self.toolhead.submit_correction(
            mcu_id, axis_idx, motor_idx, segments,
            self.travel_speed, self.travel_accel)
        deadline = start + duration + SETTLE_PAD
        while reactor.monotonic() < deadline:
            reactor.pause(reactor.monotonic() + 0.01)
```

This drops the `bridge = self.toolhead.get_bridge()` lookup (the toolhead now owns the call) and the direct `bridge.submit_correction_sequence(...)`. `duration` is now "seconds from the call until the buzz completes," so the existing `deadline` wait is still correct.

- [ ] **Step 2: Confirm the module imports**

Run: `python -c "import klippy.extras.motors_sync"`
Expected: no error.

- [ ] **Step 3: Run any motors_sync unit tests present**

Run: `python -m pytest test/ -k motors_sync -v`
Expected: PASS, or "no tests ran" if none exist (acceptable — the behavior is covered by Task 5's `submit_correction` tests; the bench validates the end-to-end buzz).

- [ ] **Step 4: Commit**

```bash
git add klippy/extras/motors_sync.py
git commit -m "feat(motors_sync): issue the buzz through the toolhead correction timeline"
```

---

## Task 7: Full gate + cleanup

**Files:** none (verification only)

- [ ] **Step 1: Rust workspace green**

Run: `cd rust && cargo nextest run`
Expected: all pass.

- [ ] **Step 2: Quick gate (ruff, rust test/clippy/fmt, watchdog canary)**

Run: `./scripts/ci.sh quick`
Expected: `5 pass 0 fail`.

- [ ] **Step 3: Python host gate (touched `klippy/`)**

Run: `./scripts/ci.sh py`
Expected: green.

- [ ] **Step 4: Final fmt check**

Run: `cd rust && cargo fmt --all --check`
Expected: no diff. If it reports changes, run `cargo fmt --all` and amend the relevant commit.

- [ ] **Step 5: Confirm the lead is single-source**

Run: `grep -rn "0\.25\b\|0\.250\b" klippy/motion.py rust/motion-bridge/src/anchor.rs rust/motion-bridge/src/planner.rs rust/motion-bridge/src/bridge.rs | grep -i "lead\|buffer\|0.15"`
Expected: the only literal `0.25` is in `anchor.rs` (`DEFAULT_LEAD_SECS`); no `0.15` correction lead, no `0.250` in motion.py. (Manual confirmation — the grep is a guide, eyeball the output.)

---

## Self-Review

**Spec coverage:**
- Anchor buzz at toolhead timeline → Task 2 (Rust accepts `start_host_secs`) + Task 5 (`Motion.submit_correction` computes it). ✓
- Advance `_mcu_pending_end_time` past the buzz → Task 5, Step 5. ✓
- Remove `CORRECTION_LEAD_SECS`, inherit lead from `get_last_move_time()` → Task 2, Step 4. ✓
- One lead constant, no keep-in-sync comment → Task 1 (Rust dedup) + Task 3 (getter) + Task 5 (motion.py consumes it). ✓
- motors_sync routed through toolhead, enable/disable untouched → Task 6. ✓
- Non-goal (no second scheduling authority) → respected; no enable-path changes. ✓

**Placeholder scan:** No TBD/TODO; every code step shows full code. The one judgment point (`ProfilePiece` constructor shape in Task 2 Step 1) is flagged with a concrete fallback instruction.

**Type consistency:** `start_host_secs: f64` consistent across Rust `submit_correction_sequence`/`stream_correction_entries` and Python wrapper/`Motion.submit_correction`. Return value is `duration`/`total_duration` (seconds) everywhere. `motion_lead_secs()` name identical in Rust, wrapper, and motion.py. `submit_correction` (Motion) vs `submit_correction_sequence` (bridge/wrapper) are intentionally distinct names for distinct layers.
