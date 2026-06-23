---
title: 'Restore host-side flush-callback driver so M106/queued pin requests apply'
type: 'bugfix'
created: '2026-06-23'
status: 'done'
baseline_commit: '93110834a491ec21819cb81ec1d1f84bf324a9c5'
context: ['{project-root}/_bmad-output/implementation-artifacts/investigations/m106-fan-not-activating-investigation.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `M106` (and every `[output_pin]`/fan/LED request routed through `GCodeRequestQueue.queue_gcode_request`) never takes effect on an idle printer. The motion-engine rewrite gutted `MCU.flush_moves` to a no-op `return` and removed mainline's periodic flush handler, so the `_flush_callbacks` that drain `GCodeRequestQueue` are registered but never invoked. The heater-fan path survives only because it bypasses this machinery via the inline `send_async_request`.

**Approach:** Restore a host-side periodic flush driver — mainline's `_flush_handler` minus the steppersync the engine now owns. Un-stub `MCU.flush_moves` to fire `self._flush_callbacks(print_time, clock)`; add a reactor flush timer + `need_flush_time` horizon to `Motion`, kicked by a real `note_mcu_movequeue_activity`; make `ToolheadShim.note_mcu_movequeue_activity` delegate instead of `pass`.

## Boundaries & Constraints

**Always:** Keep `klippy/extras/output_pin.py` unchanged (shared with mainline; it already speaks the `register_flush_callback`/`note_mcu_movequeue_activity` contract). Drive `flush_moves` on every MCU in `self.all_mcus` (a pin/fan may live on any MCU). Pass each callback `(print_time, clock)` where `clock = mcu.print_time_to_clock(print_time)`. Re-kick the timer only via the mainline `do_kick_flush_timer` latch so idle re-entry is deterministic.

**Ask First:** If the flush handler cannot converge (a callback advances `need_flush_time` without bound), or if any existing motion/stepper flush path turns out to call `flush_moves`/`note_mcu_movequeue_activity` expecting the old no-op semantics.

**Never:** Do not revive `steppersync`/trapq flushing on the host — the engine owns motion and step generation. Do not route through the engine's `passthrough_register_flush_callback`/`check_flush()` path (argless `Fn()` signature, drain-to-empty semantics — wrong for idle pin scheduling, and it would force changes to shared `output_pin.py`). Do not honor `set_step_gen_time` (engine-owned). Do not pad or advance time to mask a stale request.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| M106 on idle printer | `queue_gcode_request(speed)`, no motion | `note_mcu_movequeue_activity` kicks timer → handler `flush_moves(need_flush_time)` → rqueue drains → `set_pwm` sent; fan spins | N/A |
| kick_start fan | request returns `("delay", t)` then follow-up at later `print_time` | both full-speed and deferred target-speed apply (each carries its own future MCU clock) | N/A |
| Repeated idle requests | many M106 over time | timer idles (`NEVER`) between requests, re-kicks each; no busy-loop | N/A |
| `print_time_to_clock` < 0 | flush before clock sync | `flush_moves` returns without firing callbacks | early return |

</frozen-after-approval>

## Code Map

- `klippy/mcu.py:1531-1536` -- `register_flush_callback` (append) + `flush_moves` no-op stub to un-stub; `_flush_callbacks` init at `:779`.
- `klippy/motion.py:105-121` -- `Motion.__init__`: has `self.reactor`, `self.printer`, `self.all_mcus`, `self.mcu`, `self.motion_lead`. Add flush state + timer here.
- `klippy/motion.py:592-599` -- `get_last_move_time` (request print_time source; unchanged).
- `klippy/motion.py:1434-1435` -- `ToolheadShim.note_mcu_movequeue_activity` = `pass` → delegate to `self.motion`.
- `klippy/extras/output_pin.py:27,33-67` -- `GCodeRequestQueue` consumer (DO NOT EDIT); defines the contract being restored.
- `test/test_toolhead_shim.py` -- `FakeMcu`/`FakeKin` scaffolding for the regression test.

## Tasks & Acceptance

**Execution:**
- [x] `klippy/mcu.py:1534` -- un-stub `flush_moves(print_time, clear_history_time)`: compute `clock = self.print_time_to_clock(print_time)`, return early if `clock < 0`, else `for cb in self._flush_callbacks: cb(print_time, clock)`. No steppersync. -- restores the only mechanism that drains `GCodeRequestQueue`.
- [x] `klippy/motion.py:122,580` -- in `Motion.__init__` add `self.need_flush_time = 0.0`, `self.do_kick_flush_timer = True`, `self.flush_timer = self.reactor.register_timer(self._flush_handler)`. Add `advance_flush_time(self, mq_time)` (set `need_flush_time = max(...)`; if `do_kick_flush_timer`: clear it and `update_timer(flush_timer, NOW)`) and `_flush_handler(self, eventtime)` that flushes every `self.all_mcus` up to `need_flush_time`, loops while a callback advances `need_flush_time`, then sets `do_kick_flush_timer = True` and returns `reactor.NEVER`. -- supplies the periodic driver the rewrite dropped. **Deviation:** the kick method is `advance_flush_time` (Motion-native), NOT `note_mcu_movequeue_activity` — see Design Notes (fossil-method contract).
- [x] `klippy/motion.py:1451` -- `ToolheadShim.note_mcu_movequeue_activity` delegates to `self.motion.advance_flush_time(mq_time)` (remove `pass`). -- routes the GCodeRequestQueue kick to the real driver while keeping the fossil method only on the shim.
- [x] `test/test_motion_flush_driver.py` -- new file: unit-tests the I/O matrix — `flush_moves` fires callbacks with `(print_time, clock)` and skips on `clock < 0`; the driver kicks + flushes all MCUs; the handler converges on callback re-bump; idle returns `NEVER` and re-kicks; and the full `M106` path drains a `GCodeRequestQueue` on an idle printer. -- locks the regression.

**Acceptance Criteria:**
- Given an idle printer with a configured `[fan]`, when `M106 S255` is issued, then `queue_pwm_out`/`queue_digital_out` is transmitted and the fan runs (verified in sim and/or on neptune-bench).
- Given a registered flush callback and a `note_mcu_movequeue_activity(t)` call, when the reactor services the flush timer, then the callback is invoked with `print_time >= t` and `clock == print_time_to_clock(print_time)`.
- Given no queued requests, when the handler runs, then it returns `reactor.NEVER` and re-arms on the next `note_mcu_movequeue_activity`.
- Given `clippy ./scripts/ci.sh quick` and `./scripts/ci.sh py`, both stay green.

## Spec Change Log

## Design Notes

The engine owns motion/step flushing; this restores only the orphaned auxiliary-output flush path (pins/fans/LEDs). Flushing pin requests ahead of wall-clock is correct — each `queue_digital_out`/`queue_pwm_out` carries its own execution clock, so the MCU fires the pin at the scheduled time; the handler can therefore drain to `need_flush_time` in one pass rather than pacing like the stepper buffer. Convergence is bounded (`output_pin` advances `need_flush_time` only while applying finite rqueue entries). Mirror mainline's `do_kick_flush_timer` latch so a request arriving mid-handler still re-kicks deterministically.

**Fossil-method contract (implementation deviation):** `test/test_toolhead_shim_delegation.py::test_fossil_methods_only_on_shim` asserts the five Klipper-compat methods — including `note_mcu_movequeue_activity` — exist ONLY on `ToolheadShim`, never on the clean `Motion`. So the driver's kick entry point lives on `Motion` as `advance_flush_time(mq_time)`, and the shim's fossil `note_mcu_movequeue_activity(mq_time, set_step_gen_time)` delegates to it. `set_step_gen_time` is dropped at the seam (engine-owned). The frozen Intent still holds: the shim's `note_mcu_movequeue_activity` is now a real, driving method.

## Verification

**Commands:**
- `./scripts/ci.sh py` -- expected: new flush-driver test passes; `test_toolhead_shim*` stay green.
- `./scripts/ci.sh quick` -- expected: green (no Rust touched).

**Manual checks:**
- mcu-sim or neptune-bench: `M106 S255` on an idle printer spins the part fan; `M107` stops it; `query-logs` shows `queue_pwm_out`/`queue_digital_out` transmitted at M106 time.

## Suggested Review Order

**The seam (start here)**

- Why M106 was dead: the flush callback that drains the fan queue is now actually fired.
  [`mcu.py:1534`](../../klippy/mcu.py#L1534)

**The restored driver**

- Periodic flush state + timer registered on the real toolhead object.
  [`motion.py:124`](../../klippy/motion.py#L124)

- Kick entry point (Motion-native name; keeps the fossil method off `Motion`).
  [`motion.py:578`](../../klippy/motion.py#L578)

- The handler: drains all MCUs to `need_flush_time`, fail-loud guard mirrors mainline.
  [`motion.py:584`](../../klippy/motion.py#L584)

- The shim's fossil method now delegates to the driver instead of `pass`.
  [`motion.py:1460`](../../klippy/motion.py#L1460)

**Regression lock**

- End-to-end: a real `GCodeRequestQueue` drains on an idle printer + fail-loud guard test.
  [`test_motion_flush_driver.py:1`](../../test/test_motion_flush_driver.py#L1)
