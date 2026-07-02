---
title: 'EtherCAT endpoint death — fast fail-loud with the real cause'
type: 'bugfix'
created: '2026-06-30'
status: 'done'
baseline_commit: 'a1e049083'
context:
  - '{project-root}/docs/rewrite/mcu-c-rust-boundary.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** When the EtherCAT servo endpoint **genuinely dies** mid-print (its process crashes — e.g. the observed `ec-heartbeat-po` SIGBUS — or the socket breaks), the host's only response is a bare `process::abort()` with **no reported cause**. klippy's shutdown reason is first-come, so whatever fault arrived first (typically the collateral `-308 PieceStartInPast` from the pump no longer being fed) becomes the misleading "MCU runtime fault" headline, and `abort()` triggers a silent systemd restart that hides the real error from the operator. *(Note: the recurring crash that motivated this was found by investigation B to be host-side pump/serial slowness, not the endpoint — that path is handled by B's committed in-past guard. This spec covers the distinct, genuine endpoint-death case.)*

**Approach:** On a genuine EtherCAT endpoint death (the existing `SendError::Fatal` classification — socket `Closed`/`Io`, or the supervisor detecting peer-EOF / child-exit), **report the real cause as a clean klippy shutdown** instead of aborting: latch a distinct `EthercatEndpointDied = -203` fault through the existing "latched for klippy" channel so `invoke_shutdown`'s reason is "EtherCAT endpoint died mid-session", and **do not auto-restart** — the operator runs `FIRMWARE_RESTART` after seeing the actual error. A bounded **safety backstop** preserves the old guarantee that the machine stops: if klippy never consumes the latched cause within a grace window (reactor wedged/CPU-starved), force a last-resort abort.

## Boundaries & Constraints

**Always:**
- Trigger only on a **genuine** endpoint death — the existing ethercat `SendError::Fatal` (`Closed`/`Io`) and the supervisor's peer-EOF / child-exit detection. Do **not** change the timeout or the Transient/Fatal classification (the observed stall was host-side, not the endpoint — out of scope here).
- Fail **loud with the real cause as a clean shutdown** — latch `-203` so klippy's reason is the endpoint death; **no `process::abort` on the normal path**; klippy stays shut down until `FIRMWARE_RESTART` (no auto-restart — the operator must see the actual error). Reuse the existing latched-fault → klippy poll channel (a dedicated sibling latch is acceptable — same pattern, not a new mechanism).
- Keep a **bounded safety backstop**: if klippy hasn't consumed the latched cause within the grace, abort as a last resort so a wedged reactor can't leave the machine running with a dead endpoint.
- Host-side only — no MCU firmware change, nothing on the C/Rust seam (per `mcu-c-rust-boundary.md`). `klippy/clocksync.py` untouched.

**Never:**
- Do not investigate or fix *why* the endpoint dies, nor the host-slowness `-308` (B's domain) — out of scope.
- Do not change the EtherCAT call timeout or make any transport fatal-on-timeout (reverted — the hang was the serial MCU, not the endpoint).
- Do not auto-restart on the normal endpoint-death path — clean shutdown that surfaces the cause is the goal; the abort is a *last-resort backstop* only.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Endpoint socket dies | ethercat send → `SendError::Fatal` (`Closed`/`Io`), pump `return`s (stops feeding) | Latch `-203` cause (first-wins), arm the backstop; no abort on this path | clean reported cause |
| Endpoint process exits | supervisor sees peer-EOF / child-exit → `on_endpoint_death` | Same: latch `-203`, arm backstop, no abort | clean reported cause |
| Host shutdown | klippy poll consumes the latch within ~1 poll period | `invoke_shutdown` reason = "EtherCAT endpoint died mid-session" (`-203`); machine stays down for `FIRMWARE_RESTART` (no auto-restart) | clean shutdown |
| Reactor wedged | latch still unconsumed after the grace (klippy never shut down) | Watchdog forces `abort_after_tracing_appender_drains()` so the machine still stops | last-resort abort |
| Serial MCU transport | any serial error incl. timeout | Classified `Transient` (unchanged) — retried; endpoint-death path is ethercat-only | unchanged |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/bridge.rs` -- the pump's `on_fatal_transport` closure (called from `run_pump`'s `Fatal` arm, which already `return`s) and the supervisor's `on_endpoint_death` (peer-EOF / child-exit) are the two death sites. `report_ethercat_endpoint_death` + `arm_endpoint_death_watchdog` + `ENDPOINT_DEATH_SHUTDOWN_GRACE`; dedicated `latched_endpoint_death` field + `take_endpoint_death` pyo3 method. The existing `latched_drive_fault`/`take_drive_fault` + `ethercat_node._poll_drive_fault` timer is the channel pattern reused. `abort_after_tracing_appender_drains()` (used by the watchdog + the existing drip-stall path).
- `klippy/extras/ethercat_node.py` (`_poll_drive_fault`) + `klippy/motion_engine.py` (allowlist + wrapper) -- check `take_endpoint_death` before the drive-fault and `invoke_shutdown` with the distinct `-203` reason; returns `NEVER` (no re-arm → `FIRMWARE_RESTART`).
- `rust/runtime/src/error.rs` -- `FaultCode` `-2xx` host family (`HostDisconnect=-200` … `HostDispatcherTimeout=-202`); `EthercatEndpointDied = -203` (+ `from_u16`, `code_name`, round-trip test list).
- *Out of scope / unchanged:* `pump.rs` `SendError` classification and the call timeout (the serial-MCU stall is B's domain).

## Tasks & Acceptance

**Execution:**
- [x] `rust/runtime/src/error.rs` -- added `EthercatEndpointDied = -203` to `FaultCode`, `from_u16`, `code_name`, and the `error/tests.rs` round-trip list.
- [x] `rust/motion-engine/src/bridge.rs` -- `report_ethercat_endpoint_death` (latch `-203` cause into a dedicated `latched_endpoint_death`, first-wins, structured log; returns `true` on first latch); `on_fatal_transport` and `on_endpoint_death` call it and **do not abort** on the normal path (the pump thread already `return`s). `arm_endpoint_death_watchdog` + `ENDPOINT_DEATH_SHUTDOWN_GRACE = 5 s`: on first latch, a watchdog thread aborts as a last resort if the latch is still unconsumed after the grace. Added `take_endpoint_death` pyo3 method. (Dedicated latch sibling to `latched_drive_fault` — same channel pattern, separate semantics, so endpoint death never collides with the drive-fault-during-homing logic.)
- [x] `klippy/extras/ethercat_node.py` + `klippy/motion_engine.py` -- `_poll_drive_fault` checks `take_endpoint_death` first → `invoke_shutdown("EtherCAT endpoint died mid-session on node … : …(-203)…")`, distinct from drive-fault formatting; wrapper + allowlist entry added.
- [x] Tests -- `bridge/tests.rs`: `report_ethercat_endpoint_death` latches the `(fault -203)` message, first-cause-wins (later writer returns `false`, does not overwrite). `runtime` `-203` round-trip via the `error/tests.rs` list. `test/test_servo_param.py`: endpoint-death poll takes precedence and shuts down with the `-203` reason.
- [~] **Reverted** (`pump.rs` / `pump/wire_sink_tests.rs` back to baseline) -- the 250 ms `ETHERCAT_CALL_TIMEOUT` + `Timeout`→Fatal fast-detect: the observed stall was the **serial MCU**, not the endpoint (dderg), so the timeout/classification change is out of scope. Endpoint death is detected by the existing `Closed`/`Io` Fatal + the supervisor.

**Acceptance Criteria:**
- Given a genuine EtherCAT endpoint death (socket `Closed`/`Io`, or supervisor peer-EOF / child-exit), when it occurs, then `-203` is latched (first cause wins) and the pump stops feeding — no `process::abort` on this path.
- Given the latched cause, when klippy's poll runs, then `invoke_shutdown`'s reason is "EtherCAT endpoint died mid-session" (`-203`), the machine stays down, and recovery requires `FIRMWARE_RESTART` (no auto-restart).
- Given klippy never consumes the latch within `ENDPOINT_DEATH_SHUTDOWN_GRACE` (reactor wedged), when the grace elapses, then the watchdog forces a last-resort abort so the machine still stops.
- Given the serial MCU transport, when any error (incl. timeout) occurs, then it is classified `Transient` (unchanged); the endpoint-death path is ethercat-only.

## Spec Change Log

### 2026-06-30 — reconcile with the parallel root-cause finding (B)
- **Finding (B session, committed `a1e049083`).** The *observed* recurring crash was **host-side pump slowness**, not an EtherCAT endpoint failure — the endpoint exited *downstream* (it sees the bridge/klippy disconnect after the host shuts down). B added a pump-level guard (`pump_piece_in_past`) that fails loud at send time when a piece is already in the MCU's past, catching the slow-pump case for any transport before the MCU trips `-308`.
- **Impact on A.** A's premise ("endpoint death → pump blocks on the dead socket") was the *symptom chain* of that crash, not its root. A is retained, **re-scoped to the genuine-endpoint-hang case**: when the EtherCAT endpoint actually stops replying while pieces are still valid (e.g. the `ec-heartbeat-po` SIGBUS crash), B's in-past guard does *not* fire (pieces aren't late yet) and the send would otherwise block up to 5 s → A's 250 ms `Timeout`→Fatal fails fast and reports `-203`.
- **No conflict.** B's guard runs *before* `send_mcu_frames`; in the pump-slow case it wins (aborts first). A's path only triggers on a true ethercat-send Fatal. The first-failure-wins latch surfaces the correct cause either way. (Note: B's guard `process::abort`s; A deliberately does a clean shutdown per dderg — the two paths differ by design.)

### 2026-06-30 — review loop + drop the timeout change (dderg)
- **Trigger.** Adversarial review of A flagged: the latch `HashMap::insert` overwrote (first-cause-wins broken); single 250 ms `Timeout`→Fatal would false-report a healthy endpoint as dead on a loaded Pi (and the hang was actually the **serial MCU**, per dderg — the ethercat timeout premise was wrong); and removing *both* aborts left no independent stop if klippy's reactor wedges.
- **Resolution (dderg).** (a) **Drop the timeout/classification change entirely** — `pump.rs` / `wire_sink_tests.rs` reverted to baseline; endpoint death is detected by the existing `Closed`/`Io` Fatal + the supervisor. (b) Keep the clean `-203` shutdown + first-cause-wins (`Entry::Vacant`). (c) **Add the safety backstop** (`arm_endpoint_death_watchdog`, 5 s grace): clean shutdown normally, last-resort abort only if klippy never consumes the latch.
- **Known-bad avoided.** Mislabelling a serial/host stall as "endpoint died"; a wedged-reactor leaving a dead-endpoint machine running; a later writer clobbering the first cause.

## Design Notes

Scope is the **genuine endpoint-death** path only: the existing ethercat `SendError::Fatal` (`Closed`/`Io`, raised in the pump's `Fatal` arm which already `return`s) and the supervisor's peer-EOF / child-exit detection. The change is purely *how* that death is surfaced — latch a `-203` cause that klippy's existing fault poll turns into a clean `invoke_shutdown`, instead of the bare `process::abort` that hid the cause behind a systemd restart. The watchdog reinstates the abort's one virtue (a guaranteed stop) without its vice (auto-restart on the normal path): it fires only when the latch is still unconsumed after the grace, i.e. the reactor never ran. The host-side pump/serial slowness that produces the collateral `-308` is a separate failure handled by B's committed in-past guard.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine -p runtime` -- expected: green incl. the `report_ethercat_endpoint_death` first-cause-wins/`-203` test + the `-203` fault round-trip.
- `./scripts/ci.sh quick` -- expected: fully green.
- `./scripts/ci.sh py` -- expected: green (touches `klippy/`).
- `./scripts/ci.sh rust-mcu-h7 && ./scripts/ci.sh rust-mcu-f4 && ./scripts/ci.sh rust-mcu-g0` -- expected: build (`runtime` in the diff).

**Manual checks:**
- Neptune EtherCAT servo bench: mid-print, kill the `ethercat-endpoint-hw` process; confirm via `query-logs`/`mcu-diagnostics` that klippy shuts down (within ~1 poll period) with reason "EtherCAT endpoint died mid-session" (`-203`), the machine stays down (no systemd auto-restart), and the `endpoint_death_watchdog_abort` event does **not** fire (the clean path won).

## Suggested Review Order

**The reporting mechanism (entry point)**

- Entry point: clean-cause latch, first-cause-wins, no abort on the normal path.
  [`bridge.rs:58`](../../rust/motion-engine/src/bridge.rs#L58)
- The safety backstop — last-resort abort only if klippy never consumes the latch.
  [`bridge.rs:90`](../../rust/motion-engine/src/bridge.rs#L90)
- The two death sites arm the watchdog once, on first latch (pump fatal + supervisor).
  [`bridge.rs:813`](../../rust/motion-engine/src/bridge.rs#L813)
- The pyo3 drain klippy polls.
  [`bridge.rs:1469`](../../rust/motion-engine/src/bridge.rs#L1469)

**Wire fault + klippy surfacing**

- `EthercatEndpointDied = -203` in the `-2xx` host family.
  [`error.rs:179`](../../rust/runtime/src/error.rs#L179)
- klippy reports the `-203` cause as the shutdown reason, no auto-restart.
  [`ethercat_node.py:186`](../../klippy/extras/ethercat_node.py#L186)

**Tests**

- First-cause-wins + `(fault -203)` message format.
  [`bridge/tests.rs`](../../rust/motion-engine/src/bridge/tests.rs)
- Endpoint-death poll takes precedence and shuts down.
  [`test_servo_param.py`](../../test/test_servo_param.py)
