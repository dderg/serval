---
title: 'EtherCAT Sensorless Homing'
type: 'feature'
created: '2026-06-26'
status: 'done'
baseline_commit: '22f67e1aeaa45ab36624d68b5c163bbe7c635d88'
context:
  - '{project-root}/_bmad-output/specs/spec-ecat-sensorless-homing/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-ecat-sensorless-homing/design.md'
  - '{project-root}/_bmad-output/specs/spec-ecat-sensorless-homing/drive-reference.md'
  - '{project-root}/_bmad-output/project-context.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** An EtherCAT servo axis (Inovance A6-EC, CiA 402) cannot home without a separate MCU's endstop pin: the CSP drip ends a homing move on an external trigger, and nothing in the real-time edge turns a *missed track* (carriage hits the hard stop, drive can no longer follow the streamed position) into a homing event. Sensorless servo machines therefore either fault the drive (Er47.x) or cannot home at all.

**Approach:** Model the servo as a **host-side virtual endstop, like a probe's `z_virtual_endstop`**, pointed at the `ethercat_node` engine instead of a separate MCU. The planner keeps the coordinated CSP `home_drip`. Before the move we arm: relax the drive following-error window (`6065h`) and cap torque so a stalled track does not fault. While armed, `ethercat-rt` watches **actual torque (`6077h`)** each DC cycle; when it crosses a gentle threshold it **freezes all targets on the loop in the same cycle** (local stop, mirroring the regular MCU's in-cycle endstop stop) and emits one `EndstopTrip{endstop_id, trip_clock}`. The existing `dispatch_endstop_trip` → reconstruct → inverse-kin path consumes it; then restore limits (always) and latch the home frame via `SeedServoHome`.

## Boundaries & Constraints

**Always:**
- Planner owns the trajectory: existing coordinated CSP `home_drip` cohort only. Never the drive's autonomous Homing Mode (HM, `6098h` methods −1/−2) in any role.
- Trip on **actual torque (`6077h`)** in CSP, never on position-deviation distance. Torque sets contact force directly (gentleness). Cap torque (`6072h`/`60E0h`/`60E1h`) while armed.
- Gentleness is a hard requirement, met by two bounds: peak force = the homing torque cap; press duration ≈ one DC cycle via **local stop inside `ethercat-rt`** (no host round-trip).
- Run-time drive protection MUST always be restored (following-error window + max-torque), on every exit path incl. abort/error. This is a safety invariant — already enforced by `homing.py:_servo_drive_limits` try/finally; do not weaken it.
- The relaxed `6065h` window must exceed the torque-trip threshold so detection fires before the drive's own deviation fault.
- Coupled motors of one axis share one EtherCAT DC loop (one `ethercat-rt`); the first drive to detect stops all motion on that loop under one DC timebase. No cross-MCU clock reconciliation.
- Reuse the existing trip transport and IDs: `EndstopTrip` message, `dispatch_endstop_trip`, `home_axis_start`/`home_axis_poll`, the drip cohort, `SetDriveLimits`/`RestoreDriveLimits`/`SeedServoHome`, and `allocate_provider_id` for the `endstop_id`. Keep the `homing.rs` window/clock guards.
- Fail loudly (project rule): a trip clock predating the homing window, a stale/early trip, a move that exhausts `max_travel` with no trip, or a drive that faults despite the relaxed window must raise a clear error — never pad, advance, or silently retry.
- Edit lives at the EtherCAT real-time edge (`ethercat-rt` daemon + `motion-engine`/`host-rt` bridge + thin host glue). Printer-MCU firmware (H7/F446 step/dir) is NOT modified. C/Rust boundary rules in `docs/rewrite/mcu-c-rust-boundary.md` apply to new shared state; `libecrt.c` is C, the loop logic is Rust.
- `ethercat-rt-stub` must mirror the new hw FFI/wire surface so the CI-able stub build catches drift.

**Ask First:**
- Full **mcu-sim** end-to-end repro of all three kinematics: `tools/mcu-sim/runner.py` runs only C firmware and has **no `ethercat-rt` integration today**. Building that sim harness is a separate, larger lift. Default plan: cover the trigger with an `ethercat-rt`(+stub) integration test (arm → simulated torque cross → `EndstopTrip` frame) and Rust/Python unit tests, and validate the three kinematics **on the bench**. Confirm before investing in mcu-sim ethercat wiring.
- If the `ethercat-rt` `trip_clock` domain does not line up with the engine's `window_start_clock` for the node (the SPEC's "GAP/verify" item), and reconciling needs a clock-mapping change beyond stamping monotonic-ns aligned to piece start: HALT and confirm the approach.

**Never:**
- Stepper/TMC stallguard homing, or any new printer-MCU (H7/F446) firmware homing path.
- Drive-autonomous HM (−1/−2/1/2/35-as-finder) as a mechanism or single-motor fallback. (`SeedServoHome`'s method-35 frame-seed after our own trip+back-off stays.)
- A new `endstop_id` scheme, a parallel homing transport, or a new torque-tuning knob (reuse `homing_max_torque`).
- Redesigning probing/bed-mesh or the second-stage fine re-home beyond what the existing `use_sensorless_homing` flag already toggles.
- Reusing `arm_remote_trigger`/the trsync frame-interceptor for the servo: the ethercat node has no trsync OID. Arming is a new device-side command to `ethercat-rt` (see Design Notes).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Gentle contact | Armed; `6077h` ≥ trip threshold (optionally `606Ch`≈0) | Freeze all loop targets that DC cycle; emit one `EndstopTrip{endstop_id, trip_clock}`; latch fired-once | N/A |
| Below threshold | Armed; `6077h` < threshold | Keep dripping CSP; no trip | N/A |
| Already fired | Armed; second cross same arm | No second `EndstopTrip` (once-only latch) | N/A |
| Exhausted travel | Armed; `home_drip` reaches `max_travel`, no trip | No advance, no retry | `home_axis_poll`/host deadline raises a clear no-trigger error |
| Stale/early trip | `trip_clock` ≤ `window_start_clock` | Reject | `homing.rs:63` returns `Err` (kept) |
| Disarm on any exit | Move ends by trip/timeout/abort/error | Run-time `following_error`+`max_torque` restored; armed state cleared | `_servo_drive_limits` restore runs on exception path |
| Drive faults while armed | Er47.0/Er47.1 despite relaxed window | Surface fault, abort home | `_check_servo_drive_fault` raises G28 error |

</frozen-after-approval>

## Code Map

- `rust/mcu-protocol/src/messages.rs` -- `EndstopTrip`(:958) `{endstop_id:u8, trip_clock:u64}`, `SetDriveLimits`(:639), `MessageKind`(:8). Add `ArmSensorlessEndstop` kind+struct.
- `rust/ethercat-rt/src/wire.rs` -- `decode_command`(:88), response frames; add arm decode + `endstop_trip_frame` (CHANNEL_EVENTS emit).
- `rust/ethercat-rt/src/bin/ethercat-rt.rs` -- DC loop (cycle :855/:876), command match (:306-586), `ec_rt_set_target_position` write (:807), `cycle_index`(:279)/`monotonic_ns`. Add armed comparator + in-cycle local stop + once-only emit.
- `rust/ethercat-rt/src/bin/ethercat-rt-stub.rs` -- mirror armed comparator/emit (no FFI; in-mem torque source for tests).
- `rust/ethercat-rt/csrc/libecrt.c` -- `ec_rt_get_torque_actual`(:455), `ec_rt_get_velocity_actual`(:449), target write (:447). Local-stop = hold last commanded target on the loop.
- `rust/ethercat-rt/src/curves.rs` -- `AxisRing.armed`(:49) motion-ring sampler; local-stop freeze hook.
- `rust/host-rt/src/mcu_serial_conn.rs` -- `run_reader`(:232)/`route_frame`(:267)/`dispatch_frame`(:306, StatusHeartbeat-only). Extend to decode inbound `EndstopTrip` → registered callback.
- `rust/motion-engine/src/bridge.rs` -- `claim_ethercat_node`(:954), `home_axis_start`(:3848)/`home_axis_poll`(:4027), `handle_endstop_trip`(:2316), `dispatch_endstop_trip`(:4444), endpoint event wiring (node `mcu_handle`). Add: register endpoint EndstopTrip callback → `handle_endstop_trip(node_handle,…)`; add `arm_sensorless_endstop` engine fn.
- `rust/motion-engine/src/homing.rs` -- window/clock guard (:63). Keep; verify ethercat `trip_clock` domain.
- `klippy/extras/sim_remote_endstop.py` -- provider template: `setup_motion_endstop`, `trip_move_begin`/`trip_move_end`, chip registration.
- `klippy/extras/servo_axis.py` -- config (:67-91), `get_homing_drive_limits`(:154), `get_endstops`(:120). Add virtual-endstop pin + sensorless provider.
- `klippy/extras/ethercat_node.py` -- `get_engine_handle`(:153). Path to send the arm/disarm to `ethercat-rt`.
- `klippy/motion_engine.py` -- wrappers near `arm_remote_trigger`(:460). Add `arm_sensorless_endstop`/`disarm`.
- `klippy/extras/homing.py` -- `trip_move`(:504), `_provider_entry`(:282), `_servo_drive_limits`(:27), `finalize_homed_axis`(:115). Provider hooks already exist; wire servo provider in.
- `test/test_servo_homing.py`, `rust/ethercat-rt/tests/{torque_lifecycle,stub_loop}.rs`, `rust/motion-engine/src/servo_torque/tests.rs` -- extend.

## Tasks & Acceptance

**Execution:**
- [x] `rust/mcu-protocol/src/messages.rs` -- Add `ArmSensorlessEndstop { endstop_id: u8, torque_trip_tenth_pct: u16, enable: u8 }` + `MessageKind` variant + `Encode`/`Decode` -- host→`ethercat-rt` arm/disarm carrying the trip threshold and the provider `endstop_id` to stamp. Reuse existing `EndstopTrip` for the reverse direction.
- [x] `rust/ethercat-rt/src/wire.rs` -- Decode `ArmSensorlessEndstop` into a `Command`; add `endstop_trip_frame(endstop_id, trip_clock)` on `CHANNEL_EVENTS` mirroring response-frame helpers -- the wire surface for arm-in and trip-out.
- [x] `rust/ethercat-rt/src/bin/ethercat-rt.rs` (+ `curves.rs`, `libecrt.c` freeze) -- Add an armed state (`endstop_id`, `torque_trip_tenth_pct`, `fired` latch) set by the arm command. Each DC cycle while armed: read `ec_rt_get_torque_actual()`; if `|torque| ≥ threshold`, freeze all loop targets that cycle (hold last commanded target via the local-stop path, not the host Stop), stamp `trip_clock` (engine-clock = `monotonic_ns` aligned to piece start), emit one `EndstopTrip`, set `fired`. Disarm clears state -- the keystone trigger; in-cycle local stop bounds press duration.
- [x] `rust/ethercat-rt/src/bin/ethercat-rt-stub.rs` -- Mirror the armed comparator + emit against an in-memory torque source -- keeps stub/hw parity and makes the trigger CI-testable.
- [x] `rust/host-rt/src/mcu_serial_conn.rs` -- Extend `dispatch_frame` to decode `EndstopTrip` on `CHANNEL_EVENTS` and invoke a registered trip callback (alongside `StatusHeartbeat`) -- without this the emitted trip is silently dropped for the ethercat endpoint.
- [x] `rust/motion-engine/src/bridge.rs` -- In `claim_ethercat_node`, register a trip callback on `endpoint_conn` that calls `handle_endstop_trip(node_handle, endstop_id, trip_clock)` (the same entry the serial path uses → `dispatch_endstop_trip`). Add `arm_sensorless_endstop(handle, endstop_id, torque_trip_tenth_pct, enable)` that sends `ArmSensorlessEndstop` to the node -- routes the RT trip into the existing reconstruction path; no change to `dispatch_endstop_trip`/`homing.rs`.
- [x] `klippy/motion_engine.py` -- Add `arm_sensorless_endstop`/`disarm_sensorless_endstop` wrappers over the engine fn -- host→engine surface.
- [x] `klippy/extras/servo_axis.py` (+ `ethercat_node.py`) -- Register a pins chip exposing a virtual endstop pin (e.g. `<name>:virtual_endstop`); provider implements `setup_motion_endstop(pin_params, axis)` (returns an endstop whose `engine_mcu_handle()` = the node handle, `endstop_id` from `allocate_provider_id`, benign `arm`/`disarm`), `get_position_endstop()` = `position_endstop`, and `trip_move_begin`/`trip_move_end` that call `engine.arm_sensorless_endstop(handle, endstop_id, max_torque_tenth_pct, enable)`. Wire it so a `[axis]` with `endstop_pin = <name>:virtual_endstop` homes through this path -- models the servo as a probe-like virtual endstop; reuses `homing.py`'s provider + `_servo_drive_limits` + `finalize_homed_axis` unchanged.
- [x] `rust/ethercat-rt/.../tests` + `rust/mcu-protocol` tests -- Unit-test the armed comparator (threshold cross, below-threshold no-op, once-only latch, disarm clears) and `ArmSensorlessEndstop` codec; integration-test against the stub: arm → torque cross → `EndstopTrip` frame on `CHANNEL_EVENTS` -- covers the I/O matrix rows.
- [x] `rust/host-rt` test + `test/test_servo_homing.py` -- host-rt: inbound `EndstopTrip` frame → trip callback fires. Python: virtual-endstop provider arms/disarms via engine, `trip_move_begin/end` send arm/disarm, restore runs on the exception path, no-trigger raises -- fail-loudly negative tests included.

**Acceptance Criteria:**
- Given a sensorless-configured EtherCAT axis (no `endstop_pin` pin, `endstop_pin = <node>:virtual_endstop`), when `G28` runs, then the axis drips CSP into the hard stop, the drive raises no Er47.0/Er47.1, and the axis ends marked homed at `position_endstop` with the carriage backed off the stop.
- Given the axis is armed, when actual torque first crosses the threshold, then `ethercat-rt` freezes all loop targets in that DC cycle and emits exactly one `EndstopTrip` whose `trip_clock` falls inside the active homing window, which `dispatch_endstop_trip` reconstructs and inverse-kins to the cartesian trip position.
- Given a coupled axis (CoreXY A/B or AWD pair on one DC loop), when either drive detects the missed track, then motion on the whole loop stops and the reconstructed cartesian trip position is correct under inverse kinematics.
- Given the homing move ends by any path (trip, timeout, abort, drive fault), when control returns, then the drive's run-time `following_error` window and `max_torque` read back the configured run-time values, never the relaxed homing values.
- Given `home_drip` reaches `max_travel` with no torque cross, when the deadline elapses, then homing fails loudly with a no-trigger error — no advance, no silent success.
- Given a `trip_clock` ≤ `window_start_clock`, when dispatched, then `homing.rs` returns an error (stale/mis-synced trip).

## Spec Change Log

- 2026-06-26 (review patch) — Adversarial review found the armed comparator could trip on a drive's pre-existing holding torque the instant it is armed (e.g. a Z axis held against gravity), before any homing motion — a silent false-home. Fixed in `SensorlessArm::poll`: the latch now requires one strictly-below-threshold observation before it may fire (rising-edge). A misconfigured too-low cap now fails loudly via the `max_travel` no-trigger deadline instead of false-homing, and the trip is guaranteed to land inside the homing window (resolves the arm-before-`window_start_clock` ordering concern too). Also: removed unused `SensorlessArm` getters and narrative aside-comments; the first integration test now asserts on a `fired` sentinel rather than `trip_clock != 0`. Several lower-severity / pre-existing items were deferred — see `deferred-work.md`.
- 2026-06-26 (implementation) — The servo provider intentionally does **not** implement `get_position_endstop()`, contrary to the Task-9 wording. `homing.py:_provider_entry` forbids `position_endstop` in the `[axis]` config when the provider supplies a trigger position, but `ServoRail` requires `position_endstop` (for `infer_positive_dir` and the homed frame). The trigger position therefore falls back to `hi.position_endstop` (the rail's config), which is identical. ACs unaffected. The provider lives on `ServoRail` itself (registers pin chip `servo_<axis>`; endstop pin `servo_<axis>:virtual_endstop`); `ethercat_node.py` was not modified — the rail looks up the node for the engine handle via its existing `get_node_name`. `curves.rs`/`libecrt.c` were not modified: the existing `ring.reset()` (the `Stop` path) already holds the last commanded target, so it *is* the in-cycle local stop. Message kind `ArmSensorlessEndstop` placed at `0x006E` (command range) so its response stays out of the `0x0080..=0x00BF` event range.

## Design Notes

**Why a new arm command, not `arm_remote_trigger`.** The probe/separate-MCU path arms via `arm_remote_trigger(engine_mcu_handle, trsync_oid, endstop_id)`, which registers a frame interceptor on the MCU's `trsync_state` OID. The ethercat node has **no trsync OID** and no `trsync_state` stream, so that path is inapplicable. Instead the servo provider's `trip_move_begin`/`trip_move_end` (the same device-side-arming hooks `sim_remote_endstop.py` uses) send a new `ArmSensorlessEndstop` to `ethercat-rt`; `ethercat-rt` reports back via the existing `EndstopTrip` message. The `endstop_id` still comes from `allocate_provider_id` and the trip still flows through `dispatch_endstop_trip` unchanged — only the arm transport and the trip *emitter* differ.

**The three real gaps** (everything else exists on-branch): (1) the armed torque comparator + in-cycle local stop + once-only `EndstopTrip` emit in the `ethercat-rt` DC loop; (2) host-rt routing of an inbound `EndstopTrip` from the ethercat endpoint into a `RuntimeEvent` — `mcu_serial_conn.rs:dispatch_frame` currently decodes only `StatusHeartbeat` and **silently drops** everything else; (3) the host-py servo virtual-endstop provider + the `ArmSensorlessEndstop` message. For a pure-servo cohort, `dispatch_endstop_trip` already gets `discard_clock` from the node's Stop response and `motion_history` from the planner dispatch record (node nominal freq = `ETHERCAT_CLOCK_FREQ_HZ`), so reconstruction works without a separate stepper MCU.

**Local stop = hold last target.** The regular MCU does `handle_stop_inner()` in-cycle on trigger, then reports. `ethercat-rt`'s equivalent is to stop advancing the motion ring and hold the last commanded `target_position` on every drive of the loop *in the detecting cycle*, then emit. The host's later `Stop` broadcast (from `dispatch_endstop_trip`) confirms and returns `discard_clock` — same ordering as the MCU.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p mcu-protocol -p ethercat-rt -p host-rt -p motion-engine` -- expected: new comparator/codec/routing/integration tests green.
- `cd rust && cargo build -p ethercat-rt` (stub, no `--features hw`) -- expected: stub mirrors the new wire/arm surface, builds clean.
- `./scripts/ci.sh quick` -- expected: ruff + rust-test + clippy(`-D warnings`) + fmt + watchdog all green.
- `./scripts/ci.sh py` -- expected: `test/test_servo_homing.py` (touches `klippy/`) green.
- `cargo test --doc -p mcu-protocol` -- expected: only if doc examples touched.

**Manual checks (bench, no endstop wiring):**
- On a CoreXY (and a cartesian, and an AWD) servo bench: `G28` drives each homed axis gently into the stop; the drive shows no Er47.x; the axis ends homed at `position_endstop`; back-off clears the stop; repeated `G28` lands within homing repeatability tolerance across N runs.
- After homing by trip/timeout/abort, read back `6065h`/`6072h` (or the engine's run-time limits) and confirm they equal the configured run-time values, not the relaxed homing values.

## Suggested Review Order

**The keystone trigger (real-time edge)**

- Entry point — the armed torque comparator with the rising-edge latch; the whole design intent in one small pure type.
  [`sensorless.rs:22`](../../rust/ethercat-rt/src/sensorless.rs#L22)
- Per-cycle: read actual torque, on trip freeze the loop in-cycle (`ring.reset`) and emit one trip.
  [`ethercat-rt.rs:787`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L787)
- Arm/disarm command handler; rejects a zero threshold loudly.
  [`ethercat-rt.rs:501`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L501)
- The trip emitted as an event on `CHANNEL_EVENTS`, mirroring the heartbeat frame.
  [`wire.rs:290`](../../rust/ethercat-rt/src/wire.rs#L290)

**Protocol surface**

- New host→daemon command carrying endstop_id + trip threshold + enable.
  [`messages.rs:643`](../../rust/mcu-protocol/src/messages.rs#L643)

**Routing the trip back to the engine**

- The fix that makes the trip reach the engine: inbound `EndstopTrip` was silently dropped before.
  [`mcu_serial_conn.rs:350`](../../rust/host-rt/src/mcu_serial_conn.rs#L350)
- Endpoint trip callback → existing `dispatch_endstop_trip` (same path the serial endstop uses).
  [`bridge.rs:3014`](../../rust/motion-engine/src/bridge.rs#L3014)
- The PyO3 `arm_sensorless_endstop` engine method.
  [`bridge.rs:1206`](../../rust/motion-engine/src/bridge.rs#L1206)
- Transport for the arm command.
  [`servo_torque.rs:67`](../../rust/motion-engine/src/servo_torque.rs#L67)

**Host-side virtual endstop (the probe-like model)**

- Provider arms ethercat-rt's torque trigger before the drip; threshold = the homing torque cap.
  [`servo_axis.py:224`](../../klippy/extras/servo_axis.py#L224)
- Exposes the `virtual_endstop` pin and returns a benign-arm endstop bound to the node handle.
  [`servo_axis.py:196`](../../klippy/extras/servo_axis.py#L196)
- Thin engine wrappers.
  [`motion_engine.py:177`](../../klippy/motion_engine.py#L177)

**Stub parity + tests (peripherals)**

- Stub mirrors the comparator against an SDO-injectable torque so the trigger is CI-testable.
  [`ethercat-rt-stub.rs:438`](../../rust/ethercat-rt/src/bin/ethercat-rt-stub.rs#L438)
- End-to-end: arm → torque cross → `EndstopTrip` through the real stub binary.
  [`sensorless_homing.rs:1`](../../rust/ethercat-rt/tests/sensorless_homing.rs#L1)
- Comparator unit tests (cross, below, once-only, never-trip-if-preloaded).
  [`sensorless/tests.rs:1`](../../rust/ethercat-rt/src/sensorless/tests.rs#L1)
- Inbound trip routing test.
  [`mcu_serial_conn/tests.rs:1`](../../rust/host-rt/src/mcu_serial_conn/tests.rs#L1)
- Python provider arm/disarm tests.
  [`test_servo_homing.py:1`](../../test/test_servo_homing.py#L1)
