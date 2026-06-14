# Drive-Framed Servo Position (method-35) + Overshoot-Corrected Retract — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Make EtherCAT servo live position correct (absolute, at rest and moving) by letting the **drive own its frame** via CiA-402 homing method 35 at homing, and fix the homing retract to compensate for overshoot (uniformly, all motor types).

**Architecture:** Klipper homing stays as-is (endstop trip, servo-guarded reduced-torque trip — already in `sota`). Two changes: (1) the retract folds in the measured overshoot; (2) after the final retract, the host calls a uniform `finalize_homed_axis(handle, axis, pos_mm)`; the bridge dispatches per type — stepper no-op (already seeded by the move), EtherCAT → endpoint runs method 35 (`6098=35`, `607C=pos×cpm`, control-word `0x0F→0x1F`, wait status-word bit 12, back to CSP), zeroing `6064h` to our frame. The endpoint then reports `6064h / counts_per_mm` (drop the transient `cmap`).

**Tech:** Python (`klippy/extras/homing.py`), Rust (`kalico-protocol`, `motion-bridge`, `kalico-ethercat-rt` endpoint + `bench/libecrt.c` FFI).

**Spec:** `docs/superpowers/specs/2026-06-14-homing-seed-position-limits-design.md`.

**Conventions:** `cargo nextest run` from `rust/`; `./scripts/ci.sh quick` (+ `py` if `klippy/` touched); `cargo fmt --all --check` last. Comments discouraged; tests in separate files; fail-loud. No Co-Authored-By trailer.

**Post-merge integration (already exists — reuse, don't duplicate):**
- `homing.py:270-327` servo path: `servo_handle = node.get_bridge_handle()`, `servo_limits = rail.get_homing_drive_limits()` (torque/ferr), `_run_servo_guarded_trip` (reduced-torque trip), `set_position` at `:305`, retract at `:322-326`, `overshoot = final_pos[axis] - trip_pos[axis]`.
- `bridge.set_drive_limits/restore_drive_limits` → SDO `0x6065`/`0x6072` (torque/ferr — NOT position; unrelated to this work).
- Endpoint: `ec_rt_sdo_write(index, sub, buf, abort)` exists (`bench/libecrt.c`), `ec_rt_get_position_actual` (`6064h`), `ec_rt_get_velocity_actual` (`606Ch`). The `cmap` reporting bug: `kalico-ethercat-rt.rs:163` (decl), `:464-476` (QueryMotorState), `:616-619` (create), reset at `:396,593,656,705`.

**Out of scope:** software position limits (`607D`), `min_homing_distance`, drive sensorless homing.

---

## Task 1: Overshoot-corrected retract (uniform, host-side)

**Files:** `klippy/extras/homing.py` (`_home_axis`, ~322-326); Test: `test/test_homing_*` (mirror an existing homing test — `test/test_servo_homing.py` / `test_homing_enable.py` exist).

- [ ] **Step 1: Read the current retract + confirm overshoot is available.** `homing.py:319` already computes `overshoot = final_pos[axis] - trip_pos[axis]`; the retract at `:322-325` uses fixed `hi.retract_dist`. Bind `overshoot` before the retract block.

- [ ] **Step 2: Write the failing test.** In the homing test file, drive `_home_axis` (or the retract computation) with a fake trip where `final_pos - trip_pos = overshoot ≠ 0`, and assert the retract target is `start − direction*(retract_dist + overshoot)` (i.e. lands at `trigger − retract_dist`, independent of overshoot). Mirror the existing homing test's fakes. Run → FAIL.

- [ ] **Step 3: Implement.** In `_home_axis`, change the retract to fold in overshoot:
```python
            if hi.retract_dist:
                overshoot = final_pos[axis] - trip_pos[axis]
                retractpos = list(toolhead.get_position())
                retractpos[axis] -= direction * (hi.retract_dist + overshoot)
                toolhead.move(retractpos, hi.retract_speed)
                toolhead.wait_moves()
```
(`set_position` at `:305` stays — the toolhead needs a logical position for the retract move; it already sets `trigger_height + overshoot`, so `get_position()` here is `trigger + overshoot`, and subtracting `retract_dist + overshoot` lands at `trigger − retract_dist`.)

- [ ] **Step 4: Run test → PASS.** Then `./scripts/ci.sh py` (the homing suite must stay green — this changes stepper homing too).

- [ ] **Step 5: Commit** `fix(homing): retract compensates for endstop overshoot (all motor types)`.

---

## Task 2: Endpoint method-35 home-set (protocol + wire + FFI + handshake)

**Files:** `rust/kalico-protocol/src/messages.rs` (+ `schema_def.rs`), `rust/kalico-ethercat-rt/src/wire.rs` (+ tests), `bench/libecrt.c`/`.h` + `rust/kalico-ethercat-rt/src/ffi.rs`, `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`.

This is the riskiest task (live mode-switch). Build it incrementally.

- [ ] **Step 1: Protocol message.** Add `MessageKind::SeedServoHome` (request) + `SeedServoHomeResponse` (ack `result: i32`) in `messages.rs` (mirror `SetDriveLimits`/`SetDriveLimitsResponse`), body `SeedServoHome { home_q16: i32 }` (home mm in Q16). Add to `from_u16` + `schema_def.rs`. Codec round-trip test. (Discriminants: next free pair — verify against the enum.)

- [ ] **Step 2: wire.rs.** Add `Command::SeedServoHome { correlation_id, home_q16 }` decode + `seed_servo_home_response_frame(cid, result)` (mirror `set_drive_limits_response_frame`, `wire.rs:239`). Add the no-op arm in the stub bins. Test (decode + response round-trip).

- [ ] **Step 3: FFI for mode/control/status.** READ `bench/libecrt.c` to find how mode-of-operation (`6060`), control-word (`6040`), status-word (`6041`) are accessed today — are they in the RxPDO/TxPDO (`out_t`/`in_t`) or only via SDO? (The RxPDO `1600` default has `6040` control-word; `6060` mode is likely SDO.) Add the minimal C FFI needed, e.g.:
  - `ec_rt_sdo_write(0x6060, 0, &mode_i8)` to switch mode (reuse existing `ec_rt_sdo_write`),
  - `ec_rt_sdo_write(0x6098, 0, &method_i8=35)`, `ec_rt_sdo_write(0x607C, 0, &offset_i32)`,
  - control-word + status-word: if `6040`/`6041` are PDO fields (`out_t.controlword` / `in_t.statusword` — `statusword` already in `in_t`), drive them via the existing PDO exchange (`rt_exchange`); else add `ec_rt_sdo_write(0x6040,...)` / `ec_rt_sdo_read(0x6041,...)`.
  Confirm and add only what's missing; declare matching externs in `ffi.rs`.

- [ ] **Step 4: Implement the method-35 handshake (endpoint).** Add a handler for `Command::SeedServoHome { home_q16 }`. Sequence (no motion; runs while parked at the post-retract spot):
  1. Compute `offset_counts = (home_mm * counts_per_mm).round() as i32`.
  2. SDO: `6098 = 35` (homing method), `607C = offset_counts`.
  3. Switch `6060 = 6` (homing mode); wait for `6061` (mode display) == 6.
  4. Control word: ensure `0x0F` (enabled), then set bit 4 → `0x1F` (homing start); keep bit 4 set.
  5. Poll status word `6041`: success = bit 12 (homing attained) set & bit 13 (error) clear; fail = bit 13 set → return error. Bound the poll with a timeout (fail-loud → respond `result<0`).
  6. Clear bit 4 (`0x1F`→`0x0F`); switch `6060 = 8` (CSP); wait mode display == 8.
  7. Set a persistent `framed: bool = true` (used by Task 3 reporting). Respond ack.
  Coordinate with the gate/torque state machine: the toolhead is parked (no streaming) at this point; ensure the DC exchange continues during the handshake. Reference the control-word constants and the `0x0F→0x1F` pattern from the A6-EC manual (§homing mode control word).

- [ ] **Step 5: Build + endpoint tests.** `cargo build -p kalico-ethercat-rt`; `cargo nextest run -p kalico-ethercat-rt`; clippy `-D warnings`; fmt. (The handshake itself is bench-verified; unit-test the wire/codec + any pure helpers.)

- [ ] **Step 6: Commit** `feat(ethercat): SeedServoHome via CiA-402 method 35 (drive-frames 6064h)`.

---

## Task 3: Bridge dispatch + host finalize-after-retract

**Files:** `rust/motion-bridge/src/bridge.rs` (+ `klippy/motion_bridge.py` passthrough), `klippy/extras/homing.py`.

- [ ] **Step 1: Bridge method.** Add pyo3 `finalize_homed_axis(&self, mcu_handle, axis, pos_mm)`:
  - EtherCAT handle (`ethercat_socket.is_some()`): `kalico_call(MessageKind::SeedServoHome, encode(home_q16 = pos_mm*65536), timeout)` on `endpoint_conn`; error on non-ack/`result<0` (fail-loud). Model on `set_drive_limits` (`bridge.rs:1121`).
  - Non-EtherCAT handle: **no-op** (stepper already seeded by `set_position` + the retract move). Return Ok.
  Add the `klippy/motion_bridge.py` `MotionBridgeWrapper` passthrough (mirror `set_drive_limits`).

- [ ] **Step 2: Host call after the final retract.** In `homing.py` `_home_axis`, after the retract `wait_moves()` (`:326`) and before `_check_servo_drive_fault`, add a uniform finalize using the handle the servo path already resolves (`servo_handle`); for steppers pass the stepper MCU handle (or skip — no-op). Keep it uniform: resolve the axis's bridge handle and call `bridge.finalize_homed_axis(handle, axis, toolhead.get_position()[axis])`. (For servo, `servo_handle` is already in scope at `:276`.) This is host-uniform; the per-type behavior is in the bridge.

- [ ] **Step 3: Build + tests.** `cargo build -p motion-bridge`; `cargo nextest run -p motion-bridge`; `./scripts/ci.sh py` (homing green). clippy/fmt.

- [ ] **Step 4: Commit** `feat(bridge): finalize_homed_axis — EtherCAT method-35 home-set after retract; stepper no-op`.

---

## Task 4: Drive-framed EtherCAT reporting (remove transient cmap)

**Files:** `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`.

- [ ] **Step 1: Report from the drive frame.** Change `QueryMotorState` (`:464-476`): if `framed` (set by Task 2 method-35), report `pos_mm = 6064h / counts_per_mm` (drive-framed, absolute, persistent) and `vel_mm_s = velocity_mm_s(606Ch, rotation_distance)`; if not yet `framed`, respond empty (unhomed — honest, no fabricated number).
```rust
Command::QueryMotorState { correlation_id } => {
    if framed {
        let (pos_counts, vel_rpm) = unsafe {
            (ffi::ec_rt_get_position_actual(), ffi::ec_rt_get_velocity_actual())
        };
        let pos_mm = f64::from(pos_counts) / counts_per_mm;
        let vel_mm_s = kalico_ethercat_rt::scale::velocity_mm_s(vel_rpm, rotation_distance);
        server.respond(&motor_state_response_frame(correlation_id, pos_mm, vel_mm_s));
    } else {
        server.respond(&motor_state_empty_frame(correlation_id));
    }
}
```

- [ ] **Step 2: Remove the transient cmap *reporting* dependency.** The `cmap` is still used for *commanding* (`target_counts`, `:616-620`) — KEEP that. Only the *reporting* path moves off `cmap` (Step 1). Do not remove `cmap` itself. Remove `CountMap::actual_mm` use from the reporting path (it may become unused → drop it + its test only if nothing else uses it).

- [ ] **Step 3: Build + tests + clippy/fmt.** `cargo nextest run -p kalico-ethercat-rt`.

- [ ] **Step 4: Commit** `fix(ethercat): report drive-framed 6064h/cpm after method-35; drop transient-cmap reporting`.

---

## Task 5: End-to-end verification

**Files:** none.

- [ ] **Step 1: CI.** `./scripts/ci.sh quick` + `./scripts/ci.sh py` green (`.so` workaround if needed); `cargo fmt --all --check`.
- [ ] **Step 2: Sim regression (steppers).** kalico-sim: home + a move on an all-stepper config — confirm the overshoot-corrected retract didn't regress stepper homing, and stepper live position still reads correctly. (Sim can't model the A6-EC servo.)
- [ ] **Step 3: Bench (A6-EC, Neptune — user-triggered flash).** Home X. Verify: live position reads **absolute and correct at rest** (was 0) **and tracks while moving** (was the diff); `GET_POSITION` agrees; after a power-cycle the position persists (absolute encoder); the retract clears the rammed end (servo backs off, not pushing). Confirm `home is at <configured>` reads right (test a non-zero home offset).
- [ ] **Step 4: Bench velocity scale check** (carried from the earlier plan): confirm `velocity_mm_s(606C rpm, rotation_distance)` ≈ `Δpos/Δt` ≈ commanded during a steady move; adjust the rpm/gear factor if off.

---

## Self-review notes (author)
- Spec coverage: overshoot retract → T1; method-35 framing → T2; bridge dispatch + post-retract finalize → T3; drive-framed reporting → T4; verify → T5. Position limits (`607D`) intentionally excluded (optional).
- Integrates with sota's servo path (`servo_handle`, `_home_axis`, `set_drive_limits` torque/ferr) — does not duplicate it.
- Riskiest: T2 method-35 handshake (live mode-switch). Build wire/codec first, then the handshake; bench-verify. T2 Step 3/4 include read-the-code for the exact `6060`/`6040`/`6041` PDO-vs-SDO mechanism.
- T1 changes stepper homing too (uniform) — `ci.sh py` homing suite is the guard.
- Reporting (T4) returns empty until `framed` — honest pre-home behavior, no fabricated position.
