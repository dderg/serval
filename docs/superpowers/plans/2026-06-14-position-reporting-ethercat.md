# EtherCAT Servo Live Position Reporting — Implementation Plan (Plan 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Report the honest *actual encoder* position and *native drive velocity* of EtherCAT servo axes, feeding the SAME host surfaces Plan 1 built for steppers (`motion_report` live value, `GET_POSITION`). The velocity must be the drive's real `606Ch` value (intended for tuning, not just display).

**Architecture:** The EtherCAT endpoint already speaks the same `mcu_protocol` request/response as serial MCUs. So the endpoint gains a `QueryMotorState` handler that reads `6064h` (position, reference-unit) and `606Ch` (velocity, rpm), converts both to mm / mm·s⁻¹ using its `CountMap` + `rotation_distance`, and replies with the **same `MotorStateResponse`** Plan 1 defined. The host bridge drops its EtherCAT skip and routes the live-position query to the endpoint, mapping the reply onto the servo's cartesian slot. The host assembly (`assemble_cartesian`, slot→cartesian, the cache + poll thread, `motion_report`, `GET_POSITION`) is **unchanged** — servo slots fill the same `motors[]` array as stepper slots, so mixed stepper+servo machines work automatically.

**Tech Stack:** C (EtherCAT master PDO map — `bench/libecrt.c`/`.h`), Rust (`ethercat-rt`: ffi/capture/scale/wire/endpoint, `motion-engine`), Python (`servo_axis`/`ethercat_node`/`motion_engine` plumbing).

**Spec:** `docs/superpowers/specs/2026-06-14-position-reporting-design.md` §2. This is Plan 2 of 2; Plan 1 (stepper path, merged on this branch) defined `QueryMotorState`/`MotorStateResponse`, `assemble_cartesian`, the cache + poll thread, and the host surfaces.

**Conventions (same as Plan 1):**
- Rust suite: `cargo nextest run` from `rust/` (NOT `cargo test`); scope `-p <crate>` / `-E 'test(<name>)'`.
- Before PR: `./scripts/ci.sh quick` green; `klippy/` changes → also `./scripts/ci.sh py`; `cargo fmt --all --check` last.
- MCU C builds: `./scripts/ci.sh rust-mcu-h7` builds the Rust staticlib; the EtherCAT master C (`bench/libecrt.c`) is host-compiled — verify with the EtherCAT-rt crate build and a `clang -fsyntax-only` of the C against its header.
- Bench firmware flow (if a real A6-EC drive is used for verification): commit → push → pull on Pi → build on Pi → flash. Never scp binaries.
- Comments discouraged; tests in separate files; fail-loud.

**Unit facts (from the A6-EC manual, verify on bench):**
- `6064h` position actual = **reference unit** (load-shaft, post-electronic-gear); same scale as the `607Ah` target we already command, so the existing `counts_per_mm` round-trips. `actual_mm = origin_mm + (actual − origin_counts) / counts_per_mm`.
- `606Ch` velocity actual = **rpm** (motor shaft). `mm/s = (rpm / 60) × rotation_distance`. The electronic gear (`6091h`, default 1:1) relates motor↔load; at 1:1 the motor-rpm→mm/s via `rotation_distance` matches the reference-unit position frame. **Task 9 verifies the scale (and gear) on a real drive before this is trusted for tuning.**

---

## File map

| File | Change |
|---|---|
| `klippy/extras/servo_axis.py` | expose `get_rotation_distance()` |
| `klippy/extras/ethercat_node.py` | pass `rotation_distance` to `claim_ethercat_node` |
| `klippy/motion_engine.py` | `claim_ethercat_node` passthrough gains `rotation_distance` |
| `rust/motion-engine/src/bridge.rs` | `claim_ethercat_node` + `spawn_ethercat_endpoint` thread `rotation_distance` through; remove EtherCAT skip in `collect_motor_positions_inner`, route to endpoint |
| `bench/libecrt.c`, `bench/libecrt.h` | map `0x606C` into TXPDO; add `velocity_actual` to `in_t`/`ec_telemetry_t`; `ec_rt_get_velocity_actual`; size asserts |
| `rust/ethercat-rt/src/ffi.rs` | `EcTelemetry.velocity_actual` + `ec_rt_get_velocity_actual`; size assert 32→36 |
| `rust/ethercat-rt/src/capture.rs` (+`capture/tests.rs`) | `velocity_actual` field + offset + encode + header_json |
| `rust/ethercat-rt/src/scale.rs` (+tests) | `CountMap::actual_mm`; `velocity_mm_s(rpm, rotation_distance)` |
| `rust/ethercat-rt/src/wire.rs` (+tests) | `Command::QueryMotorState`; `motor_state_response_frame(...)` |
| `rust/ethercat-rt/src/bin/ethercat-rt.rs` | parse `rotation_distance` arg; handle `QueryMotorState` |

---

## Task 1: Thread `rotation_distance` to the endpoint

**Files:** `klippy/extras/servo_axis.py`, `klippy/extras/ethercat_node.py`, `klippy/motion_engine.py`, `rust/motion-engine/src/bridge.rs`, `rust/ethercat-rt/src/bin/ethercat-rt.rs`

The endpoint needs `rotation_distance` (mm per motor rev) to convert `606Ch` rpm → mm/s. `counts_per_mm` is already passed at claim time; add `rotation_distance` alongside it.

- [ ] **Step 1: Expose it on ServoRail.** `servo_axis.py` already stores `self.rotation_distance` (line 54). Add an accessor mirroring `get_counts_per_mm` (line 138):
```python
    def get_rotation_distance(self):
        return self.rotation_distance
```
- [ ] **Step 2: Pass from the node claim.** In `ethercat_node.py` `_claim` (~line 74, where `self._counts_per_mm = rail.get_counts_per_mm()`), add `rotation_distance = rail.get_rotation_distance()` and pass it into the `bridge.claim_ethercat_node(...)` call (add the arg in the position the wrapper/bridge expects — keep ordering consistent across Steps 3–4).
- [ ] **Step 3: Python wrapper passthrough.** In `klippy/motion_engine.py`, the `claim_ethercat_node` wrapper forwards args to `self._bridge.claim_ethercat_node(...)`. Add `rotation_distance` to the wrapper signature and forward it.
- [ ] **Step 4: Bridge pyo3 + spawn.** In `rust/motion-engine/src/bridge.rs`, add `rotation_distance: f64` to the `claim_ethercat_node` pyo3 signature (after `counts_per_mm`), and pass it into `spawn_ethercat_endpoint(...)` as a new CLI argument (mirror exactly how `counts_per_mm` is passed as a CLI arg — find `spawn_ethercat_endpoint` and the arg vector). Use a distinct flag name, e.g. `--rotation-distance`.
- [ ] **Step 5: Endpoint parses it.** In `rust/ethercat-rt/src/bin/ethercat-rt.rs`, parse the new `--rotation-distance` arg into an `f64` next to where `counts_per_mm` is parsed, and bind it for later use (Task 7). Mirror the existing arg-parsing style.
- [ ] **Step 6: Build both sides.**
Run: `cd rust && cargo build -p motion-engine -p ethercat-rt`
Expected: clean. (No behavior yet — `rotation_distance` is parsed and stored; used in Task 7.)
- [ ] **Step 7: Python lint + bridge tests.**
Run: `./scripts/ci.sh ruff` (clean) and `cargo nextest run -p motion-engine` (no regressions).
- [ ] **Step 8: Commit**
```bash
git add klippy/extras/servo_axis.py klippy/extras/ethercat_node.py klippy/motion_engine.py rust/motion-engine/src/bridge.rs rust/ethercat-rt/src/bin/ethercat-rt.rs
git commit -m "feat(ethercat): thread rotation_distance to the servo endpoint"
```

---

## Task 2: Map `606Ch` into the EtherCAT master TXPDO (C)

**Files:** `bench/libecrt.c`, `bench/libecrt.h`

Add the `0x606C` velocity-actual object to the TXPDO and surface it through the C telemetry struct + a getter.

- [ ] **Step 1: PDO entry + mapping table.** In `bench/libecrt.c`, after `TXPDO_POSITION_ACTUAL` (~line 105):
```c
#define TXPDO_VELOCITY_ACTUAL COE_ENTRY(0x606C, 0x00, 32)
```
Add `TXPDO_VELOCITY_ACTUAL,` to the `entries[]` array in `rewrite_1a00_entry_table()` (after `TXPDO_POSITION_ACTUAL,`, ~line 143).
- [ ] **Step 2: Process-data input struct.** In the `in_t` struct (~line 30), add after `int32_t position_actual;`:
```c
    int32_t  velocity_actual;
```
Update the `in_t` size assertion (~line 45) to the new size (was 32 → 36).
- [ ] **Step 3: Telemetry struct.** In `bench/libecrt.h` `ec_telemetry_t` (~line 50), add after `int32_t position_actual;`:
```c
    int32_t  velocity_actual;
```
- [ ] **Step 4: Getter + telemetry copy.** In `bench/libecrt.c`, after `ec_rt_get_position_actual` (~line 457):
```c
int32_t  ec_rt_get_velocity_actual(void)        { return g_in->velocity_actual; }
```
In `ec_rt_get_telemetry` (~line 465), add after the `position_actual` copy:
```c
    out->velocity_actual = g_in->velocity_actual;
```
- [ ] **Step 5: Syntax-check the C.**
Run a `clang -fsyntax-only` of `bench/libecrt.c` against its headers (mirror the include flags the EtherCAT-rt build uses; if the file is normally built by the endpoint's build script, run that). Confirm clean. (Real PDO behavior is verified on the bench in Task 9.)
- [ ] **Step 6: Commit**
```bash
git add bench/libecrt.c bench/libecrt.h
git commit -m "feat(ethercat): map 606Ch velocity actual into TXPDO"
```

---

## Task 3: Rust FFI `EcTelemetry.velocity_actual` + getter

**Files:** `rust/ethercat-rt/src/ffi.rs`

- [ ] **Step 1: Add the field.** In `EcTelemetry` (ffi.rs:8-20), add after `pub position_actual: i32,`:
```rust
    pub velocity_actual: i32,
```
The struct must stay `#[repr(C)]` and field order must match the C `ec_telemetry_t` exactly (velocity_actual immediately after position_actual, before `torque_actual`). Match the C ordering from Task 2 Step 3.
- [ ] **Step 2: Update the size assertion.** The `const _: () = assert!(size_of::<EcTelemetry>() == 32, ...)` (ffi.rs:22-25) → `36` (one new i32). Update the assert message to match `bench/libecrt.h`.
- [ ] **Step 3: Add the extern getter.** After `ec_rt_get_position_actual()` (ffi.rs:49):
```rust
    pub fn ec_rt_get_velocity_actual() -> i32;
```
- [ ] **Step 4: Build.**
Run: `cd rust && cargo build -p ethercat-rt`
Expected: clean (the `repr(C)` layout + size assert compile-checks the C/Rust agreement).
- [ ] **Step 5: Commit**
```bash
git add rust/ethercat-rt/src/ffi.rs
git commit -m "feat(ethercat): EcTelemetry.velocity_actual + ec_rt_get_velocity_actual FFI"
```

---

## Task 4: Capture buffer carries `velocity_actual`

**Files:** `rust/ethercat-rt/src/capture.rs`, `rust/ethercat-rt/src/capture/tests.rs`

So the existing servo-capture telemetry (used in Task 9 verification) records the new field.

- [ ] **Step 1: Write the failing test.** In `capture/tests.rs`, extend the `sample(n)` helper (tests.rs:6-18) to set `velocity_actual: n + 4,` (after `position_actual`), and extend the encode round-trip test to assert the velocity field survives encode→decode and lands at its offset. Run it — expect FAIL (no field yet).
- [ ] **Step 2: Add the offset constant.** In `capture.rs`, after `OFF_POSITION_ACTUAL` (line 25), add `const OFF_VELOCITY_ACTUAL: usize = <next free offset>;` — place velocity_actual at the END of the record to avoid shifting existing offsets (set it to the current `RECORD_SIZE`), then bump `RECORD_SIZE` by 4.
- [ ] **Step 3: Struct + encode + header.** Add `pub velocity_actual: i32,` to `DriveSample` (after `position_actual`). In `encode_record`, write it at `OFF_VELOCITY_ACTUAL` (4 LE bytes) mirroring the other i32 fields. In `header_json`, add `("velocity_actual", "i32", OFF_VELOCITY_ACTUAL),` to the field-descriptor list.
- [ ] **Step 4: Populate from telemetry.** Find where `DriveSample` is built from `EcTelemetry` (the DC-loop capture push, `ethercat-rt.rs` ~708) — add `velocity_actual: t.velocity_actual,`. (This is in the bin; if the bin doesn't compile until Task 7, it's fine to add the field here and let Task 7 finish the bin — but prefer adding just this field now so capture is complete.)
- [ ] **Step 5: Run tests.**
Run: `cd rust && cargo nextest run -p ethercat-rt -E 'test(capture)'` → PASS. Then `cargo nextest run -p ethercat-rt` → no regressions. `cargo clippy -p ethercat-rt -- -D warnings` clean.
- [ ] **Step 6: Commit**
```bash
git add rust/ethercat-rt/src/capture.rs rust/ethercat-rt/src/capture/tests.rs rust/ethercat-rt/src/bin/ethercat-rt.rs
git commit -m "feat(ethercat): capture velocity_actual telemetry"
```

---

## Task 5: Counts→mm and rpm→mm/s conversions

**Files:** `rust/ethercat-rt/src/scale.rs`, `rust/ethercat-rt/src/scale/tests.rs` (create if absent, per separate-test-file convention)

- [ ] **Step 1: Write the failing tests.** Add tests for:
  - `CountMap::actual_mm`: with `CountMap::new(counts_per_mm=100.0, actual_counts=1000, pos_mm=5.0)` (origin), `actual_mm(1000) == 5.0` and `actual_mm(1100) == 6.0` (i.e. `5.0 + (1100-1000)/100`). Also assert it's the inverse of `target_counts` on a round-trip.
  - `velocity_mm_s`: `velocity_mm_s(rpm=600, rotation_distance=40.0)` == `(600/60)*40 == 400.0`; `velocity_mm_s(0, _) == 0.0`; negative rpm → negative mm/s.
  Run — expect FAIL.
- [ ] **Step 2: Implement.** In `scale.rs`, add to `impl CountMap` (after `target_counts`, scale.rs:18):
```rust
    pub fn actual_mm(&self, actual_counts: i32) -> f64 {
        self.origin_mm
            + (f64::from(actual_counts) - f64::from(self.origin_counts)) / self.counts_per_mm
    }
```
And a free function (rpm is unit-independent of the CountMap origin):
```rust
pub fn velocity_mm_s(rpm: i32, rotation_distance: f64) -> f64 {
    (f64::from(rpm) / 60.0) * rotation_distance
}
```
- [ ] **Step 3: Run tests + lint.**
Run: `cd rust && cargo nextest run -p ethercat-rt -E 'test(scale)'` → PASS. `cargo clippy -p ethercat-rt -- -D warnings` clean.
- [ ] **Step 4: Commit**
```bash
git add rust/ethercat-rt/src/scale.rs rust/ethercat-rt/src/scale/tests.rs
git commit -m "feat(ethercat): CountMap::actual_mm + velocity_mm_s conversions"
```

---

## Task 6: Endpoint wire — `QueryMotorState` command + response frame

**Files:** `rust/ethercat-rt/src/wire.rs`, `rust/ethercat-rt/src/wire/tests.rs` (or the crate's wire test location)

The endpoint must decode an incoming `QueryMotorState` control frame and be able to build a `MotorStateResponse` frame — reusing `mcu_protocol::messages::MotorStateResponse`/`MotorSample` from Plan 1.

- [ ] **Step 1: Read the existing pattern.** In `wire.rs`, read the `Command` enum + `decode_command` (how `QueryRuntimeCaps`/`Identify` commands are decoded with their `correlation_id`) and an existing `*_response_frame` builder (e.g. `runtime_caps_response_frame`) to mirror framing exactly (`encode_message_header` + `encode_frame(CHANNEL_CONTROL, ...)`).
- [ ] **Step 2: Write the failing test.** Add a test that:
  - decodes a control frame with kind `QueryMotorState` (empty body) into `Command::QueryMotorState { correlation_id }`;
  - `motor_state_response_frame(corr_id, pos_mm, vel_mm_s)` produces a frame that, decoded, yields a `MotorStateResponse` with one `MotorSample { slot: 0, pos_q16: round(pos_mm*65536), vel_q16: round(vel_mm_s*65536) }` and the correct correlation_id/kind.
  Run — expect FAIL.
- [ ] **Step 3: Implement.**
  - Add `QueryMotorState { correlation_id: u32 }` to the `Command` enum and a decode arm in `decode_command` matching `MessageKind::QueryMotorState` (empty body).
  - Add:
```rust
pub fn motor_state_response_frame(correlation_id: u32, pos_mm: f64, vel_mm_s: f64) -> Vec<u8> {
    use mcu_protocol::messages::{Encode, MotorSample, MotorStateResponse};
    let resp = MotorStateResponse {
        motors: vec![MotorSample {
            slot: 0,
            pos_q16: (pos_mm * 65536.0).round() as i32,
            vel_q16: (vel_mm_s * 65536.0).round() as i32,
        }],
    };
    let mut body = Vec::new();
    resp.encode(&mut body);
    // frame it exactly like runtime_caps_response_frame: header(MotorStateResponse, corr_id) + body on CHANNEL_CONTROL
    <build frame mirroring the sibling builder>
}
```
  Also add an EMPTY-response helper or a flag so the endpoint can reply with `motors: vec![]` when no `CountMap` exists yet (pre-home) — e.g. `motor_state_empty_frame(correlation_id)`. (Adapt the exact header/encode calls to the real `wire.rs` helpers; the slot is `0` because the bridge remaps to the cartesian slot via `cfg.axes` in Task 8.)
- [ ] **Step 4: Run tests + lint.**
Run: `cd rust && cargo nextest run -p ethercat-rt -E 'test(wire)'` → PASS (round-trip the q16 values). `cargo clippy -p ethercat-rt -- -D warnings` clean.
- [ ] **Step 5: Commit**
```bash
git add rust/ethercat-rt/src/wire.rs rust/ethercat-rt/src/wire/tests.rs
git commit -m "feat(ethercat): QueryMotorState command + MotorStateResponse frame"
```

---

## Task 7: Endpoint handles `QueryMotorState`

**Files:** `rust/ethercat-rt/src/bin/ethercat-rt.rs`

In the command-dispatch loop (where `Command::QueryRuntimeCaps`/`Identify`/`SetTorque` are handled), add a `Command::QueryMotorState` arm that reads the live drive position+velocity, converts to mm/mm·s⁻¹, and responds.

- [ ] **Step 1: Read the dispatch + CountMap context.** Read the `for cmd in server.poll_commands()` match and how the `CountMap` (`cmap`) is created/held in the DC loop (it's a `cmap.get_or_insert_with(...)` from the first `position_actual` at torque-enable). Confirm the QueryMotorState handler runs in the same scope so it can read `cmap` and `rotation_distance` (from Task 1).
- [ ] **Step 2: Implement the handler.**
```rust
Command::QueryMotorState { correlation_id } => {
    match cmap.as_ref() {
        Some(map) => {
            let pos_counts = unsafe { ffi::ec_rt_get_position_actual() };
            let vel_rpm = unsafe { ffi::ec_rt_get_velocity_actual() };
            let pos_mm = map.actual_mm(pos_counts);
            let vel_mm_s = crate::scale::velocity_mm_s(vel_rpm, rotation_distance);
            server.respond(&wire::motor_state_response_frame(correlation_id, pos_mm, vel_mm_s));
        }
        None => {
            // no origin yet (pre-home / torque disabled): report no motors
            server.respond(&wire::motor_state_empty_frame(correlation_id));
        }
    }
}
```
Adapt `cmap` access to its real type (it may be `Option<CountMap>` or similar). `rotation_distance` is the value parsed in Task 1 Step 5.
- [ ] **Step 3: Build the endpoint.**
Run: `cd rust && cargo build -p ethercat-rt`
Expected: clean. (Also `cargo nextest run -p ethercat-rt` → no regressions; `cargo clippy -p ethercat-rt -- -D warnings` clean.)
- [ ] **Step 4: Commit**
```bash
git add rust/ethercat-rt/src/bin/ethercat-rt.rs
git commit -m "feat(ethercat): endpoint answers QueryMotorState with actual mm pos/vel"
```

---

## Task 8: Bridge routes the live query to EtherCAT endpoints

**Files:** `rust/motion-engine/src/bridge.rs`

Remove the EtherCAT skip in `collect_motor_positions_inner` (bridge.rs ~142) and, for EtherCAT configs, `mcu_call(QueryMotorState)` on the endpoint connection, mapping the reply onto the servo's cartesian slot(s) via `cfg.axes`.

- [ ] **Step 1: Replace the skip with endpoint handling.** In `collect_motor_positions_inner`, the per-config loop currently does `if conn.ethercat_socket.is_some() { continue; }`. Replace the EtherCAT branch so that, for an EtherCAT connection, it obtains `conn.endpoint_conn` (an `Arc<McuSerialConn>` — Plan-1 exploration confirmed it implements the same `mcu_call`), drops the `mcus` lock, and issues the query. Structure (mirror the serial branch's lock-scoping — clone the `Arc` inside the lock, call outside):
```rust
for cfg in &configs {
    // Decide transport under the lock, then call outside it.
    enum Q { Serial(Arc<McuHostIo>), EtherCat(Arc<McuSerialConn>) }
    let q = {
        let map = mcus.lock().unwrap_or_else(|p| p.into_inner());
        let Some(conn) = map.get(&cfg.mcu_id) else { continue };
        if conn.ethercat_socket.is_some() {
            match conn.endpoint_conn.as_ref() {
                Some(ep) => Q::EtherCat(Arc::clone(ep)),
                None => continue,
            }
        } else {
            match conn.host_io.as_ref() {
                Some(io) => Q::Serial(Arc::clone(io)),
                None => continue,
            }
        }
    };
    let (kind, body) = match &q {
        Q::Serial(io) => io.mcu_call(MessageKind::QueryMotorState, Vec::new(), timeout),
        Q::EtherCat(ep) => ep.mcu_call(MessageKind::QueryMotorState, Vec::new(), timeout),
    }.map_err(|e| format!("query mcu {}: {e:?}", cfg.mcu_id))?;
    if kind != MessageKind::MotorStateResponse {
        return Err(format!("query mcu {}: unexpected kind {kind:?}", cfg.mcu_id));
    }
    let resp = MotorStateResponse::decode_from(&mut Cursor::new(&body))
        .map_err(|e| format!("query mcu {}: decode {e:?}", cfg.mcu_id))?;
    match &q {
        Q::Serial(_) => {
            // MCU engine slots are global axis indices — use the reported slot directly.
            for m in resp.motors {
                let slot = m.slot as usize;
                if slot < MAX_AXES {
                    motors[slot] = Some(f64::from(m.pos_q16) / 65536.0);
                    vmotors[slot] = Some(f64::from(m.vel_q16) / 65536.0);
                }
            }
        }
        Q::EtherCat(_) => {
            // Endpoint doesn't know its global slot; map reply entries onto this
            // node's cartesian slot(s) positionally (first motor → first axis).
            for (m, &slot) in resp.motors.iter().zip(cfg.axes.iter()) {
                if slot < MAX_AXES {
                    motors[slot] = Some(f64::from(m.pos_q16) / 65536.0);
                    vmotors[slot] = Some(f64::from(m.vel_q16) / 65536.0);
                }
            }
        }
    }
}
```
Adapt the connection field names (`endpoint_conn`, `McuSerialConn`, `McuHostIo`) and the `McuCall`/`mcu_call` trait import to what actually exists (Plan-1 exploration: `endpoint_conn: Some(Arc<McuSerialConn>)`, and `McuSerialConn` impls `McuCall::mcu_call`). If both `McuHostIo` and `McuSerialConn` impl a common `McuCall` trait, you may unify the call via `dyn McuCall` instead of the enum — use whichever is cleaner given the real types. Keep the lock dropped before the blocking call.
- [ ] **Step 2: Build + regressions.**
Run: `cd rust && cargo build -p motion-engine && cargo nextest run -p motion-engine` → no regressions. `cargo clippy -p motion-engine -- -D warnings` clean.
- [ ] **Step 3: Unit test the EtherCAT slot mapping if feasible.** If `assemble_cartesian` + the mapping logic can be exercised without a live endpoint (e.g. by factoring the "map MotorStateResponse onto motors[] given cfg + transport-kind" into a pure helper), add a separate-file test for it (servo on slot 0 → `motors[0]` set). If it can't be cleanly isolated without a live endpoint, rely on Task 9 e2e and note it.
- [ ] **Step 4: Commit**
```bash
git add rust/motion-engine/src/bridge.rs
git commit -m "feat(bridge): route live position query to EtherCAT endpoints"
```

---

## Task 9: End-to-end + on-drive unit verification

**Files:** none (verification only)

- [ ] **Step 1: CI gates.** `./scripts/ci.sh quick` green; `./scripts/ci.sh py` green (use the `c_helper.so` arm64/amd64 workaround if needed — see Plan 1 Task 11); `cargo fmt --all --check` clean.
- [ ] **Step 2: Sim/topology check (no servo hardware needed).** With the kalico-sim skill, run a **mixed** config (a servo axis + stepper axes if the sim supports an EtherCAT stub; otherwise an all-stepper config to confirm no regression from the `collect_motor_positions_inner` refactor). Confirm `motion_report.live_position` and `GET_POSITION` still report correctly for steppers and that the EtherCAT branch doesn't break startup.
- [ ] **Step 3: On a real A6-EC drive (bench firmware flow), VERIFY THE VELOCITY SCALE — this is the "real deal" gate.**
  - Use the existing servo-capture telemetry (now carrying `velocity_actual`, Task 4): command a steady move at a known commanded velocity `v_cmd` mm/s.
  - From a capture file, compute mm/s three ways and confirm they agree within tolerance:
    1. `velocity_mm_s(raw 606C rpm, rotation_distance)` (this plan's formula),
    2. numerical derivative `Δ position_actual_mm / Δt` (`CountMap::actual_mm` over cycles),
    3. the commanded `v_cmd`.
  - If (1) disagrees with (2)/(3) by a constant factor, the `606Ch` unit/LSB or the electronic-gear (`6091h`) assumption is off → adjust `velocity_mm_s` (e.g. `×6091_ratio`, or 0.1-rpm LSB) and re-verify. Do NOT ship the velocity for tuning until (1)≈(2)≈(3).
  - Likewise sanity-check position: `actual_mm` tracks the commanded trajectory during the move and settles at the endpoint.
- [ ] **Step 4: Document the confirmed scale.** Once verified, ensure `velocity_mm_s` (and any gear factor) reflects the measured truth; if a gear/LSB factor was needed, add a brief note in the spec §2 and a regression test in `scale/tests.rs` locking the confirmed formula.
- [ ] **Step 5: Final gates + PR.** `./scripts/ci.sh quick && ./scripts/ci.sh py`; `cargo fmt --all --check`. Update/extend the PR (or open a new one) for the EtherCAT path.

---

## Self-review notes (author)

- **Spec coverage (§2):** `606Ch` TPDO mapping → Tasks 2–3; `EcTelemetry`/capture/scale → Tasks 3–5; endpoint query/response surfacing → Tasks 6–7; bridge integration into the shared cache/assembly → Task 8; counts→mm + rpm→mm/s with `rotation_distance` → Tasks 1,5; unit verification → Task 9. "First motor per axis" → Task 8 (positional `cfg.axes` mapping). Mixed stepper+servo → Task 8 + Task 9 Step 2.
- **Reuse, not divergence:** `MotorStateResponse`, `assemble_cartesian`, the cache + poll thread, `motion_report`, and `GET_POSITION` are all Plan 1 artifacts used unchanged — the only host edit is the transport branch in `collect_motor_positions_inner`.
- **Type/name consistency:** `velocity_actual` (C `in_t`/`ec_telemetry_t` ↔ Rust `EcTelemetry` ↔ `DriveSample`), `ec_rt_get_velocity_actual`, `CountMap::actual_mm`, `velocity_mm_s(rpm, rotation_distance)`, `Command::QueryMotorState`, `motor_state_response_frame` used consistently across tasks.
- **Hardware-dependent step:** Task 9 Step 3 needs a real A6-EC drive; everything else is unit/CI/sim-verifiable. The velocity scale is treated as unverified until the on-drive cross-check passes, matching the "real deal / tuning" requirement.
- **Known read-the-code points (not placeholders):** exact `cmap`/`rotation_distance` binding in the endpoint (Task 7), the `endpoint_conn`/`McuCall` real types for the bridge branch (Task 8), the `spawn_ethercat_endpoint` arg vector + endpoint arg parser (Task 1), and `capture.rs` exact offsets (Task 4). Each step says what to read and what to write.
