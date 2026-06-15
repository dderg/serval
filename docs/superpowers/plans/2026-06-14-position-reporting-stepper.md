# Stepper Live Position Reporting — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Mainsail's bracketed `[live]` position (and velocity) honest for stepper machines by asking each MCU "where are you now?" instead of reporting the planner's intent.

**Architecture:** A new mcu-protocol request/response pair (`QueryMotorState`/`MotorStateResponse`) returns each MCU's per-slot executor position+velocity (motor-space, mm, Q16). The host bridge converts motor-space → cartesian via the existing `KinematicsModule::inverse`, caches a snapshot updated by a background poll thread (non-blocking, feeds `motion_report`), and exposes a blocking query (feeds `GET_POSITION`). M114 and the commanded surfaces are untouched.

**Tech Stack:** Rust (mcu-protocol, runtime, c-api, motion-engine/pyo3), C (MCU dispatch), Python (klippy `motion_report` + `gcode_move`).

**Spec:** `docs/superpowers/specs/2026-06-14-position-reporting-design.md` (this is Plan 1 of 2; Plan 2 = EtherCAT servo surfacing).

**Conventions used in this plan:**
- Run the Rust suite with `cargo nextest run` from `rust/` (NOT `cargo test`). Scope with `-p <crate>` or `-E 'test(<name>)'`.
- Doc-tests: `cargo test --doc` (nextest skips them).
- Before a PR: `./scripts/ci.sh quick` green; if `klippy/` changed, also `./scripts/ci.sh py`.
- Wire encoding: positions/velocities are **Q16** — `i32` of `value_mm * 65536`. Decode: `q16 as f64 / 65536.0`.
- Engine slot = global axis index (0=X,1=Y,2=Z,3=E,…); on CoreXY slots 0/1 are motors A/B.

---

## File map

| File | Change |
|---|---|
| `rust/mcu-protocol/src/messages.rs` | Add `QueryMotorState`/`MotorStateResponse` `MessageKind` variants + `from_u16`; add `MotorStateResponse` body struct + `Encode`/`Decode` |
| `rust/mcu-protocol/build.rs` | Add the two kinds to the generated C `#define` table |
| `rust/runtime/src/engine.rs` | Add `Engine::motor_state(i) -> Option<(f32,f32)>` |
| `rust/runtime/src/engine_tests.rs` (or the engine's existing test module) | Test `motor_state` |
| `rust/c-api/src/runtime_ffi.rs` | Add `runtime_query_motor_state(...)` FFI |
| `src/mcu_transport_dispatch.c` | Add `handle_query_motor_state` + dispatch case + extern decl |
| `rust/motion-engine/src/position_query.rs` (new) | Pure `assemble_cartesian(...)` motor→cartesian helper + tests |
| `rust/motion-engine/src/bridge.rs` | `collect_motor_positions` (internal), `query_motor_positions` (pyo3 blocking), position cache + background poll thread, `live_motor_positions` (pyo3 non-blocking) |
| `klippy/extras/motion_report.py` | Replace zero stub: `get_status` serves cached live position/velocity |
| `klippy/extras/gcode_move.py` | `GET_POSITION` rungs from blocking query; error → `ERR` line, no raise |

---

## Task 1: Protocol message kinds

**Files:**
- Modify: `rust/mcu-protocol/src/messages.rs` (the `MessageKind` enum + `from_u16`)
- Modify: `rust/mcu-protocol/build.rs` (C `#define` generation table)

- [ ] **Step 1: Read the enum and pick discriminants**

Read `rust/mcu-protocol/src/messages.rs` — find the `MessageKind` enum and confirm `0x0044`/`0x0045` are unused (existing: `QueryRuntimeCaps=0x0040`, `RuntimeCapsResponse=0x0041`, `ClaimHandshake=0x0042`, `ClaimHandshakeReply=0x0043`). If `0x0044`/`0x0045` are taken, use the next free request/response pair and keep the rest of this plan's hex in sync.

- [ ] **Step 2: Write the failing test**

Add to the messages.rs test module (search for `mod tests` / `#[cfg(test)]` in that file; mirror an existing `from_u16` test):

```rust
#[test]
fn motor_state_kinds_roundtrip() {
    assert_eq!(MessageKind::from_u16(0x0044), Some(MessageKind::QueryMotorState));
    assert_eq!(MessageKind::from_u16(0x0045), Some(MessageKind::MotorStateResponse));
    assert_eq!(MessageKind::QueryMotorState.as_u16(), 0x0044);
    assert_eq!(MessageKind::MotorStateResponse.as_u16(), 0x0045);
}
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p mcu-protocol -E 'test(motor_state_kinds_roundtrip)'`
Expected: FAIL — `no variant named QueryMotorState`.

- [ ] **Step 4: Add the variants and from_u16 arms**

In the `MessageKind` enum, after `ClaimHandshakeReply = 0x0043,`:

```rust
    QueryMotorState = 0x0044,
    MotorStateResponse = 0x0045,
```

In `from_u16`, add the matching arms next to the other `0x004x` arms:

```rust
            0x0044 => Self::QueryMotorState,
            0x0045 => Self::MotorStateResponse,
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p mcu-protocol -E 'test(motor_state_kinds_roundtrip)'`
Expected: PASS.

- [ ] **Step 6: Add the kinds to the generated C header**

Read `rust/mcu-protocol/build.rs` and find the table it iterates to emit `#define MCU_MSG_*` lines (around line 60, the `BOOTSTRAP_TAGS` loop, and/or a per-message-kind loop). Add entries so the generator emits:

```c
#define MCU_MSG_QUERY_MOTOR_STATE 0x0044
#define MCU_MSG_MOTOR_STATE_RESPONSE 0x0045
```

Follow the exact tuple/format the existing `MCU_MSG_QUERY_RUNTIME_CAPS` line uses in that table. If the table is keyed off the `MessageKind` enum automatically, no edit is needed here — verify by Step 7.

- [ ] **Step 7: Regenerate and verify the header**

Run: `cd rust && cargo build -p mcu-protocol`
Then: `grep -n "QUERY_MOTOR_STATE\|MOTOR_STATE_RESPONSE" ../src/mcu_protocol_schema.h`
Expected: both `#define`s present with values `0x0044` / `0x0045`.

- [ ] **Step 8: Commit**

```bash
git add rust/mcu-protocol/src/messages.rs rust/mcu-protocol/build.rs src/mcu_protocol_schema.h
git commit -m "feat(protocol): add QueryMotorState/MotorStateResponse message kinds"
```

---

## Task 2: MotorStateResponse body codec

**Files:**
- Modify: `rust/mcu-protocol/src/messages.rs`

Wire format: `count: u8`, then `count` repetitions of `[slot: u8, pos_q16: i32, vel_q16: i32]` (9 bytes each, little-endian). `QueryMotorState` has an empty body, so it needs no struct.

- [ ] **Step 1: Write the failing test**

Add to the messages.rs test module (mirror the `StatusHeartbeat` encode/decode test pattern already in the file):

```rust
#[test]
fn motor_state_response_roundtrip() {
    use crate::codec::{Cursor, Decode, Encode};
    let msg = MotorStateResponse {
        motors: vec![
            MotorSample { slot: 0, pos_q16: 123 * 65536, vel_q16: -45 * 65536 },
            MotorSample { slot: 2, pos_q16: 7, vel_q16: 0 },
        ],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 1 + 2 * 9);
    let mut c = Cursor::new(&buf);
    let got = MotorStateResponse::decode_from(&mut c).unwrap();
    assert_eq!(got, msg);
}

#[test]
fn motor_state_response_empty_roundtrip() {
    use crate::codec::{Cursor, Decode, Encode};
    let msg = MotorStateResponse { motors: vec![] };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf, vec![0u8]);
    let mut c = Cursor::new(&buf);
    assert_eq!(MotorStateResponse::decode_from(&mut c).unwrap(), msg);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p mcu-protocol -E 'test(motor_state_response)'`
Expected: FAIL — `cannot find type MotorStateResponse`.

- [ ] **Step 3: Implement the structs and codec**

Add near the other body structs in messages.rs (use the codec helpers already imported in this file: `put_u8`, `get_u8`, `put_i32`, `get_i32`; and the `DecodeError::ArrayLengthExceedsBuffer { claimed, available }` guard pattern copied from `StatusHeartbeat`):

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorSample {
    pub slot: u8,
    pub pos_q16: i32,
    pub vel_q16: i32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MotorStateResponse {
    pub motors: Vec<MotorSample>,
}

impl Encode for MotorStateResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.motors.len() as u8);
        for m in &self.motors {
            put_u8(out, m.slot);
            put_i32(out, m.pos_q16);
            put_i32(out, m.vel_q16);
        }
    }
}

impl Decode for MotorStateResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let count = get_u8(c)?;
        let need = (count as usize)
            .checked_mul(9)
            .ok_or(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(count),
                available: c.remaining(),
            })?;
        if need > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(count),
                available: c.remaining(),
            });
        }
        let mut motors = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let slot = get_u8(c)?;
            let pos_q16 = get_i32(c)?;
            let vel_q16 = get_i32(c)?;
            motors.push(MotorSample { slot, pos_q16, vel_q16 });
        }
        Ok(Self { motors })
    }
}
```

If `put_i32`/`get_i32` are not already in scope in messages.rs, add them to the `use crate::codec::{...}` line (they exist in `rust/mcu-protocol/src/codec.rs`).

- [ ] **Step 4: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p mcu-protocol -E 'test(motor_state_response)'`
Expected: PASS (both tests).

- [ ] **Step 5: Commit**

```bash
git add rust/mcu-protocol/src/messages.rs
git commit -m "feat(protocol): MotorStateResponse body codec"
```

---

## Task 3: Engine motor_state accessor

**Files:**
- Modify: `rust/runtime/src/engine.rs`
- Test: the engine's existing test module (search `engine.rs` for `#[cfg(test)]` / a sibling `engine_tests.rs`; mirror whatever the crate uses — there are tests that build an `Engine` and call `seed_position`).

Reads the freshest per-tick value: `stepping_axes[i].p_prev` / `.v_prev` (written every ISR tick at engine.rs:452-453; idle→vel 0; seeded at :712-713). Do **not** read `tick_caches` (seed snapshot only).

- [ ] **Step 1: Write the failing test**

Mirror an existing engine test that constructs an `Engine` and calls `seed_position`. Add:

```rust
#[test]
fn motor_state_reads_seeded_position() {
    let mut engine = /* build the same way existing engine tests do */;
    engine.seed_position([12.5, -3.0, 7.0]);
    assert_eq!(engine.motor_state(0), Some((12.5, 0.0)));
    assert_eq!(engine.motor_state(1), Some((-3.0, 0.0)));
    assert_eq!(engine.motor_state(2), Some((7.0, 0.0)));
}
```

(If unconfigured slots return `None` in the existing setup, assert `engine.motor_state(7).is_none()` instead of a value for an unused slot.)

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p runtime -E 'test(motor_state_reads_seeded_position)'`
Expected: FAIL — `no method named motor_state`.

- [ ] **Step 3: Implement the accessor**

In `impl Engine` (near `debug_last_motor`, engine.rs:737):

```rust
pub fn motor_state(&self, i: usize) -> Option<(f32, f32)> {
    self.stepping_axes
        .get(i)
        .and_then(|s| s.as_ref())
        .map(|axis| (axis.p_prev, axis.v_prev))
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p runtime -E 'test(motor_state_reads_seeded_position)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/engine.rs
git commit -m "feat(runtime): Engine::motor_state per-slot live pos/vel accessor"
```

---

## Task 4: Runtime FFI

**Files:**
- Modify: `rust/c-api/src/runtime_ffi.rs`

Model on `runtime_get_heartbeat` (runtime_ffi.rs:1154) for the `RuntimeContext`→`IsrState`→`engine` access pattern. Returns the count of motors written (≥0) or a negative `RUNTIME_ERR_*`.

- [ ] **Step 1: Implement the FFI**

Add (import `MAX_AXES` from the runtime crate's stepping_state if not already in scope):

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_query_motor_state(
    rt: *mut Runtime,
    out_slots: *mut u8,
    out_pos_q16: *mut i32,
    out_vel_q16: *mut i32,
    max: usize,
) -> i32 {
    if rt.is_null()
        || out_slots.is_null()
        || out_pos_q16.is_null()
        || out_vel_q16.is_null()
    {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let engine = &(*isr_ptr).engine;
        let num = (engine.num_axes as usize).min(MAX_AXES);
        let mut n = 0usize;
        for i in 0..num {
            if n >= max {
                break;
            }
            if let Some((p, v)) = engine.motor_state(i) {
                out_slots.add(n).write(i as u8);
                out_pos_q16.add(n).write((p * 65536.0) as i32);
                out_vel_q16.add(n).write((v * 65536.0) as i32);
                n += 1;
            }
        }
        n as i32
    }
}
```

- [ ] **Step 2: Build to verify it compiles**

Run: `cd rust && cargo build -p c-api`
Expected: builds clean. (No unit test here — exercised end-to-end via the sim in Task 11; the engine math is covered by Task 3.)

- [ ] **Step 3: Commit**

```bash
git add rust/c-api/src/runtime_ffi.rs
git commit -m "feat(c-api): runtime_query_motor_state FFI"
```

---

## Task 5: MCU C dispatch handler

**Files:**
- Modify: `src/mcu_transport_dispatch.c`

Model on `handle_query_runtime_caps` (mcu_transport_dispatch.c:272) and `send_push_correction_pieces_response` (multi-byte body packing, :213).

- [ ] **Step 1: Add the extern declaration**

Near the top of `mcu_transport_dispatch.c` (where `extern void *runtime_handle;` is declared, ~line 13), add:

```c
extern int runtime_query_motor_state(
    void *rt, uint8_t *out_slots, int32_t *out_pos_q16,
    int32_t *out_vel_q16, size_t max);
```

- [ ] **Step 2: Add the handler**

Add near `handle_query_runtime_caps`:

```c
static void
handle_query_motor_state(uint32_t correlation_id, const uint8_t *body,
                         uint16_t body_len)
{
    (void)body;
    (void)body_len;
    uint8_t slots[8];
    int32_t pos[8];
    int32_t vel[8];
    int n = 0;
    if (runtime_handle)
        n = runtime_query_motor_state(runtime_handle, slots, pos, vel, 8);
    if (n < 0)
        n = 0;
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 1 + 8 * 9];
    encode_message_header(payload, MCU_MSG_MOTOR_STATE_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)n;
    uint8_t *p = &b[1];
    for (int i = 0; i < n; i++) {
        *p++ = slots[i];
        *p++ = (uint8_t)(pos[i] & 0xFF);
        *p++ = (uint8_t)((pos[i] >> 8) & 0xFF);
        *p++ = (uint8_t)((pos[i] >> 16) & 0xFF);
        *p++ = (uint8_t)((pos[i] >> 24) & 0xFF);
        *p++ = (uint8_t)(vel[i] & 0xFF);
        *p++ = (uint8_t)((vel[i] >> 8) & 0xFF);
        *p++ = (uint8_t)((vel[i] >> 16) & 0xFF);
        *p++ = (uint8_t)((vel[i] >> 24) & 0xFF);
    }
    uint16_t used = (uint16_t)(PER_MESSAGE_HEADER_LEN + 1 + n * 9);
    mcu_transport_send_frame(MCU_CHANNEL_CONTROL, payload, used);
}
```

- [ ] **Step 3: Wire into the dispatch switch**

In the `switch (kind)` block (mcu_transport_dispatch.c:183), add:

```c
case MCU_MSG_QUERY_MOTOR_STATE:
    handle_query_motor_state(correlation_id, body, body_len);
    return;
```

- [ ] **Step 4: Build the MCU firmware to verify it compiles**

Run: `./scripts/ci.sh rust-mcu-h7` (builds the H7 MCU target; confirms the C + Rust link).
Expected: builds clean.

- [ ] **Step 5: Commit**

```bash
git add src/mcu_transport_dispatch.c
git commit -m "feat(mcu): handle_query_motor_state dispatch"
```

---

## Task 6: Pure motor→cartesian assembly helper

**Files:**
- Create: `rust/motion-engine/src/position_query.rs`
- Modify: `rust/motion-engine/src/lib.rs` (add `mod position_query;`)

Isolates the testable math: given per-slot motor-space (pos,vel) and the kinematics tag that owns X, produce a cartesian `{axis_name: (pos_mm, vel_mm_s)}` map. Uses `KinematicsModule::inverse` (kinematics.rs:88).

- [ ] **Step 1: Write the failing test**

Create `rust/motion-engine/src/position_query.rs`:

```rust
use crate::kinematics::KinematicsModule;
use std::collections::HashMap;

const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];

/// `motors[slot]` / `vmotors[slot]` are motor-space mm / mm-s, `None` if that slot
/// was not reported. `kin_tag` is the kinematics tag of the MCU owning the spatial
/// axes. Returns cartesian per-axis (pos, vel); axes with no data are omitted.
pub fn assemble_cartesian(
    motors: &[Option<f64>; 8],
    vmotors: &[Option<f64>; 8],
    kin_tag: u8,
) -> Result<HashMap<String, (f64, f64)>, String> {
    let kin = KinematicsModule::from_tag(kin_tag).map_err(|e| e.to_string())?;
    let spat = |arr: &[Option<f64>; 8]| [arr[0].unwrap_or(0.0), arr[1].unwrap_or(0.0), arr[2].unwrap_or(0.0)];
    let pos_cart = kin.inverse(spat(motors));
    let vel_cart = kin.inverse(spat(vmotors));
    let mut out = HashMap::new();
    for axis in 0..3 {
        if motors[axis].is_some() || vmotors[axis].is_some() {
            out.insert(AXIS_NAMES[axis].to_string(), (pos_cart[axis], vel_cart[axis]));
        }
    }
    if motors[3].is_some() || vmotors[3].is_some() {
        out.insert("e".to_string(), (motors[3].unwrap_or(0.0), vmotors[3].unwrap_or(0.0)));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dispatch::{KINEMATICS_COREXY};

    #[test]
    fn cartesian_identity_passthrough() {
        let mut m = [None; 8];
        let mut v = [None; 8];
        m[0] = Some(10.0); m[1] = Some(20.0); m[2] = Some(5.0); m[3] = Some(2.0);
        v[0] = Some(1.0); v[1] = Some(-1.0); v[2] = Some(0.0); v[3] = Some(3.0);
        // tag 1 = cartesian (see dispatch.rs error text "1=cartesian")
        let out = assemble_cartesian(&m, &v, 1).unwrap();
        assert_eq!(out["x"], (10.0, 1.0));
        assert_eq!(out["y"], (20.0, -1.0));
        assert_eq!(out["z"], (5.0, 0.0));
        assert_eq!(out["e"], (2.0, 3.0));
    }

    #[test]
    fn corexy_inverse_mix() {
        // motor A = x + y, motor B = x - y. For x=10, y=4: A=14, B=6.
        let mut m = [None; 8];
        let v = [None; 8];
        m[0] = Some(14.0); m[1] = Some(6.0); m[2] = Some(0.0);
        let out = assemble_cartesian(&m, &v, KINEMATICS_COREXY).unwrap();
        assert!((out["x"].0 - 10.0).abs() < 1e-9);
        assert!((out["y"].0 - 4.0).abs() < 1e-9);
    }
}
```

- [ ] **Step 2: Register the module**

In `rust/motion-engine/src/lib.rs`, add alongside the other `mod` lines:

```rust
mod position_query;
```

(If `kinematics`, `dispatch` are private, add `pub(crate)` use as needed so `position_query` can reach `KinematicsModule` and `KINEMATICS_COREXY`; they're already used cross-module per dispatch.rs.)

- [ ] **Step 3: Run test to verify it fails, then passes**

Run: `cd rust && cargo nextest run -p motion-engine -E 'test(position_query)'`
Expected first run: FAIL (module/test new) → after Steps 1–2 compile, PASS both tests. Fix the `KinematicsModule::inverse` / `from_tag` call sites if signatures differ (see kinematics.rs:46,88).

- [ ] **Step 4: Commit**

```bash
git add rust/motion-engine/src/position_query.rs rust/motion-engine/src/lib.rs
git commit -m "feat(bridge): pure motor->cartesian position assembly helper"
```

---

## Task 7: Bridge blocking query (`query_motor_positions`)

**Files:**
- Modify: `rust/motion-engine/src/bridge.rs`

Model on `motion_state_at_clock` (bridge.rs:3672) for iterating `mcu_axis_configs` and returning a `HashMap<String,(f64,f64)>` to Python, and on `set_position` (bridge.rs:3055) for locking `mcus`, identifying serial vs EtherCAT (`conn.ethercat_socket.is_some()`), and reaching `conn.host_io`. Use `io.mcu_call(MessageKind::QueryMotorState, Vec::new(), timeout)` (host_io/mod.rs:725) and decode with `MotorStateResponse::decode_from`.

- [ ] **Step 1: Add an internal collector (no Python)**

Add a private method on `PyMotionEngine` (NOT `#[pyo3]`) so the background thread (Task 8) can reuse it:

```rust
fn collect_motor_positions(
    &self,
    timeout: std::time::Duration,
) -> Result<std::collections::HashMap<String, (f64, f64)>, String> {
    use mcu_protocol::codec::{Cursor, Decode};
    use mcu_protocol::messages::MotorStateResponse;
    use mcu_protocol::MessageKind;

    // Snapshot config + serial host_io handles under the locks, then release.
    let configs = self
        .mcu_axis_configs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();
    if configs.is_empty() {
        return Err("query_motor_positions: no axes configured".into());
    }
    // kinematics tag of the MCU owning AXIS_X (0); default cartesian (tag 1).
    let kin_tag = configs
        .iter()
        .find(|c| c.axes.contains(&0usize))
        .map(|c| c.kinematics)
        .unwrap_or(1);

    let mut motors: [Option<f64>; 8] = [None; 8];
    let mut vmotors: [Option<f64>; 8] = [None; 8];

    for cfg in &configs {
        // EtherCAT axes are handled in Plan 2; skip non-serial here.
        let call = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let Some(conn) = mcus.get(&cfg.mcu_id) else { continue };
            if conn.ethercat_socket.is_some() {
                continue;
            }
            let Some(io) = conn.host_io.as_ref() else { continue };
            io.mcu_call(MessageKind::QueryMotorState, Vec::new(), timeout)
        };
        let (kind, body) =
            call.map_err(|e| format!("query mcu {}: {e:?}", cfg.mcu_id))?;
        if kind != MessageKind::MotorStateResponse {
            return Err(format!("query mcu {}: unexpected kind {kind:?}", cfg.mcu_id));
        }
        let mut c = Cursor::new(&body);
        let resp = MotorStateResponse::decode_from(&mut c)
            .map_err(|e| format!("query mcu {}: decode {e:?}", cfg.mcu_id))?;
        for m in resp.motors {
            let slot = m.slot as usize;
            if slot < 8 {
                motors[slot] = Some(f64::from(m.pos_q16) / 65536.0);
                vmotors[slot] = Some(f64::from(m.vel_q16) / 65536.0);
            }
        }
    }
    crate::position_query::assemble_cartesian(&motors, &vmotors, kin_tag)
}
```

NOTE on locking: `mcu_call` blocks on a round-trip; the snippet above scopes the `mcus` lock to just obtaining `io` then calls outside — verify `conn.host_io` (`McuHostIo`) can be cloned/Arc'd so the call happens after the lock drops. If `host_io` is not cloneable, restructure to collect `(mcu_id, io_handle)` clones first. Read the `Mcu`/connection struct in bridge.rs (search `host_io`) and adapt.

- [ ] **Step 2: Add the pyo3 wrapper**

In the `#[pymethods] impl PyMotionEngine` block, add:

```rust
#[pyo3(signature = (timeout_s=0.25))]
fn query_motor_positions(
    &self,
    py: Python<'_>,
    timeout_s: f64,
) -> PyResult<std::collections::HashMap<String, (f64, f64)>> {
    let timeout = std::time::Duration::from_secs_f64(timeout_s.max(0.0));
    py.detach(|| self.collect_motor_positions(timeout))
        .map_err(PyRuntimeError::new_err)
}
```

- [ ] **Step 3: Build and run bridge tests**

Run: `cd rust && cargo build -p motion-engine && cargo nextest run -p motion-engine`
Expected: builds clean; existing tests pass. (The collector itself needs an MCU; it's exercised in Task 11. The math is covered by Task 6.)

- [ ] **Step 4: Commit**

```bash
git add rust/motion-engine/src/bridge.rs
git commit -m "feat(bridge): query_motor_positions blocking live position query"
```

---

## Task 8: Position cache + background poll thread + `live_motor_positions`

**Files:**
- Modify: `rust/motion-engine/src/bridge.rs`

Adds a cached snapshot updated on a fixed cadence by a dedicated thread; `get_status` reads it without blocking. On poll failure: log + keep last (fail-loud exception per spec §6).

- [ ] **Step 1: Add the cache field**

In the `PyMotionEngine` struct, add (alongside `motion_history`):

```rust
// (axis_map, host_monotonic_secs_of_last_successful_poll)
live_position_cache: Arc<Mutex<(std::collections::HashMap<String, (f64, f64)>, f64)>>,
position_poll_thread: Mutex<Option<JoinHandle<()>>>,
position_poll_stop: Arc<std::sync::atomic::AtomicBool>,
```

Initialize in the constructor (near `motion_history: Arc::new(...)`, bridge.rs:764):

```rust
live_position_cache: Arc::new(Mutex::new((std::collections::HashMap::new(), 0.0))),
position_poll_thread: Mutex::new(None),
position_poll_stop: Arc::new(std::sync::atomic::AtomicBool::new(false)),
```

- [ ] **Step 2: Spawn the poll thread**

Right after the pump thread is spawned (bridge.rs ~2603), add a poll thread. It needs `self`-shared state via `Arc` clones — clone the `Arc`s it touches (`mcu_axis_configs`, `mcus`, `live_position_cache`, `position_poll_stop`). Since `collect_motor_positions` is a method on `&self`, extract its body into a free function `collect_motor_positions_inner(configs, mcus, timeout)` that the thread and the method both call (refactor Task 7's body into that free fn taking the two `Arc<Mutex<...>>` plus timeout). Then:

```rust
{
    let configs = Arc::clone(&self.mcu_axis_configs);
    let mcus = Arc::clone(&self.mcus);
    let cache = Arc::clone(&self.live_position_cache);
    let stop = Arc::clone(&self.position_poll_stop);
    let handle = std::thread::Builder::new()
        .name("live-position-poll".into())
        .spawn(move || {
            use std::sync::atomic::Ordering;
            let period = std::time::Duration::from_millis(200); // ~5 Hz
            let timeout = std::time::Duration::from_millis(250);
            while !stop.load(Ordering::Relaxed) {
                std::thread::sleep(period);
                match crate::bridge::collect_motor_positions_inner(&configs, &mcus, timeout) {
                    Ok(map) => {
                        let now = std::time::Instant::now().elapsed().as_secs_f64(); // replace with monotonic helper used elsewhere
                        let mut c = cache.lock().unwrap_or_else(|p| p.into_inner());
                        *c = (map, now);
                    }
                    Err(e) => {
                        // fail-loud exception: log + keep last (spec §6)
                        tracing::warn!(error = %e, "live-position poll failed; serving stale cache");
                    }
                }
            }
        })
        .expect("spawn live-position-poll thread");
    *self.position_poll_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
}
```

Replace the `now` line with whatever monotonic host-clock helper the bridge already uses for timestamps (search bridge.rs for `host_now` / `Instant` usage; reuse it so the staleness stamp is consistent). If a `no axes configured` error is returned before `init_planner`, treat it as benign (skip logging) — guard with `if !configs.lock()....is_empty()`.

- [ ] **Step 3: Stop the thread on shutdown**

Find the bridge's shutdown/drop path (search for where `pump_thread` is joined). Set `self.position_poll_stop.store(true, Ordering::Relaxed)` and `join()` the `position_poll_thread` there, mirroring the pump thread teardown.

- [ ] **Step 4: Add the non-blocking pyo3 reader**

```rust
fn live_motor_positions(&self) -> std::collections::HashMap<String, (f64, f64)> {
    self.live_position_cache
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .0
        .clone()
}
```

- [ ] **Step 5: Build and test**

Run: `cd rust && cargo build -p motion-engine && cargo nextest run -p motion-engine`
Expected: builds clean; tests pass.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-engine/src/bridge.rs
git commit -m "feat(bridge): background live-position cache + live_motor_positions"
```

---

## Task 9: motion_report wiring

**Files:**
- Modify: `klippy/extras/motion_report.py`
- Test: `klippy/extras/test_motion_report.py` (create; mirror an existing klippy extras test — check `klippy/extras/` for the test layout/naming this fork uses; per CLAUDE.md unit tests live in a separate file from the code)

`PrinterMotionReport.get_status` must serve the bridge cache. `live_velocity` is the cartesian speed magnitude; `live_extruder_velocity` is the E velocity.

- [ ] **Step 1: Write the failing test**

Create `klippy/extras/test_motion_report.py` with a fake bridge exposing `live_motor_positions()`:

```python
import math

class _FakeBridge:
    def __init__(self, data):
        self._data = data
    def live_motor_positions(self):
        return dict(self._data)

def _status(monkeypatched_report):
    return monkeypatched_report.get_status(0.0)

def test_live_position_from_bridge(make_motion_report):
    # make_motion_report: a fixture/helper that builds PrinterMotionReport with
    # a fake printer whose lookup_object("motion_engine") returns _FakeBridge.
    rep = make_motion_report(_FakeBridge({
        "x": (10.0, 1.0), "y": (20.0, 0.0), "z": (5.0, 0.0), "e": (2.0, 3.0),
    }))
    st = rep.get_status(0.0)
    assert st["live_position"][0] == 10.0
    assert st["live_position"][1] == 20.0
    assert st["live_position"][2] == 5.0
    assert math.isclose(st["live_velocity"], 1.0)         # ||(1,0,0)||
    assert math.isclose(st["live_extruder_velocity"], 3.0)
```

If this fork's klippy tests don't use pytest fixtures, construct `PrinterMotionReport` directly with a minimal fake `config`/`printer` mirroring an existing `klippy/extras/test_*.py`.

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest klippy/extras/test_motion_report.py -v` (or this fork's klippy test runner — see `./scripts/ci.sh py`).
Expected: FAIL — `live_position` is `(0,0,0,0)`.

- [ ] **Step 3: Implement get_status**

Replace `PrinterMotionReport.get_status` (motion_report.py:49) and cache the bridge handle in `_connect`:

```python
    def _connect(self):
        self.last_status["steppers"] = list(sorted(self.steppers.keys()))
        self.bridge = self.printer.lookup_object("motion_engine", None)

    def get_status(self, eventtime):
        gcode = self.printer.lookup_object("gcode")
        bridge = getattr(self, "bridge", None)
        if bridge is None:
            return self.last_status
        axes = bridge.live_motor_positions()
        x, xv = axes.get("x", (0.0, 0.0))
        y, yv = axes.get("y", (0.0, 0.0))
        z, zv = axes.get("z", (0.0, 0.0))
        e, ev = axes.get("e", (0.0, 0.0))
        live_velocity = (xv * xv + yv * yv + zv * zv) ** 0.5
        return {
            "live_position": gcode.Coord(x, y, z, e),
            "live_velocity": live_velocity,
            "live_extruder_velocity": ev,
            "steppers": self.last_status["steppers"],
            "trapq": self.last_status["trapq"],
        }
```

(Confirm the `motion_engine` object name with `grep -rn 'lookup_object("motion_engine"' klippy/`; use the exact registered name.)

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest klippy/extras/test_motion_report.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/motion_report.py klippy/extras/test_motion_report.py
git commit -m "feat(motion_report): serve live position/velocity from bridge cache"
```

---

## Task 10: GET_POSITION wiring

**Files:**
- Modify: `klippy/extras/gcode_move.py` (`cmd_GET_POSITION`, :305)
- Test: `klippy/extras/test_gcode_move_get_position.py` (create)

Fill the `mcu`/`stepper` rungs with per-slot motor-space values and the `kinematic` rung with cartesian, from the **blocking** `query_motor_positions`. On failure, print `ERR` in the response — do not raise (spec §6).

- [ ] **Step 1: Write the failing test**

Create `klippy/extras/test_gcode_move_get_position.py`. Build the `GCodeMove` (or its `cmd_GET_POSITION`) with a fake toolhead + a fake bridge whose `query_motor_positions()` returns `{"x":(10.0,0.0),...}`, and a fake that raises, asserting:
- success: response text contains `kinematic: X:10.000000`;
- failure: response text contains `ERR` and the command does **not** raise.

Mirror the structure of an existing `klippy/extras/test_*.py` that drives a gcode command and captures `gcmd.respond_info`.

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest klippy/extras/test_gcode_move_get_position.py -v`
Expected: FAIL (current command calls `s.get_mcu_position()` → AttributeError).

- [ ] **Step 3: Implement**

Rewrite the measured rungs of `cmd_GET_POSITION` (keep the `toolhead`/`gcode`/`base`/`homing` rungs as-is):

```python
    def cmd_GET_POSITION(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead", None)
        if toolhead is None:
            raise gcmd.error("Printer not ready")
        bridge = self.printer.lookup_object("motion_engine", None)
        try:
            axes = bridge.query_motor_positions() if bridge is not None else {}
            measured = " ".join(
                "%s:%.6f" % (a.upper(), axes[a][0])
                for a in ("x", "y", "z", "e")
                if a in axes
            )
            kin_pos = measured if measured else "ERR"
        except Exception as e:
            kin_pos = "ERR (%s)" % (e,)
        # toolhead / gcode / base / homing rungs unchanged below ...
```

Wire `kin_pos` into the existing `gcmd.respond_info(...)` template, and drop the old `mcu:`/`stepper:` lines that called `get_mcu_position`/`get_commanded_position` (or set them to the same `measured`/`ERR` string). Read the existing `respond_info` block (gcode_move.py:333) and adapt the format string so it has no dangling `%s`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest klippy/extras/test_gcode_move_get_position.py -v`
Expected: PASS (both success and failure cases).

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/gcode_move.py klippy/extras/test_gcode_move_get_position.py
git commit -m "feat(gcode_move): GET_POSITION reports measured position, ERR on query failure"
```

---

## Task 11: End-to-end verification in the simulator

**Files:** none (verification only)

Use the `kalico-sim` skill to run firmware + host against G-code and observe live position.

- [ ] **Step 1: Build everything**

Run: `./scripts/ci.sh quick` and `./scripts/ci.sh py`
Expected: green.

- [ ] **Step 2: Run a move in sim and observe live position**

Per the `kalico-sim` skill, start the simulator, home, and stream a slow `G1` move. During the move:
- query `motion_report` status (the bracketed value path) and confirm `live_position` is **non-zero and changing**, trending toward the target;
- run `GET_POSITION` and confirm the `kinematic` rung is populated (non-zero), and `toolhead`/`gcode` rungs match commanded.

- [ ] **Step 3: Confirm settle + idle**

After the move completes, confirm `live_position` settles to the endpoint and `live_velocity` returns to ~0.

- [ ] **Step 4: Confirm failure handling**

Temporarily point the blocking query at a disconnected MCU (or stop the sim MCU) and run `GET_POSITION`; confirm it prints `ERR` and does not crash klippy. Confirm `motion_report` keeps serving the last value (no print abort).

- [ ] **Step 5: Final commit / PR prep**

```bash
./scripts/ci.sh quick && ./scripts/ci.sh py
cargo fmt --all --check    # from rust/  (last step before PR per project rule)
```
Open/update the PR once green.

---

## Self-review notes (author)

- **Spec coverage:** §1 MCU primitive → Tasks 1–5; §3 bridge query+cache → Tasks 6–8; §4 motion_report+GET_POSITION → Tasks 9–10; §5 velocity via `inverse` → Tasks 6,9; §6 cadence + fail handling → Task 8 (cache log+stale) and Task 10 (`ERR` no-raise); testing → Task 11. EtherCAT (§2) is Plan 2 — intentionally excluded.
- **Type consistency:** `MotorStateResponse{motors: Vec<MotorSample{slot,pos_q16,vel_q16}>}` is used identically in Tasks 2, 7; `assemble_cartesian(&[Option<f64>;8], &[Option<f64>;8], u8)` consistent across Tasks 6–8; `query_motor_positions`/`live_motor_positions` names consistent across Tasks 7–10.
- **Known adaptation points (read-the-code, not placeholders):** exact engine-test construction (Task 3), `McuHostIo` cloneability for lock scoping (Task 7), the monotonic host-clock helper for the staleness stamp (Task 8), and this fork's klippy test harness conventions (Tasks 9–10). Each step says what to read and what to write.
