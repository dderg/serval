# EtherCAT RT Endpoint (`kalico-ethercat-rt`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone host binary that brings the A6-EC servo up in CSP/DC, accepts kalico-native `LoadCurveCubic`/`PushSegment` frames over a unix socket, evaluates them with `rust/runtime`'s host evaluator at the DC rate, and streams the resulting position (scaled to encoder counts) to the drive — provably moving the motor from a canned test client.

**Architecture:** Two crates plus a C shim. (1) `bench/libecrt.{c,h}` — the working `ec_spin.c` SOEM/CSP/DC bring-up refactored into a clean `extern "C"` surface; C owns the fussy bring-up and the cyclic PDO+DC exchange. (2) `rust/kalico-ethercat-rt` — a binary that owns a unix-socket server speaking kalico-native (decode commands, encode responses), a `CurvePool`-backed curve store, a single-channel Bézier piece-walker reusing `eval_position_velocity`, and the steady-state DC loop that calls the C shim each cycle. A second binary `ec-test-client` sends a gentle ease-in/ease-out move to prove the chain end-to-end. Time domain is `CLOCK_MONOTONIC` nanoseconds shared by client and endpoint (`cycles_per_second = 1e9`).

**Tech Stack:** Rust (workspace member, `runtime` host feature `f32`, `kalico-protocol`, `kalico-native-transport`), C (SOEM v1.4.0, linked via `build.rs`), Linux real-time (`SCHED_FIFO`, `mlockall`, pinned core). Target host: Raspberry Pi 3B at `dderg@ethercat.local`.

---

## ⚠️ Before any EtherCAT-communicating code

Read the relevant chapters of the drive manual **in full** first:
`~/Downloads/A6-EC_series_servo_drive_manual (1).pdf` (printed page N ≈ PDF page N+2). Key: Ch. 8 Communication (printed 156–168, incl. 8.2.3 DC p.162, 8.3 Process Data p.163), Ch. 10 Troubleshooting (faults, printed 171+). Do not guess object indices, value semantics, sync-mode support, or timing — the manual is authoritative. This applies to Tasks 7, 9, 11.

## File structure

| File | Responsibility |
| --- | --- |
| `bench/libecrt.h` | C shim public `extern "C"` surface + version constants |
| `bench/libecrt.c` | SOEM bring-up + cyclic exchange (lifted from `ec_spin.c`) |
| `bench/Makefile` | builds `libecrt.a` on the Pi (links SOEM static lib) |
| `rust/kalico-ethercat-rt/Cargo.toml` | crate manifest, two bins, deps |
| `rust/kalico-ethercat-rt/build.rs` | links `libecrt.a` + SOEM + pthread/rt/m |
| `rust/kalico-ethercat-rt/src/ffi.rs` | `extern "C"` declarations for `libecrt` |
| `rust/kalico-ethercat-rt/src/scale.rs` | mm→counts scaling + origin mapping (pure) |
| `rust/kalico-ethercat-rt/src/wire.rs` | `pieces_bytes`→`Vec<WirePiece>`, control-msg decode, response encode |
| `rust/kalico-ethercat-rt/src/curves.rs` | `CurveStore` over `CurvePool`; segment + piece-walker |
| `rust/kalico-ethercat-rt/src/server.rs` | unix-socket accept + frame decode/dispatch + response write |
| `rust/kalico-ethercat-rt/src/lib.rs` | re-exports the above for integration tests |
| `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs` | endpoint main: bring-up + DC loop |
| `rust/kalico-ethercat-rt/src/bin/ec-test-client.rs` | canned gentle-move sender |

**Single-channel, single-slave for M1.** The endpoint drives one servo (EtherCAT slave 1) from one `PushSegment` handle slot (default `handle_x`). It is axis-agnostic: it never reasons about "X"; it drives "the configured handle slot." Other axes do not exist in this process.

---

## Task 0: Scaffold the crate and wire it into the workspace

**Files:**
- Create: `rust/kalico-ethercat-rt/Cargo.toml`
- Create: `rust/kalico-ethercat-rt/src/lib.rs`
- Create: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`
- Modify: `rust/Cargo.toml` (workspace members)

- [ ] **Step 1: Create the crate manifest**

`rust/kalico-ethercat-rt/Cargo.toml`:
```toml
[package]
name = "kalico-ethercat-rt"
version = "0.1.0"
edition = "2021"

[lib]
name = "kalico_ethercat_rt"
crate-type = ["rlib"]

[[bin]]
name = "kalico-ethercat-rt"
path = "src/bin/kalico-ethercat-rt.rs"

[[bin]]
name = "ec-test-client"
path = "src/bin/ec-test-client.rs"

[dependencies]
runtime = { path = "../runtime" }                 # default features => "host" (f32, std)
kalico-protocol = { path = "../kalico-protocol" }
kalico-native-transport = { path = "../kalico-native-transport" }
```

- [ ] **Step 2: Create a stub lib and the two bin stubs so the crate builds**

`rust/kalico-ethercat-rt/src/lib.rs`:
```rust
//! Host-side EtherCAT motion-node endpoint: decodes the kalico-native piece
//! stream and streams CSP position to an A6-EC servo over EtherCAT/DC.
pub mod scale;
pub mod wire;
pub mod curves;
```

`rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`:
```rust
fn main() {
    eprintln!("kalico-ethercat-rt: not yet implemented");
}
```

`rust/kalico-ethercat-rt/src/bin/ec-test-client.rs`:
```rust
fn main() {
    eprintln!("ec-test-client: not yet implemented");
}
```

Create empty module files so `lib.rs` compiles (they are filled in later tasks):
```bash
mkdir -p rust/kalico-ethercat-rt/src/bin
printf '' > rust/kalico-ethercat-rt/src/scale.rs
printf '' > rust/kalico-ethercat-rt/src/wire.rs
printf '' > rust/kalico-ethercat-rt/src/curves.rs
```

- [ ] **Step 3: Add to the workspace members**

In `rust/Cargo.toml`, add `"kalico-ethercat-rt"` to the `[workspace] members` array (after `"kalico-protocol"`).

- [ ] **Step 4: Build to verify the skeleton compiles**

Run: `cd rust && cargo build -p kalico-ethercat-rt`
Expected: PASS (warnings about unused modules are fine).

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-ethercat-rt rust/Cargo.toml
git commit -m "ethercat-rt: scaffold kalico-ethercat-rt crate + workspace member"
```

---

## Task 1: mm→counts scaling and origin mapping (pure)

The evaluator emits millimetres. The drive wants signed encoder counts. We map relative to the position captured when the first segment arms, so the first commanded target equals the current rotor position (no startup jump).

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/scale.rs`

- [ ] **Step 1: Write the failing test**

In `rust/kalico-ethercat-rt/src/scale.rs`:
```rust
//! mm -> encoder-count mapping, relative to a captured origin.

/// Fixed mm->counts gain and the origin captured at first arm.
#[derive(Debug, Clone, Copy)]
pub struct CountMap {
    pub counts_per_mm: f64,
    pub origin_counts: i32,
    pub origin_mm: f64,
}

impl CountMap {
    /// Capture the origin: `actual_counts` is the rotor position now,
    /// `pos_mm` is the trajectory position at the same instant.
    pub fn new(counts_per_mm: f64, actual_counts: i32, pos_mm: f64) -> Self {
        Self { counts_per_mm, origin_counts: actual_counts, origin_mm: pos_mm }
    }

    /// Map a trajectory position (mm) to an absolute target count.
    pub fn target_counts(&self, pos_mm: f64) -> i32 {
        let delta = (pos_mm - self.origin_mm) * self.counts_per_mm;
        self.origin_counts + delta.round() as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_maps_to_itself() {
        let m = CountMap::new(3276.8, 14578, 5.0);
        assert_eq!(m.target_counts(5.0), 14578);
    }

    #[test]
    fn positive_delta_rounds_and_adds() {
        let m = CountMap::new(1000.0, 0, 0.0);
        assert_eq!(m.target_counts(1.0004), 1000);  // 1000.4 -> 1000
        assert_eq!(m.target_counts(1.0006), 1001);  // 1000.6 -> 1001
    }

    #[test]
    fn negative_delta() {
        let m = CountMap::new(1000.0, 5000, 10.0);
        assert_eq!(m.target_counts(9.0), 4000);
    }
}
```

- [ ] **Step 2: Run to verify it fails (then passes)**

Run: `cd rust && cargo test -p kalico-ethercat-rt scale::`
Expected: PASS (the implementation is written together with the test above; this task is small enough that test+impl land together). If `target_counts` is missing, fix and re-run.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/src/scale.rs
git commit -m "ethercat-rt: mm->counts scaling relative to captured origin"
```

---

## Task 2: Decode `LoadCurveCubic.pieces_bytes` into `WirePiece`s (pure)

`LoadCurveCubic` carries `pieces_bytes` = `piece_count * 20` bytes; each 20-byte piece is `bp0,bp1,bp2,bp3,duration` as little-endian `f32` bit patterns. `runtime::cubic_curve::WirePiece` wants those same five `u32` bit patterns.

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/wire.rs`

- [ ] **Step 1: Write the failing test**

In `rust/kalico-ethercat-rt/src/wire.rs`:
```rust
//! Wire helpers: piece-bytes -> WirePiece, control-message decode, responses.

use runtime::cubic_curve::WirePiece;

#[derive(Debug, PartialEq, Eq)]
pub enum PiecesError {
    BadLength,
}

/// Split a `LoadCurveCubic.pieces_bytes` blob into `WirePiece`s.
pub fn wire_pieces_from_bytes(piece_count: u8, bytes: &[u8]) -> Result<Vec<WirePiece>, PiecesError> {
    let n = piece_count as usize;
    if bytes.len() != n * 20 {
        return Err(PiecesError::BadLength);
    }
    let mut out = Vec::with_capacity(n);
    for chunk in bytes.chunks_exact(20) {
        let rd = |i: usize| u32::from_le_bytes([chunk[i], chunk[i + 1], chunk[i + 2], chunk[i + 3]]);
        out.push(WirePiece {
            bp0_bits: rd(0),
            bp1_bits: rd(4),
            bp2_bits: rd(8),
            bp3_bits: rd(12),
            duration_bits: rd(16),
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn piece_bytes(bp: [f32; 4], dur: f32) -> Vec<u8> {
        let mut v = Vec::new();
        for x in bp { v.extend_from_slice(&x.to_le_bits().to_le_bytes()); }
        v.extend_from_slice(&dur.to_bits().to_le_bytes());
        v
    }

    #[test]
    fn decodes_one_piece() {
        let bytes = {
            let mut v = Vec::new();
            for x in [0.0f32, 0.0, 10.0, 10.0] { v.extend_from_slice(&x.to_bits().to_le_bytes()); }
            v.extend_from_slice(&0.5f32.to_bits().to_le_bytes());
            v
        };
        let pieces = wire_pieces_from_bytes(1, &bytes).unwrap();
        assert_eq!(pieces.len(), 1);
        assert_eq!(f32::from_bits(pieces[0].bp2_bits), 10.0);
        assert_eq!(f32::from_bits(pieces[0].duration_bits), 0.5);
    }

    #[test]
    fn rejects_wrong_length() {
        assert_eq!(wire_pieces_from_bytes(2, &[0u8; 20]), Err(PiecesError::BadLength));
    }
}
```
(Delete the unused `piece_bytes` helper if the linter complains — the inline builder in `decodes_one_piece` is what's used.)

- [ ] **Step 2: Run the tests**

Run: `cd rust && cargo test -p kalico-ethercat-rt wire::tests::decodes_one_piece wire::tests::rejects_wrong_length`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/src/wire.rs
git commit -m "ethercat-rt: decode LoadCurveCubic piece bytes into WirePiece"
```

---

## Task 3: Decode incoming control messages from a framed payload

Given a `Frame::Kalico` payload (per-message header + body), classify it into the commands the endpoint handles. Reuses `kalico-native-transport::decode_message_header`, `kalico_protocol::MessageKind`, and each message's `Decode`.

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/wire.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/kalico-ethercat-rt/src/wire.rs`:
```rust
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{LoadCurveCubic, MessageKind, PushSegment, ResetCurvePool};
use kalico_native_transport::wire_helpers::{decode_message_header, encode_message_header, MESSAGE_VERSION_DEFAULT};

/// A decoded control-channel command plus the correlation id to answer with.
#[derive(Debug)]
pub enum Command {
    Identify { correlation_id: u32, proto_version: u8 },
    LoadCurve { correlation_id: u32, msg: LoadCurveCubic },
    PushSegment { correlation_id: u32, msg: PushSegment },
    ResetPool { correlation_id: u32 },
    Unknown { correlation_id: u32, kind_raw: u16 },
}

#[derive(Debug, PartialEq, Eq)]
pub enum DecodeCmdError {
    BadHeader,
    BadBody,
}

/// `payload` is a `Frame::Kalico` payload: 7-byte message header + body.
pub fn decode_command(payload: &[u8]) -> Result<Command, DecodeCmdError> {
    let (hdr, body) = decode_message_header(payload).ok_or(DecodeCmdError::BadHeader)?;
    let cid = hdr.correlation_id;
    match MessageKind::from_u16(hdr.kind_raw) {
        Some(MessageKind::Identify) => {
            let proto_version = body.first().copied().unwrap_or(0);
            Ok(Command::Identify { correlation_id: cid, proto_version })
        }
        Some(MessageKind::LoadCurveCubic) => {
            let msg = LoadCurveCubic::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::LoadCurve { correlation_id: cid, msg })
        }
        Some(MessageKind::PushSegment) => {
            let msg = PushSegment::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::PushSegment { correlation_id: cid, msg })
        }
        Some(MessageKind::ResetCurvePool) => Ok(Command::ResetPool { correlation_id: cid }),
        _ => Ok(Command::Unknown { correlation_id: cid, kind_raw: hdr.kind_raw }),
    }
}

/// Build a control-channel command payload (header + body) for a `Decode`/`Encode`
/// message. Test/client helper.
pub fn frame_payload(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(7 + body.len());
    out.extend_from_slice(&encode_message_header(kind, MESSAGE_VERSION_DEFAULT, correlation_id));
    out.extend_from_slice(body);
    out
}

#[cfg(test)]
mod decode_cmd_tests {
    use super::*;

    #[test]
    fn round_trips_push_segment() {
        let seg = PushSegment {
            id: 7, handle_x: 0x0001_0000, handle_y: 0, handle_z: 0, handle_e: 0,
            t_start: 1_000, t_end: 2_000, kinematics: 0, e_mode: 0, extrusion_ratio: 0.0,
        };
        let payload = frame_payload(MessageKind::PushSegment, 42, &seg.encoded_to_vec());
        match decode_command(&payload).unwrap() {
            Command::PushSegment { correlation_id, msg } => {
                assert_eq!(correlation_id, 42);
                assert_eq!(msg.id, 7);
                assert_eq!(msg.handle_x, 0x0001_0000);
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn round_trips_load_curve() {
        let msg = LoadCurveCubic { slot_idx: 3, axis_idx: 0, piece_count: 0, pieces_bytes: vec![] };
        let payload = frame_payload(MessageKind::LoadCurveCubic, 9, &msg.encoded_to_vec());
        match decode_command(&payload).unwrap() {
            Command::LoadCurve { correlation_id, msg } => {
                assert_eq!(correlation_id, 9);
                assert_eq!(msg.slot_idx, 3);
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }
}
```

- [ ] **Step 2: Run the tests**

Run: `cd rust && cargo test -p kalico-ethercat-rt decode_cmd_tests`
Expected: PASS. (If `decode_message_header` / `encode_message_header` paths differ, fix the `use` to the real module path reported by the compiler — they live in `kalico-native-transport`'s `wire_helpers`.)

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/src/wire.rs
git commit -m "ethercat-rt: decode kalico-native control commands"
```

---

## Task 4: Encode responses (Load/Push/Reset/Identify) as full frames

The endpoint must answer each command on `CHANNEL_CONTROL` with the matching response kind and the *same* correlation id, so the eventual `KalicoNativeTransport` client (Plan 2 / Task 10) matches request to response.

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/wire.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/kalico-ethercat-rt/src/wire.rs`:
```rust
use kalico_protocol::bootstrap::{IdentifyResponse, IDENTIFY_RESPONSE_BODY_LEN};
use kalico_protocol::messages::{LoadCurveResponse, PushSegmentResponse, ResetCurvePoolResponse};
use kalico_native_transport::frame::{decode_frame, encode_frame, CHANNEL_CONTROL};

/// Wrap a header+body payload into a full Layer-1 frame on the control channel.
pub fn control_frame(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    encode_frame(CHANNEL_CONTROL, &frame_payload(kind, correlation_id, body))
}

pub fn load_curve_response_frame(cid: u32, result: i32, handle_packed: u32) -> Vec<u8> {
    let body = LoadCurveResponse { result, curve_handle_packed: handle_packed }.encoded_to_vec();
    control_frame(MessageKind::LoadCurveResponse, cid, &body)
}

pub fn push_segment_response_frame(cid: u32, result: i32, accepted_id: u32) -> Vec<u8> {
    let body = PushSegmentResponse { result, accepted_segment_id: accepted_id, credit_epoch: 0 }.encoded_to_vec();
    control_frame(MessageKind::PushSegmentResponse, cid, &body)
}

pub fn reset_pool_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = ResetCurvePoolResponse { result }.encoded_to_vec();
    control_frame(MessageKind::ResetCurvePoolResponse, cid, &body)
}

/// Canned identify response advertising one motion channel, no special caps.
pub fn identify_response_frame(cid: u32, proto_version: u8) -> Vec<u8> {
    let resp = IdentifyResponse {
        proto_version,
        firmware_ver: 1,
        build_hash: [0u8; 20],
        schema_hash: [0u8; 32],
        reset_epoch: 0,
        capabilities: 0,
        mcu_serial: *b"ETHERCAT-RT\0",
    };
    let body = resp.encode_body_to_array();
    debug_assert_eq!(body.len(), IDENTIFY_RESPONSE_BODY_LEN);
    control_frame(MessageKind::IdentifyResponse, cid, &body)
}

#[cfg(test)]
mod response_tests {
    use super::*;

    #[test]
    fn push_response_decodes_back() {
        let frame = push_segment_response_frame(42, 0, 7);
        let (chan, payload) = decode_frame(&frame).unwrap();
        assert_eq!(chan, CHANNEL_CONTROL);
        let (hdr, body) = decode_message_header(payload).unwrap();
        assert_eq!(hdr.correlation_id, 42);
        assert_eq!(MessageKind::from_u16(hdr.kind_raw), Some(MessageKind::PushSegmentResponse));
        let r = PushSegmentResponse::decode(body).unwrap();
        assert_eq!(r.accepted_segment_id, 7);
        assert_eq!(r.result, 0);
    }

    #[test]
    fn load_response_carries_handle() {
        let frame = load_curve_response_frame(5, 0, 0x0002_0003);
        let (_chan, payload) = decode_frame(&frame).unwrap();
        let (_hdr, body) = decode_message_header(payload).unwrap();
        let r = LoadCurveResponse::decode(body).unwrap();
        assert_eq!(r.curve_handle_packed, 0x0002_0003);
    }
}
```
(Confirm `mcu_serial` is `[u8; 12]`; `b"ETHERCAT-RT\0"` is exactly 12 bytes. Adjust the literal if the field width differs.)

- [ ] **Step 2: Run the tests**

Run: `cd rust && cargo test -p kalico-ethercat-rt response_tests`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/src/wire.rs
git commit -m "ethercat-rt: encode kalico-native responses as framed messages"
```

---

## Task 5: `CurveStore` over `CurvePool` (load + evaluate)

Wraps `runtime::curve_pool::CurvePool`. Loading a `LoadCurveCubic` returns a packed handle; evaluating a handle at a piece-local time returns mm. Reuses `try_alloc_and_load`, `lookup_active`, and `eval_position_velocity`.

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/curves.rs`

- [ ] **Step 1: Write the failing test**

In `rust/kalico-ethercat-rt/src/curves.rs`:
```rust
//! Curve storage + single-channel piece-walking evaluator.

use runtime::curve_pool::CurvePool;
use runtime::cubic_curve::{LoadedCubicCurve, WirePiece};
use runtime::monomial::eval_position_velocity;

use crate::wire::wire_pieces_from_bytes;

pub struct CurveStore {
    pool: CurvePool,
}

#[derive(Debug, PartialEq, Eq)]
pub enum LoadError {
    BadPieceBytes,
    PoolReject,
}

impl CurveStore {
    pub fn new() -> Self {
        Self { pool: CurvePool::new() }
    }

    /// Load a curve into `slot_idx`; return the packed handle to put in a response.
    pub fn load(&self, slot_idx: u16, piece_count: u8, pieces_bytes: &[u8]) -> Result<u32, LoadError> {
        let wire: Vec<WirePiece> =
            wire_pieces_from_bytes(piece_count, pieces_bytes).map_err(|_| LoadError::BadPieceBytes)?;
        let handle = self.pool.try_alloc_and_load(slot_idx as usize, &wire).ok_or(LoadError::PoolReject)?;
        Ok(handle.pack())
    }

    /// Borrow a loaded curve by packed handle. Returns None if stale/empty.
    /// SAFETY: the pool slot is not mutated while we hold this in the single-threaded DC loop.
    pub fn with_curve<R>(&self, handle_packed: u32, f: impl FnOnce(&LoadedCubicCurve) -> R) -> Option<R> {
        use runtime::cubic_curve::CurveHandle;
        let handle = CurveHandle::unpack(handle_packed);
        let ptr = self.pool.lookup_active(handle)?;
        // SAFETY: pointer valid for the lifetime of this call; no concurrent mutation.
        let curve: &LoadedCubicCurve = unsafe { &*ptr };
        Some(f(curve))
    }

    pub fn reset(&self) {
        self.pool.reset_all_retired_to_current();
    }
}

/// Evaluate position (mm) of a loaded curve at a given piece cursor + piece-local seconds.
pub fn eval_curve_at(curve: &LoadedCubicCurve, cursor: usize, t_local_s: f32) -> f32 {
    let (pos, _vel) = eval_position_velocity(&curve.pieces[cursor], t_local_s);
    pos
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one_piece_bytes(bp: [f32; 4], dur: f32) -> (u8, Vec<u8>) {
        let mut v = Vec::new();
        for x in bp { v.extend_from_slice(&x.to_bits().to_le_bytes()); }
        v.extend_from_slice(&dur.to_bits().to_le_bytes());
        (1, v)
    }

    #[test]
    fn ease_curve_endpoints() {
        let store = CurveStore::new();
        // Bernstein [0,0,10,10] => smooth 0->10 with zero velocity at both ends.
        let (pc, bytes) = one_piece_bytes([0.0, 0.0, 10.0, 10.0], 1.0);
        let handle = store.load(0, pc, &bytes).unwrap();

        let p0 = store.with_curve(handle, |c| eval_curve_at(c, 0, 0.0)).unwrap();
        let p1 = store.with_curve(handle, |c| eval_curve_at(c, 0, 1.0)).unwrap();
        let pmid = store.with_curve(handle, |c| eval_curve_at(c, 0, 0.5)).unwrap();

        assert!((p0 - 0.0).abs() < 1e-4, "start={p0}");
        assert!((p1 - 10.0).abs() < 1e-3, "end={p1}");
        assert!((pmid - 5.0).abs() < 1e-3, "mid={pmid}");  // symmetric ease => exactly half
    }
}
```

- [ ] **Step 2: Run the test**

Run: `cd rust && cargo test -p kalico-ethercat-rt curves::tests::ease_curve_endpoints`
Expected: PASS. (If `lookup_active`'s signature returns a different pointer/option shape, adjust `with_curve` to match the real `runtime` API; the contract is "borrow the LoadedCubicCurve for this handle.")

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/src/curves.rs
git commit -m "ethercat-rt: CurveStore over CurvePool with endpoint eval"
```

---

## Task 6: Single-channel segment piece-walker

Holds the active segment for one channel and walks pieces as monotonic time advances, returning the current position in mm. Time is `monotonic_ns`; piece durations are seconds (`*1e9` to compare).

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/curves.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/kalico-ethercat-rt/src/curves.rs`:
```rust
/// Active-segment state for one channel.
pub struct ChannelTrack {
    handle_packed: u32,
    t_start_ns: u64,
    t_end_ns: u64,
    cursor: usize,
    piece_start_ns: u64,
}

impl ChannelTrack {
    pub fn arm(handle_packed: u32, t_start_ns: u64, t_end_ns: u64) -> Self {
        Self { handle_packed, t_start_ns, t_end_ns, cursor: 0, piece_start_ns: t_start_ns }
    }

    pub fn is_done(&self, now_ns: u64) -> bool {
        now_ns >= self.t_end_ns
    }

    /// Advance the cursor past elapsed pieces and return current position (mm).
    /// Returns None if the curve is gone or the cursor is exhausted.
    pub fn sample(&mut self, store: &CurveStore, now_ns: u64) -> Option<f32> {
        if now_ns < self.t_start_ns {
            return store.with_curve(self.handle_packed, |c| eval_curve_at(c, 0, 0.0));
        }
        loop {
            let dur_s = store.with_curve(self.handle_packed, |c| {
                if self.cursor >= c.piece_count as usize { None } else { Some(c.pieces[self.cursor].duration) }
            })??;
            let dur_ns = (dur_s as f64 * 1e9) as u64;
            if now_ns.saturating_sub(self.piece_start_ns) >= dur_ns && dur_ns > 0 {
                self.piece_start_ns += dur_ns;
                self.cursor += 1;
            } else {
                break;
            }
        }
        let t_local_s = (now_ns.saturating_sub(self.piece_start_ns)) as f32 / 1e9;
        let cursor = self.cursor;
        store.with_curve(self.handle_packed, |c| {
            let idx = cursor.min(c.piece_count as usize - 1);
            eval_curve_at(c, idx, t_local_s.min(c.pieces[idx].duration))
        })
    }
}

#[cfg(test)]
mod walk_tests {
    use super::*;

    fn two_piece_bytes() -> (u8, Vec<u8>) {
        // piece 0: 0->10 (ease), 1s ; piece 1: 10->0 (ease), 1s
        let mut v = Vec::new();
        for x in [0.0f32, 0.0, 10.0, 10.0] { v.extend_from_slice(&x.to_bits().to_le_bytes()); }
        v.extend_from_slice(&1.0f32.to_bits().to_le_bytes());
        for x in [10.0f32, 10.0, 0.0, 0.0] { v.extend_from_slice(&x.to_bits().to_le_bytes()); }
        v.extend_from_slice(&1.0f32.to_bits().to_le_bytes());
        (2, v)
    }

    #[test]
    fn walks_two_pieces_continuously() {
        let store = CurveStore::new();
        let (pc, bytes) = two_piece_bytes();
        let handle = store.load(0, pc, &bytes).unwrap();
        let t0 = 1_000_000_000u64; // 1s in ns
        let mut track = ChannelTrack::arm(handle, t0, t0 + 2_000_000_000);

        let at = |track: &mut ChannelTrack, off_ns: u64| track.sample(&store, t0 + off_ns).unwrap();

        assert!((at(&mut track, 0) - 0.0).abs() < 1e-3);            // start of piece 0
        assert!((at(&mut track, 1_000_000_000) - 10.0).abs() < 1e-2); // boundary -> piece 1 start = 10
        assert!((at(&mut track, 2_000_000_000) - 0.0).abs() < 1e-2);  // end of piece 1
        assert!(track.is_done(t0 + 2_000_000_000));
    }
}
```

- [ ] **Step 2: Run the test**

Run: `cd rust && cargo test -p kalico-ethercat-rt walk_tests`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/src/curves.rs
git commit -m "ethercat-rt: single-channel Bezier piece-walker"
```

---

## Task 7: C SOEM shim `libecrt` (extract from `ec_spin.c`)

C owns the hard-won bring-up (`1C32:01=2`, SYNC0-before-SAFE-OP, CiA402 enable) and the cyclic PDO+DC exchange, exposed as a flat `extern "C"` surface. Single slave (index 1).

**Read first:** manual Ch. 8 (DC/PDO) before changing any object/sync code here.

**Files:**
- Create: `bench/libecrt.h`
- Create: `bench/libecrt.c`
- Create: `bench/Makefile`

- [ ] **Step 1: Write the public header**

`bench/libecrt.h`:
```c
#ifndef LIBECRT_H
#define LIBECRT_H
#include <stdint.h>

/* All functions operate on EtherCAT slave 1 (single-drive bring-up). */

/* go_realtime + ec_init + CSP/DC config + map + SAFE-OP + DC align + OP +
 * CiA402 enable, running an internal cyclic+DC loop with target=actual the
 * whole time. Returns 0 once "operation enabled", <0 on any failure. */
int  ec_rt_bringup(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio);

/* One steady-state DC cycle: sleep to next deadline, send+recv process data,
 * run the DC PI jitter correction, keep controlword=0x000F. Writes the PI
 * offset to *toff_ns. Returns the working counter (3 == healthy). */
int  ec_rt_cycle(int64_t *toff_ns);

/* Stage the CSP target for the next cycle's send. */
void ec_rt_set_target_position(int32_t counts);

int32_t  ec_rt_get_position_actual(void);
uint16_t ec_rt_get_statusword(void);
uint16_t ec_rt_get_error_code(void);
int32_t  ec_rt_get_following_error(void);

/* controlword = 0x0006 (disable voltage path), held for a few cycles. */
void ec_rt_disable(void);

/* dcsync0 off, back to INIT, close NIC. */
void ec_rt_shutdown(void);

#endif
```

- [ ] **Step 2: Write `libecrt.c` by lifting `ec_spin.c`'s proven logic**

`bench/libecrt.c` (bodies are the bench's, refactored — keep the exact SDO/DC ordering):
```c
#define _GNU_SOURCE
#include "libecrt.h"
#include <string.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include "ethercat.h"

#define COUNTS_PER_REV 131072.0

#pragma pack(push, 1)
typedef struct { uint16_t controlword; int32_t target_position; uint16_t touch_probe_fn; uint32_t phys_outputs; } out_t;
typedef struct { uint16_t error_code; uint16_t statusword; int32_t position_actual; int16_t torque_actual; int32_t following_error; uint16_t tp_status; int32_t tp1_pos; int32_t tp2_pos; uint32_t digital_inputs; } in_t;
#pragma pack(pop)

static char IOmap[4096];
static out_t *g_out;
static in_t  *g_in;
static int64_t g_cycle_ns;
static struct timespec g_ts;
static int64_t g_integral;

static void add_ts(struct timespec *ts, int64_t add) {
    int64_t ns = add % 1000000000LL, sec = (add - ns) / 1000000000LL;
    ts->tv_sec += sec; ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000LL) { ts->tv_nsec -= 1000000000LL; ts->tv_sec++; }
}
static void dc_sync(int64_t reftime, int64_t cycletime, int64_t *offset) {
    int64_t delta = reftime % cycletime;
    if (delta > cycletime / 2) delta -= cycletime;
    if (delta > 0) g_integral++;
    if (delta < 0) g_integral--;
    *offset = -(delta / 100) - (g_integral / 20);
}
static void go_realtime(int cpu, int prio) {
    mlockall(MCL_CURRENT | MCL_FUTURE);
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
    sched_setaffinity(0, sizeof(set), &set);
    struct sched_param sp; sp.sched_priority = prio;
    sched_setscheduler(0, SCHED_FIFO, &sp);
}
/* one PDO+DC exchange; returns wkc. */
static int rt_exchange(int64_t *toff) {
    int64_t off = 0;
    add_ts(&g_ts, g_cycle_ns + (toff ? *toff : 0));
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_ts, NULL);
    ec_send_processdata();
    int wkc = ec_receive_processdata(EC_TIMEOUTRET);
    dc_sync(ec_DCtime, g_cycle_ns, &off);
    if (toff) *toff = off;
    return wkc;
}

int ec_rt_bringup(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio) {
    g_cycle_ns = cycle_ns < 250000 ? 250000 : cycle_ns;
    g_integral = 0;
    go_realtime(rt_cpu, rt_prio);
    if (!ec_init(ifname)) return -1;
    if (ec_config_init(FALSE) <= 0) { ec_close(); return -2; }

    int8_t opmode = 8;                 /* CSP */
    ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(opmode), &opmode, EC_TIMEOUTRXM);
    uint16_t sync_dc = 2; uint32_t cyc = (uint32_t)g_cycle_ns;
    ec_SDOwrite(1, 0x1C32, 0x01, FALSE, sizeof(sync_dc), &sync_dc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C33, 0x01, FALSE, sizeof(sync_dc), &sync_dc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C32, 0x02, FALSE, sizeof(cyc), &cyc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C33, 0x02, FALSE, sizeof(cyc), &cyc, EC_TIMEOUTRXM);

    ec_configdc();
    ec_dcsync0(1, TRUE, (uint32_t)g_cycle_ns, (int32_t)(g_cycle_ns / 2));
    ec_config_map(&IOmap);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    g_out = (out_t *) ec_slave[1].outputs;
    g_in  = (in_t  *) ec_slave[1].inputs;
    g_out->controlword = 0; g_out->target_position = 0;
    g_out->touch_probe_fn = 0; g_out->phys_outputs = 0;

    clock_gettime(CLOCK_MONOTONIC, &g_ts);
    int64_t toff = 0;

    /* STABILIZE: align DC for ~1s, target tracks actual. */
    for (int64_t i = 0; i < (int64_t)(1.0e9 / g_cycle_ns); i++) {
        g_out->controlword = 0; g_out->target_position = g_in->position_actual;
        rt_exchange(&toff);
    }
    /* request OP */
    ec_slave[0].state = EC_STATE_OPERATIONAL; ec_writestate(0);
    for (int64_t i = 0; i < (int64_t)(2.0e9 / g_cycle_ns); i++) {
        g_out->target_position = g_in->position_actual;
        rt_exchange(&toff);
        if (i % 20 == 0) ec_readstate();
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) break;
    }
    if (ec_slave[0].state != EC_STATE_OPERATIONAL) return -3;

    /* CiA402 enable */
    for (int64_t pc = 0; pc < 3000; pc++) {
        uint16_t sw = g_in->statusword;
        g_out->target_position = g_in->position_actual;
        if (sw & 0x0008) g_out->controlword = ((pc / 10) % 2) ? 0x0080 : 0x0000; /* pulse fault reset */
        else if ((sw & 0x004F) == 0x0040) g_out->controlword = 0x0006;
        else if ((sw & 0x006F) == 0x0021) g_out->controlword = 0x0007;
        else if ((sw & 0x006F) == 0x0023) g_out->controlword = 0x000F;
        else if ((sw & 0x006F) == 0x0027) { g_out->controlword = 0x000F; rt_exchange(&toff); return 0; }
        else g_out->controlword = 0x0000;
        rt_exchange(&toff);
    }
    return -4;
}

int ec_rt_cycle(int64_t *toff_ns) {
    g_out->controlword = 0x000F;
    return rt_exchange(toff_ns);
}
void ec_rt_set_target_position(int32_t counts) { g_out->target_position = counts; }
int32_t  ec_rt_get_position_actual(void) { return g_in->position_actual; }
uint16_t ec_rt_get_statusword(void)      { return g_in->statusword; }
uint16_t ec_rt_get_error_code(void)      { return g_in->error_code; }
int32_t  ec_rt_get_following_error(void) { return g_in->following_error; }
void ec_rt_disable(void) {
    for (int i = 0; i < 100; i++) { g_out->controlword = 0x0006; int64_t t = 0; rt_exchange(&t); }
}
void ec_rt_shutdown(void) {
    ec_dcsync0(1, FALSE, 0, 0);
    ec_slave[0].state = EC_STATE_INIT; ec_writestate(0);
    ec_close();
}
```

- [ ] **Step 3: Write the Makefile (builds the static lib on the Pi)**

`bench/Makefile`:
```make
SOEM ?= $(HOME)/ethercat/SOEM
CFLAGS = -O2 -Wall -D_GNU_SOURCE \
  -I$(SOEM)/soem -I$(SOEM)/osal -I$(SOEM)/osal/linux -I$(SOEM)/oshw/linux -I$(SOEM)/oshw

libecrt.a: libecrt.o
	ar rcs $@ $^

libecrt.o: libecrt.c libecrt.h
	$(CC) $(CFLAGS) -c -o $@ libecrt.c

clean:
	rm -f libecrt.o libecrt.a
```

- [ ] **Step 4: Build the C lib on the Pi to verify it compiles**

```bash
SSHPASS=password sshpass -e scp bench/libecrt.c bench/libecrt.h bench/Makefile dderg@ethercat.local:~/ethercat/bench/
SSHPASS=password sshpass -e ssh dderg@ethercat.local 'cd ~/ethercat/bench && make clean && make' 
```
Expected: produces `libecrt.a`, no errors.

- [ ] **Step 5: Commit**

```bash
git add bench/libecrt.c bench/libecrt.h bench/Makefile
git commit -m "ethercat-rt: C SOEM shim libecrt extracted from ec_spin.c"
```

---

## Task 8: Rust FFI bindings + build linkage

**Files:**
- Create: `rust/kalico-ethercat-rt/src/ffi.rs`
- Create: `rust/kalico-ethercat-rt/build.rs`
- Modify: `rust/kalico-ethercat-rt/src/lib.rs` (add `pub mod ffi;`)

- [ ] **Step 1: Write the FFI declarations**

`rust/kalico-ethercat-rt/src/ffi.rs`:
```rust
//! Raw bindings to the C SOEM shim (`bench/libecrt`).
use std::os::raw::{c_char, c_int};

#[link(name = "ecrt", kind = "static")]
extern "C" {
    pub fn ec_rt_bringup(ifname: *const c_char, cycle_ns: i64, rt_cpu: c_int, rt_prio: c_int) -> c_int;
    pub fn ec_rt_cycle(toff_ns: *mut i64) -> c_int;
    pub fn ec_rt_set_target_position(counts: i32);
    pub fn ec_rt_get_position_actual() -> i32;
    pub fn ec_rt_get_statusword() -> u16;
    pub fn ec_rt_get_error_code() -> u16;
    pub fn ec_rt_get_following_error() -> i32;
    pub fn ec_rt_disable();
    pub fn ec_rt_shutdown();
}
```

- [ ] **Step 2: Write `build.rs` to find and link the libs**

`rust/kalico-ethercat-rt/build.rs`:
```rust
use std::env;

fn main() {
    // Directory holding libecrt.a (built by bench/Makefile on the host).
    // Override with ECRT_LIB_DIR; default to the repo's bench/ dir.
    let lib_dir = env::var("ECRT_LIB_DIR")
        .unwrap_or_else(|_| format!("{}/../../bench", env!("CARGO_MANIFEST_DIR")));
    println!("cargo:rustc-link-search=native={lib_dir}");
    println!("cargo:rustc-link-lib=static=ecrt");

    // SOEM static lib + its deps. Override SOEM_LIB_DIR if not alongside.
    if let Ok(soem_dir) = env::var("SOEM_LIB_DIR") {
        println!("cargo:rustc-link-search=native={soem_dir}");
    }
    println!("cargo:rustc-link-lib=static=soem");
    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-link-lib=rt");
    println!("cargo:rustc-link-lib=m");
    println!("cargo:rerun-if-changed=../../bench/libecrt.c");
    println!("cargo:rerun-if-changed=../../bench/libecrt.h");
}
```

- [ ] **Step 3: Add the module**

In `rust/kalico-ethercat-rt/src/lib.rs` add: `pub mod ffi;`

- [ ] **Step 4: Build on the Pi (FFI link only happens with the libs present)**

This task's link step only succeeds on the Pi where `libecrt.a` and `libsoem.a` exist. On the dev machine, verify the FFI module *compiles* (type-checks) without linking by building only the library target's check:

Run (dev machine): `cd rust && cargo check -p kalico-ethercat-rt`
Expected: PASS (cargo check does not link the bins).

On the Pi (full link), deferred to Task 11's deploy. Note in the commit message that link is Pi-only.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-ethercat-rt/src/ffi.rs rust/kalico-ethercat-rt/build.rs rust/kalico-ethercat-rt/src/lib.rs
git commit -m "ethercat-rt: FFI bindings + build.rs linkage to libecrt/SOEM (link is Pi-only)"
```

---

## Task 9: Endpoint main — socket server + DC loop

Single-threaded: bring the drive up, then loop forever. Each iteration: poll the socket for frames (non-blocking, with a short read timeout), apply any commands (load/push/reset/identify) and write responses, sample the active segment at the *current* monotonic time, scale to counts, set the target, run one DC cycle, log telemetry periodically.

**Files:**
- Create: `rust/kalico-ethercat-rt/src/server.rs`
- Rewrite: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`
- Modify: `rust/kalico-ethercat-rt/src/lib.rs` (add `pub mod server;`)

- [ ] **Step 1: Write the socket-server frame pump (`server.rs`)**

`rust/kalico-ethercat-rt/src/server.rs`:
```rust
//! Unix-socket server: decode kalico-native command frames, hand them to a
//! handler, write framed responses. Non-blocking poll suited to a DC loop.

use std::io::{ErrorKind, Read, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::time::Duration;

use kalico_native_transport::demux::{Demuxer, Frame};

use crate::wire::{decode_command, Command};

pub struct FrameServer {
    listener: UnixListener,
    conn: Option<UnixStream>,
    demux: Demuxer,
    buf: [u8; 4096],
}

impl FrameServer {
    pub fn bind(path: &str) -> std::io::Result<Self> {
        let _ = std::fs::remove_file(path);
        let listener = UnixListener::bind(path)?;
        listener.set_nonblocking(true)?;
        Ok(Self { listener, conn: None, demux: Demuxer::new(), buf: [0u8; 4096] })
    }

    /// Accept a pending client if we don't have one.
    fn try_accept(&mut self) {
        if self.conn.is_none() {
            match self.listener.accept() {
                Ok((stream, _)) => {
                    let _ = stream.set_read_timeout(Some(Duration::from_micros(200)));
                    self.conn = Some(stream);
                    eprintln!("ec-rt: client connected");
                }
                Err(ref e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(e) => eprintln!("ec-rt: accept error: {e}"),
            }
        }
    }

    /// Drain whatever bytes are available and return decoded commands.
    /// Responses are written by the caller via `respond`.
    pub fn poll_commands(&mut self) -> Vec<Command> {
        self.try_accept();
        let mut cmds = Vec::new();
        if let Some(stream) = self.conn.as_mut() {
            match stream.read(&mut self.buf) {
                Ok(0) => { eprintln!("ec-rt: client disconnected"); self.conn = None; }
                Ok(n) => {
                    let (frames, errs) = self.demux.feed_slice(&self.buf[..n]);
                    for e in errs { eprintln!("ec-rt: stream error: {e:?}"); }
                    for f in frames {
                        if let Frame::Kalico { payload, .. } = f {
                            match decode_command(&payload) {
                                Ok(cmd) => cmds.push(cmd),
                                Err(e) => eprintln!("ec-rt: bad command: {e:?}"),
                            }
                        }
                    }
                }
                Err(ref e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
                Err(e) => { eprintln!("ec-rt: read error: {e}"); self.conn = None; }
            }
        }
        cmds
    }

    pub fn respond(&mut self, frame: &[u8]) {
        if let Some(stream) = self.conn.as_mut() {
            if let Err(e) = stream.write_all(frame) { eprintln!("ec-rt: write error: {e}"); self.conn = None; }
        }
    }
}
```

- [ ] **Step 2: Write the endpoint main**

`rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`:
```rust
//! kalico-ethercat-rt: bring up the A6-EC in CSP/DC and stream the kalico-native
//! piece trajectory to it as encoder counts.
//!
//! Usage: kalico-ethercat-rt <ifname> [--socket PATH] [--cycle-us N]
//!        [--counts-per-mm F] [--rt-cpu N] [--rt-prio N] [--handle x|y|z|e]

use std::ffi::CString;
use std::time::{Duration, Instant};

use kalico_ethercat_rt::curves::{ChannelTrack, CurveStore};
use kalico_ethercat_rt::ffi;
use kalico_ethercat_rt::scale::CountMap;
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::wire::{
    identify_response_frame, load_curve_response_frame, push_segment_response_frame,
    reset_pool_response_frame, Command,
};

fn monotonic_ns() -> u64 {
    // CLOCK_MONOTONIC via std: anchor at process start.
    use std::sync::OnceLock;
    static START: OnceLock<Instant> = OnceLock::new();
    let start = START.get_or_init(Instant::now);
    start.elapsed().as_nanos() as u64
}

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let ifname = args.get(1).cloned().unwrap_or_else(|| "eth0".into());
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let cycle_us: i64 = arg_val(&args, "--cycle-us").and_then(|s| s.parse().ok()).unwrap_or(1000);
    let counts_per_mm: f64 = arg_val(&args, "--counts-per-mm").and_then(|s| s.parse().ok()).unwrap_or(3276.8);
    let rt_cpu: i32 = arg_val(&args, "--rt-cpu").and_then(|s| s.parse().ok()).unwrap_or(3);
    let rt_prio: i32 = arg_val(&args, "--rt-prio").and_then(|s| s.parse().ok()).unwrap_or(80);
    let handle_sel = arg_val(&args, "--handle").unwrap_or_else(|| "x".into());
    let cycle_ns = cycle_us * 1000;

    let store = CurveStore::new();
    let mut track: Option<ChannelTrack> = None;
    let mut pending: Option<(u32, u64, u64)> = None; // (handle_packed, t_start_ns, t_end_ns)
    let mut cmap: Option<CountMap> = None;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt: socket {socket}, cycle {cycle_us}us, counts/mm {counts_per_mm}, handle {handle_sel}");

    // Bring up the drive (blocks until operation enabled).
    let cif = CString::new(ifname.clone()).unwrap();
    let rc = unsafe { ffi::ec_rt_bringup(cif.as_ptr(), cycle_ns, rt_cpu, rt_prio) };
    if rc != 0 { eprintln!("ec-rt: bringup failed rc={rc}"); std::process::exit(1); }
    eprintln!("ec-rt: drive enabled, entering DC loop");

    let pick_handle = |seg: &kalico_protocol::messages::PushSegment| match handle_sel.as_str() {
        "y" => seg.handle_y, "z" => seg.handle_z, "e" => seg.handle_e, _ => seg.handle_x,
    };

    let mut prdiv = 0u64;
    loop {
        // 1) Service socket commands.
        for cmd in server.poll_commands() {
            match cmd {
                Command::Identify { correlation_id, proto_version } => {
                    server.respond(&identify_response_frame(correlation_id, proto_version));
                }
                Command::LoadCurve { correlation_id, msg } => {
                    match store.load(msg.slot_idx, msg.piece_count, &msg.pieces_bytes) {
                        Ok(handle) => server.respond(&load_curve_response_frame(correlation_id, 0, handle)),
                        Err(e) => { eprintln!("ec-rt: load err {e:?}"); server.respond(&load_curve_response_frame(correlation_id, -1, 0)); }
                    }
                }
                Command::PushSegment { correlation_id, msg } => {
                    let h = pick_handle(&msg);
                    pending = Some((h, msg.t_start, msg.t_end));
                    server.respond(&push_segment_response_frame(correlation_id, 0, msg.id));
                }
                Command::ResetPool { correlation_id } => {
                    store.reset(); track = None; pending = None;
                    server.respond(&reset_pool_response_frame(correlation_id, 0));
                }
                Command::Unknown { kind_raw, .. } => eprintln!("ec-rt: ignoring kind 0x{kind_raw:04x}"),
            }
        }

        let now = monotonic_ns();

        // 2) Arm a pending segment.
        if let Some((h, t0, t1)) = pending.take() {
            track = Some(ChannelTrack::arm(h, t0, t1));
        }

        // 3) Sample trajectory -> counts.
        if let Some(tr) = track.as_mut() {
            if let Some(pos_mm) = tr.sample(&store, now) {
                let map = cmap.get_or_insert_with(|| {
                    let actual = unsafe { ffi::ec_rt_get_position_actual() };
                    CountMap::new(counts_per_mm, actual, pos_mm as f64)
                });
                let counts = map.target_counts(pos_mm as f64);
                unsafe { ffi::ec_rt_set_target_position(counts) };
            }
            if tr.is_done(now) { track = None; }
        }

        // 4) One DC cycle.
        let mut toff = 0i64;
        let wkc = unsafe { ffi::ec_rt_cycle(&mut toff) };

        // 5) Telemetry every ~0.5s.
        prdiv += 1;
        if prdiv >= (500_000 / cycle_us as u64).max(1) {
            prdiv = 0;
            let (sw, err, pos, ferr) = unsafe {
                (ffi::ec_rt_get_statusword(), ffi::ec_rt_get_error_code(),
                 ffi::ec_rt_get_position_actual(), ffi::ec_rt_get_following_error())
            };
            eprintln!("ec-rt: wkc={wkc} sw=0x{sw:04x} err=0x{err:04x} pos={pos} ferr={ferr} toff={toff} active={}", track.is_some());
            if err != 0 { eprintln!("ec-rt: DRIVE FAULT err=0x{err:04x}, disabling"); break; }
        }
        let _ = Duration::from_secs(0); // cycle pacing is inside ec_rt_cycle
    }

    unsafe { ffi::ec_rt_disable(); ffi::ec_rt_shutdown(); }
    eprintln!("ec-rt: shutdown complete");
}
```

- [ ] **Step 3: Add the module and build-check on the dev machine**

In `rust/kalico-ethercat-rt/src/lib.rs` add `pub mod server;`.

Run (dev machine): `cd rust && cargo check -p kalico-ethercat-rt`
Expected: PASS (no link). Fix any signature mismatches against the real `runtime`/transport APIs surfaced by the compiler.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-ethercat-rt/src/server.rs rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs rust/kalico-ethercat-rt/src/lib.rs
git commit -m "ethercat-rt: endpoint main (socket server + DC loop + evaluator)"
```

---

## Task 10: Test client — send a gentle ease-in/ease-out move

A small binary that connects to the socket, sends one `LoadCurveCubic` (two ease pieces: out and back) and one `PushSegment` with `t_start = now + 100ms`, reads the responses, and exits. Uses the same `wire` helpers as the server, so it exercises real framing.

**Files:**
- Rewrite: `rust/kalico-ethercat-rt/src/bin/ec-test-client.rs`

- [ ] **Step 1: Write the client**

`rust/kalico-ethercat-rt/src/bin/ec-test-client.rs`:
```rust
//! Sends one gentle there-and-back move to a running kalico-ethercat-rt.
//! Usage: ec-test-client [--socket PATH] [--mm F] [--secs F]

use std::io::{Read, Write};
use std::os::unix::net::UnixStream;
use std::time::{Duration, Instant};

use kalico_protocol::codec::Encode;
use kalico_protocol::messages::{LoadCurveCubic, MessageKind, PushSegment};
use kalico_ethercat_rt::wire::{control_frame};

fn piece(bp: [f32; 4], dur: f32, out: &mut Vec<u8>) {
    for x in bp { out.extend_from_slice(&x.to_bits().to_le_bytes()); }
    out.extend_from_slice(&dur.to_bits().to_le_bytes());
}

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let mm: f32 = arg_val(&args, "--mm").and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let secs: f32 = arg_val(&args, "--secs").and_then(|s| s.parse().ok()).unwrap_or(2.0);

    let mut stream = UnixStream::connect(&socket).expect("connect");

    // Curve: ease 0->mm over secs, then ease mm->0 over secs. Zero velocity at all knots.
    let mut pieces = Vec::new();
    piece([0.0, 0.0, mm, mm], secs, &mut pieces);
    piece([mm, mm, 0.0, 0.0], secs, &mut pieces);
    let load = LoadCurveCubic { slot_idx: 0, axis_idx: 0, piece_count: 2, pieces_bytes: pieces };
    stream.write_all(&control_frame(MessageKind::LoadCurveCubic, 1, &load.encoded_to_vec())).unwrap();

    // The endpoint loads into the slot we named; in M1 the handle for slot 0 is
    // generation 1 => packed (1<<16)|0 = 0x00010000. Use that for the push.
    let handle_packed: u32 = 0x0001_0000;

    // t_start = now + 100ms; t_end = start + 2*secs (both pieces).
    let now_ns = || Instant::now().elapsed().as_nanos() as u64; // anchor-free; see note
    // Anchor: use a fixed epoch read once. The endpoint uses its own monotonic
    // anchor, so we send a *relative* lead and the endpoint treats t_start as
    // its-clock absolute. For M1 we send t_start as the endpoint will interpret
    // it: send 0 lead and let the endpoint arm immediately (t_start in the past).
    let _ = now_ns;
    let t_start = 0u64;
    let t_end = (2.0 * secs * 1e9) as u64;

    let seg = PushSegment {
        id: 1, handle_x: handle_packed, handle_y: 0, handle_z: 0, handle_e: 0,
        t_start, t_end, kinematics: 0, e_mode: 0, extrusion_ratio: 0.0,
    };
    stream.write_all(&control_frame(MessageKind::PushSegment, 2, &seg.encoded_to_vec())).unwrap();

    // Read responses for ~500ms then exit.
    stream.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
    let mut buf = [0u8; 1024];
    let deadline = Instant::now() + Duration::from_millis(500);
    while Instant::now() < deadline {
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => eprintln!("client: {n} response bytes"),
            Err(_) => break,
        }
    }
    eprintln!("client: sent load + push (mm={mm}, secs={secs})");
}
```

> **IMPORTANT time-domain note for the executor:** the endpoint arms `t_start`
> against *its own* `monotonic_ns()` epoch (process start). A client `t_start=0`
> means "already started" → the endpoint arms immediately and `sample()` returns
> piece-0 start (no jump, because `CountMap` captures origin at first sample).
> This is correct for M1 (immediate gentle move). When Plan 2 wires
> `motion-bridge`, it will supply real shared-clock timestamps; do not hardcode
> `t_start=0` there. Leave this comment in the code.

- [ ] **Step 2: Build-check on the dev machine**

Run: `cd rust && cargo check -p kalico-ethercat-rt --bins --features hw`
Expected: PASS (no link — `cargo check` type-checks the FFI-using endpoint bin
without linking libecrt/SOEM). Without `--features hw` the endpoint bin is gated
out (`required-features`); the FFI module and native link only exist under `hw`.
Also run `cd rust && cargo test -p kalico-ethercat-rt` (default, no `hw`): the
scale/wire/curves unit tests run locally because the default build is pure Rust.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/src/bin/ec-test-client.rs
git commit -m "ethercat-rt: test client sends a gentle there-and-back move"
```

---

## Task 11: Hardware bring-up on the Pi (manual verification)

End-to-end on `dderg@ethercat.local` with the drive powered and the motor free to spin. **Authorize motion with the user before running.** Read the manual's Ch. 10 fault list before interpreting any `err=` code.

**Files:** none (deploy + run).

- [ ] **Step 1: Build SOEM as a static lib if not already present**

The `bench/Makefile` expects `~/ethercat/SOEM/build/libsoem.a`. If missing:
```bash
SSHPASS=password sshpass -e ssh dderg@ethercat.local \
  'cd ~/ethercat/SOEM && cmake -S . -B build && make -C build -j$(nproc)'
```
Set `SOEM_LIB_DIR=~/ethercat/SOEM/build` for the Rust link.

- [ ] **Step 2: Sync the repo to the Pi and build everything there**

Per the bench-firmware flow, the Pi builds from its own checkout. Push the branch, pull on the Pi, build the C lib + Rust bins:
```bash
git push origin ethercat
SSHPASS=password sshpass -e ssh dderg@ethercat.local '
  cd ~/kalico && git fetch && git checkout ethercat && git pull &&
  cd bench && make clean && make &&
  cd ~/kalico/rust &&
  ECRT_LIB_DIR=$HOME/kalico/bench SOEM_LIB_DIR=$HOME/ethercat/SOEM/build \
    cargo build --release -p kalico-ethercat-rt --features hw'
```
Expected: both `kalico-ethercat-rt` and `ec-test-client` link and build.
**Note:** `--features hw` is required — the endpoint bin (`kalico-ethercat-rt`)
has `required-features = ["hw"]`, which enables the EtherCAT FFI and the
libecrt/SOEM native link. Without it, only the lib + `ec-test-client` build.
(If the Pi's repo lives at a different path than `~/kalico`, adjust. `ECRT_LIB_DIR` must point at the dir containing the freshly built `libecrt.a`.)

- [ ] **Step 3: Run the endpoint (terminal A) — confirm bring-up**

```bash
SSHPASS=password sshpass -e ssh dderg@ethercat.local '
  echo performance | sudo -S tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null <<< password
  cd ~/kalico/rust &&
  sudo -S ./target/release/kalico-ethercat-rt eth0 --cycle-us 1000 --counts-per-mm 3276.8 <<< password'
```
Expected: `drive enabled, entering DC loop`, then periodic `wkc=3 sw=0x… err=0x0000 … active=false` telemetry. If `bringup failed`, read the printed state/AL code and the manual Ch. 8/10; do not proceed.

- [ ] **Step 4: Send the gentle move (terminal B)**

```bash
SSHPASS=password sshpass -e ssh dderg@ethercat.local \
  'cd ~/kalico/rust && ./target/release/ec-test-client --mm 20 --secs 2'
```
Expected: the motor eases ~20 mm-equivalent out and back smoothly; endpoint telemetry shows `active=true`, `ferr` gliding through small values (no spike), `err=0x0000` throughout, returning to rest.

- [ ] **Step 5: Record the result in the bench README**

Append a "kalico-ethercat-rt" section to `bench/README.md` documenting: the socket protocol path proven, the cycle rate, `counts_per_mm` used, and the observed following-error band. Commit:
```bash
git add bench/README.md
git commit -m "ethercat-rt: document end-to-end socket->CSP bring-up result"
git push origin ethercat
```

---

## Self-review

**Spec coverage** (against `2026-05-30-ethercat-motion-node-design.md`):
- "EtherCAT = clock-synced kalico-native node": ✅ endpoint speaks kalico-native over a socket; shared `CLOCK_MONOTONIC` (Tasks 9, 10, time-domain note).
- "reuse the runtime evaluator": ✅ `CurvePool` + `eval_position_velocity` (Tasks 5, 6).
- "reuse bench SOEM/CSP/DC bring-up": ✅ `libecrt` extracted from `ec_spin.c` (Task 7).
- "axis-agnostic": ✅ `--handle` selects a slot; no X/Y/Z semantics in the endpoint (Task 9).
- "passthrough / unshaped first light": ✅ client sends raw cubic pieces, no shaper (Task 10).
- "uniform state readback (servo feedback)": partial — getters exist (`ec_rt_get_position_actual`/`following_error`, Task 7/8); exposing them over the protocol as a query is **Plan 2** (the unified query is explicitly deferred per the spec). Not a gap.
- "don't touch endstop/homing path": ✅ nothing here touches it.
- "no rewrite of STM32 path": ✅ this plan adds a new crate only; the `MotionNode` extraction is Plan 2.

**Placeholder scan:** no TBD/TODO; all code blocks are complete. The two `cargo check`-only tasks (8, 9, 10 on dev machine) are intentional — linking is Pi-only and happens in Task 11.

**Type consistency:** `CountMap` (scale.rs) used in Task 1 and Task 9; `wire_pieces_from_bytes`/`decode_command`/`control_frame`/`*_response_frame` (wire.rs) consistent across Tasks 2–4, 9, 10; `CurveStore`/`ChannelTrack`/`eval_curve_at` (curves.rs) consistent across Tasks 5, 6, 9; `ffi::*` names match `libecrt.h` (Tasks 7, 8, 9). Handle packing: `0x0001_0000` in the client (Task 10) matches `CurveHandle::pack` = `(gen<<16)|slot` with gen=1, slot=0 — valid for the first load into slot 0.

**Known executor adjustments expected:** exact `runtime` API shapes (`lookup_active` return type, `CurvePool::new` constness, `reset_all_retired_to_current` name) and transport module paths (`demux`, `frame`, `wire_helpers`) may differ slightly from what the planning agents reported; the compiler will surface these and the fixes are mechanical. The `mcu_serial` width and `IdentifyResponse` field set should be verified against `kalico-protocol/src/bootstrap.rs`.
