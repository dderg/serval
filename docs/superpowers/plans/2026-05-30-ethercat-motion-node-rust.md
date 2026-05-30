# EtherCAT Motion-Node (Rust side) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Use the `rust-engineer` subagent for every implementation task (project convention).

**Goal:** Introduce a `MotionNode` abstraction so the planner's dispatch path can drive both the existing serial stepper MCUs and a same-host EtherCAT RT process (the Plan-1 endpoint) as peer motion outputs — entirely on the Rust side, with no klippy/config changes.

**Architecture:** Approach B from the design spec. (1) Hoist the one method the producers use, `kalico_call`, onto a small object-safe `NativeCall` trait and make `producer::load_curve`/`push_segment` generic over it. (2) Add `UnixNativeConn: NativeCall`, a blocking same-host Unix-socket client speaking pure kalico-native frames. (3) Define a `MotionNode` trait whose surface is exactly the two clock lookups (`now_clock`, `clock_freq`) plus `load_and_push`; the dispatch closure's fragile per-MCU clock-base arithmetic stays in place and only its two inlined lookups become trait calls. (4) `StepperMcuNode` lifts the existing serial behaviour verbatim; `EtherCatNode` uses `monotonic_ns()` + `freq = 1e9` over `UnixNativeConn`.

**Tech Stack:** Rust (workspace: `kalico-host-rt`, `motion-bridge`, `kalico-native-transport`, `kalico-protocol`, `runtime`). No new external deps. `libc` (already a dep of `kalico-ethercat-rt`; add to `kalico-host-rt` for `monotonic_ns` if needed, or reuse an existing clock).

**Out of scope (deferred to the with-user integration step):** Klipper `printer.cfg`, axis→node mapping config surface, passthrough-anywhere wired into real config, the first live `G1` jog, and wiring `EtherCatNode` into `init_planner`'s live node map. `EtherCatNode` is built and integration-tested against a stub/real endpoint in this plan but is **not** yet constructed inside `init_planner` (there is no config surface for it yet). Its integration test is its coverage — it is not dead code.

---

## Pre-flight context (read once before Task 0)

Key current facts, verified 2026-05-30 against the worktree:

- `producer::load_curve` / `push_segment` / `push_segment_with_timeout` live in `rust/kalico-host-rt/src/producer.rs` and reach the wire **only** through `io.kalico_call(kind, body, timeout)`. They take `io: &KalicoHostIo` concretely today.
- `KalicoHostIo::kalico_call` is an **inherent** method (`rust/kalico-host-rt/src/host_io/mod.rs:753`) with signature
  `fn kalico_call(&self, kind: MessageKind, body: Vec<u8>, timeout: Duration) -> Result<(MessageKind, Vec<u8>), TransportError>`.
- On the wire `kalico_call` emits a kalico-native frame built by `build_kalico_control_frame(kind, cid, body)` in `rust/kalico-host-rt/src/host_io/kalico_native.rs` — `0x55 | len:u16 | channel:u8 | (kind:u16|version:u8|cid:u32) | body | crc16`. This is byte-identical to what the Plan-1 endpoint decodes.
- The dispatch closure is in `rust/motion-bridge/src/bridge.rs` (`init_planner`), closure starts at line 2137. The per-MCU clock-base arithmetic is roughly lines 2230–2400; the slot-alloc + `producer::load_curve` + `dispatch_push_segment` inner loop is roughly lines 2465–2557. `dispatch_push_segment` is a free fn at `bridge.rs:372`.
- `McuPushPlan` and `McuAxisConfig` are in `rust/motion-bridge/src/dispatch.rs:44` / `:92`; `McuPushPlan::set_handle` at `:102`.
- The serial path's `now_clock` source is `router.compute_ack_clock(mcu_h)` (with a 5 s block-wait loop) and `freq` is `clock_freqs[mcu_id]` (`Arc<Mutex<HashMap<u32, f64>>>`, fed by `set_clock_est`).

Run all Rust commands from `rust/` (the workspace root): `cd rust && cargo ...`.

---

## File structure

| File | Responsibility |
| --- | --- |
| `rust/kalico-host-rt/src/native_call.rs` | **New.** `NativeCall` trait + `impl NativeCall for KalicoHostIo`. |
| `rust/kalico-host-rt/src/producer.rs` | **Modify.** Genericize `load_curve`/`push_segment`/`push_segment_with_timeout`/`reset_curve_pool` from `&KalicoHostIo` to `&dyn NativeCall`. |
| `rust/kalico-host-rt/src/unix_native_conn.rs` | **New.** `UnixNativeConn: NativeCall` — blocking same-host Unix-socket client speaking kalico-native. |
| `rust/kalico-host-rt/src/lib.rs` | **Modify.** `pub mod native_call;` `pub mod unix_native_conn;`. |
| `rust/motion-bridge/src/motion_node.rs` | **New.** `MotionNode` trait + `EtherCatNode` + `StepperMcuNode`. |
| `rust/motion-bridge/src/bridge.rs` | **Modify.** Build a per-MCU `MotionNode` map; rewire the dispatch closure onto `node.now_clock()`/`clock_freq()`/`load_and_push()`. |
| `rust/motion-bridge/src/lib.rs` | **Modify.** `mod motion_node;` (+ `pub use` as needed). |

---

## Task 0: `NativeCall` trait + `impl` for `KalicoHostIo`

Introduce the one-method seam the producers actually use. Object-safe so it works as `&dyn NativeCall`.

**Files:**
- Create: `rust/kalico-host-rt/src/native_call.rs`
- Modify: `rust/kalico-host-rt/src/lib.rs`

- [ ] **Step 1: Create the trait + impl**

`rust/kalico-host-rt/src/native_call.rs`:
```rust
//! `NativeCall`: the single request/response primitive the curve/segment
//! producers need. Hoisted off `KalicoHostIo` so the producer functions can
//! drive any kalico-native peer (a serial MCU via `KalicoHostIo`, or a
//! same-host EtherCAT RT process via `UnixNativeConn`) without caring which.
//!
//! One frame out (`kind` + `body`), one frame in (matching `correlation_id`).
//! Object-safe: callers use `&dyn NativeCall`.

use std::time::Duration;

use kalico_protocol::MessageKind;

use crate::transport::TransportError;

pub trait NativeCall: Send + Sync {
    /// Issue a kalico-native control-channel call: send `kind` + `body`, block
    /// until the correlation-matched response arrives or `timeout` elapses.
    fn kalico_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError>;
}

impl NativeCall for crate::host_io::KalicoHostIo {
    fn kalico_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError> {
        // Resolves to the inherent `KalicoHostIo::kalico_call` (inherent
        // methods take priority over trait methods in method-call syntax),
        // so this forwards rather than recursing.
        crate::host_io::KalicoHostIo::kalico_call(self, kind, body, timeout)
    }
}
```

- [ ] **Step 2: Register the module**

In `rust/kalico-host-rt/src/lib.rs`, add (next to the other `pub mod` lines):
```rust
pub mod native_call;
```

- [ ] **Step 3: Build to verify it compiles**

Run: `cd rust && cargo build -p kalico-host-rt`
Expected: PASS. (If `KalicoHostIo`'s module path differs from `crate::host_io::KalicoHostIo`, fix the path the compiler reports — it is re-exported from `host_io`.)

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/native_call.rs rust/kalico-host-rt/src/lib.rs
git commit -m "host-rt: add NativeCall trait + impl for KalicoHostIo"
```

---

## Task 1: Genericize the producer functions over `&dyn NativeCall`

The producers use only `io.kalico_call`, so swapping the concrete `&KalicoHostIo` for `&dyn NativeCall` is a signature-only change. Existing callsites pass `&KalicoHostIo` (or `io.as_ref()` from `Arc<KalicoHostIo>`), which coerces to `&dyn NativeCall` automatically.

**Files:**
- Modify: `rust/kalico-host-rt/src/producer.rs`

- [ ] **Step 1: Import the trait**

At the top of `rust/kalico-host-rt/src/producer.rs`, add to the imports:
```rust
use crate::native_call::NativeCall;
```

- [ ] **Step 2: Change the four signatures**

Replace `io: &KalicoHostIo` with `io: &dyn NativeCall` in each of these functions (bodies unchanged — they already call only `io.kalico_call(...)`):

`push_segment` (currently `producer.rs:127`):
```rust
pub fn push_segment(
    io: &dyn NativeCall,
    credit: &CreditCounter,
    params: &SegmentPushParams,
) -> Result<PushedSegmentInfo, ProducerError> {
    push_segment_with_timeout(io, credit, params, DEFAULT_PUSH_RESPONSE_TIMEOUT)
}
```

`push_segment_with_timeout` (currently `producer.rs:135`):
```rust
pub fn push_segment_with_timeout(
    io: &dyn NativeCall,
    credit: &CreditCounter,
    params: &SegmentPushParams,
    timeout: Duration,
) -> Result<PushedSegmentInfo, ProducerError> {
```

`load_curve` (currently `producer.rs:302`):
```rust
pub fn load_curve(
    io: &dyn NativeCall,
    slot: u16,
    axis_idx: u8,
    params: &CurveLoadParams,
    timeout: Duration,
) -> Result<u32, ProducerError> {
```

`reset_curve_pool` (find it below `load_curve`, ~`producer.rs:400+` — it also takes `io: &KalicoHostIo` and calls only `io.kalico_call`). Change its `io` parameter to `&dyn NativeCall` the same way.

> If the now-unused `use crate::host_io::KalicoHostIo;` (or similar) in `producer.rs` triggers an unused-import warning, remove it. If `KalicoHostIo` is still referenced elsewhere in the file, leave it.

- [ ] **Step 3: Update the bridge callsites if the compiler requires it**

`bridge.rs:372` `dispatch_push_segment` takes `io: &KalicoHostIo` and calls `producer::push_segment_with_timeout(io, ...)`. `&KalicoHostIo` coerces to `&dyn NativeCall`, so the call still type-checks with no change. Leave it for now (the closure rewire in Task 5 replaces this helper).

- [ ] **Step 4: Build the whole workspace**

Run: `cd rust && cargo build --workspace`
Expected: PASS. Any failure is a missed callsite — fix by passing `&dyn NativeCall` / `io.as_ref()`.

- [ ] **Step 5: Run the existing producer tests (regression gate)**

Run: `cd rust && cargo test -p kalico-host-rt`
Expected: PASS. If a test constructs a mock and passes it to a producer fn, that mock must now `impl NativeCall`. Add the impl to the test mock (forward to its existing `kalico_call`-equivalent, or to a canned response). If no producer tests exist, that's fine — the build is the gate.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-host-rt/src/producer.rs
git commit -m "host-rt: producers take &dyn NativeCall instead of &KalicoHostIo"
```

---

## Task 2: `UnixNativeConn` — same-host kalico-native socket client

A blocking `NativeCall` over a `UnixStream`: allocate a correlation id, frame the request with the existing `build_kalico_control_frame`, write it, then read with a `Demuxer` until the correlation-matched response arrives or the deadline passes. This is the host-side peer of the Plan-1 endpoint.

**Files:**
- Create: `rust/kalico-host-rt/src/unix_native_conn.rs`
- Modify: `rust/kalico-host-rt/src/lib.rs`

- [ ] **Step 1: Write the failing integration test first**

Create `rust/kalico-host-rt/src/unix_native_conn.rs` with the implementation skeleton + a test that runs a stub responder over a real `UnixListener`:

```rust
//! `UnixNativeConn`: a blocking same-host Unix-socket client speaking pure
//! kalico-native frames. Implements [`NativeCall`] so the curve/segment
//! producers drive an EtherCAT RT endpoint exactly as they drive a serial
//! `KalicoHostIo`. Same-host ⇒ no clock-sync round-trips; the caller stamps
//! segment times on the shared `CLOCK_MONOTONIC` (see `EtherCatNode`).

use std::io::{Read, Write};
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::wire_helpers::decode_message_header;
use kalico_protocol::MessageKind;

use crate::host_io::kalico_native::build_kalico_control_frame;
use crate::native_call::NativeCall;
use crate::transport::TransportError;

/// Mutable I/O state guarded together so `kalico_call(&self, ...)` is `Sync`.
struct ConnState {
    stream: UnixStream,
    demux: Demuxer,
    buf: [u8; 4096],
}

pub struct UnixNativeConn {
    state: Mutex<ConnState>,
    next_cid: AtomicU32,
}

impl UnixNativeConn {
    /// Connect to a listening kalico-native endpoint at `path`.
    pub fn connect(path: &str) -> std::io::Result<Self> {
        let stream = UnixStream::connect(path)?;
        Ok(Self::from_stream(stream))
    }

    /// Wrap an already-connected stream (used by tests via `UnixStream::pair`).
    pub fn from_stream(stream: UnixStream) -> Self {
        Self {
            state: Mutex::new(ConnState {
                stream,
                demux: Demuxer::new(),
                buf: [0u8; 4096],
            }),
            // Start at 1 so a zero correlation id never collides with a
            // freshly-zeroed field on the wire.
            next_cid: AtomicU32::new(1),
        }
    }
}

impl NativeCall for UnixNativeConn {
    fn kalico_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError> {
        let cid = self.next_cid.fetch_add(1, Ordering::Relaxed);
        let frame = build_kalico_control_frame(kind, cid, &body);

        let mut st = self
            .state
            .lock()
            .unwrap_or_else(|p| p.into_inner());

        st.stream.write_all(&frame).map_err(TransportError::Io)?;

        // Bound each blocking read so the deadline is honoured even if the
        // peer goes silent.
        st.stream
            .set_read_timeout(Some(Duration::from_millis(50)))
            .map_err(TransportError::Io)?;

        let deadline = Instant::now() + timeout;
        loop {
            if Instant::now() >= deadline {
                return Err(TransportError::Timeout);
            }
            let ConnState { stream, demux, buf } = &mut *st;
            let n = match stream.read(buf) {
                Ok(0) => return Err(TransportError::Closed),
                Ok(n) => n,
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    continue;
                }
                Err(e) => return Err(TransportError::Io(e)),
            };
            let (frames, _errs) = demux.feed_slice(&buf[..n]);
            for f in frames {
                if let Frame::Kalico { payload, .. } = f {
                    if let Some((hdr, resp_body)) = decode_message_header(&payload) {
                        if hdr.correlation_id == cid {
                            let resp_kind = MessageKind::from_u16(hdr.kind_raw)
                                .ok_or_else(|| {
                                    TransportError::Parse(format!(
                                        "unknown response kind 0x{:04x}",
                                        hdr.kind_raw
                                    ))
                                })?;
                            return Ok((resp_kind, resp_body.to_vec()));
                        }
                        // Different correlation id (e.g. an async event):
                        // ignore and keep reading for ours.
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL};
    use kalico_native_transport::wire_helpers::{encode_message_header, MESSAGE_VERSION_DEFAULT};
    use std::thread;

    /// Stub endpoint: read one framed request, reply with `reply_kind` echoing
    /// the request's correlation id and a fixed body.
    fn spawn_stub(mut peer: UnixStream, reply_kind: MessageKind, reply_body: Vec<u8>) {
        thread::spawn(move || {
            let mut demux = Demuxer::new();
            let mut buf = [0u8; 4096];
            loop {
                let n = match peer.read(&mut buf) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => n,
                };
                let (frames, _e) = demux.feed_slice(&buf[..n]);
                for f in frames {
                    if let Frame::Kalico { payload, .. } = f {
                        let (hdr, _b) = decode_message_header(&payload).unwrap();
                        let mut out = encode_message_header(
                            reply_kind,
                            MESSAGE_VERSION_DEFAULT,
                            hdr.correlation_id,
                        )
                        .to_vec();
                        out.extend_from_slice(&reply_body);
                        let frame = encode_frame(CHANNEL_CONTROL, &out);
                        peer.write_all(&frame).unwrap();
                        return;
                    }
                }
            }
        });
    }

    #[test]
    fn round_trips_a_call_by_correlation_id() {
        let (client, server) = UnixStream::pair().unwrap();
        spawn_stub(
            server,
            MessageKind::LoadCurveResponse,
            vec![1, 2, 3, 4],
        );
        let conn = UnixNativeConn::from_stream(client);
        let (kind, body) = conn
            .kalico_call(MessageKind::LoadCurveCubic, vec![9, 9], Duration::from_secs(2))
            .expect("call ok");
        assert_eq!(kind, MessageKind::LoadCurveResponse);
        assert_eq!(body, vec![1, 2, 3, 4]);
    }

    #[test]
    fn times_out_when_peer_silent() {
        let (client, _server) = UnixStream::pair().unwrap();
        // _server never replies.
        let conn = UnixNativeConn::from_stream(client);
        let r = conn.kalico_call(
            MessageKind::PushSegment,
            vec![],
            Duration::from_millis(150),
        );
        assert!(matches!(r, Err(TransportError::Timeout)));
    }
}
```

- [ ] **Step 2: Register the module**

In `rust/kalico-host-rt/src/lib.rs`, add:
```rust
pub mod unix_native_conn;
```

- [ ] **Step 3: Run the tests to verify they fail, then pass**

Run: `cd rust && cargo test -p kalico-host-rt unix_native_conn`
Expected: PASS. Likely fix-ups the compiler will force:
- `build_kalico_control_frame`'s module path: it is in `crate::host_io::kalico_native`. If it is not `pub`, make it `pub(crate)` or `pub` (it is already used cross-module by the reactor). If `kalico_native` is a private module, add `pub(crate) use` or widen visibility minimally.
- `decode_message_header`'s returned header field names (`correlation_id`, `kind_raw`) — match the real struct (confirmed in `kalico-native-transport::wire_helpers`).
- `MessageKind::from_u16` returns `Option<MessageKind>` (confirmed).

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/unix_native_conn.rs rust/kalico-host-rt/src/lib.rs
git commit -m "host-rt: UnixNativeConn — blocking same-host kalico-native socket client"
```

---

## Task 3: `MotionNode` trait + `EtherCatNode`

Define the dispatch-facing abstraction and the EtherCAT implementation. The trait surface is the two clock lookups plus the load+push unit of work. `EtherCatNode` answers `now_clock` from the shared monotonic clock and `clock_freq` as `1e9`, and drives `load_and_push` through `UnixNativeConn` + a `CreditCounter` + a `SharedSlotPool` (the same producer path the serial node uses).

**Files:**
- Create: `rust/motion-bridge/src/motion_node.rs`
- Modify: `rust/motion-bridge/src/lib.rs`

- [ ] **Step 1: Confirm the imports you will need**

Before writing, grep for the exact paths in `bridge.rs` so the new file uses the same ones:
```bash
cd rust && grep -nE "use .*(SharedSlotPool|CreditCounter|DEFAULT_SLOT_ACQUIRE_TIMEOUT|DEFAULT_LOAD_CURVE_TIMEOUT)" motion-bridge/src/bridge.rs
```
Note the crate paths reported (e.g. `kalico_host_rt::slot_pool::SharedSlotPool`, `kalico_host_rt::credit::CreditCounter`). Use them verbatim in Step 2.

- [ ] **Step 2: Write `motion_node.rs` with the trait + `EtherCatNode` + a unit test**

`rust/motion-bridge/src/motion_node.rs` (replace the `use` paths with the ones grep reported in Step 1):
```rust
//! `MotionNode`: a clock-synced motion output the dispatch closure drives as a
//! peer. The trait surface is intentionally minimal — the per-MCU clock-base
//! arithmetic stays in the dispatch closure (`bridge.rs`); a node only answers
//! "what is now, in your clock domain?" (`now_clock`), "how fast does that
//! clock tick?" (`clock_freq`), and "load these curves and push this segment"
//! (`load_and_push`).
//!
//! `StepperMcuNode` (serial `KalicoHostIo`) and `EtherCatNode` (same-host
//! `UnixNativeConn`) are the two implementations. The EtherCAT node shares the
//! host's `CLOCK_MONOTONIC`, so its clock domain is nanoseconds with no
//! drift — `clock_freq() == 1e9`, `now_clock() == monotonic_ns()`.

use std::sync::Arc;
use std::time::Duration;

use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::producer::{self, DEFAULT_LOAD_CURVE_TIMEOUT};
use kalico_host_rt::slot_pool::{SharedSlotPool, DEFAULT_SLOT_ACQUIRE_TIMEOUT};
use kalico_host_rt::unix_native_conn::UnixNativeConn;

use crate::dispatch::McuPushPlan;
use crate::planner::DispatchError;

/// A clock-synced motion output peer.
pub trait MotionNode: Send + Sync {
    /// Current time in this node's clock domain (ticks).
    fn now_clock(&self) -> Result<u64, DispatchError>;

    /// This node's clock frequency in ticks/second.
    fn clock_freq(&self) -> f64;

    /// Load the plan's curves into the node's pool and push the segment.
    /// `plan.params.t_start` / `t_end` are already in this node's clock domain
    /// (the dispatch closure converted them using `clock_freq()` + the base it
    /// derived from `now_clock()`).
    fn load_and_push(&self, plan: McuPushPlan) -> Result<(), DispatchError>;
}

/// Read the host-wide monotonic clock in nanoseconds. Shared by every process
/// on the machine (unlike `std::time::Instant`, whose epoch is per-process),
/// so it is the common time base between this host and the EtherCAT endpoint.
fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: `ts` is a valid, writable timespec; CLOCK_MONOTONIC is always
    // available on Linux. The call only writes `ts`.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

/// EtherCAT RT endpoint as a motion node: same-host Unix socket, shared
/// monotonic clock.
pub struct EtherCatNode {
    conn: Arc<UnixNativeConn>,
    credit: Arc<CreditCounter>,
    slot_pool: Arc<SharedSlotPool>,
}

impl EtherCatNode {
    pub fn new(
        conn: Arc<UnixNativeConn>,
        credit: Arc<CreditCounter>,
        slot_pool: Arc<SharedSlotPool>,
    ) -> Self {
        Self { conn, credit, slot_pool }
    }
}

impl MotionNode for EtherCatNode {
    fn now_clock(&self) -> Result<u64, DispatchError> {
        Ok(monotonic_ns())
    }

    fn clock_freq(&self) -> f64 {
        1.0e9
    }

    fn load_and_push(&self, mut plan: McuPushPlan) -> Result<(), DispatchError> {
        load_and_push_via(
            self.conn.as_ref(),
            &self.credit,
            &self.slot_pool,
            &mut plan,
        )
    }
}

/// Shared load+push unit of work over any `NativeCall` peer. Allocates a slot
/// per curve, loads it, registers the slots against the segment id, then
/// pushes the segment. Releases all allocated slots on any failure. This is the
/// behaviour lifted verbatim from the dispatch closure's inner loop
/// (`bridge.rs:2465–2557`), parameterised over the connection.
pub(crate) fn load_and_push_via(
    io: &dyn kalico_host_rt::native_call::NativeCall,
    credit: &CreditCounter,
    slot_pool: &SharedSlotPool,
    plan: &mut McuPushPlan,
) -> Result<(), DispatchError> {
    let mut allocated_slots: Vec<u16> = Vec::with_capacity(plan.curves_to_load.len());
    for i in 0..plan.curves_to_load.len() {
        let axis_idx = plan.curves_to_load[i].0;
        let curve_params = plan.curves_to_load[i].1.clone();
        let (slot, _slot_gen) = match slot_pool.alloc_blocking(DEFAULT_SLOT_ACQUIRE_TIMEOUT) {
            Some(v) => v,
            None => {
                for s in &allocated_slots {
                    slot_pool.release(*s);
                }
                return Err(DispatchError::SlotPoolExhausted {
                    mcu_id: plan.mcu_id,
                    capacity: slot_pool.capacity(),
                    in_flight: slot_pool.in_flight_count(),
                });
            }
        };
        allocated_slots.push(slot);
        match producer::load_curve(io, slot, axis_idx as u8, &curve_params, DEFAULT_LOAD_CURVE_TIMEOUT) {
            Ok(handle) => plan.set_handle(axis_idx, handle),
            Err(e) => {
                for s in &allocated_slots {
                    slot_pool.release(*s);
                }
                return Err(DispatchError::LoadCurve {
                    mcu_id: plan.mcu_id,
                    slot,
                    seg_id: plan.params.id,
                    axis: axis_idx,
                    host_gen: _slot_gen,
                    detail: e.to_string(),
                });
            }
        }
    }
    for slot in &allocated_slots {
        slot_pool.register_segment(*slot, plan.params.id);
    }
    match producer::push_segment(io, credit, &plan.params) {
        Ok(_info) => Ok(()),
        Err(e) => {
            for s in &allocated_slots {
                slot_pool.release(*s);
            }
            Err(DispatchError::PushSegment {
                mcu_id: plan.mcu_id,
                detail: e.to_string(),
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ethercat_node_clock_domain_is_monotonic_ns() {
        // Build a throwaway node (conn never used by these getters).
        let (a, _b) = std::os::unix::net::UnixStream::pair().unwrap();
        let node = EtherCatNode::new(
            Arc::new(UnixNativeConn::from_stream(a)),
            Arc::new(CreditCounter::new(8)),
            Arc::new(SharedSlotPool::new(16)),
        );
        assert_eq!(node.clock_freq(), 1.0e9);
        let t0 = node.now_clock().unwrap();
        let t1 = node.now_clock().unwrap();
        assert!(t1 >= t0, "monotonic clock must not go backwards");
    }
}
```

- [ ] **Step 3: Add `libc` to `motion-bridge` deps (if not already present)**

Check `rust/motion-bridge/Cargo.toml` for `libc`. If absent, add under `[dependencies]`:
```toml
libc = "0.2"
```

- [ ] **Step 4: Register the module**

In `rust/motion-bridge/src/lib.rs`, add (with the other `mod` lines):
```rust
pub mod motion_node;
```

- [ ] **Step 5: Build + test**

Run: `cd rust && cargo test -p motion-bridge motion_node`
Expected: PASS. Fix-ups the compiler may force:
- The exact field set / variant names of `DispatchError::SlotPoolExhausted`, `LoadCurve`, `PushSegment` — copy them verbatim from `bridge.rs:2465–2557` (the inner loop you are lifting). If any field differs, match the real definition in `planner.rs`.
- `SharedSlotPool` method names (`alloc_blocking`, `release`, `register_segment`, `capacity`, `in_flight_count`) and `CreditCounter::new` — confirmed against `bridge.rs`; adjust if the crate path differs.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/motion_node.rs rust/motion-bridge/src/lib.rs rust/motion-bridge/Cargo.toml
git commit -m "motion-bridge: MotionNode trait + EtherCatNode (shared monotonic clock)"
```

---

## Task 4: `StepperMcuNode` — verbatim lift of the serial node behaviour

`StepperMcuNode` is the existing serial path expressed as a `MotionNode`. Its `now_clock` is the `compute_ack_clock` + block-wait loop; its `clock_freq` is the `clock_freqs` lookup; its `load_and_push` reuses `load_and_push_via` (Task 3) over the upgraded `KalicoHostIo`. **No behavioural change** — the bodies are copied from the closure, not redesigned.

**Files:**
- Modify: `rust/motion-bridge/src/motion_node.rs`

- [ ] **Step 1: Read the exact source you are lifting**

Read `bridge.rs:2230–2400` (the `freq` lookup + the `mcu_base_clock` block-wait loop — but NOT the `schedule_state` rebasing, which stays in the closure) and `bridge.rs:2465–2557` (the inner loop, already captured by `load_and_push_via`). Identify exactly which captured variables the `now_clock`/`clock_freq` logic touches:
- `clock_freqs: Arc<Mutex<HashMap<u32, f64>>>`
- `router_for_cb: Arc<Mutex<PassthroughRouter>>` (for `compute_ack_clock`)
- `fallback_counter: Arc<AtomicU64>` and `warned_mcus: Arc<Mutex<HashSet<u32>>>` (fallback-freq diagnostics)
- `mcu_h = mcu_handle_from_raw(plan.mcu_id)` and the raw `mcu_id`

- [ ] **Step 2: Add `StepperMcuNode` to `motion_node.rs`**

Append (adjust the `PassthroughRouter` / `mcu_handle_from_raw` import paths to match what `bridge.rs` uses — grep first: `cd rust && grep -nE "PassthroughRouter|mcu_handle_from_raw" motion-bridge/src/bridge.rs | head`):
```rust
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, Weak};
use std::time::Instant;

use kalico_host_rt::host_io::KalicoHostIo;
// Match the real paths from bridge.rs:
use crate::router::PassthroughRouter;
use crate::bridge::mcu_handle_from_raw; // or wherever it is defined

/// The serial stepper MCU as a motion node. Holds the same per-MCU state the
/// dispatch closure used to capture inline; `now_clock`/`clock_freq` reproduce
/// the closure's logic verbatim.
pub struct StepperMcuNode {
    pub mcu_id: u32,
    io: Weak<KalicoHostIo>,
    credit: Arc<CreditCounter>,
    slot_pool: Arc<SharedSlotPool>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    router: Arc<Mutex<PassthroughRouter>>,
    fallback_counter: Arc<AtomicU64>,
    warned_mcus: Arc<Mutex<HashSet<u32>>>,
}

impl StepperMcuNode {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mcu_id: u32,
        io: Weak<KalicoHostIo>,
        credit: Arc<CreditCounter>,
        slot_pool: Arc<SharedSlotPool>,
        clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
        router: Arc<Mutex<PassthroughRouter>>,
        fallback_counter: Arc<AtomicU64>,
        warned_mcus: Arc<Mutex<HashSet<u32>>>,
    ) -> Self {
        Self { mcu_id, io, credit, slot_pool, clock_freqs, router, fallback_counter, warned_mcus }
    }

    fn upgrade_io(&self) -> Result<Arc<KalicoHostIo>, DispatchError> {
        self.io.upgrade().ok_or(DispatchError::ConnectionDropped(self.mcu_id))
    }
}

impl MotionNode for StepperMcuNode {
    fn clock_freq(&self) -> f64 {
        // Verbatim from bridge.rs:2233 — clock_freqs lookup with 1 MHz
        // fallback + one-shot per-MCU warning.
        self.clock_freqs
            .lock()
            .unwrap()
            .get(&self.mcu_id)
            .copied()
            .filter(|f| *f > 0.0)
            .unwrap_or_else(|| {
                self.fallback_counter.fetch_add(1, Ordering::Relaxed);
                let first_for_mcu = {
                    let mut warned = self.warned_mcus.lock().unwrap_or_else(|p| p.into_inner());
                    warned.insert(self.mcu_id)
                };
                if first_for_mcu {
                    log::warn!(
                        "motion-bridge: MCU {} clock frequency not installed; using 1 MHz fallback for relative segment timing. SET_CLOCK_EST not yet wired by klippy?",
                        self.mcu_id
                    );
                }
                1_000_000.0
            })
    }

    fn now_clock(&self) -> Result<u64, DispatchError> {
        // Verbatim from bridge.rs:2260–2310 — block-wait for clock-sync to
        // publish a non-zero widened MCU clock (the "first jog after restart
        // doesn't move" fix). Returns the MCU's current widened clock.
        let mcu_h = mcu_handle_from_raw(self.mcu_id);
        let wait_start = Instant::now();
        let mut wait_iter: u32 = 0;
        loop {
            let n = {
                let r = self.router.lock().unwrap_or_else(|p| p.into_inner());
                r.compute_ack_clock(mcu_h)
                    .map_err(|e| DispatchError::ComputeAckClock(e.to_string()))?
            };
            if n > 0 {
                return Ok(n);
            }
            wait_iter += 1;
            if wait_start.elapsed() > Duration::from_secs(5) {
                return Err(DispatchError::ClockSyncTimeout {
                    mcu_id: self.mcu_id,
                    mcu_handle: mcu_h,
                });
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    fn load_and_push(&self, mut plan: McuPushPlan) -> Result<(), DispatchError> {
        let io = self.upgrade_io()?;
        load_and_push_via(io.as_ref(), &self.credit, &self.slot_pool, &mut plan)
    }
}
```

> The exact bodies of `clock_freq` and `now_clock` MUST be copied from the live `bridge.rs` lines, not from memory — preserve every diagnostic, the `lead_cycles` handling is NOT here (it belongs to the closure's schedule arithmetic), and `DispatchError` variant names/fields must match `planner.rs` exactly. If `compute_ack_clock`'s return type or `mcu_handle_from_raw`'s location differs, follow the compiler.

- [ ] **Step 3: Build (it will not be wired in yet — that's Task 5)**

Run: `cd rust && cargo build -p motion-bridge`
Expected: PASS, possibly with `dead_code` warnings for `StepperMcuNode::new` (it is constructed in Task 5). Tolerate the warning for this task; do not `#[allow(dead_code)]` it — Task 5 removes it.

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/motion_node.rs
git commit -m "motion-bridge: StepperMcuNode — serial node behaviour as a MotionNode"
```

---

## Task 5: Rewire the dispatch closure onto `MotionNode`

Replace the closure's inline per-MCU `freq`/`now_clock` lookups and inner load/push loop with `MotionNode` calls, **keeping the `schedule_state` base-clock arithmetic exactly where it is**. The closure builds a `HashMap<u32, Arc<dyn MotionNode>>` once (StepperMcuNode per existing MCU) and dispatches through it. This is the only behaviour-preserving-but-structural task; the regression gate is the existing test suite plus a clean build.

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

- [ ] **Step 1: Build the node map where `dispatch_ios` is built**

At `bridge.rs:2116`, `dispatch_ios.insert(...)` populates the per-MCU `(Weak<KalicoHostIo>, credit, slot_pool)` tuple. Immediately after that loop (after line 2119's `drop(self_pools)`), build a parallel node map. Add, using the captured `Arc`s already in scope (`clock_freqs`, `router_arc`, `fallback_counter`, `warned_mcus` — confirm these exact binding names near `bridge.rs:1956–1963` and create `warned_mcus`/`fallback_counter` clones if they are not already cloned for the closure):
```rust
use crate::motion_node::{MotionNode, StepperMcuNode};
use std::sync::Arc;

let mut nodes: HashMap<u32, Arc<dyn MotionNode>> = HashMap::new();
for (mcu_id, (io_weak, credit, slot_pool)) in &dispatch_ios {
    let node: Arc<dyn MotionNode> = Arc::new(StepperMcuNode::new(
        *mcu_id,
        io_weak.clone(),
        Arc::clone(credit),
        Arc::clone(slot_pool),
        Arc::clone(&clock_freqs),
        Arc::clone(&router_arc),
        Arc::clone(&fallback_counter),
        Arc::clone(&warned_mcus),
    ));
    nodes.insert(*mcu_id, node);
}
let nodes = Arc::new(nodes); // moved into the closure
```
> Confirm `router_arc` / `fallback_counter` / `warned_mcus` binding names against the real source (`grep -nE "let (router_arc|fallback_counter|warned_mcus|clock_freqs)" bridge.rs`). The closure currently captures `router_for_cb` (a clone of `router_arc`); reuse whatever clone the closure already owns. Build `nodes` from clones so both the closure and any other users keep their handles.

- [ ] **Step 2: In the closure, look up the node and replace the two inline lookups**

Inside the `for mut plan in mcu_plans` loop (`bridge.rs:2203`), after the `kalico_native_for_plans` skip check (keep it) and the `dispatch_ios.get`/`io_weak.upgrade()` block:

- Replace the `freq` lookup block (`bridge.rs:2233–2258`) with:
```rust
let node = match nodes.get(&plan.mcu_id) {
    Some(n) => Arc::clone(n),
    None => continue,
};
let freq = node.clock_freq();
```
- Replace the `now_clock` derivation inside the `mcu_base_clock` block (`bridge.rs:2260` the `let now_clock = loop { ... };`) with:
```rust
let now_clock = node.now_clock()?;
```
  Keep everything else in the `mcu_base_clock` block — the `lead_cycles_init`, the `schedule_state` lock, the fresh/drained/continuous branch logic, and the trailing `entry.1 = entry.1.max(t_end_clock)` update — **unchanged**. Those operate on `now_clock`/`freq` and stay in the closure.

- [ ] **Step 3: Replace the inner load/push loop with `node.load_and_push`**

The sub-plan loop fills `sub_plan.params.id`, `t_start`/`t_end`, and the kinematics/seed handling, then does slot-alloc + `producer::load_curve` + `dispatch_push_segment` (`bridge.rs:2465–2557`). Keep all the per-segment param-filling and the `homing.mark_dispatched_segment` / `next_seg_id` allocation (these are closure-owned, not node-owned). Replace only the slot-alloc + load + push body with:
```rust
node.load_and_push(sub_plan)?;
```
> `load_and_push` takes the `McuPushPlan` by value and fills handles internally (it owns the slot lifecycle). If the closure still needs `sub_plan` after this call (e.g. for logging `accepted_segment_id`), capture what you need from the `Result` instead — change `load_and_push` to return `producer::PushedSegmentInfo` if the logging is load-bearing; otherwise drop the post-push logging. Prefer dropping the verbose `[bridge-trace] push_segment ok` log to keep the seam clean.

- [ ] **Step 4: Delete the now-dead `dispatch_push_segment` helper if unused**

If nothing else calls `dispatch_push_segment` (`bridge.rs:372`), remove it. Run `cd rust && grep -n dispatch_push_segment motion-bridge/src/bridge.rs` to confirm before deleting.

- [ ] **Step 5: Build the workspace**

Run: `cd rust && cargo build --workspace`
Expected: PASS. The `StepperMcuNode::new` dead-code warning from Task 4 is now gone (it is constructed in Step 1).

- [ ] **Step 6: Run the full regression suite**

Run: `cd rust && cargo test --workspace`
Expected: PASS — same set of tests green as before this plan. This is the behaviour-preservation gate for the serial path. If a bridge/dispatch test exists and now fails, the lift changed behaviour — diff your `now_clock`/`clock_freq`/`load_and_push` bodies against the original closure lines and reconcile. Do not "fix" by changing the test.

- [ ] **Step 7: Clippy (catch accidental semantic drift + unused captures)**

Run: `cd rust && cargo clippy -p motion-bridge -p kalico-host-rt --all-targets`
Expected: no new warnings beyond pre-existing ones. Unused-variable warnings on former closure captures (`fallback_counter`, `warned_mcus`, etc.) mean a lookup did not get rewired through the node — investigate rather than silence.

- [ ] **Step 8: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "motion-bridge: dispatch via MotionNode; serial path unchanged behaviourally"
```

---

## Done criteria

- `cargo build --workspace` and `cargo test --workspace` are green.
- `NativeCall` + `UnixNativeConn` exist and are unit/integration-tested.
- `MotionNode` + `EtherCatNode` + `StepperMcuNode` exist; `EtherCatNode` is tested in isolation; the dispatch closure drives the serial path through `StepperMcuNode` with no behavioural change.
- `EtherCatNode` is intentionally **not** constructed in `init_planner` yet — that, plus the Klipper config / axis-mapping / passthrough / first live `G1` jog, is the deferred with-user integration step.

## Hardware / integration validation (deferred, with the user)

Not part of this plan's task list. When the Rust side lands:
1. The user installs the fork's klippy and writes a `printer.cfg` that validates (possibly with an STM32 over USB so the non-EtherCAT axes have a real MCU — this also regression-tests the `StepperMcuNode` extraction on real serial hardware).
2. Wire `EtherCatNode` into `init_planner`'s node map behind a config surface; map one axis to it as passthrough.
3. Start the Plan-1 `kalico-ethercat-rt` endpoint; `G1` on the EtherCAT-owned axis → planner → `EtherCatNode` → `UnixNativeConn` → endpoint → servo moves.
