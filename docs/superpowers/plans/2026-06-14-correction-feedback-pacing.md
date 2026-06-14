# Feedback-Paced Correction Stream Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Pace the correction-stream refill on real MCU drain feedback (a separate `DrainSync` fed by the MCU heartbeat's correction `retired` counts) instead of a host wall-clock guess, eliminating the `-309 KALICO_ERR_RING_FULL` that aborts the motors-sync buzz.

**Architecture:** Reuse the feedback primitive (`DrainSync` + a new `room()`-aware wait), not the main-ring `pump` thread. The MCU heartbeat gains the correction ring's per-axis `retired`; the host feeds it into a dedicated correction `DrainSync` and refills the correction stream only into known-free slots. The correction ring's MCU evaluator, `correction_active()` gate, single-motor stepping, and `PushCorrectionPieces` wire are untouched. Depth grows 16→64.

**Tech Stack:** Rust (runtime, host bridge, protocol, FFI), C (MCU dispatch + tick), PyO3, `cargo nextest`.

---

## Files

| File | Change |
|------|--------|
| `rust/runtime/src/stepping_state.rs` | `CORRECTION_RING_DEPTH` 16 → 64 |
| `rust/runtime/src/engine.rs` | add `correction_retired_counts()` accessor |
| `rust/kalico-c-api/src/runtime_ffi.rs` | add `kalico_runtime_get_correction_retired` FFI |
| `rust/kalico-c-api/include/kalico_runtime.h` | declare the new FFI |
| `rust/kalico-protocol/src/messages.rs` | `StatusHeartbeat` gains `correction_retired_counts` |
| `src/kalico_dispatch.c` | `send_status_heartbeat` appends correction counts |
| `src/runtime_tick.c` | cadence also triggers on correction `retired` change |
| `rust/motion-bridge/src/drain.rs` | add `wait_room()` |
| `rust/motion-bridge/src/bridge.rs` | correction `DrainSync` field + feed in heartbeat cb + feedback-paced `stream_correction_entries` |

Run Rust tests with `cargo nextest run` from `rust/` (per CLAUDE.md). C-MCU tasks (5) verify in kalico-sim / bench, not host-unit — flagged.

---

## Task 1: Bump correction ring depth 16 → 64

**Files:**
- Modify: `rust/runtime/src/stepping_state.rs:14`
- Verify: `rust/motion-bridge/src/bridge.rs:590-625` (parameterized asserts)

Context: `CORRECTION_RING_DEPTH` is a single Rust const used by both the MCU runtime and the host bridge (`runtime::stepping_state::CORRECTION_RING_DEPTH`), so host and MCU stay in lockstep on rebuild. The `bridge.rs` reserve asserts reference the const symbolically (`(1984 - 2 * CORRECTION_RING_DEPTH as u32) / 2`), so they recompute and still pass. The main ring shrinks slightly (e.g. 2-axis: 976 → 928 slots/axis) — still ample; no Kconfig change needed.

- [ ] **Step 1: Change the constant**

`rust/runtime/src/stepping_state.rs:14`:
```rust
pub const CORRECTION_RING_DEPTH: usize = 64;
```

- [ ] **Step 2: Confirm the parameterized reserve asserts still pass**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(reserves_correction)'`
Expected: PASS (the asserts use the symbol, so both sides recompute to 64).

- [ ] **Step 3: Confirm the per-message assert still holds**

`rust/motion-bridge/src/correction.rs:144` asserts `MAX_CORRECTION_PIECES_PER_MSG < CORRECTION_RING_DEPTH`. With `MAX_CORRECTION_PIECES_PER_MSG = 15 < 64` it holds. Run: `cd rust && cargo nextest run -p motion-bridge` — expected PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/stepping_state.rs
git commit -m "feat(correction): grow correction ring depth 16->64"
```

---

## Task 2: Runtime accessor + FFI for correction `retired`

**Files:**
- Modify: `rust/runtime/src/engine.rs` (add `correction_retired_counts()` next to `retired_counts()` at line 495)
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (add FFI after `kalico_runtime_get_heartbeat`, ~line 1192)
- Modify: `rust/kalico-c-api/include/kalico_runtime.h` (declare it)
- Test: `rust/runtime/src/engine.rs` tests module (or the existing engine test file)

Context: `retired_counts()` (engine.rs:495) returns the **main** ring's per-axis `retired` via `axis.ring.retired_count()`. The correction ring (`axis.correction_ring`, a `RingDescriptor`) has the same `retired_count()` accessor. The FFI `kalico_runtime_get_correction_retired` parallels `kalico_runtime_get_heartbeat` (runtime_ffi.rs:1154) but returns only the correction counts.

- [ ] **Step 1: Write the failing test**

Add to the engine tests (find the `#[cfg(test)] mod tests` in `rust/runtime/src/engine.rs` or its sibling test file; if engine tests live in `rust/runtime/src/engine/tests.rs`, add there):
```rust
#[test]
fn correction_retired_counts_reads_correction_ring() {
    let mut eng = Engine::new_for_test(); // existing test constructor
    // configure_axis allocates main + correction rings for axis 0
    eng.configure_axis_for_test(0); // existing helper; if named differently, use it
    // a freshly configured correction ring has retired == 0
    let counts = eng.correction_retired_counts();
    assert_eq!(counts[0], 0);
}
```
(If the engine test harness uses different constructor/helper names, mirror what `retired_counts` tests use — grep `fn retired_counts` test usages.)

- [ ] **Step 2: Run it, verify it fails**

Run: `cd rust && cargo nextest run -p runtime -E 'test(correction_retired_counts_reads_correction_ring)'`
Expected: FAIL — `correction_retired_counts` not defined.

- [ ] **Step 3: Add the accessor**

In `rust/runtime/src/engine.rs`, immediately after `retired_counts()` (ends ~line 503):
```rust
    pub fn correction_retired_counts(&self) -> [u32; MAX_AXES] {
        let mut out = [0u32; MAX_AXES];
        for (slot, entry) in out.iter_mut().zip(self.stepping_axes.iter()) {
            if let Some(axis) = entry {
                *slot = axis.correction_ring.retired_count();
            }
        }
        out
    }
```

- [ ] **Step 4: Run it, verify it passes**

Run: `cd rust && cargo nextest run -p runtime -E 'test(correction_retired_counts_reads_correction_ring)'`
Expected: PASS.

- [ ] **Step 5: Add the FFI**

In `rust/kalico-c-api/src/runtime_ffi.rs`, after `kalico_runtime_get_heartbeat` (closes ~line 1192), add (mirrors its null-checks / context access, but writes only correction counts):
```rust
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_correction_retired(
        rt: *mut KalicoRuntime,
        out_retired: *mut u32,
        max_axes: usize,
    ) -> i32 {
        if rt.is_null() || out_retired.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let engine = &(*isr_ptr).engine;
            let num_axes = engine.num_axes as usize;
            let counts = engine.correction_retired_counts();
            let n_write = num_axes.min(max_axes);
            for i in 0..n_write {
                out_retired.add(i).write(counts[i]);
            }
            #[allow(clippy::cast_possible_truncation)]
            let result = n_write as i32;
            result
        }
    }
```

- [ ] **Step 6: Declare it in the C header**

In `rust/kalico-c-api/include/kalico_runtime.h`, next to the `kalico_runtime_get_heartbeat` declaration, add:
```c
int32_t kalico_runtime_get_correction_retired(struct KalicoRuntime *rt,
                                              uint32_t *out_retired,
                                              size_t max_axes);
```
(Match the exact `struct`/typedef style used by the neighboring `kalico_runtime_get_heartbeat` declaration in that header.)

- [ ] **Step 7: Build the FFI crate + commit**

Run: `cd rust && cargo nextest run -p runtime -p kalico-c-api && cargo build -p kalico-c-api`
Expected: PASS / clean build.
```bash
git add rust/runtime/src/engine.rs rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/kalico_runtime.h
git commit -m "feat(correction): expose correction ring retired via runtime accessor + FFI"
```

---

## Task 3: `StatusHeartbeat` carries correction `retired`

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs:730-779`
- Test: same file's tests (grep `StatusHeartbeat` in `rust/kalico-protocol/` tests, or add a round-trip test)

Context: `StatusHeartbeat` is length-prefixed (`num_axes` u8 + counts + `ff_saturation_count`). Append a second `correction_retired_counts: Vec<u32>` with its own `u8` prefix, **after** `ff_saturation_count`. Decode is made tolerant: if no bytes remain after `ff_saturation_count` (an older MCU), `correction_retired_counts` defaults empty — so partial deploys don't panic.

- [ ] **Step 1: Write the failing round-trip test**

Add to `rust/kalico-protocol/src/messages.rs` tests (or its test module):
```rust
#[test]
fn status_heartbeat_round_trips_correction_counts() {
    let hb = StatusHeartbeat {
        engine_state: 2,
        fault_code: 0,
        retired_counts: vec![5, 9],
        ff_saturation_count: 3,
        correction_retired_counts: vec![7, 0],
    };
    let mut buf = Vec::new();
    hb.encode(&mut buf);
    let mut c = Cursor::new(&buf);
    let got = StatusHeartbeat::decode_from(&mut c).unwrap();
    assert_eq!(got, hb);
}

#[test]
fn status_heartbeat_decode_tolerates_missing_correction_tail() {
    // bytes from the OLD layout (no correction tail)
    let old = StatusHeartbeat {
        engine_state: 1, fault_code: 0, retired_counts: vec![4],
        ff_saturation_count: 0, correction_retired_counts: vec![],
    };
    let mut buf = Vec::new();
    // encode old layout manually: no correction tail
    put_u8(&mut buf, old.engine_state);
    put_u16(&mut buf, old.fault_code);
    put_u8(&mut buf, old.retired_counts.len() as u8);
    for &v in &old.retired_counts { put_u32(&mut buf, v); }
    put_u32(&mut buf, old.ff_saturation_count);
    let mut c = Cursor::new(&buf);
    let got = StatusHeartbeat::decode_from(&mut c).unwrap();
    assert_eq!(got.correction_retired_counts, Vec::<u32>::new());
}
```
(`StatusHeartbeat` must `#[derive(PartialEq)]` for `assert_eq!` — add it if absent.)

- [ ] **Step 2: Run, verify fail**

Run: `cd rust && cargo nextest run -p kalico-protocol -E 'test(status_heartbeat)'`
Expected: FAIL — field `correction_retired_counts` does not exist.

- [ ] **Step 3: Add the field + encode + tolerant decode**

In `rust/kalico-protocol/src/messages.rs`, the struct:
```rust
pub struct StatusHeartbeat {
    pub engine_state: u8,
    pub fault_code: u16,
    pub retired_counts: Vec<u32>,
    pub ff_saturation_count: u32,
    pub correction_retired_counts: Vec<u32>,
}
```
Encode — append after `ff_saturation_count`:
```rust
        put_u32(out, self.ff_saturation_count);
        let num_corr = self.correction_retired_counts.len() as u8;
        put_u8(out, num_corr);
        for &count in &self.correction_retired_counts {
            put_u32(out, count);
        }
```
Decode — after reading `ff_saturation_count`, tolerantly read the tail:
```rust
        let ff_saturation_count = get_u32(c)?;
        let correction_retired_counts = if c.remaining() == 0 {
            Vec::new()
        } else {
            let num_corr = get_u8(c)?;
            let need = (num_corr as usize).checked_mul(4).ok_or(
                DecodeError::ArrayLengthExceedsBuffer {
                    claimed: u32::from(num_corr), available: c.remaining(),
                })?;
            if need > c.remaining() {
                return Err(DecodeError::ArrayLengthExceedsBuffer {
                    claimed: u32::from(num_corr), available: c.remaining(),
                });
            }
            let mut v = Vec::with_capacity(num_corr as usize);
            for _ in 0..num_corr { v.push(get_u32(c)?); }
            v
        };
        Ok(Self {
            engine_state, fault_code, retired_counts,
            ff_saturation_count, correction_retired_counts,
        })
```
Also ensure `#[derive(... PartialEq ...)]` on the struct.

- [ ] **Step 4: Fix the other constructors of `StatusHeartbeat`**

Run: `cd rust && cargo build -p kalico-protocol -p motion-bridge 2>&1 | grep -A2 "missing field"` — any site that builds `StatusHeartbeat` needs `correction_retired_counts: vec![]` added. Fix each (likely test fixtures in `motion-bridge`).

- [ ] **Step 5: Run, verify pass**

Run: `cd rust && cargo nextest run -p kalico-protocol -E 'test(status_heartbeat)'`
Expected: PASS (both tests).

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-protocol/src/messages.rs
git commit -m "feat(protocol): StatusHeartbeat carries correction retired counts"
```

---

## Task 4: MCU C — emit correction counts + cadence on correction drain

**Files:**
- Modify: `src/kalico_dispatch.c` (`send_status_heartbeat`, ~line 459-499)
- Modify: `src/runtime_tick.c` (the cadence block, ~line 252-276)

Context (C, MCU-side — verified in kalico-sim / bench, **not** host-unit). `send_status_heartbeat` currently fetches main `retired` via `kalico_runtime_get_heartbeat` and writes `[st, fc, n, counts..., ff_sat]`. Append `[num_corr, corr_counts...]` after `ff_sat`, matching the Rust encode order from Task 3. The cadence block (`runtime_tick.c`) sets `pending_advance` when any main `retired` changes; add the same for correction `retired` so a draining buzz pushes fast heartbeats — and since corrections only change `retired` when active, prints are unaffected (the gate is automatic).

- [ ] **Step 1: Append correction counts in `send_status_heartbeat`**

In `src/kalico_dispatch.c`, after the `ff_saturation_count` bytes are written (just before `kalico_transport_send_frame`):
```c
    uint32_t corr[8];
    int32_t nc = kalico_runtime_get_correction_retired(runtime_handle, corr, 8);
    if (nc < 0)
        nc = 0;
    payload[off++] = (uint8_t)nc;
    for (int i = 0; i < nc; i++) {
        uint32_t v = corr[i];
        payload[off++] = (uint8_t)(v & 0xFF);
        payload[off++] = (uint8_t)((v >> 8) & 0xFF);
        payload[off++] = (uint8_t)((v >> 16) & 0xFF);
        payload[off++] = (uint8_t)((v >> 24) & 0xFF);
    }
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL, payload, (uint16_t)off);
```
(Confirm `KALICO_TX_BUF_SIZE` accommodates the extra `1 + 8*4` bytes; with 8 axes max it grows the payload by ≤33 bytes — the buffer is already sized for the main counts and headroom. If a static-assert on payload size exists, update it.)

- [ ] **Step 2: Trigger cadence on correction `retired` change**

In `src/runtime_tick.c`, inside the cadence block (alongside `last_retired_seen` for the main ring), add a parallel snapshot + fetch for correction retired:
```c
        static uint32_t last_corr_retired_seen[KALICO_FAST_STATUS_MAX_AXES];
        uint32_t corr_retired[KALICO_FAST_STATUS_MAX_AXES];
        int32_t ncorr = kalico_runtime_get_correction_retired(
            runtime_handle, corr_retired, KALICO_FAST_STATUS_MAX_AXES);
        if (ncorr > 0) {
            for (int32_t i = 0; i < ncorr; i++) {
                if (corr_retired[i] != last_corr_retired_seen[i]) {
                    pending_advance = 1;
                    last_corr_retired_seen[i] = corr_retired[i];
                }
            }
        }
```
Place this immediately before the existing `uint32_t elapsed = cur_time - last_status_emit_time;` line so the combined `pending_advance` (main OR correction) drives the same emit gate. (No new emit path — it reuses the existing `send_status_heartbeat()` call.)

- [ ] **Step 3: Build the firmware to confirm it compiles**

The C builds as part of the MCU firmware. Compile-check via the H7 target build:
Run: `./scripts/ci.sh rust-mcu-h7` (compiles the staticlib for the H7 target; if the C dispatch is built only by the full firmware make, instead confirm via a local `make` of the H7 config). Expected: clean compile, no missing-symbol for `kalico_runtime_get_correction_retired` (it's exported by Task 2).

- [ ] **Step 4: Commit**

```bash
git add src/kalico_dispatch.c src/runtime_tick.c
git commit -m "feat(mcu): emit correction ring retired in heartbeat + pace cadence on its drain"
```

---

## Task 5: `DrainSync::wait_room` (host feedback primitive)

**Files:**
- Modify: `rust/motion-bridge/src/drain.rs`
- Test: `rust/motion-bridge/src/drain.rs` tests (or `drain/tests.rs`)

Context: `DrainSync` tracks `sent`/`retired`/`baseline` per `(mcu, axis)` with a condvar. It has `wait_drained` (block until fully drained) but no "block until there's room for N more." Add `wait_room`: occupancy is `sent - retired`; room is `ring_depth - occupancy`; block on the condvar until `room >= needed` or timeout.

- [ ] **Step 1: Write the failing test**

In `rust/motion-bridge/src/drain.rs` (or its tests module):
```rust
#[test]
fn wait_room_unblocks_when_retired_advances() {
    let d = std::sync::Arc::new(DrainSync::new());
    d.add_sent(0, 0, 16); // ring of 16, fully occupied
    assert_eq!(d.room(0, 0, 16), 0);
    let d2 = d.clone();
    let h = std::thread::spawn(move || {
        d2.wait_room(0, 0, 16, 4, std::time::Duration::from_secs(2)).unwrap();
    });
    std::thread::sleep(std::time::Duration::from_millis(20));
    d.set_retired(0, 0, 5); // 5 drained -> room 5 >= 4
    h.join().unwrap();
}
```

- [ ] **Step 2: Run, verify fail**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(wait_room_unblocks)'`
Expected: FAIL — `room`/`wait_room` not defined.

- [ ] **Step 3: Implement `room` + `wait_room`**

Add to `impl DrainSync` (uses the existing `counts` mutex `c.sent`/`c.retired` and `self.cv`):
```rust
    pub fn room(&self, mcu: u32, axis: u8, ring_depth: u32) -> u32 {
        let c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        let sent = c.sent.get(&(mcu, axis)).copied().unwrap_or(0);
        let retired = c.retired.get(&(mcu, axis)).copied().unwrap_or(0);
        let occ = sent.wrapping_sub(retired);
        ring_depth.saturating_sub(occ)
    }

    pub fn wait_room(
        &self,
        mcu: u32,
        axis: u8,
        ring_depth: u32,
        needed: u32,
        timeout: Duration,
    ) -> Result<(), String> {
        let deadline = Instant::now() + timeout;
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        loop {
            let sent = c.sent.get(&(mcu, axis)).copied().unwrap_or(0);
            let retired = c.retired.get(&(mcu, axis)).copied().unwrap_or(0);
            let room = ring_depth.saturating_sub(sent.wrapping_sub(retired));
            if room >= needed {
                return Ok(());
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(format!(
                    "wait_room timeout mcu{mcu} axis{axis}: room {room} < needed {needed}"
                ));
            }
            let (guard, res) = self
                .cv
                .wait_timeout(c, deadline - now)
                .unwrap_or_else(|p| p.into_inner());
            c = guard;
            if res.timed_out() {
                // loop re-checks and will return the timeout error above
            }
        }
    }
```
Ensure `set_retired` (and `add_sent`) call `self.cv.notify_all()` so `wait_room` wakes — check the existing `set_retired`; if it doesn't notify, add `self.cv.notify_all();` at its end (mirroring whatever `wait_drained` relies on).

- [ ] **Step 4: Run, verify pass**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(wait_room)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/drain.rs
git commit -m "feat(drain): add room()/wait_room() for feedback-paced refill"
```

---

## Task 6: Host — correction `DrainSync` instance, fed by the heartbeat

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (field at ~503, construct at ~762, heartbeat cb at ~2647-2650 and the second site ~2744-2748)

Context: the bridge already holds `drain: Arc<DrainSync>` (field 503, built 762) for the main ring; the heartbeat callback feeds it `set_retired` per main axis (2649). Add a parallel `correction_drain: Arc<DrainSync>` and feed it from `hb.correction_retired_counts`.

- [ ] **Step 1: Add the field**

`rust/motion-bridge/src/bridge.rs:503`, next to `drain`:
```rust
    drain: std::sync::Arc<crate::drain::DrainSync>,
    correction_drain: std::sync::Arc<crate::drain::DrainSync>,
```

- [ ] **Step 2: Construct it**

At the struct literal (~762), next to `drain: ...::new()`:
```rust
            drain: std::sync::Arc::new(crate::drain::DrainSync::new()),
            correction_drain: std::sync::Arc::new(crate::drain::DrainSync::new()),
```

- [ ] **Step 3: Capture + feed it in the heartbeat callback**

Where `let drain_hb = self.drain.clone();` is set up before the callback (~2567), add:
```rust
            let drain_hb = self.drain.clone();
            let corr_drain_hb = self.correction_drain.clone();
```
Inside the callback, after the existing main-ring feed loop (`drain_hb.set_retired(...)` at ~2649), add:
```rust
                        for (axis, &r) in hb.correction_retired_counts.iter().enumerate() {
                            corr_drain_hb.set_retired(mcu_id, axis as u8, r);
                        }
```
Apply the same addition at the **second** heartbeat-feeding site (~2744-2748) if it likewise feeds `drain_hb` — grep `drain_hb.set_retired` to find both and add the `corr_drain_hb` loop next to each. (Capture `corr_drain_hb` in whichever closure scope each lives in.)

- [ ] **Step 4: Build (no behavior test yet — exercised in Task 7)**

Run: `cd rust && cargo build -p motion-bridge`
Expected: clean (unused `correction_drain` is consumed in Task 7; if clippy flags it as unused now, proceed — Task 7 uses it in the same branch before any gate).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(bridge): correction DrainSync instance fed by heartbeat"
```

---

## Task 7: Host — feedback-paced `stream_correction_entries`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:3747-3852` (`stream_correction_entries`)
- Test: `rust/motion-bridge/src/bridge.rs` tests / `bridge/tests.rs` (pacing via fake drain)

Context: replace the wall-clock refill (the `chunk_release_times` / `piece_end_host` / `std::thread::sleep` block, ~3776-3848) with feedback pacing on `self.correction_drain`: reset it at stream start, and before sending each chunk, `wait_room(chunk.len())`, then `add_sent(chunk.len())`. The MCU's heartbeat advances `retired` → frees room → `wait_room` unblocks. The host never sends into a full ring, so `commit_correction` can't return `-309`.

- [ ] **Step 1: Write the failing pacing test**

Add a test that drives the pacing logic with a fake-fed `DrainSync` (extract the per-chunk send into a helper if needed for testability, or test through a seam). Minimal behavioral test of the primitive used:
```rust
#[test]
fn correction_refill_waits_for_room_then_sends() {
    use crate::drain::DrainSync;
    let d = std::sync::Arc::new(DrainSync::new());
    let depth = runtime::stepping_state::CORRECTION_RING_DEPTH as u32;
    // simulate two chunks of 15 into a depth-64 ring: both fit, no wait
    d.reset();
    assert!(d.room(0, 0, depth) >= 15);
    d.add_sent(0, 0, 15);
    assert!(d.room(0, 0, depth) >= 15); // still room for the next
    d.add_sent(0, 0, 15);
}
```
(This asserts the `room`/`add_sent` accounting the refill loop relies on; the full send path is covered by kalico-sim in Task 8.)

- [ ] **Step 2: Run, verify it compiles/fails appropriately**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(correction_refill_waits)'`
Expected: PASS once `room`/`reset` exist (they do from Task 5) — this guards the accounting contract. (If you instead extract a testable refill helper, make that the failing-first target.)

- [ ] **Step 3: Replace the wall-clock loop with feedback pacing**

In `stream_correction_entries`, delete the `end_host` / `total_duration`-for-release / `chunk_release_times` / `releases` computation and the `py.detach` loop's `if let Some(rel) = release { ... sleep ... }` wait. Keep: the `start_secs`/`entries` clock projection (the stream still starts at `now + CORRECTION_LEAD_SECS`), the `msgs` chunking, the `io` handle. Replace the per-chunk loop body with:
```rust
        const DRAIN_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);
        let ring_depth = runtime::stepping_state::CORRECTION_RING_DEPTH as u32;
        let drain = self.correction_drain.clone();
        drain.reset();
        let mcu_raw = mcu_handle;

        py.detach(|| -> PyResult<()> {
            for msg in &msgs {
                let n = msg.piece_count as u32;
                drain
                    .wait_room(mcu_raw, axis_idx, ring_depth, n, DRAIN_TIMEOUT)
                    .map_err(|e| PyRuntimeError::new_err(format!(
                        "stream_correction: {e}"
                    )))?;
                let mut body = Vec::with_capacity(9 + msg.pieces_bytes.len());
                kalico_protocol::codec::Encode::encode(msg, &mut body);
                let (_kind, resp) = io
                    .kalico_call_on_channel(
                        kalico_protocol::KALICO_CHANNEL_CONTROL,
                        kalico_protocol::MessageKind::PushCorrectionPieces,
                        body,
                        std::time::Duration::from_secs(2),
                    )
                    .map_err(|e| PyRuntimeError::new_err(format!("stream_correction send: {e:?}")))?;
                use kalico_protocol::codec::Decode as _;
                let r = kalico_protocol::messages::PushCorrectionPiecesResponse::decode(&resp)
                    .map_err(|e| PyRuntimeError::new_err(format!("stream_correction decode: {e:?}")))?;
                if r.result != 0 {
                    return Err(PyRuntimeError::new_err(format!(
                        "stream_correction rejected by MCU: error {}",
                        r.result
                    )));
                }
                drain.add_sent(mcu_raw, axis_idx, n);
            }
            Ok(())
        })?;
        Ok(CORRECTION_LEAD_SECS + total_duration)
```
Keep `total_duration` (computed from the pieces for the return value — retain its computation from `start_secs`/pieces; only the *release-time* use of `end_host` is removed). Remove now-dead `chunk_release_times` import/usage; if `chunk_release_times`/`piece_end_host_times` in `correction.rs` become unused, leave them (they have their own unit tests) or remove if the implementer confirms no other caller (grep first).

- [ ] **Step 4: Run pacing + crate tests**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: PASS.

- [ ] **Step 5: Clippy + fmt**

Run: `cd rust && cargo clippy -p motion-bridge -- -D warnings && cargo fmt --check`
Expected: clean.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs rust/motion-bridge/src/correction.rs
git commit -m "feat(bridge): feedback-pace correction stream on DrainSync (kills -309)"
```

---

## Task 8: Integration — kalico-sim long buzz, no RING_FULL

**Files:** none (sim run)

Context: verify end-to-end that a buzz-length correction sequence streams to completion without `RING_FULL`. Reuse the multi-Z correction e2e config path (the kalico-sim runner already exercises `MOTOR_ADJUST`/correction streaming). This task is integration; no host-unit.

- [ ] **Step 1: Build the sim image for this branch**

Run: `bash tools/kalico-sim/run.sh --branch motors-sync` (or `docker build -t kalico-sim -f tools/kalico-sim/Dockerfile .`).
Expected: builds (Rust staticlib + motion-bridge + firmware).

- [ ] **Step 2: Run a correction sequence longer than the ring**

Drive a correction sequence of ≥80 pieces (longer than depth 64) at the bridge — via the existing kalico-sim correction/`MOTOR_ADJUST` scenario, increasing the delta/segment count so the piece count exceeds 64 and forces refill. Assert in the sim output: `motion.correction_start` and `motion.correction_drained` fire, and **no** `motion.ring_full` / `-309` appears.
Expected: stream completes; no `RING_FULL`.

- [ ] **Step 3: Commit any sim-scenario tweak**

```bash
git add tools/kalico-sim/
git commit -m "test(sim): correction sequence exceeding ring depth streams without RING_FULL"
```

---

## Task 9: Bench validation (DEFERRED — user-driven, do NOT touch the bench)

Not part of automated execution. After the above lands and `./scripts/ci.sh quick` is green, the user deploys (`flash-trident.sh motors-sync all`) and runs `SYNC_MOTORS`; success = the buzz runs gapless (no `-309`), the partner motor stays put, and the sync converges. Do not flash or run G-code without the user's explicit go-ahead.

---

## Self-Review

**Spec coverage:**
- §1 Host pacing reuse `DrainSync` (not full pump) → Tasks 5 (`wait_room`), 6 (instance), 7 (feedback loop). ✓
- §2 Drain feedback on heartbeat, gated → Task 2 (accessor/FFI), 3 (protocol), 4 (C emit + cadence-on-correction-drain, gate automatic via key-on-change). ✓
- §3 Depth 16→64 → Task 1. ✓
- §4 Non-regression: main ring untouched (correction uses a *separate* `DrainSync`, separate FFI, appended protocol field; the cadence add is OR-ed in and only fires when correction `retired` moves). The existing main-ring tests in Tasks 1/3/7 runs guard it. ✓
- §5 Unchanged eval/wire/PieceEntry → nothing in any task touches `tick_correction`, `correction_active()`, single-motor stepping, `PushCorrectionPieces`, or `PieceEntry`. ✓
- Testing matrix → unit (Tasks 2,3,5,7), runtime (2), sim (8), bench (9). ✓

**Placeholder scan:** no TBD/TODO; each code step shows the change. Two spots say "grep to find both sites / confirm helper name" — these are *find-the-exact-location* instructions with the surrounding code shown, not missing content.

**Type/signature consistency:** `correction_retired_counts()` returns `[u32; MAX_AXES]` (Task 2) consumed as `Vec<u32>` on the wire (Task 3) and fed per-axis via `set_retired` (Task 6). `wait_room(mcu, axis, ring_depth, needed, timeout)` / `room(mcu, axis, ring_depth)` defined in Task 5, used in Task 7. `correction_drain` field name consistent Tasks 6→7. FFI `kalico_runtime_get_correction_retired(rt, out_retired, max_axes)` consistent Task 2 (Rust + header) → Task 4 (C callers).
