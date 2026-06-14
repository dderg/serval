# Correction Stream Sequences Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let one motor run a contiguous multi-segment correction (a buzz / nudge sweep) as a single streamed correction stream, with the host pacing refill against the deterministic piece schedule so long sequences are not capped by the MCU correction ring.

**Architecture:** Entirely host-side in `rust/motion-bridge`. Generalize the single-trapezoid correction builder to a gapless multi-segment one (`plan_correction_sequence`), add two pure scheduling helpers (per-piece end host-times, per-chunk release times), and rework `adjust_motor` to stream chunks paced by those release times instead of bursting them all at once. The MCU already accepts incremental refill (`commit_correction` advances `new_head`, wraps slots mod `CORRECTION_RING_DEPTH`, rejects overcommit with `KALICO_ERR_RING_FULL`) — that rejection is the fail-loud net. A new PyO3 method `submit_correction_sequence` exposes the sequence path to Python; `adjust_motor` becomes its one-segment caller.

**Tech Stack:** Rust (`rust/motion-bridge`, PyO3), the existing `PushCorrectionPieces` wire contract, Python wrapper in `klippy/motion_bridge.py`. Tests via `cargo nextest`.

---

## File Structure

- `rust/motion-bridge/src/correction.rs` — add `push_segment` (refactor of `push_quadratic`/`push_linear` callers), `plan_correction_sequence`, `piece_end_host_times`, `chunk_release_times`. `plan_correction_profile` is rewritten to call `push_segment` (output unchanged).
- `rust/motion-bridge/src/correction/tests.rs` — unit tests for the new builder and scheduling helpers.
- `rust/motion-bridge/src/bridge.rs` — add `stream_correction_entries` helper; rewrite `adjust_motor` to use it; add `submit_correction_sequence` PyO3 method.
- `klippy/motion_bridge.py` — add `submit_correction_sequence` wrapper next to `adjust_motor`.

All correction-shaping logic stays in `correction.rs` (one responsibility: turn a move request into paced wire messages); `bridge.rs` only does transport + clock.

---

## Task 1: Refactor the segment builder (no behavior change)

Extract per-segment trapezoid construction so a segment can be placed at an absolute start position. `plan_correction_profile`'s output must stay byte-for-byte identical.

**Files:**
- Modify: `rust/motion-bridge/src/correction.rs:24-81` (`plan_correction_profile`, `push_quadratic`, `push_linear`)
- Test: `rust/motion-bridge/src/correction/tests.rs`

- [ ] **Step 1: Write a characterization test pinning current output**

Add to `rust/motion-bridge/src/correction/tests.rs`:

```rust
#[test]
fn single_segment_profile_has_zero_velocity_ends() {
    // Velocity ∝ (b1-b0) at start, (b3-b2) at end of a cubic-Bézier piece.
    let pieces = plan_correction_profile(3.0, 5.0, 100.0).unwrap();
    let first = pieces.first().unwrap();
    let last = pieces.last().unwrap();
    assert!((first.coeffs[1] - first.coeffs[0]).abs() < 1e-9, "starts at rest");
    assert!((last.coeffs[3] - last.coeffs[2]).abs() < 1e-9, "ends at rest");
}

#[test]
fn pieces_are_position_contiguous() {
    // Each piece's end position equals the next piece's start position (C0).
    let pieces = plan_correction_profile(7.0, 4.0, 80.0).unwrap();
    for w in pieces.windows(2) {
        assert!((w[0].coeffs[3] - w[1].coeffs[0]).abs() < 1e-9, "no position gap");
    }
}
```

- [ ] **Step 2: Run the tests to verify they pass against current code**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(single_segment_profile_has_zero_velocity_ends) or test(pieces_are_position_contiguous)'`
Expected: PASS (these characterize existing behavior).

- [ ] **Step 3: Replace `push_quadratic`/`push_linear` with base-offset `push_segment`**

In `rust/motion-bridge/src/correction.rs`, replace the body of `plan_correction_profile` (lines 24-51) and the two helpers (lines 53-81) with:

```rust
pub fn plan_correction_profile(
    delta_mm: f64,
    speed: f64,
    accel: f64,
) -> Result<Vec<ProfilePiece>, String> {
    profile_duration(delta_mm, speed, accel)?;
    let mut out = Vec::new();
    push_segment(&mut out, 0.0, delta_mm, speed, accel);
    Ok(subdivide_all(out))
}

fn subdivide_all(pieces: Vec<ProfilePiece>) -> Vec<ProfilePiece> {
    pieces
        .into_iter()
        .flat_map(|p| {
            subdivide_bernstein(p.coeffs, p.duration, MAX_CORRECTION_PIECE_SECS)
                .into_iter()
                .map(|(coeffs, duration)| ProfilePiece { coeffs, duration })
        })
        .collect()
}

/// Append one trapezoid (accel / optional cruise / decel) for `delta_mm`,
/// positioned so its motion starts at absolute position `p_start`. Each
/// segment starts and ends at zero velocity.
fn push_segment(out: &mut Vec<ProfilePiece>, p_start: f64, delta_mm: f64, speed: f64, accel: f64) {
    let sign = delta_mm.signum();
    let d = delta_mm.abs();
    let v = speed.min((d * accel).sqrt());
    let t_ramp = v / accel;
    let d_ramp = 0.5 * accel * t_ramp * t_ramp;
    let d_cruise = d - 2.0 * d_ramp;
    push_quad(out, p_start, sign, 0.0, 0.0, accel, t_ramp);
    if d_cruise > 1e-12 {
        push_lin(out, p_start, sign, d_ramp, v, d_cruise / v);
    }
    push_quad(out, p_start, sign, d_ramp + d_cruise, v, -accel, t_ramp);
}

fn push_quad(out: &mut Vec<ProfilePiece>, base: f64, sign: f64, p0: f64, v0: f64, a: f64, t: f64) {
    if t <= 0.0 {
        return;
    }
    let b0 = p0;
    let b1 = p0 + v0 * t / 3.0;
    let b2 = p0 + 2.0 * v0 * t / 3.0 + a * t * t / 6.0;
    let b3 = p0 + v0 * t + 0.5 * a * t * t;
    out.push(ProfilePiece {
        coeffs: [base + sign * b0, base + sign * b1, base + sign * b2, base + sign * b3],
        duration: t,
    });
}

fn push_lin(out: &mut Vec<ProfilePiece>, base: f64, sign: f64, p0: f64, v: f64, t: f64) {
    if t <= 0.0 {
        return;
    }
    out.push(ProfilePiece {
        coeffs: [
            base + sign * p0,
            base + sign * (p0 + v * t / 3.0),
            base + sign * (p0 + 2.0 * v * t / 3.0),
            base + sign * (p0 + v * t),
        ],
        duration: t,
    });
}
```

- [ ] **Step 4: Run the characterization tests + full correction suite**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: PASS, including the existing `chunking_respects_frame_budget_and_ring_depth` (proves single-segment output unchanged).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/correction.rs rust/motion-bridge/src/correction/tests.rs
git commit -m "refactor(correction): build segments via base-offset push_segment"
```

---

## Task 2: `plan_correction_sequence`

A gapless multi-segment builder: each segment placed at the running cumulative position.

**Files:**
- Modify: `rust/motion-bridge/src/correction.rs`
- Test: `rust/motion-bridge/src/correction/tests.rs`

- [ ] **Step 1: Write the failing tests**

Add to `rust/motion-bridge/src/correction/tests.rs`:

```rust
#[test]
fn sequence_single_segment_equals_profile() {
    let a = plan_correction_profile(3.0, 5.0, 100.0).unwrap();
    let b = plan_correction_sequence(&[3.0], 5.0, 100.0).unwrap();
    assert_eq!(a.len(), b.len());
    for (x, y) in a.iter().zip(b.iter()) {
        assert_eq!(x.coeffs, y.coeffs);
        assert_eq!(x.duration, y.duration);
    }
}

#[test]
fn sequence_is_globally_contiguous() {
    // Fading oscillation: alternating sign, shrinking amplitude.
    let pieces = plan_correction_sequence(&[1.0, -1.0, 0.6, -0.6, 0.3], 50.0, 5000.0).unwrap();
    for w in pieces.windows(2) {
        assert!((w[0].coeffs[3] - w[1].coeffs[0]).abs() < 1e-9, "no gap between segments");
    }
    // Final absolute position equals the sum of segments.
    let sum: f64 = [1.0, -1.0, 0.6, -0.6, 0.3].iter().sum();
    assert!((pieces.last().unwrap().coeffs[3] - sum).abs() < 1e-6);
}

#[test]
fn sequence_drops_subepsilon_and_rejects_all_empty() {
    let pieces = plan_correction_sequence(&[2.0, 1e-9, -2.0], 50.0, 5000.0).unwrap();
    // Only two real segments; net position returns to ~0.
    assert!((pieces.last().unwrap().coeffs[3]).abs() < 1e-6);
    assert!(plan_correction_sequence(&[1e-9, -1e-9], 50.0, 5000.0).is_err());
    assert!(plan_correction_sequence(&[1.0], 0.0, 5000.0).is_err());
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(sequence_)'`
Expected: FAIL with "cannot find function `plan_correction_sequence`".

- [ ] **Step 3: Implement `plan_correction_sequence`**

Add to `rust/motion-bridge/src/correction.rs` after `plan_correction_profile`:

```rust
const SEGMENT_EPS_MM: f64 = 1e-5;

/// Plan a contiguous, gapless piece sequence for a list of relative motor-space
/// moves. Segment k+1 begins exactly where segment k ends, in both position and
/// time. Sub-epsilon segments are skipped. At least one real segment is required.
pub fn plan_correction_sequence(
    segments: &[f64],
    speed: f64,
    accel: f64,
) -> Result<Vec<ProfilePiece>, String> {
    if !(speed > 0.0) || !(accel > 0.0) {
        return Err(format!(
            "correction sequence needs speed>0, accel>0; got {speed} {accel}"
        ));
    }
    let mut out = Vec::new();
    let mut pos = 0.0;
    let mut any = false;
    for &s in segments {
        if s.abs() < SEGMENT_EPS_MM {
            continue;
        }
        push_segment(&mut out, pos, s, speed, accel);
        pos += s;
        any = true;
    }
    if !any {
        return Err("correction sequence has no segment above SEGMENT_EPS_MM".to_string());
    }
    Ok(subdivide_all(out))
}
```

- [ ] **Step 4: Run to verify pass**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(sequence_)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/correction.rs rust/motion-bridge/src/correction/tests.rs
git commit -m "feat(correction): gapless multi-segment plan_correction_sequence"
```

---

## Task 3: Scheduling helpers (per-piece end times, per-chunk release times)

Pure functions that turn the piece schedule into the pacing decisions the streamer needs.

**Files:**
- Modify: `rust/motion-bridge/src/correction.rs`
- Test: `rust/motion-bridge/src/correction/tests.rs`

- [ ] **Step 1: Write the failing tests**

Add to `rust/motion-bridge/src/correction/tests.rs`:

```rust
#[test]
fn end_host_times_are_cumulative() {
    let pieces = vec![
        ProfilePiece { coeffs: [0.0; 4], duration: 0.10 },
        ProfilePiece { coeffs: [0.0; 4], duration: 0.25 },
        ProfilePiece { coeffs: [0.0; 4], duration: 0.05 },
    ];
    let ends = piece_end_host_times(&pieces, 100.0);
    assert_eq!(ends.len(), 3);
    assert!((ends[0] - 100.10).abs() < 1e-9);
    assert!((ends[1] - 100.35).abs() < 1e-9);
    assert!((ends[2] - 100.40).abs() < 1e-9);
}

#[test]
fn release_times_gate_only_overcommitting_chunks() {
    // 5 pieces each 0.1s, start at t=10.0; ring depth 3; chunks of 2,2,1.
    let ends: Vec<f64> = (1..=5).map(|i| 10.0 + 0.1 * i as f64).collect();
    let new_heads = [2u32, 4, 5];
    let rel = chunk_release_times(&ends, 3, &new_heads, 0.02);
    // chunk 0 (new_head 2 <= 3): no wait.
    assert!(rel[0].is_none());
    // chunk 1 (new_head 4 > 3): needs piece index 4-3-1=0 drained -> ends[0]=10.1 +margin.
    assert!((rel[1].unwrap() - (10.1 + 0.02)).abs() < 1e-9);
    // chunk 2 (new_head 5 > 3): needs piece index 5-3-1=1 drained -> ends[1]=10.2 +margin.
    assert!((rel[2].unwrap() - (10.2 + 0.02)).abs() < 1e-9);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(end_host_times_are_cumulative) or test(release_times_gate_only_overcommitting_chunks)'`
Expected: FAIL with "cannot find function".

- [ ] **Step 3: Implement the helpers**

Add to `rust/motion-bridge/src/correction.rs`:

```rust
/// Host-time at which each piece finishes, given the stream start time.
pub fn piece_end_host_times(pieces: &[ProfilePiece], start_host_secs: f64) -> Vec<f64> {
    let mut t = start_host_secs;
    pieces
        .iter()
        .map(|p| {
            t += p.duration;
            t
        })
        .collect()
}

/// Earliest host-time each chunk may be sent without overcommitting an MCU
/// correction ring of depth `ring_depth`. A chunk whose cumulative piece count
/// `new_head` fits within the ring needs no wait (`None`); otherwise it must
/// wait until the piece that has to drain first has finished, plus a margin to
/// cover message flight. `piece_end_host` is indexed by global piece number.
pub fn chunk_release_times(
    piece_end_host: &[f64],
    ring_depth: u32,
    chunk_new_heads: &[u32],
    safety_margin_secs: f64,
) -> Vec<Option<f64>> {
    chunk_new_heads
        .iter()
        .map(|&h| {
            if h <= ring_depth {
                None
            } else {
                let idx = (h - ring_depth - 1) as usize;
                Some(piece_end_host[idx] + safety_margin_secs)
            }
        })
        .collect()
}
```

- [ ] **Step 4: Run to verify pass**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(end_host_times_are_cumulative) or test(release_times_gate_only_overcommitting_chunks)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/correction.rs rust/motion-bridge/src/correction/tests.rs
git commit -m "feat(correction): piece-end and chunk-release scheduling helpers"
```

---

## Task 4: Stream with paced refill; rewrite `adjust_motor` on top

Replace the synchronous burst in `adjust_motor` with a paced streamer used by both the single-delta and sequence paths.

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:1915-1985` (`adjust_motor`)

- [ ] **Step 1: Add the `stream_correction_entries` helper**

In `rust/motion-bridge/src/bridge.rs`, inside the same `impl`/`#[pymethods]` block as `adjust_motor`, add (a non-`#[pyo3]`, plain private method):

```rust
fn stream_correction_entries(
    &self,
    py: Python<'_>,
    mcu_handle: u32,
    axis_idx: u8,
    motor_idx: u8,
    pieces: &[crate::correction::ProfilePiece],
) -> PyResult<f64> {
    const CORRECTION_LEAD_SECS: f64 = 0.15;
    const REFILL_MARGIN_SECS: f64 = 0.05;
    let ring_depth = runtime::stepping_state::CORRECTION_RING_DEPTH as u32;
    let handle = mcu_handle_from_raw(mcu_handle);

    let (start_secs, entries) = {
        let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let start = router.host_now_secs() + CORRECTION_LEAD_SECS;
        let entries = crate::correction::to_piece_entries(
            pieces,
            |secs| router.host_time_to_mcu_clock(handle, secs).unwrap_or(0),
            start,
        );
        (start, entries)
    };
    if entries.iter().any(|e| e.start_time == 0) {
        return Err(PyRuntimeError::new_err(format!(
            "stream_correction: clock unsynced for mcu {mcu_handle}"
        )));
    }
    let end_host = crate::correction::piece_end_host_times(pieces, start_secs);
    let total_duration = end_host.last().copied().unwrap_or(start_secs) - start_secs;
    let msgs = crate::correction::chunk_correction_messages(axis_idx, motor_idx, &entries);
    let new_heads: Vec<u32> = msgs.iter().map(|m| m.new_head).collect();
    let releases =
        crate::correction::chunk_release_times(&end_host, ring_depth, &new_heads, REFILL_MARGIN_SECS);

    let io = {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!("stream_correction: unknown mcu_handle {mcu_handle}"))
        })?;
        conn.host_io
            .as_ref()
            .ok_or_else(|| {
                PyRuntimeError::new_err(
                    "stream_correction: serial transport not attached for this MCU \
                     (EtherCAT correction moves are not supported yet)",
                )
            })?
            .clone()
    };

    py.detach(|| -> PyResult<()> {
        for (msg, release) in msgs.iter().zip(releases) {
            if let Some(rel) = release {
                loop {
                    let now = {
                        let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
                        router.host_now_secs()
                    };
                    if now >= rel {
                        break;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(2));
                }
            }
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
                    "stream_correction rejected by MCU: error {} (refill fell behind ring drain?)",
                    r.result
                )));
            }
        }
        Ok(())
    })?;
    Ok(CORRECTION_LEAD_SECS + total_duration)
}
```

- [ ] **Step 2: Rewrite `adjust_motor` to delegate**

Replace the entire body of `adjust_motor` (bridge.rs:1924-1985, everything after the signature) with:

```rust
    ) -> PyResult<f64> {
        let pieces = crate::correction::plan_correction_profile(delta_mm, speed, accel)
            .map_err(PyRuntimeError::new_err)?;
        self.stream_correction_entries(py, mcu_handle, axis_idx, motor_idx, &pieces)
    }
```

- [ ] **Step 3: Build and run the existing suite (no regressions)**

Run: `cd rust && cargo nextest run -p motion-bridge && cargo build -p motion-bridge`
Expected: PASS / builds. The `adjust_motor` behavior for a fitting single move is unchanged (one chunk, no wait, same return value).

- [ ] **Step 4: Clippy clean**

Run: `cd rust && cargo clippy -p motion-bridge -- -D warnings`
Expected: no warnings.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(bridge): paced refill streaming for correction moves"
```

---

## Task 5: Expose `submit_correction_sequence` to Python

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (new `#[pyo3]` method beside `adjust_motor`)
- Modify: `klippy/motion_bridge.py:411` (wrapper beside `adjust_motor`)

- [ ] **Step 1: Add the PyO3 method**

In `rust/motion-bridge/src/bridge.rs`, immediately after the `adjust_motor` method, add:

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
    ) -> PyResult<f64> {
        let pieces = crate::correction::plan_correction_sequence(&segments, speed, accel)
            .map_err(PyRuntimeError::new_err)?;
        self.stream_correction_entries(py, mcu_handle, axis_idx, motor_idx, &pieces)
    }
```

- [ ] **Step 2: Build the cdylib**

Run: `cd rust && cargo build -p motion-bridge`
Expected: builds; the new method is registered on the Python `_bridge` object.

- [ ] **Step 3: Add the Python wrapper**

In `klippy/motion_bridge.py`, after the `adjust_motor` method (line 411), add:

```python
    def submit_correction_sequence(self, mcu_id, axis_idx, motor_idx,
                                   segments, speed, accel):
        return self._bridge.submit_correction_sequence(
            mcu_id, axis_idx, motor_idx, [float(s) for s in segments],
            speed, accel
        )
```

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs klippy/motion_bridge.py
git commit -m "feat(bridge): expose submit_correction_sequence to host"
```

---

## Task 6: Full gate + end-to-end verification

**Files:** none (verification only)

- [ ] **Step 1: Run the Rust workspace suite**

Run: `cd rust && cargo nextest run -p motion-bridge -p runtime`
Expected: PASS.

- [ ] **Step 2: Run the project quick gate**

Run: `./scripts/ci.sh quick`
Expected: green (ruff, rust tests, clippy -D warnings, fmt check, watchdog canary).

- [ ] **Step 3: End-to-end in kalico-sim (manual, via the kalico-sim skill)**

Using the `kalico-sim` skill, drive a long sequence to one motor of an idle multi-motor axis and assert from the structured logs:
- `EVENT_MOTION_CORRECTION_START` fires once, `EVENT_MOTION_CORRECTION_DRAINED` once;
- only the target stepper's pin toggles; the axis position tracker is unchanged;
- a sequence longer than `CORRECTION_RING_DEPTH` completes without a `KALICO_ERR_RING_FULL` rejection (correct pacing);
- inject an artificially long first segment so refill must wait, and confirm later chunks are accepted (not rejected), proving the pacing gate works.

Record the run in the PR description. This step needs the running sim; it is verification, not a code change.

---

## Self-Review Notes

- **Spec coverage:** `plan_correction_sequence` (spec §Design.1) → Task 2; `submit_correction` self-scheduling stream + `adjust_motor` as one-segment wrapper (spec §Design.2) → Tasks 4–5; streaming refill, the one piece of new plumbing (spec §Design.2 bullet 2) → Task 4 (`stream_correction_entries` + `chunk_release_times`); zero-gap invariant (spec §Design.1 invariant) → Task 1 `pieces_are_position_contiguous` + Task 2 `sequence_is_globally_contiguous`; fail-loud on refill-behind (spec §Validation) → Task 4 `r.result != 0` error + Task 6 sim check; "no MCU/wire change, no pipeline unification" (spec §Non-goals) → honored (host-only; MCU already supports refill).
- **No velocity-blending** (spec non-goal) — builder brakes each segment to zero velocity at joints, matching the buzz's reversals; verified by `single_segment_profile_has_zero_velocity_ends`.
- **Type consistency:** `ProfilePiece` (`coeffs:[f64;4]`, `duration:f64`), `plan_correction_sequence(&[f64], f64, f64) -> Result<Vec<ProfilePiece>, String>`, `piece_end_host_times(&[ProfilePiece], f64) -> Vec<f64>`, `chunk_release_times(&[f64], u32, &[u32], f64) -> Vec<Option<f64>>`, `stream_correction_entries(Python, u32, u8, u8, &[ProfilePiece]) -> PyResult<f64>` are used identically across tasks.
