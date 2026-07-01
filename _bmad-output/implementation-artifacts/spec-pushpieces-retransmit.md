---
title: 'Motion PushPieces retransmission (serial MCU)'
type: 'feature'
created: '2026-06-30'
status: 'done'
baseline_commit: '661f59373'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/ethercat-endpoint-death-investigation.md'
  - '{project-root}/docs/rewrite/mcu-c-rust-boundary.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** On the USB-serial MCU, motion `PushPieces` uses a one-shot `kalico_call_on_channel(..., 5s)` (`call_push_pieces`). The wire layer retransmits the *request*, but once it's acked, a transient corruption that loses the **response** isn't recovered — the call waits the full 5 s, the next piece is seconds stale, and the in-past guard aborts the print. Root cause (investigation, High): CH340-under-EMI corruption; Klipper rode through by re-requesting, our PushPieces path doesn't. (Full rationale: linked investigation.)

**Approach:** Add **bounded, fast re-request** to `call_push_pieces`, **serial transport only**: a short per-attempt timeout, resending the *identical* frame up to a budget so a fresh attempt gets a clean response (recovers in ~hundreds of ms, not a 5 s crash). Retry on transient/timeout; fail-fast on a dead transport. Safe because the real MCU's PushPieces is **slot-addressed/idempotent**; the EtherCat path stays unchanged (reliable local socket, and append-based so **not** idempotent).

## Boundaries & Constraints

**Always:**
- Retry **only** the serial (USB MCU) branch of `call_push_pieces`. The EtherCat branch is byte-for-byte unchanged.
- Re-request the **identical** frame each attempt (same `start_slot`/`new_head`); idempotency rests entirely on the MCU's slot-addressed write + absolute `commit_head`. Never mutate the frame between attempts.
- Keep the **total** budget bounded and ≤ today's 5 s ceiling; on exhaustion, return the same error class as today (`Transient`) so the existing pump retry + in-past guard remain the backstop. No unbounded/infinite retry.
- Compose with Part A: retry on `Timeout`/transient corruption; **do not** retry `Closed`/`Io` (dead transport) — propagate `Fatal` immediately so fail-fast is preserved.
- Each retry attempt emits a structured log (`subsystem=mcu-comms`) so retransmissions are observable in VL.

**Ask First:**
- Adding retry to, or changing the idempotency of, the **EtherCat** PushPieces path (separate concern).

**Never:**
- Do not add a second retransmit layer inside `host-rt` (`kalico_call_on_channel`/reactor) — the wire layer already retransmits requests; this is response-loss recovery at the call layer.
- Do not change PushPieces frame contents between attempts (breaks idempotency).
- Do not touch MCU firmware or the C/Rust seam (`mcu-c-rust-boundary.md`).
- Do not change trajectory shape or timing — delivery reliability only.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Transient lost/corrupt response (serial) | one attempt times out at the short per-attempt timeout | re-request; a subsequent attempt gets a clean response → `Ok` within budget (no 5 s stall) | recovered |
| Corruption persists past budget | all attempts fail | return `Transient` after the bounded budget (≤ old 5 s) → existing pump path + in-past guard handle it | bounded, no infinite loop |
| Dead transport mid-call | `Closed`/`Io` | **no retry** — return `Fatal` immediately (Part A fail-fast preserved) | fail-loud |
| Healthy link | first attempt succeeds | returns `Ok` on attempt 1; no added latency | N/A |
| EtherCat PushPieces | any | unchanged one-shot behavior | N/A |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/pump.rs` -- `call_push_pieces` (:1038-1100): the serial branch (`McuTransport::Serial`, :1073-1088) calls `io.kalico_call_on_channel(..., self.timeout)` once. **This is where the retry loop goes.** `SendError::{Fatal,Transient}` (:257); current serial mapping sends all `TransportError` → `Transient` (:1084-1086). `WireSink.timeout` = `pump_timeout` (5 s, set `bridge.rs:3022`, wired `:3034`).
- `rust/host-rt/src/transport.rs` -- `TransportError { Io, Timeout, Closed, Parse, DispatcherTimeout, Backpressure, McuShutdown }` (:6) — classify retry vs fail-fast from these.
- `rust/host-rt/src/host_io/mod.rs` -- serial `kalico_call_on_channel` (:743): submits a reactor `McuCall`; request is wire-retransmitted (RTO), response-loss → `Timeout`/`DispatcherTimeout`. No change (context only).
- `rust/host-rt/src/mcu_serial_conn.rs` -- EtherCat `kalico_call_on_channel` (:211, one-shot). Unchanged.
- `rust/c-api/src/runtime_ffi.rs` (:875, :914) + `rust/runtime/src/piece_ring.rs` (`write_slot` :74, `commit_head` :93 `Stale`) + `src/piece_sink.c` (:203, :216) -- the slot-addressed/idempotent basis for safe re-request. No change.

## Tasks & Acceptance

**Execution:**
- [x] `rust/motion-engine/src/pump.rs` -- extracted the bounded retry policy into a pure free fn `pushpieces_retransmit_serial(mcu_id, max_attempts, attempt_call)` (for testability) and call it from the **serial** branch of `call_push_pieces`: per attempt `kalico_call_on_channel(.., PUSHPIECES_ATTEMPT_TIMEOUT=30ms)` via `body.clone()`; `Ok`→return; `Closed`/`Io`/`McuShutdown`→`SendError::Fatal` immediately (no retry — genuine MCU failure fails loud); other transient→`event=pushpieces_retry{mcu,attempt,max_attempts}` warn + retry until `PUSHPIECES_MAX_ATTEMPTS=3` (total ≈90 ms, under the 100 ms drip window) then `event=pushpieces_giveup` warn + `SendError::Transient`. EtherCat branch unchanged (single `self.timeout` call).
- [x] `rust/motion-engine/src/pump/tests.rs` -- `pushpieces_retransmit_tests`: (a) transient `Timeout` ×2 then `Ok` → `Ok` on attempt 3; first-attempt-success → exactly 1 call (no added latency); (b) `Timeout` forever, budget 4 → `Transient` after exactly 4 attempts (no infinite loop); (c) `Closed` → `Fatal`, 1 call; (d) `Io` → `Fatal`, 1 call; (e) `McuShutdown` → `Fatal`, 1 call. Byte-identical resend is by construction (`body.clone()` of a fixed `Vec`).

**Acceptance Criteria:**
- Given the serial MCU transiently loses a PushPieces response, when `call_push_pieces` runs, then it re-requests and returns `Ok` within the bounded budget — no 5 s stall and no `pump_piece_in_past`.
- Given persistent corruption beyond the budget, when the attempts exhaust, then it returns `Transient` (not `Fatal`, no infinite retry).
- Given a `Closed`/`Io` transport error, when it occurs, then `call_push_pieces` returns `Fatal` immediately with no retry (Part A fail-fast intact).
- Given the EtherCat transport, when `call_push_pieces` runs, then its behavior is unchanged.

## Design Notes

Why response-loss, not request-loss: the serial `kalico_call_on_channel` routes through the reactor's `unacked_window`, which auto-retransmits the *request* until seq-acked (so `retransmit_timeout` never fired in the crashes). Once acked, a corrupted **response** has no recovery — the call just waits the deadline. A host-level **re-request** (fresh `cid`, identical body) closes that gap; a late original response (old `cid`) is harmlessly dropped (`unknown correlation_id`).

Idempotency is the safety basis and is **serial-specific**: the real MCU writes pieces positionally (`runtime_write_piece` → `write_slot`, no head move) and commits the head absolutely (`commit_head`; `proposed <= cur` ⇒ `Stale`, treated as success). Re-applying the same frame is a no-op. The EtherCat endpoint's `push_from_bytes` is append-based (ignores `start_slot`/`new_head`) → a resend double-advances → **excluded** from retry.

Budget sizing is the subtle constraint (review finding): the retry burst blocks the *single-threaded* pump, so the whole budget must stay **under the smallest lead** or the retry itself starves the MCU of later pieces and recreates the `-308` it prevents. Leads: `DRIP_WINDOW_SECS`=100 ms, `DEFAULT_LEAD_SECS`=250 ms, `MAX_LEAD_SECS`=2 s. So `PUSHPIECES_ATTEMPT_TIMEOUT`=30 ms (a few × the ~6 ms RTT — healthy responses still return on arrival, not at timeout) × `PUSHPIECES_MAX_ATTEMPTS`=3 ≈ 90 ms total, under the 100 ms drip floor. This recovers *isolated* corruption fast; *sustained* corruption gives up in ~90 ms to the loud in-past-guard backstop (far better than the old 5 s stall→crash). A fixed-attempt budget (not `timeout/per-attempt`) keeps it lead-bounded regardless of the configured `self.timeout`.

`McuShutdown` is classed `Fatal` alongside `Closed`/`Io`: a shut-down MCU is a genuine failure that must surface loud (project "fail loudly"), not be buried under the retry budget then mislabeled `Transient`. The remaining transient variants (`Timeout`/`DispatcherTimeout`/`Parse`/`Backpressure`) are the recoverable response-loss/corruption cases; on the `McuCall` path only `Timeout` is actually reachable, the rest are defensive. Budget exhaustion emits `pushpieces_giveup` so the most important moment is observable, not just an error string.

A late original response (old `cid`) arriving after a re-request is harmlessly dropped as `unknown correlation_id` — the channel correlates responses by `cid`, not FIFO, so there is no response-stream desync (confirmed against the `McuCall`/`transport_state.pending` path).

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine` -- expected: green, incl. the new retry-loop tests.
- `./scripts/ci.sh quick` -- expected: fully green (ruff, rust-test, rust-clippy `-D warnings`, rust-fmt, watchdog).

**Manual checks (if no CLI):**
- Bench repro under serial corruption: `pushpieces_retry` events appear and the call recovers; **no** `pump_piece_in_past` / 5 s `pump_send_blocked` for transient corruption. Compare `kalico_stream_error` rate vs recovered sends via `query-logs`.

## Deferred

- Make the EtherCat endpoint PushPieces handler idempotent (honor `start_slot`/`new_head` like `runtime_write_piece`/`commit_head`, or add cid dedup) so it too is retransmit-safe → append to `deferred-work.md`. Not needed now (local socket is reliable; no corruption observed there).
