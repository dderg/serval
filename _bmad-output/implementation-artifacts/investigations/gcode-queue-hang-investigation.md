# Investigation: Long gcode queue hangs the pipeline

## Hand-off Brief

1. **What happened.** Short gcode executes fine (file or pipe); a long gcode queue
   hangs — motion stops making progress. User hypothesis (no upstream
   backpressure) is partially Confirmed: the gcode→stream-planner→pump chain is
   fully unbounded channels.
2. **Where the case stands.** Structural root cause Confirmed (Medium-High
   confidence): the only backpressure is the finite MCU piece ring, released
   SOLELY by heartbeat-borne `retired_counts`; that heartbeat path is silently
   droppable on MCU TX-buffer-full, and the `credit_freed` path is vestigial
   (nothing feeds the pump). The pump's `send_frame` is a 5 s synchronous RPC
   that can freeze the pump and delay credit application. Reactor starvation
   REFUTED (serial I/O is on a dedicated Rust thread). A council-of-models
   review (Kate/Ben/Dana) converged on H4 = Still Open and a TX-drop
   instrumentation experiment to settle it.
3. **What's needed next (USER REDIRECT, 2026-06-21).** The TX-buffer
   investigation (H4) is DEPRIORITIZED. First focus is to add a limit on the
   stream keyed to how much the pump has actually consumed — i.e. bound the
   stream-planner→pump path by pump-side consumption (pushed/retired/ring room),
   not by TX-buffer dynamics. The unbounded submit→stream→pump chain (F1) is
   the target. TX-drop instrumentation (H4) can be revisited later if the
   consumption-bound fix does not resolve the hang.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-21                                                                 |
| Status           | Active                                                                     |
| System           | Kalico rewrite; Python host (`klippy/`) + Rust motion engine (`rust/motion-engine` via PyO3 `_motion_engine.so`) |
| Evidence sources | `klippy/gcode.py`, `klippy/motion.py`, `klippy/motion_engine.py`, `rust/motion-engine/src/stream_planner.rs`, `rust/motion-engine/src/pump.rs`, `rust/motion-engine/src/bridge.rs` |

## Problem Statement

User-reported (verbatim, lightly paraphrased): "When I execute a short amount of
gcode it works fine, even from a file, but when the gcode queue is long it just
hangs. I suspect it's trying to process all of the gcode at once, not even
looking at how much was actually processed by the following pipeline."

The claim "processes all gcode at once with no downstream backpressure" is a
hypothesis (H1) to verify, not a fact.

## Evidence Inventory

| Source                                           | Status    | Notes |
| ------------------------------------------------ | --------- | ----- |
| `klippy/gcode.py:560-605` (`_process_data`)      | Available | Stronghold; `while pending_commands:` ingest loop |
| `klippy/gcode.py:294-332` (`_process_commands`)  | Available | Per-line dispatch to handlers |
| `klippy/extras/gcode_move.py:156-187` (`cmd_G1`) | Available | Calls `toolhead.move()` per G1 |
| `klippy/motion.py:356-389` (`move`)              | Available | `engine.submit_move` + `_sync_print_time` per move |
| `klippy/motion_engine.py:386-388` (`submit_move`)| Available | Thin PyO3 forward to Rust |
| `rust/motion-engine/src/bridge.rs:3344-3428`     | Available | `submit_move` PyO3 fn; `planner.submit_move(m)` |
| `rust/motion-engine/src/stream_planner.rs:62`    | Available | **`unbounded()` channel — no backpressure on submit** |
| `rust/motion-engine/src/stream_planner.rs:91-94` | Available | `submit_move` = non-blocking `sender.send` |
| `rust/motion-engine/src/bridge.rs:2786`          | Available | **pump input channel = `std::sync::mpsc::channel()` (unbounded)** |
| `rust/motion-engine/src/bridge.rs:3117-3213`     | Available | dispatch closure → `pump_tx.send` (non-blocking) |
| `rust/motion-engine/src/pump.rs:18-44`           | Available | `AxisQueue::room() = ring_depth - (pushed - retired)` |
| `rust/motion-engine/src/pump.rs:515-538`         | Available | `Heartbeat { retired_counts }` updates `q.retired` — the credit-release path |
| `rust/motion-engine/src/pump.rs:604-720`         | Available | `schedule()` → `StallFull`/`StallAhead` → `recv_timeout(10ms)` poll loop |
| Reactor fd scheduling / serial rx path           | Partial   | Need to confirm whether `_process_data` can starve heartbeat processing |
| MCU heartbeat generation + transport             | Missing   | Need to confirm heartbeats actually flow under long-queue load |
| `on_credit_freed` event wiring                   | Partial   | `bridge.rs:2316` emits to Python; relationship to heartbeat `retired_counts` unclear |

## Investigation Backlog

| # | Path to Explore                                                                 | Priority | Status   | Notes |
| - | ------------------------------------------------------------------------------- | -------- | -------- | ----- |
| 1 | Heartbeat path: MCU generation → serial rx → `attach_heartbeat_callback` → pump `Heartbeat` msg → `q.retired` advance | High | Open | Credit release only matters once ring fills (long queue) |
| 2 | Reactor fd scheduling: can a ready file-input fd starve serial-rx callbacks that deliver heartbeats? | High | Open | Direct test of the "processes all at once" hypothesis |
| 3 | `on_credit_freed` vs heartbeat `retired_counts`: are these two redundant credit paths, and is one broken? | High | Open | `bridge.rs:2316` + `pump.rs:518` both touch retired |
| 4 | `_process_data` EOF / file-input flow for long files (regular-file fd always readable) | Medium | Open | File input is the reported trigger |
| 5 | `schedule()` correctness when `retired` advances under `StallFull` — does `room()` unblock? | Medium | Open | Pump loop logic |
| 6 | Per-move overhead in `motion.py:move()` (`_fire_active_callbacks`, `resync_parked_servos`, `check_move`) — any hidden block/yield? | Low | Open | Rule out per-move stall |

## Timeline of Events

| Time | Event | Source | Confidence |
| ---- | ----- | ------ | ---------- |
| 2026-06-21 | User reports long-gcode hang; short gcode works | User | Confirmed |
| 2026-06-21 | Stronghold located at `gcode.py:579-600` ingest loop | Code reading | Confirmed |
| 2026-06-21 | Submit→stream→pump chain found to be fully unbounded | Code reading | Confirmed |

## Confirmed Findings

### Finding 1: The gcode→engine submit path has no channel-level backpressure

**Evidence:**
- `rust/motion-engine/src/stream_planner.rs:62` — `let (tx, rx) = unbounded();`
- `rust/motion-engine/src/stream_planner.rs:91-94` — `submit_move` does `self.sender.send(StreamMsg::Move(m))` (unbounded, non-blocking)
- `rust/motion-engine/src/bridge.rs:2786` — `let (pump_tx_init, pump_rx) = std::sync::mpsc::channel::<crate::pump::PumpMsg>();` (unbounded)
- `rust/motion-engine/src/bridge.rs:3117-3213` — dispatch closure sends to `pump_tx_for_cb` (non-blocking)

**Detail:** Every `G1` → `toolhead.move()` → `engine.submit_move()` enqueues into an
unbounded channel. The stream-planner thread drains it and dispatches shaped
segments into another unbounded channel feeding the pump. Neither send blocks.
The only bounded buffer in the whole chain is the MCU piece ring
(`runtime/src/piece_ring.rs`, `ring_depth` per axis). This confirms the load-bearing
part of H1: the host can outrun the pipeline arbitrarily; nothing in the
gcode→pump path looks at downstream consumption.

### Finding 2: MCU ring credit is released only via heartbeat `retired_counts`

**Evidence:**
- `rust/motion-engine/src/pump.rs:18-37` — `AxisQueue::room() = ring_depth - (pushed - retired)`
- `rust/motion-engine/src/pump.rs:515-524` — `PumpMsg::Heartbeat { retired_counts }` sets `q.retired = c`
- `rust/motion-engine/src/bridge.rs:3082-3090` — `io.attach_heartbeat_callback(...)` forwards MCU retired counts to the pump as `PumpMsg::Heartbeat`

**Detail:** The pump stalls (`StallFull`) when `room() == 0`. `room()` recovers
only when `retired` advances, which happens only when a `PumpMsg::Heartbeat`
arrives. Heartbeats originate on the MCU, traverse the serial transport, are
parsed on the host, and fire the heartbeat callback. If heartbeats stop flowing,
or the reactor can't process serial rx to deliver them, the pump stays
`StallFull` forever — which is exactly the "long queue hangs" symptom. Short
queues never fill the ring, so they never depend on this path.

## Deduced Conclusions

### Deduction 1: The hang is downstream of submit, not in gcode ingestion itself

**Based on:** Finding 1, Finding 2

**Reasoning:** `submit_move` is non-blocking on unbounded channels, so the gcode
ingest loop does not block on the engine. The observable "hang" must therefore
be motion stalling — the pump stuck in `StallFull` with `room() == 0` and
`retired` not advancing — while the host may still be ingesting commands into the
unbounded channel (which would grow without bound).

**Conclusion:** The bug is in the credit-release path (heartbeat delivery or
processing), which is only exercised once the MCU ring fills — i.e. only for
long gcode queues. This matches the symptom boundary precisely.

## Hypothesized Paths

### Hypothesis 1: Host processes all gcode at once with no downstream backpressure

**Status:** Partially Confirmed

**Theory:** The gcode processor feeds moves into the pipeline without checking
downstream consumption.

**Supporting indicators:** Finding 1 — submit→stream→pump chain is fully
unbounded.

**Would confirm:** Evidence that the unbounded queue grows unboundedly during a
long print AND that this starves or blocks the credit-release path.

**Would refute:** Evidence that an explicit backpressure/yield point exists
between `cmd_G1` and `submit_move` that I have not yet found.

**Resolution:** Channel-level finding Confirmed (Finding 1). Whether this is the
*cause* of the hang vs. a contributing factor depends on the credit-release path
(Outcome 2/3).

### Hypothesis 2: Reactor starvation — `_process_data` blocks heartbeat delivery

**Status:** Open

**Theory:** While `_process_data` churns through a large batch of G1 commands,
the cooperative reactor cannot service the serial-rx callback that delivers MCU
heartbeats, so `retired` never advances and the pump deadlocks at `StallFull`.

**Supporting indicators:** Klipper's reactor is cooperative single-threaded;
heartbeat delivery is reactor-driven.

**Would confirm:** Evidence that `_process_data` processes many commands per
invocation without yielding, and that serial rx is only serviced between
invocations, AND that the pump fills within a single invocation's window.

**Would refute:** Evidence that `_process_data` yields per-command or per-small-
batch, or that heartbeats arrive on a thread independent of the reactor.

### Hypothesis 3: Heartbeats are never sent / never parsed under streaming load

**Status:** Open

**Theory:** The MCU heartbeat is gated on something that doesn't fire during
sustained streaming, or the host parser drops heartbeats while the piece stream
is saturated.

**Supporting indicators:** None yet — needs Outcome 2.

**Would confirm:** A log/event trace showing `retired_counts` static while
pieces are queued.

**Would refute:** A log/event trace showing `retired_counts` advancing during
the hang.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Reactor fd scheduling & whether file-input fd starves serial rx | Confirms/refutes H2 | Read `klippy/reactor.py` fd dispatch + measurement |
| MCU heartbeat generation rate & transport | Confirms/refutes H3 | Read runtime heartbeat emit + `host-rt` heartbeat parse |
| `on_credit_freed` event vs heartbeat `retired_counts` relationship | Determines if a credit path is broken/duplicated | Read `bridge.rs:2300-2320` emit site + `runtime_events.rs` parse |
| Runtime trace of a long-queue hang (event_log) | Direct evidence of where flow stops | Run repro with `event_log_emit` enabled, inspect `events/*.jsonl` |

## Source Code Trace

| Element       | Detail |
| ------------- | ------ |
| Error origin  | (suspected) `rust/motion-engine/src/pump.rs` `StallFull` loop with `retired` not advancing |
| Trigger       | Long gcode queue exceeding MCU piece-ring capacity (`ring_depth` per axis) |
| Condition     | Heartbeat-driven credit release fails to advance `q.retired` |
| Related files | `klippy/gcode.py`, `klippy/motion.py`, `rust/motion-engine/src/{stream_planner,pump,bridge}.rs`, `rust/runtime/src/piece_ring.rs`, `rust/host-rt/src/host_io/runtime_events.rs` |

## Conclusion

**Confidence:** Medium-High (structural root cause Confirmed by code; exact
runtime trigger Hypothesized, pending an instrumented repro trace).

The gcode→engine→pump chain has no channel-level backpressure (Finding 1); the
only thing that paces the host against the MCU is the finite piece ring, whose
credit is released SOLELY by heartbeat-borne `retired_counts` (Finding 2). That
credit path is a single point of failure: MCU heartbeats are silently droppable
on TX-buffer-full (Finding 3), and the parallel `credit_freed` event path is
vestigial/dead — nothing feeds it to the pump (Finding 4). Additionally, the
pump's `send_frame` is a synchronous 5-second request-response RPC; a dropped
`PushPiecesResponse` stalls the pump thread for 5s, during which it cannot
process queued heartbeats (Finding 5). Reactor starvation (H2) is REFUTED —
serial I/O runs on a dedicated Rust thread, not the Python reactor. The most
likely runtime trigger is H4: under long-queue load the MCU TX buffer (320 B
serial_irq / 1024 B usb_cdc) saturates, dropping heartbeats and/or responses,
freezing the pump at `StallFull` or in `send_frame` timeouts.

## Recommended Next Steps

### Fix direction

(Pending Outcome 3.) Candidate directions depending on root cause:
- If H2 (reactor starvation): yield from `_process_data` to the reactor when the
  downstream pipeline is saturated, or bound the submit queue and block `submit_move`
  on the stream-planner channel when full.
- If H3 (heartbeat breakage): fix heartbeat generation/parse path.
- Structural (always): introduce bounded backpressure at the submit/stream
  boundary so the host cannot outrun the pipeline by orders of magnitude —
  unbounded channels are the enabling design flaw regardless of the immediate
  trigger.

### Diagnostic

- Run a long-gcode repro with structured event logging enabled and inspect
  `events/*.jsonl` for `credit_freed` / heartbeat `retired_counts` advancement
  while the queue is large.
- Add a temporary `tracing` probe on `pump.rs` `StallFull` showing `pushed`,
  `retired`, `room()` over time.

## Reproduction Plan

1. Start the host + MCU sim (or Renode dual-board) with a long gcode file
   (>ring_capacity moves, e.g. several thousand short G1s).
2. Stream the file via the gcode input fd.
3. Observe: motion starts, then stalls partway; host may continue ingesting.
4. Capture `events/*.jsonl` and the pump `StallFull` trace.
5. Expected (bug present): `retired` stops advancing while `pushed` keeps growing
   or pump stays `StallFull`.

## Side Findings

- `motion.py:380-385` reads `engine.get_last_move_time()` immediately after
  `submit_move`; since the stream planner processes the move on a separate
  thread asynchronously, the post-submit read likely returns the pre-submit
  value, making `_bump_pending_end_time` a no-op per move. Possible separate
  timing bug — not the hang, but worth noting. Evidence-graded: Hypothesized.

## Follow-up: 2026-06-21

### New Evidence

Outcome 2 (deep explore perimeter map) and Outcomes 3-4 completed.

#### Finding 3: MCU heartbeat emits are silently droppable on TX-buffer-full

**Evidence:**
- `src/mcu_transport_dispatch.c:436-474` — `send_status_heartbeat` ends with `mcu_transport_send_frame(MCU_CHANNEL_CONTROL, payload, off)` and does NOT check the return value.
- `src/mcu_transport_dispatch.c:103` — `mcu_transport_send_frame` returns `kalico_console_write_raw(tx_buf, total)`.
- `src/generic/serial_irq.c:149-167` — `kalico_console_write_raw` returns `-1` when the payload does not fit in `transmit_buf` (after compaction). `transmit_buf` is 320 B (`serial_irq.c:37`); usb_cdc variant is 1024 B (`usb_cdc.c:36`).
- `src/runtime_tick.c:255-275` — heartbeat emit is rate-limited to 100 Hz when `retired` advances; `pending_advance` retries on the next 1 ms drain tick.
- `src/runtime_tick.c:283-298` — `runtime_status_drain` is a 10 Hz unconditional fallback that always calls `send_status_heartbeat()`.

**Detail:** Every heartbeat emit can fail silently. The 10 Hz fallback + 100 Hz
rate-limited path provide resilience, but if the MCU TX buffer is chronically
full, every attempt fails and the pump never sees `retired` advance.

#### Finding 4: The `credit_freed` event path is vestigial / dead

**Evidence:**
- `rust/motion-engine/src/pump.rs:225-239` — `PumpMsg` enum has no `CreditFreed` variant; grep for `credit_freed` in `pump.rs` returns 0 matches.
- `rust/motion-engine/src/pump.rs:515-541` — `PumpMsg::Heartbeat` is the ONLY handler that sets `q.retired`.
- `klippy/motion_engine.py:350-356` — `MotionEngine.on_credit_freed` is defined and forwards to `self._engine.on_credit_freed(...)`, but grep for `fn on_credit_freed` in `rust/` returns **no Rust implementation** (would `AttributeError` if ever called).
- `klippy/serialhdl.py:104` — `credit_freed` events are skipped with comment "Handled directly by Rust EventDispatcher; skip Python routing" — but no Rust handler feeds credit to the pump.
- `rust/host-rt/src/host_io/events.rs:346-369` — `CreditFreedEvent` is synthesized from `kalico_status_v6` frames and dispatched to the Python-facing runtime_event_dispatcher only; it does NOT reach the pump.

**Detail:** The `credit_freed` mechanism is a stale vestige of a prior
slot-pool architecture. It neither feeds the pump nor is consumed by Python.
`PumpMsg::Heartbeat` is the sole credit-release path. This confirms the
single-point-of-failure structure.

#### Finding 5: Pump `send_frame` is a synchronous 5-second RPC that can stall the pump thread

**Evidence:**
- `rust/motion-engine/src/pump.rs:851-866` — `WireSink::send_frame` calls `call_push_pieces`.
- `rust/motion-engine/src/pump.rs:757-846` — `call_push_pieces` calls `kalico_call_on_channel(MCU_CHANNEL_PIECES, PushPieces, body, self.timeout)` and synchronously awaits `PushPiecesResponse`.
- `rust/motion-engine/src/bridge.rs:2802` — `pump_timeout = Duration::from_secs(5)`.
- `rust/host-rt/src/host_io/mod.rs:743-770` — `kalico_call_on_channel` waits via `rx.recv_timeout(timeout)`; returns `TransportError::Timeout` after 5 s.
- `src/mcu_transport_dispatch.c:287-315` — `send_push_pieces_response` also ignores `mcu_transport_send_frame`'s return → silently droppable on TX-full.
- `rust/motion-engine/src/pump.rs:660-707` — the pump thread is single-threaded; while blocked in `send_frame` inside the `'send` loop it cannot process queued `PumpMsg::Heartbeat` messages from its `rx` channel.

**Detail:** A dropped `PushPiecesResponse` freezes the pump thread for 5 s.
During that window, heartbeats queue in the unbounded pump channel but are not
applied, so `q.retired` does not advance. Repeated drops → the pump spends its
time in 5 s timeouts instead of dispatching → motion freezes (perceived hang).

#### Finding 6 (Refutation): Reactor starvation is NOT the cause

**Evidence:**
- `rust/host-rt/src/host_io/mod.rs:347-360` — USB-CDC serial I/O runs on a dedicated Rust reactor thread (`std::thread::spawn`), not the Python reactor.
- `rust/host-rt/src/mcu_serial_conn.rs:118-120` — EtherCAT runs on a dedicated `ec-conn-reader` thread.
- `rust/host-rt/src/host_io/events.rs:324-326` — heartbeat callback fires inline on the serial-processing thread and sends `PumpMsg::Heartbeat` to the pump via an unbounded channel.
- `klippy/serialhdl.py:434` — the Python reactor's `_engine_event_poller` (1 ms timer) only drains Python-visible events; it does NOT touch heartbeats.

**Detail:** Heartbeat delivery to the pump is fully independent of Python
reactor scheduling and gcode processing load. **H2 is REFUTED.**

### Additional Findings

- `runtime_status_drain` 10 Hz fallback and `runtime_drain` 100 Hz rate-limited
  path are Confirmed (`runtime_tick.c:255-298`); heartbeats are abundant under
  normal streaming, so the failure mode is specifically *delivery* (TX-drop or
  response-timeout stall), not *generation*.
- The MCU TX buffer is small (320 B / 1024 B) and shared by heartbeats,
  `PushPiecesResponse`, status, faults, output, and `event_log` entries — a
  burst of any of these can evict a heartbeat.

### Updated Hypotheses

- **H1** Partially Confirmed (no upstream backpressure — Finding 1).
- **H2** REFUTED (Finding 6).
- **H3** Refuted as stated (heartbeats ARE generated; the failure is delivery,
  not generation). Superseded by H4.
- **H4 (NEW) Open** — Chronic MCU TX-buffer saturation drops heartbeats and/or
  `PushPiecesResponse` frames; the pump stalls at `StallFull` or in 5 s
  `send_frame` timeouts and cannot advance `retired`. Confirm with an
  instrumented repro showing `kalico_console_write_raw` returning `-1` during
  the stall while the pump is `StallFull`.

### Backlog Changes

- #2 (reactor starvation) → Done (refuted, Finding 6).
- #3 (credit_freed vs heartbeat) → Done (Finding 4: credit_freed is vestigial).
- #1, #5 still Open but subsumed by H4 instrumentation.
- NEW: instrument `kalico_console_write_raw` -1 returns and pump `StallFull`/
  `send_frame` timeout events during a long-gcode repro.
- **DEPRIORITIZED by user redirect (2026-06-21):** the TX-buffer / H4
  instrumentation thread is parked. First focus is a stream→pump
  consumption-bound. Revisit H4 only if the consumption-bound does not resolve
  the hang.

### Updated Conclusion

Structural root cause Confirmed (Findings 1-5): unbounded upstream channels let
the host race arbitrarily far ahead of the MCU; the only backpressure (piece
ring) is released by a single, silently-droppable heartbeat path; the
synchronous 5 s `send_frame` RPC can freeze the pump thread and delay credit
application. Exact runtime trigger (H4: TX-buffer saturation) is Hypothesized.

### User redirect (2026-06-21, post-council)

The TX-buffer/heartbeat-drop investigation (H4) is **deprioritized**. The user's
stated first focus: **add a limit on the stream keyed to how much the pump has
actually consumed** — bound the stream-planner→pump (and/or submit→stream)
path by pump-side consumption (ring room / pushed / retired), not by MCU
TX-buffer dynamics. The unbounded chain (F1) is the target. The council's
TX-drop instrumentation experiment is parked, not cancelled; revisit if the
consumption-bound does not resolve the hang.

### New follow-up section below captures the council and the redirect.
