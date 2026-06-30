# Investigation: EtherCAT endpoint "death" mid-print → pump stall → collateral -308

## Hand-off Brief

1. **What happened.** Mid-print the host motion pump freezes for ~1.7 s; **both** the MCU (axis 3) **and** the EtherCAT servo endpoint (slot 0) then latch the *same* `-308 PieceStartInPast` on stale pieces — the endpoint is a **co-victim, not the cause**. The "endpoint died / no connection error" framing is fully explained: it's a motion `-308`, not a link loss.
2. **Where the case stands.** Root cause is the **~1.7 s pump stall** that delivers stale pieces to both transports. The locus of the stall (MCU-serial send vs servo-socket send vs upstream/lock) is the one open fork. The endpoint's fault + exit reasons go to **klippy stderr/journal** (`journalctl -u klipper`), not VL — which is why VL looked "silent."
3. **What's needed next.** `#1` (per-send wall-time around `send_mcu_frames`) is **implemented** and will name the stalled transport on the next crash. Flash + reproduce, then read `pump_send_blocked`.

## Case Info

| Field            | Value |
| ---------------- | ----- |
| Ticket           | N/A |
| Date opened      | 2026-06-30 |
| Status           | Active |
| System           | Neptune 3 Pro servo bench `dderg@ethercatpi5.local`; Pi 5 (RP1 GEM NIC); branch `clock-sync` @ `6b096b4fc`; X axis = A6-EC EtherCAT servo (IgH backend), Y/Z/E steppers on F401 MCU |
| Evidence sources | VictoriaLogs (host-py, host-rust, host-ec, mcu); coredumps `~/printer_data/logs/coredumps/`; `pump_piece_submit` diagnostic (live); source `rust/ethercat-rt`, `rust/motion-engine/src/pump.rs` |

## Problem Statement

User: the EtherCAT servo endpoint "dies" abruptly mid-print and the print crashes the same way it did before the clock-sync work (so clock-sync is not the fix). Key user clue: **the A6-EC servo drives show no connection error** when this happens → likely not physical EtherCAT link instability. Goal: find why the endpoint dies and propose instrumentation. (Part A — fast fail-loud + suppress the `-308` red herring — is owned by a parallel session; out of scope here.)

## Evidence Inventory

| Source | Status | Notes |
| ------ | ------ | ----- |
| VL host-ec telemetry | Available | "per-slot drive telemetry" every 0.5 s; last at 12:22:33.466 then silent (crash #2). Endpoint alive through the pump stall. |
| VL pump_piece_submit | Available | Live since flash. No submits 12:22:32.0→33.699; burst at 33.699/33.704 then `-308`. Submitted pieces are contiguous (`gap:0`), well-formed. |
| VL host-rust motion/bridge | Available | `send_frame_transient` at 33.692; `EXIT_ON_FAULT` broken-pipe + "endpoint died" at 33.902. |
| MCU fault | Available | `fault_code 65228` = `-308`; `fault_detail 262143` = axis 3, deficit saturated 0xFFFF (≥65 ms). |
| Coredumps | Partial | Only `push-pieces-pum` (klippy pump SIGABRT, deliberate) and `python`. **No `ethercat-rt`/endpoint core.** One historical `ec-heartbeat-po` SIGBUS (klippy-side thread, execfn python, corrupt stack). |
| Endpoint internal state at stall | **Missing** | Why the endpoint stops servicing the socket / why the pump send blocks — not logged. |
| Which send blocks (MCU serial vs EtherCAT socket) | **Missing** | No per-send timing instrumentation yet. |

## Timeline of Events (crash #2, 2026-06-30, all UTC)

| Time | Event | Source | Confidence |
| ---- | ----- | ------ | ---------- |
| ≤32.0 → 33.699 | Pump emits **zero** piece submits (~1.7 s) — pieces queued but not sent | pump_piece_submit gap | Confirmed |
| 32.466 / 32.966 / 33.466 | Endpoint logs drive telemetry normally — **alive during the pump stall** | host-ec | Confirmed |
| 33.466 | Last endpoint telemetry; endpoint goes silent thereafter | host-ec | Confirmed |
| 33.692 | `send_frame_transient` — pump send_mcu_frames first failure | host-rust motion | Confirmed |
| 33.699 / 33.704 | Pump finally submits axis 0/1 (servo) + 2/3 (stepper), contiguous | pump_piece_submit | Confirmed |
| 33.707 | MCU `-308` (axis 3, deficit saturated) | mcu fault | Confirmed |
| 33.902 | `EXIT_ON_FAULT` EtherCAT broken-pipe → pump abort | host-rust bridge | Confirmed |

## Confirmed Findings

### Finding 1: The endpoint was alive while the pump was frozen
**Evidence:** host-ec telemetry at 12:22:32.466/32.966/33.466 overlaps the pump-submit gap (32.0→33.699).
**Detail:** Refutes "pump hung on a dead endpoint socket." The endpoint's logging (and presumably RT) threads ran throughout most of the freeze; it only went silent at 33.466, near the *end* of the stall.

### Finding 2: The pump stalled with pieces ready, not idle
**Evidence:** `-308` deficit saturated ≥65 ms on a *contiguous, well-formed* piece (`gap:0`); pump emitted nothing for ~1.7 s then burst-submitted.
**Detail:** A drained/idle pump would underrun, not `-308`. `-308` means pieces were queued (start_times computed earlier) but delivered ~1.7 s late → the **send path blocked**, the planner did not malform pieces.

### Finding 3: The endpoint does not crash with a signal
**Evidence:** No `ethercat-rt`/endpoint coredump in `~/printer_data/logs/coredumps/` across multiple crashes; only klippy `push-pieces-pum`/`python` cores. `ps` shows the endpoint alive post-restart (`EtherCAT-IDLE` PID 7778).
**Detail:** A signalled crash (SIGSEGV/SIGABRT) would coredump under the system `core_pattern` (the klippy pump does). Its absence ⇒ the endpoint **hangs or exits cleanly**, not crashes.

### Finding 4: The servo endpoint latches the SAME `-308`, on the servo ring — it is a co-victim
**Evidence:** `journalctl -u klipper` @ 14:22:33 (local): `ec-rt: FAULT latched on slot 0 fault_val=0xfffffecc code=0xfecc — notifying host via heartbeat`. `0xfffffecc` = −308 (i32), `0xfecc` = −308 (i16). The endpoint runs a piece ring with the runtime `FaultCode` and implements `piece_start_in_past` (`rust/ethercat-rt/src/curves.rs:30`); the ring fault is latched at `rust/ethercat-rt/src/bin/ethercat-rt.rs:1245`.
**Detail:** The endpoint did not lose its link or crash — it latched `PieceStartInPast` on a **stale servo piece**, exactly as the MCU did on a stale stepper piece. Both faults are the single ~1.7 s pump stall delivering late pieces to both transports. This is why the servo shows "no connection error" (user) — it's a motion fault, not comms.

### Finding 5: The endpoint's exit/fault reasons go to stderr/journal, not VL
**Evidence:** `spawn_ethercat_endpoint` (`rust/motion-engine/src/bridge.rs:490-503`) spawns with inherited stdio (no stderr redirect); the endpoint's exit paths use `eprintln!` (`ethercat-rt.rs:478,482`, plus `std::process::exit(1)` config paths). `ec-rt:` lines appear in `journalctl -u klipper`.
**Detail:** VL (`source=host-ec`) only carries the endpoint's *structured* `tracing` events (telemetry); its plain-`eprintln!` exit/fault lines are in klippy's journal. "Silent in VL" ≠ "died silently." After the fault the endpoint exits via "bridge disconnected" once klippy aborts — a clean downstream exit (no coredump).

## Hypothesized Paths

### Hypothesis 1: Physical EtherCAT link instability
**Status:** Refuted
**Theory:** Cable/PHY/link drop kills comms.
**Resolution:** Refuted by Findings 1 & 4. The endpoint logged telemetry through the stall and then latched a motion `-308` (PieceStartInPast) on the servo ring — not a comms/link fault. No connection error on the drive. Closed.

### Hypothesis 6: The pump stall is the single root; both `-308`s are downstream
**Status:** Confirmed (mechanism), locus Open
**Theory:** A ~1.7 s stall in the single-threaded pump delays piece delivery to both transports; the servo ring and the MCU each latch `PieceStartInPast` on the resulting stale pieces.
**Resolution:** Confirmed by Findings 2 (pump frozen with pieces queued) + 4 (both sides latch −308). **Still open:** *where* the pump stalls — H2 (servo socket write), H3 (MCU serial write), or upstream (lock/scheduler/intake). `#1` instrumentation resolves this on the next crash.

### Hypothesis 2: The endpoint's host-socket-service path stalls while RT + telemetry threads keep running
**Status:** Open (strong)
**Theory:** The endpoint thread that drains `/tmp/kalico-ethercat.sock` (commands from klippy) blocks/stalls; the socket's receive buffer fills; klippy's pump `write()` blocks ~1.7 s. The RT cyclic thread keeps the servo alive (no servo error) and the telemetry thread keeps logging (until it too stalls at 33.466).
**Supporting indicators:** Endpoint alive during stall (F1); no servo error; no coredump (F3); pump blocked-not-idle (F2); single-threaded pump blocks on a full socket write.
**Would confirm:** Endpoint-side per-thread liveness showing the socket-service loop stalled while RT/telemetry ran; pump-side timing showing `send_mcu_frames` for the **servo mcu_id** blocked.
**Would refute:** Pump timing showing it blocked on the **MCU serial** send (→ H3), or endpoint socket-service loop running normally during the stall.

### Hypothesis 3: The pump blocked on the MCU serial write, not the EtherCAT socket
**Status:** Open
**Theory:** The single-threaded pump blocked on the USB-serial write to the F401 MCU (buffer full / MCU not draining), starving everything; the servo stayed happy because its RT thread holds the last setpoint and the endpoint was never the bottleneck.
**Supporting indicators:** `-308` is on a **stepper** axis (axis 3); endpoint was demonstrably fine (F1).
**Would confirm:** Per-send timing showing the MCU-bound `send_mcu_frames` call blocked ~1.7 s; MCU serial backpressure/flow-control evidence.
**Would refute:** Pump timing showing the MCU send returned fast and the servo send blocked.

### Hypothesis 4: Endpoint logging/observability back-pressure stalls it
**Status:** Open
**Theory:** `host-ec.jsonl` is ~182 MB; if the endpoint's tracing/observability writes synchronously (disk or a bounded shipper channel that fills), a logging stall could block whatever thread logs inline (incl. socket service), explaining the silent gap + socket starvation.
**Would confirm:** Endpoint stall correlates with a log-flush/disk event; moving logging off the hot path removes the stall.
**Would refute:** Endpoint logging is already async/non-blocking and decoupled from socket service.

### Hypothesis 5 (historical): the `ec-heartbeat-po` SIGBUS is a separate klippy-side defect
**Status:** Open (parked)
**Theory:** A klippy-side ethercat heartbeat/poll thread SIGBUS'd (corrupt stack) in an earlier session — possibly unrelated to the recurring stall, possibly a consequence of the socket peer vanishing.
**Would confirm:** Symbolicated backtrace against the matching `_motion_engine`/`host-rt` build.
**Note:** Lower priority; the recurring failure (crashes #1/#2) is the silent stall, which leaves no endpoint core.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Which send blocks (MCU serial vs EtherCAT socket) | Decides H2 vs H3 — the whole direction | Instrument `pump.rs`: log wall-time around each `send_mcu_frames`/`send_frame` with `mcu_id`; warn if > a few ms |
| Endpoint per-thread liveness during the stall | Confirms H2 (which endpoint thread stalls) | Endpoint-side: per-thread iteration counters/heartbeats (RT loop, socket-service, telemetry) logged on a stall watchdog |
| Why the endpoint goes silent at all | Root cause | Endpoint self-watchdog thread that logs "loop X stalled N ms" + a SIGQUIT/backtrace-on-stall handler |
| Whether the endpoint exits or just hangs | Crash vs hang | Log endpoint exit reason on every exit path; confirm process PID persists across the event (hang) vs respawns (exit) |

## Proposed Instrumentation (the user's ask)

1. **Pump send timing (host side, `rust/motion-engine/src/pump.rs`).** Bracket each `sink.send_mcu_frames(mcu_id, …)` with a monotonic clock; emit `event=pump_send_blocked, mcu_id, elapsed_ms` when it exceeds ~5 ms. This immediately tells us **which** transport (servo socket vs MCU serial) ate the 1.7 s — settling H2 vs H3 on the next crash. (Coordinate with the parallel "Part A" session, which is already touching the send/fatal path.)
2. **Endpoint per-thread liveness (`rust/ethercat-rt`).** Each long-lived loop (RT cyclic, host-socket service, telemetry) bumps an `AtomicU64` iteration counter; a dedicated watchdog thread logs `event=ec_thread_stall, thread, last_advance_ms` when any counter stops advancing > N ms. This captures the silent gap that current logging misses.
3. **Endpoint stall backtrace.** Install a SIGQUIT (or watchdog-triggered) handler that dumps all thread backtraces to the structured log, so a hang yields a stack even without a coredump.
4. **Endpoint exit-reason logging.** Log on every exit path (and panic hook) so a clean exit can no longer be silent; confirm whether the process hangs (PID persists) or exits.

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Per-send timing in `pump.rs` to settle H2 vs H3 | High | Open | Smallest change, highest discriminating power |
| 2 | Endpoint thread-liveness watchdog in `ethercat-rt` | High | Open | Captures the silent endpoint gap |
| 3 | Read `ethercat-rt.rs` main loop + socket-service threading model | High | Open | Source trace: is socket service on its own thread? shared locks with RT? |
| 4 | Endpoint logging path — sync vs async (H4) | Medium | Open | 182 MB host-ec.jsonl; check for inline/blocking writes |
| 5 | Symbolicate historical `ec-heartbeat-po` SIGBUS (H5) | Low | Open | Possibly separate defect |

## Conclusion (interim — Medium confidence)

The EtherCAT endpoint is **exonerated as the cause**. The crash is a single **~1.7 s pump stall** that delivers stale pieces to both transports; the servo ring (slot 0) and the MCU (axis 3) independently latch the *same* `-308 PieceStartInPast`. "Endpoint died / no servo connection error" is fully explained: a motion fault, surfaced via the heartbeat to the host and (downstream) a clean "bridge disconnected" exit once klippy aborts. Confidence is **Medium** only because the **stall locus** (which send, or upstream) is not yet observed — that's the last gap, and `#1` will close it deterministically on the next reproduction.

**Instrumentation status:**
- `#1` per-send timing (`pump_send_blocked{mcu, elapsed_ms}` past 5 ms) — **implemented** in `rust/motion-engine/src/pump.rs` (this session, pending flash). Names the stalled transport (servo socket vs MCU serial) and, if no slow send fires while pieces are pending, points the finger upstream (lock/scheduler/intake).
- `#2` endpoint per-thread liveness — **deprioritized**: the endpoint already reports its fault + exit reason to the journal (Finding 5); the cheaper win is routing those `eprintln!` lines into the structured log so VL carries them. Not blocking.
- `#1b` (candidate, if `#1` shows no slow send) — a pump-loop liveness heartbeat: warn when the loop has pieces pending but has emitted nothing for > N ms, to catch an *upstream* stall (lock/scheduler) rather than a transport block.

## Reproduction / verification plan

1. Flash `clock-sync` (with `#1`) to the bench; run the failing print.
2. On crash, query: `event:=pump_send_blocked _time:1h | sort by (_time)` → read `mcu` + `elapsed_ms`.
   - High `elapsed_ms` on the **servo** mcu_id → H2 (servo socket write blocks).
   - High `elapsed_ms` on the **MCU** mcu_id → H3 (MCU serial write blocks).
   - **No** `pump_send_blocked` near the fault → upstream stall; add `#1b`.
3. Cross-check `journalctl -u klipper | grep 'ec-rt:'` for the endpoint's own fault/exit lines around the crash.

## Follow-up: 2026-06-30 #2 — guard + log wiring implemented (this session)

- **Pump in-past guard** (`rust/motion-engine/src/pump.rs`, pre-send): before `send_mcu_frames`, compare each piece `start_time` against the MCU's projected clock (`mcu_clock_of`); if a piece is past by > `PUMP_PAST_GUARD_SECS = 500us` (margin over the MCU's 200us threshold, above host-projection jitter), emit `event=pump_piece_in_past{mcu,axis,start_time,mcu_now,deficit_us}` (VL) + `eprintln!` (journal) + flush + `std::process::abort()`. The host now fails loud with the offending context instead of the MCU/servo-endpoint tripping a cryptic `-308` after the fact. Fixed two unit tests that deliberately sent stale (`start_time:1`) probe pieces.
- **Endpoint eprintln → structured** (`rust/ethercat-rt/src/bin/ethercat-rt.rs`): the FAULT-latched site now also emits `event=ring_fault_latched{slot,fault_val,fault_code}`, and both exit paths emit `event=endpoint_exit{reason=sigterm|bridge_disconnected}` — so VL carries the endpoint's fault + exit reason (previously journal-only via `eprintln!`).
- **Validation:** `cargo nextest -p motion-engine -p ethercat-rt` = 579 passed; `ci.sh rust-clippy` clean.
- **Coordination:** the in-past guard overlaps Part A's fail-fast intent (`spec-ethercat-endpoint-death-failfast.md`); reconcile so the host-side `-308` pre-emption isn't duplicated. Not committed (shared worktree with Part A's in-progress edits).

## Follow-up: 2026-06-30 #3 — locus CONFIRMED on the MCU serial; EtherCAT fully exonerated

New crash 13:29:22 with the instrumentation (`a1e049083`) flashed. Decisive evidence:

- **Host guard pre-empted the crash (user's fix works).** `pump_piece_in_past{mcu=0, axis=2, deficit_us=3_009_537 (~3.0 s), mcu_now=43682219819, start_time=43429412953}` fired and aborted the host. **No MCU `-308`, no servo `FAULT latched`** in the window — the cryptic dual-`-308` was replaced by one clear host fault. (Confirmed, VL.)
- **The stall is the MCU serial, not EtherCAT.** The blocking send was `mcu=0` = `McuHandle(0)` = the stepper MCU (USB serial @ 500000); `pump_send_blocked{mcu=0, elapsed_ms=5000, ok=false}` — a **5 s blocking send that timed out**. The error is generic `send_frame_transient "pump send_mcu_frames failed"` (not "ethercat PushPieces"). (Confirmed.)
- **EtherCAT was healthy throughout.** `host-ec` telemetry kept its 0.5 s cadence across 13:29:20.3 → 21.8 — the servo endpoint never stalled. H1/H2 fully refuted; **H3 (MCU serial) CONFIRMED**.
- **`pump_send_blocked` distribution:** all slow sends are `mcu=0` (n=19996/10m, avg 5.85 ms, max 5000 ms). The MCU serial send routinely runs ~5–6 ms (so the 5 ms threshold is noisy — raise to ~50 ms); the catastrophic outlier is the 5 s timeout.
- **MCU-side correlates:** host `kalico_stream_error ×3` at 13:29:17 (serial framing/CRC errors) ~5 s before the timeout; the MCU emitted **no** structured events during the stall (silent — logs ride the dead serial). Recurring background faults: `runtime.mcu_reset (cause bits=335544322=0x14000002, iwdg_resets=0)` + `runtime.fg_freeze (pc=134252664, stall_ticks=5)` at 13:21:08 (also seen 10:48). The MCU **firmware foreground-freezes / resets**.

### Updated conclusion (High confidence on locus)
The crash is an **MCU serial stall** (`McuHandle(0)`, USB serial): the link hangs ~5 s with host-side stream errors while the MCU goes silent; the single-threaded pump blocks on that send and then tries to submit a ~3 s-stale piece. The EtherCAT servo endpoint is **not involved** (healthy telemetry throughout) — it was a red herring; in earlier crashes both the servo ring and the MCU latched `-308` only because the *same pump stall* fed both stale pieces. The host guard now converts this into one clear `pump_piece_in_past` fault instead of dual `-308`s.

### Next root-cause layer (new investigation, MCU-side)
Why does the MCU serial stall / the MCU firmware freeze? Decode with the `mcu-diagnostics` skill:
- `runtime.fg_freeze pc=134252664` (=0x80056F8) → addr2line against the flashed MCU ELF: what code holds the foreground loop for 5 ticks.
- `runtime.mcu_reset cause bits=0x14000002, iwdg_resets=0` → decode the reset-cause flags (not IWDG).
- `kalico_stream_error` framing/CRC errors → USB-serial link integrity (CH340 `1a86 USB Serial`, cable, USB host on Pi 5) vs MCU-side TX stall.
- Distinguish **H3a** (MCU firmware freeze → stops servicing serial → host send blocks) from **H3b** (USB serial link glitch, MCU otherwise fine).

## Follow-up: 2026-06-30 #4 — link vs MCU vs our-recovery; key clue "Klipper worked on this port"

- **USB link IS independently flaky:** kernel log shows the CH341 (`1a86:7523`) physically disconnect+re-enumerate at 13:20:27 and 11:54:41 UTC (device# 5→6→7). The 13:21:08 "mcu_reset" was caused by that disconnect (klippy reconnect-reset, SFTRST), not an MCU crash. (Confirmed, `journalctl -k`.)
- **But the main recurring stall (12:22, 13:29) had NO disconnect**, no `fg_freeze`, `iwdg_resets=0`, no `hard_fault` — MCU shows no sign of freezing. The `fg_freeze pc=0x8008878` (`readb` in `run_tasks`, 250 µs) is a minor, separate hiccup.
- **User clue:** stock Klipper ran fine over this exact cable/port → the link glitches are likely *tolerable*; the suspect is **our recovery**.
- **We DO have retransmission** (`host_io/reactor.rs` `UnackedWindow`/seq/ACK/NAK + `host_io/rtt.rs` `RttEstimator`). So it's not "no retries." Prime suspect: **`MIN_RTO = 500 ms` → `MAX_RTO = 5 s`** (`rtt.rs:7-9`, floor raised for Renode). Klipper retransmits in ~tens of ms; our 500 ms first-retransmit + exponential backoff matches the observed 5 s stall on a brief glitch. Secondary: frame-parser **desync without resync** (`kalico_stream_error`, `reactor.rs:836-843`, logged with no detail).
- Hardware change this session: cable moved from USB2.0 (beside an ST-Link v2 — EMI) to USB3 on the Pi 5. Single cable (good, ferrited).

### Instrumentation to add (confirm the theory; distinguishes link-dead vs our-recovery)
1. **RX byte liveness during an in-flight stall** (decisive) — in the read path (`reactor.rs poll_serial` / `SerialFrameIo`), track bytes-read + last-byte instant; when an unacked frame is overdue, emit `rx_liveness{bytes_since_send, gap_ms}`. **Bytes flowing but no ACK ⇒ our parser/RTO is the problem (link alive); RX silent ⇒ link/MCU dead.**
2. **Retransmit/ACK trace** — at the retransmit scheduler emit `retransmit{seq, attempt, rto_ms, unacked_depth}`; at ACK/NAK receipt emit `mcu_ack{seq}`/`mcu_nak{seq}`. Reveals whether retransmits fire, the RTO ladder (500 ms→1→2→4 s?), and when the MCU finally ACKs — directly tests the slow-RTO hypothesis.
3. **Stream-error detail + resync** — upgrade `kalico_stream_error` to carry the error *kind* (framing/CRC/length/desync) + bytes discarded, and emit `stream_resync{bytes_skipped}` when a valid frame is regained. Shows desync→resync latency vs Klipper.

Decision matrix on next crash: (1) RX silent → link/MCU dead; (1) RX alive + (2) retransmits firing, ACK lands after multi-hundred-ms RTO → slow-RTO is the cause (fix: lower MIN_RTO / faster retransmit like Klipper); (1) RX alive + (3) desync never resyncs → parser-resync bug.

## Follow-up: 2026-06-30 #5 — stream-error detail = genuine wire corruption; retransmit probe added

- **`kalico_stream_error` already carries the kind, and it's wire corruption:** `kalico crc mismatch ch=0 expected=0x0512 actual=0x1412`, `klipper bad trailer 0x01/0x18` (VL `error` field). So during the stalls the MCU **is** sending bytes (RX alive) but they arrive **CRC-corrupted / mis-framed** — the link mangles data (classic CH340-under-motion-EMI). Our CRC correctly rejects them → no valid ACK → RTO retransmit → corruption persists → 5 s `pump_timeout` → host guard fires.
- **This is not "no retries" and not "slow RTO":** retransmit is timeout-driven (`reactor.rs:1349`), and hitting the full 5 s = ~4 resends all met with corruption. Lowering `MIN_RTO` won't help (confirmed reasoning).
- **Leading conclusion (updated): physical USB-serial link corruption** (CH340 `1a86:7523` under stepper/servo EMI). Independent kernel evidence of full disconnects (#4) + CRC corruption here. "Klipper worked" is consistent with a lower corruption rate then (different port/EMI) or Klipper tolerating it; not yet A/B-proven.
- **Probe added** (`reactor.rs`, RTO-fire site): `retransmit_timeout{front_seq, unacked_n, rto_ms, gap_since_recv_ms}`. `gap_since_recv_ms` counts corrupt frames as inbound, so on the next stall: small/steady gap = link alive-but-corrupt (link layer); large/growing = truly silent (MCU/link dead). Plus the RTO ladder confirms sustained vs brief.
- **Mitigation in flight (user):** cable moved USB2.0 (next to ST-Link v2) → USB3 on the Pi 5; single ferrited cable. Next print tests whether the corruption rate drops.

### Practical next steps
1. Reproduce post-USB3-move; compare `kalico_stream_error` rate + `retransmit_timeout` `gap_since_recv_ms` vs before.
2. If corruption persists: it's the link — better cable routing away from motor/servo wiring, ferrites, a different USB-serial bridge, or shielding. (Software can't un-corrupt the wire; the guard + fail-loud already prevent the cryptic -308.)
3. Only if `gap_since_recv_ms` shows the link recovers mid-window would faster/different retransmit be worth it.
