# Investigation: Restart blocks for the remaining duration of in-flight motion after a crash

## Hand-off Brief

1. **What happened.** After the printer crashes mid-gcode it cannot restart for ~the remaining duration of the motion that was queued at crash time (a 10 s move ⇒ ~10 s of unavailability; a long print ⇒ power-cycle is faster). Observed on neptune-bench (which **is** EtherCAT in its X path).
2. **Where the case stands.** Two Confirmed host/engine code paths drain in real-time on restart (`gcode.py:464 request_restart → wait_moves`, gated by `DrainSync` `retired==sent`). One unresolved fork decides which path actually fires: whether the user's "crash" leaves the printer in **ready** state (then the Confirmed `wait_moves` path is the whole answer) or in **shutdown** state (`is_printer_ready` False ⇒ `wait_moves` is *skipped* ⇒ the blocker must be the endpoint respawn/handshake + surviving monotonic clock on the EtherCAT side).
3. **What's needed next.** Disambiguate the crash mode (MCU/printer **shutdown** vs **host-process** restart, and recovery via `FIRMWARE_RESTART` vs host service restart) — ideally by reading one real crash+restart event from the bench logs (`query-logs` / `mcu-diagnostics`). That single fact routes the whole diagnosis.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-29                                                                 |
| Status           | Active                                                                     |
| System           | kalico fork (Klipper→Kalico→fork), Rust motion engine + EtherCAT RT endpoint; neptune-bench (Neptune 3 Pro, X = A6-EC servo over EtherCAT) |
| Evidence sources | Source code (klippy/, rust/motion-engine, rust/ethercat-rt, rust/runtime); two Explore subagent traces |

## Problem Statement

User-reported, verbatim intent: "if printer crashes during executing some gcode, it can't boot back up while this gcode is supposed to be executing. so if it was some 10 second run, it wouldn't restart properly for those 10 seconds and if it was a long print - it's easier to just power cycle than to wait. happens on neptune-bench, not sure if it is the same for trident or not. so could be ethercat specific, or could be not."

Treated as hypothesis, not fact. The "ethercat-specific?" question is explicitly open.

## Evidence Inventory

| Source   | Status                          | Notes     |
| -------- | ------------------------------- | --------- |
| Host restart path (`klippy/gcode.py`, `klippy/motion.py`, `klippy/clocksync.py`) | Available | Traced; primary Confirmed path lives here |
| Rust drain semantics (`rust/motion-engine/src/{bridge,drain,pump}.rs`) | Available | `DrainSync` retired==sent gate confirmed |
| EtherCAT endpoint (`rust/ethercat-rt/src/...`) | Available | Buffer lifecycle, surviving monotonic clock, one-shot session confirmed |
| Real crash+restart event logs from neptune-bench | Missing | Would settle the ready-vs-shutdown fork definitively (`query-logs`/`mcu-diagnostics`) |
| Trident reproduction | Missing | Needed to confirm/deny EtherCAT-specificity (Trident H723+F446, not EtherCAT) |
| Exact "crash" definition from user | Partial | Need: MCU shutdown vs host-process death; recovery method |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Disambiguate crash mode + recovery method with user | High | Open | Decides which mechanism applies |
| 2 | Pull one real crash+restart event from bench logs | High | Open | `query-logs`/`mcu-diagnostics`; look for drain loop vs respawn-handshake timing, PieceStartInPast faults |
| 3 | Confirm `is_printer_ready` value during the observed crash | High | Open | If False, agent#1 path #1 (`wait_moves`) is bypassed — re-anchor on respawn/clock |
| 4 | Trace EtherCAT respawn handshake timing under a still-moving drive | Medium | Open | `ethercat-rt.rs:415-450` waits drive Ready (5 s); does CSP motion in-flight delay readiness? |
| 5 | Reproduce on Trident (non-EtherCAT) | Medium | Open | Confirms/denies EtherCAT-specificity |

## Timeline of Events

| Time | Event | Source | Confidence |
| ---- | ----- | ------ | ---------- |
| t0 | Print running; motion buffered ahead (stepper ring + EtherCAT per-axis rings; lead up to `MAX_LEAD_SECS=2.0` s + planner look-ahead) | `pump.rs:59,331`; `motion.py:132` | Confirmed |
| t1 | Crash (MCU shutdown / fault / host death — *mode unconfirmed*) | user report | Hypothesized |
| t2 | Restart requested | user report | Confirmed (symptom) |
| t2→t2+Δ | System unavailable for ~remaining queued-motion duration Δ | user report | Confirmed (symptom) |

## Confirmed Findings

### Finding 1: Graceful restart of a *ready* printer drains queued motion before exiting

**Evidence:** `klippy/gcode.py:464` `request_restart`:
```python
if self.is_printer_ready:
    ...
    toolhead.dwell(0.500)
    toolhead.wait_moves()          # blocks until queued motion drains
self.printer.request_exit(result)  # only after the wait
```
`cmd_RESTART` (`gcode.py:477`) and `cmd_FIRMWARE_RESTART` (`gcode.py:483`) both route here.

**Detail:** The wait runs only when `is_printer_ready` is True (set on `klippy:ready`, cleared on shutdown/disconnect, `gcode.py:156,274-286`). For a still-ready printer, restart blocks for the full queued-move duration before the process even begins to exit.

### Finding 2: "Drained" means the MCU/endpoint has physically retired every queued step (real-time)

**Evidence:** `rust/motion-engine/src/drain.rs:64-71` — drained ⇔ `retired - baseline == sent` per `(mcu, axis)`. `retired` fed only from MCU/endpoint heartbeat retired-counts (`bridge.rs:3109-3113`; ethercat `ethercat-rt.rs:1404-1423` → `pump.rs:560-568`). Host blockers: `motion.py:558 _wait_mcu_drained` (poll to `DRAIN_TIMEOUT=60s`, `motion.py:13`) and `motion.py:608 _drain_to_mcu_execution` (pause until `estimated_print_time() >= _mcu_pending_end_time`). `bridge.rs:3658 wait_moves_poll` additionally sleeps to a wall-clock `Instant` = finish time of the last move.

**Detail:** The drain is genuinely gated on real-time playout, so any restart/flush that hits it waits ~the in-flight motion duration. `flush_step_generation` (`motion.py:605`) is reached from `set_position` (`motion.py:244`), homing, extruder sync, idex — all of which run during a restart/home sequence.

### Finding 3: EtherCAT is in Neptune's motion path; its clock and buffer survive a host restart

**Evidence:** neptune-bench X = A6-EC servo on `[ethercat_node node_x]`/`[servo_x]` over `eth0`, socket `/tmp/kalico-ethercat.sock` (per `neptune-bench` skill; repo `config/printer-elegoo-neptune3-pro-2023.cfg` has zero ethercat sections — bench config is local). Endpoint clock = `CLOCK_MONOTONIC_RAW` at `ETHERCAT_CLOCK_FREQ_HZ=1_000_000_000` (`clock.rs:17-28`; `bridge.rs:92,2884-2894`) — resets only on Pi reboot. Piece rings cleared only on (a) socket disconnect=process exit, (b) `Command::Stop` (homing trip only, `bridge.rs:4687-4708`), (c) drive/sensorless fault. `invoke_shutdown` sends none of these (`ethercat_node.py:184-193`; `motion.py:230-235` discards only on `klippy:disconnect`).

**Detail:** So the bug *can* be EtherCAT-related. The asymmetry to scrutinize: the stepper side issues `runtime_reset` during config (`motion.py:1211-1220`); the EtherCAT endpoint has no equivalent discard on the shutdown/restart path — but the endpoint process also dies on socket close and respawns with empty rings, so whether stale buffer truly survives into the *new* session is not yet proven.

## Deduced Conclusions

### Deduction 1: For a *ready*-state restart, the block is fully explained

**Based on:** Findings 1 + 2.

**Reasoning:** `request_restart` → `wait_moves` → `DrainSync(retired==sent)` is a real-time drain executed before exit, only when `is_printer_ready`.

**Conclusion:** If the user's "crash" leaves the printer ready (host hang / manual restart of a healthy printer), this is the complete root cause and it is **not** EtherCAT-specific (shared drain gate).

### Deduction 2: For a *shutdown*-state crash, Finding 1's path is bypassed — a different mechanism is responsible

**Based on:** Finding 1's `if self.is_printer_ready:` guard + the fact that an MCU/printer shutdown clears that flag.

**Reasoning:** A true shutdown ⇒ `is_printer_ready` False ⇒ `wait_moves` is skipped during restart. Yet the symptom persists, so the blocker in that scenario must be elsewhere — most plausibly the EtherCAT endpoint respawn/handshake (waits drives to reach Ready, `ethercat-rt.rs:415-450`, 5 s) and/or the surviving monotonic clock interacting with buffered/stale pieces (possibly tripping `PieceStartInPast=-308`, `error.rs:187`).

**Conclusion:** The crash mode is the pivotal unknown; it selects between a generic-host cause and an EtherCAT-specific cause.

## Hypothesized Paths

### Hypothesis 1: User's "crash" is actually a ready-state restart → Finding 1 is the whole story

**Status:** Open
**Theory:** The printer never enters shutdown; the user restarts a still-ready (or hung-but-ready) printer, hitting `wait_moves`.
**Supporting indicators:** Symptom duration exactly tracks remaining motion; Finding 1 is an exact, Confirmed match.
**Would confirm:** Bench log shows `gcode:request_restart` + `wait_moves` drain loop with `is_printer_ready` True at restart time.
**Would refute:** Logs show `klippy:shutdown` before restart (flag already False).

### Hypothesis 2: Crash = MCU/printer shutdown; block is EtherCAT respawn/handshake + surviving clock

**Status:** Open
**Theory:** On shutdown the endpoint keeps clocking buffered rings against surviving `CLOCK_MONOTONIC_RAW`; respawn handshake / drive-Ready wait and/or stale-piece-vs-reset-clock interaction blocks the new session until real-time playout completes.
**Supporting indicators:** EtherCAT clock survives restart (Finding 3); no shutdown-path ring discard; handshake waits drive readiness.
**Would confirm:** Logs show respawn handshake stalling, or repeated `PieceStartInPast`, for ~the queued duration after a shutdown.
**Would refute:** New endpoint respawns clean (empty rings, immediate Ready) yet block persists — would point back to a host drain on the new instance.

### Hypothesis 3: EtherCAT-specific (does not reproduce on Trident)

**Status:** Open
**Theory:** The asymmetric missing ring-discard on the EtherCAT shutdown/restart path (vs stepper `runtime_reset`) makes this unique to the servo/EtherCAT axis.
**Supporting indicators:** Finding 3 asymmetry.
**Would confirm:** Trident (non-EtherCAT) does not reproduce.
**Would refute:** Trident reproduces identically ⇒ generic drain cause (Hypothesis 1).

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Crash mode (shutdown vs host death) + recovery method | Selects Hypothesis 1 vs 2; routes the entire diagnosis | Ask user; read one bench crash+restart event |
| `is_printer_ready` at restart time | Confirms/denies Finding 1 applies | `query-logs` around the restart |
| Whether stale EtherCAT buffer survives into the *new* session | Confirms/denies Hypothesis 2 mechanism | `mcu-diagnostics` on endpoint respawn; check ring state + clock seed |
| Trident reproduction | Confirms/denies EtherCAT-specificity (Hypothesis 3) | Repro on trident-bench |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Error origin | `klippy/gcode.py:464` `request_restart` → `toolhead.wait_moves()` (ready-state path) |
| Trigger | `RESTART` / `FIRMWARE_RESTART` while motion is queued/in-flight |
| Condition | Drain gate `retired==sent` (`rust/motion-engine/src/drain.rs:64-71`) only satisfied after real-time playout; `_wait_mcu_drained` (`motion.py:558`), `_drain_to_mcu_execution` (`motion.py:608`), `wait_moves_poll` (`bridge.rs:3658`) |
| Related files | `klippy/motion.py`, `klippy/clocksync.py`, `rust/motion-engine/src/{bridge,drain,pump}.rs`, `rust/ethercat-rt/src/bin/ethercat-rt.rs`, `rust/ethercat-rt/src/{server,clock}.rs`, `klippy/extras/ethercat_node.py`, `rust/runtime/src/{error,fault_helpers}.rs` |

## Conclusion

**Confidence:** Medium

Confirmed: a real-time drain gate (`retired==sent`) governs restart, and for a **ready**-state restart it blocks for the full queued-motion duration via `request_restart → wait_moves` — a complete, non-EtherCAT-specific explanation. Open: whether the user's "crash" actually leaves the printer ready (Hypothesis 1, exact match) or in shutdown (Hypothesis 2, where `wait_moves` is bypassed and an EtherCAT respawn/clock mechanism must account for the block). The single disambiguating fact — crash mode + recovery method, observable in one bench log event — settles which root cause stands and whether it is EtherCAT-specific.

## Recommended Next Steps

### Diagnostic
1. Confirm crash mode with the user (shutdown vs host-process death) and recovery method (`FIRMWARE_RESTART` vs host service restart).
2. Pull one real crash+restart event from neptune-bench logs (`query-logs` / `mcu-diagnostics`): check `is_printer_ready` at restart, presence of `klippy:shutdown`, the drain loop vs endpoint respawn-handshake timing, and any `PieceStartInPast` (-308) faults.
3. Reproduce on trident-bench to settle EtherCAT-specificity.

### Fix direction (deferred until root cause is pinned)
- If Hypothesis 1: restart should discard rather than drain in-flight motion on an explicit restart/abort (the user does not want to wait out motion they are abandoning) — a deliberate behavior change, not a silent recovery.
- If Hypothesis 2: add the missing EtherCAT shutdown/restart-path ring discard symmetric to the stepper `runtime_reset` (`motion.py:1211-1220`), and/or fix respawn so the new session does not wait on the dying endpoint's playout.

## Side Findings

- The stepper path discards via `runtime_reset` on config (`motion.py:1211-1220`); the EtherCAT path has no symmetric discard on the shutdown/restart path — an asymmetry worth closing regardless of which hypothesis wins.
- `DRAIN_TIMEOUT=60s` (`motion.py:13`): if a drain ever can't complete it stalls a full minute before timing out — relevant if a crash leaves `retired` permanently short of `sent`.

## Follow-up: 2026-06-30

### New Evidence

User clarified the crash mode and recovery behavior (Confirmed, user report):
- The crash is a **shutdown state** — reproducible by triggering **emergency stop (M112)**, or when the printer faults into shutdown on its own.
- After it, **neither restarting the klipper service nor `FIRMWARE_RESTART` responds** until the remaining motion time elapses.
- **Only a full reboot fixes it immediately.**

### Updated Hypotheses

- **Hypothesis 1 (ready-state `wait_moves`): REFUTED.** A shutdown clears `is_printer_ready` (`gcode.py:156,274-286`), so `request_restart` skips `wait_moves` (`gcode.py:464`). The block persists in shutdown state, so this is not the mechanism.
- **Hypothesis 2 (surviving timeline / EtherCAT respawn): ELEVATED — now primary.** "Only a reboot fixes it" is strong corroboration: a service restart and `FIRMWARE_RESTART` both leave `CLOCK_MONOTONIC_RAW` (and `/tmp`, incl. the endpoint socket) intact; only a boot resets them. The block is keyed to an absolute monotonic time = the queued-motion frontier, and scales with remaining motion. Refines to two sub-mechanisms to disambiguate:
  - **2a — old endpoint outlives the crash:** the EtherCAT endpoint process keeps running and clocking out its buffered rings after shutdown (no `Stop`/discard on the shutdown path, Finding 3); its one-shot session (`server.rs:14-17,43-44`) refuses the new klippy's connection until it finishes draining and exits — so reconnect hangs for ~the queued duration. Reboot kills it outright.
  - **2b — persistent absolute-time wait in the new session:** something the new process reads (a `/tmp` state artifact, the surviving endpoint/MCU clock, host-rt clock epoch) makes it wait until `now() >= frontier_end`. Reboot clears both the stored frontier and the clock.
- **Hypothesis 3 (EtherCAT-specific): supported but not yet proven** — pending Trident repro.

### Backlog Changes

- Promote backlog #2 (pull real M112+restart event from bench logs) and #4 (endpoint lifecycle under shutdown) to top priority.
- New: determine whether the EtherCAT endpoint process actually exits on socket EOF during a service restart, or lingers draining rings (decides 2a vs 2b).
- New: check for any state file under `/tmp` (besides the socket) that the endpoint/runtime persists a frontier/clock to.

### Updated Conclusion

Confidence Medium→ leaning the root cause is the EtherCAT endpoint/timeline surviving a shutdown (Hypothesis 2), not the host drain path. The reboot-only fix and duration-scaling are jointly diagnostic of an absolute monotonic-clock-keyed wait. Next: confirm 2a vs 2b via endpoint-lifecycle trace + one real log event.

## Follow-up: 2026-06-30 #2 — neptune-bench log evidence (VictoriaLogs)

### New Evidence (all Confirmed, queried from bench VL @ ethercatpi5.local)

1. **Crash mode in practice = MCU/transport fault → klippy auto-aborts → systemd respawn.** At shutdown: `motion backpressure: shutdown while draining (buffer_time=4.857s)` then `send_frame_fatal — pump send_mcu_frames FATAL transport error` then `reactor_exit_on_fault — transport closed via IO error on CRITICAL MCU; aborting klippy so systemd restarts it`. So the recovery process is already a **fresh klippy process** — confirming the host `wait_moves` path is irrelevant (it would need the *old* ready process).
2. **The proximate blocker is the EtherCAT servo failing PREOP→OP on restart.** Failed endpoint session `ec-2141`: `bringup_preop rc=0` (05:26:49) → `bringup_finish rc=-4` (05:26:57) with `al_state` slot0=**8 (OP)**, slot1=**2 (PREOP)** — one A6-EC slave stuck in PREOP, never reaches OP. `stage=finish_fail`.
3. **Failed OP bringup is a fixed ~8.5s timeout; restart is a retry loop.** Across 24h: every `rc=-4` finish is ~8–9s after its `bringup_start`; successes are ~3–7s. Recovery typically needs 1–3 failed attempts before a success (samples: 17:46–47 fail/fail/success ≈50s; 18:49–50 fail/fail/fail/success ≈90s; some gaps ~5min between user retries).
4. **Each attempt spawns a brand-new endpoint** (`ec-NNNN` increments, fresh `bringup_start`). The old endpoint is gone — it does NOT linger holding the socket.

### Updated Hypotheses

- **Hypothesis 2a (old endpoint outlives crash, blocks reconnect): REFUTED.** Logs show each restart spawns a fresh endpoint that runs its own bringup; no lingering endpoint.
- **Hypothesis 2b (new session waits on absolute surviving time): REFUTED as framed.** The wait is the EtherCAT OP-transition timeout + retry loop, not a monotonic-clock comparison. (The monotonic clock surviving restart is real but is not the blocking mechanism.)
- **Hypothesis 4 (NEW — primary, Confirmed proximate cause): EtherCAT servo PREOP→OP bringup fails after an unclean crash.** When klippy aborts mid-motion, the endpoint dies without (or before) cleanly disabling/resetting the A6-EC drive; the slave's AL state / sync-manager watchdog latches, and the **persistent IgH kernel master** (loaded once, survives klippy/systemd restarts) does not fully reset the slave. Bringup gets the slave to PREOP but the PREOP→OP transition times out (rc=-4) and retries. **A Pi reboot fixes it instantly because it reloads the IgH kernel master and power/state-cycles the slave AL machine to a clean INIT→OP** — this, not a monotonic clock, is why only a reboot works.

### Backlog Changes

- New (High): pin the exact code reason the PREOP→OP transition fails after unclean death — what the endpoint does to disable/reset drives on a crash vs clean exit, and whether the kernel master / slave AL error is ever cleared without reboot. (Code-trace agent in flight.)
- New (Medium): **test the duration-correlation claim directly.** Logs so far show recovery ≈ (retry count × ~8.5s), not a clean linear function of in-flight motion time; buffer_time at shutdown is ~always ~5s (look-ahead cap), so it cannot by itself explain a "long print ⇒ very long wait." Capture one crash with a known long/fast in-flight move and measure retries-to-recover to confirm or refute that harder in-flight motion ⇒ more OP-bringup retries.
- Demote: host `wait_moves`/`DrainSync` line of inquiry (Findings 1–2) — real code, but not the cause of this symptom.

### Updated Conclusion (Confidence: Medium-High on proximate cause)

Confirmed proximate cause: on a mid-print crash the printer auto-restarts (systemd), but the **EtherCAT A6-EC servo fails to re-enter OP** (slave latched at PREOP), so the new session loops on ~8.5s OP-bringup timeouts until the drive eventually recovers — and a Pi reboot short-circuits it by reloading the persistent IgH kernel master. The host-side `wait_moves` drain (original Findings 1–2) is **refuted** as the cause. Open: (a) the precise code mechanism for the failed OP transition + missing drive reset/fault-clear on crash; (b) whether retry count scales with in-flight motion (the user's duration-correlation). This is **EtherCAT-specific** — predict it will NOT reproduce on Trident.

## Follow-up: 2026-06-30 #3 — reconcile code-trace verdict vs bench logs (root cause)

### New Evidence

- **Code-trace subagent verdict = sub-mechanism 2a** (old endpoint orphaned on FIRMWARE_RESTART keeps master-clocking its `MONOTONIC_RAW` piece rings; new endpoint can't reach OP until the old one drains → duration hang). Reasoning chain is code-cited and internally coherent (M112 fires no `motion.py` `klippy:shutdown` handler; `printer.py:790 while True` keeps the OS process alive across `FIRMWARE_RESTART`; endpoint DC loop breaks only on SIGTERM/EOF, `ethercat-rt.rs:476-484`).
- **Bench logs REFUTE 2a empirically** (Confirmed):
  - Endpoint sessions are **strictly sequential, never concurrent** (24h `ec-NNNN` min/max lifetimes; the lone `ec-2122` 17:46→05:29 span is pid-reuse, an aggregation artifact). No orphan coexists with a new bringup.
  - The endpoint **dies at the crash**, not the motion duration later (`ec-3645` last log 07:50:07.608; shutdown 07:50:08.001).
  - The new endpoint **reaches PREOP `rc=0` and drives one slave to OP=8** — impossible if a prior process still owned the EtherCAT master.
- The agent's own caveat is the resolution: `klippy:disconnect → engine.shutdown() → release_mcu → SIGTERM` (`bridge.rs:1518-1543`) reaps the old endpoint each cycle, so the orphan path does not occur in practice.
- **Independently-confirmed code defect (agent §1, verified):** `klippy/motion.py:218-221` registers only `klippy:connect`/`klippy:disconnect`; **no `klippy:shutdown` handler**. `invoke_shutdown` (`printer.py:535-545`) fires only `klippy:shutdown` handlers, so on M112/self-shutdown the servo receives **no `Command::Stop`/discard** — the sole Stop path is the homing trip (`bridge.rs:4698`). Emergency stop therefore does not cleanly stop/reset the A6-EC drive.

### Updated Hypotheses

- **Hypothesis 2a: REFUTED by logs** (no concurrent endpoints; endpoint dies at crash; new endpoint becomes master). Kept on record per "hypotheses are never deleted."
- **Hypothesis 4 (EtherCAT OP-bringup fails after unclean crash): CONFIRMED as proximate cause.**
- **Hypothesis 5 (NEW — likely upstream cause, Confirmed code defect): M112/shutdown does not stop or fault-reset the servo.** No `klippy:shutdown` handler in `motion.py` ⇒ the drive is abandoned in OP/CSP at crash, latches (PDO/sync watchdog or unresolved CiA-402 state), and then fails the PREOP→OP transition on every restart attempt until it self-clears or the kernel master is reloaded (reboot). This links the confirmed defect (no clean stop) to the confirmed symptom (OP-bringup failure).

### Updated Conclusion (Confidence: Medium-High)

Root cause chain: **mid-motion crash/M112 abandons the EtherCAT servo without a clean stop/fault-reset** (confirmed defect: no `klippy:shutdown` handler in `motion.py`) → the A6-EC slave latches → **every restart's PREOP→OP bringup times out (`rc=-4`, slave stuck at PREOP)** → ~8.5s-timeout retry loop until the drive self-clears; **a Pi reboot fixes it by reloading the persistent IgH kernel master** (forces clean INIT→OP). The original host `wait_moves` drain (Findings 1–2) and the orphan-endpoint theory (2a) are both refuted. EtherCAT-specific — predicted not to reproduce on Trident.

Remaining uncertainty: the precise latch (drive-side PDO/sync watchdog vs IgH master state) and whether retry count scales with in-flight motion energy. Neither blocks the fix direction.

### Fix direction (diagnosis only — not implemented)

1. **Primary [IMPLEMENTED 2026-06-30]:** a `klippy:shutdown` handler now issues `Command::Stop`/discard to the EtherCAT endpoint so M112/shutdown halts the servo immediately. Implemented in `klippy/extras/ethercat_node.py` (the node owns the handle; supports multi-node) rather than `motion.py`. Wiring: `ethercat_node._handle_shutdown` → `motion_engine.stop_node` → `bridge.rs stop_node` (PyO3) → `servo_torque::send_stop` → `MessageKind::Stop` → endpoint ring discard. Fails loudly (raises on non-zero endpoint result / transport error; `invoke_shutdown` logs it via `logging.exception`). Also fires on drive-fault self-shutdown (`_poll_drive_fault → invoke_shutdown`). Tests: `servo_torque::tests::stop_*` (round-trip, nonzero-result, transport-error, wrong-kind). `ci.sh quick` green; needs a Neptune flash to verify on hardware.
2. **Hardening:** in the bringup path (`claim_ethercat_node` / `ethercat-rt` OP transition), perform an explicit CiA-402 fault-reset / force a full INIT→OP cycle so a latched drive recovers without a reboot; and fail loudly (clear error) after N OP-timeouts instead of a silent retry loop.

### Diagnostic to close remaining uncertainty

Controlled repro (needs motion-command permission): home, start a single known long/fast move, trigger M112 mid-move, then FIRMWARE_RESTART; measure OP-bringup retry count vs the in-flight move's remaining time. Repeat with the servo idle (crash between moves) — expected to recover cleanly, isolating "mid-motion latch" as the trigger.
