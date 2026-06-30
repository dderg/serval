# Investigation: Neptune mid-print motion stall → klippy abort → wedged until power-cycle

## Hand-off Brief

1. **What happened.** During a pure-linear print, the host loses its serial link to the critical F401 MCU (wire-corruption then `BrokenPipe`), the `host-rt` reactor takes the deliberate `EXIT_ON_FAULT` path and `std::process::abort()`s klippy (SIGABRT coredump); on systemd restart the EtherCAT X servo fails bring-up, so the machine stays dead until a power-cycle. **Confirmed.**
2. **Where the case stands.** Failure chain is Confirmed end-to-end from coredump + structured logs across three episodes today (19:47, 20:08, 20:33 CEST). The arc-fitting hypothesis is **Refuted** (repro is a Voron cube with `enable_arc_fitting=0`, 0×G2/G3). Root *trigger* — what corrupts the wire / schedules the step "Timer too close" — is narrowed to the motion pump junction path (`projection_divergence`) but the exact mechanism is not yet Confirmed.
3. **What's needed next.** Read `rust/motion-engine/src/pump.rs` junction/seam emission + `check_junction_position_continuity`, and correlate a single `junction_jump_anomalous` seam with the next `kalico_stream_error`/`Timer too close` to prove or kill the pump→wire causal link.

## Case Info

| Field            | Value                                                                                  |
| ---------------- | -------------------------------------------------------------------------------------- |
| Ticket           | N/A                                                                                    |
| Date opened      | 2026-06-29                                                                             |
| Status           | Active                                                                                  |
| System           | Neptune 3 Pro bench; Pi5 `ethercatpi5.local`; branch `neptune-crash-no-arcfit`. F401 main MCU over CH340 USB-serial (`usb-1a86_USB_Serial` @500000), X axis = A6-EC EtherCAT servo |
| Evidence sources | 3× klippy python coredumps (`~/printer_data/logs/coredumps/`), VictoriaLogs structured logs, `rust/motion-engine/src/pump.rs`, `rust/host-rt/src/host_io/mod.rs` |

## Problem Statement

"During a print the Neptune just stops moving, then pretends like it's still printing until eventually showing an error that the buffer hasn't been consumed for 60 seconds. It remains unresponsive after until I basically power cycle. On the last crash I power cycled even before it showed an error." User's working theory (branch name): arc-fitting is implicated.

## Evidence Inventory

| Source   | Status     | Notes     |
| -------- | ---------- | --------- |
| klippy coredumps ×3 | Available | `core.python.{2170,2424,3009}` @ 20:08/20:11/20:33 CEST. No debug symbols in venv python, but Rust frames symbolized. |
| VictoriaLogs / structured logs | Available | VL healthy (`/health`=OK, victorialogs+vector active). host-rust/host-py/host-ec/mcu sources all present. |
| `pump.rs` junction source | Available | Emit site for `junction_jump_anomalous` located. |
| `host_io/mod.rs` abort site | Available | Line 382 = the `std::process::abort()`. |
| Kernel USB log (dmesg) | Partial | `dmesg` grep for ch341/disconnect returned empty — did NOT confirm a kernel-level USB unplug. Needs re-check (ring may have rotated post-power-cycle). |
| Reproduction gcode | Available | `Voron_Design_Cube_v7...gcode`: 0×G2/G3, `enable_arc_fitting = 0`. |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Prove/kill pump junction (`projection_divergence`) → wire-corruption / Timer-too-close causal link | High | Open | The one remaining gap to root cause |
| 2 | Decode MCU reset cause `335544322` (=0x14000002) and confirm `fg_freeze pc=0x8008878 stall_ticks=5` is benign boot forensics vs trigger | Medium | Open | Resets look host-commanded (soft reset on reconnect), not spontaneous |
| 3 | Confirm whether CH340 physically re-enumerates (dmesg with persistent journal) vs MCU-side stream desync | Medium | Open | dmesg was empty; distinguishes HW link vs firmware framing bug |
| 4 | EtherCAT `bringup_fail`/`Config error` on restart — why X servo can't re-init without power-cycle | Medium | Open | This is the "unresponsive until power cycle" half |

## Timeline of Events (18:33Z / 20:33 CEST episode — the cleanest)

| Time (UTC) | Event | Source | Confidence |
| ---------- | ----- | ------ | ---------- |
| 18:29–18:32 | Continuous `junction_jump_anomalous` (`reason=projection_divergence`, `tick_jump_us`≈86–183, axes 2&3, `target=_motion_engine::pump`) | host-rust/motion | Confirmed |
| 18:32:10 | `kalico_stream_error` ×3: **CRC mismatch** (`expected=0x2455 actual=0x10cc`), **bad trailer** `0x59`,`0x67` | host-rust/mcu-comms | Confirmed |
| 18:32:15 | `send_frame_transient` — pump `send_mcu_frames` failed | host-rust/motion | Confirmed |
| 18:33:24.713 | `usb_drop_poll_error` `Io(BrokenPipe)` **+** `reactor_exit_on_fault` (same ms) | host-rust/mcu-comms | Confirmed |
| 18:33:24 | `std::process::abort()` → SIGABRT → coredump `core.python.3009` | coredump | Confirmed |
| 18:33:29 | systemd restarts klipper (Main PID 3295) | systemctl | Confirmed |
| 18:33:39 | `host-ec` `al_state` + `bringup_fail` → host-py `Config error` (EtherCAT X servo won't come up) | host-ec/host-py | Confirmed |

Earlier episodes show a *second* signature: 17:39:52 `kalico runtime fault` and 17:47:12 **`MCU 'mcu' shutdown: Timer too close`** preceding the abort — i.e. step-queue starvation / step scheduled in the past, not only wire corruption.

## Confirmed Findings

### Finding 1: The "crash" is a deliberate fail-loud abort, not a hang

**Evidence:** `core.python.3009` backtrace: `abort → std::process::abort → host_rt::host_io::open_with_port::{closure#0} at host-rt/src/host_io/mod.rs:382`. `rust/host-rt/src/host_io/mod.rs:381-383` is the `EXIT_ON_FAULT` path.

**Detail:** The port-bound reactor for a **CRITICAL** MCU exited non-gracefully; the thread logs `reactor_exit_on_fault` and calls `std::process::abort()` so systemd restarts klippy. `panic=abort` + this explicit abort take down the whole Python interpreter — which is why it wedges hard rather than erroring cleanly.

### Finding 2: The abort trigger is a serial-transport IO failure on the F401 MCU

**Evidence:** Paired same-millisecond records at each abort (18:33:24.713, 18:08:29.674, 17:47:42.18): `usb_drop_poll_error` with `err = Io(Custom { kind: BrokenPipe })` and `reactor_exit_on_fault`.

**Detail:** The transport to the critical MCU (F401 via CH340 USB-serial) returns `BrokenPipe`. In 2 of 3 episodes this is immediately preceded by `kalico_stream_error` CRC mismatch + bad-trailer — i.e. the wire was already corrupting before it dropped.

### Finding 3: Arc-fitting is NOT the cause

**Evidence:** Repro gcode `Voron_Design_Cube_v7...gcode` has `grep -cE '^G[23] ' = 0` and header `; enable_arc_fitting = 0`.

**Detail:** The crash reproduces on pure linear moves with arc-fitting disabled. The branch name `neptune-crash-no-arcfit` reflects this controlled test. **Refutes** the user's initial arc hypothesis.

### Finding 4: "Unresponsive until power-cycle" = failed EtherCAT re-bring-up after restart

**Evidence:** 18:33:39 `host-ec` `bringup_fail` ("bringup failed; sending handshake-fail then exiting") + `al_state` at bringup stage, then host-py `Config error`.

**Detail:** systemd *does* restart klippy (Confirmed at 18:33:29), but the A6-EC servo cannot be re-brought-up from its post-fault AL state, so the machine stays down. A full power-cycle resets the drive. This is the half of the symptom that a bare klippy restart can't fix.

## Deduced Conclusions

### Deduction 1: User's "buffer not consumed for 60s" = MCU stopped draining after comms broke

**Based on:** Findings 1–2 + earlier `motion backpressure: shutdown while draining (buffer_time=7.720s)` and `Timer too close`.

**Reasoning:** When the link corrupts/drops, the host can no longer feed or read the MCU; queued moves drain (brief continued motion → then stop), nothing refills, and the host-side drain/backpressure watchdog reports the buffer unconsumed ~60s later — exactly the "pretends to still print, then errors" symptom. On the last crash the user power-cycled before that watchdog elapsed.

## Hypothesized Paths

### Hypothesis 1: A motion-pump junction defect produces the step-timeline discontinuity that breaks comms

**Status:** Open
**Theory:** `junction_jump_anomalous` (`reason=projection_divergence`, `tick_jump_us` up to ~183 while `host_jump_us`≈0) at `rust/motion-engine/src/pump.rs:649-685` indicates consecutive seams that are contiguous in host time but ~100µs apart in MCU tick time. This step-time divergence intermittently (a) schedules an MCU step too close → `Timer too close`/`runtime fault` MCU shutdown, or (b) emits a malformed/duplicated frame → CRC/bad-trailer wire corruption → `BrokenPipe`.
**Supporting indicators:** The anomaly storm runs for minutes on `_motion_engine::pump` right up to each comms failure; both crash signatures (Timer-too-close AND CRC-corruption) point at the step/frame stream; arc-fit refuted so the defect is in the core junction/seam path, not arc segmentation.
**Would confirm:** A single anomalous seam temporally adjacent to the next `kalico_stream_error`/`Timer too close`; or code showing the seam tick projection can emit a non-monotonic / overlapping step frame.
**Would refute:** CRC corruption reproduced under load with motion idle (pure HW/CH340 link fault), or the anomaly proven to be a benign clock-projection log with no effect on emitted frames.
**Resolution:** —

### Hypothesis 2: CH340 USB-serial link fault under sustained throughput (hardware)

**Status:** Open
**Theory:** Sustained high step-rate traffic over the CH340 at 500000 baud corrupts/drops the USB link independent of planner output.
**Supporting indicators:** `BrokenPipe` is a device-went-away errno; CRC/bad-trailer is classic line corruption.
**Would confirm:** Kernel `dmesg` CH341 disconnect/re-enumerate at the drop time; repro with a different USB cable/port or a native-USB MCU.
**Would refute:** dmesg shows no USB event at the drop (corruption is purely in-protocol → points back to H1). Initial dmesg grep was empty — weakly favors H1, but inconclusive (ring may have rotated).
**Resolution:** —

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Persistent kernel USB log at drop time | Separates HW link fault (H2) from in-protocol corruption (H1) | `journalctl -k` with persistent journal, or `dmesg -w` during a live repro |
| The emitted step frame around an anomalous seam | Proves whether `projection_divergence` actually malforms a frame | Add frame-level trace at `pump.rs` seam, or decode the `kalico_stream_error` raw bytes |
| MCU-side fault detail at the drop | Confirms whether MCU rejected/desynced first | `KALICO_DIAG_DUMP` / `mcu-diagnostics` event-ring around the crash tick |

## Source Code Trace

| Element       | Detail                                      |
| ------------- | ------------------------------------------- |
| Abort origin  | `rust/host-rt/src/host_io/mod.rs:382` — `std::process::abort()` in the reactor thread's `EXIT_ON_FAULT` branch (`reactor_exit_on_fault`) |
| Trigger       | Reactor `run()` exits non-gracefully because transport returned `Io(BrokenPipe)` on a CRITICAL MCU |
| Condition     | Preceded (2/3 episodes) by `kalico_stream_error` CRC mismatch + bad trailer; storm of `junction_jump_anomalous` (`projection_divergence`) at `rust/motion-engine/src/pump.rs:649-685` throughout the print |
| Related files | `rust/motion-engine/src/pump.rs` (junction seam projection, `junction_jumps`, `check_junction_position_continuity`), `rust/host-rt/src/host_io/reactor.rs`, EtherCAT bring-up (`host-ec`) |

## Conclusion

**Confidence:** High on the failure *chain* and on refuting arc-fitting; Medium on the root *trigger*.

Confirmed: a mid-print serial-link failure on the critical F401 MCU (in-protocol CRC/trailer corruption then `BrokenPipe`) drives the `host-rt` reactor into its intentional `EXIT_ON_FAULT` abort, killing klippy; the EtherCAT servo then fails to re-bring-up on restart, requiring a power-cycle. Arc-fitting is refuted. The most promising open root-cause path is a motion-pump junction/seam defect (`projection_divergence`) that perturbs the step timeline — but whether that *causes* the wire corruption or merely co-occurs is not yet Confirmed; a CH340 hardware link fault (H2) remains a live alternative.

## Recommended Next Steps

### Fix direction
Do not touch the `EXIT_ON_FAULT` abort — it is working as designed (fail-loud). Fix the upstream trigger once Confirmed: either the pump seam tick projection (H1) or the USB-serial link (H2). Separately, the EtherCAT auto-recovery gap (Finding 4) is worth a follow-up so a host restart can re-bring-up the servo without a power-cycle.

### Diagnostic
1. Re-run the cube print with `dmesg -w` / `journalctl -kf` captured to a file in `~/printer_data/logs/` to catch (or rule out) a CH341 disconnect at the drop instant.
2. Pull `KALICO_DIAG_DUMP` / event-ring (`mcu-diagnostics`) immediately after the next crash to see the MCU's last actions before the link died.
3. Correlate one `junction_jump_anomalous` seam (with its `prev_source_line`/`next_source_line`) to the next `kalico_stream_error` to test H1.

## Reproduction Plan

Print `Voron_Design_Cube_v7_0.2mm_PLA_Elegoo Neptune 3 Pro_57m43s.gcode` (linear, arc-fit off) on `neptune-crash-no-arcfit`. Expect: `junction_jump_anomalous` storm during print, then within minutes a `kalico_stream_error`/`Timer too close`, `usb_drop_poll_error(BrokenPipe)`, `reactor_exit_on_fault`, a new `core.python.*` coredump, and EtherCAT `bringup_fail` on restart. Reproduced 3× on 2026-06-29.

## Follow-up: 2026-06-29 #2 — instrumented build, organic corruption captured

### New Evidence (Confirmed)

- **Instrumentation deployed and working.** `stream_corruption_frame` (raw hex) and `usb_drop_poll_error` errno/kind fields are live (commit `783457794`).
- **Organic corruption captured at 19:32:12 UTC and the reactor SURVIVED it.** klippy MainPID uptime was 29:53 at 19:43 (same process since the 19:13:16 spawn) — so this corruption took no intervention and did **not** abort. Confirms corruption is transient/survivable until it eventually escalates to the `BrokenPipe` drop (mirrors 18:32→18:33).
- **Corruption is INBOUND (MCU→host), on kalico channel 0** (status/telemetry). The host planner only produces *outbound* frames, so the planner cannot be the source of these CRC mismatches. **H1 (planner emits the corrupt frames) is refuted for the corruption mechanism.**
- **Desync fingerprint, not random noise.** Organic kalico CRC errors recur with `expected=0x2455`, which is little-endian bytes `55 24` = `FRAME_SYNC`(0x55)+len-low(0x24). The demuxer's declared length over-ran the true frame end and read the **next frame's header as the CRC** → classic byte-loss/length-overrun desync. The power-cycle-induced frame instead showed `expected=0x57d9` (random brown-out garbage) — a distinct fingerprint, confirming the 19:12–19:13 burst was the manual F4 power-pull, not organic.
- **Host reader is NOT starved.** `slow_poll` (poll_serial >5ms) returned **zero** events in 3h. The reactor drained RX promptly; the byte loss is below the host read layer (UART/USB/CH340 or MCU TX), not host-thread starvation.
- **Latest "crash" (SIGBUS in `_motion_engine` PyO3 bridge, ~19:11) was a manual F4 power-cycle**, per user — same underlying bug, intervened before the 60s watchdog. Not a distinct organic signature. (Side note: pulling the MCU mid-FFI SIGBUSes the bridge — a robustness gap, out of scope here.)

### Updated Hypotheses

- **H1 (planner corrupts the stream): Refuted** — corruption is inbound MCU→host.
- **H2 (link-level byte loss under load): Confirmed-leaning** — inbound desync fingerprint + slow_poll-clean reader + load correlation. Locus is the F401 UART TX, CH340, or USB, not the host reader.
- **New H3 (demuxer kalico path lacks resync):** on a kalico CRC error the demuxer discards the buffer and returns to `WaitingForFrame` with **no byte replay**, unlike the klipper path (which replays `skip(1)`). A single dropped byte can therefore swallow subsequent frames and deepen the cascade. Aggravator, not root cause. `rust/mcu-transport/src/demux.rs` InsideKalico arm.

### Backlog Changes

- Still missing the **errno at the final organic `BrokenPipe`** — every natural drop predates the instrumented build, and the one instrumented run was intervened. Need one clean run left to ride to the natural drop.

### Updated Conclusion

The organic failure is **inbound serial byte-loss on the F401→host link under print load**, which desyncs the demuxer (CRC/trailer cascade) and, when it fails to recover, ends in `BrokenPipe` → `EXIT_ON_FAULT` → klippy abort. Host-side read starvation and the motion planner are both ruled out as the corruption source. Root locus (MCU UART TX overrun vs CH340/USB) is not yet pinned.

## Follow-up: 2026-06-29 #3 — the REAL symptom is a host pump flow-control wedge (not the coredump)

Correction: there are **two distinct failure modes**, and the user's described symptom is the *second*, which the host survives (no coredump):

- **Mode A (coredump):** transport fully drops (`BrokenPipe`) → reactor `EXIT_ON_FAULT` → `abort()`. (17:47/18:08/18:33, old build.)
- **Mode B (wedge — the reported symptom):** motion stalls, host stays alive, "buffer not consumed in 60s" error, dead until power-cycle.

### Confirmed Findings (Mode B, organic, instrumented build, no intervention)

Exact error: `19:33:17 host-py "motion backpressure: buffer_time=0.000s channel_pending=4899 did not drain within 60s"` (`klippy/motion.py:704`). klippy did **not** crash — MainPID stayed up >35 min after.

Timeline:
1. 19:32:12 — inbound serial corruption blip (2 frames), reactor recovers.
2. 19:32:17 — **exactly one** `send_frame_transient` (PushPieces RPC timed out), then no further sends.
3. 19:33:17 — `channel_pending=4899` undrained 60s; `buffer_time=0.000s` (MCU starved).
4. 19:45+ — `runtime_event_subscriber_overflow` storm, **2986 events, ongoing at 19:52** → MCU is alive and streaming; the host subscriber is dropping events.

### Mechanism (Confirmed code path + Deduced trigger)

- `send_frame`/PushPieces is a request/response RPC with timeout (`pump.rs:940`, `kalico_call_on_channel`); any failure → `SendError::Transient` (`pump.rs:982`). On `Transient` the pump `break 'send` and retries via the loop — but never escalates to `Fatal`, so no crash.
- The pump only sends when `room() > 0`; `room = ring_depth − (pushed − retired)` (`pump.rs:62`). `retired` is set **absolutely** from a periodic MCU heartbeat (`PumpMsg::Heartbeat`/`retired_counts`, `pump.rs:558-568`; `attach_heartbeat_callback`, `bridge.rs:3184`).
- **Deduced wedge:** the heartbeats carrying `retired_counts` are delivered via the runtime-event subscriber that is overflowing and dropping (the 2986-event storm). With `retired` frozen, `room` stays 0, the scheduler returns `Schedule::StallAhead` for every send (`pump.rs:129`), motion never resumes — explaining the single `send_frame_transient` then silence. The backpressure watchdog detects the stall at 60s but takes **no recovery action** → silent wedge until power-cycle.

### Fix direction (priority order)

1. **Recover on drain-stall (primary).** When the channel can't drain for the timeout while the transport is alive, actively re-sync flow-control state (re-query MCU ring head/`retired`) or escalate to a transport reset/reconnect — do not sit forever. Per fail-loudly, `klippy/motion.py:704`'s 60s detection should trigger recovery, not just log.
2. **Don't let flow-control-critical heartbeats be dropped.** `runtime_event_subscriber_overflow` dropping `retired_counts` heartbeats is the freeze cause; route retired feedback on a path that can't be starved by the lossy event broadcast, or make the subscriber not drop heartbeats.
3. **Reduce the trigger.** Inbound byte-loss under load (link/baud/CH340/MCU-TX) + kalico-path demuxer resync (H3) lowers corruption frequency — but the host must still recover gracefully (item 1).

### Updated Conclusion (Confidence: High on chain, Medium on the heartbeat-drop freeze step)

The reported "stops moving → still pretends to print → 60s buffer error → dead until power-cycle" is a **host-side motion flow-control wedge**: a transient inbound-corruption blip drops the MCU `retired`-feedback heartbeats (subscriber overflow), the pump's ring-room view sticks at 0, every send `StallAhead`s, and nothing recovers it. The motion planner and host read-thread are both ruled out as causes.

## Side Findings

- `bridge` logs a high-rate `live-position poll failed; serving stale cache` during the failure window — symptom of the same comms loss, not an independent fault.
- `unknown_correlation_id` ("response for unknown correlation_id dropped") appears during the corruption window — consistent with a desynced/garbled response stream.
- `fg_freeze pc=134252664 (0x8008878) stall_ticks=5` appears identically on every MCU boot alongside host-commanded soft resets (cause `0x14000002`); likely benign boot forensics, but unconfirmed (Backlog #2).
