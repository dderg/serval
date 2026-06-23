# Investigation: Neptune Print Crash Shortly After Start

## Hand-off Brief

1. **What happened.** Neptune print runs are repeatedly shutting down on `PieceStartInPast` (`-308`, wire `65228`) after pump frames arrive at the MCU in the past.
2. **Where the case stands.** Active; VictoriaLogs is healthy and confirms the failure across recent prints after `a46fea2db`, which reverted the dispatched-frontier gate while keeping the `commit_count=0` thin-lead drain fix.
3. **What's needed next.** Investigate the post-`a46fea2db` residual `PieceStartInPast`: the feeder is throttling on total outstanding work, and the old long barrier-freeze signature is not obviously repeating.

## Case Info

| Field | Value |
| ----- | ----- |
| Ticket | N/A |
| Date opened | 2026-06-23 |
| Status | Active |
| System | Neptune 3 Pro bench at `dderg@ethercatpi5.local`; ZNP Robin Nano DW v2.2 STM32F401RCT6; X axis A6-EC EtherCAT servo over `eth0`; Y/Z/E steppers |
| Evidence sources | User report; Neptune VictoriaLogs; raw `~/printer_data/logs/events/*.jsonl`; MCU diagnostics; git/version state on Pi; prior Neptune starvation/backpressure investigations |

## Problem Statement

User reports: "neptune crashes quite quickly into the print. I tried 2 different files." Treat the reported recurrence as a hypothesis; independently verify failure class, timing, and causal order from structured logs.

## Evidence Inventory

| Source | Status | Notes |
| ------ | ------ | ----- |
| User report | Partial | Reports quick crash after print start and reproduction with two files; exact file names not yet recorded. |
| Neptune VictoriaLogs | Available | `/health` returned `OK`; query results are valid evidence. |
| Raw event JSONL on Neptune | Available | Source of truth if a targeted raw ring extract is needed. |
| MCU diagnostics | Available | Recent crash replays show repeated `diag.rust_fault err=4294966988`, `runtime.block_source stepout_burst≈28M cycles`, and `runtime.fg_freeze pc=134252840`. |
| Git/version state on Pi | Available | Pi repo is on `curvature-profile...origin/curvature-profile` at `a46fea2db`; `bench/` is untracked. |
| Prior Neptune second-run starvation investigation | Available | `_bmad-output/implementation-artifacts/investigations/neptune-second-run-starvation-investigation.md` documents an earlier host-side `stream_planner_fatal` variant. |
| Prior PieceStartInPast clock/backpressure investigation | Available | `_bmad-output/implementation-artifacts/investigations/piece-start-in-past-clock-rebase-investigation.md` already follows this exact `arrival_lead<0` failure through 2026-06-23 and confirms the proximate root. |
| Prior print-completes-early investigation | Available | `_bmad-output/implementation-artifacts/investigations/print-completes-early-investigation.md` confirms the earlier wrong-throttle-signal history and follow-up crash chain. |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Verify Neptune VictoriaLogs health and observability heartbeat | High | Done | `/health` returned `OK`. |
| 2 | Identify recent print IDs and sessions | High | Done | Recent failures include `print-1782197247`, `print-1782199207`, and `print-1782199302`. |
| 3 | Query warnings/errors around failed prints | High | Done | Same failure chain repeats: `transit_diag_alert` -> runtime fault -> EtherCAT broken pipe. |
| 4 | Query MCU crash forensics and live fault events | High | Done | Crash replay confirms `PieceStartInPast` and large stepout burst. |
| 5 | Capture deployed branch and commit on Pi | Medium | Done | `curvature-profile` at `a46fea2db`. |
| 6 | Trace active-stream lead loss in pump/router/planner | High | Done | Prior `piece-start-in-past-clock-rebase` follow-up confirms a ~1 s committed-frontier freeze and wrong pacing frontier. |
| 7 | Separate host-side late send from MCU-side stepout stall | High | Done | Prior follow-up confirms the crash is a starvation cascade from frozen dispatched frontier; `stepout_burst` is not the primary root. |
| 8 | Determine why `a46fea2db` still permits late delivery | High | Open | Gate now uses total outstanding work again; commit freeze fix is present, but post-revert prints still hit `arrival_lead<0`. |

## Timeline of Events

| Time | Event | Source | Confidence |
| ---- | ----- | ------ | ---------- |
| 2026-06-23 | User reports Neptune crashes shortly after print start with two different files | User report | Confirmed report; failure itself independently confirmed below |
| 2026-06-23 06:47:27.351Z | `print-1782197247` starts | VictoriaLogs `print_id` query | Confirmed |
| 2026-06-23 06:47:37.956Z | Pump reports `arrival_lead<0` on MCU 1 axis 0 (`-1577.6 us`) | VictoriaLogs `transit_diag_alert` | Confirmed |
| 2026-06-23 06:47:37.962Z | Host enters shutdown: `MCU 'mcu' shutdown: kalico runtime fault` | VictoriaLogs warnings/errors query | Confirmed |
| 2026-06-23 06:47:38.063Z | EtherCAT endpoint dies mid-session after the MCU runtime fault | VictoriaLogs warnings/errors query | Confirmed |
| 2026-06-23 06:47:56.786Z | MCU crash replay shows `diag.rust_fault err=4294966988 detail=67010`, `stepout_burst=28160290 cyc` | VictoriaLogs MCU runtime query | Confirmed |
| 2026-06-23 07:20:07.065Z | `print-1782199207` starts | VictoriaLogs `print_id` query | Confirmed |
| 2026-06-23 07:20:17.153Z | Pump reports `arrival_lead<0` on MCU 1 axis 0 (`-5896.8 us`) | VictoriaLogs `transit_diag_alert` | Confirmed |
| 2026-06-23 07:20:17.164Z | Host enters shutdown: `MCU 'mcu' shutdown: kalico runtime fault` | VictoriaLogs warnings/errors query | Confirmed |
| 2026-06-23 07:20:32.038Z | MCU crash replay shows `diag.rust_fault err=4294966988 detail=132992`, `stepout_burst=28025812 cyc` | VictoriaLogs MCU runtime query | Confirmed |
| 2026-06-23 07:21:42.524Z | `print-1782199302` starts | VictoriaLogs `print_id` query | Confirmed |
| 2026-06-23 07:21:42.677Z | Fresh dispatch has ~250 ms start-time lead (`seg0_deficit` fields show `deficit_us≈249,976`) | VictoriaLogs `seg0_deficit` query | Confirmed |
| 2026-06-23 07:21:52.984Z | Pump reports `arrival_lead<0` on multiple axes; examples: MCU 0 axis 3 `-4700.8 us`, MCU 1 axis 0 `-7998.0 us` | VictoriaLogs `transit_diag_alert` | Confirmed |
| 2026-06-23 07:21:53.039Z | Pump reports MCU 0 axis 3 arrived `-51757.6 us` late | VictoriaLogs `transit_diag_alert` | Confirmed |
| 2026-06-23 07:21:53.095Z | EtherCAT endpoint dies mid-session after the MCU runtime fault | VictoriaLogs warnings/errors query | Confirmed |
| 2026-06-23 07:22:07.534Z | MCU crash replay shows `diag.rust_fault err=4294966988 detail=201653`, `stepout_burst=27403756 cyc` | VictoriaLogs MCU runtime query | Confirmed |

## Confirmed Findings

### Finding 1: The repeated crash is `PieceStartInPast`, not an arbitrary EtherCAT transport failure

**Evidence:** VictoriaLogs warnings/errors for `print-1782199302` show `runtime_fault` with `fault_code=65228` at `2026-06-23T07:21:52.984Z`, followed by `MCU 'mcu' shutdown: kalico runtime fault` at `07:21:52.987Z`, followed by `EXIT_ON_FAULT - EtherCAT transport broken-pipe in pump` at `07:21:53.095Z`. `rust/runtime/src/error.rs:187` defines `PieceStartInPast = -308`; `rust/runtime/src/error.rs:202-208` defines the sign-wrapped `u16` wire encoding, making `-308` appear as `65228`.

**Detail:** The EtherCAT broken pipe is downstream fallout from fail-loud shutdown. The first actionable fault is the runtime `PieceStartInPast` condition.

### Finding 2: Multiple recent prints share the same failure signature

**Evidence:** Recent print IDs `print-1782197247`, `print-1782199207`, and `print-1782199302` all show `transit_diag_alert` rows with `arrival_lead<0`, then host shutdown on MCU runtime fault, then EtherCAT endpoint death. Their crash replays show `diag.rust_fault err=4294966988` at `06:47:56.786Z`, `07:20:32.038Z`, and `07:22:07.534Z` respectively.

**Detail:** `4294966988` is the unsigned representation of `-308`, the same `PieceStartInPast` fault. This confirms the user's "two different files" report as a repeated runtime class, not a single-file anomaly.

### Finding 3: The pump initially has lead, then sends pieces that arrive in the MCU past

**Evidence:** In `print-1782199302`, `seg0_deficit` at `2026-06-23T07:21:42.677Z` reports `deficit_us≈249,976` for MCU 1 and `≈249,990` for MCU 0, matching the intended fresh-start lead. Ten seconds later, `transit_diag_alert` reports negative arrival lead on both MCUs: examples include MCU 1 axis 0 `arrival_lead_us=-7998.0` at `07:21:52.985Z`, MCU 0 axis 2 `-7681.2` at `07:21:52.995Z`, and MCU 0 axis 3 `-51757.6` at `07:21:53.039Z`. The alert is emitted at `rust/motion-engine/src/pump.rs:890-930` when `r.front_start_time - r.arrival_clock < 0`.

**Detail:** The run does not begin late; it loses enough dispatch lead during active streaming that pieces arrive after their scheduled start time.

### Finding 4: MCU crash replay points at stepout-side CPU occupancy, but the semantic fault is still late piece arrival

**Evidence:** Crash replay for the latest three failures reports `runtime.block_source stepout_burst=28160290`, `28025812`, and `27403756` cycles, with `usb_burst=0`; `runtime.tim5_ia` remains around `14k-19k` cycles; `runtime.isr_phase=9`; `runtime.fg_freeze pc=134252840 stall_ticks=5`. `addr2line` against the Pi ELF maps `0x08008928` to `readb` at `out/board-generic/board/io.h:30` and `0x0800812d` to `periodic_event` at `src/sched.c:67`.

**Detail:** MCU diagnostics show the reset replay observed foreground freeze and stepout burst, but the live host/motion evidence already shows pieces arriving in the past before shutdown. The stepout burst is a consequence or contributing condition to investigate, not yet the proven initiating cause.

## Deduced Conclusions

### Deduction 1: Current failures are an active-stream lead starvation variant

**Based on:** Findings 1, 2, and 3.

**Reasoning:** A fresh run starts with roughly the configured 250 ms lead, so this is not the exact old "new print starts already in the past" signature. The same run later emits `arrival_lead<0` immediately before the MCU latches `PieceStartInPast`. Therefore the relevant defect is that the active pump/planner path fails to maintain dispatch lead until the MCU consumes the pieces.

**Conclusion:** Root-cause tracing should start from pump scheduling, router timing, and planner backpressure/commit gating rather than EtherCAT transport recovery.

### Deduction 2: This is related to but not identical to the prior Neptune second-run starvation case

**Based on:** Prior case file `_bmad-output/implementation-artifacts/investigations/neptune-second-run-starvation-investigation.md:5-17` and Findings 1-3.

**Reasoning:** The prior case aborts host-side with `stream_planner_fatal` when a segment is scheduled in the past before it reaches the MCU. Today's case allows dispatch to proceed until the MCU itself reports `PieceStartInPast`. Both share the same late-segment invariant, but the failure boundary moved from host planner guard to MCU runtime fault.

**Conclusion:** Recent changes around dispatch gating/backpressure are high-priority suspects, but the hypothesis remains open until source trace confirms a specific mechanism.

### Deduction 3: Prior branch investigations explain the fix/revert sequence, not the whole current failure

**Based on:** `_bmad-output/implementation-artifacts/investigations/piece-start-in-past-clock-rebase-investigation.md:281-360` and `_bmad-output/implementation-artifacts/investigations/print-completes-early-investigation.md:275-292`.

**Reasoning:** The prior 2026-06-23 follow-up matches the present logs: `arrival_lead<0` cascade across axes/MCUs, committed frontier frozen for about 1 s, and gate math using submitted frontier (`pending_end`) instead of dispatched-to-pump frontier (`dispatch_committed`). It also identifies the trigger: barrier commit can produce `commit_count=0` on a small buffer, freezing delivered frontier while the feeder remains throttled.

**Conclusion:** The prior delivery-accurate fix landed in `d437ce8a0`, but `a46fea2db` reverted only the gate-signal half while keeping the barrier no-progress fix. The current logs are post-revert and require a fresh trace; they should not be summarized as simply "delivery-accurate pacing was never implemented."

## Hypothesized Paths

### Hypothesis 1: Neptune fails shortly after print start independent of G-code file

**Status:** Superseded

**Theory:** The failure is caused by a shared runtime condition early in print execution rather than one malformed print file.

**Supporting indicators:** User reports two different files fail quickly into the print.

**Would confirm:** Structured logs show the same failure signature shortly after `print_id` start across at least two print IDs.

**Would refute:** Logs show only one failed print, different failure signatures, operator cancellation, or file-specific parsing/motion errors.

**Resolution:** Confirmed by repeated `PieceStartInPast` failures across recent print IDs.

### Hypothesis 2: Recent dispatch/backpressure gating allows the stream to run out of lead during active printing

**Status:** Confirmed

**Theory:** The deployed branch at `a46fea2db` includes recent motion changes around outstanding work and dispatch frontier gating. The pump starts with ~250 ms lead but later sends pieces after their start time, implying that host-side gating, batching, or frame transit allows the MCU frontier to catch up.

**Supporting indicators:** `seg0_deficit` shows healthy initial lead; `transit_diag_alert` later shows late arrivals; recent commit history on the Pi is dominated by motion pacing/backpressure changes.

**Would confirm:** Source trace and logs show the planner/pump waiting on the wrong frontier, underestimating outstanding MCU work, or dispatching too little/too late before `arrival_lead` crosses zero.

**Would refute:** Logs show pump remains ahead but the MCU independently stalls for long enough to consume all lead, or a specific firmware stepout path blocks piece intake despite timely host sends.

**Resolution:** Confirmed for the pre-`a46fea2db` failure and then partially reverted. Current HEAD intentionally gates on total outstanding work (`_mcu_pending_end_time - est`) again.

### Hypothesis 3: MCU stepout-side burst independently consumes the lead and causes timely pieces to become late

**Status:** Refuted as primary root

**Theory:** The repeated `stepout_burst≈28M cycles` blocks foreground or timing progress long enough that pieces arrive late even if the host schedules them with adequate lead.

**Supporting indicators:** Crash replay consistently identifies `stepout_burst`, not USB burst, as the largest block source.

**Would confirm:** Raw MCU ring events show stepout burst preceding the first negative `arrival_lead`, or a live `KALICO_DIAG_DUMP` during a reproduction shows stepout-side hogging before the pump loses lead.

**Would refute:** Host logs show the pump sent frames late relative to MCU clock before any MCU-side burst, or stepout burst is only a post-fault/replay artifact.

**Resolution:** Refuted as the primary mechanism by the prior follow-up: the measured sequence is a dispatched-frontier freeze and starvation cascade; stepout diagnostics are secondary to the late-delivery fault.

### Hypothesis 4: Barrier commit freezes the dispatched frontier with `commit_count=0` while the feeder is throttled

**Status:** Fixed in code path; current recurrence unproven

**Theory:** Incremental barrier commit can hold a small buffer wholly uncommitted when no clean seam exists at or before the barrier; because the gate uses submitted frontier, it pauses the feeder instead of feeding enough moves to unstick the barrier.

**Supporting indicators:** Prior follow-up records batch 33 with `n=4, barrier=3, commit_count=0`, a 1.07 s `dispatch_committed` gap, and `feed_throttle` held across the freeze.

**Would confirm:** Already confirmed by `_bmad-output/implementation-artifacts/investigations/piece-start-in-past-clock-rebase-investigation.md:281-360`.

**Would refute:** N/A; currently confirmed for the prior matching crash. A new reproduction could only show whether an additional mechanism is now also present.

**Resolution:** `d437ce8a0` added the thin-lead drain path for this condition and `a46fea2db` kept it. Post-revert logs still show transient `commit_count=0`, but they are followed by committed dispatch within tens to hundreds of milliseconds, not the original ~1 s frozen-frontier signature.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| New reproduction after delivery-accurate pacing fix | Verifies the confirmed root is resolved and no next bug is exposed | Run either failing file and compare `pending_end`, `dispatch_committed`, `est`, `feed_throttle`, and `transit_diag_alert`. |
| Exact print file names and whether the failures were the two latest print IDs | Confirms which logs map to the user's two files | Ask user or correlate Moonraker job metadata if available. |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Error origin | MCU runtime `PieceStartInPast` (`rust/runtime/src/error.rs:187`) surfaced as `runtime_fault`; host pump warning emitted from `rust/motion-engine/src/pump.rs:890-930`. |
| Trigger | Pump sends a `PushPieces` frame whose returned `front_start_time` is earlier than the MCU `arrival_clock`. |
| Condition | Active print starts with ~250 ms lead, then dispatched pieces arrive with negative lead immediately before shutdown. |
| Related files | `rust/motion-engine/src/pump.rs`, `rust/motion-engine/src/stream_planner.rs`, `rust/motion-engine/src/bridge.rs`, `rust/host-rt/src/mcu_serial_conn.rs`, `src/generic/fault_handler.c`, `src/sched.c`, `rust/runtime/src/tick.rs`. |

## Conclusion

**Confidence:** Medium

Confirmed: recent Neptune print crashes are repeated `PieceStartInPast` runtime faults caused by pieces arriving at the MCU after their scheduled front start time. The previous fix sequence is now clear: `d437ce8a0` fixed both the dispatched-frontier gate and barrier no-progress freeze; `a46fea2db` reverted the gate-signal half because dispatched-frontier gating let submitted-but-uncommitted work run away, while retaining the thin-lead drain fix. The current post-`a46fea2db` crash is therefore a residual/new late-delivery failure, not simply an unimplemented prior fix.

## Recommended Next Steps

### Fix direction

Do not blindly reapply `d437ce8a0`'s dispatched-frontier gate; it was explicitly reverted by `a46fea2db`. Next trace should compare `_mcu_pending_end_time`, actual dispatched frontier, pump send cadence, and MCU arrival lead across the final second to isolate why total-outstanding pacing still allows late delivery.

### Diagnostic

Collect one reproduction with `pending_end`, `dispatch_committed`, `est`, `feed_throttle`, `commit_decision`, `thin_lead_drain`, pump send cadence, and `transit_diag_alert`. Specifically verify whether the final second is host dispatch latency, pump transport latency, MCU stepout-side blocking, or a remaining frontier/accounting mismatch.

## Reproduction Plan

Use either of the two failing print files. Capture the `print_id`, then query `seg0_deficit`, `transit_diag_alert`, `runtime_fault`, `runtime.block_source`, `diag.rust_fault`, and pump send cadence for the print window. If adding instrumentation, keep it in the structured event pipeline.

## Side Findings

- The repeated `seg0_deficit` message says "negative deficit_us => in past" even while the numeric `deficit_us≈250000` represents intended fresh-start lead; this wording was already called out in the prior Neptune investigation and remains misleading.
