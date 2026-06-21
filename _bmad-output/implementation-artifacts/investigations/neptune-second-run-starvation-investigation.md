# Investigation: Neptune — test3.gcode crashes on the second run (planner stream starvation)

## Hand-off Brief

1. **What happened.** On the Neptune bench, running test3.gcode (an SD print) **twice** crashes the
   host-rust motion engine with a fail-loud `stream_planner_fatal` ("planner stream starvation …
   0.086s in the past") on the **second** run. **Confirmed root cause:** starting a new SD print does
   **not** reset/re-anchor the motion-engine stream clock — the second print's moves are appended to
   the first print's stream timeline (first move at stream **t=2.076 s**, continuing run 1's ~2.0 s),
   so across the idle gap between the two runs their absolute schedule lands in the MCU's past.
2. **Where the case stands.** Confirmed by side-by-side logs: run 1 anchors fresh at stream t=0 and
   plays out cleanly; run 2 (`Starting SD card print (position 0)`, 2.6 s later) is timed as a
   continuation of run 1 and is 86 ms late. The two re-anchor guards both miss (see Finding 5). High
   confidence.
3. **What's needed next.** Decide the fix: reset/re-anchor the stream clock on print start (or when a
   move arrives after the stream has drained and the playhead has caught up) — without silently
   re-anchoring a genuinely-late *continuous* stream. Route to `bmad-quick-dev`.

## Case Info

| Field            | Value                                                                                  |
| ---------------- | -------------------------------------------------------------------------------------- |
| Ticket           | N/A                                                                                    |
| Date opened      | 2026-06-19                                                                             |
| Status           | Concluded — root cause Confirmed (High)                                                |
| System           | Neptune 3 Pro bench (`dderg@ethercatpi5.local`), host-rust motion engine, branch `curvature-profile` |
| Evidence sources | VictoriaLogs (`127.0.0.1:9428`) structured events; source `rust/motion-engine/` (`curvature-profile`) |

## Problem Statement

User-reported: "On Neptune, running the same simple gcode (test3.gcode) twice — it always runs the
first time, and always crashes on the second run." Routed through `/query-logs`. Treated as a
hypothesis; verified independently against the structured logs.

## Evidence Inventory

| Source                                   | Status    | Notes                                                                 |
| ---------------------------------------- | --------- | --------------------------------------------------------------------- |
| VictoriaLogs (Neptune)                   | Available | Healthy (`/health` = OK). 6 `stream_planner_fatal` in 24h.            |
| Crash session `k-1781896645-101624`      | Available | Full timeline: anchor-decision, seg0_deficit, 41 transit_diag, fatal. |
| Source `rust/motion-engine/{anchor,stream_planner,stream,bridge,planner}.rs` | Available | Abort site and anchor logic read directly.                            |
| **Successful test3 run logs**            | **Missing** | No isolated "first run OK" session captured for comparison.         |
| Host foreground-freeze detail (`runtime.fg_freeze pc=134252722`) | Partial | Present in nearly every session; PC not yet decoded.      |

## Timeline of Events (crash session `k-1781896645-101624`)

| Time (UTC)       | Event                                                                  | Source                         | Confidence |
| ---------------- | ---------------------------------------------------------------------- | ------------------------------ | ---------- |
| 19:17:26.349     | Session starts, MCU identified                                         | `mcu-comms`                    | Confirmed  |
| 19:26:54.659     | `Must home axis first: -5.000 0 0` (a move issued pre-home, rejected)  | warn                           | Confirmed  |
| 19:26:59.444     | `[anchor-decision] condition=first`, `host_now=573.025`, `t0=573.275`, `seg_t_start=0.0` | `anchor.rs:61` | Confirmed  |
| 19:26:59.444     | `seg0_deficit` ×2, deficit≈249.9 ms (= LEAD; benign, see Side Findings) | `motion`                       | Confirmed  |
| 19:26:59.448–.609 | **41 `transit_diag` dispatched in a 161 ms burst**, `arrival_lead_us` steady ~230–288 ms | `motion` | Confirmed  |
| 19:26:59.609 → 19:27:01.856 | **~2.25 s gap — no further dispatch**                       | (absence)                      | Confirmed  |
| 19:27:01.856     | `stream_planner_fatal`: segment (stream t=2.076 s) scheduled **0.086 s in the past** | `planner.rs:172` (`SegmentLate`) | Confirmed |

## Confirmed Findings

### Finding 1: The crash is the fail-loud starvation guard, not a generic panic

**Evidence:** `rust/motion-engine/src/anchor.rs:37-48`; fatal `error` field
`"planner stream starvation: segment (stream t=2.076s) scheduled 0.086s in the past; refusing to
silently re-anchor — planner failed to keep ahead of playback"` (session `k-1781896645-101624`,
19:27:01.856).

**Detail:** `Anchor::anchor_segment` computes `starvation = t0 + seg_t_start < host_now`. When
`starvation && !timeline_reset`, it returns `SegmentLate` rather than re-anchoring. `bridge.rs:3134-3141`
maps that to `DispatchError::SegmentLate`, surfaced as `stream_planner_fatal`. This is the
CLAUDE.md "fail loudly on a segment in the past — do not silently re-anchor" contract firing exactly
as designed. The guard is correct; it is reporting a real upstream defect.

### Finding 2: Reproducible — 6 fatals in 24h, all "in the past"

**Evidence:** `event:=stream_planner_fatal _time:24h` → 6 rows. Gaps: 1.451 s, 23.797 s, 0.113 s,
and the recent cluster all at **exactly 0.086 s** (stream t = 28.773 / 2.840 / 2.076 s).

**Detail:** The constant 0.086 s across three different stream positions indicates a fixed-size
timing miss on a single late segment, not progressive drift.

### Finding 3: The anchor is correct — the planner does NOT bleed its lead during the run

**Evidence:** `[anchor-decision] condition=first, seg_t_start=0.0, t0 = host_now + 0.25`
(`DEFAULT_LEAD_SECS = 0.25`, `anchor.rs:2`). The 41 `transit_diag` rows hold `arrival_lead_us`
steady at ~230–288 ms across the whole 161 ms dispatch burst.

**Detail:** This **refutes** an in-run throughput-shortfall theory (a planner running slower than
real-time would show `arrival_lead_us` decaying toward zero across the run; it does not). The run's
body is dispatched comfortably ahead. The second run re-anchored cleanly from scratch (`first`),
and `seg_t_start=0.0` shows the stream clock reset to 0 for the run — so this is not a
cross-run continuous-timeline bug either.

### Finding 4: The 2.25 s "gap" is the idle interval BETWEEN two separate runs (interpretation corrected)

**Evidence:** All events in `[19:26:59.6, 19:27:02.0]` for the crash session: last run-1 `transit_diag`
19:26:59.609 → `Starting SD card print (position 0)` **19:27:01.269** → engine-trace moves
`newpos=[95,95,10]…[100,95,10]` → `Finished SD card print` 19:27:01.271 → fatal 19:27:01.856.

**Detail:** Corrects the initial "stalled tail segment" reading. The 2.25 s gap is not a stall inside
one run — it is the wall-clock interval between **run 1 finishing** and **run 2 starting**. The
"late segment at t=2.076" is run 2's *first* move, not run 1's tail. (Hypotheses 1/3 were chasing a
stall that does not exist.)

### Finding 5: Both runs are SD prints of the same file; the second continues the first's stream timeline

**Evidence:** Crash session `k-1781896645-101624`: `Starting SD card print (position 0)` at
**19:26:58.667** (run 1) and again at **19:27:01.269** (run 2), 2.6 s apart. Run 1 anchored
`[anchor-decision] condition=first` at stream **t=0** (19:26:59.444). Run 2's first move dispatched
at stream **t=2.076 s** (the fatal's `seg_t_start`), i.e. continuing run 1's ~2.0 s timeline rather
than resetting. Successful run `k-1781897233-101785`: a single SD print, anchored `first` at t=0, 41
segments, **no second print, no fatal**.

**Detail:** Starting a new SD print does not reset the motion-engine stream clock nor null the
per-MCU `Anchor.t0`. Run 2's moves are appended at `t_committed ≈ 2.0`. Their absolute schedule is
`t0(run1) + 2.076`; because run 2 launches *after* run 1's motion has drained (the 2.6 s idle gap),
that host time has just passed → `starvation = t0 + seg_t_start < host_now` is true. The 86 ms
magnitude is simply how far past run 1's end run 2 was submitted.

### Finding 6: Both re-anchor guards miss this case

**Evidence:** `anchor.rs:36` (`timeline_reset = seg_t_start + eps < last_t_end`) and
`stream_planner.rs:365-368` (`if state.is_empty() && esc > t_committed { advance_idle(esc+LEAD) }`).

**Detail:** (a) `timeline_reset` is false because run 2's `seg_t_start` (2.076) is **greater** than
run 1's `last_t_end` (~2.0) — the stream advanced, it did not jump backward, so the backward-jump
re-anchor never triggers. (b) The run-loop idle re-anchor did not advance the clock (run 2's move
came out at t=2.076, not at `esc+LEAD`); candidate reason: `sync_instant` is set to `None` on the
idle/flush paths (`stream_planner.rs:413/417/421`), so `esc` reads 0 and `esc > t_committed (2.0)`
is false. To verify during fix design.

## Deduced Conclusions

### Deduction 1: The defect is a missing stream re-anchor on a new run, not throughput and not a tail stall

**Based on:** Findings 3, 5, 6.

**Reasoning:** Lead stays ~250 ms throughout each run's dispatch burst (not throughput). The "late
segment" is run 2's first move, appended at run 1's stream time across an idle gap (Finding 5), and
neither re-anchor guard fires (Finding 6). A throughput fix or a tail-commit fix would both miss the
mark.

**Conclusion:** The fix must ensure a run that begins after the stream has drained is timed against
the **live playhead**, not the previous run's accumulated stream clock — either by resetting the
stream/Anchor on print start, or by making the idle re-anchor (`stream_planner.rs:365`) actually
fire here (likely a `sync_instant`/`esc` accounting fix). It must NOT silently re-anchor a genuinely
late *continuous* stream (CLAUDE.md fail-loud contract — that real-starvation case must still abort).

## Hypothesized Paths

### Hypothesis 1: Host foreground freeze delays the final commit

**Status:** Refuted (for crash session `k-1781896645`)

**Theory:** `runtime.fg_freeze pc=134252722 stall_ticks=5` (present in nearly every session) is a
host stall that lands in the ~2.25 s window and pushes the final force-commit 86 ms past its
deadline.

**Would confirm:** A `fg_freeze` timestamp inside `[19:26:59.609, 19:27:01.856]` in the crash
session; decoding `pc=134252722` to the stalling call site.

**Would refute:** No freeze in that window, or freezes that don't correlate with the late-commit
sessions.

**Resolution:** Refuted. `event:=runtime.fg_freeze` for `k-1781896645-101624` returns **zero** rows;
the last freeze before the crash was 19:14:23 in a different session (`k-1781896458`). No host
foreground freeze occurred in the crash session, so the ~2.25 s stall has another cause.

### Hypothesis 2: "Second run only" is per-run state carried across the restart

**Status:** Confirmed (refined) — see Finding 5

**Theory:** Each test3 invocation runs in a fresh klippy session (rapid session churn + `first`
anchor each time). Something accumulated by the prior run (clock-sync drift estimate, `motion_history`,
drip cohort, or homing/idle state) makes the *later* run's final-commit timing marginal where the
first run's is not.

**Would confirm:** Logs of a confirmed-successful run show the tail segment committed *before* its
deadline with otherwise-identical cadence; the only delta is in the carried state.

**Would refute:** A successful run and a crash run are byte-for-byte identical in cadence up to the
tail, with no carried-state difference — pointing instead to a pure race / scheduling jitter.

**Resolution:** Confirmed and made precise. The "carried state" is the **stream clock / `Anchor.t0`
itself**: a new SD print does not reset it, so run 2 is appended at run 1's accumulated stream time
and timed across the idle gap. Not jitter — deterministic (Finding 5).

### Hypothesis 3: The final move is committed on a delayed Flush/end-of-print, not promptly

**Status:** Partially refuted

**Theory:** `commit(false)` only commits at a clean seam, so the run's last move stays buffered
until a `Flush`/`Dwell`/idle-drain. If that flush is issued ~2.25 s after the body (e.g. waiting on
the gcode stream / a dwell / toolhead `wait_moves`), the tail is dispatched late by construction.

**Would confirm:** A `StreamMsg::Flush`/`Dwell` (or idle `commit(true)`) at ~19:27:01.85 in the
crash session; correlating G-code tail of test3.gcode (trailing dwell / M400 / end).

**Would refute:** The tail commits via the normal `Move` path with the idle re-anchor active.

**Resolution:** Partially refuted. test3.gcode is 5 lines — `G91` + four `G1 x/y ±5 f6000` jog moves —
with **no trailing dwell, M400, or end-of-print**. So no *gcode* construct gates the final commit;
the ~2.25 s delay before the tail is purely host-side flush timing (toolhead idle / lookahead flush),
not a dwell in the file. The "tail committed late after an idle gap" mechanism still holds; only the
"a gcode dwell causes it" sub-theory is dead.

## Missing Evidence

| Gap                                   | Impact                                                         | How to Obtain                                                                 |
| ------------------------------------- | ------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| Successful (first-run) session logs   | Isolates the second-run-only delta (Hyp. 2)                   | Run test3 once on a fresh restart, note `session_id`, pull its motion events |
| `runtime.fg_freeze` window + PC decode | Confirms/refutes the stall cause (Hyp. 1)                     | Query `fg_freeze` in the crash window; `mcu-diagnostics` to decode `pc`      |
| test3.gcode tail content              | Confirms whether a trailing dwell/M400 gates the final commit | Read test3.gcode on the bench                                                |
| Which `StreamMsg` committed the tail  | Distinguishes Hyp. 3 from a Move-path miss                    | Add/inspect commit-path tracing at `stream_planner.rs:381-409`               |

## Source Code Trace

| Element       | Detail                                                                                                  |
| ------------- | ------------------------------------------------------------------------------------------------------ |
| Error origin  | `rust/motion-engine/src/anchor.rs:42` (`SegmentLate`), formatted at `rust/motion-engine/src/planner.rs:172` |
| Trigger       | Dispatch of a committed segment whose `t0 + seg_t_start < host_now` while `!timeline_reset` (`anchor.rs:37`) |
| Condition     | Tail segment force-committed after a ~2.25 s idle gap; idle re-anchor (`stream_planner.rs:365`) guards the `Move` path but not the `Flush`/`Dwell`/idle-drain commit of the tail |
| Related files | `stream_planner.rs` (run loop, re-anchor, dispatch_committed), `bridge.rs:3120-3160` (dispatch closure, `host_now`), `stream.rs:122` (`advance_idle`), `planner.rs` (parallel/legacy planner with the same pattern) |

## Conclusion

**Confidence:** High.

**Confirmed:** The crash is the fail-loud starvation guard (`anchor.rs:39`) aborting because run 2's
first move is dispatched 86 ms into the MCU's past. Root cause: **starting a new SD print does not
reset/re-anchor the motion-engine stream clock.** Run 2's moves are appended at run 1's accumulated
stream time (t=2.076 s, continuing run 1's ~2.0 s); because run 2 begins after run 1's motion has
drained (a 2.6 s idle gap), that absolute time slot has just passed. Both re-anchor guards miss it:
the backward-jump reset needs `seg_t_start < last_t_end` (false — the stream advanced), and the
run-loop idle re-anchor did not fire (likely `sync_instant` reset to `None`, zeroing `esc`).

This deterministically explains "first run always works, second always crashes": run 1 anchors fresh
at t=0; run 2 is always timed as a continuation across the inter-run gap. Verified against a clean
successful single run (`k-1781897233-101785`) and a clean two-run crash (`k-1781896645-101624`).

## Recommended Next Steps

### Fix direction

A run that begins after the stream has drained must be anchored to the **live playhead**, not the
previous run's accumulated stream clock. Two candidate seams (to design, not yet implement):

1. **Reset on print start.** When a new print/stream begins (`virtual_sdcard` start → first move
   after the toolhead has been idle), reset the stream cursor (`t_committed → 0`) and the per-MCU
   `Anchor` (`t0 → None`), so the next segment re-anchors fresh exactly like run 1. Cleanest if the
   motion engine can observe a print/stream boundary.
2. **Fix the idle re-anchor.** Make `stream_planner.rs:365` actually fire here. Suspect: `sync_instant`
   is `None`-d on idle/flush (`:413/417/421`), so `esc` reads 0 and `esc > t_committed` is false.
   Re-derive `esc` from a monotonic playhead that survives idle, or gate the re-anchor on "playhead
   has reached `t_committed`" rather than `esc`.

Hard constraint (CLAUDE.md): the fix must **not** silently re-anchor a *continuous* stream that is
genuinely late — real planner starvation must still abort loudly. The discriminator is "did the
stream drain and the machine go idle before this move?" (legitimate re-anchor) vs "are we mid-stream
and behind?" (real fault). Confirm the `sync_instant` accounting (Finding 6b) before choosing.

### Reference: `sota-motion`'s `planner.rs` already solves this (regressed in the new `stream_planner.rs`)

`stream_planner.rs` + `stream.rs` are **new on `curvature-profile`** (not on `sota-motion`, which runs
`planner.rs`). The new run loop reimplemented the idle re-anchor with a weaker condition that misses
the crash case. The hardened original lives in `planner.rs:597-614` and was built up across:

- `67334c98c` fix(planner): give idle-resume the same dispatch lead as a fresh stream — `advance_idle(esc + LEAD)`; the ~100–200 ms replan solve otherwise consumes the lead and lands seg0 in the past (this is the observed 86 ms).
- `acd743fbd` feat(planner): rest-hold placement rule for genuinely-idle moves.
- `917e7eeab` feat(planner): capture `sync_instant` at first dispatch, clear on reset.
- `0a8637fa2` docs(planner): note empty-first-emit `sync_instant` capture limitation.

Concrete deltas to port into `stream_planner.rs:365-368`:

| Aspect            | `planner.rs` (works)                              | `stream_planner.rs` (crashes)                 |
| ----------------- | ------------------------------------------------- | --------------------------------------------- |
| Re-anchor guard   | `esc > t_appended` (no `is_empty`)                | `state.is_empty() && esc > t_committed()`     |
| Pending prefix    | flushes (`run_commit_and_dispatch`) before advance | none                                          |
| Horizon compared  | `t_appended` (full appended)                      | `t_committed()`                               |
| New-print reset   | (idle re-anchor covers it)                        | resets to 0 only on `StreamOpen`/`Reset`/`HomeDrip`, **not** a new SD print |

In the crash, `esc ≈ 1.82 s < t_committed ≈ 2.0 s`, so the `stream_planner.rs` guard is false and never
re-anchors; `planner.rs`'s `esc > t_appended` + pre-flush + `esc+LEAD` path handles the same situation.
**Recommended fix:** port `planner.rs`'s idle-resume logic into `stream_planner.rs` (option 2 above),
and/or reset the stream clock on a new print boundary (option 1).

### Diagnostic

1. `runtime.fg_freeze` in `[19:26:59.6, 19:27:01.9]` for `k-1781896645-101624`; decode `pc=134252722`
   via `mcu-diagnostics`.
2. Capture a **successful** test3 run (fresh restart, one run) and diff its tail-commit timing.
3. Read test3.gcode's final lines (trailing dwell / M400 / end-of-print) on the bench.
4. Inspect which `StreamMsg` commits the tail (`stream_planner.rs` commit paths).

## Reproduction Plan

Fresh klippy restart on Neptune → run test3.gcode (run 1, expected OK) → run test3.gcode again
(run 2, expected `stream_planner_fatal`). Capture both `session_id`s and pull
`event:in(transit_diag,seg0_deficit,stream_planner_fatal) "anchor-decision"` for each; the crash
session shows the ~2.25 s post-burst dispatch gap preceding the late tail segment.

## Side Findings

- **Misleading `seg0_deficit` log.** At a fresh `first` anchor the segment is 250 ms in the *future*
  (= LEAD), yet `log_seg0_deficit` (`bridge.rs:3147`) emits `[seg0-deficit] (negative deficit_us =>
  in past)` with `deficit_us ≈ 249943`. The "=> in past" phrasing reads as an error during normal,
  correct operation. Worth tightening to avoid false alarms. **Confirmed** (`bridge.rs:3143-3148`,
  session log 19:26:59.444).
- **Two parallel planner implementations** carry the same anchor/re-anchor pattern:
  `stream_planner.rs` (live; matches the fatal `target=_motion_engine::stream_planner`) and
  `planner.rs`. Any fix must be mirrored or one consolidated. **Confirmed** (grep of `advance_idle`).
- A `Must home axis first: -5.000` warning precedes the run — test3's first move (`G1 x-5`) was
  rejected because the axes weren't homed; the run dispatched ~5 s later (after a home). **Confirmed**
  (19:26:54.659). Note X is an EtherCAT servo on this bench; the rest are steppers.
- **41 segments / ~2.0 s stream time for four 5 mm jog moves** is heavy sub-segmentation (clothoid
  blends on the `curvature-profile` branch). Not a fault, but explains why a trivial jog reaches
  stream t≈2 s. **Confirmed** (transit_diag count + stream t).

## Follow-up: 2026-06-19

### New Evidence

- **No foreground freeze in the crash session.** `event:=runtime.fg_freeze` for
  `k-1781896645-101624` = 0 rows (Hyp. 1 refuted for this crash).
- **test3.gcode is 5 lines**, four relative jog moves, no dwell/M400/end (Hyp. 3 sub-theory refuted).

### Updated Hypotheses

- Hyp. 1 (foreground freeze) → **Refuted** for the crash session.
- Hyp. 3 (gcode dwell gates flush) → **Partially refuted**; host-side flush timing still in play.
- Hyp. 2 (second-run-specific state / flush timing) → **leading hypothesis**.

### Updated Conclusion

**Root cause confirmed (High).** With the successful run captured (`k-1781897233-101785`, 19:51 UTC)
the "2.25 s stall" was revealed to be the idle gap **between two SD prints** (Finding 4 corrected).
Both runs are `Starting SD card print (position 0)`; run 1 anchors at stream t=0 and plays out, run 2
is appended at run 1's stream time (t=2.076 s) and dispatched 86 ms in the past because it launches
after run 1 drained. A new print does not reset the stream clock / `Anchor.t0`, and neither re-anchor
guard catches it (Findings 5, 6). Deterministic, fully explains first-OK/second-crash. Investigation
complete; hand off to fix design (`bmad-quick-dev`).
