# Investigation: Mainsail reports print "complete" before motion finishes

## Hand-off Brief

1. **What happened.** "Reports complete immediately" is not a reporting lag — the planner **aborts** every run (`stream_planner_fatal: OverCommitted`, 4/4 sessions; `std::process::abort()`). The host streams the whole 7.5 KB print (340 moves) in ~0.6 s with zero flow control; the resulting flood starves the real-time planner, which re-anchors and pins an entry velocity its chain re-fit can't sustain → `OverCommitted` → klippy dies; the MCU drains its ring, so motion appears to finish and Mainsail flips to "complete."
2. **Where the case stands.** Root cause Confirmed to Medium-High. Offline synchronous replay of the exact gcode — even with bench limits and giant single-batch commits — does **not** reproduce, isolating the fault to the real-time streaming commit/anchor path under starvation. The missing flow control (H1) is the trigger.
3. **What's needed next.** Fix flow control: pace the host feed on real `buffer_time` (engine true frontier), mainline-style, so the planner is never flooded/starved. This both fixes the early-complete report and prevents the OverCommit abort (paced incremental commits are proven stable offline). Latent planner-robustness issue (partial-commit entry-velocity pinning vs re-fit) remains as a secondary finding.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-22                                                                 |
| Status           | Active                                                                     |
| System           | Neptune 3 Pro bench (`ethercatpi5.local`), branch `curvature-profile`; rewrite streaming planner |
| Evidence sources | VictoriaLogs (host-rust/host-py structured logs); source: `klippy/motion.py`, `rust/motion-engine/src/{bridge,pump,stream_planner}.rs`; `klippy/extras/{virtual_sdcard,print_stats}.py` |

## Problem Statement

User: "it runs my cold_run just fine now. one thing tho, it reports in mainsail that the print is complete right away, before it's actually complete." Built to be compatible with mainline Klipper's progress/completion reporting.

## Evidence Inventory

| Source   | Status | Notes |
| -------- | ------ | ----- |
| VL session `k-1782131997-24798` (cold_run, post-drain-fix) | Available | 357 submits / 1124 dispatch pieces; burst histograms captured |
| `klippy/motion.py` `_check_pause` | Available | gates on `engine.pump_backlog()` vs watermarks (200/100); `buffer_time` only logged |
| `klippy/extras/virtual_sdcard.py` | Available | mainline-identical; `note_complete()` at EOF (`work_handler`, L356) |
| `klippy/extras/print_stats.py` | Available | mainline-identical; `state="complete"` via `note_complete` |
| `rust/motion-engine/src/pump.rs` | Available | `backlog = sum(q.pieces.len())` = unpushed pump-queue depth (L732) |
| `rust/motion-engine/src/stream_planner.rs` | Available | `last_move_time()` = `t_end` of last dispatched segment (L181, L275) |
| cold_run.gcode contents | Partial | not yet read; needed to explain the 10–30s host-idle gap |
| MCU ring depth on Neptune (pieces/axis) | Partial | code says ~496/axis; not confirmed for this config |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Trace host throttle: does anything pace the SD reader to motion, given `pump_backlog` stays low? | High | Open | `_check_pause` + bounded pump channel + submit_move synchronous path |
| 2 | Confirm `note_complete` fires at the burst (EOF), and the MCU motion frontier extends well past it | High | Open | need piece `start_time`s / MCU est_print_time at burst vs last piece |
| 3 | Explain the 10–30s host-idle gap (blocking gcode? M109/G4? large slow first moves?) | Medium | Open | read cold_run.gcode; check for blocking commands |
| 4 | Confirm pump pushes whole small print into MCU ring without pump-queue backing up (why backlog stays <200) | High | Open | pump push vs MAX_LEAD vs ring depth interaction |
| 5 | Compare to mainline: buffer_time watermark gating in toolhead | Low | Open | grounding for fix direction |

## Timeline of Events (session k-1782131997-24798)

| Time (rel) | Event | Source | Confidence |
| ---------- | ----- | ------ | ---------- |
| 0–10s | ~6 `submit_move_enter`, ~40 `pipe_axis` pieces trickle out | VL histogram | Confirmed |
| 10–30s | host idle — zero submits/dispatch | VL histogram | Confirmed |
| 30–32s | 351 `submit_move_enter` + 1088 `pipe_axis` pieces dumped at once | VL histogram | Confirmed |
| ~32s | `feed_throttle_enter` count = 0 for entire run | VL stats | Confirmed |
| ~32s | (deduced) `work_handler` hits EOF → `note_complete()` → Mainsail "complete" | virtual_sdcard.py:356 | Deduced |

## Confirmed Findings

### Finding 1: Host dispatches the print body in a single end-of-run burst, not paced to motion

**Evidence:** VL session `k-1782131997-24798`. `submit_move_enter` per-2s histogram: 2,2,0,0,1,1,0×9,351 (final bucket). `pipe_axis` histogram: 12,12,0,4,4,4,0×9,1088 (final bucket). Total 357 submits / 1124 dispatch pieces over a 32.15s span.

**Detail:** Both ingress and dispatch front-load nothing and back-load ~98% of the work into the last 2 seconds. This is the direct mechanism by which `file_position` reaches EOF far ahead of executed motion.

### Finding 2: The feed throttle never engaged

**Evidence:** Same session, `event:=feed_throttle_enter | stats count()` = 0.

**Detail:** `_check_pause` (klippy/motion.py:586-588) returns without blocking whenever `pump_backlog() <= pump_backlog_high (200)`. It never blocked across the whole run, so it provided no pacing.

### Finding 3: `pump_backlog` measures only unpushed pump-queue depth, not MCU-buffered motion

**Evidence:** `rust/motion-engine/src/pump.rs:732` — `let unpushed = queues.values().map(|q| q.pieces.len()).sum(); backlog.store(unpushed, ...)`. `q.pieces` is the deque of pieces not yet pushed to the MCU ring (popped on successful `send_frame`, pump.rs:699-702).

**Detail:** Pieces already pushed into the MCU ring are invisible to `pump_backlog`. The signal cannot reflect "seconds of motion queued ahead."

### Finding 4: The symptom is a planner abort, not a reporting lag — and it happens every run

**Evidence:** `stream_planner_fatal` present in sessions 27239, 27359, 31489, 31997 (4/4 recent). Session 31997: `error="commit: velocity plan: OverCommitted { line_no: 109 }"` at 12:40:48.591; `fatal()` calls `std::process::abort()` (rust/motion-engine/src/stream_planner.rs:206-216). Session's last events are buffered `transit_diag` flushed at 48.66–48.69 (the appender drain at abort), then the session ends.

**Detail:** The two post-drain-fix sessions (31489, 31997) both fatal; the pre-fix replay session (30620) did not — the drain fix (commit `be6580ba1`) removed the replay and exposed this next failure. klippy aborts; the MCU keeps draining its ring, so motion appears to finish and Mainsail flips to "complete."

### Finding 5: The print body is dispatched in ~0.6 s with zero pacing; both pipeline channels are unbounded

**Evidence:** host-py log "Starting SD card print (position 0)" at 12:40:47.981; all 340 print moves' `[engine-trace] move` lines fall in 47.982–48.59 (~0.6 s). Planner input channel: `let (tx, rx) = unbounded();` (rust/motion-engine/src/stream_planner.rs:78). Pump channel: `std::sync::mpsc::channel::<PumpMsg>()` — unbounded (rust/motion-engine/src/bridge.rs:2810). cold_run.gcode is 7557 bytes < the 8192-byte `work_handler` read, so the whole file is read in one `read()`.

**Detail:** `submit_move` never blocks (unbounded planner channel); dispatch never blocks (unbounded pump channel); `_check_pause` never throttled (Finding 2). Nothing paces the SD reader, so the entire file floods the planner at once.

### Finding 6: The 21 s "gap" was pre-print setup, not the print (refutes H2)

**Evidence:** host-py at 12:40:26.628 `move newpos=[117.5,117.5,10.0]` (manual Z10 positioning); SD print only starts at 12:40:47.981. The early one-line force commits (batches 0–14) and large `total_t` values belong to setup/positioning, not cold_run. cold_run.gcode has no blocking command (only `M73`; no G28/M109/G4/M400).

### Finding 7: OverCommit is a partial-commit entry-velocity / chain-re-fit inconsistency, triggered by the flood — not geometry, limits, or batch size

**Evidence:** Offline `dump_stream_trajectory` replay of the exact cold_run.gcode plans all 340 moves into ~1018 segments with **no** OverCommit, Z flat at 0.2 — at commit caps 64/167/340/512 and with both harness defaults (300/5000/5.0) and the exact bench limits (max_velocity 100, max_accel 1000, max_jerk 1e6, scv 5). Source: partial commit pins `self.entry_v = profile.moves[commit_count-1].exit_v` (stream.rs:327-331); next `plan_velocity_warm_start(&outcome, cfg, self.entry_v)` (stream.rs:237) throws `OverCommitted` when `entry_v > entry_ceiling` (velocity.rs:200) or `> entry_brake` (velocity.rs:293). Bench commit sequence (31997): batch 16 commit 4, batch 17 n=167 entry_v=3.29 commit 131, batch 18 n=163 commit 137, then the next plan OverCommitted at move 109 (the fatal post-dates batch 18's `commit_decision`).

**Detail:** Under the unthrottled flood the run_loop commits almost everything (keeps only `keep_secs`=0.5 s tail), pinning a non-zero entry velocity at the seam; the subsequent re-fit of the chain (now including more flooded moves) lowers the boundary move's curvature ceiling below the pinned `entry_v`. Steady paced/incremental commits (offline cap-64) never do this — which is why pacing the feed is expected to fix it.

## Deduced Conclusions

### Deduction 1: For a print that fits the MCU ring, the host has no motion-rate backpressure

**Based on:** Findings 2 + 3.

**Reasoning:** If the entire (small) print's pieces fit in the MCU ring (~496/axis), the pump drains its queue into the ring as fast as it receives, so `q.pieces` stays near empty → `pump_backlog` stays well below 200 → `_check_pause` never blocks → `work_handler` reads to EOF unthrottled.

**Conclusion:** `note_complete()` fires at EOF while the MCU still holds the full print — matching the symptom. Remaining gap: confirm the pump actually pushes the whole print into the ring (vs. MAX_LEAD holding pieces back, which would raise backlog).

## Hypothesized Paths

### Hypothesis 1: Wrong throttle signal — gating on pump-queue depth instead of buffer_time (USER'S THEORY)

**Status:** Confirmed (as the trigger)

**Resolution:** Confirmed by Findings 2, 3, 5. `_check_pause` gates only on `pump_backlog`, which is structurally blind to MCU-buffered motion and never tripped; both Rust pipeline channels are unbounded; the print body flooded in ~0.6 s. The missing motion-rate (buffer_time) flow control is the trigger. Its consequence is more severe than the original "early report" framing: the flood starves the real-time planner and causes the OverCommit abort (Finding 7).

**Theory:** `_check_pause` should gate on motion-time-queued-ahead (mainline `buffer_time`), but instead gates on `pump_backlog`, which is blind to MCU-ring-buffered motion. Small prints fit the ring, so the host never blocks and reports complete early.

**Supporting indicators:** Findings 1–3; Deduction 1; `buffer_time_high/low` are read from config (motion.py:137-142) but only logged, never used to gate.

**Would confirm:** Show that at the burst, the engine's real motion frontier (`get_last_move_time()`) minus `mcu.estimated_print_time()` was large (≫2s) while `pump_backlog` stayed <200 — i.e. lots of motion queued but the gate blind to it.

**Would refute:** If `pump_backlog` did exceed 200 at some point (throttle should have fired but didn't — different bug), or if motion actually finished ~when "complete" was reported (no real gap).

### Hypothesis 2: The end burst is an artifact of a blocking command releasing, not pure read-ahead

**Status:** Refuted

**Theory:** A blocking gcode line stalls `work_handler`; on release the queued remainder streams out fast.

**Resolution:** Refuted by Finding 6. The "gap" was pre-print manual setup; the SD print only started at 12:40:47.981 and ran as a single unthrottled ~0.6 s flood. cold_run.gcode contains no blocking command.

### Hypothesis 3: Completion is reported by a path other than virtual_sdcard EOF

**Status:** Refuted (superseded)

**Theory:** Mainsail "complete" comes from M73/`display_status` rather than `print_stats.state`.

**Resolution:** Superseded by Finding 4 — the run does not reach a clean EOF at all; the planner aborts (`std::process::abort()`), klippy dies mid-stream, and the MCU drains its ring. The "complete" appearance is the abort/teardown, not a normal completion path.

### Hypothesis 4: OverCommit is a standalone planner-logic bug on this geometry/limits

**Status:** Refuted

**Theory:** The zigzag infill plus tight bench limits make the velocity plan infeasible regardless of streaming.

**Resolution:** Refuted by Finding 7 — offline synchronous replay of the exact gcode with the exact bench limits and giant single-batch commits plans cleanly. The fault requires the real-time streaming commit/anchor cadence under starvation.

### Hypothesis 5: Partial-commit entry-velocity pinning is not robust to chain re-fit (latent root)

**Status:** Confirmed (mechanism), secondary to H1

**Theory:** A partial commit pins `entry_v = exit_v` at a seam; the next plan's re-fit (with newly arrived moves) can lower the boundary ceiling below the pinned `entry_v` → `OverCommitted`.

**Resolution:** Confirmed by source (stream.rs:327-331, 237; velocity.rs:200/293) and the bench commit sequence (Finding 7). This is the proximate fault, but it only manifests under the flood/starvation that H1 causes; paced incremental commits are stable (offline). Latent risk: any future real-time starvation (e.g. a genuinely slow plan) could re-trigger it, so it is worth hardening independently.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Engine frontier vs est_print_time at the burst | Directly confirms/refutes H1 | Throttle logs never fired; add a one-shot probe or read dispatch-margin `margin_us` traces in the session |
| cold_run.gcode contents | Explains H2 idle gap | Read file on bench / locally |
| Piece `start_time` schedule of the burst vs `note_complete` wall-clock | Quantifies the real "complete-too-early" gap | `[dispatch-margin]` trace (pump.rs:225) `start_ns`/`margin_us` for last pieces |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Symptom origin | `klippy/extras/virtual_sdcard.py:356` `note_complete()` at `work_handler` EOF |
| Trigger | `work_handler` reads file to EOF unthrottled |
| Condition | `_check_pause` (klippy/motion.py:573-621) gates only on `engine.pump_backlog()`; `buffer_time` unused |
| Root signal defect | `rust/motion-engine/src/pump.rs:732` backlog = unpushed pump-queue depth, excludes MCU-ring motion |
| Related | `bridge.rs:3372` `submit_move` (synchronous planner call); `stream_planner.rs:181` `last_move_time()` (true frontier, available but ungated) |

## Conclusion

**Confidence:** Medium-High. Confirmed: the symptom is a planner `OverCommitted` abort every run (Finding 4); the host streams the whole print in ~0.6 s with no motion-rate flow control and unbounded pipeline channels (Finding 5, H1); the OverCommit is a partial-commit entry-velocity vs chain-re-fit inconsistency, reproduced by the flood and absent from paced/offline runs (Finding 7, H5). Residual uncertainty: the precise run_loop/anchor cadence step that selects the infeasible seam under starvation is inferred, not single-stepped; a run_loop-level repro would make it High.

**Root cause (chain):** missing motion-rate flow control (`_check_pause` gates on `pump_backlog`, which is blind to MCU-buffered motion and never trips; both Rust channels unbounded) → the SD reader floods all 340 print moves into the planner in ~0.6 s → the real-time planner falls behind / starves and commits near-everything per batch, pinning a non-zero entry velocity at a seam → the next plan's chain re-fit lowers that boundary move's ceiling below the pinned `entry_v` → `OverCommitted` → `std::process::abort()` → klippy dies, the MCU drains its ring, and Mainsail flips to "complete."

## Recommended Next Steps

### Fix direction

**Primary (fixes both the report and the abort):** give the host real motion-rate flow control. Gate `_check_pause` on motion-time-queued-ahead using the engine's true frontier — `engine.get_last_move_time() - mcu.estimated_print_time()` vs `buffer_time_high`/`buffer_time_low` (already in config, currently only logged), mainline-style — instead of (or in addition to) `pump_backlog`. This paces the SD reader to motion (so `note_complete` tracks real progress) and prevents the flood that starves the planner. Paced incremental commits are proven stable offline (Finding 7).

**Secondary (harden, latent):** make partial-commit entry-velocity pinning robust to re-fit — e.g. clamp the committed exit velocity to the re-fit boundary ceiling, or re-plan/degrade instead of `process::abort()` on a recoverable `OverCommitted`. Aborting the whole klippy process on a continuity violation is severe and turns a recoverable planner condition into a crash.

### Diagnostic

Confirm H1 quantitatively (frontier vs est at burst) and read cold_run.gcode for H2.

## Reproduction Plan

Run `cold_run.gcode` on the Neptune bench; observe Mainsail flips to "complete" within ~1–2s of the dispatch burst while motion continues. Logs: `submit_move_enter` / `pipe_axis` histograms show end-loaded burst; `feed_throttle_enter` count 0.

## Side Findings

- Earlier session `k-1782130620-23962` (347 submit / 60676 dispatch) is the pre-fix replay bug (170× re-dispatch), already fixed (commit `be6580ba1`); not relevant to this case except as a baseline.

## Follow-up: 2026-06-22 #2

### Thread

Test whether the streamed `OverCommitted` is a missing brake-to-rest look-ahead
guarantee (the recalled spec: do not commit a seam unless the kept tail can
decelerate the committed exit velocity to a full stop), or the windowed-fit
inconsistency (Follow-up #1 / H5 / take-3).

### New Evidence

Instrumented `geometry::velocity::plan_velocity_warm_start` to log move 55's
boundary corner cap (`v[k]`, `boundary_vlim`, endpoint curvature) in every fit
window it appears in, replaying cold_run at one-move-per-commit (bench limits
100/1000, jerk 1e6). Result:

```
plan[45..77] … plan[53..86]:  v[55]=60.000  boundary=63.496  kappa=0.24803
plan[54..87] (aborts):        v[55]=45.063  boundary=45.063  kappa=0.49244
```

Move 55's fitted endpoint curvature **doubled (0.24803 → 0.49244)** the instant
the window front advanced one move (53→54, i.e. one more move committed),
dropping its corner cap from 60 to 45 mm/s — below the already-committed
`entry_v ≈ 62.7`. That is the `OverCommitted { line_no: 55 }` abort.

### Additional Findings (Confirmed)

- **Finding 8: A move's fitted corner cap is not invariant to the fit window's
  front edge.** Trimming one committed move off the front doubled move 55's
  curvature. Evidence: instrumentation above, deterministic.
- **Mechanism (Confirmed by source):** biclothoid blends consume length from the
  moves they join (`blend_trim`, `head_consumption`, fitter.rs:200-205). When the
  committed move ahead of a corner is trimmed, the now-leading move becomes the
  chain head and takes the `r.start == 0` head-reserve branch (fitter.rs:203),
  changing the length budget available to the *next* corner's blend, which
  re-solves the biclothoid to a different (here 2×) curvature.

### Updated Hypotheses

- **H6 (user's recalled spec — brake-to-rest look-ahead missing): Refuted.** The
  warm-start always pins terminal `v = 0` (velocity.rs:108-115) and `keep_secs`
  holds back a trailing tail (stream.rs:19-23), so committed exit velocities are
  brakeable to a stop within the kept tail by construction. The aborting term is
  the *corner* cap (`disk_reach_v_rev` to `v[1]`, velocity.rs:281-291), not the
  brake-to-stop term — and that corner cap *changed after commit*. The spec is
  real and honored; it does not cover a post-commit curvature change.
- **H5 (windowed-fit inconsistency / take-3): Confirmed as root, large
  magnitude.** Same mechanism as the 1% line-53 jitter, here 2× curvature / ~25%
  cap drop. Front-edge blend-budget non-determinism.

### Updated Conclusion

Root cause of the genuine (non-jitter) `OverCommitted` is **front-edge chain-fit
non-determinism**: committing+trimming a move changes the leading corner's blend
budget, re-fitting it at a higher curvature whose lower corner cap the already
-committed entry velocity violates. The brake-to-rest invariant is not the
defect. Fix = take-3 (window-invariant fit at the committed seam): the leading
move after a commit must reserve the same head budget / reproduce the same blend
geometry it had when its leading seam was committed, so its corner cap cannot
drop below the committed exit velocity. The velocity-guard slack
(WARM_START_REFIT_SLACK_REL) only absorbs the small-magnitude tail of this same
effect and cannot (and must not) absorb the 2× case.

## Follow-up: 2026-06-22 #3

### Thread

After take-3 (commit ac2288830) fixed the fit `OverCommitted` crash, the ORIGINAL
symptom remains: Mainsail "requested position" jumps far ahead (to ~layer 2 / for
a short print, straight to the final position) and the print reports complete long
before motion finishes; a new crash then occurs mid-layer-2. User's steer:
"segment in the past" is a symptom; the core is the requested position not tracking
motion — the host runs a lot of gcode upfront and fills the buffer.

### New Evidence (session k-1782142874-30191, extrusion print, post-take-3)

- **No `stream_planner_fatal`** — take-3 held (the fit OverCommit is gone).
- `submit_move_enter` histogram (per 5s): ~6 in 0–15s (setup), then **351 in one burst at t=20s**, a 65s gap, then **866 at t=85s** (the crash). Submission is bursty, not paced — each burst is ~an MCU-ring's worth of motion.
- Crash chain: repeated `seg0_deficit "negative deficit_us => in past"` (deficit ~250 ms) → `send_frame_transient: MCU rejected PushPieces -142` (axes 2,3) → `fault_event_received fault_code=-308` (15:46:40.747) → `send_frame_fatal: ethercat PushPieces mcu 1: Closed` (15:46:40.888). The transport/MCU fault is downstream of the over-fill, not a root cause.

### Confirmed Findings

- **Finding 9: "requested position" tracks SUBMISSION, not execution.** `motion.commanded_pos[:] = move.end_pos` at submit (klippy/motion.py:402), reported as get_status `position` (motion.py:334). Mainsail's requested position = `gcode_move.gcode_position` = `last_position - base_position` (klippy/extras/gcode_move.py:124-145), and `last_position` advances as each G1 is interpreted (submission). So the reported requested position is the last *submitted* move's endpoint.
- **Finding 10: submission is unthrottled w.r.t. motion (H1 mechanism, still present).** `_check_pause` gates only on `engine.pump_backlog()` (klippy/motion.py:586-588); `buffer_time_high/low` are read from config (137-141) but never used to gate. `pump_backlog` is pump-queue depth, which stays low until the (large) MCU ring fills — so the host ingests ~a full ring ahead before blocking, in bursts. For a print smaller than one ring, the whole file ingests at once → requested position jumps to the end → `note_complete` at EOF immediately.

### Updated Hypotheses

- **H1 (wrong throttle signal — gate on pump_backlog not buffer_time): Resolved.** This was the root of the requested-position-jumps-ahead + early-complete symptom AND the over-fill that lands pieces in the MCU past (−142 → fault → crash). The fix is implemented in `_bmad-output/implementation-artifacts/spec-submission-aware-backpressure.md`: the host now gates `_check_pause` on a submission-aware engine signal `queued_motion_secs = (get_last_move_time − est) + uncommitted_intake_secs` (the planner frontier plus a planner-thread nominal tally of received-but-uncommitted moves) against `buffer_time_high/low`, the planner input channel is a bounded fail-loud backstop, and the `MAX_LEAD_SECS` 2.0 band-aid is reverted to 1.0. `pump_backlog` is demoted to diagnostics.

### Conclusion (Follow-up #3)

**Confidence: High.** The requested position and completion track unthrottled bursty submission; the host fills ~an MCU ring ahead because `_check_pause` gates on pump-queue depth, not motion-time-ahead. Fix = real `buffer_time` flow control: gate the host feed on the engine's TRUE motion frontier `engine.get_last_move_time() - mcu.estimated_print_time()` against `buffer_time_high/low` (already in config; currently only logged) — NOT the host's projected `_mcu_pending_end_time` (the earlier "phantom buffer" that caused under-feed). This paces submission so `gcode_position`/`commanded_pos` track motion within ~2 s, `note_complete` fires near the real end, and the over-fill (hence −142/crash) is prevented. Keep `pump_backlog` only as a secondary piece-count safety if desired.

## Follow-up: 2026-06-23

### Thread

Symptom is BACK on branch `ethercat-ipc-hardening` despite the spec (`spec-submission-aware-backpressure.md`, status `done`): bench user reports the requested position/current layer in Mainsail races **minutes / ~20 layers** ahead of physical motion, the gap grows monotonically, and prints intermittently crash with PieceStartInPast (`-308`). Question driving this follow-up: "can we advance the bookkeeping when the queue is actually CONSUMED, and what does the pipeline + inter-stage queueing look like now?"

### New Evidence

**The spec's host gate was reverted in three post-spec commits — the throttle now gates on exactly the signal the spec's `Never` list forbids.**

| Commit | Gate signal it installed | Stated reason it moved |
| ------ | ------------------------ | ---------------------- |
| `cb75014f0` (spec impl) | `queued_motion_secs` (engine: optimized frontier + uncommitted) | — (the spec) |
| `917b8b29f` | same, grounded to host clock | re-anchor produced two-clock mismatch |
| `ab538756d` | **`_mcu_pending_end_time - est`** (host, Σ `min_move_t`) | claimed engine signal "collapsed to 0 after re-anchor" (two clocks) |
| `d437ce8a0` | `dispatched_lead_secs` (committed frontier only) | `_mcu_pending_end_time` over-read lead (2.31s) vs delivered (0.72s) → starved MCU |
| `a46fea2db` (**current HEAD**) | **`_mcu_pending_end_time - est`** restored | `dispatched_lead_secs` blind to the 8192-deep input-channel backlog → flood → `-308` |

**Confirmed — current gate (the regression):** `klippy/motion.py:661` `buffer_time = self._mcu_pending_end_time - est`; `_check_pause` (motion.py:656) throttles on it. `_mcu_pending_end_time` accumulates `move.min_move_t` (motion.py:471/498 → `_bump_pending_end_time` 642-647). `min_move_t = move_d / feedrate` (motion.py:455) = cruise time at top speed, **no accel ramps, no junction slowdowns**. The spec's `Never` (spec line 34): *"Gate on … host-side all-nominal `_mcu_pending_end_time`."*

**Confirmed — the correct engine signal still exists but is unused by the gate.** `queued_motion_secs()` (`bridge.rs:3838-3860`) = `(t0 + last_move_time − host_now) + uncommitted`. `last_move_time` = `last.t_end` (`stream_planner.rs:460/482/557`) = the **optimized** segment end (real planned time, post junction/jerk/biclothoid), NOT nominal. It is wired only into `lookahead_end_print_time()` (motion.py:632), not into `_check_pause`.

### Confirmed Findings

- **Finding 11: all three gate signals tried so far structurally UNDER-report the real-time motion buffer — each for a different reason.** (1) `_mcu_pending_end_time` counts `min_move_t`-seconds; motion executes in real-planned-seconds (≥ min). The gate holds far more real motion than `buffer_time` claims → requested position runs ahead. (2) `dispatched_lead_secs` excludes the up-to-8192-move input-channel backlog → blind → flood. (3) `queued_motion_secs` (the one with the right *content* — optimized frontier, consumption-anchored via `est`) was abandoned over a re-anchor clock-grounding concern. Net: every shipped gate has read LOW, so the feeder over-runs.
- **Finding 12: the gate signal and the execution clock use different time units.** `min_move_t` (cruise lower bound) vs `t_end` (optimized). For a thin/fast cube where a layer is ~1-2 s of real motion, even a bounded `real/min` over-buffer of tens of seconds reads as ~20 layers — matching "20 layers / minutes ahead." Whether it is strictly unbounded or a large bounded ratio is not yet separated (needs the per-move `real_t / min_move_t` distribution on the cube).

### Pipeline Map (stages and the queue between each)

```
gcode reader (virtual_sdcard work_handler)
  │  motion.move() → engine.submit_move()  [SYNC FFI; + _bump_pending_end_time(min_move_t); + _check_pause()  ← GATE HERE]
  ▼
[Q1] planner input channel — bounded(INPUT_CHANNEL_CAP=8192), try_send fail-loud   (stream_planner.rs:184,326)
  ▼
stream planner thread (run_loop): junction-v / jerk-SLP / biclothoid fit → optimized segments
        frontier = last.t_end (stream time) ; uncommitted_intake_secs += nominal_t on intake, −= on commit
  ▼
[Q2] dispatch: coalesce ≤64 moves, lower to per-axis pieces → PumpMsg
  ▼
[Q3] pump channel — std mpsc (unbounded)   (bridge.rs)
  ▼
pump: per-axis held-queue q.pieces (the deep backlog); ship to MCU/ec-rt up to horizon (MAX_LEAD_SECS) & room
        pump_backlog = Σ q.pieces.len()  (unpushed only)
  ▼
[Q4] ec-rt AxisRing — capacity 1024 (ethercat); MCU piece_ring (serial). plays at DC cycle, retires pieces
  ▼  heartbeat: retired_count (+ now real ring_len=len/capacity, fixed 2026-06-23) → pump frees room
physical axis
```

The "advance on consumption" the user asks for already has a home: **`est = mcu.estimated_print_time(now)` rises as `[Q4]` retires pieces** (real execution). A gate of the form `frontier − est` therefore *automatically* advances as the queue is consumed — provided `frontier` is the real optimized motion end and is on the same clock as `est`. That is exactly `queued_motion_secs`; it is the only candidate whose numerator is real (not `min_move_t`) and whose anchor is consumption (not submission count).

### Updated Hypotheses

- **H7: the correct fix is to gate on a consumption-anchored, real-duration frontier (`queued_motion_secs`), and the blocker is solely its re-anchor clock grounding.** Status: **Confirmed — grounding holds; abandonment was premature.** `last_move_time` is stream-relative; `queued_motion_secs` adds `t0` to ground it to host time. **Confirming evidence:** new tests `rust/motion-engine/src/anchor/tests.rs::grounded_frontier_reports_real_queued_seconds_after_reanchor` and `::grounding_cancels_the_stream_time_baseline` drive an idle-gap underrun re-anchor at a 500 s stream baseline and show the grounded frontier `t0 + last_move_time − host_now` reports REAL queued seconds (`lead + committed_span − playhead_advance`), independent of the baseline — it does NOT collapse to 0. `Anchor::anchor_segment` (anchor.rs:61) recomputes `t0 = host_now + lead − seg_t_start` on every re-anchor, cancelling the stream-time offset that `ab538756d` blamed for the collapse. That commit's "two different clocks → collapsed to 0" describes the *ungrounded* form; `917b8b29f`'s `t0` grounding (still live in `queued_motion_secs`, bridge.rs:3860) already fixed it. **Residual (bounded, not a collapse):** the host reads `t0` (dispatch_anchor) and `last_move_time` (planner) as two atomics updated per-segment in the dispatch path; a read mid-commit sees ≤1 segment of skew (~ms), not a collapse. **Net: the engine signal is safe to gate on.**
- **H8: `min_move_t` is the wrong duration unit for backpressure regardless of clock.** Status: **Confirmed (mechanism).** Even with a perfect clock, `Σ min_move_t` ≠ real motion seconds; only the optimized `t_end` reflects what the MCU will actually take. Any host-side nominal accumulator repeats this error.

### Fix Direction

Gate `_check_pause` on the engine's optimized, consumption-anchored frontier (`queued_motion_secs`-style: `(frontier − est) + bounded_uncommitted`) instead of `_mcu_pending_end_time`. Prerequisite: settle H7 — verify (and if needed repair) the re-anchor clock grounding so the signal cannot collapse. Bound the uncommitted tail by keeping the input channel shallow enough that, with a working gate, `[Q1]` never deep-fills (the 8192 cap is a fail-loud backstop, not a working depth) — this is what makes the nominal `uncommitted_intake_secs` term negligible, closing the `dispatched_lead_secs`-was-blind complaint (`a46fea2db`) without reverting to `min_move_t`.

### Status

**Fix implemented (2026-06-24).** H7 confirmed (grounding holds; new anchor tests). `klippy/motion.py:_check_pause` now gates on `self.engine.queued_motion_secs()` (the real optimized frontier minus host clock, `+ uncommitted_intake_secs`), replacing `_mcu_pending_end_time − est` (the `min_move_t` phantom buffer). The signal includes the input-channel backlog (via `uncommitted_intake_secs`), so it is not blind like `dispatched_lead_secs` (closes `a46fea2db`); the 8192 channel stays as the fail-loud backstop (unchanged). Test `test/test_motion_backpressure.py` rewritten to drive the gate through a `FakeEngine` exposing the new signal; 6/6 green, ruff clean. A genuine MCU stall is caught fail-loud by the bounded channel (the wall-clock signal drains, then `submit_move` errors at cap) rather than by `DRAIN_TIMEOUT`.

**Residual (out of scope, latent):** `_mcu_pending_end_time` is still accumulated from `move.min_move_t` (motion.py:471/498) and still feeds non-gate logic — `get_last_move_time()` / `check_busy` idle detection and any per-commit command scheduling (e.g. fan/pin at queued-motion end, `9fbb8b803`). Those under-estimate the real motion end by the same `real − min` gap, so e.g. a fan keyed off `get_last_move_time()` fires slightly early. The spec's `Always`/`Ask First` deliberately left this plumbing; worth a follow-up to point those consumers at the engine frontier too. Needs bench validation that requested position now tracks motion within ~1-2 s.
