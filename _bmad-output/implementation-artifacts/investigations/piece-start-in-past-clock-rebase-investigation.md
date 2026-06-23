# Investigation: PieceStartInPast (-308) from clock-offset sample jitter

## Hand-off Brief (15-second read)

The print crashes with MCU fault **-308 PieceStartInPast**, not the over-fill (-142) bug.
**Two writers** feed the engine router's single mcu0 clock anchor every clocksync sample:
the dedicated callback (projection-correct: `time_avg + min_half_rtt`, `clock_avg`) AND the
legacy `serialhdl.set_clock_est` path (`time_avg + TRANSMIT_EXTRA`, **`clock_avg − 3·pred_stddev`**).
The second path's `3σ` term is a serialqueue *transmit* safety margin (~1720 µs here) that has
no business in the motion anchor; the two anchors' projections differ by **~2 ms** — 10× the
MCU's 200 µs PieceStartInPast budget. When the serialhdl path writes the slot last and a
dispatch follows, pieces get `start_time`s ~2 ms early → in the MCU's past → -308 → reset.
**The backpressure gate is confirmed healthy and is NOT the cause** — it merely lets the print
run long enough to hit this. Fix: stop the serialhdl path from writing the engine router; feed
the anchor only from the dedicated callback. The MCU-side 200 µs guard stays untouched.

## Case Info

- **Slug:** piece-start-in-past-clock-rebase
- **Date opened:** 2026-06-22
- **Bench:** Neptune (ethercatpi5.local), VictoriaLogs `http://ethercatpi5.local:9428`
- **Branch:** curvature-profile
- **Crash session:** `k-1782156166-35485`, print `print-1782156195`, fault at 19:23:25.866Z
- **Status:** Active — root cause Confirmed (mechanism), fix direction open
- **Predecessor case:** `print-completes-early-investigation.md` (backpressure — RESOLVED, gate
  oscillates 1.0↔2.0 cleanly; this is the next bug it exposed)

## Problem Statement

User report: "the reporting works better, but it seems like gcode is submitted faster than it
is consumed. it shows the current printing speed of 32000mm/s and crashes after a while. and
on a bigger file it crashes right away." Premise check: submission is **paced** (gate
healthy); the 32000 mm/s and crash are a *projection* anomaly, not a submission-rate problem.

## Evidence Inventory

| Evidence | Grade | Source |
|---|---|---|
| Fault is -308 PieceStartInPast (detail 67737, wire_u16 65228) | Confirmed | `fault_event_received`, session 156166, 19:23:25.866 |
| Crash chain: seg0_deficit(healthy) → junction_jump_anomalous → -308 → conn EOF | Confirmed | VL warn+error stream, session 156166 |
| junction_jump_anomalous reason=projection_divergence, tick_jump=90.38µs, host_jump≈0, axes 1/2/3 | Confirmed | VL payload, 19:23:22.963 |
| Gate healthy: buffer_time oscillates 2.0↔1.0, est advances at wall-rate | Confirmed | `feed_throttle_*`, session 156166 |
| `set_clock_est_rebased` fires every ~985 ms all print long (per MCU) | Confirmed | VL clocksync stream, session 156166 |
| Consecutive mcu0 rebases: freq stable, RAW-vs-MONOTONIC Δ=0.02µs, offset_raw Δ=−243µs | Confirmed | VL rebase payloads, 19:23:25.844 |
| junction_jump_anomalous present in pre-gate sessions (1782142874, 143218, 152807) | Confirmed | VL history (prior turn) |

## Confirmed Findings

1. **The fault is -308 PieceStartInPast.** Not -142 over-fill, not -311. (`fault_event_received
   fault_code=-308`.)
2. **The MCU drift budget is 200 µs.** `MAX_START_IN_PAST_SECS = 200e-6` →
   `deficit = now - entry.start_time > 200µs + sample_period` raises -308.
   `rust/runtime/src/motion_core.rs:114-127`.
3. **The host→tick projection is the live clocksync estimate, re-read per batch.**
   `project = |mcu_id, host_secs| r.host_time_to_mcu_clock(...)`
   `rust/motion-engine/src/bridge.rs:3176-3180`; the piece `start_time` is set from it at
   `rust/motion-engine/src/enqueue.rs:209-210`.
4. **The projection is `last_clock + (host_secs - clock_offset)·freq`.**
   `rust/host-rt/src/passthrough_queue/router.rs:487-489`. A `clock_offset` excursion biases
   every freshly-dispatched start_time by `Δoffset·freq` ticks.
5. **`set_clock_est_rebased` wholesale-overwrites `clock_freq`/`clock_offset`/`last_clock`** with
   no Rust-side smoothing. `rust/host-rt/src/passthrough_queue/router.rs:359-361`.
6. **The discontinuity originates in `offset_raw` (Python side), not the Rust RAW re-read.**
   Across two consecutive rebases: `freq` identical; `bridge_now_raw - bridge_now_instant`
   constant to 0.02 µs; `offset_raw` jumped −243 µs. The Rust rebase math
   (`clock_offset = offset_raw - (bridge_now_raw - bridge_now_instant)`,
   `router.rs:340-342`) faithfully passes Python's jitter through.
7. **The gate is healthy and exonerated.** `buffer_time` oscillates 2.0↔1.0; `est` advances at
   true wall-rate; clocksync cadence is a clean ~985 ms with stable `freq`. The gate does not
   stall clocksync.

## Deduced Conclusions

- **Root cause (Confirmed):** The engine router keeps **one** clock anchor per MCU, but **two
  independent writers update it every clocksync sample** with different conventions:
  - **Path A (legacy, contaminating):** `clocksync.py:172` → `serial.set_clock_est(new_freq,
    time_avg + TRANSMIT_EXTRA, int(clock_avg − 3·pred_stddev), clock)` → `serialhdl.py:464`
    → `_motion_engine.set_clock_est` → `set_clock_est_rebased`. Anchor = (offset
    `time_avg + TRANSMIT_EXTRA`, last_clock `clock_avg − 3σ`).
  - **Path B (correct):** `clocksync.py:188` → `_engine_clock_est_cb` (`mcu.py:1279`) →
    `_motion_engine.set_clock_est` → `set_clock_est_rebased`. Anchor = (offset
    `time_avg + min_half_rtt`, last_clock `clock_avg`).

  Both target the same `McuHandle(0)` slot; whichever writes last wins until the next sample,
  so the router anchor flips between A and B every ~985 ms. Path A's `−3·pred_stddev` is a
  serialqueue *transmit* safety margin (schedule commands early); in the crash session
  `3σ ≈ 144536 ticks ≈ 1720 µs`. Net projection difference between the two anchors for a fixed
  host instant ≈ **−2 ms** (dominated by the `3σ` term, plus `(TRANSMIT_EXTRA − min_half_rtt)`).
  When Path A writes last and a piece is dispatched, its `start_time` is projected ~2 ms early;
  by the time the MCU reaches it, `now − start_time > 200 µs` → **-308**.
- **Why intermittent / bigger-file-crashes-sooner:** `pred_stddev` (σ) breathes with sync
  quality. Small σ → divergence < 200 µs → survives; large σ → divergence ≫ 200 µs → crash.
  More dispatches (bigger file) = more chances to land a dispatch right after a Path-A write.
- **junction_jump_anomalous (tick_jump≈90µs, host_jump≈0)** is the host-side early-warning
  image of the same anchor flip — `host_jump≈0` proves the planner intended contiguity, so the
  divergence is purely in the host→tick anchor. The anchor.rs timeline is innocent; the
  contaminating writer is the culprit. (90 µs is the residual visible at a *specific* junction;
  the full A-vs-B gap is ~2 ms.)
- **Why now:** Pre-gate, the submission flood crashed the print (-142) before it ran long
  enough to land a dispatch on a Path-A write with large σ. With the gate pacing submission,
  the print runs for tens of seconds and hits it. junction_jump_anomalous in pre-gate sessions
  confirms the double-writer pre-existed.
- **The 32000 mm/s (Hypothesized):** A UI/reporting artifact of `requested_pos` jumping when
  the anchor flips, not a planned velocity. Secondary; not yet pinned.

## Hypothesized Paths

| # | Hypothesis | Status | Confirm/Refute |
|---|---|---|---|
| 1 | New code not running on bench | Refuted | feed_throttle_* + gate oscillation prove curvature-profile build is live |
| 2 | Backpressure gate stalls clocksync, degrading the estimate | Refuted | freq stable, cadence clean ~985 ms, est tracks wall-rate |
| 3 | NTP slew (RAW vs MONOTONIC) injects the per-sample offset jump | Refuted | `bridge_now_raw - bridge_now_instant` constant to 0.02 µs across rebases |
| 4 | Re-anchor (t0 jump) causes the divergence | Refuted | host_jump≈0 at the anomaly — a t0 jump would move host_jump too |
| 5 | Single-writer offset jitter >200 µs | Superseded by #7 | The "jitter" was an artifact of differencing two interleaved writer tracks |
| 6 | 32000 mm/s is a reporting artifact of requested_pos jump at anchor flip | Open | Pull requested_pos / motion_report around a junction anomaly |
| 7 | Two writers (serialhdl `set_clock_est` + cb) clobber the single mcu0 anchor with conflicting conventions | **Confirmed** | Two records/sample, same freq, last_clock 1720µs (=3σ) apart, offset 243µs apart; serialhdl.py:464 + mcu.py:1284 both call `_motion_engine.set_clock_est` |
| 8 | mainline clocksync smoothing was dropped in the rewrite | Refuted | clocksync.py `_handle_clock` is the full DECAY-weighted regression with outlier rejection, intact |

## Timeline (crash session 156166)

```
19:22:48  print start (host-py)
19:22:53–19:23:15  seg0_deficit ×~20  (= +0.25s lead, HEALTHY — not the fault)
19:22:55–19:23:14  set_clock_est_rebased every ~985 ms (per MCU), freq stable
19:23:22.963  junction_jump_anomalous ×3 (axes 1/2/3) projection_divergence, tick_jump=90µs
19:23:25.844  rebase pair: offset_raw Δ=−243µs (clock-offset excursion)
19:23:25.866  fault_event_received -308 PieceStartInPast
19:23:26.051  conn EOF / send_frame_fatal  → MCU reset
```

## Source Code Trace

- **Fault origin:** `rust/runtime/src/motion_core.rs:114-127` (`get_piece_for_time`,
  `MAX_START_IN_PAST_SECS = 200e-6`, `fault.piece_start_in_past`).
- **Trigger:** a freshly-dispatched piece whose `start_time` is >200 µs behind MCU `now`.
- **Condition:** `clock_offset` reseeded with a jittery `offset_raw` between the dispatch of
  that piece and its arrival at the MCU playhead.
- **Projection:** `rust/host-rt/src/passthrough_queue/router.rs:487-489`
  (`host_time_to_mcu_clock`).
- **Reseed:** `router.rs:332-363` (`set_clock_est_rebased`), called per clocksync sample.
- **Dispatch bake-in:** `rust/motion-engine/src/bridge.rs:3176-3206` (project closure) →
  `rust/motion-engine/src/enqueue.rs:209-210` (`start_time = project(mcu_id, host_secs)`).
- **Early-warning detector:** `rust/motion-engine/src/pump.rs:279-500`
  (`junction_jumps`, threshold `(tick_jump - host_jump).abs() > 50µs`).
- **Upstream (not yet read):** the Python clocksync / mirror callback that computes
  `offset_raw = time_avg + min_half_rtt` and calls into `set_clock_est_rebased`. This is where
  the 200µs+ offset jitter is produced or could be smoothed.

## Final Conclusion

**Confidence: High** (Confirmed end-to-end in logs + code). The crash is -308 PieceStartInPast,
caused by **two writers clobbering the engine router's single mcu0 clock anchor** every
clocksync sample with conflicting conventions; the legacy `serialhdl.set_clock_est` path
carries a serialqueue `−3σ` transmit-safety bias (~1720 µs) into the motion anchor, producing
a ~2 ms projection swing that puts freshly-dispatched pieces in the MCU's past. The backpressure
gate is healthy and exonerated. The clocksync regression itself is correct and intact.

## Fix Direction

**Preferred — sever the contaminating writer (one-spot, low-risk):** the engine router anchor
must be fed **only** by the dedicated `_engine_clock_est_cb` (Path B: `time_avg + min_half_rtt`,
`clock_avg`). Make `serialhdl.set_clock_est` (`serialhdl.py:460-470`) stop calling
`_motion_engine.set_clock_est`. The `conv_time`/`conv_clock` it carries (`TRANSMIT_EXTRA`,
`clock_avg − 3σ`) are serialqueue *transmit-scheduling* parameters, not projection parameters —
exactly what the `clocksync.py:183` comment warns must not reach the anchor. Path B already
updates the anchor every sample with the projection-correct, smooth values, so severing Path A
does **not** stop reseeding and does **not** make pieces go stale.

- Verify nothing else depends on `serialhdl.set_clock_est` reaching the engine in engine mode
  (it early-returns when `_motion_engine is None`, so today it exists *solely* to feed the
  engine — strong sign it is pure contamination). If the engine needs a separate
  transmit-scheduling clock, give it a **distinct** slot, not the motion anchor.
- Keep all three safety layers: smooth single-writer anchor (prevent) + anchor.rs
  re-anchor-forward on underrun (stutter, not crash) + MCU 200 µs budget (final tripwire,
  untouched).

**Reject unless agreed — widen the 200 µs budget.** Masks the instability, violates fail-loud,
risks real step bursts.

## Diagnostic Steps (to close remaining gaps)

- Confirm the fix end-to-end on the bench: after severing Path A, the per-timestamp double
  `set_clock_est_rebased` records collapse to one, junction_jump_anomalous stops, and the print
  completes without -308.
- Pull `requested_pos` / motion_report around a junction anomaly to confirm the 32000 mm/s
  reporting artifact (Hypothesis 6).

## Reproduction Plan

Run any multi-minute print on the Neptune bench on curvature-profile; -308 appears within tens
of seconds once a clocksync sample's offset excursion exceeds 200 µs. Larger files crash sooner
(more dispatches → more chances to land on a bad sample), matching the user's "bigger file
crashes right away."

---

## Follow-up: 2026-06-22 #2 — double-writer fix landed, -308 persists (DIFFERENT root cause)

The double-writer fix was implemented and is **confirmed working**, but the print still crashes
-308. The new crash is a **delivery-latency** fault, not a projection fault.

### Confirmed (session k-1782159613-36385, fault 20:21:53.190, detail 200254 = axis 3, ~3.6 ms)

- **Double-writer fix works.** Now **one** `set_clock_est_rebased` record per timestamp (was
  two); `set_clock_est` / Path A fires **0** times; `junction_jump_anomalous` divergence dropped
  from ~2 ms to **~70 µs** (`tick_jump_us=69.7`, `host_jump≈0`). The clock anchor is clean.
- **The -308 is now late delivery, not bad projection.** `transit_diag_alert` at the fault:
  `"piece arrived in MCU past (arrival_lead<0)"` — `arrival_clock − host_front_start_time < 0`.
  The pump *sent* the front piece **after** its scheduled start: mcu0 axis3 `arrival_lead =
  −2905 µs` (trigger), cascading to −2024, −34503, −39900 µs across axes. The projection is
  correct; the piece reached the MCU too late.
- **Nominal pipeline is healthy.** `feed_throttle` oscillates cleanly 1.0↔2.0; `dispatch_committed`
  t_end advances ~1 s/s; no dispatch gap. The host keeps up in *nominal* time, but the *just-in-time*
  pump delivery (lead `MAX_LEAD_SECS = 1.0 s`, `pump.rs:300`) slipped ~1 s, draining the MCU
  buffer until the playhead overran the front piece. Commit `48ad66dd4` had temporarily widened
  this to 2.0 s (a "thin-buffer" band-aid) and was reverted — re-exposing the thin margin.
- **`freq` instability was startup-only** (84041652 @ 20:17 → settled ~84002200 by 20:37, stable
  through the fault). Not a factor.
- **Second anomaly — the host MCU is being given motion it can't keep on time.** `McuHandle(1)`
  is the Pi Linux/process MCU (`freq = 1e9` ns-ticks; `mcu_at_send ≈ 90109 s`; seeded once via
  `set_clock_est_from_sample`, **never** regression-synced — correct for a host MCU). Only one
  physical board exists (`mcu`, CH340 USB, handle 0). Yet the engine dispatches an **axis-0 motion
  stream** to McuHandle(1) (`seg0_deficit` ×16, transit_diag axis 0 `arrival_lead = −6568 µs`).
  Whether the host MCU should receive a motion stream at all is an open config/engine question.

### Hypotheses (this follow-up)

| # | Hypothesis | Status | Confirm/Refute |
|---|---|---|---|
| 9 | -308 is now caused by pump delivering pieces later than the 1.0 s just-in-time lead | **Confirmed** | transit_diag `arrival_lead<0`; nominal pipeline healthy; reverted 2.0 s band-aid |
| 10 | A host compute spike (per-move re-lowering) starves the pump thread → late send | Open | Burst of all pipe stages at one timestamp (21:52.077) is suggestive; need per-stage durations / CPU evidence |
| 11 | Host MCU (McuHandle(1)) receiving an unsynced motion stream aggravates shared pump/serial | Open | mcu1 axis0 late by 6.5 ms; shared transport contention plausible but not isolated |

### deferred-work.md relevance (user's question)

**Partial / contributing, not the whole story.** The throughput items match the
"host falls behind real-time → late delivery" mechanism:
- *"Redundant re-lowering of the held-back tail"* (continuity-commit follow-up): O(tail) per-move
  CPU, "dominant per-move cost for very dense streams." A CPU spike starving the pump fits H10.
- *"ChannelFull + gate blindness to in-channel depth"* (backpressure follow-up): explicitly
  "a CPU-bound planner that falls behind a fast feeder."

NOT covered by deferred-work, and more proximate per the logs: (a) the thin pump delivery lead
(`MAX_LEAD_SECS` reverted from the 2.0 band-aid) — H9, Confirmed; (b) the host MCU receiving an
unsynced motion stream — H11. So deferred-work explains a *contributing* throughput pressure but
not the proximate trigger.

### Fix direction (this follow-up)

- The principled fix is **not** simply re-raising `MAX_LEAD_SECS` (that's the band-aid). It is to
  guarantee the pump never *sends* a piece whose start is already in the MCU's past: detect
  `arrival_lead < 0` at the pump and re-anchor-forward (stutter), and/or make the delivery lead
  robust to host scheduling jitter. Pair with eliminating the host-side stall source (H10).
- Resolve the host-MCU-gets-motion question (H11): if McuHandle(1) shouldn't carry steppers,
  the engine must not enroll it as a motion target; if it should, it needs a real time base.
- Keep all three safety layers; do not widen the MCU 200 µs budget.

**Status: Active** — H9 Confirmed (delivery latency); H10/H11 open; deferred-work assessed as
contributing-not-proximate.

## Follow-up: 2026-06-23 — incremental planning shipped, crash persists; backpressure signal is the proximate root cause

Incremental stream planning (spec `_bmad-output/specs/spec-incremental-planning/`, impl
`spec-incremental-stream-planning.md`) was implemented and run on the EtherCAT bench. **Print still
crashes with the same -308 PieceStartInPast.** Session `k-1782180588-39437`, print
`print-1782180628`, fault 02:10:38.884, `fault_detail=202722` = `0x317A2` → **axis 3, ~6.1 ms
deficit** (larger than the prior ~3.6 ms). MCU watchdog-reset (`iwdg_resets=1`). This refutes the
H10 framing (plan compute cost) as the proximate cause and confirms the pre-implementation caveat:
the link "plan stall → late delivery" was Deduced, never measured, and was wrong as the *primary*
mechanism.

### Confirmed — the incremental planning fix works, and it does not matter

- **Incremental commit is functioning.** `pipe_plan` now carries `line_lo`/`line_hi`/`batch`; the
  spans **advance** (`L113→121→125→127→133`) in small ~8-line increments, not the old overlapping
  60-line re-plans. `commit_decision` shows `barrier`/`commit_count`/eviction live. Buffer shrinks.
- **Plan compute is not the bottleneck.** `pipe_plan` still spikes to 142 ms (avg 29 ms) — *per ~8
  moves*, and the planner is **idle ~90% of the time** (bursts of 2 batches then ~930 ms gaps). The
  217 ms→142 ms change is noise; planning was never saturating a core.

### Confirmed — the actual crash mechanism: a ~1 s frontier freeze starves both MCUs

- **Cascade across all axes / both MCUs** from the same frozen `host_front_start_time`
  (transit_diag_alert): `02:10:38.880 axis2 mcu0 −0.27 ms → 38.884 axis0 mcu1 −7.7 ms (EtherCAT) →
  38.884 axis3 mcu0 −5.2 ms → 38.954 axis1 −67.6 → 38.960 axis2 −73.1 → 39.025 axis3 −139.3 ms` →
  fault + reset. Lateness **grows** — a starvation cascade, not a one-off slip.
- **Frontier frozen ~1.07 s.** `dispatch_committed` t_end jumps 13.13 (wall 37.752) → 14.72 (wall
  38.825) with **nothing in between**. The committed/dispatched trajectory frontier did not advance
  for 1.07 s; that exceeds the ~1 s pump lead → MCU starves.
- **Two interlocking causes of the freeze:**
  1. **New barrier-commit regression — `commit_count=0` on a small buffer.** Batch 33 (wall
     38.767): `n=4, barrier=3, commit_count=0` — 4 moves in buffer, *none* committed (no clean seam
     at-or-before the barrier). The barrier logic held the whole small buffer uncommitted, freezing
     the frontier. New failure mode introduced by the incremental change (previously it committed at
     clean seams under `keep_secs`).
  2. **Backpressure gate had the feeder throttled across the freeze.** `feed_throttle` oscillates
     2.0↔1.0 with ~1 s period; the fatal cycle `enter 37.695 → exit 38.758` paused the feeder for
     the entire freeze, starving the planner of the moves that would advance the barrier and unstick
     the commit.

### Confirmed — the lying signal (exact composition, was the last Deduced link)

`feed_throttle_enter` carries the breakdown: **`buffer_time = pending_end − est`** (verified
`50.616 − 48.308 = 2.308`). `pending_end` = host `_mcu_pending_end_time`, advanced **on move
submission**, not on commit/dispatch. Three frontiers at the freeze (wall 38.759, stream-time;
print↔stream offset ≈ 35.896):

| Level | Frontier | stream-t | in gate math |
|---|---|---|---|
| A. **Submitted** to planner (`pending_end`) | 14.72 | top of subtraction (**wrong**) |
| B. **Dispatched to pump** (`dispatch_committed` t_end, frozen) | 13.13 | should be the top |
| C. **Executed by MCU** (`est`) | 12.41 | subtracted (correct) |

- Gate computed `A − C = 14.72 − 12.41 = ` **2.31 s** → throttle, keep feeder paused.
- Reality `B − C = 13.13 − 12.41 = ` **0.72 s** delivered lead → starving.
- The 1.59 s gap is the planner's submitted-but-uncommitted backlog. The over-read **grew** as the
  freeze accumulated uncommitted moves (0.27 s @ 37.695 → 1.59 s @ 38.759): the gate became *more*
  confident exactly as the MCU got *more* starved.
- **`pump_backlog=886` is not a confounder** — it equals the 0.72 s delivered lead counted in
  finely-discretized pieces (~0.8 ms each) and plateaus during the freeze; the pump is faithfully
  holding the thin lead, not stalling.

### Hypotheses (this follow-up)

| # | Hypothesis | Status | Confirm/Refute |
|---|---|---|---|
| 12 | Incremental planning works but is irrelevant to the crash (plan compute never the bottleneck) | **Confirmed** | line ranges advance; planner idle ~90%; crash persists, deficit larger |
| 13 | Crash is a ~1 s committed-frontier freeze starving both MCUs (cascade) | **Confirmed** | dispatch_committed gap 13.13→14.72 over 1.07 s; transit_diag cascade −0.27→−139 ms |
| 14 | Barrier-commit `commit_count=0` on a small no-clean-seam buffer freezes the frontier | **Confirmed** | batch 33 `n=4 barrier=3 commit_count=0`; new regression vs `keep_secs` |
| 15 | Gate paces on submitted frontier (A) not dispatched-to-pump (B); over-reads delivered lead | **Confirmed** | `buffer_time = pending_end − est`; A−C=2.31 vs B−C=0.72; over-read = uncommitted backlog |

H10 (plan-compute starves pump) **Refuted** as proximate cause. H9 (delivery latency) remains the
symptom; H15 is its proximate root cause. H11 (host MCU motion stream) still open but secondary —
the freeze starves it via the same shared frozen frontier.

### deferred-work.md relevance

H15 is exactly the deferred **"submission-aware backpressure / `queued_motion_secs` does not reflect
real delivery"** item (deferred-work.md 2026-06-22, submission-aware-backpressure review) — now
Confirmed as load-bearing for the crash, not a theoretical hazard. The incremental-planning change
*widened* the A−B gap (bigger uncommitted hold-back, `commit_count=0` extreme), making the
pre-existing signal inaccuracy fatal.

### Fix direction (this follow-up) — a new change, separate from the incremental-planning work

Two faces, one root (delivery-accurate pacing):
- **Gate paces on the dispatched-to-pump frontier (B), not the submitted `pending_end` (A).** Use
  the engine's committed/dispatched `last_move_time` for lead. Then during a freeze the gate sees
  the real 0.72 s draining, does **not** throttle, and keeps feeding — which also advances the
  barrier and unsticks the commit. (`klippy/motion.py` `_check_pause` /
  `rust/motion-engine/src/bridge.rs` `queued_motion_secs`.)
- **Barrier-commit must not freeze the frontier when delivered lead is thin.** On a would-be
  `commit_count=0` with draining lead, make progress (best available seam / force-advance) rather
  than holding a small buffer wholly uncommitted. (`rust/motion-engine/src/stream.rs` commit
  selection.) This is the specific regression the incremental change introduced.
- Do **not** re-raise `lead_secs`/`MAX_LEAD_SECS`/ring (band-aids, masks the signal bug).

**Status: Active** — H12–H15 Confirmed; proximate root cause = backpressure gate pacing on the
submitted frontier (A) instead of the dispatched-to-pump frontier (B), with the barrier-commit
`commit_count=0` freeze as the trigger. Incremental planning works as specified but is orthogonal to
the crash.
