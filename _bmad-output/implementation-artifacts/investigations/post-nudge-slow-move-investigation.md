# Investigation: Slow/weird motion after z_tilt_adjust per-motor nudges

## Hand-off Brief

1. **What happened.** During `z_tilt_adjust`, the host gcode-feed throttle stalls for ~20 s between probe points because it now gates on the engine's `queued_motion_secs()` signal, which desyncs to a phantom ~26 s "backlog" after the homing/nudge moves — confirmed in bench logs (session `k-1782577178-22340`, 16:24:12→34 on 2026-06-27).
2. **Where the case stands.** **Root cause Confirmed (High).** Regression = commit `12a75796a` (+ siblings) repointing `_check_pause` from the drain-grounded `_mcu_pending_end_time - est` to the ungrounded `engine.queued_motion_secs()`. Bench `backpressure_view` shows `buffer_time`/`dispatched_lead` stuck at ~26 s while `channel_pending=0` and the lead drains only 0.5 s over 22 s (a fixed phantom offset, not a real queue).
3. **What's needed next.** Fix: re-ground the engine queued-motion signal to the host clock after a drain/nudge (or stop gating on it for non-streaming ops). See Fix direction.

## Case Info

| Field            | Value                                                                          |
| ---------------- | ------------------------------------------------------------------------------ |
| Ticket           | N/A                                                                            |
| Date opened      | 2026-06-27                                                                     |
| Status           | Concluded — root cause Confirmed (High)                                        |
| System           | kalico fork, branch `post-nudge-slow-move`; Trident bench (H723 + F446)        |
| Evidence sources | git history, rust/motion-engine, klippy/motion.py, structured logs (pending)   |

## Problem Statement

User: running `z_tilt_adjust` on the Trident bench. After it adjusts individual motors, motion
becomes "super weird and slow." Suspected related to how nudge blocks motion and how velocity is
computed afterward. Bisection: `pr/nonblocking-flush-drain` = last good; PRs 116/117 crashed during
Z-tilt (couldn't test); #122 tested bad. Not every commit tested.

## Evidence Inventory

| Source                       | Status     | Notes                                                                                 |
| ---------------------------- | ---------- | ------------------------------------------------------------------------------------- |
| git diff good..HEAD          | Available  | motion.py backpressure rewrite; bridge.rs anchor code unchanged between branches       |
| klippy/motion.py             | Available  | `_check_pause` throttle source changed; `_ground_..._drain`; `motion_drain_finalize`   |
| rust nudge/anchor/planner    | Available  | `run_nudge`, `anchor_segment`, `queued_motion_secs` traced                              |
| structured logs (bench run)  | **Pending**| `backpressure_view` event added by the regression commits — records the stuck term     |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Pull `motion/backpressure_view` + `feed_throttle_enter/exit` from a z_tilt run | High | Open | Decisive: shows buffer_time vs channel_pending magnitude during the slowdown |
| 2 | Confirm whether `queued_motion_secs` stays large vs `channel_pending` stuck | High | Open | Two candidate stuck-terms; logs disambiguate |
| 3 | Check `917b8b29f` "ground the queued-motion signal to host clock" — does it cover the nudge case? | Medium | Open | Authors knew about grounding; nudge path may be the uncovered case |

## Timeline of Events

| Time | Event | Source | Confidence |
| ---- | ----- | ------ | ---------- |
| good branch | `_check_pause` gates on `_mcu_pending_end_time - est` (grounded after every drain) | `pr/nonblocking-flush-drain:klippy/motion.py:660` | Confirmed |
| `cb75014f0` | `queued_motion_secs` added to engine (submission-aware signal) | git log -S | Confirmed |
| `12a75796a` et al. | `_check_pause` repointed to `engine.queued_motion_secs()` + `pending_channel_moves()` | git log -S; motion.py diff | Confirmed |
| run-time | z_tilt → force_move.manual_move → submit_nudge → run_nudge advances `last_move_time` | force_move.py:55; stream_planner.rs:551 | Confirmed |

## Confirmed Findings

### Finding 1: z_tilt per-motor adjust is the nudge path
`klippy/extras/z_tilt.py:55` calls `force_move.manual_move(...)`; `klippy/extras/force_move.py:55`
calls `toolhead.submit_nudge(...)`. So every per-motor z_tilt correction is a nudge.

### Finding 2: The throttle's input signal was changed (the regression)
Good branch `_check_pause` (`pr/nonblocking-flush-drain:klippy/motion.py:660,683`):
`buffer_time = self._mcu_pending_end_time - est`. HEAD `_check_pause`
(`klippy/motion.py:662`): `buffer_time = self.engine.queued_motion_secs()` plus a new
`channel_pending = self.engine.pending_channel_moves()` gate. Introduced by `12a75796a`
and siblings (`917b8b29f`, `ab538756d`, `9fbb8b803`), all inside `pr/nonblocking-flush-drain..HEAD`.

### Finding 3: The old signal self-heals after a drain; the new one is not grounded
`_ground_pending_end_time_after_engine_drain` (`klippy/motion.py:636`) clamps
`_mcu_pending_end_time` to `est + motion_lead` after every drain — so the OLD `buffer_time`
collapses to ~`motion_lead` after z_tilt's flushes (no throttle). The NEW signal
`queued_motion_secs()` is computed entirely in Rust from `dispatch_anchor.t0() + last_move_time -
host_now + uncommitted` (`rust/motion-engine/src/bridge.rs:3921-3943`) and is **not** grounded by
the Python drain — `motion_drain_finalize` is now a no-op `{}` (`bridge.rs:3664`).

### Finding 4: Nudges advance the engine's `last_move_time`
`run_nudge` (`rust/motion-engine/src/stream_planner.rs:515-553`): `state.commit(true)` drains,
nudge profile is dispatched, then `state.advance_time(total_dur)` and
`last_move_time_bits.store(state.t_committed())` (lines 551-552). `last_move_time` is monotonic and
only ever advanced — it is the exact quantity the new throttle reads.

### Finding 5: Nudges share the MAIN dispatch_anchor (correction to an early assumption)
`rust/motion-engine/src/bridge.rs:3170,3187`: `nudge_anchor_arc = Arc::clone(&anchor_mutex)` where
`anchor_mutex = Arc::clone(&self.dispatch_anchor)`. The nudge dispatch updates the same anchor's
`last_t_end`. (Earlier hypothesis of a separate nudge anchor — **refuted**.)

### Finding 6: Bench logs show the throttle stuck on a phantom ~26 s backlog during z_tilt (decisive)
Session `k-1782577178-22340`, 2026-06-27. The 16:24 window is a z_tilt probing sequence
(`subsystem=homing`): X homed 16:24:06, Y 16:24:07, **Z probe 16:24:12.747**, **next Z probe
16:24:33.819** (21 s later), Z 16:24:34.109. `motion/backpressure_view` across that gap:

| _time | buffer_time | dispatched_lead | uncommitted | channel_pending | throttling |
| ----- | ----------- | --------------- | ----------- | --------------- | ---------- |
| 16:24:07.624 | 2.72 | 2.67 | 0.05 | 0 | true |
| 16:24:12.748 | **28.34** | **26.73** | 1.61 | 0 | true |
| 16:24:34.109 | **26.19** | **26.19** | 0.0 | **1** | true |
| 16:24:39.716 | 2.71 | 1.14 | 1.57 | 0 | (recovered) |

`dispatched_lead = t0 + last_move_time - host_now` (`bridge.rs:3946-3967`) held ~26 s for 22 s of
wall-clock, draining only 0.5 s. A real queue sheds ~1 s per wall-second; a flat offset means
`t0`/`last_move_time` are desynced from the host clock by a constant ~26 s. `feed_throttle_enter`
samples corroborate (buffer_time 26–28, channel_pending 0–1).

### Finding 7: The stuck term is the engine frontier, not the channel (Hypothesis 2 refuted)
`channel_pending` stayed 0–1 throughout (well under `channel_low` = 4096). Only the
`buffer_time`/`dispatched_lead` term was high. Confirmed by `feed_throttle_enter` and
`backpressure_view`. So the new `channel_pending` gate is innocent; the `queued_motion_secs` gate is
the culprit.

### Finding 8: Anchor desync marker fires right before the spike
`motion/seg0_deficit` "negative deficit_us => in past" burst at 16:24:02–10, in pairs, with
`deficit_us ≈ 249998` (= `DEFAULT_LEAD_SECS` 0.25 s, `anchor.rs:2`). Emitted by the fresh-(re)anchor
path (`bridge.rs:3318`) — the homing/nudge moves repeatedly land seg0 in the past, i.e. the producer
falls behind the playhead exactly where the frontier then reads as a 26 s phantom.

## Deduced Conclusions

### Deduction 1: The slowdown is the gcode-feed throttle, not the velocity solver
**Based on:** Findings 1-4. **Reasoning:** "weird and slow" matches the host pausing 10 ms at a
time in `_check_pause`'s drain loop (`klippy/motion.py:690-708`) — it feeds moves at host-pause
cadence rather than computing slow trajectories. The bisection lands exactly on the commits that
changed *what the throttle reads*, not on any velocity/solver change. **Conclusion:** the planner is
still producing fast trajectories; the host is metering them out slowly because the new backpressure
signal reads "buffer full" when it isn't.

## Hypothesized Paths

### Hypothesis 1: `queued_motion_secs` stays large after the nudge sequence
**Status:** **Confirmed** (Finding 6/7/8). Bench logs show `dispatched_lead` stuck at ~26 s (flat,
non-draining, `channel_pending=0`) for 21 s between z_tilt probe points while `throttling=true`.
**Resolution:** confirmed by session `k-1782577178-22340` 16:24:12→34.
**Theory:** After z_tilt, `last_move_time` holds the nudge-advanced frontier and the anchor `t0`
is not re-grounded for the resumed stream, so `t0 + last_move_time - host_now` (+ `uncommitted`)
reads as a multi-second backlog, keeping `_check_pause` in its drain loop.
**Supporting indicators:** signal no longer grounded by the Python drain (Finding 3); nudge advances
`last_move_time` (Finding 4); old grounded signal worked (Finding 2/3).
**Would confirm:** `backpressure_view` events after z_tilt showing `buffer_time` >
`buffer_time_high` (default 2.0) while the machine is actually idle/near-idle.
**Would refute:** `buffer_time` near 0 in those events (then the stuck term is `channel_pending`).
**Caveat:** `anchor_segment` re-anchors on underrun (`anchor.rs:41`) when `t0 + seg_t_start <
host_now`, which *should* fix `t0` on the first resumed move. The arithmetic for a *sustained* high
reading is not yet proven on paper — hence logs are decisive. The subagent's worked numeric example
was internally inconsistent and is not relied upon.

### Hypothesis 2: `channel_pending` is the stuck term
**Status:** **Refuted** (Finding 7) — `channel_pending` stayed 0–1 throughout the stall.
**Theory:** `pending_channel_moves()` (= `sender.len()`) stays above `channel_low` (cap/2 = 4096)
keeping the throttle engaged.
**Supporting indicators:** new `or channel_pending > self._channel_low` clause (`motion.py:691-693`).
**Would refute (likely):** z_tilt submits few moves, so the channel is nearly empty — this term is
expected to be ~0. Logs confirm.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| `backpressure_view`/`feed_throttle_*` events from a real z_tilt run | Confirms which term is stuck + magnitude → moves Hypothesis 1 to Confirmed | query-logs over VictoriaLogs, filter subsystem=motion event=backpressure_view around a z_tilt_adjust |
| Whether re-anchor fires on the first resumed move | Settles the "sustained vs transient" question on paper | add/inspect `anchor-decision` / `anchor_underrun` log lines (already emitted, anchor.rs:43,63) |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Error origin | `klippy/motion.py:662` (`buffer_time = self.engine.queued_motion_secs()`) + drain loop 690-708 |
| Trigger | `z_tilt_adjust` → `force_move.manual_move` → `submit_nudge` → `run_nudge` advances `last_move_time` |
| Condition | New throttle reads ungrounded `queued_motion_secs()`; old `_mcu_pending_end_time` was grounded after every drain |
| Related files | `rust/motion-engine/src/bridge.rs:3921-3943,3664`; `stream_planner.rs:515-553`; `anchor.rs`; `klippy/extras/force_move.py`, `z_tilt.py` |

## Conclusion

**Confidence:** High.

The slowdown is a host gcode-feed throttle regression. Commit `12a75796a` (+ `917b8b29f`,
`ab538756d`, `9fbb8b803`) repointed `_check_pause` from the drain-grounded `_mcu_pending_end_time -
est` to the engine's ungrounded `queued_motion_secs()` (= `dispatch_anchor.t0 + last_move_time -
host_now + uncommitted`). During z_tilt's homing/probe/nudge sequence the anchor/`last_move_time`
desync from the host clock (seg0 lands "in past" repeatedly), and the signal reports a phantom ~26 s
backlog. Because the throttle now trusts that signal, it stalls the gcode feed for ~20 s between
probe points — the toolhead crawls. Bench logs (session `k-1782577178-22340`, 16:24:12→34) show
`dispatched_lead` flat at ~26 s, draining 0.5 s in 22 s, `channel_pending=0`, `throttling=true`. The
old signal never had this problem because `_ground_pending_end_time_after_engine_drain` re-grounds it
to `est + motion_lead` after every drain; the Rust signal is never grounded (`motion_drain_finalize`
is a no-op).

## Recommended Next Steps

### Diagnostic (decisive)
Pull from the Trident bench around a z_tilt_adjust:
`subsystem=motion` `event=backpressure_view` (buffer_time, channel_pending, dispatched_lead,
uncommitted_intake, throttling) and `feed_throttle_enter`/`feed_throttle_exit`
(engine_frontier, waited_s). Also `anchor_underrun` / `[anchor-decision]` to see if/when t0
re-anchors. This disambiguates Hypothesis 1 vs 2 and proves sustained-vs-transient.

### Fix direction (after confirmation)
If `queued_motion_secs` is the stuck term: ground the engine signal after a drain/nudge the same way
`_mcu_pending_end_time` is grounded — e.g. reconcile `last_move_time`/anchor `t0` to the host clock
on `motion_drain_finalize` (currently a no-op), or have `run_nudge` reset the frontier to the host
playhead after dispatching. Note `917b8b29f` already attempted host-clock grounding — check why it
misses the nudge case.

## Reproduction Plan

On the Trident bench (test-only): home, run `Z_TILT_ADJUST`, then issue any normal move and observe
the slow cadence. Capture structured logs across the window. Compare against
`pr/nonblocking-flush-drain` (expected: normal speed).

## Build Spec (confirmed via code read + panel pre-mortem)

**Decision:** ship the tight grounding patch + a loud tripwire now; schedule the typed-boundary
refactor as a follow-up gated on one test. Floating-ahead frontier is by-design (anchor.rs:24-30) — do
NOT collapse the lookahead.

**Scope confirmations:**
- `motion_drain_finalize(&self)` (bridge.rs:3664) reaches `dispatch_anchor` (:668), `router.host_now_secs()`
  (:655), `planner.last_move_time()` (:661) via interior mutability — no signature change.
- `Anchor` (anchor.rs) has no `t0` setter — add a grounding method.
- Hot-path self-grounding is underrun-only (bridge.rs:3206-3209, anchor.rs:41) — a phantom (frontier
  AHEAD) never underruns, so the class survives off the `wait_moves` path.
- Exposed nudge callers with no immediate `wait_moves`: `FORCE_MOVE` gcode (force_move.py:74-78),
  `angle.py:183/669`, `motors_sync.py:387`. (`z_tilt_ng.py:79` DOES wait_moves → reported bug fixed.)

**Change set:**
1. `anchor.rs`: `pub fn reground(&mut self, host_now: f64, frontier_u: f64)` → `t0 = Some(host_now -
   frontier_u); last_t_end = frontier_u;`.
2. `bridge.rs:3664`: implement `motion_drain_finalize` → read `last_move_time` + `host_now`, call
   `dispatch_anchor.reground(host_now, last_move_time)`. Post-drain precondition (channel empty) holds
   by construction (motion.py:558-568), so `queued_motion_secs()` → ~0 after.
3. Tripwire (fail-loudly doctrine): in finalize, capture pre-grounding `queued_motion_secs`; if it
   exceeded a tolerance (e.g. > buffer_time_high) while the channel was empty, emit a loud structured
   `motion`/`anchor_phantom_grounded` warn event with the gap magnitude. (Warn, not shutdown — a stale
   throttle signal must not brick a print; escalation left to the user.)
4. Test (red→green): assert that after a nudge advances `last_move_time` + drain + finalize,
   `queued_motion_secs() ≈ 0`. Red today (~26 s), green after the snap. Add a guard test that finalize
   is a no-op when there is no phantom (idempotent at true quiescence).

**Follow-up (separate PR, gated on Quinn's test):** a phantom-injecting nudge with NO following
`wait_moves`, then keep streaming — assert `queued_motion_secs` re-converges within one buffer refill.
If it stalls (predicted, given the exposed callers above), introduce typed `HostSeconds`/`EngineSeconds`
so the grounding conversion is explicit and un-skippable, or ground after every nudge dispatch.

## Side Findings

- `motion_drain_finalize` is now an empty no-op (`bridge.rs:3664`) while `_wait_mcu_drained` still
  calls it (`motion.py:568`) — dead seam; the grounding that would naturally live here is absent.
- The regression commits' messages ("ground the queued-motion signal to the host clock", "gate
  submission on the host frontier, not the engine signal") show the authors iterated on exactly this
  grounding problem — the nudge interaction looks like an uncovered corner of that work.
