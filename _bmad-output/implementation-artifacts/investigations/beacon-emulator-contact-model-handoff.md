# Handoff: beacon emulator's contact model doesn't track Z — RESOLVED (emulator side)

**Date:** 2026-07-06 · **Branch:** beacon-emulator · **Status:** emulator contact
model fixed; remaining failures re-diagnosed as motion-engine issues

## Outcome

`test_proximity_probing` and `test_contact_probing` are green and unmarked.
The emulator now genuinely step-tracks Z and fires the contact trigger at the
step-tracked bed crossing. The still-failing scenarios turned out NOT to be
emulator gaps — see "Remaining failures" below.

## What was actually wrong (three separate bugs)

1. **Step direction was never exported.** The Rust step-queue producer writes
   `StepEntry.dir` as a signed ±1 (`dispatch_stepper.rs`,
   `let dir: i8 = if signed_steps > 0 { 1 } else { -1 }`), but the sim drain in
   `src/linux/runtime_tick_host.c` treated it as a 0/1 flag:
   `sim_notify_step(0, line, dir ? -1 : 1)` — always −1. Every Z step counted
   as *downward*, so the emulator's tracked Z only ever descended. Homing
   passed by luck (descent is downward); the post-home retract dragged the
   tracked Z to −3mm, which the model maps below `model_range` min → the
   handoff's "below calibrated model range" symptom. Fix: pass `dir` through
   as the signed step delta.

2. **Contact trigger didn't latch the trsync.** `_fire_homing_trigger`
   (proximity) marks `_trsync_can_trigger[oid]=False` and stores the reason;
   `_fire_contact_trigger` did not. The fork's `trip_move_end` cleanup then
   sent `trsync_trigger reason=2` (REASON_HOST_REQUEST), the stub accepted it
   as a fresh trigger, and the fork's `last_reason` flipped 1→2 → the
   "descend completed with trsync reason 2" symptom (2 is HOST_REQUEST in the
   fork's numbering, not comms-timeout as the original handoff guessed). Real
   trsync latches the first trigger reason. Fixed in `_fire_contact_trigger`.

3. **The 0.5s fallback timer raced the real trigger.** With step export
   working, the z≤0 crossing in `_step_poll_loop` is the real contact model;
   the timer is now armed only when the stub has no step socket at all.

The old TODO ("export kalico-runtime steps … so a line can ever lock") was
stale — locking worked; the *sign* of the exported steps was the bug.

Original per-scenario symptoms, repro commands, and the emulator architecture
notes from the previous revision of this handoff are in git history
(63da2065b).

## Anchor semantics (now understood, no change needed)

The stub anchors `_z_anchor_mm = 10.0` at the first observed step. That frame
can be ~90mm off klippy's frame (homing starts at Z=105), but it doesn't
matter: the proximity trigger is defined by the emulator's *own* threshold
model crossing, and klippy defines the post-home frame from that same trigger
— both frames re-align at the trigger to within step/poll latency (~0.01mm,
verified: klippy set Z=1.9499 vs emulator z=1.956 at rest). Contact re-anchors
to 0 at bed contact via `_anchor_z(0.0)`.

## Remaining failures — motion-engine, not emulator

- `test_contact_auto_calibrate` (xfail): calibration streams samples during a
  0.25s dwell before the descend; those samples query motion state at host
  times where the axes are idle, and the engine retains no history for idle
  axes (observed X-axis window: 18.169644..18.179207s — 10ms). The samples
  come back without `pos` and the fork's `_calibrate` KeyErrors on
  `s["pos"]`. On real hardware every sample is answerable from klippy stepper
  history. Same family as `sim-trip-time-resolution-handoff.md`.
- `test_poke` (skip): hangs because the *first plain travel move* panics the
  `kalico-shape` thread — "shaper: axis 0: shaping window needs unavailable
  history at t=0.9999999999999999" (`motion-pipeline/src/shaper.rs:130`) —
  the move never completes, BEACON_POKE never responds. Reproduce with
  `tools/sim/run.sh test -k test_poke` after dropping the skip; the panic is
  in klippy.stdout.
- `test_bed_mesh` (skip): unhandled reactor exception, untriaged; likely the
  same shaper/history family. Needs its own session.
