# Handoff: beacon emulator's contact model doesn't track Z — RESOLVED (emulator side)

**Date:** 2026-07-06 · **Branch:** beacon-emulator · **Status:** emulator contact
model fixed; motion-engine follow-ups fixed too (poke and auto-calibrate
green, unmarked); only bed_mesh (beacon stream stall) remains

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

RESOLVED 2026-07-06 (follow-up session), except bed_mesh:

- `test_poke` — FIXED, unmarked, green. Root cause was in the shaper
  (`motion-pipeline/src/shaper.rs`): `at_stream_boundary` was derived as
  `history.is_empty()`, so only the *first* emit batch of a stream clamped
  its convolution window before the stream start; a second batch whose back
  window still reached before the start (the lowerer's front hold pad covers
  *forward* support, which for a symmetric kernel puts the back need exactly
  on the stream start, 1 ulp of rounding from failure) hit the
  `MissingHistory` panic. Now the shaper tracks `history_trimmed` — the
  clamp stays valid until real history has been dropped. Regression test:
  `smooth_shaper_second_batch_window_before_stream_start_clamps`.
- `test_contact_auto_calibrate` — FIXED, unmarked, green. Three parts:
  1. Engine: `HistoryStore` now keeps a `HoldBeforeRing` per axis — the rest
     between the surviving endpoint and the first piece recorded into an
     emptied ring answers with the held position (pieces are the only way an
     axis moves). This unblocked the contact nozzle-touch phase entirely.
  2. Fork: one stale sample (streamed pre-reanchor, flushed post-reanchor)
     is legitimately unanswerable in the new frame; the seam drops it (no
     `pos`) but `_calibrate` KeyError'd. Fixed via dderg/beacon_klipper PR #5
     (merged, `ac20aee`); pin bumped in `tools/sim/fetch_plugins.sh` and the
     sim Dockerfile symlink updated for master's beacon_kalico.py →
     beacon_motion_engine.py rename.
  3. Flake: the fork's nozzle-touch repeatability gate (`autocal_tolerance`,
     default 0.008) is marginal against the emulator's ~0.01mm trigger
     quantization ("Sample spread too large (0.0082 > 0.0080)"); the sim
     config sets `autocal_tolerance: 0.02`.
- `test_bed_mesh` (still skip): NOT the same family. Mid-scan the emulator's
  beacon stream stalls → "Beacon sensor not receiving data" shutdown → the
  fork's mesh scan callback KeyErrors on the pos-less samples flushed after
  shutdown, killing the reactor. Beacon stream throughput/stall under the
  virtual clock; needs its own session.
