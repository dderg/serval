# Handoff: fix the stream-planner `OverCommitted` mid-print abort

**Date:** 2026-07-01 · **Branch:** clock-sync · **Status:** root cause CONFIRMED, fix designed, not started
**Companion:** full forensics in `mcu-foreground-stall-crash.md` (same dir, Follow-up #4)

## TL;DR

Mid-print the host motion-engine aborts with
`commit: velocity plan: OverCommitted { line_no: N }` → `std::process::abort()` →
klippy shuts down → MCU gets `emergency_stop` ("Command request") → print dies. It is
**intermittent** (timing-dependent commit-seam positions), **host-side only**, and has
nothing to do with the MCU or EtherCAT (those were red herrings — the MCU snapshot on
reconnect is a *replayed* stale forensic record). The user has decided the fix:
**change the fitter↔planner contract so committed geometry is never re-fit** ("once the
fitter commits, there is no going back").

## Root cause (Confirmed)

The streaming planner commits look-ahead windows incrementally. A commit can cut at a
"clean seam" (zero curvature) that falls **inside a raw move** (blend exit, or a
collinear split where one raw move emitted several pieces). The committed head is trimmed
(`trim_front_to_seam`) and the **remainder is re-fit next window**, with
`committed_head_len` fed to `fit_chain_with_head_restore` to try to reconstruct the
pre-trim corner. That reconstruction is not seam-invariant: the re-fitted corner comes
out sharper, so its velocity cap drops **below the entry velocity already committed** →
fail-loud abort (correct guard; the bug is the drift).

**Coredump proof** (`core.kalico-stream-p.7655.1782892320`, gdb on `run_loop` frame's
`StreamState`):
- `entry_v = 28.141 mm/s` (committed at the seam)
- `last_v_barrier = 21.185 mm/s` (re-fitted corner ceiling) → **~33% over-commit, not epsilon**
- `committed_head_len = 0.400 mm` (the trimmed "short front")
- config in-core: scv=8, fit_tol=0.005, jerk=100k, arc_fit=None, max_v=100, accel=1000

Recurring across prints/lines (deterministic geometry, non-deterministic seam): line
15842 (07-01 07:52), 26933 (06-30 22:24), and a sibling `RestAnchorAccel` at 18499.

## Key code

- Abort: `rust/motion-engine/src/stream_planner.rs:386` (`fatal`), called from `run_loop`.
- Commit + seam choice: `rust/motion-engine/src/stream.rs:324` (`commit`), seam loop
  `403-421` (`is_clean_seam && head_trim_feasible`), trim/re-fit path `480-511`.
- The leaky reconstruction: `trim_front_to_seam` (`stream.rs:637`), `committed_head_len`
  (field `162`, set `506`), `fit_chain_with_head_restore` (`stream.rs:336`).
- Corner budget with restore: `rust/geometry/src/fitter.rs:216`
  `budget = 0.5*(line_in.s_len() + head_len_restore).min(line_out.s_len())`.
- Restore routed to the **first junction only**: `rust/geometry/src/fitter/causal.rs:40`.
- The two `OverCommitted` guards: `rust/geometry/src/velocity.rs:228` (entry curvature
  ceiling) and `:329` (brake-to-corner reachability).
- Comment that already names the failure: `stream.rs:497-505`.

## The decision + design options

Contract: **committed geometry is frozen; never re-fit it.** Three ways to honor it:

- **A. Commit only at raw-move boundaries (never trim).** Structurally kills the bug;
  delete the trim + head_restore machinery. Risk: blends push clean seams mid-move, so
  commits may coarsen → **must measure throughput** (CLAUDE.md: throughput non-negotiable).
- **B. Freeze + splice committed fitted geometry forward.** Literal "no going back" but
  does *not* alone fix the velocity mismatch (entry_v was committed against the full-length
  corner). Needs C.
- **C. (recommended) Commit the seam velocity conservatively.** Today `head_restore`
  inflates the budget back to full length so the re-fit tries to reproduce the optimistic
  corner. Invert it: commit `entry_v` using the **post-trim (remainder-length) corner cap**,
  so any future re-fit is guaranteed ≥ what was promised. Keeps fine-grained commits (no
  throughput cost), and lets you delete the head_restore reconstruction.

**Before implementing:** confirm with the repro whether the drift is the corner *budget*
(fit side → C is right) or the *finality barrier* being computed past a not-yet-final
corner (→ fix moves into the barrier calc, `stream.rs:400-421` + `plan_velocity_warm_start`).

## Offline reproduction harness (built, ready — not yet run to completion)

`rust/motion-engine/tests/seam_voron_repro.rs` (`#[ignore]`d). Drives the real stream
planner over the full Voron cube via `seam_test_harness::run_moves`, sweeping `FixedCap`
1..40 (small cap = early commit = short front = the trigger). Config is faithful to the
bench (matches the coredump byte-for-byte): scv=8, fit_tol=0.005, rest are harness
defaults which already equal the host (`bridge.rs:3511`).

Run it:
```
VORON_GCODE=/path/to/voron_cube.gcode \
  cargo test -p motion-engine --release --test seam_voron_repro -- --ignored --nocapture
```
Gcode: `~/printer_data/gcodes/Voron_Design_Cube_v7_0.2mm_PLA_Elegoo Neptune 3 Pro_57m43s.gcode`
on the bench (dderg@ethercatpi5.local); the offending corners are ordinary 90° perimeter
turns. Expected: at least one small cap aborts with `OverCommitted` (may be a different
line than live — seam position varies, the *defect* is what reproduces). Batch/large-cap
runs will pass (long fronts) — that's the "sometimes completes a whole print" case.
NOTE: the in-session run never finished its release compile; run it fresh.

To pin budget-vs-barrier: add a one-shot log at `velocity.rs:228/329` printing
`entry_v`, `entry_ceiling`/`entry_brake`, and `caps[0].kin.{kappa0,length}` when it's
about to return `OverCommitted`, then re-run the sweep.

## Validation gate for the fix

1. Repro test: no cap aborts after the fix.
2. Throughput A/B via klipper-sim (`~/Developer/klipper-sim`, `--klipper-root` the
   worktree, blendarc mode, same limits): committed trajectory time + commit cadence must
   not regress vs current.
3. `./scripts/ci.sh quick` green; existing `seam_continuity` / `seam_schedule_fuzz` tests
   still pass.

## Instrumentation status (kept)

- **`tests/seam_voron_repro.rs`** — KEEP (uncommitted). It is the reproduction + fix-
  validation tool for the next session. Not in CI (`#[ignore]`).
- **MCU foreground-stall instrumentation** (committed 2d0518677/a3719b05d: `fg_task`,
  `fg_msg`, `fg_demux`, `fg_msg_head`, LIVE_MAGIC robustness in `fault_handler.c` /
  `mcu_demux.c` / `event_log.h` / `log_codes.rs`) — KEEP. It correctly *exonerated* the
  MCU here and is low-cost general foreground-stall diagnostics worth retaining. Prune
  later only if it proves noisy. Do not amend/revert history to remove it.

## What's confirmed vs open

- Confirmed: root cause, the abort chain, the coredump numbers, the recurring pattern,
  that MCU/EtherCAT are downstream/red-herrings, and the design direction.
- Open: exact leak point (budget vs barrier) — pin with the repro before coding; then
  implement C (or A) and run the validation gate.
