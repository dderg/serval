# Investigation: real-print abort — "head-trim geometry: ZeroMotion"

## Hand-off Brief

1. **What happened.** A real print (Voron cube) aborted the stream planner with
   `commit: head-trim geometry: ZeroMotion` (Confirmed, VL `stream_planner_fatal`
   2026-06-22 06:51:55). The earlier wall-clock-deadline crash is gone; the
   float-ahead/coalesce work holds (testlong now completes, stuttering).
2. **Where the case stands.** Root cause is localized to the continuity-commit
   head-trim (`rust/motion-engine/src/stream.rs::trim_front_to_seam`): a chosen
   commit boundary required head-trimming a kept move that was either non-`Line`
   or left a degenerate (zero-length) remainder. A **selection-time guard**
   (`head_trim_feasible`) now refuses such boundaries and logs `head_trim_refused`
   with the geometry; the abort path is unreachable from selection. The exact
   triggering geometry was **not** reproduced locally (Medium confidence on the
   precise mechanism, High on the fix preventing the abort).
3. **What's needed next.** One bench run to capture a `head_trim_refused` warn
   (keep_line + spatial type + remainder) — that pins the exact geometry without
   crashing. Separately, symptom #2 (Mainsail "actual" position leading
   "requested") is still open and host-side.

## Case Info

| Field            | Value                                                          |
| ---------------- | ------------------------------------------------------------- |
| Ticket           | N/A                                                           |
| Date opened      | 2026-06-22                                                    |
| Status           | Active (fix landed; exact trigger geometry unconfirmed)      |
| System           | Neptune 3 Pro bench (ethercatpi5), curvature-profile branch  |
| Evidence sources | VictoriaLogs (host-rust motion), source code, real gcode     |

## Problem Statement

After the coalesce + float-ahead fixes (commit e3b40a62f): testlong.gcode prints
to completion (with visible stutter — start delay, move, pause, move). A real
print gcode "crashed right away." Mainsail still shows actual position updating
ahead of requested.

## Confirmed Findings

### Finding 1: the abort is the head-trim, not the old starvation path

**Evidence:** VL `event=stream_planner_fatal`, `error="commit: head-trim
geometry: ZeroMotion"`, session `k-1782110756-14053`, 06:51:55.813.

**Detail:** `StreamError::Geometry(GeometryError::ZeroMotion)` originates only in
`stream.rs::trim_front_to_seam` — either the front buffer move is not a
`Segment::Line` (else-branch) or `Line::try_new(seam, end)` got a zero-length
remainder. `run_loop`'s `commit(false).unwrap_or_else(|e| fatal(...))` turns it
into `process::abort`.

### Finding 2: the float-ahead/coalesce fix is working

**Evidence:** VL `event=anchor_underrun` (06:51:39) re-anchoring forward, and the
old `SegmentLate`/"scheduled in the past" fatal no longer appears. testlong
completes.

**Detail:** The stutter the user observes is the designed stall-not-crash
recovery plus startup ramp-up — the planner still produces trajectory slower
than playback at stream start, but now rides it out instead of aborting.

### Finding 3: the crash decision was a blended kept boundary

**Evidence:** VL `commit_decision n=5 commit_count=4 unblended=1 total_t=0.95
entry_v=0 t_committed=0` immediately preceding the fatal.

**Detail:** A fresh (post-idle) coalesced batch. commit_count=4 of 5 outputs; the
kept boundary (output 4) had `blend_consumed_head` true → `trim_front_to_seam`
ran and hit ZeroMotion.

## Refuted / Eliminated

- **Arcs as the early trigger.** `arc_fit` is commented out in printer.cfg; the
  real gcode has 1 G2/G3 total and 0 G5 — not in the early section. Refuted for
  the "right away" crash.
- **Old wall-clock-deadline abort.** `DispatchError::SegmentLate` removed;
  anchor floats ahead. Not present in the crash session.

## Hypothesized Paths

### Hypothesis 1: degenerate/non-Line head-trim front in a mixed startup batch

**Status:** Open (fix guards it; exact geometry unconfirmed).

**Theory:** The first print batch after probing mixes extrude-only (retract/prime),
travel, Z, and perimeter moves. Some boundary selection chose a cut requiring a
head-trim whose kept move was non-`Line` or whose remainder rounded to exactly
zero.

**Supporting indicators:** crash decision shows a fresh mixed batch; analytically
a single line-line blend always leaves remainder ≥ ~half the kept move, so the
trigger is an unmodeled interaction (mixed move types / source-line mapping).

**Would confirm:** a `head_trim_refused` warn on the next bench run reporting the
`keep_line` + `spatial` discriminant + `remainder`.

**Would refute:** the warn never fires and the crash recurs through a different
path (would mean ZeroMotion has another source).

## Fix Direction (landed)

Selection-time guard in `stream.rs`:
- During lowering, capture each output segment's start position (`seam_xyz`).
- `head_trim_feasible(moves, i, seam_xyz[i])`: a boundary needing
  `blend_consumed_head` is only accepted if the kept buffer move is a `Line` with
  remainder `> TRIM_EPS_MM` (1e-6 mm); otherwise refused + `head_trim_refused`
  warn.
- Selection picks the latest clean **and feasible** seam within `keep_secs`;
  refused boundaries are skipped (the move waits for more context, or the
  buffer-cap/idle force-drain — which clears to rest without head-trim — handles
  it). No freeze, no abort.

Regression tests (motion-engine):
- `stream::tests::voron_cube_perimeter_streams_without_degenerate_trim` (real
  perimeter, incremental commits).
- `stream_planner::tests::nonstop_flood_of_real_perimeter_drains_without_crashing`
  (real planner thread, continuous flood — the requested nonstop-input test;
  exercises coalescing + float-ahead under load).

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | --- | --- |
| Exact triggering move geometry | Confirms Hypothesis 1; lets us decide whether the refused boundary should instead be supported (general non-line trim) | Bench run → capture `event=head_trim_refused` |
| Why Mainsail "actual" leads "requested" (symptom #2) | Separate UX/correctness bug | Trace `gcode_move`/`motion_report` vs `commanded_pos`/`print_time` on host |

## Follow-up: 2026-06-22 — symptom #2 (Mainsail "actual" leads/lags "requested")

### Finding 4: live position came from a 200ms hardware poll, not the trajectory

**Evidence:** `rust/motion-engine/src/bridge.rs:2879` spawns a `live-position-poll`
thread sampling `collect_motor_positions_inner` (physical motor positions) every
200ms into `live_position_cache`; `motion_report.get_status` served that via
`engine.live_motor_positions()` (`klippy/extras/motion_report.py:54`, pre-fix).

**Detail:** "actual" (Mainsail live_position) was a 200ms-quantized hardware poll
on the MCU/encoder clock; "requested" (`gcode_move.gcode_position`) is set at
parse on the host clock. Different source, granularity, and clock → the two
drift. Homing moves the motors before `gcode_position` resets (actual leads);
printing lags by up to 200ms (actual delayed). Mainline instead evaluates the
*commanded* trajectory at `estimated_print_time` (trapq), which tracks
`gcode_position` in one clock domain.

### Finding 5: toolhead `print_time` status was hard-zero

**Evidence:** `klippy/motion.py` set `self.print_time = 0.0` in `__init__` and
never updated it; `get_status` reported it verbatim.

### Fix (landed, host-only — needs a klippy restart, no MCU/Rust rebuild)

- `motion_report.get_status` now evaluates the committed trajectory at
  `estimated_print_time` via the existing `engine.motion_state_at(print_time=…)`
  (mainline parity), falling back to the last good sample on error/empty; missing
  axes hold their previous value. (`klippy/extras/motion_report.py`)
- `motion.get_status` reports `print_time = self._mcu_pending_end_time` (the
  frontier), not a constant 0. (`klippy/motion.py`)
- Tests rewritten: `test/test_motion_report.py`.

### Deferred

- `live_motor_positions()` and the 200ms `live-position-poll` thread are now
  dead (no callers). Removing them sheds needless MCU bus traffic; left as a
  focused follow-up (Rust change → bench rebuild). `query_motor_positions`
  (on-demand) stays — used by parked-servo resync and gcode_move.
- Semantic note: live_position is now the *commanded* trajectory. If the bench
  wants the EtherCAT servo's *actual encoder* (following error) surfaced, add it
  as a separate status field rather than overloading live_position.

## Status

Active — crash fix landed (abort prevented by construction, exact trigger to be
confirmed from `head_trim_refused`); symptom #2 fix landed (host-only).
