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

## Status

Active — fix landed (abort prevented by construction); exact trigger geometry to
be confirmed from the next run's `head_trim_refused` warn.
