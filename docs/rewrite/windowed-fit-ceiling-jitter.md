# Handoff: windowed-fit ceiling jitter → streamed-commit `OverCommitted`

Status: **RESOLVED 2026-06-22** (take-3 implemented). Fix: `fit_chain_with_head_restore`
(rust/geometry/src/fitter.rs) adds the committed head length back into the leading
junction's blend budget so a corner re-fits to the same curvature it had before the
commit trimmed the head; `StreamState` (rust/motion-engine/src/stream.rs) records the
trim in `committed_head_len` and threads it into the next fit. The interim
`WARM_START_REFIT_SLACK_REL` slack was reverted. Regression: `cold_run_infill_streams_without_overcommit`
(stream/tests.rs) + `leading_corner_curvature_invariant_to_head_trim` (fitter/tests.rs).
History below kept for context.

## One-line

A move's *fitted* speed ceiling is not invariant to which streaming fit window
it is fit in. The streaming commit pins a seam exit velocity computed in window
N; the re-plan in window N+1 re-derives a ~1% lower bound for the same move and
the strict `OverCommitted` guard aborts klippy.

## Symptom

- `stream_planner_fatal: velocity plan: OverCommitted { line_no: N }` →
  `std::process::abort()` (rust/motion-engine/src/stream_planner.rs `fatal`),
  klippy dies mid-print; the MCU drains its ring so motion looks like it
  "completed early" in Mainsail.
- Reproduces on `cold_run.gcode` (Neptune bench) every run once the buffer-drain
  fix (`be6580ba1`) is in. Not present before that fix only because the replay
  bug masked it.

## Confirmed mechanism (evidence-graded)

Reproduced offline and as a unit test. Instrumented `entry_brake` at the abort:

```
OVERCOMMIT[brake] line=53 entry_v=56.6655 entry_brake=56.0503
                  arc_to_end=31.0181 len=0.2811 accel=1000.0 n=96
```

- Overshoot is **0.615 mm/s (~1.1%)**. Guard epsilon `VELOCITY_EPS_MM_S = 1e-9`
  (geometry/src/velocity.rs:13) is a numeric epsilon, so the guard is effectively
  exact-equality and the 1% jitter trips it.
- Braking to a full stop is trivially feasible: `arc_to_end = 31 mm`, the jerk/
  accel-to-stop term is far above 56 mm/s. The **binding** term is
  `disk::disk_reach_v_rev(caps[0], v[1], len)` over a **0.28 mm boundary
  micro-segment** — i.e. the corner-speed cap `v[1]` of the move just after the
  committed seam.
- `v[1]` is set by the *fitted* curvature of that next move. The same physical
  move is fit slightly differently when the look-ahead window slides (front
  trimmed at the committed seam, new moves appended at the back), so its ceiling
  jitters ~1% between the window that produced `entry_v` and the window that
  re-checks it.

### Why it is a false positive, not a real over-commit

`entry_v` was *validated feasible* when committed (the committing plan pins
terminal velocity to zero and found the seam exit brakeable within that window).
The re-plan has **more** braking room, not less. The only thing that changed is
the re-derived first-move ceiling — a fit artifact, not a physical over-speed.
Hence the abort is spurious.

## Where it lives

- Guard: `rust/geometry/src/velocity.rs` — `plan_velocity_warm_start`,
  `entry_ceiling` check (~:200) and `entry_brake` check (~:297). The "terminal
  velocity stays pinned to zero … not something to silently clamp" doc-comment
  (commit `a45012ac9`) states the intended stance: real over-commits MUST fail
  loud. That stance is correct; this case is sub-tolerance fit jitter, which is a
  different animal.
- Seam exit pinning: `rust/motion-engine/src/stream.rs` —
  `self.entry_v = profile.moves[commit_count - 1].exit_v` (~:327).
- Chain fit (the jitter source): `geometry::fit_chain` /
  `rust/geometry/src/fitter.rs`. **Open question RESOLVED (2026-06-22 #2):** it is
  not a global-spline perturbation — it is a **front-edge head-reserve**
  non-determinism. Biclothoid blends consume length from the moves they join
  (`blend_trim`, `head_consumption`). When the committed move ahead of a corner is
  trimmed off the window front, the now-leading move takes the `r.start == 0`
  head-reserve branch (fitter.rs:200-205), changing the length budget available to
  the *next* corner's blend, which re-solves the biclothoid at a different
  curvature. Measured: cold_run move 55 endpoint curvature jumped 0.24803 →
  0.49244 (≈2×, corner cap 60 → 45 mm/s) when the window front advanced 53 → 54.

### Confirmed magnitudes

| window front | move 55 kappa | corner cap | note |
| ------------ | ------------- | ---------- | ---- |
| ≤ 53         | 0.24803       | 60.0 mm/s  | committed `entry_v ≈ 62.7` was valid here |
| 54 (aborts)  | 0.49244       | 45.1 mm/s  | cap now below committed entry → OverCommitted |

The brake-to-rest invariant is NOT the defect: warm-start pins terminal `v=0`
and `keep_secs` holds back a tail, so committed exit velocities are brakeable to a
stop by construction. The aborting term is the *corner* cap, and it changed after
commit.

## Take-3 root fix (this document's purpose)

Goal: a move's fitted curvature/ceiling is **invariant to the fit window**, so the
pinned seam velocity and the re-check agree exactly — no tolerance, no throughput
margin, loud-fail stance preserved for genuine over-commits.

Candidate approaches (pick after answering the local-vs-global question above):

1. **Carry committed boundary geometry forward.** The commit boundary is a clean
   (zero-curvature) seam. Cache the fitted segments for the kept tail from the
   producing window and reuse them in the next plan instead of re-fitting, so the
   curvatures that produced `entry_v` are exactly the ones re-checked. Needs the
   fit to be splittable at a clean seam without perturbation.
2. **Local/window-invariant fit.** If the fit is global, restructure so curvature
   near the front of the window does not depend on moves appended at the back
   (e.g. clamp the fit's dependency radius, or fit per-corner locally). Larger
   change; removes jitter for all downstream consumers, not just this guard.

### Acceptance / regression

- Un-ignore and pass `cold_run_infill_streams_without_overcommit`
  (rust/motion-engine/src/stream/tests.rs) — replays the real cold_run infill
  through one-move-per-commit and asserts no commit errors.
- Add an invariance assertion: the fitted curvature/ceiling of a move is equal
  (within a tight numeric tol) whether fit in a short window or a long one.
- Bench: `cold_run.gcode` streams to completion with no `stream_planner_fatal`.

## Interim mitigation (shipping first, take-1)

Relax the warm-start guard to accept `entry_v` exceeding the bound by a small
*relative* tolerance (fit-jitter band), keeping the loud abort for overshoots
beyond it (genuine over-commits). Trajectory stays continuous (the forward pass
pulls velocity down to `v[1]` after the elevated entry). This unblocks printing
but leaves the jitter in place — take-3 removes the need for the band entirely.

## Reproduction

```
# offline, deterministic — small commit cap = aggressive partial commits
cd rust
cargo run --release -q -p motion-engine --example dump_stream_trajectory -- \
    <cold_run.gcode> /tmp/out.csv --cap 4
#   bench limits 100/1000/jerk1e6 → "commit failed: OverCommitted { line_no: 52 }"
#   --cap 64+ plans cleanly → confirms it is commit-granularity / window jitter

# unit test (currently #[ignore]d)
cargo nextest run -p motion-engine --run-ignored all \
    -E 'test(cold_run_infill_streams_without_overcommit)'
```

Investigation case file: `_bmad-output/implementation-artifacts/investigations/print-completes-early-investigation.md`.
