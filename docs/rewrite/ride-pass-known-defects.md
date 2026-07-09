# Ride pass: known defects and redesign notes

Status as of 2026-07-09, from the fuzz/audit investigation (see
`rust/pipeline-snapshot/src/audit.rs` and the pinned `#[ignore]` regressions
in `rust/pipeline-snapshot/tests/planner_fuzz.rs`). Shipped so far:
chord-sag splice tolerance (commit 737d3e2bb), the stall fixes (defect 3),
and the super-rail lookahead guard (defect 2). The rest below are diagnosed,
reproducible, and deliberately not patched piecemeal — each attempted local
fix destabilized another part of the integrator, so they need one designed
treatment.

## Shipped

**Splice joint tolerance from chord sag.** The forward pass lands on the
sampled cap's linear chord; the brake chain is the exact cubic under it. On
steep brakes they disagree by the chord sag `|v''|·ds²/8` (`v'' = -a²/v³`),
far above the old fixed `1e-5` joint gate, so the exact-chain splice was
rejected and the pass chord-rode the envelope — a per-cell acceleration
staircase ending decels at thousands of mm/s² at seam nodes. Snapshot effect:
seam accel discontinuities down 25–60×, peak jerk 4–7×.

**Stalled states no longer integrate backwards** (was defect 3).
`state::advance` now truncates a step at the stall fold (`next_stall` was
already computed for the crossing solvers), so `s` is monotone by
construction; a `peel_feasible` march that reaches rest returns feasible
immediately (a stalled arc cannot cross the non-negative cap ahead) instead
of spinning its guard at the frozen state; and a pass step that ends at rest
still decelerating is an explicit rest event — chain goes opaque, the pass
resumes from `v = 0, a = 0` (or pins to the node when that node commands
rest). No test in the workspace exercises the rest event today; the whole
suite, the pinned `#[ignore]` failures, and a 5000-case
`hard_invariants_hold` run are byte-identical in outcome to before.

**Super-rail descents skip the kink lookahead** (was defect 2). A cell whose
cap chord drops faster than the accel rail is infeasible from everywhere
within it — no departure point exists — so `ride_step` no longer runs the
kink lookahead there: the cell is crossed as the cap chord it is, marked
infeasible (the same marking that already gated `binding`). The zero-progress
Peel storm itself was no longer reproducible standalone once defect 3
shipped (the storm's grind ran through the stall/rest machinery); direct
`reach_pass` probes with single- and multi-cell super-rail walls, straight
and curved, all crossed in bounded steps before the guard, and are pinned as
regression tests in `ride/tests.rs`. Effects of the guard: whole suite,
pinned `#[ignore]` failures, and 5000-case `hard_invariants_hold` identical;
one snapshot changed (`neptune_cube/printer/discontinuity`) — a single wall
crossing at t≈5.197 s re-tiles into one extra piece with seam metrics
unchanged and a rigid ~0.27 µs downstream time shift.

## Open defects

### 1. Cap walls ride as chords (accel staircase, the z-step pins)

`ride_step` detaches to Flight only when the cell's cap chord exceeds the
accel *rail*. A cap step-up within the rail (e.g. the backward pass climbing
a seam wall from cap 1 to 20.5: chord +22147 under rail 33143) rides as a
single `j = 0` chord phase — an instantaneous accel staircase the jerk limit
forbids, which poisons the brake chain, produces kinematically impossible
`(v, a)` sample pairs at seams, and drives the lowering quintic out of its
position window (`feed_drop_with_z_step_escapes_profile_window`,
`z_step_then_micro_reversal_escapes_profile_window`).

The naive fix — detach when `cap_a[cell] > st.a + j·dt` — is correct in
direction but was blocked by defects 2 and 3, both now shipped. Re-tested
with the naive detach scratch-applied on top of the defect-2 guard
(2026-07-09): no storms or hangs anywhere anymore, and
`feed_drop_with_z_step_escapes_profile_window` passes, but it is still not
landable as-is — `z_step_then_micro_reversal_escapes_profile_window` keeps
failing (window escape at a different point), two `reach_pass` wall shapes
lose chain completeness (a step-up wall followed by a wall drop, and a
curved wall under low jerk), and the seed-pinned `hard_invariants_hold`
finds a fresh lowering failure (`quintic profile velocity below zero`,
minimal input: two collinear planar moves, 0.0025 mm at feed 5.6 then 1 mm
at feed 437.6, scv 0, accel 33778, jerk 832727).

### 2. ~~Peel against super-rail descents makes zero progress~~ — fixed, see Shipped

### 3. ~~Stalled states integrate backwards~~ — fixed, see Shipped

Kept for numbering continuity. Note the rest-event path deliberately does
not fail loudly: stalls under a real cap still occur when 1/2-style
overbraking drives the profile to rest, and hard-failing there breaks
currently-green cases. Once 1 is fixed, tighten the rest event into
an assert.

### 4. Chord smear at splice joints (residual X/Y seam accel ≤ ~22 mm/s²)

With the sag-scaled gate, splices succeed but the joint still inherits up to
a cell of chord smear in `(v, a)`. Root fix attempted: evaluate the cap on
envelope-bound cells from the brake chain itself (per-cell Hermite of
`w = v²` built from chain node states — exact on constant-accel stretches,
O(ds⁴) through jerk swings, one Horner per lookup). Mathematically right,
but it shifts peel/land/trigger placement enough to excite 2 and 3, and the
integrator's tolerances (`TRIGGER_BISECT_ITERS`, snap bands, span merging)
are tuned against the chord representation. Do this only together with 1–3.

## Suggested order

3 and 2 are done; next 1, then 4. Verify each against: `cargo nextest run -p pipeline-snapshot`
(seed-pinned fuzz corpus), the `#[ignore]`d pins with `--run-ignored all`,
large-`PROPTEST_CASES` runs of `target_budgets_hold`, and the snapshot
suite's `seam_max_da`/`worst_seams` deltas.

## Not the ride pass

- E-axis seam steps (da 42–105, dv ≤ 0.19 on the discontinuity case) are the
  follower model: `FollowerDemand` is piecewise-linear in ratio, so
  `E_accel = r·a_t + r'·v²` steps by `v²·Δr'` (and E velocity by `Δr·v`) at
  extrusion-rate changes. Mainline Klipper permits instant E-velocity steps
  up to `instantaneous_corner_velocity` (1 mm/s default) and never bounds E
  accel, so this is a modeling-policy choice, not a regression.
- Tiny positive scv (0 < scv ≲ 0.1) NaNs the velocity plan (three pinned
  repros) — corner-blend math, upstream of the ride pass.
- The non-arc_fit fitter emits micro line stubs between clothoid runs with
  full endpoint curvature stepping onto them (both G2 taper mechanisms need
  half the stub's length and silently skip; the emitter checks only
  position continuity). Fitter scope.
