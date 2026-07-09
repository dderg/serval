# Ride pass: known defects and redesign notes

Status as of 2026-07-09, from the fuzz/audit investigation (see
`rust/pipeline-snapshot/src/audit.rs` and the pinned `#[ignore]` regressions
in `rust/pipeline-snapshot/tests/planner_fuzz.rs`). Shipped so far:
chord-sag splice tolerance (commit 737d3e2bb), the stall fixes (defect 3),
the super-rail lookahead guard (defect 2), and the wall-aware ride pass
(defect 1).

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

**Cap walls no longer ride as chords** (was defect 1). A *wall* is a cell
whose chord demands more excess brake-slope shed than the cap holds —
`(slope² − prev²)/2j > cap_v`, measuring the *step* from the previous
chord's slope so a continuous brake curve's own steep tail (where the
profile arrives already carrying the slope) stays out of the class — within
the accel rail (super-rail cells keep the defect-2 chord treatment). Walls
now get anchored handling in `ride::reach_pass`:

- **Descending walls** are taken by a bang-bang boundary-value brake
  (jerk-down / hold / jerk-up, `brake_bvp`) that departs at the latest
  feasible point (the trigger oracle *is* BVP solvability, so the feasible
  region ends exactly where the anchored brake stops fitting) and arrives at
  the wall run's end node with exactly its cap value and the following
  chord's slope. Wall chords themselves constrain nothing; the profile owes
  only the step's bottom at its node — the same "discretization error never
  enters the emitted chain" idea as the disk integrator's anchored landings
  (PR #225). Walls that don't bind (current speed already under the bottom)
  are ignored; an unsolvable BVP (no room) falls back to the marching peel.
- **Ascending walls** — the mirror class, a chord whose slope gain from the
  previous chord costs more speed (`(slope² − prev²)/2j`) than the cap
  allows — detach to Flight, and Flight's direct cap landing refuses those
  chords too (landing at a wall foot used to snap `a` onto the wall chord,
  recreating the staircase the detach avoided). The class must be intrinsic
  and step-based: a state-based reachability test (`cap_a > st.a + j·dt`)
  sits exactly at the boundary the brake envelope's own jerk-swing cells
  step by, and chattered ride/flight along every accel ramp — a visible
  velocity ripple that E amplified via `r'·v²`.
- **Super-rail chord crossings no longer poison the state**: when the
  rail-clamped chord slope is unrecoverable (its jerk swing back sheds more
  speed than the profile holds — the `v → 0.0077` collapse that produced
  negative lowering quintics), the arrival state resumes from the slope the
  cap ahead commands instead. Recoverable crossings keep the chord slope, so
  ordinary arc-corner descents are untouched (a blanket resume measurably
  slowed them).
- **Splices now also fire when a binding stretch begins mid-ride** (the
  previous stretch ending at an infeasible node left no mode transition to
  hang the splice on), so brake-to-rest tails adopt the envelope's exact
  chain instead of chord-riding it into an early stall.

Effects: all three defect-1 window-escape pins pass and are un-ignored
(`feed_drop_with_z_step_escapes_profile_window`,
`z_step_then_micro_reversal_escapes_profile_window`,
`planar_micro_move_decel` still green), the previously-failing wall shapes
are pinned as regression tests in `ride/tests.rs`
(`jerk_wall_is_taken_by_an_anchored_brake`,
`mesa_step_up_then_drop_keeps_the_chain_complete`,
`ascending_wall_detaches_to_flight`), the whole suite and a 5000-case
`hard_invariants_hold` run are green, and the snapshot suite is
byte-identical to baseline — the wall treatment only changes inputs whose
caps carry genuine sampled velocity steps, which the snapshot corpus's
well-formed g-code never produces.

## Open defects

### 1. ~~Cap walls ride as chords~~ — fixed, see Shipped

Kept for numbering continuity. Known residual gaps of the wall treatment,
none currently failing a test: the oracle only checks the *first* binding
wall ahead (two walls closer together than the second one's brake distance
would fall back to the marching peel), and `brake_bvp` bounds its hold at
the rail sampled at the departure and anchor states, so a strongly curved
wall stretch could over-promise mid-maneuver.

### 2. ~~Peel against super-rail descents makes zero progress~~ — fixed, see Shipped

### 3. ~~Stalled states integrate backwards~~ — fixed, see Shipped

Kept for numbering continuity. Note the rest-event path deliberately does
not fail loudly: stalls under a real cap can still occur (the BVP fallback
to marching peel when a wall leaves no room, tiny-scv notches), and
hard-failing there breaks currently-green cases. With 1 fixed, tightening
the rest event into an assert is worth re-probing — run the fuzz tiers with
a temporary panic there to census the remaining stall sources first.

### 4. Chord smear at splice joints (residual X/Y seam accel ≤ ~22 mm/s²)

With the sag-scaled gate, splices succeed but the joint still inherits up to
a cell of chord smear in `(v, a)`. Root fix attempted: evaluate the cap on
envelope-bound cells from the brake chain itself (per-cell Hermite of
`w = v²` built from chain node states — exact on constant-accel stretches,
O(ds⁴) through jerk swings, one Horner per lookup). Mathematically right,
but it shifts peel/land/trigger placement enough to excite 2 and 3, and the
integrator's tolerances (`TRIGGER_BISECT_ITERS`, snap bands, span merging)
are tuned against the chord representation. 1–3 are now fixed, so this is
unblocked — re-attempt on top of the wall-aware pass.

## Suggested order

1, 2 and 3 are done; next 4, then the defect-3 rest-event assert.
Verify each against: `cargo nextest run -p pipeline-snapshot`
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
