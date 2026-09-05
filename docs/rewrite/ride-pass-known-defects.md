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
hard-failing there breaks currently-green cases. Census (2026-07-09, a
temporary panic at the rest event under a positive cap): still fires
across the CI fuzz corpus at real caps (48–151 mm/s), and in the
`z_step_then_micro_reversal` pin — the assert cannot be tightened yet.
The remaining sources are the overbrake family: committed peels whose
feasibility verdict came from `committed_march`'s stall-is-`Clear` rule,
the same semantics that sank the Hermite cap re-attempt. Fixing the rest
events and un-blocking the Hermite cap are the same job: decide what the
oracle should say about commitments that end in rest.

### 4. Chord smear at splice joints (residual X/Y seam accel ≤ ~22 mm/s²)

Partially fixed by the anchored splice joints (see below); the residual X/Y
seam metric itself did not move and needs its source found first.

**Hermite cap re-attempt (failed, 2026-07-09).** The root fix suggested here
— evaluate envelope-bound cells from the chain via a `w = v²` Hermite — was
re-attempted on top of the wall-aware pass and still shifts trigger
placement destructively, through a sharper mechanism than before: with the
cap lifted off the chord onto the exact curve, committed arcs that used to
*cross* the chord instead fit under the cap and stall, and
`committed_march`'s stall verdict is `Clear` (feasible, the defect-3
choice), so the peel trigger commits too late, undershoots the tail, and
releases to a flight that must rest — rejected splices and negative
lowering quintics. Tightening the stall/tangency verdicts (rest under a
positive cap = `Crossed`, tangency must be recoverable) fixes that case but
regresses brake-to-rest behavior everywhere, because the envelope's own
tail sits exactly on `a² = 2jv`. The attempt is parked in the stash
(`hermite cap attempt`); don't retry it without first deciding what the
oracle should say about commitments that end in rest.

**Anchored splice joints (shipped in the working tree).** The productive
reframe: leave the chord world alone (trigger placement byte-identical) and
keep the discretization error out of the *emitted chain* instead, the same
anchoring idea as the wall brake and the disk integrator's landings:

- Every cap landing (peel contact, snap-land, flight reland) re-solves its
  landing arc onto the snapped state exactly (`anchor_landing`), so the
  snap kick never enters the chain — in both passes, which makes the brake
  chain itself C1 through its own landings.
- Splice joints bend the landing tail onto the chain's exact `(v, a)`
  (`anchor_tail`, walking up to 4 phases back), and where the joint sits
  inside the chain's own full-jerk swing — unreachable from the chord
  shadow, which lags the swing by half a cell of accel, at any depth —
  the pass merges onto the chain *ahead*, at its next phase boundaries
  (`merge_onto_chain`).
- Solvers: a graded two-phase closed form (`graded_bvp`, gentle
  corrections) falling back to the bang-bang hold families (`brake_bvp`
  and its mirror `boost_bvp`); every solution is validated by integration
  (stall folds make unvalidated solves land elsewhere — one fuzz case
  walked 38 mm off), by an under-cap/under-rail node sweep with the
  contact band's tolerance, and by a rail check at every interior phase
  joint — the graded solve constrains its jerks, not the accel excursion
  they produce, and an unchecked merge overshot `neptune_cube/fast`'s
  accel limit by 25%.

Effects: whole suite, `ci.sh quick`, and the pinned `#[ignore]` outcomes
identical to baseline; snapshot suite has 9 changed cases — seam
improvements where splice joints were the binding smear
(`neptune_cube/fast/discontinuity` Y `seam_max_da` 3.19 → 0.77,
`arc_fit/printer/circle` E 31.1 → 30.2), noise-level shifts elsewhere, and
one sub-0.1 mm/s² wobble on `facet_debris` X/Y (0.002 → 0.081).

**The headline X/Y residual — found and fixed, and it was never the ride
pass.** Diagnosed from the `perpendicular_corners_step_seam_accel` pin (two
plain 90° corners at printer limits; now un-`#[ignore]`d and green): the
straight lowering's micro-phase merge absorbed a leading nanosecond phase
(the ride pass emits one at member entry when the blend's rail-clamped exit
accel differs from the rail by a hair) by extending the host span *without
rebasing the host's coefficients* — time-shifting the whole host phase by
the merged duration and leaving a `v·lead` position gap (~1e-6 mm) at the
next joint. `bezier_pieces_to_nurbs` then welds joints by control point,
which converts a C0 gap δ into a `6δ/h²` acceleration corruption of the
next piece — 1.2 mm/s² on a 2 ms jerk swing, 11.3 on a 1 ms one, unbounded
as pieces shrink. Fixed by Taylor-shifting the host coefficients to the
span start (`lowering/straight.rs`). Snapshot effect: `printer/layer_5`
X/Y `seam_max_da` 52.5/20.0 → 13.1/15.4, `printer/discontinuity` Y
22.4 → 17.4. The remaining X/Y seams and the E-axis ones (follower model)
are separate stories.

**Follower pressure advance has a separate derivative contract.** In
`post_processor/chained_shaper_pa/extruding_corner`, fitting the unadvanced
follower position and acceleration first admitted quadratic pieces. Their
zero interior jerk erased the intended `k·E'''` contribution to advanced
acceleration. Leading linear and nonlinear advance now transform the
analytic follower P/V/A/J before fitting, so the existing position and
acceleration budgets apply to the actual motor command. Separately,
polynomial derivative-gain assembly preserves discontinuities with
degree-plus-one knot multiplicity instead of altering the next piece's
first control point; Bezier extraction and refinement support those seams.
No fit tolerance or snapshot baseline was changed.

For that case, at 10,001 uniform samples over `[0.32615, 0.3297245]` seconds,
the advanced E-acceleration error against
`0.05·(d|v_XY|/dt + 0.04·d²|v_XY|/dt²)` fell from max/RMS
`1789.65/1581.38` to `25.04/9.87` mm/s². The case-wide E acceleration seam
maximum fell from `30795.38` to `160.48` mm/s². Nonzero residuals remain:
this is tolerance-bounded fitting, not an exact analytic output curve, and
the leader spline's own derivative seams remain part of the input.

**The unshaped tanh fast corner had two additional fitting defects.**
In `nonlinear_pa/tanh/fast_corner`, a synthetic one-ULP-after-knot seed was
coalesced into the preceding fit interval. Its supposedly interior
endpoint then sampled the next phase's acceleration, making a flat
cruise ring toward the next phase's −150 mm/s² state. Exact-knot selection
also depended on the signal cursor's previous location. Keeping the
original knots and deterministic one-sided ownership removes that source
contamination.

The remaining acceleration waves were legal under the ordinary residual
budget but invented extrema absent from the analytic law. Constant-
acceleration nonlinear input now provides an analytic monotonicity
certificate, checked over each candidate's entire jerk polynomial using
its Bernstein controls. The certificate requires endpoint-P/V/A rungs
on nonconstant spans and seeds tanh's analytic acceleration extrema;
unsupported input keeps the ordinary fit contract.

Fresh exact-case evaluation, without regenerating a baseline, reduces
the maximum E-acceleration residual on `[0.27, 0.29998]` from
148.7001 to `2.41e-9` mm/s². On `[0.39, 0.43]`, it falls from
45.9746 to 0.001370 mm/s²; velocity and position residuals are at most
`3.77e-6` mm/s and `1.13e-8` mm. A dense scan including both sides of
internal fitted seams has no negative jerk or decreasing acceleration
through 0.43198 s. These are finite-precision polynomial approximations,
not exact analytic curves; no tolerance or baseline changed.

Also pre-existing (baseline, not these changes): the arc_fit cases
overshoot the accel limit at arc-to-arc junctions (max |a_XY| up to ~1750
against `max_accel: 1000`, e.g. `arc_fit/printer/circle` t≈1.4729).
Sourced: it is *fit-stage sliver ringing*, not the planner — the junction
carries a curvature kink (κ 0.14 → 0.55 over ~0.17 mm), `refine_span`
bisects the sampled fit down to ~0.14 ms pieces around it, and a sub-µm
position wiggle within the fit tolerance reads back as ±1000 mm/s² of
derivative garbage at that piece length (the planner's own samples are
rail-clamped ≤ 1000, and the actual dv/dt there is ~−600). Same failure
mode the straight lowering's micro-phase comments warn about, on the
sampled path: the fix is bounding derivative error on sliver pieces in the
sampled fit (or seeding a knot so the kink doesn't force slivers), in
`motion-pipeline/src/lowering.rs` / `ladder.rs`. No pin: the fuzz harness
cannot express arcs; reproduce via `arc_fit/printer/circle` and the
max-|a_XY| sweep.

## Suggested order

1, 2 and 3 are done, 4's splice joints are anchored, and the corner-seam
residual turned out to be the lowering's micro-phase merge (fixed); next
the defect-3 rest-event assert, and sourcing the pre-existing arc_fit
corner-apex accel overshoot.
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
