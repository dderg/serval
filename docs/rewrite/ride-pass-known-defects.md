# Ride pass: known defects and redesign notes

Status as of 2026-07-08, from the fuzz/audit investigation (see
`rust/pipeline-snapshot/src/audit.rs` and the pinned `#[ignore]` regressions
in `rust/pipeline-snapshot/tests/planner_fuzz.rs`). One fix shipped
(chord-sag splice tolerance, commit 737d3e2bb); the rest below are diagnosed,
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
direction but unmasks defects 2 and 3.

### 2. Peel against super-rail descents makes zero progress

A raw-vlc wall dropping faster than the rail is infeasible from anywhere in
its cell. If checked-mode ride enters the kink lookahead there, the departure
bisect degenerates to the current position and Peel grinds contact-sized
steps (~16 nm) forever: the cap outruns the brake, so the contact bisection
converges onto `touch = 0`. Today such walls are usually crossed by the
chord fast path only because the stride oracle happens to skip the lookahead
(`assume`); any upstream mode churn forces the checked path and the storm.
Guarding the lookahead with `cap_a[cell] < -(rail)` (chord-cross instead of
peel) is necessary but was not sufficient in testing — see 3.

### 3. Stalled states integrate backwards

`state::advance` clamps `v` at zero but keeps integrating `s` from the raw
kinematics, so a state that stalls while still carrying negative
acceleration walks *backwards* through the grid (observed: `s = -1.56` on a
0.5 mm run; also a state in cell 1223 while the node loop targeted node
1225). Flight/Peel then bounce across cells with nonsense cap/slope reads.
Any fix that changes when the pass brakes (1, 2, or a cap representation
change) excites this. A stall with `a < 0` mid-track is a planner-invariant
violation and should fail loudly or be handled as an explicit rest event,
never integrated through.

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

Fix 3 first (it is the invariant everything else trips over), then 2, then 1,
then 4. Verify each against: `cargo nextest run -p pipeline-snapshot`
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
