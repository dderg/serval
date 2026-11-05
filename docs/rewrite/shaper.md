# The shaper: axis chains, smooth kernels, and the motor command

The shaper is the last stage of the streaming motion pipeline
(`setup_pipeline` in `rust/motion-core/src/worker.rs`: fitter → planner →
turning guard → lowerer → **shaper**). It takes the lowered trajectory — per-axis piecewise
polynomial tracks that already satisfy the velocity/accel/jerk limits and the
clothoid corner blending — and turns it into what each motor should actually
be commanded to do. Everything the classic Klipper stack does after the
trapezoid generator (input shaping, smoothers, pressure advance) lives here,
unified under one model, plus things that stack cannot express.

In the live host, fitting, planning, the turning guard and lowering run on `kalico-planning`,
and the shaper runs on its own `kalico-shaper` thread
(`motion_pipeline::StagedPipeline`). Lowered items reach it in order through
a bounded inbox (`SHAPER_INBOX_CAP`), so a slow shaper backpressures planning
and a barrier still acknowledges only after everything ahead of it has been
shaped. Snapshot generation runs the same four stages synchronously through
`motion_pipeline::Pipeline`; numerical parallel work within a stage remains.
That parallel work — the planner's member profiles, the shaper's leader
target and span fits, follower source and kernel fits, arc lengths — fans
out on one persistent worker pool the pipeline shares between the two
stages (`kalico-pool-N` threads, one per spare core up to three, spawned on
the first fan-out): the calling thread takes jobs alongside whichever
helpers are idle, results merge in job order, and a job's panic resurfaces
on the calling thread.

The core idea: **resonance suppression and mechanical compensation are
per-axis linear operators on the planned trajectory**, and there are exactly
two kinds worth having:

1. **Smoothing kernels** — convolution with a compactly supported, unit-mass
   polynomial kernel. This is the continuous generalization of input shaping:
   Klipper's discrete ZV/MZV impulse trains and Kalico bleeding_edge_v2's
   "smoothers" are both special cases of "convolve position with a kernel
   whose spectrum has a notch at the resonance".
2. **Derivative gains** — the operator `y = x + k1·ẋ + k2·ẍ`
   (`ChainStage::DerivativeGains`). Pressure advance is `k2 = 0`. Full
   inverse dynamics of a resonant mode (`mode_inverse`) uses both terms.

Each axis gets a *chain* of these stages, configured declaratively:

```ini
[axis x]
post_processors: shaper_x, inv_x

[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 120

[post_processor inv_x]
type: mode_inverse
frequency_hz: 120
damping_ratio: 0.1

[axis e]
follows: x, y, z
post_processors: pa
max_velocity: 50
max_accel: 5000

[post_processor pa]
type: linear_pressure_advance
k: 0.03
```

Sources: `rust/trajectory/src/chain.rs` (stage model, compilation,
`AxisChainSet`), `rust/trajectory/src/kernel.rs` (kernel construction),
`rust/trajectory/src/algos/` (the post-processor roster),
`rust/motion-pipeline/src/shaper.rs` (the streaming stage),
`rust/motion-pipeline/src/follower_projection.rs` (follower axes).

## Why continuous kernels instead of impulse trains

Classic input shaping convolves the command with a train of 2–5 delta
impulses. That works, but the output inherits the input's smoothness class —
a jerk-limited but accel-discontinuous profile stays accel-discontinuous,
just copied N times. A *smooth* kernel of the same notch quality produces a
C²-continuous command regardless of input, which matters at our speeds: the
steppers see no accel steps, and everything downstream (lowering to steps,
EtherCAT servo feedforward) gets a twice-differentiable signal.

The kernels are piecewise polynomials (`PiecewisePolynomialKernel`), so the
convolution of a polynomial trajectory piece with the kernel is computed
**exactly** — `ShapedSignal` integrates with Gauss quadrature between the
breakpoints of input and kernel, never straddling a polynomial change
(`signal_breakpoints` in shaper.rs). No sampling error enters the signal
itself; approximation happens only at the final refit (below), under an
explicit tolerance.

Roster (`rust/trajectory/src/algos/`):

- `smooth_zv`, `smooth_mzv` (`frequency_hz`) — the Kalico bleeding_edge_v2
  smoother polynomials (`shaper_defs.py`, `get_zv_smoother` /
  `get_mzv_smoother`), Maxima-optimized so a damped oscillator's residual
  vibration stays low in a band around the target frequency. Support width is
  `0.8025/f` (zv) and `0.95625/f` (mzv). The unit-interval coefficient tables
  live in `kernel.rs`; `build_unit_polynomial_kernel` rescales them to the
  target duration, normalizes to unit integral, and **shifts the kernel so
  its first moment is zero** — a zero-mean kernel adds no systematic
  time lag, so the shaped track stays phase-aligned with the plan.
- `smooth_bell` (`smooth_time`) — the C¹ quartic bell `c·(h²−t²)²`, a
  general-purpose low-pass with no tuned notch.
- `smooth_triangle` (`smooth_time`) — the classic triangle (double boxcar);
  cheap general smoothing. After an extruder's advance it is an explicit
  override that trades a bead bulge at every corner for lower extruder
  acceleration; the recommended extruder chain is the advance alone.
- `linear_pressure_advance` (`k`) — `DerivativeGains { k1: k, k2: 0 }`.
- `tanh_pressure_advance`, `recipr_pressure_advance` (`linear_advance`,
  `nonlinear_offset`, `linearization_velocity`) — `NonlinearAdvance`, the
  operator `y = x + a(ẋ)` with
  `a(v) = linear_advance·v + nonlinear_offset·s(v / linearization_velocity)`.
  Kalico bleeding_edge_v2's two nonlinear pressure-advance models, ported
  to the chain: `s(u) = tanh(u)` and `s(u) = u/(1+|u|)`. The latter is the
  odd extension of bleeding_edge_v2's `1 − 1/(1+u)` — identical for the
  forward flow the model was written for, but finite on retraction, where
  the original is singular at `u = −1` and sign-flipped past it. Both
  occupy the same slot as `DerivativeGains` (one gain stage per axis) and
  degrade to it exactly when `nonlinear_offset = 0`. Because `a` is not
  polynomial in the track, the stage is applied by re-fitting the
  transformed signal (`apply_nonlinear_advance_to_track`) under the same
  ladder budgets the convolution refit uses, and a move carrying one
  pre-kernel takes the sampled lowering path rather than the closed-form
  per-piece one.
  Piece boundaries retain their exact times; derivatives are sampled from
  inside each fit interval, with deterministic left ownership at a shared
  knot. On constant-acceleration input, the nonlinear law also certifies
  acceleration monotonicity: tanh changes direction at
  `v = ±linearization_velocity·atanh(1/√3)`, while reciprocal has fixed
  direction on each side of zero velocity. The fitter seeds those
  transitions and classifies intervals using those same root times, avoiding
  endpoint-velocity roundoff that could remove the certificate. Candidates'
  Bernstein jerk controls must prove the certified sign within the floating-point
  basis-conversion error bound, not the physical fit budget. Nonconstant certified
  spans use endpoint-P/V/A Hermite rungs, so a low-order plateau cannot spend the
  acceleration budget as a jump between pieces. A bounded adaptive interval check
  bounds acceleration error between probes using the candidate's Bernstein hull
  and the monotone signal's endpoint values; inconclusive intervals request
  refinement. The span floor cannot bypass this check. General polynomial input
  supplies no such certificate and retains the ordinary fitting contract.
- `mode_inverse` (`frequency_hz`, `damping_ratio`) —
  `DerivativeGains { k1: 2ζ/ω, k2: 1/ω² }` with `ω = 2πf`.

A kernel's *variance* (second moment, `kernel_variance_s2`) is its one
number the planner needs — see the corner budget section.

Moment convolution differentiates the input polynomials, not the kernel. At
input joins, position jumps contribute `Δp·k(t−b)` to output velocity and
`Δp·k'(t−b)` to acceleration; slope jumps contribute `Δv·k(t−b)` to
acceleration. All three terms are required when derivative gains produce
genuine discontinuities. Preserving their polynomial pieces without these
terms would make the convolved position, velocity, and acceleration inconsistent.

### Oscillation property tests

`motion-pipeline/src/phase_ripple_fuzz.rs` checks flat cruise velocity and
acceleration before a changing phase, through both the shared axis fitter and
nonlinear advance. `nonlinear_ripple_fuzz.rs` checks analytic ramp fit budgets,
acceleration direction inside pieces, and velocity/acceleration behavior at joins.
The generators vary axis labels, flow direction, phase durations, time origins,
velocity scales, and tanh/reciprocal parameters, including velocity-zero crossings.
The oracles differentiate native Bernstein controls rather than unstable
position-power conversions; the ramp oracle searches jerk extrema independently
of the fitter's sign certificate.

Both modules retain shrinking/replay seeds in `motion-pipeline/proptest-regressions/`.
Normal Rust tests run 128 cases per property; `./scripts/ci.sh rust-fuzz` includes
them at `PROPTEST_CASES` (20,000 by default). Axis labels exercise shared scalar
fitting, not full kinematic routing or physical resonance response. These tests
reject invented oscillations where the analytic signal supplies a shape contract;
they do not forbid legitimate oscillatory motion.

## Composition rules

`CompiledChain::compile` enforces the v1 composition contract, failing
loudly at config time rather than producing silently wrong motion:

- at most **one kernel** and **one derivative-gain stage** per axis
  (`UnsupportedComposition`);
- a gain stage with `k2 ≠ 0` must come **after** a kernel
  (`AccelGainNeedsPrecedingKernel`): applied before the kernel it would run
  in the lowerer, whose curved-path sampler carries no jerk and cannot form
  the transformed velocity. After a kernel the operator commutes with the
  convolution — `x + k1·ẋ + k2·ẍ` of `k ∗ f` is `k ∗ (f + k1·ḟ + k2·f̈)` —
  so the motor command is the exact convolution of the gained input
  polynomials (`AxisSignalTable::with_derivative_gains`), with the input's
  acceleration and velocity steps carried as kernel-weighted jump terms.
  The input is only position-continuous at its knots, so `ḟ` and `f̈` also
  carry impulses there: a velocity step `Δḟ` puts `k2·Δḟ·δ` into the gained
  input, a position step `Δf` puts `k1·Δf·δ + k2·Δf·δ′`. Gaining the pieces
  drops them, so `GainImpulseResponse` adds their convolution back —
  `δ ∗ k = k`, `δ′ ∗ k = k′` evaluated at each step.

Order within a chain is meaningful: stages before the kernel act on the
input signal, the kernel convolves the result, stages after the kernel act
on the convolution. That last position is special — see next section.

## Motor command vs toolhead signal

This is the load-bearing distinction of the whole design.

A smoothing kernel changes where the *toolhead* goes: the physical machine
follows the convolved path (that is the point — the resonance never gets
excited). A trailing derivative-gain stage does **not** change where the
toolhead goes; it changes what the *motor* is told to do so that the
toolhead follows the kernel-convolved path *more faithfully*.

`mode_inverse` makes this concrete. Model the axis as a damped second-order
mode: toolhead position `x_th` responds to motor position `u` as

```
ẍ_th + 2ζω·ẋ_th + ω²·x_th = ω²·u
```

Inverting: to make the toolhead track a desired signal `x` exactly, command

```
u = x + (2ζ/ω)·ẋ + (1/ω²)·ẍ
```

which is exactly `DerivativeGains { k1: 2ζ/ω, k2: 1/ω² }` applied to the
kernel output. The kernel guarantees `x` is C² (so `ẍ` exists and is
continuous — this is why `k2` requires a preceding kernel), and the gain
stage pre-distorts the command with a velocity feedforward and an
acceleration counter-drive.

Consequences you will see on the graphs, both intentional:

- the motor command carries a `k2·jerk` counter-drive — visible as
  velocity/accel ripple through corners that the toolhead signal does not
  have;
- an axis's motor command leads its toolhead signal by `k1·v` — with
  different per-axis frequencies this shows up as a constant, velocity-
  dependent offset of the motor path from the toolhead path on diagonals.

And a built-in diagnostic: if those artifacts ever show up **on printed
parts**, the `(f, ζ)` model for that axis is wrong — the physical plant was
supposed to cancel them.

The shaper materializes the distinction (`apply_motor_side_stages` in
shaper.rs): trailing gains are applied as the *last* transformation, after
follower projection, and `Shaper::with_toolhead_tap` can mirror every
emitted segment as it stands *before* that stage. A leader axis's motor
command is its own velocity-continuous fit of that exact gained convolution
(`fit_motor_commands`, fit context `motor_command`), never a derivative of
the fitted toolhead track: a fit is only C1–C2 at its knots, so
differentiating it twice would step the command's velocity by `k2·Δjerk` at
every seam. The snapshot pipeline uses the tap to emit optional
`toolhead_*_pieces`, which is what the snapshot viewer and playground plot
as the toolhead signal alongside the motor command.
`CompiledChain::has_motor_side_gains` / `AxisChainSet::has_motor_side_stages`
define "motor-side": a gain stage that follows a kernel.

## Followers: the extruder rides the toolhead

`[axis e] follows: x, y, z` declares a *projected follower*
(`AxisChainSet::followers`). A follower's track is not convolved with the
leaders' kernels and not planned against the leaders' timing; it is
**re-projected onto the leaders' shaped motion**
(`follower_projection.rs`):

- The raw move stream defines an extrusion-per-path-distance profile `r(s)`:
  each spatial segment contributes a span of raw arc length carrying its
  follower demand ratio (zero for travel moves).
- The projection extrudes each raw millimeter when the *shaped* path
  traverses it: follower velocity is `r(s_shaped(t)) · |v_shaped(t)|`,
  position is the running integral.

This makes extrusion physically consistent with what the nozzle actually
does: the follower keeps moving through rest holds (kernel creep is real
traversal of the raw path's tail), it is continuous across every seam, and
the total extruded amount tracks the shaped path's true length, not the
G-code's: a blend that cuts a corner lays that much less than the commanded
total, and blends that run longer than their polyline (a loop of moves
shorter than the corner budget) lay more. `planner_conservation_fuzz` holds
the planner to it. Extrude-only moves ride no spatial path and add in
directly.

Where the raw toolhead rests but the shaped leaders still move — the
kernels' spill past a stop and their reach ahead of a start — the follower
lays the ratio over the **chord**, not the arc: the move before extrudes over
the shaped leaders' straight-line distance from the rest point where the
spill starts, the move after over their distance to it where the spill
ends (`FollowerState::ingest`). The chords telescope to the raw rest
positions, so a stroke from rest to rest lays exactly its commanded E —
the slicer's volume, which the user sets and can check
(`a_shaped_stroke_from_rest_to_rest_extrudes_its_command`) — however the
shapers curve the spill or the fit wiggles near the rest point. It differs
from `r·∫ds` by `r` times the spill's arc minus its chord: sub-micron on the
bench and Trident configs, where axes with different shapers bend the
spill off the line to the rest point. `advance_conservation` holds the
projection to this rule with and without an advance.

The ratio is not projected raw. Every follower smooths `r(s)` along the path
(`follower_projection/ratio_ramp.rs`) with the kernel `¾(1 − x²)` of
half-width `h(s)`: half the path the shaped leaders cover in the ramp time
`τ` around `s` — `v·τ/2` at cruise, `a·τ²/8` at a turnaround — closing to
zero at every rest. A ratio step then reaches the follower as a smoothstep
`S` over about `τ` (`S′ ≤ 3/2`, `|S″| ≤ 6`), however finely the path is cut
into moves, and `|h′| ≤ 0.75` keeps the laid ratio a convex combination of
the commanded ones. Each seam derives its own `τ`
(`geometry::RatioRampRule::ramp_time`) from the leaders' shaped speed,
acceleration and jerk there, the steps within half the longest ramp of it,
the advance's slope `k` (`CompiledChain::advance_slope`: a linear
advance's `k`, a nonlinear one's small-signal slope) and the axis's
`max_velocity` / `max_accel` (`V_e`, `A_e`): the shortest in
[`RATIO_RAMP_MIN_TIME_S`, `RATIO_RAMP_MAX_TIME_S`] = [5, 30] ms whose peak
advanced follower velocity and acceleration stay within both. At a steady
speed, with `Δ = |Δr|·v`, the acceleration bound is `τ ≥ √(6kΔ/A_e)` for
`τ ≤ 2k` and `τ ≥ 1.5Δ/A_e` without an advance; the velocity bound is about
`τ ≥ 1.45kΔ/(V_e − r·v)` for the larger flow `r·v` (the overshoot
coefficient rises to 1.5 as `τ/k` shrinks). An unlimited follower takes
5 ms everywhere. Around its seam a step's `τ` holds within `τ` of it and
falls off by half the time beyond; the leaders are sampled every 2.5 ms.
Where even 30 ms exceeds the limits, the velocity planner caps the speed at
the join (`velocity::extruder_caps::ratio_step_ceiling`, through
`RatioRampRule::step_speed_cap`) rather than smearing the step over more
path. A rest of the leaders `d` along the path from the step narrows its
window too: the width envelope rises off a rest by at most
`RATIO_RAMP_ENVELOPE_SLOPE = c = 0.25` per millimetre, so a window past the
step reaches back to it only out to `c·d/(1 + c)`, and the step's ramp gets
no more than that path on the rest's side. Each move's
`ExtrusionBudget::rest_room_mm` carries the path from its start to the
nearest rest (an extrude-only move or a stream end) within
`rest_room_reach_mm` of the fastest the leaders cross it, and the cap holds
the ramp that room leaves within the limits as well.

The cap searches the planned speed `v₀` at the join, not the shaped one.
The planned speeds within the leaders' kernel reach of the join stay under
the fastest ceiling `C` within the move's reach
(`ExtrusionBudget::fastest_ceiling_mm_s`) and within `A` per second of
`v₀`, so a kernel `w` shapes them into a speed of at most
`∫|w(u)|·min(C, v₀ + A·|u|) du = mass·C − A·Φ((C − v₀)/A)`
(`geometry::LeaderSpill`, which tabulates `Φ(x) = ∫|w|·(x − |u|)⁺` and its
slope over 16 cells of the kernels' reach). It reaches `mass·C` as `v₀`
reaches `C` and falls continuously below it, so a step that fits at the
ceiling caps nothing and a vanishing step needs a vanishing slowdown. The
leaders' own advanced flow `r·(σ̇ + k·σ̈)` is shaped the same way from the
planned `v + k·a ≤ min(Q, v₀ + k·A + A·|u|)`, with `Q` the least of
`C + k·A` and `V_e/r_b`: every move about the step budgets its peak ratio
`r` at a level `r_b ≥ r` and holds `r_b·(cⱼ + k·Aⱼ) ≤ V_e` itself, so the
own flow never reaches `V_e` from the step's independent worst case of
speed and budget, the way a noise step of `Δr ≤ 3e-5` once forced a stop
at every such join of the Trident first layers.

Critically, the projection reads the leaders' **toolhead signal**, not their
motor command — the follower must track where the nozzle physically is, and
the mode-inverse counter-drive is motion the toolhead never performs. This
is why `apply_motor_side_stages` runs only after `project_followers`.

The follower's own chain then applies on top of the projection in the same
stage order: leading gains (pressure advance) act on the projected track,
the follower's kernel (e.g. `smooth_triangle`) convolves that, trailing
gains act on the convolution.

### The turning guard

An advance reads the shaped path's arc length `σ`: it asks the extruder for
`r·(σ̈ + k·σ⃛) + c·(r·σ̈)²`, with `σ̈ = a·v̂` and `σ⃛ = j·v̂ + a⊥²/|v|`. The
planner bounds that per move from the *planned* path, where `a⊥²/|v|` is at
most `v³κ²` under the disk. On the shaped path the turning term reads the
*shaped* speed, which the leaders' kernels fold far under the planned one
wherever turns lie closer together than their reach — a reversal the
kernels round into a crawl of a few mm/s asked a kernel-free advance for
33 kmm/s² against a 20 kmm/s² limit. A follower kernel bounds the same term
on its own (`own_jerk_per_accel_hz`), so only kernel-free followers with an
advance need more.

For them `motion_pipeline::turning_guard::TurningGuard` sits between the
planner and the lowerer, in the one pipeline the live host and the
snapshots share. It holds committed moves until every instant whose kernels
read them is proven to fit the extruder's `max_accel`:

- **Screen** (`screen::passes`): from the planned moves within the kernels'
  reach alone, per-axis boxes of planned velocity and acceleration, split by
  each kernel's positive and negative mass, bound the shaped speed below and
  the shaped acceleration, `|a × v|` and the tangential jerk (`G·A`) above.
  A move whose bound fits passes; nearly all do.
- **Shadow** (`shadow::shaped_at`, `CellMotion::between`): otherwise the
  planned motion, sampled every ≤ 0.25 ms (48 cells across the narrowest
  leader kernel), is convolved with each leader kernel linearised cell by
  cell, every term carrying an explicit error radius, and every grid cell
  across the move gets rigorous bounds on the shaped speed, acceleration,
  `|a × v|`, jerk and their parts along the velocity, including between the
  grid instants. A cell that binds is checked again on a grid 8× finer.
  Below `ADVANCE_VALLEY_SPEED_MM_S` the advance lays a valley instead and the
  cell asks nothing of the turning term.
- **Back-off**: a cell that still binds scales the acceleration budgets of
  the moves it reads by `(limit/demand)^(2/3)`, clamped to [0.5, 0.95], and
  re-plans the span around them between two seams whose speeds it keeps
  (`geometry::replan_span`): the same caps, stops, seam ceilings and
  extrusion budgets the planner applies, on the lowered budgets. The
  re-planned moves are checked again; a move backed off 64 times fails
  loudly.
- **Release**: a move leaves once no pending check can back off a move it
  depends on: the earliest planned time a pending check reads, less the
  kernels' backward reach, has a brake horizon
  (`geometry::brake_horizon`) — the last seam a rest there cannot lower on
  the current budgets — and moves whose kernels read only before that seam
  are emitted. The guard's output thus reads its input sequence alone,
  however the planner batches its commits.

The guard holds up to the last committed move plus the brake horizon and
the kernels' reach: on the dense print about 0.2 s of planned motion at the
median and 0.3–0.4 s at worst (a single long slow move on the first layers
holds its whole length). The pacer reads the dispatched runway, past the
guard, so the hold makes it drain earlier, never late; a drain flushes the
guard in under a millisecond. Kernel followers pass straight through.

### Rests

The projection reads the leaders' path speed `|v|` and, for an advance, its
rate `d|v|/dt = v̂·a` and its derivative `v̂·j + |a⊥|²/|v|` off the leaders'
*fitted* polynomials. Near a stop those fits move at their own fit noise. X
and Y stop at slightly different times, and the direction `v̂` turns through
nanometre loops that no toolhead makes. `|a⊥|²/|v|` then spikes without
bound over a microsecond, and the advance would turn the spike into
extruder acceleration.

A **rest** (`follower_projection/rest.rs`) is a maximal stretch inside the
support where the leaders' speed is at most the speed their fits cannot
resolve. That speed is the leader fit's velocity resolution: the
acceleration tolerance times the shortest span such a fit holds at the
leaders' position, speed and time scales (`velocity_resolution_at_scales`).
No constant of its own enters.

Inside a rest:

- `|v|` and so the odometer are unchanged. Path distance, and with it
  material, is exactly what it is outside rests.
- The rate and its derivative are read along a fixed direction: the unit
  velocity the leaders enter the rest with, up to the rest's *split* (where
  they are slowest), and the one they leave with from the split on.
  `d|v|/dt = d̂·a` and `d²|v|/dt² = d̂·j`. The fit-noise curvature term is
  dropped.

The consequences:

- A stop-and-go along a line enters and leaves along the same direction. It
  reads smooth through the split.
- A reversal, or a turn made at rest, enters and leaves along different
  directions. Its rate steps at the split by what the two directions read
  of the acceleration. That step is the jump the motion makes, kept and not
  smoothed:
  - the split is a semantic break of the fit;
  - it is a break of the follower's enclosure windows, each side enclosed
    with its own direction.
- The rest's ends are breaks of the enclosure windows, not of the fit.
  There the rate is continuous, and the dropped curvature term at the
  floor speed is the only change.

Rests never reach into the end slivers, which keep their own semantics. A
rest that runs into one reads the single direction the leaders have at its
other end.

### Square-root onsets

An unshaped leader that leaves its curvature cap (a rail `Departure` pair)
raises its tangential acceleration as `√τ`, `τ` the time from the cap. Its
jerk there is `J/√τ` plus bounded terms, `J = 0.75·|c₃|/h^{3/2}` from the
pair's coefficients (`SquareRootOnset`). An advance reads that jerk through
its gain `G` (steepest advance slope × ratio bound × odometer scale), so the
follower's acceleration is unbounded at the onset. No fit holds it.

The **onset sliver** (`OnsetSliver`, `follower_projection/enclosure.rs`) is
where the singular term alone passes the follower's acceleration tolerance:
`τ < (G·J/accel_tol)²` on the departure side, clipped to the pair. The
width comes from the tolerance; no constant of its own enters. Inside the
sliver, one typed exemption (`UnshapedLeaderStep`) applies. It is the same
allowance an advance's velocity step at an unshaped leader's acceleration
step takes: `instantaneous_corner_velocity` plus what the leaders' fit
rounding makes.

- The acceleration bound is dropped. The enclosure's pieces carry the
  exemption (`ExactPiece::unshaped_step`) and the hull skips their
  acceleration.
- Position is proven within tolerance, as everywhere.
- The fit's velocity is proven within the allowance of the signal's.
- The velocity change the singular term makes across the sliver,
  `2·G·J·√τ`, must stay within the allowance. A larger one is refused
  before the fit with `PostProcessError::UnshapedLeaderOnsetVelocityStep`.

### The ratio ramp in enclosures

Every follower ramps its ratio steps (`RatioSmoothing`), and every advance
reads the laid velocity weighted by its valleys, `R = W·r̃`. A follower's
enclosure window models both (`follower_projection/ratio_ramp/models.rs`):

- **Correction.** The smoothing adds `X(s)` to the table: moments of the
  seams inside the window, weighted by powers of the width `h(s)`
  (`SeamMoments`). Over a window its formula is fixed: the width's node
  interval, the seams inside the window and those behind its point. The
  bends (`RatioSmoothing::bends`) are where these change, and windows
  break there. Each derivative `X⁽ⁿ⁾`, `n ≤ 3`, is its Taylor polynomial
  about the window's centre arc plus the Lagrange remainder past it. The
  remainder is bounded by the same series computed over the window's
  whole odometer reach, in ball arithmetic: an interval Taylor series.
- **Valley weight.** `W`, `W'`, `W''` on the side of each valley the
  window's reach lies on.
- **Read.** The advance reads `R·ṡ`, `R'·ṡ² + R·s̈`, and
  `R·s⃛ + 3R'·ṡ·s̈ + R''·ṡ³`.

A reach a few ulps wide can straddle a bend located an ulp off from where
the evaluator flips. Such a reach is enclosed by every formula its floats
read, each widened by its departure from the others. A width node reads
its jet from the interval it opens, so the evaluator keeps the width's
third derivative there.

## Streaming mechanics: windows, frontier, drain

The shaper is a streaming stage over channels; it never sees the whole
trajectory. Correctness reduces to window bookkeeping:

- A convolution `(f ∗ k)(t)` reads `f` over `[t − k_hi, t − k_lo]` — the
  kernel's support enters **reflected** (`ChainStage::input_window`). The
  smoother kernels are asymmetric after mean-centering, so this distinction
  is real.
- `AxisChainSet::{forward_support, back_support}` bound how far ahead the
  shaper must buffer (`pending`) and how much emitted raw history it must
  retain (`history`). Follower supports **cascade**: a projected follower
  with its own kernel reads the projection, which reads the leaders'
  convolution — supports add (`axis_support`).
- `supported_count` gates emission: a segment is emitted once its whole
  forward window is covered by buffered lookahead. For followers with their
  own kernel there is an explicit second gate on the *shaping frontier* (the
  last pending segment whose direct convolutions are final), because segment
  granularity can leave a long straddling segment unprojectable even when
  the time-based bound is met. Projection runs permanently ahead through the
  frontier and caches one fitted pre-kernel track per raw segment, so every
  emit reads bit-identical convolution inputs.
- A `Drain` marker flushes the buffered tail with the window clamped past
  the terminal rest — exact, not speculative, because the lowerer holds the
  timeline at that rest for the chains' forward support before any
  subsequent motion. Time gaps (dwells, drain holds) evaluate as the
  position held at the preceding rest, and `assert_gap_is_a_hold` verifies
  both sides agree — an axis moving across a gap is a bug and panics.
- Missing history or lookahead is a loud `PostProcessError`, never an
  extrapolation.
- `SetAxisChains` swaps chains at a rest point. Kept history makes the
  resumed track exactly continuous with what was committed; only a *grown*
  back support invalidates retention (then the stream-boundary clamp
  applies).

## The refit

The exact convolved signal is not a polynomial the rest of the system can
carry, so each emitted segment is refit (`fit_axis_from_signal`) with the
ladder the lowerer uses: two endpoint-anchored quadratics that carry the
span's travel, a cubic through both endpoint velocities, a quintic Hermite
base matching the signal's exact `(p, v, a)` at both span ends, then degree
6/7 from interior residuals — bisecting spans until the fit meets the
configured `max_path_deviation` and `max_accel_deviation`. Sliver spans
below `SHAPED_FIT_MIN_SPAN_S` are carried as exact-endpoint linear pieces
instead of dividing by a near-zero duration. Finally
`pad_segment_axes_to_uniform_degree` zero-pads so kinematics lane mixing
sees one degree per segment. Every rung is endpoint-anchored, so seams are
C⁰ by construction; the quadratics solve an endpoint velocity from the
travel, so a seam they win can step in velocity.

A pressure advance applied before projection (`p + k·ṗ` before or without
a follower kernel, or any nonlinear advance) turns such a velocity step
into a position step the extruder would have to take in zero time. The
tracks that advance differentiates — the leaders a projected follower
rides, and the follower's own raw fit when its leaders are unshaped — are
therefore fitted with `velocity_continuous`: the quadratics are withheld
and every seam carries the signal's own velocity.

A follower's fits are held to `max_path_deviation` times its largest demand
ratio (`follower_tol_scale`; the projected source fit takes half of it).
Where an advance applies before projection — any advance placed before a
follower kernel or with none —
the source fit fits the advanced signal itself, and each of its pieces of
duration `h` is held to the acceleration budget
`max(max_accel_deviation, 8ε/h²)` (`AccelBudget::PositionEquivalent`), `ε`
being that position tolerance. Every rung meets the signal's position at
both piece ends, and an error pinned there whose acceleration stays within
`8ε/h²` stays within `ε` in between (`|δ(t)| ≤ max|δ''|·t(h−t)/2`): the
acceleration budget spends the position budget, no more. Where no kernel
follows, the fit is the motor's command, which still holds `[axis e]
max_accel`: a piece whose acceleration the looser budget
would carry past `max_accel + max_accel_deviation` is refit to the absolute
budget, which holds a signal planned within the limit within that (a
ratio ramp released at 150 mm/s on a 20 000 mm/s² extruder took a cubic
72 mm/s² past it). The melt lag `K`
the bead model uses (`e_cmd = e_out + K·ė_out`, `tools/corner_flow.py`)
turns a command error within `ε` into a nozzle-flow error of at most
`2ε/K` — `ε = 0.5 · 0.005 · 0.033 ≈ 83 nm` of filament and `K = 0.02 s`
bound it at 0.5 % of a 50 mm/s line and 0.08 % of a 300 mm/s one. An
absolute 50 mm/s² on that signal, which carries `k·r·jerk` of the shaped
path (~65 000 mm/s² on a 70 000 mm/s² Trident with `k = 0.02`), demanded
pieces no deposition resolves. A linear advance commutes with the kernel,
but applied after the source fit it would turn the fit's velocity error
`δ'` into `k·δ'` of command position, and the melt lag that advance models
undoes it: `δ'` reaches the nozzle flow whole, a bead error of `δ'/(r·v)`
that grows as a corner slows. Under the absolute budget that left a 3–4
point corner-error floor on every kernel chain at
`square_corner_velocity: 5` (`smooth_time: 0.002` on a 3 000 mm/s² bowden
with `k = 0.5` and a 5 000 mm/s² direct drive with `k = 0.05`: 4.2 % and
5.1 % against the kernel's own 0.9 % and 1.6 %); fitted as an advanced
source, kernel chains match an exact offline convolution of the same shaped
path within 0.26 points. Only derivative gains placed after a kernel are
still applied to the plain projection's fit, the kernel's input, where an
acceleration gain must be. Every other follower fit keeps the absolute
budget; spending the position budget on a kernel's output moved the corner
bead by up to 0.15 points at 50 mm/s. Leaders, and the follower's weld,
edge-break and floor checks, keep the absolute budget; the fit audit holds
every piece to the budget its fit resolved for it.

Every rung that passes the probes is proven between them. The proof is the
Bernstein hull of the rung's error against the signal's exact polynomials,
or against its Taylor-model enclosure where it has none (the followers, the
advanced followers and the cap-contact sources). A signal with neither
fails loudly with `PostProcessError::UnprovableSpan`: a formula without a
model is a gap to close, not a span to estimate.

Leader refits use ordered parallel work without changing the fitted signal.
Axes run concurrently. Ordinary windows distribute whole target segments;
a target with at least 512 exact cut spans distributes those spans instead.
If the parallel spans exceed the global refinement budget, that target is
rerun serially so failure behavior stays deterministic.

Follower projection may evaluate a leader through cached local Bezier power
polynomials. Every cache candidate piece is checked against the original NURBS
at its far Gauss node, where the power basis's conversion error has grown the
most, before use. An ill-conditioned conversion falls back to direct
NURBS evaluation; it must never inject false velocity or acceleration into the
follower source.

## The corner budget holds the shaped path

Kernels round corners: convolving a corner with a kernel of variance `σ²`
pulls the path inward by up to `≈ ½·σ²·a`, `a` the acceleration the corner is
crossed with. The corner budget is held on the path after that pull, not on
the planned path (`docs/Config_Reference_Motion.md`, "Corner budget on the
shaped path"). `geometry::ShapedCorners` predicts a corner's shaped miss.
It reads a table built once per X/Y kernel pair by convolving a canonical
corner (a symmetric clothoid blend crossed at a share of its curvature
ceiling under the acceleration disk), worst over its orientation, with
lengths in `A·σ²` so one table serves every acceleration. The engine audit
(`scripts/corpus.sh --path-accuracy`) checks the prediction end to end.

- The fitter cuts each line-line corner by the amount that lets the shaped
  apex be crossed fastest within the budget, at the move's `max_accel`
  (`junction_deviation` in `rust/geometry/src/fitter.rs`). Arc runs and
  collinear merges are fitted to half the budget.
- The planner (`rust/geometry/src/velocity/corner_caps.rs`) reads each
  corner's planned acceleration after the extruder and step-capacity caps.
  It caps the speed of the blend halves at the apex speed the prediction
  allows, and caps each curve's speed so the shaper's pull stays in the
  budget's room. Where a stop at the corner would round it past an explicit
  budget, it slows the moves that reach the corner. The planner splits the
  lines either side of such a corner (`motion-pipeline/src/planner/approach.rs`),
  so only a short approach slows. Each committed move carries its hold
  baked into its limits (`MoveVelocity::corner_held`), so a turning-guard
  re-plan keeps it without the corner's context.

The implicit budgets (no corner option, or `square_corner_velocity`) start
at each corner's own floor at its planned acceleration. They never slow a
corner below the acceleration the planner gives it anyway, so the fitted
rounding no longer collapses toward a stop as `max_accel` grows, which
sank the earlier deduction at the acceleration cap.

Known open items:

- the planner does not yet fold the `k2·jerk` motor-acceleration demand of
  `mode_inverse` into its accel limits (TODO at the top of
  `rust/geometry/src/frontend.rs`) — at high jerk settings the motor command
  can demand more accel than `max_accel`;
- the prediction assumes a symmetric blend braked and left at the full disk;
  an asymmetric blend or a slower approach is covered by its margin only.

## Observing it

Snapshots carry the motor-command trajectory (`traj_*_pieces`) and, whenever
a chain has motor-side gains, the commanded toolhead signal
(`toolhead_*_pieces`, omitted otherwise so kernel-only baselines are
unchanged).

For visualization, the snapshot viewer and the playground
(`snapshots/web/`) go one step further than the commanded signal: they
**simulate the physical toolhead**. Given per-axis `(f, ζ)` fields, the
motor command is run through the resonant mode
`ẍ + 2ζω·ẋ + ω²·x = ω²·u` (exact per-step LTI propagation in
`trajectory-view.js`), and the dotted teal toolhead lanes and path show the
simulated response. That closes the loop on everything above:

- with a plain smoother and the sim tuned to the machine's resonance you
  see the residual vibration the kernel leaves behind;
- switch the sim frequency away from the kernel's and you see what a
  mistuned shaper does;
- turn `mode_inverse` on at the sim's exact `(f, ζ)` and the simulated
  toolhead collapses onto the kernel output — the counter-drive and damping
  feedforward cancel the plant by construction.

Watch a square at high accel while flipping `mode_inverse` on and off and
the counter-drive, the feedforward, and the corner budget all become
visible.

## Classic-Klipper compatibility

`SET_PRESSURE_ADVANCE` (what the Mainsail/Fluidd UI sends) is served by
`klippy/extras/pressure_advance_compat.py`, loaded for every config. Per
extruder it maps `ADVANCE=` onto the `k` of the single
`linear_pressure_advance` post-processor on that extruder's axis and
`SMOOTH_TIME=` onto the `smooth_time` of the single `smooth_triangle` one;
the extruder's `get_status` reports the live engine values back as
`pressure_advance` / `smooth_time`, which is where the frontends read them
from. If the extruder's axis carries no such post-processor the command
answers that the knob is disabled instead of erroring, and the status
fields are omitted. An ambiguous chain (two processors of the same type on
one axis) also reports as disabled; disambiguate with `post_processor:` /
`smooth_post_processor:` in an explicit `[pressure_advance_compat]`
section, or drive the chain directly with `SET_POST_PROCESSOR`.
