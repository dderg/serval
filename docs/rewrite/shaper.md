# The shaper: axis chains, smooth kernels, and the motor command

The shaper is the last stage of the streaming motion pipeline
(`setup_pipeline` in `rust/motion-core/src/worker.rs`: fitter → planner →
lowerer → **shaper**). It takes the lowered trajectory — per-axis piecewise
polynomial tracks that already satisfy the velocity/accel/jerk limits and the
clothoid corner blending — and turns it into what each motor should actually
be commanded to do. Everything the classic Klipper stack does after the
trapezoid generator (input shaping, smoothers, pressure advance) lives here,
unified under one model, plus things that stack cannot express.

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
post_processors: pa, st

[post_processor pa]
type: linear_pressure_advance
k: 0.03

[post_processor st]
type: smooth_triangle
smooth_time: 0.02
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
  cheap general smoothing, used e.g. on the extruder.
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
  per-piece one. The chain must also carry a smoothing kernel (either
  side of the advance): the advance follows the commanded rate
  instantly, so at seams where the flow ratio steps a bare advance
  would command a discontinuous extruder position — compilation
  rejects a kernel-less chain.
- `mode_inverse` (`frequency_hz`, `damping_ratio`) —
  `DerivativeGains { k1: 2ζ/ω, k2: 1/ω² }` with `ω = 2πf`.

A kernel's *variance* (second moment, `kernel_variance_s2`) is its one
number the planner needs — see the corner budget section.

## Composition rules

`CompiledChain::compile` enforces the v1 composition contract, failing
loudly at config time rather than producing silently wrong motion:

- at most **one kernel** and **one derivative-gain stage** per axis
  (`UnsupportedComposition`);
- a gain stage with `k2 ≠ 0` must come **after** a kernel
  (`AccelGainNeedsPrecedingKernel`): applied before the kernel it would run
  in the lowerer, whose curved-path sampler carries no jerk and cannot form
  the transformed velocity. After a kernel the track is polynomial and the
  operator is exact algebra on coefficients
  (`apply_derivative_gains_to_track`: differentiate the monomial pieces
  twice, add `k1·ẋ + k2·ẍ` termwise).

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
emitted segment as it stands *before* that stage. The snapshot pipeline uses
the tap to emit optional `toolhead_*_pieces`, which is what the snapshot
viewer and playground plot as the toolhead signal alongside the motor
command. `CompiledChain::has_motor_side_gains` /
`AxisChainSet::has_motor_side_stages` define "motor-side": a gain stage
that follows a kernel.

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
the total extruded amount tracks the shaped path's true length — permanently
short of the commanded total by exactly the corner-cut length. Extrude-only
moves ride no spatial path and add in directly.

Critically, the projection reads the leaders' **toolhead signal**, not their
motor command — the follower must track where the nozzle physically is, and
the mode-inverse counter-drive is motion the toolhead never performs. This
is why `apply_motor_side_stages` runs only after `project_followers`.

The follower's own chain then applies on top of the projection in the same
stage order: leading gains (pressure advance) act on the projected track,
the follower's kernel (e.g. `smooth_triangle`) convolves that, trailing
gains act on the convolution.

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
carry, so each emitted segment is refit (`fit_axis_from_signal`): a quintic
Hermite base matching the signal's exact `(p, v, a)` at both span ends,
escalated to degree 6/7 from interior residuals (the same ladder the lowerer
uses), bisecting spans until the fit meets `SHAPED_FIT_TOL_MM = 1e-3` and
`SHAPED_FIT_TOL_ACCEL_MM_S2 = 50`. Sliver spans below
`SHAPED_FIT_MIN_SPAN_S` are carried as exact-endpoint linear pieces instead
of dividing by a near-zero duration. Finally
`pad_segment_axes_to_uniform_degree` zero-pads so kinematics lane mixing
sees one degree per segment. Matching endpoint `(p, v, a)` per span keeps
the emitted track C² at every seam by construction.

## Corner deviation and kernel smoothing are separate knobs

Kernels round corners: convolving a corner traversed at accel `a` with a
kernel of variance `σ²` pulls the path inward by `≈ ½·σ²·a`. That smoothing
is deliberately **not** deducted from the fitter's corner budget: the
clothoid fitter always spends the full `corner_deviation` on blend
geometry (`junction_deviation` in `rust/geometry/src/fitter.rs`), so the
fitted path is identical whatever kernels are active and whatever the
acceleration limit is. An earlier design subtracted the kernel's
worst-case share (predicted at the accel *cap*), which made the fitted
rounding shrink — down to a full-stop sharp corner — as the accel limit
grew, because the corner never actually rides the cap. The
`snapshots/cases/corner_accel/` cases pin the accel-invariance that
replaced it. The kernel's inward pull rides on top of the blend and
scales with the acceleration actually ridden through the corner; it is
the shaper's own quality/smoothing tradeoff, tuned by kernel duration.

Known open items:

- the planner does not yet fold the `k2·jerk` motor-acceleration demand of
  `mode_inverse` into its accel limits (TODO at the top of
  `rust/geometry/src/frontend.rs`) — at high jerk settings the motor command
  can demand more accel than `max_accel`;
- when the kernel share exceeds the whole SCV budget the collision behavior
  is still to be decided;
- the per-junction deduction uses the worst-case σ², which is blunt for
  dense facets.

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
extruder it maps `ADVANCE=` onto the linear coefficient of the single
advance-family post-processor on that extruder's axis
(`linear_pressure_advance`'s `k`, or a `tanh_pressure_advance` /
`recipr_pressure_advance`'s `linear_advance`), `SMOOTH_TIME=` onto the
`smooth_time` of the single `smooth_triangle` one, and — matching
bleeding-edge-v2's command surface — `OFFSET=` / `VELOCITY=` onto a
nonlinear processor's `nonlinear_offset` / `linearization_velocity`;
the extruder's `get_status` reports the live engine values back as
`pressure_advance` / `nonlinear_offset` / `linearization_velocity` /
`smooth_time`, which is where the frontends read them from. If the
extruder's axis carries no such post-processor the command answers that
the knob is disabled instead of erroring, and the status fields are
omitted (`OFFSET`/`VELOCITY` report disabled on a linear-only axis). An
ambiguous chain (two advance-family processors on one axis) also reports
as disabled; disambiguate with `post_processor:` /
`smooth_post_processor:` in an explicit `[pressure_advance_compat]`
section, or drive the chain directly with `SET_POST_PROCESSOR`.

This makes the classic tower workflow apply to the nonlinear model
parameter by parameter, exactly as on bleeding-edge-v2:

```
TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=ADVANCE START=0 STEP_DELTA=0.002 STEP_HEIGHT=5
TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=OFFSET START=0 STEP_DELTA=0.02 STEP_HEIGHT=5
TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=VELOCITY START=2 STEP_DELTA=1 STEP_HEIGHT=5
```

(`VELOCITY` must stay positive — start its tower above zero.)

## PA tower test print

`klippy/extras/pa_test.py` (ported from bleeding-edge-v2) prints the PA
calibration tower directly from the firmware: `[pa_test]` +
`[virtual_sdcard]` in the config, then `PRINT_PA_TOWER NOZZLE=...
TARGET_TEMP=...`. Unlike upstream's gcode-provider plumbing, our port
writes the generated tower to `pa_tower.gcode` in the virtual-sdcard
directory and starts it through the regular `SDCARD_PRINT_FILE` path, so
`PAUSE`/`RESUME`/`CANCEL_PRINT` and progress reporting work unchanged.
The extruder must already be heated: `TARGET_TEMP` only cross-checks the
configured target.

The tower's "SCV notch" — a tiny slow segment on each straight face — is
a corner stand-in: to the E axis a corner is nothing but a dip in
toolhead speed, and putting the dip on a flat wall makes the PA
transient legible. bev2 realizes it with mainline's junction-SCV
semantics (0.2 mm at the configured `square_corner_velocity`); this
planner blends corners as clothoids, so the port re-derives both notch
parameters from an offline sweep of the real pipeline (90° corner vs
straight-line notch, accel 20–100k, jerk 5e6–2e7, deviation 0.02–0.1,
cruise 80–300 mm/s):

- a clothoid corner's minimum speed is `0.863 ×` the formula SCV at the
  effective limits — constant across the whole sweep — so the notch
  speed defaults to `0.863 × square_corner_velocity` from live toolhead
  status (which uses the same effective accel the tower prints under);
  `SCV_VELOCITY=` overrides it verbatim;
- a clothoid only kisses its minimum (0.3–3 ms across the sweep), while
  a fixed 0.2 mm notch can pin it for tens of ms, so the notch length
  defaults to `notch speed × 1 ms`, clamped to [0.02, 0.2] mm (≈2–4 ms
  dwell); `NOTCH_MM=` overrides. The entry/exit ramps need no matching:
  the same tangential jerk-limited planner shapes them on both paths.

Sample config + helper macros, adapted from bev2's Nonlinear Pressure
Advance guide (`TESTPARAM`: 0 = ADVANCE, 1 = OFFSET, 2 = VELOCITY — bev2's
`TIME_OFFSET` parameter has no counterpart in this planner's advance
model; `VELOCITY` towers start at 1 because `linearization_velocity`
must stay positive):

```
[pa_test]
size_x: 100
size_y: 50
height: 50
origin_x: 175    # bed center
origin_y: 175
layer_height: 0.2
first_layer_height: 0.24
perimeters: 2
brim_width: 6
slow_velocity: 20
medium_velocity: 50
fast_velocity: 150
filament_diameter: 1.75
fan_speed: 0.5

[delayed_gcode start_pa_test]
gcode:
    {% set vars = printer["gcode_macro RUN_PA_TEST"] %}
    ; PUT YOUR START GCODE HERE (homing, bed mesh, M190/M109 heating)
    {% set flow_percent = vars.flow_rate|float * 100.0 %}
    {% if flow_percent > 0 %}
        M221 S{flow_percent}
    {% endif %}
    {% set height = printer.configfile.settings.pa_test.height %}
    {% if vars.testparam == 0 %}
        TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=ADVANCE START=0 FACTOR=0.001
    {% elif vars.testparam == 1 %}
        TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=OFFSET START=0 FACTOR=0.01
    {% elif vars.testparam == 2 %}
        TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=VELOCITY START=1 FACTOR={vars.velocity_factor}
    {% endif %}
    ; PRINT_PA_TOWER must be the last command: it starts the print and returns
    PRINT_PA_TOWER {vars.rawparams} FINAL_GCODE_ID=end_pa_test

[delayed_gcode end_pa_test]
gcode:
    ; PUT YOUR END GCODE HERE (heaters off, present print)
    RESTORE_GCODE_STATE NAME=PA_TEST_STATE

[gcode_macro RUN_PA_TEST]
variable_bed_temp: -1
variable_hotend_temp: -1
variable_flow_rate: -1
variable_testparam: 0
variable_velocity_factor: 0.2
variable_rawparams: ''
gcode:
    {% if params.NOZZLE is not defined %}
    {action_raise_error('NOZZLE= parameter must be provided')}
    {% endif %}
    {% if params.TARGET_TEMP is not defined %}
    {action_raise_error('TARGET_TEMP= parameter must be provided')}
    {% endif %}
    {% if params.TESTPARAM is defined and params.TESTPARAM|int not in (0, 1, 2) %}
    {action_raise_error('TESTPARAM must be 0 (ADVANCE), 1 (OFFSET) or 2 (VELOCITY)')}
    {% endif %}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=bed_temp VALUE={params.BED_TEMP|default(60)}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=hotend_temp VALUE={params.TARGET_TEMP}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=flow_rate VALUE={params.FLOW_RATE|default(-1)}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=testparam VALUE={params.TESTPARAM|default(0)}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=velocity_factor VALUE={params.VELOCITY_FACTOR|default(0.2)}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=rawparams VALUE="'{rawparams}'"
    SAVE_GCODE_STATE NAME=PA_TEST_STATE
    UPDATE_DELAYED_GCODE ID=start_pa_test DURATION=0.01
```

bev2's tuning order carries over: tower `TESTPARAM=1` first (pick
`nonlinear_offset` off the left face's slow transitions), then
`TESTPARAM=0` (pick `linear_advance` where left and front faces converge
together, nudging the offset ±10% when they disagree). The tower runs at
the `[printer]` max accel — set it to what you actually print with.
