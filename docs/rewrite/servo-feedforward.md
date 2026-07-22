# Servo feedforward + identification

> See also the [serval-dashboard](https://github.com/dderg/serval-dashboard)
> repository for the full `SERVO_*` calibration command and script reference.

## The dynamics model

The machine's inertia and friction live on the **Cartesian axes** — the moved
mass, the linear rails, the gantry — not on the individual motors. The model
therefore works in *mode space*: a constant frame matrix `F` (n_modes ×
n_slots) maps the per-motor commanded kinematics the endpoint already receives
into Cartesian mode kinematics, one scalar mass/viscous/coulomb per mode, and
the resulting mode forces project back to motor torques through `Fᵀ`:

```
ν  = F·ω          (mode velocities from slot velocities; same for accel)
τ  = Fᵀ·( m∘(F·α) + b∘ν + c∘sign(ν) )
```

| Symbol | Meaning | Units |
|--------|---------|-------|
| τ | torque feedforward per motor slot | 0.1% of rated (60B2h native) |
| α, ω | commanded accel/velocity per motor stream | mm/s², mm/s |
| F | frame matrix, modes × slots (constant) | dimensionless |
| mₖ | mode mass, > 0 | (0.1% rated) / (mm/s²) |
| bₖ | mode viscous friction, ≥ 0 | (0.1% rated) / (mm/s) |
| cₖ | mode Coulomb friction, ≥ 0, **symmetric** — no fwd/rev split | 0.1% rated |

`sign(ν)` is strict (+1, −1, or exactly 0). There is no deadband: the FF is
evaluated on *commanded* velocity, which is exactly zero at rest, so the
Coulomb term engages on the first commanded cycle (breakaway) and never
chatters.

On an AWD belt, an optional per-pair `direction_split` redistributes the
mode-model torque between two slots without changing their common-mode torque.
For a pair whose frame columns satisfy `F[:,second] = λ F[:,first]`, with
`λ = +1` for equal columns and `λ = -1` for opposite columns, the corrected
differential is

```
D = τ_first - λ·τ_second = 2·w·|τ_base,first|
```

where `w` is the signed `direction_split`. The endpoint adds half of `D` to
the first slot and subtracts `λ·D/2` from the second. The sign convention is
only the order in `slots`. Swapping the slots requires `w′ = -λ·w`: equal
columns (`λ = +1`) negate the coefficient, while opposite columns (`λ = -1`)
preserve it. There is no global split, motor-orientation scalar, or metadata;
`λ` follows directly from the equal/opposite frame columns. The bound `abs(w)
< 0.5` keeps the redistribution below the pair's base share.

Velocity feedforward (60B1h) is `ωᵢ · counts_per_mm`, where `counts_per_mm =
encoder_counts_per_rev / rotation_distance` and the result is in counts/s.

**F is constant.** No configuration dependence — evaluation per DC cycle is
two small matrix-vector products. The endpoint receives the model at claim
time alongside `counts_per_mm` and never interprets the kinematics.

**Plain English.** On CoreXY the X carriage and the Y gantry each have their
own moved mass and their own rails; a motor sees some mix of both depending on
the move direction. The frame rows are exactly that bookkeeping: the x row
says how each motor's motion adds up to X-carriage motion, the y row likewise.
Fit one mass, one viscous drag, and one stiction level per Cartesian axis,
and `Fᵀ` hands every motor its correct share — including the *holding* torque
a stationary belt must supply on a pure diagonal move, which no per-motor
friction term can express. A buzz excitation on any slot contaminates the
mode velocities, so an active buzz suppresses both the Coulomb term and every
pair direction-split correction on the whole node for its duration.

## Configuration (`[servo_x]`)

```ini
[servo_x]
protocol: ethercat
node: node_x
rotation_distance: 40           # mm/rev — counts_per_mm = encoder_counts_per_rev / this
encoder_counts_per_rev: 131072  # A6-EC: 131072

#velocity_ff: True              # stream 60B1h velocity feedforward (kinematic, no profile)
#dynamics_profile: dynamics_x.toml  # path to profile TOML; enables 60B2h torque FF
#ff_max_torque: 30.0          # torque-offset ceiling, % of rated (0, 400], default 30.0
#invert_direction: True         # reverse the drive (default False)
```

`invert_direction` (bool, default `False`): reverses the drive's motion
direction. The sign is applied coherently to the target position, the 60B1h
velocity offset, and the 60B2h torque offset, so feedforward keeps pushing the
right way after the flip — unlike the drive-side CiA-402 polarity object
(`0x607E`), which leaves torque unflipped. The raw inertia capture is left in
the drive's native frame; the fit is sign-invariant, so an inverted drive needs
no special dynamics profile.

`velocity_ff` (bool, default `False`): when set, the endpoint streams the
computed motor velocity as a 60B1h velocity offset each DC cycle. Purely
kinematic — no fitted profile needed. Wrong values degrade tracking, nothing
else; following error before/after on identical strokes is the metric.
Bring-up writes the drive's FF percentage registers to 0, not 100% — the
A6-EC applies communication FF at (100% + C01.14/C01.17); see
[`ethercat-bench-bringup.md`](ethercat-bench-bringup.md).

`dynamics_profile` (path, default none): path to a dynamics profile TOML (see
below). When present, enables 60B2h torque feedforward. Without it the torque
offset is always 0, bit-identical to pre-FF behavior.

The option can sit in two places, and they are mutually exclusive per node:

- On each motor — every motor on the node points at its own single-mode
  profile (identity frame). The host stacks them into a block-diagonal node
  model: each motor's torque feedforward depends only on its own kinematics.
  This is the cartesian case, where the axes are independent. All motors on
  the node must carry one, or none — a partial set fails the claim.
- On `[ethercat_node]` — one combined profile for the whole node, whose frame
  rows express the cross-axis coupling (CoreXY, where a Cartesian mode mixes
  every motor and every motor serves both modes).

Setting it in both places at once is a config error.

`ff_max_torque` (float, default `30.0`, range (0, 400]): ceiling applied to
the raw computed torque offset before it is written to 60B2h, in % of rated
torque. The endpoint counts every clamped cycle and reports the cumulative
count in each `StatusHeartbeat` (`ff_saturation_count`). Saturation during
aggressive tuning is expected; persistent saturation on a steady print is a
miscalibrated profile.

On a coupled node (node-level `dynamics_profile`) every motor's torque FF
mixes all motors' commanded kinematics, so asymmetry in the FF path skews the
shared model instead of tuning one motor. The per-motor FF options —
`velocity_ff`, `ff_max_torque` (the `COUPLED_UNIFORM_OPTIONS` list in
`ethercat_node.py`) — must therefore be identical across the node's motors;
a mismatch is a config error.

## Dynamics profile TOML

Generated by `servo-ident`, version-controlled alongside the printer config.
The endpoint validates it at startup — any violation fails the claim with a
message naming the file and the violated invariant.

```toml
# generated by servo-ident; units: torque in 0.1% rated, motion in mm
version = 6
axes = ["motor_a", "motor_a1", "motor_b", "motor_b1"]  # slot names; order fixes frame columns
modes = ["x", "y"]                                     # mode names; order fixes frame rows
frame = [[0.25, -0.25, -0.25, -0.25],                  # F, modes × slots — built from the
         [0.25, -0.25, 0.25, 0.25]]                    # kinematics' belt signs, never by hand
mass = [0.0123, 0.0119]      # m per mode, (0.1% rated)/(mm/s²)
viscous = [0.09, 0.11]       # b per mode
coulomb = [160.0, 175.0]     # c per mode, symmetric magnitude
compliance = [0.0, 1.76e-5]  # optional (version 7); 1/ω_b² per mode, s²
fit_rms_residual = [0.8, 0.7, 0.8, 0.9]  # per motor, 0.1% rated — fit quality, informational
ff_lead_us = 0.0             # optional; dead-time compensation, microseconds [0, 10000], default 0.0

[[pair]]                                  # optional; one record per AWD belt
slots = ["motor_a", "motor_a1"]           # order defines the coefficient sign
direction_split = -0.125                  # signed, finite, abs(value) < 0.5
```

`compliance` (per-mode, s², version 7 only): the belt-compliance
correction `1/ω_b²`, where `ω_b = 2π·f_b` is the **locked-rotor** belt
frequency of that mode — the frequency the carriage rings at when the
rotor does not move. It is *not* the coupled frequency a plain ringdown
measures (there the rotor recoils on the position-loop spring in series
with the belt, which reads low); using the raw coupled frequency
over-corrects.

With a nonzero compliance the endpoint inverts the two-mass plant per DC
cycle: the load obeys `m·ẍ_L = k_b(x_m − x_L)`, so the rotor trajectory
that makes the carriage follow the commanded curve exactly is

```
x_m = x + a/ω_b²        (position target lead)
v_m = v + j/ω_b²        (60B1h velocity offset)
a_m = a + s/ω_b²        (accel the 60B2h torque model sees)
```

where j and s are the trajectory's analytic jerk and snap (evaluated from
the streamed Chebyshev pieces — never finite-differenced). The rotor
deliberately leads the command during acceleration by exactly the belt
stretch the accel will consume, so the belt force stays the smooth `m·a`
and the carriage never rings *from commanded motion*. Residual excitation
the command didn't cause (cogging, friction reversals, model error) still
rings at the old coupled frequency — keep a light input shaper or the
belt damper for that. On a coupled node the per-mode terms compose
through the frame: the endpoint applies `G = F⁺·diag(compliance)·F` in
slot space, so per-axis f_b values map correctly onto CoreXY motors.

The position lead lives in the same transient offset channel as the trim
and strain compensation — it is never baked into the streamed target
anchor, and it is exactly zero at constant velocity and at rest. It is
bounded by `max_accel/ω_b²` (tens of µm), and the snap term through the
torque path is clamped by `ff_max_torque` like every other torque FF
contribution. The correction needs accel-smooth trajectories: with a
`smooth_bell`/`smooth_mzv` shaper kernel the snap term is bounded and
small; with raw trapezoids the jerk impulses would step the target.
A buzz excitation suppresses the correction for its duration (a buzz has
no ring piece behind it, so its jerk/snap are undefined).

`ff_lead_us` (float, default `0.0`, range `[0, 10000]`): dead-time
compensation for the feedforward path, in microseconds. The 60B1h/60B2h
offsets are sampled this far ahead of the position target, so the torque
they command lands when the trajectory demands it instead of one
command-to-torque latency later. The position target itself is untouched.
The value is continuous — the endpoint peeks the commanded curve at an
arbitrary future nanosecond, so it is not quantized to whole DC cycles —
and `SERVO_TUNE_DYNAMICS TERMS=LEAD` finds it empirically (or measure it
by cross-correlating `torque_actual` against `torque_offset` in a
tracking capture); leading past the true latency flips the sign of the
edge error. Lookahead past the end of the streamed trajectory, or into a
dwell gap, reads as a stationary target (zero FF — the same zero-order
hold the un-led path converges to). A profile may carry only
`ff_lead_us` with zero `viscous`/`coulomb` (`mass` stays positive) — a
timing-only profile that compensates dead time without contributing any
meaningful torque feedforward.

Validation rules (any failure = hard claim error):
- `version` must equal 6 or 7 — older profiles are not supported; refit
  with `SERVO_FIT_DYNAMICS`. `compliance` requires version 7.
- `axes` must contain unique, non-empty strings
- `frame` must be `n_modes × n_slots` with `n_slots = len(axes)`,
  `n_modes = len(modes)`, `1 ≤ n_modes ≤ n_slots`, every row nonzero, rows
  linearly independent
- `mass` entries > 0; `viscous` and `coulomb` entries ≥ 0; all values finite
- `compliance` entries, when present, must be finite, ≥ 0, and at most
  `6.4e-4` s² (a mode softer than 20 Hz is a typo, not a belt)
- each optional `pair` names two distinct, otherwise-unused `axes` entries
  with exactly equal or opposite frame columns; `direction_split` is finite
  and has absolute value below `0.5`
- `n_slots` must equal the endpoint's slot count
- `ff_lead_us`, when present, must be finite and within `[0, 10000]`

## Identification workflow

The whole loop is driven from the console by the `SERVO_*` commands the
`[servo_calibration]` extension registers (add a bare `[servo_calibration]`
to `printer.cfg`; the motor datasheet values, safe stroke window, drive
names, and excitation grid go in that section as overridable defaults). That
extension and the host-side fitter live in the
[serval-dashboard](https://github.com/dderg/serval-dashboard) repository;
install it and build the fitter per its README before running the workflow.

### Step 1 — excite, capture, and fit in one command

```
SERVO_FIT_DYNAMICS TORQUE_NM=1.27 INERTIA_KGM2=0.000057 ROT_DIST=40
```

This homes, runs the `SERVO_MEASURE_INERTIA` excitation grid (defaults:
ACCELS `5000,10000,20000` × SPEEDS `100,400`, 3 iterations each, constant-
acceleration triangle strokes), captures per-DC-cycle PDO data into the run
directory, and hands the capture to `servo-cal fit`, which reads the `.scap`
directly. The profile is written to
`~/printer_data/config/servo_dynamics/dynamics_<name>_<timestamp>.toml` — a
new fit never replaces an existing profile; switching is an explicit config
edit. `TORQUE_NM`/`INERTIA_KGM2`/`ROT_DIST` are optional; when given, the fit
also prints the recommended drive load-inertia ratio C00.06.

After fitting the shared mass/viscous/coulomb model, AWD identification finds
groups of exactly two equal or opposite frame columns and fits each pair's
signed differential against `2·|τ_base,first|`. Pair order follows frame/slot
order. A group larger than two is ambiguous and fails instead of guessing;
zero and unmatched columns are simply not pairs. During `DIRECTION_SPLIT`
refinement, when the current AWD kinematic layout identifies two slots as a
pair, unequal parallel columns fail instead of being silently paired. No motor
direction signs are passed to the fitter.

The stroke engine refuses any (speed, accel) pair where `v²/a` exceeds the
stroke span — that combination cannot reach the target speed within the
available travel and would not produce the intended excitation.

### Step 2 — point the config at the profile

```ini
[servo_x]
...
velocity_ff: True
dynamics_profile: ~/printer_data/config/servo_dynamics/dynamics_ident_<timestamp>.toml
```

Restart klippy. The endpoint loads and validates the profile at claim time.

### Step 2½ — optional empirical tuning

The regression fit can vary with the excitation grid's speeds and
accelerations. `SERVO_TUNE_DYNAMICS` (see the
[serval-dashboard](https://github.com/dderg/serval-dashboard) repository)
tunes the profile empirically: a coordinate descent that streams trial
models into the running endpoint (no restart), captures one XY pattern run
per round, and scores each mode by the transient-window rms of its
following error — the excursion right after each commanded transition,
where feedforward has authority before the inner servo loop corrects it.
Mass/viscous/coulomb tune as 1-D line searches (the mass probe's first
direction follows the onset bias), `TERMS=LEAD` tunes the shared
feedforward lead time on the decel-to-stop windows, and
`TERMS=DIRECTION_SPLIT` searches a signed additive delta per pair and can
therefore augment an older v6 profile with no pair tables or refine a zero
coefficient. The best model is written as a new TOML and left live until
restart — repoint `dynamics_profile` at it to keep it.

### Step 3 — validate tracking

```
SERVO_MEASURE_TRACKING SPEED=400 ACCEL=20000
```

Prints per-move following-error peak/rms, overshoot, settle time, and the
peak FF offsets seen during motion (the velocity figure should match the
commanded speed). With velocity FF at true unity the cruise following error
collapses to encoder noise — bench reference: 0.017 mm rms at 400 mm/s,
versus v/Kp ≈ 1.8 mm without FF.

### Offline / manual path

The capture-conversion and fitting pieces also run standalone, host-side.
Run the excitation on the printer first (`SERVO_MEASURE_INERTIA`, or the
full `SERVO_FIT_DYNAMICS` from Step 1), which leaves a `.scap` capture
behind; then:

```sh
python3 scripts/servo_capture.py run.scap --csv run.csv
servo-ident \
    --capture run.csv \
    --frame "1" \
    --modes x \
    --axes x \
    --out dynamics_x.toml \
    --rated-torque-nm 1.27 \
    --rotor-inertia-kgm2 0.000057 \
    --rotation-distance-mm 40
```

The CSV export derives `t` from the cycle index and `cycle_ns`, emits the
`accel_cmd`/`vel_cmd` capture channels as the kinematics columns, and
`torque_actual` as-is (already 0.1% rated). Only motion-active cycles are
written.

**Capture CSV contract.** The fitter reads a CSV with header columns:
- `t` — time in seconds
- `accel_<axis>` — commanded acceleration in mm/s² (one column per axis)
- `vel_<axis>` — commanded velocity in mm/s (one column per axis)
- `torque_<axis>` — measured torque from 6077h in 0.1% rated (one column per axis)

`accel_<axis>`/`vel_<axis>` are the planner's exact analytic kinematics for each
cycle (the `accel_cmd`/`vel_cmd` channels), not derivatives of the measured
trajectory — noise-free and independent of the drive's gains and inertia ratio.
The fitter restricts the regression to steady constant-acceleration plateaus,
where the closed loop has caught up to the command, so the jerk transitions —
where actual motion lags command and the soft-loop "negative inertia" artifact
lives — never bias the fit.

`--frame` is the mode matrix, rows `;`-separated and entries `,`-separated —
`"1"` for a single Cartesian servo, `"0.5,0.5;0.5,-0.5"` for a coupled A/B
pair (with `--modes x,y --axes a,b` and both motors' columns in the CSV). The
`SERVO_FIT_DYNAMICS` command builds it from the kinematics' slot order and
invert flags; write it by hand only for offline experiments.

`servo-ident` exits with code **2** and a reason when the data cannot support a
fit:
- `SaturatedTorque` — too many samples near the 6077h saturation limit (lower
  acceleration or check drive current limits)
- `InsufficientExcitation` — condition number of the regression matrix is too
  high (need more acceleration range; check that strokes actually ran)
- `ResidualTooLarge` — fit residual exceeds the sanity bound (slow the
  excitation, then re-capture)
- no steady-accel plateaus — every stroke's constant-acceleration phase was
  shorter than the settle window; lengthen strokes or lower acceleration

On success it prints fit diagnostics and, when the three physical parameters
are given, the recommended **drive load-inertia ratio C00.06** (light-direction
value on CoreXY — the stability-critical case):

```
fit: 4800 samples/motor, rms residual 0.72 (0.1% rated), condition 2.3e+03
recommended C00.06 (light direction): 142%
```

Cross-check against the drive's F30.10 auto-tune on a Cartesian axis to
validate the whole pipeline before using it on a CoreXY where F30.10 has no
ground truth.

## Runtime behavior

FF is computed each DC cycle from the same cubic Bézier pieces the endpoint
already walks for position. When no pieces are available (ring empty), both
offsets are written as 0.

**Non-finite FF output** is a latched fault: the endpoint emits a fault-state
`StatusHeartbeat`, disables the drive, and exits. This is a bug in the profile
or the fit, not a tuning artifact.

**Torque clamp saturation** increments `ff_saturation_count` in every
`StatusHeartbeat`. The counter is cumulative for the session. Sustained
saturation on a healthy print with a converged profile means the profile
underestimates the inertia — re-run identification.

**Both offsets are zero when no profile/flag is set.** Behavior is
bit-identical to pre-FF operation.

## Rollout ladder

1. **PDO remap soak** — both offsets hardwired to 0; confirms the new PDO
   layout and SDO writes (FF routing) do not disturb tracking.
2. **Velocity FF** — `velocity_ff: True`, no profile. Following-error
   before/after on identical strokes is the metric.
3. **Identification** — run on the Cartesian bench axis; cross-check C00.06
   vs F30.10 auto-tune.
4. **Torque FF** — add `dynamics_profile`; compare following error.
5. **Trident CoreXY** — own future design: multi-slave support + shadow axes
   (the off-diagonal M entries earn their keep here).
