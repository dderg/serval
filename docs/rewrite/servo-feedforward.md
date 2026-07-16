# Servo feedforward + identification

> See also:
> [`servo-calibration.md`](servo-calibration.md) for the full `SERVO_*` command
> and script reference.

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

**Pair load-share differential.** On a dual-motor-per-belt (AWD) machine the
two motors of a belt do not share reaction forces 50/50: the split follows
the belt span stiffnesses, which vary with carriage position. Left
unmodeled, the position loops manufacture the difference from following
error (one mate overshoots per direction, the other lands clean). Pure
span-stiffness geometry is force-agnostic — the spans transmit whatever
force the carriage needs — so the split is ONE shared `w(p) = w0 + w1·p`
for all force components. The profile carries those two coefficients per
belt pair (`w1` per mm of belt coordinate) and the endpoint adds,
antisymmetrically within the pair,

```
D̂ = (w0 + w1·p_belt) · F_belt        Δτ = ±D̂/2 in belt frame
```

where `F_belt` is the belt's total force (inertial + viscous + coulomb)
from the mode model and `p_belt` the first mate's commanded belt position.
The pair's total torque is unchanged by construction — the term only
redistributes it, including on moves where the belt is loaded but
stationary (diagonals). Like the Coulomb term, the differential is
suppressed for the whole node while a buzz is active. The coefficients come
from the fit's differential regression; its even (|F|-shaped,
role-dependent) components are reported as diagnostics — a large one means
check belt tension or pulley drag — and never fed forward.

Per-component split structure (a different `w(p)` for inertial vs viscous
vs coulomb) has no physical mechanism and can be faked by V/C column
collinearity, role leakage, or residual strain, so it is never fed forward
either. The fit still runs the free six-coefficient per-component
regression as a diagnostic and, given ≥2 capture windows, compares the two
by leave-one-window-out held-out prediction. If the free fit predicts
unseen windows ≥5% better than shared `w(p)`, the report warns that the
rank-1 constraint may be discarding real structure — that is a prompt to
investigate the mechanics (or the capture quality), not a switch the
profile can flip.

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
mode velocities, so an active buzz suppresses the Coulomb term for the whole
node for its duration.

## Configuration (`[servo_x]`)

```ini
[servo_x]
protocol: ethercat
node: node_x
rotation_distance: 40           # mm/rev — counts_per_mm = encoder_counts_per_rev / this
encoder_counts_per_rev: 131072  # A6-EC: 131072

#velocity_ff: True              # stream 60B1h velocity feedforward (kinematic, no profile)
#dynamics_profile: dynamics_x.toml  # path to profile TOML; enables 60B2h torque FF
#ff_torque_clamp: 30.0          # torque-offset clamp, % of rated (0, 400], default 30.0
#ff_lead_cycles: 0              # sample FF offsets this many DC cycles ahead, [0, 40]
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

`ff_torque_clamp` (float, default `30.0`, range (0, 400]): clamp applied to
the raw computed torque offset before it is written to 60B2h, in % of rated
torque. The endpoint counts every clamped cycle and reports the cumulative
count in each `StatusHeartbeat` (`ff_saturation_count`). Saturation during
aggressive tuning is expected; persistent saturation on a steady print is a
miscalibrated profile.

`ff_lead_cycles` (int, default `0`, range [0, 40]): dead-time compensation for
the feedforward path. The 60B1h/60B2h offsets are sampled this many DC cycles
ahead of the position target, so the torque they command lands when the
trajectory demands it instead of one command-to-torque latency later. The
position target itself is untouched. Measure the latency first (cross-correlate
`torque_actual` against `torque_offset` in a tracking capture), then set the
lead to the transport share of it — leading past the true latency flips the
sign of the edge error. Lookahead past the end of the streamed trajectory, or
into a dwell gap, reads as a stationary target (zero FF), which is the same
thing the un-led path converges to. The option is per-motor because the
latency it compensates is a per-drive property.

On a coupled node (node-level `dynamics_profile`) every motor's torque FF
mixes all motors' commanded kinematics, so asymmetry in the FF path skews the
shared model instead of tuning one motor. The per-motor FF options —
`velocity_ff`, `ff_torque_clamp`, `ff_lead_cycles` (the
`COUPLED_UNIFORM_OPTIONS` list in `ethercat_node.py`) — must therefore be
identical across the node's motors; a mismatch is a config error.

## Dynamics profile TOML

Generated by `servo-ident`, version-controlled alongside the printer config.
The endpoint validates it at startup — any violation fails the claim with a
message naming the file and the violated invariant.

```toml
# generated by servo-ident; units: torque in 0.1% rated, motion in mm
version = 5
axes = ["motor_a", "motor_a1", "motor_b", "motor_b1"]  # slot names; order fixes frame columns
modes = ["x", "y"]                                     # mode names; order fixes frame rows
frame = [[0.25, -0.25, -0.25, -0.25],                  # F, modes × slots — built from the
         [0.25, -0.25, 0.25, 0.25]]                    # kinematics' belt signs, never by hand
mass = [0.0123, 0.0119]      # m per mode, (0.1% rated)/(mm/s²)
viscous = [0.09, 0.11]       # b per mode
coulomb = [160.0, 175.0]     # c per mode, symmetric magnitude
fit_rms_residual = [0.8, 0.7, 0.8, 0.9]  # per motor, 0.1% rated — fit quality, informational

[[pair]]                     # zero or more; only dual-drive belts have them
slots = ["motor_a", "motor_a1"]        # first name = '+' side of the differential
belt_position_split = [0.02, -0.0003]                # [w0, w1 per mm of belt coordinate]
```

Validation rules (any failure = hard claim error):
- `version` must equal 5 — older profiles are not supported; refit with
  `SERVO_FIT_DYNAMICS`
- `frame` must be `n_modes × n_slots` with `n_slots = len(axes)`,
  `n_modes = len(modes)`, `1 ≤ n_modes ≤ n_slots`, every row nonzero, rows
  linearly independent
- `mass` entries > 0; `viscous` and `coulomb` entries ≥ 0; all values finite
- each `pair` must name two distinct `axes` entries whose frame columns are
  exactly parallel (same belt); no slot may appear in two pairs; both
  split values finite
- `n_slots` must equal the endpoint's slot count

## Identification workflow

The whole loop is driven from the console by the `SERVO_*` commands the
`[servo_calibration]` extension registers (add a bare `[servo_calibration]`
to `printer.cfg`; the motor datasheet values, safe stroke window, drive
names, and excitation grid go in that section as overridable defaults). One-
time prerequisite: build the fitter on the host with
`cargo build --release -p servo-ident` (from `rust/`).

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

### Step 2½ — optional empirical refinement

The regression fit can vary with the excitation grid's speeds and
accelerations. `SERVO_REFINE_DYNAMICS` (see
[servo-calibration.md](servo-calibration.md)) refines the loaded profile
empirically: it streams scaled candidate models into the running endpoint,
measures tracking per candidate (overshoot for the mass term, following
error for the viscous term), converges on the best scale by golden-section
search, and writes the winning profile as a new TOML — repoint
`dynamics_profile` and restart to keep it. The live model is always
restored to the baseline when the command finishes.

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
