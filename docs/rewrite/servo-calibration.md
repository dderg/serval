# Servo calibration reference

Command reference for tuning an A6-EC servo axis (EtherCAT). The
`[servo_calibration]` extension registers the `SERVO_*` console commands. Each
experiment command writes a run directory under `captures_root` (a
`manifest.json`, one `step_<name>.scap` per step, optional accelerometer CSVs)
and then invokes the `servo-cal` Rust binary — `analyze` writes `results.json`
with a typed verdict, `fit` writes a dynamics profile. Drive-parameter access
comes from `[servo_param]`/`[servo_capture]`. For the run-directory and
`results.json` schemas see
[servo-cal-contracts.md](servo-cal-contracts.md); for the theory behind the
inertia/feedforward fit see [servo-feedforward.md](servo-feedforward.md); for
the capture format see
[servo-telemetry-capture.md](servo-telemetry-capture.md).

## Enabling

Add a bare section to `printer.cfg` and put the values that do not change
between runs in it:

```ini
[servo_calibration]
servos: motor_a, motor_b        # drives excited by the CoreXY grid (name one for a single-motor axis)
rated_torque_nm: 0.32           # motor datasheet rated torque, N*m
rotor_inertia_kgm2: 0.0000033   # rotor inertia, kg*m^2 (datasheet 1e-4 value x 1e-4)
x_start: 30                     # safe stroke window, mm
x_end: 220
y_start: 30
y_end: 220
accels: 5000, 10000, 20000      # excitation grid, mm/s^2
speeds: 100, 400                # excitation grid, mm/s
iterations: 3
dwell_ms: 700
```

Every option is optional and every command accepts a matching per-run override
(`SERVO_CALIBRATE_GAINS AXIS=Y START=40`). `rated_torque_nm`/
`rotor_inertia_kgm2` have no default — a command that needs them errors unless
they are configured or passed.

| Option | Default | Used by |
|---|---|---|
| `servos` | `stepper_x, stepper_y` | single-drive default for `SERVO=`/`AXIS=`-less commands; on `coupled_xy` kinematics the measure/fit commands derive their drives from the kinematics instead (`SERVOS=` overrides) |
| `rated_torque_nm` | — | inertia-ratio commands (`TORQUE_NM=`) |
| `rotor_inertia_kgm2` | — | inertia-ratio commands (`INERTIA_KGM2=`) |
| `x_start` / `x_end` | `20` / `200` | X strokes (`START`/`END`, `X_START`/`X_END`) |
| `y_start` / `y_end` | `20` / `200` | Y strokes (`Y_START`/`Y_END`) |
| `accels` | `5000, 10000, 20000` | excitation grid (`ACCELS=`) |
| `speeds` | `100, 400` | excitation grid (`SPEEDS=`) |
| `iterations` | `3` | strokes per grid point (`ITERATIONS=`) |
| `dwell_ms` | `700` | settle between strokes (`DWELL_MS=`) |
| `travel_speed` | `100` | CoreXY centering moves between grid points |
| `accel_chip` | — | accelerometer section name (e.g. `adxl345`); when set, `SERVO_CALIBRATE_GAINS` also records vibration per step (`ACCEL_CHIP=`) |
| `captures_root` | `~/printer_data/logs/servo_captures` | parent directory for experiment run directories |
| `journal_params` | — | comma list of drive SDO addresses (`addr[:type]`, e.g. `0x2001.0x31:u16`) read back from every captured drive at run start and recorded under `ambient.journal_params` in the manifest — the campaign's varied registers (notch mode, etc.) |
| `servo_cal_binary` | `rust/target/snapshot/servo-cal` | path to the `servo-cal` analysis binary |

Prerequisites: the EtherCAT servo stack (`[servo_param]`, `[servo_capture]`)
must be configured, and the `servo-cal` binary must be built once on the host
with `cargo build --profile snapshot -p servo-ident` (from `rust/`).

## Tuning order

1. **Enable feedforward** — set `velocity_ff: True` on each `[motor]` so the
   tuning runs measure the loop as it will actually be driven (see
   [servo-feedforward.md](servo-feedforward.md)).
2. **`SERVO_CALIBRATE_INERTIA_RATIO`** — identify the load inertia and
   set the base C00.06, before touching the loop gains. On `coupled_xy`
   kinematics this runs the coupled X+Y grid and fits both belt directions;
   `SERVO_SWEEP_INERTIA` empirically verifies / refines C00.06 later, at the
   tuned gains.
3. **`SERVO_APPLY_GAINS`** then **`SERVO_CALIBRATE_GAINS`** — find the loop
   gains.
4. **`SERVO_FIT_DYNAMICS`** — fit the dynamic profile at the final gains and
   point `dynamics_profile` at it to enable torque feedforward.
5. **`SERVO_REFINE_DYNAMICS`** — empirically refine the fitted profile on the
   running endpoint (mass first, then viscous, then coulomb) when the
   regression fit varies with the excitation grid; point `dynamics_profile`
   at the refined TOML it writes.

**`SERVO_MEASURE_TRACKING`** is the before/after check for any single change.
**`SERVO_AUTOTUNE`** packages this exact order into one command — see
[SERVO_AUTOTUNE](#servo_autotune) below — for a bench that has already built
trust in the verdicts; the manual sequence above remains the way to run any
one step in isolation or to diagnose a step SERVO_AUTOTUNE aborted on.

Every stroke is paced `M400 / G4 / M400` so it replans from idle, and the
stroke engine refuses any `(speed, accel)` pair whose `v²/a` exceeds the stroke
span — that pair cannot reach the target speed within the travel and would not
produce the intended excitation.

## Measurement commands

Every capture reads each drive's EtherCAT sync loss counter (C13.04) before
and after the strokes. The drive silently tolerates up to C13.02 (default 8)
consecutive lost/late sync events before faulting, so a tolerated loss shows
up nowhere except this counter — but it makes the drive's internal position
demand skip and double-step, injecting a following-error transient of
exactly one cycle of travel. When the counter moved during a capture the
command prints a WARNING naming the drives and deltas (and emits a
`calibration/sync_loss` event): that step's tracking metrics are
contaminated and must not be compared or scored. `SERVO_SHOW_TUNING` also
reads C13.02/C13.04 for manual checks.

#### SERVO_MEASURE_TRACKING
Single accel/speed stroke run with capture, then prints per-move following
error, overshoot and settling — the before/after check for any tuning change.
Params: `AXIS` (X) `START` `END` `SPEED` (100) `ACCEL` (3000) `ITERATIONS` (3)
`DWELL_MS` `NAME` (track). Writes a run directory and runs `servo-cal analyze`.

#### SERVO_MEASURE_DIFFERENTIAL
Anti-phase position chirp on one AWD belt pair via the engine-resident buzz
generator: the two drives of the belt are commanded in opposite directions,
so the carriage holds (nominally) still while the drives strain the belt
against each other. The capture therefore isolates the differential
(rotor-vs-rotor) dynamics — the modes excited when paired drives fight —
and the analysis reports each detected mode's frequency, closed-loop peak
gain, half-power damping ratio and coherence, plus a differential FRF
(magnitude, phase, coherence, differential-torque spectrum) the dashboard
renders. Belt strain between the pair is **twice** `AMPLITUDE`; the command
caps `AMPLITUDE` at 0.5 mm. Needs two drives per belt. Params: `BELT` (A)
`FREQ_START` (20) `FREQ_END` (250) `HZ_PER_SEC` (5) `DURATION`
(band/`HZ_PER_SEC`) `AMPLITUDE` (0.05 mm) `RAMP` `DWELL_MS` `NAME` (diff).
Writes a run directory and runs `servo-cal analyze`.

#### SERVO_DIFF_DAMPER
Arms (or disarms) the engine-resident differential belt-pair damper on an
AWD machine. Every EtherCAT cycle the endpoint differentiates the pair's
raw encoder positions, low-passes the differential velocity and streams an
**antisymmetric** torque offset (60B2h) to the pair — a virtual dashpot
connected between the two rotors. Because the torques are equal and
opposite through the belt, the carriage sees no net force, and on
synchronized motion the differential velocity is zero, so the damper costs
no torque during printing. Unlike a notch filter it is frequency-agnostic:
it damps the inter-motor belt mode wherever toolhead position has moved it.
`GAIN` is in units of 0.1% rated torque per mm/s of differential velocity;
`GAIN=0` disarms the belt's damper. The injected torque is clamped to
`CLAMP` (0.1% rated torque, command ceiling 300) and the velocity is
low-passed at `LPF_HZ`. Velocity comes from host-side position
differencing, NOT the drive's 606Ch estimate — the drive's estimator lag
pushes delayed velocity feedback past 90° in the very band being damped,
which pumps the mode instead; `LEAD_US` (first-order lead, microseconds)
compensates the remaining EtherCAT transport and drive torque-path lag and
is tuned empirically: if the A/B sweep shows the peak *sharpening* above
some frequency, add lead until the whole band damps. State lives in the
running endpoint — re-arm after a firmware restart. Verify with an A/B
`SERVO_MEASURE_DIFFERENTIAL` sweep: the pair mode's damping ratio should
rise with the damper on. Params: `BELT` (AB) `GAIN` (required) `CLAMP`
(50) `LPF_HZ` (300) `LEAD_US` (0).

#### SERVO_DIFF_TRIM
Arms (or disarms) the engine-resident differential belt-pair **trim** —
standstill zeroing of the pair fight, config section `[servo_diff_trim]`.
Servo sync and homing leave a run-to-run differential preload between the
two drives of a belt (each enable seeds at a slightly different relax
point), so the same strain-comp map can show a different peak differential
torque on every home+sync cycle. Whenever the pair sits at **commanded
standstill** the endpoint low-passes its mechanical-frame differential
torque and integrates it into a small flat **antisymmetric position
offset** on top of the streamed targets and the strain-comp map: the pair
unwinds against itself while the carriage never moves. During motion the
loop freezes entirely (filter and integrator) — an in-motion differential
torque is legitimate (commanded feedforward, direction- and
toolhead-position-dependent inner-loop load) and must not be nulled. A
slot counts as quiescent only when its piece ring is empty (pieces land at
least the feedforward lead before their start, so an empty ring also
proves no lead-window torque is being commanded), no buzz is running, the
strain-comp ramp has settled, and the pair has been still for `SETTLE_MS`
(torque relax + telemetry lag after a decel). The offset resets on a pair
sync or torque-gate drop — the `SERVO_SYNC` release is the new zero.
`GAIN` is mm/s of offset slew per 1% differential torque — start around
`0.0001` and raise it until the offset converges within your typical
dwells; too much gain (with the loop crossover approaching the LPF
corner) oscillates. `GAIN=0` freezes the loop with the learned offset
held. Retuning any knob updates the running pair in place — the learned
offset and filter state carry over — and `REMOVE=1` drops the pair (and
its offset) entirely. `MAX_OFFSET_UM` (µm, ceiling 500) bounds the
offset — the trim's total authority; hitting it logs a
`diff_trim_clamped` warning: residual fight beyond what the trim may
absorb. Torque LPF at `LPF_HZ` (floor 0.1). Config options (`gain`,
`max_offset_um`, `lpf_hz`, `settle_ms`) arm the trim at startup when
`gain` is non-zero; the command overrides them live for tuning and
`SAVE=1` stages the current values for `SAVE_CONFIG`. Params: `BELT`
(AB) `GAIN` `MAX_OFFSET_UM` (150) `LPF_HZ` (2) `SETTLE_MS` (300)
`REMOVE` (0) `SAVE` (0).

#### SERVO_MEASURE_STRAIN_MAP
The measurement half of the belt strain map (CoreXY only). Rasters the bed
with slow constant-speed strokes — serpentine X sweeps stepped along Y by
`LINE_SPACING`, then Y sweeps stepped along X — recording one capture per
line, each stroked forward and back so the direction-dependent (friction)
half of the differential torque averages out of the analysis. The
per-belt differential pair torque as a function of (x, y) is the raw
material for separating trapped preload (DC), pulley/idler runout
(periodic in travel at each element's circumference — 40 mm motor
pulleys) and geometry/squareness (smooth 2D) — and eventually for the
feedforward strain-compensation map. `LINE_SPACING` must stay under half
the shortest period of interest (10 mm covers the 40 mm and ~13 mm
elements). Before rastering the carriage parks at the region center and
`SERVO_SYNC` releases the trapped preload, so the DC of every map is
measured from the same zero (`SYNC=0` skips, and the manifest records
`zero_sync`); without `[servo_sync]` configured the command errors. The
run directory is charted by the dashboard's strain tab. Params: `SPEED`
(50) `ACCEL` (1000) `LINE_SPACING` (10) `X_START` `X_END` `Y_START`
`Y_END` `DWELL_MS` `TAG` (strain) `SYNC` (1).

#### Strain compensation (SERVO_STRAIN_COMP_TUNE / SERVO_MEASURE_STRAIN_RESPONSE / SERVO_MEASURE_PAIR_STIFFNESS / SERVO_STRAIN_COMP_BUILD / SERVO_STRAIN_COMP_FIT / SERVO_STRAIN_COMP)
The application half of the strain map, config section
`[servo_strain_comp]`. The endpoint carries a per-belt 2D lookup table of
**antisymmetric position offsets** keyed on the commanded carriage
position: every cycle it reconstructs (x, y) from the streamed lane
positions, bilinearly interpolates each belt's grid, and offsets the
pair's two drives by equal and opposite amounts — the rotors absorb the
position-dependent tension variation (belt thickness lumps, pitch
nonuniformity, frame geometry) instead of fighting through the belt,
while the carriage never moves. Offsets ride outside the command anchors
(like the differential trim's), are clamped to ±500 µm (the grid's span
too, since re-anchoring can apply the full span) and slew-limited to
1 mm/s, so enabling, replacing, or clearing a map can never yank the
targets.

**Re-anchoring.** The map's DC follows the mechanics. Whenever torque
drops — SERVO_SYNC, M84, idle timeout — the free rotors relax the pair's
differential strain at wherever the carriage sits, including a hand-move
while unpowered, so the position where torque returns is the new
physical zero. The endpoint re-anchors the map there automatically: it
samples the grid at the re-engage position and applies everything
relative to that value, so a freshly relaxed gantry is never re-racked
and jogging after an idle timeout just works. The accepted limitation:
the map's residual error is then measured from the re-engage position
instead of the calibrated zero, so an anchor at a field extreme can
roughly double the worst-case residual. When accuracy matters — before a
print, before measuring a residual strain map (`MERGE=1`) — run
SERVO_SYNC at the map's zero point to restore the calibrated anchor;
nothing does this for you. The live anchor bias is visible in the
`strain_comp_state` event (`anchor_bias_um`).

**The stiffness is a matrix.** The two belts share the gantry, so an
antisymmetric offset on one pair also strains the other — on the Trident
bench the cross term is ~25% of the direct term, symmetric (reciprocity),
and same-signed with the racking direction. Dividing each belt's field by
its own scalar stiffness therefore copies every single-belt feature into
the other belt at the coupling ratio (the diagonal "ghost" in a
verification map). The build instead solves the 2×2 system per grid node,
`offsets = -inv(K) @ strain`: each belt gets its own correction plus a
partial same-sign helper offset for the other belt's field. A
near-singular matrix (cross terms rivaling the direct terms) fails
loudly.

**Measure the matrix rolling, not parked.** A parked belt reads
stiffer than one rolling over the pulleys and idlers — on the bench
the parked probe (`SERVO_MEASURE_PAIR_STIFFNESS`) reads ~428/−122
while every rolling measurement lands at ~353/−89, consistently across
independent excitations. The map operates while moving, so calibrate
in that regime; the parked probe supplies the starting values (its
SHAPE — the cross/direct ratio — carries over; only the overall scale
is off), and the tune loop converges the scale against reality.

`SERVO_STRAIN_COMP_TUNE RUN=<baseline raster>` is that loop: rebuild
the FULL map from the run at the trial matrix (never merging), enable
it, sweep an X **and** a Y verification line, and refit every belt's
direct and cross stiffness from the measured response to the applied
offsets — the two sweeps swap the belts' roles, so all four matrix
elements are measured independently (a single X line cannot separate
them: the own- and cross-corrections it applies are near-collinear,
and the old row-scale loop silently froze the cross:direct ratio at
whatever was passed in). Repeat until the measured matrix reproduces
the applied one — per element, within `TOL` of the row's direct
stiffness. Each pass costs two line sweeps, not a raster, and one
correction step usually lands it. When it converges the tuned
full-bed map is already on disk and enabled — it is the same map that
was being verified — and the matrix is stored for future builds and
recorded in the map (`stiffness_pct_per_mm`, `cross_pct_per_mm` per
pair). Running out of `MAX_ITERS`, lines the map doesn't vary along,
collinear own/cross corrections, or a measured direct stiffness
outside (0.2, 5)× the applied one all fail loudly. Know what the
numbers cover: the verification metric is the **smooth elastic
field** (forward/backward passes are averaged, so direction-dependent
friction asymmetry cancels out of it, and sub-20 mm ripple is below
the field model's bandwidth) — raw differential measurements keep
both, so they read higher than the tune's residual and no
position-keyed map can close that gap. Open-loop alternates that measure the matrix directly:
`SERVO_MEASURE_STRAIN_RESPONSE` steps a constant antisymmetric offset
through each pair's bank (0, ±STEP_UM, ±2·STEP_UM) while stroking one
line — the line's field cancels out of the offset-response slope, no
map or baseline needed; and `SERVO_STRAIN_COMP_FIT
BASELINE=<uncompensated run> RUN=<compensated run>` regresses the
field change between an existing run pair against the offsets the map
applied. The exact scale is in any case not critical for `MERGE=1`
convergence: a fractional matrix error leaves the same fraction of
the field behind, and each merge pass shrinks it by that factor
again.

The workflow: (1) `SERVO_MEASURE_STRAIN_MAP` rasters the baseline;
`SERVO_MEASURE_PAIR_STIFFNESS` (parked) supplies starting values.
(2) `SERVO_STRAIN_COMP_TUNE RUN=<raster> SPACING=5` converges the
matrix and leaves the tuned map enabled. Its rebuilds are the same
build as `SERVO_STRAIN_COMP_BUILD RUN=<dir>`, which fits each belt's
dense line
samples with a structured field model — 1D components at 2 mm knots
along each belt phase (x+y, x−y; CoreXY only) and along each axis, plus
a smooth 2D remainder — evaluates the model at the output grid nodes,
zeroes the maps at the region center (SERVO_SYNC's zero point), solves
the per-node 2×2 system, and writes `map_file` (default
`~/printer_data/config/strain_comp.json`). The model matters:
point-sampling the raster at grid nodes aliases everything shorter than
twice the node pitch, and the dominant fine structure is belt-phase
diagonal at the 40 mm pulley period — on the bench it left a ~35%
diagonal residue that the model build removes because diagonals stay
diagonal between the raster lines. Pass `SPACING=5` on CoreXY so the
40 mm harmonics also survive the endpoint's bilinear lookup (57×55
stays within the 64/4096 grid caps on a 300 mm bed; the build fails
loudly beyond them).
(3) `SERVO_STRAIN_COMP ENABLE=1` resolves the map's motor names to
slots/lanes on the live topology and uploads it (the tune already
leaves it enabled); `ENABLE=0` ramps the compensation back out. Verify
with a full raster when the whole-bed picture matters — the residual
field should collapse. (4) Fold later residuals straight into the map:
`SERVO_STRAIN_COMP_BUILD RUN=<verification run> MERGE=1` (no stiffness
params needed: the recorded matrix is reused), then `ENABLE=1`.
Params: tune `RUN` (required) `SPACING` `TOL` (0.05) `MAX_ITERS` (5)
`Y` (map zero) `SPEED` (50) `ACCEL` (1000) `SETTLE` (0.8) `DWELL_MS`
`TAG` `SYNC` plus the build's matrix overrides; response `SPEED` (50)
`ACCEL` (1000) `STEP_UM` (50) `SETTLE` (0.8) `Y` (area center)
`X_START`/`X_END` `DWELL_MS` `TAG` `SYNC`;
parked stiffness `STEP_UM` (50) `SETTLE` (0.8) `AXIS`; build `RUN`
(required) `STIFFNESS_A`/`STIFFNESS_B` with `CROSS_AB`/`CROSS_BA`
(%/mm matrix override; `CROSS_AB` is belt A's response to a belt B
offset, 0 disables the cross term) `SPACING` (run's line spacing);
fit `BASELINE` and `RUN` (both required).

#### SERVO_MEASURE_INERTIA
Records the excitation grid for the inertia/friction fit (no report — it is the
capture building block behind the fit commands). The active kinematics decides
the shape of the grid:

- **`coupled_xy` kinematics** (CoreXY): one capture of **every** belt drive
  with X and Y strokes at every grid point (`SERVOS=` overrides; the default
  is every motor the kinematics says drives the belts), so the fit sees each
  Cartesian mode excited on its own (X strokes excite only the x mode, Y
  strokes only the y mode). Before each stroke set the
  toolhead moves (at `travel_speed`) to the active axis' start with the idle
  axis centered in its range, so both belt runs are near-equal length during
  the measurement. Bounds come from `X_START`/`X_END`/`Y_START`/`Y_END`.
- **cartesian kinematics**: captures every motor that moves `AXIS` (every
  drive of an AWD rail), bounded by `START`/`END`. `SERVOS`, `X_START`,
  `X_END`, `Y_START`, `Y_END` only apply to `coupled_xy` kinematics and are
  rejected with an error otherwise.

Params: `AXIS` (X) `START` `END` `X_START` `X_END` `Y_START` `Y_END` `ACCELS`
`SPEEDS` `ITERATIONS` `DWELL_MS` `NAME` (ident) `SERVOS`.

## Fit / inertia-ratio commands

#### SERVO_FIT_DYNAMICS
Runs the `SERVO_MEASURE_INERTIA` grid, fits one mass/viscous/coulomb triple
per Cartesian mode (see the model in
[servo-feedforward.md](servo-feedforward.md)), and writes a timestamped
feedforward profile. Optional `TORQUE_NM` + `INERTIA_KGM2` also print the
recommended C00.06. The active kinematics decides the frame the fit and the
profile share:

- **`coupled_xy` kinematics**: runs the X+Y grid over every belt drive and
  fits the x and y modes through the frame matrix built from the kinematics'
  slot order and invert flags (passed to the fitter as `--frame`/`--modes`;
  on AWD each belt's drives share its columns and all four drives must sit
  on one node). The resulting profile goes on
  `[ethercat_node] dynamics_profile` (node-level, coupled) rather than
  per-motor.
- **cartesian kinematics**: fits a single mode with an identity frame. On a
  multi-drive (AWD) axis `DRIVE=` picks which drive the scalar fit
  describes — required there, since the capture records every drive.

Params: as `SERVO_MEASURE_INERTIA` plus `TORQUE_NM` `INERTIA_KGM2` `NAME`
(ident) `DRIVE`. Captures the grid into a run directory and runs
`servo-cal fit --capture <step>.scap`; the profile lands in
`~/printer_data/config/servo_dynamics/dynamics_<name>_<stamp>.toml` and a new
fit never overwrites an existing profile.

#### SERVO_REFINE_DYNAMICS
Empirical refinement of an existing dynamics profile, for when the
`SERVO_FIT_DYNAMICS` regression differs run-to-run with the excitation
grid. Golden-section search over a scale factor applied to the baseline
profile's per-mode **mass** (`TERM=MASS`, default), **viscous**
(`TERM=VISCOUS`) or **coulomb** (`TERM=COULOMB`) vector, or an additive signed
per-pair **direction split** (`TERM=DIRECTION_SPLIT`): each candidate
model is streamed into the *running*
endpoint (no restart) and measured with one tracking capture of the full
`SERVO_MEASURE_INERTIA` `ACCELS` × `SPEEDS` grid, then scored from
`servo-cal analyze` — mean per-move **ferr_peak** for `MASS` and
`COULOMB` (friction error peaks at breakaway, right at the start of the
window), and mean per-move **ferr_rms** for `VISCOUS` (viscous error
shows up as cruise following error). The analyzer's per-move error window starts
`ff_lead_cycles` samples **before** the commanded move (torque
feedforward is sent that many cycles early, so its error signature
lands ahead of the position command; the run manifest carries the
value) and runs **through the settle duration**, so ferr_peak/ferr_rms
cover FF lead-in, in-move tracking, and endpoint overshoot alike (for
every command that reads these metrics, not just the refine);
`overshoot` remains reported separately as the post-move-only peak. On
`coupled_xy`, every vector term (`MASS`, `VISCOUS`, `COULOMB`) refines the
two modes **sequentially** — first a
search over the x-mode entry with X-only strokes, then, on top of the X
winner, the y mode with Y-only strokes — because the two modes are
independent physical quantities (the moved mass, the rail friction) and
one shared scale cannot serve both; an axis stroke leaves the other
mode's velocity at exactly zero, so each phase's score depends only on
its own entry. The provenance keys are `refined_scale_x`/`refined_scale_y`.
Scoring the mean over
the whole grid keeps a scale that helps at one operating point but hurts
at another from winning; every per-scale line also lists mean overshoot,
ferr_rms, and ferr_peak so the non-scored metrics can be sanity-checked.
The
baseline is `PROFILE=` or the node-level `[ethercat_node]
dynamics_profile`; per-motor profiles are not supported (point `PROFILE=`
at an equivalent node-level TOML). The search brackets `[LO, HI]` around
1.0 and stops when the bracket is narrower than `TOL` or `MAX_EVALS`
candidates have been measured; an explicit baseline measurement at scale
1.0 always competes, and the winner is the best *measured* candidate. A
`torque_saturated` flag on any step aborts the run — clipped strokes
cannot score a candidate. `resonance_detected` is ignored here: scaling
a feedforward term does not move the loop's resonances, and the ratio
metric is amplitude-blind (the strongest 20–450 Hz PSD peak over the
mean 1–4 Hz power), so high-accel refine strokes — which put almost
nothing in the low band — trip it on µm-level mechanical peaks the
machine shows on every normal move.

`DIRECTION_SPLIT` runs one sequential phase per pair. For each candidate step,
the profile frame defines `lambda`: `+1` for exactly equal pair columns and
`-1` for exactly opposite columns. Analyzer moves are joined by `move`; both
drives must have identical move sets and matching `start_ms`/`end_ms`, nonzero
directions, and `direction_second = lambda * direction_first`. Each aligned
move contributes the signed differential `q = ferr_mean_moving_first -
lambda * ferr_mean_moving_second`. The scorer averages `q` separately as
`q_plus` for first-drive direction `+1` and `q_minus` for direction `-1`,
requires both bins, and minimizes `ferr_mean_direction_imbalance =
abs(q_plus + q_minus) / 2`. Thus a persistent
per-motor error that reverses with travel direction cancels, while an even
pair-split error remains. Candidate output reports `q_plus`, `q_minus`, and
`ferr_mean_direction_imbalance`; malformed alignment or direction data aborts
refinement.

The candidate is an additive delta, not a scale, so delta `0` is the measured
baseline and a profile with no `[[pair]]` records can be augmented without
refitting the common dynamics. Missing pairs are taken from the current
slot-ordered AWD kinematic layout when it agrees with the profile frame,
otherwise from groups of exactly two equal or opposite frame columns; zero and
unmatched columns are ignored, ambiguous larger exact-match groups fail, and a
kinematically known pair with unequal parallel columns fails rather than
guessing.
The default delta bracket is `[-0.25, 0.25]`, reduced when needed to keep every
candidate at `abs(direction_split) < 0.5`, with default `TOL=0.01`. Explicit
`LO`/`HI` must contain zero and keep both bracket ends in range. The signed
convention is solely `slots = [first, second]`, with differential
`tau_first - lambda*tau_second`; swapping the slots requires
`w' = -lambda*w` (equal columns negate `w`, opposite columns preserve it), and
no motor orientation metadata participates.
The live model is **always** restored to the baseline afterwards
(also on failure; if klippy dies mid-run the endpoint keeps the last
candidate until restart). When a candidate beats its baseline the refined
profile is written to a new TOML under
`~/printer_data/config/servo_dynamics/` (with
`refined_source`/`refined_term`/`refined_scale`/`refined_run` provenance
keys — `refined_scale_x`/`refined_scale_y` for the sequential corexy
refines and `refined_delta_<first-slot>` for direction splits — never
overwriting) and the `dynamics_profile` paste line is printed
— config edit + restart is the only way to keep it; when the baseline
wins, nothing is written. Refine `MASS` first, then `TERM=VISCOUS`
against the refined profile, then `TERM=COULOMB` against
that, and `TERM=DIRECTION_SPLIT` last on AWD. Params: `TERM` (MASS) `AXIS`
(X) `SERVOS` `PROFILE` `LO` (0.7) `HI` (1.3) `TOL` (0.02) `MAX_EVALS` (10)
`START` `END` `X_START` `X_END` `Y_START` `Y_END` `ACCELS` `SPEEDS`
`ITERATIONS` `DWELL_MS` `TAG` (refdyn) `NAME` (refined_<term>).

#### SERVO_CALIBRATE_INERTIA_RATIO
Step 2 of tuning: identify the load inertia and print the recommended C00.06.
`TORQUE_NM` and `INERTIA_KGM2` are **required** (config or param). On
`coupled_xy` kinematics this runs the X+Y grid over every belt drive, fits the
per-mode masses, and prints C00.06 for both directions (per drive on AWD);
the drive takes one scalar, so start from the light-direction number and
confirm with `SERVO_SWEEP_INERTIA` (both motors must be the same model). On
cartesian kinematics it fits the single axis named by `AXIS`. Params: as
`SERVO_MEASURE_INERTIA` plus `TORQUE_NM` `INERTIA_KGM2` `NAME` (inertia).
Apply the printed number with `SERVO_SET_INERTIA_RATIO`.

## Drive-parameter / gain commands

`SERVO` selects the drive; it defaults to the sole configured servo when
`servos` names exactly one, otherwise it is required.

#### SERVO_SHOW_TUNING
Reads back tuning mode (C00.04), stiffness level (C00.05), load inertia ratio
(C00.06), gain set 1 (C01.00–02), and the velocity/torque feedforward params
(C01.13–18). Param: `SERVO`.

#### SERVO_SET_INERTIA_RATIO
Writes C00.06 load inertia ratio in percent. Params: `RATIO` (0..12000) `SERVO`.

#### SERVO_APPLY_GAINS
Switches the drive to manual tuning (C00.04=0), writes gain set 1, and prints
the readback. `POS_GAIN` is 0.1 rad/s, `SPEED_GAIN` 0.1 Hz, `INTEGRAL` 0.01 ms;
defaults are the factory Low preset. Params: `POS_GAIN` (400) `SPEED_GAIN`
(250) `INTEGRAL` (3184) `SERVO`.

#### SERVO_CALIBRATE_GAINS
Gain sweep, shaper-calibrate style: for each `SPEED_GAINS` entry (0.1 Hz units)
it derives the position gain (`×1.6`) and integral (`1250000 ÷ gain`), records
one capture per step into the run directory, then `servo-cal analyze` writes
`results.json` whose verdict names the highest gain step without resonance or a
torque rail. Always **restores the gains that were active before the sweep**
when it finishes — also on failure — so the machine is never left on a tested
value by accident; keeping a result is always an explicit act (`APPLY=1` or
`SERVO_APPLY_GAINS`). With an accelerometer
(`accel_chip` config option or `ACCEL_CHIP=`) each step also records vibration
data (`step_<name>_accel.csv` next to the `.scap`). `APPLY=1` (default 0,
report-only) writes the verdict's recommended gains *after* the restore,
reads them back (a mismatch is a command error, nothing left half-applied),
and runs one `SERVO_MEASURE_TRACKING` to report before/after following-error
peak and overshoot; a null verdict (every step flagged) makes `APPLY=1` a
command error naming the reason instead of writing anything. `SERVO=` (comma
list) restricts the sweep to a subset of the axis servos; adding
`BASE_SPEED_GAIN=` then pins every non-swept axis servo at that gain (same
`×1.6`/`Ti` derivation, recorded as `base_gains` in the manifest) for the whole
sweep — the asymmetric-gain experiment: hold one belt pair soft while sweeping
the other pair higher; those servos are restored to their prior gains too.
Params:
`SPEED_GAINS` (500,650,800,1000) `AXIS` (X) `START` `END`
`SPEED` (100) `ACCEL` (3000) `ITERATIONS` (2) `DWELL_MS` `TAG` (cal)
`ACCEL_CHIP` `APPLY` `SERVO` `BASE_SPEED_GAIN`.

#### SERVO_GAIN_LADDER
Gain sweep that climbs until analysis flags trouble, instead of a fixed
`SPEED_GAINS` list. Runs the ladder `[SAFE, START, START+STEP, … ≤ MAX]`.
Without `PARAM` it climbs the speed gain with the coupled derivation
(position gain `×1.6`, integral `1250000 ÷ gain`); with
`PARAM=position|speed|integral` it climbs **that one gain in its device
units, holding the other two at their pre-ladder values** — the
single-knob edge finder (the drives must agree on their current gains,
else a command error tells you to align them first). After **each** rung
at or above `START` completes its
capture, `servo-cal analyze` runs on the run so far and that rung's step flags
are inspected; the first rung whose step carries `resonance_detected`,
`torque_saturated` or `settle_window_truncated` **stops the climb** — higher
rungs are never executed. The `SAFE` baseline (always the first rung) never
counts as a stop reason. The ladder always **restores the pre-ladder gains**
at the end (also on failure) — keep a rung with `SERVO_APPLY_GAINS`.
Output is the usual verdict one-liner (recommended step, reason,
run dir) plus, on an early stop, one line naming the rung and the flags that
stopped it. `START` names the first climb value, not a stroke bound — the
stroke
window comes from the configured axis bounds. A mid-ladder analysis failure
(binary non-zero, unreadable `results.json`) aborts loudly; the run directory
keeps everything captured so far. Params: `SAFE` `START` `STEP` (50, must be
> 0) `MAX` (≥ `START`) `PARAM` `AXIS` (X) `SPEED` (100) `ACCEL` (3000)
`ITERATIONS` (2) `DWELL_MS` `TAG` (ladder) `SERVO`.

#### SERVO_SWEEP_INERTIA
Empirical inertia sweep: apply the tuned gains first, then this writes each
C00.06 ratio in `RATIOS`, records one capture per step, and runs
`servo-cal analyze` (`results.json` reports per-step metrics; no automated pick
— read the overshoot trend to choose the ratio). Reverts to the lowest ratio
afterwards. Because there is no automated pick, `APPLY=1` always errors here
(nothing to apply) — choose a ratio from the report and write it with
`SERVO_SET_INERTIA_RATIO`. Params: `RATIOS`
(40,70,100,130) `AXIS` (X) `START` `END` `SPEED` (100) `ACCEL` (3000)
`ITERATIONS` (2) `DWELL_MS` `TAG` (inertia) `APPLY` `SERVO`.

#### SERVO_AUTOTUNE
Packaged tuning sequence, the manual order above run as one state machine:
baseline `SERVO_MEASURE_TRACKING` → `SERVO_CALIBRATE_INERTIA_RATIO` (identify
only) → apply the recommended C00.06 (`SERVO_SET_INERTIA_RATIO`-equivalent) →
coarse gains (`SERVO_APPLY_GAINS` factory defaults) → `SERVO_CALIBRATE_GAINS`
sweep (apply the winner) → `SERVO_REFINE_GAIN` on the speed gain (apply the
winner) → `SERVO_FIT_DYNAMICS` → a final `SERVO_MEASURE_TRACKING` against the
baseline. Each stage transition is logged
(`calibration.autotune_stage`: `stage`, `run_dir`, `outcome`) so the dashboard
can show the sequence as it runs.

`APPLY` defaults to 0: a dry run that still measures the baseline and
identifies the inertia ratio (both read-only), then walks every remaining
stage reporting what it *would* write instead of touching the drive.
`APPLY=1` performs every stage for real and aborts loudly — naming the stage
and run directory — on any of:

- a `torque_saturated` or `resonance_detected` flag on the chosen step of any
  sweep stage (checked whether or not that stage's write ends up gated by
  `APPLY`, since continuing past a flagged step is unsafe regardless);
- a null recommendation (no clean step) on a stage that needs to promote one;
- the final verification's following-error peak regressing more than 20%
  against the baseline.

`APPLY=1` requires `rated_torque_nm`/`rotor_inertia_kgm2` (config or
`TORQUE_NM=`/`INERTIA_KGM2=`) up front — it errors before the first stroke
rather than mid-sequence. The C00.06 recommendation is recovered from the
`servo-cal fit` console output (the same "recommended C00.06 (light
direction): N%" line `SERVO_CALIBRATE_INERTIA_RATIO` already prints) — there
is no separate machine-readable field for it, since `fit` writes a profile
TOML, not a `results.json`. `SERVO_FIT_DYNAMICS` never edits `printer.cfg`;
it prints the `dynamics_profile` line to paste, exactly as it does standalone.
A successful `APPLY=1` run never persists anything to a tuning profile by
itself — run `SERVO_SAVE_TUNING SERVO=... NAME=...` afterwards to keep it.
Params: `AXIS` (X) `APPLY` (0) `TORQUE_NM` `INERTIA_KGM2` `SPEED_GAINS`
`DWELL_MS`.

## Command → output

Every experiment command writes a run directory
`<captures_root>/<tag>_<YYYYmmdd_HHMMSS>/` holding `manifest.json`, one
`step_<name>.scap` per step, optional `step_<name>_accel.csv` recordings, and
(for the analyze commands) `results.json` + `plot_series.json`. The command
prints a one-line verdict plus the run-directory path; the metrics table
streams from `servo-cal` in the interim before the dashboard (Part 3) lands.
Schemas: [servo-cal-contracts.md](servo-cal-contracts.md).

| Command | Invokes | Output |
|---|---|---|
| `SERVO_MEASURE_TRACKING` | `servo-cal analyze` | run dir + `results.json` (per-motor + combined tracking metrics; records every motor driving the axis — both lanes on CoreXY) |
| `SERVO_MEASURE_DIFFERENTIAL` | `servo-cal analyze` | run dir + `results.json` (differential FRF modes: frequency, peak gain, damping, coherence; dashboard renders the FRF) |
| `SERVO_DIFF_DAMPER` | — | no run dir; reconfigures the running endpoint |
| `SERVO_DIFF_TRIM` | — | no run dir; reconfigures the running endpoint |
| `SERVO_MEASURE_STRAIN_MAP` | dashboard `/api/runs/<name>/strain` | run dir with one capture per raster line; charted by the dashboard's strain tab |
| `SERVO_MEASURE_STRAIN_RESPONSE` | in-klippy fit | run dir with one capture per offset step; reports + stores the rolling stiffness matrix |
| `SERVO_STRAIN_COMP_TUNE` | in-klippy loop | run dir with one capture per iteration; converges the matrix, leaves the tuned map written + enabled |
| `SERVO_CALIBRATE_GAINS` | `servo-cal analyze` | run dir + `results.json` verdict (highest clean gain step); `APPLY=1` also writes + verifies |
| `SERVO_GAIN_LADDER` | `servo-cal analyze` (per rung + final) | run dir + `results.json` verdict; climbs until a rung flags trouble, then applies `SAFE` |
| `SERVO_REFINE_GAIN` | `servo-cal analyze` | run dir + `results.json` verdict; `APPLY=1` also writes + verifies |
| `SERVO_SWEEP_INERTIA` | `servo-cal analyze` | run dir + `results.json` (no automated pick, so `APPLY=1` always errors) |
| `SERVO_SWEEP_ACCEL` | `servo-cal analyze` | run dir + `results.json` verdict (max non-railing accel); `APPLY=1` verifies at the recommended accel (no SDO write) |
| `SERVO_FIT_DYNAMICS`, `SERVO_CALIBRATE_INERTIA_RATIO` | `servo-cal fit` | run dir + `~/printer_data/config/servo_dynamics/dynamics_<name>_<stamp>.toml` + C00.06 |
| `SERVO_REFINE_DYNAMICS` | `servo-cal analyze` (per candidate) | run dir + refined `dynamics_<name>_<stamp>.toml` when a scale beats the baseline (pick is host-side; live model always reverted) |
| `SERVO_MEASURE_INERTIA` | — | run dir + `.scap` capture only (the building block behind the fit commands) |
| `SERVO_AUTOTUNE` | all of the above, in sequence | one run dir per stage; `APPLY=0` (default) is a dry rehearsal, `APPLY=1` runs and applies for real |

## The manual capture analyzer

`scripts/servo_capture.py` remains the standalone single-file `.scap` analyzer
for ad-hoc inspection (`--help` for the full option list): following-error,
overshoot/settling, torque-saturation metrics per drive; `--fft` prints
resonance peaks, `--plot` opens a time-series dashboard, `--png` saves one
headless, `--combine-corexy A[:s],B[:s]` with `--axis` renders the CoreXY
dashboard; `--drive` restricts to one drive in a multi-drive capture, `--csv`
exports samples. The four gain/inertia/refine/accel sweep-report scripts and
the fit-dynamics wrapper script were deleted — their metrics and verdict logic
moved into `servo-cal`.
