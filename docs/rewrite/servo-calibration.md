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
| `servo_cal_binary` | `rust/target/release/servo-cal` | path to the `servo-cal` analysis binary |

Prerequisites: the EtherCAT servo stack (`[servo_param]`, `[servo_capture]`)
must be configured, and the `servo-cal` binary must be built once on the host
with `cargo build --release -p servo-ident` (from `rust/`).

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

**`SERVO_MEASURE_TRACKING`** is the before/after check for any single change.

Every stroke is paced `M400 / G4 / M400` so it replans from idle, and the
stroke engine refuses any `(speed, accel)` pair whose `v²/a` exceeds the stroke
span — that pair cannot reach the target speed within the travel and would not
produce the intended excitation.

## Measurement commands

#### SERVO_MEASURE_TRACKING
Single accel/speed stroke run with capture, then prints per-move following
error, overshoot and settling — the before/after check for any tuning change.
Params: `AXIS` (X) `START` `END` `SPEED` (100) `ACCEL` (3000) `ITERATIONS` (3)
`DWELL_MS` `NAME` (track). Writes a run directory and runs `servo-cal analyze`.

#### SERVO_MEASURE_INERTIA
Records the excitation grid for the inertia/friction fit (no report — it is the
capture building block behind the fit commands). The active kinematics decides
the shape of the grid:

- **`coupled_xy` kinematics** (CoreXY): one capture of **every** belt drive
  with X and Y strokes at every grid point (`SERVOS=` overrides; the default
  is every motor the kinematics says drives the belts), so the coupled fit
  can separate the diagonal and off-diagonal inertia (X strokes excite
  `m_diag+m_off`, Y strokes `m_diag−m_off`). Before each stroke set the
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
Runs the `SERVO_MEASURE_INERTIA` grid, fits mass/viscous/coulomb, and writes a
timestamped feedforward profile. Optional `TORQUE_NM` + `INERTIA_KGM2` also
print the recommended C00.06. The active kinematics decides the fit
structure:

- **`coupled_xy` kinematics**: runs the X+Y grid over every belt drive and
  fits the coupled mass matrix. The drive list and, on AWD, the belt pairing
  are derived from the kinematics motor lists (two drives per belt fit
  `--structure corexy-awd`: shared per-drive mass/coupling, per-drive
  friction; all four drives must sit on one node). The resulting profile
  goes on `[ethercat_node] dynamics_profile` (node-level, coupled) rather
  than per-motor.
- **cartesian kinematics**: fits a single axis. On a multi-drive (AWD) axis
  `DRIVE=` picks which drive the scalar fit describes — required there,
  since the capture records every drive.

Params: as `SERVO_MEASURE_INERTIA` plus `TORQUE_NM` `INERTIA_KGM2` `NAME`
(ident) `DRIVE`. Captures the grid into a run directory and runs
`servo-cal fit --capture <step>.scap`; the profile lands in
`~/printer_data/config/servo_dynamics/dynamics_<name>_<stamp>.toml` and a new
fit never overwrites an existing profile.

#### SERVO_CALIBRATE_INERTIA_RATIO
Step 2 of tuning: identify the load inertia and print the recommended C00.06.
`TORQUE_NM` and `INERTIA_KGM2` are **required** (config or param). On
`coupled_xy` kinematics this runs the X+Y grid over every belt drive, fits the
coupled mass matrix, and prints C00.06 for both directions (per drive on AWD);
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
torque rail. Reverts to the lowest gains afterwards. With an accelerometer
(`accel_chip` config option or `ACCEL_CHIP=`) each step also records vibration
data (`step_<name>_accel.csv` next to the `.scap`). Params:
`SPEED_GAINS` (500,650,800,1000) `AXIS` (X) `START` `END`
`SPEED` (100) `ACCEL` (3000) `ITERATIONS` (2) `DWELL_MS` `TAG` (cal)
`ACCEL_CHIP` `SERVO`.

#### SERVO_SWEEP_INERTIA
Empirical inertia sweep: apply the tuned gains first, then this writes each
C00.06 ratio in `RATIOS`, records one capture per step, and runs
`servo-cal analyze` (`results.json` reports per-step metrics; no automated pick
— read the overshoot trend to choose the ratio). Reverts to the lowest ratio
afterwards. Params: `RATIOS`
(40,70,100,130) `AXIS` (X) `START` `END` `SPEED` (100) `ACCEL` (3000)
`ITERATIONS` (2) `DWELL_MS` `TAG` (inertia) `SERVO`.

#### SERVO_SET_STIFFNESS
Vendor-table tuning path: standard mode (C00.04=1) + C00.05 stiffness level
1..31 (factory 12); the drive derives gain set 1 from the level. Params:
`LEVEL` `SERVO`.

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
| `SERVO_CALIBRATE_GAINS` | `servo-cal analyze` | run dir + `results.json` verdict (highest clean gain step) |
| `SERVO_REFINE_GAIN` | `servo-cal analyze` | run dir + `results.json` verdict |
| `SERVO_SWEEP_INERTIA` | `servo-cal analyze` | run dir + `results.json` (no automated pick) |
| `SERVO_SWEEP_ACCEL` | `servo-cal analyze` | run dir + `results.json` verdict (max non-railing accel) |
| `SERVO_FIT_DYNAMICS`, `SERVO_CALIBRATE_INERTIA_RATIO` | `servo-cal fit` | run dir + `~/printer_data/config/servo_dynamics/dynamics_<name>_<stamp>.toml` + C00.06 |
| `SERVO_MEASURE_INERTIA` | — | run dir + `.scap` capture only (the building block behind the fit commands) |

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
