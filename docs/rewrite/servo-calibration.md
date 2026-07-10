# Servo calibration reference

Command and script reference for tuning an A6-EC servo axis (EtherCAT). The
`[servo_calibration]` extension registers the `SERVO_*` console commands; they
drive the host scripts under `scripts/` and the drive-parameter access provided
by `[servo_param]`/`[servo_capture]`. For the theory behind the inertia/
feedforward fit see [servo-feedforward.md](servo-feedforward.md); for the
capture format see [servo-telemetry-capture.md](servo-telemetry-capture.md).

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
| `servos` | `stepper_x, stepper_y` | single-drive default for `SERVO=`/`AXIS=`-less commands; the CoreXY measure/calibrate commands derive their drives from the kinematics (`SERVOS=` overrides) |
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

Prerequisites: the EtherCAT servo stack (`[servo_param]`, `[servo_capture]`)
must be configured, and the fitter must be built once on the host with
`cargo build --release -p servo-ident` (from `rust/`).

## Tuning order

1. **Enable feedforward** — set `velocity_ff: True` on each `[motor]` so the
   tuning runs measure the loop as it will actually be driven (see
   [servo-feedforward.md](servo-feedforward.md)).
2. **`SERVO_CALIBRATE_INERTIA_RATIO[_COREXY]`** — identify the load inertia and
   set the base C00.06, before touching the loop gains.
   `SERVO_SWEEP_INERTIA` empirically verifies / refines C00.06 later, at the
   tuned gains.
3. **`SERVO_APPLY_GAINS`** then **`SERVO_CALIBRATE_GAINS`** — find the loop
   gains.
4. **`SERVO_FIT_DYNAMICS[_COREXY]`** — fit the dynamic profile at the final
   gains and point `dynamics_profile` at it to enable torque feedforward.

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
`DWELL_MS` `NAME` (track). Runs `servo_capture.py`.

#### SERVO_MEASURE_INERTIA
Records the excitation grid for the inertia/friction fit (no report — it is the
capture building block behind the fit commands). Captures every motor that
moves the axis (both lanes on CoreXY, every drive of an AWD rail). Params:
`AXIS` (X) `START` `END` `ACCELS` `SPEEDS` `ITERATIONS` `DWELL_MS` `NAME`
(ident).

#### SERVO_MEASURE_INERTIA_COREXY
One capture of **every** belt drive with X and Y strokes at every grid point
(`SERVOS=` overrides; the default is every motor the kinematics says drives
the belts), so the
coupled fit can separate the diagonal and off-diagonal inertia (X strokes
excite `m_diag+m_off`, Y strokes `m_diag−m_off`). Before each stroke set the
toolhead moves (at `travel_speed`) to the active axis' start with the idle axis
centered in its range, so both belt runs are near-equal length during the
measurement. Params: `SERVOS` `X_START`
`X_END` `Y_START` `Y_END` `ACCELS` `SPEEDS` `ITERATIONS` `DWELL_MS` `NAME`
(ident).

#### SERVO_MEASURE_FRICTION
Slow constant-speed sweeps for the torque-vs-position friction map; captures
every motor that moves the axis. Params:
`AXIS` (X) `START` `END` `SPEED` (20) `ACCEL` (300) `ITERATIONS` (2) `DWELL_MS`
`NAME` (friction).

## Fit / inertia-ratio commands

#### SERVO_FIT_DYNAMICS
Runs the `SERVO_MEASURE_INERTIA` grid, fits mass/viscous/coulomb, and writes a
timestamped feedforward profile. Optional `TORQUE_NM` + `INERTIA_KGM2` also
print the recommended C00.06. On a multi-drive (AWD) axis `DRIVE=` picks
which drive the scalar fit describes — required there, since the capture
records every drive. Params: as `SERVO_MEASURE_INERTIA` plus
`TORQUE_NM` `INERTIA_KGM2` `NAME` (ident) `DRIVE`. Runs `servo_fit_dynamics.py`; the
profile lands in `~/printer_data/config/servo_dynamics/` and a new fit never
overwrites an existing profile.

#### SERVO_FIT_DYNAMICS_COREXY
As above for CoreXY: runs the X+Y grid over every belt drive
(`SERVO_MEASURE_INERTIA_COREXY`) and fits the coupled mass matrix. The drive
list and, on AWD, the belt pairing are derived from the kinematics motor
lists (two drives per belt fit `--structure corexy-awd`: shared per-drive
mass/coupling, per-drive friction; all four drives must sit on one node).
The resulting profile goes on `[ethercat_node] dynamics_profile` (node-level,
coupled) rather than per-motor. Params: as `SERVO_MEASURE_INERTIA_COREXY`
plus `TORQUE_NM` `INERTIA_KGM2` `NAME` (ident).

#### SERVO_CALIBRATE_INERTIA_RATIO
Step 2 of tuning: identify the load inertia and print the recommended C00.06.
`TORQUE_NM` and `INERTIA_KGM2` are **required** (config or param). Params: as
`SERVO_MEASURE_INERTIA` plus `TORQUE_NM` `INERTIA_KGM2` `NAME` (inertia). Apply
the printed number with `SERVO_SET_INERTIA_RATIO`.

#### SERVO_CALIBRATE_INERTIA_RATIO_COREXY
As above for CoreXY: runs the X+Y grid over every belt drive, fits the
coupled mass matrix, and prints C00.06 for both directions (per drive on
AWD). The drive takes
one scalar, so start from the light-direction number and confirm with
`SERVO_SWEEP_INERTIA`. `TORQUE_NM` and `INERTIA_KGM2` required; both motors must
be the same model. Params: as `SERVO_MEASURE_INERTIA_COREXY` plus `TORQUE_NM`
`INERTIA_KGM2` `NAME` (inertia).

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
one capture per step, and renders a comparison PNG with a recommendation into
`~/printer_data/config/servo_calibrate_results/`. Reverts to the lowest gains
afterwards. With an accelerometer (`accel_chip` config option or `ACCEL_CHIP=`)
each step also records vibration data (`<step>_accel_<stamp>.csv` next to the
`.scap`) and the report gains a bottom row, shaper-calibrate style: vibration
frequency response per step plus a stacked per-step spectrogram. Params:
`SPEED_GAINS` (500,650,800,1000) `AXIS` (X) `START` `END`
`SPEED` (100) `ACCEL` (3000) `ITERATIONS` (2) `DWELL_MS` `TAG` (cal)
`ACCEL_CHIP` `SERVO`. Runs `servo_gain_report.py`.

#### SERVO_SWEEP_INERTIA
Empirical inertia sweep: apply the tuned gains first, then this writes each
C00.06 ratio in `RATIOS`, records one capture per step, and renders a
comparison PNG (read the start/end overshoot to pick the ratio; no automated
recommendation). Reverts to the lowest ratio afterwards. Params: `RATIOS`
(40,70,100,130) `AXIS` (X) `START` `END` `SPEED` (100) `ACCEL` (3000)
`ITERATIONS` (2) `DWELL_MS` `TAG` (inertia) `SERVO`. Runs
`servo_inertia_report.py`.

#### SERVO_SET_STIFFNESS
Vendor-table tuning path: standard mode (C00.04=1) + C00.05 stiffness level
1..31 (factory 12); the drive derives gain set 1 from the level. Params:
`LEVEL` `SERVO`.

## Command → script → output

| Command | Script | Output |
|---|---|---|
| `SERVO_MEASURE_TRACKING` | `servo_capture.py` | tracking metrics to console + per-motor & combined PNG in `~/printer_data/config/servo_calibrate_results/` (records every motor driving the axis — both lanes on CoreXY) |
| `SERVO_FIT_DYNAMICS[_COREXY]`, `SERVO_CALIBRATE_INERTIA_RATIO[_COREXY]` | `servo_fit_dynamics.py` | `~/printer_data/config/servo_dynamics/dynamics_<name>_<stamp>.toml` + C00.06 |
| `SERVO_CALIBRATE_GAINS` | `servo_gain_report.py` | comparison PNG in `~/printer_data/config/servo_calibrate_results/` |
| `SERVO_SWEEP_INERTIA` | `servo_inertia_report.py` | comparison PNG in `~/printer_data/config/servo_calibrate_results/` |
| `SERVO_MEASURE_INERTIA[_COREXY]`, `SERVO_MEASURE_FRICTION` | — | `.scap` capture only |

All captures land in `~/printer_data/logs/servo_captures/` as
`<name>_<YYYYmmdd_HHMMSS>.scap`; per-step accelerometer recordings land next
to them as `<name>_accel_<YYYYmmdd_HHMMSS>.csv`.

## Host scripts

Each script runs standalone (`--help` for the full option list); the commands
above invoke them with the running klippy interpreter.

- **`servo_capture.py`** — analyze a `.scap`: following-error, overshoot/
  settling, torque-saturation metrics per drive; `--fft` prints resonance peaks,
  `--plot` opens a time-series dashboard, `--png` saves one headless (into
  `--plot-dir`, or `--plot-out PATH`), `--combine-corexy A[:s],B[:s]` with
  `--axis` renders the CoreXY dashboard — on-axis and cross-axis tracking error
  with each stroke overlaid, per-motor torque, and moving-vs-stationary axis
  position; the optional per-motor sign `:-1` un-inverts a servo whose
  `invert_direction` flips its encoder counts out of the kinematic frame; an
  AWD belt lists both of its motors joined by `+`
  (`motor_a:1+motor_a1:1,motor_b:-1+motor_b1:1`) and their mean forms the
  belt trace,
  `--drive` restricts to one drive in a multi-drive capture, `--csv` exports
  samples.
- **`servo_fit_dynamics.py`** — resolve the newest capture for `--name`, export
  the fitter CSV, run `servo-ident`, and write the profile TOML (`--structure
  scalar|corexy`, `--drive` for scalar fits of a multi-drive capture,
  `--pairs 'a0,a1;b0,b1'` for 4-drive AWD corexy captures — fitted as
  `corexy-awd` — `--rated-torque-nm`, `--rotor-inertia-kgm2`,
  `--rotation-distance-mm`, `--out-dir`).
- **`servo_gain_report.py`** — gain-sweep comparison PNG + metrics table +
  recommendation (`--tag`, `--steps`); picks up `<step>_accel_*.csv`
  accelerometer recordings next to each `.scap` and adds a frequency-response
  + spectrogram row when present (`--require-accel` makes a missing recording
  an error).
- **`servo_inertia_report.py`** — inertia-ratio sweep comparison PNG + metrics
  table, no automated recommendation (`--tag`, `--steps`).
- **`servo_fit_compare.py`** — diagnostic (not driven by a command): fits the
  scalar inertia three ways from one `.scap` (commanded accel, velocity
  derivative, position second-derivative) and compares, to check the fit is
  stable across C00.06 settings.
