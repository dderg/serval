# Vendor motors-sync as a First-Party Kalico Addon

Date: 2026-06-14
Status: design, pending review
Supersedes: `2026-06-14-motors-sync-plugin-port-design.md` and its plan (the
external two-repo port).
Builds on: `2026-06-14-correction-stream-sequences-design.md` — the move
primitive (`submit_correction_sequence`) still stands and is the move path this
addon consumes.

## Why

The external-fork approach broke at the bench: the plugin's machine-geometry
discovery assumes mainline Klipper's config schema (`[printer] kinematics`,
`[stepper_x]` sections), and our fork uses `[kinematics]` / `[motor …]` /
`[limit …]` with different names. That is not a one-line fix — the plugin reads
the old schema in four places, and a future fork schema change would silently
break a separate repo again.

motors-sync is also inherently coupled to motion internals (stepper discovery,
TMC phase reads, the per-motor move path). Exporting enough public API to serve
it at arm's length would mean freezing half the motion engine as a contract.

Decision: **vendor it in-tree.** It becomes a first-party Kalico addon that
moves in lockstep with the engine. Because it ships with us, it may read our
kinematics/axis internals directly; if we refactor those internals, we refactor
the addon in the same commit. No frozen discovery API to design or defend.

## Non-goals / decisions

- **Vendor, don't bridge.** `motors_sync.py` lives at `klippy/extras/`. The
  separate `dderg/motors-sync` fork is retired to a provenance role (records
  the upstream commit we vendored from); no live two-repo coupling, no Pi
  symlink dance.
- **No new public discovery API.** The addon reads our kinematics object
  directly (`kind`, `rails`, `get_status`) and the stepper runtime
  (`get_step_dist`, `get_name`). We reuse existing accessors where convenient
  and read internal structure where not. Declaring a formal public/internal API
  boundary for the motion engine is worthwhile but is **separate, later work** —
  not gated by this. A one-line note in the vendored file records that it
  intentionally couples to motion internals as a first-party addon.
- **Disabled by default.** A Klipper `extras` module loads only if its config
  section exists, so no `[motors_sync]` section = not loaded. No extra
  mechanism.
- **Keep the move backend.** The per-motor move path is unchanged from the
  superseded port spec: single nudges and the gapless buzz both go through
  `submit_correction_sequence`; `force_move.manual_move` (already implemented)
  remains the general single-stepper seam. The vendored
  `StepperManualMove.manual_move` is the bridge-backed version already written.
- **License: preserve, state, credit (GPLv3 → GPLv3).** motors-sync is GPLv3,
  same as Kalico, so it is adopted outright with three obligations met (below).

## Design

### 1. Vendoring + attribution

Copy `motors_sync.py` to `klippy/extras/motors_sync.py`. The relative imports it
uses (`from .z_tilt import ZAdjustStatus`) already resolve under
`klippy/extras`. To satisfy GPLv3:

- **Keep** Maksim Bolgov's copyright line and the GPLv3 license line verbatim in
  the file header.
- **State the change** (GPLv3 §5a): add a header note —
  "Vendored into Kalico and adapted to the new motion engine, 2026-06. Based on
  motors-sync by Maksim Bolgov (https://github.com/MRX8024/motors-sync)."
- **Credit** in a docs entry (e.g. a CREDITS/attribution line) linking upstream
  and naming the author.

The provenance (upstream commit SHA we vendored from) is recorded in the
header note so a future feature can be diffed/cherry-picked manually.

### 2. Discovery layer — read fork internals

The four mainline-schema reads, each rerouted to a fork runtime source. The
addon obtains the kinematics object once via
`toolhead.get_kinematics()` (call it `kin`).

| Mainline read (current plugin) | Fork source |
|---|---|
| `config.getsection('printer').get('kinematics')` | `kin.kind` (`"cartesian"` / `"corexy"`) |
| `config.getsection('stepper_'+ax)` `position_min/max` → rail center | `kin.get_status(None)['axis_minimum'][i]` / `['axis_maximum'][i]`, midpoint, where `i` is the axis's lane index |
| `get_steppers()` filtered by name `'stepper_'+ax` | `kin.rails[lane].get_steppers()` — the motors on that lane |
| `[stepper_'+ax]` `rotation_distance` / `full_steps_per_rotation` → `move_d`, `rel_buzz_d` | the stepper runtime object — `get_step_dist()` and `get_rotation_distance()` |

Axis-name → lane index mapping uses the kinematics' own lane table: the lane
whose claimed axis name equals the plugin's axis letter. Lanes are
`x`=0, `y`=1, `z`=2 by role, and `kin.claimed_axes()` / `kin.lanes()` give the
axis name bound to each lane, so the addon resolves `'x' → lane 0`, `'y' → lane
1` from the object rather than assuming.

For corexy, lane 0 (the `a_motors` belt) and lane 1 (the `b_motors` belt) are
the two belts the plugin syncs as its "x" and "y" axes — each lane already
holds the motor pair (`motor_a`, `motor_a1`) the addon nudges. For cartesian,
lane 0/1 are `x_motors`/`y_motors`. The plugin's existing CoreXY "conflict
motor" logic (disable the opposite belt's motor during measurement) is fed from
the opposite lane's steppers; the logic itself is unchanged.

`move_d` and `rel_buzz_d` keep their existing formulas; only their inputs move
from a config section to the stepper object. `rel_buzz_d = move_d *
microsteps * 5` is equivalent to the current `rotation_distance /
full_steps_per_rotation * 5` (since `move_d = rotation_distance /
full_steps_per_rotation / microsteps`), and `microsteps` is still read from the
addon's own `[motors_sync]` section, so the buzz amplitude is preserved without
touching a stepper config section. The plan pins the exact stepper method after
checking `klippy/stepper.py`.

**Already compatible, no change:** the TMC driver lookup
(`_get_tmc_drivers`) uses `mcu_stepper.get_name()` → `"tmc5160 motor_a"`, which
matches the fork's `[tmc5160 motor_a]` sections. The accelerometer/sensor paths
(`accel_chip`, beacon, `angle.py`, TMC phase query) were confirmed compatible
earlier.

### 3. Move backend (unchanged)

`StepperManualMove` is the bridge-backed version from the superseded port:
`manual_move` resolves the binding via `toolhead.get_motor_binding(name)` and
calls `toolhead.get_bridge().submit_correction_sequence(...)`, with a
reactor-wait anchored before the call; `steppers_enable` keeps the
`stepper_enable` path; the dead trapq/`chelper`/`DummyPrinterMotionQueuing`
machinery is gone.

### 4. Classes touched

`KinematicsParser` (kinematics type), `BaseKinematics` /
`CartesianKinematics` / `CoreXYKinematics` (rail center + motors-on-axis
discovery), `MotionAxis.__init__` (`move_d` / `rel_buzz_d` inputs), and the
already-rewritten `StepperManualMove`. The sensor, model-solving, sync-loop,
and stats code is untouched.

## Validation / fail-loud

- Unknown stepper / unbound motor → `get_motor_binding` raises `config_error`
  (existing, loud).
- The addon requires a `[kinematics]` machine with the synced axes present; if
  an axis the addon is configured to sync has no matching lane, fail with a
  clear error naming the axis and the available claimed axes (do not silently
  skip).
- `submit_correction_sequence` retains its loud failures (non-idle axis, ring
  overflow, refill-behind).

## Testing

- **Host unit (where isolatable):** the axis-name → lane resolution and the
  rail-center midpoint computation are pure given a fake kinematics object
  (`kind`, `rails`, `claimed_axes`, `get_status`) — mirror the fake pattern in
  `test/test_motion_topology.py`. Assert `'x' → lane 0` motors, `'y' → lane 1`,
  and center = midpoint of the status min/max.
- **kalico-sim:** load the vendored addon on a 2-motor-per-belt config; confirm
  it instantiates (no schema error), and a buzz drives a correction stream on
  one motor with the partner idle and axis position unchanged.
- **Bench (Trident):** `[motors_sync]` loads; `SYNC_MOTORS` runs; the buzz is
  one continuous shake on one belt motor (no inter-swing pause); partner motor
  still; axis position unchanged; magnitude converges across repeats.

## Risks

- **Discovery mapping correctness.** The axis→lane and lane→motor-pair mapping
  is the new surface; covered by the host unit test and the bench run. A
  mismatch fails loud (axis-not-found) rather than mis-syncing silently.
- **`get_step_dist` vs configured microsteps.** The exact relationship between
  the hardware step distance and the addon's configured `microsteps` is pinned
  in the plan against `klippy/stepper.py`; if they differ, the buzz amplitude
  formula is adjusted there. Bench convergence is the final check.
