# Trajectory oracle

Reference dataset of mainline-Kalico planner output for a curated set of
short G-code inputs, plus diff harness + fork-side adapter that lets us
assert this fork produces an equivalent trajectory. Purely offline; no
MCU, no hardware, no `printer.cfg` touch.

## Status

- **Mainline reference (`expected/*.csv`):** captured, all 4 inputs.
- **Fork capture (`actual_fork/*.csv`):** generated via
  `tests/oracle/regen_fork.py` (klippy batch-mode + `[bridge-trace]
  move` parse + trapezoidal reconstruction). All 4 inputs currently
  diverge from mainline — see `Fork-side workflow` below for the
  meaning of divergence at this layer.

The fork's klippy used to crash inside batch mode at
`serialhdl.py:412` because `ffi_main` is `None` in bridge mode. Fixed
in `klippy/serialhdl.py:connect_file` by lazy-initialising `chelper`
only in that debug-only entry point; production `connect_pipe` /
`connect_uart` paths are unchanged.

## Layout

```
tests/oracle/
├── README.md             # this file
├── cfg/
│   └── oracle.cfg        # minimal CoreXY config (no shapers, no PA)
├── inputs/
│   ├── 01_x10.gcode          # single X jog — the bench-broken case
│   ├── 02_two_segments.gcode # two collinear X moves — junction trapezoid
│   ├── 03_xy_diagonal.gcode  # XY diagonal — CoreXY mixing
│   └── 04_xy_corner.gcode    # right-angle corner — lookahead matters
├── expected/             # mainline Kalico (upstream/main) trajectory CSVs
│   ├── 01_x10.csv
│   ├── 01_x10.meta.json
│   └── ...
├── actual_fork/          # this fork's captures (currently only .log
│   │                       crash dumps — see Status above)
│   ├── 01_x10.log
│   └── ...
└── diff.py               # diff harness: sample-by-sample, first divergence
```

## Inputs — why these four

| Input | What it exercises |
|---|---|
| `01_x10` | Smallest possible non-zero motion. Mirrors the bench-broken jog. |
| `02_two_segments` | Two collinear segments — same direction, no cornering, but the planner emits a trapezoid across the junction. |
| `03_xy_diagonal` | Single move with both axes — CoreXY A/B stepper mixing exercised. |
| `04_xy_corner` | Right-angle corner — engages junction-deviation cornering and lookahead. |

All inputs intentionally **omit `G28`**. klipper-sim's klippy runs without
endstops; a `G28` in that environment turns into a several-hundred-second
"home that never finds the switch" trajectory and swamps the actual G1 in
the CSV. We replace it with `SET_KINEMATIC_POSITION X=0 Y=0 Z=0`, which
informs the toolhead of the origin without commanding any motion.

We did not include a G5 case — our fork supports cubic Bézier internally
but klipper-sim's batch-mode klippy parses G5 with the unmodified upstream
gcode handler, which has no idea about our internal Bézier representation
and would either reject it or silently rewrite. Once the fork-side oracle
works, add a G5 input here.

## Regenerate `expected/` (mainline reference)

Mainline Kalico is checked out at `/tmp/kalico-mainline` (a worktree of
this repo on `upstream/main`). To recreate:

```bash
# One-time setup, only if /tmp/kalico-mainline doesn't exist:
cd /Users/daniladergachev/Developer/kalico
git fetch upstream
git worktree add /tmp/kalico-mainline upstream/main

# Generate all four expected CSVs:
cd /Users/daniladergachev/Developer/klipper-sim
for n in 01_x10 02_two_segments 03_xy_diagonal 04_xy_corner; do
  python3 simulate_gcode.py \
    --klipper-root /tmp/kalico-mainline \
    --klipper-dict test/fixtures/klipper.dict \
    --config /Users/daniladergachev/Developer/kalico/tests/oracle/cfg/oracle.cfg \
    --runner docker \
    /Users/daniladergachev/Developer/kalico/tests/oracle/inputs/${n}.gcode \
    -o /Users/daniladergachev/Developer/kalico/tests/oracle/expected/${n}.csv
done
```

Runtime: ~30 s per input under Docker (Bookworm container build + klippy
batch mode + chelper rebuild on first run).

The CSV schema is whatever klipper-sim writes (currently 13 columns):
`t, x, y, z, e, vx, vy, vz, ve, ax, ay, az, ae`, one row per 100 µs.
`diff.py` discovers columns dynamically — schema additions are tolerated.

## Fork-side workflow

### Regenerate `actual_fork/`

```bash
cd /Users/daniladergachev/Developer/kalico
python3 tests/oracle/regen_fork.py
```

This runs the fork's klippy in batch mode for each input:

```
python3 klippy/klippy.py tests/oracle/cfg/oracle.cfg \
    -i tests/oracle/inputs/<stem>.gcode \
    -o /tmp/oracle_fork_<stem>.out \
    -d ~/Developer/klipper-sim/test/fixtures/klipper.dict \
    -l tests/oracle/actual_fork/<stem>.log
```

It then parses `[bridge-trace] move:` records from `klippy.log`
(emitted by `klippy/motion_toolhead.py:367`) and reconstructs a 100 µs
CSV with the same schema as `expected/*.csv`.

### `[bridge-trace] move:` schema

Per-line format (one line per `MotionToolhead.move` call,
unconditional — fires whether or not the bridge is attached):

```
[bridge-trace] move: newpos=[X, Y, Z, E] speed=<mm/s> dx=DX dy=DY dz=DZ de=DE feedrate=<mm/s> bridge_is_none=<bool>
```

- `newpos`: absolute end position of the move in (x, y, z, e), mm
- `speed`: gcode-requested feedrate (`F` value), already converted to
  mm/s (NOT mm/min — `gcode.py` divides by 60)
- `dx, dy, dz, de`: per-axis deltas for this move, mm; `move.axes_d`
- `feedrate`: post-clamp scalar speed (`move_d / min_move_t`), mm/s.
  Same value as `speed` except when Z-only and capped to
  `max_z_velocity`
- `bridge_is_none`: True in debug mode (no MCU), False in production

Sample rate: **one record per submitted move**, not time-sampled. The
adapter reconstructs the per-100µs trajectory by integrating the
moves with a trapezoidal kinematic profile.

### Reconstruction model (mainline parity, not fork-engine fidelity)

`regen_fork.py:sample_trajectory` is a faithful re-implementation of
mainline Klipper's trapezoidal planner: forward/backward velocity
pass, junction-deviation cornering (`klippy/toolhead.py:91-117`,
`scv² × (√2-1) / max_accel`), accel/cruise/decel decomposition per
move. It uses the same `max_velocity`, `max_accel`,
`square_corner_velocity` the mainline reference run uses.

**What the diff vs `expected/*.csv` does and does not cover:**
- It DOES catch divergences in *which moves the fork's host-side
  planner forwards to the bridge* (`MotionToolhead.move` arguments) —
  i.e. anything bug-shaped in `motion_toolhead.py`, `motion_bridge.py`'s
  pre-bridge accounting, gcode-side coordinate transforms, or move
  validation/clamping.
- It does NOT cover post-bridge anything: the Rust planner
  (`rust/motion-bridge/src/planner.rs`), shaper, beta iteration,
  emit_shaped, or runtime step emission. The fork uses cubic Bézier +
  smooth_zv shaping with a fundamentally different trajectory shape
  than mainline's trapezoid + smooth_zv kernel. Even on a healthy fork
  the per-100µs samples diverge — *this is expected, not a bug*. The
  diff reports concrete numbers (sample index, axis, delta) which
  localise where divergence begins.

For a fork-engine-fidelity oracle, the next step would be to log full
Bézier control points per `[bridge-trace] seg-dispatch`
(`rust/motion-bridge/src/bridge.rs:2186` currently logs only
endpoints) and replace `sample_trajectory` with a Bézier evaluator.
That is the right layer to compare cubic-vs-cubic, post-shaping; it's
left for the follow-up that adds the per-piece dump to Rust.

### Failure path

If the fork's klippy fails to start (non-zero exit) or emits no
`[bridge-trace] move:` lines, the adapter writes only
`actual_fork/<stem>.log` and `diff.py` reports `MISSING FORK CAPTURE`.

## Diff

```bash
python3 tests/oracle/diff.py            # all inputs
python3 tests/oracle/diff.py 01_x10     # one input by stem
```

Output:
- `[<stem>] OK (N samples match within tol)` — perfect match.
- `[<stem>] FIRST DIVERGENCE: <col> diverges at sample N (t=X): expected=A actual=B delta=C tol=D` — first row whose `(t, axis)` falls outside tolerance.
- `[<stem>] MISSING FORK CAPTURE: …` — fork hasn't produced a CSV yet (current state).
- `[<stem>] MISSING EXPECTED: …` — mainline reference missing; regenerate.

Tolerances (`tests/oracle/diff.py`):
- position 1e-4 mm,
- velocity 1e-2 mm/s,
- acceleration 1.0 mm/s² (deliberately loose — accel comes from differentiating an integrated step trace, which is noisy at the per-100µs grid).

Exit codes: 0 = all match, 1 = at least one divergence, 2 = missing capture.

## Next oracle layer

The current fork adapter compares at the **move-handoff** layer
(what `MotionToolhead.move` forwards to the bridge), with a mainline-
parity trapezoidal reconstruction. To compare at the **post-shaping
Bézier** layer (true fork-engine output) we'd need:

1. Extend `rust/motion-bridge/src/bridge.rs:2186` to log full
   per-piece Bézier control points + degree + axis assignments
   (currently logs only endpoints), and
2. Replace `regen_fork.py:sample_trajectory` with a Bézier evaluator
   that reproduces the runtime's piece evaluation at 100 µs.

That gives a real cubic-vs-cubic comparison; it has nothing to do with
mainline's trapezoid CSV, so it'd live as a separate `expected_bezier/`
reference captured from a known-good fork checkpoint rather than from
upstream/main.
