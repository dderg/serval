# Trajectory oracle

Reference dataset of mainline-Kalico planner output for a curated set of
short G-code inputs, plus a diff harness to assert that our fork
(`sota-motion`) produces equivalent output. Purely offline; no MCU,
no hardware, no `printer.cfg` touch.

## Status (2026-05-21)

- **Mainline reference (`expected/*.csv`):** captured, all 4 inputs.
- **Fork capture (`actual_fork/*.csv`):** **not produced**. Our fork
  crashes inside klipper-sim's batch-mode klippy at the MCU-identify
  step (see `actual_fork/*.log`):

    ```
    File "klippy/serialhdl.py", line 412, in connect_file
        self.serialqueue = self.ffi_main.gc(
    AttributeError: 'NoneType' object has no attribute 'gc'
    ```

    Root cause is architectural, not a bug to patch around. Our fork
    routes all wire I/O through `motion_bridge_native.so` and never
    initialises `chelper`'s `ffi_main` (`serialhdl.py:30` sets it to
    `None`). Even if we worked around the init crash, the fork's
    planner output goes through Rust FFI to the MCU's runtime
    step generator, not through klippy's `serialqueue → queue_step`
    pipe that klipper-sim decodes. The MCU-stream oracle therefore
    cannot capture this fork's trajectory output unmodified.

    What this means for the bug we are chasing (silent motors during
    jogs): the divergence happens *below* the layer this oracle
    observes — the host-side planner submits moves to the bridge
    correctly; the breakage is in `motion_bridge_native.so` /
    `runtime` / MCU step emission. A separate oracle layer is needed
    for the fork. See "Next oracle layer" below.

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

## Regenerate `actual_fork/` (current branch)

Same command as above, but pointed at this checkout:

```bash
cd /Users/daniladergachev/Developer/klipper-sim
for n in 01_x10 02_two_segments 03_xy_diagonal 04_xy_corner; do
  python3 simulate_gcode.py \
    --klipper-root /Users/daniladergachev/Developer/kalico \
    --klipper-dict test/fixtures/klipper.dict \
    --config /Users/daniladergachev/Developer/kalico/tests/oracle/cfg/oracle.cfg \
    --runner docker \
    /Users/daniladergachev/Developer/kalico/tests/oracle/inputs/${n}.gcode \
    -o /Users/daniladergachev/Developer/kalico/tests/oracle/actual_fork/${n}.csv
done
```

As of 2026-05-21 this fails for the reason described in Status. The crash
log is preserved at `actual_fork/<input>.log`.

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

## Next oracle layer (when the fork crash is unblocked)

Two paths, both expand this oracle rather than replacing it:

1. **Patch fork to survive klipper-sim batch mode.** Initialise
   `chelper.get_ffi()` lazily in `serialhdl.SerialReader.connect_file`
   even when running in bridge mode, allocate the legacy serialqueue,
   and have the bridge replay its outbound moves into that queue as
   `queue_step` messages so klipper-sim's decoder sees them. This is
   the cheapest way to make the fork CSV-comparable to mainline at
   the existing `expected/` schema, but it covers only the host-side
   planner — the MCU runtime / `motion_bridge_native.so` step
   generation are NOT exercised.

2. **Bridge-trace oracle (different schema).** Capture the
   `[bridge-trace]` log lines our `motion_toolhead.py` already emits
   (`klippy/motion_toolhead.py:367`) — per-move
   `(dx, dy, dz, de, feedrate)` records. Save these alongside the
   100 µs CSV in a `*.moves.jsonl` and write a second diff that
   matches at the move level. This exercises a different layer (move
   handoff to bridge, not step emission) and is the cleaner long-term
   answer if we keep the bridge architecture.
