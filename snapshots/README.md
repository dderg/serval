# Snapshot tests

A standalone test pillar for the motion planner, alongside the rust unit tests
(`cargo nextest`), the python unit tests (`pytest`), and the simulator tests.

A folder under `cases/` is a **group**. Every `*.cfg` in it (any `<name>.cfg`,
not a magic `printer.cfg`) runs against every `*.gcode` in it — a config × gcode
matrix — so a **case** is one (config, gcode) pair and its name is
`<group>/<cfg stem>/<gcode stem>`. Running a case drives the real planner
(`_motion_engine.pipeline_snapshot`) and compares the full raw trajectory to a
committed `baselines/<group>/<cfg>/<stem>.baseline.json.gz` (deterministic gzip).
A deviation fails; you review before/after in the browser and re-baseline on an
explicit accept. UI-snapshot testing for trajectories.

## Run

Needs the built `_motion_engine` cdylib (same `python3` you run with). Runs
locally, e.g. on macOS:

```sh
make -f Makefile.rust motion-engine      # build the engine for your python3

snapshots/snapshot-tests.sh              # local
snapshots/snapshot-tests.sh --ci         # CI: fail like a plain test, no server
snapshots/snapshot-tests.sh --view       # read-only baseline gallery
```

- All cases match → exits 0, nothing else.
- A case is **changed** (or a newly added case is **pending**) → it prints a URL
  and starts the review server. It does **not** open a browser — visit the
  printed `http://127.0.0.1:8765` yourself.
- The review server runs **only while the script runs**. **Accept all** writes
  the baselines and, with nothing left to review, the server stops itself; the
  script re-checks and exits with the final status. Nothing is left listening.
- `--ci` skips the server and just fails, so CI gets a plain red.
- `python3 snapshots/run.py --accept CASE...` / `--accept-all` writes the baselines of the named (or all) changed or pending cases without the server, for agents that have already reviewed the diff.
- `--view` starts the browser UI in read-only mode and renders the
  committed baselines without running the planner or offering accept actions.

## Configs and the live parity gate

A case config is a printer config: `[printer]`, `[kinematics]`, `[motor]` and
`[axis]` sections as a live printer declares them. Every config-driven plan —
the cases, `tools/bench/analyze.py`, the replays, the corpus and `init_planner`
on the printer — derives its planner through
`planner_config::from_doc::planner_config_from_text`, and every G-code move
through `motion_pipeline::live_config::GcodeMoveBase`, so the live host and
the snapshot path cannot plan the same G-code under different settings.

`scripts/ci.sh live-parity` (`snapshots/test_live_bridge_parity.py`) holds
them to it: klippy's own G-code handlers drive the real `MotionEngine` bridge
with a detached planner (`init_planner_detached`: the production ingress and
pipeline, no MCU), and every pipeline input and shaped segment must equal the
snapshot path's bit for bit over the corpus — every case, the sim prints on
both bench configs, the dense print on every bench config and profile, and
the capacity ladders and dense print on `capacity.cfg` under the F401's
reported step capacity, passed to both sides (`step_benchmarks`). A
failure names the first diverging input (move number and field) or segment.
One file: `uv run python snapshots/live_bridge_parity.py GCODE CONFIG`.

## G-code

Cases, the playground and `tools/bench/analyze.py` read G-code through one
interpreter (`rust/pipeline-snapshot/src/waypoints.rs`) that plans each file
move for move as the live host would submit it
(`snapshots/test_live_parity.py` replays the same files through klippy's own
handlers and compares).

- **Mirrored:**
  - Moves and coordinates: `G0`/`G1`, `G90`/`G91`, `G92`, `M82`/`M83`.
  - Limits: `M204`, `SET_VELOCITY_LIMIT`, `RESET_VELOCITY_LIMIT`.
  - Overrides and state: `M220`, `M221`, `SET_GCODE_OFFSET`, `SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE`.
  - Stops at rest: `G4`, `M400`, `M18`/`M84`, `SET_KINEMATIC_POSITION`.
  - Defaults match klippy: E is absolute until `M83`, and the speed is 25 mm/s until the first `F`.
- **Refused by name, with the reason:**
  - `G2`/`G3`, `G5`, `G10`/`G11`, `G28`, `G20`.
  - Probing, levelling and calibration moves; `FORCE_MOVE` and stepper buzz or resonance tests.
  - `SET_PRESSURE_ADVANCE`, `SET_POST_PROCESSOR`, `SET_INPUT_SHAPER`, `BED_MESH_PROFILE`/`CLEAR`/`OFFSET` and other planner-setup commands.
  - `PAUSE`/`RESUME`/`CANCEL_PRINT`/`EXCLUDE_OBJECT`.
  - `M109`/`M190`/`TEMPERATURE_WAIT` after the first move.
  - A `SET_KINEMATIC_POSITION` that moves the position after the first move.
- **Ignored, as live ignores them:** every other command, such as temperatures, fans, `M117` and `M73`.

## Playground

`web/static/playground.html` is an interactive spin-off of the review viewer:
paste any G-code (read as described under [G-code](#g-code)), tweak the planner config
(velocity/accel/scv/jerk limits, deviation tolerances) and watch
the toolpath, velocity, acceleration and jerk panels re-plan live. It runs the
**real pipeline** — the same `Fitter → Planner → run_lowerer → Shaper` stages —
compiled to WASM (`rust/motion-playground`, sharing `rust/pipeline-snapshot`
with `_motion_engine.pipeline_snapshot`), so it needs no server or Python:
everything plans client-side, in a worker. **Pin baseline** freezes the current
plan so config changes can be A/B-flipped exactly like snapshot review.

Both the review viewer and the playground color the path panel by *measured*
curvature behavior on the executed (post-shaper) trajectory — Zero, Constant,
Linear, or Other — plus Cusp/Gap markers for a near-zero-speed instant or a
piece-domain mismatch, rather than by the fitter's own line/arc/clothoid
labels. A dedicated Curvature panel plots κ(t) directly alongside velocity,
acceleration, and jerk.

Reach it at `http://127.0.0.1:8765/playground` while any review/`--view`
server runs, or host `snapshots/web/static/` anywhere static (both WASM
bundles live inside it; `snapshot-tests.sh` builds them when stale). The
public copy lives at <https://dderg.github.io/kalico/playground/> — the
`playground-pages` workflow rebuilds it into the `playground/` corner of
the `gh-pages` branch on every push to `main`.

## Layout

```
snapshots/
  run.py            standalone runner (compare every case; exit code)
  harness.py        library: discover / run_case / canonical_json / compare
  test_harness.py   unit tests for harness (a python-unit test, run by pytest)
  snapshot-tests.sh entry point: run.py, and the review server on a change
  web/              the review server + static front end
  cases/<group>/    one or more <name>.cfg × every *.gcode (a matrix)
  baselines/<group>/<cfg>/ <stem>.baseline.json.gz per (config, gcode) pair
```

To add a case, drop a `.gcode` into a group (it runs under every `.cfg` there),
add another `<name>.cfg` to a group to fan its G-code across more configs, or
make a new group folder with at least one `.cfg`. Then run the tests and accept
the baselines.

`run.py` and the cases are **not** pytest — they are this pillar. `test_harness.py`
is a normal python-unit test (the `py` job collects it), and also carries the
G-code parsing and config reading `run_case` uses to actually drive a case.
Baselines are full raw trajectories so a richer diff view can be built later
without re-recording them.
