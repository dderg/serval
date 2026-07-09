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
- `--view` starts the browser UI in read-only mode and renders the
  committed baselines without running the planner or offering accept actions.

## Playground

`web/static/playground.html` is an interactive spin-off of the review viewer:
paste any G-code (G0/G1, G90/G91, G92, M82/M83), tweak the planner config
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
the `gh-pages` branch on every push to `sota-motion`.

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
