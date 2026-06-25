# Snapshot tests

A standalone test pillar for the motion planner, alongside the rust unit tests
(`cargo nextest`), the python unit tests (`pytest`), and the simulator tests.

Each **case** is a single `.gcode` file under `cases/`. A folder under `cases/`
is a **group**: every `.gcode` in it shares that folder's one `printer.cfg`, so
a case's name is `<group>/<gcode stem>`. Running a case drives the real planner
(`_motion_engine.pipeline_snapshot`) and compares the full raw trajectory to a
committed `<stem>.baseline.json.gz` (deterministic gzip, Git LFS) sitting next to
the G-code. A deviation fails; you review before/after in the browser and
re-baseline on an explicit accept. UI-snapshot testing for trajectories.

## Run

Needs the built `_motion_engine` cdylib (same `python3` you run with); the web
review additionally needs matplotlib. Both run locally, e.g. on macOS:

```sh
make -f Makefile.rust motion-engine      # build the engine for your python3
python3 -m pip install matplotlib        # for the web review only

snapshots/snapshot-tests.sh              # local
snapshots/snapshot-tests.sh --ci         # CI: fail like a plain test, no server
```

- All cases match → exits 0, nothing else.
- A case is **changed** (or a newly added case is **pending**) → it prints a URL
  and starts the review server. It does **not** open a browser — visit the
  printed `http://127.0.0.1:8765` yourself.
- The review server runs **only while the script runs**. **Accept all** writes
  the baselines and, with nothing left to review, the server stops itself; the
  script re-checks and exits with the final status. Nothing is left listening.
- `--ci` skips the server and just fails, so CI gets a plain red.

## Layout

```
snapshots/
  run.py            standalone runner (compare every case; exit code)
  harness.py        library: discover / run_case / canonical_json / compare
  test_harness.py   unit tests for harness (a python-unit test, run by pytest)
  snapshot-tests.sh entry point: run.py, and the review server on a change
  web/              the review server + static front end
  cases/<group>/    printer.cfg shared by every *.gcode in the folder,
                    plus a <stem>.baseline.json.gz per case (LFS)
```

To add a case, drop a `.gcode` into the group whose `printer.cfg` it should run
under (or make a new group folder with its own `printer.cfg`), run the tests,
and accept the baseline.

`run.py` and the cases are **not** pytest — they are this pillar. `test_harness.py`
is a normal python-unit test (the `py` job collects it). The G-code parsing,
config reading and panel rendering are reused directly from `scripts/viz_pipeline.py`
(the same VISUALIZE tool), not duplicated; baselines are full raw trajectories so a
richer diff view can be built later without re-recording them.
