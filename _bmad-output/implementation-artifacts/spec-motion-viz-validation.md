---
title: 'Motion-planner snapshot validation harness'
type: 'feature'
created: '2026-06-25'
status: 'done'
baseline_commit: 'b88070dc6'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-viz-validation/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-motion-viz-validation/snapshot-contents.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** We validate motion-planner/arc-fit changes by hand-rendering `viz_pipeline.py` panels and eyeballing them. That manual loop caught real defects (on-curve acceleration overshoot under a binding jerk limit; tangent fillets left un-eased) but does not scale and leaves no regression guard, and we are about to make more fitter/planner changes.

**Approach:** A pytest-driven snapshot harness (UI-snapshot style) that runs each case's G-code + `printer.cfg` through the real `_motion_engine.pipeline_snapshot` and records the **full raw trajectory dict** as a checked-in `baseline.json`. A re-run that deviates fails; a CLI renders before/after PNGs and re-baselines only on explicit `accept`. New cases are pending until accepted.

## Boundaries & Constraints

**Always:**
- Drive the real planner via `_motion_engine.pipeline_snapshot(...)` — never a reimplemented kinematic model.
- Baseline = the raw `pipeline_snapshot` dict (`kin_*`, `fitted_segments`, `traversal_time_s`), canonically serialized; comparison is on that raw dict, not on the PNG.
- A case is `<case>/case.gcode` + `<case>/printer.cfg`; config (max_velocity/accel/scv/jerk, `[arc_fit]`) is read by the existing `read_printer_config()`.
- Acceleration/jerk shown in PNGs use viz's exact `_build_time_series` math (`a_scalar = √(a_t²+a_n²)`); reuse it, do not re-derive.
- A baseline is written only by an explicit `accept` — never auto-blessed on deviation, never for a new case.
- Fail loudly: a malformed case (missing gcode/cfg) or a missing dylib raises a clear error, never a silent skip in the CLI.

**Ask First:**
- Any change to the `pipeline_snapshot` Rust binding or its sampling resolution (would invalidate every baseline).
- Committing the seed-case baselines (they encode current — including known-bad — behavior).

**Never:**
- No hand-authored numeric pass/fail thresholds — the harness is a change detector, not a correctness oracle.
- No pixel-diffing PNGs as the gate.
- Mid-case limit changes (CAP-5 `M204`/`SET_VELOCITY_LIMIT` mid-stream) — out of scope this increment (deferred; needs a binding change).
- Not a replacement for `cargo nextest` or the seam-continuity harness; not bench validation.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Unchanged planner | case with `baseline.json` | snapshot == baseline → green | N/A |
| New case | case dir, no `baseline.json` | flagged PENDING → red; review renders after-only | N/A |
| Planner change | case with baseline, output differs | snapshot != baseline → red/CHANGED; review renders before+after | N/A |
| Accept | pending/changed cases | `accept [--all\|<case>]` writes current → `baseline.json` (staged); next run green | N/A |
| Dylib not built | no `_motion_engine` | pytest cases skip (existing conftest); review/accept CLI exits non-zero | raise with build hint |
| Malformed case | missing `case.gcode` or `printer.cfg` | run aborts naming the case | raise ValueError |

</frozen-after-approval>

## Code Map

- `scripts/viz_pipeline.py` -- source of truth for `read_printer_config`, `parse_gcode`, `_build_time_series`, `render`; does `_reexec_in_printer_env()` + top-level matplotlib import — must be factored so the harness can import the pure bits without the CLI/reexec.
- `scripts/viz_core.py` -- **new**: extracted importable core (config read, gcode parse, time-series, render with lazy matplotlib); no reexec at import.
- `rust/motion-engine/src/viz.rs` -- `pipeline_snapshot()` (the engine call; do not change).
- `klippy/arc_fit_config.py` -- `arc_fit_from_config()`; reused as-is.
- `tests/motion_engine/conftest.py` -- already skips this dir's tests when the dylib is absent; cases live under it to inherit that.
- `Makefile.rust` -- `motion-engine` target builds the dylib prerequisite.

## Tasks & Acceptance

**Execution:**
- [x] `scripts/viz_core.py` -- extract `read_printer_config`, `_linearize_arc`, `parse_gcode`, `_build_time_series`, `_plot_derivative`, `render` from `viz_pipeline.py`; make matplotlib a lazy import inside `render`; no `_reexec` at module import.
- [x] `scripts/viz_pipeline.py` -- reduce to CLI + reexec that imports from `viz_core`; behavior unchanged.
- [x] `tests/motion_engine/snapshots/harness.py` -- `discover_cases`, `run_case` (read cfg → `parse_gcode` → `pipeline_snapshot`), `canonical_json` (sorted keys, round-trip float repr), `compare(baseline,current) -> EXACT|CHANGED|NEW`, `read_baseline`/`write_baseline`; raise loudly on malformed case or absent dylib.
- [x] `tests/motion_engine/snapshots/review.py` -- argparse CLI: `review` (render before/after PNGs for CHANGED/NEW into a gitignored `review/` dir via `viz_core.render`) and `accept [--all|<case>...]` (write current snapshot → `baseline.json`).
- [x] `tests/motion_engine/snapshots/test_motion_snapshots.py` -- pytest parametrized over discovered cases; CHANGED/NEW → fail with a message pointing at the `review` command.
- [x] `tests/motion_engine/snapshots/test_harness.py` -- unit tests for `canonical_json`/`compare`/status with synthetic dicts (no dylib needed; runs always via `no_engine` marker).
- [x] `tests/motion_engine/snapshots/cases/{clean_arc,tangent_fillet,accel_overshoot_on_curve}/{case.gcode,printer.cfg}` -- seed cases; `tangent_fillet` (verified κ-step 0→0.1, no clothoid) and `accel_overshoot_on_curve` (verified sustained ~20% disk overshoot) capture current known-bad behavior, `clean_arc` a well-behaved baseline.
- [x] `tests/motion_engine/snapshots/.gitignore` -- ignore `review/`.
- [x] `tests/motion_engine/conftest.py` -- register a `no_engine` marker and exempt so-marked tests from the dir-wide dylib skip (so the pure harness unit tests run without the cdylib).

**Acceptance Criteria:**
- Given the dylib is built and baselines committed, when `python -m pytest tests/motion_engine/snapshots`, then every case is green.
- Given a deliberate edit that changes a trajectory, when the suite runs, then exactly the affected case turns red as CHANGED and `review` produces a before/after PNG pair for it.
- Given a new case with no baseline, when the suite runs, then it is red/PENDING and stays red until `accept` writes its first baseline.
- Given changed/pending cases, when `review.py accept --all`, then each `baseline.json` is rewritten to the current snapshot and the next suite run is green.
- Given the dylib is not built, when the suite runs, then the snapshot cases skip (not fail) via the existing conftest, while `test_harness.py` still runs.

## Design Notes

Baseline is the raw `pipeline_snapshot` dict; `a_scalar`/`j_scalar` are render-time views, never stored — so any trajectory change is caught and a richer diff UI can be built later on the same artifact (CAP-4). Comparison is exact float equality on a canonical serialization; this is deterministic on one machine (the planner is deterministic). Cross-environment bit-reproducibility (dev Mac vs Pi vs CI) is the spec's deferred open question — note it in the harness docstring; do not add a tolerance now.

`review.py accept` is the human-in-the-loop gate (CAP-3). The web UI for highlighting changed path regions is explicitly later — v1 is PNG-first per CAP-4.

Baselines are the full raw trajectory stored as deterministic gzip (`baseline.json.gz`, `mtime=0`) under `snapshots/baselines/` — ~3× smaller than raw text, byte-stable so an unchanged trajectory never churns, and still the complete raw artifact without cluttering G-code case folders.

## Verification

**Commands:**
- `make -f Makefile.rust motion-engine` -- expected: builds `_motion_engine`, copied to `klippy/`.
- `python -m pytest tests/motion_engine/snapshots -q` -- expected: all seed cases green after their baselines are accepted; `test_harness.py` green regardless of dylib.
- `python tests/motion_engine/snapshots/review.py review` then `... accept --all` -- expected: renders before/after PNGs, then re-baselines pending cases.
- `./scripts/ci.sh ruff` -- expected: green (new Python passes ruff check + format).

## Suggested Review Order

**The regression gate (read first)**

- The whole-trajectory contract: run a case, compare raw dict to baseline, fail on deviation.
  [`harness.py:143`](../../tests/motion_engine/snapshots/harness.py#L143)
- Canonical serialization — sorted keys, round-trip floats, `allow_nan=False` fails loud on a poisoned sample; baselines stored as deterministic gzip.
  [`harness.py:111`](../../tests/motion_engine/snapshots/harness.py#L111)
- Drives the real engine; raises loudly on a malformed case or absent cdylib.
  [`harness.py:85`](../../tests/motion_engine/snapshots/harness.py#L85)

**Human-in-the-loop re-baseline**

- `accept` is the only write path; refuses without explicit names or `--all`.
  [`review.py:76`](../../tests/motion_engine/snapshots/review.py#L76)
- `review` renders before/after PNGs for every changed/pending case.
  [`review.py:46`](../../tests/motion_engine/snapshots/review.py#L46)

**Shared core extraction (behavior-preserving)**

- `render` lazy-imports matplotlib so the module is import-safe pre-reexec.
  [`viz_core.py:204`](../../scripts/viz_core.py#L204)
- CLI now imports the extracted core; reexec ordering unchanged.
  [`viz_pipeline.py:36`](../../scripts/viz_pipeline.py#L36)

**Test wiring & cases (peripherals)**

- Parametrized snapshot test; changed/pending → fail pointing at the review CLI.
  [`test_motion_snapshots.py:30`](../../tests/motion_engine/snapshots/test_motion_snapshots.py#L30)
- `no_engine` marker exempts pure-logic tests from the dir-wide cdylib skip.
  [`conftest.py:38`](../../tests/motion_engine/conftest.py#L38)
