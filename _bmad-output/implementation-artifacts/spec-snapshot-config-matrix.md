---
title: 'Snapshot tests: config × gcode matrix'
type: 'feature'
created: '2026-06-28'
status: 'done'
context: []
baseline_commit: '016fbea3de2b5d1beb6da5947b5c0b09c3febb66'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** A snapshot group runs every `.gcode` against exactly one magic `printer.cfg`. Trying the same G-code under several planner configs means copy-pasting it into multiple group folders. There is no way to fan one G-code across many configs.

**Approach:** Treat every `*.cfg` in a group folder as a config (drop the magic `printer.cfg` name) and discover the full cross-product: each `.gcode` runs against each `.cfg`. Case identity grows a config dimension, ordered `<group>/<cfg_stem>/<gcode_stem>`, so each config is a self-contained section in the review UI. Existing groups keep their `printer.cfg` (now just a config named `printer`); their baselines are relocated by file rename — content unchanged, never regenerated.

## Boundaries & Constraints

**Always:** Discovery is `*.cfg` × `*.gcode` per group, both sorted, empty `.gcode` skipped. Case name and baseline path are `<group>/<cfg_stem>/<gcode_stem>`. The frontend (grid + dropdown) and server already round-trip arbitrary-depth `/`-names via a single URL-encoded segment — preserve that. Fail loudly: a group with `.gcode` files but zero `*.cfg` raises.

**Ask First:** Renaming the existing `printer.cfg` files to meaningful names (left as-is for this change). Adding any new `.cfg`/`.gcode` cases (the user adds those).

**Never:** Regenerate or re-bless baseline *content* — the migration is a pure relocation (`git mv`). Do not special-case single-config groups to keep their old 2-level names. Do not touch the planner, `viz_pipeline`, the comparison/tolerance machinery, or the WASM viewer's plotting.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Matrix discovery | group with 2 `.cfg` + 2 `.gcode` | 4 cases, names `<group>/<cfg>/<gcode>`, baseline `baselines/<group>/<cfg>/<gcode>.baseline.json.gz` | N/A |
| Single config (migrated) | group with `printer.cfg` + N `.gcode`, relocated baselines | N cases named `<group>/printer/<gcode>`, all EXACT | N/A |
| Empty gcode | `.gcode` blank/whitespace | skipped, no case | N/A |
| Config, no gcode | group has `.cfg` but no `.gcode` | zero cases, no error | N/A |
| Gcode, no config | group has `.gcode` but no `.cfg` | discovery raises (fail loudly) | `ValueError` naming the group |

</frozen-after-approval>

## Code Map

- `snapshots/harness.py` -- `Case`, `discover_cases`, `run_case`, module docstring; `CONFIG_NAME` removed. Core change.
- `snapshots/run.py` -- wrap the bare `discover_cases()` call so a `ValueError` prints `ERROR` and returns exit 2 (matches its existing setup-error contract).
- `snapshots/web/server.py` -- `_render_png`/`_serve_img`/`_serve_snapshot_data` already handle N-level names; `scan()` already routes discover exceptions to the banner. No change expected; verify only.
- `snapshots/web/static/app.js` -- grid grouping: section key = name up to the **last** `/` (`group/cfg`); card label = the leaf (gcode).
- `snapshots/web/static/viewer.js` -- `rebuildCaseSelect`: emit `<optgroup>` per leading path (`group/cfg`), option value = full name, option text = leaf.
- `snapshots/test_harness.py` -- update fixtures/asserts to the 3-level identity; add matrix + gcode-without-cfg tests.
- `snapshots/README.md` -- document the matrix and `<name>.cfg`.
- `snapshots/baselines/<group>/*.baseline.json.gz` -- relocate to `<group>/printer/` (12 files, `git mv`).

## Tasks & Acceptance

**Execution:**
- [x] `snapshots/harness.py` -- remove `CONFIG_NAME`; in `discover_cases`, loop `group.glob("*.cfg")` (sorted) × `group.glob("*.gcode")` (sorted, non-empty), emit a `Case` per pair with `name`/`baseline_path` = `<group>/<cfg_stem>/<gcode_stem>`, `config_path` = the cfg; raise `ValueError` if a group has `.gcode` but no `.cfg`; update docstring + the `run_case` missing-config message.
- [x] `snapshots/run.py` -- guard `discover_cases()` with try/except `ValueError` → print `ERROR` + return 2.
- [x] `snapshots/web/static/app.js` -- group cards by leading path (before last `/`); card shortName = leaf segment.
- [x] `snapshots/web/static/viewer.js` -- build the case `<select>` with `<optgroup>` per leading path; options labeled by leaf.
- [x] `snapshots/test_harness.py` -- migrate existing tests to 3-level names; add: 2×2 matrix cross-product, gcode-without-cfg raises, baseline path shape.
- [x] `snapshots/baselines/` -- `git mv <group>/<stem>.baseline.json.gz <group>/printer/<stem>.baseline.json.gz` for all 12 (content unchanged).
- [x] `snapshots/README.md` -- replace the "one `printer.cfg`" language with the `*.cfg` × `*.gcode` matrix and the `<group>/<cfg>/<gcode>` naming.

**Acceptance Criteria:**
- Given a group with multiple `.cfg` files, when discovery runs, then every `.gcode`×`.cfg` pair is a distinct case with its own baseline path.
- Given the migrated repo, when `snapshots/snapshot-tests.sh --ci` runs, then all 12 existing cases report EXACT (no PENDING, no pruning).
- Given a changed/matrix case, when the review server runs, then the grid sections by `group/cfg`, the dropdown groups by `group/cfg`, and before/after + accept still work.
- Given a group with a `.gcode` and no `.cfg`, when discovery runs, then it raises a `ValueError` naming the group.

## Verification

**Commands:**
- `cd snapshots && python3 run.py` -- expected: 12 ok, 0 changed, 0 pending (after migration).
- `./scripts/ci.sh py` -- expected: `test_harness.py` green (touches `klippy/`-adjacent python pillar).
- `./scripts/ci.sh ruff` -- expected: clean (check + format).

**Manual checks:**
- Drop a second `.cfg` into one group, run `snapshots/snapshot-tests.sh`, confirm the new config's cases appear PENDING, grid sections by `group/cfg`, dropdown optgroups by `group/cfg`, and Accept writes `baselines/<group>/<newcfg>/<gcode>.baseline.json.gz`.

## Suggested Review Order

**Discovery & case identity (core)**

- Entry point: the `*.cfg` × `*.gcode` cross-product and the new `<group>/<cfg>/<gcode>` name + nested baseline path.
  [`harness.py:93`](../../snapshots/harness.py#L93)

- Fail-loud: a group with `.gcode` but no `.cfg` raises (the deliberate one-directional guard).
  [`harness.py:90`](../../snapshots/harness.py#L90)

- Runner surfaces the new discovery `ValueError` as `ERROR` + exit 2, before pruning.
  [`run.py:29`](../../snapshots/run.py#L29)

**Review UI**

- Grid sections by leading path (`group/cfg`) via `lastIndexOf`, card labels by gcode leaf.
  [`app.js:46`](../../snapshots/web/static/app.js#L46)

- Dropdown `<optgroup>` per config, keyed by label so grouping is order-independent (review-hardened).
  [`viewer.js:850`](../../snapshots/web/static/viewer.js#L850)

**Tests & docs (peripheral)**

- Matrix cross-product (2×2) + per-pair path assertions.
  [`test_harness.py:95`](../../snapshots/test_harness.py#L95)

- gcode-without-cfg fail-loud test.
  [`test_harness.py:120`](../../snapshots/test_harness.py#L120)

- Baselines relocated by pure 100% git rename into `<group>/printer/` (content untouched).
  [`baselines/`](../../snapshots/baselines/neptune_cube/printer/layer_5.baseline.json.gz)

- README rewritten to the matrix model.
  [`README.md:6`](../../snapshots/README.md#L6)
