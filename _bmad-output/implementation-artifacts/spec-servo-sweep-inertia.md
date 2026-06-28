---
title: 'SERVO_SWEEP_INERTIA — empirical inertia-ratio sweep with visual report'
type: 'feature'
created: '2026-06-28'
status: 'done'
context: []
baseline_commit: '83d5aedb7a111ec177f109c7346fa55311d7420f'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The analytical inertia identification (`servo-ident` least-squares fit feeding `SERVO_CALIBRATE_INERTIA_RATIO`) produces a poor C00.06 load-inertia-ratio recommendation — the cruise is blind to inertia, so the regression has no leverage. The signal that *does* respond to inertia is the transient ringing/overshoot at the accel (move start) and decel (move end) edges, where the drive's inertia-model feedforward dominates.

**Approach:** Add an empirical sweep — exactly mirroring the existing `SERVO_CALIBRATE_GAINS` → `servo_gain_report.py` pipeline, but varying the drive's C00.06 inertia ratio (`0x2000.0x07`) instead of the PID gains. Hold the (pre-tuned) gains fixed, write one ratio per step via live SDO, record one capture per step, then render the **same visual panels** as the gain report, labeled by inertia ratio. This first cut is presentation-only: no automated recommendation. Peak-overshoot scoring is a deliberate later step.

## Boundaries & Constraints

**Always:** Reuse `_SERVO_STROKES` / `_SERVO_CAL_PREP` / `_SERVO_CAL_RESTORE` for motion; reuse the capture naming (`<step>_<YYYYmmdd_HHMMSS>.scap`) and the `step_metrics` analysis from `servo_gain_report.py`. Validate every ratio against the C00.06 range 0..12000 (same as `SERVO_SET_INERTIA_RATIO`). Revert to the first (lowest) ratio after the sweep, like the gain macro reverts gains. Fail loudly on a missing capture / unparseable filename / both-files-and-steps.

**Ask First:** Adding any automated "recommended ratio" pick (out of scope for this cut). Changing the drive tuning mode (C00.04) or touching gain SDOs inside the sweep.

**Never:** Do not touch the host-side feedforward mass (`dynamics_profile` / `ethercat-rt`). Do not modify `servo_gain_report.py` behavior (import from it; do not refactor it). Do not change `servo-ident` / the analytical path. Do not issue G-code or hit hardware during development.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Resolve sweep steps | `--steps cal_r40,cal_r100`, captures present | newest `.scap` per step, sorted by ratio | — |
| Stale captures from older run | two `_r40_<ts>.scap`, different timestamps | picks the newest | — |
| Filename parse | `inertia_r70_20260628_120000.scap` | ratio `70` extracted | non-matching name → skipped/rejected loudly |
| Missing capture for a step | `--steps cal_r40`, no matching file | — | `SystemExit` naming the step |
| Mutually exclusive inputs | explicit files **and** `--steps` | — | `SystemExit("not both")` |
| Ratio out of range | macro `RATIOS=40,99999` | — | macro `action_raise_error` (0..12000) |

</frozen-after-approval>

## Code Map

- `config/servo_calibration.cfg` -- add `[gcode_macro SERVO_SWEEP_INERTIA]` + `[gcode_shell_command servo_inertia_report]`; clone `SERVO_CALIBRATE_GAINS` structure, sweeping `0x2000.0x07`.
- `scripts/servo_inertia_report.py` -- new; reuses `step_metrics` + `RESONANCE_BAND_HZ` from `servo_gain_report.py` (`LOW_BAND_HZ` is consumed inside `step_metrics`, not imported), owns ratio-keyed file resolution + ratio-labeled `render`.
- `scripts/servo_gain_report.py` -- import source only (`step_metrics`, bands); not modified.
- `test/test_servo_inertia_report.py` -- new; mirrors `test_servo_gain_report.py`.
- `test/test_servo_gain_report.py` -- pattern reference.

## Tasks & Acceptance

**Execution:**
- [x] `scripts/servo_inertia_report.py` -- new script: `STEP_RE = _r(\d+)_\d{8}_\d{6}\.scap$`, `ratio_from_name`, `find_sweep_files(dir, tag)` (glob `<tag>_r*.scap`), `find_named_steps(dir, names)`, and `main` with `--captures/--steps/--captures-dir/--tag(default 'inertia')/--out-dir/--out/--drive`. Reuse `step_metrics` from `servo_gain_report`; write a `render` whose legend reads "inertia <ratio>%", curve-panel x-axis "inertia ratio (%)", and table first column the ratio. Print the per-step metrics table; emit NO "recommended" line.
- [x] `config/servo_calibration.cfg` -- add `SERVO_SWEEP_INERTIA` (params `RATIOS` comma-list, `AXIS START END SPEED ACCEL ITERATIONS DWELL_MS TAG SERVO`): prep; per ratio validate 0..12000, `RESPOND` progress, `SERVO_PARAM SET=0x2000.0x07 VALUE={ratio} TYPE=u16`, `SERVO_CAPTURE_START SERVO={servo} NAME={tag}_r{ratio}`, `_SERVO_STROKES`, `SERVO_CAPTURE_STOP`; revert C00.06 to first ratio; restore; `RUN_SHELL_COMMAND servo_inertia_report PARAMS="--tag {tag} --steps {names}"`. Add the matching `[gcode_shell_command servo_inertia_report]`.
- [x] `test/test_servo_inertia_report.py` -- unit-test the I/O matrix rows: newest-per-step, stale exclusion, ratio sort, missing-step `SystemExit`, files+steps exclusivity.

**Acceptance Criteria:**
- Given a directory of `<tag>_r<ratio>_<ts>.scap` captures, when `servo_inertia_report.py --tag <tag> --steps ...` runs, then it writes one comparison PNG (spectrum + time-domain ferr + overshoot panels) labeled by inertia ratio and prints a metrics table, with no recommendation line.
- Given two captures for one step, when steps are resolved, then only the newest is used.
- Given a step name with no capture, when resolved, then the script exits non-zero naming that step.
- Given the macro with an in-range `RATIOS` list, when invoked, then it writes C00.06 once per ratio, captures each as `<tag>_r<ratio>_<ts>.scap`, and reverts to the first ratio.

## Spec Change Log

- 2026-06-28 (review iter 1, patches only — no loopback): Edge/Blind hunters found the macro's `RATIOS` parse silently coerced empty/garbage tokens to `0`, which passed the `0..12000` guard (the gain sibling is accidentally safe because its range starts at 100) — fixed with `|int(-1)` so non-integers fail loudly. Same reviewers found duplicate ratios aliased captures and that "revert to first (lowest)" was false on unsorted input — fixed by dedupe + `|sort`. Acceptance auditor flagged the untested "no captures found" boundary (added test) and an overstated Code Map import line (corrected). Rejected: "writes wrong register 0x07" (repo convention is `C00.06 → 0x2000.0x07`, matching both sibling macros) and "step_metrics arity" (signature is `(path, drive=None)`). KEEP: report imports `step_metrics` verbatim and never modifies `servo_gain_report.py`; no automated recommendation; surfaces confined to the cfg + two new files.

## Design Notes

`step_metrics(path, drive)` in `servo_gain_report.py` is label-agnostic (returns a dict of cruise/overshoot/spectrum metrics) — import and reuse it verbatim; only the gain-specific `gains_from_name`, `render` labels, and `recommend` differ. The macro is a near-line-for-line clone of `SERVO_CALIBRATE_GAINS` with the three gain SDO writes per step replaced by the single `0x2000.0x07` write, and `_p%d_s%d_i%d` step naming replaced by `_r%d`. Gains are assumed already applied (`SERVO_APPLY_GAINS`) — the sweep does not change tuning mode or gains.

## Verification

**Commands:**
- `pytest test/test_servo_inertia_report.py -n0 -v` -- expected: all pass.
- `./scripts/ci.sh ruff` -- expected: clean (check + format) on the new script/test.

**Manual checks:**
- `python scripts/servo_inertia_report.py --help` lists `--steps/--tag/--drive`; no syntax/import error from the `servo_gain_report` import.
- `config/servo_calibration.cfg` parses (macro block well-formed; `RATIOS` range guard present).

## Suggested Review Order

**The sweep (drive side)**

- Entry point: the macro that writes one C00.06 ratio per step and captures each.
  [`servo_calibration.cfg:262`](../../config/servo_calibration.cfg#L262)

- Fail-loud ratio parse (`|int(-1)` rejects empty/garbage that `0..12000` would pass) + dedupe + sort.
  [`servo_calibration.cfg:276`](../../config/servo_calibration.cfg#L276)

- The only SDO it touches — C00.06 (`0x2000.0x07`); reverts to the lowest ratio after.
  [`servo_calibration.cfg:290`](../../config/servo_calibration.cfg#L290)

**The report (host side)**

- Reuses `step_metrics`/`RESONANCE_BAND_HZ` from the gain report verbatim — no fork of the analysis.
  [`servo_inertia_report.py:35`](../../scripts/servo_inertia_report.py#L35)

- Ratio-keyed filename parse + newest-per-step resolution (glob avoids the r40/r400 prefix trap).
  [`servo_inertia_report.py:50`](../../scripts/servo_inertia_report.py#L50)

- Ratio-labeled panels, overshoot first; prints the table, emits NO recommendation (out of scope).
  [`servo_inertia_report.py:79`](../../scripts/servo_inertia_report.py#L79)

**Tests**

- Mirrors the gain-report tests + the four fail-loud boundaries (missing step, files+steps, bad name, none found).
  [`test_servo_inertia_report.py:57`](../../test/test_servo_inertia_report.py#L57)
