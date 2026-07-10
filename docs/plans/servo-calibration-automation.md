# Plan: Rust analysis core, machine-readable verdicts, closed-loop autotune

## Why

The servo tuning loop (measure → fit → apply → verify) is the throughput
bottleneck now that the planner core is solid, and its implementation resists
automation:

1. **Decisions live in pixels.** Every run ends in a PNG plus a console table;
   a human reads the overshoot column and types the next command. The
   recommendation logic that does exist (`recommend()` in
   `scripts/servo_accel_report.py`, the gain pick in
   `scripts/servo_gain_report.py`) prints text and exits — nothing upstream
   can consume it.
2. **Two languages co-own the capture format.** Rust writes `.scap`
   (`rust/ethercat-rt/src/capture.rs`, `rust/motion-services/servo_capture`)
   and Python re-parses it with hand-rolled dtype tables
   (`scripts/servo_capture.py`). The bridge back to Rust for the dynamics fit
   is a lossy CSV export (`export_ident_csv` → `servo-ident` re-parses by
   column-name matching).
3. **Filenames are the database.** Sweep metadata is encoded as
   `<tag>_p2000_s1250_i1000_<stamp>.scap` and re-parsed by four slightly
   different regexes (`STEP_RE` in `servo_gain_report.py`,
   `ratio_from_name` in `servo_inertia_report.py`, `value_from_name` in
   `servo_refine_report.py`, `accel_from_name` in `servo_accel_report.py`),
   each with its own copy of `find_sweep_files` and "newest capture wins"
   resolution. A new sweep dimension means a new filename grammar and
   touching every parser.
4. **Shared code has no home.** Report scripts import each other
   (`from servo_gain_report import cruise_mask`) behind `sys.path.insert`
   hacks; the verdict logic lives in the least-typed, least-tested corner of
   the repo while the quality bar (nextest, clippy `-D warnings`, goldens)
   lives in the Rust workspace.
5. **Four sweep commands are one program written four times.**
   `SERVO_CALIBRATE_GAINS`, `SERVO_REFINE_GAIN`, `SERVO_SWEEP_INERTIA`, and
   `SERVO_SWEEP_ACCEL` all do: for each value → write a drive/motion
   parameter → capture a stroke run → name the step; then shell out to a
   step-specific report script.
6. **Dead weight.** `SERVO_MEASURE_FRICTION` produces a capture nothing
   consumes — `servo-ident` already fits coulomb+viscous friction from the
   inertia grid. `scripts/servo_fit_compare.py` is a one-off diagnostic no
   command drives.

And the daily experience is the real cost. A real session (trident bench,
2026-07-10, 15:07–15:17): the same two-step gain sweep — safe gain 550 to
revert to, target gain 700 to make clean — ran **fourteen times in ten
minutes**, with notch registers (`0x2001.0x31` adaptive-notch mode, via
`SET_AUTO_NOTCH_*` / `SET_PARAM_ALL_MOTORS` macros) varied between runs.
The varied values — the experiment's actual independent variable — appear
nowhere in the artifacts: fourteen `gains_cal_<stamp>.png` files differ only
by timestamp, and which notch setting produced which PNG lives in the
operator's memory and the console scrollback. Comparing means opening PNGs
one at a time in Mainsail; iterating means arrow-up through console history
while reports flood the console with text.

Target state, split by role rather than by inertia:

- **Analysis, metrics, verdicts: Rust** — a `servo-cal` binary owning `.scap`
  end-to-end, emitting `results.json` with typed verdicts.
- **Orchestration: Python (klippy)** — necessarily; sweeps, strokes, SDO
  writes, and the autotune state machine are woven into printer objects and
  the reactor. Shrinks to thin, *typechecked* glue.
- **Review: a bench dashboard, not PNGs.** The snapshot-test interaction
  model (`snapshots/web`: local server, browser review, explicit accept)
  applied to calibration runs: run list, attempt journal, verdict deltas,
  re-run form. Mainsail is not forked — Moonraker's HTTP API takes the
  G-code. Matplotlib and the report PNGs are deleted outright; the browser
  renders the plot series.

## Part 0 — deletions (first PR, no dependencies)

- Delete `cmd_SERVO_MEASURE_FRICTION` and its doc section in
  `docs/rewrite/servo-calibration.md`.
- Delete `scripts/servo_fit_compare.py` and its doc bullet.
- Keep `SERVO_SET_STIFFNESS` / `SERVO_SHOW_TUNING` / `SERVO_APPLY_GAINS` —
  small SDO plumbing the orchestrator will reuse.

## Part 1 — `servo-cal`: the Rust analysis core

Extend the `servo-ident` crate (or a sibling sharing its models) into one
binary with subcommands:

- `servo-cal analyze <run-dir|.scap>` — native `.scap` reader (format code
  placed so the `ethercat-rt` writer and this reader share one definition),
  ports of `compute_metrics`, `torque_summary`, settle/overshoot detection,
  Welch PSD / moving PSD / peak picking, `cruise_mask`, corexy belt combine.
  Output: `results.json` (metrics + verdict, schema below) plus downsampled
  plot series for the dashboard.
- `servo-cal fit` — absorbs today's `servo-ident` CLI but reads `.scap`
  directly; the CSV export path in Python dies. Existing profile TOML output
  unchanged.
- `servo-cal analyze --dump-csv` — escape hatch for ad-hoc numpy
  prototyping (the `scripts/fitter_prototype/` workflow must not get harder).

Parity strategy: commit two or three truncated real `.scap` fixtures; record
the current Python pipeline's metrics on them as goldens *before* porting;
nextest asserts the Rust port matches within tolerance (PSD windowing and
settle-edge cases are where drift will hide — test those specifically).

Exit: `servo-cal analyze` reproduces today's metrics on the fixtures;
`cargo nextest run -p servo-ident` covers parser, metrics, and verdicts.

## Part 2 — experiment manifests and structured results

An *experiment* (one command invocation: a sweep, a grid, a tracking check)
gets a run directory instead of loose files:

```
~/printer_data/logs/servo_captures/<tag>_<stamp>/
  manifest.json     # written by klippy before the first stroke
  step_<name>.scap  # one per step (grid/tracking runs: one capture total)
  step_<name>_accel.csv
  results.json      # written by servo-cal analyze
  plot_series.json  # downsampled traces for the dashboard
```

- `manifest.json` (klippy writes): experiment type, axis, stroke plan, grid,
  per-step parameters (the gains/ratio/accel actually applied, with SDO
  readback), motor list with inversion + rotation distance, kinematics type,
  git rev, structured-log session id.
- **Ambient drive state** in the manifest — the fix for the invisible
  independent variable. At run start klippy reads back a configured journal
  set of SDO params (`[servo_calibration] journal_params:` — notch registers,
  adaptive-notch mode, whatever the current tuning campaign varies) from
  every captured drive, and additionally records every `SERVO_PARAM` write
  issued since the previous run. A run's manifest then answers "what was
  different about this attempt" by itself.
- `results.json` (servo-cal writes): per-step metrics + a **verdict**:
  `{recommended: {...}, confidence, flags: [torque_saturated,
  resonance_detected, ...], reject_reason}`. Serialized from Rust structs —
  the schema is source code, not convention.
- The filename regexes and all `find_sweep_files` copies die. No legacy
  loader — old flat captures are bench debris; loading one fails loudly with
  the path that was tried.
- klippy reads `results.json` back after `servo-cal` exits, prints the
  verdict and run id (one line — the dashboard is the reading surface, so
  the metric walls disappear from the console), and emits it via
  `structured_log.event("calibration", ...)` so outcomes are queryable with
  `logq.py` like everything else.
- The four report scripts are deleted here, not replaced: their metrics
  moved into `servo-cal` in Part 1 and their figures move into the
  dashboard in Part 3. Until Part 3 lands, `servo-cal analyze` prints the
  metrics table to stdout as the interim reading surface.

## Part 3 — the calibration dashboard (`servo-cal serve`)

The snapshot-review pattern applied to calibration. `servo-cal` grows a
`serve` subcommand: list run directories, serve `results.json` /
`plot_series.json` / `manifest.json`, analyze-on-demand for a capture that
has no results yet, and host a static single-page frontend. Runs on the
bench Pi next to Moonraker.

The page (deliberately boring — `snapshots/web` spirit, not a second
playground). The primary view is a **journal, not a gallery** — tuning is a
sequence of attempts differing in a few parameters, and the page must read
that way:

- **Attempt journal**: one row per run, newest first, auto-refreshing as
  `results.json` files land (a two-step ITERATIONS=1 sweep completes in
  ~40 s, so refresh latency matters). Each row: time, the **ambient-param
  diff against the previous attempt** (`C01.30: 2→1`, from the manifest),
  and headline metrics per step — the target step's resonance flag, peak
  frequency, overshoot, following error. "Which notch setting made gain 700
  clean" becomes a column scan instead of PNG archaeology.
- **Overlay drill-down**: tick attempts in the journal to overlay their
  tracking traces and spectrograms in one chart under the table. Comparison
  is pure rendering of captures already on disk — it never re-runs a sweep,
  never opens a second window, and costs nothing beyond selecting rows. When
  the metric columns already answer the question, the chart is never opened
  at all.
- **One-form iteration**: the swept command params *and* the ambient SDO
  params in one editable form, pre-filled from the selected attempt; submit
  issues the `SERVO_PARAM` writes then the sweep command via Moonraker's
  `printer/gcode/script`. One `cors_domains` line in the bench's
  `moonraker.conf` is the only Mainsail-stack change — no fork.
- **Apply button** (enabled from Part 6): sends the verdict's apply payload
  as the corresponding `APPLY=1` command.

Exit: a notch-tuning iteration (adjust params → run sweep → read outcome →
adjust again) happens entirely on one page, and every attempt's context is
on disk; no PNGs are produced anywhere in the pipeline.

## Part 4 — tuning profiles: persist what a calibration session won

The `SAVE_CONFIG` equivalent, without touching `printer.cfg`. Today a tuning
session's outcome lives in the drives' volatile registers and the console
scrollback; a power cycle or drive swap erases it. The apply half already
exists — `[motor] params:` blocks are parsed by
`servo_param.parse_params_block` and pushed at claim time with readback
verification (`ethercat_node._push_drive_params`). What's missing is the
write-back:

- `[motor] tuning_profile: <name>` — resolves to
  `~/printer_data/config/servo_tuning/<name>.toml`, applied at claim time
  through the same push path, before the config `params:` block. A register
  appearing in both the profile and `params:` is a config error (fail loud),
  not a precedence puzzle.
- Profile format: the tuned register set (gain set C01.00–02, inertia ratio
  C00.06, the `journal_params` — notch registers etc.) plus a provenance
  block: the run id it was promoted from, date, bench, and the headline
  metrics it achieved. A profile answers "why these values" by itself.
- `SERVO_SAVE_TUNING SERVO=... NAME=...` reads the registers back from the
  drive and writes the profile file (never overwriting — timestamped like
  dynamics profiles, switching is an explicit config edit).
- Dashboard: a **"Promote to profile"** button on a journal row writes the
  profile from that run's manifest readbacks — the attempt that won the
  session becomes the persistent state in one click.

Drive state becomes reproducible from files: swap a drive, reconnect,
the profile re-applies; configs stay constant-valued and git-diffable.

## Part 5 — one sweep engine, one stroke planner (klippy)

**Sweep engine** in `klippy/extras/servo_calibration.py`: a sweep is
`(parameter adapter, values, stroke plan)`. Parameter adapters own
apply/readback/revert for their knob:

- gain-set (speed gain → derived position/integral triple, C01.00–02)
- single gain (the `SERVO_REFINE_GAIN` 1-D case)
- inertia ratio (C00.06)
- motion accel (no SDO write — a stroke-plan parameter)

The four sweep commands become ~10-line declarations over the engine.

**Stroke planner**: one module producing stroke plans from the real
kinematics object — axis strokes, CoreXY diagonals, centering moves, grid
pacing, the `v²/a` reachability check. Replaces `_stroke_plan`, `_strokes`,
`_emit_strokes`, `_goto_xy`, `_axis_bounds`, `_xy_bounds` and the per-command
CoreXY duplication. **Command surface change (intentional):** the `_COREXY`
command variants fold into their base commands — the kinematics decides, the
way `SERVO_MEASURE_TRACKING` already does. Update
`docs/rewrite/servo-calibration.md` in the same PR.

**Typecheck the residue:** pyright (strict) scoped to the calibration
modules only — the extra and the stroke planner — via an explicit include
list in `pyproject.toml`, wired as a `ci.sh` job. `Protocol` shims for the
handful of printer objects the extra touches. No ambition to type the wider
klippy estate.

Exit: `ServoCalibration` under ~400 lines; adding a new sweepable parameter
is one adapter class; pyright green in CI.

## Part 6 — closed loop

The dashboard earns the trust this part spends: after enough runs of
agreeing (or disagreeing, and fixing the verdict functions) with the
recommendation shown next to the data, automation is a button, then a
default — not a leap.

- Verdicts gain an `apply` payload (SDO writes, or a dynamics-profile path).
- Sweep commands accept `APPLY=1`: apply the recommended value, read it back,
  run one verification stroke, report before/after tracking metrics. Default
  stays report-only. The dashboard's Apply button sends the same command.
- `SERVO_AUTOTUNE` orchestrator running the documented tuning order as a
  state machine: baseline tracking → inertia ratio (apply) → coarse gains →
  gain sweep (apply) → refine (apply) → dynamics fit → verify tracking vs
  baseline. Each stage consumes the previous stage's `results.json`; any
  verdict flag (`torque_saturated`, `resonance_detected`) or a verification
  regression aborts loudly with the run directory in the message. Every stage
  transition goes to the structured log; the dashboard shows the sequence as
  it runs.
- The dynamics-profile step never edits `printer.cfg` — it writes the profile
  and prints the `dynamics_profile` line to paste, exactly as today.

## Testing

- `.scap` fixtures + Python-recorded goldens drive the Part 1 parity tests
  (nextest); verdict functions get their own cases per flag.
- Sweep-engine and orchestrator control flow: drive `ServoCalibration` under
  the existing `test/klippy_testing` harness with a mocked engine wrapper and
  a stub `servo-cal` (per-step apply/readback/capture recorded and asserted);
  no bench needed.
- Dashboard endpoints (`servo-cal serve`): nextest over the run-dir listing
  and JSON routes with fixture run directories; the frontend is exercised on
  the bench, same as `snapshots/web`.
- Bench validation at the end of each part: one `SERVO_CALIBRATE_GAINS` run
  on the EtherCAT bench, comparing verdict + traces against the previous
  part.

## Out of scope (follow-up plans)

- Folding `motors_sync.py` (vendored third-party per `pyproject.toml`) and
  the resonance/shaper stack onto the same capture store, stroke planner,
  and dashboard.
