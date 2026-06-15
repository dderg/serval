# Probe Interface Restoration (Spec C)

Restores the probe orchestration surface that the bridge-native `[probe]`
rewrite (816846c19) left out, so the probe-consuming extras work again.
Spec C of the beacon plan
([`beacon-fork-survey.md`](../../rewrite/beacon-fork-survey.md));
no beacon dependency — this fixes our own stranded consumers for any probe.

Reference implementation: this repo's `main` branch is vanilla Kalico, so
`git show main:klippy/extras/probe.py` is the canonical source for
`ProbePointsHelper` and the accessor semantics being restored.

## Problem

The probe rewrite replaced the 1011-line Kalico `probe.py` with a
bridge-native `PrinterProbe` covering only single-probe machinery
(`PROBE`, `QUERY_PROBE`, `PROBE_ACCURACY`). The orchestration layer was
never restored:

- `bed_mesh.py:491`, `z_tilt.py:176`, `quad_gantry_level.py:35`,
  `screws_tilt_adjust.py:50` construct `probe.ProbePointsHelper` —
  deleted → config-load failure for any of those sections.
- `axis_twist_compensation.py:178` calls `probe.get_lift_speed()`
  (missing) and indexes `run_probe(gcmd)[2]` (ours returns scalar z).
- `ProbePointsHelper.start_probe` calls `probe.multi_probe_begin()` /
  `multi_probe_end()` (missing).

Interface lineage, to be precise about dialects: upstream **Kalico** uses
the old-Klipper probe interface (`run_probe` returning a position,
`multi_probe_begin/end`, `ProbePointsHelper`) plus Kalico-only additions
(`RetrySession` hexagonal re-probe, nozzle scrubber). Klipper's 2024
session rewrite (`start_probe_session`/`pull_probed_results`) was never
adopted by Kalico and nothing in our tree calls it. **Decision: restore
the Kalico-shaped interface; no session layer.** A session abstraction
built now would ship unvalidated — if the beacon fork (spec D) or
`rapid_scan` scanning ever needs one, it gets designed then against a
real device/emulator, with `ProbePointsHelper.start_probe`'s `METHOD=`
dispatch as the named integration point.

## Scope

In (host Python only):

- `ProbePointsHelper` restored in `klippy/extras/probe.py` (ported from
  `main`, minus the retry/scrubber tier).
- `PrinterProbe` gains `get_lift_speed(gcmd=None)`, `multi_probe_begin()`,
  `multi_probe_end()`; `run_probe(gcmd)` returns a full position.
- Fail-loud stub in `z_tilt.py`'s `ZAdjustHelper.adjust_steppers`.

Out (each fails loudly if reached):

- Per-motor Z offset moves (bridge + firmware primitive) and therefore a
  working `Z_TILT_ADJUST` / `QUAD_GANTRY_LEVEL` adjust step — own spec
  later. The bridge drives every stepper bound to a kinematic slot in
  lockstep (motion_toolhead.py:632); `MCU_stepper.set_trapq` /
  `generate_steps` are vestigial (stepper.py:165,173), so mainline
  Kalico's detach-move-reattach trick would silently move all Z motors
  identically — wrong results, hence the stub.
- `RetrySession` / hexagonal re-probe offsets / `GcodeNozzleScrubber`
  (Kalico tier). Their config options are not read, so klippy's
  unused-option check rejects them at boot. The seam for restoring the
  tier is the helper's per-point `probe.run_probe(gcmd)` call.
- Klipper session interface and `METHOD=rapid_scan`.
- `bltouch` / `dockable_probe` / `smart_effector` providers.
- `PROBE_CALIBRATE`, `Z_OFFSET_APPLY_PROBE`.

## `PrinterProbe` changes

- `run_probe(gcmd)` → `[x, y, z]`: toolhead XY at probe time, measured
  trip z (the existing multi-sample average/median) as Z. This is what
  `ProbePointsHelper` appends to `results` and what finalize callbacks
  and `axis_twist_compensation` expect. Signature takes only `gcmd` — no
  `retry_session` parameter until the retry tier returns.
- `get_lift_speed(gcmd=None)`: `gcmd.get_float("LIFT_SPEED", lift_speed)`
  when a command is given, else configured `lift_speed`.
- `multi_probe_begin()` / `multi_probe_end()`: no-ops on a plain GPIO
  probe; the lifecycle seam dockable/BLTouch/beacon implementations
  override later.
- `cmd_PROBE` adapts to the new return shape; `last_z_result` semantics
  unchanged.

## `ProbePointsHelper` port

Same constructor signature as `main` (`config`, `finalize_callback`,
`default_points=None`, `option_name="points"`, `use_offsets=False`,
`enable_horizontal_z_clearance=False`) and same public surface
(`get_probe_points`, `minimum_points`, `update_probe_points`,
`use_xy_offsets`, `get_lift_speed`, `start_probe`), so all consumer call
sites work verbatim.

Kept: the probing loop (`_move_next` / `_lift_toolhead` / `_next_pos`),
`horizontal_move_z` + `adaptive_horizontal_move_z` +
`horizontal_z_clearance`, XY probe-offset application (`use_offsets`),
`enforce_lift_speed`, the finalize-callback protocol (return `"retry"`
or a numeric error → re-probe the batch; numeric error also feeds
adaptive horizontal_move_z), `METHOD=manual` via
`manual_probe.ManualProbeHelper` (intact in our tree),
`horizontal_move_z < probe z_offset` rejection.

Deleted relative to `main`:

- `RetrySession` plumbing — `_next_pos` returns the plain probe point.
- `GcodeNozzleScrubber`.
- `METHOD=rapid_scan` — **hard error** ("rapid_scan not supported"),
  diverging from Kalico's silent downgrade-to-automatic, per the
  fail-loudly constraint.

Toolhead surface used: `manual_move`, `get_last_move_time`,
`get_position` — all present and semantically compatible on
`MotionToolhead`.

## z_tilt / QGL fail-loud stub

`ZAdjustHelper.adjust_steppers` raises: per-motor Z adjustment is not
yet implemented. Consequence: `Z_TILT_ADJUST` and
`QUAD_GANTRY_LEVEL` probe their points, report measured deviations, then
error at the adjust step. The probing half is real exercise of the
helper; the raise prevents the silent lockstep no-op. The `set_trapq`
juggling body is deleted, not bypassed.

## Error handling

All existing probe.py hard errors unchanged (already-triggered,
no-trigger-within-travel, tolerance-after-retries, unhomed PROBE, …).
New: rapid_scan rejection; per-motor-adjust raise;
`horizontal_move_z < z_offset` config/command error; deleted-tier config
options rejected by the unused-option check.

## Testing

- Unit tests (separate file per repo convention) for pure logic:
  `run_probe` result shape and aggregation, helper point/offset
  arithmetic, finalize-retry protocol driven by a stub probe object.
- kalico-sim end-to-end: config with `[probe]` + `[bed_mesh]` +
  `[screws_tilt_adjust]` + `[axis_twist_compensation]` + `[z_tilt]`.
  `BED_MESH_CALIBRATE` and `SCREWS_TILT_ADJUST` complete;
  `Z_TILT_ADJUST` fails with the not-implemented error after probing;
  `PROBE` output position has the full shape. Existing probe sim
  coverage stays green.
