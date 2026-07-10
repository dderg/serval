# servo-cal data contracts

Wire formats between klippy (orchestration), `servo-cal` (analysis), and the
dashboard. Implementations must match this file; change the file first if a
schema has to move. Companion plan:
[servo-calibration-automation.md](../plans/servo-calibration-automation.md).

## Run directory

One experiment (one command invocation) = one directory:

```
<captures_root>/<tag>_<YYYYmmdd_HHMMSS>/
  manifest.json                 # klippy, written before the first stroke
  step_<step>.scap              # one per step
  step_<step>_accel.csv         # optional, next to its step
  results.json                  # servo-cal analyze
  plot_series.json              # servo-cal analyze
```

`captures_root` default: `~/printer_data/logs/servo_captures`.

## manifest.json (version 1, klippy writes)

```json
{
  "version": 1,
  "experiment": "gain_sweep|refine_sweep|inertia_sweep|accel_sweep|tracking|inertia_grid",
  "tag": "cal",
  "created_utc": "2026-07-10T15:15:16Z",
  "axis": "X",
  "kinematics": "corexy",
  "git_rev": "abc123",
  "session_id": "k-...",
  "stroke_plan": {"start": 30.0, "end": 220.0, "speed": 100.0,
                  "accel": 3000.0, "iterations": 1, "dwell_ms": 700},
  "motors": [{"name": "motor_a", "invert": false,
              "rotation_distance": 40.0, "counts_per_mm": 3276.8}],
  "belts": "motor_a:1+motor_a1:-1,motor_b:-1+motor_b1:-1",
  "steps": [{"name": "p880_s550_i2273",
             "swept": {"position": 880, "speed": 550, "integral": 2273},
             "applied": [{"servo": "motor_a", "addr": "0x2001.0x01",
                          "type": "u16", "value": 880}],
             "capture": "step_p880_s550_i2273.scap",
             "accel": "step_p880_s550_i2273_accel.csv"}],
  "ambient": {
    "journal_params": {"motor_a": {"0x2001.0x31": 2}},
    "param_writes_since_last_run": [
      {"servo": "motor_a", "addr": "0x2001.0x31", "value": 1,
       "time_utc": "2026-07-10T15:14:02Z"}]
  }
}
```

`belts` is null off corexy. `accel` is null when no accelerometer recorded.
`ambient.journal_params` holds the readback of every
`[servo_calibration] journal_params:` address per captured drive, taken at
run start. Missing readback is a hard error, not an omitted key.

## results.json (version 1, servo-cal writes)

```json
{
  "version": 1,
  "fs_hz": 4000.0,
  "settle_band_counts": 50,
  "torque_limit_per_mille": 1400,
  "steps": [{
    "name": "p880_s550_i2273",
    "drives": {"motor_a": {
      "metrics": { /* same shape as scripts/servo_capture.py
                      compute_metrics(): samples, moves[], torque{},
                      torque_saturation_pct, ferr_crosscheck_max,
                      optional ff_*_offset_max */ },
      "psd_peaks": [[freq_hz, power], ...],
      "resonance": {"detected": false, "ratio": 1.9, "peak_hz": 37.5}
    }},
    "combined": {"on_ferr_peak_mm": 0.02, "on_ferr_rms_mm": 0.008,
                 "cross_ferr_peak_mm": 0.01},
    "accel": {"present": true, "psd_peaks": [[freq_hz, power], ...]},
    "flags": ["resonance_detected", "torque_saturated",
              "settle_window_truncated"]
  }],
  "verdict": {
    "recommended_step": "p880_s550_i2273",
    "reason": "highest gain step without resonance or torque rail",
    "flags": [],
    "apply": [{"servo": "motor_a", "addr": "0x2001.0x01",
               "type": "u16", "value": 880}]
  }
}
```

`combined` is null off corexy, `accel` null without a recording.
`verdict.recommended_step` / `apply` are null when no step qualifies —
`reason` then says why. Resonance: moving-segment following-error PSD, power
ratio of the 20–450 Hz band peak to the 1–4 Hz band mean, detected at ratio
≥ 8.0 (ports `scripts/servo_gain_report.py` before its deletion).

## plot_series.json (version 1, servo-cal writes)

Downsampled for drawing, ≤ 2000 points per series (stride, no averaging).

```json
{
  "version": 1,
  "steps": [{
    "name": "p880_s550_i2273",
    "fs_hz": 4000.0,
    "stride": 8,
    "t_s": [0.0, 0.002],
    "moving": [[0.035, 1.119]],
    "drives": {"motor_a": {"ferr_counts": [..], "torque_per_mille": [..]}},
    "combined": {"on_ferr_mm": [..], "cross_ferr_mm": [..]},
    "accel": {"t_s": [..], "magnitude": [..]}
  }]
}
```

## servo-cal CLI

```
servo-cal analyze <run-dir>            # writes results.json + plot_series.json,
                                       # prints the metrics table
servo-cal analyze --scap <file.scap>   # single capture, table to stdout
servo-cal analyze ... --dump-csv PATH  # raw per-drive series as CSV
servo-cal fit ...                      # servo-ident fit, --capture takes .scap
servo-cal serve --dir <captures_root> [--port 8085]
```

Exit non-zero with a one-line reason on any malformed input (fail loud, no
partial results.json). klippy resolves the binary at
`rust/target/release/servo-cal` relative to the repo root, overridable via
`[servo_calibration] servo_cal_binary:`.

`serve` endpoints: `GET /api/runs` (list: name, mtime, experiment, verdict
summary), `GET /api/runs/<name>/manifest|results|plot_series`,
`POST /api/runs/<name>/analyze` (run analyze if results.json missing or
stale), `GET /` static SPA. G-code submission goes browser → Moonraker
(`POST /printer/gcode/script`), not through servo-cal.

## Tuning profile

`~/printer_data/config/servo_tuning/<name>.params` — same line syntax as a
`[motor] params:` block (`parse_params_block`), `#` comment lines carry
provenance (run dir, created_utc, metrics summary). Referenced as
`[motor] tuning_profile: <name>`, pushed at claim time before the `params:`
block; an address present in both is a config error.

## Structured log events (subsystem `calibration`)

- `calibration.run_start` — `run_dir`, `experiment`, `tag`, `axis`
- `calibration.run_done` — `run_dir`, `recommended_step`, `flags`,
  `duration_s`
- `calibration.autotune_stage` — `stage`, `run_dir`, `outcome`
