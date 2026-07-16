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
  "experiment": "gain_sweep|gain_ladder|refine_sweep|inertia_sweep|accel_sweep|tracking|inertia_grid|differential",
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
    "notches": {"motor_a": {
      "mode": 0,
      "notch1": {"freq_hz": 1187, "width": 6, "depth": 8},
      "notch2": {"freq_hz": 4000, "width": 2, "depth": 0},
      "notch3": {"freq_hz": 8000, "width": 0, "depth": 1000},
      "notch4": {"freq_hz": 8000, "width": 0, "depth": 1000},
      "notch5": {"freq_hz": 8000, "width": 0, "depth": 1000}}},
    "param_writes_since_last_run": [
      {"servo": "motor_a", "addr": "0x2001.0x31", "value": 1,
       "time_utc": "2026-07-10T15:14:02Z"}]
  }
}
```

`belts` is null off corexy. `accel` is null when no accelerometer recorded.
A `differential` run's `stroke_plan` instead holds the chirp:
`{"belt": "A", "freq_start": 20.0, "freq_end": 250.0, "hz_per_sec": 5.0,
"duration": 46.0, "ramp": 1.5, "amplitude": 0.05, "dwell_ms": 500}`; `axis`
is the belt letter, `motors` lists the pair in slot order and `belts` is
null.
`ambient.journal_params` holds the readback of every
`[servo_calibration] journal_params:` address per captured drive, taken at
run start. `ambient.notches` is always recorded per captured drive, also at
run start: the adaptive-notch mode (C01.30) and all five notch filters'
center frequency / width / depth (C01.40–4E), read back from the drive so a
run can later answer "what notches were active". Missing readback is a hard
error, not an omitted key. The dashboard's "ambient diff vs previous" column
diffs this block (as `notchN.field: before→after`) alongside
`journal_params`, skipping notch comparison against runs that predate the
block.

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

A `differential` step instead carries an empty `drives` map and a
`differential` block — the anti-phase belt-pair FRF (H1 Welch estimate,
differential commanded position → differential encoder position, drive
signs from the capture header's `invert`):

```json
{"differential": {"pair": ["motor_a", "motor_a1"], "segments": 12,
  "modes": [{"freq_hz": 42.1, "gain": 3.2, "gain_db": 10.1,
             "damping": 0.031, "coherence": 0.94}]}}
```

Modes are strict local maxima of |FRF| inside the commanded band with
coherence ≥ 0.5, deduped within max(3 Hz, 5%), at most 5, sorted by
frequency; `damping` is the half-power ratio (null when a half-power
crossing leaves the estimate). The verdict recommends nothing and lists the
modes in `reason`. Its plot step adds a `differential` block (band-restricted
arrays): `{"freq_hz": [..], "mag_db": [..], "phase_deg": [..],
"coherence": [..], "torque_db": [..], "coherence_min": 0.5,
"band": [20.0, 250.0], "modes": [..]}` — the per-drive `psd`/time series are
computed over the chirp's active span rather than motion-flag segments.

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
    "accel": {"t_s": [..], "magnitude": [..]},
    "psd": {"freq_hz": [..], "per_drive": {"motor_a": [..]},
            "accel": {"freq_hz": [..], "psd": [..]} }
  }]
}
```

`psd` is the full moving-segment following-error Welch PSD per drive — the
same `(freqs, psd)` arrays `resonance` in `results.json` is computed from, not
recomputed or downsampled (`≤ 2000` bins, same cap as the other series but
never stride-thinned since Welch already caps bins around 513). `freq_hz` is
shared by every drive. `psd.accel` is null without an accelerometer
recording, otherwise the accel-magnitude PSD on its own frequency grid.

## servo-cal CLI

```
servo-cal analyze <run-dir>            # writes results.json + plot_series.json,
                                       # prints the metrics table
servo-cal analyze --scap <file.scap>   # single capture, table to stdout
servo-cal analyze ... --dump-csv PATH  # raw per-drive series as CSV
servo-cal fit ...                      # servo-ident fit, --capture takes .scap
servo-cal serve --dir <captures_root> [--port 8085]
                [--live-sock /tmp/kalico-ethercat.sock.live]
```

Exit non-zero with a one-line reason on any malformed input (fail loud, no
partial results.json). klippy resolves the binary at
`rust/target/snapshot/servo-cal` relative to the repo root, overridable via
`[servo_calibration] servo_cal_binary:`.

`serve` endpoints: `GET /api/runs` (list: name, mtime, experiment, verdict
summary), `GET /api/runs/<name>/manifest|results|plot_series`,
`POST /api/runs/<name>/analyze` (run analyze if results.json missing or
stale), `GET /` static SPA. G-code submission goes browser → Moonraker
(`POST /printer/gcode/script`), not through servo-cal.

## Live telemetry tap (ethercat-rt ↔ servo-cal)

The ethercat-rt endpoint binds a second unix socket at
`<control-socket>.live` (default `/tmp/kalico-ethercat.sock.live`,
mode 0666). Per connection it writes one scap-v2 header line — exactly
`capture::header_json`, drives named `slot0..slotN` since motor names
live in klippy config — then streams fixed-size capture records while
the client stays connected. One client at a time; `servo-cal serve` is
that client (`--live-sock`), and fans the data out to browsers over
`/api/live_tap`.

The DC thread pushes records only while a client is connected, through
a bounded preallocated channel; overflow drops records rather than
stalling the cycle or failing the session, and every drop is visible to
the consumer as a `cycle_index` jump (records carry the absolute DC
cycle counter). A reconnect gets a fresh header; `cycle_index` continues
from the same running counter. Consumers must render jumps as gaps.

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
