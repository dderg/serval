# Servo capture fixtures

Real captures from the trident bench (CoreXY AWD, four A6-EC drives at the
4 kHz DC cycle), recorded 2026-07-10 by `SERVO_CALIBRATE_GAINS
SPEED_GAINS=550,700 ITERATIONS=1 AXIS=X` during a notch-tuning session:

- `cal_p880_s550_i2273_*` — the safe gain step (speed gain 550), with its
  accelerometer recording.
- `cal_p1120_s700_i1786_*` — the target gain step (speed gain 700), with its
  accelerometer recording.

Each `.scap` holds one out-and-back stroke pair across all four drives.
Files are gzipped verbatim; `test_servo_capture_goldens.py` decompresses to
a temp file before loading.

`goldens.json` freezes the `scripts/servo_capture.py` analysis of these
captures. It exists to prove output parity when the analysis is ported to
Rust (`servo-cal`, see `docs/plans/servo-calibration-automation.md`) and to
catch unintended changes to the Python pipeline until then. Regenerate after
an intentional metrics change:

```sh
uv run python test/test_servo_capture_goldens.py --regen
```
