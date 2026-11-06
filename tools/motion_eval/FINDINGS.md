# SCV + input shaper: measured (offline, motion_eval)

## Conclusion

1. **SCV + IS is not ideal.** With input shaping on, the shaper rounds every
   corner itself. A full stop at max accel is still rounded by about `σ²·a/2`,
   which is 0.16–0.21 mm on the bench config and 0.25 mm on the fast one; at
   scv 1 we measure 0.16–0.19 mm and 0.23–0.26 mm. SCV's own geometry, the
   junction deviation `scv²(√2−1)/a`, is 0.001 mm at scv 5 and 10 000 mm/s²,
   about 100× smaller. So in practice SCV is an angle-to-speed rule that
   ignores the shaper. On curved, dense geometry, scv 5 slows corners that end
   up rounded far less (p50 0.03–0.05 mm) than the stop corners beside them
   (≈0.19 mm).
2. **The junction limit is the knob that costs time.** Removing the SCV term
   saves this much (numbers are corpus total / voron top layers / circles /
   zigzag infill):
   - bench: 10% / 37% / 44% / 23%
   - fast: 7% / 49% / 35% / 22%

   The other limits cost little:
   - `minimum_cruise_ratio` costs 1% of corpus time (at most 6%, on zigzag).
   - Extruder `instantaneous_corner_velocity` costs under 1%.
   - Accel ramps that remain once all corner limits are gone cost 6% (bench)
     and 2% (fast).
3. **The shaper-aware corner limit (harness prototype) is faster at equal
   deviation, but only on curved or dense geometry.** The prototype sets the
   junction speed so that the predicted *shaped* corner deviation stays within
   a budget, and caps speed on curves at `σ²v²κ/2 ≤ budget`. Results with
   corner p99 no worse than the default (scv 5, mcr 0.5):
   - circles: −38% (bench), −23% (fast)
   - voron top layers: −25 to −26% (bench), −16% (fast, smooth_mzv)
   - isolated corners and first-layer perimeters: −2 to +0.2%; these are
     dominated by the shaper's rounding at stops (the floor), which no junction
     speed changes.
   - one regression: fast + mzv on voron top layers, +9.6%. The predictor
     underestimates the deviation of dense corner sequences at 30 000 mm/s², so
     it needs a low budget there.

   The best single setting for the whole corpus (p99 ≤ default in every case)
   gives −3.5% (bench, mzv), −3.7% (bench, smooth_mzv), −1.3% (fast,
   smooth_mzv) and +1.7% (fast, mzv). The corpus total is dominated by the
   70 s first-layer slice, which barely moves.
4. **Side effects.** Faster corners raise the instantaneous velocity steps
   that mzv lets through: 3.7 mm/s at the default against about 9 mm/s at an
   equal-deviation budget. That is harmless for the tuned mode, but it is a
   risk for torque and for off-frequency modes. Smoothers let no steps through,
   but executed peak accel rises above the configured max as junction speed
   grows: on bench, 10.5–10.6k mm/s² at equal-deviation budgets and 13.6k at
   scv 35, against 10k configured.
5. **Next.** Replace the SCV junction term with a shaped-deviation budget
   (vertex term + curve term), with the default budget near the shaper floor.
   Before that, fix prediction for corner sequences shorter than the kernel.
   Leave `minimum_cruise_ratio` alone. Ringing from the larger mzv velocity
   steps can only be judged on hardware.

## Setup

- **Harness**: unmodified klippy in batch mode. The executed motion is a model
  of `kin_shaper.c`, and every decoded MCU step is checked against it; in all
  2232 runs the worst step used 99.7% of its allowed error. Corner deviation
  is measured per vertex as the larger of two distances: from the shaped path
  to the commanded legs, and from the vertex to the shaped path.
- **bench** (`configs/bench.cfg`): corexy, 10 000 mm/s², 500 mm/s, mzv at
  50 Hz X / 44 Hz Y. This mirrors the project bench printer: 10 000 mm/s²,
  50/44 Hz per the serval notes. There Y uses ZV; here both axes use one
  family so the families can be compared.
- **fast** (`configs/fast.cfg`): 30 000 mm/s², 800 mm/s, smooth_mzv at 70 Hz.
  It stresses the high-accel end the task asked for (30 000+), with a stiff,
  light-toolhead resonance and Kalico's smoother family.
- **Corpus** (`corpus/`, built by `corpus/build_corpus.py`):
  - 1400-command slices of voron first layers (perimeters, slow) and voron top
    layers (dense), from serval `tools/sim/gcode`.
  - circles of r 1.5–12 mm drawn with 0.0125 mm chord error at 200 mm/s.
  - G2/G3 arcs (r 2 and 5 mm).
  - zigzag infill (2/4/8 mm lines).
  - zigzags with 45/90/135/170° turns at 50/150/300 mm/s.
- **Knobs swept**:
  - scv 1–35
  - mcr 0 / 0.25 / 0.5
  - shaper family mzv and smooth_mzv, at the config's frequencies
  - corner budget 0.03–0.4 mm (prototype, `shaper_corner.py`)

## Shaper floor: a stop at a 90°/170° corner (scv 1)

| config | shaper | σ ms (X/Y) | σ²·a/2 mm (X/Y) | measured p99 mm, 90° / 170° |
|---|---|---|---|---|
| bench | mzv | 5.71 / 6.49 | 0.163 / 0.210 | 0.185 / 0.164 |
| bench | smooth_mzv | 5.81 / 6.60 | 0.169 / 0.218 | 0.191 / 0.170 |
| fast | mzv | 4.08 | 0.249 | 0.231 / 0.249 |
| fast | smooth_mzv | 4.15 | 0.258 | 0.239 / 0.258 |

## Where the time goes (default shaper, scv 5, mcr 0.5)

Each column is the saving from removing one limit: scv→∞, mcr→0, both, and
then also extruder corner velocity→∞. "Accel ramps" is what remains above
feed-rate time once all of those are gone.

| config | case | base s | corner limit | min_cruise_ratio | both | extruder corner | accel ramps | below cruise |
|---|---|---|---|---|---|---|---|---|
| bench | circles | 5.37 | 2.37 (44%) | 0.19 (4%) | 2.38 (44%) | 0.03 (1%) | 0.19 (4%) | 49% |
| bench | voron_top_layers | 8.72 | 3.21 (37%) | 0.30 (3%) | 3.31 (38%) | 0.05 (1%) | 3.08 (35%) | 54% |
| bench | zigzag_infill | 5.35 | 1.22 (23%) | 0.30 (6%) | 1.45 (27%) | 0.05 (1%) | 2.10 (39%) | 64% |
| bench | voron_first_layer_perimeters | 74.69 | 4.38 (6%) | 0.25 (0%) | 4.45 (6%) | 0.04 (0%) | 1.61 (2%) | 15% |
| bench | corners_turn90 | 6.35 | 0.35 (5%) | 0.00 (0%) | 0.35 (5%) | 0.05 (1%) | 0.03 (0%) | 14% |
| bench | **corpus sum** | 119.29 | 12.44 (10%) | 1.05 (1%) | 12.85 (11%) | 0.35 (0%) | 7.25 (6%) | |
| fast | circles | 4.31 | 1.49 (35%) | 0.20 (5%) | 1.49 (35%) | 0.02 (0%) | 0.02 (0%) | 49% |
| fast | voron_top_layers | 6.53 | 3.20 (49%) | 0.28 (4%) | 3.25 (50%) | 0.05 (1%) | 1.32 (20%) | 55% |
| fast | zigzag_infill | 3.27 | 0.73 (22%) | 0.11 (3%) | 0.80 (24%) | 0.02 (1%) | 0.70 (21%) | 58% |
| fast | voron_first_layer_perimeters | 70.51 | 1.79 (3%) | 0.05 (0%) | 1.79 (3%) | 0.03 (0%) | 0.26 (0%) | 6% |
| fast | **corpus sum** | 108.63 | 7.66 (7%) | 0.64 (1%) | 7.78 (7%) | 0.19 (0%) | 2.35 (2%) | |

## SCV sweep (mcr 0.5): corpus time vs worst per-case corner deviation

Time is the mean over cases relative to scv 5. Deviation is the largest
per-case p99 / max; it is set by the floor cases, so it hides the gains on
curved cases (see the per-case tables).

| scv | bench mzv time | p99 / max mm | max jump mm/s | fast smooth_mzv time | p99 / max mm | peak accel |
|---|---|---|---|---|---|---|
| 1 | 1.295 | 0.185 / 0.194 | 0.7 | 1.202 | 0.258 / 0.258 | 30004 |
| 3 | 1.074 | 0.189 / 0.198 | 2.2 | 1.051 | 0.260 / 0.260 | 30052 |
| 5 | 1.000 | 0.208 / 0.208 | 3.7 | 1.000 | 0.261 / 0.261 | 30125 |
| 8 | 0.945 | 0.208 / 0.208 | 5.9 | 0.958 | 0.263 / 0.263 | 30310 |
| 12 | 0.912 | 0.210 / 0.218 | 8.9 | 0.928 | 0.266 / 0.267 | 30610 |
| 20 | 0.893 | 0.230 / 0.238 | 13.0 | 0.904 | 0.273 / 0.275 | 31494 |
| 35 | 0.879 | 0.276 / 0.285 | 22.7 | 0.892 | 0.296 / 0.296 | 33403 |

minimum_cruise_ratio at scv 5 (bench, mzv): time 1.000 (mcr 0.5), 0.988
(0.25), 0.985 (0). p99 is 0.208–0.209 for all three.

Per case, scv 1 → 5 → 20 (mcr 0.5):

| config | case | s | p50 mm | p99 mm |
|---|---|---|---|---|
| bench | voron_top_layers | 14.83 → 8.72 → 5.54 | 0.015 → 0.050 → 0.186 | 0.102 → 0.123 → 0.218 |
| bench | circles | 9.21 → 5.37 → 3.02 | 0.014 → 0.031 → 0.137 | 0.110 → 0.129 → 0.229 |
| fast | voron_top_layers | 9.36 → 6.53 → 3.58 | 0.018 → 0.034 → 0.198 | 0.121 → 0.138 → 0.256 |
| fast | circles | 5.92 → 4.31 → 2.86 | 0.017 → 0.025 → 0.066 | 0.137 → 0.149 → 0.211 |

## Shaper-aware budget vs SCV at equal deviation

The fastest setting with corner p99 ≤ the default's p99 in every case. One
setting is used for the whole corpus:

| config | shaper | default corpus s | best SCV setting | budget setting | s | vs default |
|---|---|---|---|---|---|---|
| bench | mzv | 119.3 | scv 5, mcr 0.5 (the default) | budget 0.09, mcr 0 | 115.1 | −3.5% |
| bench | smooth_mzv | 119.3 | scv 5, mcr 0.5 | budget 0.1, mcr 0.25 | 114.8 | −3.7% |
| fast | mzv | 108.6 | scv 5, mcr 0.5 | budget 0.03, mcr 0.5 | 110.5 | +1.7% |
| fast | smooth_mzv | 108.6 | scv 5, mcr 0.5 | budget 0.05, mcr 0.5 | 107.2 | −1.3% |

Per case, with the best setting chosen separately for each case. No SCV/mcr
setting beats the default except on zigzag, where scv 1–2 with mcr 0 is
2.4% faster:

| config | shaper | case | default s / p99 mm | budget setting | s | vs default |
|---|---|---|---|---|---|---|
| bench | mzv | circles | 5.37 / 0.129 | 0.1, mcr 0.5 | 3.32 | −38.2% |
| bench | mzv | voron_top_layers | 8.72 / 0.123 | 0.09, mcr 0 | 6.53 | −25.1% |
| bench | mzv | voron_first_layer_perimeters | 74.69 / 0.190 | 0.15, mcr 0 | 73.20 | −2.0% |
| bench | mzv | zigzag_infill | 5.35 / 0.137 | 0.08, mcr 0 | 5.22 | −2.4% (scv 1 mcr 0: 5.20) |
| bench | mzv | corners_turn90 | 6.35 / 0.194 | 0.15, mcr 0.25 | 6.36 | +0.0% |
| bench | smooth_mzv | circles | 5.37 / 0.133 | 0.1, mcr 0 | 3.31 | −38.4% |
| bench | smooth_mzv | voron_top_layers | 8.72 / 0.128 | 0.1, mcr 0.25 | 6.45 | −26.0% |
| fast | mzv | circles | 4.31 / 0.145 | 0.05, mcr 0.5 | 3.31 | −23.1% |
| fast | mzv | voron_top_layers | 6.53 / 0.133 | 0.03, mcr 0 | 7.16 | +9.6% |
| fast | smooth_mzv | circles | 4.31 / 0.149 | 0.05, mcr 0 | 3.31 | −23.3% |
| fast | smooth_mzv | voron_top_layers | 6.53 / 0.138 | 0.05, mcr 0.5 | 5.48 | −16.2% |
| fast | smooth_mzv | zigzag_infill | 3.27 / 0.168 | 0.08, mcr 0 | 3.22 | −1.5% |

### Pareto front, voron_top_layers, bench mzv (SCV and budget runs)

Default for reference: scv 5, mcr 0.5 → 8.72 s, p99 0.123, max jump 3.7.
Every SCV point is dominated except scv 35.

| setting | s | p50 mm | p99 mm | max mm | max jump mm/s |
|---|---|---|---|---|---|
| budget 0.03, mcr 0.5 | 10.97 | 0.027 | 0.100 | 0.120 | 3.7 |
| budget 0.05, mcr 0.5 | 8.04 | 0.059 | 0.108 | 0.125 | 6.0 |
| budget 0.07, mcr 0.5 | 7.12 | 0.081 | 0.110 | 0.127 | 7.4 |
| budget 0.09, mcr 0.5 | 6.67 | 0.100 | 0.117 | 0.137 | 8.8 |
| budget 0.1, mcr 0.5 | 6.50 | 0.109 | 0.124 | 0.141 | 9.6 |
| budget 0.12, mcr 0.5 | 6.21 | 0.128 | 0.142 | 0.144 | 10.1 |
| budget 0.15, mcr 0.5 | 5.88 | 0.153 | 0.169 | 0.170 | 10.9 |
| budget 0.2, mcr 0.5 | 5.55 | 0.183 | 0.215 | 0.216 | 14.7 |
| scv 35, mcr 0.25 | 5.43 | 0.190 | 0.221 | 0.245 | 17.5 |

### Pareto front, voron_top_layers, fast smooth_mzv

Default: scv 5, mcr 0.5 → 6.53 s, p99 0.138.

| setting | s | p50 mm | p99 mm | max mm | peak accel |
|---|---|---|---|---|---|
| budget 0.03, mcr 0.5 | 7.70 | 0.025 | 0.120 | 0.158 | 30002 |
| budget 0.05, mcr 0.5 | 5.48 | 0.057 | 0.134 | 0.159 | 30002 |
| budget 0.07, mcr 0.5 | 4.53 | 0.087 | 0.146 | 0.166 | 30002 |
| budget 0.1, mcr 0.5 | 4.07 | 0.119 | 0.149 | 0.187 | 30002 |
| budget 0.15, mcr 0.5 | 3.70 | 0.165 | 0.178 | 0.200 | 30002 |
| budget 0.2, mcr 0.5 | 3.46 | 0.209 | 0.221 | 0.223 | 31059 |
| budget 0.3, mcr 0 | 3.29 | 0.247 | 0.268 | 0.308 | 34712 |

## Caveats

- Everything here is offline: geometric deviation of the commanded stepper
  motion. Frame dynamics, ringing, extrusion and pressure advance are not
  modelled. The velocity steps are only a proxy for ringing.
- The prototype's prediction is per vertex (decelerate at full accel into
  the corner, accelerate out) plus a curvature cap. It misses runs of corners
  shorter than the kernel, so it exceeds its budget on dense geometry, worst
  at 30 000 mm/s². Results are compared at *measured* deviation, so this
  costs speed but does not flatter the prototype.
- Rest stops still round at the floor. Like SCV, the budget limit cannot
  change that without slowing the approach (serval did that).

## Upstream issues found on the way

- `stepcompress.c` `queue_append_far()` wrote past the step queue when a far
  step arrived with the queue full. The result was heap corruption ("realloc():
  invalid next size") in batch mode at 520 MHz. Klipper master has the same
  code. Fixed in a36285d01, with the regression case
  `test/klippy/stepcompress_far_step.test`.
- Batch mode (`-o`) can exit with output still queued in the serial queue,
  silently truncating the stream under load. `capture.py` drains the queue
  before disconnect.

## Reproduce

```
python3 ~/Developer/serval/scripts/slots.py run -- \
    uv run python -m tools.motion_eval.sweep --jobs 12   # 2232 runs, ~8 min
# tables: tools/motion_eval/.build/sweep/tables.md (all sweeps, all fronts)
```
