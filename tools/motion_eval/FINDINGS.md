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
3. **The product `[corner_deviation] max_deviation` holds its budget on the
   whole corpus, and is faster on curved geometry at equal worst-case
   deviation.** Over 1296 budget runs (9 cases × 2 configs × mzv/smooth_mzv
   × 12 budgets × 3 mcr), the measured corner maximum never exceeds the
   budget by more than 1.9e-6 mm, which is below the 50 µs sampling
   tolerance. Compared at equal *maximum* deviation (the fastest budget
   whose max ≤ the default's max in that case):
   - circles: −33% (bench mzv), −18% (bench smooth_mzv), −27/−25% (fast)
   - voron top layers: −25% (fast, both shapers), −3% (bench)
   - isolated corners, first-layer perimeters, arcs on mzv: −2.4 to +1.4%
   - slower: zigzag infill +5 to +15%, arcs on smoothers +11% (fast) and
     +48% (bench)

   One budget for the whole corpus (max ≤ default in every case) gives
   −0.4% (bench mzv), +3.0% (bench smooth_mzv), +0.3% (fast mzv) and
   −1.3% (fast smooth_mzv). The 70 s first-layer slice dominates the total.
4. **Side effects.** With mzv, faster corners let larger velocity steps
   through: 3.7 mm/s at the default against 10–30 mm/s at an
   equal-deviation budget. That is harmless for the tuned mode, but it is a
   risk for torque and for off-frequency modes. With smoothers, the limiter
   bounds executed peak accel to max_accel; that cap is the likely cause of
   the slower arcs [INFERENCE].
   The bound held in 647 of 648 smoother runs (one run reached 10 080
   against 10 000 mm/s²).
5. **Planner CPU: the <20% target is not met.** The Python limiter adds
   about 18 µs per move to upstream's 4.4 µs lookahead (+424%) on this
   host. In absolute terms that is a small share of one core even for dense
   G-code, but a Pi 5 will be slower [INFERENCE]. Reaching the target needs
   the limiter in C or Rust.
6. **Next.** Ringing from the larger mzv velocity steps can only be judged
   on hardware. Bench mzv voron top layers gains only 3%: the per-junction
   bound sums neighbour contributions conservatively, so the measured
   maximum sits well below the budget there (budget 0.15 measures 0.133).

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

## `[corner_deviation]` vs SCV at equal maximum deviation

The product limiter (`klippy/extras/corner_deviation.py`, hooked into
`Move.calc_junction` through `ToolHead.set_junction_limiter`) replaces the SCV
junction term. Budget runs use `[corner_deviation] max_deviation` and no
SCV. Each case uses the fastest budget/mcr setting whose measured corner
**max** is ≤ the default's (scv 5, mcr 0.5) max in that case.

| config | shaper | default s | best single budget (all cases) | s | vs default |
|---|---|---|---|---|---|
| bench | mzv | 119.3 | 0.15, mcr 0.5 | 118.8 | −0.4% |
| bench | smooth_mzv | 119.3 | 0.15, mcr 0 | 122.9 | +3.0% |
| fast | mzv | 108.6 | 0.15, mcr 0 | 108.9 | +0.3% |
| fast | smooth_mzv | 108.6 | 0.2, mcr 0.25 | 107.2 | −1.3% |

Per case (max step: largest velocity step mzv lets through; smoothers let
none through):

| config | shaper | case | default s / max mm | budget, mcr | s | vs default | max step mm/s default / budget |
|---|---|---|---|---|---|---|---|
| bench | mzv | circles | 5.37 / 0.152 | 0.15, 0 | 3.61 | −32.8% | 3.7 / 10.3 |
| bench | mzv | voron_top_layers | 8.72 / 0.135 | 0.15, 0.5 | 8.49 | −2.6% | 3.7 / 9.9 |
| bench | mzv | voron_first_layer_perimeters | 74.69 / 0.201 | 0.2, 0 | 72.90 | −2.4% | 3.7 / 21.5 |
| bench | mzv | zigzag_infill | 5.35 / 0.149 | 0.15, 0 | 5.44 | +5.5% (scv 2 mcr 0: −3.6%) | 3.6 / 5.0 |
| bench | mzv | corners_turn90 | 6.35 / 0.201 | 0.2, 0 | 6.31 | −0.7% | 2.0 / 6.7 |
| bench | mzv | arcs | 0.91 / 0.208 | 0.4, 0.5 | 0.91 | +0.0% | 2.9 / 2.9 |
| bench | smooth_mzv | circles | 5.37 / 0.156 | 0.15, 0 | 4.40 | −18.0% | |
| bench | smooth_mzv | voron_top_layers | 8.72 / 0.136 | 0.2, 0 | 8.42 | −3.5% | |
| bench | smooth_mzv | zigzag_infill | 5.35 / 0.152 | 0.2, 0 | 5.62 | +9.8% | |
| bench | smooth_mzv | arcs | 0.91 / 0.215 | 0.25, 0 | 1.34 | +48.2% | |
| fast | mzv | circles | 4.31 / 0.192 | 0.15, 0 | 3.17 | −26.5% | 3.7 / 14.7 |
| fast | mzv | voron_top_layers | 6.53 / 0.164 | 0.2, 0 | 4.91 | −24.8% | 3.7 / 17.9 |
| fast | mzv | voron_first_layer_perimeters | 70.51 / 0.234 | 0.25, 0.25 | 69.99 | −0.7% | 3.7 / 29.6 |
| fast | mzv | zigzag_infill | 3.27 / 0.167 | 0.2, 0.5 | 3.43 | +6.2% | 3.6 / 5.0 |
| fast | smooth_mzv | circles | 4.31 / 0.198 | 0.2, 0 | 3.23 | −25.1% | |
| fast | smooth_mzv | voron_top_layers | 6.53 / 0.166 | 0.25, 0 | 4.93 | −24.6% | |
| fast | smooth_mzv | zigzag_infill | 3.27 / 0.170 | 0.2, 0.25 | 3.68 | +14.5% | |
| fast | smooth_mzv | arcs | 0.88 / 0.096 | 0.15, 0 | 0.98 | +11.1% | |

Isolated corners (turn 45/90/135/170°) are within −0.9 to +1.4% everywhere.
The full table, including peak accel, is in `.build/sweep/tables.md`.

### Physical invariants (1296 budget runs, all cases)

| invariant | result |
|---|---|
| measured corner max ≤ max_deviation | worst excess 1.9e-6 mm, below ½·max_accel·(50 µs)² (12.5e-6 mm at 10k) |
| smoother peak executed accel ≤ max_accel | 647 of 648 runs; voron_top_layers bench smooth_mzv 0.12 mcr 0 reached 10 080 |
| no limiter registered: trapq identical to upstream toolhead | 9 cases × 2 configs, `tests/test_junction_hook.py` |

`tests/test_corner_deviation.py` asserts the first two invariants on the
corner, circle, zigzag and top-layer cases for both configs and shaper
families at budgets 0.05 and 0.2.

### Planner CPU (this host, median per run)

| config | lookahead µs/move, scv 5 | with limiter | of which limiter | added |
|---|---|---|---|---|
| bench | 4.4 | 23.1 | 18.0 | +424% |
| fast | 4.4 | 23.5 | 18.2 | +435% |

The <20% target is not met in Python.

### Pareto front, voron_top_layers, bench mzv

Default: scv 5, mcr 0.5 → 8.72 s, max 0.135 (dominated by budget 0.15).

| setting | s | p99 mm | max mm | max step mm/s |
|---|---|---|---|---|
| budget 0.1, mcr 0.25 | 10.48 | 0.067 | 0.068 | 7.7 |
| budget 0.12, mcr 0.25 | 9.19 | 0.082 | 0.109 | 8.7 |
| budget 0.15, mcr 0.5 | 8.49 | 0.112 | 0.133 | 9.9 |
| budget 0.2, mcr 0 | 7.71 | 0.143 | 0.146 | 18.5 |
| scv 8, mcr 0.25 | 6.79 | 0.166 | 0.167 | 5.9 |
| budget 0.25, mcr 0.25 | 6.28 | 0.200 | 0.202 | 20.1 |
| scv 12, mcr 0.25 | 5.81 | 0.210 | 0.210 | 8.9 |
| scv 35, mcr 0.25 | 5.43 | 0.221 | 0.245 | 17.5 |

### Pareto front, voron_top_layers, fast smooth_mzv

Default: scv 5, mcr 0.5 → 6.53 s, max 0.166.

| setting | s | p99 mm | max mm | peak accel |
|---|---|---|---|---|
| budget 0.1, mcr 0 | 6.90 | 0.057 | 0.062 | 29889 |
| budget 0.15, mcr 0.25 | 5.96 | 0.094 | 0.098 | 29429 |
| budget 0.2, mcr 0.25 | 5.53 | 0.127 | 0.142 | 29431 |
| budget 0.25, mcr 0.25 | 4.95 | 0.135 | 0.161 | 29409 |
| scv 12, mcr 0.25 | 4.34 | 0.169 | 0.201 | 30644 |
| scv 35, mcr 0.25 | 3.31 | 0.267 | 0.269 | 33423 |

## Caveats

- Everything here is offline: geometric deviation of the commanded stepper
  motion. Frame dynamics, ringing, extrusion and pressure advance are not
  modelled. The velocity steps are only a proxy for ringing.
- The limiter's bound is conservative per junction: it books neighbour
  steps and stop floors as absolute sums, so the measured max often sits
  below the budget, which costs speed on dense geometry.
- Rest stops still round at the floor. Like SCV, the budget limit cannot
  change that without lowering accel; below the floor it does lower accel
  around the corner, which is what costs time at small budgets.

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
    uv run python -m tools.motion_eval.sweep --jobs 12   # 2232 runs, ~10 min
# tables: tools/motion_eval/.build/sweep/tables.md (all sweeps, all fronts)
```
