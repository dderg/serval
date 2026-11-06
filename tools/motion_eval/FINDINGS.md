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
3. **The floor-referenced `[corner_deviation]` is faster on the whole
   corpus.** Each junction runs as fast as keeps its isolated, shaped corner
   within the shaper floor for that corner (`|σ²·a|/2` with that corner's
   per-axis accelerations): a corner never rounds more than a stop there
   already can. Acceleration is never lowered. Corpus time against the
   default (scv 5, mcr 0.5): −7.4% (bench mzv), −3.8% (bench smooth_mzv),
   −6.1% (fast mzv), −4.6% (fast smooth_mzv). Dense geometry gains most:
   circles −31 to −43%, voron top layers −23 to −49%, zigzag infill −1 to
   −17%. It rounds more than the default in most cases (the default's
   worst corner is below the floor), up to the floor and never above it.
4. **At equal worst-case or equal p99 deviation it is not slower except in
   three places.** With `max_deviation` lowering junction speeds only, and
   the fastest of floor and caps 0.05–0.25 chosen per case:
   - equal max: corpus −4.5 / −2.6 / −3.6 / −2.9%. Slower: arcs on bench
     (+2.5% mzv, +2.6% smooth_mzv; the default rounds those arcs 0.21 mm,
     more than their floor of 0.16–0.17 mm, which the cap cannot exceed),
     fast smooth_mzv zigzag infill (+0.6%), isolated 135/170° corners
     +0.1% at most (cap granularity).
   - equal p99: corpus −5.1 / −3.5 / −1.3 / −2.7%. Slower, besides the arcs:
     fast mzv voron top layers (+1.8%), bench smooth_mzv zigzag (+0.7%),
     fast smooth_mzv zigzag (+0.6%). In these, stop corners already sit at
     the default's p99; once the neighbouring junctions are faster, the
     moves into those stops decelerate over the whole kernel and reach
     their full floor, so a cap has to slow everything else.

   One cap for the whole corpus (max ≤ default in every case) is 0.07 mm:
   −0.8 / −0.3 / −1.9 / −1.7%.
5. **Side effects.** mzv passes larger velocity steps: up to 47 mm/s in
   floor mode (fast, 45° corners) against 3.7 mm/s at the default. That is
   harmless for the tuned mode but a risk for torque and off-frequency
   modes; only hardware can tell. With smoothers the executed peak
   acceleration stays within 3.3% above max_accel (the default reaches
   3.3% too); the limiter's stated allowance is 5%.
6. **Planner CPU.** On the densest case (arcs, ~1 450 moves per second of
   print) the limiter costs 3.2–3.7 µs per move on this host: 0.5% of one
   core, about 1.3% of a Pi 5 core at an assumed 2.5× slowdown. Lookahead
   per move goes from 3.8–4.3 µs (scv 5) to 6.5–6.9 µs.

## Setup

- **Harness**: unmodified klippy in batch mode. The executed motion is a model
  of `kin_shaper.c`, and every decoded MCU step is checked against it; in all
  1476 runs the worst step used 99.7% of its allowed error. Corner deviation
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
  - `[corner_deviation]` floor mode and caps 0.05–0.25 mm, at mcr 0.5

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

## `[corner_deviation]`: floor mode and floor+cap against the default

`klippy/extras/corner_deviation.py`, hooked into `Move.calc_junction` through
`ToolHead.set_junction_limiter`, replaces the SCV junction term. Floor runs
have the section with no option; cap runs add `max_deviation`. All at
mcr 0.5, against the default scv 5, mcr 0.5. "Equal max" and "equal p99" take
the fastest of floor and every cap whose corner max (or p99) is within the
default's in that case.

| config | shaper | default s | floor s | vs default | per case, equal max | vs default | per case, equal p99 | vs default | one cap, equal max | s | vs default |
|---|---|---|---|---|---|---|---|---|---|---|---|
| bench | mzv | 119.3 | 110.5 | −7.4% | 113.9 | −4.5% | 113.2 | −5.1% | 0.07 | 118.4 | −0.8% |
| bench | smooth_mzv | 119.3 | 114.7 | −3.8% | 116.2 | −2.6% | 115.1 | −3.5% | 0.07 | 119.0 | −0.3% |
| fast | mzv | 108.6 | 102.0 | −6.1% | 104.8 | −3.6% | 107.2 | −1.3% | 0.07 | 106.6 | −1.9% |
| fast | smooth_mzv | 108.6 | 103.7 | −4.6% | 105.4 | −2.9% | 105.7 | −2.7% | 0.07 | 106.8 | −1.7% |

| config | shaper | case | default s (max / p99 mm) | floor s (max / p99 mm) | floor vs default | equal max | vs default | equal p99 | vs default |
|---|---|---|---|---|---|---|---|---|---|
| bench | mzv | arcs | 0.91 (0.208 / 0.208) | 0.93 (0.163 / 0.163) | +2.5% | floor: 0.93 s | +2.5% | floor: 0.93 s | +2.5% |
| bench | mzv | circles | 5.37 (0.152 / 0.129) | 3.06 (0.246 / 0.222) | −43.1% | cap 0.14: 3.18 s | −40.8% | cap 0.12: 3.28 s | −39.0% |
| bench | mzv | corners_turn135 | 5.84 (0.189 / 0.184) | 5.82 (0.224 / 0.224) | −0.4% | cap 0.18: 5.84 s | +0.0% | cap 0.18: 5.84 s | +0.0% |
| bench | mzv | corners_turn170 | 5.38 (0.167 / 0.167) | 5.37 (0.249 / 0.248) | −0.3% | cap 0.16: 5.38 s | +0.1% | cap 0.16: 5.38 s | +0.1% |
| bench | mzv | corners_turn45 | 6.68 (0.183 / 0.180) | 6.54 (0.220 / 0.220) | −2.1% | cap 0.18: 6.58 s | −1.5% | cap 0.18: 6.58 s | −1.5% |
| bench | mzv | corners_turn90 | 6.35 (0.201 / 0.194) | 6.26 (0.245 / 0.245) | −1.5% | cap 0.2: 6.31 s | −0.7% | cap 0.2: 6.31 s | −0.7% |
| bench | mzv | voron_first_layer_perimeters | 74.69 (0.201 / 0.190) | 72.25 (0.238 / 0.229) | −3.3% | cap 0.2: 72.54 s | −2.9% | cap 0.18: 72.91 s | −2.4% |
| bench | mzv | voron_top_layers | 8.72 (0.135 / 0.123) | 5.70 (0.233 / 0.173) | −34.6% | cap 0.07: 7.79 s | −10.7% | cap 0.11: 6.59 s | −24.5% |
| bench | mzv | zigzag_infill | 5.35 (0.149 / 0.137) | 4.56 (0.193 / 0.193) | −14.8% | cap 0.16: 5.33 s | −0.3% | cap 0.16: 5.33 s | −0.3% |
| bench | smooth_mzv | arcs | 0.91 (0.215 / 0.215) | 0.93 (0.169 / 0.169) | +2.6% | floor: 0.93 s | +2.6% | floor: 0.93 s | +2.6% |
| bench | smooth_mzv | circles | 5.37 (0.156 / 0.133) | 3.72 (0.162 / 0.151) | −30.8% | cap 0.15: 3.72 s | −30.7% | cap 0.14: 3.73 s | −30.6% |
| bench | smooth_mzv | corners_turn135 | 5.84 (0.196 / 0.191) | 5.84 (0.213 / 0.205) | −0.0% | cap 0.18: 5.85 s | +0.1% | cap 0.18: 5.85 s | +0.1% |
| bench | smooth_mzv | corners_turn170 | 5.38 (0.174 / 0.174) | 5.38 (0.173 / 0.172) | +0.0% | cap 0.18: 5.38 s | +0.0% | cap 0.18: 5.38 s | +0.0% |
| bench | smooth_mzv | corners_turn45 | 6.68 (0.190 / 0.187) | 6.66 (0.206 / 0.199) | −0.2% | cap 0.2: 6.66 s | −0.2% | cap 0.2: 6.66 s | −0.2% |
| bench | smooth_mzv | corners_turn90 | 6.35 (0.209 / 0.201) | 6.35 (0.209 / 0.202) | −0.1% | cap 0.2: 6.35 s | −0.1% | cap 0.2: 6.35 s | −0.1% |
| bench | smooth_mzv | voron_first_layer_perimeters | 74.69 (0.206 / 0.195) | 73.87 (0.223 / 0.199) | −1.1% | cap 0.2: 73.95 s | −1.0% | cap 0.2: 73.95 s | −1.0% |
| bench | smooth_mzv | voron_top_layers | 8.72 (0.136 / 0.128) | 6.70 (0.168 / 0.167) | −23.1% | cap 0.07: 8.05 s | −7.7% | cap 0.13: 6.89 s | −21.0% |
| bench | smooth_mzv | zigzag_infill | 5.35 (0.152 / 0.141) | 5.28 (0.154 / 0.145) | −1.3% | cap 0.2: 5.28 s | −1.3% | cap 0.16: 5.38 s | +0.7% |
| fast | mzv | arcs | 0.88 (0.093 / 0.093) | 0.88 (0.093 / 0.093) | +0.0% | cap 0.1: 0.88 s | +0.0% | cap 0.1: 0.88 s | +0.0% |
| fast | mzv | circles | 4.31 (0.192 / 0.145) | 2.82 (0.258 / 0.228) | −34.6% | cap 0.18: 2.85 s | −33.9% | cap 0.05: 3.64 s | −15.5% |
| fast | mzv | corners_turn135 | 5.56 (0.238 / 0.238) | 5.54 (0.296 / 0.296) | −0.3% | cap 0.22: 5.56 s | +0.1% | cap 0.22: 5.56 s | +0.1% |
| fast | mzv | corners_turn170 | 5.09 (0.252 / 0.252) | 5.08 (0.258 / 0.257) | −0.1% | cap 0.25: 5.08 s | −0.1% | cap 0.25: 5.08 s | −0.1% |
| fast | mzv | corners_turn45 | 6.41 (0.250 / 0.248) | 6.33 (0.301 / 0.301) | −1.3% | cap 0.22: 6.35 s | −0.9% | cap 0.22: 6.35 s | −0.9% |
| fast | mzv | corners_turn90 | 6.07 (0.238 / 0.237) | 6.01 (0.341 / 0.341) | −1.0% | cap 0.22: 6.06 s | −0.2% | cap 0.22: 6.06 s | −0.2% |
| fast | mzv | voron_first_layer_perimeters | 70.51 (0.234 / 0.204) | 69.33 (0.240 / 0.239) | −1.7% | cap 0.25: 69.49 s | −1.4% | cap 0.22: 69.73 s | −1.1% |
| fast | mzv | voron_top_layers | 6.53 (0.164 / 0.133) | 3.35 (0.316 / 0.249) | −48.7% | cap 0.07: 5.23 s | −19.9% | cap 0.05: 6.65 s | +1.8% |
| fast | mzv | zigzag_infill | 3.27 (0.167 / 0.164) | 2.71 (0.271 / 0.271) | −17.3% | cap 0.2: 3.26 s | −0.4% | cap 0.2: 3.26 s | −0.4% |
| fast | smooth_mzv | arcs | 0.88 (0.096 / 0.096) | 0.88 (0.096 / 0.096) | +0.0% | cap 0.1: 0.88 s | +0.0% | cap 0.1: 0.88 s | +0.0% |
| fast | smooth_mzv | circles | 4.31 (0.198 / 0.149) | 2.92 (0.221 / 0.193) | −32.2% | cap 0.2: 2.93 s | −32.1% | cap 0.07: 3.21 s | −25.5% |
| fast | smooth_mzv | corners_turn135 | 5.56 (0.247 / 0.247) | 5.55 (0.260 / 0.260) | −0.1% | cap 0.25: 5.56 s | −0.0% | cap 0.25: 5.56 s | −0.0% |
| fast | smooth_mzv | corners_turn170 | 5.09 (0.261 / 0.261) | 5.08 (0.260 / 0.260) | −0.0% | floor: 5.08 s | −0.0% | floor: 5.08 s | −0.0% |
| fast | smooth_mzv | corners_turn45 | 6.41 (0.259 / 0.257) | 6.39 (0.287 / 0.281) | −0.4% | cap 0.16: 6.39 s | −0.4% | cap 0.16: 6.39 s | −0.4% |
| fast | smooth_mzv | corners_turn90 | 6.07 (0.247 / 0.245) | 6.06 (0.260 / 0.257) | −0.2% | cap 0.25: 6.06 s | −0.2% | cap 0.25: 6.06 s | −0.2% |
| fast | smooth_mzv | voron_first_layer_perimeters | 70.51 (0.238 / 0.207) | 69.78 (0.240 / 0.217) | −1.0% | cap 0.25: 69.87 s | −0.9% | cap 0.25: 69.87 s | −0.9% |
| fast | smooth_mzv | voron_top_layers | 6.53 (0.166 / 0.138) | 3.82 (0.234 / 0.234) | −41.5% | cap 0.07: 5.37 s | −17.8% | cap 0.07: 5.37 s | −17.8% |
| fast | smooth_mzv | zigzag_infill | 3.27 (0.170 / 0.168) | 3.17 (0.182 / 0.182) | −3.2% | cap 0.2: 3.29 s | +0.6% | cap 0.2: 3.29 s | +0.6% |

### Physical invariants (540 floor and floor+cap runs, all cases)

| invariant | result |
|---|---|
| corner max ≤ shaper floor at max_accel, `max_accel·hypot(σx², σy²)/2` | all runs; worst 96.6% of it |
| acceleration never lowered: trapq move accels identical to the default run | all runs |
| smoother executed peak accel ≤ (1 + 5%)·max_accel | all 270 smoother runs; worst +3.3% (default: +3.3%) |
| no limiter registered: trapq identical to the upstream toolhead | 9 cases × 2 configs, `tests/test_junction_hook.py` |

`tests/test_corner_deviation.py` asserts the first three on the 90/170°
corner, zigzag, circle and top-layer cases, both configs and shaper families,
in floor mode and with a cap of 0.1 mm.

### Planner CPU (densest case, median of its runs during the sweep)

| config | densest case | moves/s of print | lookahead µs/move, scv 5 | with limiter | of which limiter | % of one core here | % of a Pi 5 core (2.5× slower) |
|---|---|---|---|---|---|---|---|
| bench | arcs | 1449 | 3.8 | 6.9 | 3.7 | 0.54% | 1.3% |
| fast | arcs | 1485 | 4.3 | 6.5 | 3.2 | 0.48% | 1.2% |

The previous, hard-budget limiter added 18 µs per move. The floor mode reads
one precomputed table cell per junction: the cell's speed at the floor is
stored, and only a cap needs a binary search.

### Pareto front, voron_top_layers, bench mzv

Default: scv 5, mcr 0.5 → 8.72 s, max 0.135. Floor mode: 5.70 s, max 0.233.

| setting | s | p99 mm | max mm | max step mm/s |
|---|---|---|---|---|
| scv 1, mcr 0.5 | 14.83 | 0.102 | 0.122 | 0.7 |
| floor+cap 0.05 | 9.74 | 0.105 | 0.123 | 4.2 |
| floor+cap 0.07 | 7.79 | 0.110 | 0.126 | 7.0 |
| floor+cap 0.1 | 6.77 | 0.110 | 0.141 | 8.8 |
| floor+cap 0.14 | 6.15 | 0.144 | 0.145 | 10.4 |
| floor+cap 0.2 | 5.72 | 0.173 | 0.186 | 12.4 |
| scv 20, mcr 0.5 | 5.54 | 0.218 | 0.219 | 12.8 |
| scv 35, mcr 0.25 | 5.43 | 0.221 | 0.245 | 17.5 |

## Caveats

- Everything here is offline: geometric deviation of the commanded stepper
  motion. Frame dynamics, ringing, extrusion and pressure advance are not
  modelled. The velocity steps are only a proxy for ringing.
- The corner table assumes full-length legs at full acceleration through the
  kernel. Corners between short moves round less than that, so near a stop
  the cap is conservative (zigzag infill).
- The smoother acceleration limit sums only steps of a curve continuing like
  the current junction; it is approximate, hence the stated 5% allowance.
- Rest stops still round at the floor. Like SCV, the limiter does not change
  that, and it never lowers acceleration to do so.

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
    uv run python -m tools.motion_eval.sweep --jobs 12   # 1476 runs, ~6 min
# tables: tools/motion_eval/.build/sweep/tables.md (all sweeps, all fronts)
```

# Owner operating point (50k, scv 50, 200/130 Hz)

`configs/owner.cfg`: corexy, max_accel 50 000, max_velocity 800, smooth_mzv
at X 200 Hz / Y 130 Hz, mcr 0.5. `python -m tools.motion_eval.owner_point`
runs the corpus at scv 5–80, with `[corner_deviation]` floor mode, and at
scv 50 with shaping off for reference. Results are in `.build/owner_point/`.

## Conclusion

1. **smooth_mzv absorbs SCV's velocity steps as far as ringing of the tuned
   mode goes. It does not absorb them in acceleration.** The smoother turns
   each step Δv into a burst Δv·w(t) over its 4.8 ms (X) / 7.4 ms (Y)
   window. A 90° corner at scv 50 commands a 50 mm/s per-axis step. The
   executed peak acceleration then reaches 1.62× max_accel on X (81 000
   mm/s²) and 1.22× on Y; the p99 over corners is 1.54×. At scv 20 the peak
   is 1.15×, at scv 5 1.02×. Peak jerk is 280 km/s³ at scv 50 against
   78 km/s³ at scv 5.
2. **Ringing does not grow with scv near the tuned frequency.** Residual
   vibration after corners, for a damped oscillator at the shaper frequency
   (worst axis):
   - At ζ 0.05: 8–10 µm at every scv from 5 to 80; at ζ 0.1: 0.5 µm.
   - With shaping off at scv 50: 227 µm (ζ 0.05) and 197 µm (ζ 0.1).
   - Mistuned by ±10–20%: 10–80 µm, with no trend in scv. The 0.8× case is
     largest at low scv, which points at the accel ramps rather than the
     corner steps.
   - Untuned higher modes do grow with the steps: at 2.5× f, 13 µm (scv 5)
     → 21 µm (scv 50) → 24 µm (scv 80); at 4× f, 1.7 → 3.5 → 4.4 µm. With
     shaping off at scv 50 these are 55 µm and 33 µm.
3. **What scv 50 buys.**
   - Corpus time −1.1% against scv 20, −2.9% against scv 10, −5.1% against
     scv 5; scv 80 saves another 0.3%. Almost all of it comes from dense
     paths: voron top layers take 2.73 s at scv 50 against 3.18 s at scv 20.
   - Costs: about 60% acceleration overshoot, 3.6× the peak jerk, twice the
     residual of a 2.5× f mode, and corners rounding up to 0.146 mm, which is
     1.17× the shaper floor. scv 20 keeps corners below the floor.
4. **Does it need solving?** Not for ringing of the tuned mode. The open
   risk is the acceleration burst: if 50 000 mm/s² is close to the motors'
   torque limit, scv 50 asks for 81 000 at corners, with skipped steps or
   layer shifts as the failure mode. Only hardware can say how much
   headroom there is.
5. **`[corner_deviation]` floor mode is 2.1% slower than scv 50 here.** It
   is faster than scv 5 and 10 but 1.0% slower than scv 20. It holds the
   executed peak within its stated allowance (1.03×). Its residuals at
   ζ 0.05 are within 5% of scv 5's up to 1.0× f, lower at 1.1–1.2× f, and
   16% higher at 2.5× f (15 against 13 µm; scv 50: 21 µm). The time it gives
   up against scv 50 is the price of not bursting above max_accel at corners.
6. **max_velocity 500 / 800 / 1200 changes nothing.** Only voron top
   layers' travel moves exceed 500 mm/s in this corpus, which moves totals
   by about 7 ms, and the relative numbers are identical. The corpus prints
   at ≤300 mm/s, so faster print moves are untested here.

The smooth_mzv kernel is zero at its support edges. That is a design
constraint in `klippy/extras/shaper_defs.py` (lines 178–180: "w(-T_sm/2) =
w(T_sm/2) = 0"; the polynomial is `get_mzv_smoother`, lines 199–211).
Numerically, with klippy's coefficients, w(±T_sm/2) ≈ −3e−12 /s against a
peak of 524 /s at 200 Hz and 340 /s at 130 Hz. So the executed acceleration
is continuous and no velocity step survives shaping (the executed velocity
jumps are 0). w′ is not zero at the edges (+3.6e6 / −2.5e6 /s² at 200 Hz),
so jerk jumps there but stays bounded: Δv·max|w′| = 50 mm/s × 3.64e6 /s² =
182 km/s³, which matches the 176 km/s³ measured on the 90° corners at scv 50.

## Summary (corpus, worst axis or case)

| setting | corpus s | vs scv 50 | peak accel / max_accel (x / y) | corner peak accel p99 / max_accel | peak jerk km/s³ (x / y) | corner dev p99 / max mm | max / floor |
|---|---|---|---|---|---|---|---|
| scv 5 | 105.6 | +5.1% | 1.024 / 1.005 | 1.024 | 78 / 50 | 0.103 / 0.111 | 0.89 |
| scv 10 | 103.3 | +2.9% | 1.063 / 1.018 | 1.063 | 94 / 56 | 0.107 / 0.115 | 0.92 |
| scv 20 | 101.5 | +1.1% | 1.149 / 1.058 | 1.149 | 165 / 72 | 0.117 / 0.123 | 0.99 |
| scv 35 | 100.7 | +0.3% | 1.447 / 1.136 | 1.420 | 247 / 98 | 0.132 / 0.135 | 1.08 |
| scv 50 | 100.4 | +0.0% | 1.616 / 1.222 | 1.535 | 280 / 127 | 0.139 / 0.146 | 1.17 |
| scv 80 | 100.1 | −0.3% | 1.719 / 1.379 | 1.719 | 423 / 185 | 0.164 / 0.164 | 1.31 |
| corner_deviation | 102.6 | +2.1% | 1.033 / 1.013 | 1.033 | 179 / 69 | 0.106 / 0.114 | 0.91 |
| scv 50, no shaping | 100.4 | +0.0% | 1.000 / 1.000 (plus Δv impulses) | 1.000 | (impulses) | 0.001 / 0.001 | — |

Shaper floor σ²·max_accel/2: X 0.053 mm, Y 0.125 mm. "max / floor" divides
by the larger, Y.

## Residual vibration after corners (max over constant-velocity windows, worst axis, µm)

Oscillator at ratio × the axis' shaper frequency (X 200, Y 130 Hz).

| setting | ζ | 0.8× | 0.9× | 1.0× | 1.1× | 1.2× | 2.5× | 4× |
|---|---|---|---|---|---|---|---|---|
| scv 5 | 0.05 | 78.1 | 24.2 | 8.4 | 13.1 | 20.8 | 13.2 | 1.7 |
| scv 20 | 0.05 | 64.6 | 23.2 | 7.9 | 13.1 | 15.7 | 12.6 | 1.8 |
| scv 50 | 0.05 | 55.2 | 30.9 | 10.5 | 15.9 | 17.5 | 20.9 | 3.5 |
| scv 80 | 0.05 | 60.7 | 33.3 | 10.3 | 23.1 | 20.6 | 23.9 | 4.4 |
| corner_deviation | 0.05 | 81.6 | 23.6 | 8.4 | 10.7 | 18.5 | 15.3 | 1.7 |
| scv 50, no shaping | 0.05 | 277.2 | 304.8 | 226.6 | 198.8 | 152.3 | 55.3 | 33.5 |
| scv 5 | 0.1 | 39.6 | 15.4 | 0.55 | 7.5 | 9.3 | 7.0 | 1.1 |
| scv 20 | 0.1 | 38.4 | 15.1 | 0.49 | 8.5 | 10.6 | 8.1 | 1.1 |
| scv 50 | 0.1 | 39.5 | 15.8 | 0.51 | 8.9 | 12.8 | 13.3 | 2.2 |
| scv 80 | 0.1 | 40.7 | 16.1 | 0.57 | 8.9 | 13.0 | 16.3 | 3.0 |
| corner_deviation | 0.1 | 40.3 | 13.8 | 0.45 | 6.6 | 8.4 | 8.2 | 1.1 |
| scv 50, no shaping | 0.1 | 240.8 | 230.9 | 197.0 | 155.1 | 135.6 | 47.4 | 27.6 |

## Per case: print time s (peak accel / max_accel)

| case | scv 5 | scv 20 | scv 50 | scv 80 | corner_deviation |
|---|---|---|---|---|---|
| arcs | 0.88 (0.44) | 0.88 (0.44) | 0.88 (0.44) | 0.88 (0.44) | 0.88 (0.44) |
| circles | 3.78 (0.84) | 2.83 (0.94) | 2.80 (1.07) | 2.80 (1.07) | 2.96 (0.85) |
| corners_turn45 | 6.36 (1.02) | 6.34 (1.07) | 6.31 (1.25) | 6.30 (1.44) | 6.35 (1.01) |
| corners_turn90 | 6.02 (1.02) | 6.00 (1.13) | 5.99 (1.42) | 5.97 (1.72) | 6.01 (1.03) |
| corners_turn135 | 5.50 (1.02) | 5.49 (1.15) | 5.48 (1.42) | 5.47 (1.69) | 5.50 (1.03) |
| corners_turn170 | 5.03 (1.02) | 5.02 (1.09) | 5.02 (1.20) | 5.01 (1.33) | 5.03 (1.03) |
| voron_first_layer_perimeters | 69.63 (0.76) | 69.16 (0.76) | 68.79 (1.14) | 68.65 (1.51) | 69.40 (0.76) |
| voron_top_layers | 5.68 (1.01) | 3.18 (1.07) | 2.73 (1.62) | 2.73 (1.62) | 3.75 (1.02) |
| zigzag_infill | 2.73 (1.01) | 2.63 (1.13) | 2.44 (1.42) | 2.29 (1.72) | 2.70 (1.03) |

## Caveats for this point

- The oscillator is a single mode per axis, driven by the executed carriage
  motion; belt stretch, frame modes coupling X and Y, and corexy motor
  coupling are not modelled. Residuals are measured only in constant-velocity
  windows after a corner, not during curves.
- The acceleration overshoot is the executed, shaped carriage acceleration.
  Whether the motors deliver it, or skip, depends on torque headroom that
  the harness cannot see.
- The corpus is sliced for slower machines: print moves at ≤300 mm/s.
