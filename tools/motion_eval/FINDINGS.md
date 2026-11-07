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
   default (scv 5, mcr 0.5): −7.3 / −7.2% (bench mzv / smooth_mzv), −6.0 /
   −5.7% (fast mzv / smooth_mzv). Dense geometry gains most: circles −35 to
   −43%, voron top layers −34 to −49%, zigzag infill −7.5 to −17% (fast
   smooth_mzv's 2 mm zigzag reverses next to moves shorter than the
   kernel's reach and now stops there, conclusion 4). It rounds more than
   the default in most cases (the default's worst corner is below the
   floor), up to the floor and never above it.
4. **There is no deviation cap below the floor any more.** `max_deviation`
   lowered junction speeds so corners round at most that much, judged as
   isolated corners. The random fuzz found caps exceeded by 5–17% (seeds
   421, 1306, 2682, 4438): corners closer together than the kernel's reach,
   with a leg that accelerates and decelerates between them, round beyond
   any isolated corner, and a look-back over the cluster did not bound the
   accel-bump case. Its corpus gain at equal worst-case deviation had been
   ≤ 2–5%. The mode is removed (5e22ba62f). The same physics broke the floor
   on a targeted cluster fuzz (up to 1.106×): a junction next to a move
   shorter than its speed × kernel reach now stops when it turns more than
   60° (a stop has no velocity step and rounds within the floor). On the
   owner corpus that adds 6 stops and +0.01% time. Clusters of smaller
   turns still reach 1.063× the floor on the cluster fuzz (seeds 319, 577);
   that is the stated FLOOR_MARGIN of 10%, fuzz-enforced.
5. **Side effects.** mzv passes larger velocity steps: up to 47 mm/s in
   floor mode (fast, 45° corners) against 3.7 mm/s at the default. That is
   harmless for the tuned mode but a risk for torque and off-frequency
   modes; only hardware can tell. With smoothers the corner bursts are not
   limited by default (as with scv): the executed peak reaches 1.29×
   max_accel (bench 90° corners; the default scv 5 reaches 1.03×), within
   the worst the planner admits (`corner_accel_x/y`) in every run (at most
   66% of it). `max_corner_accel` bounds the burst instead (5% stated
   allowance).
6. **Planner CPU.** The per-junction limit is C
   (`chelper/corner_deviation.c`): 0.43 µs per junction against 2.56 µs in
   Python, 0.38 / 0.40 µs after the cluster rules (unbounded / bounded;
   microbenchmark, owner smoothers). In this sweep (parallel jobs on a
   loaded box) the limiter's share of lookahead on the densest case (arcs,
   ~1 450 moves per second of print) is 0.9–2.7 µs per move including the
   harness's timing wrapper; see "Production readiness" for klippy CPU on
   one core.

## Setup

- **Harness**: unmodified klippy in batch mode. The executed motion is a model
  of `kin_shaper.c`, and every decoded MCU step is checked against it; in all
  972 runs the worst step used 99.6% of its allowed error. Corner deviation
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
  - `[corner_deviation]` floor mode, at mcr 0.5

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

## `[corner_deviation]`: floor mode against the default

`klippy/extras/corner_deviation.py`, hooked into `Move.calc_junction` through
`ToolHead.set_junction_limiter`, replaces the SCV junction term. Floor runs
have the section with no option, at mcr 0.5, against the default scv 5,
mcr 0.5.

| config | shaper | default s | floor s | vs default | cases where floor max ≤ default |
|---|---|---|---|---|---|
| bench | mzv | 119.3 | 110.6 | −7.3% | 1 of 9 |
| bench | smooth_mzv | 119.3 | 110.7 | −7.2% | 1 of 9 |
| fast | mzv | 108.6 | 102.1 | −6.0% | 1 of 9 |
| fast | smooth_mzv | 108.6 | 102.4 | −5.7% | 1 of 9 |

| config | shaper | case | default s (max / p99 mm) | floor s (max / p99 mm) | floor vs default | max step mm/s default / floor | peak accel default / floor |
|---|---|---|---|---|---|---|---|
| bench | mzv | arcs | 0.91 (0.208 / 0.208) | 0.93 (0.162 / 0.162) | +2.6% | 2.9 / 2.9 | 8972 / 8968 |
| bench | mzv | circles | 5.37 (0.152 / 0.129) | 3.06 (0.246 / 0.222) | −43.0% | 3.7 / 13.6 | 10000 / 10000 |
| bench | mzv | corners_turn135 | 5.84 (0.189 / 0.184) | 5.82 (0.224 / 0.224) | −0.4% | 2.4 / 10.7 | 10000 / 10000 |
| bench | mzv | corners_turn170 | 5.38 (0.167 / 0.167) | 5.37 (0.249 / 0.248) | −0.3% | 2.2 / 13.4 | 10000 / 10000 |
| bench | mzv | corners_turn45 | 6.68 (0.183 / 0.180) | 6.54 (0.220 / 0.220) | −2.1% | 3.2 / 20.4 | 10000 / 10000 |
| bench | mzv | corners_turn90 | 6.35 (0.201 / 0.194) | 6.26 (0.245 / 0.245) | −1.5% | 2.0 / 13.2 | 10000 / 10000 |
| bench | mzv | voron_first_layer_perimeters | 74.69 (0.201 / 0.190) | 72.27 (0.238 / 0.229) | −3.2% | 3.7 / 25.0 | 9969 / 9334 |
| bench | mzv | voron_top_layers | 8.72 (0.135 / 0.123) | 5.77 (0.184 / 0.174) | −33.8% | 3.7 / 12.2 | 10000 / 10000 |
| bench | mzv | zigzag_infill | 5.35 (0.149 / 0.137) | 4.56 (0.193 / 0.193) | −14.8% | 3.6 / 13.2 | 10000 / 10000 |
| bench | smooth_mzv | arcs | 0.91 (0.215 / 0.215) | 0.93 (0.167 / 0.167) | +2.6% | 0.0 / 0.0 | 9119 / 7627 |
| bench | smooth_mzv | circles | 5.37 (0.156 / 0.133) | 3.07 (0.255 / 0.216) | −42.9% | 0.0 / 0.0 | 10225 / 12836 |
| bench | smooth_mzv | corners_turn135 | 5.84 (0.196 / 0.191) | 5.82 (0.232 / 0.232) | −0.4% | 0.0 / 0.0 | 10331 / 11699 |
| bench | smooth_mzv | corners_turn170 | 5.38 (0.174 / 0.174) | 5.37 (0.258 / 0.257) | −0.3% | 0.0 / 0.0 | 10240 / 12404 |
| bench | smooth_mzv | corners_turn45 | 6.68 (0.190 / 0.187) | 6.54 (0.228 / 0.228) | −2.1% | 0.0 / 0.0 | 10203 / 11969 |
| bench | smooth_mzv | corners_turn90 | 6.35 (0.209 / 0.201) | 6.26 (0.254 / 0.254) | −1.5% | 0.0 / 0.0 | 10217 / 12929 |
| bench | smooth_mzv | voron_first_layer_perimeters | 74.69 (0.206 / 0.195) | 72.34 (0.244 / 0.233) | −3.1% | 0.0 / 0.0 | 8064 / 11190 |
| bench | smooth_mzv | voron_top_layers | 8.72 (0.136 / 0.128) | 5.77 (0.178 / 0.177) | −33.8% | 0.0 / 0.0 | 10138 / 10408 |
| bench | smooth_mzv | zigzag_infill | 5.35 (0.152 / 0.141) | 4.57 (0.202 / 0.202) | −14.6% | 0.0 / 0.0 | 10209 / 12572 |
| fast | mzv | arcs | 0.88 (0.093 / 0.093) | 0.88 (0.093 / 0.093) | +0.0% | 3.1 / 3.1 | 12224 / 12224 |
| fast | mzv | circles | 4.31 (0.192 / 0.145) | 2.82 (0.258 / 0.227) | −34.5% | 3.7 / 20.3 | 29718 / 23179 |
| fast | mzv | corners_turn135 | 5.56 (0.238 / 0.238) | 5.54 (0.296 / 0.296) | −0.3% | 2.4 / 16.9 | 28002 / 28002 |
| fast | mzv | corners_turn170 | 5.09 (0.252 / 0.252) | 5.08 (0.258 / 0.257) | −0.1% | 2.2 / 14.3 | 29896 / 29896 |
| fast | mzv | corners_turn45 | 6.41 (0.250 / 0.248) | 6.33 (0.301 / 0.301) | −1.3% | 3.2 / 46.6 | 29481 / 29481 |
| fast | mzv | corners_turn90 | 6.07 (0.238 / 0.237) | 6.01 (0.341 / 0.341) | −1.0% | 2.0 / 28.3 | 28002 / 28002 |
| fast | mzv | voron_first_layer_perimeters | 70.51 (0.234 / 0.204) | 69.34 (0.240 / 0.239) | −1.7% | 3.7 / 37.0 | 27651 / 23097 |
| fast | mzv | voron_top_layers | 6.53 (0.164 / 0.133) | 3.36 (0.252 / 0.249) | −48.5% | 3.7 / 21.2 | 30000 / 30000 |
| fast | mzv | zigzag_infill | 3.27 (0.167 / 0.164) | 2.71 (0.271 / 0.271) | −17.3% | 3.6 / 28.3 | 23179 / 30000 |
| fast | smooth_mzv | arcs | 0.88 (0.096 / 0.096) | 0.88 (0.096 / 0.096) | +0.0% | 0.0 / 0.0 | 12657 / 12657 |
| fast | smooth_mzv | circles | 4.31 (0.198 / 0.149) | 2.82 (0.267 / 0.227) | −34.5% | 0.0 / 0.0 | 17220 / 23717 |
| fast | smooth_mzv | corners_turn135 | 5.56 (0.247 / 0.247) | 5.54 (0.307 / 0.307) | −0.3% | 0.0 / 0.0 | 28342 / 31247 |
| fast | smooth_mzv | corners_turn170 | 5.09 (0.261 / 0.261) | 5.08 (0.268 / 0.266) | −0.1% | 0.0 / 0.0 | 30077 / 30029 |
| fast | smooth_mzv | corners_turn45 | 6.41 (0.259 / 0.257) | 6.33 (0.310 / 0.310) | −1.3% | 0.0 / 0.0 | 29667 / 29815 |
| fast | smooth_mzv | corners_turn90 | 6.07 (0.247 / 0.245) | 6.01 (0.353 / 0.353) | −1.0% | 0.0 / 0.0 | 28342 / 32058 |
| fast | smooth_mzv | voron_first_layer_perimeters | 70.51 (0.238 / 0.207) | 69.36 (0.247 / 0.246) | −1.6% | 0.0 / 0.0 | 16146 / 20740 |
| fast | smooth_mzv | voron_top_layers | 6.53 (0.166 / 0.138) | 3.36 (0.256 / 0.253) | −48.5% | 0.0 / 0.0 | 30125 / 30338 |
| fast | smooth_mzv | zigzag_infill | 3.27 (0.170 / 0.168) | 3.03 (0.235 / 0.233) | −7.5% | 0.0 / 0.0 | 24361 / 30803 |

### Physical invariants (36 floor-mode runs, all cases)

| invariant | result |
|---|---|
| corner max ≤ shaper floor at max_accel, `max_accel·hypot(σx², σy²)/2` | all runs; worst 96.6% of it |
| acceleration never lowered: trapq move accels identical to the default run | all runs |
| smoother executed peak accel ≤ what the planner admits (`corner_accel_x/y`) | all 18 smoother runs; worst 66% of it (peak 1.29× max_accel, bench 90° corners; the default reaches 1.03×) |
| no limiter registered: trapq identical to the upstream toolhead | 9 cases × 2 configs, `tests/test_junction_hook.py` |

`tests/test_corner_deviation.py` asserts the first three on the 90/170°
corner, zigzag, circle and top-layer cases, both configs and shaper families,
in floor mode and with `max_corner_accel` 1.03·max_accel, and the smoother
peak within that bound·(1 + 5%) and within `corner_accel_x/y`.

### Planner CPU (densest case, median of its runs during the sweep)

| config | densest case | moves/s of print | lookahead µs/move, scv 5 | with limiter | of which limiter |
|---|---|---|---|---|---|
| bench | arcs | 1449 | 8.0 | 15.3 | 2.7 |
| fast | arcs | 1485 | 7.0 | 12.9 | 0.9 |

Measured with parallel sweep jobs on a loaded box, so the columns are noisy;
the limiter column is the C call plus the harness's timing wrapper.

### Pareto front, voron_top_layers, bench mzv

Default: scv 5, mcr 0.5 → 8.72 s, max 0.135. Floor mode: 5.77 s, max 0.184.

| setting | s | p99 mm | max mm | max step mm/s |
|---|---|---|---|---|
| scv 1, mcr 0.5 | 14.83 | 0.102 | 0.122 | 0.7 |
| scv 3, mcr 0.5 | 10.92 | 0.111 | 0.127 | 2.2 |
| scv 5, mcr 0.5 | 8.72 | 0.123 | 0.135 | 3.7 |
| scv 8, mcr 0.5 | 6.93 | 0.164 | 0.164 | 5.9 |
| floor, mcr 0.5 | 5.77 | 0.174 | 0.184 | 12.2 |
| scv 20, mcr 0.5 | 5.54 | 0.218 | 0.219 | 12.8 |
| scv 35, mcr 0.25 | 5.43 | 0.221 | 0.245 | 17.5 |

## Caveats

- Everything here is offline: geometric deviation of the commanded stepper
  motion. Frame dynamics, ringing, extrusion and pressure advance are not
  modelled. The velocity steps are only a proxy for ringing.
- The corner table assumes full-length legs at constant acceleration
  through the kernel. A short move that accelerates and decelerates
  between two corners breaks that; such junctions stop when they turn more
  than 60° (conclusion 4), and smaller turns in such clusters can round up
  to the stated 10% (FLOOR_MARGIN) above the floor.
- With `max_corner_accel`, the burst bound is the largest Σ dv·|density|
  over the step placements the plan allows plus the legs at full
  acceleration: a true bound, at a small time cost on dense curves.
  Without it, the burst is bounded only by what the table admits
  (`corner_accel_x/y`, isolated corners and curves plus a stated 25% for
  clusters).
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
- `SET_INPUT_SHAPER` right after motion, to a shaper whose generation window
  is longer than the current one, shut the printer down ("stepcompress:
  Invalid sequence"): the new shaper's position at the last flushed step
  still averaged the last moves. Found by the fuzz (seed 1044, default
  planner), reproduced on upstream; Klipper master af81d23fe has the same
  code. Fixed in 44cd81786 (the old shapers generate the stationary tail
  for the window growth first), `tests/test_shaper_switch.py`.
- At print start the toolhead stays in Priming until 2 s of moves are
  queued, planned in one gcode callback (2630 moves, 23 ms of CPU on this
  box for arcs). With klippy at a 12% CPU quota that callback is ~290 ms
  late and the MCU shut down with "Timer too close" at print start (with
  the Python corner_deviation, 5d53e78f8 and its threaded build; the C
  limiter passed its one run, the default always);
  BUFFER_TIME_START 0.5 instead of 0.25 s made it pass, so the 0.25 s start
  buffer is what the burst outruns (the exact late command was not traced).
  Left as is; corner_deviation's share of that callback is 0.43 µs per
  move.
- Live klippy flushes the lookahead with a 2 s window while priming and
  0.25 s after, batch mode at other moves; with minimum_cruise_ratio > 0
  the cruise speed of a move near a flush differs (bench: 0.085 mm/s on
  voron top layers ×15, upstream and kv2 alike). The fuzz's lazy-flush check
  expects identical trapqs only where the default planner's is identical.

## Reproduce

```
python3 ~/Developer/serval/scripts/slots.py run -- \
    uv run python -m tools.motion_eval.sweep --jobs 12   # 972 runs, ~6 min
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
   window. A 90° corner at scv 50 commands a 50 mm/s per-axis step; 90°
   corners alone peak at 1.42–1.50× max_accel. The corpus peak, 1.62× on X
   (81 000 mm/s²) and 1.22× on Y, comes from a polyline arc of radius
   ≈ 2.5 mm in voron top layers, where every chord is a junction step and
   also accelerates (PRINTER_TEST.md's stress circle reaches 2.16×). The
   p99 over corners is 1.54×. At scv 20 the peak is 1.15×, at scv 5 1.02×.
   Peak jerk is 280 km/s³ at scv 50 against 78 km/s³ at scv 5.
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
5. **What `[corner_deviation]` cost here was the corner-burst bound, not
   the floor.** Variants on the corpus against scv 50 (100.4 s):

   | setting | corpus s | vs scv 50 | voron top / circles / zigzag s | peak / max_accel X / Y | corner dev p99 / max mm (max / floor) | residual ζ 0.05 at 1.0× / 2.5× f µm |
   |---|---|---|---|---|---|---|
   | scv 50 | 100.4 | 0 | 2.73 / 2.80 / 2.44 | 1.616 / 1.222 | 0.139 / 0.146 (1.17) | 10.5 / 20.9 |
   | burst bounded (`max_corner_accel: 51500`; the default before, single-junction bound, not a true bound) | 103.7 | +3.2% | 4.56 / 3.07 / 2.74 | 1.012 / 1.004 | 0.103 / 0.111 (0.89) | 8.3 / 14.0 |
   | burst bounded now, `max_corner_accel: 51500` | 105.8 | +5.4% | 6.09 / 3.42 / 2.75 | 1.012 / 1.000 | 0.103 / 0.111 (0.89) | 11.8 / 16.2 |
   | burst bounded now, `max_corner_accel: 65000` | 102.2 | +1.8% | 3.88 / 2.89 / 2.63 | 1.208 / 1.060 | 0.114 / 0.120 (0.96) | 8.5 / 14.4 |
   | burst bounded now, `max_corner_accel: 81000` (scv 50's own peak) | 101.5 | +1.1% | 3.42 / 2.83 / 2.50 | 1.483 / 1.203 | 0.123 / 0.123 (0.98) | 10.3 / 12.9 |
   | burst unbounded (the default now) | 101.1 | +0.7% | 3.08 / 2.81 / 2.43 | 1.483 / 1.239 | 0.123 / 0.123 (0.98) | 11.7 / 12.9 |
   | unbounded, cap 0.146 replacing the floor (experiment) | 100.2 | −0.2% | 2.72 / 2.80 / 2.35 | 2.477 / 1.302 | 0.146 / 0.146 (1.17) | 15.4 / 18.7 |

   **The unbounded default bursts less than scv 50 on this corpus (1.48×
   against 1.62× X) but more on some geometry.** On the printer test's
   45° octagon (`corners_floor` against `corners_scv50`) the planned X
   peak is 1.78× max_accel against 1.42×: floor mode runs those turns at
   up to 200 mm/s where scv 50 runs 112 mm/s. On the stress program's
   small circle (`stress_floor` against `stress_scv50`) it is 2.23× against
   2.16× on X and 2.47× against 2.30× on a corexy motor. The default
   bounds the deviation, not the burst. `max_corner_accel: 81000` caps the
   burst at 1.62×, what scv 50 demands on this corpus (101.5 s, +1.1%):
   1.48× X on the octagon (scv 50 1.42×) and 0.85× X / 1.27× motor on the
   small circle (scv 50 2.16× / 2.30×).

   The bound cost 2.6 of the 3.2 points; the floor the remaining 0.6
   (voron top layers 3.08 against 2.73 s). The cap row needed a build in
   which a cap replaces the floor: 0.8% faster than the default, at 2.48×
   max_accel on X (corners_turn170); not proposed, and cap mode is gone
   (conclusion 4 above). The burst is now opt-in (`max_corner_accel`), and
   its bound is a true upper bound on clustered corners too (the
   single-junction bound let clusters reach 1.10× it): the executed burst
   is the legs' acceleration plus Σ dv·|density| of the steps within the
   kernel's support, maximized over every placement the plan allows (the
   steps' gaps are at least the moves' shortest traversal times). A first
   version counting every step at peak density cost 108.9 s at 51 500 and
   104.6 s at 81 000; the shaped one 105.8 / 101.5 s. What remains is the
   legs' term, taken at full acceleration together with the steps' peak.
   Unbounded, the planner admits at most 144 300 mm/s² on X and 99 400 on
   Y at 50 000 (`corner_accel_x/y`: the isolated corner and curve bound
   from the table, 115 500 / 79 500, plus the 25% stated for clusters;
   measured 97 700 on the worst cell, a 14.9° turn at 713 mm/s), and
   `CORNER_DEVIATION_REPORT MOTOR_ACCEL=50000` recommends max_accel
   17 321, at which that corner measures 33 800 and voron top layers 36 100
   (`tests/test_corner_accel.py`).
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
| corner_deviation | 101.1 | +0.7% | 1.483 / 1.239 | 1.483 | 398 / 99 | 0.123 / 0.123 | 0.98 |
| corner_deviation, max_corner_accel 51 500 | 105.8 | +5.4% | 1.012 / 1.000 | 1.012 | 120 / 59 | 0.103 / 0.111 | 0.89 |
| corner_deviation, max_corner_accel 81 000 | 101.5 | +1.1% | 1.483 / 1.203 | 1.483 | 312 / 99 | 0.123 / 0.123 | 0.98 |
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
| corner_deviation | 0.05 | 54.2 | 27.6 | 11.7 | 16.3 | 17.0 | 12.9 | 2.3 |
| corner_deviation, bounded 51 500 | 0.05 | 71.2 | 27.3 | 11.8 | 21.4 | 18.3 | 16.2 | 1.7 |
| scv 50, no shaping | 0.05 | 277.2 | 304.8 | 226.6 | 198.8 | 152.3 | 55.3 | 33.5 |
| scv 5 | 0.1 | 39.6 | 15.4 | 0.55 | 7.5 | 9.3 | 7.0 | 1.1 |
| scv 20 | 0.1 | 38.4 | 15.1 | 0.49 | 8.5 | 10.6 | 8.1 | 1.1 |
| scv 50 | 0.1 | 39.5 | 15.8 | 0.51 | 8.9 | 12.8 | 13.3 | 2.2 |
| scv 80 | 0.1 | 40.7 | 16.1 | 0.57 | 8.9 | 13.0 | 16.3 | 3.0 |
| corner_deviation | 0.1 | 40.0 | 15.7 | 0.52 | 8.9 | 11.1 | 8.6 | 1.6 |
| corner_deviation, bounded 51 500 | 0.1 | 40.1 | 15.7 | 0.54 | 9.1 | 9.4 | 8.3 | 1.1 |
| scv 50, no shaping | 0.1 | 240.8 | 230.9 | 197.0 | 155.1 | 135.6 | 47.4 | 27.6 |

## Per case: print time s (peak accel / max_accel)

| case | scv 5 | scv 20 | scv 50 | scv 80 | corner_deviation | bounded 51 500 |
|---|---|---|---|---|---|---|
| arcs | 0.88 (0.44) | 0.88 (0.44) | 0.88 (0.44) | 0.88 (0.44) | 0.88 (0.44) | 1.09 (0.57) |
| circles | 3.78 (0.84) | 2.83 (0.94) | 2.80 (1.07) | 2.80 (1.07) | 2.81 (1.07) | 3.42 (0.83) |
| corners_turn45 | 6.36 (1.02) | 6.34 (1.07) | 6.31 (1.25) | 6.30 (1.44) | 6.32 (1.10) | 6.36 (1.00) |
| corners_turn90 | 6.02 (1.02) | 6.00 (1.13) | 5.99 (1.42) | 5.97 (1.72) | 5.99 (1.48) | 6.02 (1.01) |
| corners_turn135 | 5.50 (1.02) | 5.49 (1.15) | 5.48 (1.42) | 5.47 (1.69) | 5.49 (1.23) | 5.50 (1.01) |
| corners_turn170 | 5.03 (1.02) | 5.02 (1.09) | 5.02 (1.20) | 5.01 (1.33) | 5.02 (1.16) | 5.03 (1.00) |
| voron_first_layer_perimeters | 69.63 (0.76) | 69.16 (0.76) | 68.79 (1.14) | 68.65 (1.51) | 69.06 (1.29) | 69.53 (0.76) |
| voron_top_layers | 5.68 (1.01) | 3.18 (1.07) | 2.73 (1.62) | 2.73 (1.62) | 3.08 (1.47) | 6.09 (1.00) |
| zigzag_infill | 2.73 (1.01) | 2.63 (1.13) | 2.44 (1.42) | 2.29 (1.72) | 2.43 (1.48) | 2.75 (1.01) |

## Caveats for this point

- The oscillator is a single mode per axis, driven by the executed carriage
  motion; belt stretch, frame modes coupling X and Y, and corexy motor
  coupling are not modelled. Residuals are measured only in constant-velocity
  windows after a corner, not during curves.
- The acceleration overshoot is the executed, shaped carriage acceleration.
  Whether the motors deliver it, or skip, depends on torque headroom that
  the harness cannot see.
- The corpus is sliced for slower machines: print moves at ≤300 mm/s.

# Production readiness

Evidence for running `[corner_deviation]` and the shaper-switch fix on a
real printer, gathered offline and live on the dev box (the Pi bench runs
separately). Owner config `configs/owner.cfg` (corexy, smooth_mzv 200/130
Hz, 50 000 mm/s²); live runs use `configs/owner_linux.cfg`, the same
machine on the Linux-process MCU at 4 microsteps.

## Bugs found and fixed

| found by | bug | fix |
|---|---|---|
| live klippy (bench) | `stats` dict collided with statistics.py's `stats()` callback: crash 1 s after ready | 19348edc9; `tests/test_printer_objects.py` calls every object's `stats` and json-dumps every status |
| bench, 5d53e78f8 | table build (0.6–2 s on the Pi) in the connect handler; print-start burst late 53–103 ms; "Timer too close" at 1 cpu/1500 MHz | 7b467cbad: junction limit in C (0.43 against 2.56 µs), tables built in a nice-19 thread and cached; gcode waits until ready |
| fuzz seed 1044 (default planner, upstream too) | SET_INPUT_SHAPER to a longer kernel right after motion: "stepcompress: Invalid sequence", shutdown | 44cd81786; `tests/test_shaper_switch.py` |
| fuzz seed 2 | junction to a pure Z move: ZeroDivisionError in the curve speed | such junctions stop (`xy_stops`), 7717e74de |
| fuzz seed 56 | short-chord arc sped up between curve-capped junctions; the cruise cap was flush-dependent | curve cap leaves room for a short move's gain, cruise cap removed, 7717e74de |
| fuzz seeds 18, 22 | accelerating arcs: legs' accel plus curve steps reached 1.054× max_accel | at the time: curve steps shared (1 + 3%)·max_accel with the legs, 7717e74de; now the opt-in max_corner_accel |
| fuzz seed 64 | not a limiter bug: with minimum_cruise_ratio > 0 the stock lookahead cruises faster after a forced stop | forced-flush check at minimum_cruise_ratio 0 |
| fuzz generator, seed 253 | minimum_cruise_ratio 1.0 is an invalid config | drawn from [0, 0.95) |
| owner | the burst bound cost 2.6 of corner_deviation's 3.2% against scv 50 | 7b117b1e4: bound opt-in (`max_corner_accel`), `corner_accel_x/y`, `CORNER_DEVIATION_REPORT` |
| fuzz seeds 356, 421, 1306, 2682, 4438 | max_deviation exceeded by 1–17% on corner clusters | cap mode removed, 5e22ba62f |
| cluster fuzz seeds 7, 79, 111, 207, 250 | floor exceeded by up to 1.106× on micro-move clusters | junctions next to a short move turning > 60° stop, 5e22ba62f |
| cluster fuzz seeds 143, 170, 387 | cluster steps summed 10–20% past the isolated-corner accel bound | max_corner_accel bounds the steps' superposition (5e22ba62f, shaped in 1f44ab989); the unbounded worst states a 25% cluster margin |
| cluster fuzz seeds 319, 577 | turns ≤ 60° in clusters still round 1.04–1.06× the floor | stated FLOOR_MARGIN 10%, fuzz-enforced, 1f44ab989 |
| cluster fuzz seeds 32, 36, 120, 202 | harness: step check failed by < 0.2% (clock rounding; smoother quadrature at a 25 µs grid) | 2-tick rounding, speed-peak term (5e22ba62f); the fuzz re-checks on a 10 µs grid where the 25 µs one misses a step (1f44ab989) |
| owner steer | max_corner_accel: the windowed peak-density bound cost +8.5% at 51 500 | bound by kernel-shaped superposition over the allowed step placements, 1f44ab989 (+5.4% at 51 500, +1.1% at 81 000) |

## Fuzz

Final runs at 1f44ab989 (`tools/motion_eval/.build/fuzz*/results.jsonl`):

| run | cases | violations | floor ratio max | executed / admitted max (floor run, bounded run) |
|---|---|---|---|---|
| `fuzz.py --seeds 0:10000` (random gcode) | 10 000 | 0 | 0.9999 | 1.0000, 1.0000 |
| `fuzz.py --generator clusters --seeds 0:2000` | 2 000 | 0 | 1.063 (seed 577; FLOOR_MARGIN 10%) | 1.0000, 1.0000 |

Each case plans the default planner, floor mode, floor bounded by
max_corner_accel 1.03·max_accel, floor with lazy flushes, the default
planner with the same lazy flushes, and floor with forced flushes at
minimum_cruise_ratio 0 (6–8 klippy runs), and checks completion, finite
moves, the harness's step check, the floor, unchanged accels and final
step positions, the admitted acceleration (unbounded: CornerAccel.worst
with its 25% cluster margin; bounded: max_corner_accel exactly) and flush
invariance. The 10k run took 3.5 h with 10 jobs. Regression subsets run in
the test suite: `tests/fuzz_seeds.txt`, `tests/fuzz_cluster_seeds.txt`,
`tests/fuzz_cases/*.json`.

## Flush invariance

- Lazy flushes (`MOTION_EVAL_LAZY_FLUSH_AT`, random add_move counts) leave
  every junction limit identical (all fuzz cases, `tests/test_flush_invariance.py`
  on corpus cases, seeds in `tests/flush_seeds.txt`), and the trapq
  identical wherever the default planner's own trapq is identical with the
  same flushes: with minimum_cruise_ratio > 0 the stock lookahead smooths
  cruise speeds over the flush window (84 of the 2000 cluster cases, 0 of the 10 000 random ones;
  the bench saw the same on voron top layers ×15, upstream and kv2 alike).
- Forced flushes (M400 after random lines), at minimum_cruise_ratio 0:
  limits identical wherever both runs compute the junction, no move faster.
- Klipper's runout flush (`_handle_step_flush`) is accounted for: a lazy
  flush that leaves too little planned stops the toolhead, and those
  junctions are exempt (0 cases in both final runs).

## Live against batch (linux-process MCU, step recorder preload)

`tests/test_live.py` on corners_turn90, circles and voron_top_layers,
owner_linux.cfg, with and without `[corner_deviation]`, statuses polled
every 0.25 s and one statistics period waited: trapq identical to batch
(one time offset), every step position identical, step times within
2·MAX_STEPCOMPRESS_ERROR after an affine clock fit, every commanded step
executed, no status query failed. At 1f44ab989 all 6 pass; with two live
runs in parallel next to the fuzz (load 25) the linux-process MCU once fell
behind ("Stepper too far in past", voron top layers with
corner_deviation) and the case passed twice alone: the MCU process, not the
plan. Earlier runs measured step time errors of 21–29 µs against the
50.04 µs allowed and step jitter p99 ≈ 14 µs.

## Reactor and startup (this box, taskset one core, ReactorMeter 5 ms timer)

| condition | default late ms | corner_deviation late ms | note |
|---|---|---|---|
| full speed | 21.2 | 24.3 | the print-start burst; in print ≤ 7.7 ms both |
| 30% CPU quota | 74.9 | 92.6 | |
| 12% CPU quota | 211.6 | 227.8 | 5d53e78f8 shut down here ("Timer too close") |

Ready is not delayed (0.77 / 0.79 s to ready); a cold table is ready 0.5 s
after ready at full speed (1.9 s at 30%), gcode waits for it; GIL and CPU
sharing with the build thread cost ≤ 3 ms of lateness at full speed. Bench
(Pi 5, 5d53e78f8) saw 53–103 ms on the print-start burst with
corner_deviation against < 6–36 ms without; the C limiter (0.38–0.43 µs per
junction) is what the bench re-run should confirm.

## Planner CPU (owner_linux, klippy pinned to one core)

| limiter | case | moves/s of print | lookahead µs/move | of which limiter | klippy CPU here | Pi 5 (×2.5) |
|---|---|---|---|---|---|---|
| default | arcs ×10 | 1240 | 2.1 | 0.0 | 1.7% | 4.3% |
| corner_deviation | arcs ×10 | 1242 | 2.9 | 0.6 | 2.0% | 5.0% |
| default | voron_top ×3 | 231 | 2.0 | 0.0 | 1.1% | 2.8% |
| corner_deviation | voron_top ×3 | 415 | 2.9 | 0.5 | 1.6% | 4.0% |

The limiter column includes the harness's timing wrapper. "Pi 5" assumes
a Pi 5 core is 2.5× slower than this one, an assumption, not a
measurement. corner_deviation prints voron top layers faster, so more moves
per second of print. Measured at 7b117b1e4; the cluster rules leave the
unbounded junction cost unchanged.

## What the default does not bound

The unbounded default (no `max_corner_accel`) keeps corners within the
shaper floor but does not keep the corner burst at or below scv 50's. It
is lower on the owner corpus (peak 1.48× against 1.62× X) and higher on
the printer test's 45° octagon (1.78× against 1.42× X) and small circle
(2.23× against 2.16× X; 2.47× against 2.30× on a corexy motor).
`max_corner_accel: 81000` caps it at 1.62×, scv 50's corpus burst (101.5
s, +1.1% against scv 50, 0.4% slower than the default): 1.48× X on the
octagon and 0.85× X / 1.27× motor on the small circle. 51 500 is the
low-burst setting (105.8 s, +5.4%). Both are in the printer protocol
(`bounded81000`, `bounded`).

## Printer protocol

`PRINTER_TEST.md`, `printer_test.py` and `printer_test/*.gcode` (corners
and stress programs for scv 5/20/50, floor, bounded81000 and bounded), `accel_residual.py`
(accelerometer record → corner acceleration and residual, tested on
synthetic records in `tests/test_accel_residual.py`).
