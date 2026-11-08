# kv2 on the hardware bench: results

## Release candidate 7a0b875d4

**Verdict: PASS.** kv2/main 7a0b875d4 passes every gate on the bench. One locked, detached session (`~/bench/kv2/20261107-214845`) ran 174 runs: G1, G2, G4, 30 trials, 90 clock trials and 4 soaks of ~34 min. Each run in it was checked by every check the harness enforces.

7a0b875d4 includes:
- the clocksync fix;
- the C limiter with the kernel-shaped `max_corner_accel` bound;
- cap mode removed;
- the 60° short-move stop rule.

Firmware is stock Kalico built from the same commit (`v2026.04.00-366-g7a0b875d4`). Corner tables are cached in `/home/pi/.cache/kalico/corner_deviation`, recorded in `~/kv2/.kv2-deploy.json`. `owner_like_cd_bounded` = `owner_like_cd` + `max_corner_accel: 81000`.

| gate | config | result | key number |
|---|---|---|---|
| G1 live print | default | PASS | 2 runs, worst capture−ref 27.2 / bound 33.8 µs |
| G2 executed = planned | default | PASS | 1 runs, worst capture−ref 1.9 / bound 27.5 µs |
| G2 executed = planned | owner_like | PASS | 4 runs, worst capture−ref 28.7 / bound 33.7 µs |
| G2 executed = planned | owner_like_cd | PASS | 4 runs, worst capture−ref 29.2 / bound 34.9 µs |
| G2 executed = planned | owner_like_cd_bounded | PASS | 4 runs, worst capture−ref 29.1 / bound 33.7 µs |
| Soak (replay judge) | owner_like_cd | PASS | 1 runs, worst capture−ref 26.9 / bound 34.9 µs |
| Soak (replay judge) | owner_like_cd 1 cpu/1500 | PASS | 1 runs, worst capture−ref 27.2 / bound 34.7 µs |
| Soak (replay judge) | owner_like_cd_bounded | PASS | 1 runs, worst capture−ref 27.8 / bound 34.8 µs |
| Soak (replay judge) | owner_like_cd_bounded 1 cpu/1500 | PASS | 1 runs, worst capture−ref 27.7 / bound 34.6 µs |
| G4 upstream identity | owner_like arcs | PASS | batch messages identical |
| G4 upstream identity | owner_like speed_ladder | PASS | batch messages identical |
| G4 upstream identity | owner_like voron_top_layers | PASS | batch messages identical |
| Trials arcs_x30 1 cpu/1500 | owner_like_cd (cold table cache) | PASS | 0/1 failed, lateness p99 11.1 ms max 11.2 ms |
| Trials arcs_x30 1 cpu/1500 | owner_like | PASS | 0/10 failed, lateness p99 35.4 ms max 53.5 ms |
| Trials arcs_x30 1 cpu/1500 | owner_like_cd_bounded | PASS | 0/10 failed, lateness p99 26.7 ms max 28.4 ms |
| Trials arcs_x30 1 cpu/1500 | owner_like_cd (warm table cache) | PASS | 0/9 failed, lateness p99 10.0 ms max 10.1 ms |
| Clock trials 1 cpu/1500 | owner_like_cd-print | PASS | 0/30 failed, calibration -471…+1331 ppm |
| Clock trials 1 cpu/1500 | owner_like-print | PASS | 0/30 failed, calibration -452…+1232 ppm |
| Clock trials 1 cpu/1500 | owner_like-idle | PASS | 0/30 failed, calibration -491…+1344 ppm |
| Restoration | serval state | PASS | differences [] |

How each row is judged:
- **G1 / G2.** Batch-mode reference on this box; event capture; net steps exact, every step paired, capture−reference ≤ 25 µs + the run's MCU lateness + 0.1 µs; the MCU executed every step of the live queue_step tap. Single-pass rows: the live tap equals the batch step sequence. The voron ×15 rows use the live-trapq replay.
- **G4.** corner_deviation off: upstream 1ee39ff2c and kv2 send identical decoded host→MCU message streams in batch mode, produce identical captured step sequences on the bench, and the upstream rows pass the same G2 checks.
- **Trials.** 1 cpu / 1500 MHz, interleaved; lateness is the probe's timer lateness during the print. The first owner_like_cd trial built its corner table from an empty cache.
- **Clock trials.** Klippy start + 15 s at 1 cpu / 1500 MHz. Calibration is the frequency estimate after connect, relative to CLOCK_FREQ. No sample was ignored after connect.
- **Soaks.** Replay judge, extruder included, 10 s windowed clock fit, all four checks. Replay = tap at 72.9–73.3 % of steps on the identical clock, max 25.00 µs; MCU lateness max 9.5–9.8 µs.

One harness fix during the analysis. The capture of `g2 owner_like speed_ladder` ends 9 bytes into an anchor-only block (mask 0x80), where the capture stop cut the stream. The analyzer used to refuse any cut tail. It now accepts a cut block only when the block carries no edges, and fails otherwise (`compare.load_capture`, with a test). Re-analyzed, that run passes all checks: capture−reference 28.73 µs against a bound of 33.69 µs, and step counts equal to the plan.

## Conclusion

- **G1 pass.** kv2 klippy runs live on the Pi 5 against stock Kalico F401 firmware built from kv2. It printed `speed_ladder.gcode` and the corpus cases `voron_top_layers` and `arcs` with no error, on the default, owner-like and owner-like + `[corner_deviation]` configs.
- **G2 pass on every single-pass program; one open host finding.** Each pass checked net steps, every step paired, and timing against a bound of `MAX_STEPCOMPRESS_ERROR` (25 µs) plus this run's measured MCU lateness. Event mode, all 11 runs (3 programs × 3 configs, plus the upstream rows):
  - The MCU executed every commanded step. The captured sequence equals the live queue_step tap on every run, including the dense ×15 rows.
  - Worst capture−reference error is 29.2 µs, against a bound of 33.8 µs.
  - Open finding, now explained: with `voron_top_layers` repeated 15×, the live host commands 40 X / 46 Y fewer steps than batch mode, with identical net steps. The divergence is deterministic (same step index in 8 runs, any host load) and the tap shows the live schedule already differs, so it is not an MCU loss.
    - First step divergence (reference t ≈ 3.97 s): a ±1 reversal pair that batch emits and live does not. It is step generation, not the planner: replaying batch's own trapq with live-like flush batches drops the same pair (see "Live-trapq replay reference").
    - First trapq divergence comes later (t = 4.05 s): a 0.085 mm/s cruise difference (follow-up section 1). It is upstream behaviour too.
    - Long prints are now judged against the live-trapq replay, which passes on both x15 runs.
- **G3: real-time headroom is large on a Pi 5, and smaller with `[corner_deviation]`.**
  - Densest corpus cases at owner-like, full Pi 5: klippy uses 4–8 % of one core (1 s peak ≤ 14 %). buffer_time stays ≥ 1.3 s while the host feeds (≈ 2.2 s median). No stalls, no `Timer too close`.
  - Without `corner_deviation`, both dense cases survive 1 core at 1500 MHz with a 25 % CFS quota (25 % of one core). They fail with `Timer too close` only at a 12 % quota.
  - With `[corner_deviation]`, `arcs_x30` already fails with `Timer too close` at 1 core / 1500 MHz with no quota. `voron_top_layers_x15` survives down to 12 %.
  - With `corner_deviation` on, klippy stalls its reactor at startup: Stats gaps up to 3.9 s at 1 core / 1500 MHz, 1.7–1.8 s at full speed. This is the live-only class of cost that batch mode cannot show.
- **G4 pass.** On owner-like (`corner_deviation` off), upstream `1ee39ff2c` and kv2:
  - send byte-for-byte the same decoded host→MCU message stream in batch mode (6 config × program cases);
  - produce identical captured step sequences on the bench (3 programs);
  - produce live taps that match after the per-run offset: 99.999 % of steps for voron, 93 % for arcs (max 0.02 µs), 93 % for speed_ladder (max 23.5 µs). The departures are stepcompress choosing other steps, within max_error.
- **Finding, fixed:** `[corner_deviation]` crashed live klippy about 1 s after ready. Its `stats` dict collided with `statistics.py`'s `.stats()` callback convention, giving `TypeError: 'dict' object is not callable`. Batch mode never runs the Stats timer, so motion_eval could not see it. The first session (kv2 a0306ad35) recorded all `owner_like_cd` rows as FAILED with this cause (Appendix A). Fixed on kv2/main 19348edc9; rerun on 5d53e78f8 (Appendix B).

## Setup

| part | what |
|---|---|
| host | Pi 5, kv2 tree from `git archive` in `~/kv2/trees/<sha>`, own venv `~/kv2/env`, `~/kv2/printer.cfg`, `~/kv2/logs/klippy.log`; the klipper service is pointed at it by a drop-in `klipper.service.d/kv2-host.conf`. `~/klipper` is never touched. |
| MCU | stock Kalico built from the kv2 commit under test (`tools/kv2_bench/f401.config`: STM32F401, 25 MHz crystal, no bootloader, USB PA11/PA12, `INITIAL_PINS=!PB4`, as serval's `f401.config`). It is flashed over the Pi's SWD, and the run requires the MCU to report the clean `git describe` (`v0.12.0-1141-ga0306ad35`, later `-g5d53e78f8`). |
| instrument | F407 event mode (11.9 ns) through serval's `capture.py` and `capture_events.py`, imported from the `kv2-bench-serval` worktree. Nothing is shared with kalico. |
| reference | `klippy -i/-o/-d` batch run on this box from the same commit, config (`serial` → `/dev/null`), gcode and F401 `klipper.dict`. The decoded queue_step stream covers every stepper including the extruder (`batch_capture.py`, `reference.py`). |
| live tap | `motion_report/dump_stepper` subscriptions over the API socket (`pi_agent.py`). This is the queue_step schedule the live host sent, with no production code changed. |
| pins | X PB12/13, Y PB14/15, Z PB6/7, E PB8/9; endstops ^PA9/^PA10/^PA15 (inputs); heater PA1 with `temperature_mcu` |

## Timing bound and its source

Each queue_step step lies in [ideal − min(25 µs, interval/2), ideal] (`stepcompress.c` `minmax_point`, `MAX_STEPCOMPRESS_ERROR = 0.000025` in `klippy/stepper.py`). The bound covers the live host and the batch run alike, so live−batch differs by at most ±25 µs. The MCU adds its execution latency above a floor. That latency is measured on the same run as capture vs live tap: one linear fit, F407 vs F401 crystal −69.3 ppm, floor wander < 1 µs over 90 s.

**Bound = 25 µs + 0.1 µs instrument + this run's MCU lateness max.**

| measured | value |
|---|---|
| MCU lateness above floor | p99 1.3–2.0 µs, max 2.3–9.2 µs (the max on two motors stepping together at 90 kHz) |
| live tap vs batch (same epoch offset) | single-pass voron and arcs (default config) identical; speed_ladder 82 %, owner arcs 90–93 %; max departure 24.99 µs < 25 µs |
| capture vs batch | max 29.2 µs ≤ bound 33.8 µs on all paired runs |

Every run is one epoch: the live and batch timelines differ by a single offset, and no run jumped mid-motion.

## Step rates (owner-like = owner-real; no scaling)

The owner's printer is 16 microsteps, rotation_distance 40, so 80 steps/mm on each CoreXY motor and 705 steps/mm of filament. The busiest owner-like program (`speed_ladder` at 800 mm/s) peaks at:
- 90.5 k steps/s on one motor, on the diagonals, where a single motor moves at v·√2;
- 128 k steps/s aggregate, on X-only moves.

The dense slicer case peaks at 77 k steps/s per motor and 89.5 k aggregate. All of these sit under the F401's measured ~172 k steps/s single-motor and ~300 k total ceiling (serval `tools/bench/README.md`, single-edge 2 µs). They also fit the event-capture USB budget. So the owner-like configs run the owner's real step rates, unscaled. A 32-microstep owner would need 181 k steps/s on one motor, which is beyond the F401. That was not run.

## Headroom method

- CPU comes from `/proc/<klippy>/stat` and per-thread stats sampled every 0.25 s over the print window (`cpu.jsonl`).
- buffer_time and print_stall come from klippy's 1 Hz `Stats` lines. They are judged only while print_time grows, because the buffer drains to 0 at the end by design.
- Host degradation uses serval's `throttle.py`: CPUAffinity 0 and the cpufreq `scaling_max_freq` cap, always restored. A `CPUQuota` drop-in (`kv2-quota.conf`) is added on top.
- Caveat [INFERENCE]: mean CPU % barely rises at the 1500 MHz cap (8.0 → 8.2 %), so its effect is not shown. The failures appear with 1 core plus quota, which means the limit is scheduling latency, not mean load.

## Restoration (every session)

Each session records the pre-state first: the deploy record, ExecStart argv, drop-ins, klippy cmdline, printer state, MCU version, printer.cfg sha and cpufreq caps. On exit it:
1. removes the kv2 drop-ins;
2. rebuilds the serval F401 firmware from the commit the deploy record names (`kv2-bench-fw`, build id 9e6ffb6d…, checked before any change);
3. flashes it with serval's `deploy.py flash-target`;
4. requires `restoration.json` `differences == []`.

All three sessions ended with `[]`. The interrupted first full session was restored in a separate locked step. `state.py` after the last session:

```
printer    ready ['Printer is ready']
deployed   2a9597f4b HEAD dirty=0 /home/dderg/Developer/serval-cbud @ 2026-11-06T06:31:09.861371+01:00
mcu serial /dev/serial/by-id/usb-Klipper_stm32f401xc_1800190001504E334B353720-if00
capture    present
```

## Reproduce

```
cd ~/Developer/kv2-bench
uv run python ~/Developer/kv2-bench-serval/tools/bench/lock.py hold -- \
    uv run python -m tools.kv2_bench.session [--gates g1,g2,g3,g4] \
    [--configs default,owner_like,owner_like_cd] [--g3-configs ...] [--kv2-ref kv2/main]
uv run python -m tools.kv2_bench.report ~/bench/kv2/<stamp>
```

Run it detached (`setsid nohup ...`): a tool harness that kills the job's process group skips the restore. The interrupted session showed this.

Sessions: `~/bench/kv2/20261107-120619` (G1 smoke), `-121054` (kv2 a0306ad35: G1, G2, G3 without cd; cd rows failed), `-124110` (kv2 5d53e78f8: owner-like G2, G2 + G3 with cd, G4).

## Appendix A: session 20261107-121054 (kv2/main a0306ad35)

### Executed vs planned (tap runs)

| gate | tree | config | program | print | net | paired | MCU ran every commanded step | capture−ref abs max µs | bound µs | MCU lateness p99 / max µs | tap = ref after offset | tap−ref abs max µs | epochs | slope ppm |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g1 | kv2 | default | speed_ladder | complete | True | True | True | 27.14 | 33.70 | 1.99 / 8.60 | 82.341% | 24.99 | 1 | -69.39 |
| g1 | kv2 | default | voron_top_layers | complete | True | True | True | 2.61 | 28.23 | 1.29 / 3.13 | 100.000% | 0.00 | 1 | -69.32 |
| g2 | kv2 | default | arcs | complete | True | True | True | 1.92 | 27.54 | 1.36 / 2.44 | 100.000% | 0.00 | 1 | -69.30 |
| g2 | kv2 | owner_like | speed_ladder | complete | True | True | True | 28.96 | 33.69 | 1.91 / 8.59 | 81.908% | 24.88 | 1 | -69.38 |
| g2 | kv2 | owner_like | voron_top_layers | complete | True | True | True | 8.39 | 34.03 | 1.62 / 8.93 | 100.000% | 0.00 | 1 | -69.29 |
| g2 | kv2 | owner_like | arcs | complete | True | True | True | 23.23 | 27.76 | 1.48 / 2.66 | 93.220% | 22.76 | 1 | -68.76 |
| g2 | kv2 | owner_like_cd | speed_ladder | FAILED | None | None | None | – | – | – / – | –% | – | – | – |
| g2 | kv2 | owner_like_cd | voron_top_layers | FAILED | None | None | None | – | – | – / – | –% | – | – | – |
| g2 | kv2 | owner_like_cd | arcs | FAILED | None | None | None | – | – | – / – | –% | – | – | – |

### Reference peak step rates (steps/s)

| config | program | steps | peak stepper_x | peak stepper_y | peak stepper_z | peak extruder | peak aggregate |
|---|---|---|---|---|---|---|---|
| default | speed_ladder | 3263768 | 56.6 k | 56.6 k | 0.0 k | 0.0 k | 80.0 k |
| default | voron_top_layers | 102580 | 24.5 k | 34.6 k | 0.1 k | 17.2 k | 56.3 k |
| default | arcs | 21372 | 16.7 k | 16.7 k | 0.0 k | 3.5 k | 27.1 k |
| owner_like | speed_ladder | 3266976 | 90.5 k | 90.5 k | 0.0 k | 0.0 k | 128.0 k |
| owner_like | voron_top_layers | 101748 | 38.6 k | 77.0 k | 0.5 k | 38.4 k | 89.5 k |
| owner_like | arcs | 22036 | 17.0 k | 17.0 k | 0.0 k | 3.5 k | 27.4 k |
| owner_like_cd | speed_ladder | 3267006 | 90.5 k | 90.5 k | 0.0 k | 0.0 k | 128.0 k |
| owner_like_cd | voron_top_layers | 102846 | 34.8 k | 77.0 k | 0.3 k | 38.4 k | 89.5 k |
| owner_like_cd | arcs | 22036 | 17.0 k | 17.0 k | 0.0 k | 3.5 k | 27.4 k |
| owner_like | arcs_x30 | 1049618 | 17.0 k | 17.0 k | 0.0 k | 3.5 k | 27.4 k |
| owner_like | voron_top_layers_x15 | 1579171 | 38.6 k | 77.0 k | 3.8 k | 38.5 k | 89.5 k |

### Headroom

| gate | config | program | tap | host | print | fatal lines | klippy CPU mean % | peak 1 s % | busiest threads (mean %) | buffer_time min / median s | stalls |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g1 | default | speed_ladder | True | 4 cpu, 2400 MHz | complete | 0 | 0.6 | 2.0 | python 0, stepper_x 0, stepper_y 0 | 1.999 / 2.410 | 0 |
| g1 | default | voron_top_layers | True | 4 cpu, 2400 MHz | complete | 0 | 1.2 | 2.0 | python 0, stepper_x 0, stepper_y 0 | 8.434 / 8.434 | 0 |
| g2 | default | arcs | True | 4 cpu, 2400 MHz | complete | 0 | 0.6 | 1.0 | python 0, stepper_x 0, stepper_y 0 | 0.690 / 0.690 | 0 |
| g2 | owner_like | speed_ladder | True | 4 cpu, 2400 MHz | complete | 0 | 0.6 | 2.0 | python 0, stepper_x 0, stepper_y 0 | 2.027 / 2.380 | 0 |
| g2 | owner_like | voron_top_layers | True | 4 cpu, 2400 MHz | complete | 0 | 5.5 | 8.0 | stepper_x 2, stepper_y 2, python 0 | 2.753 / 2.753 | 0 |
| g2 | owner_like | arcs | True | 4 cpu, 2400 MHz | complete | 0 | 2.3 | 3.0 | stepper_x 1, stepper_y 1, python 0 | 0.780 / 0.780 | 0 |
| g2 | owner_like_cd | speed_ladder | True | 4 cpu, 2400 MHz | FAILED: BenchError('printer did not reach ready within 60 s (last state: shutdown)') | 0 | – | – | – | – / – | – |
| g2 | owner_like_cd | voron_top_layers | True | 4 cpu, 2400 MHz | FAILED: BenchError('printer did not reach ready within 60 s (last state: shutdown)') | 0 | – | – | – | – / – | – |
| g2 | owner_like_cd | arcs | True | 4 cpu, 2400 MHz | FAILED: BenchError('printer did not reach ready within 60 s (last state: shutdown)') | 0 | – | – | – | – / – | – |
| g3 | owner_like | arcs_x30 | False | 4 cpu, 2400 MHz | complete | 0 | 4.2 | 7.0 | python 2, stepper_x 1, stepper_y 1 | 1.911 / 2.237 | 0 |
| g3 | owner_like | arcs_x30 | False | 1 cpu | complete | 0 | 4.2 | 8.0 | python 2, stepper_x 1, stepper_y 1 | 1.685 / 2.287 | 0 |
| g3 | owner_like | arcs_x30 | False | 1500 MHz | complete | 0 | 4.4 | 11.0 | python 2, stepper_y 1, stepper_x 1 | 1.639 / 2.249 | 0 |
| g3 | owner_like | arcs_x30 | False | 1 cpu, 1500 MHz | complete | 0 | 4.4 | 8.0 | python 2, stepper_x 1, stepper_y 1 | 1.768 / 2.240 | 0 |
| g3 | owner_like | arcs_x30 | False | 1 cpu, 1500 MHz, quota 50% | complete | 0 | 4.4 | 7.0 | python 2, stepper_x 1, stepper_y 1 | 1.438 / 2.277 | 0 |
| g3 | owner_like | arcs_x30 | False | 1 cpu, 1500 MHz, quota 25% | complete | 0 | 4.5 | 12.0 | python 2, stepper_x 1, stepper_y 1 | 1.375 / 2.260 | 0 |
| g3 | owner_like | arcs_x30 | False | 1 cpu, 1500 MHz, quota 12% | FAILED: BenchError("klippy entered state 'shutdown'") | 103 | – | – | – | – / – | – |
| g3 | owner_like | voron_top_layers_x15 | False | 4 cpu, 2400 MHz | complete | 0 | 8.0 | 11.0 | stepper_x 3, stepper_y 3, python 2 | 1.363 / 2.210 | 0 |
| g3 | owner_like | voron_top_layers_x15 | False | 1 cpu | complete | 0 | 7.9 | 11.0 | stepper_x 3, stepper_y 3, python 2 | 1.402 / 2.199 | 0 |
| g3 | owner_like | voron_top_layers_x15 | False | 1500 MHz | complete | 0 | 8.1 | 11.0 | stepper_x 3, stepper_y 3, python 2 | 1.421 / 2.207 | 0 |
| g3 | owner_like | voron_top_layers_x15 | False | 1 cpu, 1500 MHz | complete | 0 | 8.2 | 13.0 | stepper_x 3, stepper_y 3, python 2 | 1.299 / 2.210 | 0 |
| g3 | owner_like | voron_top_layers_x15 | False | 1 cpu, 1500 MHz, quota 50% | complete | 0 | 8.2 | 14.0 | stepper_x 3, stepper_y 3, python 2 | 1.989 / 2.148 | 0 |
| g3 | owner_like | voron_top_layers_x15 | False | 1 cpu, 1500 MHz, quota 25% | complete | 0 | 8.2 | 11.0 | stepper_x 3, stepper_y 3, python 2 | 1.988 / 2.145 | 0 |
| g3 | owner_like | voron_top_layers_x15 | False | 1 cpu, 1500 MHz, quota 12% | FAILED: BenchError("klippy entered state 'shutdown'") | 4 | – | – | – | – / – | – |

### Upstream vs kv2

| program | batch messages kv2 = upstream | captured sequences kv2 = upstream | upstream tap = kv2 tap after offset | tap kv2−upstream abs max µs |
|---|---|---|---|---|

## Appendix B: session 20261107-124110 (kv2/main 5d53e78f8)

### Executed vs planned (tap runs)

| gate | tree | config | program | print | net | paired | MCU ran every commanded step | capture−ref abs max µs | bound µs | MCU lateness p99 / max µs | tap = ref after offset | tap−ref abs max µs | epochs | slope ppm |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g2 | kv2 | owner_like | speed_ladder | complete | True | True | True | 29.03 | 34.31 | 2.01 / 9.21 | 81.908% | 24.88 | 1 | -69.49 |
| g2 | kv2 | owner_like | voron_top_layers | complete | True | True | True | 7.43 | 33.07 | 1.62 / 7.97 | 100.000% | 0.00 | 1 | -69.41 |
| g2 | kv2 | owner_like | arcs | complete | True | True | True | 23.23 | 27.91 | 1.49 / 2.81 | 93.220% | 22.76 | 1 | -68.88 |
| g2 | kv2 | owner_like | voron_top_layers_x15 | complete | True | False | True | – | – | 1.63 / 9.08 | –% | – | – | – |
| g2 | kv2 | owner_like_cd | speed_ladder | complete | True | True | True | 29.21 | 33.77 | 1.96 / 8.67 | 81.596% | 24.96 | 1 | -69.47 |
| g2 | kv2 | owner_like_cd | voron_top_layers | complete | True | True | True | 6.59 | 32.22 | 1.53 / 7.12 | 100.000% | 0.00 | 1 | -69.37 |
| g2 | kv2 | owner_like_cd | arcs | complete | True | True | True | 24.33 | 27.43 | 1.46 / 2.33 | 89.971% | 20.85 | 1 | -65.74 |
| g2 | kv2 | owner_like_cd | voron_top_layers_x15 | complete | True | False | True | – | – | 1.59 / 8.05 | –% | – | – | – |
| g4 | upstream | owner_like | speed_ladder | complete | True | True | True | 29.18 | 33.72 | 1.87 / 8.62 | 81.907% | 24.94 | 1 | -69.30 |
| g4 | upstream | owner_like | voron_top_layers | complete | True | True | True | 7.63 | 33.26 | 1.62 / 8.16 | 99.999% | 0.01 | 1 | -69.22 |
| g4 | upstream | owner_like | arcs | complete | True | True | True | 23.22 | 28.36 | 1.49 / 3.26 | 93.216% | 22.74 | 1 | -68.68 |

### Reference peak step rates (steps/s)

| config | program | steps | peak stepper_x | peak stepper_y | peak stepper_z | peak extruder | peak aggregate |
|---|---|---|---|---|---|---|---|
| owner_like | speed_ladder | 3266976 | 90.5 k | 90.5 k | 0.0 k | 0.0 k | 128.0 k |
| owner_like | voron_top_layers | 101748 | 38.6 k | 77.0 k | 0.5 k | 38.4 k | 89.5 k |
| owner_like | arcs | 22036 | 17.0 k | 17.0 k | 0.0 k | 3.5 k | 27.4 k |
| owner_like | voron_top_layers_x15 | 1579171 | 38.6 k | 77.0 k | 3.8 k | 38.5 k | 89.5 k |
| owner_like_cd | speed_ladder | 3267010 | 90.5 k | 90.5 k | 0.0 k | 0.0 k | 128.0 k |
| owner_like_cd | voron_top_layers | 103442 | 34.8 k | 77.0 k | 0.3 k | 38.3 k | 89.5 k |
| owner_like_cd | arcs | 22036 | 17.0 k | 17.0 k | 0.0 k | 3.5 k | 27.4 k |
| owner_like_cd | voron_top_layers_x15 | 1604539 | 34.8 k | 77.0 k | 3.8 k | 38.5 k | 89.5 k |
| owner_like_cd | arcs_x30 | 1049966 | 17.0 k | 17.0 k | 0.0 k | 3.5 k | 27.4 k |

### Headroom

| gate | config | program | tap | host | print | fatal lines | klippy CPU mean % | peak 1 s % | busiest threads (mean %) | buffer_time min / median s | stalls |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g2 | owner_like | speed_ladder | True | 4 cpu, 2400 MHz | complete | 0 | 0.6 | 2.0 | python 0, stepper_x 0, stepper_y 0 | 2.032 / 2.395 | 0 |
| g2 | owner_like | voron_top_layers | True | 4 cpu, 2400 MHz | complete | 0 | 6.8 | 9.0 | stepper_x 2, stepper_y 2, python 1 | 2.597 / 2.597 | 0 |
| g2 | owner_like | arcs | True | 4 cpu, 2400 MHz | complete | 0 | 2.9 | 4.0 | python 1, stepper_x 1, stepper_y 1 | 0.755 / 0.755 | 0 |
| g2 | owner_like | voron_top_layers_x15 | True | 4 cpu, 2400 MHz | complete | 0 | 8.4 | 11.0 | stepper_x 3, stepper_y 3, python 2 | 1.400 / 2.219 | 0 |
| g2 | owner_like_cd | speed_ladder | True | 4 cpu, 2400 MHz | complete | 0 | 0.6 | 2.0 | python 0, stepper_y 0, stepper_x 0 | 2.000 / 2.328 | 0 |
| g2 | owner_like_cd | voron_top_layers | True | 4 cpu, 2400 MHz | complete | 0 | 3.4 | 7.0 | python 1, stepper_x 1, stepper_y 1 | 4.778 / 4.778 | 0 |
| g2 | owner_like_cd | arcs | True | 4 cpu, 2400 MHz | complete | 0 | 2.3 | 4.0 | python 1, stepper_x 1, stepper_y 1 | 0.871 / 0.871 | 0 |
| g2 | owner_like_cd | voron_top_layers_x15 | True | 4 cpu, 2400 MHz | complete | 0 | 4.5 | 7.0 | python 2, stepper_x 1, stepper_y 1 | 2.009 / 2.353 | 0 |
| g3 | owner_like_cd | arcs_x30 | False | 4 cpu, 2400 MHz | complete | 0 | 5.9 | 9.0 | python 3, stepper_x 2, stepper_y 2 | 2.010 / 2.273 | 0 |
| g3 | owner_like_cd | arcs_x30 | False | 1 cpu | complete | 0 | 5.7 | 10.0 | python 3, stepper_x 2, stepper_y 2 | 1.269 / 2.258 | 0 |
| g3 | owner_like_cd | arcs_x30 | False | 1500 MHz | complete | 0 | 6.1 | 16.0 | python 3, stepper_x 2, stepper_y 2 | 1.910 / 2.277 | 0 |
| g3 | owner_like_cd | arcs_x30 | False | 1 cpu, 1500 MHz | FAILED: BenchError("klippy entered state 'shutdown'") | 4 | – | – | – | – / – | – |
| g3 | owner_like_cd | arcs_x30 | False | 1 cpu, 1500 MHz, quota 50% | FAILED: BenchError("klippy entered state 'shutdown'") | 4 | – | – | – | – / – | – |
| g3 | owner_like_cd | voron_top_layers_x15 | False | 4 cpu, 2400 MHz | complete | 0 | 4.2 | 8.0 | python 1, stepper_x 1, stepper_y 1 | 2.007 / 2.350 | 0 |
| g3 | owner_like_cd | voron_top_layers_x15 | False | 1 cpu | complete | 0 | 4.1 | 6.0 | python 1, stepper_x 1, stepper_y 1 | 2.008 / 2.337 | 0 |
| g3 | owner_like_cd | voron_top_layers_x15 | False | 1500 MHz | complete | 0 | 4.3 | 9.0 | python 1, stepper_x 1, stepper_y 1 | 1.932 / 2.338 | 0 |
| g3 | owner_like_cd | voron_top_layers_x15 | False | 1 cpu, 1500 MHz | complete | 0 | 4.3 | 10.0 | python 2, stepper_x 1, stepper_y 1 | 1.653 / 2.348 | 0 |
| g3 | owner_like_cd | voron_top_layers_x15 | False | 1 cpu, 1500 MHz, quota 50% | complete | 0 | 4.3 | 10.0 | python 2, stepper_x 1, stepper_y 1 | 1.524 / 2.335 | 0 |
| g3 | owner_like_cd | voron_top_layers_x15 | False | 1 cpu, 1500 MHz, quota 25% | complete | 0 | 4.3 | 11.0 | python 2, stepper_x 1, stepper_y 1 | 1.715 / 2.321 | 0 |
| g3 | owner_like_cd | voron_top_layers_x15 | False | 1 cpu, 1500 MHz, quota 12% | complete | 0 | 4.3 | 7.0 | python 1, stepper_x 1, stepper_y 1 | 1.933 / 2.338 | 0 |
| g4 | owner_like | speed_ladder | True | 4 cpu, 2400 MHz | complete | 0 | 0.6 | 2.0 | python 0, stepper_x 0, stepper_y 0 | 2.014 / 2.364 | 0 |
| g4 | owner_like | voron_top_layers | True | 4 cpu, 2400 MHz | complete | 0 | 5.2 | 9.0 | stepper_x 2, stepper_y 2, python 0 | 2.529 / 2.529 | 0 |
| g4 | owner_like | arcs | True | 4 cpu, 2400 MHz | complete | 0 | 1.3 | 2.0 | stepper_x 1, stepper_y 1, python 0 | 0.843 / 0.843 | 0 |

### Upstream vs kv2

| program | batch messages kv2 = upstream | captured sequences kv2 = upstream | upstream tap = kv2 tap after offset | tap kv2−upstream abs max µs |
|---|---|---|---|---|
| arcs | True | True | 93.220% | 0.02 |
| speed_ladder | True | True | 92.759% | 23.54 |
| voron_top_layers | True | True | 99.999% | 0.01 |


## Follow-up: x15 divergence and corner_deviation stalls (session `~/bench/kv2/20261107-131347`, kv2/main 312ddf16a)

This session was evidence only; no production code changed. Its tools:
- `trapq_diff.py` compares the live toolhead trapq (`motion_report/dump_trapq` via `pi_agent.py --trapqs toolhead`) with the batch `trapq_append` record.
- `klippy_probe.py` is an instrumented launcher, started by the service drop-in. It records timer lateness against the requested waketime, callback wall times and the CornerTable build. It does not modify the klippy tree.
- py-spy 0.4 (`record --nonblocking -f raw`, into `~/kv2/env`) profiled the print. `profile_report.py` summarizes the result.

The session was restored; `restoration.json` shows no differences.

### 1. Live vs batch divergence on `voron_top_layers` ×15 is upstream behaviour

| tree | live trapq phases | batch phases | first diverging phase | batch move | gcode | live−batch |
|---|---|---|---|---|---|---|
| upstream 1ee39ff2c | 44412 | 44341 | 3738 (batch t = 4.0505 s) | 1649 | program line 1769 `G1 X118.946 Y145.494 E.00914` | cruise 368.827 vs 368.742 mm/s; accel phase 123 vs 121 µs, decel 102 vs 100 µs; same start_v, accel, direction and start time |
| kv2 312ddf16a | 44412 | 44341 | 3738 | 1649 | same line | identical numbers |

- **Where.** It is the 2nd repetition: move 1649 = 1313 (1st repetition + return travel) + 336. The same passage in the 1st repetition (move 336) matches batch exactly.
- **What differs.** The planned cruise speed of one mid-perimeter move differs by 0.085 mm/s. There is no stop, no Z move and no M400/G4 nearby. It is not the repetition boundary.
- **Upstream too.** Upstream and kv2 reproduce it at the same phase with the same values.
- **Feed path.** virtual_sdcard: Moonraker `POST /printer/print/start` → `SDCARD_PRINT_FILE`.
- **Forced-stop hypothesis refuted.** The hypothesis was a non-lazy flush (priming timer or `_handle_step_flush` runout) when gcode arrives slowly. The data rule it out:
  - no stop was inserted (start_v identical, no zero-velocity phase);
  - `print_stall` stayed 0;
  - buffer_time stayed 2.0–3.0 s around the divergence (klippy.log Stats lines at print_time 43.5–46.7 s on upstream: buffer_time 2.967, 2.366, 2.196, 2.043, 2.169).
- **[INFERENCE] likely mechanism, not verified.** The lookahead flush window differs.
  - Live klippy leaves the NeedPrime/Priming state, where the window is `set_flush_time(BUFFER_TIME_HIGH)` = 2 s.
  - It enters the main state, where the window is `LOOKAHEAD_FLUSH_TIME` = 0.25 s. That transition is driven by real time (`_check_pause` / `_priming_handler`).
  - Batch mode has no real time, so its lazy flushes cut the lookahead queue at other moves.
  - The `minimum_cruise_ratio` smoothing of a move near a cut then sees a different window.
  - This fits the observed pattern: the 1st repetition matches, and a later move differs by a fraction of a mm/s.
- **Consequence for the harness.** Long dense programs cannot be judged step-for-step against batch mode. Only net steps and live-tap vs capture are valid there. Single-pass corpus cases (≤ 3 s) are unaffected.

### 2. corner_deviation reactor stalls and the G3 `Timer too close`

Failures on kv2 5d53e78f8 (session `-124110`), `owner_like_cd` `arcs_x30`:

| host | fatal lines | MCU shutdown received | host state at failure |
|---|---|---|---|
| 1 cpu, 1500 MHz | `MCU 'mcu' shutdown: Timer too close` / `Transition to shutdown state: MCU shutdown` | 4.99 s after print start | buffer_time 2.0–2.9 s; Stats shows print_time jumping 1.7 s in one 1 s interval just before |
| 1 cpu, 1500 MHz, quota 50 % | same | 4.98 s after print start | buffer_time 2.1–2.9 s; print_time +1.5 s in one interval |

The host had 2 s of motion queued. So this is not underrun of the motion queue. A `Timer too close` with a full buffer points to a host-scheduled MCU command sent late during a long reactor stall [INFERENCE]: heater/verify_heater refresh or clock sync, not step data.

On 312ddf16a, whose klippy/ is identical to 5d53e78f8, the same two conditions passed (1 run each). The failure is intermittent: 2 of 2 failed earlier, 0 of 2 failed here.

Probed runs, 312ddf16a. "Timer lateness" is the largest delay between a timer's requested waketime and its run. It is the measurable reactor stall.

| config | host | connect phase wall s | CornerTable build s | table done before print start | max timer lateness during print | py-spy top self (share of samples) |
|---|---|---|---|---|---|---|
| owner_like | full | 0.76 | – | – | 7 ms (verify_heater check_event) | reactor poll 12 %, `_process_lookahead` 7 %, `cmd_G1` 6 %, `LookAheadQueue.flush` 5 %, `set_junction` 5 % |
| owner_like | 1 cpu, 1500 MHz | 0.82 | – | – | 36 ms (webhooks query) | `motion_queuing._advance_flush_time` (C step generation) 32 %, reactor 8 %, extruder `process_move` 6 % |
| owner_like_cd | full | 1.50 | 0.62 | yes, 3.0 s before | 53 ms (verify_heater check_event) | reactor 9 %, `CornerDeviation._step_bound` 5 %, `_process_lookahead` 4 %, `set_junction` 4 %, `planArc` 4 % |
| owner_like_cd | 1 cpu, 1500 MHz | 1.93 | 0.98 | yes, 3.9 s before | 88 ms (webhooks query) | `_advance_flush_time` 30 %, reactor 8 %, `cmd_G1` 3 % |
| owner_like_cd | 1 cpu, 1500 MHz, quota 50 % | 3.12 | 1.99 | yes, 3.3 s before | 103 ms (webhooks query) | `_advance_flush_time` 31 %, reactor 8 %, `corner_deviation.junction_v2` 2.5 % |

- **Table build.** The CornerTable is built in the klippy:connect handler, before ready. It finished 3–4 s before every print started, so it was never in progress during a print.
- **Startup.** The connect phase (ready-gating callback) took 1.5–3.1 s with `[corner_deviation]`, against 0.8 s without. Process start to print start was 4.5–6.6 s; that includes Moonraker polling.
- **Probe gap.** The probe misses the ready instant: klippy calls the `klippy:ready` handlers directly, not through `send_event`. So lateness is split at print start, not at ready. Everything after ready but before print start was ≤ 12 ms late.
- **Stalls during the print.** With `corner_deviation` the reactor stalls 2.4–7× longer than without (53–103 ms vs 7–36 ms) under the same host limits. At 1 core / 1500 MHz the largest stalls approach the 100 ms scale of host-scheduled MCU deadlines, which is consistent with the intermittent `Timer too close`.
- **Profile caveats.**
  - The longest single callbacks (virtual_sdcard `work_handler`, `ReactorCallback.invoke`) pause inside themselves, so their wall times (51 s, ~0.24 s) are not stalls.
  - py-spy self time in `_advance_flush_time` includes the GIL-released C step generation (`steppersyncmgr_gen_steps`), which competes for the single core.

## Live-trapq replay reference (offline, data of session `-131347`)

`replay.py` regenerates the expected step stream from the live run's own toolhead (and extruder) trapq tap:
- The batch-mode klippy of the tree under test runs `replay_capture.py`. It sets the start position, appends every recorded trapq phase at its live print_time, and generates steps in the live background flusher's 0.25 s batches.
- Steps go through the unmodified chelper itersolve, input shaper and stepcompress path, with the run's printer.cfg. The shaper comes from the config; the allowlist forbids `SET_INPUT_SHAPER` in programs.
- So it judges step generation, transport and MCU execution, while `trapq_diff.py` judges the planner. Both run on the live print_time axis, so the expected shared offset is 0 ticks.

| run | replay = tap sequence (X / Y / Z steps) | shared offset | steps at identical clock | max departure | capture vs replay | bound |
|---|---|---|---|---|---|---|
| upstream x15 | yes (617366 / 616751 / 23200) | 0 ticks | 98.32 % | 24.71 µs ≤ 25 µs | 24.86 µs | 34.93 µs (MCU lateness max 9.83 µs) |
| kv2 x15 | yes (617366 / 616751 / 23200) | 0 ticks | 98.32 % | 24.71 µs ≤ 25 µs | 24.78 µs | 34.10 µs (MCU lateness max 9.00 µs) |

These runs predate the extruder trapq tap, so their extruder was not replayed. `pi_agent` now taps `toolhead,extruder`.

### Offline self-check

`replay.self_check` replays the trapqs a batch run recorded (`batch_capture.py` now saves them), extruder included, and compares with that batch run's own steps. Owner_like, owner_like_cd and default configs, on voron_top_layers, arcs, speed_ladder, voron ×15 and one soak cycle:
- 13 of 14 cases give identical sequences, offset 0, max departure ≤ 24.96 µs.
- The exception is owner_like voron ×15: replay has 617404 X steps, batch 617406.
  - The difference is one ±1 reversal pair at t ≈ 3.97 s, which batch emits and replay drops. It is exactly where the live run diverged from batch.
  - So itersolve's detection of a sub-step excursion depends on the step-generation window boundaries. That is the first cause of the live/batch x15 divergence, ahead of the 4.05 s planner difference.
  - It is also the residual risk of the replay reference: a window-dependent ±1 pair would show as an adjacent-pair mismatch with equal net steps. Neither live x15 replay showed one.

## Soak session (prepared, not run)

```
uv run python ~/Developer/kv2-bench-serval/tools/bench/lock.py hold -- \
    uv run python -m tools.kv2_bench.session --gates soak --configs owner_like,owner_like_cd
```

- **Program.** voron_top_layers, arcs, circles, zigzag_infill and corners_turn90, concatenated with a travel to each slice's start. The cycle count comes from one cycle's batch plan: ×85 for owner_like and ×76 for owner_like_cd, ≈ 30 min each.
- **Matrix.** Full Pi 5 and 1 cpu / 1500 MHz, for each config.
- **Recording.** Event-mode capture, stepper and trapq tap (toolhead, extruder), klippy_probe timer lateness, CPU samples.
- **Judging.** Each run is judged by `replay.judge`: replay = tap sequence, departure ≤ 25 µs, capture = replay sequence, capture within 25 µs + MCU lateness. Batch is not used for these runs. Results go to `<run>/replay/replay_judgement.json`.
- **Size.** About 4 h of bench time including restarts. A tap is ~160 kB/s, so about 300 MB per run on the Pi; it is deleted after each fetch.
- Run it detached (`setsid nohup`).

## Data request: timing of the corner_deviation lateness peaks and the two failures

Session `-131347`, probe top-40 lists, times relative to print start (virtual_sdcard start):

| run | max timer lateness | at | late callback | default (no cd) same host, first 1.5 s |
|---|---|---|---|---|
| owner_like_cd full | 53.1 ms | +0.067 s | `verify_heater.HeaterCheck.check_event` (with `statistics.generate_stats` 26.8 ms at the same instant) | not in the run's top-40 lateness list, so < 5.7 ms; its max of 7.1 ms is at +38 s |
| owner_like_cd 1 cpu/1500 | 87.6 ms | +0.118 s | `webhooks.QueryStatusHelper._do_query` | 36.3 ms at +0.085 s (`_do_query`) |
| owner_like_cd 1 cpu/1500 + 50 % | 102.8 ms | +0.169 s | `webhooks.QueryStatusHelper._do_query` | – (not run without cd) |

After the first 1.5 s the cd runs' worst lateness is 12.3 / 21.7 / 21.8 ms, the latter two at +1.61 s.

So the cd maxima sit in the print-start burst, consistent with the priming hypothesis. Without cd the same burst stays ≤ 36 ms on the slow host and does not appear in the top-40 at full speed.

The two `Timer too close` failures on 5d53e78f8 (session `-124110`, `owner_like_cd` `arcs_x30`, 1 cpu/1500 and the same with 50 % quota) **refute the priming hypothesis for themselves**:
- The MCU shutdown clock is 496631016. The last `clock` reply, 447987208 received at +4.409 s, extrapolates that to +4.99 s, and the shutdown report arrived at +4.988 s.
- The send-queue dump covers the last 100 messages from +2.11 to +4.987 s. The last host message (a queue_step batch) went out at +4.987 s, 1 ms before the shutdown.
- The timer that fired too late was therefore scheduled at ~+4.98 s (+4.984 s in the quota run), well after the start burst.
- buffer_time read 1.5–2.9 s at the time, and Stats shows print_time jumping 1.5–1.7 s in the last interval before the shutdown.
- Both logs dump `clocksync state` with an estimated MCU frequency of 83.18 / 83.24 MHz (−0.97 %, nominal 84 MHz). The estimate's `time_avg` is 4.8 s / 5.7 s before print start, and `pred_variance` is 1.4e10 / 1.3e9. So the host's clock estimate was stale and degraded when the late step batch was sent.
- [INFERENCE] The failure mechanism is a mid-print (+5 s) reactor stall that delayed step generation/transmission, compounded by a degraded clock estimate. It is not the priming burst. The probe was not running in those two runs, so the stalled callback is not identified.

## Trials session (prepared, not run)

```
uv run python ~/Developer/kv2-bench-serval/tools/bench/lock.py hold -- \
    uv run python -m tools.kv2_bench.session --gates trials,soak --configs owner_like,owner_like_cd [--trials 10]
```

- **Matrix.** `arcs_x30` at 1 cpu / 1500 MHz, N = 10 per config, interleaved: owner_like_cd, owner_like, owner_like_cd, ...
- **Judging.** Batch reference (arcs_x30 matched batch step for step on every earlier run).
- **Probe.** `klippy_probe.py` now has a stall watchdog thread. Every 5 ms it checks the earliest requested timer waketime.
  - Once a timer is 20 ms overdue, the stall is still in progress. The watchdog records:
    - every thread's stack (`sys._current_frames`); the main thread's stack is the stalled reactor callback;
    - the toolhead's queuing state (Priming / NeedPrime / main), lookahead queue length and print_time;
    - the overdue timer.
  - It then starts `sudo py-spy dump` of klippy, all threads, outside klippy's CPU affinity. It tries `--native` first and falls back without it; `pyspy_native` records which worked (py-spy 0.4.3 on the Pi's aarch64 lists the flag).
  - Each timer more than 20 ms late is logged with the same toolhead state.
  - All lateness of 2 ms or more is sampled, plus a histogram. Ready and shutdown are now timestamped by event handlers.
  - `tests/test_klippy_probe.py` runs the probe on a stub klippy whose timer sleeps 150 ms. It checks that the stall's stack names the sleeping callback, and that the toolhead state is captured.
- **Report.** `trials_report.py <session>` gives, per config:
  - failure rate;
  - per-run maximum lateness (p50/p90/max) and pooled lateness ≥ 2 ms;
  - counts of ≥ 20 ms events by toolhead state;
  - for each failure: shutdown time since print start, and the stalls in the 2 s before it, with overdue timer, toolhead state, main-thread stack and py-spy dump.
- **Soak.** The soak rows judge the extruder too: `pi_agent` taps the toolhead and extruder trapqs, and the replay regenerates every tapped trapq.

## Trials + soak on kv2/main 533396833 (session `~/bench/kv2/20261107-135731`)

kv2/main 533396833 is merged into kv2/bench. It changed corner_deviation:
- the per-junction limit is now in C;
- the table is built in a nice-19 thread and cached as npz;
- gcode is held until the table is published.

Setup for this session:
- The table cache is `/home/pi/.cache/kalico/corner_deviation`. The service user (`pi`) creates it and must be able to write it; the session checks this with `test -w`. The path is recorded in the kv2 deploy record `~/kv2/.kv2-deploy.json`, together with the commit, tree and firmware.
- The first owner_like_cd trial started from an emptied cache. The `cleared_cache_files` it recorded show the cache was not already empty, because the g2 rows had warmed it.
- The session ran detached under one lock hold and was restored at the end: `restoration.json` differences `[]`, `state.py`: printer ready, deployed 2a9597f4b, MCU serial present, capture present.

### Single-pass rows: batch vs live (the C limiter keeps the identity)

| config | program | tap = batch sequence | steps at the same clock | max departure | capture−batch / bound |
|---|---|---|---|---|---|
| owner_like | speed_ladder | yes | 81.9 % | 24.88 µs | 28.92 / 33.61 µs |
| owner_like | voron_top_layers | yes | 100 % | 0 | 8.21 / 33.85 µs |
| owner_like | arcs | yes | 93.2 % | 22.74 µs | 23.21 / 27.76 µs |
| owner_like_cd | speed_ladder | yes | 81.6 % | 24.96 µs | 29.52 / 34.68 µs |
| owner_like_cd | voron_top_layers | yes | 100 % | 0 | 6.73 / 32.36 µs |
| owner_like_cd | arcs | yes | 90.0 % | 20.85 µs | 24.32 / 27.58 µs |

The voron ×15 rows were judged by the live-trapq replay, extruder included. All checks passed:
- owner_like: replay = tap at 98.8 % of steps, max 24.70 µs; capture 24.70 µs against a bound of 34.27 µs.
- owner_like_cd: replay = tap at 98.3 % of steps, max 25.00 µs; capture 25.48 µs against a bound of 33.28 µs.

### Trials: arcs_x30, 1 cpu / 1500 MHz, 10 per config, interleaved

| config | runs | failures | per-run max lateness p50 / p90 / max | pooled lateness ≥ 2 ms: p99 / max | ≥ 20 ms events by toolhead state |
|---|---|---|---|---|---|
| owner_like | 10 | 0 | 26.4 / 76.1 / 79.1 ms | 74.9 / 79.1 ms | main 6 |
| owner_like_cd, cold cache (trial 0) | 1 | 0 | 23.3 ms | 22.8 / 23.3 ms | main 1, NeedPrime 1 |
| owner_like_cd, warm cache | 9 | 1 | 24.8 / 31.1 / 31.5 ms | 29.8 / 31.5 ms | main 6, NeedPrime 8 |

- **Table build.** Cold build 1.30 s; warm cache load 0.19–0.20 s. Both were published 0.19 s (warm) and 1.30 s (cold) after klippy:ready, before the print started (gcode held).
- **Lateness during the build.** The worst timer lateness while the build thread ran was 0 ms. The cold run's 23.3 ms maximum came later, at +0.11 s after print start (`webhooks._do_query`, toolhead main).
- **corner_deviation is no longer worse than the default.**
  - The largest lateness is now in the default config: 67–79 ms at +0.05–0.11 s after print start, in the print-start burst (`_do_query` overdue, toolhead main).
  - With cd, gcode is held until the table is published, so the start burst is smaller: ≤ 31.5 ms.
- **py-spy.** `--native` works on the Pi (`pyspy_native: true`).

**The one failure (warm owner_like_cd, run `141954`): `MCU 'mcu' shutdown: Timer too close` at +6.74 s.**
- The watchdog caught no stall. The run's maximum timer lateness was 13.3 ms, so the reactor was responsive, and there is no stall stack to give.
- The shutdown arrived 1 ms after the host sent a `queue_step oid=3/0 interval=10500 count=2000` batch. That batch's first step (clock 2680927396) was ~4.6 ms before the MCU's shutdown clock 2681311075, so the step data arrived late. buffer_time was 2.0–3.0 s throughout.
- The host's MCU clock estimate was wrong from klippy start:
  - every Stats line from −3.5 s to +6.5 s shows `freq=83220190` (−0.93 %), frozen;
  - passing runs show 84.006–84.007 MHz, updating;
  - the shutdown dump reads `clock_est=(152830.558 … 83220190.983) pred_variance=1.49e10`.
- The connect-time calibration (8 `get_clock` samples in `ClockSync.connect`) finished before the corner table build started (probe: build start at +0.875 s after process start, after connect). So the table thread cannot have skewed it in this run.
- **Same pattern in the two earlier failures on 5d53e78f8.** All three `Timer too close` failures share the same signature:
  - an estimate of 83.18 / 83.24 / 83.22 MHz, frozen;
  - the shutdown 9.8 / 10.7 / 10.6 s after the estimate's last accepted sample (`time_avg`).
- **[INFERENCE] mechanism.** That matches `_handle_clock`'s outlier filter: it ignores samples where `clock > exp_clock` for up to 10 s after the last accepted one. A low frequency estimate therefore stays frozen for 10 s while est_print_time falls behind the MCU, and the host transmits step data too late.
- This is upstream clocksync code. The bad initial estimate is a calibration taken under 1-core load. All three failures were owner_like_cd runs, but the calibration precedes the table build, and with 3 failures in about 25 cd runs and 0 in about 25 default runs at 1 cpu, the link to corner_deviation is not established.

### Soak: ~34 min dense print (corpus slices ×85 / ×76), judged by the live-trapq replay

| config | host | X / Y / Z / E steps | replay = tap (sequence, ≤ 25 µs) | steps at the same clock | capture = replay sequence | capture−replay (single fit) / bound | MCU lateness, 10 s-window fit p99 / max | klippy CPU mean / peak | buffer_time min | timer lateness max | stalls ≥ 20 ms | fatal |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| owner_like | full | 19277326 / 16906296 / 2840000 / 4811225 | yes, 25.00 µs, offset 0 | 69.6 % | yes | 40.0 / 62.5 µs | 1.47 / 9.26 µs | 3.4 / 16 % | 1.80 s | 21.7 ms | 2 | 0 |
| owner_like | 1 cpu / 1500 MHz | same | yes, 25.00 µs, offset 0 | 70.0 % | yes | 84.4 / 126.1 µs | 1.47 / 9.89 µs | 3.5 / 17 % | 1.39 s | 51.1 ms | 14 | 0 |
| owner_like_cd | full | 17365162 / 15288134 / 2537600 / 4301853 | yes, 25.00 µs, offset 0 | 74.9 % | yes | 39.8 / 61.2 µs | 1.38 / 9.61 µs | 3.4 / 12 % | 1.17 s | 21.1 ms | 2 | 0 |
| owner_like_cd | 1 cpu / 1500 MHz | same | yes, 25.00 µs, offset 0 | 74.9 % | yes | 48.2 / 78.6 µs | 1.38 / 8.90 µs | 3.4 / 12 % | 1.18 s | 36.2 ms | 4 | 0 |

- All four soaks pass every replay-judge check.
- The X/Y/Z/E step counts are identical between the two hosts for each config, so the live plan was the same at 1 cpu.
- **Single-fit bounds are loose.** On a 34-minute run, one linear F407↔F401 clock fit absorbs no crystal wander. The "MCU lateness" term and bound of the single-fit check therefore grow to 36–101 µs (p50 13–31 µs).
- **Windowed fits restore the short-run numbers.** Refitting capture vs live tap per 10 s window gives MCU lateness p50 0.52 µs, p99 1.4–1.5 µs, max 8.9–9.9 µs, the same as the short runs. The tight capture-vs-replay result with the same windows is below.
- **Tight capture-vs-replay check.** With 10 s windows, capture vs replay is 28.27 / 27.57 / 28.18 / 26.56 µs (row order of the table). The bounds are 25 µs + window MCU lateness max + 0.1 µs = 34.36 / 34.99 / 34.71 / 34.00 µs, so all four pass with the tight bound too. Next tool change: make `replay.judge` and `compare.timeline` use windowed fits for runs longer than a few minutes; today the single fit only gives the loose bound above.

## Frozen clock estimate → `Timer too close`: root cause and fix (kv2/clocksync)

### Reproduction (session `~/bench/kv2/20261107-172702`, kv2/main 713e70c9d, 1 cpu / 1500 MHz)

Each trial is a klippy start plus 15 s, either printing arcs ×15 or idle, interleaved, 30 of each. The probe records every clocksync sample: send and receive time, MCU clock, decision, and the estimate after it.

| kind | runs | `Timer too close` | connect calibration ≈ −1 % (−9.6k…−10.6k ppm) | calibration range |
|---|---|---|---|---|
| owner_like_cd print | 30 | 1 | 6 | −10366 … +2241 ppm |
| owner_like print | 30 | 4 | 7 | −10626 … +2012 ppm |
| owner_like idle | 30 | 0 | 4 | −10627 … +2156 ppm |

- **Rates.** 17 of 90 starts (19 %) calibrate about 1 % low, independent of corner_deviation and of motion. Of the 13 low starts that printed, 5 shut down with `Timer too close` (8 % of printing starts).
- **Every low start freezes.** It ignores 9–10 consecutive samples, then resets.
- **Every failure had a low start.**

### Mechanism

`ClockSync.connect` seeds its regression with the `get_uptime` reply, then adds 8 `get_clock` samples 50 ms apart with DECAY = 1/30, so the uptime anchor keeps ~76 % weight.

- The uptime reply follows identify and its latency is not checked. In every −1 % start it read the MCU clock 3.2–3.6 ms later than the 8 `get_clock` samples' line predicts. In good starts the gap is −0.1…−0.6 ms.
- Over the 0.36 s calibration span that skews the slope to −1 %. A least-squares fit over the 8 samples alone gives −157 and +1130 ppm for the two worst runs.
- A low estimate predicts too small a clock, so every later sample appears to lead the prediction. `_handle_clock` ignores leading samples for 10 s after the last accepted one, on the assumption that they were sent late. The estimate therefore freezes.
- est_print_time falls 0.93 % behind the MCU: ~93 ms at 10 s. The serialqueue releases step messages late, and the MCU shuts down with `Timer too close` 9.8–10.7 s after the last accepted sample.
- All four earlier bench failures fit: 83.18–83.24 MHz estimates, frozen, failing at about 10 s.

Klipper master and Kalico main have the same code: no upstream fix to port.

### Fix (kv2/clocksync, 2 commits on 713e70c9d)

`klippy/clocksync.py` +70 −28, `klippy/mcu.py` ±1, test +150, data +100 (321 insertions, 29 deletions).

1. **Calibration.** `get_uptime` only extends the clock to 64 bits. The estimate is seeded with a least-squares fit of the 8 `get_clock` samples, whose round trips are measured.
2. **Outlier filter.** A sample may lead the prediction by at most its own round trip, because a late send reads the clock at most that late. A larger lead means the estimate is wrong: the prediction variance is reset and the sample is used instead of ignored. A low estimate can no longer freeze.
3. **Loud failure.** A calibration more than 5 % off CLOCK_FREQ raises `ClockSyncError` ("MCU clock measured at … Hz, …% off its CLOCK_FREQ …"). `mcu.py` turns it into an MCU connect error. 5 % leaves room for internal RC oscillators, which run ±1–3 %.
4. **SecondarySync** inherits connect and `_handle_clock`, so it gets all three; covered by a test.

**Tests.** `test/test_clocksync.py` replays the recorded failing start, `test/data/clocksync_late_uptime.json` (late uptime, 8 calibration samples, 10 samples after), into ClockSync. All four tests fail on the old code:
- calibration −10428 ppm vs the bound of 3000;
- prediction error growing to 10.3 ms vs the bound of 5 ms;
- no `ClockSyncError`.

They pass on the fix: calibration +1063 ppm, prediction error ≤ 1.55 ms, a clear error for an 8 %-off clock, secondary MCU within bound. Gate: `PASS kv2-gate: pytest[230 passed, 4 xfailed]`.

### Verification (session `~/bench/kv2/20261107-180624`, kv2/clocksync 82f014e79, same trial set)

| kind | runs | `Timer too close` | calibration range | samples ignored after connect |
|---|---|---|---|---|
| owner_like_cd print | 30 | 0 | −567 … +1306 ppm | 0 |
| owner_like print | 30 | 0 | −501 … +1316 ppm | 0 |
| owner_like idle | 30 | 0 | −558 … +1366 ppm | 0 |

Worst estimate error after connect over all 90 starts: max 1411 ppm, median 594 ppm. The baseline was max 9904 ppm, median 1464 ppm, with 17 runs ignoring up to 10 samples.

Soak on the fix (owner_like_cd ×84 ≈ 34 min, replay judge with the new 10 s windowed clock fit):

| host | X / Y / Z / E steps | replay = tap | capture−replay / bound | MCU lateness max | klippy CPU | buffer_time min | fatal |
|---|---|---|---|---|---|---|---|
| full | 19060250 / 16736950 / 2806400 / 4754618 | yes, 73.0 % at the same clock, max 25.00 µs | 27.79 / 34.84 µs | 9.74 µs | 4.6 % | 1.38 s | 0 |
| 1 cpu / 1500 MHz | same | yes, 73.3 %, max 25.00 µs | 27.26 / 34.95 µs | 9.85 µs | 4.7 % | 1.25 s | 0 |

- **Every check passes.** The step counts at 1 cpu equal those at full speed.
- **The windowed clock fit (`compare.timeline` for runs > 120 s) gives the tight bound directly.** Re-judging the earlier 1-cpu owner_like soak gives 27.57 µs against a bound of 35.00 µs, where the single fit gave 84.4 / 126.1 µs.
- **Restoration:** differences `[]`; `state.py`: printer ready, deployed 2a9597f4b, MCU serial present, capture present.
