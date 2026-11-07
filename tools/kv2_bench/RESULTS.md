# kv2 on the hardware bench: results

## Conclusion

- **G1 pass.** kv2 klippy runs live on the Pi 5 against stock Kalico F401 firmware built from kv2. It printed `speed_ladder.gcode` and the corpus cases `voron_top_layers` and `arcs` with no error, on the default, owner-like and owner-like + `[corner_deviation]` configs.
- **G2 pass on every single-pass program; one open host finding.** Each pass checked net steps, every step paired, and timing against a bound of `MAX_STEPCOMPRESS_ERROR` (25 µs) plus this run's measured MCU lateness. Event mode, all 11 runs (3 programs × 3 configs, plus the upstream rows):
  - The MCU executed every commanded step. The captured sequence equals the live queue_step tap on every run, including the dense ×15 rows.
  - Worst capture−reference error is 29.2 µs, against a bound of 33.8 µs.
  - Open finding: with `voron_top_layers` repeated 15×, the live host commands 40 X / 46 Y fewer steps than batch mode. Net steps stay identical. The divergence is deterministic (same step index in 8 runs, any host load), it starts in the second repetition (reference t = 3.70 s), and the tap shows the live schedule already differs. So the live planner differs from batch planning there; it is not an MCU loss. Not root-caused.
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

