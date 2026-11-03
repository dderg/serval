# Hardware bench

Pi 5 runs klippy against an STM32F401 printer MCU. An STM32F407 capture board on this box records the F401's STEP/DIR lines. Print a gcode, capture, analyze, unattended.

## Parts

| | Host | Link | Flashed by |
|---|---|---|---|
| Pi 5 | `ssh bench-pi` (pi@192.168.1.149, key `~/.ssh/bench_pi_ed25519`), Raspberry Pi OS Lite trixie, service `klipper`, Moonraker :7125 | | |
| F401 (STM32F401RCT6, black-pill, 25 MHz) | Pi USB | no bootloader, flash at 0x08000000 | Pi GPIO SWD via openocd |
| F407 (DevEBox STM32F4XX_M, F407VGT6, 8 MHz HSE) | this box USB | | ST-Link V2 clone on this box |

## Wiring

Pi → F401 (SWD):

| Pi pin | GPIO | F401 |
|---|---|---|
| 18 | GPIO24 | SWDIO |
| 22 | GPIO25 | SWCLK |
| 16 | GPIO23 | R (NRST) |
| 20 | GND | GND |

ST-Link → F407: SWDIO→DIO, SWCLK→CLK, GND→GND. No NRST (header has none); reset is SYSRESETREQ.

F401 → F407 (capture):

| Channel | STEP | DIR |
|---|---|---|
| 0 a | PB12→PE9 (TIM1 CH1) | PB13→PE11 (CH2) |
| 1 b | PB14→PA6 (TIM3) | PB15→PA7 |
| 2 z | PB6→PD12 (TIM4) | PB7→PD13 |
| 3 e | PB8→PC6 (TIM8) | PB9→PC7 |

Plus 2× GND.

## Commands

Run from the repo root.

```
uv run python tools/bench/deploy.py sync [--tree DIR]  # rsync a worktree (default: this one) to Pi ~/klipper, delete stale, build-native.sh if rust/ changed, restart klipper, write ~/klipper/.bench-deploy.json
uv run python tools/bench/deploy.py reset-target   # SWD-reset the F401 (for an MCU that no longer answers; a latched shutdown is already cleared by the FIRMWARE_RESTART that restart_klipper/prepare_printer send when klippy settles in shutdown), restart klipper
uv run python tools/bench/lock.py hold -- <cmd...> # run a multi-step session under the bench lock
uv run python tools/bench/suite.py [--quick] [--expect-fail NAME]  # regression matrix: sync + 8 runs (3 with --quick) + analyze, table, ~/bench/suites/<ts>/summary.json
uv run python tools/bench/deploy.py flash-target   # build F401 fw (f401.config), flash via Pi SWD, restart klippy
uv run python tools/bench/deploy.py flash-capture  # build capture-fw, flash F407 via ST-Link
uv run python tools/bench/deploy.py config         # validate + push printer.cfg, FIRMWARE_RESTART
uv run python tools/bench/run.py <gcode> [--runs-dir ~/bench/runs] [--timeout 1800] [--events [--step-tap]]
uv run python tools/bench/analyze.py <run_dir> [--gap S]
uv run python tools/bench/capture.py --help        # standalone capture (--events for event mode)
uv run python tools/bench/buzz.py --motor a --steps 2000 --period-us 100  # event-mode calibration against busy-waited DIAG_STEPPER_BUZZ steps
uv run python tools/bench/cdc_throughput.py --seconds 60  # capture-board USB throughput probe (firmware mode "T"; Gate 0 of capture-fw/EVENT_MODE.md)
```

`run.py` sanitizes the gcode (allowlist; unknown commands raise), pushes config, restarts, uploads, captures, prints, fetches logs, and cancels the print if the capture or the print fails. Test gcodes: `tools/bench/gcode/` (`gen_speed_ladder.py` regenerates the speed/extrude ladders). `--config tools/bench/configs/<name>.cfg` runs with an alternate printer config. `--host-cpus N --host-max-mhz M` emulate a weaker host (klipper CPUAffinity drop-in + cpufreq `scaling_max_freq`), always restored afterwards, and are recorded in `run.json`. `--events` captures in event mode. `--step-tap` (with `--events`) sets `SERVAL_STEP_TAP_DIR` for klipper through a systemd drop-in (removed afterwards) and fetches `step_tap_mcu<N>_p<pid>_s<session>.bin`; `analyze.py` then adds an `mcu_execution` section with per-step MCU lateness against the commanded schedule (`capture-fw/EVENT_MODE.md`, "Host step tap").

## Helpers

Offline (no lock, read run dirs under `~/bench/runs`):

```
uv run python tools/bench/summarize.py '~/bench/runs/20261101-22*' [--json]   # one line per run: verdict, stalls, abs_max/p99 us per motor, epochs (span delta, ppm), mcu lateness p99, deployed head
uv run python tools/bench/events.py <run_dir> [--around T --window S] [--names a,b] [--tail N]   # event counts by source/level/name in the run window, error/fatal records verbatim, klippy Stats print_time/buffer_time/sysload trend
```

Hardware (take the lock; re-entrant inside `lock.py hold`):

```
uv run python tools/bench/gcode.py "G1 X10" "M400"      # lines sent in order through Moonraker; reply printed, any error exits non-zero (--dry-run prints the requests)
uv run python tools/bench/state.py [--json]              # printer state, deployed identity, klipper pid, MCU serial, capture board (--dry-run lists the queries; read-only, no lock)
uv run python tools/bench/ab.py --ref origin/main --ref HEAD --gcode G [--config C] [--runs 3] [--host-cpus N --host-max-mhz M] [--dry-run]
```

`ab.py` builds a clean detached worktree per ref (committed content only), `sync --tree`s it, runs and analyzes the gcode N times, prints per-run lines and a per-ref table (ok count, errors, stalls, worst abs_max per motor), and always removes the worktrees. One lock hold covers all refs; the Pi is left on the last ref.

Repo dev tooling: `scripts/land-pr.sh <branch> [--title T --body B] [--dry-run]` pushes, opens the PR if missing, waits for the checks (prints the failed-log tail and exits non-zero on red), merges with a merge commit, then `git branch -f main origin/main` and removes the branch's worktree.

Planner-side tools (offline, repo `scripts/`) for changes that alter the reference the analyzer judges against, since the analyzer plans with the same engine:

```
uv run python scripts/review_baselines.py --pr N          # physical review of the snapshot cases a PR moves (REGRESSION / SUSPICIOUS / EXPECTED)
uv run python scripts/accept_reviewed.py N                # accept + commit baselines after review (refuses on REGRESSION / unexplained SUSPICIOUS)
uv run python scripts/bench_compare.py A B [--runs 5]     # interleaved replay throughput A/B (use the CPU row on a busy box)
scripts/corpus.sh [--configs fast,printer] [-k S]         # plan the 59-case corpus (51 snapshots + 4 sim prints x 2 configs)
python3 scripts/integrate_stack.py 443 444 [--run CMD]    # try a PR stack on a throwaway worktree before syncing it to the Pi
```

`land-pr.sh` can be run from the branch's own worktree and re-updates the PR branch when main moves. Sync the committed, integrated tree to the Pi (`deploy.py sync --tree`), not a worktree with other agents' edits.

## Run dir (`~/bench/runs/<YYYYmmdd-HHMMSS>/`)

`print.gcode`, `printer.cfg`, `capture.bin` (count mode: int8, N×4, one row per 25 µs tick; channels a b z e; + = DIR high) or `events.bin` (event mode), `capture.json` (`mode`, `tick_hz`), `klippy.log`, `events/` (structured logs), `run.json`, and after analysis `analysis.json`, `*.png`.

## Capture

Count mode: 40 kHz snapshots, i8 step delta per channel. Event mode: every STEP and DIR edge timestamped at 84 MHz (11.9 ns) by input capture with DMA, decoded by `capture_events.py`; the analyzer uses the exact times and bins them for the count checks. Design, proofs and gate results: `capture-fw/EVENT_MODE.md`. Wire protocol: docstring of `capture.py`. Latched faults are raised by the host reader; host stall timeout 1 s.

## Analyzer

Reference is offline `_motion_engine.pipeline_snapshot_axis_samples`; needs local `make -f Makefile.rust motion-engine-fast`. Extruder total is shorter than gcode E by design (`docs/rewrite/shaper.md`, "Followers").

Timing check `timing_within_50us_or_0.05_step` (constants `TIMING_TOLERANCE_US`, `POSITION_EQUIVALENT_TOLERANCE_STEPS`). A step with reference rate ≥ 200 Hz fails only when it is both more than 50 µs off the aligned reference and more than 0.05 step off in position. Position error is |time error| × reference step rate. Near zero velocity, a tiny position difference becomes a large time difference.

Run `20261103-231302` (capacity ladder, event mode) failed the old time-only check on 1 of 1.62 M E steps. That step sat at a turnaround: reference rate 227 Hz, against 2.9 kHz on either side. It was −57.3 µs off, which is 0.013 step (0.03 µm of filament). The MCU ran the commanded schedule faithfully there: commanded and captured intervals agree within ~1 µs. So the gap is between the host's schedule and the offline reference, not a firmware fault. The run's next-worst steps were −29.2 and −27.7 µs below 2 kHz, and above 2 kHz its extremes were −25.6/+22.7 µs.

The bound 0.05 step / 50 µs puts the crossover at 1 kHz. From 1 kHz up, any step more than 50 µs off is also more than 0.05 step off, so high-rate steps are judged by the 50 µs bound exactly as before. Below 1 kHz the time allowance grows to 0.05 step / rate, which is 250 µs at the 200 Hz floor. The turnaround step above sits 4× inside that. A step misplaced by more than a twentieth of a step still fails at every rate. Lost or extra steps fail `all_steps_matched` and `net_steps_match`.

## Recovery

- F401: hard reset via NRST from the Pi always works. `uhubctl` does NOT power-cycle it (Pi 5 port power stays on).
- NRST (Pi GPIO23) is an input with the Pi's pull-up whenever openocd does not hold it: `gpio=23=ip,pu` in the Pi's `/boot/firmware/config.txt` covers boot, and `-pull-up` on the srst line in `openocd-target.cfg` restores the bias when openocd releases the line (without it the release leaves the pin unbiased). Check with `ssh bench-pi pinctrl get 23` → `ip pu | hi`.
- A pin-only reset (`runtime.mcu_reset` cause bits `0x04000000`, PINRSTF without SFTRSTF/IWDGRSTF) comes from openocd srst (or the board's reset button): `deploy.py reset-target`/`flash-target`, which `suite.py` also runs on its own after any failed row (the row then carries `recovered_with_reset_target`). `FIRMWARE_RESTART` shows `0x14000000` (PINRSTF + SFTRSTF).
- F407: if its firmware ever disables the SWD pins, set BOOT0=1 and press reset (manual).
- Config validation forbids PA13/PA14 (SWD) and PA8 (reserved for clock output).

Any tool that touches the Pi or the boards (`run.py`, every `deploy.py` subcommand) takes an exclusive lock on `~/bench/.lock` and waits up to `--lock-timeout` seconds (default 1800), reporting the holder every 30 s, so several agents can share the bench safely. `lock.py hold -- <cmd>` keeps the lock across several steps (children inherit `BENCH_LOCK_TOKEN` and re-enter it).

`run.json` records `deployed` (the Pi's `.bench-deploy.json`: HEAD, branch, `git diff HEAD` sha256, dirty paths, worktree); `run.py` fails if it is missing. `suite.py` is the one-command regression check: it takes the lock once, syncs, runs each row (first_layers, speed_ladder, speed_ladder_fast, extrude_ladder, epochs, rest_ladder, dense_top, dense_top_fast_throttled; rest_ladder runs `configs/rest_ladder.cfg`, fast Z limits whose slow root searches meet the reanchor cuts its G4 rests trigger mid-print, the case a budget-truncated cut drain once turned into LatticeDrift), records crashed rows and recovers with `reset-target`, and exits non-zero if any row fails; rows expected to fail must be named with `--expect-fail` (an expected-fail row that passes also fails the suite). Agent usage guide: `.claude/skills/bench/SKILL.md`.

## Replay throughput (Pi 5)

`gcode_replay_bench` on the bench Pi 5 (2.4 GHz, `ondemand` pinned at max, `throttled=0x0`, 54–60 °C, klippy idle), full `voron_dense_top_layers.gcode`, worst 5 s stream window realtime factor (`ci.sh replay-budget`'s metric), cores limited with `taskset`. The target is ≥ 2× at 2 cores, which leaves the other two to klippy and the OS. Main `80ad5b4d8`, 2026-11-04; repeat runs agree within 2 %.

| config | 4 cores | 2 cores |
|---|---:|---:|
| `printer.cfg` | 15.8 | 15.1 |
| `configs/capacity.cfg` | 8.90 | 7.68 |
| `configs/fast.cfg` | 8.43 | 7.27 |
| `configs/p1_bowden_bedslinger_kernel_cd0.04.cfg` | 44.3 | 42.3 |
| `configs/p2_i3_direct_drive_ramp_cd0.01.cfg` | 6.96 | 5.92 |
| `configs/p3_voron_direct_drive_ramp_cd0.01.cfg` | 4.09 | 3.35 |
| `configs/p4_high_accel_kernel_cd0.01.cfg` | 5.72 | 5.21 |
| `configs/p4_high_accel_ramp_cd0.01.cfg` | 6.30 | 5.30 |
| `configs/ramp.cfg` | 13.8 | 11.5 |
| `configs/trident_kernel.cfg` | 2.89 | 2.29 |
| `configs/trident_ramp.cfg` | 6.96 | 5.01 |
| `configs/profiles/p1_bowden_bedslinger_kernel.cfg` | 11.8 | 10.4 |
| `configs/profiles/p1_bowden_bedslinger_ramp.cfg` | 18.7 | 15.8 |
| `configs/profiles/p2_i3_direct_drive_kernel.cfg` | 4.87 | 4.00 |
| `configs/profiles/p2_i3_direct_drive_ramp.cfg` | 4.83 | 3.44 |
| `configs/profiles/p3_voron_direct_drive_kernel.cfg` | 2.82 | 2.16 |
| `configs/profiles/p3_voron_direct_drive_ramp.cfg` | 3.18 | 2.12 |
| `configs/profiles/p4_high_accel_kernel.cfg` | 4.38 | 3.71 |
| `configs/profiles/p4_high_accel_ramp.cfg` | 7.29 | 5.25 |

At 2 cores the `kalico-shaper` thread is busy ≈ 100 % of the wall time and the process uses ≈ 1.8 cores: the planning thread fits each lowered segment's sources while the shaper shapes the ones before it. The cost is in the shaper's leader fits and follower projection (about half of all CPU, `ExactConvolution::pieces_over` the largest single item) and in the planning thread's source fits (about 40 %, analytic span evaluation and the ladder proofs). `gcode_replay_bench --digest` hashes the emitted trajectory, so a throughput change can be held to an identical output.

## MCU step-rate capacity (F401, classic host-stepped path)

`gcode/capacity_ladder.gcode` (generator `gen_capacity_ladder.py`) steps all four motors at once, at the same rate each, in rungs of 31–283 kHz aggregate; `capacity_ladder_high.gcode` continues from 283 kHz in 31 kHz rungs, in count mode only because event mode's USB budget ends near 300 k steps/s. Both run with `configs/capacity.cfg` (printer.cfg with raised limits and no pressure advance or smoothing on E). The rungs follow each other without rests, and E has no PA chain, because both runs at first halted on the host with `LatticeDrift` (a reanchor at a rest whose clock-sync drain ran out of budget; owned and fixed separately). PA was dropped before that cause was known and is not the trigger, but the published numbers are without it. `analysis.json` → `mcu_execution.lateness_by_step_rate_hz` bins every step's MCU lateness by the aggregate commanded step rate within ±0.5 ms.

Runs `20261103-231302` and `-232815` (F401 firmware from main 439bcff00; `src/` unchanged through the host tree's 98a751318; single-edge 2 µs pulse = two timer events per step), lateness relative to the isolated-step floor:

| aggregate steps/s | p99 lateness | p99.9 | max | steps > 5 µs late |
|---|---|---|---|---|
| 25–100 k | 2.9–4.4 µs | 4.6–5.2 µs | 13.8 µs | 0.03–0.3 % |
| 100–200 k | 4.1–4.8 µs | 6.3–9.9 µs | 15.6 µs | 0.1–0.6 % |
| 200–225 k | 4.7–5.1 µs | 9.7–10.2 µs | 15.8 µs | 0.9–1.0 % |
| 225–250 k | 7.2–7.7 µs | 10.8–11.3 µs | 16.0 µs | 2.2–2.7 % |
| 250–275 k | 7.7–8.4 µs | 10.4–11.0 µs | 21.5 µs | 4–5 % |
| 275–300 k | 9.9–10.7 µs | 15.8–16.6 µs | 30.3 µs | 10–12 % |

The ceiling: both `capacity_ladder_high` runs (`-232624`, `-232717`) held the 283 kHz rung and shut down with `Stepper too far in past` within 20 ms of the 314 kHz rung starting (last 10 ms window ≈ 300 kHz). That is about 600 k timer events/s, 140 F401 cycles per event, so the hard ceiling is ≈ 300 k steps/s and lateness grows past ≈ 225 k (75 %). The TMC both-edge path takes one event per step and is not measured here (no TMC on the bench).

Other families, scaled from the F401 by clock and by the instructions one more step event adds to a dispatch (unicorn run of each CI build's real timer IRQ). The effective cycles per instruction are assumptions, not measurements:

| build | clock | instructions / event | assumed cycles / instruction | estimated ceiling, single-edge |
|---|---|---|---|---|
| F401 (measured) | 84 MHz | 91 | 1.54 (fitted) | 300 k steps/s |
| F446 | 180 MHz | 91 | 1.54 | ≈ 640 k |
| F103 | 72 MHz | 96 | ≈ 1.7 (flash wait states, no ART) | ≈ 220 k |
| G0B1 | 64 MHz | 109–123 | ≈ 1.6 (M0+) | ≈ 160–180 k |
| H723 | 520 MHz | 81 | ≈ 2.5 (GPIO and DWT behind slower buses) | ≈ 1.3 M |

## Limits

- No endstops, heaters or TMC wired: no homing (`run.py` uses `SET_KINEMATIC_POSITION`).
- klippy takes the non-TMC single-edge 2 µs step path, not the TMC both-edge path real printers use.
- 25 µs time resolution in count mode; 11.9 ns in event mode. Against the ideal reference event mode still includes host step compression (p99 ~3–6 µs, tail to the 25 µs `stepcompress_max_error`); `--step-tap` removes it by comparing against the commanded schedule.
- Pi 5 is faster than the Pi 4 printers.

## Planned wiring (not done)

- Endstops: F407 PD8–PD11 → F401 via 1 kΩ.
- Thermistor DAC: F407 PA4/PA5 → F401 ADC via 1 kΩ.
- Clock: F401 PA8 → F407 PA15.
- TMC UART emulation: F407 PA2 ↔ F401 via 1 kΩ (F401 pin to be chosen from the black-pill header).
