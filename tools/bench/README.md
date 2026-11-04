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
uv run python tools/bench/suite.py [--quick] [--expect-fail NAME]  # regression matrix: sync + 9 runs (3 with --quick) + analyze, table, ~/bench/suites/<ts>/summary.json
uv run python tools/bench/deploy.py flash-target   # build F401 fw (f401.config), flash via Pi SWD, restart klippy
uv run python tools/bench/deploy.py flash-capture  # build capture-fw, flash F407 via ST-Link
uv run python tools/bench/deploy.py config         # validate + push printer.cfg, FIRMWARE_RESTART
uv run python tools/bench/run.py <gcode> [--runs-dir ~/bench/runs] [--timeout 1800] [--events [--step-tap]]
uv run python tools/bench/analyze.py <run_dir> [--gap S] [--corner-flow] [--step-benchmark PULSED,BOTH_EDGE,CLOCK_HZ]
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

### Corner flow (`--corner-flow`)

`analyze.py --corner-flow` adds `corner_flow` to `analysis.json`: the nozzle flow through every extruding corner (a turn ≥ 20° between two extruding moves) and short-segment curve (≥ 3 vertices turning ≥ 1° between extruding segments ≤ 2 mm) of the print. It uses the definition `tools/corner_flow.py` uses, from the shared `tools/flow_metric.py`. A first-order melt model (`e_cmd = e_out + K·de_out/dt`) runs with K set to the config's pressure-advance k. Its output is measured per mm of the nozzle path in 0.02 mm bins. The commanded bead is subtracted and the difference is divided by the nominal bead, so 0 means the commanded bead exactly and +0.5 means 50 % over. Each feature reports `apex` (the deviation at the apex), `peak`/`dip`/`width_mm` (bins beyond ±5 %) and `blob_nl` within ±15 mm, or ±half the shorter leg, around the apex. It also reports the extruder's `ve_max`/`ve_min`/`ae_max` over that span while the nozzle moves, from 2 ms / 5 ms box-smoothed derivatives.

Every feature is measured from three sources on the same block-local time grid (10 µs):
- `offline`: the snapshot plan's continuous x, y and e.
- `reference`: the analyzer's reference steps.
- `capture`: the captured a/b/e steps, mapped to stream time by the epoch's alignment.

A step event is an exact point of the motion, because the motor steps where its position crosses a lattice line. So the step tracks are rebuilt by joining the events linearly, and x, y come from a, b through the kinematics' motor weights.

`max_abs` splits the gap between capture and offline into two parts:
- `reference_minus_offline` is step quantization. It is deterministic from the plan and is the bound the bench can resolve.
- `capture_minus_reference` is what the printer stepped differently from the plan, mostly the host's step compression (≤ 25 µs). At 0.02 mm bins that is visible near slow apexes and on fast curves.

Near an apex, where the nozzle nearly stops, flow per mm is ill-conditioned, so the quantization grows with how sharp the corner is. An exact synthetic capture reproduces the reference metrics to < 1e-4 (`test_corner_flow_bench.py`), and the offline source equals `corner_flow.py`'s `measure` for the same program.

`gcode/corner_flow.gcode` (`gen_corner_flow.py`) holds 90°/45°/135° interior corners at 50/100/200/300 mm/s, a 10 mm-radius arc of 0.5 mm segments at 100/200 mm/s and a 2 mm zigzag at 50/150 mm/s. Each case runs between rests, so each is an epoch of its own. The bead is 0.4 × 0.2 mm from 1.75 mm filament.

Main @ b0e093d46, event mode:
- `printer.cfg` (PA k 0.03 + smooth_triangle 0.04 on E), run `corner_flow/20261104-054727`. Peaks offline / captured:
  - 45° interior: +60/+63 % at 50 mm/s, +116/+125 % at 100, +176/+184 % at 200, +181/+196 % at 300.
  - 90°: +8/+9, +21/+22, +53/+57, +57/+58 %.
  - 135°: +0.2/+1.4, +1/+2, +9/+12, +20/+23 %.
  - Dips: −3 to −15 %.
  - Quantization bound: 10.2 points on peak, 1.4 on dip. Capture minus reference: ≤ 8.2 / 2.6 points.
- `configs/ramp.cfg` (smooth_triangle 2 ms + ratio_ramp_time 15 ms), run `corner_flow/20261104-054856`.
  - Offline peaks ≤ 1.4 % at every corner.
  - Captured peaks reach 27 % at a 45° interior corner at 300 mm/s, but the reference steps show 28 % there too. The quantization bound is 26.4 points on peak and 2.3 on dip, and the capture stays within 7.7 points of the reference.

## Recovery

- F401: hard reset via NRST from the Pi always works. `uhubctl` does NOT power-cycle it (Pi 5 port power stays on).
- NRST (Pi GPIO23) is an input with the Pi's pull-up whenever openocd does not hold it: `gpio=23=ip,pu` in the Pi's `/boot/firmware/config.txt` covers boot, and `-pull-up` on the srst line in `openocd-target.cfg` restores the bias when openocd releases the line (without it the release leaves the pin unbiased). Check with `ssh bench-pi pinctrl get 23` → `ip pu | hi`.
- A pin-only reset (`runtime.mcu_reset` cause bits `0x04000000`, PINRSTF without SFTRSTF/IWDGRSTF) comes from openocd srst (or the board's reset button): `deploy.py reset-target`/`flash-target`, which `suite.py` also runs on its own after any failed row (the row then carries `recovered_with_reset_target`). `FIRMWARE_RESTART` shows `0x14000000` (PINRSTF + SFTRSTF).
- F407: if its firmware ever disables the SWD pins, set BOOT0=1 and press reset (manual).
- Config validation forbids PA13/PA14 (SWD) and PA8 (reserved for clock output).

Any tool that touches the Pi or the boards (`run.py`, every `deploy.py` subcommand) takes an exclusive lock on `~/bench/.lock` and waits up to `--lock-timeout` seconds (default 1800), reporting the holder every 30 s, so several agents can share the bench safely. `lock.py hold -- <cmd>` keeps the lock across several steps (children inherit `BENCH_LOCK_TOKEN` and re-enter it).

`run.json` records `deployed` (the Pi's `.bench-deploy.json`: HEAD, branch, `git diff HEAD` sha256, dirty paths, worktree, rust build id); `run.py` fails if it is missing, and `analyze.py` refuses a run whose deployed rust build id is not the one its local engine was built from — a reference planned by other code holds the capture against motion the printer never commanded. The refusal prints the recipe to reanalyze from a worktree at the deployed head (`scripts/wt.sh new reanalyze-<head> --base <head>`, `make -f Makefile.rust motion-engine-fast`, the same `analyze.py` command) and warns when the deployed tree was dirty, since the head alone then may not reproduce it. `suite.py` is the one-command regression check: it builds the local engine first (`make -f Makefile.rust motion-engine-fast` in its own tree; a failed build aborts the suite with make's output instead of analyzing with a stale engine), takes the lock once, syncs, runs each row (first_layers, speed_ladder, speed_ladder_fast, extrude_ladder, epochs, rest_ladder, dense_top, dense_top_fast_throttled, corner_flow; rest_ladder runs `configs/rest_ladder.cfg`, fast Z limits with pressure advance + smooth_triangle on E, whose slow root searches meet the reanchor cuts its G4 rests trigger mid-print — the case a budget-truncated cut drain once turned into LatticeDrift; corner_flow captures in event mode and analyzes with `--corner-flow`, its summary row carrying `corner_flow_max_abs`), records crashed rows and recovers with `reset-target`, and exits non-zero if any row fails; rows expected to fail must be named with `--expect-fail` (an expected-fail row that passes also fails the suite). Agent usage guide: `.claude/skills/bench/SKILL.md`.

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

**CI throughput gate.** Wall-clock factors on a shared runner are noise, so `ci.sh throughput-gate` (`scripts/throughput_gate.py`) counts instructions instead: the first 1000 lines of the dense print replay through `gcode_replay_bench --synchronous` under `taskset -c 0` (no shaper thread, no pool helpers: one thread, a deterministic count) in valgrind cachegrind, and only instructions whose source file lies under `rust/` count — libc, ld.so, std and registry crates (crossbeam's spin loops, glibc's per-CPU memcpy) are excluded. The three slowest 2-core configs above are gated against `tools/bench/throughput_baseline.json`: the gate fails when a config's count moves more than 2 % either way (repeat runs differ by ~1 ppm; growth is a regression, shrinkage must be accepted so the baseline ratchets down), or when the predicted Pi 2-core worst window K × stream_secs / Ir falls below 2.0×. To accept a deliberate change, run `./scripts/ci.sh throughput-gate --accept --reason "…"` on a committed tree (valgrind, or else the `scripts/Dockerfile-build` image with the checkout at the CI runner's path), commit the baseline file, and put the reason in a ```` ```throughput-accept ```` block in the PR body; CI and `scripts/ship.py` refuse a moved baseline without it. K is calibrated per config from two 2-core runs of the full print on the Pi at the commit the counts come from (the lower of the two), and the baseline file records them: `./scripts/ci.sh throughput-gate --calibrate-pi-worst p3_voron_direct_drive_ramp=X1,X2 …` (`~/.cache/tp/pi_ab.sh "label=<worktree>" 0,1 <cfgs>` under `tools/bench/lock.py hold` measures them). Recalibrate whenever an accept changes what the Pi would measure by more than the gate's tolerance.

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

### Self-benchmark

Every MCU measures its own step cost. On the first `get_step_benchmark` (`src/stepper_bench.c`), it runs one benchmark stepper, 32 steps at 25 µs intervals. The steps go through the real timer interrupt (SysTick, or TIM2 on G0) and the real classic dispatch path, with pulses going to a RAM sink instead of a pin (`gpio_out_bench_sink`). Between steps the MCU executes a nop sled larger than its instruction cache: 2 KB, or 32 KB on H7 for its 16 KB L1. That evicts the cache the way the main loop's tasks do between a lone motor's steps.

The timer reports how many ticks its interrupt held the CPU (`timer_dispatch_busy_ticks`), counted from the moment it was due to fire to the handler's exit. That count includes entry latency, the dispatch prologue and the in-handler spin to the unstep. The benchmark reports the worst of three runs per step, once for the 2 µs pulsed mode and once for the both-edge mode.

Klippy queries the benchmark at connect and refuses firmware without the command. The result is in `printer.mcu.step_benchmark`. `CONFIG_MCU_SIM` builds report `simulated=1` and do not run it.

F401: 497 ticks/step pulsed (169 k steps/s) and 240 ticks/step both-edge. The values are identical across reflashes. The single-motor ceiling is about 172 k: a lone motor sustained at 169 k ran without a shutdown, and one at 174 k shut down. `suite.py` fails unless the pulsed capacity lies within `BENCHMARK_BAND` (0.85–1.10) of `SINGLE_MOTOR_CEILING_STEPS_PER_S` (172 k); the F401 sits at 0.98.

The earlier benchmark timed `sched_timer_dispatch` in a cache-hot loop at 301 ticks/step. That counted none of the interrupt's cost, and the cap built on it shut the MCU down on a lone motor (see "Single motor at the cap").

CI runs the benchmark of every test/configs firmware and of `f401.config` in unicorn (`step_benchmark_emu.py`, from `scripts/ci-build-mcu.sh`). The emulator counts one tick per instruction and delivers the timer interrupt the firmware programs, by calling its handler once the interrupt is due and PRIMASK is clear, after the core's exception entry (12 cycles, 15 on the G0's M0+). It does not model wait states. The build fails on a shutdown or on a result outside 40–600 instructions/step. Instructions per isolated step, pulsed / both-edge:

| firmware | pulsed | both-edge |
|---|---|---|
| F401 | 380 | 168 |
| F103 | 371 | 176 |
| G0B1 | 309 | 139 |
| H723 | 343 | 182 |

The pulsed figure includes the spin to the 2 µs unstep inside the interrupt. On H7 the unstep is far enough away that it gets its own interrupt.

The same run checks that a step edge comes as late after its waketime whether the timer interrupt dispatched it or the dispatch loop did. A second benchmark run sets a bare timer 40–400 instructions before each step, so steps also follow another dispatch inside the interrupt. The check takes an irq-path edge when the interrupt was taken on time and dispatched that timer first, and a loop-path edge when the loop was spinning on the timer up to the waketime. It fails on any early edge, or when one dispatch function's edges spread over more than 8 instructions across the two paths. Edge lateness in instructions, irq / loop path, for pulsed and both-edge:

| firmware | pulsed | both-edge |
|---|---|---|
| F401 | 25 / 25–27 | 23 / 23–25 |
| F103 | 34 / 36 | 32 / 34 |
| G0B1 | 36 / 35–36 | 33 / 32–33 |
| H723 | 27 / 27–29 | 27 / 27–29 |

H723 has no both-edge optimization, so both modes dispatch through the same function and are judged together. Both timer backends wake early and spin to the head timer's waketime: SysTick 100 ticks ahead, the G0's TIM2 64 ticks ahead (`TIMER_IRQ_LEAD_TICKS`). The dispatch loop ends its interrupts-enabled wait at the same lead, so both paths leave the same spin. Before that the G0 irq path was 61 instructions late against 43–48 in the loop, and SysTick's loop path came 12 instructions after its irq path. The G0 lead is the emulated 27 cycles from interrupt to the loop's first timer read, scaled by the F401's hardware-to-emulator ratio (44 emulated cycles against a measured p99.9 of 68–100). It is not measured on G0 hardware.

### How the capacity caps moves

The planner turns the benchmark into a per-move speed cap (`rust/motion-pipeline/src/live_config/step_capacity.rs`, `docs/Config_Reference_Motion.md`, "MCU step capacity"); klippy (`klippy/motion_step_capacity.py`) only hands it each MCU's benchmark, clock and stepped motors. Each MCU gets `STEP_CAPACITY_FRACTION` × CLOCK_FREQ ticks per second; the fraction comes from the timing budget below. A motor costs its benchmarked ticks per step (pulsed or both-edge, by its own mode) times the steps per second the move asks of it. The planner keeps the sum over each MCU's motors within the capacity, counting pressure advance on the extruder. On this bench (F401, 497 ticks/step pulsed) the fraction of 0.80 gives 135 k steps/s. `capacity.cfg` raises the speed limits past the cap, so its ladder rungs above the cap are planned slower than the G-code asks and `step_capacity_capped` events report it.

`run.py` records every MCU's step benchmark in `run.json` under `step_benchmarks`, taken from the MCU status `step_capacity`, together with the capacity fraction the live planner applied. `analyze.py` plans its reference under the same cap, so a run where the cap binds is analyzed against the plan the printer executed, even after the fraction changes. A `run.json` from before that recording plans an uncapped reference. For such a run, `--step-benchmark 301,160,84000000,0.75` supplies the F401's benchmark and the fraction live used then (0.75 until it became 0.625). It is charged to every captured motor, stepped pulsed, since the bench has one MCU and no `dedge` driver. `analysis.json` says which reference it judged in `reference_step_capacity`: `recorded`, `supplied` or `absent`.

### Capacity fraction from a timing budget

Budget: at the load the cap allows, p99.9 MCU step lateness × that motor's own step rate (lateness in microsteps of position) must stay ≤ 0.5 microstep. A step that is 0.5 microstep late is still nearer its own lattice point than the next one. The host's step compression already allows 25 µs by design, which is 1.3 microsteps at the 52 k steps/s per motor these runs reach. The analyzer's 0.05-step criterion is not usable here: the MCU's intrinsic ~5 µs p99.9 dispatch jitter alone exceeds it at any useful speed, even at a quarter of capacity. That criterion measures speed, not load.

The fraction must hold the budget for two workloads at the cap, each run in event mode with the step tap and `configs/capacity.cfg` so that every move asks more than the cap:
- `single_motor_cap`: one motor alone, a CoreXY 45° diagonal stepping only `a`. Each step pays its own timer interrupt.
- `capacity_ladder_high`: four motors at equal rates. Their steps batch inside the dispatch loop.

The table gives `mcu_execution.lateness_by_step_rate_hz` → `position_p99_9_steps` in the dominant (capped) bin. Bench F401, isolated-step benchmark 497 ticks/step, firmware with #670, #671 and #683. The fraction was set per run in a throwaway tree.

| fraction | cap | single motor | four motors |
|---|---|---|---|
| 1.0 | 169 k | 0.89 (`20261104-125307`) | 0.24 (`-125357`) |
| 0.9 | 152 k | 0.69 (`-125811`) | 0.21 (`-125859`) |
| 0.85 | 144 k | 0.09, 0.09, 0.10 (`-131827`, `-131918`, suite `20261104-143747`) | 0.20 (`-143608`) |
| **0.8** | 135 k | **0.05, 0.05, 0.05** (`-130322`, `-132317`, `-132439`) | **0.19** (`-130412`) |

The single motor sets the fraction. 0.85 is the largest fraction measured within budget, and 0.9 misses it. The single-motor lateness rises steeply between the two, because at 0.9 a lone motor has the CPU nearly saturated. No fraction shut the MCU down. Four motors stay far inside the budget even at 1.0, because their steps are cheaper than the benchmark charges.

The live fraction is **0.80**, one step back from that cliff. A shutdown is catastrophic, and real printers run more MCU tasks than this bench, which has no heaters, no ADC sampling and no TMC UART yet. Re-derive the fraction under that load once the phase-2 wiring (`WIRING_PHASE2.md`) exists.

Earlier derivations used the four-motor ladder only, against the cache-hot benchmark (301 ticks/step):
- First: 0.625, the largest fraction under 0.5 microstep. At 0.65 the result was 0.52 on both runs; at 0.75, 0.62. Runs `20261104-044956` … `-051928`.
- With #670/#671/#683: 0.775 (0.44 and 0.46). At 0.80 the result was 0.55–0.59.

That benchmark under-charged a lone motor. `single_motor_cap` shut the MCU down with `Stepper too far in past` at 0.625 on every firmware, and the old firmware (`481397432`) shut down even at 0.50 (`-114535`). Main ran at 0.50 as an interim fix (#684) until this benchmark.

### Step timing jitter sources (F401)

These were measured with an on-target probe: DWT-stamped histograms read over SWD from a throwaway firmware. Fixes were checked in event mode.

| source | measured | fix |
|---|---|---|
| SysTick path vs dispatch-loop path | SysTick entry to step edge took 176–200 cycles (2.1–2.4 µs). A loop-dispatched step's edge came 72 cycles after due. Low-load lateness was a flat 1.2–3.0 µs band, p50 2.2 µs. | #671: wake SysTick early and spin to the deadline. Low-load p50 is now 0.1–0.3 µs. |
| Flash wait states | 5 WS at 84 MHz. Entry-to-edge spread was 0.67 µs from cache misses. | #670: 2 WS. Capped-bin p99.9 position fell 18 %. |
| Forced task defer | 14,570 forced 5 µs pauses per capped ladder, each delaying the next steps 4.4–12 µs. | #671: 1 ms interval instead of 100 µs. Capped p99.9 fell 27 %. |
| Over-long SysTick lead | The clean prologue p99.9 is 100 cycles. A 190-cycle lead spun about 120 cycles per interrupt. | #683: lead 100. |
| Running the hot path from SRAM | The self-benchmark rose from 301 to 391 ticks/step: S-bus instruction fetch contends with data. | Rejected. |
| USB OTG interrupt (priority 1) | Overlapped 0.02 % of SysTick entries, delaying them 1.6–2.1 µs. | None; below p99.9. |
| Masked critical sections in the main loop | Entries while awake were 7 % of the total; p99.9 entry delay grew by at most 0.3 µs. | None. |
| SysTick entry from WFI | A constant 32 cycles. | None; the host's clock offset absorbs it. |

What remains at the cap is queueing behind other events: about half the isolated steps wait, uniformly up to roughly 6 µs. That is load, and the capacity fraction trades it off.

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
