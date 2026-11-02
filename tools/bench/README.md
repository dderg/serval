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
uv run python tools/bench/deploy.py reset-target   # SWD-reset the F401 (clears a latched MCU shutdown), restart klipper
uv run python tools/bench/lock.py hold -- <cmd...> # run a multi-step session under the bench lock
uv run python tools/bench/suite.py [--quick] [--expect-fail NAME]  # regression matrix: sync + 7 runs (3 with --quick) + analyze, table, ~/bench/suites/<ts>/summary.json
uv run python tools/bench/deploy.py flash-target   # build F401 fw (f401.config), flash via Pi SWD, restart klippy
uv run python tools/bench/deploy.py flash-capture  # build capture-fw, flash F407 via ST-Link
uv run python tools/bench/deploy.py config         # validate + push printer.cfg, FIRMWARE_RESTART
uv run python tools/bench/deploy.py mco            # enable F401 MCO1 (PA8, HSE/4) over SWD without resetting it
uv run python tools/bench/run.py <gcode> [--runs-dir ~/bench/runs] [--timeout 1800] [--events [--step-tap]] [--sim configs/full.sim.json --config configs/full.cfg]
uv run python tools/bench/analyze.py <run_dir> [--gap S]
uv run python tools/bench/capture.py --help        # standalone capture (--events for event mode)
uv run python tools/bench/buzz.py --motor a --steps 2000 --period-us 100  # event-mode calibration against busy-waited DIAG_STEPPER_BUZZ steps
uv run python tools/bench/cdc_throughput.py --seconds 60  # capture-board USB throughput probe (firmware mode "T"; Gate 0 of capture-fw/EVENT_MODE.md)
```

`run.py` sanitizes the gcode (allowlist; unknown commands raise), pushes config, restarts, uploads, captures, prints, fetches logs, and cancels the print if the capture or the print fails. Test gcodes: `tools/bench/gcode/` (`gen_speed_ladder.py` regenerates the speed/extrude ladders). `--config tools/bench/configs/<name>.cfg` runs with an alternate printer config. `--host-cpus N --host-max-mhz M` emulate a weaker host (klipper CPUAffinity drop-in + cpufreq `scaling_max_freq`), always restored afterwards, and are recorded in `run.json`. `--events` captures in event mode. `--step-tap` (with `--events`) sets `SERVAL_STEP_TAP_DIR` for klipper through a systemd drop-in (removed afterwards) and fetches `step_tap_mcu<N>_p<pid>_s<session>.bin`; `analyze.py` then adds an `mcu_execution` section with per-step MCU lateness against the commanded schedule (`capture-fw/EVENT_MODE.md`, "Host step tap").

## Run dir (`~/bench/runs/<YYYYmmdd-HHMMSS>/`)

`print.gcode`, `printer.cfg`, `capture.bin` (count mode: int8, N×4, one row per 25 µs tick; channels a b z e; + = DIR high) or `events.bin` (event mode), `capture.json` (`mode`, `tick_hz`), `klippy.log`, `events/` (structured logs), `run.json`, and after analysis `analysis.json`, `*.png`.

## Capture

Count mode: 40 kHz snapshots, i8 step delta per channel. Event mode: every STEP and DIR edge timestamped at 84 MHz (11.9 ns) by input capture with DMA, decoded by `capture_events.py`; the analyzer uses the exact times and bins them for the count checks. Design, proofs and gate results: `capture-fw/EVENT_MODE.md`. Wire protocol: docstring of `capture.py`. Latched faults are raised by the host reader; host stall timeout 1 s.

## Analyzer

Reference is offline `_motion_engine.pipeline_snapshot_axis_samples`; needs local `make -f Makefile.rust motion-engine-fast`. Extruder total is shorter than gcode E by design (`docs/rewrite/shaper.md`, "Followers").

## Recovery

- F401: hard reset via NRST from the Pi always works. `uhubctl` does NOT power-cycle it (Pi 5 port power stays on).
- F407: if its firmware ever disables the SWD pins, set BOOT0=1 and press reset (manual).
- Config validation forbids PA13/PA14 (SWD) and PA8 (reserved for clock output).

Any tool that touches the Pi or the boards (`run.py`, every `deploy.py` subcommand) takes an exclusive lock on `~/bench/.lock` and waits up to `--lock-timeout` seconds (default 1800), reporting the holder every 30 s, so several agents can share the bench safely. `lock.py hold -- <cmd>` keeps the lock across several steps (children inherit `BENCH_LOCK_TOKEN` and re-enter it).

`run.json` records `deployed` (the Pi's `.bench-deploy.json`: HEAD, branch, `git diff HEAD` sha256, dirty paths, worktree); `run.py` fails if it is missing. `suite.py` is the one-command regression check: it takes the lock once, syncs, runs each row (first_layers, speed_ladder, speed_ladder_fast, extrude_ladder, epochs, dense_top, dense_top_fast_throttled), records crashed rows and recovers with `reset-target`, and exits non-zero if any row fails; rows expected to fail must be named with `--expect-fail` (an expected-fail row that passes also fails the suite). Agent usage guide: `.claude/skills/bench/SKILL.md`.

## Limits

- Endstops, heaters and TMC2209s are simulated by the capture board once the phase-2 wiring exists (`configs/full.cfg`); `run.py` still drops `G28` and starts from `SET_KINEMATIC_POSITION`, so homing runs need their own gcode path.
- klippy takes the non-TMC single-edge 2 µs step path, not the TMC both-edge path real printers use.
- 25 µs time resolution in count mode; 11.9 ns in event mode. Against the ideal reference event mode still includes host step compression (p99 ~3–6 µs, tail to the 25 µs `stepcompress_max_error`); `--step-tap` removes it by comparing against the commanded schedule.
- Pi 5 is faster than the Pi 4 printers.

## Phase 2: simulated peripherals

Wiring: `WIRING_PHASE2.md` (endstops/probe, two thermistor DACs, heater/fan duty inputs, MCO clock reference, single-wire TMC UART). Design, protocol and latency bounds: `capture-fw/PHASE2.md`; pure logic in `sim-logic/` (`cargo test` there). The capture firmware drives the endstops low (not triggered) from boot, serves four TMC2209s on the UART and feeds both thermistors from a heater model at all times; walls and aux records need a capture.

`run.py <gcode> --config configs/full.cfg --sim configs/full.sim.json [--events]` sends the walls/heater config, captures with aux records (`aux.bin`), pokes MCO1 on after the printer is ready, and counts both STEP edges on the channels `full.sim.json` lists (must match the dedge klippy writes; the analyzer fails otherwise). `analysis.json` then has a `simulator` section and `mcu_execution` uses the MCO count as its clock reference. `deploy.py mco` re-enables MCO1 by hand (also run by `flash-target`/`reset-target`). Hardware validation steps (V0–V7, exact commands and pass criteria) are in `capture-fw/PHASE2.md`; they use `deploy.py config --config configs/full.cfg` and `sim_probe.py` (`query`, `gcode`, `endstops`, `capture` around a gcode sequence, `heat`).
