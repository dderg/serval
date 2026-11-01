---
name: bench
description: Use when asked to test host or firmware changes on real hardware — step timing against the planner's reference, MCU behavior under real prints, producer/underrun crashes, slow-host emulation — using the Pi 5 + F401 + F407 capture bench in tools/bench/. Not for simulator runs (use sim) or log queries (use query-logs).
---

# Hardware bench (tools/bench)

A Pi 5 runs klippy against an STM32F401 printer MCU; an F407 board on this box records its STEP/DIR edges at 40 kHz; `analyze.py` diffs the capture against the offline planner's reference. Wiring, parts, capture protocol: `tools/bench/README.md`.

## Rules

- **One bench, many agents.** Every hardware step takes `~/bench/.lock` (exclusive flock; waits `--lock-timeout` s, default 1800, then fails loudly). For a multi-step session wrap the whole thing so nobody syncs other code between your sync and your run:

  ```bash
  uv run python tools/bench/lock.py hold -- bash my_session.sh
  ```

  Children inherit `BENCH_LOCK_TOKEN` and re-enter the lock; anything else waits. Never kill another holder.
- **Always `sync` first.** The Pi runs whatever was last synced; `run.json` → `deployed` (HEAD, branch, `git diff HEAD` sha256, dirty paths, worktree) says which code a run tested. Another agent's uncommitted edits in your worktree ship with `sync`. To test a clean ref: `git worktree add /tmp/x <ref>` then `deploy.py sync --tree /tmp/x`.
- Offline-only work needs no lock. Fail loudly: don't patch production code from the bench to make a run pass.

## Loop

```bash
uv run python tools/bench/deploy.py sync            # rsync, build-native.sh if rust/ changed, restart klipper, write deploy record
uv run python tools/bench/deploy.py config          # only if you edited printer.cfg
uv run python tools/bench/run.py tools/bench/gcode/speed_ladder.gcode   # prints run dir
uv run python tools/bench/analyze.py ~/bench/runs/<dir> [--gap S]       # writes analysis.json (+png)
```

`run.py` flags: `--config tools/bench/configs/fast.cfg` (alternate printer config, pushed and recorded), `--host-cpus N --host-max-mhz M` (weaker-host emulation: klipper CPUAffinity + cpufreq cap, always restored; Pi min is 1500 MHz; recorded in `run.json`), `--timeout`. Gcodes: `tools/bench/gcode/` (speed/extrude ladders from `gen_speed_ladder.py`, `e_*`), and `tools/sim/gcode/voron_*.gcode` (must sanitize: allowlist G0-G4/G90-G92/M82/M83/M400; heaters/fans dropped; anything else raises). `analyze.py` needs the local engine: `make -f Makefile.rust motion-engine-fast`.

Run dirs: `~/bench/runs/<YYYYmmdd-HHMMSS>/` — `print.gcode`, `printer.cfg` (the one used), `capture.bin/json`, `klippy.log`, `events/*.jsonl` (structured logs; query with the query-logs/mcu-diagnostics skills, files hold the Pi's whole history so filter by the run's time window), `run.json`, `analysis.json`.

## Reading analysis.json

`ok` = all four `checks`:

- `net_steps_match` — per motor, captured net steps (DIR-signed) equal the reference's. False = lost or extra steps.
- `all_steps_matched` — every captured step pairs with a reference step within 100 ms and vice versa (`unmatched_capture_steps`/`unmatched_reference_steps` = 0; lattice-edge ties are counted in `edge_tie_steps`, not as failures).
- `no_stalls` — no 5 ms bin where the reference has ≥3 steps and the capture none (or reverse); `stalls.items[]` give times.
- `timing_abs_max_within_50us` — max |capture − aligned reference| over motors. **Sign: positive = step late.** Healthy runs show p99 ≈ 13–25 µs, abs_max ≈ 33–37 µs (25 µs tick quantization).

Also useful: `epochs[]` (one per motion burst; `ppm` = clock drift fit, `residual_wander_us`, capture-vs-stream span delta), `motors.*.peak_step_rate_captured_hz` vs `_reference_hz`, `max_abs_count_error_steps`. More than one capture epoch vs one reference epoch (`AnalysisError: N capture epochs vs 1`) means the printer stopped and restarted mid-run (starved host); rerun analyze with a larger `--gap`.

Known deviations that are design, not bugs: extruder net E is shorter than gcode E (corner cut / shaper follower, `docs/rewrite/shaper.md`), exposed as `corexy.e_net_vs_gcode_corner_cut_mm`; sub-200 Hz steps are excluded from timing stats (`timing_steps_below_rate_floor`).

Crash triage: `journalctl -u klipper` on the Pi (`ssh bench-pi`) and `events/host-rust.jsonl` (`anchor_underrun`, `stream_worker_fatal`, `pacer_reserve_raised`); `klippy.log` `Stats` lines give `print_time`/`buffer_time` (a frozen `print_time` with draining `buffer_time` = producer stopped).

## Recovery

- klippy aborted: systemd restarts it after ~10 s; `prepare_printer` waits for ready.
- MCU latched in shutdown (survives klippy restarts; `prepare_printer` times out "printer did not become ready"): `deploy.py reset-target`. Full reflash: `deploy.py flash-target` (F401 fw from `f401.config` via Pi SWD). Capture board: `deploy.py flash-capture`.
- F407 whose firmware disabled SWD: set BOOT0=1 and press reset (manual). Ask the user for wiring changes (endstops, TMC UART, clock wire are planned, see README).

## Limits

No endstops/heaters/TMC wired (no homing; runs use `SET_KINEMATIC_POSITION`); klippy takes the non-TMC single-edge 2 µs step path, not the TMC both-edge path; 25 µs time resolution; Pi 5 is faster than Pi 4 printers; config pins PA13/PA14/PA8 are forbidden. Prefer `--host-cpus/--host-max-mhz` to approximate weaker hosts.
