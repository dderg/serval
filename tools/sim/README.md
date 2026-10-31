# Kalico Simulator

Full-stack simulator: the real MACH_LINUX firmware ELFs (H7 + F4) and real
klippy, talking genuine msgproto over PTYs, with hardware faked by two
LD_PRELOAD shims — `libvtime` (shared-memory virtual clock) and
`libsim_intercept` (GPIO/SPI/PWM/ADC + chip-emulator sockets). It catches
firmware bugs, protocol errors, and timing/state-machine issues that unit
tests and offline planner runs cannot.

## Usage

```bash
tools/sim/run.sh                    # build image + self-test print
tools/sim/run.sh --gcode f.gcode    # print a G-code file
tools/sim/run.sh test               # full e2e pytest suite
tools/sim/run.sh test -k probe      # subset
tools/sim/run.sh serve              # long-lived printer for Moonraker
tools/sim/run.sh shell              # poke around inside the image
tools/sim/run.sh test --branch X    # any mode for another branch; the mode comes first
```

## Layout

| Path | Purpose |
|------|---------|
| `world.py` | `SimWorld` — spawns MCUs/emulators/klippy, drives G-code over the API socket |
| `configs.py` | Generated printer configs (minimal, multi-Z, phase stepping, beacon, probe variants) |
| `cli.py` | Container entrypoint: print / serve |
| `conftest.py` | `sim_world` pytest fixture (dumps logs on failure) |
| `tests/` | `sim_unit` emulator contract tests + `needs_elf` e2e scenarios |
| `emulators/` | TMC5160/TMC2209/MAX31865 chip emulators, Beacon MCU emulator |
| `preload/` | libvtime + libsim_intercept LD_PRELOAD shims |
| `configs/` | MACH_LINUX firmware build configs (CONFIG_MCU_SIM=y) |
| `fetch_plugins.sh` | Pins the dderg/beacon_klipper `motion-stack-rename` fork |

## How the pieces fit

- The firmware is built with `CONFIG_MCU_SIM=y`. Motion sampling and
  command handling share the Linux scheduler thread; no background thread
  mutates the runtime. The next scheduler deadline bounds virtual time
  with 400 µs slack. Every timed wait in a preloaded process (sleeps,
  `poll`/`epoll`/`select`, futexes, condvars, semaphores) nudges virtual
  time toward its deadline within the speed cap and pacer floors, and the
  thread owning the virtual timer gets `SIGALRM` when it comes due. Step
  queues notify the shim (auto-endstops, Beacon Z tracking). The
  timer-in-past and timer-too-close checks stay live; only the tick-gap
  fault is compiled out, because the sim's tick is a paced thread rather
  than an interrupt. No source patching happens at image build — the sim
  is a first-class build config.
- klippy preloads `libvtime` too and shares the MCUs' virtual clock, so a
  virtual-time stall freezes host and MCU together and clock sync sees the
  nominal MCU frequency. The harness measures its timeouts on the same
  clock (`VirtualDeadline`) and fails a run whose virtual clock stops
  advancing for 60 s of real time. `logs/vtime.csv` records virtual
  against real time for every world.
- Auto-endstops (libsim_intercept): `SimWorld` passes each rendered classic
  motor's step/dir pins into the shim. GPIO edges advance 50-step walls that
  assert gpio200/201/202/203. Sample lanes retain runtime notifications on
  X=18/Y=7/Z=15.
- Each SimWorld allocates its own virtual-clock segment
  (`/dev/shm/vtime-<pid>-<n>`, passed to the shims via `VTIME_SHM_NAME`),
  so worlds never share state. Local `run.sh test` runs 4 pytest-xdist
  workers by default and CI shards run sequentially; `SIM_TEST_JOBS=N`
  overrides (0 = sequential), and `SIM_TEST_TARGETS` narrows the run to
  specific test files (how the CI shards split the suite across runners).
- `VTIME_SPEED` caps virtual seconds per real second (default 1.0). Every
  process shares the virtual clock, so a lower speed only buys more real
  CPU time per virtual second; it never changes the printer's timing.
  Tests that require a minimum elapsed MCU-clock window must still reach
  that window.

## CI

- `./scripts/ci.sh sim` runs the in-process `sim_unit` tests (no ELF).
- `tools/sim/run.sh test` runs the full-stack e2e suite (Docker).
