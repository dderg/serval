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
tools/sim/run.sh --branch X ...     # any of the above for another branch
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

- The firmware is built with `CONFIG_MCU_SIM=y`: the motion tick registers
  as a vtime pacer (virtual time can never skip a sample period), step
  queues notify the shim (auto-endstops, Beacon Z tracking), and the
  timer-in-past/timer-too-close/tick-gap checks that only exist to police
  real-time hardware are compiled out. No source patching happens at image
  build — the sim is a first-class build config.
- klippy runs at real CPU speed; only MCU processes live on the virtual
  clock. Loading vtime into klippy deadlocks — don't.
- Auto-endstops (libsim_intercept): step-queue lines X=18/Y=7/Z=15 count
  toward a 50-step wall, asserting endstop lines gpio200/201/202/203.
- Each SimWorld allocates its own virtual-clock segment
  (`/dev/shm/vtime-<pid>-<n>`, passed to the shims via `VTIME_SHM_NAME`),
  so worlds never share state. `run.sh test` still runs tests
  sequentially by default — klippy lives on the real clock, and CPU
  contention from concurrent worlds flakes its timing budgets.
  `SIM_TEST_JOBS=N` opts into pytest-xdist parallelism;
  `SIM_TEST_TARGETS` narrows the run to specific test files (how the CI
  shards split the suite across separate runners).

## CI

- `./scripts/ci.sh sim` runs the in-process `sim_unit` tests (no ELF).
- `tools/sim/run.sh test` runs the full-stack e2e suite (Docker).
