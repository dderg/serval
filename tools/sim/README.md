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
  with 400 µs slack, and an imminent timer waits on the virtual clock
  instead of spinning on it. Step queues notify the shim (auto-endstops,
  Beacon Z tracking). The timer-in-past and timer-too-close checks stay
  live; only the tick-gap fault is compiled out, because the sim's tick is
  a paced thread rather than an interrupt. No source patching happens at
  image build — the sim is a first-class build config.
- Every process (MCUs, klippy) preloads `libvtime` and shares one virtual
  clock, which is event driven rather than tied to real time:
  - Every timed wait (sleeps, `poll`/`epoll`/`select`, futexes, condvars,
    semaphores) publishes the virtual time it waits for; whoever advances
    the clock onto it wakes that thread at once (a real-time signal).
  - Virtual time stands still while any klippy thread is outside a
    blocking wait, and cannot pass a klippy wait deadline, so host
    computation costs no printer time and klippy can never be late.
  - While test code runs between world calls, virtual time is paced at
    real time (the printer idles while its "operator" thinks); inside a
    world wait (`gcode`, `print_file`, log/status waits, `wait_until`) it
    runs as fast as the processes allow.
  - Harness code that watches or pokes the printer while it moves must sit
    on the clock too: `world.wait_until(pred, ...)`, `EndstopWall`,
    `EndstopPulser`, or a `HarnessPacer` for new emulators. A plain
    `time.sleep` polling loop running beside a free-running wait samples
    the printer at an arbitrary, load-dependent cadence.
- Harness timeouts are virtual seconds (`VirtualDeadline`); a run whose
  virtual clock stops advancing for 60 s of real time fails as wedged.
  `logs/vtime.csv` records virtual against real time for every world.
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
- `VTIME_SPEED` caps virtual seconds per real second inside world waits
  (tests default to uncapped). A world built with `host_gated=False`
  (`test_print_pressure.py`) instead runs free at its speed whether or not
  klippy keeps up — the throughput gate for a host that must keep pace
  with real time.

## CI

- `./scripts/ci.sh sim` runs the in-process `sim_unit` tests (no ELF).
- `tools/sim/run.sh test` runs the full-stack e2e suite (Docker).
