---
name: kalico-sim
description: Use when asked to test firmware or host-side changes end-to-end without a physical printer, reproduce motion/homing bugs in simulation, validate a branch before merging, run G-code against real firmware, or compare branch behavior (e.g. main vs feature branch). Also use when setting up, debugging, or extending the Docker-based simulator.
---

# Kalico Simulator

Full-stack Klipper/Kalico simulator that runs the real MACH_LINUX firmware + klippy in Docker. It drives the actual MCU step/SPI/GPIO paths through an LD_PRELOAD shim, so it catches firmware bugs, protocol errors, and state-machine/timing issues that unit tests and offline planner runs cannot.

There is **one mode: full** (real firmware). A planner-only "batch" mode used to exist; it was removed — planner *correctness* is covered by the Rust unit tests (`cargo nextest`) and planner *timing* by running the planner directly. The simulator's job is real-firmware end-to-end.

## Quick Start

```bash
# From the simulator worktree or any branch that has tools/kalico-sim/:

# Self-test (generates a test G-code, runs the full pipeline):
docker run --rm kalico-sim

# Run a G-code file through the virtual SD card:
docker run --rm -v /path/to/file.gcode:/gcode/print.gcode:ro \
    kalico-sim --gcode /gcode/print.gcode --timeout 120

# Build + run for the current branch/worktree (incremental, cache-keyed by branch):
bash tools/kalico-sim/run.sh

# Build + run for a specific branch:
bash tools/kalico-sim/run.sh --branch sota-motion
```

## Architecture

```
┌──────────────────── Docker container ────────────────────┐
│                                                          │
│  ┌──────────┐  PTY  ┌────────┐  PTY  ┌──────────┐      │
│  │ MCU H7   │◄─────►│ klippy │◄─────►│ MCU F4   │      │
│  │ (klipper │       │(Python)│       │ (klipper │      │
│  │  .elf)   │       │        │       │  .elf)   │      │
│  └──────────┘       └────────┘       └──────────┘      │
│  LD_PRELOAD:         no shim          LD_PRELOAD:       │
│  libsim_intercept    (real time)      libsim_intercept  │
│  (GPIO/SPI/PWM)                       (GPIO/SPI/PWM)    │
│                                                          │
│  Virtual clock (libvtime) paces the simulated world.    │
└──────────────────────────────────────────────────────────┘
```

- **Speed**: ~1x real time (limited by MCU step execution in the Docker VM)
- **Output**: print time, pass/fail, error details
- **Use for**: firmware bug detection, protocol validation, motion/homing repro, branch comparison
- **Multi-MCU**: H7 + F4 both spawn (concurrently) and connect via PTY
- **Endstops**: auto-triggered via step counting in the GPIO shim (after N step pulses, the linked endstop GPIO triggers)

## Building the Docker Image

```bash
# Build for current branch (run.sh enables BuildKit + per-branch cache key):
bash tools/kalico-sim/run.sh

# Build for a specific branch (run.sh prepares an isolated build context):
bash tools/kalico-sim/run.sh --branch <branch-name>

# Direct build of the current tree:
DOCKER_BUILDKIT=1 docker build -t kalico-sim -f tools/kalico-sim/Dockerfile .
```

For branches with the Rust motion engine (like sota-motion), the Dockerfile:
1. Installs the Rust toolchain
2. Patches missing Linux stubs (`fix_linux_build.sh`)
3. Builds the Rust staticlib + the `_motion_engine.so` host module
4. Links everything into `klipper-h7-sim.elf` / `klipper-f4-sim.elf`

### Incremental builds (BuildKit cache mounts)

The Dockerfile uses BuildKit cache mounts so rebuilds recompile only what changed:
- **Rust `target/` + cargo registry** are cache-mounted (keyed by `SIM_CACHE_KEY` = branch). Editing one crate recompiles that crate, not the workspace.
- **Per-MCU firmware `OUT` dirs** (`out-h7/`, `out-f4/`) are cache-mounted — no `make clean`, so a C-source edit recompiles only the changed objects.
- The firmware stage copies only the parts of `tools/` it needs, so editing `runner.py` does **not** invalidate the firmware build.

Typical warm-cache rebuild costs: Python-only edit ~6s; one C file ~H7 11s / F4 1s; one Rust crate seconds. First (cold) build is ~50s.

## Parallel / multi-agent use

Each `docker run` gets fully private namespaces (PIDs, mounts, **its own `/dev/shm`**, sockets, PTYs), so concurrent runs cannot collide — validated with many simultaneous self-tests. `run.sh` extracts each branch build into a **unique, self-cleaning staging dir** and partitions the compile caches by branch (`SIM_CACHE_KEY`), so concurrent builds from different worktrees/branches neither race nor clobber each other's caches.

```bash
# Run N self-tests / G-code runs in parallel — no cross-talk:
for f in a.gcode b.gcode c.gcode d.gcode; do
    docker run --rm -v /path/$f:/gcode/f.gcode:ro \
        kalico-sim --gcode /gcode/f.gcode &
done
wait
```

## Files

| File | Purpose |
|------|---------|
| `tools/kalico-sim/Dockerfile` | Docker image — Ubuntu + gcc + Rust + firmware build (BuildKit cache mounts) |
| `tools/kalico-sim/run.sh` | Launcher: isolated build context, BuildKit, per-branch cache key, build + run |
| `tools/kalico-sim/runner.py` | Python orchestrator: spawns MCUs, klippy, monitors, reports |
| `tools/kalico-sim/libvtime/libsim_intercept.c` | GPIO/SPI/PWM/IIO LD_PRELOAD shim with auto-endstop |
| `tools/kalico-sim/libvtime/libvtime.c` | Virtual time shim (shared-memory clock) |
| `tools/kalico-sim/emulators/beacon_mcu.py` | Full Beacon eddy-current probe MCU emulator |
| `tools/kalico-sim/emulators/beacon_identify_dict.py` | Beacon firmware identify dictionary |
| `tools/kalico-sim/configs/h7-sim.config` | MACH_LINUX build config for H7-flavored MCU |
| `tools/kalico-sim/configs/f4-sim.config` | MACH_LINUX build config for F4-flavored MCU |
| `tools/kalico-sim/patches/fix_linux_build.sh` | Patches the tree for MACH_LINUX link errors |

## Beacon MCU Emulator

The simulator includes a full Beacon eddy-current probe emulator (`emulators/beacon_mcu.py`) that speaks Klipper's msgproto wire protocol over a PTY. It emulates everything the real Beacon firmware does:

**Implemented features:**
- Full msgproto wire protocol (identify, config, finalize, clock sync)
- Delta-compressed frequency sample streaming (`beacon_data` at 1600 Hz)
- Thermal telemetry (`beacon_status` at 10 Hz — MCU temp, supply voltage, coil temp)
- Z-aware frequency model: `freq = base + coeff / (z + offset)` — frequency varies realistically with distance to bed
- NVM reads (65536-byte image with calibration sentinels)
- Proximity homing trigger: watches frequency vs threshold, fires trsync
- Contact homing trigger: fires trsync after configurable delay
- Contact query state tracking
- Accelerometer streaming (`beacon_accel_data` at 6 kSps)
- trsync protocol (config, start, trigger, set_timeout, stepper_stop_on_trigger)

**Usage:** The emulator starts automatically when klippy's config references a Beacon probe. The runner creates the Beacon PTY and passes it to klippy's config via the serial override system.

**Adjusting Z position:**
```python
beacon_stub.set_z(5.0)  # 5mm above bed — affects frequency samples
```

## GPIO Shim — How It Works

`libsim_intercept.so` intercepts `open`, `ioctl`, `read`, `write`, `close` for:
- `/dev/gpiochip*` → simulated GPIO lines (step, dir, enable, endstop)
- `/dev/spidev*` → routed to chip emulator sockets (TMC5160, MAX31865)
- `/sys/class/pwm/*` → simulated PWM (heaters, fans)
- `/sys/bus/iio/*` → simulated ADC (thermistors)

**Auto-endstop**: The shim counts rising edges on step pins. After N steps (default 50), it sets the linked endstop GPIO to triggered. After the endstop triggers, it clears after 10 retract steps. This simulates physical endstop contact during homing.

**Control socket**: Each MCU gets a `sim_control` Unix socket at `$KALICO_SIM_SOCK_DIR/sim_control` for runtime GPIO/ADC injection:
```
set_gpio_input chip=0 line=10 value=1   # trigger endstop
set_adc channel=0 value=3900            # set ADC reading
get_gpio_output chip=0 line=0           # read step pin
```

## Printer Config for Sim

The simulator generates a minimal config when none is provided. Key constraints:

- **Pin format**: `gpiochip0/gpioN` (MACH_LINUX, not STM32 `PA3`)
- **Homing speed**: ≤10 mm/s (Docker VM jitter causes "Stepper too far in past" at higher rates)
- **`[force_move]` enabled**: Allows `SET_KINEMATIC_POSITION` as homing fallback
- **`[input_shaper]` with `smooth_mzv`**: Required on sota-motion (the motion engine rejects freq=0)
- **`[virtual_sdcard]` path**: Must match the directory where G-code files are placed

## Adding a New Test

1. Create a G-code file in `tools/kalico-sim/tests/`
2. Run it: `docker run --rm -v /path/to/test.gcode:/gcode/t.gcode:ro kalico-sim --gcode /gcode/t.gcode --timeout 120`
3. For branch comparison: run the same G-code against two Docker images built from different branches

## Common Issues

| Issue | Fix |
|-------|-----|
| "Stepper too far in past" | Docker VM jitter — use `--privileged` or reduce homing speed. |
| "Unknown pin chip name 'probe'" | Config references Beacon probe. Use minimal config (no `--config` flag). |
| "shaper frequency must be finite" | Add `[input_shaper]` with `shaper_freq_x/y: 50` and `shaper_type: smooth_mzv` |
| Rust linking errors on sota-motion | `fix_linux_build.sh` should run automatically. Check the Dockerfile `RUN bash ... fix_linux_build.sh` step. |
| SIGSEGV with both LD_PRELOAD shims | Order matters: `libvtime.so:libsim_intercept.so` (vtime FIRST). |
| `_motion_engine.so` cannot be loaded | The Rust host module did not build/copy — check the rust-build stage. |
| "klippy exited with code 255" | Config error — check the klippy log. Usually a missing extruder or wrong pin names. |
| First two rebuilds after a big merge are slow | BuildKit re-warming its layer cache; it converges to incremental speeds within a couple of builds. |
