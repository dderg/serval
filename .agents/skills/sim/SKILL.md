---
name: sim
description: Use when asked to test firmware or host-side changes end-to-end without a physical printer, reproduce motion/homing/probing bugs in simulation, validate a branch before merging, run G-code against real firmware, or compare branch behavior. Also use when setting up, debugging, or extending the Docker-based simulator in tools/sim/.
---

# Kalico Simulator (tools/sim)

Full-stack simulator: the real MACH_LINUX firmware ELFs (H7 + F4) and real klippy, speaking genuine msgproto over PTYs, with hardware faked by two LD_PRELOAD shims — `libvtime` (shared-memory virtual clock) and `libsim_intercept` (GPIO/SPI/PWM/ADC + chip-emulator sockets). It catches firmware bugs, protocol errors, and timing/state-machine issues that unit tests and offline planner runs cannot. The sim is a first-class firmware build config (`CONFIG_MCU_SIM=y`) — no source patching happens at image build.

## Quick start

```bash
tools/sim/run.sh                    # build image (incremental) + self-test print
tools/sim/run.sh --gcode f.gcode    # print a G-code file, report speedup
tools/sim/run.sh test               # full e2e pytest suite (homing, probe, beacon, phase stepping, motor adjust)
tools/sim/run.sh test -k probe      # subset by pytest -k expression
tools/sim/run.sh serve              # long-lived printer for Moonraker/Mainsail
tools/sim/run.sh shell              # bash inside the image
tools/sim/run.sh --branch X test    # any mode against another branch
tools/sim/run.sh --no-cache         # force full rebuild
```

Branch comparison: run the same mode with and without `--branch <base>`; each branch gets its own image tag (`kalico-sim-<branch>`) and its own BuildKit compile caches, so parallel builds don't clobber each other.

## Writing a new scenario test

Tests live in `tools/sim/tests/` and use the `sim_world` fixture from `tools/sim/conftest.py` (marker `needs_elf`). A test boots a printer from generated config text and drives it over klippy's API socket:

```python
from tools.sim import configs

def test_my_scenario(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X10 F3000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None
```

`SimWorld` (tools/sim/world.py) essentials: `gcode_ok()` / `gcode()` (dict with "error" key), `status()`, `toolhead_position()/toolhead_z()`, `print_file()`, `mark_log()` + `log_tail()` (incremental klippy.log reads), `events_text()` (structured events/*.jsonl), `wait_for_log_text()`, `sim_control(mcu)` (GPIO/ADC injection), `EndstopPulser` (fake switch cycling). Config generators are in `tools/sim/configs.py` (minimal, multi-Z + motor_adjust, phase stepping, sensorless-phase, beacon CoreXY, probe variants). On failure the fixture dumps klippy/MCU/event log tails automatically.

Run one test: `tools/sim/run.sh test -k my_scenario`.

## Architecture

```
┌──────────────────── Docker container ────────────────────┐
│  ┌──────────┐  PTY  ┌────────┐  PTY  ┌──────────┐       │
│  │ MCU H7   │◄─────►│ klippy │◄─────►│ MCU F4   │       │
│  │ elf      │       │ (real  │       │ elf      │       │
│  └──────────┘       │  time) │       └──────────┘       │
│  LD_PRELOAD:        └───┬────┘       LD_PRELOAD:        │
│  vtime+intercept        │ PTY        vtime+intercept    │
│                    ┌────┴─────┐                          │
│                    │ beacon   │  + TMC5160/2209/MAX31865 │
│                    │ emulator │    socket emulators      │
│                    └──────────┘                          │
└───────────────────────────────────────────────────────────┘
```

- klippy runs at real CPU speed; only MCU processes live on the virtual clock (loading vtime into klippy deadlocks).
- `CONFIG_MCU_SIM=y` firmware: motion tick registers as a vtime pacer (virtual time never skips a sample period), step queues notify the shim, and the timer-in-past / timer-too-close / tick-gap checks that police real-time hardware are compiled out (gated in src/linux/timer.c, src/sched.c, rust/runtime tick.rs + motion_core.rs).
- Auto-endstops: step-queue lines X=18/Y=7/Z=15 count toward a 50-step wall → endstop lines gpio200/201/202/203. Direct injection via each MCU's `sim_control` unix socket (`set_gpio_input`, `set_adc`, `get_gpio_output`, `get_steps`).
- Beacon: `emulators/beacon_mcu.py` speaks full msgproto (identify, streaming, trsync, contact, NVM, accel) and tracks Z via the shim's step counter. The klippy-side plugin is the `dderg/beacon_klipper` fork branch `motion-stack-rename`, pinned in `fetch_plugins.sh` — NOT upstream beacon3d (incompatible with the motion rewrite).
- One virtual clock per container (`/dev/shm/vtime`) → e2e tests run sequentially inside a container; parallelism = multiple `docker run`s (fully namespace-isolated, safe for concurrent agents).

## Build caching

BuildKit cache mounts keyed by branch (`SIM_CACHE_KEY`): Rust target/, per-MCU firmware OUT dirs (out-h7/, out-f4/). Warm rebuild costs: python-only edit ~seconds (firmware stage not invalidated — tools/sim python is deliberately NOT copied into the firmware stage), one C file ~10s, one Rust crate ~seconds. Cold build ~a few minutes.

## Common issues

| Issue | Fix |
|-------|-----|
| "Stepper too far in past" | Docker VM jitter — use `--privileged` or reduce homing speed (≤10 mm/s). |
| Beacon module import errors | The image must use the dderg/beacon_klipper `motion-stack-rename` fork (fetch_plugins.sh pin), not upstream. |
| klippy exits 255 | Config error — the fixture/CLI prints the klippy.log tail; usually wrong pin format (use `gpiochip0/gpioN`) or a missing section. |
| SIGSEGV with both shims | LD_PRELOAD order matters: `libvtime.so:libsim_intercept.so` (vtime FIRST — world.py does this). |
| First rebuilds after a big merge are slow | BuildKit re-warming; converges within a couple of builds. |

## CI

`./scripts/ci.sh sim` runs the in-process `sim_unit` emulator/shim contract tests (no ELF needed). The e2e suite (`run.sh test`) is Docker-only.
