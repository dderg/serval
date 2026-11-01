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
tools/sim/run.sh test --branch X    # mode first, then --branch: any mode against another branch
tools/sim/run.sh --no-cache         # force full rebuild
SIM_JITTER=3 tools/sim/run.sh test # pin the container + 3 bursty burners to one cpuset (CI-runner jitter repro)
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

`SimWorld` (tools/sim/world.py) essentials: `gcode_ok()` / `gcode()` (dict with "error" key), `status()`, `toolhead_position()/toolhead_z()`, `print_file()`, `mark_log()` + `log_tail()` (incremental klippy.log reads), `events_text()` (structured events/*.jsonl), `wait_for_log_text()`, `wait_until(pred, timeout, what)` (poll a condition on printer time), `sim_control(mcu)` (GPIO/ADC injection), `EndstopPulser` / `EndstopWall` (emulated switches). Config generators are in `tools/sim/configs.py` (minimal, multi-Z + motor_adjust, phase stepping, sensorless-phase, beacon CoreXY, probe variants). On failure the fixture dumps klippy/MCU/event log tails automatically.

Timing rule for test code: anything that watches or pokes the printer *while it moves* (toggling a pin when an axis reaches a point, waiting for step counts) must use `world.wait_until`, `EndstopWall`, `EndstopPulser`, or a `HarnessPacer` for a new emulator thread. Inside world waits virtual time runs much faster than real time, so a `time.sleep` polling loop samples the printer at an arbitrary, load-dependent cadence.

Run one test: `tools/sim/run.sh test -k my_scenario`.

## Architecture

```
┌──────────────────── Docker container ────────────────────┐
│  ┌──────────┐  PTY  ┌────────┐  PTY  ┌──────────┐       │
│  │ MCU H7   │◄─────►│ klippy │◄─────►│ MCU F4   │       │
│  │ elf      │       │ (same  │       │ elf      │       │
│  └──────────┘       │  clock)│       └──────────┘       │
│  LD_PRELOAD:        └───┬────┘       LD_PRELOAD:        │
│  vtime+intercept        │ PTY        vtime+intercept    │
│                    ┌────┴─────┐                          │
│                    │ beacon   │  + TMC5160/2209/MAX31865 │
│                    │ emulator │    socket emulators      │
│                    └──────────┘                          │
└───────────────────────────────────────────────────────────┘
```

- One event-driven virtual clock for every process (klippy preloads libvtime too). Each timed wait publishes its wake time and is woken the moment the clock reaches it; virtual time stands still while any klippy thread is busy and never passes a klippy wait deadline, so host compute costs no printer time. Between world calls (test code running) time is paced at real time; inside world waits it runs free (~7-15x on prints). Harness timeouts (`gcode_ok(timeout=…)`, `print_file`, log waits) are virtual seconds; a run whose virtual clock stops for 60 s of real time fails as wedged. `logs/vtime.csv` records virtual vs real time per world. `host_gated=False` (print-pressure test) runs free at `vtime_speed` so klippy must keep up like a real host.
- A klippy thread woken by another (futex/condvar/semaphore/pipe/eventfd) holds the clock until the kernel actually runs it: each such wake makes the next clock advance scan `/proc` for runnable klippy threads first. Clock advances and floor drops (an MCU arming an earlier timer, a klippy thread starting to run) share one lock, so a preempted advancer cannot step past a floor that dropped under it. A wedge (`virtual clock stuck … wedged`) names what holds the clock. To reproduce scheduling-dependent flakes use `SIM_JITTER=N` (cpuset `SIM_JITTER_CPUS`, default `0,1`; burners removed on exit), not a flat `SIM_DOCKER_CPUS` quota; `tools/sim/tests/test_vtime_wake_holds.py` forces the starved-woken-thread case deterministically via `VTIME_TEST_WOKEN_SPIN_US`.
- Bytes klippy writes to a device (MCU pty, or an emulator pty registered via `VirtualClock.claim_link`) hold the clock until the device has read and handled them: requests are answered at the virtual instant they were sent (`get_clock` latency 0), whatever the scheduling. Emulator reactor threads must report handled bytes through their `DeviceLink`. The firmware is built with `_FORTIFY_SOURCE`, so libvtime intercepts `__read_chk`/`__ppoll_chk`/`__poll_chk` as well as the plain calls — a new interposed libc call needs its fortified twin too. The MCU pacer registers inside `timer_init` and holds from then on (a released pacer at boot let time leap past boot timers: "Rescheduled timer in the past" before connect). A process klippy forks drops the host role. Each klippy gets its own `-I` input tty (parallel worlds raced on `/tmp/printer`).
- Worlds keep klippy's default `clock_sync_stable_ppm` (5). With device links `get_clock` is answered at the instant it is sent, so the estimator converges as on hardware; the former 1000 ppm sim override let the engine freeze its host→MCU projection on a 300–600 ppm-off connect-time estimate, so secondary-MCU steps drifted ~20 ms off the plan over 30–60 s and snapped back at the idle rebase (contact probes drifted ~1 µm/sample, then jumped 0.02 mm). Never loosen it to hide a convergence problem.
- Per-MCU crystals: `sim_world(cfg, crystal_ppm={"h7": -20.0, "f4": 30.0})` (env `VTIME_CRYSTAL_PPM` per MCU process) runs that MCU's clock off the shared timeline; GPIO step timestamps (`get_step_times`) stay on the shared timeline, so cross-MCU step timing is directly comparable (`test_clock_resync.py`). The engine re-syncs every serial MCU's frozen host→MCU projection at every rest (idle resume and dwell rejoin), logged as `clock_resync` (`correction_us`, `rest_s`, `applied`); a correction that would reach back past the rest's start waits for a longer rest. A device whose reading thread is stepping its own imminent timer no longer holds the clock with its unread input (two MCUs used to wedge each other).
- A `poll`/`ppoll`/`select`/`epoll` on descriptors that are already ready returns without moving the virtual clock. serialport-rs polls before every write/read; the wait used to advance time to the next pacer floor first, delaying every get_clock by an MCU tick and limiting the clock estimate to ±1–3 ppm / +0.3 ms. A device whose reader is stepping its own timer lets time run only to that timer. `test_clock_estimate.py` asserts the engine's live clock record tracks each skewed crystal (< 5 ppm, < 50 µs after 10 s).
- Emulators run their triggers on pacer seats: the Beacon/Cartographer sample loop, homing-threshold monitor and bed-contact monitor read Z from the step counter at each paced check (the harness reserves four seats). In tests, a poke that must land at a printer position goes inside the `world.wait_until` predicate — after the wait returns, a world kept free-running by another call (a G28 in a thread) runs on before the poke lands. klippy logs from its own thread: assert on log text with `mark_log()` + `expect_log(needle)`, never `klippy_log_text()` right after `gcode_ok`.
- `CONFIG_MCU_SIM=y` firmware: motion tick registers as a vtime pacer (virtual time never skips a sample period), step queues notify the shim. Timer-too-close and timer-in-past checks stay live; only the tick-gap fault is compiled out (rust/runtime tick.rs), since the sim's tick is a paced thread, not an interrupt.
- Auto-endstops: step-queue lines X=18/Y=7/Z=15 count toward a 50-step wall → endstop lines gpio200/201/202/203. Direct injection via each MCU's `sim_control` unix socket (`set_gpio_input`, `set_adc`, `get_gpio_output`, `get_steps`).
- Beacon: `emulators/beacon_mcu.py` speaks full msgproto (identify, streaming, trsync, contact, NVM, accel) and tracks Z via the shim's step counter. The klippy-side plugin is the `dderg/beacon_klipper` fork branch `motion-stack-rename`, pinned in `fetch_plugins.sh` — NOT upstream beacon3d (incompatible with the motion rewrite).
- Each SimWorld has its own virtual clock (`/dev/shm/vtime-<pid>-<n>`, handed to the shims via `VTIME_SHM_NAME`). Local `run.sh test` runs 4 pytest-xdist workers by default, CI shards run sequentially; `SIM_TEST_JOBS=N` overrides (0 = sequential), `SIM_TEST_TARGETS` picks test files. Separate `docker run`s remain fully isolated for concurrent agents.

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
