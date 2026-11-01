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
SIM_JITTER=3 tools/sim/run.sh test  # share a cpuset with 3 bursty CPU burners
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
  instead of spinning on it. The pacer registers inside `timer_init`,
  before the MCU clock starts, and holds the clock from then on: while it
  was released until the first timer dispatch, a starved MCU let virtual
  time leap tens of seconds past its boot timers ("Rescheduled timer in
  the past" before klippy ever connected). Step queues notify the shim
  (auto-endstops, Beacon Z tracking). The timer-in-past and
  timer-too-close checks stay live; only the tick-gap fault is compiled
  out, because the sim's tick is a paced thread rather than an interrupt.
  No source patching happens at image build — the sim is a first-class
  build config.
- Every process (MCUs, klippy) preloads `libvtime` and shares one virtual
  clock, which is event driven rather than tied to real time:
  - Every timed wait (sleeps, `poll`/`epoll`/`select`, futexes, condvars,
    semaphores) publishes the virtual time it waits for; whoever advances
    the clock onto it wakes that thread at once (a real-time signal).
  - Virtual time stands still while any klippy thread is outside a
    blocking wait, and cannot pass a klippy wait deadline, so host
    computation costs no printer time and klippy can never be late.
  - A klippy thread that another thread woke (futex, condvar, semaphore,
    pipe or eventfd) is runnable before its wait returns. Every such wake
    makes the next advance first check `/proc` for runnable klippy
    threads, so a woken thread holds the clock until the kernel actually
    runs it. Without this, a thread starved for tens of milliseconds let
    the 1 ms-stepping reactor carry virtual time seconds ahead of it.
  - Every clock advance computes its bound and stores the new time under
    one shared lock, which every floor drop (an MCU arming an earlier
    timer, a klippy thread starting to run) also takes: an advancer
    preempted between the two can no longer step past a floor that
    dropped meanwhile.
  - Bytes klippy writes to a device (an MCU's pty, or an emulator's
    registered with `VirtualClock.claim_link`) hold the clock until the
    device has read and handled them, so a request is answered at the
    virtual instant klippy sent it however late the device process is
    scheduled (a `get_clock` answered a scheduling-dependent time later
    skews clock sync). An MCU stepping to its own imminent timer may run
    ahead of its unread input; its next task round reads it. The firmware
    is built with `_FORTIFY_SOURCE`, so libvtime also intercepts
    `__read_chk`, `__ppoll_chk` and `__poll_chk`: without them the MCU's
    console reads and idle ppoll bypassed the shim entirely. A device
    whose reading thread is itself stepping to its own imminent timer
    cannot read until time reaches that timer, so its unread input lets
    the clock run exactly that far: two MCUs each waiting on their own
    timer with input pending used to wedge each other.
  - A `poll`/`ppoll`/`select`/`epoll` whose descriptors are already ready
    returns at once without moving the clock. serialport-rs polls for
    writability before every write and readability before every read, and
    the wait used to advance virtual time to the next pacer floor first:
    each get_clock left klippy one MCU tick (0.2–2 ms) before it was
    written, and the clock estimate converged to only ±1–3 ppm with a
    +0.3 ms offset bias. Now it tracks a skewed crystal exactly
    (`test_clock_estimate.py`).
  - The reverse direction holds too: bytes an MCU writes to its pty hold
    the clock while a klippy thread is waiting on that pty (in `ppoll` or
    a blocking `read`), until klippy has read them, so a reply reaches
    klippy at the virtual instant the MCU sent it (`get_clock` was read
    0–1.4 ms late, median 0.4 ms). The MCU counts a reply before its
    `write` returns: klippy wakes in real time the moment the bytes land,
    and until it marks itself running only that count keeps the clock
    still. The hold needs a waiting reader, so an MCU writing while klippy
    has no reader on its pty (boot output, a restart) cannot wedge.
  - `world.inject(mcu, ...)` faults one MCU's host→MCU link in virtual
    time, without touching klippy: `delay_command="queue_digital_out",
    by_s=0.34` holds the next frame carrying that command, and everything
    klippy sends behind it, for `by_s`; `drop_command=...` discards the
    next `count` such frames (klippy retransmits); `stall_s=0.05` holds the
    whole link. `late_command="queue_digital_out", by_s=0.337` models the
    host producing just that command late while every other frame goes out
    on time: the command is cut from its frame (same seq, CRC resealed) and
    appended to the first frame the MCU will accept once `by_s` has passed,
    so it reaches the MCU in order with a valid seq. A standalone frame
    would need a seq of its own and every later seq and ack renumbered both
    ways; riding the next frame costs the gap to it (the MCU log reports
    the actual hold). The MCU's libvtime walks the incoming Klipper frames
    with each command's parameter kinds from its data dictionary
    (`out/klipper-<board>-sim.dict`, passed as `MCU_SIM_CMD_SHAPES`), wakes
    the MCU when held bytes come due, and logs each fault to the MCU log
    (`[vtime] fault: delaying frame carrying msgid ...`, `... sending late
    msgid ... held N ns`). Held bytes still count as read, so a fault never
    holds the clock. A late command that shuts the MCU down longjmps out of
    its frame, losing any frames the MCU read behind it in the same chunk:
    the MCU NAKs (`[mcu-seq] MISMATCH`) until klippy retransmits them, and
    whether any were read behind it is up to scheduling.
  - `sim_world(..., crystal_ppm={"f4": 30.0})` gives an MCU its own
    oscillator (`VTIME_CRYSTAL_PPM`): every clock that process reads runs
    that many ppm off the shared timeline, and the deadlines it sets map
    back. Step timestamps from the GPIO shim stay on the shared timeline,
    so step timing compares across MCUs.
  - A process klippy forks drops the host role in the child, so its
    waits and its exit cannot touch klippy's accounting (a forked child's
    exit used to clear `host_present` for the whole world).
  - Worlds keep klippy's default `clock_sync_stable_ppm` (5 ppm). With
    `get_clock` answered at the instant it is sent, the estimator converges
    as it does on hardware. The former 1000 ppm sim override let the engine
    freeze its host→MCU projection on a 300–600 ppm-off estimate taken at
    connect: a secondary MCU's steps drifted ~20 ms off the plan over
    30–60 s, then snapped back at the idle rebase (contact probes drifted
    ~1 µm per sample, then jumped 0.02 mm).
  - While test code runs between world calls, virtual time is paced at
    real time (the printer idles while its "operator" thinks); inside a
    world wait (`gcode`, `print_file`, log/status waits, `wait_until`) it
    runs as fast as the processes allow.
  - Harness code that watches or pokes the printer while it moves must sit
    on the clock too: `world.wait_until(pred, ...)`, `EndstopWall`,
    `EndstopPulser`, or a `HarnessPacer` for new emulators. A plain
    `time.sleep` polling loop running beside a free-running wait samples
    the printer at an arbitrary, load-dependent cadence. A poke that must
    land at a printer position (closing a switch once an axis passes a
    point) belongs inside the `wait_until` predicate: after the wait
    returns, a world held free-running by another call runs on before the
    poke lands. The Beacon/Cartographer emulators sample, check homing
    thresholds and check bed contact on pacer seats, reading Z from the
    step counter at each check; the harness reserves four seats.
  - klippy writes its log from its own thread, so a gcode response can
    arrive before the lines it printed: assert on log text with
    `mark_log()` + `expect_log(needle)`, not `klippy_log_text()` right
    after `gcode_ok`.
- Harness timeouts are virtual seconds (`VirtualDeadline`); a run whose
  virtual clock stops advancing for 60 s of real time fails as wedged, and
  the error names what holds the clock (host threads, floors, the lock).
  `logs/vtime.csv` records virtual against real time for every world.
- `SIM_JITTER=N` pins the test container and N burner containers (50 ms
  spin / 20 ms sleep) to one cpuset (`SIM_JITTER_CPUS`, default `0,1`),
  reproducing the uneven preemption of a loaded CI runner that a flat
  `SIM_DOCKER_CPUS` quota does not; the burners are removed on exit.
  `tools/sim/tests/test_vtime_wake_holds.py` is the deterministic version:
  `VTIME_TEST_WOKEN_SPIN_US` keeps every early-woken klippy thread runnable
  without running, and the clock must not move meanwhile.
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
  Each klippy gets its own `-I` input tty in its world directory: parallel
  worlds racing on the default `/tmp/printer` symlink failed klippy start.
- `VTIME_SPEED` caps virtual seconds per real second inside world waits
  (tests default to uncapped). A world built with `host_gated=False`
  (`test_print_pressure.py`) instead runs free at its speed whether or not
  klippy keeps up — the throughput gate for a host that must keep pace
  with real time.

## CI

- `./scripts/ci.sh sim` runs the in-process `sim_unit` tests (no ELF).
- `tools/sim/run.sh test` runs the full-stack e2e suite (Docker).
