# Simulator and verification

Serval's simulator is a Docker full-stack integration environment. It runs real MACH_LINUX firmware ELFs and real Klippy over PTYs, with virtual MCU time and emulated GPIO/SPI/PWM/ADC hardware. It catches protocol, firmware, state-machine, and timing-integration regressions that unit tests or offline planner output cannot.

> It is not physical-printer validation. The simulator emulates devices/endstops and disables selected checks that exist to police physical timing. A pass does not prove wiring, power, thermals, mechanics, or a drive/board combination safe.

## Commands

Run from the repository root:

```bash
tools/sim/run.sh                         # build image and self-test print
tools/sim/run.sh --gcode job.gcode       # run supplied G-code
tools/sim/run.sh test                    # full end-to-end pytest suite
tools/sim/run.sh test -k homing          # focused E2E selection
tools/sim/run.sh test --keep-logs        # retain worlds in .sim-logs/
tools/sim/run.sh serve                    # long-lived printer for Moonraker
tools/sim/run.sh shell                    # shell in image
tools/sim/run.sh test --branch sota-motion
```

Docker BuildKit is required. The image tag is branch-specific so concurrent worktrees do not test a stale image. `--branch` builds an archived branch with current simulator tooling overlaid for comparison.

## Test layers

`./scripts/ci.sh sim` runs in-process tests marked `sim_unit` and does not require firmware ELFs. `tools/sim/run.sh test` is the Docker full-stack suite. The latter is sequential by default: Klippy itself runs at real CPU speed, and concurrent worlds can create timing flakes. Set `SIM_TEST_JOBS=N` only when deliberately trading determinism for throughput; `SIM_TEST_TARGETS` narrows target files.

Use `--keep-logs` for a failure report. Each world then preserves Klippy, MCU, and structured-event artifacts. Do not preload virtual time into Klippy; virtual time belongs to MCU processes and loading it into Klippy can deadlock.

## What to test

Use focused scenarios while developing, then choose the wider layer matching the change: Python contracts for host logic; simulator unit tests for emulator contracts; full E2E for firmware, protocol, homing, probe, phase, transport, or state-machine behavior; snapshot tests for intended trajectory-output changes. The [Developer guide](Development.md) gives the complete test ladder.
