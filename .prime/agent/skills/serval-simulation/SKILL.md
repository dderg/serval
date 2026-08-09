---
name: serval-simulation
description: Run, debug, or extend Serval's full-stack Docker simulator and trajectory snapshot verification. Use for firmware/protocol/homing/probe timing regressions without a physical printer.
---
# Serval simulation and snapshots

Read `docs/Simulator.md`, `tools/sim/README.md`, and `AGENTS.md`. The simulator runs real MACH_LINUX firmware and Klippy over PTYs with virtual MCU time and emulated devices; it is not physical-hardware validation.

## Commands

```bash
./scripts/ci.sh sim
tools/sim/run.sh test
tools/sim/run.sh test -k homing
tools/sim/run.sh test --keep-logs
tools/sim/run.sh --branch <branch> test
snapshots/snapshot-tests.sh --ci
```

`sim` is in-process contract coverage; `run.sh test` is Docker full-stack E2E. Worlds run sequentially by default because Klippy uses real CPU time. Preserve artifacts with `--keep-logs`; do not preload virtual time into Klippy. Inspect trajectory deltas before explicitly accepting a snapshot baseline—never bulk-rebaseline.
