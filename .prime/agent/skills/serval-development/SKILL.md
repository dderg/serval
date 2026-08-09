---
name: serval-development
description: Build, change, test, or review Serval's Python/Rust/C firmware motion stack. Use when modifying planner, protocol, MCU runtime, FFI, configuration, or CI in this repository.
---
# Serval development

Read `AGENTS.md` first. Serval has Python host, Rust planner/runtime, C firmware, and hardware timing contracts; code/tests are authoritative.

## Mandatory boundaries

- Host native artifacts, the host, and every flashed MCU are one protocol version. Never mix revisions.
- Preserve fail-stop timing behavior, bounded backpressure, clock synchronization, watchdogs, wire layouts, and FFI ownership (C never frees Rust memory).
- F4/G0/H7 are printing targets; F103 builds but is not print-supported. A simulator/compile pass does not prove hardware safety.

## Workflow

1. Locate the owning layer: `klippy/`, `rust/`, `src/`, transport, or endpoint.
2. Build native modules for real motion: `./scripts/build-native.sh`.
3. Add a focused regression test, then run the smallest relevant gate.
4. Widen validation and report commands actually run.

```bash
./scripts/ci.sh ruff
./scripts/ci.sh py
./scripts/ci.sh rust-host
./scripts/ci.sh quick
./scripts/ci.sh docs
```

For C/Rust/ABI/protocol changes, run the applicable `rust-mcu-*`, `cbindgen-drift`, `c-smoke`, `rust-no-stepper`, and firmware-link gates described in `scripts/ci.sh`. Regenerate ABI headers with `tools/regen_headers.sh`; never edit `rust/c-api/include/runtime.h` manually.

For full details, read `AGENTS.md` and `docs/Development.md`.
