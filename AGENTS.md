# Serval coding-agent guide

## Scope and authority

Serval is an actively developed Kalico fork that replaces the host motion stack with a Rust streaming, jerk-limited planner. It is safety-sensitive and multi-language: Python host/control code, Rust planner/runtime, C firmware, wire/FFI boundaries, and Docker simulation.

Treat executable code and tests as the authority. Public configuration, G-code, status, and protocol behavior are APIs. Serval Quickstart, migration, motion reference, and feature-status pages override inherited Kalico/Klipper documentation where they conflict. `docs/rewrite/`, `docs/plans/`, `docs/human-spec/`, and `docs/superpowers/` are design/history records, not normative operator instructions.

## Repository map

- `klippy/`: Python host lifecycle, config, G-code, MCU connection and extras. Motion bridge: `motion_setup.py`, `motion_kinematics.py`, `motion_engine.py`, `engine_mcu.py`, `mcu.py`.
- `rust/`: pinned workspace. `geometry` path/velocity geometry; `trajectory` tracks/processors; `planner-config` schema; `motion-core` orchestration; `motion-engine` PyO3; `motion-pipeline`/`pipeline-snapshot` planning regression; `mcu-protocol`/`mcu-transport` wire; `runtime`/`c-api` embedded ABI; `ethercat-rt` servo endpoint. Pipeline: fit → planner → lowerer → shaper → dispatcher → pump, assembled in `rust/motion-core/src/worker.rs`.
- `src/`: C firmware, ports, Kconfig and trajectory execution. `lib/` is vendored support.
- `rust/c-api/include/runtime.h`: generated, checked-in C ABI; regenerate, never hand-edit.
- `test/`: Python unit/contract tests; `test/configs/`: firmware CI configs.
- `tools/sim/`: Docker full-stack simulator, not a planner mock. `snapshots/`: deterministic real-planner baseline tests.
- `scripts/`: supported entry points; `ci.sh` is the gate dispatcher; `build-native.sh` builds host artifacts; `ci-build-mcu.sh` checks C+Rust firmware links.
- `docs/`: MkDocs source. Update `docs/Overview.md` and `docs/_kalico/mkdocs.yml` for new public pages. `site/` is generated.

## Build/runtime contracts

- Bootstrap Python: `uv sync --group dev` (Python 3.9+). Cargo within `rust/` uses `rust/rust-toolchain.toml`; do not casually change it.
- Build native host modules: `./scripts/build-native.sh`. It produces `klippy/_config_doc.so`, `_motion_engine.so`, `_shaper_ident.so`. Rebuild after Rust changes or branch changes. `--fast` is snapshot iteration; `--config-only` cannot exercise real motion.
- Klippy requires the config extension. Motion tests need the real engine; the native-less stub is only for explicit import/config tests.
- Firmware: `make menuconfig && make`. F4/G0/H7 are print targets; F103 builds/boots but is not print-supported. Host, artifacts, and **every flashed MCU** are one protocol version—never mix revisions.
- Use `tools/regen_headers.sh` for ABI header regeneration and review header drift.

## Safety and invariants

- Never flash/restart physical hardware, use `sudo`, set EtherCAT capabilities, or run bench commands unless explicitly asked and recovery conditions are clear.
- Compilation or simulation never proves hardware support. Preserve solid vs sim/bench vs exploratory wording in `docs/Feature_Status.md`.
- Timing/protocol faults are fail-stop. Do not hide them by weakening lead, bounded channels, backpressure, watchdogs, or fault reporting. Fix cause, home, and verify position before resume.
- Homing has far less buffered lead than normal motion and is sensitive to host/transport stalls.
- Preserve FFI ownership, wire layout/message IDs, clock handling, target runtime settings, and C/Rust symbol agreement. C must not free Rust allocations.
- Limits constrain post-processor motor output, not just nominal path. `mode_inverse` requires preceding smoothing; config must reject unsafe topology/order instead of silently reinterpreting it.

## Test ladder

Run from repository root; report exactly what ran and what was omitted.

- Python: `./scripts/ci.sh ruff`, then `./scripts/ci.sh py`; `py-typecheck` is deliberately scoped to servo modules.
- Rust host: `./scripts/ci.sh rust-host`; `./scripts/ci.sh quick` is the normal fast gate, not a replacement for integration/target coverage.
- Runtime/C/protocol/FFI: use affected `rust-mcu-*`, `cbindgen-drift`, `c-smoke`, `rust-no-stepper`, `rust-ethercat-hw`, and full firmware matrix as appropriate. Do not remove watchdog/panic checks to pass CI.
- Simulator contracts: `./scripts/ci.sh sim`; full stack/protocol: `tools/sim/run.sh test` (focus with `-k`, preserve logs via `--keep-logs`). Worlds are sequential by default.
- Planner output: `snapshots/snapshot-tests.sh --ci`; inspect differences before explicitly accepting a baseline. Never bulk-rebaseline.
- Docs: `./scripts/ci.sh docs` or `cd docs/_kalico && uv run mkdocs build --strict`.
- Always run `git diff --check`. Do not commit generated artifacts (`out/`, `target/`, `*.so`, `site/`, `.ci-logs/`, `.sim-logs/`).

## Documentation ownership

Update public references in the same PR:

- motion topology/limits/axes/motors/processors → `docs/Config_Reference_Motion.md`;
- inherited options → `docs/Config_Reference.md`;
- compatibility behavior → `Config_Changes.md` and `Config_Migration.md` when applicable;
- G-code/status/API → `G-Codes.md`, `Status_Reference.md`, `API_Server.md`;
- maturity/measurements/limitations → `Feature_Status.md` with evidence and tier;
- architecture/workflow → `Architecture.md` / `Development.md`.

References must include units, defaults, bounds, prerequisites, failure modes, compatibility impact, and safe recovery where relevant. New public pages must be linked in both Overview and MkDocs nav.
