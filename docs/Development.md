# Developing Serval

This guide describes the repository's supported local workflows. Serval contains Python host code, a Rust workspace, C firmware, and Docker-backed integration tests; choose the smallest relevant gate first.

## Prerequisites

- Python **3.9+** (the project metadata is `pyproject.toml`); use `uv sync --group dev` for the Python development environment.
- A Rust toolchain. The repository pins its toolchain under `rust/`; `rustup` selects it when Cargo runs there.
- A C toolchain and the cross targets needed for firmware work.
- Docker with BuildKit for the full simulator. Docker is optional for focused host/Rust unit work.

For the comprehensive local CI workflow, `scripts/ci.sh` documents optional one-time tools: Rust embedded targets, nightly Miri, `cargo-nextest`, and `cargo-deny`. Do not install hardware-only EtherCAT dependencies unless working on that endpoint.

## Bootstrap and native artifacts

```bash
git clone https://github.com/dderg/serval.git
cd serval
uv sync --group dev
./scripts/build-native.sh
```

The native build is required for actual motion. It creates the three modules under `klippy/` described in [Architecture](Architecture.md). Import-only tests may intentionally use a stub when the engine is absent, but any test that exercises motion must use the real module. Rebuild after changing Rust code or switching a branch that changes native sources.

Useful variants:

```bash
./scripts/build-native.sh --fast          # snapshot-profile engine for fast iteration
./scripts/build-native.sh --config-only   # config extension only
./scripts/build-native.sh --bench --ethercat stub
```

`--bench --ethercat hw` links the real IgH EtherCAT stack and belongs only on a correctly prepared bench host.

## Test ladder

Run checks from the repository root. Start narrow, then widen before a pull request.

| Change area | First command | What it covers |
| --- | --- | --- |
| Python formatting/lint | `./scripts/ci.sh ruff` | Repository Ruff check and format check. |
| Python unit/contract tests | `./scripts/ci.sh py` | Python suite; Docker is used when available, otherwise the local interpreter is used. |
| Rust unit/docs/format/lint | `./scripts/ci.sh rust-host` | Workspace tests, documentation tests, Clippy, and `cargo fmt --check`. |
| Fast broad gate | `./scripts/ci.sh quick` | The script's fast lint/Rust subset. |
| Simulator unit contracts | `./scripts/ci.sh sim` | In-process simulator tests marked `sim_unit`; no firmware ELFs. |
| Full simulator E2E | `tools/sim/run.sh test` | Docker full stack: real MACH_LINUX firmware ELFs and Klippy over PTYs. |
| Firmware or protocol changes | `./scripts/ci.sh` | Full script-selected local gate set; also run the exact MCU target or simulator scenario affected. |

See `./scripts/ci.sh --help` for the complete job list and `./scripts/ci.sh -v <job>` to stream a job. CI is authoritative for its platform matrix; a successful local subset does not prove a firmware/hardware change safe.

### Simulator

The simulator is a full integration environment, not an offline planner mock. It builds a Docker image, runs real MACH_LINUX firmware processes and Klippy, and connects them through PTYs with virtual-time and device emulation shims.

```bash
tools/sim/run.sh                         # build and run a self-test print
tools/sim/run.sh --gcode path/to/job.gcode
tools/sim/run.sh test -k homing
tools/sim/run.sh test --keep-logs
tools/sim/run.sh serve                   # long-lived simulated printer
tools/sim/run.sh shell
```

Tests are sequential by default because Klippy uses real CPU time and parallel worlds can create timing flakes. `SIM_TEST_JOBS=N` opts into parallelism. `--keep-logs` preserves simulator artifacts in `.sim-logs/`; include relevant logs when reporting a failure. Read `tools/sim/README.md` before changing simulator behavior.

### Firmware builds

For a real board, configure and compile in the usual firmware flow:

```bash
make menuconfig
make
```

The permitted Serval targets are intentionally narrower than upstream. Confirm support in [Quickstart](Quickstart.md#check-your-board-first) and [Feature status](Feature_Status.md) before presenting a build as usable on hardware. A firmware/protocol change should be exercised in simulation and on the target board; do not flash an unreviewed change to a printer that cannot be safely recovered.

## Documentation and configuration changes

Treat configuration and G-code as public APIs. Update the matching reference in the same pull request:

- motion topology, limits, axes, motors, or post-processors: `docs/Config_Reference_Motion.md`;
- classic/inherited options: `docs/Config_Reference.md`;
- behavior-changing configuration: `docs/Config_Changes.md` and, where applicable, `docs/Config_Migration.md`;
- commands/status/API: `docs/G-Codes.md`, `docs/Status_Reference.md`, or `docs/API_Server.md`;
- support maturity or a demonstrated limitation: `docs/Feature_Status.md`.

Add audience-facing pages to both `docs/Overview.md` and `docs/_kalico/mkdocs.yml`. Build the documentation site with the docs project's own environment when changing navigation or Markdown:

```bash
cd docs/_kalico
uv sync
uv run mkdocs build --strict
```

Warnings are valuable: do not mask an unresolved link or anchor by weakening validation.

## Pull-request expectations

Use a focused branch, explain the user-visible behavior and risk, and report exactly what you ran (including omitted gates and hardware). Keep generated artifacts out of commits unless the repository explicitly tracks them. Before review, inspect `git diff --check`, run the relevant test ladder, and update documentation with the implementation.

This repository inherits contribution material from Kalico; where it conflicts with the active Serval branch, the checked-in code, CI, and this guide describe the current engineering workflow. Preserve third-party licensing and existing file headers when modifying code.
