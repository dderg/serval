---
project_name: 'improved-kalico'
user_name: 'dderg'
date: '2026-06-17'
sections_completed: ['technology_stack', 'language_specific', 'framework_specific', 'testing', 'code_quality', 'workflow', 'dont_miss']
existing_patterns_found: 25
status: 'complete'
rule_count: 121
optimized_for_llm: true
---

# Project Context for AI Agents

_This file contains critical rules and patterns that AI agents must follow when implementing code in this project. Focus on unobvious details that agents might otherwise miss._

---

## Technology Stack & Versions

- **Rust workspace** (`rust/`, 14 crates): toolchain pinned `1.85.0` via `rust-toolchain.toml`; editions `2021` (host) / `2024` (`motion-engine`, `rust-version=1.85`); resolver `2`. Bumping the toolchain is a deliberate, regression-tested act — MCU codegen is sensitive to LLVM target-feature renames and FPU flag strings.
- **Python host** (`klippy/`, `pyproject.toml`): Python `>=3.9`; `numpy~=2.x`, `greenlet`, and `cffi` are version-pinned **per Python minor** via PEP 508 marker expressions — any new dep in that family must carry the full set of markers.
- **C MCU firmware** (`src/`, `firmware/`): targets `thumbv7em-none-eabi` (H723 Cortex-M7, F446 Cortex-M4) and `thumbv6m-none-eabi` (G0B1 Cortex-M0+); links the `rust/c-api` staticlib + the checked-in `c-api/include/nurbs.h` cbindgen header.
- **PyO3 bridge** (`rust/motion-engine` cdylib `_motion_engine`): `pyo3 0.29` `abi3-py39`; `Makefile.rust` builds `--features extension-module` and copies the artifact to `klippy/_motion_engine.so` (`.so` extension on all platforms; macOS `.dylib` is renamed on copy).
- **Key Rust deps (workspace):** `clarabel 0.11` (SOCP solver — bumping is a planner-correctness change), `thiserror 2`, `heapless 0.8`, `tracing 0.1`, `crossbeam-channel 0.5`, `arc-swap 1`, `serde_json 1`, `time 0.3` (formatting only, never parsing).
- **Python dev/prototype:** `ruff>=0.9.3` + `pre-commit>=4.0.1` + `pytest>=8.3.4` + `pytest-xdist>=3.6.1`; prototype group adds `scipy>=1.14`, `matplotlib>=3.8` (Py ≥3.10 only).

## Critical Implementation Rules

### Language-Specific Rules

#### Rust
- **`unsafe_code = "deny"`** workspace-wide (`rust/Cargo.toml` `[lints.rust]`). Adding `unsafe` requires a workspace-level override with rationale — never a per-module `#[allow(unsafe_code)]`.
- **Pedantic clippy is on at `warn`** and CI runs `cargo clippy --workspace --all-targets -- -D warnings`, so any new pedantic lint fires red. A large allow-list already exists in `[workspace.lints.clippy]` with rationale comments — extend it only with a comment explaining why; do not silence lints inline with `#[allow(...)]` unless the case is truly one-off.
- **Module layout:** `foo.rs` + sibling `foo/` directory (modern Rust style). A `foo.rs` module ends with `#[cfg(test)] mod tests;` pointing at `foo/tests.rs`.
- **Unit tests live in a separate file** from the tested code (enforced by `CLAUDE.md`). Pattern: `#[cfg(test)] mod tests;` at the foot of `foo.rs`, bodies in `foo/tests.rs` opening with `use super::*;`. Never inline `#[cfg(test)] mod tests { … }`.
- **No comments — code is self-documenting.** Comments get outdated and lie. Make the code say it: rename, extract, assert, or compute the value. The only acceptable comment is a **same-line reminder or bookmark** (e.g. a `// TODO:` / `// FIXME:` marker). Remove useless pre-existing comments in files you touch. Block/section comments that narrate code are forbidden.
- **Fail loudly.** When adding checks for unexpected conditions, raise a clear error — do not recover, advance, or pad. Example: a movement segment arriving late → raise, do not shift the start time.
- **Editions are mixed:** `motion-engine` is `edition = "2024"` with `rust-version = "1.85"`; other crates are `edition = "2021"`. Respect the edition of the crate you're editing.
- **MCU cross-build flags are load-bearing.** `rust/.cargo/config.toml` sets `target-cpu=cortex-m4` for `thumbv7em` (must be safe on both M7 and M4 — M7-only encodings HardFault the F446) and `--cfg portable_atomic_unsafe_assume_single_core` for `thumbv6m`. Do not change these without reading the rationale comments and re-running the MCU CI jobs.
- **Release profile is tuned for the Pi host:** `opt-level=3`, `lto="fat"`, `codegen-units=1`, `panic="abort"`, `debug=true`. Build-time crates (`syn`, `cbindgen`) are forced to `opt-level=0` to avoid OOM on low-memory hosts. Don't reorder these without a Pi build check.
- **`extension-module` PyO3 feature:** production cdylib only — see Framework-Specific Rules for the mechanism and the macOS trap.

#### Python
- **Ruff line-length 80**, indent 4, with `I001`/`I002` (import sorting + required-import) and `B006` (no mutable defaults) extended-selected. `E501`/`E741`/`F841` etc. are ignored — don't fight them.
- **Ruff excludes** `./.github`, `./.history`, `./config`, `./docs`, `./lib`, `./src` — these are vendored/generated; do not add new top-level dirs to the exclude list casually.
- **`pytest` config** (`pyproject.toml`): `pythonpath = [".", "klippy"]`; `testpaths = ["test", "tests"]` only. `tools/` is deliberately excluded — its CI-able subset runs via explicit path + `sim_unit` marker (see `scripts/ci.sh sim`). Do not widen `testpaths`.
- **Pytest markers** are a hard contract: `sim_unit` (CI-able, no ELF/hardware), `needs_elf`, `needs_renode`, `needs_hardware`. Mark every new test correctly or CI will either skip it or fail it.
- **Structured logging only** on the MCU/structured-diag side (`event_log_emit` → `events/*.jsonl`, codes in `rust/runtime/src/log_codes.rs`). Do not use `printf`/`output()` for new diagnostics.

### Framework-Specific Rules

#### PyO3 bridge (`rust/motion-engine` → `klippy/_motion_engine.so`)
- **The cdylib lib name is `_motion_engine`** (underscore-prefixed) so it does not shadow the pure-Python wrapper `klippy/motion_engine.py`. `klippy/` code imports `motion_engine` (the wrapper) or accesses `mcu._motion_engine`; never `import _motion_engine` directly in new klippy code.
- **Build via `make -f Makefile.rust motion-engine` from the repo root.** It passes `--features extension-module` and copies `rust/target/release/lib_motion_engine.{so,dylib}` → `klippy/_motion_engine.so`. The file Python imports must end in `.so`; on macOS the cargo `.dylib` is renamed on copy (Linux already emits `.so`).
- **No maturin / setuptools / `pyproject.toml` build backend.** The bridge is cargo + a Makefile copy step. Do not "modernize" the build.
- **`extension-module` is a crate-level Cargo feature, NOT in `[features] default`.** The Makefile adds it only for the production cdylib; `cargo test`/`cargo run` inherit defaults and never link it. Mechanism: the feature tells the linker *not* to link `libpython` — Python provides the symbols at runtime via `dlopen`. **macOS trap:** a test binary with `extension-module` links cleanly on macOS (deferred symbol resolution) but fails on Linux (unresolved `PyErr_Print`) — so a Mac dev will pass locally and break Linux CI. If you `cargo add` a dev-dep that transitively pulls `pyo3`, pull it with `default-features = false` to avoid re-breaking the test build.
- **`abi3-py39`** is the PyO3 ABI floor — the built extension works on any Python ≥3.9 without rebuilding. Don't tighten it; if you use a >3.9-only PyO3 API, verify the floor still holds.

#### cbindgen / C FFI (`rust/c-api`)
- **Two headers are checked in:** `c-api/include/nurbs.h` and `c-api/include/runtime.h`. Both are cbindgen-generated; **never edit them by hand.**
- **Regenerate via `tools/regen_headers.sh`** (what `cbindgen-drift` CI runs), or directly: `cargo run -p c-api --bin gen-headers --no-default-features --features header-nurbs` (or `--features header-runtime`). **Exactly one** header feature per invocation — `--no-default-features` is required because the crate `default` enables both, which the bin rejects. Config lives in `c-api/cbindgen.toml` (nurbs) and `c-api/cbindgen-runtime.toml` (runtime).
- **`cbindgen-drift` CI** (`scripts/ci.sh:88`) regenerates and fails on any `git diff` in `rust/c-api/include/`. Run it locally before committing FFI changes.
- **`c-smoke` CI** (`scripts/ci.sh:93`) builds `c-api` and runs `--test c_smoke_build`, compiling a C TU that includes the headers against the produced archive — this is the ABI-consumption check.
- **FFI signature convention:** `#[unsafe(no_mangle)] pub unsafe extern "C" fn nurbs_*` (Rust 2024 unsafe-attr syntax). Raw pointers (`*const`/`*mut`) at the seam, never references. **Every FFI fn carries a `// SAFETY:` comment** stating preconditions (non-null, valid, lifetime); null/validity is the **C caller's responsibility** — the Rust side does not runtime-null-check (see `rust/c-api/src/nurbs_ffi.rs` for the exemplar).
- **`#[repr(C)]` and `extern "C"` only** at the seam. No Rust-specific layout.
- **Type ownership:** C never frees Rust-allocated memory. Constructors and destructors come in pairs across the boundary; pointer types are opaque to C.
- **`c-api` is the single crate that allows `unsafe`:** `#![allow(unsafe_code)]` in `rust/c-api/src/lib.rs` (FFI requires it). This is a scoped exception to the workspace `unsafe_code = "deny"`; every other crate keeps the deny.

#### Panic & abort policy (FFI safety)
- **Workspace `[profile.release] panic = "abort"`** (`rust/Cargo.toml`) — release panics abort, never unwind into C.
- **MCU `c-api` has a custom `#[panic_handler]`** (`rust/c-api/src/lib.rs:14`) that calls `rust_panic_latch()` → Klipper `shutdown()` (noreturn). MCU panics abort loudly.
- **`panic-grep` CI** (`scripts/ci.sh:119`) emits LLVM-IR for the MCU build and **fails if `panic_bounds_check` appears in any `nurbs_*` de Boor evaluator function.** NURBS eval must be panic-free — no panicking indexing, no `unwrap`, no `expect` on the hot eval path.

#### `no_std` MCU crates
- **`c-api`, `nurbs`, `runtime`** are `#![cfg_attr(not(feature = "host"), no_std)]`: MCU build = `no_std`, host build = `std`. Do not `use std::*` in these crates unless gated by `#[cfg(feature = "host")]`. `heapless` and `portable-atomic` are the no_std-friendly collections/atomics.

#### C/Rust MCU boundary (`docs/rewrite/mcu-c-rust-boundary.md`)
- **C owns boot, safety-critical paths, and all shared-memory placement.** Rust owns the motion engine. The seam is `extern "C"` + `#[repr(C)]` only.
- **No `#[link_section]` on Rust statics** to place them in C-named sections — C places the storage and Rust references it through the FFI. (This rule is MCU-specific; host crates may use `#[link_section]` legitimately.)
- **Read `docs/rewrite/mcu-c-rust-boundary.md` before** adding shared state between C and Rust on the MCU.
- **MCU boundary integration is NOT in CI.** The `sim` CI job explicitly excludes `needs_renode` and `needs_hardware` (`scripts/ci.sh:180`). Renode dual-board sim and bench flashing (Neptune/Trident) are **manual** verification — do not assume `cargo nextest` covers MCU integration.

#### EtherCAT endpoint (`rust/ethercat-rt`)
- **`--features hw` is opt-in** and builds the IgH (EtherLab) backend on the Pi (`csrc/libecrt_igh.c`, links `-lethercat`; `IGH_DIR`/`IGH_LIB_DIR` default to `/opt/etherlab`). IgH is the only EtherCAT master backend — SOEM was removed. Never built in CI. The stub binary `ethercat-rt-stub` (no `--features`) is the CI-able path and **must mirror the hw FFI surface** so a stub build failure catches drift.
- **Missing libethercat → cargo link error** against `-lethercat` at build. **Missing `setcap` → runtime `EPERM`** on raw socket creation (not a build error). The hw binary must fail loudly at startup if caps are missing.
- **`make -f Makefile.rust setcap-ethercat`** (sudo, once per rebuild) grants `cap_net_raw,cap_sys_nice,cap_ipc_lock+ep` so the endpoint runs unprivileged (raw socket, RT sched, mlockall).

### Testing Rules

#### Pre-PR gate (merge-blocking)
- **`./scripts/ci.sh quick` must be fully green before opening or updating a PR.** It bundles exactly: `ruff`, `rust-test`, `rust-clippy`, `rust-fmt`, `watchdog-canary` (`scripts/ci.sh:217`). Same set CI runs first; red here = red PR. `quick` does NOT run Python host tests — if you touched `klippy/`, also run `./scripts/ci.sh py`. A failing test in review means the review is rejected, not that the reviewer triages your failure.
- **Local-iteration job names use HYPHENS** (the bash functions use underscores, but the user-facing subcommands do not): `./scripts/ci.sh rust-test`, `rust-clippy`, `rust-fmt`, `rust-loom`, `rust-mcu-h7`, `rust-mcu-f4`, `rust-mcu-g0`, `rust-no-stepper`, `cbindgen-drift`, `c-smoke`, `miri`, `panic-grep`, `deny`, `sim`, `py`, `docs`. `./scripts/ci.sh rust_clippy` fails (job-not-found).

#### Running a single test
- **Rust / nextest** (substring over the fully-qualified test name, no `--exact`): `cargo nextest run -p trajectory -E 'test(curve_de_boor)'` or `cargo nextest run -p runtime -E 'test(/^loom_seqlock/)'` (regex pin).
- **Rust / doc-tests** (skipped by nextest): `cargo test --doc -p trajectory` — scope with `-p` or it rebuilds every doctest in the workspace.
- **Rust / Loom** is the documented exception to "never `cargo test`": `cargo test -p runtime --release loom_seqquick -- --exact` (Loom's virtual scheduler requires `cargo test`, not nextest).
- **Python:** `pytest test/test_toolhead.py::test_move_full -v` or `pytest -k "corner and not slow" -v`. **Use `-n0` when debugging** — xdist under `-n auto` captures stdout and reorders failures; you will lose hours otherwise.

#### Rust
- **Run `cargo nextest run` from `rust/`, NOT `cargo test`** (Loom excepted). ~110 test binaries; `cargo test` runs them one-at-a-time (~100s), nextest schedules into one pool (~11s).
- **Unit tests live in a separate file** — see Language-Specific Rules for the full pattern. The `#[cfg(test)] mod tests;` *declaration* lives at the foot of `src/<module>.rs`; test *functions* live in `src/<module>/tests.rs` opening with `use super::*;`. Never write `#[cfg(test)] mod tests { fn ... }` blocks in production files.
- **Unit vs integration — the decision rule, not just the location:**
  - **Unit** (`rust/<crate>/src/<mod>/tests.rs`): tests internal/`pub(crate)`/private items, single crate, no I/O. Where ~90% of logic tests belong. If the public API doesn't expose what you need, **add a unit test; do not widen the public API to test it.**
  - **Integration** (`rust/<crate>/tests/*.rs`): black-box, public API only, each file is a separate crate — cannot see private items, cannot `mod common;` across files without `#[path = "common/mod.rs"] mod common;`. Use for cross-crate or end-to-end behavior.
- **Integration-test helpers:** `rust/<crate>/tests/common/mod.rs`, pulled in per-file via `#[path = "common/mod.rs"] mod common;`. Don't spin a `test_utils` crate unless ≥3 integration files share non-trivial setup.
- **Test-only deps go in `[dev-dependencies]`**, never `[dependencies]`. `proptest` is the property-test library (already a dev-dep in `gcode`, `host-rt`, `nurbs`, `runtime`); reach for it on math-invariant code (NURBS convex-hull containment, continuity at knots, trajectory monotonicity). No `insta`/`approx`/`rstest` — snapshot and float-comparison conventions are ad-hoc; state your tolerance explicitly in the test.
- **Test naming:** `module_under_test_condition_or_input`, snake_case. Test module mirrors source module (`jerk_limit.rs` → `mod tests` covering `jerk_limit::*`).
- **`clippy -D warnings` across `--workspace --all-targets`** — tests are linted too; a pedantic violation in a test is a red PR. Run via `./scripts/ci.sh rust-clippy`, not a hand-rolled `cargo clippy` (ci.sh pins exact flags).
- **`cargo fmt --all -- --check`** enforced — run via `./scripts/ci.sh rust-fmt`.
- **Loom** for new lock-free/atomic code in `runtime` (`./scripts/ci.sh rust-loom`; slow, run on demand not every save).
- **Miri** for new `unsafe` in `runtime`/`c-api` (`./scripts/ci.sh miri`; nightly-only, slow).
- **`panic-grep`** (`./scripts/ci.sh panic-grep`) greps the **MCU release binary's** panic strings — it fails if `panic_bounds_check` appears in any `nurbs_*` de Boor *production* eval function. `#[cfg(test)]` modules are not compiled into the MCU release build, so test code can panic freely; the rule applies to **production eval-path code**: no `unwrap`/`expect`/direct indexing/`panic!`/`unreachable!` on the hot eval path.

#### Python
- **`pytest` + `pytest-xdist`.** `pythonpath = [".", "klippy"]`; `testpaths = ["test", "tests"]`. `test/` = unit tests (`test_*.py`, the only `conftest.py`); `tests/` = integration/sim/host dirs (`klipper_sim`, `klippy_host`, `motion_engine`, `tmc_sensorless`). `tools/` is deliberately excluded — its CI-able subset runs via explicit path + `sim_unit` marker.
- **Pytest markers are a hard contract** (registered in `pyproject.toml`, never redefine): `sim_unit` = pure-Python, no ELF/MCU/hardware; add `needs_elf`/`needs_renode`/`needs_hardware` only if the test will fail without that dependency. `./scripts/ci.sh sim` selects `sim_unit and not needs_hardware and not needs_renode`.
- **Fixtures:** `test/conftest.py` is the single cross-suite fixture site (no `tests/conftest.py`). Put new fixtures at the shallowest conftest that covers all callers; don't duplicate.
- **Ruff** over the whole repo (`./scripts/ci.sh ruff`): `ruff check` + `ruff format --check`. Tests are not exempt.

#### Negative-test obligation (fail-loudly)
- CLAUDE.md mandates failing loudly on late segments, overflow, malformed input. **Every `return Err(...)`/`panic!`/`abort` on a planner/MCU input boundary has a paired test asserting that exact error fires.** An error path without a test is a loud failure nobody hears.

#### MCU / firmware
- **MCU Rust is built for three targets** via `./scripts/ci.sh rust-mcu-h7` / `rust-mcu-f4` / `rust-mcu-g0` (`thumbv7em-none-eabi` for H723+F446, `thumbv6m-none-eabi` for G0B1); each sets `RUNTIME_STORAGE_SIZE`/`RUNTIME_PIECE_RING_SIZE` env vars. A change to `runtime`/`c-api`/`nurbs` must pass all three. Install targets locally with `rustup target add thumbv7em-none-eabi thumbv6m-none-eabi`.
- **`./scripts/ci.sh rust-no-stepper`** builds the workspace without `motion-module-stepper` — verifies the feature gate compiles. Run it when you touch the stepper dispatch surface.
- **`./scripts/ci.sh cbindgen-drift`** when you touch `rust/c-api` headers — regenerates and fails on any `git diff` in `rust/c-api/include/`.

#### Throughput regression (NOT automated)
- **There is no automated throughput-regression gate yet — the planner pipeline is not yet end-to-end.** This is a known gap, not a regression; add throughput verification when the pipeline matures. `rust/trajectory/tests/continuous_throughput_repro.rs` is a *repro* of a past bug, not a gate. Until a gate exists, treat any planner change as needing manual throughput spot-checks. Per CLAUDE.md, print throughput is non-negotiable — the gate is owed, not optional, once the pipeline is working.

#### Hardware / sim verification (manual, outside CI)
- The `sim` CI job excludes `needs_renode` and `needs_hardware`. For manual verification use the skills: `mcu-sim` (Docker-based sim, no physical printer), `renode-simulation` (dual-board Renode), `neptune-bench` (Neptune 3 Pro), `trident-bench` (Trident H723/F446).

### Code Quality & Style Rules

#### Rust
- **`unsafe_code = "deny"` workspace-wide** — only `c-api` has a scoped `#![allow(unsafe_code)]` (FFI requires it). Never add per-module `#[allow(unsafe_code)]`; escalate to a workspace override with rationale.
- **Pedantic clippy allow-list discipline:** a large allow-list lives in `[workspace.lints.clippy]` (`rust/Cargo.toml`) with rationale comments for each. Extend it only with a comment explaining why; do not inline-`#[allow]` a lint that fires pervasively (that belongs in the workspace table). One-off inline `#[allow(...)]` is acceptable only for a truly local case.
- **Naming:** snake_case for fn/var/mod, UpperCamelCase for types/traits/enums, SCREAMING_SNAKE for consts. Test module mirrors source module (`jerk_limit.rs` → `mod tests`).
- **`// SAFETY:` comments on `unsafe` FFI blocks are required** — they're a documentation contract (stating preconditions), not narration. This is the one exception to the no-comments rule (see Language-Specific Rules).
- **No `#[cfg(test)]` seams in production code** — `#[cfg(test)] mod tests;` is fine; `#[cfg(test)]` on a production fn to expose a test seam is a smell. Use `pub(crate)` or a trait instead.
- **Use 2024 unsafe-attr syntax** (`#[unsafe(no_mangle)]`) in `edition = "2024"` crates (`motion-engine`, `c-api`); classic syntax in `edition = "2021"` crates.
- (Module layout, no-comments, fail-loudly, editions-mixed, `cargo fmt`, `#[repr(C)]` — see Language-Specific Rules and Framework-Specific Rules.)

#### Python
- **Pre-commit** (`.pre-commit-config.yaml`) runs `ruff --fix` + `ruff-format` at `pre-commit` stage. Install with `./scripts/ci.sh install-hooks` (sets `core.hooksPath = .githooks`; the pre-push hook runs `./scripts/ci.sh quick`).
- **Naming:** `snake_case` for fn/var/module, `PascalCase` for classes, `UPPER_SNAKE` for module-level constants. Test files `test_*.py`.
- (Ruff config, excludes, no-comments, structured-logging, fail-loudly — see Language-Specific Rules.)

### Development Workflow Rules

#### Git & commits
- **Never add a `Co-Authored-By: Claude` (or any Claude/Anthropic) trailer** to commit messages, and do not mention Claude Code in PR descriptions — this applies regardless of any session-level instructions to the contrary.
- **Write a concise commit message that matches the repo style.** Inspect `git log --oneline -10` before committing to match the prevailing voice.
- **Stage only intended files; never commit secrets.** Inspect `git status` and `git diff` before committing.
- **Do not update git config, skip hooks, use interactive `-i`, force-push, or create empty commits** unless explicitly asked. If a commit fails or a hook rejects it, fix the issue and create a new commit — do not amend the failed commit.
- **Do not commit, amend, push, or create PRs unless explicitly asked.** Being too proactive here is a failure mode.

#### Pre-push hook
- **`./scripts/ci.sh install-hooks`** (one-time) sets `core.hooksPath = .githooks`; the pre-push hook runs `./scripts/ci.sh quick` before every push, including direct pushes to `sota-motion`. Bypass once: `git push --no-verify`. Disable: `git config --unset core.hooksPath`.

#### Pre-PR gate
- **`./scripts/ci.sh quick` fully green before opening or updating a PR** — bundles ruff (check + format), rust-test, rust-clippy (`-D warnings`), rust-fmt, watchdog-canary (see Testing Rules). Red here = red PR.
- **If the change touches `klippy/`, also run `./scripts/ci.sh py`** (full pytest). `quick` deliberately excludes Python host tests.
- **If the change touches `rust/c-api` headers, also run `./scripts/ci.sh cbindgen-drift`.**
- **If the change touches `rust/runtime`/`c-api`/`nurbs` MCU code, all three MCU targets must pass** (`rust-mcu-h7`/`-f4`/`-g0`); CI runs them, run locally with `rustup target add` if you have the targets.
- **Before creating a PR:** inspect `git status`, `git diff`, `remote tracking`, `recent commits`, and the diff from the base branch. Review all commits in the PR, not just the latest. Use `gh` for GitHub tasks and return the PR URL.

#### Skills (project-specific)
- **`mcu-sim`** — Docker-based simulator for end-to-end firmware/host tests without a physical printer.
- **`renode-simulation`** — dual-board Renode sim (inter-board comms, GPIO/UART inspection).
- **`mcu-diagnostics`** / **`query-logs`** — structured-log investigation (events/*.jsonl, VictoriaLogs LogsQL).
- **`neptune-bench`** / **`trident-bench`** — test-bench addresses + flash scripts.

### Critical Don't-Miss Rules

#### Anti-patterns (do not do these)
- **Do not ship a measurably slower trajectory to make planning easier.** Print throughput is non-negotiable (CLAUDE.md). If the Pi can't keep up, the answer is to optimize the implementation, parallelize across cores, or upgrade the host — never a cheaper algorithm that produces a slower trajectory. The throughput-regression gate is not yet built (the planner pipeline is not end-to-end); see Testing Rules for the current manual-verification expectation.
- **Do not unwind a panic across `extern "C"`.** Workspace `[profile.release] panic = "abort"` and the MCU `#[panic_handler]` → `rust_panic_latch()` (Klipper `shutdown()`, noreturn) are the safety net. On the `c-api` FFI path, never rely on `catch_unwind`; rely on the abort policy and keep NURBS eval panic-free (`panic-grep` CI).
- **Do not `use std::*` in `c-api`/`nurbs`/`runtime`** without `#[cfg(feature = "host")]`. These crates are `#![cfg_attr(not(feature = "host"), no_std)]`; the MCU build is `no_std`. `heaples` + `portable-atomic` are the no_std-friendly substitutes.
- **Do not `#[link_section]` a Rust static on the MCU** to place it in a C-named section. C owns shared-memory placement; Rust references C-placed storage through the FFI. (MCU-specific; host crates may use `#[link_section]` legitimately.)
- **Do not enable `extension-module` on a test binary.** It links cleanly on macOS (deferred symbol resolution) but fails on Linux (`unresolved PyErr_Print`) — you will pass locally and break Linux CI. `extension-module` is NOT in `[features] default`; the Makefile adds it only for the production cdylib.
- **Do not hand-edit `c-api/include/nurbs.h` or `c-api/include/runtime.h`.** Both are cbindgen-generated; `cbindgen-drift` CI regenerates and fails on any `git diff`. Regenerate via `tools/regen_headers.sh` or `cargo run -p c-api --bin gen-headers --no-default-features --features header-nurbs` (exactly one header feature per invocation).
- **Do not inline `#[cfg(test)] mod tests { fn ... }` blocks** in production files. Test functions live in `src/<module>/tests.rs`; the parent declares `#[cfg(test)] mod tests;` at its foot.
- **Do not write comments.** Comments get outdated and lie. Make the code say it: rename, extract, assert, compute. Same-line `// TODO:`/`// FIXME:` bookmarks are the only exception. `// SAFETY:` on `unsafe` FFI blocks is a required contract, not narration.
- **Do not recover, advance, or pad on unexpected input.** Fail loudly — `return Err(...)`, `panic!` (host), `abort` (MCU), `raise` (Python). A movement segment arriving late → raise, do not shift the start time.
- **Do not widen the public API to test a private item.** Add a unit test in `src/<mod>/tests.rs` (which can see private items via `use super::*;`) instead.
- **Do not put test-only deps in `[dependencies]`.** They go in `[dev-dependencies]`; shipping a runtime dep used only in tests is a red PR.
- **Do not use `printf`/`output()` for new MCU/structured diagnostics.** Use `event_log_emit` → `events/*.jsonl` (codes in `rust/runtime/src/log_codes.rs`).

#### Edge cases & gotchas
- **MCU `target-cpu=cortex-m4` is load-bearing for `thumbv7em`.** The same Rust target covers H723 (Cortex-M7) and F446 (Cortex-M4); M7-only instruction encodings HardFault the F446. Do not raise `target-cpu` to `cortex-m7` without reading the rationale in `rust/.cargo/config.toml` and re-running all three MCU CI jobs.
- **`thumbv6m` (G0B1) has no LDREX/STREX.** `portable_atomic_unsafe_assume_single_core` cfg activates the interrupt-mask fallback for atomics — sound only because the G0B1 is single-core. Do not add this cfg to `thumbv7em` (H7/F4 have native exclusive-monitor instructions; the fallback is never used there and the cfg would be misleading).
- **Two cbindgen headers, not one.** `nurbs.h` and `runtime.h` are generated from `cbindgen.toml` and `cbindgen-runtime.toml` respectively. `gen-headers` rejects `--features header-nurbs,header-runtime` (both at once); `--no-default-features` is required because the crate `default` enables both.
- **`c-api` is the only crate with `#![allow(unsafe_code)]`.** Every other crate keeps the workspace `unsafe_code = "deny"`. Adding `unsafe` elsewhere requires a workspace-level override with rationale.
- **`panic-grep` greps the MCU release binary, not test code.** `#[cfg(test)]` modules are not compiled into the MCU release build; test code can panic freely. The rule applies to **production eval-path code** in `nurbs`: no `unwrap`/`expect`/direct indexing/`panic!`/`unreachable!` on the hot eval path.
- **Loom is the documented exception to "never `cargo test`.** Loom's virtual scheduler requires `cargo test`, not nextest. Run via `./scripts/ci.sh rust-loom`.
- **`ci.sh` job names use HYPHENS** (`rust-test`, `rust-clippy`, `cbindgen-drift`); the bash functions use underscores (`job_rust_test`). `./scripts/ci.sh rust_clippy` fails with job-not-found.
- **`quick` excludes Python tests.** `./scripts/ci.sh quick` runs ruff + rust-test + rust-clippy + rust-fmt + watchdog-canary only. If you touched `klippy/`, run `./scripts/ci.sh py` separately.
- **`test/` vs `tests/`:** `test/` = pytest unit tests (`test_*.py`, the single `conftest.py`); `tests/` = integration/sim/host dirs (`klipper_sim`, `klippy_host`, `motion_engine`, `tmc_sensorless`). `tools/` is deliberately not in `testpaths`.
- **No `insta`/`approx`/`rstest`.** Snapshot and float-comparison conventions are ad-hoc — state your tolerance explicitly in the test. `proptest` is the property-test library (dev-dep in `gcode`, `host-rt`, `nurbs`, `runtime`).
- **MCU boundary integration is NOT in CI.** The `sim` job excludes `needs_renode`/`needs_hardware`. Renode and bench flashing (Neptune/Trident) are manual — use the `renode-simulation`/`neptune-bench`/`trident-bench` skills.

#### Performance gotchas
- **No automated throughput-regression gate exists yet — the planner pipeline is not yet end-to-end.** This is a known gap, not a regression; add throughput verification when the pipeline matures. Until then, treat any planner change as needing manual throughput spot-checks.
- **Release profile is tuned for the Pi host** — `opt-level=3`, `lto="fat"`, `codegen-units=1`, `panic="abort"`, `debug=true`. Build-time crates (`syn`, `cbindgen`) are forced to `opt-level=0` to avoid OOM on low-memory hosts. Do not reorder these without a Pi build check.
- **`host_cargo` in `ci.sh`** uses `RUSTFLAGS="-Clink-arg=-fuse-ld=lld"` on Linux only — widening this past the host target would drop the macOS cdylib `-undefined dynamic_lookup` and the cross-build target-cpu/--nmagic flags. Do not widen `RUSTFLAGS` in CI without checking both macOS and MCU builds.

---

## Usage Guidelines

**For AI Agents:**
- Read this file before implementing any code in this project.
- Follow ALL rules exactly as documented. When in doubt, prefer the more restrictive option.
- The Critical Don't-Miss Rules section is a quick-reference for the most common mistakes — but the canonical rules live in the earlier sections; when a "do not" bullet and a detailed rule conflict, the detailed rule wins.
**For Humans:**
- Keep this file lean and focused on agent needs — remove rules that become obvious over time.
- Update when the technology stack, CI jobs, or conventions change.
- Review quarterly for outdated rules (the codebase is a rewrite in progress; conventions will shift).
- The throughput-regression gate (referenced in Testing Rules and Critical Don't-Miss Rules) is owed once the planner pipeline is end-to-end — add it then.

Last Updated: 2026-06-17
