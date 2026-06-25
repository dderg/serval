---
title: 'EtherCAT backend toggle: SOEM ⇄ IgH'
type: 'feature'
created: '2026-06-25'
status: 'done'
baseline_commit: '1fa573a69b61fb921b13d2707c096772a07bd574'
context: ['{project-root}/rust/ethercat-rt/csrc/libecrt.h']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The hardware EtherCAT endpoint is hard-wired to SOEM (`csrc/libecrt.c`, `-lsoem`). We are migrating to the IgH (EtherLab) master eventually, but during development we need to flip between the known-good SOEM backend and the work-in-progress IgH backend by a rebuild, without touching the Rust `ec_rt_*` FFI surface.

**Approach:** Keep the `extern "C"` seam in `src/ffi.rs` backend-agnostic. Add a build-time Cargo feature `igh` (which implies `hw`) that makes `build.rs` compile a new IgH C implementation and link `libethercat` instead of `libecrt.c`+SOEM. Ship `csrc/libecrt_igh.c` as a compiling-and-linking **skeleton** that defines every `ec_rt_*` symbol against IgH's `<ecrt.h>` but fails loudly as unimplemented — the real IgH port is developed incrementally behind this toggle later.

## Boundaries & Constraints

**Always:**
- `src/ffi.rs` stays unchanged — both backends satisfy the identical `ec_rt_*` symbol set. The `libecrt.h` contract header is shared and master-agnostic (only `stdint.h`).
- `igh = ["hw"]`: `--features hw` → SOEM (unchanged default), `--features igh` (or `hw,igh`) → IgH. `igh` takes precedence when both are present.
- IgH skeleton fails loudly: bring-up returns a new `EC_RT_ERR_IGH_UNIMPLEMENTED` so the endpoint refuses to run rather than silently driving nothing.
- Mirror the SOEM env-var pattern: `IGH_DIR` (default `/opt/etherlab`, headers under `include/`) and `IGH_LIB_DIR` (default `$IGH_DIR/lib`), link `-lethercat`.

**Ask First:**
- Any change to `src/ffi.rs`, the `libecrt.h` signatures, or `EcTelemetry` layout — these are the shared contract.
- Writing actual IgH master logic (domains/PDO/DC bring-up) beyond the failing skeleton.

**Never:**
- Do not implement the full IgH master in this task — skeleton only.
- Do not make the backend a runtime switch (the two masters link different libraries — compile-time only).
- Do not build SOEM or IgH in CI; the `ethercat-rt-stub` (no features) remains the CI-able path.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| SOEM build | `--features hw` | `build.rs` compiles `libecrt.c`, links SOEM (unchanged) | SOEM missing → existing `-lsoem` link error |
| IgH build | `--features igh`, IgH installed | `build.rs` compiles `libecrt_igh.c`, links `-lethercat` | IgH missing → `<ecrt.h>` / `-lethercat` link error (loud, expected) |
| IgH run | IgH endpoint started | `ec_rt_bringup_preop` returns `EC_RT_ERR_IGH_UNIMPLEMENTED` | endpoint exits loudly, no torque |
| No features | `cargo build`/`nextest` | pure-Rust unit tests, no C compiled | N/A |

</frozen-after-approval>

## Code Map

- `rust/ethercat-rt/Cargo.toml` -- add `igh = ["hw"]` feature.
- `rust/ethercat-rt/build.rs` -- branch on `CARGO_FEATURE_IGH`: IgH C file + `IGH_DIR`/`IGH_LIB_DIR` + `-lethercat`, else existing SOEM path.
- `rust/ethercat-rt/csrc/libecrt.h` -- add `EC_RT_ERR_IGH_UNIMPLEMENTED (-17)`; shared contract, no master headers.
- `rust/ethercat-rt/csrc/libecrt_igh.c` -- NEW. Includes `libecrt.h` + IgH `<ecrt.h>`; defines all 24 `ec_rt_*` FFI symbols (signatures from `libecrt.h`) as loud-unimplemented stubs.
- `rust/ethercat-rt/csrc/libecrt.c` -- SOEM impl, reference for the symbol set; untouched.
- `rust/ethercat-rt/src/ffi.rs` -- backend-agnostic FFI surface; untouched.
- `Makefile.rust` -- add `ethercat-endpoint-igh` target (`--features igh`) beside `ethercat-endpoint-hw`.

## Tasks & Acceptance

**Execution:**
- [x] `rust/ethercat-rt/csrc/libecrt.h` -- add `EC_RT_ERR_IGH_UNIMPLEMENTED (-17)` to the error table -- gives the skeleton a loud, distinct failure code.
- [x] `rust/ethercat-rt/csrc/libecrt_igh.c` -- new IgH backend skeleton: `#include "libecrt.h"` + `#include <ecrt.h>`, reference a `libethercat` symbol so linking is genuinely exercised, define every `ec_rt_*` function (match `libecrt.h`); int-returning bring-up/cycle/SDO funcs return `EC_RT_ERR_IGH_UNIMPLEMENTED`, getters return 0, voids no-op. Mark bodies `// TODO: IgH port`.
- [x] `rust/ethercat-rt/build.rs` -- if `CARGO_FEATURE_HW` unset return; else if `CARGO_FEATURE_IGH` set compile `libecrt_igh.c` with `IGH_DIR`/`IGH_LIB_DIR` includes and link `ethercat`+`pthread`+`rt`+`m`; else existing SOEM branch. Add `rerun-if-env-changed=IGH_DIR`/`IGH_LIB_DIR` and `rerun-if-changed=csrc/libecrt_igh.c`.
- [x] `rust/ethercat-rt/Cargo.toml` -- add `igh = ["hw"]` under `[features]`.
- [x] `Makefile.rust` -- add `ethercat-endpoint-igh` (`cargo build -p ethercat-rt --features igh --bin ethercat-rt --release`); add it to the phony/aggregate list. `setcap-ethercat` already covers the produced binary.

**Acceptance Criteria:**
- Given no features, when `cargo nextest run -p ethercat-rt`, then it builds and passes unchanged (no C compiled).
- Given `--features hw`, when `build.rs` runs, then it still compiles `libecrt.c` and links SOEM — the SOEM path is byte-for-byte behavior-unchanged.
- Given `--features igh` with IgH installed, when `cargo build -p ethercat-rt --features igh --bin ethercat-rt`, then it compiles `libecrt_igh.c`, links `-lethercat`, and the binary links cleanly (all 24 symbols defined).
- Given the IgH endpoint runs, when bring-up is attempted, then `ec_rt_bringup_preop` returns `EC_RT_ERR_IGH_UNIMPLEMENTED` and the endpoint exits without enabling torque.

## Design Notes

The toggle has no invalid state, so no `compile_error!` is needed: `hw` alone = SOEM, `hw+igh` = IgH (igh wins). `igh = ["hw"]` lets `--features igh` satisfy the `required-features = ["hw"]` on the `ethercat-rt` bin, so that line is untouched.

The skeleton's job is to prove the build/link wiring end-to-end, not to drive hardware. Referencing one `libethercat` symbol (e.g. `ecrt_version_magic()`) inside an otherwise-stubbed `ec_rt_bringup_preop` guarantees the linker actually pulls `-lethercat` rather than dropping an unreferenced lib, surfacing a missing/mis-pathed IgH install at build time.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p ethercat-rt` -- expected: pass, no C build (default-feature path unaffected).
- `cd rust && cargo build -p ethercat-rt --bin ethercat-rt-stub` -- expected: stub still builds (CI-able path intact).
- `./scripts/ci.sh quick` -- expected: green (ruff/rust-test/clippy/fmt/watchdog).

**Manual checks (bench, outside CI — IgH/SOEM never built in CI):**
- On the Pi with IgH installed: `make -f Makefile.rust ethercat-endpoint-igh` links cleanly; running the endpoint exits with `EC_RT_ERR_IGH_UNIMPLEMENTED`.
- `make -f Makefile.rust ethercat-endpoint-hw` (SOEM) still builds and behaves exactly as before.

## Suggested Review Order

**Backend selection (the toggle)**

- Entry point: the compile-time dispatch — `igh` (implies `hw`) wins, else SOEM.
  [`build.rs:21`](../../rust/ethercat-rt/build.rs#L21)
- The `igh = ["hw"]` feature that makes `--features igh` satisfy the bin's `required-features`.
  [`Cargo.toml:21`](../../rust/ethercat-rt/Cargo.toml#L21)
- New IgH build branch: `IGH_DIR`/`IGH_LIB_DIR`, compile skeleton, link `-lethercat`.
  [`build.rs:29`](../../rust/ethercat-rt/build.rs#L29)
- SOEM branch, extracted verbatim — confirm it is behavior-unchanged.
  [`build.rs:54`](../../rust/ethercat-rt/build.rs#L54)

**IgH backend skeleton (loud-unimplemented)**

- The fail-loudly helper: references `ecrt_version_magic()` to force the link, returns the new error.
  [`libecrt_igh.c:16`](../../rust/ethercat-rt/csrc/libecrt_igh.c#L16)
- Bring-up entry — refuses to run rather than silently driving nothing.
  [`libecrt_igh.c:22`](../../rust/ethercat-rt/csrc/libecrt_igh.c#L22)
- The distinct error code added to the shared, master-agnostic contract header.
  [`libecrt.h:20`](../../rust/ethercat-rt/csrc/libecrt.h#L20)

**Build entry point (peripheral)**

- New `ethercat-endpoint-igh` make target beside the SOEM one.
  [`Makefile.rust:42`](../../Makefile.rust#L42)
