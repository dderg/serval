---
title: 'Drop SOEM — make IgH the only EtherCAT master backend'
type: 'refactor'
created: '2026-06-26'
status: 'done'
baseline_commit: 'c74e174cc9aa27cece3b9595c381a875e1ecb301'
context: ['{project-root}/CLAUDE.md', '{project-root}/_bmad-output/project-context.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The EtherCAT endpoint carries two interchangeable C master backends — SOEM (`csrc/libecrt.c`) and IgH (`csrc/libecrt_igh.c`) — behind a `hw`/`igh` Cargo feature fork. Both are now feature-complete over the same `ec_rt_*` FFI, but maintaining two backends means any future endpoint change (notably the deferred multi-drive N-slave rewrite) must be written twice. We want one backend.

**Approach:** Delete SOEM entirely and make IgH the sole hardware backend. Collapse the `hw`/`igh` feature fork so the single `hw` feature compiles `libecrt_igh.c` + links `libethercat`. Default the neptune-bench flash script to igh and make it refuse `soem`. Purge stale SOEM references and the long-stale "IgH is a skeleton / EC_RT_ERR_IGH_UNIMPLEMENTED" comments from build files, docs, and project-context.

## Boundaries & Constraints

**Always:**
- Keep the `ec_rt_*` FFI surface (`ffi.rs`, `libecrt.h`, `EcTelemetry`/`ec_telemetry_t` layout) byte-for-byte unchanged — IgH already satisfies it.
- The default-feature build (no `hw`) must stay pure-Rust and CI-green; the `ffi` module stays `#[cfg(feature = "hw")]`-gated.
- Behavior-preserving only: do not change any RT-priority constant, scheduling logic, DC-loop timing, or PDO layout.

**Ask First:**
- Bench verification that the single A6-EC drive still homes and moves on the IgH-only build (manual, on the Pi — not CI). The user runs this; do not flash or issue gcode.

**Never:**
- Do NOT touch multi-drive / N-slave work — `NUM_AXES`, `SLAVE_POS`, per-slave PDO state all stay single-drive (that is the deferred Goal B).
- Do NOT keep a SOEM fallback, compat shim, or dead `build_soem`/`igh`-feature code "just in case." Remove it.
- Do NOT rename the `hw` Cargo feature or the `ethercat-endpoint-hw` Makefile target (minimize churn; `hw` = "real hardware master", now IgH).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Build hw endpoint | `make -f Makefile.rust ethercat-endpoint-hw` on the Pi | Compiles `libecrt_igh.c`, links `libethercat`, produces `ethercat-rt` binary | Missing `/opt/etherlab` → cc/link error, loud |
| Default CI build | `cargo build`/`nextest` (no `hw`) | Pure-Rust, no C compiled, green | N/A |
| Flash default backend | `flash-neptune.sh <ref>` (no 2nd arg) | Builds IgH endpoint, starts IgH kernel master | N/A |
| Flash refuses SOEM | `flash-neptune.sh <ref> soem` (or `hw`) | Exits non-zero with "SOEM removed; IgH is the only backend" | exit 2, no flash performed |

</frozen-after-approval>

## Code Map

- `rust/ethercat-rt/csrc/libecrt.c` -- SOEM backend; **delete**.
- `rust/ethercat-rt/csrc/libecrt_igh.c` -- IgH backend; sole survivor. Strip SOEM-comparison wording in comments (wording only).
- `rust/ethercat-rt/csrc/libecrt.h` -- shared FFI header; keep unchanged.
- `rust/ethercat-rt/build.rs` -- delete `build_soem()` + `SOEM_DIR`/`SOEM_LIB_DIR` env wiring; under `hw` call `build_igh()` directly (drop the `CARGO_FEATURE_IGH` branch).
- `rust/ethercat-rt/Cargo.toml` -- remove `igh = ["hw"]`; rewrite the feature doc comment.
- `Makefile.rust` -- make `ethercat-endpoint-hw` build IgH; delete `ethercat-endpoint-igh` target + its `.PHONY` entry; fix comments.
- `~/.claude/skills/neptune-bench/scripts/flash-neptune.sh` -- default `igh`, refuse `soem`/`hw`, drop the dead SOEM restore branch, fix stale comments.
- `~/.claude/skills/neptune-bench/SKILL.md` -- update backend section to IgH-only.
- `rust/ethercat-rt/src/{mailbox.rs,thread_prio.rs,bin/ethercat-rt.rs}` -- doc-comments naming SOEM's raw socket; reword to the IgH/master reality (no logic change).
- `docs/rewrite/ethercat-bench-bringup.md` -- SOEM build/install/behavior text → IgH.
- `_bmad-output/project-context.md` -- EtherCAT endpoint bullet (`--features hw builds against SOEM`) → IgH.

## Tasks & Acceptance

**Execution:**
- [x] `rust/ethercat-rt/build.rs` -- removed `build_soem` and SOEM env vars; `hw` compiles `libecrt_igh.c` + links `ethercat`/`pthread`/`rt`/`m` -- single backend.
- [x] `rust/ethercat-rt/Cargo.toml` -- dropped `igh` feature; doc comment + build-dep comment reflect `hw` = IgH only.
- [x] `rust/ethercat-rt/csrc/libecrt.c` -- deleted (`git rm`) -- SOEM gone. Also removed dead `EC_RT_ERR_IGH_UNIMPLEMENTED` from `libecrt.h`.
- [x] `Makefile.rust` -- one endpoint target (`ethercat-endpoint-hw`, IgH); removed `ethercat-endpoint-igh` + `.PHONY` entry; comments drop SOEM + the "skeleton/UNIMPLEMENTED" claim.
- [x] `flash-neptune.sh` -- `BACKEND=${2:-igh}`; `igh)` → `ethercat-endpoint-hw`; `soem|hw)` → loud refusal; removed dead SOEM restore branch; fixed header/inline comments.
- [x] `neptune-bench SKILL.md` -- backend section + usage line → IgH-only, soem refused.
- [x] `rust/ethercat-rt/src/{mailbox.rs,thread_prio.rs,bin/ethercat-rt.rs}` + `libecrt_igh.c` comments -- reworded SOEM references; no logic change.
- [x] `docs/rewrite/{ethercat-bench-bringup.md,motion-node-unification.md}`, `_bmad-output/project-context.md` -- SOEM → IgH.

**Acceptance Criteria:**
- Given a clean checkout, when `rg -i soem` over the living surface (`rust/`, `Makefile.rust`, `docs/rewrite/`, neptune-bench skill — excluding `target/`), then the only matches are the intentional "SOEM was removed" refusal text in the bench skill/flash script. Dated `docs/superpowers/**` design archives are historical records and are left untouched.
- Given the default feature set, when `./scripts/ci.sh quick`, then green (rust-test, clippy `-D warnings`, fmt, ruff, watchdog-canary).
- Given the Pi with `/opt/etherlab`, when `make -f Makefile.rust ethercat-endpoint-hw`, then it compiles and links `libethercat` and emits the `ethercat-rt` binary.
- Given `flash-neptune.sh <ref> soem`, when run, then it exits non-zero before any build/flash with a message that SOEM was removed.
- Given `flash-neptune.sh <ref>` with no backend arg, when run, then it builds the IgH endpoint and starts the IgH kernel master (default = igh).

## Verification

**Commands:**
- `cd rust && cargo nextest run -p ethercat-rt` -- expected: all pass (default features, no C).
- `./scripts/ci.sh quick` -- expected: green.
- `rg -i 'soem|EC_RT_ERR_IGH_UNIMPLEMENTED' rust/ Makefile.rust docs/rewrite/ -g '!target'` -- expected: no matches (the bench skill keeps the intentional refusal text; `docs/superpowers/**` archives untouched).
- `bash -n ~/.claude/skills/neptune-bench/scripts/flash-neptune.sh` -- expected: syntax OK.

**Manual checks (Pi/bench, user-run — Ask First):**
- `make -f Makefile.rust ethercat-endpoint-hw` on the Pi links against `libethercat` and produces the binary.
- After `flash-neptune.sh <ref>` (default igh): the single A6-EC X axis homes and executes a move (single-drive regression, unchanged from prior IgH behavior).

## Suggested Review Order

**The single-backend decision (entry point)**

- The fork collapses here: `hw` now unconditionally builds IgH; SOEM branch + env vars gone.
  [`build.rs:18`](../../rust/ethercat-rt/build.rs#L18)

- Feature surface shrinks to one: `igh` feature removed, only `hw` remains.
  [`Cargo.toml:16`](../../rust/ethercat-rt/Cargo.toml#L16)

**Backend code & ABI (highest risk)**

- SOEM source deleted (622 lines); confirm nothing referenced it.
  [`libecrt.c (deleted)`](../../rust/ethercat-rt/csrc/libecrt.c)

- Dead skeleton error code removed from the shared header; FFI signatures untouched.
  [`libecrt.h:19`](../../rust/ethercat-rt/csrc/libecrt.h#L19)

- IgH header reframed as the sole backend (comment-only); `SLAVE_POS` value unchanged.
  [`libecrt_igh.c:4`](../../rust/ethercat-rt/csrc/libecrt_igh.c#L4)

**Build & bench tooling**

- One endpoint target (IgH); `ethercat-endpoint-igh` removed.
  [`Makefile.rust:35`](../../Makefile.rust#L35)

- Flash script defaults to igh and loudly refuses soem (out-of-repo): `~/.claude/skills/neptune-bench/scripts/flash-neptune.sh` case statement.

**Behavior-preserving comment cleanup (lowest risk)**

- RT-scheduling rationale generalized from "SOEM socket" to "the EtherCAT master"; no priority/logic change.
  [`thread_prio.rs:10`](../../rust/ethercat-rt/src/thread_prio.rs#L10)
