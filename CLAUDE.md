We are working on a complete rewrite of the motion planner and more:

# Non-negotiable constraints

- **Print throughput is non-negotiable.** The planner never knowingly chooses a cheaper algorithmic architecture that produces a measurably slower trajectory than the best one we can compute on the active hardware. "Best we can compute" is realistic — finite discretization N, local-optimum convergence (SLP for the non-convex jerk relaxation; the Consolini-Locatelli SOCP itself is convex but not a closed-form), tolerance settings tuned to the hardware budget. Within those engineering realities, the planner aims for the tightest trajectory it can; we do not give up trajectory time to make planning easier. Host compute is something we spend in service of trajectory optimality — not the other way around. If the Pi can't keep up, the answer is to optimize the implementation, parallelize across cores, or upgrade the host; the answer is never to ship a cheaper algorithm that produces a measurably slower trajectory on representative slicer output. State-of-the-art is the target, not safe-and-good-enough.

- Fail loudly. When adding checks for unexpected things to the code, instead of trying
  to recover, unless it was discussed and agreed on explicitly, the default solution is
  to fail loudly with a clear error code. This helps us catch bugs quicker. Example: movement segment arrives to the planner late, causing the start time to be in the past. Do not advance or pad the
  start time, raise an error instead. this way we notice the issue and have a chance to address it

- Comments are a failure of expression. Instead of writing one, make the code say it:
  rename, extract, assert, or compute the value. If you need a comment it means you need to make the code better. 
  TODO-style markers are fine. If you notice some useless pre-existing comments in the file you are editing - remove them.

- Unit tests live in a separate file from the tested code.

# Testing

Run the Rust suite with `cargo nextest run` from `rust/`, not `cargo test`.
`cargo test` executes the ~110 test binaries one at a time (each only
parallelizes internally), which leaves most cores idle — the full suite takes
~100s. `nextest` schedules every test into one global pool: same suite, ~11s.
Use `cargo nextest run -p <crate>` or `-E 'test(<name>)'` to scope down.
Doc-tests are the one gap — `nextest` skips them, so run `cargo test --doc`
when you touch doc examples.

# Before opening or updating a PR

Run `./scripts/ci.sh quick` and get it fully green — it bundles ruff
(check + format) over the whole repo, the Rust workspace tests, clippy
with `-D warnings`, `cargo fmt --check`, and the watchdog canary. This is
the same set CI runs first, so a red gate here is a red PR. `quick` does
NOT include the Python host tests — if the change touches `klippy/`, also
run `./scripts/ci.sh py`. Individual jobs: `./scripts/ci.sh <job>` (see
the header of `scripts/ci.sh` for the full list, e.g. `ruff`,
`rust-clippy`, `rust-mcu-h7`).

# Observability / structured logging

Log via the structured pipeline (`event_log_emit` → `events/*.jsonl`), not
`printf`/`output()` — it replaces `klippy.log` for MCU/structured diagnostics;
the wire-stable event table is `rust/runtime/src/log_codes.rs`. To read or add
logs — `DIAG_DUMP`, crash forensics, filtering — use the `mcu-diagnostics`
and `query-logs` skills.

# Reference docs

- **MCU C/Rust boundary — architectural invariant:** [`docs/rewrite/mcu-c-rust-boundary.md`](docs/rewrite/mcu-c-rust-boundary.md). Read this before adding shared state between C and Rust on the MCU, or before reaching for `#[link_section]` on a Rust static. Rules: C owns boot, safety-critical paths, and all shared-memory placement; Rust owns the motion engine; the seam is `extern "C"` + `#[repr(C)]` only.
