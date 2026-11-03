# Non-negotiable constraints

- The goal of this repository is to expand the boundaries of ultra fast printing at good quality.

- Fail loudly. When adding checks for unexpected things to the code, instead of trying
  to recover, unless it was discussed and agreed on explicitly, the default solution is
  to fail loudly with a clear error code. This helps us catch bugs quicker. Example: movement segment arrives to the planner late, causing the start time to be in the past. Do not advance or pad the
  start time, raise an error instead. this way we notice the issue and have a chance to address it

- Comments are a failure of expression. Instead of writing one, make the code say it:
  rename, extract, assert, or compute the value. If you need a comment it means you need to make the code better. 
  TODO-style markers are fine. If you notice some useless pre-existing comments in the file you are editing - remove them.

- Unit tests live in a separate file from the tested code.

# Release assertions

`debug_assert!` does not exist in release builds, which are what the live path
and the snapshot cdylib run (no CI job builds release with `debug-assertions`;
tests run in the dev profile where they are on). An invariant whose violation
would produce wrong motion or NaN is therefore `assert!` in host crates, with
the offending values in the message. Keep `debug_assert!` only for expensive
self-checks. On a hot path keep the check but put the failure out of line (a
`#[cold] #[inline(never)]` panic function, as `geometry::vec3::normalize` does);
measure with `scripts/bench_compare.py` (its CPU row is steadier than wall time on a
busy box). Arithmetic overflow is checked in release for the host crates
(`rust/Cargo.toml` per-package `overflow-checks`; the MCU crates are untouched),
except `nurbs`, `geometry` and `trajectory`, whose loops cost +2..6% CPU with it on.

# Testing

Run the Rust suite with `cargo nextest run` from `rust/`, not `cargo test`.
`cargo test` executes the ~110 test binaries one at a time (each only
parallelizes internally), which leaves most cores idle — the full suite takes
~100s. `nextest` schedules every test into one global pool: same suite, ~11s.
Use `cargo nextest run -p <crate>` or `-E 'test(<name>)'` to scope down.
Doc-tests are the one gap — `nextest` skips them, so run `cargo test --doc`
when you touch doc examples.

Test output is quiet by design: `ci.sh <job>` prints one `PASS <tally>` line
off a terminal, and dumps the last 100 lines of the log when the job fails;
nextest prints failures and slow tests only. Never re-run a gate verbosely to
"see" a green result — the tally line is the result.

When you do need the detail, the complete log of every job is already on disk
at `.ci-logs/<job>.log` — read or grep that instead of re-running. `ci.sh -v
<job>` streams live if you really want it in the transcript. For a simulator
post-mortem, `tools/sim/run.sh test --keep-logs` (also via `ci.sh sim-e2e
--keep-logs`) leaves each world's `klippy.log`, MCU logs, `printer.cfg` and
`events/*.jsonl` under `.sim-logs/run/<test>/world0/`; without it the `--rm`
container deletes them.

# Before opening or updating a PR

Run `./scripts/ci.sh quick` and get it fully green — it bundles ruff
(check + format) over the whole repo, the Rust workspace tests, clippy
with `-D warnings`, `cargo fmt --check`, the public-API baseline diff
(`rust/api/*.txt`, refresh with `scripts/public-api.sh --update` when a
contract change is intended), and the watchdog canary. This is
the same set CI runs first, so a red gate here is a red PR. `quick` does
NOT include the Python host tests — if the change touches `klippy/`, also
run `./scripts/ci.sh py`. Individual jobs: `./scripts/ci.sh <job>` (see
the header of `scripts/ci.sh` for the full list, e.g. `ruff`,
`rust-clippy`, `rust-mcu-h7`).

# Observability / structured logging

Log via the structured pipeline (`event_log_emit` → `events/*.jsonl`), not
`printf`/`output()` — it replaces `klippy.log` for MCU/structured diagnostics;
the wire-stable event table is `rust/runtime-contract/src/log_codes.rs`. To read or add
logs — `DIAG_DUMP`, crash forensics, filtering — use the `mcu-diagnostics`
and `query-logs` skills.

# Build caches and CARGO_TARGET_DIR

Never `export CARGO_TARGET_DIR` in a persistent shell: it leaks into the next
task's builds and the engine `make` copies. `scripts/ci.sh` replaces an inherited
value (set `CI_KEEP_CARGO_TARGET_DIR=1` to opt in) with one target dir shared by
every worktree, `~/.cache/serval-target` (`SERVAL_TARGET_DIR` moves it,
`CI_OWN_TARGET=1` gives that run the worktree's own `rust/target`): a fresh
worktree's `./scripts/ci.sh quick` takes ~85 s against the warm shared dir and
~220 s with a cold own target, and no per-task `~/.cache/*-target` is needed for
gates. Concurrent cargo runs wait on the dir's lock only while compiling.
Everything else outside `ci.sh` uses the worktree's own `rust/target`; the other
shared target dir is
`~/.cache/pending-baselines/target`, which `pending_baselines.py`,
`review_baselines.py` and `integrate_stack.py` set per subprocess. For a one-off
variant build set `CARGO_TARGET_DIR=...` on that single command, not in the
shell. `uv run python scripts/gc_caches.py [--days N] [--apply]` lists and
removes stale `~/.cache/*-target` dirs and abandoned pending-baselines
worktrees/stacks/samples.

# Landing a PR

`scripts/land-pr.sh <branch>` pushes the branch, opens the PR if needed, waits for
the checks, merges with a merge commit and moves local `main` to `origin/main`
without checking anything out. Red checks print the failed-run log tail and
exit non-zero. `--dry-run` prints the plan. It is safe to run from the
branch's own worktree (it leaves it before removing it) and updates the PR
branch on GitHub and re-waits when `main` moved while the checks ran; no need to
copy it elsewhere first.

`python3 scripts/land_queue.py QUEUE [--trees DIR] [--dry-run]` lands several
branches in order (a stack, or a batch of independent PRs): per branch it merges
`origin/main` if behind, builds the snapshot engine, reviews against `origin/main`
(no expectation: any moved snapshot stops the queue; with an expectation every case
must be EXPECTED, and the changed cases are accepted and committed from the
`<branch>.accept.txt` message), then runs `land-pr.sh` with `<branch>.md` as the PR
body. A red "Sim e2e" run is rerun once, loudly; any other red check stops the queue.
QUEUE lines are `branch|title|expect`; landed branches are appended to `QUEUE.landed`
so a stopped queue resumes. Use it instead of a hand-written loop around `land-pr.sh`.

`uv run python scripts/pending_baselines.py [-v|--all]` lists every open PR with
its stack parent, mergeability, CI state, the snapshot cases whose output
changes (max |dp|/|dv|/|da| per axis against its base, plus the cases differing
from the baselines committed at its head) and a stack-aware merge order. Engines
are built in throwaway worktrees and cached by commit under
`~/.cache/pending-baselines`. `--review <pr>` checks that PR head out and
starts the snapshot review UI so the baselines can be accepted and pushed.

`uv run python scripts/integrate_stack.py 443 444 445 [--run CMD ...] [--keep]` (or
`--order-from-pending`) merges PR heads in that order onto `origin/main` in a
throwaway worktree (merge commits, never pushed), stops at the first conflict
naming the files and the pair, runs each `--run` command in the integrated tree
(cargo target dir shared with `pending_baselines.py`) and prints a table of
merge steps and command tallies (last two output lines; full logs under
`~/.cache/pending-baselines/stack-logs`). The worktree is removed unless `--keep`.
Run it with plain `python3`, not `uv run` (uv's python breaks the pyo3 test
binaries). Snapshot checks need the engine built first:
`--run "make -f Makefile.rust motion-engine-fast && uv run python snapshots/run.py"`.

`uv run python scripts/bench_compare.py <commitA> <commitB> [--configs fast,printer]
[--runs 5] [--gcode PATH]` is the replay-throughput A/B: it builds
`gcode_replay_bench` at both commits (one reusable worktree, shared target dir,
binaries cached by commit under `~/.cache/pending-baselines/bench`), runs them
interleaved with alternating order, and prints median wall seconds, realtime
factor and the per-stage breakdown with the change B vs A. Use it instead of
hand-rolled alternating runs; run it on an otherwise idle machine (the load
average is printed).

`scripts/corpus.sh [--configs fast,printer] [-k SUBSTR] [--audit]` plans the
59-case corpus (51 snapshot cases + the four `tools/sim/gcode` prints on
`fast.cfg` and `printer.cfg`) through the production pipeline and checks each
result is finite. To measure something across the corpus, write a probe test
starting with `mod corpus;` that iterates `corpus::corpus()` and run it with
`scripts/corpus.sh --probe FILE.rs`: the file is copied into
`rust/pipeline-snapshot/tests/` for the run and removed afterwards. The corpus
lives in `rust/pipeline-snapshot/tests/corpus/mod.rs`, shared with the fit audit.

`uv run python scripts/accept_reviewed.py <pr> [--accept-suspicious CASE=REASON ...]`
(from the PR branch's own worktree, pushed and clean) runs
`scripts/review_baselines.py --pr N`, refuses on any REGRESSION or on a SUSPICIOUS
case without a stated reason, writes the EXPECTED, reasoned and new baselines
through `snapshots/run.py --accept`, checks every snapshot is exact and commits
`snapshots/baselines` with the review summary and reasons in the body. Run the
snapshot tools with `uv run python` (they need numpy); a cache entry in which
every case failed identically is never kept.

# Reference docs

- **MCU C/Rust boundary — architectural invariant:** [`docs/rewrite/mcu-c-rust-boundary.md`](docs/rewrite/mcu-c-rust-boundary.md). Read this before adding shared state between C and Rust on the MCU, or before reaching for `#[link_section]` on a Rust static. Rules: C owns boot, safety-critical paths, and all shared-memory placement; Rust owns the motion engine; the seam is `extern "C"` + `#[repr(C)]` only.

- **Motion planner entry point:** `setup_pipeline` in `rust/motion-core/src/worker.rs` wires the streaming stages (fitter → planner → lowerer → shaper); the pipe's front door (ingress guard, pacing, control tokens) is `rust/motion-core/src/worker/ingress.rs` — read those first when exploring the planner.

# Git

Never rewrite git history, never amend commits, never force-push.

# Snapshots

Baseline files define how it is in the base branch. It doesn't mean baseline represents correct behavior. Snapshot tests exist so a reviewer can compare before/after to see what exactly changed in the motion planner behavior.

Baselines are regenerated by the agent that owns the change, only after reviewing the diff with `scripts/review_baselines.py`, and the review verdict (what changed, why it is expected) goes in the PR or commit body. Never accept a change you can't explain. Accept without the review server: `uv run python snapshots/run.py --accept CASE...` or `--accept-all` (writes the baselines of the named, or all, CHANGED/NEW cases; the review server path is unchanged).
