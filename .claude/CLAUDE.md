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
run `./scripts/ci.sh py`. The `quick` gates run concurrently and report in a fixed order, so wall time is the slowest gate (cold worktree ~220 s, warm ~40 s on this box; the serial run was ~320 s / ~210 s); each gate's full log is still `.ci-logs/<gate>.log`. Individual jobs: `./scripts/ci.sh <job>` (see
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
task's builds and the engine `make` copies. `scripts/ci.sh` unsets an inherited
value (set `CI_KEEP_CARGO_TARGET_DIR=1` to opt in); gates use the worktree's own
`rust/target`.

Never share one target dir between worktrees. Cargo's unit hash omits the workspace
path and its freshness test compares source mtimes with the artifact's, so a worktree
whose sources are older than another's build links that build's crates (reproduced:
an example using a constant only the other worktree's `geometry` defined compiled and
ran; bound-ext hit it in `ci.sh`). The tools that build for review and bench give every
worktree its own dir, `pending_baselines.target_dir_for(tree)` under
`~/.cache/pending-baselines/targets/` (`pending_baselines.py`, `integrate_stack.py`,
`bench_compare.py`, `accept_reviewed.py`, `land_queue.py`, the audit in
`review_baselines.py`). A test must still find repo files through the `source-tree`
crate (`source_tree::repo_file("snapshots/...")`), never `env!("CARGO_MANIFEST_DIR")`.
For a one-off variant build set `CARGO_TARGET_DIR=...` on that single command, not in
the shell. `uv run python scripts/gc_caches.py [--days N] [--apply]` lists and removes
stale `~/.cache/*-target` dirs, per-worktree targets and abandoned pending-baselines
worktrees/stacks.

# Engine provenance

`_motion_engine.build_id()` is the sha256 (first 40 hex) over every file under `rust/`
that git tracks or would add (path, length, content, sorted), read from the working
tree at build time by `rust/motion-engine/build.rs`, so dirty and new files count and
`git rev-parse HEAD` is not involved. `snapshots/engine_build_id.py` computes the same
value from a tree. `harness._import_engine()` (snapshots/run.py, tools/bench/analyze.py,
moves.py, tools/corner_flow.py, the review/pending/accept tools, live_replay) refuses an
engine whose id differs from the tree its `.so` sits in, or that has none, with
"rebuild with make -f Makefile.rust motion-engine-fast". klippy logs it at startup as the
`motion`/`engine_build` event; `tools/bench/deploy.py` records `rust_build_id` in the deploy
identity. An engine built outside a git tree reports `unknown` and is refused by the tools.

# Heavy-job slots

The box is shared by many agents; stacked builds and test runs pushed load to 25-60, which
timed out nextest and distorted benchmarks. Heavy commands queue for one of
`SERVAL_SLOTS` (default 3) machine-wide slots via `scripts/slots.py` (flock files under
`~/.cache/serval-slots`, freed by the kernel when the holder dies): every `ci.sh` job
except in CI takes one for its whole run (`CI_NO_SLOT=1` opts out), the `make`/`cargo` builds in
`pending_baselines`, the fit audit and the fuzz replay take one, and `bench_compare.py`
builds in a normal slot but measures in an exclusive one (all slots, nothing else heavy
runs). Wrap your own loops: `python3 scripts/slots.py run [--exclusive] -- <cmd>`;
`slots.py status` shows who holds the slots. Waiting is queued, prints the holders every
60 s and fails after 2 h. Waiters are served first come, first served (a pid queue), so a
chain of exclusive jobs cannot starve the others (it did, with the first version: a series
of exclusive bench runs held every slot for 25+ minutes). An exclusive job still blocks all
other heavy work while it runs: keep each one short and do not hold it across a session.
Nested calls under a held slot do not take a second one.

# Starting and shipping a change

Start in a warm worktree: `scripts/wt.sh new <topic>` prints the path of an idle pool
worktree (`~/Developer/serval-pool-N`, detached and clean) switched to a new branch cut
from `origin/main`, creating one only when none is idle. A fresh worktree's first
`ci.sh quick` is ~250-310 s of cold compile; a reused pool worktree with a one-crate
edit took 131 s under load 60 (cold 310 s on the same box), and ~40 s when nothing
changed. `land-pr.sh` returns a pool worktree to idle (detached at `origin/main`, its
branch deleted) instead of removing it. Other worktree names are removed after landing
as before.

Ship with one command from the branch's worktree: `python3 scripts/ship.py --title T
--body-file F [--expect SPEC --accept-msg FILE]`. It refuses a dirty tree, merges
`origin/main` (naming conflicted files), runs `cargo fmt` and `ruff format` (committing
the result), `ci.sh quick` (+ `ci.sh py` when klippy/, test/ or Python under tools/ or
scripts/ changed), then `land_queue.py --branch`: snapshot review against `origin/main`
(nothing may move without `--expect`; with it every case must be EXPECTED and is accepted
from the message file), then `land-pr.sh`. Do not hand-roll fetch/merge/fmt/clippy/
review/accept/land sequences; they were ~10 calls per landing across 64 landings.

Edit tool failures are 23% of all edit calls in the audited sessions (164 of 701), each
costing a re-read and a retry: the causes are anchoring to lines the last read did not
display (a partial range, a grep hit) and a stale file tag after another edit. Read the
exact range you edit with the same call that gave you the tag, send all changes to a file
in one `edit`, and take the tag from the previous edit's response. Do not fall back to
`python3 - <<EOF` replace scripts (261 in those sessions): they skip the anchors that make
edits reviewable and silently match more than you meant.
# Hardware lock

The Pi bench is shared: every use goes through `tools/bench/lock.py hold -- <cmd>`, which
queues behind the current holder and prints who holds it and for how long (waits of
20 minutes were seen). Hold it for the whole session script, not per command.

# Disk space

/home filled to 22 MB free and rustc ICE'd for every agent. The sources were: ~495 scratch
cargo targets in `pending-baselines/targets` (one per commit, 277 GB), ad-hoc `~/.cache/*-target`
dirs (~280 GB), 117 docker images of 3.6 GB / 0.65 GB (one per branch, ~400 GB with build cache),
and 20-60 GB `rust/target` in every worktree. Now: scratch worktrees for samples, audits and fuzz
replays are single fixed trees (`wt/scratch`, `wt-audit/scratch`, `fuzz-triage/main`) checked out
to the commit needed, so there is one warm target each; local docker images are tagged per
worktree directory (`klipper-build-<dir>`, `kalico-sim-<dir>`), bounded by the pool, and `land-pr.sh`
removes a non-pool worktree's images with it. `scripts/gc_caches.py` also drops repo docker images
older than 12 h that no container uses, build cache older than a week, scratch targets beyond 24,
and with `--worktree-targets` the `rust/target` of idle (detached, clean) or merged-branch
worktrees not used by any process; it never touches worktree sources. `wt.sh new` and `ship.py`
run it automatically (`--if-free-below 100`), and `ci.sh` refuses to start below 10 GB free
(`CI_MIN_FREE_GB`). `~/.cache/huggingface` and `go-build` are the user's: leave them.

# Never use git stash

The stash is one list shared by every worktree of the repository: one agent's `git stash
pop` took another agent's work in progress. Commit WIP on your branch (a `WIP` commit you
amend away before shipping is fine; history is never rewritten on main), or save it with
`git diff > ~/.cache/<topic>.patch` and `git apply` it later. `git stash`, `merge --autostash`
and `pull --autostash` are refused by `.githooks/reference-transaction`, installed into the
shared hooks directory by `scripts/install-git-guards.sh` (run by `wt.sh new`; run it once by
hand if you do not use wt.sh). A git alias cannot do this: git does not let an alias shadow
a builtin command (checked: `alias.stash` is ignored), but every ref update passes through the
hook, and it works in all worktrees at once. Deleting an existing stash entry is still
allowed so old ones can be dropped.

# Never work in ~/Developer/serval

`~/Developer/serval` is the shared checkout: it stays on `main` tracking `origin/main`
and `land-pr.sh` fast-forwards it. Do not edit, commit, build or switch branches
there; create a worktree (`git worktree add ~/Developer/serval-<topic> -b <branch>
origin/main`) and work in that. Run the tools from your worktree, or from
`~/Developer/serval/scripts/` when you have none: they are current there.

# Landing a PR

`scripts/land-pr.sh <branch>` pushes the branch, opens the PR if needed, waits for
the checks, merges with a merge commit and moves local `main` to `origin/main`
without switching any checkout (a checked-out `main` is fast-forwarded only when it
has no uncommitted changes, so `~/Developer/serval` stays current after every landing). Red checks print the failed-run log tail and
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

`land-pr.sh` and `land_queue.py` re-run themselves from `origin/main` after fetching
(`land-pr.sh` copies `scripts/land-pr.sh` and `gh_util.py` into a temp dir; `land_queue.py`
extracts `scripts/` from `git archive origin/main`; `LAND_PR_REEXEC` / `LAND_QUEUE_REEXEC`
stop the loop, `SERVAL_REPO_ROOT` keeps the repo root), so a branch carrying an older copy
still lands with the current logic (including fast-forwarding `~/Developer/serval`). They
fail loudly when the fetch fails.

Every `gh` call in the tooling goes through `scripts/gh_util.py` (`pending_baselines.run`
routes `gh` there; `land-pr.sh` defines `gh()` as the wrapper). Many agents polling
GitHub trip its secondary rate limit, and a bare `gh pr checks --watch` then exits
non-zero with "API rate limit exceeded", which looks like a red check. The wrapper
retries only on that error and on transient network failures (a dropped connection
made `--watch` exit like a red check on PR #652) (Retry-After if sent, else 30 s doubling to 300 s with 25 %
jitter, 8 attempts, then a loud failure) and returns any other failure at once.
`git fetch`/`push`/`pull`/`ls-remote` in land-pr.sh, ship.py, land_queue and everything routed through
`pending_baselines.run` go through `gh_util.py --git`: retried (5 attempts, 5 s doubling to 60 s,
jitter) only on server and network errors (Internal Server Error, HTTP 5xx, connection reset,
could not resolve host, RPC failed, early EOF); `Permission denied`, `Authentication failed` and
`non-fast-forward`/`rejected` fail at once, loudly, and local subcommands are never retried.
Wait with one `gh pr checks --watch --interval 60`, never a loop of short polls; read PR
metadata once per run and pass it along; use `gh_util.py <gh args>` in new scripts.

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
`gcode_replay_bench` at both commits (one reusable worktree with its own target dir,
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

# CI shape and delegating work

Workflows run on `ubuntu-24.04`, never `ubuntu-latest`: GitHub moved `latest` to `ubuntu-26.04`
(image 20261025.179, GCC 15) from ~17:50 on 2026-11-04, and every Rust job went red at once: GCC 15
rejects implicit declarations in the ci-igh stub, and rust-cache restored pyo3-build-config output
recorded on 24.04 (`-lpython3.12` link failures, fixed in ci.sh by the interpreter stamp). The label
moves on GitHub's schedule, not ours; move to a newer image in a PR that fixes what it breaks.


A PR waits for the slowest check. Measured over the last ~200 PR runs
(median job minutes): rust-fuzz 8.3 (three partitions now), sim-e2e `rest`
8.0 (`rest-slow` split off), rust-test 5.9, sim-e2e shards 4–5 (of which
~3.5 was each shard rebuilding the same docker image, now built once by the `sim-image` job and loaded by the shards: shard jobs 61-212 s instead of 240-480 s, total job time -47%, workflow wall ~7.7 min because the image build (240 s) is now serial before the slowest shard (`rest`, 212 s)), snapshot 3.8. Sim shards are balanced by per-file test time (probe 33 s of tests, print 37, rest-slow 52,
rest-pinned 70, beacon 85, rest ~80 after moving `test_beacon_stream_stall` and
`test_homing_min_dist_corexy` out) plus ~25 s to load the image. Building the engine in
the image is ~125 s of dependency compiles and ~50 s of workspace crates on a 4-core runner
(CI log of `Compiling` lines), but a cargo cache mount or deps-only layer would have to be
restored or exported through the gha cache on every run, which costs about what it saves;
not done. `./scripts/ci.sh quick` with a worktree's own target: ~285 s cold, ~185 s warm
on a loaded workstation (load 30-40), of which `rust-fit-audit` is 180 s (it replays a
whole dense print per test; its nextest timeout is 600 s so a loaded box does not kill it);
every other gate is under 30 s warm.
The `rust-fuzz` and `replay-budget` are
not required but `land-pr.sh` waits for them. Branch protection is strict, so
every time `main` moves before a merge the PR branch is updated and the whole
suite reruns: land a batch with `scripts/land_queue.py` (one branch at a time,
merging `main` first) rather than starting several `land-pr.sh` in parallel.

`rust-fuzz` failures are triaged, not retried blindly. The job prints one
`FUZZ_FAILURE {json}` line per failing test (test, seed, panic location, message;
`scripts/fuzz_triage.py parse`). On a red run `land-pr.sh` calls `fuzz_triage.py triage`:
it requires every red job to be `rust-fuzz` with a seed, replays the seed against
`origin/main` in a scratch worktree under `~/.cache/pending-baselines/fuzz-triage/`
(`cc` seeds through the suite's `proptest-regressions` file, `PROPTEST_RNG_SEED` seeds
through the env). If main fails it too, the bug is pre-existing: an issue labelled
`fuzz-triage` is found or opened, deduplicated by `fuzz-triage-key: <binary>::<test>@<panic
location>` in its body, the seed is commented once, the failed job is rerun and landing
continues (at most 3 reruns). If main passes the seed, the PR broke it or is behind a fix
already on main: land-pr.sh stops with the repro command. Timeouts, other red jobs and
seedless failures stop as before. Every proptest suite must keep its failing seed
(`FileFailurePersistence::Direct("proptest-regressions/<file>.txt")` or a fixed `rng_seed`;
the `source-tree` test `proptest_seeds` enforces it), because proptest's default
persistence finds no crate root under nextest and silently drops the seed.

A red check is investigated, not rerun, unless it is a known flake with an open
issue; rerun those once and add the run id to the issue. Sim flakes so far were harness races between wall-clock threads and the free-running virtual clock, or reads of klippy's async log: look there first (`tools/sim/world.py` `settled_log_tail`, `HarnessPacer.hold_until`). `rust-fuzz` failures on a fresh
seed are real counterexamples: commit the `cc` seed line to
`proptest-regressions/` and file an issue.

When delegating: give the worker the worktree path to use, say that gates use that worktree's own `rust/target`, and have it end
with a named list of checks for the lead to run once, since parallel
`ci.sh quick` runs from several workers contend for the same CPUs. Do not have
a worker re-derive repo context that is in this file or a skill; point to it.
