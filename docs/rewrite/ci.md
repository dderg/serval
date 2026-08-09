# CI & local testing

CI must be **trustworthy**: a green check means the gate really ran, and a red
check means something is actually wrong. This page describes how that is kept
true, and how to reproduce CI locally.

## One source of truth: `scripts/ci.sh`

Every CI gate is defined once, in `scripts/ci.sh`. The GitHub workflows do not
contain inline test commands — each step calls `./scripts/ci.sh <job>`. The same
script is what you run locally, so **CI and local cannot drift** (the old
`scripts/ci-local.sh` was a hand-copied parallel definition that silently
disagreed with the workflows — that is exactly the failure mode this prevents).

```sh
./scripts/ci.sh            # run every gate, with a pass/fail summary
./scripts/ci.sh quick      # fast pre-push subset: ruff + rust test/clippy/fmt
./scripts/ci.sh rust-test  # one gate (CI runs rust-test / rust-clippy / rust-fmt as parallel jobs)
./scripts/ci.sh py 3.13    # klippy pytest under one Python version (needs docker)
./scripts/ci.sh sim        # sim unit tests (tools/sim, no ELF)
```

Jobs: `ruff rust-host rust-build rust-test rust-clippy rust-fmt rust-loom
rust-mcu-h7 rust-mcu-f4 cbindgen-drift c-smoke deny miri panic-grep
watchdog-canary py docs sim`.

One-time prerequisites for the full run:

```sh
rustup target add thumbv7em-none-eabi
rustup component add --toolchain nightly miri
cargo install cargo-nextest --locked   # process-per-test isolation
cargo install cargo-deny               # optional
```

## Output: one line per gate unless it fails

A gate's log is worth nothing when it passes and everything when it fails, so
`ci.sh` prints it accordingly. Interactive terminals and GitHub Actions get the
full live stream — a human wants progress, a CI job log wants the record.
Anywhere else (an agent, a script capturing stdout) each gate collapses to one
line carrying its tally, and a failing gate dumps the last 100 lines of its log:

```
ruff                 PASS  389 files already formatted
rust-test            PASS  Summary [ 3.789s] 2367 tests run: 2367 passed, 5 skipped
py                   PASS  751 passed, 5 skipped, 2 warnings in 32.98s
```

The same rule covers the docker builds behind `py`/`sim`/`sim-e2e` and the
workspace doc-tests, and `rust/.config/nextest.toml` sets `status-level =
"slow"` so a green Rust run is one summary line instead of 2367 PASS lines.
Nothing is discarded on the failure path.

### Getting the full output back

Quiet is the default, not a wall. Three ways through it:

```sh
./scripts/ci.sh -v py                     # stream the gate live
cat .ci-logs/py.log                       # complete log of the last `py` run
./scripts/ci.sh sim-e2e --keep-logs -k probe   # keep the simulator's own logs
```

Every job — quiet or streamed, passing or failing — writes its complete output
to `.ci-logs/<job>.log` (gitignored, overwritten per job per run). Prefer
reading that file over re-running a gate verbosely: the failure dump is capped
at 100 lines, the file is not.

`--keep-logs` is a `tools/sim/run.sh test` flag (reachable through `ci.sh
sim-e2e`). The sim's `--rm` container normally takes every world's logs with
it; the flag puts pytest's basetemp on a host mount, leaving the whole tree in
`.sim-logs/run/<test-name>0/world0/`:

```
logs/klippy.log            klippy's own log
logs/klippy.stdout         tracebacks that never reached the log
logs/h7.log, f4.log        MCU process stdout/stderr
logs/events/host-py.jsonl  structured event store (also host-rust, <mcu>.jsonl)
printer.cfg                the exact config the world booted
```

The `.jsonl` files are the same structured records `scripts/logq.py` queries on
a real host, minus the VictoriaLogs round trip — read them with `jq`.

## Checking locally before a PR

The single source of truth runs the same gates CI does. Before opening a PR to
merge a chunk of work, run the fast subset (or the full suite):

```sh
./scripts/ci.sh quick      # ruff + rust test/clippy/fmt
./scripts/ci.sh            # everything
```

### Optional pre-push hook

If you want that gate to run automatically on every `git push`, enable the
tracked hook:

```sh
./scripts/ci.sh install-hooks    # one-time; sets core.hooksPath = .githooks
```

It is **opt-in and off by default**, because it runs on *every* push and so adds
latency to tight loops — notably the trident deploy loop (commit → push → pull on
the Pi → compile → flash), where the host-side `quick` gate is irrelevant to the
firmware build. With it enabled, bypass a single push with `git push
--no-verify`; disable entirely with `git config --unset core.hooksPath`.

## Policy: no fake-green

Tests are not silenced to make CI pass. In particular:

- **No `#[ignore = "flaky"]` as a band-aid.** Flakiness from shared global state
  is fixed at the source (thread-local capture, clock injection) and the runner
  is `cargo nextest` (process-per-test). Genuinely deferred tests carry a
  *truthful* reason (e.g. "removed in PR #11", "vacuous until H723 corpus
  collected") — never a fabricated "flaky" label.
- **No silent skips.** A test that prints "fixture not found, skipping" and
  passes is a no-op pretending to be coverage. Fixtures are committed, or
  generated by the gate, or the test fails honestly.
- **Excluded test classes are explicit.** Simulator tests are tagged with
  pytest markers (`sim_unit`, `needs_elf`, `needs_hardware`);
  CI runs `sim_unit` and excludes ELF/hardware *by marker on the command
  line*, so the exclusion is visible, not hidden in `testpaths`.

## Where CI runs

CI runs on **pull requests** — that is the gate you rely on when merging a chunk
of work. `main` is committed to **directly** during
development and bench iteration; those direct pushes are intentionally *not*
CI-gated, so they stay fast. The contract is: **red on a PR means a real
problem.** When you want assurance before a direct commit, run `./scripts/ci.sh`
locally (above).

Branch protection is **not** used and not needed for a solo direct-commit
workflow. If collaborators are ever added, the optional
[`scripts/setup-branch-protection.sh`](../../scripts/setup-branch-protection.sh)
can require these checks on others' PRs — it sets `enforce_admins: false`, so it
never blocks direct maintainer pushes.
