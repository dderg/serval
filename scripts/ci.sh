#!/usr/bin/env bash
# Usage:
#   ./scripts/ci.sh                 # run all gates with a summary (local)
#   ./scripts/ci.sh quick           # fast subset: ruff + rust test/clippy/fmt
#   ./scripts/ci.sh install-hooks   # enable the pre-push hook (runs `quick` per push)
#   ./scripts/ci.sh <job>           # run one gate, exit with its status (CI)
#   ./scripts/ci.sh -v <job>        # stream the gate's output live
#
# Output policy: a terminal, GitHub Actions and `-v` get the live stream.
# Everywhere else (agents, scripts capturing stdout) a gate prints one PASS
# line with its tally, or FAIL plus the last 100 lines of the log. Either way
# the complete log of every job stays in .ci-logs/<job>.log until the next run
# of that job overwrites it.
#
# Prerequisites (one-time, for the full local run):
#   rustup target add thumbv7em-none-eabi thumbv6m-none-eabi thumbv7m-none-eabi
#   rustup component add --toolchain nightly miri
#   cargo install cargo-nextest --locked        # or: curl -LsSf https://get.nexte.st/latest/<os> | tar zxf - -C ~/.cargo/bin
#   cargo install cargo-deny                     # optional
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUST="$ROOT/rust"
LOG_DIR="$ROOT/.ci-logs"
VERBOSE=""

# A docker build narrates every layer even when all of them are cached: ~400
# lines that say nothing on success and everything on failure. Streaming is for
# a human watching progress and for CI job logs; a captured stdout gets silence
# on success and the complete log on failure.
stream_output() { [ -n "$VERBOSE" ] || [ -t 2 ] || [ -n "${GITHUB_ACTIONS:-}" ]; }

run_quiet() {
    if stream_output; then
        "$@" >&2
        return
    fi
    local log rc=0
    log="$(mktemp)"
    "$@" >"$log" 2>&1 || rc=$?
    [ "$rc" -eq 0 ] || { printf '%s failed (rc=%s):\n' "$1" "$rc" >&2; cat "$log" >&2; }
    rm -f "$log"
    return "$rc"
}

# Built locally from the worktree's own scripts/Dockerfile-build, tagged per
# branch so concurrent worktrees never test against a stale or mismatched
# image (mirrors tools/sim/run.sh's kalico-sim-<branch> tagging). Docker's
# content-addressed layer cache makes repeat calls near-instant when nothing
# relevant changed.
docker_image() {
    local branch tag
    branch="$(cd "$ROOT" && git rev-parse --abbrev-ref HEAD 2>/dev/null || echo head)"
    # Detached HEAD (e.g. GitHub Actions' PR checkout) yields the literal
    # string "HEAD" rather than a branch name — fall back to the commit.
    [ "$branch" = "HEAD" ] && branch="$(cd "$ROOT" && git rev-parse --short HEAD 2>/dev/null || echo head)"
    tag="klipper-build-${branch//\//-}"
    tag="$(echo "$tag" | tr '[:upper:]' '[:lower:]')"
    run_quiet docker build -f "$ROOT/scripts/Dockerfile-build" -t "$tag" "$ROOT" || return 1
    echo "$tag"
}

# Linux-only: env RUSTFLAGS replaces (does not merge with) the per-target
# rustflags in rust/.cargo/config.toml, so widening this past the host target
# would drop the macOS cdylib `-undefined dynamic_lookup` and the cross-build
# target-cpu/--nmagic flags.
host_cargo() {
    if [ "$(uname -s)" = Linux ] && command -v ld.lld >/dev/null 2>&1; then
        RUSTFLAGS="-Clink-arg=-fuse-ld=lld" cargo "$@"
    else
        cargo "$@"
    fi
}

job_rust_build()  { cd "$RUST" && cargo build --workspace; }

job_rust_test() {
    cd "$RUST"
    host_cargo nextest run --workspace --profile ci
    run_quiet host_cargo test --workspace --doc
}

job_rust_clippy() { cd "$RUST" && cargo clippy --workspace --all-targets -- -D warnings; }
job_rust_fmt()    { cd "$RUST" && cargo fmt --all -- --check; }

job_rust_host()   { job_rust_test && job_rust_clippy && job_rust_fmt; }

job_rust_loom() {
    cd "$RUST"
    RUSTFLAGS="--cfg loom" cargo test -p runtime --release \
        --test loom_seqlock \
        --test loom_force_idle
}

MCU_ENV=(RUNTIME_STORAGE_SIZE=32768 RUNTIME_SAMPLE_RATE_HZ=10000)

job_rust_mcu_h7() {
    cd "$RUST"
    env "${MCU_ENV[@]}" \
        cargo build -p c-api --no-default-features \
        --features mcu-h7,header-runtime,motion-module-stepper \
        --target thumbv7em-none-eabi
}

job_rust_mcu_f4() {
    cd "$RUST"
    env "${MCU_ENV[@]}" \
        cargo build -p c-api --no-default-features \
        --features mcu-f4,header-runtime,motion-module-stepper \
        --target thumbv7em-none-eabi
}

job_rust_mcu_g0() {
    cd "$RUST"
    env "${MCU_ENV[@]}" \
        cargo build -p c-api --no-default-features \
        --features mcu-g0,header-runtime,motion-module-stepper \
        --target thumbv6m-none-eabi
}

# F103 is RAM-starved next to the H7/F4/G0 boards, so it gets its own storage
# profile instead of MCU_ENV, and a separate CARGO_TARGET_DIR because
# thumbv7m shares no artifacts with thumbv7em/thumbv6m.
job_rust_mcu_f1() {
    cd "$RUST"
    env CARGO_TARGET_DIR=target-f1 \
        RUNTIME_STORAGE_SIZE=10240 \
        RUNTIME_SAMPLE_RATE_HZ=2000 \
        cargo build -p c-api --no-default-features \
        --features mcu-f1,header-runtime,motion-module-stepper \
        --target thumbv7m-none-eabi --release
}

job_rust_no_stepper() {
    cd "$RUST"
    cargo build -p runtime --no-default-features --features host
    cargo test -p runtime --no-default-features --features host --no-run
}

job_cbindgen_drift() {
    "$ROOT/tools/regen_headers.sh"
    git -C "$ROOT" diff --exit-code rust/c-api/include/
}

job_c_smoke() {
    cd "$RUST"
    cargo build -p c-api --no-default-features \
        --features host,header-runtime --release
    cargo test -p c-api --no-default-features \
        --features host,header-runtime \
        --test c_smoke_build
}

# The hw EtherCAT endpoint (`--features hw`) links libethercat, present only on
# the bench Pi, so it is built nowhere else — its compile errors otherwise only
# surface on a flash. `cargo check` runs build.rs (compiling csrc/libecrt_igh.c
# via cc against the committed CI stub of ecrt.h) and typechecks the binary,
# catching both the C and Rust compile errors without linking libethercat.
# Linux-only: libecrt_igh.c uses Linux sched/clock APIs absent on macOS.
job_rust_ethercat_hw() {
    if [ "$(uname -s)" != Linux ]; then
        echo "rust-ethercat-hw: skipped on $(uname -s) (libecrt_igh.c needs Linux sched/clock APIs)"
        return 0
    fi
    cd "$RUST"
    IGH_DIR="$RUST/ethercat-rt/csrc/ci-igh" \
        cargo check -p ethercat-rt --features hw --bin ethercat-rt
}

job_deny() {
    if command -v cargo-deny >/dev/null 2>&1; then
        cargo deny --manifest-path "$RUST/Cargo.toml" check
    else
        echo "cargo-deny not installed (cargo install cargo-deny) — CI runs it via cargo-deny-action; skipping locally"
    fi
}

job_miri() {
    cd "$RUST"
    MIRIFLAGS="-Zmiri-ignore-leaks" cargo +nightly miri test -p runtime --features host \
        --test fault_encoding \
        --test motion_core_accel \
        --test seqlock_unit
    MIRIFLAGS="-Zmiri-ignore-leaks" cargo +nightly miri test -p runtime --features host \
        --lib phase_lut
}

job_panic_grep() {
    cd "$RUST"
    env "${MCU_ENV[@]}" \
        cargo rustc -p c-api --release \
        --no-default-features \
        --features mcu-h7,header-runtime,motion-module-stepper \
        --target thumbv7em-none-eabi -- --emit=llvm-ir
    shopt -s nullglob
    local ll_files=(target/thumbv7em-none-eabi/release/deps/*.ll)
    if [ ${#ll_files[@]} -eq 0 ]; then
        echo "No LLVM-IR files emitted; build step likely failed silently"
        return 1
    fi

    local total
    total=$(grep -hc 'panic_bounds_check' "${ll_files[@]}" 2>/dev/null | awk '{s+=$1} END{print s+0}')
    echo "panic_bounds_check total in MCU release build: ${total}"
    echo "  by function:"
    awk '/^define/{fn=$0} /panic_bounds_check/{print fn}' "${ll_files[@]}" \
        | grep -oE 'kalico_[a-z0-9_]+' | sort | uniq -c | sed 's/^/    /' || true

}

job_watchdog_canary() {
    grep -qF 'runtime_liveness_ok' "$ROOT/src/stm32/watchdog.c"
}

# Keep in lockstep with .github/workflows/ci-lintformat.yaml: an unpinned
# ruff drifts to whatever released last, so the local gate and CI disagree
# the moment a new rule ships. Bump both together.
RUFF_VERSION="0.15.21"

job_ruff() {
    if command -v uvx >/dev/null 2>&1; then
        uvx "ruff@$RUFF_VERSION" check "$ROOT" &&
            uvx "ruff@$RUFF_VERSION" format --check "$ROOT"
    elif command -v ruff >/dev/null 2>&1; then
        ruff check "$ROOT" && ruff format --check "$ROOT"
    else
        echo "ruff not installed (pip install ruff / uvx ruff)"
        return 1
    fi
}

job_py_typecheck() {
    cd "$ROOT" && uv run basedpyright
}

job_py() {
    local ver="${1:-3.13}"
    if command -v docker >/dev/null 2>&1; then
        docker run -v "$ROOT:/klipper" "$(docker_image)" --python "$ver" py.test -n auto
    else
        echo "docker unavailable — running py.test on the local interpreter only (CI runs 3.9-3.14)"
        cd "$ROOT" && python -m pytest -n auto
    fi
}

job_sim() {
    local sel="sim_unit and not needs_hardware"
    local paths="tools/sim \
        tools/test_host_io_seq_wrap.py \
        tools/test_motion_idle_timeout.py \
        tools/test_motion_static.py"
    if command -v docker >/dev/null 2>&1; then
        docker run --rm -v "$ROOT:/klipper" -w /klipper --entrypoint bash "$(docker_image)" -lc \
            "make -C tools/sim/preload >/dev/null && uv run py.test -n auto $paths -m '$sel'"
    else
        echo "docker unavailable — running sim unit tests on the local interpreter"
        make -C "$ROOT/tools/sim/preload" >/dev/null 2>&1 || true
        cd "$ROOT" && python -m pytest -n auto "$paths" -m "$sel"
    fi
}

job_sim_e2e() { "$ROOT/tools/sim/run.sh" test "$@"; }

# The producer must outrun the printer everywhere; the committed asset is the
# dense top-layer region that exhausted the bench's 0.25 s anchor lead. The
# 1.3x floor is deliberately loose — a healthy pipeline clears 2.5x on the
# slowest CI runner while the underrun class lands under 1x — so a red gate
# means "this would crash a print", not "the runner was busy".
job_replay_budget() {
    cd "$RUST" && cargo run --release -p motion-core --example gcode_replay_bench -- \
        "$ROOT/tools/sim/gcode/voron_dense_top_layers.gcode" --min-worst-x 1.3
}

job_docs() { cd "$ROOT/docs/_kalico" && uv run mkdocs build --strict; }

job_snapshot() {
    if command -v uv >/dev/null 2>&1; then
        cd "$ROOT" && uv run snapshots/snapshot-tests.sh --ci
    else
        "$ROOT/snapshots/snapshot-tests.sh" --ci
    fi
}

PASS=0; FAIL=0
FAILED_JOBS=()

red()    { printf '\033[1;31m%s\033[0m\n' "$*"; }
green()  { printf '\033[1;32m%s\033[0m\n' "$*"; }

tally() { sed -e '/^[[:space:]]*$/d' -e 's/^[[:space:]]*//' "$1" | tail -1 | cut -c1-120; }

job_log() { mkdir -p "$LOG_DIR"; echo "$LOG_DIR/$1.log"; }

run_check() {
    local name="$1"; shift
    printf '%-20s ' "$name"
    local log rc=0
    log="$(job_log "$name")"
    # Standalone statement, NOT `... && rc=0 || rc=$?`: in a `&&`/`||`
    # condition context bash ignores the subshell's `set -e`, so multi-command
    # jobs would report only their last command's status and a failing
    # nextest run could hide behind passing doc-tests.
    ( set -e; "$@" ) >"$log" 2>&1
    rc=$?
    if [ "$rc" -eq 0 ]; then
        printf '\033[1;32mPASS\033[0m  %s  \033[2m[%s]\033[0m\n' \
            "$(tally "$log")" ".ci-logs/$name.log"; PASS=$((PASS + 1))
    else
        red "FAIL ($rc) — full log: .ci-logs/$name.log"
        FAIL=$((FAIL + 1)); FAILED_JOBS+=("$name")
        sed 's/^/    /' "$log" | tail -100
    fi
    return "$rc"
}

run_all() {
    local quick="${1:-false}"
    run_check "ruff"            job_ruff
    run_check "rust-test"       job_rust_test
    run_check "rust-clippy"     job_rust_clippy
    run_check "rust-fmt"        job_rust_fmt
    run_check "watchdog-canary" job_watchdog_canary
    if [ "$quick" != "true" ]; then
        run_check "cbindgen-drift"  job_cbindgen_drift
        run_check "c-smoke"         job_c_smoke
        run_check "rust-ethercat-hw" job_rust_ethercat_hw
        run_check "rust-mcu-h7"     job_rust_mcu_h7
        run_check "rust-mcu-f4"     job_rust_mcu_f4
        run_check "rust-mcu-g0"     job_rust_mcu_g0
        run_check "rust-mcu-f1"     job_rust_mcu_f1
        run_check "rust-no-stepper" job_rust_no_stepper
        run_check "rust-loom"       job_rust_loom
        run_check "miri"            job_miri
        run_check "panic-grep"      job_panic_grep
        run_check "deny"            job_deny
        run_check "docs"            job_docs
        run_check "py"              job_py
        run_check "py-typecheck"    job_py_typecheck
        run_check "sim"             job_sim
        run_check "snapshot"        job_snapshot
        run_check "replay-budget"   job_replay_budget
    fi
    echo "────────────────────────────────────────"
    printf '  %s   %s\n' "$(green "$PASS pass")" "$([ "$FAIL" -gt 0 ] && red "$FAIL fail" || echo "0 fail")"
    [ "$FAIL" -eq 0 ] || { printf '  failed: %s\n' "${FAILED_JOBS[*]}"; }
    echo "────────────────────────────────────────"
    return "$FAIL"
}

usage() {
    awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"
}

job_install_hooks() {
    cd "$ROOT"
    [ -x .githooks/pre-push ] || {
        echo "error: .githooks/pre-push missing or not executable" >&2
        return 1
    }
    git config core.hooksPath .githooks
    echo "pre-push hook enabled (core.hooksPath = .githooks)."
    echo "  runs './scripts/ci.sh quick' before every push — incl. direct pushes to main"
    echo "  bypass once:  git push --no-verify"
    echo "  disable:      git config --unset core.hooksPath"
}

if [ "${1:-}" = "-v" ] || [ "${1:-}" = "--verbose" ]; then
    VERBOSE=1
    shift
fi

case "${1:-all}" in
    all)                 run_all false; exit $? ;;
    quick|--quick)       run_all true; exit $? ;;
    install-hooks|hooks) job_install_hooks; exit $? ;;
    -h|--help|help)      usage; exit 0 ;;
esac

name="$1"; shift
case "$name" in
    rust-host)        job=(job_rust_host) ;;
    rust-build)       job=(job_rust_build) ;;
    rust-test)        job=(job_rust_test) ;;
    rust-clippy)      job=(job_rust_clippy) ;;
    rust-fmt)         job=(job_rust_fmt) ;;
    rust-loom)        job=(job_rust_loom) ;;
    rust-mcu-h7)      job=(job_rust_mcu_h7) ;;
    rust-mcu-f4)      job=(job_rust_mcu_f4) ;;
    rust-mcu-g0)      job=(job_rust_mcu_g0) ;;
    rust-mcu-f1)      job=(job_rust_mcu_f1) ;;
    rust-no-stepper)  job=(job_rust_no_stepper) ;;
    cbindgen-drift)   job=(job_cbindgen_drift) ;;
    c-smoke)          job=(job_c_smoke) ;;
    rust-ethercat-hw) job=(job_rust_ethercat_hw) ;;
    deny)             job=(job_deny) ;;
    miri)             job=(job_miri) ;;
    panic-grep)       job=(job_panic_grep) ;;
    watchdog-canary)  job=(job_watchdog_canary) ;;
    ruff)             job=(job_ruff) ;;
    py)               job=(job_py "${1:-3.13}") ;;
    py-typecheck)     job=(job_py_typecheck) ;;
    docs)             job=(job_docs) ;;
    sim)              job=(job_sim) ;;
    sim-e2e)          job=(job_sim_e2e ${@+"$@"}) ;;
    replay-budget)    job=(job_replay_budget) ;;
    snapshot)         job=(job_snapshot) ;;
    *) echo "unknown job: $name" >&2; usage >&2; exit 2 ;;
esac

if stream_output; then
    VERBOSE=1
    log="$(job_log "$name")"
    "${job[@]}" 2>&1 | tee "$log"
    exit "${PIPESTATUS[0]}"
fi
run_check "$name" "${job[@]}"
exit $?
