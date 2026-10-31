#!/usr/bin/env bash
# Public-API snapshot gate for the planner contract crates.
#
#   scripts/public-api.sh            # diff each crate's public surface against rust/api/<crate>.txt
#   scripts/public-api.sh --update   # rewrite the baselines from the current source
#
# A contract change is a reviewed diff of rust/api/<crate>.txt, in the same
# commit as the code that needs it. Requires `cargo install cargo-public-api
# --locked` and the pinned nightly below (rustdoc JSON); std's internal paths
# such as `alloc::sync::Arc` move between nightlies, so CI pins the same one.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUST="$ROOT/rust"
API_DIR="$RUST/api"
CRATES=(geometry trajectory motion-pipeline planner-config)
export RUSTUP_TOOLCHAIN=nightly-2026-10-30

command -v cargo-public-api >/dev/null 2>&1 || {
    echo "cargo-public-api missing: cargo install cargo-public-api --locked" >&2
    exit 2
}
rustup run "$RUSTUP_TOOLCHAIN" rustc --version >/dev/null 2>&1 || {
    echo "$RUSTUP_TOOLCHAIN missing: rustup toolchain install $RUSTUP_TOOLCHAIN --profile minimal" >&2
    exit 2
}

render() {
    (cd "$RUST" && cargo public-api -p "$1" --simplified \
        --omit blanket-impls,auto-trait-impls,auto-derived-impls 2>/dev/null)
}

mkdir -p "$API_DIR"
rc=0
for crate in "${CRATES[@]}"; do
    baseline="$API_DIR/$crate.txt"
    if [ "${1:-}" = "--update" ]; then
        render "$crate" >"$baseline"
        echo "updated $baseline ($(wc -l <"$baseline" | tr -d ' ') items)"
        continue
    fi
    if [ ! -f "$baseline" ]; then
        echo "FAIL $crate: no baseline at $baseline (run scripts/public-api.sh --update)" >&2
        rc=1
        continue
    fi
    if diff -u "$baseline" <(render "$crate") >"$API_DIR/.$crate.diff"; then
        echo "ok   $crate"
    else
        echo "FAIL $crate: public API drifted from $baseline" >&2
        cat "$API_DIR/.$crate.diff" >&2
        echo "review the diff, then: scripts/public-api.sh --update" >&2
        rc=1
    fi
    rm -f "$API_DIR/.$crate.diff"
done
exit "$rc"
