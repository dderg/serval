#!/usr/bin/env bash
# Run the planner corpus: every snapshot case, the sim prints (and their
# z-hop variants) and the corner_flow cases on the bench configs and on every
# machine profile, through the production pipeline, side by side.
#
#   scripts/corpus.sh [--configs fast,printer] [-k SUBSTRING] [--audit]
#   scripts/corpus.sh --probe FILE.rs [--configs ...] [-k ...]
#   scripts/corpus.sh --path-accuracy [-k ...]
#
# Without --probe it plans every selected case and checks the result is
# finite and holds the config's z caps. --probe copies FILE.rs to
# rust/pipeline-snapshot/tests/ for the run
# and removes it afterwards; the probe starts with `mod corpus;` and iterates
# `corpus::corpus()` (see tests/corpus/mod.rs), so nobody has to copy the case
# list into a throwaway test. --audit builds with the fit-audit feature.
# --path-accuracy measures the shaped path against the G-code polyline on the
# bench and machine-profile configs (tests/path_accuracy.rs).
# Output is printed (--no-capture) so a probe can report per case.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PROBE_TEST=zz_probe
probe="" features=() target=corpus_plans
while [ $# -gt 0 ]; do
    case "$1" in
        --configs) export CORPUS_CONFIGS="${2:?--configs needs a value}"; shift 2 ;;
        -k) export CORPUS_FILTER="${2:?-k needs a value}"; shift 2 ;;
        --audit) features=(--features fit-audit); shift ;;
        --probe) probe="${2:?--probe needs a file}"; shift 2 ;;
        --path-accuracy) target=path_accuracy; shift ;;
        *) echo "usage: $0 [--configs a,b] [-k SUBSTR] [--audit] [--probe FILE.rs] [--path-accuracy]" >&2; exit 2 ;;
    esac
done

[ -n "${CI_KEEP_CARGO_TARGET_DIR:-}" ] || unset CARGO_TARGET_DIR
export PATH="$HOME/.cargo/bin:$PATH"
cd "$ROOT/rust"

if [ -n "$probe" ]; then
    [ -f "$probe" ] || { echo "corpus: no such probe file $probe" >&2; exit 2; }
    probe_path="$ROOT/rust/pipeline-snapshot/tests/$PROBE_TEST.rs"
    cp "$probe" "$probe_path"
    trap 'rm -f "$probe_path"' EXIT
    target=$PROBE_TEST
fi

cargo nextest run -p pipeline-snapshot "${features[@]}" --test "$target" \
    --run-ignored all --no-capture
