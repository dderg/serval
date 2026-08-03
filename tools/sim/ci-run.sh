#!/usr/bin/env bash
# Run the sim e2e suite on GitHub Actions instead of local Docker.
#
#   tools/sim/ci-run.sh                     # full shard suite (ci-sim-e2e.yaml)
#   tools/sim/ci-run.sh test_probe.py       # one runner for that file
#   tools/sim/ci-run.sh test_probe.py test_print.py "beacon and contact"
#                                           # one runner PER item, in parallel
#
# Items containing ".py" or "::" are pytest targets (bare names resolve
# under tools/sim/tests/); anything else is a pytest -k expression.
# Pushes the current branch first, then dispatches and watches the run.
set -euo pipefail

branch=$(git rev-parse --abbrev-ref HEAD)
[[ "$branch" != "HEAD" ]] || { echo "detached HEAD; check out a branch first" >&2; exit 1; }

if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "warning: uncommitted changes will NOT run in CI" >&2
fi

git push origin "$branch"

if [[ $# -eq 0 ]]; then
    workflow=ci-sim-e2e.yaml
    dispatch_args=()
else
    workflow=sim-e2e-dispatch.yaml
    tests=$(IFS=,; printf '%s' "$*")
    dispatch_args=(-f "tests=$tests")
fi

prev_id=$(gh run list --workflow "$workflow" --branch "$branch" --limit 1 \
    --json databaseId -q '.[0].databaseId' 2>/dev/null || true)

gh workflow run "$workflow" --ref "$branch" ${dispatch_args[@]+"${dispatch_args[@]}"}

echo "waiting for the run to appear..."
run_id=""
for _ in $(seq 30); do
    sleep 2
    run_id=$(gh run list --workflow "$workflow" --branch "$branch" --limit 1 \
        --json databaseId -q '.[0].databaseId' 2>/dev/null || true)
    [[ -n "$run_id" && "$run_id" != "$prev_id" ]] && break
    run_id=""
done
[[ -n "$run_id" ]] || { echo "dispatched, but the run never appeared; check gh run list" >&2; exit 1; }

gh run watch "$run_id" --exit-status
