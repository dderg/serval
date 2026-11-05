#!/usr/bin/env bash
# Install the repository-wide git guards into the shared hooks directory, so
# they apply to every worktree of this clone at once (idempotent; wt.sh runs it).
#
#   scripts/install-git-guards.sh
#
# reference-transaction blocks `git stash`; post-checkout nudges cold worktrees to wt.sh;
# the throughput-baseline merge driver (scripts/merge_throughput_baseline.py) resolves
# concurrent baseline accepts instead of leaving conflicts (.gitattributes selects it).
# It does not touch core.hooksPath, so the opt-in pre-push hook is unaffected.
set -euo pipefail

common="$(git rev-parse --git-common-dir)"
hooks="$common/hooks"
hooks_source="$(cd "$(dirname "$0")/.." && pwd)/.githooks"
if [ -n "$(git config --get core.hooksPath || true)" ]; then
    echo "install-git-guards: core.hooksPath is set; add the hook there instead" >&2
    exit 1
fi
mkdir -p "$hooks"
main_tree="$(git worktree list --porcelain | awk 'NR == 1 { print substr($0, 10) }')"
driver="python3 $main_tree/scripts/merge_throughput_baseline.py %O %A %B %P"
if [ "$(git config --local --get merge.throughput-baseline.driver || true)" != "$driver" ]; then
    git config --local merge.throughput-baseline.name "field-wise merge of the throughput baseline"
    git config --local merge.throughput-baseline.driver "$driver"
    echo "install-git-guards: registered the throughput-baseline merge driver"
fi
for name in reference-transaction post-checkout; do
    source_hook="$hooks_source/$name"
    [ -f "$source_hook" ] || { echo "install-git-guards: $source_hook not found" >&2; exit 1; }
    if ! cmp -s "$source_hook" "$hooks/$name" 2>/dev/null; then
        install -m 755 "$source_hook" "$hooks/$name"
        echo "install-git-guards: installed $name into $hooks"
    fi
done
