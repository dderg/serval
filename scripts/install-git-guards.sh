#!/usr/bin/env bash
# Install the repository-wide git guards into the shared hooks directory, so
# they apply to every worktree of this clone at once (idempotent; wt.sh runs it).
#
#   scripts/install-git-guards.sh
#
# Today: reference-transaction, which blocks `git stash` (see the hook).
# It does not touch core.hooksPath, so the opt-in pre-push hook is unaffected.
set -euo pipefail

common="$(git rev-parse --git-common-dir)"
hooks="$common/hooks"
source_hook="$(cd "$(dirname "$0")/.." && pwd)/.githooks/reference-transaction"
[ -f "$source_hook" ] || { echo "install-git-guards: $source_hook not found" >&2; exit 1; }
if [ -n "$(git config --get core.hooksPath || true)" ]; then
    echo "install-git-guards: core.hooksPath is set; add the hook there instead" >&2
    exit 1
fi
mkdir -p "$hooks"
if ! cmp -s "$source_hook" "$hooks/reference-transaction" 2>/dev/null; then
    install -m 755 "$source_hook" "$hooks/reference-transaction"
    echo "install-git-guards: installed reference-transaction into $hooks"
fi
