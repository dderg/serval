#!/usr/bin/env bash
# Make the shared checkout (the first entry of `git worktree list`, normally
# ~/Developer/serval on main) read-only for edits, so a relative path typed
# with the wrong working directory fails loudly instead of editing main.
#
#   scripts/guard-main-checkout.sh apply [TREE]   tracked files and their directories lose write
#   scripts/guard-main-checkout.sh lift  [TREE]   give the owner write back (before git updates it)
#
# It only acts on a checkout named `serval` (GUARD_ANY=1 overrides, for tests).
# land-pr.sh lifts, fast-forwards main and applies again; install-git-guards.sh
# and wt.sh apply it. Untracked files and .git are never touched, so git, fetch
# and `git worktree add` keep working. Work happens in worktrees (CLAUDE.md).
set -euo pipefail

mode="${1:?usage: guard-main-checkout.sh apply|lift [TREE]}"
tree="${2:-$(git worktree list --porcelain | awk 'NR == 1 { print substr($0, 10) }')}"
[ -d "$tree/.git" ] || { echo "guard-main-checkout: $tree is not the main checkout" >&2; exit 1; }
if [ "$(basename "$tree")" != serval ] && [ -z "${GUARD_ANY:-}" ]; then
    exit 0
fi

case "$mode" in
    apply) change="a-w" ;;
    lift) change="u+w" ;;
    *) echo "guard-main-checkout: unknown mode $mode" >&2; exit 2 ;;
esac

cd "$tree"
{
    git ls-files -z
    git ls-files -z | xargs -0 -n1 dirname -z | sort -zu
    printf '.\0'
} | xargs -0 chmod "$change" 2>/dev/null || true
