#!/usr/bin/env bash
# Start a topic branch in a warm worktree.
#
#   scripts/wt.sh new <topic> [--base REF]   prints the worktree path
#   scripts/wt.sh list                       the pool and which entries are idle
#
# Every new worktree used to start with a cold rust/target: ~250 s to the first
# green `ci.sh quick` against ~42 s in a warm one (75 worktrees over the audited
# sessions). The pool keeps worktrees named <dir>/serval-pool-N, whose target
# dirs stay warm: `new` takes an idle one (detached HEAD, clean tree), switches
# it to a new branch cut from the base (default origin/main) and prints its
# path; land-pr.sh returns a pool worktree to idle (detached at origin/main)
# instead of removing it. Cargo's freshness check compares mtimes in the same
# path, so reusing one worktree is safe; sharing a target dir between paths is
# not (see CLAUDE.md).
set -euo pipefail

POOL_DIR="${WT_POOL_DIR:-$HOME/Developer}"
POOL_PREFIX="serval-pool-"

die() { echo "wt: $*" >&2; exit 1; }

pool_paths() {
    git worktree list --porcelain | awk -v p="$POOL_DIR/$POOL_PREFIX" \
        '/^worktree / { path = substr($0, 10); if (index(path, p) == 1) print path }'
}

is_idle() {
    local path="$1"
    [ -z "$(git -C "$path" symbolic-ref -q HEAD || true)" ] \
        && [ -z "$(git -C "$path" status --porcelain --untracked-files=no)" ]
}

next_pool_path() {
    local n=1
    while [ -e "$POOL_DIR/$POOL_PREFIX$n" ]; do n=$((n + 1)); done
    echo "$POOL_DIR/$POOL_PREFIX$n"
}

new_topic() {
    local topic="${1:?usage: wt.sh new <topic> [--base REF]}"; shift
    local base="origin/main"
    if [ "${1:-}" = "--base" ]; then base="${2:?--base needs a ref}"; fi
    git fetch -q origin
    git rev-parse --verify --quiet "refs/heads/$topic" >/dev/null && die "branch $topic exists"
    local path=""
    local candidate
    while read -r candidate; do
        [ -n "$candidate" ] && is_idle "$candidate" && { path="$candidate"; break; }
    done < <(pool_paths)
    if [ -z "$path" ]; then
        path="$(next_pool_path)"
        git worktree add -q --detach "$path" "$base"
    fi
    git -C "$path" switch -q -c "$topic" "$base"
    echo "$path"
}

list_pool() {
    local path
    while read -r path; do
        [ -n "$path" ] || continue
        if is_idle "$path"; then echo "idle  $path"; else echo "busy  $path ($(git -C "$path" rev-parse --abbrev-ref HEAD))"; fi
    done < <(pool_paths)
}

case "${1:-}" in
    new) shift; new_topic "$@" ;;
    list) list_pool ;;
    *) die "usage: wt.sh new <topic> [--base REF] | list" ;;
esac
