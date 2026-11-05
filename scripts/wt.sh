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
#
# Allocation is serialised by an flock on <dir>/.serval-pool.lock and recorded in a
# lease file (<gitdir>/wt-lease: owner pid, topic) that land-pr.sh removes. A
# worktree is idle only if detached, clean (untracked files count) and not leased
# by a live pid.
set -euo pipefail

POOL_DIR="${WT_POOL_DIR:-$HOME/Developer}"
POOL_PREFIX="serval-pool-"

die() { echo "wt: $*" >&2; exit 1; }

pool_paths() {
    git worktree list --porcelain | awk -v p="$POOL_DIR/$POOL_PREFIX" \
        '/^worktree / { path = substr($0, 10); if (index(path, p) == 1) print path }'
}

lease_file() { echo "$(git -C "$1" rev-parse --absolute-git-dir)/wt-lease"; }

lease_owner_alive() {
    local file pid
    file="$(lease_file "$1")"
    [ -f "$file" ] || return 1
    pid="$(sed -n 's/^pid=//p' "$file")"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

is_idle() {
    local path="$1"
    [ -z "$(git -C "$path" symbolic-ref -q HEAD || true)" ] \
        && [ -z "$(git -C "$path" status --porcelain)" ] \
        && ! lease_owner_alive "$path"
}

claim() {
    printf 'pid=%s\ntopic=%s\nsince=%s\n' "${WT_OWNER_PID:-$PPID}" "$2" "$(date +%s)" > "$(lease_file "$1")"
}

next_pool_path() {
    local n=1
    while [ -e "$POOL_DIR/$POOL_PREFIX$n" ]; do n=$((n + 1)); done
    echo "$POOL_DIR/$POOL_PREFIX$n"
}

new_topic() {
    local topic="${1:?usage: wt.sh new <topic> [--base REF]}"; shift
    "$(dirname "$0")/install-git-guards.sh" >&2
    "$(dirname "$0")/guard-main-checkout.sh" apply >&2 || true
    python3 "$(dirname "$0")/gc_caches.py" --if-free-below "${WT_GC_BELOW_GB:-100}" --apply >&2 || true
    local base="origin/main"
    if [ "${1:-}" = "--base" ]; then base="${2:?--base needs a ref}"; fi
    git fetch -q origin
    exec 9>"$POOL_DIR/.serval-pool.lock"
    flock 9
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
    claim "$path" "$topic"
    git -C "$path" switch -q -c "$topic" "$base"
    echo "$path"
}

list_pool() {
    local path
    while read -r path; do
        [ -n "$path" ] || continue
        if is_idle "$path"; then echo "idle  $path"; else echo "busy  $path ($(git -C "$path" rev-parse --abbrev-ref HEAD), lease: $(tr '\n' ' ' < "$(lease_file "$path")" 2>/dev/null || echo none))"; fi
    done < <(pool_paths)
}

case "${1:-}" in
    new) shift; new_topic "$@" ;;
    list) list_pool ;;
    *) die "usage: wt.sh new <topic> [--base REF] | list" ;;
esac
