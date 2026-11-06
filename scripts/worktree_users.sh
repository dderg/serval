#!/usr/bin/env bash
# Print "<pid> <cmdline>" for every process other than this one's ancestors whose
# cwd is inside the worktree given as $1. An agent's persistent shell keeps its
# cwd in the tree between commands, so this is how a pool worktree in use is told
# from an idle one.
set -euo pipefail

root="$(cd "${1:?usage: worktree_users.sh <worktree>}" && pwd -P)"

ancestors=" "
pid=$$
while [ "$pid" -gt 1 ]; do
    ancestors="$ancestors$pid "
    pid="$(awk '/^PPid:/ { print $2 }' "/proc/$pid/status" 2>/dev/null || echo 1)"
done

for proc in /proc/[0-9]*; do
    pid="${proc#/proc/}"
    case "$ancestors" in *" $pid "*) continue ;; esac
    cwd="$(readlink "$proc/cwd" 2>/dev/null || true)"
    case "$cwd" in
        "$root" | "$root"/*) printf '%s %s\n' "$pid" "$(tr '\0' ' ' < "$proc/cmdline" 2>/dev/null | cut -c1-100)" ;;
    esac
done
