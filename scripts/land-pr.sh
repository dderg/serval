#!/usr/bin/env bash
# Land a branch through a pull request: push, open the PR if missing, wait for
# the checks, merge with a merge commit, then move local main to origin/main
# and remove the branch's worktree. Never checks out anything.
#
#   scripts/land-pr.sh <branch> [--title T] [--body B] [--dry-run]
#
# Fails loudly on red checks and prints the tail of the failed run's log.
set -euo pipefail

FAILED_LOG_TAIL_LINES=80
POLL_SECONDS=30

usage() { echo "usage: $0 <branch> [--title T] [--body B] [--dry-run]" >&2; exit 2; }
die() { echo "land-pr: $*" >&2; exit 1; }

[ $# -ge 1 ] || usage
branch="$1"; shift
title="" body="" dry_run=0
while [ $# -gt 0 ]; do
    case "$1" in
        --title) title="${2:?--title needs a value}"; shift 2 ;;
        --body) body="${2:?--body needs a value}"; shift 2 ;;
        --dry-run) dry_run=1; shift ;;
        *) usage ;;
    esac
done

run() {
    if [ "$dry_run" -eq 1 ]; then echo "+ $*"; else "$@"; fi
}

git rev-parse --verify --quiet "refs/heads/$branch" >/dev/null || die "no local branch $branch"
[ "$branch" != "main" ] || die "refusing to land main"

worktree_of_branch() {
    git worktree list --porcelain | awk -v b="refs/heads/$branch" '
        /^worktree / { path = substr($0, 10) }
        $0 == "branch " b { print path }'
}

pr_number() {
    gh pr view "$branch" --json number,state --jq 'select(.state=="OPEN") | .number' 2>/dev/null || true
}

run git push -u origin "$branch"

number="$(pr_number)"
if [ -z "$number" ]; then
    create=(gh pr create --base main --head "$branch")
    if [ -n "$title" ]; then
        create+=(--title "$title" --body "${body:-$title}")
    else
        create+=(--fill)
    fi
    run "${create[@]}"
    [ "$dry_run" -eq 1 ] || number="$(pr_number)"
fi
[ "$dry_run" -eq 1 ] && number="<new>"
[ -n "$number" ] || die "could not find the PR for $branch"
echo "PR #$number"

if [ "$dry_run" -eq 0 ]; then
    sleep "$POLL_SECONDS"
    if ! gh pr checks "$number" --watch --interval "$POLL_SECONDS"; then
        echo "land-pr: checks failed on PR #$number" >&2
        run_id="$(gh run list --branch "$branch" --status failure --limit 1 --json databaseId --jq '.[0].databaseId')"
        [ -z "$run_id" ] || gh run view "$run_id" --log-failed | tail -n "$FAILED_LOG_TAIL_LINES" >&2
        exit 1
    fi
else
    echo "+ gh pr checks $number --watch"
fi

run gh pr merge "$number" --merge
run git fetch origin
run git branch -f main origin/main

tree="$(worktree_of_branch)"
if [ -n "$tree" ] && [ "$tree" != "$(git rev-parse --show-toplevel)" ]; then
    run git worktree remove "$tree"
fi
[ "$dry_run" -eq 1 ] || git log --oneline -1 origin/main
