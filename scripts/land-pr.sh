#!/usr/bin/env bash
# Land a branch through a pull request: push, open the PR if missing, wait for
# the checks, merge with a merge commit, then move local main to origin/main
# and remove the branch's worktree. Never switches a checkout; a checked-out
# main is fast-forwarded only when it has no uncommitted changes.
#
#   scripts/land-pr.sh <branch> [--title T] [--body B] [--dry-run]
#
# Safe to run from the branch's own worktree: the whole script is parsed
# before it runs and it leaves the worktree before removing it. When main
# moved while the checks ran, the PR branch is updated on GitHub and the
# checks are awaited again. Fails loudly on red checks and prints the tail of
# the failed run's log.
set -euo pipefail

reexec_current_copy() {
    [ -z "${LAND_PR_REEXEC:-}" ] || return 0
    local top fresh
    top="$(git rev-parse --show-toplevel)" || { echo "land-pr: run it inside the repository" >&2; exit 1; }
    local try
    for try in 1 2 3; do
        git -C "$top" fetch -q origin && break
        [ "$try" -lt 3 ] || { echo "land-pr: git fetch origin failed 3 times" >&2; exit 1; }
        sleep $((try * 10))
    done
    fresh="$(mktemp -d)"
    git -C "$top" archive origin/main scripts | tar -x -C "$fresh" \
        || { echo "land-pr: could not extract scripts/ from origin/main" >&2; exit 1; }
    [ -f "$fresh/scripts/land-pr.sh" ] \
        || { echo "land-pr: origin/main has no scripts/land-pr.sh" >&2; exit 1; }
    if cmp -s "$fresh/scripts/land-pr.sh" "$0"; then
        rm -rf "$fresh"
        return 0
    fi
    echo "land-pr: this copy differs from origin/main's; running origin/main's" >&2
    LAND_PR_REEXEC=1 GH_UTIL="$fresh/scripts/gh_util.py" SERVAL_REPO_ROOT="$top" \
        exec bash "$fresh/scripts/land-pr.sh" "$@"
}
reexec_current_copy "$@"

FAILED_LOG_TAIL_LINES=80
POLL_SECONDS=60
MAX_UPDATES=3
MAX_FUZZ_RERUNS=3

gh_util="${GH_UTIL:-$(cd "$(dirname "$0")" && pwd)/gh_util.py}"
[ -f "$gh_util" ] || gh_util="$(git rev-parse --show-toplevel)/scripts/gh_util.py"
[ -f "$gh_util" ] || { echo "land-pr: scripts/gh_util.py not found" >&2; exit 1; }
fuzz_triage="$(dirname "$gh_util")/fuzz_triage.py"
[ -f "$fuzz_triage" ] || { echo "land-pr: scripts/fuzz_triage.py not found beside gh_util.py" >&2; exit 1; }
gh() { python3 "$gh_util" "$@"; }
git_net() { python3 "$gh_util" --git "$@"; }

usage() { echo "usage: $0 <branch> [--title T] [--body B] [--dry-run]" >&2; exit 2; }
die() { echo "land-pr: $*" >&2; exit 1; }

run() {
    if [ "$dry_run" -eq 1 ]; then echo "+ $*"; else "$@"; fi
}

main_worktree() {
    git worktree list --porcelain | awk 'NR == 1 { print substr($0, 10) }'
}

worktree_of_branch() {
    git worktree list --porcelain | awk -v b="refs/heads/${1:-$branch}" '
        /^worktree / { path = substr($0, 10) }
        $0 == "branch " b { print path }'
}

pr_number() {
    gh pr view "$branch" --json number,state --jq 'select(.state=="OPEN") | .number' 2>/dev/null || true
}

open_pr() {
    local create=(gh pr create --base main --head "$branch")
    if [ -n "$title" ]; then
        create+=(--title "$title" --body "${body:-$title}")
    else
        create+=(--fill)
    fi
    run "${create[@]}"
}

await_checks() {
    local fuzz_reruns=0
    while true; do
        sleep "$POLL_SECONDS"
        if gh pr checks "$number" --watch --interval "$POLL_SECONDS"; then
            return 0
        fi
        echo "land-pr: checks failed on PR #$number" >&2
        local run_id head
        head="$(git rev-parse "$branch")"
        run_id="$(gh run list --branch "$branch" --status failure --limit 20 --json databaseId,headSha --jq "[.[] | select(.headSha == \"$head\")][0].databaseId")"
        [ -z "$run_id" ] || gh run view "$run_id" --log-failed | tail -n "$FAILED_LOG_TAIL_LINES" >&2
        [ -n "$run_id" ] && [ "$fuzz_reruns" -lt "$MAX_FUZZ_RERUNS" ] || return 1
        local rc=0
        python3 "$fuzz_triage" triage "$branch" "$run_id" || rc=$?
        case "$rc" in
            0) fuzz_reruns=$((fuzz_reruns + 1))
               echo "land-pr: a failure that main has too; reran the failed job ($fuzz_reruns/$MAX_FUZZ_RERUNS)" >&2 ;;
            10) die "a property-test failure that is this PR's to fix (see above)" ;;
            *) return 1 ;;
        esac
    done
}

merge_when_current() {
    local attempt
    for attempt in $(seq 1 "$MAX_UPDATES"); do
        if gh pr merge "$number" --merge; then
            return 0
        fi
        local state
        state="$(gh pr view "$number" --json mergeStateStatus --jq .mergeStateStatus)"
        [ "$state" = "BEHIND" ] || die "PR #$number cannot be merged (merge state $state)"
        echo "land-pr: main moved; updating the PR branch (attempt $attempt/$MAX_UPDATES)"
        gh pr update-branch "$number"
        await_checks || return 1
    done
    die "PR #$number is still behind main after $MAX_UPDATES updates"
}

advance_main() {
    local tree
    tree="$(worktree_of_branch main)"
    if [ -z "$tree" ]; then
        run git branch -f main origin/main
    elif [ "$dry_run" -eq 1 ]; then
        echo "+ git -C $tree merge --ff-only origin/main"
    elif [ -n "$(git -C "$tree" status --porcelain --untracked-files=no)" ]; then
        echo "land-pr: $tree has main checked out with uncommitted changes; main was not advanced" >&2
    else
        local guard
        guard="$(dirname "$gh_util")/guard-main-checkout.sh"
        [ ! -x "$guard" ] || "$guard" lift "$tree"
        git -C "$tree" merge --ff-only origin/main \
            || echo "land-pr: $tree has a main that is not a fast-forward of origin/main; main was not advanced" >&2
        [ ! -x "$guard" ] || "$guard" apply "$tree"
    fi
}

finish() {
    run git_net fetch origin
    advance_main
    local tree
    tree="$(worktree_of_branch)"
    if [ -n "$tree" ] && [ "$dry_run" -eq 0 ]; then
        local safe
        safe="$(main_worktree)"
        cd "$safe"
        case "$(basename "$tree")" in
            serval-pool-*)
                git -C "$tree" switch -q --detach origin/main \
                    && git branch -D "$branch" >/dev/null \
                    && rm -f "$(git -C "$tree" rev-parse --absolute-git-dir)/wt-lease" \
                    && echo "land-pr: $tree is idle again (warm target kept; scripts/wt.sh new reuses it)" \
                    || echo "land-pr: $tree has uncommitted changes and stays on $branch" >&2
                ;;
            *)
                slot="$(basename "$tree" | tr '[:upper:]' '[:lower:]')"
                git worktree remove "$tree" \
                    && docker rmi "klipper-build-$slot" "kalico-sim-$slot" >/dev/null 2>&1 \
                    || true
                [ ! -d "$tree" ] || echo "land-pr: $tree has uncommitted or untracked files and was kept; remove it with 'git worktree remove --force $tree' when done" >&2
                ;;
        esac
    elif [ -n "$tree" ]; then
        echo "+ git worktree remove $tree"
    fi
    [ "$dry_run" -eq 1 ] || git log --oneline -1 origin/main
}

main() {
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

    git rev-parse --verify --quiet "refs/heads/$branch" >/dev/null || die "no local branch $branch"
    [ "$branch" != "main" ] || die "refusing to land main"

    run git_net push -u origin "$branch"
    number="$(pr_number)"
    if [ -z "$number" ]; then
        open_pr
        [ "$dry_run" -eq 1 ] || number="$(pr_number)"
    fi
    [ "$dry_run" -eq 1 ] && number="<new>"
    [ -n "$number" ] || die "could not find the PR for $branch"
    echo "PR #$number"

    if [ "$dry_run" -eq 1 ]; then
        echo "+ gh pr checks $number --watch"
        echo "+ gh pr merge $number --merge (gh pr update-branch and re-check when behind)"
    else
        await_checks || exit 1
        merge_when_current || exit 1
    fi
    finish
}

main "$@"
