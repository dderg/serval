#!/usr/bin/env python3
"""Accept the snapshot baselines of a reviewed PR and commit them.

    uv run python scripts/accept_reviewed.py <pr> [--accept-suspicious CASE=REASON ...]
        [--no-commit] [--dry-run]

Run from the worktree of the PR's branch, with the branch pushed (the review
runs on the PR head). It runs scripts/review_baselines.py --pr N, then:

- refuses when any case is a REGRESSION;
- refuses when a SUSPICIOUS case is not listed with --accept-suspicious
  CASE=REASON (the reason is required and goes into the commit body);
- accepts the EXPECTED cases, the listed SUSPICIOUS cases and the new cases
  through snapshots/run.py --accept, then checks that every snapshot is exact;
- commits snapshots/baselines with the review summary in the body.

Never accept a change you cannot explain.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path

import pending_baselines as pending

ROOT = pending.ROOT
BODY_WIDTH = 72
SUBJECT_LIMIT = 50


class AcceptError(RuntimeError):
    pass


def parse_suspicious(items: list[str]) -> dict[str, str]:
    reasons = {}
    for item in items:
        case, sep, reason = item.partition("=")
        if not sep or not reason.strip():
            raise AcceptError(
                f"--accept-suspicious {item!r}: write CASE=REASON, the reason "
                "is required"
            )
        reasons[case.strip()] = reason.strip()
    return reasons


def decide(
    review: dict, suspicious: dict[str, str]
) -> tuple[list[str], list[str]]:
    cases = review["cases"]
    known = {case["name"] for case in cases}
    errors = [
        f"--accept-suspicious names a case the review did not flag: {name}"
        for name in suspicious
        if name not in known
    ]
    for case in cases:
        if case["verdict"] == "REGRESSION":
            reasons = "; ".join(case["regressions"])
            errors.append(f"REGRESSION {case['name']}: {reasons}")
        elif case["verdict"] == "SUSPICIOUS" and case["name"] not in suspicious:
            reasons = "; ".join(case["suspicions"])
            errors.append(
                f"SUSPICIOUS {case['name']} is not accepted: {reasons} "
                "(read the case, then list it with --accept-suspicious CASE=REASON)"
            )
    accepted = [
        case["name"]
        for case in cases
        if case["verdict"] == "EXPECTED" or case["name"] in suspicious
    ]
    return accepted + sorted(review["summary"]["new"]), errors


def commit_message(
    pr: int, title: str, review: dict, suspicious: dict[str, str]
) -> str:
    cases = review["cases"]
    counts = {
        verdict: sum(case["verdict"] == verdict for case in cases)
        for verdict in ("EXPECTED", "SUSPICIOUS")
    }
    subject = f"Accept snapshot baselines for #{pr}"[:SUBJECT_LIMIT]
    lines = [
        f"Review of {review['head'][:10]} against {review['base'][:10]} by "
        f'scripts/review_baselines.py for "{title}": {len(cases)} changed '
        f"cases, {counts['EXPECTED']} expected, {counts['SUSPICIOUS']} "
        f"suspicious accepted with a stated reason, 0 regressions; "
        f"{len(review['summary']['new'])} new cases."
    ]
    if review["expectations"]:
        lines.append("Expectations: " + " | ".join(review["expectations"]))
    for name, reason in sorted(suspicious.items()):
        lines.append(f"Suspicious, accepted: {name}: {reason}")
    body = "\n\n".join(
        textwrap.fill(line, BODY_WIDTH, subsequent_indent="  ")
        for line in lines
    )
    return f"{subject}\n\n{body}\n"


def pr_info(number: int) -> dict:
    return json.loads(
        pending.run(
            ["gh", "pr", "view", str(number), "--json", "headRefOid,title"]
        )
    )


def git_output(*args: str) -> str:
    return pending.run(["git", *args]).strip()


def require_pr_head_checked_out(info: dict) -> None:
    head = git_output("rev-parse", "HEAD")
    if head != info["headRefOid"]:
        raise AcceptError(
            f"HEAD {head[:10]} is not the PR head {info['headRefOid'][:10]}: "
            "run this in the PR branch's worktree, pushed and clean"
        )
    if git_output("status", "--porcelain", "--untracked-files=no"):
        raise AcceptError("the worktree has uncommitted changes")


def run_review(number: int) -> dict:
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "review.json"
        result = subprocess.run(
            [
                "uv",
                "run",
                "python",
                str(Path(__file__).with_name("review_baselines.py")),
                "--pr",
                str(number),
                "--json",
                str(out),
            ],
            cwd=ROOT,
            text=True,
        )
        if not out.exists():
            raise AcceptError(
                f"review_baselines.py failed ({result.returncode})"
            )
        return json.loads(out.read_text())


def write_baselines(cases: list[str]) -> None:
    env = {"CARGO_TARGET_DIR": str(pending.TARGET_DIR)}
    pending.run(
        ["make", "-f", "Makefile.rust", "motion-engine-fast"], ROOT, env
    )
    pending.run(
        ["uv", "run", "python", "snapshots/run.py", "--accept", *cases], ROOT
    )
    leftover = subprocess.run(
        ["uv", "run", "python", "snapshots/run.py"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    if leftover.returncode != 0:
        raise AcceptError(
            "snapshots are still not exact after accepting:\n"
            + leftover.stdout[-1500:]
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("pr", type=int)
    parser.add_argument(
        "--accept-suspicious",
        action="append",
        default=[],
        metavar="CASE=REASON",
    )
    parser.add_argument("--no-commit", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    suspicious = parse_suspicious(args.accept_suspicious)
    info = pr_info(args.pr)
    require_pr_head_checked_out(info)
    review = run_review(args.pr)
    cases, errors = decide(review, suspicious)
    if errors:
        print("\nrefusing to accept:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1
    message = commit_message(args.pr, info["title"], review, suspicious)
    if args.dry_run:
        print(f"\nwould accept {len(cases)} case(s):")
        print("\n".join(f"  {name}" for name in cases))
        print(f"\ncommit message:\n{message}")
        return 0
    if not cases:
        print("nothing to accept")
        return 0
    write_baselines(cases)
    if not args.no_commit:
        pending.run(["git", "add", "snapshots/baselines"], ROOT)
        pending.run(["git", "commit", "-q", "-m", message], ROOT)
        print(git_output("log", "--oneline", "-1"))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (AcceptError, pending.PendingBaselinesError) as exc:
        sys.exit(f"accept_reviewed: {exc}")
