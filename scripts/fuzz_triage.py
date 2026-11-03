#!/usr/bin/env python3
"""Tell a pre-existing property-test failure from a PR's own regression.

    python3 scripts/fuzz_triage.py parse < log
    python3 scripts/fuzz_triage.py triage BRANCH RUN_ID

rust-fuzz runs the proptest suites with a fresh seed, so a bug already on
main fails unrelated PRs at random. `parse` prints one machine-readable line
per failing test from a nextest log:

    FUZZ_FAILURE {"binary": "pkg::bin", "test": "t", "seed": "<hex>", "location": "file.rs:12:3", "message": "..."}

`triage` is what land-pr.sh runs on a red run. It requires every failed job
to be a rust-fuzz job with a seed, replays each seed against origin/main in
a scratch worktree (its own target dir), and then:

- the seed also fails on main: the bug is pre-existing; an issue keyed by
  test and panic location is found or opened (a label-scoped search, no
  duplicates), the seed is commented once, the failed job is rerun, exit 0;
- the seed passes on main and the PR contains origin/main: the PR broke it;
  the repro command is printed, exit 10;
- the seed passes on main and the PR is behind it: main may already have the
  fix; merge main and rerun, exit 10;
- anything else (another job red, a timeout with no seed, an unlocatable
  test): exit 11, land-pr.sh fails as before.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import pending_baselines as pending

ANSI = re.compile(r"\x1b\[[0-9;]*m|\^\[\[[0-9;]*m")
HEADER = re.compile(
    r"^\s*(?:FAIL|TIMEOUT|SIGABRT|SIGSEGV)\s+\[[^\]]*\]\s+\(\s*\d+/\s*\d+\)\s+(\S+)\s+(\S+)\s*$"
)
SEED = re.compile(r"^\s*cc ([0-9a-f]{64})\b")
RNG_SEED = re.compile(r"PROPTEST_RNG_SEED=(\d+)")
PERSISTENCE_PATH = re.compile(r'"(proptest-regressions/[^"]+\.txt)"')
PIPELINE_BINARIES = ("seam_schedule_fuzz", "planner_fuzz")
PIPELINE_CASES = "312"
PANIC = re.compile(r"thread '([^']+)' panicked at ([^\s]+?):\s*$")
FUZZ_JOB_PREFIX = "rust-fuzz"
LABEL = "fuzz-triage"
KEY_MARKER = "fuzz-triage-key:"
RERUN_EXIT, REGRESSION_EXIT, UNTRIAGEABLE_EXIT = 0, 10, 11
REPRO_PROPTEST_CASES = "1"
SCRATCH = pending.CACHE / "fuzz-triage"


@dataclass(frozen=True)
class Failure:
    binary: str
    test: str
    seed: str | None
    location: str | None
    message: str | None

    @property
    def key(self) -> str:
        return f"{self.binary}::{self.test}@{self.location}"

    @property
    def package(self) -> str:
        return self.binary.split("::")[0]

    @property
    def target(self) -> str:
        return self.binary.split("::")[-1]


def clean(text: str) -> list[str]:
    return [ANSI.sub("", line) for line in text.splitlines()]


def parse_log(text: str) -> list[Failure]:
    lines = clean(text)
    marked = [
        Failure(**json.loads(line.split("FUZZ_FAILURE", 1)[1]))
        for line in lines
        if "FUZZ_FAILURE {" in line
    ]
    if marked:
        return marked
    headers = [
        (index, match.group(1), match.group(2))
        for index, line in enumerate(lines)
        if (match := HEADER.match(line))
    ]
    failures: dict[tuple[str, str], Failure] = {}
    for position, (index, binary, test) in enumerate(headers):
        if (binary, test) in failures:
            continue
        end = (
            headers[position + 1][0]
            if position + 1 < len(headers)
            else len(lines)
        )
        block = lines[index:end]
        seed = next(
            (f"cc:{m.group(1)}" for line in block if (m := SEED.match(line))),
            None,
        ) or next(
            (
                f"rng:{m.group(1)}"
                for line in block
                if (m := RNG_SEED.search(line))
            ),
            None,
        )
        panics = [
            (
                m.group(2),
                block[offset + 1].strip() if offset + 1 < len(block) else "",
            )
            for offset, line in enumerate(block)
            if (m := PANIC.search(line)) and m.group(1) == test
        ]
        inner = [p for p in panics if "/tests/" not in p[0]]
        location, message = (inner or panics or [(None, None)])[0]
        failures[(binary, test)] = Failure(
            binary, test, seed, location, message
        )
    return list(failures.values())


def format_line(failure: Failure) -> str:
    return "FUZZ_FAILURE " + json.dumps(asdict(failure), sort_keys=True)


def find_issue(issues: list[dict], key: str) -> dict | None:
    marker = f"{KEY_MARKER} {key}"
    return next((i for i in issues if marker in (i.get("body") or "")), None)


def issue_body(failure: Failure, run_url: str, branch: str) -> str:
    return (
        f"{KEY_MARKER} {failure.key}\n\n"
        f"A fresh-seed property test fails on main, found while landing `{branch}`.\n\n"
        f"- test: `{failure.binary}` `{failure.test}`\n"
        f"- panic: `{failure.location}`: {failure.message}\n"
        f"- seed: `{failure.seed}`\n"
        f"- run: {run_url}\n\n"
        f"Repro: {seed_instructions(failure)}"
    )


def cases_for(failure: Failure) -> str:
    return (
        PIPELINE_CASES
        if failure.target in PIPELINE_BINARIES
        else REPRO_PROPTEST_CASES
    )


def seed_kind(failure: Failure) -> tuple[str, str]:
    kind, _, value = (failure.seed or ":").partition(":")
    return kind, value


def repro_command(failure: Failure) -> str:
    kind, value = seed_kind(failure)
    env = f"PROPTEST_RNG_SEED={value} " if kind == "rng" else ""
    return (
        f"{env}PROPTEST_CASES={cases_for(failure)} cargo nextest run -p {failure.package} "
        f"--test {failure.target} -E 'test(={failure.test})'"
    )


def seed_instructions(failure: Failure) -> str:
    kind, value = seed_kind(failure)
    if kind == "rng":
        return f"`{repro_command(failure)}`"
    return (
        f"add `cc {value}` to the suite's `proptest-regressions/` file, then "
        f"`{repro_command(failure)}`"
    )


def gh(*args: str) -> str:
    return pending.run(["gh", *args])


def failed_jobs(run_id: str) -> list[str]:
    jobs = json.loads(gh("run", "view", run_id, "--json", "jobs"))["jobs"]
    return [j["name"] for j in jobs if j["conclusion"] == "failure"]


def scratch_main(rev: str = "origin/main") -> Path:
    pending.run(["git", "fetch", "-q", "origin"])
    sha = pending.run(["git", "rev-parse", rev]).strip()
    tree = SCRATCH / f"main-{sha[:10]}"
    if not tree.exists():
        SCRATCH.mkdir(parents=True, exist_ok=True)
        pending.run(["git", "worktree", "prune"])
        pending.run(
            ["git", "worktree", "add", "-q", "--detach", str(tree), sha]
        )
    pending.run(["git", "checkout", "-q", "--", "rust"], tree)
    subprocess.run(
        ["git", "clean", "-fdq", "--", "rust/*/proptest-regressions"], cwd=tree
    )
    return tree


def regression_file(tree: Path, failure: Failure) -> Path | None:
    found = sorted(tree.glob(f"rust/*/tests/{failure.target}.rs"))
    if len(found) != 1:
        return None
    declared = PERSISTENCE_PATH.search(found[0].read_text())
    return found[0].parents[1] / declared.group(1) if declared else None


def fails_on_main(failure: Failure, rev: str = "origin/main") -> bool | None:
    tree = scratch_main(rev)
    kind, value = seed_kind(failure)
    env = {**os.environ, "PROPTEST_CASES": cases_for(failure)}
    if kind == "rng":
        env["PROPTEST_RNG_SEED"] = value
    else:
        path = regression_file(tree, failure)
        if path is None:
            return None
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a") as handle:
            handle.write(f"cc {value} # fuzz_triage replay\n")
    result = subprocess.run(
        [
            "cargo",
            "nextest",
            "run",
            "-p",
            failure.package,
            "--test",
            failure.target,
            "--profile",
            "ci",
            "-E",
            f"test(={failure.test})",
        ],
        cwd=tree / "rust",
        env=env,
        capture_output=True,
        text=True,
    )
    return result.returncode != 0


def report_pre_existing(failure: Failure, run_id: str, branch: str) -> None:
    issues = json.loads(
        gh(
            "issue",
            "list",
            "--label",
            LABEL,
            "--state",
            "open",
            "--limit",
            "100",
            "--json",
            "number,body",
        )
    )
    found = find_issue(issues, failure.key)
    run_url = f"https://github.com/{repo_slug()}/actions/runs/{run_id}"
    if found is None:
        try:
            gh(
                "label",
                "create",
                LABEL,
                "--color",
                "D93F0B",
                "--description",
                "fresh-seed property test failing on main",
            )
        except pending.PendingBaselinesError:
            pass
        url = gh(
            "issue",
            "create",
            "--label",
            LABEL,
            "--title",
            f"fuzz: {failure.test} panics at {failure.location}",
            "--body",
            issue_body(failure, run_url, branch),
        ).strip()
        print(f"fuzz_triage: opened {url}", file=sys.stderr)
        return
    thread = json.loads(
        gh("issue", "view", str(found["number"]), "--json", "body,comments")
    )
    seen = " ".join([thread["body"]] + [c["body"] for c in thread["comments"]])
    if seed_kind(failure)[1] in seen:
        return
    gh(
        "issue",
        "comment",
        str(found["number"]),
        "--body",
        f"Seen again while landing `{branch}`: seed `{failure.seed}`, run {run_url}.",
    )
    print(f"fuzz_triage: commented on #{found['number']}", file=sys.stderr)


def repo_slug() -> str:
    return gh(
        "repo", "view", "--json", "nameWithOwner", "--jq", ".nameWithOwner"
    ).strip()


def contains_main(branch: str) -> bool:
    return (
        subprocess.run(
            [
                "git",
                "merge-base",
                "--is-ancestor",
                "origin/main",
                f"origin/{branch}",
            ],
            cwd=pending.ROOT,
        ).returncode
        == 0
    )


def triage(branch: str, run_id: str) -> int:
    names = failed_jobs(run_id)
    if not names or not all(n.startswith(FUZZ_JOB_PREFIX) for n in names):
        print(
            f"fuzz_triage: failed jobs are not all rust-fuzz: {names}",
            file=sys.stderr,
        )
        return UNTRIAGEABLE_EXIT
    log = gh("run", "view", run_id, "--log-failed")
    failures = parse_log(log)
    if not failures or any(
        f.seed is None or f.location is None for f in failures
    ):
        print(
            "fuzz_triage: a failure has no seed or panic location",
            file=sys.stderr,
        )
        return UNTRIAGEABLE_EXIT
    for failure in failures:
        on_main = fails_on_main(failure)
        if on_main is None:
            print(
                f"fuzz_triage: cannot place {failure.binary}", file=sys.stderr
            )
            return UNTRIAGEABLE_EXIT
        if not on_main:
            behind = not contains_main(branch)
            print(
                f"fuzz_triage: seed {failure.seed} passes on origin/main"
                + (
                    "; the branch is behind main, which may already fix it: merge main and rerun"
                    if behind
                    else f"; this PR broke {failure.binary} {failure.test}"
                )
                + f"\n  repro: add 'cc {failure.seed}' to the crate's proptest-regressions/"
                f"{failure.target}.txt, then {repro_command(failure)}",
                file=sys.stderr,
            )
            return REGRESSION_EXIT
        report_pre_existing(failure, run_id, branch)
    gh("run", "rerun", run_id, "--failed")
    return RERUN_EXIT


def main() -> int:
    if sys.argv[1:2] == ["parse"]:
        for failure in parse_log(sys.stdin.read()):
            print(format_line(failure))
        return 0
    if sys.argv[1:2] == ["triage"] and len(sys.argv) == 4:
        return triage(sys.argv[2], sys.argv[3])
    sys.exit(__doc__)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except pending.PendingBaselinesError as exc:
        sys.exit(f"fuzz_triage: {exc}")
