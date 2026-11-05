#!/usr/bin/env python3
"""Run `gh` under the repo's one rate-limit policy.

    python3 scripts/gh_util.py pr checks 12 --watch --interval 60
    python3 scripts/gh_util.py --git push -u origin my-branch

Same arguments as gh, same stdout and exit code. Many agents poll GitHub at
once and trip its secondary rate limit; `gh` then fails with "API rate limit
exceeded", which a caller cannot tell apart from a genuinely red check. This
wrapper retries only on a rate-limit or transient network failure (dropped
connection, 502-504), waiting Retry-After when GitHub
sent it and otherwise exponential backoff with jitter, and gives up loudly
after MAX_ATTEMPTS. Any other failure is returned untouched on the first try.
Long waits belong in one `--watch` call, not in a loop of short polls.
"""

from __future__ import annotations

import random
import re
import subprocess
import sys
import time

MAX_ATTEMPTS = 8
BASE_BACKOFF_SECONDS = 30.0
MAX_BACKOFF_SECONDS = 300.0
JITTER_FRACTION = 0.25
POLL_INTERVAL_SECONDS = 60
RATE_LIMIT_MARKERS = (
    "rate limit",
    "abuse detection",
    "HTTP 429",
    "check your internet connection",
    "githubstatus.com",
    "connection reset",
    "i/o timeout",
    "TLS handshake timeout",
    "HTTP 502",
    "HTTP 503",
    "HTTP 504",
)
RETRY_AFTER = re.compile(r"retry[- ]after:?\s*(\d+)", re.IGNORECASE)


GIT_MAX_ATTEMPTS = 5
GIT_BASE_BACKOFF_SECONDS = 5.0
GIT_MAX_BACKOFF_SECONDS = 60.0
GIT_NETWORK_COMMANDS = ("fetch", "push", "pull", "ls-remote", "clone")
GIT_TRANSIENT_MARKERS = (
    "internal server error",
    "the requested url returned error: 5",
    "http 5",
    "connection reset",
    "connection timed out",
    "could not resolve host",
    "early eof",
    "rpc failed",
    "tls handshake timeout",
    "i/o timeout",
    "unexpected disconnect",
    "the remote end hung up unexpectedly",
    "rate limit",
)


class GhRateLimitError(RuntimeError):
    pass


def is_transient_git_failure(stderr: str) -> bool:
    """Server and network trouble only. 'Permission denied', 'Authentication
    failed', 'non-fast-forward' and 'rejected' without a server error stay
    failures the caller must see at once."""
    lowered = stderr.lower()
    return any(marker in lowered for marker in GIT_TRANSIENT_MARKERS)


def git_backoff_seconds(attempt: int, rng=random.random) -> float:
    base = min(GIT_BASE_BACKOFF_SECONDS * 2**attempt, GIT_MAX_BACKOFF_SECONDS)
    return base * (1 + JITTER_FRACTION * (2 * rng() - 1))


def is_rate_limited(stderr: str) -> bool:
    lowered = stderr.lower()
    return any(marker.lower() in lowered for marker in RATE_LIMIT_MARKERS)


def retry_after_seconds(stderr: str) -> float | None:
    found = RETRY_AFTER.search(stderr)
    return float(found.group(1)) if found else None


def backoff_seconds(
    attempt: int, retry_after: float | None, rng=random.random
) -> float:
    if retry_after is not None:
        return retry_after
    base = min(BASE_BACKOFF_SECONDS * 2**attempt, MAX_BACKOFF_SECONDS)
    return base * (1 + JITTER_FRACTION * (2 * rng() - 1))


def run_streaming(args: list[str]) -> tuple[int, str]:
    process = subprocess.Popen(["gh", *args], stderr=subprocess.PIPE, text=True)
    seen = []
    for line in process.stderr:
        sys.stderr.write(line)
        seen.append(line)
    return process.wait(), "".join(seen)


def git_subcommand(args: list[str]) -> str | None:
    skip = False
    for arg in args:
        if skip:
            skip = False
        elif arg in ("-C", "-c"):
            skip = True
        elif not arg.startswith("-"):
            return arg
    return None


def run_git_streaming(args: list[str]) -> tuple[int, str]:
    process = subprocess.Popen(
        ["git", *args], stderr=subprocess.PIPE, text=True
    )
    seen = []
    for line in process.stderr:
        sys.stderr.write(line)
        seen.append(line)
    return process.wait(), "".join(seen)


def run_git(args: list[str], runner=run_git_streaming, sleep=time.sleep) -> int:
    """git, retried on transient server and network failures when the
    subcommand talks to the remote; every other failure, and every other
    subcommand, is returned on the first try."""
    retryable = git_subcommand(args) in GIT_NETWORK_COMMANDS
    for attempt in range(GIT_MAX_ATTEMPTS if retryable else 1):
        code, stderr = runner(args)
        if code == 0 or not retryable or not is_transient_git_failure(stderr):
            return code
        if attempt + 1 == GIT_MAX_ATTEMPTS:
            break
        wait = git_backoff_seconds(attempt)
        print(
            f"gh_util: transient git failure (attempt {attempt + 1}/{GIT_MAX_ATTEMPTS}); "
            f"waiting {wait:.0f}s before `git {' '.join(args[:3])}`",
            file=sys.stderr,
        )
        sleep(wait)
    raise GhRateLimitError(
        f"git still failing transiently after {GIT_MAX_ATTEMPTS} attempts: git {' '.join(args)}"
    )


def run_gh(args: list[str], runner=run_streaming, sleep=time.sleep) -> int:
    for attempt in range(MAX_ATTEMPTS):
        code, stderr = runner(args)
        if code == 0 or not is_rate_limited(stderr):
            return code
        wait = backoff_seconds(attempt, retry_after_seconds(stderr))
        print(
            f"gh_util: transient failure (attempt {attempt + 1}/{MAX_ATTEMPTS}); "
            f"waiting {wait:.0f}s before `gh {' '.join(args[:3])}`",
            file=sys.stderr,
        )
        sleep(wait)
    raise GhRateLimitError(
        f"still failing transiently after {MAX_ATTEMPTS} attempts: gh {' '.join(args)}"
    )


def main() -> int:
    try:
        if sys.argv[1:2] == ["--git"]:
            return run_git(sys.argv[2:])
        return run_gh(sys.argv[1:])
    except GhRateLimitError as exc:
        sys.exit(f"gh_util: {exc}")


if __name__ == "__main__":
    sys.exit(main())
