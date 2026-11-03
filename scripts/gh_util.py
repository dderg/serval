#!/usr/bin/env python3
"""Run `gh` under the repo's one rate-limit policy.

    python3 scripts/gh_util.py pr checks 12 --watch --interval 60

Same arguments as gh, same stdout and exit code. Many agents poll GitHub at
once and trip its secondary rate limit; `gh` then fails with "API rate limit
exceeded", which a caller cannot tell apart from a genuinely red check. This
wrapper retries only on a rate-limit failure, waiting Retry-After when GitHub
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
RATE_LIMIT_MARKERS = ("rate limit", "abuse detection", "HTTP 429")
RETRY_AFTER = re.compile(r"retry[- ]after:?\s*(\d+)", re.IGNORECASE)


class GhRateLimitError(RuntimeError):
    pass


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


def run_gh(args: list[str], runner=run_streaming, sleep=time.sleep) -> int:
    for attempt in range(MAX_ATTEMPTS):
        code, stderr = runner(args)
        if code == 0 or not is_rate_limited(stderr):
            return code
        wait = backoff_seconds(attempt, retry_after_seconds(stderr))
        print(
            f"gh_util: rate limited (attempt {attempt + 1}/{MAX_ATTEMPTS}); "
            f"waiting {wait:.0f}s before `gh {' '.join(args[:3])}`",
            file=sys.stderr,
        )
        sleep(wait)
    raise GhRateLimitError(
        f"still rate limited after {MAX_ATTEMPTS} attempts: gh {' '.join(args)}"
    )


def main() -> int:
    try:
        return run_gh(sys.argv[1:])
    except GhRateLimitError as exc:
        sys.exit(f"gh_util: {exc}")


if __name__ == "__main__":
    sys.exit(main())
