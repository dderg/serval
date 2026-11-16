#!/usr/bin/env python3
"""Prove the sim image never ships a stale native artifact.

Builds tools/sim/Dockerfile three times from this tree's files, streamed as a
tar context so the worktree is never edited: with a fresh token compiled into
a host crate and into a firmware crate, with a second token, and untouched.
Each image's _motion_engine.so and firmware ELFs must hold exactly the
tokens of the sources it was built from.

    python3 tools/sim/check_fresh.py      (or ./scripts/ci.sh sim-fresh)
"""

from __future__ import annotations

import dataclasses
import io
import pathlib
import secrets
import subprocess
import sys
import tarfile
import tempfile

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DOCKERFILE = "tools/sim/Dockerfile"
IMAGE_TAG = f"kalico-sim-fresh-{REPO_ROOT.name.lower()}"
GREP_NO_MATCH = 1


@dataclasses.dataclass(frozen=True)
class Probe:
    source: str
    fn_signature: str
    artifacts: tuple[str, ...]


PROBES = (
    Probe(
        "rust/motion-core/src/worker.rs",
        "pub fn setup_pipeline(",
        ("/kalico/klippy/_motion_engine.so",),
    ),
    Probe(
        "rust/runtime/src/state.rs",
        "pub unsafe fn init(rt_ptr: *mut RuntimeContext)",
        ("/kalico/out/klipper-h7-sim.elf", "/kalico/out/klipper-f4-sim.elf"),
    ),
)


def with_token(text: str, probe: Probe, token: str) -> str:
    lines = text.splitlines(keepends=True)
    starts = [i for i, line in enumerate(lines) if probe.fn_signature in line]
    if len(starts) != 1:
        raise SystemExit(
            f"check_fresh: expected one `{probe.fn_signature}` in "
            f"{probe.source}, found {len(starts)}; update PROBES"
        )
    body = next(
        i
        for i in range(starts[0], len(lines))
        if lines[i].rstrip().endswith("{")
    )
    lines.insert(body + 1, f'    core::hint::black_box("{token}");\n')
    return "".join(lines)


def tree_files() -> list[str]:
    listing = subprocess.run(
        ["git", "ls-files", "-z", "-co", "--exclude-standard"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
    ).stdout.decode()
    return sorted(
        {
            name
            for name in listing.split("\0")
            if name and (REPO_ROOT / name).exists()
        }
    )


def stream_context(stdin, token: str | None) -> None:
    probed = {probe.source: probe for probe in PROBES} if token else {}
    with tarfile.open(
        fileobj=stdin, mode="w|", format=tarfile.GNU_FORMAT
    ) as tar:
        for name in tree_files():
            path = REPO_ROOT / name
            if name not in probed:
                tar.add(path, arcname=name, recursive=False)
                continue
            data = with_token(path.read_text(), probed[name], token).encode()
            info = tar.gettarinfo(path, arcname=name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))


def build(token: str | None) -> None:
    with tempfile.TemporaryFile() as log:
        proc = subprocess.Popen(
            ["docker", "build", "-t", IMAGE_TAG, "-f", DOCKERFILE, "-"],
            stdin=subprocess.PIPE,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            stream_context(proc.stdin, token)
            proc.stdin.close()
        except BrokenPipeError:
            pass
        rc = proc.wait()
        if rc != 0:
            log.seek(0)
            sys.stdout.write(log.read().decode(errors="replace")[-20000:])
            raise SystemExit(f"check_fresh: docker build failed (rc={rc})")


def holds(artifact: str, token: str) -> bool:
    rc = subprocess.run(
        ["docker", "run", "--rm", "--entrypoint", "grep", IMAGE_TAG]
        + ["-q", "-a", "-F", token, artifact],
    ).returncode
    if rc not in (0, GREP_NO_MATCH):
        raise SystemExit(f"check_fresh: grep {artifact} rc={rc}")
    return rc == 0


def expect(present: str | None, absent: list[str]) -> list[str]:
    failures = []
    for artifact in (a for probe in PROBES for a in probe.artifacts):
        if present and not holds(artifact, present):
            failures.append(f"{artifact}: missing {present} (stale)")
        failures += [
            f"{artifact}: still holds {token} (stale)"
            for token in absent
            if holds(artifact, token)
        ]
    return failures


def main() -> int:
    first = f"sim-fresh-{secrets.token_hex(8)}"
    second = f"sim-fresh-{secrets.token_hex(8)}"
    rounds = (
        (first, []),
        (second, [first]),
        (None, [first, second]),
    )
    failures = []
    for token, absent in rounds:
        build(token)
        round_failures = expect(token, absent)
        state = "STALE" if round_failures else "fresh"
        print(f"{token or 'untouched'} (absent: {absent}): {state}")
        failures += round_failures
    subprocess.run(
        ["docker", "rmi", IMAGE_TAG], check=True, capture_output=True
    )
    for failure in failures:
        print(f"  {failure}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
