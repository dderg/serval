"""Stock Kalico F401 firmware built from a clean kv2 commit.

The build runs in a detached worktree of the commit (FW_WORKTREE), so the
version the MCU reports is that commit's `git describe` with no -dirty
suffix, and the dictionary it writes is the one the batch reference plans
with. The kconfig is tools/kv2_bench/f401.config (STM32F401, 25 MHz crystal,
no bootloader, USB on PA11/PA12, as serval's tools/bench/f401.config).
"""

from __future__ import annotations

import dataclasses
import json
import shutil
import subprocess
from pathlib import Path

from . import bench

FW_WORKTREE = Path("~/Developer/kv2-bench-fwsrc").expanduser()
KCONFIG = Path(__file__).with_name("f401.config")
SLOTS = Path("~/Developer/serval/scripts/slots.py").expanduser()
OUT = "out/kv2-f401"


@dataclasses.dataclass(frozen=True)
class Firmware:
    commit: str
    elf: Path
    dictionary: Path
    version: str

    def record(self) -> dict:
        return {
            "commit": self.commit,
            "elf": str(self.elf),
            "elf_sha256": bench.sha256(self.elf),
            "dictionary_sha256": bench.sha256(self.dictionary),
            "version": self.version,
        }


def checkout(path: Path, commit: str) -> None:
    if not path.exists():
        bench.git("worktree", "add", "--detach", str(path), commit)
    status = bench.git(
        "status", "--porcelain", "--untracked-files=no", cwd=path
    )
    if status:
        raise bench.BenchError(f"{path} has local changes:\n{status}")
    bench.git("checkout", "--detach", "-q", commit, cwd=path)


def build(commit: str) -> Firmware:
    checkout(FW_WORKTREE, commit)
    out = FW_WORKTREE / OUT
    out.mkdir(parents=True, exist_ok=True)
    kconfig = out / "f401.config"
    shutil.copyfile(KCONFIG, kconfig)
    (out / "compile_time_request.o").unlink(missing_ok=True)
    make = ["make", f"OUT={out}/", f"KCONFIG_CONFIG={kconfig}"]
    for args in ([*make, "olddefconfig"], [*make, "-j8"]):
        subprocess.run(
            ["python3", str(SLOTS), "run", "--", *args],
            cwd=FW_WORKTREE,
            check=True,
            capture_output=True,
        )
    dictionary = out / "klipper.dict"
    version = json.loads(dictionary.read_text())["version"]
    abbrev = version.rsplit("-g", 1)[-1]
    if (
        version.endswith("-dirty")
        or len(abbrev) < 7
        or not commit.startswith(abbrev)
    ):
        raise bench.BenchError(
            f"firmware version {version!r} does not name clean commit {commit}"
        )
    return Firmware(commit, out / "klipper.elf", dictionary, version)
