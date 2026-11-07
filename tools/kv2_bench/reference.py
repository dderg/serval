"""Offline reference: the step stream klippy batch mode plans for a program.

Runs `tools.kv2_bench.batch_capture` (motion_eval's capture plus the
extruder) against the dictionary of the very firmware the bench MCU runs, so
CLOCK_FREQ and the command set match, and decodes the queue_step stream into
absolute step clocks and directions per stepper (motion_eval's stepdecode,
which checks every wire message against the host's step history).
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

from klippy import msgproto
from tools.motion_eval import stepdecode

from .program import Program

KV2_ROOT = Path(__file__).resolve().parents[2]
SERIAL_LINE = re.compile(r"^serial:.*$", flags=re.MULTILINE)
LOG_TAIL_CHARS = 6000


class ReferenceError(RuntimeError):
    pass


@dataclasses.dataclass
class Steps:
    clocks: np.ndarray
    signs: np.ndarray

    @property
    def net(self) -> int:
        return int(self.signs.sum(dtype=np.int64))


@dataclasses.dataclass
class Reference:
    steppers: dict[str, Steps]
    clock_freq: float
    meta: dict

    def save(self, out_dir: Path) -> None:
        arrays = {}
        for name, steps in self.steppers.items():
            arrays[f"{name}.clocks"] = steps.clocks
            arrays[f"{name}.signs"] = steps.signs
        np.savez_compressed(out_dir / "reference.npz", **arrays)
        (out_dir / "reference.json").write_text(
            json.dumps({"clock_freq": self.clock_freq, **self.meta}, indent=2)
        )

    @classmethod
    def load(cls, out_dir: Path) -> Reference:
        meta = json.loads((out_dir / "reference.json").read_text())
        clock_freq = meta.pop("clock_freq")
        with np.load(out_dir / "reference.npz") as data:
            names = sorted({key.rsplit(".", 1)[0] for key in data.files})
            steppers = {
                name: Steps(data[f"{name}.clocks"], data[f"{name}.signs"])
                for name in names
            }
        return cls(steppers, clock_freq, meta)


def batch_config(live_config: str) -> str:
    replaced, count = SERIAL_LINE.subn("serial: /dev/null", live_config)
    if count != 1:
        raise ReferenceError(
            f"config must have exactly one serial line, found {count}"
        )
    return replaced


def git_head(tree: Path) -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=tree,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def message_digest(dictionary: Path, output: Path) -> tuple[str, int]:
    """sha256 over every decoded host->MCU message in order. The raw file is
    not comparable between runs: how serialqueue packs messages into blocks
    depends on thread timing."""
    parser = msgproto.MessageParser()
    parser.process_identify(dictionary.read_bytes(), decompress=False)
    data = bytearray(output.read_bytes())
    digest = hashlib.sha256()
    count = pos = 0
    while pos < len(data):
        length = parser.check_packet(data[pos : pos + msgproto.MESSAGE_MAX])
        if length <= 0:
            raise ReferenceError(f"corrupt packet at byte {pos} of {output}")
        for params in stepdecode._parse_block(parser, data[pos : pos + length]):
            digest.update(repr(sorted(params.items())).encode())
            count += 1
        pos += length
    return digest.hexdigest(), count


def build(
    klippy_tree: Path,
    live_config: Path,
    program: Program,
    dictionary: Path,
    out_dir: Path,
) -> Reference:
    out_dir.mkdir(parents=True, exist_ok=True)
    config_path = out_dir / "batch.cfg"
    config_path.write_text(batch_config(live_config.read_text()))
    gcode_path = out_dir / "program.gcode"
    gcode_path.write_text(program.text())
    output_path = out_dir / "mcu.out"
    log_path = out_dir / "batch_klippy.log"
    capture_path = out_dir / "batch_capture.json"
    cmd = [
        sys.executable,
        "-m",
        "tools.kv2_bench.batch_capture",
        str(capture_path),
        str(config_path),
        "-i",
        str(gcode_path),
        "-o",
        str(output_path),
        "-d",
        str(dictionary),
        "-l",
        str(log_path),
    ]
    env = dict(os.environ, PYTHONPATH=f"{klippy_tree}:{KV2_ROOT}")
    result = subprocess.run(
        cmd, cwd=klippy_tree, env=env, capture_output=True, text=True
    )
    if result.returncode or not capture_path.exists():
        log = log_path.read_text() if log_path.exists() else ""
        raise ReferenceError(
            f"batch klippy failed (exit {result.returncode})\n--- stderr ---\n"
            f"{result.stderr[-LOG_TAIL_CHARS:]}\n--- klippy.log tail ---\n"
            f"{log[-LOG_TAIL_CHARS:]}"
        )
    capture = json.loads(capture_path.read_text())
    steppers = capture["steppers"]
    foreign = sorted(n for n, s in steppers.items() if s["mcu"] != "mcu")
    if foreign:
        raise ReferenceError(f"steppers on secondary MCUs: {foreign}")
    freqs = {s["clock_freq"] for s in steppers.values()}
    if len(freqs) != 1:
        raise ReferenceError(f"steppers disagree on CLOCK_FREQ: {freqs}")
    by_oid = stepdecode.decode(
        dictionary,
        output_path,
        {
            s["oid"]: (s["invert_dir"], s["step_history"])
            for s in steppers.values()
        },
    )
    decoded = {}
    for name, stepper in steppers.items():
        trace = by_oid[stepper["oid"]]
        signs = np.diff(trace.positions, prepend=0).astype(np.int8)
        if len(signs) and not np.all(np.abs(signs) == 1):
            raise ReferenceError(f"{name}: decoded positions skip steps")
        decoded[name] = Steps(trace.clocks.astype(np.int64), signs)
    messages_sha256, messages = message_digest(dictionary, output_path)
    meta = {
        "klippy_tree": str(klippy_tree),
        "klippy_head": git_head(klippy_tree),
        "config": str(live_config),
        "config_sha256": sha256(live_config),
        "program": program.name,
        "program_lines": len(program.lines),
        "dictionary": str(dictionary),
        "dictionary_sha256": sha256(dictionary),
        "messages_sha256": messages_sha256,
        "messages": messages,
        "moves": len(capture["moves"]),
        "settings": capture["settings"],
        "corner_deviation": capture["corner_deviation"],
        "step_dist": {n: s["step_dist"] for n, s in steppers.items()},
        "invert_dir": {n: s["invert_dir"] for n, s in steppers.items()},
    }
    reference = Reference(decoded, freqs.pop(), meta)
    reference.save(out_dir)
    return reference
