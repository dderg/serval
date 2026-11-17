#!/usr/bin/env python3
"""FIRMWARE_RESTART resets a Linux MCU even when it reads klippy's
emergency_stop and config_reset in one chunk (#879).

    python3 scripts/linux_mcu_restart_check.py out/klipper.elf out/klipper.dict

Runs the real (MCU_SIM=n) Linux MCU on a pty, writes both frames with one
write(), and exits non-zero unless the process re-executes, which replaces
its pty: config_reset execs the binary anew."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "klippy"))
import msgproto  # noqa: E402

START_TIMEOUT_S = 5.0
RESET_TIMEOUT_S = 5.0
POLL_S = 0.02


def frame(parser: msgproto.MessageParser, seq: int, command: str) -> bytes:
    body = parser.create_command(command)
    head = [msgproto.MESSAGE_MIN + len(body), msgproto.MESSAGE_DEST | seq]
    block = head + body
    return bytes(block + msgproto.crc16_ccitt(block) + [msgproto.MESSAGE_SYNC])


def wait_for(predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(POLL_S)
    return predicate()


def main(elf: str, dictionary: str) -> int:
    parser = msgproto.MessageParser()
    parser.process_identify(Path(dictionary).read_bytes(), decompress=False)
    restart = frame(parser, 0, "emergency_stop") + frame(
        parser, 1, "config_reset"
    )
    with tempfile.TemporaryDirectory() as tmp:
        link = os.path.join(tmp, "mcu")
        mcu = subprocess.Popen([os.path.abspath(elf), "-I", link])
        try:
            if not wait_for(lambda: os.path.islink(link), START_TIMEOUT_S):
                print(f"FAIL: {elf} never created its pty {link}")
                return 1
            first_pty = os.readlink(link)
            fd = os.open(link, os.O_RDWR | os.O_NOCTTY)
            tty.setraw(fd)
            os.write(fd, restart)
            reset = wait_for(
                lambda: os.path.islink(link) and os.readlink(link) != first_pty,
                RESET_TIMEOUT_S,
            )
            os.close(fd)
            if not reset:
                print(
                    "FAIL: emergency_stop + config_reset in one write left"
                    f" the MCU on {first_pty}: config_reset was not executed"
                )
                return 1
            print(
                f"OK: the MCU re-executed ({first_pty} -> {os.readlink(link)})"
            )
            return 0
        finally:
            mcu.kill()
            mcu.wait()


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
