"""Host side of the F407 step-capture instrument.

Protocol (little-endian), one frame per 128 snapshot ticks of 25 us:
    magic "SVC1" | seq u32 | first_tick u32 | ticks u16 | status u8 | detail u8
    | ticks * CHANNEL_COUNT signed step deltas (i8, + when DIR pin is high)
status != 0 is a latched instrument fault; detail is the channel.
"""

from __future__ import annotations

import argparse
import enum
import json
import os
import select
import struct
import termios
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path

DEVICE = "/dev/serial/by-id/usb-serval-bench_step-capture_f407-capture-if00"
CHANNEL_COUNT = 4
TICK_HZ = 40_000
FRAME_MAGIC = b"SVC1"
HEADER = struct.Struct("<4sIIHBB")
STALL_TIMEOUT_S = 1.0
CMD_START = b"S"
CMD_STOP = b"X"


class Fault(enum.IntEnum):
    DIR_OVERCAPTURE = 1
    DELTA_OVERFLOW = 2
    DIR_DESYNC = 3
    RING_OVERFLOW = 4


class CaptureError(RuntimeError):
    pass


@dataclass
class CaptureResult:
    ticks: int
    path: Path


class Capture:
    def __init__(
        self, out_dir: Path, both_edges_mask: int, device: str = DEVICE
    ):
        self._out_dir = out_dir
        self._mask = both_edges_mask
        self._device = device
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None
        self._ticks = 0

    def start(self) -> None:
        self._out_dir.mkdir(parents=True, exist_ok=True)
        fd = os.open(self._device, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(fd)
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, CMD_STOP + b"\0")
        self._drain(fd)
        os.write(fd, CMD_START + bytes([self._mask]))
        self._write_meta(
            started_monotonic=time.monotonic(), started_wall=time.time()
        )
        self._thread = threading.Thread(
            target=self._run, args=(fd,), daemon=True
        )
        self._thread.start()

    def stop(self) -> CaptureResult:
        self._stop.set()
        assert self._thread is not None
        self._thread.join()
        if self._error is not None:
            raise self._error
        return CaptureResult(self._ticks, self._out_dir / "capture.bin")

    def check(self) -> None:
        if self._error is not None:
            raise self._error

    def _drain(self, fd: int) -> None:
        while select.select([fd], [], [], 0.2)[0]:
            os.read(fd, 65536)

    def _write_meta(self, **extra) -> None:
        meta = {
            "tick_hz": TICK_HZ,
            "channel_count": CHANNEL_COUNT,
            "both_edges_mask": self._mask,
            **extra,
        }
        (self._out_dir / "capture.json").write_text(json.dumps(meta, indent=2))

    def _run(self, fd: int) -> None:
        try:
            with open(self._out_dir / "capture.bin", "wb") as sink:
                self._pump(fd, sink)
        except BaseException as exc:
            self._error = exc
        finally:
            os.write(fd, CMD_STOP + b"\0")
            os.close(fd)

    def _pump(self, fd: int, sink) -> None:
        buf = bytearray()
        expected_seq = 0
        last_data = time.monotonic()
        while not self._stop.is_set():
            if select.select([fd], [], [], 0.1)[0]:
                buf += os.read(fd, 65536)
            elif time.monotonic() - last_data > STALL_TIMEOUT_S:
                raise CaptureError(f"capture stalled after {self._ticks} ticks")
            while len(buf) >= HEADER.size:
                magic, seq, first_tick, ticks, status, detail = (
                    HEADER.unpack_from(buf)
                )
                if magic != FRAME_MAGIC:
                    raise CaptureError(
                        f"bad frame magic {magic!r} at tick {self._ticks}"
                    )
                frame_len = HEADER.size + ticks * CHANNEL_COUNT
                if len(buf) < frame_len:
                    break
                if seq != expected_seq:
                    raise CaptureError(
                        f"frame seq gap: got {seq}, expected {expected_seq}"
                    )
                if first_tick != self._ticks:
                    raise CaptureError(
                        f"tick gap: got {first_tick}, expected {self._ticks}"
                    )
                if status:
                    raise CaptureError(
                        f"instrument fault {Fault(status).name} on channel {detail} at tick {first_tick}"
                    )
                sink.write(buf[HEADER.size : frame_len])
                del buf[:frame_len]
                expected_seq += 1
                self._ticks += ticks
                last_data = time.monotonic()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record STEP/DIR deltas from the F407 capture board"
    )
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--seconds", type=float, required=True)
    parser.add_argument(
        "--both-edges-mask", type=lambda s: int(s, 0), default=0
    )
    args = parser.parse_args()
    capture = Capture(args.out_dir, args.both_edges_mask)
    capture.start()
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        capture.check()
        time.sleep(0.1)
    result = capture.stop()
    print(f"{result.ticks} ticks -> {result.path}")


if __name__ == "__main__":
    main()
