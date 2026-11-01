"""Measure sustained USB CDC throughput from the F407 capture board.

uv run python tools/bench/cdc_throughput.py --seconds 60

The firmware's throughput mode (command "T") streams SVT1 frames as fast as
USB accepts them: the SVC1 header layout with the ticks field holding the
payload length, and every payload byte equal to the low byte of seq. Every
frame is checked; a seq gap, bad magic or corrupted payload is fatal.
"""

from __future__ import annotations

import argparse
import os
import select
import termios
import time
import tty

from capture import CMD_STOP, DEVICE, HEADER, CaptureError

THROUGHPUT_MAGIC = b"SVT1"
CMD_THROUGHPUT = b"T"
READ_CHUNK = 1 << 20
STALL_TIMEOUT_S = 1.0
PATTERNS = [bytes([value]) for value in range(256)]


def measure(seconds: float, device: str = DEVICE) -> dict:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY)
    try:
        tty.setraw(fd)
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, CMD_STOP + b"\0")
        while select.select([fd], [], [], 0.2)[0]:
            os.read(fd, READ_CHUNK)
        os.write(fd, CMD_THROUGHPUT + b"\0")
        return _pump(fd, seconds)
    finally:
        os.write(fd, CMD_STOP + b"\0")
        os.close(fd)


def _pump(fd: int, seconds: float) -> dict:
    buf = bytearray()
    expected_seq = 0
    payload_bytes = 0
    per_second: list[int] = []
    start = None
    last_data = time.monotonic()
    while True:
        now = time.monotonic()
        if start is not None and now - start >= seconds:
            break
        if select.select([fd], [], [], 0.1)[0]:
            buf += os.read(fd, READ_CHUNK)
            last_data = now
        elif now - last_data > STALL_TIMEOUT_S:
            raise CaptureError(
                f"throughput stream stalled at frame {expected_seq}"
            )
        while len(buf) >= HEADER.size:
            magic, seq, _, length, status, _ = HEADER.unpack_from(buf)
            if magic != THROUGHPUT_MAGIC:
                raise CaptureError(
                    f"bad frame magic {magic!r} at frame {expected_seq}"
                )
            frame_len = HEADER.size + length
            if len(buf) < frame_len:
                break
            if seq != expected_seq:
                raise CaptureError(
                    f"frame seq gap: got {seq}, expected {expected_seq}"
                )
            if status:
                raise CaptureError(
                    f"throughput frame {seq} carries status {status}"
                )
            payload = bytes(buf[HEADER.size : frame_len])
            if payload != PATTERNS[seq & 0xFF] * length:
                raise CaptureError(f"frame {seq} payload corrupted")
            del buf[:frame_len]
            expected_seq += 1
            if start is None:
                start = time.monotonic()
                continue
            second = int(time.monotonic() - start)
            while len(per_second) <= second:
                per_second.append(0)
            per_second[second] += frame_len
            payload_bytes += frame_len
    elapsed = time.monotonic() - start
    full = per_second[: int(elapsed)]
    return {
        "seconds": elapsed,
        "frames": expected_seq,
        "bytes": payload_bytes,
        "kb_per_s": payload_bytes / elapsed / 1000.0,
        "min_second_kb": min(full) / 1000.0,
        "max_second_kb": max(full) / 1000.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()
    result = measure(args.seconds)
    print(
        f"{result['kb_per_s']:.1f} KB/s sustained over {result['seconds']:.1f} s "
        f"({result['frames']} frames, {result['bytes']} B; per-second "
        f"min {result['min_second_kb']:.1f} max {result['max_second_kb']:.1f} KB)"
    )


if __name__ == "__main__":
    main()
