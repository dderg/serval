"""Host side of the F407 step-capture instrument.

Every frame (little-endian):
    magic u32[4] | seq u32 | position u32 | length u16 | status u8 | detail u8
status != 0 is a latched instrument fault; detail is the channel.

Count mode ("SVC1", command S): one frame per 128 snapshot ticks of 25 us;
position is the first tick, length the tick count, payload length *
CHANNEL_COUNT signed step deltas (i8, + when DIR pin is high).

Event mode ("SVE1", command E): payload is a slice of one continuous byte
stream (position = its offset, length = its size), decoded by
capture_events.py.

Simulated peripherals (tools/bench/capture-fw/PHASE2.md): command C sends a
sim config (sim_config.py); commands s and e start count/event mode with aux
records. Aux frames ("SVA1") share the stream's seq; position is the first
record index, length the record count, payload length * AUX_RECORD_LEN
bytes, decoded by capture_aux.py and written to aux.bin.
"""

from __future__ import annotations

import argparse
import contextlib
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

from sim_config import SimConfig
from sim_config import command as sim_command
from sim_config import describe as sim_describe

DEVICE = "/dev/serial/by-id/usb-serval-bench_step-capture_f407-capture-if00"
CHANNEL_COUNT = 4
TICK_HZ = 40_000
EVENT_TICK_HZ = 84_000_000
COUNT_MAGIC = b"SVC1"
EVENT_MAGIC = b"SVE1"
AUX_MAGIC = b"SVA1"
AUX_RECORD_LEN = 44
AUX_CONFIG_ID_OFFSET = 36
HEADER = struct.Struct("<4sIIHBB")
STALL_TIMEOUT_S = 1.0
POSITION_MODULUS = 1 << 32
CMD_START = b"S"
CMD_EVENTS = b"E"
CMD_START_AUX = b"s"
CMD_EVENTS_AUX = b"e"
CMD_STOP = b"X"


class Fault(enum.IntEnum):
    DIR_OVERCAPTURE = 1
    DELTA_OVERFLOW = 2
    DIR_DESYNC = 3
    RING_OVERFLOW = 4
    STEP_OVERCAPTURE = 5
    TICK_LATE = 6
    DMA_STALL = 7
    DIR_ISR_LATE = 8
    DMA_LAP = 9
    TIMEBASE_DESYNC = 10
    OUTPUT_OVERFLOW = 11
    DMA_ERROR = 12
    DIR_QUEUE_FULL = 13
    NON_MONOTONIC = 14
    MCO_LOST = 15
    AUX_OVERFLOW = 16
    DIR_AMBIGUOUS = 17
    WALL_STEPS_OVERFLOW = 18


class CaptureError(RuntimeError):
    pass


@dataclass
class CaptureResult:
    units: int
    unit: str
    path: Path


class Capture:
    def __init__(
        self,
        out_dir: Path,
        both_edges_mask: int,
        events: bool = False,
        sim: SimConfig | None = None,
        device: str = DEVICE,
    ):
        if sim is not None and sim.both_edges_mask != both_edges_mask:
            raise CaptureError(
                f"both-edges mask {both_edges_mask:#x} disagrees with the sim "
                f"config's {sim.both_edges_mask:#x}"
            )
        self._out_dir = out_dir
        self._mask = both_edges_mask
        self._events = events
        self._sim = sim
        self._aux_records = 0
        self._device = device
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None
        self._units = 0

    @property
    def _sink_path(self) -> Path:
        return self._out_dir / ("events.bin" if self._events else "capture.bin")

    @property
    def _magic(self) -> bytes:
        return EVENT_MAGIC if self._events else COUNT_MAGIC

    @property
    def _unit(self) -> str:
        return "bytes" if self._events else "ticks"

    def start(self) -> None:
        self._out_dir.mkdir(parents=True, exist_ok=True)
        fd = os.open(self._device, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(fd)
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, CMD_STOP + b"\0")
        self._drain(fd)
        if self._sim is not None:
            os.write(fd, sim_command(self._sim))
        if self._events:
            command = CMD_EVENTS if self._sim is None else CMD_EVENTS_AUX
        else:
            command = CMD_START if self._sim is None else CMD_START_AUX
        os.write(fd, command + bytes([self._mask]))
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
        if self._sim is not None and self._units and not self._aux_records:
            raise CaptureError(
                f"no aux records in {self._units} {self._unit} of a sim capture"
            )
        return CaptureResult(self._units, self._unit, self._sink_path)

    def check(self) -> None:
        if self._error is not None:
            raise self._error

    def _drain(self, fd: int) -> None:
        while select.select([fd], [], [], 0.2)[0]:
            os.read(fd, 65536)

    def _write_meta(self, **extra) -> None:
        meta = {
            "mode": "events" if self._events else "counts",
            "tick_hz": EVENT_TICK_HZ if self._events else TICK_HZ,
            "channel_count": CHANNEL_COUNT,
            "both_edges_mask": self._mask,
            "aux": self._sim is not None,
            "sim_config_id": None if self._sim is None else self._sim.config_id,
            "sim_config": None if self._sim is None else self._sim.source,
            "sim": None if self._sim is None else sim_describe(self._sim),
            **extra,
        }
        (self._out_dir / "capture.json").write_text(json.dumps(meta, indent=2))

    def _run(self, fd: int) -> None:
        try:
            with (
                open(self._sink_path, "wb") as sink,
                (
                    open(self._out_dir / "aux.bin", "wb")
                    if self._sim is not None
                    else contextlib.nullcontext()
                ) as aux,
            ):
                self._pump(fd, sink, aux)
        except BaseException as exc:
            self._error = exc
        finally:
            os.write(fd, CMD_STOP + b"\0")
            os.close(fd)

    def _take_aux(
        self, position: int, length: int, payload: bytes, aux
    ) -> None:
        if self._sim is None:
            raise CaptureError(
                "aux frame from a capture started without a sim config"
            )
        if position != self._aux_records % POSITION_MODULUS:
            raise CaptureError(
                f"aux record gap: got {position}, expected {self._aux_records}"
            )
        for index in range(length):
            at = index * AUX_RECORD_LEN + AUX_CONFIG_ID_OFFSET
            (config_id,) = struct.unpack_from("<H", payload, at)
            if config_id != self._sim.config_id:
                raise CaptureError(
                    f"firmware runs sim config {config_id}, sent "
                    f"{self._sim.config_id}: it rejected the config"
                )
        aux.write(payload)
        self._aux_records += length

    def _pump(self, fd: int, sink, aux) -> None:
        buf = bytearray()
        expected_seq = 0
        last_data = time.monotonic()
        while not self._stop.is_set():
            if select.select([fd], [], [], 0.1)[0]:
                buf += os.read(fd, 65536)
            elif time.monotonic() - last_data > STALL_TIMEOUT_S:
                raise CaptureError(
                    f"capture stalled after {self._units} {self._unit}"
                )
            while len(buf) >= HEADER.size:
                magic, seq, position, length, status, detail = (
                    HEADER.unpack_from(buf)
                )
                if magic == AUX_MAGIC:
                    payload_len = length * AUX_RECORD_LEN
                elif magic == self._magic:
                    payload_len = (
                        length if self._events else length * CHANNEL_COUNT
                    )
                else:
                    raise CaptureError(
                        f"bad frame magic {magic!r} after {self._units} {self._unit}"
                    )
                frame_len = HEADER.size + payload_len
                if len(buf) < frame_len:
                    break
                if seq != expected_seq:
                    raise CaptureError(
                        f"frame seq gap: got {seq}, expected {expected_seq}"
                    )
                if magic == AUX_MAGIC:
                    self._take_aux(
                        position,
                        length,
                        bytes(buf[HEADER.size : frame_len]),
                        aux,
                    )
                    del buf[:frame_len]
                    expected_seq += 1
                    last_data = time.monotonic()
                    continue
                if position != self._units % POSITION_MODULUS:
                    raise CaptureError(
                        f"{self._unit} gap: got {position}, expected {self._units}"
                    )
                if status:
                    raise CaptureError(
                        f"instrument fault {Fault(status).name} on channel {detail} at {position} {self._unit}"
                    )
                sink.write(buf[HEADER.size : frame_len])
                del buf[:frame_len]
                expected_seq += 1
                self._units += length
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
    parser.add_argument(
        "--events", action="store_true", help="timestamp every edge (SVE1)"
    )
    args = parser.parse_args()
    capture = Capture(args.out_dir, args.both_edges_mask, events=args.events)
    capture.start()
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        capture.check()
        time.sleep(0.1)
    result = capture.stop()
    print(f"{result.units} {result.unit} -> {result.path}")


if __name__ == "__main__":
    main()
