"""Orchestration for the full-stack simulator.

A SimWorld owns one simulated printer: the MCU firmware processes (real
MACH_LINUX ELFs under the libvtime + libsim_intercept LD_PRELOAD shims),
the SPI/UART chip emulators, the optional Beacon probe emulator, and the
klippy host process. Tests and the CLI drive it over klippy's API socket.
"""

from __future__ import annotations

import contextlib
import dataclasses
import functools
import itertools
import json
import mmap
import os
import pathlib
import signal
import socket
import struct
import subprocess
import threading
import time
from typing import Optional

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
VTIME_SHM_DIR = "/dev/shm"
VTIME_SHM_SIZE = 8192
UNBOUNDED_VTIME_SPEED = 1000.0

READY_POLL_S = 0.05
PRINT_POLL_S = 0.2
CLOCK_SYNC_BOOT_TIMEOUT = 120.0

_vtime_shm_counter = itertools.count()

_AUTO_ENDSTOP_LINES = {
    "x": 200,
    "a": 200,
    "y": 201,
    "b": 201,
    "z": 202,
    "e": 210,
}


def _config_sections(config_text: str) -> dict[str, dict[str, str]]:
    sections: dict[str, dict[str, str]] = {}
    current: Optional[dict[str, str]] = None
    for raw_line in config_text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("[") and line.endswith("]"):
            current = sections.setdefault(line[1:-1].strip(), {})
            continue
        if current is None or ":" not in line:
            continue
        key, value = line.split(":", 1)
        current[key.strip()] = value.strip()
    return sections


def _parse_gpio_pin(pin: str) -> tuple[str, int, int]:
    normalized = pin.lstrip("!^~")
    mcu, separator, gpio = normalized.partition(":")
    if not separator:
        mcu, gpio = "", mcu
    chip, separator, line = gpio.partition("/")
    if (
        not separator
        or not chip.startswith("gpiochip")
        or not line.startswith("gpio")
    ):
        raise SimError(f"unsupported simulated motor pin {pin!r}")
    return mcu, int(chip[8:]), int(line[4:])


def command_shapes(dict_path: pathlib.Path) -> tuple[dict[str, int], str]:
    """Host→MCU command ids from an MCU data dictionary, and each command's
    parameter kinds for libvtime's frame walker: "n" for a VLQ integer, "b"
    for a length-prefixed buffer."""
    if not dict_path.exists():
        raise SimError(f"missing {dict_path}: rebuild the sim image")
    commands = json.loads(dict_path.read_text())["commands"]
    ids = {}
    shapes = []
    for fmt, msgid in commands.items():
        name, *params = fmt.split()
        ids[name] = msgid
        kinds = "".join(
            "b" if param.split("=", 1)[1] in ("%s", "%*s", "%.*s") else "n"
            for param in params
        )
        shapes.append(f"{msgid}={kinds}")
    return ids, ";".join(shapes)


def configured_step_tracks(
    config_text: str,
) -> dict[str, list[tuple[int, int, int, int, int, int, int, int]]]:
    sections = _config_sections(config_text)
    motor_axes: dict[str, str] = {}
    kinematics = sections.get("kinematics", {})
    for axis in ("x", "a", "y", "b", "z"):
        for motor in kinematics.get(f"{axis}_motors", "").split(","):
            if motor.strip():
                motor_axes[motor.strip()] = axis
    for section_name, values in sections.items():
        if not section_name.startswith("axis "):
            continue
        axis = section_name[5:].strip()
        for motor in values.get("motors", "").split(","):
            if motor.strip():
                motor_axes[motor.strip()] = axis

    tmc_motors = {
        section_name.split(None, 1)[1].strip()
        for section_name in sections
        if section_name.split(None, 1)[0].startswith("tmc")
        and len(section_name.split(None, 1)) == 2
    }
    tracks: dict[str, list[tuple[int, int, int, int, int, int, int, int]]] = {}
    wall_owners: set[tuple[str, str]] = set()
    for section_name, values in sections.items():
        if (
            not section_name.startswith("motor ")
            or values.get("drive") != "stepper"
        ):
            continue
        motor = section_name[6:].strip()
        axis = motor_axes.get(motor)
        if axis is None:
            axis = next(
                (
                    candidate
                    for candidate in _AUTO_ENDSTOP_LINES
                    if motor.startswith(candidate)
                ),
                "",
            )
        if axis not in _AUTO_ENDSTOP_LINES:
            continue
        step_mcu, step_chip, step_line = _parse_gpio_pin(values["step_pin"])
        dir_mcu, dir_chip, dir_line = _parse_gpio_pin(values["dir_pin"])
        dir_invert = values["dir_pin"].lstrip().startswith("!")
        pulse_duration = float(values.get("step_pulse_duration", 1e-7))
        both_edge = motor in tmc_motors and pulse_duration <= 5e-7
        if step_mcu != dir_mcu:
            raise SimError(
                f"motor {motor!r} step_pin and dir_pin use different MCUs"
            )
        mcu_name = step_mcu or "h7"
        wall_key = (mcu_name, axis)
        if wall_key in wall_owners:
            endstop_lines = [210]
        else:
            wall_owners.add(wall_key)
            endstop_lines = (
                [202, 203] if axis == "z" else [_AUTO_ENDSTOP_LINES[axis]]
            )
        for endstop_line in endstop_lines:
            tracks.setdefault(mcu_name, []).append(
                (
                    step_chip,
                    step_line,
                    dir_chip,
                    dir_line,
                    int(dir_invert),
                    0,
                    endstop_line,
                    int(both_edge),
                )
            )
    return tracks


class SimError(Exception):
    pass


def vtime_shm_name_alloc() -> str:
    return f"/vtime-{os.getpid()}-{next(_vtime_shm_counter)}"


# struct vtime_shm (tools/sim/preload/vtime.h): harness_paced u32 at 8, pace
# anchors (virtual ns, real CLOCK_MONOTONIC ns) at 16 and 24, pacer slot
# count u32 at 32, pacer slots {u32 active, u32, u64 floor, u64 slack} from 40,
# device link slot count u32 at 3600, devices_handling i32 at 3604, device
# links {u32 pts + 1, u32, u64 written, u64 consumed} from 3608.
_HARNESS_PACED_OFFSET = 8
_HARNESS_PACE_ANCHOR_OFFSET = 16
_PACER_SLOTS_USED_OFFSET = 32
_PACERS_OFFSET = 40
_PACER_SLOT_SIZE = 24
_HOST_PRESENT_OFFSET = 232
_WAITER_SLOTS_USED_OFFSET = 240
_WAITER_TARGET_OFFSET = 248
_WAITER_THREAD_OFFSET = 1272
_WAITER_IS_FLOOR_OFFSET = 2808
_CLOCK_LOCK_OWNER_OFFSET = 3320
_HOST_TID_OFFSET = 3344
_MAX_PACERS = 8
_MAX_WAITERS = 128
_MAX_HOST_THREADS = 64
_LINK_SLOTS_USED_OFFSET = 3600
_LINKS_OFFSET = 3608
_LINK_SLOT_SIZE = 48
_MAX_LINKS = 8
_LINK_FAULTS_OFFSET = 3992
_LINK_FAULT_SIZE = 24
FAULT_DELAY, FAULT_DROP, FAULT_STALL = 1, 2, 3
# Pacer and device-link slots the harness reserves before any MCU process
# claims one.
HARNESS_PACER_SLOTS = 4
HARNESS_LINK_SLOTS = 4


def vtime_create(shm_name: str, start_ns: int = 1_000_000_000) -> None:
    path = VTIME_SHM_DIR + shm_name
    shm = bytearray(VTIME_SHM_SIZE)
    struct.pack_into("<Q", shm, 0, start_ns)
    struct.pack_into("<I", shm, _PACER_SLOTS_USED_OFFSET, HARNESS_PACER_SLOTS)
    struct.pack_into("<I", shm, _LINK_SLOTS_USED_OFFSET, HARNESS_LINK_SLOTS)
    with open(path, "wb") as f:
        f.write(shm)
    os.chmod(path, 0o666)


def vtime_destroy(shm_name: str) -> None:
    try:
        os.unlink(VTIME_SHM_DIR + shm_name)
    except FileNotFoundError:
        pass


class VirtualClock:
    """The world's virtual clock as seen from the harness.

    While test code runs between calls into the world, the printer idles at
    real-time pace, the way it would while an operator types; inside a world
    wait (`running_free`) virtual time runs as fast as the processes allow.
    """

    def __init__(self, shm_name: str):
        with open(VTIME_SHM_DIR + shm_name, "r+b") as f:
            self._shm = mmap.mmap(f.fileno(), VTIME_SHM_SIZE)
        self._lock = threading.Lock()
        self._free_depth = 0
        self._free_pacers = list(range(HARNESS_PACER_SLOTS))
        self._free_links = list(range(HARNESS_LINK_SLOTS))

    def now_ns(self) -> int:
        return struct.unpack_from("<Q", self._shm)[0]

    def deadline(self, timeout: float) -> "VirtualDeadline":
        return VirtualDeadline(self, timeout)

    def pace(self) -> None:
        struct.pack_into(
            "<QQ",
            self._shm,
            _HARNESS_PACE_ANCHOR_OFFSET,
            self.now_ns(),
            time.monotonic_ns(),
        )
        struct.pack_into("<I", self._shm, _HARNESS_PACED_OFFSET, 1)

    @contextlib.contextmanager
    def running_free(self):
        with self._lock:
            if self._free_depth == 0:
                struct.pack_into("<I", self._shm, _HARNESS_PACED_OFFSET, 0)
            self._free_depth += 1
        try:
            yield
        finally:
            with self._lock:
                self._free_depth -= 1
                if self._free_depth == 0:
                    self.pace()

    def claim_pacer(self) -> int:
        with self._lock:
            if not self._free_pacers:
                raise SimError(
                    f"more than {HARNESS_PACER_SLOTS} harness threads pace "
                    "the virtual clock at once"
                )
            return self._free_pacers.pop()

    def hold_at(self, slot: int, vtime_ns: int) -> None:
        base = _PACERS_OFFSET + slot * _PACER_SLOT_SIZE
        struct.pack_into("<QQ", self._shm, base + 8, vtime_ns, 0)
        struct.pack_into("<I", self._shm, base, 1)

    def release_pacer(self, slot: int) -> None:
        base = _PACERS_OFFSET + slot * _PACER_SLOT_SIZE
        struct.pack_into("<I", self._shm, base, 0)
        with self._lock:
            self._free_pacers.append(slot)

    def claim_link(self, slave_fd: int) -> "DeviceLink":
        rdev = os.fstat(slave_fd).st_rdev
        pts = (os.major(rdev) - 136) * 256 + os.minor(rdev)
        with self._lock:
            if not self._free_links:
                raise SimError(
                    f"more than {HARNESS_LINK_SLOTS} emulated devices read "
                    "from klippy at once"
                )
            slot = self._free_links.pop()
        base = _LINKS_OFFSET + slot * _LINK_SLOT_SIZE
        struct.pack_into("<I5Q", self._shm, base + 4, 0, 0, 0, 0, 0, 0)
        struct.pack_into("<I", self._shm, base, pts + 1)
        return DeviceLink(self._shm, base)

    def release_link(self, link: "DeviceLink") -> None:
        struct.pack_into("<I", self._shm, link.base, 0)
        with self._lock:
            self._free_links.append(
                (link.base - _LINKS_OFFSET) // _LINK_SLOT_SIZE
            )

    def link_slot_of_pts(self, pts: int) -> int:
        used = struct.unpack_from("<I", self._shm, _LINK_SLOTS_USED_OFFSET)[0]
        for slot in range(min(used, _MAX_LINKS)):
            base = _LINKS_OFFSET + slot * _LINK_SLOT_SIZE
            if struct.unpack_from("<I", self._shm, base)[0] == pts + 1:
                return slot
        raise SimError(f"no device link registered for pts{pts}")

    def post_link_fault(
        self, slot: int, kind: int, msgid: int, count: int, delay_ns: int
    ) -> None:
        base = _LINK_FAULTS_OFFSET + slot * _LINK_FAULT_SIZE
        generation = struct.unpack_from("<I", self._shm, base)[0]
        struct.pack_into(
            "<IiIQ", self._shm, base + 4, kind, msgid, count, delay_ns
        )
        struct.pack_into("<I", self._shm, base, (generation + 1) & 0xFFFFFFFF)

    def holds(self) -> str:
        """What currently stops virtual time: the shm state a wedge leaves."""
        shm = self._shm
        now = self.now_ns()
        host_present, host_running = struct.unpack_from(
            "<Ii", shm, _HOST_PRESENT_OFFSET
        )
        lock_owner, host_pid, wake_epoch, idle_epoch = struct.unpack_from(
            "<IIQQ", shm, _CLOCK_LOCK_OWNER_OFFSET
        )
        links_used, devices_handling = struct.unpack_from(
            "<Ii", shm, _LINK_SLOTS_USED_OFFSET
        )
        parts = [
            f"host_present={host_present} host_running={host_running} "
            f"host_pid={host_pid} wake_epoch={wake_epoch} "
            f"idle_epoch={idle_epoch} clock_lock_owner={lock_owner} "
            f"devices_handling={devices_handling}"
        ]
        for i in range(min(links_used, _MAX_LINKS)):
            pts, _, written, consumed = struct.unpack_from(
                "<IIQQ", shm, _LINKS_OFFSET + i * _LINK_SLOT_SIZE
            )
            if pts and written != consumed:
                parts.append(
                    f"device pts{pts - 1} unread={written - consumed}B"
                )
        used = struct.unpack_from("<I", shm, _PACER_SLOTS_USED_OFFSET)[0]
        for i in range(min(used, _MAX_PACERS)):
            active, _, floor, slack = struct.unpack_from(
                "<IIQQ", shm, _PACERS_OFFSET + i * _PACER_SLOT_SIZE
            )
            if active:
                parts.append(
                    f"pacer{i} floor+slack={(floor + slack - now) / 1e6:+.3f}ms"
                )
        used = struct.unpack_from("<I", shm, _WAITER_SLOTS_USED_OFFSET)[0]
        for i in range(min(used, _MAX_WAITERS)):
            target = struct.unpack_from(
                "<Q", shm, _WAITER_TARGET_OFFSET + i * 8
            )[0]
            is_floor = struct.unpack_from(
                "<I", shm, _WAITER_IS_FLOOR_OFFSET + i * 4
            )[0]
            if target and is_floor:
                who = struct.unpack_from(
                    "<Q", shm, _WAITER_THREAD_OFFSET + i * 8
                )[0]
                parts.append(
                    f"host waiter tid {who & 0xFFFFFFFF} "
                    f"floor={(target - now) / 1e6:+.3f}ms"
                )
        for i in range(_MAX_HOST_THREADS):
            tid = struct.unpack_from("<I", shm, _HOST_TID_OFFSET + i * 4)[0]
            if not tid:
                continue
            try:
                stat = pathlib.Path(
                    f"/proc/{host_pid}/task/{tid}/stat"
                ).read_text()
            except OSError:
                continue
            comm, _, rest = stat.partition("(")[2].rpartition(")")
            if rest.split()[0] == "R":
                parts.append(f"host thread {tid} ({comm}) runnable")
        return "; ".join(parts)


class DeviceLink:
    """An emulated device's input from klippy, on the virtual clock.

    klippy's libvtime counts every byte it writes to the device's pty and
    virtual time stands still until the emulator reports them handled, so
    a request is answered at the instant klippy sent it however late the
    emulator thread gets scheduled.
    """

    def __init__(self, shm: mmap.mmap, base: int):
        self._shm = shm
        self.base = base

    def consumed(self, nbytes: int) -> None:
        offset = self.base + 16
        total = struct.unpack_from("<Q", self._shm, offset)[0] + nbytes
        struct.pack_into("<Q", self._shm, offset, total)


class HarnessPacer:
    """A harness thread's seat on the virtual clock.

    Virtual time cannot pass the instant the thread sleeps toward until the
    thread has woken, so code that watches and pokes the printer while it
    moves (an emulated switch, a test reacting to motion) observes it at its
    own cadence of printer time however fast the world runs.
    """

    POLL_REAL_S = 0.0002

    def __init__(self, clock: VirtualClock):
        self._clock = clock
        self._slot = clock.claim_pacer()
        clock.hold_at(self._slot, clock.now_ns())

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self._clock.release_pacer(self._slot)

    def sleep(
        self, seconds: float, stop: Optional[threading.Event] = None
    ) -> bool:
        """Sleep `seconds` of printer time; False if `stop` was set."""
        target = self._clock.now_ns() + int(seconds * 1e9)
        self._clock.hold_at(self._slot, target)
        while self._clock.now_ns() < target:
            if stop is None:
                time.sleep(self.POLL_REAL_S)
            elif stop.wait(self.POLL_REAL_S):
                return False
        return True


class VirtualDeadline:
    """A harness timeout measured in the printer's time, so a slow host
    machine never fails a test. Virtual time that stops advancing for
    WEDGE_REAL_S of real time is a wedged simulator and fails loudly."""

    WEDGE_REAL_S = 60.0

    def __init__(self, clock: VirtualClock, timeout: float):
        self._clock = clock
        self._last_ns = clock.now_ns()
        self._end_ns = self._last_ns + int(timeout * 1e9)
        self._last_progress = time.monotonic()

    def expired(self) -> bool:
        now_ns = self._clock.now_ns()
        if now_ns != self._last_ns:
            self._last_ns = now_ns
            self._last_progress = time.monotonic()
        elif time.monotonic() - self._last_progress > self.WEDGE_REAL_S:
            raise SimError(
                f"virtual clock stuck at {now_ns / 1e9:.6f}s for "
                f"{self.WEDGE_REAL_S:.0f}s of real time: simulator wedged; "
                f"{self._clock.holds()}"
            )
        return now_ns >= self._end_ns


class VtimeRecorder:
    """Samples (real, virtual) clock pairs so a run shows how fast virtual
    time advanced and where it stalled."""

    PERIOD_S = 0.05

    def __init__(self, clock: VirtualClock, csv_path: pathlib.Path):
        self._clock = clock
        self.csv_path = csv_path
        self.samples: list[tuple[int, int]] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)
        self.csv_path.write_text(
            "real_ns,vtime_ns\n"
            + "".join(f"{r},{v}\n" for r, v in self.samples)
        )

    def _run(self) -> None:
        while not self._stop.is_set():
            self.samples.append((time.monotonic_ns(), self._clock.now_ns()))
            self._stop.wait(self.PERIOD_S)

    def rate_summary(self) -> str:
        s = self.samples
        if len(s) < 2 or s[-1][0] == s[0][0]:
            return "vtime: too few samples"
        virtual = (s[-1][1] - s[0][1]) / 1e9
        real = (s[-1][0] - s[0][0]) / 1e9
        return (
            f"vtime: {virtual:.1f}s of printer time in {real:.1f}s real "
            f"({virtual / real:.2f}x); per-sample trace in {self.csv_path}"
        )


@dataclasses.dataclass
class McuProcess:
    name: str
    process: subprocess.Popen
    pty_path: str
    log_path: pathlib.Path
    sock_dir: pathlib.Path

    @property
    def sim_control(self) -> str:
        return str(self.sock_dir / "sim_control")


API_POLL_REAL_S = 0.2


def _api_request(
    api_socket: str, method: str, params: dict, deadline: VirtualDeadline
) -> dict:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(API_POLL_REAL_S)
    buf = b""
    with sock:
        sock.connect(api_socket)
        req = {"id": 1, "method": method, "params": params}
        sock.sendall(json.dumps(req).encode() + b"\x03")
        while not deadline.expired():
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            if b"\x03" in buf:
                break
    if b"\x03" not in buf:
        return {}
    body = buf.split(b"\x03", 1)[0]
    try:
        return json.loads(body.decode())
    except json.JSONDecodeError:
        return {"raw": body.decode(errors="replace")}


class SimControl:
    """Line protocol to a shim's sim_control socket (GPIO/ADC injection)."""

    def __init__(self, sock_path: str):
        self.sock_path = sock_path

    def send(self, cmd: str, timeout: float = 1.0) -> str:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        try:
            with sock:
                sock.connect(self.sock_path)
                sock.sendall((cmd + "\n").encode())
                return sock.recv(256).decode().strip()
        except (ConnectionRefusedError, FileNotFoundError, socket.timeout):
            return ""

    def set_gpio_input(self, chip: int, line: int, value: int) -> str:
        return self.send(
            f"set_gpio_input chip={chip} line={line} value={value}"
        )

    def gpio_edges(self, chip: int, line: int) -> int:
        response = self.send(f"get_gpio_edges chip={chip} line={line}")
        if not response.startswith("edges="):
            raise SimError(
                f"get_gpio_edges chip={chip} line={line}: {response!r}"
            )
        return int(response.split()[0].split("=", 1)[1])

    def gpio_output(self, chip: int, line: int) -> int:
        response = self.send(f"get_gpio_output chip={chip} line={line}")
        if not response.startswith("value="):
            raise SimError(
                f"get_gpio_output chip={chip} line={line}: {response!r}"
            )
        return int(response.split()[0].split("=", 1)[1])

    def step_position(self, line: int) -> dict[str, int]:
        response = self.send(f"get_steps line={line}")
        values = dict(item.split("=", 1) for item in response.split())
        if values.keys() != {"steps", "min", "max", "vt"}:
            raise SimError(f"unexpected get_steps reply: {response!r}")
        return {name: int(value) for name, value in values.items()}

    def set_endstop_wall(self, line: int, steps: int) -> None:
        response = self.send(f"set_endstop_wall line={line} steps={steps}")
        if response != "ok":
            raise SimError(
                f"set_endstop_wall line={line} steps={steps}: {response!r}"
            )

    def enable_step_pin_emit(self) -> None:
        """Arm the runtime's physical per-stepper step/dir pin output. Off
        by default: the ioctl-per-edge traffic distorts sim timing."""
        response = self.send("set_step_emit enable=1")
        if response.strip() != "ok":
            raise SimError(f"set_step_emit enable=1: {response!r}")

    def get_step_times(self, line: int) -> dict[str, int]:
        response = self.send(f"get_step_times line={line}")
        values = dict(item.split("=", 1) for item in response.split())
        required = {
            "count",
            "first_ns",
            "last_ns",
            "sum_ns",
            "sum_cycles",
            "sum_index_cycles",
        }
        if values.keys() != required:
            raise SimError(f"unexpected get_step_times reply: {response!r}")
        return {name: int(value) for name, value in values.items()}

    def reset_step_times(self, line: int) -> None:
        response = self.send(f"reset_step_times line={line}")
        if response.strip() != "ok":
            raise SimError(f"reset_step_times failed: {response!r}")


class _EmulatorThread:
    """A background thread emulating hardware against the printer, paced on
    the world's virtual clock."""

    def __init__(self, clock: VirtualClock):
        self.clock = clock
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def __enter__(self):
        self._thread = threading.Thread(target=self._paced_run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._released()

    def _paced_run(self) -> None:
        with HarnessPacer(self.clock) as pacer:
            self._run(lambda seconds: pacer.sleep(seconds, self._stop))

    def _run(self, sleep) -> None:
        raise NotImplementedError

    def _released(self) -> None:
        raise NotImplementedError


class EndstopPulser(_EmulatorThread):
    """Cycles endstop GPIO lines low/high so a homing move always sees a
    trigger within ~1s of printer time and a clear retract window,
    mimicking a switch."""

    def __init__(
        self,
        control: SimControl,
        endstop_pins: list,
        clock: VirtualClock,
        initial_delay: float = 0.5,
        low_duration: float = 1.0,
    ):
        super().__init__(clock)
        self.control = control
        self.endstop_pins = endstop_pins
        self.initial_delay = initial_delay
        self.low_duration = low_duration

    def _run(self, sleep) -> None:
        # The MCU must be fully up before GPIO commands stick.
        if not sleep(self.initial_delay):
            return
        while True:
            for chip, line in self.endstop_pins:
                self.control.set_gpio_input(chip, line, 0)
            if not sleep(self.low_duration):
                return
            for chip, line in self.endstop_pins:
                self.control.set_gpio_input(chip, line, 1)
            if not sleep(0.1):
                return

    def _released(self) -> None:
        for chip, line in self.endstop_pins:
            self.control.set_gpio_input(chip, line, 0)


class EndstopWall(_EmulatorThread):
    """Drive an endstop GPIO like a switch mounted at `wall_mm` on one
    cartesian axis: high at/past the wall, low below it. `tracker.xy()`
    reports the physical position from the shim's step counters."""

    POLL_S = 0.005

    def __init__(self, tracker, control, axis_idx, pin, wall_mm, clock):
        super().__init__(clock)
        self.tracker = tracker
        self.control = control
        self.axis_idx = axis_idx
        self.pin = pin
        self.wall_mm = wall_mm

    def _run(self, sleep) -> None:
        level = None
        while True:
            pos = self.tracker.xy()[self.axis_idx]
            want = 1 if pos >= self.wall_mm else 0
            if want != level:
                self.control.set_gpio_input(*self.pin, want)
                level = want
            if not sleep(self.POLL_S):
                return

    def _released(self) -> None:
        self.control.set_gpio_input(*self.pin, 0)


def _runs_free(method):
    """A world call that waits on the printer lets virtual time run free."""

    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self.vclock.running_free():
            return method(self, *args, **kwargs)

    return wrapper


class SimWorld:
    """One simulated printer. Construct, then boot(config_text).

    PTY paths (h7_pty / f4_pty / beacon_pty) and gcode_dir are fixed at
    construction so config text can reference them before boot.
    """

    def __init__(
        self,
        workdir: pathlib.Path,
        repo_root: pathlib.Path = REPO_ROOT,
        dual_mcu: bool = True,
        sc_mcu: bool = False,
        beacon: bool = False,
        cartographer: bool = False,
        verbose: bool = False,
        vtime_speed: float = UNBOUNDED_VTIME_SPEED,
        host_gated: bool = True,
        crystal_ppm: Optional[dict[str, float]] = None,
    ):
        self.workdir = pathlib.Path(workdir)
        self.repo_root = repo_root
        self.dual_mcu = dual_mcu
        self.sc_mcu = sc_mcu
        self.want_beacon = beacon
        self.want_cartographer = cartographer
        self.verbose = verbose
        # Upper bound on virtual seconds per real second. With host_gated,
        # virtual time stands still while klippy computes, so the cap only
        # matters for runs a human watches; without it (throughput tests),
        # klippy must keep up with the cap like a real host must keep up
        # with real time.
        self.vtime_speed = float(os.environ.get("VTIME_SPEED", vtime_speed))
        self.host_gated = host_gated
        self.crystal_ppm = crystal_ppm or {}
        self.vtime_shm_name = vtime_shm_name_alloc()
        self.vclock: Optional[VirtualClock] = None
        self.vtime_recorder: Optional[VtimeRecorder] = None

        self.log_dir = self.workdir / "logs"
        self.gcode_dir = self.workdir / "gcodes"
        self.h7_pty = str(self.workdir / "pty_h7")
        self.f4_pty = str(self.workdir / "pty_f4")
        self.beacon_pty = str(self.workdir / "pty_beacon")
        self.cartographer_pty = str(self.workdir / "pty_cartographer")
        self.api_socket = str(self.workdir / "klippy.sock")
        self.klippy_log = self.log_dir / "klippy.log"

        self.mcus: list[McuProcess] = []
        self.klippy_proc: Optional[subprocess.Popen] = None
        self.chip_servers: list = []
        self.tmc5160_by_cs: dict = {}
        self.beacon = None
        self.cartographer = None
        self._log_offset = 0
        self._started = False
        self._step_tracks: dict[
            str, list[tuple[int, int, int, int, int, int, int, int]]
        ] = {}
        self._z_step_lines: dict[str, int] = {}
        self._command_ids: dict[str, dict[str, int]] = {}

    # ------------------------------------------------------------- boot

    def boot(
        self,
        config_text: str,
        ready_timeout: float = 120.0,
        expect_boot_error: Optional[str] = None,
        spawn_mcus: bool = True,
    ) -> None:
        assert not self._started, "SimWorld.boot() called twice"
        self._started = True
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.gcode_dir.mkdir(parents=True, exist_ok=True)
        aliases = configured_step_tracks(config_text)
        config_sections = _config_sections(config_text)
        for alias, tracks in aliases.items():
            if alias == "h7":
                process_name = "h7"
            else:
                serial = config_sections.get(f"mcu {alias}", {}).get("serial")
                if serial == self.h7_pty:
                    process_name = "h7"
                elif serial == self.f4_pty:
                    process_name = "f4"
                else:
                    raise SimError(f"unknown MCU prefix {alias!r} on motor pin")
            self._step_tracks.setdefault(process_name, []).extend(tracks)
            for step_chip, step_line, _, _, _, _, endstop_line, _ in tracks:
                if endstop_line == 202:
                    if step_chip != 0:
                        raise SimError("probe emulators require Z on gpiochip0")
                    self._z_step_lines.setdefault(process_name, step_line)

        shim_so, vtime_so = self._ensure_shims_built()
        vtime_create(self.vtime_shm_name)
        self.vclock = VirtualClock(self.vtime_shm_name)
        self.vtime_recorder = VtimeRecorder(
            self.vclock, self.log_dir / "vtime.csv"
        )
        self.vtime_recorder.start()

        if spawn_mcus:
            self._spawn_mcus(shim_so, vtime_so)
            self._start_chip_emulators()
            if self.want_beacon:
                self._start_beacon()
            if self.want_cartographer:
                self._start_cartographer()

        cfg_path = self.workdir / "printer.cfg"
        if "[danger_options]" not in config_text:
            # Homing trip deadlines are wall-clock budgets sized for
            # real-time motion; virtual time legally runs slower than real
            # time under load (pacer floors), so give every sim world the
            # slack to keep the guard without becoming a host-scheduling
            # lottery. Inserted before any autosave block — that section
            # must stay at the end of the file.
            danger = "\n[danger_options]\nhoming_trip_deadline_margin: 30\n"
            marker = config_text.find("#*#")
            if marker == -1:
                config_text += danger
            else:
                config_text = (
                    config_text[:marker] + danger + "\n" + config_text[marker:]
                )
        cfg_path.write_text(config_text)
        self._spawn_klippy(cfg_path)

        if expect_boot_error is not None:
            line = self.wait_for_log_text(expect_boot_error, timeout=60)
            if line is None:
                raise SimError(
                    f"expected boot error {expect_boot_error!r} did not appear"
                )
            return
        if not self._wait_ready(ready_timeout):
            tail = self.klippy_log_text()[-3000:]
            raise SimError(f"klippy failed to become ready:\n{tail}")
        self._wait_clock_sync(CLOCK_SYNC_BOOT_TIMEOUT)

    def _ensure_shims_built(self) -> tuple:
        shim_dir = self.repo_root / "tools" / "sim" / "preload"
        shim_so = shim_dir / "libsim_intercept.so"
        vtime_so = shim_dir / "libvtime.so"
        if not shim_so.exists() or not vtime_so.exists():
            subprocess.check_call(["make", "-C", str(shim_dir)])
        return shim_so, vtime_so

    def _elf(self, name: str) -> pathlib.Path:
        elf = self.repo_root / "out" / name
        if not elf.exists():
            raise SimError(
                f"missing {elf} — build the sim image (tools/sim/run.sh) "
                "or build the MACH_LINUX ELFs from tools/sim/configs/"
            )
        return elf

    def _spawn_mcus(self, shim_so, vtime_so) -> None:
        specs = [("h7", self._elf("klipper-h7-sim.elf"), self.h7_pty)]
        if self.dual_mcu:
            second = (
                "klipper-sc-sim.elf" if self.sc_mcu else "klipper-f4-sim.elf"
            )
            specs.append(("f4", self._elf(second), self.f4_pty))

        spawned: dict = {}
        errors: dict = {}

        def _spawn(name, elf, pty):
            try:
                spawned[name] = self._spawn_one_mcu(
                    name, elf, pty, shim_so, vtime_so
                )
            except Exception as exc:
                errors[name] = exc

        threads = [threading.Thread(target=_spawn, args=spec) for spec in specs]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        if errors:
            raise next(iter(errors.values()))
        self.mcus = [spawned[name] for name, _, _ in specs]

    def _spawn_one_mcu(
        self, name, elf, pty_path, shim_so, vtime_so
    ) -> McuProcess:
        sock_dir = self.workdir / "sim" / name
        sock_dir.mkdir(parents=True, exist_ok=True)
        if os.path.exists(pty_path):
            os.unlink(pty_path)
        log_path = self.log_dir / f"{name}.log"
        log_fd = open(log_path, "wb")
        env = os.environ.copy()
        # vtime first, intercept second (constructor/interpose ordering).
        env["LD_PRELOAD"] = f"{vtime_so}:{shim_so}"
        env["VTIME_SHM_NAME"] = self.vtime_shm_name
        env["VTIME_SPEED"] = str(self.vtime_speed)
        env["MCU_SIM_SOCK_DIR"] = str(sock_dir)
        env["MCU_SIM_GPIO_STEP_TRACKING"] = "1"
        if name in self.crystal_ppm:
            env["VTIME_CRYSTAL_PPM"] = repr(self.crystal_ppm[name])
        command_ids, shapes = command_shapes(elf.with_suffix(".dict"))
        self._command_ids[name] = command_ids
        env["MCU_SIM_CMD_SHAPES"] = shapes
        tracks = self._step_tracks.get(name, [])
        env["MCU_SIM_STEP_TRACKS"] = ";".join(
            ",".join(str(value) for value in track) for track in tracks
        )
        if self.verbose:
            env["MCU_SIM_SHIM_VERBOSE"] = "1"
            env["VTIME_DEBUG"] = "1"
        proc = subprocess.Popen(
            [str(elf), "-I", pty_path],
            stdout=log_fd,
            stderr=subprocess.STDOUT,
            env=env,
        )
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if os.path.exists(pty_path):
                return McuProcess(name, proc, pty_path, log_path, sock_dir)
            if proc.poll() is not None:
                log_fd.close()
                raise SimError(
                    f"{name}: firmware exited early (rc={proc.returncode})\n"
                    + log_path.read_text(errors="replace")
                )
            time.sleep(READY_POLL_S)
        proc.kill()
        log_fd.close()
        raise SimError(f"{name}: PTY {pty_path} did not appear in 10s")

    def _start_chip_emulators(self) -> None:
        from tools.sim.emulators.chip_socket_server import ChipSocketServer
        from tools.sim.emulators.max31865_emulator import MAX31865Emulator
        from tools.sim.emulators.tmc2209_emulator import TMC2209Emulator
        from tools.sim.emulators.tmc5160_emulator import TMC5160Emulator

        h7_sock = self.mcus[0].sock_dir
        # SPI chip-select wiring matches the sim configs' cs_pin lines.
        for cs_line in (5, 4, 6, 3):
            chip = TMC5160Emulator()
            self.tmc5160_by_cs[cs_line] = chip
            srv = ChipSocketServer(
                str(h7_sock / f"spi_cs_0_{cs_line}"),
                chip.transfer,
                framed=False,
            )
            srv.start()
            self.chip_servers.append(srv)
        srv = ChipSocketServer(
            str(h7_sock / "spi_cs_0_40"),
            MAX31865Emulator().transfer,
            framed=False,
        )
        srv.start()
        self.chip_servers.append(srv)

        srv = ChipSocketServer(
            str(h7_sock / "tmcuart_0"),
            TMC2209Emulator(slave_addr=0).handle,
            chunk=10,
        )
        srv.start()
        self.chip_servers.append(srv)

        if self.dual_mcu:
            f4_sock = self.mcus[1].sock_dir
            for i in range(3):
                srv = ChipSocketServer(
                    str(f4_sock / f"tmcuart_{i}"),
                    TMC2209Emulator(slave_addr=0).handle,
                    chunk=10,
                )
                srv.start()
                self.chip_servers.append(srv)

    def _start_beacon(self) -> None:
        from tools.sim.emulators.beacon_mcu import BeaconMcuStub

        z_mcu = self.mcus[1] if self.dual_mcu else self.mcus[0]
        z_step_line = self._z_step_lines.get(z_mcu.name, 15)
        self.beacon = BeaconMcuStub(
            self.beacon_pty,
            log_path=str(self.log_dir / "beacon_traffic.log"),
            step_sock_path=z_mcu.sim_control,
            z_step_line=z_step_line,
            clock=self.vclock,
        )
        self.beacon.start_sample_stream(z_target_mm=10.0, rate_hz=200)

    def _start_cartographer(self) -> None:
        from tools.sim.emulators.cartographer_mcu import CartographerMcuStub

        z_mcu = self.mcus[1] if self.dual_mcu else self.mcus[0]
        z_step_line = self._z_step_lines.get(z_mcu.name, 15)
        self.cartographer = CartographerMcuStub(
            self.cartographer_pty,
            log_path=str(self.log_dir / "cartographer_traffic.log"),
            step_sock_path=z_mcu.sim_control,
            z_step_line=z_step_line,
            clock=self.vclock,
        )
        self.cartographer.start_sample_stream(z_target_mm=10.0, rate_hz=200)

    def _spawn_klippy(self, cfg_path: pathlib.Path) -> None:
        env = os.environ.copy()
        # Klippy shares the MCUs' virtual clock: a host clock that slipped
        # against the MCU clock whenever virtual time stalled made every
        # MCU look like a wandering crystal to clock sync.
        _, vtime_so = self._ensure_shims_built()
        env["LD_PRELOAD"] = str(vtime_so)
        env["VTIME_SPEED"] = str(self.vtime_speed)
        if self.host_gated:
            env["VTIME_HOST"] = "1"
        if self.mcus:
            env["MCU_SIM_SOCK_DIR"] = str(self.mcus[0].sock_dir)
        env["VTIME_SHM_NAME"] = self.vtime_shm_name
        third_party = self.repo_root / "tools" / "sim" / "third_party_repos"
        for plugin_dir in ("beacon_klipper", "cartographer_klipper"):
            plugin_repo = third_party / plugin_dir
            if plugin_repo.exists():
                env["PYTHONPATH"] = ":".join(
                    filter(None, [str(plugin_repo), env.get("PYTHONPATH", "")])
                )
        stdout_log = open(self.log_dir / "klippy.stdout", "wb")
        self.klippy_proc = subprocess.Popen(
            [
                "python3",
                str(self.repo_root / "klippy" / "klippy.py"),
                str(cfg_path),
                "-l",
                str(self.klippy_log),
                "-a",
                self.api_socket,
                "-I",
                str(self.workdir / "printer_input"),
            ],
            env=env,
            stdout=stdout_log,
            stderr=subprocess.STDOUT,
            cwd=str(self.repo_root),
        )
        stdout_log.close()

    @_runs_free
    def _wait_ready(self, timeout: float) -> bool:
        deadline = self.vclock.deadline(timeout)
        while not deadline.expired():
            if self.klippy_proc.poll() is not None:
                return False
            try:
                info = _api_request(
                    self.api_socket, "info", {}, self.vclock.deadline(3.0)
                )
            except OSError:
                info = {}
            state = (info.get("result") or {}).get("state")
            if state == "ready":
                return True
            if state in ("error", "shutdown"):
                return False
            time.sleep(READY_POLL_S)
        return False

    @_runs_free
    def _wait_clock_sync(self, timeout: float) -> None:
        try:
            resp = _api_request(
                self.api_socket, "objects/list", {}, self.vclock.deadline(5.0)
            )
        except OSError as err:
            raise SimError(f"objects/list failed after ready: {err}") from err
        pending = [
            name
            for name in resp.get("result", {}).get("objects", [])
            if name == "mcu" or name.startswith("mcu ")
        ]
        deadline = self.vclock.deadline(timeout)
        while pending:
            status = self.status({name: None for name in pending})
            pending = [
                name
                for name in pending
                if not status.get(name, {}).get("clock_sync_converged")
            ]
            if not pending:
                return
            if self.klippy_proc.poll() is not None:
                raise SimError("klippy exited while waiting for clock sync")
            if deadline.expired():
                raise SimError(
                    f"MCU clock sync did not converge within {timeout:.0f}s"
                    f" after ready (mcu: {', '.join(pending)})"
                )
            time.sleep(READY_POLL_S)

    # ------------------------------------------------------------ drive

    @_runs_free
    def gcode(self, script: str, timeout: float = 30.0) -> dict:
        return _api_request(
            self.api_socket,
            "gcode/script",
            {"script": script},
            self.vclock.deadline(timeout),
        )

    def gcode_ok(self, script: str, timeout: float = 30.0) -> dict:
        resp = self.gcode(script, timeout=timeout)
        if not resp:
            raise SimError(f"{script}: no response within {timeout}s")
        if isinstance(resp, dict) and resp.get("error"):
            raise SimError(f"{script}: {resp['error']}")
        return resp

    @_runs_free
    def status(
        self, objects: Optional[dict] = None, timeout: float = 5.0
    ) -> dict:
        if objects is None:
            objects = {
                "print_stats": None,
                "toolhead": None,
                "virtual_sdcard": None,
            }
        try:
            resp = _api_request(
                self.api_socket,
                "objects/query",
                {"objects": objects},
                self.vclock.deadline(timeout),
            )
        except OSError:
            return {}
        return resp.get("result", {}).get("status", {})

    def toolhead_position(self) -> Optional[list]:
        pos = self.status().get("toolhead", {}).get("position")
        return list(pos) if pos else None

    def toolhead_z(self) -> Optional[float]:
        pos = self.toolhead_position()
        return float(pos[2]) if pos and len(pos) >= 3 else None

    @_runs_free
    def wait_print_done(self, timeout: float = 600.0) -> None:
        deadline = self.vclock.deadline(timeout)
        log_offset = (
            self.klippy_log.stat().st_size if self.klippy_log.exists() else 0
        )
        log_remainder = ""
        while not deadline.expired():
            if self.klippy_proc.poll() is not None:
                raise SimError("klippy exited during print")
            if self.klippy_log.exists():
                with self.klippy_log.open("rb") as log:
                    log.seek(log_offset)
                    appended = log.read()
                    log_offset = log.tell()
                text = log_remainder + appended.decode(errors="replace")
                lines = text.splitlines(keepends=True)
                log_remainder = (
                    lines.pop()
                    if lines and not lines[-1].endswith(("\n", "\r"))
                    else ""
                )
                for line in lines:
                    if "shutdown:" in line.lower():
                        raise SimError(f"printer shutdown: {line.strip()}")
            ps = self.status({"print_stats": None}).get("print_stats", {})
            state = ps.get("state", "")
            if state == "complete":
                return
            if state == "error":
                raise SimError(f"print error: {ps.get('message', '')}")
            if state == "cancelled":
                raise SimError("print cancelled")
            time.sleep(PRINT_POLL_S)
        raise SimError(f"print did not finish within {timeout}s")

    def print_file(
        self, gcode_path: pathlib.Path, timeout: float = 600.0
    ) -> float:
        """Print a G-code file via the virtual SD card; returns print time."""
        import shutil

        dest = self.gcode_dir / gcode_path.name
        if not dest.exists():
            shutil.copy2(gcode_path, dest)
        self.gcode_ok(f"SDCARD_PRINT_FILE FILENAME={gcode_path.name}")
        self.wait_print_done(timeout=timeout)
        ps = self.status({"print_stats": None}).get("print_stats", {})
        return ps.get("total_duration") or ps.get("print_duration") or 0.0

    # -------------------------------------------------------- inspect

    def klippy_log_text(self) -> str:
        if self.klippy_log.exists():
            return self.klippy_log.read_text(errors="replace")
        return ""

    def log_tail(self) -> str:
        """Text appended to klippy.log since the previous log_tail() call."""
        data = self.klippy_log.read_bytes() if self.klippy_log.exists() else b""
        out = data[self._log_offset :].decode(errors="replace")
        self._log_offset = len(data)
        return out

    def mark_log(self) -> None:
        self.log_tail()

    @_runs_free
    def expect_log(self, needle: str, timeout: float = 10.0) -> str:
        """Wait for `needle` to appear in klippy.log after the last
        mark_log()/log_tail() call; returns everything appended since the
        mark. klippy's log writer is async, so response text can land in
        the file a moment after the API response returns."""
        deadline = self.vclock.deadline(timeout)
        while True:
            data = (
                self.klippy_log.read_bytes()
                if self.klippy_log.exists()
                else b""
            )
            appended = data[self._log_offset :].decode(errors="replace")
            if needle in appended:
                self._log_offset = len(data)
                return appended
            if deadline.expired():
                raise SimError(
                    f"{needle!r} did not appear in klippy.log within "
                    f"{timeout}s; appended:\n{appended[-2000:]}"
                )
            time.sleep(READY_POLL_S)

    def inject(
        self,
        mcu: str,
        *,
        delay_command: Optional[str] = None,
        drop_command: Optional[str] = None,
        by_s: float = 0.0,
        count: int = 1,
        stall_s: Optional[float] = None,
    ) -> None:
        """Fault the host→MCU link of process `mcu` ("h7", "f4"), applied by
        the MCU as it reads, in virtual time: hold the next `count` frames
        carrying `delay_command` (and everything behind them) for `by_s`,
        drop the next `count` frames carrying `drop_command`, or hold the
        whole link for `stall_s`. Klippy is not touched."""
        chosen = [
            arg
            for arg in (delay_command, drop_command, stall_s)
            if arg is not None
        ]
        if len(chosen) != 1:
            raise SimError(
                "inject takes exactly one of delay_command, drop_command,"
                " stall_s"
            )
        pty_path = {"h7": self.h7_pty, "f4": self.f4_pty}[mcu]
        pts = int(os.readlink(pty_path).rsplit("/", 1)[1])
        slot = self.vclock.link_slot_of_pts(pts)
        if stall_s is not None:
            self.vclock.post_link_fault(
                slot, FAULT_STALL, 0, 0, int(stall_s * 1e9)
            )
            return
        command = delay_command if delay_command is not None else drop_command
        ids = self._command_ids[mcu]
        if command not in ids:
            raise SimError(f"{mcu} has no command {command!r}")
        kind = FAULT_DELAY if delay_command is not None else FAULT_DROP
        self.vclock.post_link_fault(
            slot, kind, ids[command], count, int(by_s * 1e9)
        )

    def shutdown_line(self) -> Optional[str]:
        for line in self.klippy_log_text().splitlines():
            if "shutdown:" in line.lower():
                return line.strip()
        return None

    @_runs_free
    def wait_for_log_text(
        self, needle: str, timeout: float = 60.0
    ) -> Optional[str]:
        deadline = self.vclock.deadline(timeout)
        while True:
            for line in self.klippy_log_text().splitlines():
                if needle in line:
                    return line.strip()
            if self.klippy_proc.poll() is not None:
                return None
            if deadline.expired():
                return None
            time.sleep(0.2)

    @_runs_free
    def wait_until(
        self, pred, timeout: float, what: str, poll: float = 0.005
    ) -> None:
        """Poll `pred` every `poll` seconds of printer time — the cadence a
        test reacting to motion (poking a pin the moment the axis reaches a
        point) needs however fast the world runs."""
        deadline = self.vclock.deadline(timeout)
        with HarnessPacer(self.vclock) as pacer:
            while not pred():
                if deadline.expired():
                    raise SimError(f"timeout waiting for {what}")
                pacer.sleep(poll)

    def events_text(self) -> str:
        events_dir = self.log_dir / "events"
        out = []
        for ev_file in sorted(events_dir.glob("*.jsonl")):
            out.append(ev_file.read_text(errors="replace"))
        return "".join(out)

    def sim_control(self, mcu: str = "h7") -> SimControl:
        for m in self.mcus:
            if m.name == mcu:
                return SimControl(m.sim_control)
        raise SimError(f"no such MCU: {mcu}")

    def dump_diagnostics(self) -> None:
        print("=== sim diagnostics ===")
        if self.vtime_recorder:
            print(self.vtime_recorder.rate_summary())
        tail = self.klippy_log_text()[-6000:]
        if tail:
            print("--- klippy.log tail ---")
            print(tail)
        stdout_log = self.log_dir / "klippy.stdout"
        if stdout_log.exists():
            print("--- klippy.stdout tail ---")
            print(stdout_log.read_text(errors="replace")[-4000:])
        for mcu in self.mcus:
            if mcu.log_path.exists():
                print(f"--- {mcu.name}.log tail ---")
                print(mcu.log_path.read_text(errors="replace")[-2000:])
        events = self.events_text()
        if events:
            print("--- events/*.jsonl tail ---")
            for line in events.splitlines()[-80:]:
                print(line)
        print("=== end sim diagnostics ===")

    # ------------------------------------------------------- teardown

    def shutdown(self) -> None:
        if self.klippy_proc and self.klippy_proc.poll() is None:
            self.klippy_proc.terminate()
            try:
                self.klippy_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.klippy_proc.kill()
        for mcu in self.mcus:
            if mcu.process.poll() is None:
                mcu.process.send_signal(signal.SIGTERM)
        for mcu in self.mcus:
            try:
                mcu.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                mcu.process.kill()
        for srv in self.chip_servers:
            srv.stop()
        if self.beacon:
            self.beacon.stop()
        if self.cartographer:
            self.cartographer.stop()
        if self.vtime_recorder:
            self.vtime_recorder.stop()
        vtime_destroy(self.vtime_shm_name)
