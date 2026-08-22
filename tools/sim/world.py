"""Orchestration for the full-stack simulator.

A SimWorld owns one simulated printer: the MCU firmware processes (real
MACH_LINUX ELFs under the libvtime + libsim_intercept LD_PRELOAD shims),
the SPI/UART chip emulators, the optional Beacon probe emulator, and the
klippy host process. Tests and the CLI drive it over klippy's API socket.
"""

from __future__ import annotations

import dataclasses
import itertools
import json
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
VTIME_SHM_SIZE = 256
VTIME_STRUCT_FMT = "<QIIII"

READY_POLL_S = 0.05
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


def vtime_create(shm_name: str, start_ns: int = 1_000_000_000) -> None:
    path = VTIME_SHM_DIR + shm_name
    with open(path, "wb") as f:
        header = struct.pack(VTIME_STRUCT_FMT, start_ns, 0, 0, 1, 0)
        f.write(header + b"\x00" * (VTIME_SHM_SIZE - len(header)))
    os.chmod(path, 0o666)


def vtime_destroy(shm_name: str) -> None:
    try:
        os.unlink(VTIME_SHM_DIR + shm_name)
    except FileNotFoundError:
        pass


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


def _api_request(
    api_socket: str, method: str, params: dict, timeout: float
) -> dict:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    buf = b""
    with sock:
        sock.connect(api_socket)
        req = {"id": 1, "method": method, "params": params}
        sock.sendall(json.dumps(req).encode() + b"\x03")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                break
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


class EndstopPulser:
    """Cycles endstop GPIO lines low/high so a homing move always sees a
    trigger within ~1s and a clear retract window, mimicking a switch."""

    def __init__(self, control: SimControl, endstop_pins: list):
        self.control = control
        self.endstop_pins = endstop_pins
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def __enter__(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        for chip, line in self.endstop_pins:
            self.control.set_gpio_input(chip, line, 0)

    def _run(self):
        # The MCU must be fully up before GPIO commands stick.
        time.sleep(0.5)
        while not self._stop.is_set():
            for chip, line in self.endstop_pins:
                self.control.set_gpio_input(chip, line, 0)
            if self._stop.wait(1.0):
                break
            for chip, line in self.endstop_pins:
                self.control.set_gpio_input(chip, line, 1)
            if self._stop.wait(0.1):
                break


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
        vtime_speed: float = 1.0,
    ):
        self.workdir = pathlib.Path(workdir)
        self.repo_root = repo_root
        self.dual_mcu = dual_mcu
        self.sc_mcu = sc_mcu
        self.want_beacon = beacon
        self.want_cartographer = cartographer
        self.verbose = verbose
        # Virtual-clock speed relative to real time. Below 1.0 the simulated
        # world runs slower than reality, inflating every host-side latency
        # budget (drip windows, trsync timeouts) by 1/speed — the margin that
        # keeps timing-sensitive scenarios deterministic under CPU load.
        self.vtime_speed = vtime_speed
        self.vtime_shm_name = vtime_shm_name_alloc()

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
            # real-time motion, and the 5ppm clock-sync stability gate is
            # sized for hardware oscillators. Virtual time legally runs
            # slower than real time under load (pacer floors) and jitters
            # far beyond real crystal drift, so give every sim world the
            # slack to keep both guards without becoming a host-scheduling
            # lottery. Inserted before any autosave block — that section
            # must stay at the end of the file.
            danger = (
                "\n[danger_options]\n"
                "homing_trip_deadline_margin: 30\n"
                "clock_sync_stable_ppm: 1000\n"
            )
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
        # The motion tick thread registers as a vtime pacer: virtual time
        # can never advance past the tick the engine is about to execute.
        env["LD_PRELOAD"] = f"{vtime_so}:{shim_so}"
        env["VTIME_SHM_NAME"] = self.vtime_shm_name
        env["VTIME_SPEED"] = os.environ.get(
            "VTIME_SPEED", str(self.vtime_speed)
        )
        env["MCU_SIM_SOCK_DIR"] = str(sock_dir)
        env["MCU_SIM_GPIO_STEP_TRACKING"] = "1"
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
            vtime_shm_name=self.vtime_shm_name,
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
            vtime_shm_name=self.vtime_shm_name,
        )
        self.cartographer.start_sample_stream(z_target_mm=10.0, rate_hz=200)

    def _spawn_klippy(self, cfg_path: pathlib.Path) -> None:
        env = os.environ.copy()
        # Klippy deliberately does NOT load vtime: it runs at real CPU
        # speed while the MCU processes live on the virtual clock; loading
        # vtime in klippy deadlocks (both sides block on I/O and neither
        # advances time).
        if self.mcus:
            env["MCU_SIM_SOCK_DIR"] = str(self.mcus[0].sock_dir)
        # No preload, but sim-only extras (sim_remote_endstop) read the
        # virtual clock word directly to time emulated physical events.
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
            ],
            env=env,
            stdout=stdout_log,
            stderr=subprocess.STDOUT,
            cwd=str(self.repo_root),
        )
        stdout_log.close()

    def _wait_ready(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.klippy_proc.poll() is not None:
                return False
            try:
                info = _api_request(self.api_socket, "info", {}, timeout=3.0)
            except OSError:
                info = {}
            state = (info.get("result") or {}).get("state")
            if state == "ready":
                return True
            if state in ("error", "shutdown"):
                return False
            time.sleep(READY_POLL_S)
        return False

    def _wait_clock_sync(self, timeout: float) -> None:
        try:
            resp = _api_request(
                self.api_socket, "objects/list", {}, timeout=5.0
            )
        except OSError as err:
            raise SimError(f"objects/list failed after ready: {err}") from err
        pending = [
            name
            for name in resp.get("result", {}).get("objects", [])
            if name == "mcu" or name.startswith("mcu ")
        ]
        deadline = time.monotonic() + timeout
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
            if time.monotonic() > deadline:
                raise SimError(
                    f"MCU clock sync did not converge within {timeout:.0f}s"
                    f" after ready (mcu: {', '.join(pending)})"
                )
            time.sleep(READY_POLL_S)

    # ------------------------------------------------------------ drive

    def gcode(self, script: str, timeout: float = 30.0) -> dict:
        return _api_request(
            self.api_socket,
            "gcode/script",
            {"script": script},
            timeout=timeout,
        )

    def gcode_ok(self, script: str, timeout: float = 30.0) -> dict:
        resp = self.gcode(script, timeout=timeout)
        if not resp:
            raise SimError(f"{script}: no response within {timeout}s")
        if isinstance(resp, dict) and resp.get("error"):
            raise SimError(f"{script}: {resp['error']}")
        return resp

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
                timeout=timeout,
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

    def wait_print_done(self, timeout: float = 600.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.klippy_proc.poll() is not None:
                raise SimError("klippy exited during print")
            shutdown_line = self.shutdown_line()
            if shutdown_line:
                raise SimError(f"printer shutdown: {shutdown_line}")
            ps = self.status().get("print_stats", {})
            state = ps.get("state", "")
            if state == "complete":
                return
            if state == "error":
                raise SimError(f"print error: {ps.get('message', '')}")
            if state == "cancelled":
                raise SimError("print cancelled")
            time.sleep(READY_POLL_S)
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
        ps = self.status().get("print_stats", {})
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

    def expect_log(self, needle: str, timeout: float = 10.0) -> str:
        """Wait for `needle` to appear in klippy.log after the last
        mark_log()/log_tail() call; returns everything appended since the
        mark. klippy's log writer is async, so response text can land in
        the file a moment after the API response returns."""
        deadline = time.monotonic() + timeout
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
            if time.monotonic() >= deadline:
                raise SimError(
                    f"{needle!r} did not appear in klippy.log within "
                    f"{timeout}s; appended:\n{appended[-2000:]}"
                )
            time.sleep(READY_POLL_S)

    def shutdown_line(self) -> Optional[str]:
        for line in self.klippy_log_text().splitlines():
            if "shutdown:" in line.lower():
                return line.strip()
        return None

    def wait_for_log_text(
        self, needle: str, timeout: float = 60.0
    ) -> Optional[str]:
        deadline = time.monotonic() + timeout
        while True:
            for line in self.klippy_log_text().splitlines():
                if needle in line:
                    return line.strip()
            if self.klippy_proc.poll() is not None:
                return None
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.2)

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
        vtime_destroy(self.vtime_shm_name)
