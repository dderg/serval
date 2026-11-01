"""Shared access to the bench Pi: Moonraker HTTP, ssh/scp, printer config push."""

from __future__ import annotations

import json
import re
import subprocess
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

PI_SSH = "bench-pi"
PI_HOST = "192.168.1.149"
MOONRAKER = f"http://{PI_HOST}:7125"
PI_CONFIG = "printer_data/config/printer.cfg"
PI_LOGS = "printer_data/logs"
REPO_CFG = Path(__file__).with_name("printer.cfg")
READY_TIMEOUT_S = 60.0
FORBIDDEN_PINS = {
    "PA13": "F401 SWD data pin",
    "PA14": "F401 SWD clock pin",
    "PA8": "reserved for clock output",
}


class BenchError(RuntimeError):
    pass


def http(
    method: str,
    path: str,
    body: bytes | None = None,
    headers: dict | None = None,
) -> dict:
    request = urllib.request.Request(
        MOONRAKER + path, data=body, method=method, headers=headers or {}
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        payload = json.loads(response.read())
        return payload.get("result", payload)


def gcode_script(script: str) -> None:
    body = json.dumps({"script": script}).encode()
    http(
        "POST",
        "/printer/gcode/script",
        body,
        {"Content-Type": "application/json"},
    )


def upload(name: str, data: bytes) -> None:
    boundary = uuid.uuid4().hex
    parts = (
        (
            f'--{boundary}\r\nContent-Disposition: form-data; name="file"; filename="{name}"\r\n'
            "Content-Type: application/octet-stream\r\n\r\n"
        ).encode()
        + data
        + f"\r\n--{boundary}--\r\n".encode()
    )
    http(
        "POST",
        "/server/files/upload",
        parts,
        {"Content-Type": f"multipart/form-data; boundary={boundary}"},
    )


def printer_state() -> str:
    return http("GET", "/printer/info")["state"]


def wait_ready() -> None:
    deadline = time.monotonic() + READY_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            if printer_state() == "ready":
                return
        except OSError:
            pass
        time.sleep(1)
    raise BenchError("printer did not become ready")


def firmware_restart() -> None:
    deadline = time.monotonic() + READY_TIMEOUT_S
    while True:
        try:
            http("POST", "/printer/firmware_restart")
            break
        except urllib.error.HTTPError as exc:
            if exc.code != 503 or time.monotonic() > deadline:
                raise
            time.sleep(1)
    time.sleep(2)
    wait_ready()


def klipper_pid() -> int:
    return int(ssh("systemctl show -p MainPID --value klipper").strip())


def restart_klipper() -> None:
    ssh("sudo systemctl restart klipper")
    time.sleep(2)
    wait_ready()


def ssh(command: str) -> str:
    result = subprocess.run(
        ["ssh", PI_SSH, command], capture_output=True, text=True
    )
    if result.returncode != 0:
        raise BenchError(
            f"ssh {command!r} failed ({result.returncode}):\n{result.stdout}{result.stderr}"
        )
    return result.stdout


def scp_to_pi(local: Path, remote: str) -> None:
    subprocess.run(["scp", "-q", str(local), f"{PI_SSH}:{remote}"], check=True)


def scp_from_pi(remote: str, local: Path, recursive: bool = False) -> None:
    args = (
        ["scp", "-q"]
        + (["-r"] if recursive else [])
        + [f"{PI_SSH}:{remote}", str(local)]
    )
    subprocess.run(args, check=True)


def validate_config(text: str) -> None:
    for number, line in enumerate(text.splitlines(), 1):
        code = line.split("#", 1)[0]
        for pin in re.findall(r"\bP[A-K]\d{1,2}\b", code, flags=re.IGNORECASE):
            reason = FORBIDDEN_PINS.get(pin.upper())
            if reason:
                raise BenchError(
                    f"printer.cfg line {number}: pin {pin.upper()} is forbidden ({reason})"
                )


def push_config() -> bool:
    text = REPO_CFG.read_text()
    validate_config(text)
    if ssh(f"cat {PI_CONFIG}") == text:
        return False
    scp_to_pi(REPO_CFG, PI_CONFIG)
    return True


def prepare_printer() -> None:
    state = printer_state()
    if state != "ready":
        raise BenchError(f"printer state is {state!r}, expected 'ready'")
    push_config()
    firmware_restart()
