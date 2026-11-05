"""Shared access to the bench Pi: Moonraker HTTP, ssh/scp, printer config push."""

from __future__ import annotations

import json
import re
import subprocess
import time
import urllib.error
import urllib.parse
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
SCP_ATTEMPTS = 3
SCP_RETRY_DELAY_S = 2.0
FORBIDDEN_PINS = {
    "PA13": "F401 SWD data pin",
    "PA14": "F401 SWD clock pin",
    "PA8": "reserved for clock output",
}
F407_DRIVEN_PINS = {"PA9": "PD8", "PA10": "PD9", "PA15": "PD10", "PB0": "PD11"}
PIN_TOKEN = re.compile(r"\bP[A-K]\d{1,2}\b", flags=re.IGNORECASE)
KEY_LINE = re.compile(r"^(\w+)\s*[:=]")


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


def wait_state(accepted: tuple[str, ...]) -> str:
    deadline = time.monotonic() + READY_TIMEOUT_S
    state = "unreachable"
    while time.monotonic() < deadline:
        try:
            state = printer_state()
        except OSError:
            state = "unreachable"
        if state in accepted:
            return state
        time.sleep(1)
    raise BenchError(
        f"printer did not reach {'/'.join(accepted)} within "
        f"{READY_TIMEOUT_S:.0f} s (last state: {state})"
    )


def wait_ready() -> None:
    wait_state(("ready",))


def wait_settled() -> str:
    return wait_state(("ready", "shutdown", "error"))


def ensure_ready() -> None:
    if wait_settled() != "ready":
        firmware_restart()


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


DEPLOY_RECORD = "klipper/.bench-deploy.json"


def write_deployed(identity: dict) -> None:
    result = subprocess.run(
        ["ssh", PI_SSH, f"cat > {DEPLOY_RECORD}"],
        input=json.dumps(identity, indent=2),
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise BenchError(f"writing {DEPLOY_RECORD} failed:\n{result.stderr}")


def read_deployed() -> dict:
    return json.loads(ssh(f"cat {DEPLOY_RECORD}"))


def flashed_firmware() -> dict:
    """What each board was last flashed with, by board, as the deploy record
    holds it: a sync replaces the host's identity, never the boards'."""
    record = ssh(f"cat {DEPLOY_RECORD} 2>/dev/null || true").strip()
    return json.loads(record).get("firmware", {}) if record else {}


def record_firmware(board: str, identity: dict) -> None:
    record = ssh(f"cat {DEPLOY_RECORD} 2>/dev/null || true").strip()
    deployed = json.loads(record) if record else {}
    deployed.setdefault("firmware", {})[board] = identity
    write_deployed(deployed)


def mcu_version(mcu: str = "mcu") -> str:
    status = http("GET", f"/printer/objects/query?{urllib.parse.quote(mcu)}")
    return status["status"][mcu]["mcu_version"]


def klipper_pid() -> int:
    return int(ssh("systemctl show -p MainPID --value klipper").strip())


def restart_klipper() -> None:
    ssh("sudo systemctl restart klipper")
    time.sleep(2)
    ensure_ready()


def ssh(command: str) -> str:
    result = subprocess.run(
        ["ssh", PI_SSH, command], capture_output=True, text=True
    )
    if result.returncode != 0:
        raise BenchError(
            f"ssh {command!r} failed ({result.returncode}):\n{result.stdout}{result.stderr}"
        )
    return result.stdout


def _scp(args: list[str]) -> None:
    errors = []
    for attempt in range(SCP_ATTEMPTS):
        if attempt:
            time.sleep(SCP_RETRY_DELAY_S)
        result = subprocess.run(args, capture_output=True, text=True)
        if result.returncode == 0:
            return
        errors.append(f"exit {result.returncode}: {result.stderr.strip()}")
    raise BenchError(
        f"{' '.join(args)} failed after {SCP_ATTEMPTS} attempts: "
        + " | ".join(errors)
    )


def scp_to_pi(local: Path, remote: str) -> None:
    _scp(["scp", "-q", str(local), f"{PI_SSH}:{remote}"])


def scp_from_pi(remote: str, local: Path, recursive: bool = False) -> None:
    _scp(
        ["scp", "-q"]
        + (["-r"] if recursive else [])
        + [f"{PI_SSH}:{remote}", str(local)]
    )


def is_input_pin_key(section: str, key: str) -> bool:
    """The config keys a pin is only ever read through: an endstop, or the
    probe's trigger."""
    return key == "endstop_pin" or (
        section.split()[:1] == ["probe"] and key == "pin"
    )


def validate_config(text: str) -> None:
    """Refuse a printer config that would make the F401 drive a pin it must
    not. The SWD pins and the MCO pin are never klippy's. The phase-2
    endstop/probe lines are driven push-pull by the F407 from its boot, so
    on the F401 they may only be read: an output there would fight the
    F407's driver."""
    section, key = "", ""
    for number, line in enumerate(text.splitlines(), 1):
        code = re.split(r"[#;]", line, maxsplit=1)[0]
        stripped = code.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            section, key = stripped[1:-1].strip().lower(), ""
            continue
        if code[:1] not in (" ", "\t"):
            match = KEY_LINE.match(stripped)
            key = match.group(1).lower() if match else ""
        for token in PIN_TOKEN.findall(code):
            pin = token.upper()
            reason = FORBIDDEN_PINS.get(pin)
            if reason:
                raise BenchError(
                    f"config line {number}: pin {pin} is forbidden ({reason})"
                )
            if pin in F407_DRIVEN_PINS and not is_input_pin_key(section, key):
                raise BenchError(
                    f"config line {number}: pin {pin} is driven by the F407's "
                    f"{F407_DRIVEN_PINS[pin]}; it may only be an endstop_pin or "
                    f"the [probe] pin, not {key or 'a value'} in [{section}]"
                )


def push_config(config: Path = REPO_CFG) -> bool:
    text = config.read_text()
    validate_config(text)
    if ssh(f"cat {PI_CONFIG}") == text:
        return False
    scp_to_pi(config, PI_CONFIG)
    return True


def prepare_printer(config: Path = REPO_CFG) -> None:
    wait_settled()
    push_config(config)
    firmware_restart()


MCO_LIVE_CFG = "~/klipper/tools/bench/openocd-target-live.cfg"
MCO_WRITES = (
    (0x40023830, 0x00000001, 0x00000000),
    (0x40023808, 0x06400000, 0x07600000),
    (0x40020008, 0x00030000, 0x00030000),
    (0x40020024, 0x00000000, 0x0000000F),
    (0x40020000, 0x00020000, 0x00030000),
)
MCO_WORD = re.compile(r"(0x[0-9a-fA-F]{8}):\s+([0-9a-fA-F]{8})")


def enable_mco() -> None:
    writes = "; ".join(
        f"mmw {a:#010x} {s:#010x} {c:#010x}" for a, s, c in MCO_WRITES
    )
    reads = "; ".join(f"mdw {a:#010x}" for a, _, _ in MCO_WRITES)
    output = ssh(
        f'openocd -f {MCO_LIVE_CFG} -c "init; {writes}; {reads}; exit" 2>&1'
    )
    words = {int(a, 16): int(v, 16) for a, v in MCO_WORD.findall(output)}
    for address, set_bits, clear_bits in MCO_WRITES:
        mask = set_bits | clear_bits
        if address not in words or words[address] & mask != set_bits:
            raise BenchError(
                f"F401 MCO1 setup did not stick at {address:#010x}: {output}"
            )
