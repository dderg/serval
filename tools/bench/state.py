"""Report the bench state: printer, deployed code, klipper pid, boards.

uv run python tools/bench/state.py [--json] [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import os

import pi
from capture import DEVICE as CAPTURE_DEVICE

MCU_SERIAL_GLOB = "/dev/serial/by-id/usb-Klipper_*"

QUERIES = {
    "printer_state": "Moonraker GET /printer/info",
    "deployed": "ssh: cat ~/klipper/.bench-deploy.json",
    "klipper_pid": "ssh: systemctl show -p MainPID klipper",
    "mcu_serial": f"ssh: ls {MCU_SERIAL_GLOB}",
    "capture_board": f"local path {CAPTURE_DEVICE}",
}


def collect() -> dict:
    info = pi.http("GET", "/printer/info")
    deployed = pi.read_deployed()
    return {
        "printer_state": info["state"],
        "state_message": info.get("state_message", "").splitlines()[:1],
        "deployed": {
            "head": deployed["head"][:9],
            "branch": deployed["branch"],
            "worktree": deployed["worktree"],
            "dirty_paths": deployed["dirty_paths"],
            "timestamp": deployed["timestamp"],
        },
        "klipper_pid": pi.klipper_pid(),
        "mcu_serial": pi.ssh(
            f"ls {MCU_SERIAL_GLOB} 2>/dev/null || true"
        ).split(),
        "capture_board": os.path.exists(CAPTURE_DEVICE),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.dry_run:
        for name, source in QUERIES.items():
            print(f"{name}: {source}")
        return
    state = collect()
    if args.json:
        print(json.dumps(state, indent=2))
        return
    deployed = state["deployed"]
    print(f"printer    {state['printer_state']} {state['state_message']}")
    print(
        f"deployed   {deployed['head']} {deployed['branch']} "
        f"dirty={len(deployed['dirty_paths'])} {deployed['worktree']} "
        f"@ {deployed['timestamp']}"
    )
    print(f"klipper    pid {state['klipper_pid']}")
    print(f"mcu serial {', '.join(state['mcu_serial']) or 'MISSING'}")
    print(f"capture    {'present' if state['capture_board'] else 'MISSING'}")


if __name__ == "__main__":
    main()
