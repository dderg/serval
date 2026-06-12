import json
import socket
import time

import pytest

from tools.sim_klippy.orchestrator.sim_control_client import SimControlClient

pytestmark = pytest.mark.needs_elf

# Must match pin-overrides.toml [stepper_*.config_set] endstop_pin lines.
X_ENDSTOP_LINE = 200
Y_ENDSTOP_LINE = 201

# pin-overrides.toml pins position_endstop = position_max = 20 for both axes;
# these tests override homing_retract_dist to RETRACT_DIST, so a correctly
# behaving homing sequence leaves the head at position_endstop - retract_dist.
ENDSTOP_POS = 20.0
RETRACT_DIST = 5.0
EXPECTED_GCODE_POS = ENDSTOP_POS - RETRACT_DIST

_RETRACT_X = {"stepper_x.config_set": {"homing_retract_dist": str(RETRACT_DIST)}}
_RETRACT_Y = {"stepper_y.config_set": {"homing_retract_dist": str(RETRACT_DIST)}}


def _request(api_socket: str, req: dict) -> dict:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(3.0)
    s.connect(api_socket)
    s.sendall(json.dumps(req).encode() + b"\x03")
    buf = b""
    while True:
        try:
            c = s.recv(4096)
        except Exception:
            break
        if not c:
            break
        buf += c
        if b"\x03" in buf:
            break
    s.close()
    out = buf.split(b"\x03", 1)[0]
    try:
        return json.loads(out.decode()) if out else {}
    except Exception:
        return {}


def _info(api_socket: str) -> dict:
    return _request(api_socket, {"id": 1, "method": "info", "params": {}})


def _query(api_socket: str, objects: dict) -> dict:
    r = _request(
        api_socket,
        {"id": 1, "method": "objects/query", "params": {"objects": objects}},
    )
    return r.get("result", {}).get("status", {})


def _wait_ready(sim, timeout: float = 30.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = _info(sim.api_socket)
        if (r.get("result") or {}).get("state") == "ready":
            return
        time.sleep(0.5)
    pytest.fail("klippy not ready before homing test")


def _set_pin(sim, line: int, value: int) -> None:
    with SimControlClient(sim.h7_sim_control) as c:
        c.set_gpio_input(chip=0, line=line, value=value)


@pytest.mark.parametrize(
    "sim_extra_overrides,axis,axis_idx,line,gcode",
    [
        (_RETRACT_X, "x", 0, X_ENDSTOP_LINE, "G28 X"),
        (_RETRACT_Y, "y", 1, Y_ENDSTOP_LINE, "G28 Y"),
    ],
    indirect=["sim_extra_overrides"],
)
def test_g28_retract_updates_gcode_position(
    sim, axis, axis_idx, line, gcode
):
    """gcode_move position must reflect the post-retract toolhead position.

    Regression guard for the dropped homing:home_rails_end event: homing
    set_position fires before the retract, so without the end-of-homing
    event gcode_move freezes at the endstop coordinate (20) while the
    toolhead actually backs off to 15. M114 / the UI / the next G1 origin
    would all use the stale value.
    """
    _wait_ready(sim)
    _set_pin(sim, line, 1)

    r = sim.gcode(gcode, timeout=30.0)
    assert "error" not in r or not r.get("error"), (
        f"{gcode} failed: {r.get('error')}"
    )

    st = _query(
        sim.api_socket,
        {"toolhead": ["position"], "gcode_move": ["position"]},
    )
    toolhead_pos = st.get("toolhead", {}).get("position", [None] * 4)
    gcode_pos = st.get("gcode_move", {}).get("position", [None] * 4)

    th = toolhead_pos[axis_idx]
    gc = gcode_pos[axis_idx]

    assert th is not None and abs(th - EXPECTED_GCODE_POS) < 1.0, (
        f"toolhead {axis} expected ~{EXPECTED_GCODE_POS} after retract, "
        f"got {th} (toolhead={toolhead_pos})"
    )
    assert gc is not None and abs(gc - EXPECTED_GCODE_POS) < 1.0, (
        f"gcode_move {axis} expected ~{EXPECTED_GCODE_POS} (endstop "
        f"{ENDSTOP_POS} minus retract {RETRACT_DIST}); got {gc}. A value "
        f"near {ENDSTOP_POS} means homing:home_rails_end was not emitted "
        f"and gcode_move froze at the pre-retract coordinate "
        f"(gcode_move={gcode_pos}, toolhead={toolhead_pos})."
    )
