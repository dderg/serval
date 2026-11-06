"""Real-time smoke: klippy streams servo samples into ethercat-rt-stub.

klippy runs live against a Klipper linux-process MCU, which supplies the
print_time clock; every axis is an EtherCAT servo, so the MCU drives no pins.
See ethercat_live_support for the binaries it needs.
"""

from __future__ import annotations

import threading
import time

import pytest
from ethercat_live_support import (
    API_TIMEOUT_S,
    STUB,
    STUB_CYCLE_US,
    Api,
    Printer,
    needs_live,
    short_tmp,
    spawn_stub,
    wait_for,
)
from ethercat_support import GCODE

pytestmark = needs_live

SETTLE_S = 1.5
COUNTS_PER_MM = [131072 / 40, 131072 / 40, 131072 / 8]

PRINTER_CFG = """
[mcu]
serial: {mcu_tty}

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 5000
max_z_velocity: 10
max_z_accel: 100

[ethercat]
cycle_us: {cycle_us}
socket: {socket}
{endpoint}

[stepper_x]
servo: ethercat
ethercat_slave: 0
ethercat_axis: 0
encoder_counts_per_rev: 131072
rotation_distance: 40
endstop_pin: ethercat:x
position_endstop: 0
position_max: 250

[stepper_y]
servo: ethercat
ethercat_slave: 1
ethercat_axis: 1
encoder_counts_per_rev: 131072
rotation_distance: 40
endstop_pin: ethercat:y
position_endstop: 0
position_max: 250

[stepper_z]
servo: ethercat
ethercat_slave: 2
ethercat_axis: 2
encoder_counts_per_rev: 131072
rotation_distance: 8
endstop_pin: ethercat:z
position_endstop: 0
position_max: 200

[input_shaper]
shaper_type: smooth_mzv
shaper_freq_x: 50
shaper_freq_y: 40
"""

LONG_MOVES = "\n".join(
    "G1 X%d Y%d F3000" % (50 + 100 * (i % 2), 50 + 100 * ((i // 2) % 2))
    for i in range(40)
)
SECOND_PRINT = "G1 X60 Y140 Z12 F6000\nG1 X100 Y100 Z10 F6000\nM400"


@pytest.fixture
def tmp(tmp_path_factory):
    return short_tmp(tmp_path_factory)


def printer_cfg(tmp, endpoint):
    return PRINTER_CFG.format(
        mcu_tty="{mcu_tty}",
        cycle_us=STUB_CYCLE_US,
        socket=tmp / "ec.sock",
        endpoint=endpoint,
    )


def test_live_print_streams_every_cycle_to_the_stub(tmp):
    printer = Printer(tmp, printer_cfg(tmp, "endpoint_binary: %s" % (STUB,)))
    try:
        assert printer.state()[0] == "ready", printer.state()
        resp = printer.api.script(GCODE)
        assert "error" not in resp, resp
        # Idle long enough for real time to pass the stream's end, so the
        # next print must re-anchor after verifying the gap held still
        time.sleep(SETTLE_S)
        resp = printer.api.script(SECOND_PRINT)
        assert "error" not in resp, resp
        time.sleep(SETTLE_S)
        state = printer.state()
        status = printer.api.status("ethercat")["ethercat"]
        assert state[0] == "ready", state
        assert status["fault"] == ""
        assert status["anchors"] >= 2
        # Every cycle the plugin pushed was played by every slot: no hole,
        # no late run, no underrun, nothing discarded.
        assert status["retired_cycles"] == [status["pushed_cycles"]] * 3
    finally:
        printer.close()
    log = printer.log.read_text()
    assert "ethercat: endpoint" in log and "claimed 3 slaves" in log


def test_endpoint_death_mid_print_shuts_down(tmp):
    stub = spawn_stub(
        tmp / "ec.sock", [(axis, COUNTS_PER_MM[axis]) for axis in range(3)]
    )
    printer = Printer(tmp, printer_cfg(tmp, ""))
    try:
        assert printer.state()[0] == "ready", printer.state()
        printer.api.script("SET_KINEMATIC_POSITION X=100 Y=100 Z=10")
        script_api = Api(printer.api_path)
        script_result = []
        moving = threading.Thread(
            target=lambda: script_result.append(script_api.script(LONG_MOVES)),
            daemon=True,
        )
        moving.start()

        def playing():
            retired = printer.api.status("ethercat")["ethercat"][
                "retired_cycles"
            ]
            return bool(retired) and min(retired) > 0

        wait_for(playing, "the stub to play samples")
        stub.kill()
        wait_for(
            lambda: printer.state()[0] == "shutdown", "klippy to shut down"
        )
        state, message = printer.state()
        assert "ethercat: lost the endpoint" in message, message
        moving.join(API_TIMEOUT_S)
        assert "error" in script_result[0], script_result
    finally:
        stub.kill()
        stub.wait()
        printer.close()
