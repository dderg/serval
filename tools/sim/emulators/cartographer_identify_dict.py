"""Identify dictionary served by the CartographerMcuStub.

Same contract as beacon_identify_dict.py: every command/response format
string scanner.py calls ``lookup_command()`` / ``lookup_query_command()``
for, byte-for-byte, plus the core klippy MCU bring-up surface and the
trsync commands the kalico seam (scanner_motion_engine.py) arms. Source
of truth is ``tools/sim/third_party_repos/cartographer_klipper/
scanner.py`` (dderg/cartographer-klipper, ``kalico-seam`` branch); the
config section sets ``sensor: cartographer`` so every device command is
prefixed ``cartographer_``.

Constants scanner.py reads at ``_handle_mcu_identify``: ``CLOCK_FREQ``
(20 MHz lands in the 20..100 MHz band, so scanner derives
``sensor_freq = CLOCK_FREQ / 2``), ``ADC_MAX``, and
``CARTOGRAPHER_ADC_SMOOTH_COUNT``. The ``version`` string doubles as
the firmware version the saved scanner model must match
(``model_fw_version`` — ScannerModel.validate raises on mismatch).
"""

from __future__ import annotations

import json
import zlib

CORE_COMMANDS = [
    "get_uptime",
    "get_clock",
    "get_config",
    "allocate_oids count=%c",
    "finalize_config crc=%u",
    "emergency_stop",
    "clear_shutdown",
    "debug_nop",
    "debug_ping data=%*s",
    "debug_read order=%c addr=%u",
    "debug_write order=%c addr=%u val=%u",
    "config_trsync oid=%c",
    "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c",
    "trsync_set_timeout oid=%c clock=%u",
    "trsync_trigger oid=%c reason=%c",
    "stepper_stop_on_trigger oid=%c trsync_oid=%c",
]

CARTOGRAPHER_COMMANDS = [
    "cartographer_stream en=%u",
    "cartographer_set_threshold trigger=%u untrigger=%u",
    "cartographer_home trsync_oid=%c trigger_reason=%c trigger_invert=%c"
    " threshold=%u trigger_method=%u",
    "cartographer_stop_home",
    "cartographer_base_read len=%c offset=%hu",
]

CORE_RESPONSES = [
    "identify_response offset=%u data=%.*s",
    "uptime high=%u clock=%u",
    "clock clock=%u",
    "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
    "stats count=%u sum=%u sumsq=%u",
    "shutdown clock=%u static_string_id=%hu",
    "is_shutdown static_string_id=%hu",
    "pong data=%*s",
    "debug_result val=%u",
    "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
]

CARTOGRAPHER_RESPONSES = [
    "cartographer_data clock=%u data=%u temp=%u",
    "cartographer_base_data bytes=%*s offset=%hu",
]

CLOCK_FREQ = 20_000_000
SENSOR_FREQ = CLOCK_FREQ / 2

VERSION = "v0.0.0-sim"

CONFIG = {
    "MCU": "cartographer",
    "CLOCK_FREQ": CLOCK_FREQ,
    "STATS_SUMSQ_BASE": 1,
    "ADC_MAX": 4095,
    "CARTOGRAPHER_ADC_SMOOTH_COUNT": 16,
}


def build_identify_dict() -> dict:
    next_id = 2
    commands: dict = {}
    responses: dict = {}

    for fmt in CORE_COMMANDS + CARTOGRAPHER_COMMANDS:
        commands[fmt] = next_id
        next_id += 1
    for fmt in CORE_RESPONSES + CARTOGRAPHER_RESPONSES:
        if fmt.startswith("identify_response"):
            responses[fmt] = 0
            continue
        responses[fmt] = next_id
        next_id += 1

    return {
        "app": "CartographerStub",
        "version": VERSION,
        "build_versions": "sim",
        "license": "GPL-3.0-or-later",
        "enumerations": {
            "pin": {"gpio0": [0, 32]},
            "static_string_id": {"shutdown": 0},
        },
        "commands": commands,
        "responses": responses,
        "output": {},
        "config": CONFIG,
    }


def build_identify_blob() -> bytes:
    raw = json.dumps(build_identify_dict()).encode("utf-8")
    return zlib.compress(raw)


IDENTIFY_BLOB = build_identify_blob()
