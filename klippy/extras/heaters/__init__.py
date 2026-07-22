# Tracking of PWM controlled heaters and their temperature control
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from ..control_mpc import ControlMPC
from .control import (
    AMBIENT_TEMP,
    CONTROL_ALGOS,
    PID_PARAM_BASE,
    PID_SETTLE_DELTA,
    PID_SETTLE_SLOPE,
    ControlBangBang,
    ControlDualLoopPID,
    ControlInnerPID,
    ControlPID,
    ControlVelocityPID,
    HeaterControl,
)
from .heater import (
    KELVIN_TO_CELSIUS,
    MAX_HEAT_TIME,
    MAX_MAINTHREAD_TIME,
    QUELL_STALE_TIME,
    DualSensorHeater,
    Heater,
)
from .manager import PrinterHeaters
from .profiles import (
    PID_PROFILE_OPTIONS,
    PID_PROFILE_VERSION,
    ProfileManager,
)

__all__ = [
    "AMBIENT_TEMP",
    "CONTROL_ALGOS",
    "ControlBangBang",
    "ControlDualLoopPID",
    "ControlInnerPID",
    "ControlMPC",
    "ControlPID",
    "ControlVelocityPID",
    "DualSensorHeater",
    "Heater",
    "HeaterControl",
    "KELVIN_TO_CELSIUS",
    "MAX_HEAT_TIME",
    "MAX_MAINTHREAD_TIME",
    "PID_PARAM_BASE",
    "PID_PROFILE_OPTIONS",
    "PID_PROFILE_VERSION",
    "PID_SETTLE_DELTA",
    "PID_SETTLE_SLOPE",
    "PrinterHeaters",
    "ProfileManager",
    "QUELL_STALE_TIME",
]


def load_config(config):
    return PrinterHeaters(config)
