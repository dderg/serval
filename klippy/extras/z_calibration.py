# Klipper plugin for a self-calibrating Z offset.
#
# Copyright (C) 2021-2023  Titus Meyer <info@protoloft.org>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


def load_config(config):
    raise config.error(
        "z_calibration is not implemented yet on the motion-engine rewrite: "
        "it depends on the stepper/microstep model the host no longer has. "
        "Remove the [z_calibration] section."
    )
