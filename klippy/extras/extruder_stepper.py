# Code for supporting multiple steppers in single filament extruder.
#
# Copyright (C) 2019 Simo Apell <simo.apell@live.fi>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


def load_config_prefix(config):
    raise config.error(
        "[extruder_stepper] is not supported — declare the motor in a "
        "[<motor>] section and add it to [axis <name>] motors:"
    )
