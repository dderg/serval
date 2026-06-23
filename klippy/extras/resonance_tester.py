# A utility class to test resonances of the printer
#
# Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import shaper_calibrate

NOT_IMPLEMENTED_MSG = (
    "%s is not implemented in this build: the move-based resonance sweep "
    "was removed. A buzz-based implementation is pending."
)


class ResonanceTester:
    def __init__(self, config):
        self.printer = config.get_printer()

        accel_chips = config.get("accel_chips", None)
        accel_chip = config.get("accel_chip", None)
        accel_chip_x = config.get("accel_chip_x", None)
        accel_chip_y = config.get("accel_chip_y", None)

        if accel_chips is not None:
            chip_names = [chip.strip() for chip in accel_chips.split(",")]
            self.accel_chip_names = [("xy", chip) for chip in chip_names]
        elif accel_chip_x is not None:
            self.accel_chip_names = [
                ("x", accel_chip_x.strip()),
                ("y", accel_chip_y.strip()),
            ]
            if self.accel_chip_names[0][1] == self.accel_chip_names[1][1]:
                self.accel_chip_names = [("xy", self.accel_chip_names[0][1])]
        elif accel_chip is not None:
            self.accel_chip_names = [("xy", accel_chip.strip())]
        else:
            raise config.error(
                "No accelerometer chips configured. At least one of accel_chips,"
                " accel_chip, or accel_chip_x/accel_chip_y must be specified."
            )

        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "MEASURE_AXES_NOISE",
            self.cmd_MEASURE_AXES_NOISE,
            desc=self.cmd_MEASURE_AXES_NOISE_help,
        )
        self.gcode.register_command(
            "TEST_RESONANCES",
            self.cmd_TEST_RESONANCES,
            desc=self.cmd_TEST_RESONANCES_help,
        )
        self.gcode.register_command(
            "SHAPER_CALIBRATE",
            self.cmd_SHAPER_CALIBRATE,
            desc=self.cmd_SHAPER_CALIBRATE_help,
        )
        self.printer.register_event_handler("klippy:connect", self.connect)

    def connect(self):
        self.accel_chips = []
        for chip_axis, chip_name in self.accel_chip_names:
            try:
                chip = self.printer.lookup_object(chip_name)
                self.accel_chips.append((chip_axis, chip))
            except self.printer.config_error as e:
                logging.exception(
                    "Error looking up accelerometer chip '%s': %s",
                    chip_name,
                    str(e),
                )
                raise

    cmd_TEST_RESONANCES_help = "Runs the resonance test for a specifed axis"

    def cmd_TEST_RESONANCES(self, gcmd):
        raise gcmd.error(NOT_IMPLEMENTED_MSG % ("TEST_RESONANCES",))

    cmd_SHAPER_CALIBRATE_help = (
        "Simular to TEST_RESONANCES but suggest input shaper config"
    )

    def cmd_SHAPER_CALIBRATE(self, gcmd):
        raise gcmd.error(NOT_IMPLEMENTED_MSG % ("SHAPER_CALIBRATE",))

    cmd_MEASURE_AXES_NOISE_help = (
        "Measures noise of all enabled accelerometer chips"
    )

    def cmd_MEASURE_AXES_NOISE(self, gcmd):
        meas_time = gcmd.get_float("MEAS_TIME", 2.0)
        raw_values = [
            (chip_axis, chip.start_internal_client())
            for chip_axis, chip in self.accel_chips
        ]
        self.printer.lookup_object("toolhead").dwell(meas_time)
        for chip_axis, aclient in raw_values:
            aclient.finish_measurements()
        helper = shaper_calibrate.ShaperCalibrate(self.printer)
        for chip_axis, aclient in raw_values:
            if not aclient.has_valid_samples():
                raise gcmd.error(
                    "%s-axis accelerometer measured no data" % (chip_axis,)
                )
            data = helper.process_accelerometer_data(aclient)
            vx = data.psd_x.mean()
            vy = data.psd_y.mean()
            vz = data.psd_z.mean()
            gcmd.respond_info(
                "Axes noise for %s-axis accelerometer: "
                "%.6f (x), %.6f (y), %.6f (z)" % (chip_axis, vx, vy, vz)
            )


def load_config(config):
    return ResonanceTester(config)
