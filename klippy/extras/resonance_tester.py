# A utility class to test resonances of the printer
#
# Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import multiprocessing
import os
import time
from collections import namedtuple

from . import shaper_calibrate
from .resonance_buzz import servo_buzz_motor_names

MAX_BUZZ_FREQ = 800.0

SweepParams = namedtuple(
    "SweepParams",
    "freq_start freq_end accel_per_hz duration ramp amplitude_mm",
)


def _write_samples_to_file(filename, samples, chirp_line):
    with open(filename, "w") as f:
        if chirp_line is not None:
            f.write(chirp_line + "\n")
        f.write("#time,accel_x,accel_y,accel_z\n")
        for t, accel_x, accel_y, accel_z in samples:
            f.write("%.6f,%.6f,%.6f,%.6f\n" % (t, accel_x, accel_y, accel_z))


def chirp_metadata_line(sweep, amplitude_mm, graph_max_freq=None):
    line = (
        "# chirp freq_start=%.3f freq_end=%.3f duration=%.3f ramp=%.4f"
        " accel_per_hz=%.3f amplitude_mm=%.6f"
        % (
            sweep.freq_start,
            sweep.freq_end,
            sweep.duration,
            sweep.ramp,
            sweep.accel_per_hz,
            amplitude_mm,
        )
    )
    if graph_max_freq is not None:
        line += " graph_max_freq=%.1f" % (graph_max_freq,)
    return line


def write_raw_data_blocking(printer, aclient, filename, chirp_line=None):
    samples = aclient.get_samples()
    write_proc = multiprocessing.Process(
        target=_write_samples_to_file, args=(filename, samples, chirp_line)
    )
    write_proc.start()
    reactor = printer.get_reactor()
    while write_proc.is_alive():
        reactor.pause(reactor.monotonic() + 0.050)
    if write_proc.exitcode != 0:
        raise printer.command_error(
            "Writing raw accelerometer data to %s failed (exit code %s)"
            % (filename, write_proc.exitcode)
        )


class TestAxis:
    def __init__(self, axis):
        self._name = axis

    def matches(self, chip_axis):
        if self._name == "z":
            return True
        return self._name in chip_axis

    def get_name(self):
        return self._name

    def buzz_axis(self):
        return self._name


def _parse_axis(gcmd, raw_axis):
    if raw_axis is None:
        raise gcmd.error("AXIS parameter is required")
    raw_axis = raw_axis.lower()
    if raw_axis in ("x", "y", "z"):
        return TestAxis(raw_axis)
    dirs = raw_axis.split(",")
    if len(dirs) != 2:
        raise gcmd.error("Invalid format of axis '%s'" % (raw_axis,))
    try:
        dir_x = float(dirs[0].strip())
        dir_y = float(dirs[1].strip())
    except ValueError:
        raise gcmd.error("Unable to parse axis direction '%s'" % (raw_axis,))
    if dir_x != 0.0 and dir_y == 0.0:
        return TestAxis("x")
    if dir_y != 0.0 and dir_x == 0.0:
        return TestAxis("y")
    raise gcmd.error("diagonal buzz is not implemented yet")


class ResonanceTester:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.load_object(config, "resonance_buzz")
        self.printer.load_object(config, "servo_capture")
        self.move_speed = config.getfloat("move_speed", 50.0, above=0.0)
        self.min_freq = config.getfloat("min_freq", 5.0, minval=1.0)
        self.max_freq = config.getfloat(
            "max_freq", 135.0, minval=self.min_freq, maxval=MAX_BUZZ_FREQ
        )
        self.accel_per_hz = config.getfloat("accel_per_hz", 75.0, above=0.0)
        self.graph_max_freq = config.getfloat("graph_max_freq", None, above=0.0)
        self.hz_per_sec = config.getfloat(
            "hz_per_sec", 1.0, minval=0.1, maxval=2.0
        )
        self.max_smoothing = config.getfloat("max_smoothing", None, minval=0.05)
        self.probe_points = config.getlists(
            "probe_points", None, seps=(",", "\n"), parser=float, count=3
        )

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

    def _parse_chips(self, accel_chips):
        if not accel_chips:
            return None
        parsed_chips = []
        for chip_name in accel_chips.split(","):
            chip = self.printer.lookup_object(chip_name.strip())
            parsed_chips.append(chip)
        return parsed_chips

    def _parse_point(self, gcmd):
        test_point = gcmd.get("POINT", None)
        if test_point is None:
            return None
        coords = test_point.split(",")
        if len(coords) != 3:
            raise gcmd.error("Invalid POINT parameter, must be 'x,y,z'")
        try:
            return [float(p.strip()) for p in coords]
        except ValueError:
            raise gcmd.error(
                "Invalid POINT parameter, must be 'x,y,z'"
                " where x, y and z are valid floating point numbers"
            )

    def _parse_sweep(self, gcmd, test_accel_per_hz=None):
        freq_start = gcmd.get_float("FREQ_START", self.min_freq, minval=1.0)
        freq_end = gcmd.get_float(
            "FREQ_END", self.max_freq, minval=freq_start, maxval=MAX_BUZZ_FREQ
        )
        if test_accel_per_hz is None:
            accel_per_hz = gcmd.get_float(
                "ACCEL_PER_HZ", self.accel_per_hz, above=0.0
            )
        else:
            accel_per_hz = test_accel_per_hz
        hz_per_sec = gcmd.get_float(
            "HZ_PER_SEC", self.hz_per_sec, above=0.0, maxval=2.0
        )
        amplitude_mm = gcmd.get_float("AMPLITUDE", 0.0, minval=0.0)
        duration = max(abs(freq_end - freq_start) / hz_per_sec, 0.1)
        ramp = min(0.1 * duration, 3.0 / min(freq_start, freq_end))
        return SweepParams(
            freq_start, freq_end, accel_per_hz, duration, ramp, amplitude_mm
        )

    def _run_test(
        self,
        gcmd,
        axes,
        helper,
        sweep,
        raw_name_suffix=None,
        accel_chips=None,
        test_point=None,
        capture_name_suffix=None,
    ):
        toolhead = self.printer.lookup_object("toolhead")
        buzz = self.printer.lookup_object("resonance_buzz")
        calibration_data = {axis: None for axis in axes}
        if capture_name_suffix is None:
            capture_name_suffix = time.strftime("%Y%m%d_%H%M%S")

        test_points = (
            [test_point] if test_point else (self.probe_points or [None])
        )

        for point in test_points:
            if point is not None:
                toolhead.manual_move(point, self.move_speed)
                if len(test_points) > 1 or test_point is not None:
                    gcmd.respond_info(
                        "Probing point (%.3f, %.3f, %.3f)" % tuple(point)
                    )
            for axis in axes:
                toolhead.wait_moves()
                toolhead.dwell(0.500)
                if len(axes) > 1:
                    gcmd.respond_info("Testing axis %s" % axis.get_name())

                raw_values = []
                if accel_chips is None:
                    for chip_axis, chip in self.accel_chips:
                        if axis.matches(chip_axis):
                            aclient = chip.start_internal_client()
                            raw_values.append((chip_axis, aclient, chip.name))
                else:
                    for chip in accel_chips:
                        aclient = chip.start_internal_client()
                        raw_values.append((axis.get_name(), aclient, chip.name))

                servo_names = servo_buzz_motor_names(
                    self.printer, axis.buzz_axis()
                )
                scap = None
                if servo_names:
                    scap = self.printer.lookup_object("servo_capture")
                    scap_path = scap.capture_path(
                        self.get_data_name(
                            "raw_servo",
                            capture_name_suffix,
                            axis,
                            point if len(test_points) > 1 else None,
                        )
                    )
                    scap.start_capture_to(scap_path, servo_names)
                try:
                    amplitude_mm = buzz.run_sweep(
                        gcmd,
                        axis.buzz_axis(),
                        sweep.freq_start,
                        sweep.freq_end,
                        sweep.duration,
                        sweep.ramp,
                        sweep.accel_per_hz,
                        sweep.amplitude_mm,
                    )
                finally:
                    if scap is not None:
                        scap_path, scap_samples, cycle_us = scap.stop_capture()
                if scap is not None:
                    gcmd.respond_info(
                        "Servo encoder capture written to %s file "
                        "(%d samples at %.1f kHz)"
                        % (scap_path, scap_samples, 1000.0 / cycle_us)
                    )

                chirp_line = chirp_metadata_line(
                    sweep, amplitude_mm, self.graph_max_freq
                )
                for chip_axis, aclient, chip_name in raw_values:
                    aclient.finish_measurements()
                    if raw_name_suffix is not None:
                        raw_name = self.get_filename(
                            "raw_data",
                            raw_name_suffix,
                            axis,
                            point if len(test_points) > 1 else None,
                            chip_name,
                        )
                        write_raw_data_blocking(
                            self.printer, aclient, raw_name, chirp_line
                        )
                        gcmd.respond_info(
                            "Raw accelerometer data written to "
                            "%s file" % (raw_name,)
                        )
                if helper is None:
                    continue
                for chip_axis, aclient, chip_name in raw_values:
                    if not aclient.has_valid_samples():
                        raise gcmd.error(
                            "accelerometer '%s' measured no data" % (chip_name,)
                        )
                    new_data = helper.process_accelerometer_data(aclient)
                    if calibration_data[axis] is None:
                        calibration_data[axis] = new_data
                    else:
                        calibration_data[axis].add_data(new_data)
        return calibration_data

    cmd_TEST_RESONANCES_help = "Runs the resonance test for a specifed axis"

    def cmd_TEST_RESONANCES(self, gcmd):
        axis = _parse_axis(gcmd, gcmd.get("AXIS"))
        chips_str = gcmd.get("CHIPS", None)
        test_point = self._parse_point(gcmd)
        test_accel_per_hz = gcmd.get_float("ACCEL_PER_HZ", None, above=0.0)
        accel_chips = self._parse_chips(chips_str) if chips_str else None

        outputs = gcmd.get("OUTPUT", "resonances").lower().split(",")
        for output in outputs:
            if output not in ("resonances", "raw_data"):
                raise gcmd.error(
                    "Unsupported output '%s', only 'resonances'"
                    " and 'raw_data' are supported" % (output,)
                )
        if not outputs:
            raise gcmd.error(
                "No output specified, at least one of 'resonances'"
                " or 'raw_data' must be set in OUTPUT parameter"
            )
        name_suffix = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        if not self.is_valid_name_suffix(name_suffix):
            raise gcmd.error("Invalid NAME parameter")
        csv_output = "resonances" in outputs
        raw_output = "raw_data" in outputs

        helper = (
            shaper_calibrate.ShaperCalibrate(self.printer)
            if csv_output
            else None
        )
        sweep = self._parse_sweep(gcmd, test_accel_per_hz)

        data = self._run_test(
            gcmd,
            [axis],
            helper,
            sweep,
            raw_name_suffix=name_suffix if raw_output else None,
            accel_chips=accel_chips,
            test_point=test_point,
            capture_name_suffix=name_suffix,
        )[axis]
        if csv_output:
            csv_name = self.save_calibration_data(
                "resonances",
                name_suffix,
                helper,
                axis,
                data,
                point=test_point,
                max_freq=1.5 * sweep.freq_end,
                accel_per_hz=sweep.accel_per_hz,
            )
            gcmd.respond_info(
                "Resonances data written to %s file" % (csv_name,)
            )

    cmd_SHAPER_CALIBRATE_help = (
        "Simular to TEST_RESONANCES but suggest input shaper config"
    )

    def cmd_SHAPER_CALIBRATE(self, gcmd):
        axis = gcmd.get("AXIS", None)
        if not axis:
            calibrate_axes = [TestAxis("x"), TestAxis("y")]
        elif axis.lower() not in ("x", "y"):
            raise gcmd.error("Unsupported axis '%s'" % (axis,))
        else:
            calibrate_axes = [TestAxis(axis.lower())]
        chips_str = gcmd.get("CHIPS", None)
        accel_chips = self._parse_chips(chips_str) if chips_str else None

        max_smoothing = gcmd.get_float(
            "MAX_SMOOTHING", self.max_smoothing, minval=0.05
        )

        name_suffix = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        if not self.is_valid_name_suffix(name_suffix):
            raise gcmd.error("Invalid NAME parameter")

        input_shaper = self.printer.lookup_object("input_shaper", None)
        helper = shaper_calibrate.ShaperCalibrate(self.printer)
        sweep = self._parse_sweep(gcmd)

        calibration_data = self._run_test(
            gcmd,
            calibrate_axes,
            helper,
            sweep,
            accel_chips=accel_chips,
            capture_name_suffix=name_suffix,
        )

        configfile = self.printer.lookup_object("configfile")
        max_freq = 1.5 * sweep.freq_end
        for axis in calibrate_axes:
            axis_name = axis.get_name()
            gcmd.respond_info(
                "Calculating the best input shaper parameters for %s axis"
                % (axis_name,)
            )
            calibration_data[axis].normalize_to_frequencies()
            systime = self.printer.get_reactor().monotonic()
            toolhead = self.printer.lookup_object("toolhead")
            scv = toolhead.get_status(systime)["square_corner_velocity"]
            best_shaper, all_shapers = helper.find_best_shaper(
                calibration_data[axis],
                max_smoothing=max_smoothing,
                scv=scv,
                max_freq=max_freq,
                logger=gcmd.respond_info,
            )
            gcmd.respond_info(
                "Recommended shaper_type_%s = %s, shaper_freq_%s = %.1f Hz"
                % (axis_name, best_shaper.name, axis_name, best_shaper.freq)
            )
            if input_shaper is not None:
                helper.apply_params(
                    input_shaper, axis_name, best_shaper.name, best_shaper.freq
                )
            helper.save_params(
                configfile, axis_name, best_shaper.name, best_shaper.freq
            )
            csv_name = self.save_calibration_data(
                "calibration_data",
                name_suffix,
                helper,
                axis,
                calibration_data[axis],
                all_shapers,
                max_freq=max_freq,
                accel_per_hz=sweep.accel_per_hz,
            )
            gcmd.respond_info(
                "Shaper calibration data written to %s file" % (csv_name,)
            )
        gcmd.respond_info(
            "The SAVE_CONFIG command will update the printer config file\n"
            "with these parameters and restart the printer."
        )

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

    def is_valid_name_suffix(self, name_suffix):
        return name_suffix.replace("-", "").replace("_", "").isalnum()

    def get_data_name(
        self, base, name_suffix, axis=None, point=None, chip_name=None
    ):
        name = base
        if axis:
            name += "_" + axis.get_name()
        if chip_name:
            name += "_" + chip_name.replace(" ", "_")
        if point:
            name += "_%.3f_%.3f_%.3f" % (point[0], point[1], point[2])
        return name + "_" + name_suffix

    def get_filename(
        self, base, name_suffix, axis=None, point=None, chip_name=None
    ):
        name = self.get_data_name(base, name_suffix, axis, point, chip_name)
        return os.path.join("/tmp", name + ".csv")

    def save_calibration_data(
        self,
        base_name,
        name_suffix,
        helper,
        axis,
        calibration_data,
        all_shapers=None,
        point=None,
        max_freq=None,
        accel_per_hz=None,
    ):
        output = self.get_filename(base_name, name_suffix, axis, point)
        helper.save_calibration_data(
            output, calibration_data, all_shapers, max_freq, accel_per_hz
        )
        return output


def load_config(config):
    return ResonanceTester(config)
