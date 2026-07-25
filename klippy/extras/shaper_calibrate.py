# Automatic calibration of input shapers
#
# Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import importlib
import importlib.util
import multiprocessing
import os
import pathlib
import traceback

from . import shaper_defs

MIN_FREQ = 5.0
MAX_FREQ = 1000.0

AUTOTUNE_SHAPERS = ["zv", "mzv", "ei", "2hump_ei", "3hump_ei"]


def _load_shaper_ident():
    candidates = [pathlib.Path(__file__).parent.parent / "_shaper_ident.so"]
    native_dir = os.environ.get("KALICO_NATIVE_DIR")
    if native_dir:
        candidates.append(pathlib.Path(native_dir) / "_shaper_ident.so")
    attempts = []
    for path in candidates:
        if not path.is_file():
            attempts.append("%s (missing)" % (path,))
            continue
        try:
            spec = importlib.util.spec_from_file_location("_shaper_ident", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        except ImportError as e:
            attempts.append("%s (%s)" % (path, e))
            continue
        return module
    raise ImportError(
        "klippy requires the native _shaper_ident module for resonance "
        "calibration; build it with 'make -f Makefile.rust shaper-ident'. "
        "Tried: " + "; ".join(attempts)
    )


_shaper_ident = None

######################################################################
# Frequency response calculation and shaper auto-tuning
######################################################################


class CalibrationData:
    def __init__(self, freq_bins, psd_sum, psd_x, psd_y, psd_z):
        self.freq_bins = freq_bins
        self.psd_sum = psd_sum
        self.psd_x = psd_x
        self.psd_y = psd_y
        self.psd_z = psd_z
        self._psd_list = [self.psd_sum, self.psd_x, self.psd_y, self.psd_z]
        self._psd_map = {
            "x": self.psd_x,
            "y": self.psd_y,
            "z": self.psd_z,
            "all": self.psd_sum,
        }
        self.data_sets = 1

    def add_data(self, other):
        np = self.numpy
        joined_data_sets = self.data_sets + other.data_sets
        for psd, other_psd in zip(self._psd_list, other._psd_list):
            # `other` data may be defined at different frequency bins,
            # interpolating to fix that.
            other_normalized = other.data_sets * np.interp(
                self.freq_bins, other.freq_bins, other_psd
            )
            psd *= self.data_sets
            psd[:] = (psd + other_normalized) * (1.0 / joined_data_sets)
        self.data_sets = joined_data_sets

    def set_numpy(self, numpy):
        self.numpy = numpy

    def normalize_to_frequencies(self):
        for psd in self._psd_list:
            # Avoid division by zero errors
            psd /= self.freq_bins + 0.1
            # Remove low-frequency noise
            low_freqs = self.freq_bins < 2.0 * MIN_FREQ
            psd[low_freqs] *= self.numpy.exp(
                -((2.0 * MIN_FREQ / (self.freq_bins[low_freqs] + 0.1)) ** 2)
                + 1.0
            )

    def get_psd(self, axis="all"):
        return self._psd_map[axis]


CalibrationResult = collections.namedtuple(
    "CalibrationResult",
    ("name", "freq", "vals", "vibrs", "smoothing", "score", "max_accel"),
)


class ShaperCalibrate:
    def __init__(self, printer):
        self.printer = printer
        self.error = printer.command_error if printer else Exception
        try:
            self.numpy = importlib.import_module("numpy")
        except ImportError:
            raise self.error(
                "Failed to import `numpy` module, make sure it was "
                "installed via `~/klippy-env/bin/pip install` (refer to "
                "docs/Measuring_Resonances.md for more details)."
            )
        global _shaper_ident
        if _shaper_ident is None:
            _shaper_ident = _load_shaper_ident()

    def background_process_exec(self, method, args):
        if self.printer is None:
            return method(*args)
        import queuelogger

        parent_conn, child_conn = multiprocessing.Pipe()

        def wrapper():
            queuelogger.clear_bg_logging()
            try:
                res = method(*args)
            except:
                child_conn.send((True, traceback.format_exc()))
                child_conn.close()
                return
            child_conn.send((False, res))
            child_conn.close()

        # Start a process to perform the calculation
        calc_proc = multiprocessing.Process(target=wrapper)
        calc_proc.daemon = True
        calc_proc.start()
        # Wait for the process to finish
        reactor = self.printer.get_reactor()
        gcode = self.printer.lookup_object("gcode")
        eventtime = last_report_time = reactor.monotonic()
        while calc_proc.is_alive():
            if eventtime > last_report_time + 5.0:
                last_report_time = eventtime
                gcode.respond_info("Wait for calculations..", log=False)
            eventtime = reactor.pause(eventtime + 0.1)
        # Return results
        is_err, res = parent_conn.recv()
        if is_err:
            raise self.error("Error in remote calculation: %s" % (res,))
        calc_proc.join()
        parent_conn.close()
        return res

    def calc_freq_response(self, raw_values):
        np = self.numpy
        if raw_values is None:
            return None
        if isinstance(raw_values, np.ndarray):
            samples = raw_values.tolist()
        else:
            samples = raw_values.get_samples()
            if not samples:
                return None

        result = _shaper_ident.calc_freq_response(samples)
        if result is None:
            return None
        freq_bins, psd_sum, psd_x, psd_y, psd_z = result
        return CalibrationData(
            np.asarray(freq_bins),
            np.asarray(psd_sum),
            np.asarray(psd_x),
            np.asarray(psd_y),
            np.asarray(psd_z),
        )

    def process_accelerometer_data(self, data):
        calibration_data = self.background_process_exec(
            self.calc_freq_response, (data,)
        )
        if calibration_data is None:
            raise self.error(
                "Internal error processing accelerometer data %s" % (data,)
            )
        calibration_data.set_numpy(self.numpy)
        return calibration_data

    def fit_shaper(
        self,
        shaper_cfg,
        calibration_data,
        shaper_freqs,
        damping_ratio,
        scv,
        max_smoothing,
        test_damping_ratios,
        max_freq,
    ):
        if not shaper_freqs:
            shaper_freqs = (None, None, None)
        if isinstance(shaper_freqs, tuple):
            shaper_freqs_range = tuple(f or None for f in shaper_freqs)
            shaper_freqs_list = None
        else:
            shaper_freqs_range = None
            shaper_freqs_list = list(shaper_freqs)

        result = _shaper_ident.fit_shaper(
            shaper_cfg.name,
            calibration_data.freq_bins.tolist(),
            calibration_data.psd_sum.tolist(),
            shaper_freqs_range,
            shaper_freqs_list,
            damping_ratio or None,
            scv,
            max_smoothing or None,
            test_damping_ratios or None,
            max_freq or None,
        )
        if result is None:
            return None
        name, freq, vals, vibrs, smoothing, score, max_accel = result
        return CalibrationResult(
            name=name,
            freq=freq,
            vals=self.numpy.asarray(vals),
            vibrs=vibrs,
            smoothing=smoothing,
            score=score,
            max_accel=max_accel,
        )

    def find_best_shaper(
        self,
        calibration_data,
        shapers=None,
        damping_ratio=None,
        scv=None,
        shaper_freqs=None,
        max_smoothing=None,
        test_damping_ratios=None,
        max_freq=None,
        logger=None,
    ):
        best_shaper = None
        all_shapers = []
        shapers = shapers or AUTOTUNE_SHAPERS
        for shaper_cfg in shaper_defs.INPUT_SHAPERS:
            if shaper_cfg.name not in shapers:
                continue
            shaper = self.background_process_exec(
                self.fit_shaper,
                (
                    shaper_cfg,
                    calibration_data,
                    shaper_freqs,
                    damping_ratio,
                    scv,
                    max_smoothing,
                    test_damping_ratios,
                    max_freq,
                ),
            )
            if logger is not None:
                logger(
                    "Fitted shaper '%s' frequency = %.1f Hz "
                    "(vibrations = %.1f%%, smoothing ~= %.3f)"
                    % (
                        shaper.name,
                        shaper.freq,
                        shaper.vibrs * 100.0,
                        shaper.smoothing,
                    )
                )
                logger(
                    "To avoid too much smoothing with '%s', suggested "
                    "max_accel <= %.0f mm/sec^2"
                    % (shaper.name, round(shaper.max_accel / 100.0) * 100.0)
                )
            all_shapers.append(shaper)
            if (
                best_shaper is None
                or shaper.score * 1.2 < best_shaper.score
                or (
                    shaper.score * 1.05 < best_shaper.score
                    and shaper.smoothing * 1.1 < best_shaper.smoothing
                )
            ):
                # Either the shaper significantly improves the score (by 20%),
                # or it improves the score and smoothing (by 5% and 10% resp.)
                best_shaper = shaper
        return best_shaper, all_shapers

    def save_params(self, configfile, axis, shaper_name, shaper_freq):
        if axis == "xy":
            self.save_params(configfile, "x", shaper_name, shaper_freq)
            self.save_params(configfile, "y", shaper_name, shaper_freq)
        else:
            configfile.set("input_shaper", "shaper_type_" + axis, shaper_name)
            configfile.set(
                "input_shaper", "shaper_freq_" + axis, "%.1f" % (shaper_freq,)
            )

    def apply_params(self, input_shaper, axis, shaper_name, shaper_freq):
        if axis == "xy":
            self.apply_params(input_shaper, "x", shaper_name, shaper_freq)
            self.apply_params(input_shaper, "y", shaper_name, shaper_freq)
            return
        gcode = self.printer.lookup_object("gcode")
        axis = axis.upper()
        input_shaper.cmd_SET_INPUT_SHAPER(
            gcode.create_gcode_command(
                "SET_INPUT_SHAPER",
                "SET_INPUT_SHAPER",
                {
                    "SHAPER_TYPE_" + axis: shaper_name,
                    "SHAPER_FREQ_" + axis: shaper_freq,
                },
            )
        )

    def save_calibration_data(
        self,
        output,
        calibration_data,
        shapers=None,
        max_freq=None,
        accel_per_hz=None,
    ):
        try:
            max_freq = max_freq or MAX_FREQ
            with open(output, "w") as csvfile:
                csvfile.write("freq,psd_x,psd_y,psd_z,psd_xyz,accel_per_hz")
                if shapers:
                    for shaper in shapers:
                        csvfile.write(",%s(%.1f)" % (shaper.name, shaper.freq))
                csvfile.write("\n")
                num_freqs = calibration_data.freq_bins.shape[0]
                for i in range(num_freqs):
                    if calibration_data.freq_bins[i] >= max_freq:
                        break
                    csvfile.write(
                        "%.1f,%.3e,%.3e,%.3e,%.3e,%.1f"
                        % (
                            calibration_data.freq_bins[i],
                            calibration_data.psd_x[i],
                            calibration_data.psd_y[i],
                            calibration_data.psd_z[i],
                            calibration_data.psd_sum[i],
                            accel_per_hz,
                        )
                    )
                    if shapers:
                        for shaper in shapers:
                            csvfile.write(",%.3f" % (shaper.vals[i],))
                    csvfile.write("\n")
        except IOError as e:
            raise self.error("Error writing to file '%s': %s", output, str(e))
