# Kinematic input shaper configuration for the corner-blending planner
#
# Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020-2023  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Plan 8 Chunk 2 Task 12 — the post-hoc step-generator kin_shaper.c cascade
# is retired. The [input_shaper] config section now exists purely to carry
# per-axis shaper_type / shaper_freq / damping_ratio parameters that the
# planner (`klippy/blendplanner.py::_bake_shaper_polynomial`) reads at
# emit time to bake the kernel directly into the quintic polynomial.
#
# SET_INPUT_SHAPER still works — it flushes the toolhead, updates the
# config, and forces a step-gen resync so the next emitted polynomial
# picks up the new shaper. No FFI calls, no fused-kernel, no target_passband.

import collections

from . import shaper_defs


# Smoother-family name set — used for config-validation and the shaper_* /
# smoother_* polymorphism. Covers both the cardinal B-spline chain
# (bs1..bs5) and the legacy smooth-IS family (smooth_zv, smooth_mzv,
# smooth_ei, smooth_2hump_ei, smooth_zvd_ei, smooth_si).
_SMOOTHER_FAMILY_NAMES = frozenset(s.name for s in shaper_defs.INPUT_SMOOTHERS)


def parse_float_list(list_str):
    def parse_str(s):
        res = []
        for line in s.split("\n"):
            for coeff in line.split(","):
                res.append(float(coeff.strip()))
        return res

    try:
        return parse_str(list_str)
    except:
        return None


class TypedInputShaperParams:
    shapers = {s.name: s.init_func for s in shaper_defs.INPUT_SHAPERS}

    def __init__(self, axis, shaper_type, config):
        self.axis = axis
        self.shaper_type = shaper_type
        self.damping_ratio = shaper_defs.DEFAULT_DAMPING_RATIO
        self.shaper_freq = 0.0
        if config is not None:
            if shaper_type not in self.shapers:
                raise config.error(
                    "Unsupported shaper type: %s" % (shaper_type,)
                )
            self.damping_ratio = config.getfloat(
                "damping_ratio_" + axis,
                self.damping_ratio,
                minval=0.0,
                maxval=1.0,
            )
            self.shaper_freq = config.getfloat(
                "shaper_freq_" + axis, self.shaper_freq, minval=0.0
            )

    def get_type(self):
        return self.shaper_type

    def get_axis(self):
        return self.axis

    def update(self, shaper_type, gcmd):
        if shaper_type not in self.shapers:
            raise gcmd.error("Unsupported shaper type: %s" % (shaper_type,))
        axis = self.axis.upper()
        self.damping_ratio = gcmd.get_float(
            "DAMPING_RATIO_" + axis, self.damping_ratio, minval=0.0, maxval=1.0
        )
        self.shaper_freq = gcmd.get_float(
            "SHAPER_FREQ_" + axis, self.shaper_freq, minval=0.0
        )
        self.shaper_type = shaper_type

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.shaper_type),
                ("shaper_freq", "%.3f" % (self.shaper_freq,)),
                ("damping_ratio", "%.6f" % (self.damping_ratio,)),
            ]
        )


class CustomInputShaperParams:
    SHAPER_TYPE = "custom"

    def __init__(self, axis, config):
        self.axis = axis
        self.n, self.A, self.T = 0, [], []
        if config is not None:
            shaper_a_str = config.get("shaper_a_" + axis)
            shaper_t_str = config.get("shaper_t_" + axis)
            self.n, self.A, self.T = self._parse_custom_shaper(
                shaper_a_str, shaper_t_str, config.error
            )

    def get_type(self):
        return self.SHAPER_TYPE

    def get_axis(self):
        return self.axis

    def update(self, shaper_type, gcmd):
        if shaper_type != self.SHAPER_TYPE:
            raise gcmd.error("Unsupported shaper type: %s" % (shaper_type,))
        axis = self.axis.upper()
        shaper_a_str = gcmd.get("SHAPER_A_" + axis, None)
        shaper_t_str = gcmd.get("SHAPER_T_" + axis, None)
        if (shaper_a_str is None) != (shaper_t_str is None):
            raise gcmd.error(
                "Both SHAPER_A_%s and SHAPER_T_%s parameters"
                " must be provided" % (axis, axis)
            )
        if shaper_a_str is not None:
            self.n, self.A, self.T = self._parse_custom_shaper(
                shaper_a_str, shaper_t_str, gcmd.error
            )

    def _parse_custom_shaper(self, custom_a_str, custom_t_str, parse_error):
        A = parse_float_list(custom_a_str)
        if A is None:
            raise parse_error("Invalid shaper A string: '%s'" % (custom_a_str,))
        if min([abs(a) for a in A]) < 0.001:
            raise parse_error("All shaper A coefficients must be non-zero")
        if sum(A) < 0.001:
            raise parse_error(
                "Shaper A parameter must sum up to a positive number"
            )
        T = parse_float_list(custom_t_str)
        if T is None:
            raise parse_error("Invalid shaper T string: '%s'" % (custom_t_str,))
        if T != sorted(T):
            raise parse_error("Shaper T parameter is not ordered: %s" % (T,))
        if len(A) != len(T):
            raise parse_error(
                "Shaper A and T parameters must have the same length:"
                " %d vs %d" % (len(A), len(T),)
            )
        dur = T[-1] - T[0]
        if len(T) > 1 and dur < 0.001:
            raise parse_error(
                "Shaper duration is too small (%.6f sec)" % (dur,)
            )
        if dur > 0.2:
            raise parse_error(
                "Shaper duration is too large (%.6f sec)" % (dur,)
            )
        # Synthesize shaper_freq / damping_ratio attributes from the impulse
        # train so blendmath / blendplanner can treat CustomInputShaperParams
        # uniformly with TypedInputShaperParams. For the planner path only
        # the (A, T) tuple matters; freq is informational.
        return len(A), A, T

    def get_shaper(self):
        return self.n, self.A, self.T

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.SHAPER_TYPE),
                ("shaper_a", ",".join(["%.6f" % (a,) for a in self.A])),
                ("shaper_t", ",".join(["%.6f" % (t,) for t in self.T])),
            ]
        )


class AxisInputShaper:
    """Thin holder for per-axis FIR (zv / mzv / custom) shaper config.

    After Plan 8 Chunk 2 Task 12 this class owns no step-gen kinematics
    state — the kernel is baked into the planner polynomial inside
    QuinticShape.compose_phase_polynomials via
    blendplanner._bake_shaper_polynomial. The .params attribute is
    the sole entry point downstream consumers read.
    """

    def __init__(self, params):
        self.params = params

    def get_name(self):
        return "shaper_" + self.get_axis()

    def get_type(self):
        return self.params.get_type()

    def get_axis(self):
        return self.params.get_axis()

    def is_enabled(self):
        freq = float(getattr(self.params, "shaper_freq", 0.0) or 0.0)
        return freq > 0.0

    def update(self, shaper_type, gcmd):
        self.params.update(shaper_type, gcmd)

    def disable_shaping(self):
        """Zero out the params so the planner sees freq=0 (no bake)."""
        was_enabled = self.is_enabled()
        if was_enabled:
            self._saved = (
                getattr(self.params, "shaper_freq", 0.0),
            )
            self.params.shaper_freq = 0.0
        else:
            self._saved = None
        return was_enabled

    def enable_shaping(self):
        saved = getattr(self, "_saved", None)
        if not saved:
            return False
        self.params.shaper_freq = saved[0]
        self._saved = None
        return True

    def report(self, gcmd):
        info = " ".join(
            [
                "%s_%s:%s" % (key, self.get_axis(), value)
                for (key, value) in self.params.get_status().items()
            ]
        )
        gcmd.respond_info(info)


class TypedInputSmootherParams:
    smoothers = {s.name: s.init_func for s in shaper_defs.INPUT_SMOOTHERS}

    def __init__(self, axis, smoother_type, config):
        self.axis = axis
        self.smoother_type = smoother_type
        self.smoother_freq = 0.0
        if config is not None:
            self._validate_type(smoother_type, config.error)
            self.smoother_freq = config.getfloat(
                "smoother_freq_" + axis, self.smoother_freq, minval=0.0
            )

    @classmethod
    def _validate_type(cls, smoother_type, error_ctor):
        if smoother_type in cls.smoothers:
            return
        raise error_ctor("Unsupported shaper type: %s" % (smoother_type,))

    def get_type(self):
        return self.smoother_type

    def get_axis(self):
        return self.axis

    def update(self, smoother_type, gcmd):
        self._validate_type(smoother_type, gcmd.error)
        axis = self.axis.upper()
        self.smoother_freq = gcmd.get_float(
            "SMOOTHER_FREQ_" + axis, self.smoother_freq, minval=0.0
        )
        self.smoother_type = smoother_type

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.smoother_type),
                ("smoother_freq", "%.3f" % (self.smoother_freq,)),
            ]
        )


class CustomInputSmootherParams:
    SHAPER_TYPE = "smoother"

    def __init__(self, axis, config):
        self.axis = axis
        self._raw_coeffs = []
        self.smooth_time = 0.0
        if config is not None:
            self.smooth_time = config.getfloat(
                "smooth_time_" + axis, self.smooth_time, minval=0.0
            )
            self._raw_coeffs = list(
                reversed(config.getfloatlist("coeffs_" + axis, self._raw_coeffs))
            )

    def get_type(self):
        return self.SHAPER_TYPE

    def get_axis(self):
        return self.axis

    def update(self, shaper_type, gcmd):
        if shaper_type != self.SHAPER_TYPE:
            raise gcmd.error("Unsupported shaper type: %s" % (shaper_type,))
        axis = self.axis.upper()
        self.smooth_time = gcmd.get_float(
            "SMOOTH_TIME_" + axis, self.smooth_time
        )
        coeffs_str = gcmd.get("COEFFS_" + axis, None)
        if coeffs_str is not None:
            try:
                coeffs = parse_float_list(coeffs_str)
                coeffs.reverse()
            except:
                raise gcmd.error("Invalid format for COEFFS parameter")
            self._raw_coeffs = coeffs

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.SHAPER_TYPE),
                (
                    "shaper_coeffs",
                    ",".join(["%.9e" % (a,) for a in reversed(self._raw_coeffs)]),
                ),
                ("shaper_smooth_time", self.smooth_time),
            ]
        )


class AxisInputSmoother:
    """Per-axis bs-family (cardinal B-spline chain) smoother config holder.

    Mirrors AxisInputShaper — no step-gen state, no FFI calls. Exposes
    ``params`` so blendplanner can read ``smoother_type`` / ``smoother_freq``
    when baking the smooth-IS kernel into the quintic polynomial.
    """

    def __init__(self, params):
        self.params = params

    def get_name(self):
        return "smoother_" + self.get_axis()

    def get_type(self):
        return self.params.get_type()

    def get_axis(self):
        return self.params.get_axis()

    def is_bs_family(self):
        return self.get_type() in _SMOOTHER_FAMILY_NAMES

    def is_enabled(self):
        freq = float(getattr(self.params, "smoother_freq", 0.0) or 0.0)
        if freq > 0.0:
            return True
        smooth_time = float(getattr(self.params, "smooth_time", 0.0) or 0.0)
        return smooth_time > 0.0

    def update(self, shaper_type, gcmd):
        self.params.update(shaper_type, gcmd)

    def disable_shaping(self):
        was_enabled = self.is_enabled()
        if was_enabled:
            self._saved = (
                getattr(self.params, "smoother_freq", 0.0),
                getattr(self.params, "smooth_time", 0.0),
            )
            if hasattr(self.params, "smoother_freq"):
                self.params.smoother_freq = 0.0
            if hasattr(self.params, "smooth_time"):
                self.params.smooth_time = 0.0
        else:
            self._saved = None
        return was_enabled

    def enable_shaping(self):
        saved = getattr(self, "_saved", None)
        if not saved:
            return False
        freq, smooth_time = saved
        if hasattr(self.params, "smoother_freq"):
            self.params.smoother_freq = freq
        if hasattr(self.params, "smooth_time"):
            self.params.smooth_time = smooth_time
        self._saved = None
        return True

    def report(self, gcmd):
        info = " ".join(
            [
                "%s_%s:%s" % (key, self.get_axis(), value)
                for (key, value) in self.params.get_status().items()
            ]
        )
        gcmd.respond_info(info)


class ShaperFactory:
    def __init__(self):
        pass

    def _create_shaper(self, axis, type_name, config=None):
        if type_name == CustomInputSmootherParams.SHAPER_TYPE:
            return AxisInputSmoother(CustomInputSmootherParams(axis, config))
        if type_name == CustomInputShaperParams.SHAPER_TYPE:
            return AxisInputShaper(CustomInputShaperParams(axis, config))
        if type_name in TypedInputShaperParams.shapers:
            return AxisInputShaper(
                TypedInputShaperParams(axis, type_name, config)
            )
        if type_name in TypedInputSmootherParams.smoothers:
            return AxisInputSmoother(
                TypedInputSmootherParams(axis, type_name, config)
            )
        return None

    def create_shaper(self, axis, config):
        shaper_type = config.get("shaper_type", "mzv")
        shaper_type = config.get("shaper_type_" + axis, shaper_type).lower()
        shaper = self._create_shaper(axis, shaper_type, config)
        if shaper is None:
            raise config.error("Unsupported shaper type '%s'" % (shaper_type,))
        return shaper

    def update_shaper(self, shaper, gcmd):
        shaper_type = gcmd.get("SHAPER_TYPE", None)
        if shaper_type is None:
            shaper_type = gcmd.get(
                "SHAPER_TYPE_" + shaper.get_axis().upper(), shaper.get_type()
            )
        shaper_type = shaper_type.lower()
        try:
            shaper.update(shaper_type, gcmd)
            return shaper
        except gcmd.error:
            pass
        shaper = self._create_shaper(shaper.get_axis(), shaper_type)
        if shaper is None:
            raise gcmd.error("Unsupported shaper type '%s'" % (shaper_type,))
        shaper.update(shaper_type, gcmd)
        return shaper


class InputShaper:
    """[input_shaper] config object.

    Plan 8 Chunk 2 Task 12 — this class no longer owns any C-side stepper
    kinematics state. It's a pure Python config holder: the planner reads
    get_shapers() at emit time to figure out what to bake.

    Target_smoothing config is retired (Plan 8 spec §4) — specifying it
    raises an error pointing the user at the replacement.
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.toolhead = None
        self.shaper_factory = ShaperFactory()
        self.shapers = [
            self.shaper_factory.create_shaper("x", config),
            self.shaper_factory.create_shaper("y", config),
        ]
        # Plan 8 §4: target_smoothing retires with the post-hoc shaper.
        # The shaper-cap math (blendmath.suppressed_junction_v) now reads
        # shaper_freq + damping directly; there is no runtime cap knob.
        if config.get("target_smoothing", None) is not None:
            raise config.error(
                "[input_shaper] target_smoothing is retired (Plan 8). "
                "Remove the key — corner velocities now derive from the "
                "baked-in shaper kernel directly."
            )
        if config.get("target_passband", None) is not None:
            raise config.error(
                "[input_shaper] target_passband is retired (Plan 8). "
                "Remove the key — the feedforward-inverse path was "
                "retired with the post-hoc cascade."
            )
        # Unused under baked-in shaping but kept for config-reject parity
        # if somebody still lists enabled_extruders: silently ignore.
        self.config_extruder_names = config.getlist("enabled_extruders", [])
        # Register gcode commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_INPUT_SHAPER",
            self.cmd_SET_INPUT_SHAPER,
            desc=self.cmd_SET_INPUT_SHAPER_help,
        )
        gcode.register_command(
            "ENABLE_INPUT_SHAPER",
            self.cmd_ENABLE_INPUT_SHAPER,
            desc=self.cmd_ENABLE_INPUT_SHAPER_help,
        )
        gcode.register_command(
            "DISABLE_INPUT_SHAPER",
            self.cmd_DISABLE_INPUT_SHAPER,
            desc=self.cmd_DISABLE_INPUT_SHAPER_help,
        )

    def get_shapers(self):
        return self.shapers

    def connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")

    def _flush_for_shaper_update(self):
        """Force a step-gen resync so the next emitted move picks up the
        new shaper. Pre-Plan-8 this triggered the C-side FFI rebuild;
        under baked-in shaping a plain flush is enough — the next
        QuinticBlendMove.__init__ reads the updated shapers list.

        Plan 9 A3: also invalidate the toolhead's cached shaper snapshot
        (consumed by Move.__init__) so moves constructed after the update
        see the new shaper configuration."""
        if self.toolhead is not None:
            self.toolhead.flush_step_generation()
            refresh = getattr(self.toolhead, "_refresh_shapers_snapshot", None)
            if refresh is not None:
                refresh()

    def disable_shaping(self):
        self._flush_for_shaper_update()
        for shaper in self.shapers:
            shaper.disable_shaping()

    def enable_shaping(self):
        self._flush_for_shaper_update()
        for shaper in self.shapers:
            shaper.enable_shaping()

    cmd_SET_INPUT_SHAPER_help = "Set cartesian parameters for input shaper"

    def cmd_SET_INPUT_SHAPER(self, gcmd):
        params = gcmd.get_command_parameters()
        if any(k == "TARGET_SMOOTHING" for k in params):
            raise gcmd.error(
                "TARGET_SMOOTHING is retired (Plan 8). Shaper-derived "
                "corner caps now come from the baked-in kernel."
            )
        if any(k == "TARGET_PASSBAND" for k in params):
            raise gcmd.error(
                "TARGET_PASSBAND is retired (Plan 8). The feedforward-"
                "inverse path was retired with the post-hoc cascade."
            )
        if params:
            self._flush_for_shaper_update()
            self.shapers = [
                self.shaper_factory.update_shaper(shaper, gcmd)
                for shaper in self.shapers
            ]
        for shaper in self.shapers:
            shaper.report(gcmd)

    def get_status(self, eventtime):
        # Plan 8: no target_smoothing / target_passband knobs to report.
        return {}

    cmd_ENABLE_INPUT_SHAPER_help = "Enable input shaper for given objects"

    def cmd_ENABLE_INPUT_SHAPER(self, gcmd):
        self._flush_for_shaper_update()
        axes = gcmd.get("AXIS", "")
        msg = ""
        for axis_str in axes.split(","):
            axis = axis_str.strip().lower()
            if not axis:
                continue
            shapers = [s for s in self.shapers if s.get_axis() == axis]
            if not shapers:
                raise gcmd.error("Invalid AXIS='%s'" % (axis_str,))
            for s in shapers:
                if s.enable_shaping():
                    msg += "Enabled input shaper for AXIS='%s'\n" % (axis_str,)
                else:
                    msg += (
                        "Cannot enable input shaper for AXIS='%s': "
                        "was not disabled\n" % (axis_str,)
                    )
        gcmd.respond_info(msg)

    cmd_DISABLE_INPUT_SHAPER_help = "Disable input shaper for given objects"

    def cmd_DISABLE_INPUT_SHAPER(self, gcmd):
        self._flush_for_shaper_update()
        axes = gcmd.get("AXIS", "")
        msg = ""
        for axis_str in axes.split(","):
            axis = axis_str.strip().lower()
            if not axis:
                continue
            shapers = [s for s in self.shapers if s.get_axis() == axis]
            if not shapers:
                raise gcmd.error("Invalid AXIS='%s'" % (axis_str,))
            for s in shapers:
                if s.disable_shaping():
                    msg += "Disabled input shaper for AXIS='%s'\n" % (axis_str,)
                else:
                    msg += (
                        "Cannot disable input shaper for AXIS='%s': not "
                        "enabled or was already disabled\n" % (axis_str,)
                    )
        gcmd.respond_info(msg)


def load_config(config):
    return InputShaper(config)
