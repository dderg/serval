# Kinematic input shaper to minimize motion vibrations in XY plane
#
# Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020-2023  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging

from klippy import chelper

from . import shaper_defs
from . import extruder_smoother


# Maximum pieces per smoother kernel. Mirrors SMOOTHER_MAX_PIECES in
# klippy/chelper/integrate.h. Bumping this requires a matching C change.
_FFI_MAX_PIECES = 9
# Doubles per piece in the flat FFI buffer: [t_start, t_end, c_0..c_5] = 8
# doubles. Mirrors struct smoother_piece coeff layout in integrate.h.
_FFI_DOUBLES_PER_PIECE = 8


def _marshal_pieces_to_buffer(ffi_main, C_pieces):
    """Flatten piecewise smoother coeffs to the C FFI buffer layout.

    Returns (n_pieces, buf). Pads each piece's coefficient list with zeros
    to SMOOTHER_MAX_DEGREE (=5). Raises if the kernel exceeds
    SMOOTHER_MAX_PIECES — indicates a Python-side bug that tried to pass
    an over-sized piecewise kernel; the C side cannot store it.
    """
    n_pieces = len(C_pieces)
    if n_pieces > _FFI_MAX_PIECES:
        raise ValueError(
            "Smoother has %d pieces; C side supports up to %d"
            % (n_pieces, _FFI_MAX_PIECES)
        )
    buf = ffi_main.new("double[]", n_pieces * _FFI_DOUBLES_PER_PIECE)
    for i, (t_start, t_end, coeffs) in enumerate(C_pieces):
        base = i * _FFI_DOUBLES_PER_PIECE
        buf[base + 0] = float(t_start)
        buf[base + 1] = float(t_end)
        for k in range(6):
            buf[base + 2 + k] = float(coeffs[k]) if k < len(coeffs) else 0.0
    return n_pieces, buf


# Cardinal B-spline chain family name set — the invertible shapers for
# which the Pillar-1 feedforward inverse is computed. Classic FIR shapers
# (zv, mzv) are impulse trains with spectral nulls and are not inverted.
_BS_FAMILY_NAMES = frozenset(s.name for s in shaper_defs.INPUT_SMOOTHERS)


def _raise_migration_error(error_ctor, retired_name, replacement):
    """Raise the standard smooth_* -> bs* migration message.

    Shared by all three smooth_* entry points (config-load
    TypedInputSmootherParams validation, ShaperFactory.create_shaper,
    ShaperFactory.update_shaper) so the user-visible wording stays in a
    single place.
    """
    raise error_ctor(
        "shaper_type '%s' was replaced in Magnum Opus with the "
        "cardinal B-spline chain family. Use shaper_type = '%s' "
        "for equivalent behavior." % (retired_name, replacement)
    )


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

    def get_shaper(self):
        if not self.shaper_freq:
            A, T = shaper_defs.get_none_shaper()
        else:
            A, T = self.shapers[self.shaper_type](
                self.shaper_freq, self.damping_ratio
            )
        return len(A), A, T

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
                " %d vs %d"
                % (
                    len(A),
                    len(T),
                )
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
    def __init__(self, params):
        self.params = params
        self.n, self.A, self.T = params.get_shaper()
        self.t_offs = shaper_defs.get_shaper_offset(self.A, self.T)
        self.saved = None

    def get_name(self):
        return "shaper_" + self.get_axis()

    def get_type(self):
        return self.params.get_type()

    def get_axis(self):
        return self.params.get_axis()

    def is_extruder_smoothing(self, exact_mode):
        return not exact_mode and self.A

    def is_enabled(self):
        return self.n > 0

    def update(self, shaper_type, gcmd):
        self.params.update(shaper_type, gcmd)
        self.n, self.A, self.T = self.params.get_shaper()
        self.t_offs = shaper_defs.get_shaper_offset(self.A, self.T)

    def update_stepper_kinematics(self, sk):
        ffi_main, ffi_lib = chelper.get_ffi()
        axis = self.get_axis().encode()
        success = (
            ffi_lib.input_shaper_set_shaper_params(
                sk, axis, self.n, self.A, self.T
            )
            == 0
        )
        if not success:
            self.disable_shaping()
            ffi_lib.input_shaper_set_shaper_params(
                sk, axis, self.n, self.A, self.T
            )
        return success

    def update_extruder_kinematics(self, sk, exact_mode):
        ffi_main, ffi_lib = chelper.get_ffi()
        axis = self.get_axis().encode()
        if not self.is_extruder_smoothing(exact_mode):
            # Make sure to disable any active input smoothing
            n_pieces, buf = _marshal_pieces_to_buffer(ffi_main, [])
            success = (
                ffi_lib.extruder_set_smoothing_params(
                    sk, axis, n_pieces, buf, 0.0, 0.0
                )
                == 0
            )
            success = (
                ffi_lib.extruder_set_shaper_params(
                    sk, axis, self.n, self.A, self.T
                )
                == 0
            )
        else:
            shaper_type = self.get_type()
            status = self.params.get_status()
            damping_ratio = float(
                status.get("damping_ratio", shaper_defs.DEFAULT_DAMPING_RATIO)
            )

            # Plan 5: Python now emits piecewise kernel coefficients in
            # real-t power basis, so init_smoother must apply the
            # 1/t_sm^(i+1) rescaling (normalize_coeffs=True). Pre-Plan-5
            # this scaling happened on the C side for the flat-coeff FFI.
            C_pieces, t_sm = extruder_smoother.get_extruder_smoother(
                shaper_type,
                self.T[-1] - self.T[0],
                damping_ratio,
                normalize_coeffs=True,
            )
            smoother_offset = self.t_offs - 0.5 * t_sm
            n_pieces, buf = _marshal_pieces_to_buffer(ffi_main, C_pieces)
            success = (
                ffi_lib.extruder_set_smoothing_params(
                    sk, axis, n_pieces, buf, t_sm, smoother_offset
                )
                == 0
            )
        if not success:
            self.disable_shaping()
            ffi_lib.extruder_set_shaper_params(sk, axis, self.n, self.A, self.T)
        return success

    def disable_shaping(self):
        was_enabled = False
        if self.saved is None and self.n:
            self.saved = (self.n, self.A, self.T)
            was_enabled = True
        A, T = shaper_defs.get_none_shaper()
        self.n, self.A, self.T = len(A), A, T
        return was_enabled

    def enable_shaping(self):
        if self.saved is None:
            # Input shaper was not disabled
            return False
        self.n, self.A, self.T = self.saved
        self.saved = None
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
        hint = shaper_defs.RETIRED_SMOOTHER_MIGRATION.get(smoother_type)
        if hint is not None:
            _raise_migration_error(error_ctor, smoother_type, hint)
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

    def get_smoother(self):
        """Return (C_pieces, smooth_time) piecewise kernel description."""
        if not self.smoother_freq:
            return shaper_defs.get_none_smoother()
        return self.smoothers[self.smoother_type](
            self.smoother_freq, shaper_defs.DEFAULT_DAMPING_RATIO, True
        )

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

    def get_smoother(self):
        """Return (C_pieces, smooth_time) — wrap flat coeffs in a single piece."""
        if not self._raw_coeffs or self.smooth_time <= 0.0:
            return shaper_defs.get_none_smoother()
        return shaper_defs.init_smoother(
            self._raw_coeffs, self.smooth_time, True
        )

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
    def __init__(self, params):
        self.params = params
        self.C_pieces, self.smooth_time = params.get_smoother()
        self.t_offs = shaper_defs.get_smoother_offset(
            self.C_pieces, self.smooth_time, normalized=True
        )
        self.saved_smooth_time = 0.0
        # Plan 5 Pillar 1 (Task 9) — fused feedforward-inverse kernel.
        # Populated by recompute_fused_kernel() whenever target_passband
        # or the underlying smoother is updated. When None, the forward
        # kernel (self.C_pieces / self.smooth_time) is passed to FFI as
        # before — this is the graceful-degradation path taken by non-bs
        # families, shaper_type=none, and target_smoothing=0.
        self.C_fused = None
        self.t_fused = 0.0
        # G = ||h||_1 — saturation-feedback gain consumed by Task 12's
        # AxisShaperSnapshot. Defaults to 1.0 (identity cascade) so
        # downstream consumers can read it unconditionally.
        self.G_axis = 1.0

    def get_name(self):
        return "smoother_" + self.get_axis()

    def get_type(self):
        return self.params.get_type()

    def get_axis(self):
        return self.params.get_axis()

    def is_bs_family(self):
        return self.get_type() in _BS_FAMILY_NAMES

    def is_extruder_smoothing(self, exact_mode):
        return True

    def is_enabled(self):
        return self.smooth_time > 0.0

    def update(self, shaper_type, gcmd):
        self.params.update(shaper_type, gcmd)
        self.C_pieces, self.smooth_time = self.params.get_smoother()
        self.t_offs = shaper_defs.get_smoother_offset(
            self.C_pieces, self.smooth_time, normalized=True
        )
        # Invalidate the cached fused kernel — InputShaper will recompute
        # via recompute_fused_kernel() in the _update_input_shaping path
        # before the next FFI call.
        self.C_fused = None
        self.t_fused = 0.0
        self.G_axis = 1.0

    def recompute_fused_kernel(self, target_passband):
        """Build C_fused = h ⊛ w for bs-family + enabled + non-sentinel.

        Skips for:
          - non-bs-family (classic FIR -> no inverse exists)
          - disabled smoother (smooth_time <= 0 or empty pieces)
          - target_passband <= 0 or caller already suppressed
            (target_smoothing=0 sentinel lives on InputShaper and is
             enforced by passing target_passband=0 here).

        Leaves (C_fused, t_fused, G_axis) = (None, 0.0, 1.0) on skip so
        update_stepper_kinematics falls back to the forward-only path.
        """
        # Reset caches; callers can assume non-None only when a valid
        # fused kernel was produced.
        self.C_fused = None
        self.t_fused = 0.0
        self.G_axis = 1.0
        if not self.is_bs_family():
            return
        if not self.is_enabled() or not self.C_pieces:
            return
        if target_passband is None or target_passband <= 0.0:
            # Sentinel path: fall back to forward kernel. G = 1 so D4's
            # saturation cap treats this axis as if no feedforward was
            # applied (v_cap_fn stays at its pre-Pillar-1 value).
            return
        shaper_freq = getattr(self.params, "smoother_freq", 0.0)
        if shaper_freq <= 0.0:
            return
        # Lazy import — the inverse computation pulls in numpy/FFT and
        # is only relevant for bs-family smoothers.
        from klippy.extras import bspline_inverse
        try:
            pb_max_hz = target_passband * shaper_freq
            # tukey_alpha=0.05 matches the §4.3 reference table values.
            h, T_h, dt = bspline_inverse.compute_inverse_fir(
                self.C_pieces, self.smooth_time, f_sh_hz=shaper_freq,
                pb_max_hz=pb_max_hz, tukey_alpha=0.05,
            )
            C_fused = bspline_inverse.fit_fused_kernel(
                self.C_pieces, self.smooth_time, h, T_h, dt,
                n_pieces=_FFI_MAX_PIECES, degree=5,
            )
            import numpy as _np
            G = float(_np.sum(_np.abs(h)) * dt)
        except Exception:
            # Defensive: an inverse-design failure must not brick the
            # shaper module. Log and fall back to forward-only.
            logging.exception(
                "input_shaper: fused kernel computation failed for "
                "axis %s, shaper %s, f=%.3f; falling back to "
                "forward-only kernel",
                self.get_axis(), self.get_type(), shaper_freq,
            )
            return
        self.C_fused = C_fused
        self.t_fused = self.smooth_time + T_h
        self.G_axis = G

    def update_stepper_kinematics(self, sk):
        ffi_main, ffi_lib = chelper.get_ffi()
        axis = self.get_axis().encode()
        # Plan 5 Pillar 1 (Task 9): when a fused kernel has been computed
        # for this axis, hand it to the C side instead of the forward-only
        # kernel. The FFI signature is unchanged — same 9 × degree-5
        # piecewise buffer — only the content and support width change.
        if self.C_fused is not None and self.smooth_time > 0.0:
            pieces = self.C_fused
            t_sm_ffi = self.t_fused
        else:
            pieces = self.C_pieces if self.smooth_time > 0.0 else []
            t_sm_ffi = self.smooth_time
        n_pieces, buf = _marshal_pieces_to_buffer(ffi_main, pieces)
        success = (
            ffi_lib.input_shaper_set_smoother_params(
                sk, axis, n_pieces, buf, t_sm_ffi
            )
            == 0
        )
        if not success:
            self.disable_shaping()
            n_pieces, buf = _marshal_pieces_to_buffer(ffi_main, [])
            ffi_lib.input_shaper_set_smoother_params(
                sk, axis, n_pieces, buf, 0.0
            )
        return success

    def update_extruder_kinematics(self, sk, exact_mode):
        ffi_main, ffi_lib = chelper.get_ffi()
        axis = self.get_axis().encode()
        # Make sure to disable any active input shaping
        A, T = shaper_defs.get_none_shaper()
        ffi_lib.extruder_set_shaper_params(sk, axis, len(A), A, T)
        if exact_mode:
            # Plan 5 Pillar 1 (Task 10): extruder shares the XY fused
            # kernel when bs-family + feedforward is active. Otherwise
            # falls back to the forward-only kernel as before.
            if self.C_fused is not None and self.smooth_time > 0.0:
                pieces = self.C_fused
                t_sm_ffi = self.t_fused
            else:
                pieces = self.C_pieces if self.smooth_time > 0.0 else []
                t_sm_ffi = self.smooth_time
            n_pieces, buf = _marshal_pieces_to_buffer(ffi_main, pieces)
            success = (
                ffi_lib.extruder_set_smoothing_params(
                    sk, axis, n_pieces, buf, t_sm_ffi, self.t_offs
                )
                == 0
            )
        else:
            # Plan 5 normalize_coeffs=True: see update_extruder_kinematics
            # branch in AxisInputShaper for the rationale.
            smoother_type = self.get_type()
            C_e_pieces, t_sm = extruder_smoother.get_extruder_smoother(
                smoother_type,
                self.smooth_time,
                shaper_defs.DEFAULT_DAMPING_RATIO,
                normalize_coeffs=True,
            )
            n_pieces, buf = _marshal_pieces_to_buffer(ffi_main, C_e_pieces)
            success = (
                ffi_lib.extruder_set_smoothing_params(
                    sk, axis, n_pieces, buf, t_sm, self.t_offs
                )
                == 0
            )
        if not success:
            self.disable_shaping()
            n_pieces, buf = _marshal_pieces_to_buffer(ffi_main, [])
            ffi_lib.extruder_set_smoothing_params(
                sk, axis, n_pieces, buf, 0.0, 0.0
            )
        return success

    def disable_shaping(self):
        was_enabled = False
        if self.smooth_time:
            self.saved_smooth_time = self.smooth_time
            was_enabled = True
        self.smooth_time = 0.0
        return was_enabled

    def enable_shaping(self):
        if not self.saved_smooth_time:
            # Input smoother was not disabled
            return False
        self.smooth_time = self.saved_smooth_time
        self.saved_smooth_time = 0.0
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
        # Plan 5 migration: retired smooth_* names get a friendly error.
        hint = shaper_defs.RETIRED_SMOOTHER_MIGRATION.get(shaper_type)
        if hint is not None:
            _raise_migration_error(config.error, shaper_type, hint)
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
        # Plan 5 migration: surface the friendly bs* hint even at runtime
        # (SET_INPUT_SHAPER SHAPER_TYPE=smooth_mzv), BEFORE the retry path
        # below masks it as "Unsupported shaper type".
        hint = shaper_defs.RETIRED_SMOOTHER_MIGRATION.get(shaper_type)
        if hint is not None:
            _raise_migration_error(gcmd.error, shaper_type, hint)
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
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.toolhead = None
        self.extruders = []
        self.exact_mode = 0
        self.config_extruder_names = config.getlist("enabled_extruders", [])
        self.shaper_factory = ShaperFactory()
        self.shapers = [
            self.shaper_factory.create_shaper("x", config),
            self.shaper_factory.create_shaper("y", config),
        ]
        # Position cusp (mm) at a 180° reversal — pins the shaper-
        # derived per-axis accel budget used by the corner blender.
        # Smaller → tighter quality, slower corners; larger → looser
        # quality, faster corners. Defaults to Klipper's historical
        # 0.12 mm. Exposed so klippy/blendmath._extract_shapers can
        # read it at runtime, and so SHAPER_CALIBRATE can thread it
        # to find_shaper_max_accel for consistent projections.
        self.target_smoothing = config.getfloat(
            "target_smoothing", 0.12, minval=0.0,
        )
        # Plan 5 Pillar 1 (Task 9) — passband upper bound for the
        # feedforward inverse design. pb_max_hz = target_passband *
        # shaper_freq. Lower → less aggressive correction, tighter
        # passband; higher → wider passband at cost of larger G (per
        # new_shaper_family.md §4.3 second table). Default 0.3 matches
        # the spec reference convention; applies only to bs-family
        # smoothers (classic FIR shapers are not invertible and skip
        # this path).
        self.target_passband = config.getfloat(
            "target_passband", 0.3, above=0.0, below=1.0,
        )
        self.input_shaper_stepper_kinematics = []
        self.orig_stepper_kinematics = []
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
        for en in self.config_extruder_names:
            extruder = self.printer.lookup_object(en)
            if not hasattr(extruder, "get_extruder_steppers"):
                raise self.printer.config_error(
                    "Invalid extruder '%s' in [input_shaper]" % (en,)
                )
            self.extruders.append(extruder)
        # Configure initial values
        self._update_input_shaping(error=self.printer.config_error)

    def _get_input_shaper_stepper_kinematics(self, stepper):
        # Lookup stepper kinematics
        sk = stepper.get_stepper_kinematics()
        if sk in self.orig_stepper_kinematics:
            # Already processed this stepper kinematics unsuccessfully
            return None
        if sk in self.input_shaper_stepper_kinematics:
            return sk
        self.orig_stepper_kinematics.append(sk)
        ffi_main, ffi_lib = chelper.get_ffi()
        is_sk = ffi_main.gc(ffi_lib.input_shaper_alloc(), ffi_lib.free)
        stepper.set_stepper_kinematics(is_sk)
        res = ffi_lib.input_shaper_set_sk(is_sk, sk)
        if res < 0:
            stepper.set_stepper_kinematics(sk)
            return None
        self.input_shaper_stepper_kinematics.append(is_sk)
        return is_sk

    def _effective_target_passband(self):
        """Passband upper-bound argument for inverse design.

        target_smoothing == 0 is the sentinel that disables the
        shaper-derived velocity cap; in that regime we also skip the
        feedforward inverse (G == 1, C_fused == None, kin_shaper.c sees
        the forward-only kernel as before). Returning 0.0 here routes
        recompute_fused_kernel() into its sentinel branch.
        """
        if self.target_smoothing <= 0.0:
            return 0.0
        return self.target_passband

    def _recompute_fused_kernels(self):
        """Rebuild per-axis fused kernels for all bs-family smoothers.

        Must run before we iterate steppers so update_stepper_kinematics
        sees a fresh C_fused. Non-bs shapers (zv/mzv) and
        AxisInputShaper instances skip via hasattr — only the smoother
        branch carries the fused-kernel machinery.
        """
        tp = self._effective_target_passband()
        for shaper in self.shapers:
            if hasattr(shaper, "recompute_fused_kernel"):
                shaper.recompute_fused_kernel(tp)

    def get_axis_G(self, axis_char):
        """Return ||h||_1 for axis 'x'/'y'/'z'/…, or 1.0 if unavailable.

        Consumed by Task 12's AxisShaperSnapshot.inverse_G for the
        Pillar-1 saturation-feedback cap. Returns 1.0 for:
          - axes with no active shaper
          - classic FIR shapers (no inverse computed)
          - the target_smoothing=0 sentinel
          - shaper_type=none
        so the caller can multiply through unconditionally.
        """
        for shaper in self.shapers:
            if shaper.get_axis() != axis_char:
                continue
            return float(getattr(shaper, "G_axis", 1.0))
        return 1.0

    def _update_input_shaping(self, error=None):
        self.toolhead.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        kin = self.toolhead.get_kinematics()
        # Plan 5 Pillar 1 (Task 9): recompute fused kernels once per
        # config change; stepper/extruder FFI calls below pull from
        # the cached C_fused on each AxisInputSmoother.
        self._recompute_fused_kernels()
        failed_shapers = []
        for s in kin.get_steppers():
            if s.get_trapq() is None:
                continue
            is_sk = self._get_input_shaper_stepper_kinematics(s)
            if is_sk is None:
                continue
            old_delay = ffi_lib.input_shaper_get_step_gen_window(is_sk)
            for shaper in self.shapers:
                if shaper in failed_shapers:
                    continue
                if not shaper.update_stepper_kinematics(is_sk):
                    failed_shapers.append(shaper)
            new_delay = ffi_lib.input_shaper_get_step_gen_window(is_sk)
            if old_delay != new_delay:
                self.toolhead.note_step_generation_scan_time(
                    new_delay, old_delay
                )
        for e in self.extruders:
            for es in e.get_extruder_steppers():
                failed_shapers.extend(
                    es.update_input_shaping(self.shapers, self.exact_mode)
                )
        if failed_shapers:
            error = error or self.printer.command_error
            raise error(
                "Failed to configure shaper(s) %s with given parameters"
                % (", ".join([s.get_name() for s in failed_shapers]))
            )

    def disable_shaping(self):
        for shaper in self.shapers:
            shaper.disable_shaping()
        self._update_input_shaping()

    def enable_shaping(self):
        for shaper in self.shapers:
            shaper.enable_shaping()
        self._update_input_shaping()

    cmd_SET_INPUT_SHAPER_help = "Set cartesian parameters for input shaper"

    def cmd_SET_INPUT_SHAPER(self, gcmd):
        target_smoothing = gcmd.get_float(
            "TARGET_SMOOTHING", None, minval=0.0
        )
        if target_smoothing is not None:
            self.target_smoothing = target_smoothing
        # Plan 5 Pillar 1 (Task 9) — runtime knob for the inverse
        # design passband. Mirrors TARGET_SMOOTHING: changing it
        # requires a C-side rebuild because C_fused is cached on
        # each AxisInputSmoother.
        target_passband = gcmd.get_float(
            "TARGET_PASSBAND", None, above=0.0, below=1.0
        )
        if target_passband is not None:
            self.target_passband = target_passband
        params = gcmd.get_command_parameters()
        # TARGET_SMOOTHING alone only updates the Python attribute
        # (blendmath reads it live on each blend). Any shaper-specific
        # parameter triggers the C-level rebuild via _update_input_shaping
        # which flushes step generation - avoid doing that when not needed.
        # TARGET_PASSBAND changes require the rebuild too because the
        # cached fused kernel depends on it.
        trivial_keys = {"TARGET_SMOOTHING"}
        shaper_param_present = any(
            k not in trivial_keys | {"TARGET_PASSBAND"} for k in params
        )
        if shaper_param_present:
            self.shapers = [
                self.shaper_factory.update_shaper(shaper, gcmd)
                for shaper in self.shapers
            ]
            self._update_input_shaping()
        elif target_passband is not None:
            # target_passband change alone still needs the fused-kernel
            # rebuild + FFI flush.
            self._update_input_shaping()
        for shaper in self.shapers:
            shaper.report(gcmd)
        gcmd.respond_info(
            "target_smoothing:%.6f target_passband:%.6f"
            % (self.target_smoothing, self.target_passband)
        )

    def get_status(self, eventtime):
        return {
            "target_smoothing": self.target_smoothing,
            "target_passband": self.target_passband,
        }

    cmd_ENABLE_INPUT_SHAPER_help = "Enable input shaper for given objects"

    def cmd_ENABLE_INPUT_SHAPER(self, gcmd):
        self.toolhead.flush_step_generation()
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
        extruders = gcmd.get("EXTRUDER", "")
        self.exact_mode = gcmd.get_int("EXACT", self.exact_mode)
        for en in extruders.split(","):
            extruder_name = en.strip()
            if not extruder_name:
                continue
            extruder = self.printer.lookup_object(extruder_name)
            if not hasattr(extruder, "get_extruder_steppers"):
                raise gcmd.error("Invalid EXTRUDER='%s'" % (en,))
            if extruder not in self.extruders:
                self.extruders.append(extruder)
                msg += "Enabled input shaper for '%s'\n" % (en,)
            else:
                msg += "Input shaper already enabled for '%s'\n" % (en,)
        self._update_input_shaping()
        gcmd.respond_info(msg)

    cmd_DISABLE_INPUT_SHAPER_help = "Disable input shaper for given objects"

    def cmd_DISABLE_INPUT_SHAPER(self, gcmd):
        self.toolhead.flush_step_generation()
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
        extruders = gcmd.get("EXTRUDER", "")
        for en in extruders.split(","):
            extruder_name = en.strip()
            if not extruder_name:
                continue
            extruder = self.printer.lookup_object(extruder_name)
            if extruder in self.extruders:
                to_re_enable = [s for s in self.shapers if s.disable_shaping()]
                for es in extruder.get_extruder_steppers():
                    es.update_input_shaping(self.shapers, self.exact_mode)
                for shaper in to_re_enable:
                    shaper.enable_shaping()
                self.extruders.remove(extruder)
                msg += "Disabled input shaper for '%s'\n" % (en,)
            else:
                msg += "Input shaper not enabled for '%s'\n" % (en,)
        self._update_input_shaping()
        gcmd.respond_info(msg)


def load_config(config):
    return InputShaper(config)
