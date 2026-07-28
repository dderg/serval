import re

PA_TYPE = "linear_pressure_advance"
ST_TYPE = "smooth_triangle"
NONLINEAR_TYPES = ("tanh_pressure_advance", "recipr_pressure_advance")
ADVANCE_TYPES = (PA_TYPE,) + NONLINEAR_TYPES

ADVANCE_PARAM_KEYS = {
    PA_TYPE: "k",
    "tanh_pressure_advance": "linear_advance",
    "recipr_pressure_advance": "linear_advance",
}

_EXTRUDER_NAME = re.compile(r"extruder\d*")


class ActiveTarget:
    enabled = True

    def __init__(self, post_processor, param_key, initial_value):
        self.post_processor = post_processor
        self.param_key = param_key
        self.initial_value = initial_value


class DisabledTarget:
    enabled = False

    def __init__(self, reason):
        self.reason = reason


class ExtruderTargets:
    def __init__(self, advance, offset, velocity, smooth_time):
        self.advance = advance
        self.offset = offset
        self.velocity = velocity
        self.smooth_time = smooth_time


def _section_suffix(sc):
    return sc.get_name().split(None, 1)[1]


def _post_processor_sections(config):
    return {
        _section_suffix(sc): sc
        for sc in config.get_prefix_sections("post_processor ")
    }


def _axis_sections(config):
    return {
        _section_suffix(sc): sc for sc in config.get_prefix_sections("axis ")
    }


def _extruder_sections(config):
    return [
        sc
        for sc in config.get_prefix_sections("extruder")
        if _EXTRUDER_NAME.fullmatch(sc.get_name())
    ]


def _validated_override(config, option, pp_sections, types):
    name = config.get(option, None)
    if name is None:
        return None
    sc = pp_sections.get(name)
    if sc is None or sc.get("type", None) not in types:
        raise config.error(
            "[pressure_advance_compat] %s: '%s' is not a declared "
            "[post_processor] of type %s" % (option, name, "/".join(types))
        )
    return name


def _resolve_target(
    pp_sections, axes, axis_name, ty, param_key, override, override_option
):
    if override is not None:
        section = pp_sections[override]
        return ActiveTarget(
            override, param_key, section.getfloat(param_key, 0.0, minval=0.0)
        )
    if axis_name is None or axis_name not in axes:
        return DisabledTarget(
            "extruder axis '%s' is not a declared [axis] section" % (axis_name,)
        )
    references = [
        p.strip() for p in axes[axis_name].getlist("post_processors", [])
    ]
    candidates = [
        name
        for name in references
        if name in pp_sections and pp_sections[name].get("type", None) == ty
    ]
    if not candidates:
        return DisabledTarget(
            "no [post_processor] of type %s on [axis %s]" % (ty, axis_name)
        )
    if len(candidates) > 1:
        return DisabledTarget(
            "multiple [post_processor]s of type %s on [axis %s] (%s); use "
            "SET_POST_PROCESSOR NAME=... or disambiguate with '%s:' in "
            "[pressure_advance_compat]"
            % (ty, axis_name, ", ".join(sorted(candidates)), override_option)
        )
    name = candidates[0]
    return ActiveTarget(
        name, param_key, pp_sections[name].getfloat(param_key, 0.0, minval=0.0)
    )


def _advance_family_targets(
    pp_sections, axes, axis_name, override, override_option
):
    """bleeding-edge-v2 knob mapping: ADVANCE fits any advance model's
    linear coefficient; OFFSET/VELOCITY exist only on the nonlinear ones."""
    if override is not None:
        name, ty = override, pp_sections[override].get("type", None)
    else:
        if axis_name is None or axis_name not in axes:
            missing = DisabledTarget(
                "extruder axis '%s' is not a declared [axis] section"
                % (axis_name,)
            )
            return missing, missing, missing
        references = [
            p.strip() for p in axes[axis_name].getlist("post_processors", [])
        ]
        candidates = [
            n
            for n in references
            if n in pp_sections
            and pp_sections[n].get("type", None) in ADVANCE_TYPES
        ]
        if not candidates:
            missing = DisabledTarget(
                "no advance-family [post_processor] (%s) on [axis %s]"
                % ("/".join(ADVANCE_TYPES), axis_name)
            )
            return missing, missing, missing
        if len(candidates) > 1:
            ambiguous = DisabledTarget(
                "multiple advance-family [post_processor]s on [axis %s] "
                "(%s); use SET_POST_PROCESSOR NAME=... or disambiguate "
                "with '%s:' in [pressure_advance_compat]"
                % (axis_name, ", ".join(sorted(candidates)), override_option)
            )
            return ambiguous, ambiguous, ambiguous
        name = candidates[0]
        ty = pp_sections[name].get("type", None)
    section = pp_sections[name]
    advance_key = ADVANCE_PARAM_KEYS[ty]
    advance = ActiveTarget(
        name, advance_key, section.getfloat(advance_key, 0.0, minval=0.0)
    )
    if ty == PA_TYPE:
        linear_only = DisabledTarget(
            "'%s' is a %s; OFFSET/VELOCITY apply to %s"
            % (name, PA_TYPE, "/".join(NONLINEAR_TYPES))
        )
        return advance, linear_only, linear_only
    offset = ActiveTarget(
        name,
        "nonlinear_offset",
        section.getfloat("nonlinear_offset", 0.0, minval=0.0),
    )
    velocity = ActiveTarget(
        name,
        "linearization_velocity",
        section.getfloat("linearization_velocity", 0.0, minval=0.0),
    )
    return advance, offset, velocity


class PressureAdvanceCompat:
    def __init__(self, config):
        self.printer = config.get_printer()
        pp_sections = _post_processor_sections(config)
        axes = _axis_sections(config)
        pa_override = _validated_override(
            config, "post_processor", pp_sections, ADVANCE_TYPES
        )
        st_override = _validated_override(
            config, "smooth_post_processor", pp_sections, (ST_TYPE,)
        )
        self.extruders = {}
        for sc in _extruder_sections(config):
            axis_name = sc.get("axis", None)
            advance, offset, velocity = _advance_family_targets(
                pp_sections, axes, axis_name, pa_override, "post_processor"
            )
            self.extruders[sc.get_name()] = ExtruderTargets(
                advance,
                offset,
                velocity,
                _resolve_target(
                    pp_sections,
                    axes,
                    axis_name,
                    ST_TYPE,
                    "smooth_time",
                    st_override,
                    "smooth_post_processor",
                ),
            )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_PRESSURE_ADVANCE",
            self.cmd_SET_PRESSURE_ADVANCE,
            desc=self.cmd_SET_PRESSURE_ADVANCE_help,
        )

    cmd_SET_PRESSURE_ADVANCE_help = (
        "Classic-Klipper shim: set pressure advance parameters on the "
        "extruder axis' post_processors (ADVANCE, OFFSET, VELOCITY, "
        "SMOOTH_TIME as in bleeding-edge-v2)"
    )

    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        extruder_name = self._resolve_extruder_name(gcmd)
        targets = self.extruders[extruder_name]
        updates = [
            (
                "SMOOTH_TIME",
                targets.smooth_time,
                gcmd.get_float("SMOOTH_TIME", None, minval=0.0),
            ),
            (
                "ADVANCE",
                targets.advance,
                gcmd.get_float("ADVANCE", None, minval=0.0),
            ),
            (
                "OFFSET",
                targets.offset,
                gcmd.get_float("OFFSET", None, minval=0.0),
            ),
            (
                "VELOCITY",
                targets.velocity,
                gcmd.get_float("VELOCITY", None, above=0.0),
            ),
        ]
        if all(value is None for _, _, value in updates):
            gcmd.respond_info(self._format_report(extruder_name, targets))
            return
        for param, target, value in updates:
            if value is not None:
                self._apply(gcmd, extruder_name, param, target, value)

    def _resolve_extruder_name(self, gcmd):
        name = gcmd.get("EXTRUDER", None)
        if name is None:
            toolhead = self.printer.lookup_object("toolhead")
            name = toolhead.get_extruder().get_name()
        if name not in self.extruders:
            raise gcmd.error(
                "SET_PRESSURE_ADVANCE: '%s' is not a configured extruder"
                % (name,)
            )
        return name

    def _apply(self, gcmd, extruder_name, param, target, value):
        if not target.enabled:
            gcmd.respond_info(
                "SET_PRESSURE_ADVANCE: cannot set %s for extruder '%s': %s"
                % (param, extruder_name, target.reason)
            )
            return
        engine = self._engine(gcmd)
        try:
            engine.update_post_processor(
                target.post_processor, target.param_key, value
            )
        except (ValueError, RuntimeError) as e:
            raise gcmd.error(str(e))

    def _engine(self, gcmd):
        engine = self.printer.lookup_object("motion_engine", None)
        if engine is None:
            raise gcmd.error(
                "SET_PRESSURE_ADVANCE: motion_engine is not available"
            )
        return engine

    def _format_report(self, extruder_name, targets):
        lines = ["extruder '%s':" % (extruder_name,)]
        for label, target in (
            ("pressure_advance", targets.advance),
            ("nonlinear_offset", targets.offset),
            ("linearization_velocity", targets.velocity),
            ("smooth_time", targets.smooth_time),
        ):
            if target.enabled:
                lines.append(
                    "%s: %.6f (post_processor '%s')"
                    % (
                        label,
                        self._current_value(target),
                        target.post_processor,
                    )
                )
            else:
                lines.append("%s: disabled (%s)" % (label, target.reason))
        return "\n".join(lines)

    def _current_value(self, target):
        engine = self.printer.lookup_object("motion_engine", None)
        if engine is not None:
            value = engine.post_processor_param(
                target.post_processor, target.param_key
            )
            if value is not None:
                return value
        return target.initial_value

    def get_status_fields(self, extruder_name):
        targets = self.extruders.get(extruder_name)
        if targets is None:
            return {}
        fields = {}
        named = (
            ("pressure_advance", targets.advance),
            ("nonlinear_offset", targets.offset),
            ("linearization_velocity", targets.velocity),
            ("smooth_time", targets.smooth_time),
        )
        for key, target in named:
            if target.enabled:
                fields[key] = self._current_value(target)
        return fields


def load_config(config):
    return PressureAdvanceCompat(config)
