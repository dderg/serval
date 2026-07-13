import re

PA_TYPE = "linear_pressure_advance"
ST_TYPE = "smooth_triangle"

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
    def __init__(self, advance, smooth_time):
        self.advance = advance
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


def _validated_override(config, option, pp_sections, ty):
    name = config.get(option, None)
    if name is None:
        return None
    sc = pp_sections.get(name)
    if sc is None or sc.get("type", None) != ty:
        raise config.error(
            "[pressure_advance_compat] %s: '%s' is not a declared "
            "[post_processor] of type %s" % (option, name, ty)
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


class PressureAdvanceCompat:
    def __init__(self, config):
        self.printer = config.get_printer()
        pp_sections = _post_processor_sections(config)
        axes = _axis_sections(config)
        pa_override = _validated_override(
            config, "post_processor", pp_sections, PA_TYPE
        )
        st_override = _validated_override(
            config, "smooth_post_processor", pp_sections, ST_TYPE
        )
        self.extruders = {}
        for sc in _extruder_sections(config):
            axis_name = sc.get("axis", None)
            self.extruders[sc.get_name()] = ExtruderTargets(
                _resolve_target(
                    pp_sections,
                    axes,
                    axis_name,
                    PA_TYPE,
                    "k",
                    pa_override,
                    "post_processor",
                ),
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
        "Classic-Klipper shim: set pressure advance / smooth time on the "
        "extruder axis' post_processors"
    )

    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        extruder_name = self._resolve_extruder_name(gcmd)
        targets = self.extruders[extruder_name]
        advance = gcmd.get_float("ADVANCE", None, minval=0.0)
        smooth_time = gcmd.get_float("SMOOTH_TIME", None, minval=0.0)
        if advance is None and smooth_time is None:
            gcmd.respond_info(self._format_report(extruder_name, targets))
            return
        if smooth_time is not None:
            self._apply(
                gcmd,
                extruder_name,
                "SMOOTH_TIME",
                targets.smooth_time,
                smooth_time,
            )
        if advance is not None:
            self._apply(
                gcmd, extruder_name, "ADVANCE", targets.advance, advance
            )

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
        if targets.advance.enabled:
            fields["pressure_advance"] = self._current_value(targets.advance)
        if targets.smooth_time.enabled:
            fields["smooth_time"] = self._current_value(targets.smooth_time)
        return fields


def load_config(config):
    return PressureAdvanceCompat(config)
