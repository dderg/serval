PA_TYPE = "linear_pressure_advance"


def _discover_post_processor(config, override):
    referenced = set()
    for sc in config.get_prefix_sections("axis "):
        for ref in sc.getlist("post_processors", []):
            referenced.add(ref.strip())
    pa_by_name = {}
    for sc in config.get_prefix_sections("post_processor "):
        name = sc.get_name().split(None, 1)[1]
        if sc.get("type", None) == PA_TYPE:
            pa_by_name[name] = sc
    if override is not None:
        sc = pa_by_name.get(override)
        if sc is None:
            raise config.error(
                "[pressure_advance_compat] post_processor: '%s' is not a "
                "declared [post_processor] of type %s" % (override, PA_TYPE)
            )
        return override, sc
    candidates = [n for n in pa_by_name if n in referenced]
    if not candidates:
        raise config.error(
            "[pressure_advance_compat] found no [post_processor] of type %s "
            "referenced by any [axis]; declare one or set post_processor: "
            "<name>" % PA_TYPE
        )
    if len(candidates) > 1:
        raise config.error(
            "[pressure_advance_compat] found multiple %s post_processors %s; "
            "disambiguate with post_processor: <name>"
            % (PA_TYPE, sorted(candidates))
        )
    name = candidates[0]
    return name, pa_by_name[name]


class PressureAdvanceCompat:
    def __init__(self, config):
        self.printer = config.get_printer()
        override = config.get("post_processor", None)
        self.post_processor, pa_section = _discover_post_processor(
            config, override
        )
        self.last_advance = pa_section.getfloat("k", 0.0, minval=0.0)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_PRESSURE_ADVANCE",
            self.cmd_SET_PRESSURE_ADVANCE,
            desc=self.cmd_SET_PRESSURE_ADVANCE_help,
        )

    cmd_SET_PRESSURE_ADVANCE_help = (
        "Classic-Klipper shim: set pressure advance on the "
        "linear_pressure_advance post_processor"
    )

    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        gcmd.get("EXTRUDER", None)
        if gcmd.get_float("SMOOTH_TIME", None) is not None:
            gcmd.respond_info(
                "SET_PRESSURE_ADVANCE: SMOOTH_TIME is not supported by the "
                "motion engine and is ignored"
            )
        advance = gcmd.get_float("ADVANCE", None, minval=0.0)
        if advance is None:
            gcmd.respond_info(
                "pressure_advance: %.6f (post_processor '%s')"
                % (self.last_advance, self.post_processor)
            )
            return
        engine = self.printer.lookup_object("motion_engine", None)
        if engine is None:
            raise gcmd.error(
                "SET_PRESSURE_ADVANCE: motion_engine is not available"
            )
        try:
            engine.update_post_processor(self.post_processor, "k", advance)
        except (ValueError, RuntimeError) as e:
            raise gcmd.error(str(e))
        self.last_advance = advance


def load_config(config):
    return PressureAdvanceCompat(config)
