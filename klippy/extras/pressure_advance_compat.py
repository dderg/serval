PA_TYPE = "linear_pressure_advance"
ST_TYPE = "smooth_triangle"


def _referenced_names(config):
    referenced = set()
    for sc in config.get_prefix_sections("axis "):
        for ref in sc.getlist("post_processors", []):
            referenced.add(ref.strip())
    return referenced


def _sections_by_type(config, ty):
    by_name = {}
    for sc in config.get_prefix_sections("post_processor "):
        name = sc.get_name().split(None, 1)[1]
        if sc.get("type", None) == ty:
            by_name[name] = sc
    return by_name


def _discover_post_processor(config, override):
    by_name = _sections_by_type(config, PA_TYPE)
    if override is not None:
        sc = by_name.get(override)
        if sc is None:
            raise config.error(
                "[pressure_advance_compat] post_processor: '%s' is not a "
                "declared [post_processor] of type %s" % (override, PA_TYPE)
            )
        return override, sc
    candidates = [n for n in by_name if n in _referenced_names(config)]
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
    return name, by_name[name]


def _discover_smooth_post_processor(config, override):
    by_name = _sections_by_type(config, ST_TYPE)
    if override is not None:
        sc = by_name.get(override)
        if sc is None:
            raise config.error(
                "[pressure_advance_compat] smooth_post_processor: '%s' is not "
                "a declared [post_processor] of type %s" % (override, ST_TYPE)
            )
        return override, sc
    candidates = [n for n in by_name if n in _referenced_names(config)]
    if not candidates:
        return None, None
    if len(candidates) > 1:
        raise config.error(
            "[pressure_advance_compat] found multiple %s post_processors %s; "
            "disambiguate with smooth_post_processor: <name>"
            % (ST_TYPE, sorted(candidates))
        )
    name = candidates[0]
    return name, by_name[name]


class PressureAdvanceCompat:
    def __init__(self, config):
        self.printer = config.get_printer()
        override = config.get("post_processor", None)
        self.post_processor, pa_section = _discover_post_processor(
            config, override
        )
        self.last_advance = pa_section.getfloat("k", 0.0, minval=0.0)
        smooth_override = config.get("smooth_post_processor", None)
        self.smooth_post_processor, st_section = (
            _discover_smooth_post_processor(config, smooth_override)
        )
        self.last_smooth_time = (
            st_section.getfloat("smooth_time", 0.0, minval=0.0)
            if st_section is not None
            else 0.0
        )
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
        smooth_time = gcmd.get_float("SMOOTH_TIME", None, minval=0.0)
        advance = gcmd.get_float("ADVANCE", None, minval=0.0)
        if smooth_time is not None:
            self._apply_smooth_time(gcmd, smooth_time)
        if advance is not None:
            self._apply_advance(gcmd, advance)
        if advance is None and smooth_time is None:
            gcmd.respond_info(
                "pressure_advance: %.6f smooth_time: %.6f (post_processor '%s')"
                % (
                    self.last_advance,
                    self.last_smooth_time,
                    self.post_processor,
                )
            )

    def _engine(self, gcmd):
        engine = self.printer.lookup_object("motion_engine", None)
        if engine is None:
            raise gcmd.error(
                "SET_PRESSURE_ADVANCE: motion_engine is not available"
            )
        return engine

    def _apply_advance(self, gcmd, advance):
        engine = self._engine(gcmd)
        try:
            engine.update_post_processor(self.post_processor, "k", advance)
        except (ValueError, RuntimeError) as e:
            raise gcmd.error(str(e))
        self.last_advance = advance

    def _apply_smooth_time(self, gcmd, smooth_time):
        if self.smooth_post_processor is None:
            gcmd.respond_info(
                "SET_PRESSURE_ADVANCE: SMOOTH_TIME ignored; no [post_processor] "
                "of type %s is referenced by any [axis]" % ST_TYPE
            )
            return
        engine = self._engine(gcmd)
        try:
            engine.update_post_processor(
                self.smooth_post_processor, "smooth_time", smooth_time
            )
        except (ValueError, RuntimeError) as e:
            raise gcmd.error(str(e))
        self.last_smooth_time = smooth_time


def load_config(config):
    return PressureAdvanceCompat(config)
