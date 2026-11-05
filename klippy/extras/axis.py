RESERVED_LETTERS = ("i", "j", "p", "q", "f", "g", "m", "n", "t")
RATIO_RAMP_RULE = (
    "every follower now ramps each extrusion-ratio step over the shortest"
    " time between 5 and 30 ms that keeps its advance within the axis's"
    " max_velocity and max_accel, and the planner slows the toolhead at a"
    " step where 30 ms is not enough. Delete the option; set [axis]"
    " max_velocity / max_accel (or the [extruder]'s"
    " max_extrude_only_velocity / max_extrude_only_accel they inherit)"
    " instead"
)


class AxisSection:
    def __init__(self, config):
        self.name = config.get_name().split(None, 1)[1]
        if (
            len(self.name) != 1
            or not self.name.islower()
            or not self.name.isalpha()
        ):
            raise config.error(
                "[%s]: axis name must be a single letter a-z"
                % config.get_name()
            )
        if self.name in RESERVED_LETTERS:
            raise config.error(
                "[%s]: letter '%s' is reserved for G-code structure"
                % (config.get_name(), self.name)
            )
        self.follows = [
            a.strip().lower() for a in config.getlist("follows", [])
        ]
        self.motors = [m.strip() for m in config.getlist("motors", [])]
        self.post_processors = [
            p.strip() for p in config.getlist("post_processors", [])
        ]
        if config.get("ratio_ramp_time", None, note_valid=False) is not None:
            raise config.error(
                "[%s] option 'ratio_ramp_time' was removed: %s"
                % (config.get_name(), RATIO_RAMP_RULE)
            )
        self.max_velocity = config.getfloat("max_velocity", None, above=0.0)
        self.max_accel = config.getfloat("max_accel", None, above=0.0)
        self.instantaneous_corner_velocity = config.getfloat(
            "instantaneous_corner_velocity", None, above=0.0
        )

    def get_status(self, eventtime):
        return {
            "follows": list(self.follows),
            "motors": list(self.motors),
            "post_processors": list(self.post_processors),
            "max_velocity": self.max_velocity,
            "max_accel": self.max_accel,
            "instantaneous_corner_velocity": self.instantaneous_corner_velocity,
        }


def load_config_prefix(config):
    return AxisSection(config)
