class LimitSection:
    def __init__(self, config):
        self.name = config.get_name().split(None, 1)[1]
        self.axes = [a.strip().lower() for a in config.getlist("axes")]
        if not self.axes:
            raise config.error(
                "[%s]: axes must not be empty" % config.get_name()
            )
        self.max_velocity = config.getfloat("max_velocity", None, above=0.0)
        self.max_accel = config.getfloat("max_accel", None, above=0.0)
        self.max_jerk = config.getfloat("max_jerk", None, minval=0.0)
        if self.max_jerk == 0.0:
            self.max_jerk = float("inf")
        if (
            self.max_velocity is None
            and self.max_accel is None
            and self.max_jerk is None
        ):
            raise config.error(
                "[%s]: declare at least one of max_velocity, max_accel, "
                "max_jerk" % config.get_name()
            )

    def get_status(self, eventtime):
        return {
            "axes": list(self.axes),
            "max_velocity": self.max_velocity,
            "max_accel": self.max_accel,
            "max_jerk": self.max_jerk,
        }


def load_config_prefix(config):
    return LimitSection(config)
