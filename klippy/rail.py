import collections

HomingInfo = collections.namedtuple(
    "HomingInfo",
    [
        "speed",
        "position_endstop",
        "retract_speed",
        "retract_dist",
        "positive_dir",
        "second_homing_speed",
        "use_sensorless_homing",
        "min_home_dist",
        "accel",
    ],
)


class BaseRail:
    def __init__(self):
        self.second_homing_speed = 0.0
        self.use_sensorless_homing = False
        self.min_home_dist = 0.0
        self.homing_accel = None

    def _parse_position_range(self, config):
        self.position_min = config.getfloat("position_min", 0.0)
        self.position_max = config.getfloat(
            "position_max", above=self.position_min
        )

    def _parse_homing_speeds(self, config):
        self.homing_speed = config.getfloat("homing_speed", 5.0, above=0.0)
        self.homing_retract_dist = config.getfloat(
            "homing_retract_dist", 5.0, minval=0.0
        )
        self.homing_retract_speed = config.getfloat(
            "homing_retract_speed", self.homing_speed, above=0.0
        )
        self.min_home_dist = config.getfloat(
            "min_home_dist", self.homing_retract_dist, minval=0.0
        )

    def _finalize_homing(self, config, endstop_is_virtual):
        if (
            self.position_endstop < self.position_min
            or self.position_endstop > self.position_max
        ):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % config.get_name()
            )
        self.use_sensorless_homing = config.getboolean(
            "use_sensorless_homing", endstop_is_virtual
        )

        self._parse_homing_speeds(config)

        default_second_homing_speed = self.homing_speed / 2.0
        if self.use_sensorless_homing:
            default_second_homing_speed = self.homing_speed

        self.second_homing_speed = config.getfloat(
            "second_homing_speed", default_second_homing_speed, above=0.0
        )
        self.homing_positive_dir = config.getboolean(
            "homing_positive_dir", None
        )

        self.homing_accel = config.getfloat("homing_accel", None, above=0.0)

        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.0:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.0:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'"
                    % (config.get_name(),)
                )
            config.getboolean("homing_positive_dir", self.homing_positive_dir)
        elif (
            self.homing_positive_dir
            and self.position_endstop == self.position_min
        ) or (
            not self.homing_positive_dir
            and self.position_endstop == self.position_max
        ):
            raise config.error(
                "Invalid homing_positive_dir / position_endstop in '%s'"
                % (config.get_name(),)
            )

    def get_range(self):
        return self.position_min, self.position_max

    def get_tmc_current_helpers(self):
        return []

    def get_homing_info(self):
        return HomingInfo(
            speed=self.homing_speed,
            position_endstop=self.position_endstop,
            retract_speed=self.homing_retract_speed,
            retract_dist=self.homing_retract_dist,
            positive_dir=self.homing_positive_dir,
            second_homing_speed=self.second_homing_speed,
            use_sensorless_homing=self.use_sensorless_homing,
            min_home_dist=self.min_home_dist,
            accel=self.homing_accel,
        )
