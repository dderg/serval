from . import stepper

DRIVE_CHOICES = {"stepper": "stepper", "servo": "servo"}

_KIN_COREXY = 0
_KIN_CARTESIAN = 1


def resolve_motor_section(config, name, referenced_by):
    if not config.has_section(name):
        raise config.error(
            "%s references motor '%s' but no [%s] section exists"
            % (referenced_by, name, name)
        )
    section = config.getsection(name)
    drive = section.getchoice("drive", DRIVE_CHOICES, "stepper")
    return section, drive


KINEMATICS_TYPES = {
    "corexy": [
        ("a_motors", "axis_x", 0),
        ("b_motors", "axis_y", 1),
        ("z_motors", "axis_z", 2),
    ],
    "cartesian": [
        ("x_motors", "axis_x", 0),
        ("y_motors", "axis_y", 1),
        ("z_motors", "axis_z", 2),
    ],
}


def load_kinematics(config, motion):
    if config.getsection("printer").get("kinematics", None) is not None:
        raise config.error(
            "[printer] kinematics is not supported: declare a [kinematics] "
            "section (type + axis role bindings + motor lists)"
        )
    if not config.has_section("kinematics"):
        raise config.error("[kinematics] section is required")
    section = config.getsection("kinematics")
    kind = section.get("type")
    if kind not in KINEMATICS_TYPES:
        raise config.error(
            "[kinematics] type '%s' is not supported (supported: %s)"
            % (kind, ", ".join(sorted(KINEMATICS_TYPES)))
        )
    return _LinearKinematics(config, motion, kind, KINEMATICS_TYPES[kind])


class _LinearKinematics:
    supports_dual_carriage = False

    def __init__(self, config, motion, kind, role_specs):
        self._motion = motion
        self.kind = kind
        self._role_specs = role_specs
        self._printer = config.get_printer()
        section = config.getsection("kinematics")

        self._lanes = self._read_lanes(config, section)
        if [lane_idx for lane_idx, _, _ in self._lanes] != list(
            range(len(self._lanes))
        ):
            raise config.error(
                "[kinematics] internal error: lanes must be contiguous 0..N "
                "(got %s)" % ([lane[0] for lane in self._lanes],)
            )
        self.rails = [self._build_lane(config, lane) for lane in self._lanes]
        self.limits = [(1.0, -1.0)] * 3

        self._printer.load_object(config, "homing").resolve_endstops()
        self._printer.register_event_handler(
            "stepper_enable:motor_off", self._handle_motor_off
        )

    def _read_lanes(self, config, section):
        lanes = []
        for role_motors_key, axis_role_key, lane_idx in self._role_specs:
            axis_name = section.get(axis_role_key)
            if not config.has_section("axis " + axis_name):
                raise config.error(
                    "[kinematics] %s binds to axis '%s' but no [axis %s] "
                    "section exists" % (axis_role_key, axis_name, axis_name)
                )
            motor_names = section.getlist(role_motors_key, [])
            if not motor_names:
                raise config.error(
                    "[kinematics] %s declares no motors (lane %d needs at "
                    "least one motor)" % (role_motors_key, lane_idx)
                )
            lanes.append((lane_idx, axis_name, motor_names))
        lanes.sort(key=lambda lane: lane[0])
        return lanes

    def _build_lane(self, config, lane):
        lane_idx, axis_name, motor_names = lane
        role_motors_key = self._role_specs[lane_idx][0]
        referenced_by = "[kinematics] %s" % role_motors_key
        motor_sections = []
        drives = set()
        for motor_name in motor_names:
            motor_section, drive = resolve_motor_section(
                config, motor_name, referenced_by
            )
            motor_sections.append(motor_section)
            drives.add(drive)
        if len(drives) > 1:
            raise config.error(
                "%s mixes stepper and servo motors in one lane; a lane must "
                "be all-stepper or all-servo" % referenced_by
            )
        drive = drives.pop()
        if drive == "servo":
            return self._build_servo_lane(config, lane, motor_sections)
        rail = stepper.AxisRail(
            config.getsection("axis " + axis_name), motor_sections
        )
        rail.setup_itersolve(
            "cartesian_stepper_alloc", "xyz"[lane_idx].encode()
        )
        return rail

    def _build_servo_lane(self, config, lane, motor_sections):
        lane_idx, axis_name, motor_names = lane
        if len(motor_sections) != 1:
            raise config.error(
                "[kinematics] servo lane '%s' must reference exactly one "
                "servo motor" % axis_name
            )
        from .extras import servo_axis

        axis_config = config.getsection("axis " + axis_name)
        rail = servo_axis.ServoRail(axis_config, motor_sections[0])
        servo_axis.register_torque_enable(self._printer, config, rail)
        return rail

    def _handle_motor_off(self, print_time):
        self.clear_homing_state((0, 1, 2))

    def claimed_axes(self):
        return [axis_name for _, axis_name, _ in self._lanes]

    def lanes(self):
        return self._lanes

    def coupled_xy(self):
        return self.kind == "corexy"

    def mcu_tag(self, lanes_on_mcu):
        on_mcu = set(lanes_on_mcu)
        if self.coupled_xy() and 0 in on_mcu and 1 in on_mcu:
            return _KIN_COREXY
        return _KIN_CARTESIAN

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def active_rails(self, dx, dy, dz):
        moved = {
            axis: abs(delta) > 1e-9 for axis, delta in zip("xyz", (dx, dy, dz))
        }
        coupled = dict(moved)
        if self.coupled_xy():
            coupled["x"] = coupled["y"] = moved["x"] or moved["y"]
        active = []
        for lane_idx, _, _ in self._lanes:
            if coupled["xyz"[lane_idx]]:
                active.append(self.rails[lane_idx])
        return active

    def calc_position(self, stepper_positions):
        def rail_pos(rail):
            vals = [
                stepper_positions.get(s.get_name(), 0.0)
                for s in rail.get_steppers()
            ]
            if not vals:
                return 0.0
            return sum(vals) / len(vals)

        return [rail_pos(rail) for rail in self.rails]

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0] or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < limits[0][0]
            or xpos > limits[0][1]
            or ypos < limits[1][0]
            or ypos > limits[1][1]
        ):
            self._check_endstops(move)
        if not move.axes_d[2]:
            return
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self._motion._axis_limit("z", "max_velocity") * z_ratio,
            self._motion._axis_limit("z", "max_accel") * z_ratio,
        )

    def set_position(self, newpos, homing_axes=()):
        self._motion.bridge.set_position(newpos[0], newpos[1], newpos[2])
        for axis in homing_axes:
            self.limits[axis] = self.rails[axis].get_range()

    def note_z_not_homed(self):
        self.clear_homing_state([2])

    def clear_homing_state(self, axes):
        for i in (0, 1, 2):
            if i in axes:
                self.limits[i] = (1.0, -1.0)

    def get_status(self, eventtime):
        from . import gcode as gcode_mod

        x_min, x_max = self.rails[0].get_range()
        y_min, y_max = self.rails[1].get_range()
        z_min, z_max = self.rails[2].get_range()
        homed = "".join(
            a
            for i, a in enumerate("xyz")
            if self.limits[i][0] <= self.limits[i][1]
        )
        return {
            "homed_axes": homed,
            "axis_minimum": gcode_mod.Coord(x_min, y_min, z_min, 0.0),
            "axis_maximum": gcode_mod.Coord(x_max, y_max, z_max, 0.0),
        }
