from . import stepper

_KIN_COREXY = 0
_KIN_CARTESIAN = 1


def load_kinematics(config, motion):
    """Build the kinematics from the topology the native reader parsed and
    validated ([kinematics] type/roles, [motor] drives, follower slotting,
    orphan rejection) in Motion._load_motion_config."""
    if motion.kinematics_decl is None:
        raise config.error("[kinematics] section is required")
    kind, lanes, _followers = motion.kinematics_decl
    frontend_visible_kinematics = kind
    config.getsection("printer").get("kinematics", frontend_visible_kinematics)
    return _LinearKinematics(config, motion, kind, lanes)


class _LinearKinematics:
    supports_dual_carriage = False

    def __init__(self, config, motion, kind, lanes):
        self._motion = motion
        self.kind = kind
        self._printer = config.get_printer()

        self._lanes = [
            (lane_idx, axis_name, motors)
            for lane_idx, axis_name, motors, _drive in lanes
        ]
        self.rails = [self._build_lane(config, lane) for lane in lanes]
        self.limits = [(1.0, -1.0)] * 3
        self._parked_dirty = [False, False, False]

        self._printer.load_object(config, "homing").resolve_endstops(self)
        self._printer.register_event_handler(
            "stepper_enable:motor_off", self._handle_motor_off
        )

    def _build_lane(self, config, lane):
        lane_idx, axis_name, motor_names, drive = lane
        motor_sections = [
            config.getsection("motor " + name) for name in motor_names
        ]
        if drive == "servo":
            return self._build_servo_lane(config, axis_name, motor_sections)
        rail = stepper.AxisRail(
            config.getsection("axis " + axis_name),
            list(zip(motor_sections, motor_names)),
        )
        rail.setup_itersolve(
            "cartesian_stepper_alloc", "xyz"[lane_idx].encode()
        )
        return rail

    def _build_servo_lane(self, config, axis_name, motor_sections):
        from .extras import servo_axis

        axis_config = config.getsection("axis " + axis_name)
        rail = servo_axis.ServoRail(axis_config, motor_sections)
        servo_axis.register_torque_enable(self._printer, config, rail)
        return rail

    def _handle_motor_off(self, print_time):
        for i in (0, 1, 2):
            if self._is_servo(i) and self.limits[i][0] <= self.limits[i][1]:
                self._parked_dirty[i] = True
            else:
                self.clear_homing_state([i])

    def _is_servo(self, axis):
        from .extras import servo_axis

        return isinstance(self.rails[axis], servo_axis.ServoRail)

    def mark_servo_parked(self, axes):
        for i in axes:
            if self._is_servo(i) and self.limits[i][0] <= self.limits[i][1]:
                self._parked_dirty[i] = True

    def parked_dirty_axes(self):
        return [i for i in (0, 1, 2) if self._parked_dirty[i]]

    def clear_parked_dirty(self, axes):
        for i in axes:
            self._parked_dirty[i] = False

    def _axis_rails(self):
        return {i: rail for i, rail in enumerate(self.rails)}

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
        moved = [abs(dx) > 1e-9, abs(dy) > 1e-9, abs(dz) > 1e-9]
        if self.coupled_xy():
            moved[0] = moved[1] = moved[0] or moved[1]
        return [
            self.rails[lane_idx]
            for lane_idx, _, _ in self._lanes
            if moved[lane_idx]
        ]

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
        move.limit_speed(self._motion.max_z_velocity * z_ratio)

    def set_position(self, newpos, homing_axes=()):
        self._motion.engine.set_position(newpos[0], newpos[1], newpos[2])
        for axis in homing_axes:
            self.limits[axis] = self.rails[axis].get_range()
            self._parked_dirty[axis] = False

    def note_z_not_homed(self):
        self.clear_homing_state([2])

    def clear_homing_state(self, axes):
        for i in (0, 1, 2):
            if i in axes:
                self.limits[i] = (1.0, -1.0)
                self._parked_dirty[i] = False

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
