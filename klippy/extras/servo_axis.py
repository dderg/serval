import collections

from .. import pins
from ..motion_endstop import allocate_provider_id
from ..rail import BaseRail
from . import servo_param

VIRTUAL_ENDSTOP_PIN = "virtual_endstop"

MAX_TORQUE_PCT_6072H = 400.0
ENGINE_FF_LEAD_CYCLES_MAX = 40


def read_dynamics_profile_option(config, option="dynamics_profile"):
    path = config.get(option, None)
    if path is None:
        return None
    try:
        with open(path, "rb"):
            pass
    except OSError as e:
        raise config.error(
            "[%s] %s: cannot read dynamics profile '%s': %s"
            % (config.get_name(), option, path, e)
        )
    return path


class ServoVirtualEndstop:
    """Servo axis as a virtual endstop. arm/disarm are benign; the real
    device-side arming is the provider's trip_move_begin/trip_move_end."""

    def __init__(self, printer, node_name, endstop_id):
        self._printer = printer
        self._node_name = node_name
        self.endstop_id = endstop_id

    def engine_mcu_handle(self):
        node = self._printer.lookup_object("ethercat_node " + self._node_name)
        return node.get_engine_handle()

    def is_triggered(self):
        return False

    def query_endstop(self, print_time):
        return False

    def arm(self, poll_period):
        del poll_period

    def disarm(self):
        pass


_homing_info = collections.namedtuple(
    "homing_info",
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


def infer_positive_dir(
    config, axis, position_endstop, position_min, position_max
):
    """The endstop must sit at or beyond an end of travel; placing it past
    position_min/position_max keeps that margin between the reachable range
    and the physical crash point a sensorless home lands on."""
    if position_endstop <= position_min:
        return False
    if position_endstop >= position_max:
        return True
    raise config.error(
        "[axis %s]: position_endstop %.3f must be at or beyond position_min "
        "(%.3f) or position_max (%.3f)"
        % (axis, position_endstop, position_min, position_max)
    )


class ServoMotor:
    """One EtherCAT drive of a servo rail: everything configured per
    [motor ...] section, while the rail owns the axis-level options."""

    def __init__(self, motor_config, has_endstop):
        protocol = motor_config.get("protocol")
        if protocol != "ethercat":
            raise motor_config.error(
                "[%s]: only 'protocol: ethercat' is supported "
                "(got %r)" % (motor_config.get_name(), protocol)
            )
        self.node_name = motor_config.get("node")
        self.chain_index = motor_config.getint("ethercat_chain_index", minval=0)
        self.motor_name = motor_config.get_name().split(None, 1)[1]
        self.rotation_distance = motor_config.getfloat(
            "rotation_distance", above=0.0
        )
        self.encoder_counts_per_rev = motor_config.getint(
            "encoder_counts_per_rev", minval=1
        )
        self.velocity_ff = motor_config.getboolean("velocity_ff", False)
        self.ff_max_torque = motor_config.getfloat(
            "ff_max_torque", 30.0, above=0.0, maxval=MAX_TORQUE_PCT_6072H
        )
        self.ff_lead_cycles = motor_config.getint(
            "ff_lead_cycles", 0, minval=0, maxval=ENGINE_FF_LEAD_CYCLES_MAX
        )
        self.invert_direction = motor_config.getboolean(
            "invert_direction", False
        )
        if has_endstop:
            self.homing_following_error = motor_config.getfloat(
                "homing_following_error", 2.5, above=0.0
            )
            self.homing_max_torque = motor_config.getfloat(
                "homing_max_torque",
                50.0,
                above=0.0,
                maxval=MAX_TORQUE_PCT_6072H,
            )
        else:
            self.homing_following_error = 0.0
            self.homing_max_torque = 0.0
        self.following_error = motor_config.getfloat(
            "following_error", None, above=0.0
        )
        self.max_torque = motor_config.getfloat(
            "max_torque", None, above=0.0, maxval=MAX_TORQUE_PCT_6072H
        )
        self.dynamics_profile = read_dynamics_profile_option(motor_config)
        try:
            self.sdo_params = servo_param.parse_params_block(
                motor_config.get("params", "")
            )
        except ValueError as e:
            raise motor_config.error(
                "[%s] params: %s" % (motor_config.get_name(), e)
            )
        self.tuning_profile_name = motor_config.get("tuning_profile", None)
        if self.tuning_profile_name is not None:
            try:
                profile_params, profile_path = servo_param.load_tuning_profile(
                    self.tuning_profile_name
                )
            except ValueError as e:
                raise motor_config.error(
                    "[%s] tuning_profile: %s" % (motor_config.get_name(), e)
                )
            overlap = sorted(
                {(i, s) for i, s, _sz, _v in profile_params}
                & {(i, s) for i, s, _sz, _v in self.sdo_params}
            )
            if overlap:
                index, subindex = overlap[0]
                raise motor_config.error(
                    "[%s]: 0x%04x.%d is set by both tuning_profile %s (%s) "
                    "and the params: block — remove it from one"
                    % (
                        motor_config.get_name(),
                        index,
                        subindex,
                        self.tuning_profile_name,
                        profile_path,
                    )
                )
            self.sdo_params = profile_params + self.sdo_params

    def get_motor_name(self):
        return self.motor_name

    def get_node_name(self):
        return self.node_name

    def get_chain_index(self):
        return self.chain_index

    def get_counts_per_mm(self):
        return self.encoder_counts_per_rev / self.rotation_distance

    def get_rotation_distance(self):
        return self.rotation_distance

    def get_ff_config(self):
        return (self.velocity_ff, self.ff_max_torque, self.ff_lead_cycles)

    def get_invert_direction(self):
        return self.invert_direction

    def get_dynamics_profile(self):
        return self.dynamics_profile

    def get_sdo_params(self):
        return self.sdo_params

    def get_homing_drive_limits(self):
        counts_per_mm = self.get_counts_per_mm()
        return (
            int(round(self.homing_following_error * counts_per_mm)),
            int(round(self.homing_max_torque * 10.0)),
        )

    def get_session_drive_limits(self):
        counts_per_mm = self.get_counts_per_mm()
        counts = None
        if self.following_error is not None:
            counts = int(round(self.following_error * counts_per_mm))
        tenth_pct = None
        if self.max_torque is not None:
            tenth_pct = int(round(self.max_torque * 10.0))
        return counts, tenth_pct


class ServoRail(BaseRail):
    def __init__(self, axis_config, motor_configs):
        super().__init__()
        self.printer = axis_config.get_printer()
        self.name = axis_config.get_name()
        self.axis = self.name.split()[-1]
        if self.axis not in ("x", "y", "z"):
            raise axis_config.error(
                "[axis %s]: axis must be one of x/y/z (got %r)"
                % (self.axis, self.axis)
            )
        self._parse_position_range(axis_config)
        self.endstop_pin = axis_config.get("endstop_pin", None)
        if self.endstop_pin is None:
            self.position_endstop = 0.0
            self.homing_speed = 0.0
            self.homing_retract_dist = 0.0
            self.homing_retract_speed = 0.0
            self.homing_positive_dir = False
        else:
            self.position_endstop = axis_config.getfloat("position_endstop")
            self._parse_homing_speeds(axis_config)
            self.homing_positive_dir = infer_positive_dir(
                axis_config,
                self.axis,
                self.position_endstop,
                self.position_min,
                self.position_max,
            )
        self.motors = [
            ServoMotor(mc, self.endstop_pin is not None) for mc in motor_configs
        ]
        nodes = {m.get_node_name() for m in self.motors}
        if len(nodes) > 1:
            raise axis_config.error(
                "[axis %s]: servo motors %s span EtherCAT nodes %s — all "
                "motors of one axis must share a node"
                % (
                    self.axis,
                    ", ".join(m.get_motor_name() for m in self.motors),
                    ", ".join(sorted(nodes)),
                )
            )
        self._active_callbacks = []
        self._virtual_endstop = None
        if self.printer is not None:
            ppins = self.printer.lookup_object("pins")
            for motor in self.motors:
                ppins.register_chip(motor.get_motor_name(), self)

    def get_name(self, short=False):
        if short:
            return self.axis
        return self.name

    def get_steppers(self):
        return []

    def get_motors(self):
        return self.motors

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def get_endstops(self):
        return []

    def setup_itersolve(self, alloc_func, *params):
        return

    def set_position(self, coord):
        return

    def get_commanded_position(self):
        return 0.0

    def get_node_name(self):
        return self.motors[0].get_node_name()

    def setup_motion_endstop(self, pin_params, axis):
        if pin_params["pin"] != VIRTUAL_ENDSTOP_PIN:
            raise pins.error(
                "%s only provides the '%s' virtual pin, not '%s'"
                % (
                    self.motors[0].get_motor_name(),
                    VIRTUAL_ENDSTOP_PIN,
                    pin_params["pin"],
                )
            )
        if axis != "xyz".index(self.axis):
            raise pins.error(
                "servo axis %s is only usable as the %s endstop"
                % (self.axis, self.axis.upper())
            )
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error("Can not pullup/invert the servo virtual endstop")
        self._virtual_endstop = ServoVirtualEndstop(
            self.printer,
            self.get_node_name(),
            allocate_provider_id(self.printer),
        )
        return self._virtual_endstop

    def _engine_node(self):
        node = self.printer.lookup_object(
            "ethercat_node " + self.get_node_name()
        )
        if node.get_engine_handle() is None:
            raise self.printer.command_error(
                "servo sensorless homing: ethercat_node %s has no engine handle"
                % (self.get_node_name(),)
            )
        return node

    def trip_move_begin(self, entry):
        node = self._engine_node()
        engine = self.printer.lookup_object("motion_engine")
        for motor in self.motors:
            _, torque_trip_tenth_pct = motor.get_homing_drive_limits()
            engine.arm_sensorless_endstop(
                node.get_engine_handle(),
                node.get_slot_for_motor(motor.get_motor_name()),
                entry["endstop"].endstop_id,
                torque_trip_tenth_pct,
                True,
            )

    def trip_move_end(self, entry):
        node = self._engine_node()
        engine = self.printer.lookup_object("motion_engine")
        for motor in self.motors:
            engine.disarm_sensorless_endstop(
                node.get_engine_handle(),
                node.get_slot_for_motor(motor.get_motor_name()),
                entry["endstop"].endstop_id,
            )


def iter_servo_motors(kin):
    for rail in getattr(kin, "rails", ()):
        if isinstance(rail, ServoRail):
            for motor in rail.get_motors():
                yield rail, motor


def resolve_servo_motor(printer, name, context):
    """Resolve a SERVO= style name to one (rail, motor) pair. Motor names
    match directly; an axis/rail name only resolves when the rail has a
    single motor — with AWD the caller must name the drive."""
    kin = printer.lookup_object("toolhead").get_kinematics()
    pairs = list(iter_servo_motors(kin))
    for rail, motor in pairs:
        if name == motor.get_motor_name():
            return rail, motor
    for rail in {rail for rail, _motor in pairs}:
        if name in (rail.get_name(), rail.get_name(short=True)):
            motors = rail.get_motors()
            if len(motors) == 1:
                return rail, motors[0]
            raise printer.command_error(
                "%s: axis %s is driven by multiple servos (%s) — name the "
                "motor"
                % (
                    context,
                    rail.get_name(short=True),
                    ", ".join(m.get_motor_name() for m in motors),
                )
            )
    known = ", ".join(motor.get_motor_name() for _rail, motor in pairs)
    raise printer.command_error(
        "%s: no servo motor named %r (known: %s)"
        % (context, name, known or "none")
    )


class MotionTorqueLine:
    def __init__(self, printer, node_name, motor_name):
        self._printer = printer
        self._node_name = node_name
        self._motor_name = motor_name

    def set_digital(self, print_time, value):
        node = self._printer.lookup_object("ethercat_node " + self._node_name)
        return node.set_motor_torque(self._motor_name, bool(value), print_time)


def register_torque_enable(printer, config, rail):
    from . import stepper_enable

    line = MotionTorqueLine(printer, rail.get_node_name(), rail.get_name())
    enable = stepper_enable.StepperEnablePin(line, 0)
    printer.load_object(config, "stepper_enable").register_motor(
        rail.get_name(), rail, enable
    )
