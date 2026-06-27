import collections

from .. import pins
from ..motion_endstop import allocate_provider_id
from ..rail import BaseRail
from . import servo_param

VIRTUAL_ENDSTOP_PIN = "virtual_endstop"


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
    if position_endstop == position_min:
        return False
    if position_endstop == position_max:
        return True
    raise config.error(
        "[axis %s]: position_endstop %.3f must equal position_min (%.3f) "
        "or position_max (%.3f)"
        % (axis, position_endstop, position_min, position_max)
    )


class ServoRail(BaseRail):
    def __init__(self, axis_config, motor_config):
        super().__init__()
        self.printer = axis_config.get_printer()
        self.name = axis_config.get_name()
        self.axis = self.name.split()[-1]
        if self.axis not in ("x", "y", "z"):
            raise axis_config.error(
                "[axis %s]: axis must be one of x/y/z (got %r)"
                % (self.axis, self.axis)
            )
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
        self.ff_torque_clamp = motor_config.getfloat(
            "ff_torque_clamp", 30.0, above=0.0, maxval=400.0
        )
        self.invert_direction = motor_config.getboolean(
            "invert_direction", False
        )
        self._parse_position_range(axis_config)
        self.endstop_pin = axis_config.get("endstop_pin", None)
        if self.endstop_pin is None:
            self.position_endstop = 0.0
            self.homing_speed = 0.0
            self.homing_retract_dist = 0.0
            self.homing_retract_speed = 0.0
            self.homing_positive_dir = False
            self.homing_following_error = 0.0
            self.homing_max_torque = 0.0
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
            self.homing_following_error = motor_config.getfloat(
                "homing_following_error", 2.5, above=0.0
            )
            self.homing_max_torque = motor_config.getfloat(
                "homing_max_torque", 50.0, above=0.0, maxval=400.0
            )
        self.following_error = motor_config.getfloat(
            "following_error", None, above=0.0
        )
        self.max_torque = motor_config.getfloat(
            "max_torque", None, above=0.0, maxval=400.0
        )
        self._active_callbacks = []
        try:
            self.sdo_params = servo_param.parse_params_block(
                motor_config.get("params", "")
            )
        except ValueError as e:
            raise motor_config.error(
                "[%s] params: %s" % (motor_config.get_name(), e)
            )
        self._virtual_endstop = None
        if self.printer is not None:
            ppins = self.printer.lookup_object("pins")
            ppins.register_chip("servo_" + self.axis, self)

    def get_name(self, short=False):
        if short:
            return self.axis
        return self.name

    def get_steppers(self):
        return []

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def get_endstops(self):
        return []

    def setup_itersolve(self, alloc_func, *params):
        return

    def add_extra_stepper(self, config):
        raise config.error(
            "servo_%s does not support extra steppers" % self.axis
        )

    def set_position(self, coord):
        return

    def get_commanded_position(self):
        return 0.0

    def get_node_name(self):
        return self.node_name

    def get_chain_index(self):
        return self.chain_index

    def get_motor_name(self):
        return self.motor_name

    def get_counts_per_mm(self):
        return self.encoder_counts_per_rev / self.rotation_distance

    def get_rotation_distance(self):
        return self.rotation_distance

    def get_ff_config(self):
        return (self.velocity_ff, self.ff_torque_clamp)

    def get_invert_direction(self):
        return self.invert_direction

    def get_sdo_params(self):
        return self.sdo_params

    def get_homing_drive_limits(self):
        counts_per_mm = self.encoder_counts_per_rev / self.rotation_distance
        return (
            int(round(self.homing_following_error * counts_per_mm)),
            int(round(self.homing_max_torque * 10.0)),
        )

    def setup_motion_endstop(self, pin_params, axis):
        if pin_params["pin"] != VIRTUAL_ENDSTOP_PIN:
            raise pins.error(
                "servo_%s only provides the '%s' virtual pin, not '%s'"
                % (self.axis, VIRTUAL_ENDSTOP_PIN, pin_params["pin"])
            )
        if axis != "xyz".index(self.axis):
            raise pins.error(
                "servo_%s:%s is only usable as the %s endstop"
                % (self.axis, VIRTUAL_ENDSTOP_PIN, self.axis.upper())
            )
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error("Can not pullup/invert the servo virtual endstop")
        self._virtual_endstop = ServoVirtualEndstop(
            self.printer, self.node_name, allocate_provider_id(self.printer)
        )
        return self._virtual_endstop

    def _engine_handle(self):
        return self._engine_node().get_engine_handle()

    def _engine_node(self):
        node = self.printer.lookup_object("ethercat_node " + self.node_name)
        if node.get_engine_handle() is None:
            raise self.printer.command_error(
                "servo sensorless homing: ethercat_node %s has no engine handle"
                % (self.node_name,)
            )
        return node

    def _engine_slot(self):
        return self._engine_node().get_slot_for_motor(self.motor_name)

    def trip_move_begin(self, entry):
        _, torque_trip_tenth_pct = self.get_homing_drive_limits()
        engine = self.printer.lookup_object("motion_engine")
        engine.arm_sensorless_endstop(
            self._engine_handle(),
            self._engine_slot(),
            entry["endstop"].endstop_id,
            torque_trip_tenth_pct,
            True,
        )

    def trip_move_end(self, entry):
        engine = self.printer.lookup_object("motion_engine")
        engine.disarm_sensorless_endstop(
            self._engine_handle(),
            self._engine_slot(),
            entry["endstop"].endstop_id,
        )

    def get_session_drive_limits(self):
        counts_per_mm = self.encoder_counts_per_rev / self.rotation_distance
        counts = None
        if self.following_error is not None:
            counts = int(round(self.following_error * counts_per_mm))
        tenth_pct = None
        if self.max_torque is not None:
            tenth_pct = int(round(self.max_torque * 10.0))
        return counts, tenth_pct


class MotionTorqueLine:
    def __init__(self, printer, node_name, motor_name):
        self._printer = printer
        self._node_name = node_name
        self._motor_name = motor_name

    def set_digital(self, print_time, value):
        node = self._printer.lookup_object("ethercat_node " + self._node_name)
        node.set_motor_torque(self._motor_name, bool(value), print_time)


def register_torque_enable(printer, config, rail):
    from . import stepper_enable

    line = MotionTorqueLine(printer, rail.get_node_name(), rail.get_name())
    enable = stepper_enable.StepperEnablePin(line, 0)
    printer.load_object(config, "stepper_enable").register_motor(
        rail.get_name(), rail, enable
    )
