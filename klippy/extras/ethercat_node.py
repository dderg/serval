import logging
import os
from collections import namedtuple

from . import servo_axis

# One drive slot passed to engine.claim_ethercat_node. The engine extracts each
# field by attribute name (mirrored by a Rust named struct), so a reordered
# field fails loud rather than silently swapping, say, axis and chain_index.
EthercatDrive = namedtuple(
    "EthercatDrive",
    [
        "chain_index",
        "axis",
        "counts_per_mm",
        "rotation_distance",
        "following_error_counts",
        "max_torque_tenth_pct",
        "velocity_ff",
        "ff_max_torque",
        "ff_lead_cycles",
        "invert_direction",
        "dynamics_profile",
    ],
)

# Default endpoint binary: ethercat_node.py lives at
# <repo>/klippy/extras/, so three os.path.dirname hops reach <repo>.
_REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
_DEFAULT_ENDPOINT = os.path.join(
    _REPO_ROOT, "rust", "target", "release", "ethercat-rt"
)

DRIVE_FAULT_POLL_PERIOD = 1.0

EC_RT_MAX_SLAVES = 8

CYCLE_US_QUANTUM = 250

# Per-motor options that must be identical across a coupled node: a
# node-level dynamics profile computes each motor's torque feedforward
# from every motor's commanded kinematics, so asymmetry in the FF path
# skews the coupled model instead of tuning one motor.
COUPLED_UNIFORM_OPTIONS = (
    ("velocity_ff", lambda motor: motor.get_ff_config()[0]),
    ("ff_max_torque", lambda motor: motor.get_ff_config()[1]),
    ("ff_lead_cycles", lambda motor: motor.get_ff_config()[2]),
)


class EtherCatNode:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        socket_path = config.get("socket").strip()
        if not socket_path:
            raise config.error(
                "ethercat_node %s: 'socket' must be a non-empty path"
                % (self.name,)
            )
        self.socket_path = socket_path
        interface = config.get("interface").strip()
        if not interface:
            raise config.error(
                "ethercat_node %s: 'interface' must be a non-empty "
                "NIC name (e.g. eth0)" % (self.name,)
            )
        self.interface = interface
        self.endpoint = os.path.abspath(
            config.get("endpoint", _DEFAULT_ENDPOINT)
        )
        self.cycle_us = config.getint("cycle_us", CYCLE_US_QUANTUM)
        if self.cycle_us <= 0 or self.cycle_us % CYCLE_US_QUANTUM != 0:
            raise config.error(
                "ethercat_node %s: cycle_us=%d is invalid — the sync cycle "
                "must be a positive integer multiple of %d us"
                % (self.name, self.cycle_us, CYCLE_US_QUANTUM)
            )
        self.dynamics_profile = config.get("dynamics_profile", None)
        self.engine_handle = None
        self._counts_per_mm = None
        self._slot_by_motor = {}
        self._torque_motors = set()
        self.printer.register_event_handler("klippy:mcu_identify", self._claim)
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )
        self.printer.load_object(config, "servo_capture")
        self.printer.load_object(config, "servo_param")

    def _find_motors(self):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        found = []
        for lane_idx, _axis_name, _motor_names in kin.lanes():
            rail = kin.rails[lane_idx]
            if not isinstance(rail, servo_axis.ServoRail):
                continue
            for motor in rail.get_motors():
                if motor.get_node_name() == self.name:
                    found.append((lane_idx, motor))
        if not found:
            raise self.printer.config_error(
                "ethercat_node %s: no servo motor with node=%s — "
                "cannot locate any drives" % (self.name, self.name)
            )
        return found

    def _validate_chain(self, motors):
        by_index = {}
        for _global_axis, motor in motors:
            idx = motor.get_chain_index()
            if idx >= EC_RT_MAX_SLAVES:
                raise self.printer.config_error(
                    "ethercat_node %s: motor %s ethercat_chain_index=%d "
                    "exceeds the %d-drive endpoint limit (valid 0..%d)"
                    % (
                        self.name,
                        motor.get_motor_name(),
                        idx,
                        EC_RT_MAX_SLAVES,
                        EC_RT_MAX_SLAVES - 1,
                    )
                )
            if idx in by_index:
                raise self.printer.config_error(
                    "ethercat_node %s: motors %s and %s share "
                    "ethercat_chain_index=%d — each drive on a chain needs a "
                    "distinct position"
                    % (
                        self.name,
                        by_index[idx],
                        motor.get_motor_name(),
                        idx,
                    )
                )
            by_index[idx] = motor.get_motor_name()

    def _validate_dynamics_profiles(self, motors):
        per_servo = [
            (motor.get_motor_name(), motor.get_dynamics_profile())
            for _global_axis, motor in motors
        ]
        configured = [
            name for name, profile in per_servo if profile is not None
        ]
        if not configured:
            return
        if self.dynamics_profile is not None:
            raise self.printer.config_error(
                "ethercat_node %s: dynamics_profile is set on [ethercat_node] "
                "and on [motor %s]; a node is either coupled (one node-level "
                "profile) or independent (one profile per motor), not both"
                % (self.name, configured[0])
            )
        missing = [name for name, profile in per_servo if profile is None]
        if missing:
            raise self.printer.config_error(
                "ethercat_node %s: dynamics_profile must be set on every motor "
                "or none — missing on: %s" % (self.name, ", ".join(missing))
            )

    def _validate_coupled_uniformity(self, motors):
        if self.dynamics_profile is None:
            return
        for option, read in COUPLED_UNIFORM_OPTIONS:
            values = {
                motor.get_motor_name(): read(motor)
                for _global_axis, motor in motors
            }
            if len(set(values.values())) > 1:
                raise self.printer.config_error(
                    "ethercat_node %s: a coupled (node-level) "
                    "dynamics_profile computes each motor's torque "
                    "feedforward from every motor's commanded kinematics, "
                    "so %s must be identical across the node — got %s"
                    % (
                        self.name,
                        option,
                        ", ".join(
                            "%s=%s" % (name, value)
                            for name, value in sorted(values.items())
                        ),
                    )
                )

    def _claim(self):
        if self.engine_handle is not None:
            return
        motors = sorted(
            self._find_motors(),
            key=lambda pair: (pair[0], pair[1].get_chain_index()),
        )
        self._validate_chain(motors)
        self._slot_by_motor = {
            motor.get_motor_name(): slot
            for slot, (_global_axis, motor) in enumerate(motors)
        }
        self._validate_dynamics_profiles(motors)
        self._validate_coupled_uniformity(motors)
        drives = []
        for global_axis, motor in motors:
            following_error_counts, max_torque_tenth_pct = (
                motor.get_session_drive_limits()
            )
            velocity_ff, ff_max_torque, ff_lead_cycles = motor.get_ff_config()
            drives.append(
                EthercatDrive(
                    chain_index=motor.get_chain_index(),
                    axis=global_axis,
                    counts_per_mm=motor.get_counts_per_mm(),
                    rotation_distance=motor.get_rotation_distance(),
                    following_error_counts=following_error_counts,
                    max_torque_tenth_pct=max_torque_tenth_pct,
                    velocity_ff=velocity_ff,
                    ff_max_torque=ff_max_torque,
                    ff_lead_cycles=ff_lead_cycles,
                    invert_direction=motor.get_invert_direction(),
                    dynamics_profile=motor.get_dynamics_profile(),
                )
            )
        self._counts_per_mm = motors[0][1].get_counts_per_mm()
        engine = self.printer.lookup_object("motion_engine")
        try:
            self.engine_handle = engine.claim_ethercat_node(
                self.name,
                self.socket_path,
                self.interface,
                self.endpoint,
                self.cycle_us,
                self.dynamics_profile,
                drives,
            )
        except RuntimeError as e:
            raise self.printer.config_error(str(e))
        logging.info(
            "ethercat_node %s: claimed handle=%s socket=%s interface=%s "
            "endpoint=%s drives=%s dynamics_profile=%s",
            self.name,
            self.engine_handle,
            self.socket_path,
            self.interface,
            self.endpoint,
            drives,
            self.dynamics_profile,
        )
        for slot, (_global_axis, motor) in enumerate(motors):
            self._push_drive_params(motor, slot)
        reactor = self.printer.get_reactor()
        reactor.register_timer(
            self._poll_drive_fault,
            reactor.monotonic() + DRIVE_FAULT_POLL_PERIOD,
        )

    def _handle_shutdown(self):
        if self.engine_handle is None:
            return
        engine = self.printer.lookup_object("motion_engine")
        engine.stop_node(self.engine_handle)
        logging.info(
            "ethercat_node %s: servo motion discarded on shutdown (handle=%s)",
            self.name,
            self.engine_handle,
        )

    def _poll_drive_fault(self, eventtime):
        engine = self.printer.lookup_object("motion_engine")
        death = engine.take_endpoint_death(self.engine_handle)
        if death is not None:
            self.printer.invoke_shutdown(
                "EtherCAT endpoint died mid-session on node %s: %s"
                % (self.name, death)
            )
            return self.printer.get_reactor().NEVER
        fault = engine.take_drive_fault(self.engine_handle)
        if fault is None:
            return eventtime + DRIVE_FAULT_POLL_PERIOD
        self.printer.invoke_shutdown(
            "EtherCAT drive fault 0x%04x on node %s — drive parked by the"
            " realtime endpoint" % (fault, self.name)
        )
        return self.printer.get_reactor().NEVER

    def _push_drive_params(self, motor, slot):
        params = motor.get_sdo_params()
        if not params:
            return
        engine = self.printer.lookup_object("motion_engine")
        for index, subindex, size, value in params:
            try:
                engine.sdo_write(
                    self.engine_handle, slot, index, subindex, size, value
                )
            except RuntimeError as e:
                raise self.printer.config_error(
                    "ethercat_node %s: claim-time drive param "
                    "0x%04x.%d = %d (slot %d) failed: %s"
                    % (self.name, index, subindex, value, slot, e)
                )
            logging.info(
                "ethercat_node %s: drive param 0x%04x.%d = %d pushed (slot %d)",
                self.name,
                index,
                subindex,
                value,
                slot,
            )

    def get_engine_handle(self):
        return self.engine_handle

    def get_counts_per_mm(self):
        return self._counts_per_mm

    def get_cycle_us(self):
        return self.cycle_us

    def get_slot_for_motor(self, motor_name):
        return self._slot_by_motor.get(motor_name)

    def get_drive_count(self):
        return len(self._slot_by_motor)

    def get_dynamics_profile(self):
        return self.dynamics_profile

    def set_motor_torque(self, motor_name, value, print_time):
        if self.engine_handle is None:
            raise self.printer.command_error(
                "servo torque: ethercat_node %s has no engine handle"
                % (self.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        if value:
            first = not self._torque_motors
            self._torque_motors.add(motor_name)
            if first:
                return engine.set_torque_deferred(
                    self.engine_handle, True, print_time
                )
        else:
            self._torque_motors.discard(motor_name)
            if not self._torque_motors:
                engine.set_torque(self.engine_handle, False, print_time)
        return None


def load_config_prefix(config):
    return EtherCatNode(config)
