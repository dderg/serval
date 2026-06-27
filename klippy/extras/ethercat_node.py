import logging
import os

from . import servo_axis

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
        self.engine_handle = None
        # Derived at claim time, not __init__: the [servo_*] sections are parsed
        # by the toolhead AFTER [ethercat_node] sections (printer._read_config
        # loads prefix sections before motion), so the matching
        # ServoRail does not exist yet here.
        self._counts_per_mm = None
        self._slot_by_motor = {}
        # Claim during mcu-identify. printer._connect sends
        # "klippy:mcu_identify" before invoking the "klippy:connect"
        # handlers (klippy/printer.py), and motion._init_planner
        # runs on "klippy:connect" — so the handle is populated before the
        # planner is built. This mirrors MCU._mcu_identify's claim_mcu call.
        self.printer.register_event_handler("klippy:mcu_identify", self._claim)
        self.printer.load_object(config, "servo_capture")
        self.printer.load_object(config, "servo_param")

    def _find_rails(self):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        found = []
        for lane_idx, _axis_name, _motor_names in kin.lanes():
            rail = kin.rails[lane_idx]
            if (
                isinstance(rail, servo_axis.ServoRail)
                and rail.get_node_name() == self.name
            ):
                found.append((lane_idx, rail))
        if not found:
            raise self.printer.config_error(
                "ethercat_node %s: no [servo_*] section with node=%s — "
                "cannot locate the servo rail" % (self.name, self.name)
            )
        return found

    def _validate_chain(self, rails):
        by_index = {}
        for _global_axis, rail in rails:
            idx = rail.get_chain_index()
            if idx >= EC_RT_MAX_SLAVES:
                raise self.printer.config_error(
                    "ethercat_node %s: motor %s ethercat_chain_index=%d "
                    "exceeds the %d-drive endpoint limit (valid 0..%d)"
                    % (
                        self.name,
                        rail.get_motor_name(),
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
                        rail.get_motor_name(),
                        idx,
                    )
                )
            by_index[idx] = rail.get_motor_name()
        ff_configs = {rail.get_ff_config() for _global_axis, rail in rails}
        if len(ff_configs) != 1:
            raise self.printer.config_error(
                "ethercat_node %s: drives disagree on velocity_ff / "
                "dynamics_profile / ff_torque_clamp — these are node-wide and "
                "must match across all motors on the chain" % (self.name,)
            )

    def _claim(self):
        if self.engine_handle is not None:
            return
        rails = sorted(self._find_rails(), key=lambda pair: pair[0])
        self._validate_chain(rails)
        self._slot_by_motor = {
            rail.get_motor_name(): slot
            for slot, (_global_axis, rail) in enumerate(rails)
        }
        drives = []
        for global_axis, rail in rails:
            following_error_counts, max_torque_tenth_pct = (
                rail.get_session_drive_limits()
            )
            drives.append(
                (
                    rail.get_chain_index(),
                    global_axis,
                    rail.get_counts_per_mm(),
                    rail.get_rotation_distance(),
                    following_error_counts,
                    max_torque_tenth_pct,
                )
            )
        velocity_ff, dynamics_profile, ff_torque_clamp = rails[0][
            1
        ].get_ff_config()
        self._counts_per_mm = rails[0][1].get_counts_per_mm()
        engine = self.printer.lookup_object("motion_engine")
        try:
            self.engine_handle = engine.claim_ethercat_node(
                self.name,
                self.socket_path,
                self.interface,
                self.endpoint,
                velocity_ff,
                dynamics_profile,
                ff_torque_clamp,
                drives,
            )
        except RuntimeError as e:
            raise self.printer.config_error(str(e))
        logging.info(
            "ethercat_node %s: claimed handle=%s socket=%s interface=%s "
            "endpoint=%s drives=%s velocity_ff=%s dynamics_profile=%s "
            "ff_torque_clamp=%s",
            self.name,
            self.engine_handle,
            self.socket_path,
            self.interface,
            self.endpoint,
            drives,
            velocity_ff,
            dynamics_profile,
            ff_torque_clamp,
        )
        for slot, (_global_axis, rail) in enumerate(rails):
            self._push_drive_params(rail, slot)
        reactor = self.printer.get_reactor()
        reactor.register_timer(
            self._poll_drive_fault,
            reactor.monotonic() + DRIVE_FAULT_POLL_PERIOD,
        )

    def _poll_drive_fault(self, eventtime):
        engine = self.printer.lookup_object("motion_engine")
        fault = engine.take_drive_fault(self.engine_handle)
        if fault is None:
            return eventtime + DRIVE_FAULT_POLL_PERIOD
        self.printer.invoke_shutdown(
            "EtherCAT drive fault 0x%04x on node %s — drive parked by the"
            " realtime endpoint" % (fault, self.name)
        )
        return self.printer.get_reactor().NEVER

    def _push_drive_params(self, rail, slot):
        params = rail.get_sdo_params()
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

    def get_slot_for_motor(self, motor_name):
        return self._slot_by_motor.get(motor_name)

    def get_drive_count(self):
        return len(self._slot_by_motor)


def load_config_prefix(config):
    return EtherCatNode(config)
