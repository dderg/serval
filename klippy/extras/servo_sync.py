from .. import engine_wait
from . import servo_axis

MOTION_DRAIN_TIMEOUT = 60.0
TORQUE_ACTUAL_INDEX = 0x6077


def _signed16(raw):
    return raw - 0x10000 if raw >= 0x8000 else raw


class SyncableAxis:
    def __init__(self, lane_idx, rail, node):
        self.lane_idx = lane_idx
        self.rail = rail
        self.node = node

    def axis_name(self):
        return self.rail.get_name(short=True)

    def slot_motors(self):
        return [
            (self.node.get_slot_for_motor(motor.get_motor_name()), motor)
            for motor in self.rail.get_motors()
        ]


class ServoSync:
    cmd_SERVO_SYNC_help = (
        "Release trapped belt strain the M84 way, XY only: torque off every "
        "belt drive at once, let the mechanics relax freely, torque back "
        "on. Each drive re-seeds at its actual position and the next move "
        "re-adopts the measured carriage position (exactly like after M84), "
        "so nothing is lost or rehomed. AXIS=X|Y releases one pair; "
        "TORQUE_OK sets the residual-fight error threshold; SETTLE "
        "overrides the relax time in seconds; RETRIES re-runs the release "
        "for still-fighting axes before erroring."
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        self.torque_ok_pct = config.getfloat("torque_ok", 3.0, above=0.0)
        self.settle_time = config.getfloat("settle_time", 1.0, above=0.0)
        self.retries = config.getint("retries", 0, minval=0)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_SYNC", self.cmd_SERVO_SYNC, desc=self.cmd_SERVO_SYNC_help
        )

    def _syncable_axes(self, gcmd, axis_filter):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        found = []
        for lane_idx, axis_name, _motor_names in kin.lanes():
            rail = kin.rails[lane_idx]
            if not isinstance(rail, servo_axis.ServoRail):
                continue
            if len(rail.get_motors()) != 2:
                continue
            if rail.get_name(short=True) == "z":
                continue
            if (
                axis_filter is not None
                and rail.get_name(short=True) != axis_filter
            ):
                continue
            node = self.printer.lookup_object(
                "ethercat_node " + rail.get_node_name()
            )
            found.append(SyncableAxis(lane_idx, rail, node))
        if not found:
            if axis_filter == "z":
                raise gcmd.error(
                    "SERVO_SYNC: Z is not supported — inter-screw discrepancy"
                    " is gantry racking, handled by leveling, and a coasting"
                    " Z drive is not gravity-safe"
                )
            raise gcmd.error(
                "SERVO_SYNC: no belt axis with exactly two servo drives%s"
                % (
                    ""
                    if axis_filter is None
                    else " matching AXIS=%s" % axis_filter.upper()
                )
            )
        return found

    def _drain_motion(self, gcmd, engine):
        try:
            engine_wait.wait_for(
                self.printer,
                lambda: engine.motion_drained() or None,
                "SERVO_SYNC motion drain",
                MOTION_DRAIN_TIMEOUT,
            )
        except engine_wait.EngineWaitTimeout:
            raise gcmd.error(
                "SERVO_SYNC: motion did not drain within %.0fs"
                % MOTION_DRAIN_TIMEOUT
            )

    def _node_handle(self, gcmd, node):
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "SERVO_SYNC: ethercat_node %s has no engine handle" % node.name
            )
        return handle

    def _read_torques(self, engine, handle, entries):
        torques = {}
        for entry in entries:
            for slot, _motor in entry.slot_motors():
                _size, raw = engine.sdo_read(
                    handle, slot, TORQUE_ACTUAL_INDEX, 0
                )
                torques[slot] = _signed16(raw)
        return torques

    def _describe(self, entry, baseline, final):
        parts = []
        for slot, motor in entry.slot_motors():
            parts.append(
                "%s %+.1f%% -> %+.1f%%"
                % (
                    motor.get_motor_name(),
                    baseline[slot] / 10.0,
                    final[slot] / 10.0,
                )
            )
        return "axis %s released: %s" % (entry.axis_name(), "; ".join(parts))

    def _release(self, gcmd, toolhead, engine, by_node, settle, torque_ok_pct):
        residual_entries = []
        residual_motors = []
        for node, entries in by_node.values():
            handle = self._node_handle(gcmd, node)
            baseline = self._read_torques(engine, handle, entries)
            print_time = toolhead.get_last_move_time()
            for entry in entries:
                node.set_motor_torque(entry.rail.get_name(), False, print_time)
            # The disable executes at print_time on the MCU clock; a wall
            # clock pause (or a drain-based wait) can finish before it ever
            # fires, and the re-enable then CANCELS the pending disable — no
            # release at all. Waiting on the same clock guarantees the
            # disable executed and the mechanics got the full relax window.
            toolhead.wait_until_print_time(print_time + settle)
            print_time = toolhead.get_last_move_time()
            waiters = [
                node.set_motor_torque(entry.rail.get_name(), True, print_time)
                for entry in entries
            ]
            for waiter in waiters:
                if waiter is not None:
                    waiter()
            final = self._read_torques(engine, handle, entries)
            for entry in entries:
                gcmd.respond_info(self._describe(entry, baseline, final))
                fighting = [
                    motor.get_motor_name()
                    for slot, motor in entry.slot_motors()
                    if abs(final[slot]) > torque_ok_pct * 10.0
                ]
                if fighting:
                    residual_entries.append(entry)
                    residual_motors.extend(fighting)
        return residual_entries, residual_motors

    def run(
        self,
        gcmd,
        axis_filter=None,
        torque_ok_pct=None,
        settle=None,
        retries=None,
    ):
        if torque_ok_pct is None:
            torque_ok_pct = self.torque_ok_pct
        if settle is None:
            settle = self.settle_time
        if retries is None:
            retries = self.retries
        entries = self._syncable_axes(gcmd, axis_filter)
        toolhead = self.printer.lookup_object("toolhead")
        engine = self.printer.lookup_object("motion_engine")
        toolhead.wait_moves()
        self._drain_motion(gcmd, engine)
        residual = []
        for attempt in range(retries + 1):
            by_node = {}
            for entry in entries:
                by_node.setdefault(id(entry.node), (entry.node, []))[1].append(
                    entry
                )
            entries, residual = self._release(
                gcmd, toolhead, engine, by_node, settle, torque_ok_pct
            )
            if not residual:
                break
            if attempt < retries:
                gcmd.respond_info(
                    "SERVO_SYNC: %s still fighting, retrying release (%d/%d)"
                    % (", ".join(residual), attempt + 1, retries)
                )
        toolhead.get_kinematics().mark_servo_parked((0, 1))
        if residual:
            raise gcmd.error(
                "SERVO_SYNC: %s still fighting after release%s — "
                "did the torque cycle execute? (mechanical binding?)"
                % (
                    ", ".join(residual),
                    " and %d retries" % retries if retries else "",
                )
            )

    def cmd_SERVO_SYNC(self, gcmd):
        axis_filter = gcmd.get("AXIS", None)
        if axis_filter is not None:
            axis_filter = axis_filter.lower()
        self.run(
            gcmd,
            axis_filter,
            torque_ok_pct=gcmd.get_float("TORQUE_OK", self.torque_ok_pct),
            settle=gcmd.get_float("SETTLE", self.settle_time, above=0.0),
            retries=gcmd.get_int("RETRIES", self.retries, minval=0),
        )


def load_config(config):
    return ServoSync(config)
