from .. import engine_wait
from . import servo_axis

MOTION_DRAIN_TIMEOUT = 60.0

SYNC_ERROR_TEXT = {
    -840: "another sync is already running on the node",
    -841: "servo torque is not enabled",
    -842: "motion ring is not empty",
    -843: "release mask does not select valid drive slots",
    -844: "a coasting drive never settled (settle timeout)",
    -846: "drives still fighting after re-enable",
    -847: "motion arrived during sync",
    -849: "sync aborted by stop or drive fault",
}


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
        "Release the strain the belt drives have built up against each "
        "other: de-energize every belt drive at once, let the mechanics "
        "relax freely, re-energize (each drive re-seeds at its settled "
        "position, so no position is lost). Standstill torque is verified "
        "before and after. AXIS=X|Y releases just one pair; TORQUE_OK "
        "overrides the config threshold."
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        self.torque_ok_pct = config.getfloat(
            "torque_ok", 3.0, above=0.0, maxval=40.0
        )
        self.settle_timeout = config.getfloat(
            "settle_timeout", 2.0, above=0.0, maxval=60.0
        )
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

    def _describe(self, entry, report):
        result, _slot_mask, baseline, final, released = report
        parts = []
        for slot, motor in entry.slot_motors():
            parts.append(
                "%s %+.1f%% -> %+.1f%% (rotor moved %+.4f mm)"
                % (
                    motor.get_motor_name(),
                    baseline[slot] / 10.0,
                    final[slot] / 10.0,
                    released[slot] / motor.get_counts_per_mm(),
                )
            )
        text = "axis %s released: %s" % (entry.axis_name(), "; ".join(parts))
        if result != 0:
            text += " — FAILED: %s (code %d)" % (
                SYNC_ERROR_TEXT.get(result, "unknown error"),
                result,
            )
        return text

    def default_tuning(self):
        return {
            "torque_ok_tenth_pct": int(round(self.torque_ok_pct * 10.0)),
            "settle_timeout_ms": int(round(self.settle_timeout * 1000.0)),
        }

    def run(self, gcmd, axis_filter=None, tuning=None):
        if tuning is None:
            tuning = self.default_tuning()
        axes = self._syncable_axes(gcmd, axis_filter)
        toolhead = self.printer.lookup_object("toolhead")
        engine = self.printer.lookup_object("motion_engine")
        toolhead.wait_moves()
        self._drain_motion(gcmd, engine)
        by_node = {}
        for entry in axes:
            by_node.setdefault(id(entry.node), (entry.node, []))[1].append(
                entry
            )
        failed = False
        for node, entries in by_node.values():
            handle = node.get_engine_handle()
            if handle is None:
                raise gcmd.error(
                    "SERVO_SYNC: ethercat_node %s has no engine handle"
                    % node.name
                )
            slot_mask = 0
            for entry in entries:
                for slot, _motor in entry.slot_motors():
                    slot_mask |= 1 << slot
            report = engine.sync_servo_release(
                handle,
                slot_mask,
                tuning["torque_ok_tenth_pct"],
                tuning["settle_timeout_ms"],
            )
            for entry in entries:
                gcmd.respond_info(self._describe(entry, report))
            failed = failed or report[0] != 0
        if failed:
            raise gcmd.error("SERVO_SYNC failed — see measurements above")

    def cmd_SERVO_SYNC(self, gcmd):
        axis_filter = gcmd.get("AXIS", None)
        if axis_filter is not None:
            axis_filter = axis_filter.lower()
        tuning = {
            "torque_ok_tenth_pct": int(
                round(gcmd.get_float("TORQUE_OK", self.torque_ok_pct) * 10.0)
            ),
            "settle_timeout_ms": int(round(self.settle_timeout * 1000.0)),
        }
        self.run(gcmd, axis_filter, tuning)


def load_config(config):
    return ServoSync(config)
