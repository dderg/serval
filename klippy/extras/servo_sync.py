from .. import engine_wait
from . import servo_axis

MOTION_DRAIN_TIMEOUT = 60.0

SYNC_ERROR_TEXT = {
    -840: "another sync is already running on the node",
    -841: "servo torque is not enabled",
    -842: "motion ring is not empty",
    -843: "axis is not driven by exactly two servos on one node",
    -844: "coasting drive never settled (settle timeout)",
    -845: "torque stayed high and the coasting rotor never released any "
    "strain — mechanical binding?",
    -846: "pair still fighting after re-enable",
    -847: "motion arrived during sync",
    -848: "dither parameters rejected",
    -849: "sync aborted by stop or drive fault",
}


class SyncableAxis:
    def __init__(self, lane_idx, rail, node):
        self.lane_idx = lane_idx
        self.rail = rail
        self.node = node

    def axis_name(self):
        return self.rail.get_name(short=True)

    def motor_for_slot(self, slot):
        for motor in self.rail.get_motors():
            if self.node.get_slot_for_motor(motor.get_motor_name()) == slot:
                return motor
        raise KeyError(slot)


class ServoSync:
    cmd_SERVO_SYNC_help = (
        "Release the strain between the paired servo drives of each belt "
        "axis: coast the secondary, dither the primary through stiction, "
        "re-seed the secondary at its settled position. Standstill torque "
        "is verified at every phase. AXIS=X|Y limits to one axis; "
        "TORQUE_OK/AMPLITUDE/FREQ/DURATION override the config tuning."
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        self.torque_ok_pct = config.getfloat(
            "torque_ok", 3.0, above=0.0, maxval=40.0
        )
        self.settle_timeout = config.getfloat(
            "settle_timeout", 2.0, above=0.0, maxval=60.0
        )
        self.dither_amplitude = config.getfloat(
            "dither_amplitude", 0.1, above=0.0, maxval=1.0
        )
        self.dither_frequency = config.getfloat(
            "dither_frequency", 4.0, above=0.0, maxval=50.0
        )
        self.dither_duration = config.getfloat(
            "dither_duration", 0.5, above=0.0, maxval=5.0
        )
        self.max_rounds = config.getint("max_rounds", 2, minval=1, maxval=5)
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

    def _run_pair(self, gcmd, engine, entry, tuning):
        handle = entry.node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "SERVO_SYNC: ethercat_node %s has no engine handle"
                % entry.node.name
            )
        return engine.sync_servo_pair(
            handle,
            entry.lane_idx,
            tuning["torque_ok_tenth_pct"],
            tuning["settle_timeout_ms"],
            tuning["dither_amplitude_nm"],
            tuning["dither_freq_millihz"],
            tuning["dither_duration_ms"],
        )

    def _describe(self, entry, report):
        (
            result,
            primary_slot,
            secondary_slot,
            baseline_p,
            baseline_s,
            released,
            dithered,
            final_p,
            final_s,
            delta_counts,
        ) = report
        primary = entry.motor_for_slot(primary_slot)
        secondary = entry.motor_for_slot(secondary_slot)
        delta_mm = delta_counts / secondary.get_counts_per_mm()
        text = (
            "axis %s (%s holds, %s re-seeded): fight (%+.1f%%, %+.1f%%) -> "
            "coast %+.1f%% -> dither %+.1f%% -> final (%+.1f%%, %+.1f%%); "
            "released %d counts (%.4f mm)"
            % (
                entry.axis_name(),
                primary.get_motor_name(),
                secondary.get_motor_name(),
                baseline_p / 10.0,
                baseline_s / 10.0,
                released / 10.0,
                dithered / 10.0,
                final_p / 10.0,
                final_s / 10.0,
                delta_counts,
                delta_mm,
            )
        )
        if result != 0:
            text += " — FAILED: %s (code %d)" % (
                SYNC_ERROR_TEXT.get(result, "unknown error"),
                result,
            )
        return text

    def cmd_SERVO_SYNC(self, gcmd):
        axis_filter = gcmd.get("AXIS", None)
        if axis_filter is not None:
            axis_filter = axis_filter.lower()
        tuning = {
            "torque_ok_tenth_pct": int(
                round(gcmd.get_float("TORQUE_OK", self.torque_ok_pct) * 10.0)
            ),
            "settle_timeout_ms": int(round(self.settle_timeout * 1000.0)),
            "dither_amplitude_nm": int(
                round(
                    gcmd.get_float("AMPLITUDE", self.dither_amplitude)
                    * 1_000_000.0
                )
            ),
            "dither_freq_millihz": int(
                round(gcmd.get_float("FREQ", self.dither_frequency) * 1000.0)
            ),
            "dither_duration_ms": int(
                round(gcmd.get_float("DURATION", self.dither_duration) * 1000.0)
            ),
        }
        axes = self._syncable_axes(gcmd, axis_filter)
        toolhead = self.printer.lookup_object("toolhead")
        engine = self.printer.lookup_object("motion_engine")
        toolhead.wait_moves()
        self._drain_motion(gcmd, engine)
        for round_idx in range(self.max_rounds):
            reports = [
                (entry, self._run_pair(gcmd, engine, entry, tuning))
                for entry in axes
            ]
            for entry, report in reports:
                gcmd.respond_info(self._describe(entry, report))
            results = [report[0] for _entry, report in reports]
            if all(r == 0 for r in results):
                return
            # A later pair's dither can push a small residual into an
            # earlier pair; only that cross-coupling failure mode earns
            # another round. Anything else is a hard error now.
            if any(r != -846 for r in results if r != 0):
                raise gcmd.error("SERVO_SYNC failed — see measurements above")
            if round_idx + 1 >= self.max_rounds:
                raise gcmd.error(
                    "SERVO_SYNC: pairs still fighting after %d rounds"
                    % self.max_rounds
                )
            gcmd.respond_info(
                "SERVO_SYNC: residual fight after round %d, running another"
                " round" % (round_idx + 1)
            )


def load_config(config):
    return ServoSync(config)
