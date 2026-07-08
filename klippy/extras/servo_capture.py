import os
import re
import time

CAPTURE_DIR = "~/printer_data/logs/servo_captures"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class ServoCapture:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.capture_dir = os.path.expanduser(CAPTURE_DIR)
        self.active = None  # (motor_name, node, path) while running
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_CAPTURE_START",
            self.cmd_SERVO_CAPTURE_START,
            desc=self.cmd_SERVO_CAPTURE_START_help,
        )
        gcode.register_command(
            "SERVO_CAPTURE_STOP",
            self.cmd_SERVO_CAPTURE_STOP,
            desc=self.cmd_SERVO_CAPTURE_STOP_help,
        )

    def _resolve_node(self, gcmd):
        from . import servo_axis

        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        servo_motors = list(servo_axis.iter_servo_motors(kin))
        if not servo_motors:
            raise gcmd.error("SERVO_CAPTURE: no servo motors configured")
        motors = self._select_motors(gcmd, servo_motors)
        node_names = sorted({m.get_node_name() for m in motors})
        if len(node_names) != 1:
            raise gcmd.error(
                "SERVO_CAPTURE: selected servos span multiple EtherCAT "
                "nodes (%s); a capture records one node"
                % (", ".join(node_names),)
            )
        node = self.printer.lookup_object("ethercat_node " + node_names[0])
        drives = [
            (node.get_slot_for_motor(m.get_motor_name()), m.get_motor_name())
            for m in motors
        ]
        return node, drives

    def _select_motors(self, gcmd, servo_motors):
        axis = gcmd.get("AXIS", None)
        servo = gcmd.get("SERVO", None)
        if axis is not None and servo is not None:
            raise gcmd.error("SERVO_CAPTURE: pass AXIS= or SERVO=, not both")
        known = ", ".join(m.get_motor_name() for _r, m in servo_motors)
        if axis is not None:
            on_axis = [
                m
                for rail, m in servo_motors
                if rail.get_name(short=True) == axis.lower()
            ]
            if not on_axis:
                axes = ", ".join(
                    sorted({r.get_name(short=True) for r, _m in servo_motors})
                )
                raise gcmd.error(
                    "SERVO_CAPTURE: no servo on axis %r (have: %s)"
                    % (axis, axes)
                )
            return on_axis
        if servo is not None:
            from . import servo_axis

            motors = []
            for token in (s.strip() for s in servo.split(",")):
                _rail, motor = servo_axis.resolve_servo_motor(
                    self.printer, token, "SERVO_CAPTURE"
                )
                if motor in motors:
                    raise gcmd.error(
                        "SERVO_CAPTURE: servo %r listed twice" % (token,)
                    )
                motors.append(motor)
            return motors
        if len(servo_motors) != 1:
            raise gcmd.error(
                "SERVO_CAPTURE: multiple servo motors configured (%s); "
                "AXIS= or SERVO= is required" % (known,)
            )
        return [servo_motors[0][1]]

    cmd_SERVO_CAPTURE_START_help = (
        "Start a servo telemetry capture at the node's DC sync rate. Target "
        "the drive with AXIS= or SERVO= (motor name; comma list captures "
        "several drives on one node). Wrap test moves and finish with M400 "
        "before SERVO_CAPTURE_STOP."
    )

    def cmd_SERVO_CAPTURE_START(self, gcmd):
        if self.active is not None:
            raise gcmd.error(
                "SERVO_CAPTURE: capture already active (%s)" % (self.active[2],)
            )
        tag = gcmd.get("NAME", "capture")
        if not NAME_RE.fullmatch(tag):
            raise gcmd.error(
                "SERVO_CAPTURE: NAME must match [A-Za-z0-9_-]+, got %r" % (tag,)
            )
        node, drives = self._resolve_node(gcmd)
        label = ",".join(name for _, name in drives)
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "SERVO_CAPTURE: servo %r has no engine handle (node not "
                "claimed)" % (label,)
            )
        path = os.path.join(
            self.capture_dir,
            "%s_%s.scap" % (tag, time.strftime("%Y%m%d_%H%M%S")),
        )
        started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        engine = self.printer.lookup_object("motion_engine")
        try:
            engine.start_servo_capture(handle, path, started_utc, drives)
        except RuntimeError as e:
            raise gcmd.error("SERVO_CAPTURE: start failed: %s" % (e,))
        self.active = (label, node, path)
        gcmd.respond_info("Servo capture started: %s" % (path,))

    cmd_SERVO_CAPTURE_STOP_help = "Stop the active servo telemetry capture."

    def cmd_SERVO_CAPTURE_STOP(self, gcmd):
        if self.active is None:
            raise gcmd.error("SERVO_CAPTURE: no capture active")
        motor_name, node, path = self.active
        self.active = None
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "SERVO_CAPTURE: servo %r vanished mid-capture" % (motor_name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        result, samples, overflow_cycle = engine.stop_servo_capture(handle)
        if result != 0:
            failed = os.path.splitext(path)[0] + ".failed.scap"
            raise gcmd.error(
                "Servo capture FAILED (endpoint code %d, overflow_cycle=%s); "
                "partial data in %s" % (result, overflow_cycle, failed)
            )
        cycle_us = node.get_cycle_us()
        sample_rate_hz = 1_000_000.0 / cycle_us
        gcmd.respond_info(
            "Servo capture stopped: %s\n"
            "samples=%d (%.2f s at the %.1f kHz DC cycle)"
            % (
                path,
                samples,
                samples * cycle_us / 1_000_000.0,
                sample_rate_hz / 1000.0,
            )
        )


def load_config(config):
    return ServoCapture(config)
