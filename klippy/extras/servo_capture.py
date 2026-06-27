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

        servo = gcmd.get("SERVO", None)
        toolhead = self.printer.lookup_object("toolhead")
        servo_rails = [
            rail
            for rail in getattr(toolhead.get_kinematics(), "rails", ())
            if isinstance(rail, servo_axis.ServoRail)
        ]
        if not servo_rails:
            raise gcmd.error("SERVO_CAPTURE: no servo motors configured")
        known = ", ".join(rail.get_motor_name() for rail in servo_rails)
        if servo is None:
            if len(servo_rails) != 1:
                raise gcmd.error(
                    "SERVO_CAPTURE: multiple servo motors configured (%s); "
                    "SERVO= is required" % (known,)
                )
            rail = servo_rails[0]
        else:
            rail = next(
                (
                    r
                    for r in servo_rails
                    if servo
                    in (
                        r.get_motor_name(),
                        r.get_name(),
                        r.get_name(short=True),
                    )
                ),
                None,
            )
            if rail is None:
                raise gcmd.error(
                    "SERVO_CAPTURE: no servo motor named %r (known: %s)"
                    % (servo, known)
                )
        motor_name = rail.get_motor_name()
        node = self.printer.lookup_object(
            "ethercat_node " + rail.get_node_name()
        )
        return node, node.get_slot_for_motor(motor_name), motor_name

    cmd_SERVO_CAPTURE_START_help = (
        "Start a servo telemetry capture (1 kHz). Wrap test moves and finish "
        "with M400 before SERVO_CAPTURE_STOP."
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
        node, slot, motor_name = self._resolve_node(gcmd)
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "SERVO_CAPTURE: servo %r has no engine handle (node not "
                "claimed)" % (motor_name,)
            )
        path = os.path.join(
            self.capture_dir,
            "%s_%s.scap" % (tag, time.strftime("%Y%m%d_%H%M%S")),
        )
        started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        drives = [(slot, motor_name)]
        engine = self.printer.lookup_object("motion_engine")
        try:
            engine.start_servo_capture(handle, path, started_utc, drives)
        except RuntimeError as e:
            raise gcmd.error("SERVO_CAPTURE: start failed: %s" % (e,))
        self.active = (motor_name, node, path)
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
        gcmd.respond_info(
            "Servo capture stopped: %s\n"
            "samples=%d (%.2f s at the 1 kHz DC cycle)"
            % (path, samples, samples / 1000.0)
        )


def load_config(config):
    return ServoCapture(config)
