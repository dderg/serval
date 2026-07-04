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
        servo_rails = [
            rail
            for rail in getattr(toolhead.get_kinematics(), "rails", ())
            if isinstance(rail, servo_axis.ServoRail)
        ]
        if not servo_rails:
            raise gcmd.error("SERVO_CAPTURE: no servo motors configured")
        rails = self._select_rails(gcmd, servo_rails)
        node_names = sorted({r.get_node_name() for r in rails})
        if len(node_names) != 1:
            raise gcmd.error(
                "SERVO_CAPTURE: selected servos span multiple EtherCAT "
                "nodes (%s); a capture records one node"
                % (", ".join(node_names),)
            )
        node = self.printer.lookup_object("ethercat_node " + node_names[0])
        drives = [
            (node.get_slot_for_motor(r.get_motor_name()), r.get_motor_name())
            for r in rails
        ]
        return node, drives

    def _select_rails(self, gcmd, servo_rails):
        axis = gcmd.get("AXIS", None)
        servo = gcmd.get("SERVO", None)
        if axis is not None and servo is not None:
            raise gcmd.error("SERVO_CAPTURE: pass AXIS= or SERVO=, not both")
        known = ", ".join(rail.get_motor_name() for rail in servo_rails)
        if axis is not None:
            on_axis = [
                r for r in servo_rails if r.get_name(short=True) == axis.lower()
            ]
            if not on_axis:
                axes = ", ".join(r.get_name(short=True) for r in servo_rails)
                raise gcmd.error(
                    "SERVO_CAPTURE: no servo on axis %r (have: %s)"
                    % (axis, axes)
                )
            if len(on_axis) > 1:
                raise gcmd.error(
                    "SERVO_CAPTURE: axis %r drives multiple servos (%s); "
                    "SERVO= is required" % (axis, known)
                )
            return on_axis
        if servo is not None:
            rails = []
            for token in (s.strip() for s in servo.split(",")):
                rail = next(
                    (
                        r
                        for r in servo_rails
                        if token
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
                        % (token, known)
                    )
                if rail in rails:
                    raise gcmd.error(
                        "SERVO_CAPTURE: servo %r listed twice" % (token,)
                    )
                rails.append(rail)
            return rails
        if len(servo_rails) != 1:
            raise gcmd.error(
                "SERVO_CAPTURE: multiple servo motors configured (%s); "
                "AXIS= or SERVO= is required" % (known,)
            )
        return servo_rails

    cmd_SERVO_CAPTURE_START_help = (
        "Start a servo telemetry capture (1 kHz). Target the drive with AXIS= "
        "or SERVO= (motor name; comma list captures several drives on one "
        "node). Wrap test moves and finish with M400 before "
        "SERVO_CAPTURE_STOP."
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
        gcmd.respond_info(
            "Servo capture stopped: %s\n"
            "samples=%d (%.2f s at the 1 kHz DC cycle)"
            % (path, samples, samples / 1000.0)
        )


def load_config(config):
    return ServoCapture(config)
