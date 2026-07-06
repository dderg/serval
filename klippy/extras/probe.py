import math

from klippy import pins
from klippy.motion_endstop import MotionEndstop, allocate_provider_id

from . import manual_probe

Z_AXIS = 2
ACCURACY_DEFAULT_SAMPLES = 10
NO_MOVEMENT_EPSILON = 0.005


def calc_probe_z_result(values, method):
    if method == "median":
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0
    if method != "average":
        raise ValueError("unknown samples_result '%s'" % (method,))
    return sum(values) / len(values)


def validate_virtual_endstop_request(pin_params, axis):
    if pin_params["pin"] != "z_virtual_endstop":
        raise pins.error(
            "probe only provides the virtual pin 'z_virtual_endstop',"
            " not '%s'" % (pin_params["pin"],)
        )
    if pin_params["invert"] or pin_params["pullup"]:
        raise pins.error("Can not pullup/invert probe virtual endstop")
    if axis != Z_AXIS:
        raise pins.error(
            "probe:z_virtual_endstop is only usable as the Z endstop"
        )


class PrinterProbe:
    cmd_PROBE_help = "Probe Z-height at the current XY position"
    cmd_QUERY_PROBE_help = "Return the current probe state"
    cmd_PROBE_ACCURACY_help = "Probe Z-height repeatedly and report statistics"
    cmd_Z_OFFSET_APPLY_PROBE_help = "Adjust the probe's z_offset"

    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object("pins")
        pin_desc = config.get("pin")
        pin_params = ppins.lookup_pin(
            pin_desc, can_invert=True, can_pullup=True
        )
        if not hasattr(pin_params["chip"], "create_oid"):
            raise config.error(
                "[probe] pin must be a GPIO pin on an MCU, not '%s'"
                % (pin_desc,)
            )
        self._endstop = MotionEndstop(
            pin_params, allocate_provider_id(self.printer)
        )

        self.z_offset = config.getfloat("z_offset")
        self.x_offset = config.getfloat("x_offset", 0.0)
        self.y_offset = config.getfloat("y_offset", 0.0)
        self.speed = config.getfloat("speed", 5.0, above=0.0)
        self.lift_speed = config.getfloat("lift_speed", self.speed, above=0.0)
        self.samples = config.getint("samples", 1, minval=1)
        self.sample_retract_dist = config.getfloat(
            "sample_retract_dist", 2.0, above=0.0
        )
        self.samples_result = config.getchoice(
            "samples_result", ["median", "average"], "average"
        )
        self.samples_tolerance = config.getfloat(
            "samples_tolerance", 0.100, minval=0.0
        )
        self.samples_retries = config.getint(
            "samples_tolerance_retries", 0, minval=0
        )

        self.last_query = False
        self.last_z_result = 0.0

        self.name = config.get_name()
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        ppins.register_chip("probe", self)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "PROBE", self.cmd_PROBE, desc=self.cmd_PROBE_help
        )
        gcode.register_command(
            "QUERY_PROBE", self.cmd_QUERY_PROBE, desc=self.cmd_QUERY_PROBE_help
        )
        gcode.register_command(
            "PROBE_ACCURACY",
            self.cmd_PROBE_ACCURACY,
            desc=self.cmd_PROBE_ACCURACY_help,
        )
        gcode.register_command(
            "Z_OFFSET_APPLY_PROBE",
            self.cmd_Z_OFFSET_APPLY_PROBE,
            desc=self.cmd_Z_OFFSET_APPLY_PROBE_help,
        )
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(self._endstop, "probe")

    def setup_motion_endstop(self, pin_params, axis):
        validate_virtual_endstop_request(pin_params, axis)
        return self._endstop

    def get_position_endstop(self):
        return self.z_offset

    def get_offsets(self):
        return self.x_offset, self.y_offset, self.z_offset

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        return self.lift_speed

    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass

    def get_status(self, eventtime):
        return {
            "name": "probe",
            "last_query": self.last_query,
            "last_z_result": self.last_z_result,
        }

    def _check_homed(self, gcmd, toolhead):
        curtime = self.printer.get_reactor().monotonic()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        if "z" not in kin_status["homed_axes"]:
            raise gcmd.error("Must home before probe")

    def _probe_once(self, gcmd, toolhead, homing_obj, engine, speed):
        kin = toolhead.get_kinematics()
        rail = kin._axis_rails().get(Z_AXIS)
        if rail is None:
            raise gcmd.error("PROBE: no Z rail configured")
        pos_min = rail.get_range()[0]
        current_z = toolhead.get_position()[Z_AXIS]
        max_travel = current_z - pos_min
        if max_travel <= 0.0:
            raise gcmd.error("PROBE: toolhead already at or below position_min")
        trip_pos, final_pos = homing_obj.trip_move(
            gcmd,
            toolhead,
            engine,
            Z_AXIS,
            -1.0,
            speed,
            max_travel,
            {
                "endstop": self._endstop,
                "provider": self,
                "trigger_position": None,
            },
        )
        if abs(trip_pos[Z_AXIS] - current_z) < NO_MOVEMENT_EPSILON:
            raise gcmd.error(
                "Probe triggered prior to movement — probe is already in"
                " contact or the trigger is stuck"
            )
        newpos = list(toolhead.get_position())
        newpos[Z_AXIS] = final_pos[Z_AXIS]
        toolhead.set_position(newpos)
        return trip_pos[Z_AXIS]

    def _retract(self, toolhead, target_z, lift_speed):
        newpos = list(toolhead.get_position())
        newpos[Z_AXIS] = target_z
        toolhead.move(newpos, lift_speed)
        toolhead.wait_moves()

    def run_probe(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        homing_obj = self.printer.lookup_object("homing")
        engine = self.printer.lookup_object("motion_engine")
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        sample_count = gcmd.get_int("SAMPLES", self.samples, minval=1)
        retract = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        tolerance = gcmd.get_float(
            "SAMPLES_TOLERANCE", self.samples_tolerance, minval=0.0
        )
        max_retries = gcmd.get_int(
            "SAMPLES_TOLERANCE_RETRIES", self.samples_retries, minval=0
        )
        method = gcmd.get("SAMPLES_RESULT", self.samples_result)
        if method not in ("median", "average"):
            raise gcmd.error("SAMPLES_RESULT must be median or average")
        self._check_homed(gcmd, toolhead)
        retries = 0
        measured = []
        while True:
            z = self._probe_once(gcmd, toolhead, homing_obj, engine, speed)
            measured.append(z)
            if max(measured) - min(measured) > tolerance:
                if retries >= max_retries:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                measured = []
            self._retract(toolhead, z + retract, lift_speed)
            if len(measured) >= sample_count:
                break
        epos = list(toolhead.get_position()[:3])
        epos[Z_AXIS] = calc_probe_z_result(measured, method)
        return epos

    def cmd_PROBE(self, gcmd):
        pos = self.run_probe(gcmd)
        gcmd.respond_info(
            "probe at %.3f,%.3f is z=%.6f" % (pos[0], pos[1], pos[2])
        )
        self.last_z_result = pos[2]

    def cmd_QUERY_PROBE(self, gcmd):
        triggered = self._endstop.is_triggered()
        self.last_query = triggered
        gcmd.respond_info("probe: %s" % ("TRIGGERED" if triggered else "open"))

    def cmd_PROBE_ACCURACY(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        homing_obj = self.printer.lookup_object("homing")
        engine = self.printer.lookup_object("motion_engine")
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        sample_count = gcmd.get_int(
            "SAMPLES", ACCURACY_DEFAULT_SAMPLES, minval=1
        )
        retract = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        self._check_homed(gcmd, toolhead)
        pos = toolhead.get_position()
        gcmd.respond_info(
            "PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
            " (samples=%d retract=%.3f speed=%.1f lift_speed=%.1f)"
            % (pos[0], pos[1], pos[2], sample_count, retract, speed, lift_speed)
        )
        measured = []
        for _ in range(sample_count):
            z = self._probe_once(gcmd, toolhead, homing_obj, engine, speed)
            measured.append(z)
            self._retract(toolhead, z + retract, lift_speed)
        average = calc_probe_z_result(measured, "average")
        median = calc_probe_z_result(measured, "median")
        sigma = (
            sum((v - average) ** 2 for v in measured) / len(measured)
        ) ** 0.5
        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f,"
            " range %.6f, average %.6f, median %.6f, standard deviation %.6f"
            % (
                max(measured),
                min(measured),
                max(measured) - min(measured),
                average,
                median,
                sigma,
            )
        )

    def cmd_Z_OFFSET_APPLY_PROBE(self, gcmd):
        offset = self.gcode_move.get_status()["homing_origin"].z
        if offset == 0.0:
            gcmd.respond_info("Nothing to do: Z Offset is 0")
            return
        new_calibrate = self.z_offset - offset
        gcmd.respond_info(
            "%s: z_offset: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer."
            % (self.name, new_calibrate)
        )
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.name, "z_offset", "%.3f" % (new_calibrate,))


class ProbePointsHelper:
    def __init__(
        self,
        config,
        finalize_callback,
        default_points=None,
        option_name="points",
        use_offsets=False,
        enable_horizontal_z_clearance=False,
    ):
        self.printer = config.get_printer()
        self.finalize_callback = finalize_callback
        self.probe_points = default_points
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object("gcode")
        if default_points is None or config.get(option_name, None) is not None:
            self.probe_points = config.getlists(
                option_name, seps=(",", "\n"), parser=float, count=2
            )
        def_move_z = config.getfloat("horizontal_move_z", 5.0)
        self.horizontal_move_z = self.default_horizontal_move_z = def_move_z
        self.enable_horizontal_z_clearance = enable_horizontal_z_clearance
        self.horizontal_z_clearance = self.default_horizontal_z_clearance = None
        if enable_horizontal_z_clearance:
            z_clearance = config.getfloat("horizontal_z_clearance", None)
            self.default_horizontal_z_clearance = z_clearance
            self.horizontal_z_clearance = z_clearance
        self.adaptive_horizontal_move_z = config.getboolean(
            "adaptive_horizontal_move_z", False
        )
        self.min_horizontal_move_z = config.getfloat(
            "min_horizontal_move_z", 1.0
        )
        self.speed = config.getfloat("speed", 50.0, above=0.0)
        self.use_offsets = config.getboolean(
            "use_probe_xy_offsets", use_offsets
        )
        self.enforce_lift_speed = config.getboolean("enforce_lift_speed", False)
        self.lift_speed = self.speed
        self.probe_offsets = (0.0, 0.0, 0.0)
        self.results = []

    def get_probe_points(self):
        return self.probe_points

    def minimum_points(self, n):
        if len(self.probe_points) < n:
            raise self.printer.config_error(
                "Need at least %d probe points for %s" % (n, self.name)
            )

    def update_probe_points(self, points, min_points):
        self.probe_points = points
        self.minimum_points(min_points)

    def use_xy_offsets(self, use_offsets):
        self.use_offsets = use_offsets

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        return self.lift_speed

    def _lift_toolhead(self):
        toolhead = self.printer.lookup_object("toolhead")
        speed = self.lift_speed
        if not self.results and not self.enforce_lift_speed:
            speed = self.speed
        z_pos = self.horizontal_move_z
        if self.horizontal_z_clearance is not None and self.results:
            z_pos = toolhead.get_position()[2] + self.horizontal_z_clearance
        toolhead.manual_move([None, None, z_pos], speed)

    def _next_pos(self):
        nextpos = list(self.probe_points[len(self.results)])
        if self.use_offsets:
            nextpos[0] -= self.probe_offsets[0]
            nextpos[1] -= self.probe_offsets[1]
        return nextpos

    def _move_next(self):
        toolhead = self.printer.lookup_object("toolhead")
        done = False
        finalize = len(self.results) >= len(self.probe_points)
        if finalize:
            toolhead.get_last_move_time()
            res = self.finalize_callback(self.probe_offsets, self.results)
            if isinstance(res, (int, float)):
                if res == 0:
                    done = True
                if self.adaptive_horizontal_move_z:
                    error = math.ceil(res)
                    self.horizontal_move_z = max(
                        error + self.probe_offsets[2],
                        self.min_horizontal_move_z,
                    )
            elif res != "retry":
                done = True
        self._lift_toolhead()
        if finalize:
            self.results = []
        if done:
            return True
        toolhead.manual_move(self._next_pos(), self.speed)
        return False

    def start_probe(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        probe = self.printer.lookup_object("probe", None)
        method = gcmd.get("METHOD", "automatic").lower()
        if method not in ("automatic", "manual"):
            raise gcmd.error(
                "METHOD=%s is not supported (use automatic or manual)"
                % (method,)
            )
        self.results = []
        def_move_z = self.default_horizontal_move_z
        self.horizontal_move_z = gcmd.get_float("HORIZONTAL_MOVE_Z", def_move_z)
        if self.enable_horizontal_z_clearance:
            self.horizontal_z_clearance = gcmd.get_float(
                "HORIZONTAL_Z_CLEARANCE", self.default_horizontal_z_clearance
            )
        enforce_lift_speed = gcmd.get_int(
            "ENFORCE_LIFT_SPEED", None, minval=0, maxval=1
        )
        if enforce_lift_speed is not None:
            self.enforce_lift_speed = enforce_lift_speed
        if probe is None or method == "manual":
            self.lift_speed = self.speed
            self.probe_offsets = (0.0, 0.0, 0.0)
            self._manual_probe_start()
            return
        self.lift_speed = probe.get_lift_speed(gcmd)
        self.probe_offsets = probe.get_offsets()
        if self.horizontal_move_z < self.probe_offsets[2]:
            raise gcmd.error(
                "horizontal_move_z can't be less than probe's z_offset"
            )
        probe.multi_probe_begin()
        while True:
            done = self._move_next()
            if done:
                break
            pos = probe.run_probe(gcmd)
            self.results.append(pos)
        probe.multi_probe_end()

    def _manual_probe_start(self):
        done = self._move_next()
        if not done:
            gcmd = self.gcode.create_gcode_command("", "", {})
            manual_probe.ManualProbeHelper(
                self.printer, gcmd, self._manual_probe_finalize
            )

    def _manual_probe_finalize(self, kin_pos):
        if kin_pos is None:
            return
        self.results.append(kin_pos)
        self._manual_probe_start()


def load_config(config):
    return PrinterProbe(config)
