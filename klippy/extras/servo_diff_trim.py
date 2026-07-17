# Differential belt-pair trim: standstill zeroing of the pair fight.
#
# Servo sync and homing leave a run-to-run differential preload between the
# two drives sharing one belt. The engine measures each pair's low-passed
# differential torque at commanded standstill and slowly integrates it into
# a flat antisymmetric position offset riding on top of the strain
# compensation map, nulling the trapped fight without a calibration sweep.
# During motion the loop freezes: an in-motion differential torque is
# legitimate (commanded feedforward, direction- and position-dependent
# load). A SERVO_SYNC or torque drop resets the offset — the release is the
# new zero.
from . import servo_strokes

MAX_GAIN = 2.0
OFFSET_UM_CEILING = 500.0
MAX_SETTLE_MS = 60000
BELTS = ("A", "B")


class ServoDiffTrim:
    cmd_SERVO_DIFF_TRIM_help = (
        "Arm or disarm the differential belt-pair trim: at commanded "
        "standstill the engine integrates each pair's low-passed "
        "differential torque into a small antisymmetric position offset, "
        "zeroing the fight left behind by servo sync / homing variance; "
        "the loop freezes during motion. GAIN is in mm/s of offset slew "
        "per 1% differential torque; GAIN=0 disarms. MAX_OFFSET_UM bounds "
        "the offset (hitting it logs a warning). SETTLE_MS is how long "
        "the pair must sit still before measuring resumes. SAVE=1 stores "
        "the values for SAVE_CONFIG. Params BELT=A|B|AB GAIN "
        "MAX_OFFSET_UM LPF_HZ SETTLE_MS SAVE"
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gain = config.getfloat("gain", 0.0, minval=0.0, maxval=MAX_GAIN)
        self.max_offset_um = config.getfloat(
            "max_offset_um", 150.0, above=0.0, maxval=OFFSET_UM_CEILING
        )
        self.lpf_hz = config.getfloat("lpf_hz", 2.0, above=0.0)
        self.settle_ms = config.getint(
            "settle_ms", 300, minval=0, maxval=MAX_SETTLE_MS
        )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_DIFF_TRIM",
            self.cmd_SERVO_DIFF_TRIM,
            desc=self.cmd_SERVO_DIFF_TRIM_help,
        )
        self.printer.register_event_handler("klippy:ready", self._on_ready)

    def _arm_belt(self, gcmd, belt):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        pair_names, _motors, handle, slots = servo_strokes.belt_pair(
            self.printer, gcmd, kin, belt, "SERVO_DIFF_TRIM"
        )
        engine = self.printer.lookup_object("motion_engine")
        engine.set_diff_trim(
            handle,
            slots[0],
            slots[1],
            int(round(self.gain * 1e6)),
            int(round(self.max_offset_um)),
            int(round(self.lpf_hz * 1000.0)),
            self.settle_ms,
        )
        return pair_names

    def _on_ready(self):
        if self.gain <= 0.0:
            return
        for belt in BELTS:
            self._arm_belt(_ReadyContext(self.printer), belt)

    def cmd_SERVO_DIFF_TRIM(self, gcmd):
        belts = gcmd.get("BELT", "AB").upper()
        if belts not in ("A", "B", "AB"):
            raise gcmd.error("BELT must be A, B or AB (got %r)" % (belts,))
        self.gain = gcmd.get_float(
            "GAIN", self.gain, minval=0.0, maxval=MAX_GAIN
        )
        self.max_offset_um = gcmd.get_float(
            "MAX_OFFSET_UM",
            self.max_offset_um,
            above=0.0,
            maxval=OFFSET_UM_CEILING,
        )
        self.lpf_hz = gcmd.get_float("LPF_HZ", self.lpf_hz, above=0.0)
        self.settle_ms = gcmd.get_int(
            "SETTLE_MS", self.settle_ms, minval=0, maxval=MAX_SETTLE_MS
        )
        for belt in belts:
            pair_names = self._arm_belt(gcmd, belt)
            if self.gain > 0.0:
                gcmd.respond_info(
                    "belt %s trim armed (%s vs %s): gain %.3f (mm/s)/%%, "
                    "max offset %.0f um, lpf %.2f Hz, settle %d ms"
                    % (
                        belt,
                        pair_names[0],
                        pair_names[1],
                        self.gain,
                        self.max_offset_um,
                        self.lpf_hz,
                        self.settle_ms,
                    )
                )
            else:
                gcmd.respond_info("belt %s trim disarmed" % (belt,))
        if gcmd.get_int("SAVE", 0):
            configfile = self.printer.lookup_object("configfile")
            configfile.set("servo_diff_trim", "gain", "%.6f" % (self.gain,))
            configfile.set(
                "servo_diff_trim",
                "max_offset_um",
                "%.1f" % (self.max_offset_um,),
            )
            configfile.set("servo_diff_trim", "lpf_hz", "%.3f" % (self.lpf_hz,))
            configfile.set(
                "servo_diff_trim", "settle_ms", "%d" % (self.settle_ms,)
            )
            gcmd.respond_info(
                "servo_diff_trim settings staged; run SAVE_CONFIG to persist"
            )


class _ReadyContext:
    """Stands in for a gcmd when arming from the ready callback: helpers
    raise `gcmd.error`, which here is the printer's command error class."""

    def __init__(self, printer):
        self.error = printer.command_error


def load_config(config):
    return ServoDiffTrim(config)
