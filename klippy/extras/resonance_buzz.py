import math

MOTOR_A = 0b001
MOTOR_B = 0b010
MOTOR_Z = 0b100


def buzz_axis_to_motor_mask(axis, coupled):
    axis = axis.lower()
    if coupled:
        corexy_in_phase = (MOTOR_A | MOTOR_B, 0)
        corexy_anti_phase = (MOTOR_A | MOTOR_B, MOTOR_B)
        mapping = {
            "x": corexy_in_phase,
            "y": corexy_anti_phase,
            "z": (MOTOR_Z, 0),
        }
    else:
        mapping = {
            "x": (MOTOR_A, 0),
            "y": (MOTOR_B, 0),
            "z": (MOTOR_Z, 0),
        }
    if axis not in mapping:
        raise ValueError("unsupported buzz axis %r" % (axis,))
    return mapping[axis]


BUZZ_PEAK_ACCEL_CEILING_MM_S2 = 15000.0
BUZZ_MAX_AMPLITUDE_MM = 5.0


def sinusoid_peak_accel(accel_per_hz, freq_hz):
    return accel_per_hz * freq_hz


def sinusoid_amplitude_mm(accel_per_hz, freq_hz):
    return accel_per_hz / (4.0 * math.pi**2 * freq_hz)


class ResonanceBuzz:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "RESONANCE_BUZZ",
            self.cmd_RESONANCE_BUZZ,
            desc=self.cmd_RESONANCE_BUZZ_help,
        )
        self.gcode.register_command(
            "RESONANCE_BUZZ_SWEEP",
            self.cmd_RESONANCE_BUZZ_SWEEP,
            desc=self.cmd_RESONANCE_BUZZ_SWEEP_help,
        )

    def run_sweep(
        self,
        gcmd,
        axis_name,
        freq_start,
        freq_end,
        duration,
        ramp,
        accel_per_hz,
        amplitude_mm,
    ):
        if axis_name not in ("x", "y", "z"):
            raise gcmd.error("AXIS must be x, y, or z")
        toolhead = self.printer.lookup_object("toolhead")
        motion = self.printer.lookup_object("motion")
        kin = toolhead.get_kinematics()
        coupled = bool(getattr(kin, "coupled_xy", lambda: False)())
        try:
            axis_mask, sign_mask = buzz_axis_to_motor_mask(axis_name, coupled)
        except ValueError as e:
            raise gcmd.error(str(e))

        if amplitude_mm <= 0.0:
            highest_freq = max(freq_start, freq_end)
            if (
                sinusoid_peak_accel(accel_per_hz, highest_freq)
                > BUZZ_PEAK_ACCEL_CEILING_MM_S2
            ):
                accel_per_hz = BUZZ_PEAK_ACCEL_CEILING_MM_S2 / highest_freq
                gcmd.respond_info(
                    "RESONANCE_BUZZ: clamped accel_per_hz to %.1f to keep peak "
                    "accel <= %.0f mm/s^2 at %.1f Hz"
                    % (
                        accel_per_hz,
                        BUZZ_PEAK_ACCEL_CEILING_MM_S2,
                        highest_freq,
                    )
                )
            amplitude_mm = sinusoid_amplitude_mm(accel_per_hz, freq_start)
        if amplitude_mm > BUZZ_MAX_AMPLITUDE_MM:
            raise gcmd.error(
                "RESONANCE_BUZZ amplitude %.3f mm at %.1f Hz exceeds %.1f mm "
                "ceiling" % (amplitude_mm, freq_start, BUZZ_MAX_AMPLITUDE_MM)
            )

        toolhead.wait_moves()
        motion.submit_resonance_buzz(
            axis_mask,
            sign_mask,
            int(round(freq_start * 1000.0)),
            int(round(freq_end * 1000.0)),
            int(round(amplitude_mm * 1e6)),
            int(round(duration * 1000.0)),
            int(round(ramp * 1000.0)),
        )
        phasing = ""
        if coupled and axis_name in ("x", "y"):
            phasing = " (corexy A/B %s)" % (
                "in-phase" if axis_name == "x" else "anti-phase"
            )
        if abs(freq_end - freq_start) < 1e-6:
            freq_desc = "freq=%.1fHz" % (freq_start,)
        else:
            freq_desc = "sweep=%.1f->%.1fHz" % (freq_start, freq_end)
        gcmd.respond_info(
            "RESONANCE_BUZZ axis=%s %s amplitude@start=%.1fum duration=%.2fs%s"
            % (axis_name, freq_desc, amplitude_mm * 1000.0, duration, phasing)
        )
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + duration + 0.1)
        return duration

    cmd_RESONANCE_BUZZ_help = (
        "Excite a single resonance frequency on one axis via the engine-"
        "resident buzz generator"
    )

    def cmd_RESONANCE_BUZZ(self, gcmd):
        axis_name = gcmd.get("AXIS", "x").lower()
        freq = gcmd.get_float("FREQ", 50.0, above=0.0)
        duration = gcmd.get_float("DURATION", 1.0, above=0.0)
        accel_per_hz = gcmd.get_float("ACCEL_PER_HZ", 75.0, above=0.0)
        amplitude_mm = gcmd.get_float("AMPLITUDE", 0.0, minval=0.0)
        ramp = gcmd.get_float(
            "RAMP", min(duration * 0.25, 3.0 / freq), above=0.0
        )
        self.run_sweep(
            gcmd,
            axis_name,
            freq,
            freq,
            duration,
            ramp,
            accel_per_hz,
            amplitude_mm,
        )

    cmd_RESONANCE_BUZZ_SWEEP_help = (
        "Sweep a frequency band on one axis via the engine-resident sweep "
        "generator (one fade-in/out for the whole sweep). Phase-stepping axes "
        "run a continuous chirp; STEP/DIR axes run a fixed-frequency staircase."
    )

    def cmd_RESONANCE_BUZZ_SWEEP(self, gcmd):
        axis_name = gcmd.get("AXIS", "x").lower()
        freq_start = gcmd.get_float("FREQ_START", 5.0, above=0.0)
        freq_end = gcmd.get_float("FREQ_END", 135.0, above=0.0)
        accel_per_hz = gcmd.get_float("ACCEL_PER_HZ", 75.0, above=0.0)
        amplitude_mm = gcmd.get_float("AMPLITUDE", 0.0, minval=0.0)
        hz_per_sec = gcmd.get_float("HZ_PER_SEC", 1.0, above=0.0)
        span = abs(freq_end - freq_start)
        duration = gcmd.get_float("DURATION", 0.0, minval=0.0)
        if duration <= 0.0:
            duration = max(span / hz_per_sec, 0.1)
        three_periods_of_lowest = 3.0 / min(freq_start, freq_end)
        tenth_of_sweep = 0.1 * duration
        ramp = gcmd.get_float(
            "RAMP",
            min(tenth_of_sweep, three_periods_of_lowest),
            above=0.0,
        )
        self.run_sweep(
            gcmd,
            axis_name,
            freq_start,
            freq_end,
            duration,
            ramp,
            accel_per_hz,
            amplitude_mm,
        )


def load_config(config):
    return ResonanceBuzz(config)
