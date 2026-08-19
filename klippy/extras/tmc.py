# Common helper code for TMC stepper drivers
#
# Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging
import math

from klippy import pins, stepper, structured_log
from klippy.motion_endstop import MotionEndstop, allocate_provider_id

TRANSPORT_PULSE = 0
TRANSPORT_PHASE = 1

######################################################################
# Field helpers
######################################################################


# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1


class FieldHelper:
    def __init__(
        self,
        all_fields,
        signed_fields=None,
        field_formatters=None,
        registers=None,
    ):
        if field_formatters is None:
            field_formatters = {}
        if signed_fields is None:
            signed_fields = []
        self.all_fields = all_fields
        self.signed_fields = {sf: 1 for sf in signed_fields}
        self.field_formatters = field_formatters
        self.registers = registers
        if self.registers is None:
            self.registers = collections.OrderedDict()
        self.field_to_register = {
            f: r for r, fields in self.all_fields.items() for f in fields
        }

    def lookup_register(self, field_name, default=None):
        return self.field_to_register.get(field_name, default)

    def get_field(self, field_name, reg_value=None, reg_name=None):
        # Returns value of the register field
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        field_value = (reg_value & mask) >> ffs(mask)
        if (
            field_name in self.signed_fields
            and ((reg_value & mask) << 1) > mask
        ):
            field_value -= 1 << field_value.bit_length()
        return field_value

    def set_field(self, field_name, field_value):
        # Update the desired configuration; returns the new register value
        reg_name = self.field_to_register[field_name]
        new_value = self.override_register(reg_name, {field_name: field_value})
        self.registers[reg_name] = new_value
        return new_value

    def override_register(self, reg_name, field_overrides):
        """Register value with transient field overrides applied on top
        of the desired configuration. The desired configuration is not
        touched, so a later full register replay restores it."""
        reg_value = self.registers.get(reg_name, 0)
        for field_name, field_value in field_overrides.items():
            mask = self.all_fields[reg_name][field_name]
            reg_value = (reg_value & ~mask) | (
                (field_value << ffs(mask)) & mask
            )
        return reg_value

    def set_config_field(self, config, field_name, default):
        # Allow a field to be set from the config file
        config_name = "driver_" + field_name.upper()
        reg_name = self.field_to_register[field_name]
        mask = self.all_fields[reg_name][field_name]
        maxval = mask >> ffs(mask)
        if maxval == 1:
            val = config.getboolean(config_name, default)
        elif field_name in self.signed_fields:
            val = config.getint(
                config_name,
                default,
                minval=-(maxval // 2 + 1),
                maxval=maxval // 2,
            )
        else:
            val = config.getint(config_name, default, minval=0, maxval=maxval)
        if default is None and val is None:
            return
        return self.set_field(field_name, val)

    def pretty_format(self, reg_name, reg_value):
        # Provide a string description of a register
        reg_fields = self.all_fields.get(reg_name, {})
        reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
        fields = []
        for mask, field_name in reg_fields:
            field_value = self.get_field(field_name, reg_value, reg_name)
            sval = self.field_formatters.get(field_name, str)(field_value)
            if sval and sval != "0":
                fields.append(" %s=%s" % (field_name, sval))
        return "%-11s %08x%s" % (reg_name + ":", reg_value, "".join(fields))

    def get_reg_fields(self, reg_name, reg_value):
        # Provide fields found in a register
        reg_fields = self.all_fields.get(reg_name, {})
        return {
            field_name: self.get_field(field_name, reg_value, reg_name)
            for field_name, mask in reg_fields.items()
        }


######################################################################
# Driver mode tracking
######################################################################


class TMCModeTracker:
    """Single source of truth for the mode the driver silicon is in.

    Every lifecycle actor (enable path, StallGuard arm/disarm, phase
    stepping enter/exit) moves the driver between modes through
    transition(), which fails loudly on a sequence the hardware cannot
    honor instead of letting the actors clobber each other's state.
    """

    DISABLED = "disabled"
    PULSE = "pulse"
    PHASE_DIRECT = "phase_direct"
    SG_HOMING = "sg_homing"

    def __init__(self, printer, stepper_name):
        self.printer = printer
        self.stepper_name = stepper_name
        self.mode = self.DISABLED

    def transition(self, allowed, new_mode, what):
        if self.mode not in allowed:
            raise self.printer.command_error(
                "TMC %s: %s is illegal while the driver is in %s mode"
                % (self.stepper_name, what, self.mode)
            )
        previous, self.mode = self.mode, new_mode
        return previous

    def require(self, allowed, what):
        self.transition(allowed, self.mode, what)


######################################################################
# Periodic error checking
######################################################################


class TMCErrorCheck:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        name_parts = config.get_name().split()
        self.stepper_name = " ".join(name_parts[1:])
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.check_timer = None
        self.last_drv_status = self.last_drv_fields = None
        # Setup for GSTAT query
        reg_name = self.fields.lookup_register("drv_err")
        if reg_name is not None:
            self.gstat_reg_info = [0, reg_name, 0xFFFFFFFF, 0xFFFFFFFF, 0]
        else:
            self.gstat_reg_info = None
        self.clear_gstat = True
        # Setup for DRV_STATUS query
        self.irun_field = "irun"
        reg_name = "DRV_STATUS"
        mask = err_mask = cs_actual_mask = 0
        if name_parts[0] == "tmc2130":
            # TMC2130 driver quirks
            self.clear_gstat = False
            cs_actual_mask = self.fields.all_fields[reg_name]["cs_actual"]
        elif name_parts[0] == "tmc2660":
            # TMC2660 driver quirks
            self.irun_field = "cs"
            reg_name = "READRSP@RDSEL2"
            cs_actual_mask = self.fields.all_fields[reg_name]["se"]
        err_fields = ["ot", "s2ga", "s2gb", "s2vsa", "s2vsb"]
        warn_fields = ["otpw", "t120", "t143", "t150", "t157"]
        for f in err_fields + warn_fields:
            if f in self.fields.all_fields[reg_name]:
                mask |= self.fields.all_fields[reg_name][f]
                if f in err_fields:
                    err_mask |= self.fields.all_fields[reg_name][f]
        self.drv_status_reg_info = [0, reg_name, mask, err_mask, cs_actual_mask]
        # Setup for temperature query
        self.adc_temp = None
        self.adc_temp_reg = self.fields.lookup_register("adc_temp")
        if self.adc_temp_reg is not None:
            pheaters = self.printer.load_object(config, "heaters")
            pheaters.register_monitor(config)

    def _query_register(self, reg_info, try_clear=False):
        last_value, reg_name, mask, err_mask, cs_actual_mask = reg_info
        cleared_flags = 0
        count = 0
        while True:
            try:
                val = self.mcu_tmc.get_register(reg_name)
            except self.printer.command_error as e:
                count += 1
                if count < 3 and str(e).startswith("Unable to read tmc uart"):
                    # Allow more retries on a TMC UART read error
                    reactor = self.printer.get_reactor()
                    reactor.pause(reactor.monotonic() + 0.050)
                    continue
                raise
            if val & mask != last_value & mask:
                fmt = self.fields.pretty_format(reg_name, val)
                logging.info("TMC '%s' reports %s", self.stepper_name, fmt)
            reg_info[0] = last_value = val
            if not val & err_mask:
                if not cs_actual_mask or val & cs_actual_mask:
                    break
                irun = self.fields.get_field(self.irun_field)
                if self.check_timer is None or irun < 4:
                    break
                if self.irun_field == "irun" and not self.fields.get_field(
                    "ihold"
                ):
                    break
                # CS_ACTUAL field of zero - indicates a driver reset
            count += 1
            if count >= 3:
                fmt = self.fields.pretty_format(reg_name, val)
                raise self.printer.command_error(
                    "TMC '%s' reports error: %s" % (self.stepper_name, fmt)
                )
            if try_clear and val & err_mask:
                try_clear = False
                cleared_flags |= val & err_mask
                self.mcu_tmc.set_register(reg_name, val & err_mask)
        return cleared_flags

    def _query_temperature(self):
        try:
            self.adc_temp = self.mcu_tmc.get_register(self.adc_temp_reg)
        except self.printer.command_error as e:
            # Ignore comms error for temperature
            self.adc_temp = None
            return

    def _do_periodic_check(self, eventtime):
        try:
            self._query_register(self.drv_status_reg_info)
            if self.gstat_reg_info is not None:
                self._query_register(self.gstat_reg_info)
            if self.adc_temp_reg is not None:
                self._query_temperature()
        except self.printer.command_error as e:
            # A CRC-checked UART read surfaces corruption as a failed read,
            # never as a false fault, and the driver self-protects in hardware
            # (thermal/short) in real time. An unreachable driver therefore
            # tells us nothing actionable; keep monitoring rather than shut
            # down. A validly-read fault still raises a non-uart error below.
            if str(e).startswith("Unable to read tmc uart"):
                logging.warning(
                    "TMC %s: driver unreachable, skipping periodic check: %s",
                    self.stepper_name,
                    str(e),
                )
                return eventtime + 1.0
            self.printer.invoke_shutdown(str(e))
            return self.printer.get_reactor().NEVER
        return eventtime + 1.0

    def reset_detect_supported(self):
        return self.gstat_reg_info is not None and self.clear_gstat

    def stop_checks(self):
        if self.check_timer is None:
            return
        self.printer.get_reactor().unregister_timer(self.check_timer)
        self.check_timer = None

    def start_checks(self):
        if self.check_timer is not None:
            self.stop_checks()
        cleared_flags = 0
        self._query_register(self.drv_status_reg_info)
        if self.gstat_reg_info is not None:
            cleared_flags = self._query_register(
                self.gstat_reg_info, try_clear=self.clear_gstat
            )
        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        self.check_timer = reactor.register_timer(
            self._do_periodic_check, curtime + 1.0
        )
        if cleared_flags:
            reset_mask = self.fields.all_fields["GSTAT"]["reset"]
            if cleared_flags & reset_mask:
                return True
        return False

    def get_status(self, eventtime=None):
        if self.check_timer is None:
            return {"drv_status": None, "temperature": None}
        temp = None
        if self.adc_temp is not None:
            temp = round((self.adc_temp - 2038) / 7.7, 2)
        last_value, reg_name = self.drv_status_reg_info[:2]
        if last_value != self.last_drv_status:
            self.last_drv_status = last_value
            fields = self.fields.get_reg_fields(reg_name, last_value)
            self.last_drv_fields = {n: v for n, v in fields.items() if v}
        return {"drv_status": self.last_drv_fields, "temperature": temp}


######################################################################
# G-Code command helpers
######################################################################


class TMCCommandHelper:
    def __init__(self, config, mcu_tmc, current_helper):
        self.printer = config.get_printer()
        self.stepper_name = " ".join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.current_helper = current_helper
        self.echeck_helper = TMCErrorCheck(config, mcu_tmc)
        self.fields = mcu_tmc.get_fields()
        self.read_registers = self.read_translate = None
        self.toff = None
        self.stepper = None
        self.mode_tracker = TMCModeTracker(self.printer, self.stepper_name)
        self._post_enable_cb = None
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.printer.register_event_handler(
            "klippy:mcu_identify", self._handle_mcu_identify
        )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        # Set microstep config options
        TMCMicrostepHelper(config, mcu_tmc)
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_TMC_FIELD",
            "STEPPER",
            self.name,
            self.cmd_SET_TMC_FIELD,
            desc=self.cmd_SET_TMC_FIELD_help,
        )
        gcode.register_mux_command(
            "INIT_TMC",
            "STEPPER",
            self.name,
            self.cmd_INIT_TMC,
            desc=self.cmd_INIT_TMC_help,
        )
        gcode.register_mux_command(
            "SET_TMC_CURRENT",
            "STEPPER",
            self.name,
            self.cmd_SET_TMC_CURRENT,
            desc=self.cmd_SET_TMC_CURRENT_help,
        )

    def _init_registers(self, print_time=None):
        # Send registers
        for reg_name in list(self.fields.registers.keys()):
            val = self.fields.registers[reg_name]  # Val may change during loop
            self.mcu_tmc.set_register(reg_name, val, print_time)

    def set_post_enable_callback(self, cb):
        self._post_enable_cb = cb

    cmd_INIT_TMC_help = "Initialize TMC stepper driver registers"

    def cmd_INIT_TMC(self, gcmd):
        logging.info("INIT_TMC %s", self.name)
        self.mode_tracker.require(
            (TMCModeTracker.DISABLED, TMCModeTracker.PULSE),
            "INIT_TMC register replay",
        )
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        self._init_registers(print_time)

    cmd_SET_TMC_FIELD_help = "Set a register field of a TMC driver"

    def cmd_SET_TMC_FIELD(self, gcmd):
        field_name = gcmd.get("FIELD").lower()
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise gcmd.error("Unknown field name '%s'" % (field_name,))
        value = gcmd.get_int("VALUE", None)
        velocity = gcmd.get_float("VELOCITY", None, minval=0.0)
        if (value is None) == (velocity is None):
            raise gcmd.error("Specify either VALUE or VELOCITY")
        if velocity is not None:
            if self.mcu_tmc.get_tmc_frequency() is None:
                raise gcmd.error(
                    "VELOCITY parameter not supported by this driver"
                )
            value = TMCtstepHelper(
                self.mcu_tmc, velocity, pstepper=self.stepper
            )
        reg_val = self.fields.set_field(field_name, value)
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        self.mcu_tmc.set_register(reg_name, reg_val, print_time)

    cmd_SET_TMC_CURRENT_help = "Set the current of a TMC driver"

    def cmd_SET_TMC_CURRENT(self, gcmd):
        ch = self.current_helper
        (
            prev_cur,
            prev_hold_cur,
            req_hold_cur,
            max_cur,
            prev_home_cur,
        ) = ch.get_current()
        run_current = gcmd.get_float(
            "CURRENT", None, minval=0.0, maxval=max_cur
        )
        hold_current = gcmd.get_float(
            "HOLDCURRENT", None, above=0.0, maxval=max_cur
        )
        home_current = gcmd.get_float(
            "HOMECURRENT", None, above=0.0, maxval=max_cur
        )
        if (
            run_current is not None
            or hold_current is not None
            or home_current is not None
        ):
            if run_current is not None:
                ch.set_run_current(run_current)
            else:
                run_current = prev_cur

            if hold_current is None:
                hold_current = req_hold_cur

            if home_current is not None:
                ch.set_home_current(home_current)

            toolhead = self.printer.lookup_object("toolhead")
            print_time = toolhead.get_last_move_time()
            ch.set_current(run_current, hold_current, print_time)
            (
                prev_cur,
                prev_hold_cur,
                req_hold_cur,
                max_cur,
                prev_home_cur,
            ) = ch.get_current()
        # Report values
        if prev_hold_cur is None:
            gcmd.respond_info(
                "Run Current: %0.2fA Home Current: %0.2fA"
                % (prev_cur, prev_home_cur)
            )
        else:
            gcmd.respond_info(
                "Run Current: %0.2fA Hold Current: %0.2fA Home Current: %0.2fA"
                % (prev_cur, prev_hold_cur, prev_home_cur)
            )

    def _get_phases(self):
        return (256 >> self.fields.get_field("mres")) * 4

    def get_phase_offset(self):
        return None, self._get_phases()

    # Stepper enable/disable tracking
    def _apply_driver_config(self, restore_toff, print_time=None):
        if restore_toff and self.toff is not None:
            self.fields.set_field("toff", self.toff)
        self._init_registers(print_time)
        if self._post_enable_cb is not None:
            self._post_enable_cb()

    def _do_disable(self, print_time):
        try:
            if self.toff is not None:
                val = self.fields.set_field("toff", 0)
                reg_name = self.fields.lookup_register("toff")
                self.mcu_tmc.set_register(reg_name, val, print_time)
            self.echeck_helper.stop_checks()
            self.mode_tracker.transition(
                (
                    TMCModeTracker.DISABLED,
                    TMCModeTracker.PULSE,
                    TMCModeTracker.PHASE_DIRECT,
                    TMCModeTracker.SG_HOMING,
                ),
                TMCModeTracker.DISABLED,
                "stepper disable",
            )
        except (self.printer.command_error, RuntimeError) as e:
            self.printer.invoke_shutdown(
                "TMC %s disable failed: %s" % (self.stepper_name, e)
            )

    def _handle_mcu_identify(self):
        # Lookup stepper object
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        self.stepper.set_tmc_current_helper(self.current_helper)

        # Note pulse duration and step_both_edge optimizations available
        self.stepper.setup_default_pulse_duration(0.000000100, True)

    def _handle_stepper_enable(self, print_time, is_enable):
        if is_enable:
            # Inline, not deferred like disable below: the engine ships the
            # move right after this with no lookahead, so deferring to the
            # reactor loses the first move on a driver not yet ready.
            self._do_enable(print_time)
            return

        def cb(ev):
            return self._do_disable(print_time)

        self.printer.get_reactor().register_callback(cb)

    def _do_enable(self, print_time):
        try:
            if self._post_enable_cb is not None:
                self._apply_driver_config(restore_toff=True)
                return
            did_reset = self.echeck_helper.start_checks()
            reinit = (
                did_reset or not self.echeck_helper.reset_detect_supported()
            )
            if reinit:
                self._apply_driver_config(restore_toff=True)
            elif self.toff is not None:
                val = self.fields.set_field("toff", self.toff)
                reg_name = self.fields.lookup_register("toff")
                self.mcu_tmc.set_register(reg_name, val, print_time)
            self.mode_tracker.transition(
                (TMCModeTracker.DISABLED,),
                TMCModeTracker.PULSE,
                "stepper enable",
            )
        except (self.printer.command_error, RuntimeError) as e:
            self.printer.invoke_shutdown(
                "TMC %s enable failed: %s" % (self.stepper_name, e)
            )

    def _handle_connect(self):
        # Check if using step on both edges optimization
        pulse_duration, step_both_edge = self.stepper.get_pulse_duration()
        if step_both_edge:
            self.fields.set_field("dedge", 1)
        # Check for soft stepper enable/disable
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        enable_line.register_state_callback(self._handle_stepper_enable)
        if not enable_line.has_dedicated_enable():
            self.toff = self.fields.get_field("toff")
            self.fields.set_field("toff", 0)
            logging.info(
                "Enabling TMC virtual enable for '%s'", self.stepper_name
            )
        # A previous session may have left the firmware's phase ISR
        # streaming XDIRECT on this SPI bus (klippy restarts do not reset
        # the mcu); its transfers corrupt the register init below, so
        # silence it first. Harmless when it is already off.
        tmc_spi = getattr(self.mcu_tmc, "tmc_spi", None)
        if tmc_spi is not None:
            disable_spi = tmc_spi.spi.get_mcu().try_lookup_command(
                "kalico_phase_stepping_disable_spi"
            )
            if disable_spi is not None:
                disable_spi.send([])
        # Send init
        try:
            if self.mcu_tmc.mcu.non_critical_disconnected:
                logging.info(
                    "TMC %s failed to init - non_critical_mcu: %s is disconnected!",
                    self.name,
                    self.mcu_tmc.mcu.get_name(),
                )
            else:
                self._apply_driver_config(restore_toff=False)
        except self.printer.command_error as e:
            logging.info("TMC %s failed to init: %s", self.name, str(e))

    # get_status information export
    def get_status(self, eventtime=None):
        current = self.current_helper.get_current()
        res = {
            "mcu_phase_offset": None,
            "phase_offset_position": None,
            "run_current": current[0],
            "hold_current": current[1],
        }
        res.update(self.echeck_helper.get_status(eventtime))
        return res

    # DUMP_TMC support
    def setup_register_dump(self, read_registers, read_translate=None):
        self.read_registers = read_registers
        self.read_translate = read_translate
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "DUMP_TMC",
            "STEPPER",
            self.name,
            self.cmd_DUMP_TMC,
            desc=self.cmd_DUMP_TMC_help,
        )

    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"

    def cmd_DUMP_TMC(self, gcmd):
        logging.info("DUMP_TMC %s", self.name)
        reg_name = gcmd.get("REGISTER", None)
        if reg_name is not None:
            reg_name = reg_name.upper()
            val = self.fields.registers.get(reg_name)
            if (val is not None) and (reg_name not in self.read_registers):
                # write-only register
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            elif reg_name in self.read_registers:
                # readable register
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            else:
                raise gcmd.error("Unknown register name '%s'" % (reg_name))
        else:
            gcmd.respond_info("========== Write-only registers ==========")
            for reg_name, val in self.fields.registers.items():
                if reg_name not in self.read_registers:
                    gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            gcmd.respond_info("========== Queried registers ==========")
            for reg_name in self.read_registers:
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))


######################################################################
# TMC virtual pins
######################################################################


class TMCVirtualPinHelper:
    def __init__(self, config, mcu_tmc, mode_tracker):
        self.printer = config.get_printer()
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.mode_tracker = mode_tracker
        if self.fields.lookup_register("diag0_stall") is not None:
            if config.get("diag0_pin", None) is not None:
                self.diag_pin = config.get("diag0_pin")
                self.diag_pin_field = "diag0_stall"
            else:
                self.diag_pin = config.get("diag1_pin", None)
                self.diag_pin_field = "diag1_stall"
        else:
            self.diag_pin = config.get("diag_pin", None)
            self.diag_pin_field = None
        self.mcu_endstop = None
        self.phase_mode_helper = None
        self._reenter_phase = False
        self._sg_sample_timer = None
        name_parts = config.get_name().split()
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)

    def setup_motion_endstop(self, pin_params, axis):
        if pin_params["pin"] != "virtual_endstop":
            raise pins.error(
                "tmc drivers only provide the virtual pin 'virtual_endstop',"
                " not '%s'" % (pin_params["pin"],)
            )
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error("Can not pullup/invert tmc virtual endstop")
        if self.diag_pin is None:
            raise pins.error("tmc virtual endstop requires diag pin config")
        if self.mcu_endstop is None:
            ppins = self.printer.lookup_object("pins")
            diag_params = ppins.parse_pin(
                self.diag_pin, can_invert=True, can_pullup=True
            )
            if not hasattr(diag_params["chip"], "create_oid"):
                raise pins.error(
                    "tmc diag pin '%s' must be a GPIO pin on an MCU"
                    % (self.diag_pin,)
                )
            self.mcu_endstop = MotionEndstop(
                diag_params, allocate_provider_id(self.printer)
            )
        return self.mcu_endstop

    def sensorless_homing_configured(self):
        return self.mcu_endstop is not None

    def trip_move_begin(self, entry):
        self.arm()

    def trip_move_end(self, entry):
        self.disarm()

    def _exit_phase_mode_for_homing(self):
        pmh = self.phase_mode_helper
        if pmh is None or not pmh.phase_stepping_active():
            return False
        pmh.exit_phase_mode()
        if pmh.phase_stepping_active():
            raise self.printer.command_error(
                "phase stepping still active after exit_phase_mode; "
                "refusing to start a StallGuard homing move"
            )
        return True

    def arm(self):
        """Transiently override the driver configuration for a StallGuard
        homing move; the desired configuration is untouched, so disarm
        restores by replaying it."""
        self._reenter_phase = self._exit_phase_mode_for_homing()
        self.mode_tracker.transition(
            (TMCModeTracker.PULSE,),
            TMCModeTracker.SG_HOMING,
            "StallGuard arm",
        )
        fields = self.fields
        override = fields.override_register
        if fields.lookup_register("sgthrs", None) is not None:
            self.mcu_tmc.set_register(
                "SGTHRS", fields.registers.get("SGTHRS", 0)
            )
        if fields.lookup_register("en_pwm_mode", None) is None:
            # "stallguard4" drivers only stall-detect in stealthchop
            self.mcu_tmc.set_register(
                "TPWMTHRS", override("TPWMTHRS", {"tpwmthrs": 0})
            )
            gconf_val = override("GCONF", {"en_spreadcycle": 0})
        else:
            # earlier drivers only stall-detect in spreadcycle
            gconf_val = override(
                "GCONF", {"en_pwm_mode": 0, self.diag_pin_field: 1}
            )
        self.mcu_tmc.set_register("GCONF", gconf_val)
        if fields.get_field("tcoolthrs") == 0:
            self.mcu_tmc.set_register(
                "TCOOLTHRS", override("TCOOLTHRS", {"tcoolthrs": 0xFFFFF})
            )
        thigh_reg = fields.lookup_register("thigh", None)
        if thigh_reg is not None:
            self.mcu_tmc.set_register(
                thigh_reg, override(thigh_reg, {"thigh": 0})
            )
        readback = {}
        name_to_reg = getattr(self.mcu_tmc, "name_to_reg", {})
        for reg_name in ("GCONF", "CHOPCONF", "DRV_STATUS", "TSTEP"):
            if reg_name in name_to_reg:
                readback[reg_name] = "%08x" % (
                    self.mcu_tmc.get_register(reg_name),
                )
        structured_log.event(
            "phase_stepping",
            "sg_armed",
            msg="StallGuard armed register readback",
            stepper=self.mode_tracker.stepper_name,
            **readback,
        )
        reactor = self.printer.get_reactor()
        if self._sg_sample_timer is None:
            self._sg_sample_timer = reactor.register_timer(
                self._sample_sg_status, reactor.monotonic() + 0.25
            )
        else:
            reactor.update_timer(
                self._sg_sample_timer, reactor.monotonic() + 0.25
            )

    def _sample_sg_status(self, eventtime):
        if self.mode_tracker.mode != TMCModeTracker.SG_HOMING:
            return self.printer.get_reactor().NEVER
        name_to_reg = getattr(self.mcu_tmc, "name_to_reg", {})
        sample = {}
        for reg_name in ("DRV_STATUS", "TSTEP", "GCONF", "IOIN"):
            if reg_name in name_to_reg:
                sample[reg_name] = "%08x" % (
                    self.mcu_tmc.get_register(reg_name),
                )
        structured_log.event(
            "phase_stepping",
            "sg_sample",
            msg="StallGuard homing sample",
            stepper=self.mode_tracker.stepper_name,
            **sample,
        )
        return eventtime + 0.25

    def disarm(self):
        self.mode_tracker.transition(
            (TMCModeTracker.SG_HOMING,),
            TMCModeTracker.PULSE,
            "StallGuard disarm",
        )
        fields = self.fields
        restore_regs = ["GCONF", "TCOOLTHRS"]
        if fields.lookup_register("en_pwm_mode", None) is None:
            restore_regs.insert(0, "TPWMTHRS")
        thigh_reg = fields.lookup_register("thigh", None)
        if thigh_reg is not None:
            restore_regs.append(thigh_reg)
        for reg_name in restore_regs:
            self.mcu_tmc.set_register(
                reg_name, fields.registers.get(reg_name, 0)
            )
        if self._reenter_phase:
            self._reenter_phase = False
            self.phase_mode_helper.enter_phase_mode()


######################################################################
# Config reading helpers
######################################################################


# Helper to initialize the wave table from config or defaults
def TMCWaveTableHelper(config, mcu_tmc):
    set_config_field = mcu_tmc.get_fields().set_config_field
    set_config_field(config, "mslut0", 0xAAAAB554)
    set_config_field(config, "mslut1", 0x4A9554AA)
    set_config_field(config, "mslut2", 0x24492929)
    set_config_field(config, "mslut3", 0x10104222)
    set_config_field(config, "mslut4", 0xFBFFFFFF)
    set_config_field(config, "mslut5", 0xB5BB777D)
    set_config_field(config, "mslut6", 0x49295556)
    set_config_field(config, "mslut7", 0x00404222)
    set_config_field(config, "w0", 2)
    set_config_field(config, "w1", 1)
    set_config_field(config, "w2", 1)
    set_config_field(config, "w3", 1)
    set_config_field(config, "x1", 128)
    set_config_field(config, "x2", 255)
    set_config_field(config, "x3", 255)
    set_config_field(config, "start_sin", 0)
    set_config_field(config, "start_sin90", 247)


# Helper to configure the microstep settings
def TMCMicrostepHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    stepper_name = " ".join(config.get_name().split()[1:])
    motor_section = "motor " + stepper_name
    if not config.has_section(motor_section):
        raise config.error(
            "Could not find config section '[%s]' required by tmc driver"
            % (motor_section,)
        )
    sconfig = config.getsection(motor_section)
    steps = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
    mres = sconfig.getchoice("microsteps", steps)
    fields.set_field("mres", mres)
    fields.set_field("intpol", config.getboolean("interpolate", True))


# Helper for calculating TSTEP based values from velocity
def TMCtstepHelper(mcu_tmc, velocity, pstepper=None, config=None):
    if velocity <= 0.0:
        return 0xFFFFF
    if pstepper is not None:
        step_dist = pstepper.get_step_dist()
    else:
        stepper_name = " ".join(config.get_name().split()[1:])
        sconfig = config.getsection("motor " + stepper_name)
        rotation_dist, steps_per_rotation = stepper.parse_step_distance(sconfig)
        step_dist = rotation_dist / steps_per_rotation
    mres = mcu_tmc.get_fields().get_field("mres")
    step_dist_256 = step_dist / (1 << mres)
    tmc_freq = mcu_tmc.get_tmc_frequency()
    threshold = int(tmc_freq * step_dist_256 / velocity + 0.5)
    return max(0, min(0xFFFFF, threshold))


# Helper to configure stealthChop-spreadCycle transition velocity
def TMCStealthchopHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    en_pwm_mode = False
    velocity = config.getfloat("stealthchop_threshold", None, minval=0.0)
    tpwmthrs = 0xFFFFF

    if velocity is not None:
        en_pwm_mode = True
        tpwmthrs = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("tpwmthrs", tpwmthrs)

    reg = fields.lookup_register("en_pwm_mode", None)
    if reg is not None:
        fields.set_field("en_pwm_mode", en_pwm_mode)
    else:
        # TMC2208 uses en_spreadCycle
        fields.set_field("en_spreadcycle", not en_pwm_mode)


class BaseTMCCurrentHelper:
    def __init__(self, config, mcu_tmc, max_current, has_sense_resistor=True):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()

        if has_sense_resistor:
            self.sense_resistor = config.getfloat("sense_resistor", above=0.0)

        # config_{run|hold|home}_current
        # represents an initial value set via config file
        self.config_run_current = config.getfloat(
            "run_current", above=0.0, maxval=max_current
        )
        self.config_hold_current = config.getfloat(
            "hold_current", max_current, above=0.0, maxval=max_current
        )
        self.config_home_current = config.getfloat(
            "home_current",
            self.config_run_current,
            above=0.0,
            maxval=max_current,
        )
        self.current_change_dwell_time = config.getfloat(
            "current_change_dwell_time", 0.5, above=0.0
        )

        # req_{run|hold|home}_current
        # represents a requested value, which starts with
        # the configured value but can change during runtime
        # e.g. SET_TMC_CURRENT
        self.req_run_current = self.config_run_current
        self.req_hold_current = self.config_hold_current
        self.req_home_current = self.config_home_current

        # actual_current represents the actual current set to a stepper
        # It fluctuates between req_run_current and req_home_current
        # during homing
        self.actual_current = self.req_run_current

        self.max_current = max_current

    def set_home_current(self, new_home_current):
        self.req_home_current = min(self.max_current, new_home_current)

    def set_run_current(self, new_run_current):
        self.req_run_current = min(self.max_current, new_run_current)

    def set_current_for_homing(self, print_time, pre_homing) -> float:
        target = self.req_home_current if pre_homing else self.req_run_current
        if target == self.actual_current:
            return 0.0
        self.set_current(target, self.req_hold_current, print_time)
        return self.current_change_dwell_time

    def apply_current(self, print_time):
        pass

    def set_current(self, new_current, hold_current, print_time, force=False):
        if (
            new_current == self.actual_current
            and hold_current == self.req_hold_current
            and not force
        ):
            return
        self.req_hold_current = hold_current
        self.actual_current = new_current
        self.apply_current(print_time)


# Helper to configure StallGuard and CoolStep minimum velocity
def TMCVcoolthrsHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat("coolstep_threshold", None, minval=0.0)
    tcoolthrs = 0
    if velocity is not None:
        tcoolthrs = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("tcoolthrs", tcoolthrs)


# Helper to configure StallGuard and CoolStep maximum velocity and
# SpreadCycle-FullStepping (High velocity) mode threshold.
def TMCVhighHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat("high_velocity_threshold", None, minval=0.0)
    thigh = 0
    if velocity is not None:
        thigh = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("thigh", thigh)


######################################################################
# Phase stepping (SPI direct mode)
######################################################################


def validate_phase_stepping_config(config, stepper_section):
    sct = config.getfloat("stealthchop_threshold", 0.0, minval=0.0)
    if sct > 0.0:
        raise config.error(
            "phase_stepping=True is incompatible with stealthchop_threshold "
            "(StealthChop is bypassed in direct mode). Remove "
            "stealthchop_threshold from [%s] or disable phase_stepping."
            % config.get_name()
        )
    mres = stepper_section.getint("microsteps", 256)
    if mres != 256:
        raise config.error(
            "phase_stepping=True requires microsteps: 256; [%s] has "
            "microsteps: %d." % (stepper_section.get_name(), mres)
        )


class PhaseSpiArbiter:
    """Refcounted foreground ownership of a TMC SPI bus whose ISR streams
    XDIRECT coil writes. ISR transfers interleaving with a foreground
    register access shift the TMC response pipeline and corrupt the
    read-back, so every foreground transfer suspends the ISR writer for
    its duration. Suspends nest; the writer is re-armed only when the
    last suspension lifts and a driver is still in phase mode."""

    def __init__(self):
        self._count = 0
        self._enable_cmd = None
        self._disable_cmd = None
        self._active_cbs = []

    def register(self, enable_cmd, disable_cmd, active_cb):
        self._enable_cmd = enable_cmd
        self._disable_cmd = disable_cmd
        if active_cb not in self._active_cbs:
            self._active_cbs.append(active_cb)

    def _isr_active(self):
        return any(cb() for cb in self._active_cbs)

    def suspend(self):
        self._count += 1
        if self._count == 1 and self._disable_cmd is not None:
            if self._isr_active():
                self._disable_cmd.send([])

    def resume(self):
        assert self._count > 0, "unbalanced PhaseSpiArbiter.resume"
        self._count -= 1
        if self._count == 0 and self._enable_cmd is not None:
            if self._isr_active():
                self._enable_cmd.send([])


def lookup_phase_spi_arbiter(mcu):
    arbiter = getattr(mcu, "_tmc_phase_spi_arbiter", None)
    if arbiter is None:
        arbiter = PhaseSpiArbiter()
        mcu._tmc_phase_spi_arbiter = arbiter
    return arbiter


class TMCPhaseStepping:
    """Direct-mode (SPI-driven) phase stepping shared by SPI TMC drivers.

    The driver class provides: printer, name, fields, mcu_tmc (SPI),
    _mode_tracker, _echeck_helper, and PHASE_DIRECT_REGISTER — the name of
    its register at address 0x2D that carries the direct coil currents.
    """

    PHASE_DIRECT_REGISTER = None
    PHASE_JOG_MAX_PER_SAMPLE = 1
    PHASE_SETTLE_TIMEOUT = 0.5

    def _setup_phase_stepping(self, config):
        self._phase_stepping = False
        self._phase_bus_id = None
        self._phase_cs_pin_id = None
        self._phase_stepper_oid = None
        self._phase_axis_idx = None
        self._cached_mscnt = None
        self._phase_state_query = None
        self._phase_group = None
        stepper_name = " ".join(config.get_name().split()[1:])
        motor_section = "motor " + stepper_name
        if not config.has_section(motor_section):
            return False
        stepper_section = config.getsection(motor_section)
        if not stepper_section.getboolean("phase_stepping", False):
            return False
        validate_phase_stepping_config(config, stepper_section)
        self._phase_stepping = True
        return True

    def set_phase_stepper_oid(self, oid):
        self._phase_stepper_oid = oid

    def set_phase_group(self, tmcs):
        self._phase_group = tmcs

    def needs_pulse_mode_windows(self):
        return self._virtual_pin_helper.sensorless_homing_configured()

    def _switch_host_transport(self, axis_idx, transport):
        """Hand the lane between its two mcu bindings on the host side. The
        engine drains the outgoing transport, reconciles its executed position
        off the mcu and seeds the incoming one with it, so the host and the mcu
        change transport on the same position."""
        engine = self.printer.lookup_object("motion_engine", None)
        if engine is None:
            raise self.printer.command_error(
                "phase_stepping: the motion engine is required to switch "
                "transport for %s" % (self.name,)
            )
        handle = self._phase_mcu().get_engine_handle()
        if handle is None:
            raise self.printer.command_error(
                "phase_stepping: mcu of %s carries no motion engine handle, so "
                "its lane has no host transport to switch" % (self.name,)
            )
        structured_log.event(
            "phase_stepping",
            "transport_switch_request",
            msg="host transport switch requested",
            stepper=self.name,
            axis_idx=axis_idx,
            transport=transport,
        )
        engine.switch_axis_transport(handle, axis_idx, transport)

    def _phase_group_members(self):
        return self._phase_group or [self]

    def _in_phase_mode(self):
        return self._mode_tracker.mode == TMCModeTracker.PHASE_DIRECT

    def phase_stepping_active(self):
        return any(t._in_phase_mode() for t in self._phase_group_members())

    def _phase_spi_arbiter(self):
        return lookup_phase_spi_arbiter(self._phase_mcu())

    def _phase_mcu(self):
        return self.mcu_tmc.tmc_spi.spi.get_mcu()

    def _lookup_phase_commands(self):
        mcu_obj = self._phase_mcu()
        if self._phase_stepper_oid is None:
            raise self.printer.command_error(
                "phase_stepping: stepper oid not registered for %s "
                "(motion init_planner did not run?)" % (self.name,)
            )
        enable_spi = mcu_obj.lookup_command("kalico_phase_stepping_enable_spi")
        disable_spi = mcu_obj.lookup_command(
            "kalico_phase_stepping_disable_spi"
        )
        set_axis_mode = mcu_obj.lookup_command(
            "kalico_set_axis_mode axis_idx=%c mode=%c"
        )
        jog = mcu_obj.lookup_command(
            "kalico_phase_jog_to oid=%c target_phase=%hu"
            " max_microsteps_per_sample=%hu"
        )
        align = mcu_obj.lookup_command(
            "kalico_phase_align_to oid=%c target_phase=%hu"
        )
        if self._phase_state_query is None:
            self._phase_state_query = mcu_obj.lookup_query_command(
                "kalico_get_phase_state oid=%c",
                "motion_phase_state oid=%c axis_idx=%c mode=%c phase=%hu"
                " settled=%c",
                oid=self._phase_stepper_oid,
            )
        lookup_phase_spi_arbiter(mcu_obj).register(
            enable_spi, disable_spi, self.phase_stepping_active
        )
        return enable_spi, disable_spi, set_axis_mode, jog, align

    def _query_phase_state(self):
        return self._phase_state_query.send([self._phase_stepper_oid])

    def enter_phase_mode(self):
        for t in self._phase_group_members():
            if not t._in_phase_mode():
                t._enter_phase_mode_single()

    def _enter_phase_mode_single(self):
        self._mode_tracker.transition(
            (
                TMCModeTracker.DISABLED,
                TMCModeTracker.PULSE,
                TMCModeTracker.PHASE_DIRECT,
            ),
            TMCModeTracker.PHASE_DIRECT,
            "phase mode entry",
        )
        _enable_spi, _disable_spi, set_axis_mode, _jog, align = (
            self._lookup_phase_commands()
        )
        arbiter = self._phase_spi_arbiter()
        # Suspend ISR direct-register writes during our foreground SPI
        # traffic; resume() re-arms the writer since this member is
        # already tracked as phase-active.
        arbiter.suspend()
        # CHOPCONF (toff>0) must reach the chip before direct_mode: the
        # bootstrap charge pump depends on the chopper switching, and
        # direct_mode with toff=0 drains the bootstrap caps (uv_cp).
        self.mcu_tmc.set_register("CHOPCONF", self.fields.registers["CHOPCONF"])
        gconf_val = self.fields.override_register(
            "GCONF", {"en_pwm_mode": 0, "direct_mode": 1}
        )
        self.mcu_tmc.set_register("GCONF", gconf_val)
        mscnt = self.mcu_tmc.get_register("MSCNT") & 0x3FF
        self._cached_mscnt = mscnt
        angle = mscnt * 2.0 * math.pi / 1024.0
        coil_a = int(round(248.0 * math.cos(angle)))
        coil_b = int(round(248.0 * math.sin(angle)))
        xdirect_val = ((coil_b & 0xFFFF) << 16) | (coil_a & 0xFFFF)
        self.mcu_tmc.set_register(self.PHASE_DIRECT_REGISTER, xdirect_val)
        structured_log.event(
            "phase_stepping",
            "xdirect_preload",
            msg="XDIRECT preload",
            stepper=self.name,
            mscnt=mscnt,
            coil_a=coil_a,
            coil_b=coil_b,
        )
        state = self._query_phase_state()
        self._phase_axis_idx = state["axis_idx"]
        # Mode first, then the host transport: the pump anchors the sample
        # lane the moment the transport switches, and an anchored lane
        # ticking while the axis mode byte still reads Pulse is a
        # PhaseModeNotAvailable fault. The reverse order on exit keeps the
        # same invariant from the other side.
        set_axis_mode.send([self._phase_axis_idx, 1])
        state = self._query_phase_state()
        if state["mode"] != 1:
            raise self.printer.command_error(
                "phase mode entry: mcu did not apply Phase mode on %s "
                "(mode=%d)" % (self.name, state["mode"])
            )
        # set_axis_mode(1) also seeded the runtime's step count from the
        # classic executor mcu-side, so the align pins the phase against
        # the final count and the transport switch's host-side seed is a
        # no-op rather than a late shift that would drag the coils away
        # from the preload.
        align.send([self._phase_stepper_oid, mscnt])
        arbiter.resume()
        self._switch_host_transport(self._phase_axis_idx, TRANSPORT_PHASE)
        # The ISR's inline direct-register SPI writes corrupt concurrent
        # foreground register reads (false drv_err/uv_cp shutdowns), so the
        # periodic checks must stay off while phase mode is active.
        self._echeck_helper.stop_checks()
        structured_log.event(
            "phase_stepping",
            "enter",
            msg="phase mode entered",
            stepper=self.name,
            axis_idx=self._phase_axis_idx,
            mscnt=mscnt,
        )

    def exit_phase_mode(self):
        active = [t for t in self._phase_group_members() if t._in_phase_mode()]
        if not active:
            raise self.printer.command_error(
                "exit_phase_mode called but %s is not in phase mode"
                % (self.name,)
            )
        _enable_spi, _disable_spi, set_axis_mode, _jog, _align = (
            self._lookup_phase_commands()
        )
        for t in active:
            state = t._query_phase_state()
            if state["mode"] != 1:
                structured_log.event(
                    "phase_stepping",
                    "mode_desync",
                    level=logging.ERROR,
                    msg="phase mode bookkeeping desync",
                    stepper=t.name,
                    mcu_mode=state["mode"],
                )
                raise self.printer.command_error(
                    "phase mode bookkeeping desync on %s: host=phase mcu=%d"
                    % (t.name, state["mode"])
                )
        # All jogs are issued while the axis is still in Phase mode — the
        # mode flips to Pulse only once, after every motor in the group sits
        # on its cached MSCNT.
        for t in active:
            _e, _d, _s, t_jog, _a = t._lookup_phase_commands()
            t_jog.send(
                [
                    t._phase_stepper_oid,
                    t._cached_mscnt,
                    self.PHASE_JOG_MAX_PER_SAMPLE,
                ]
            )
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + self.PHASE_SETTLE_TIMEOUT
        for t in active:
            trail = []
            while True:
                state = t._query_phase_state()
                trail.append((state["phase"], state["settled"]))
                if state["settled"] and state["phase"] == t._cached_mscnt:
                    break
                if reactor.monotonic() > deadline:
                    structured_log.event(
                        "phase_stepping",
                        "jog_timeout",
                        level=logging.ERROR,
                        msg="phase handover jog did not settle",
                        stepper=t.name,
                        phase=state["phase"],
                        target=t._cached_mscnt,
                        trail=repr(trail[:10] + trail[-10:]),
                    )
                    raise self.printer.command_error(
                        "phase handover jog did not settle on %s "
                        "(phase=%d target=%d)"
                        % (t.name, state["phase"], t._cached_mscnt)
                    )
                reactor.pause(reactor.monotonic() + 0.005)
        arbiter = self._phase_spi_arbiter()
        arbiter.suspend()
        for t in active:
            t.mcu_tmc.set_register("GCONF", t.fields.registers.get("GCONF", 0))
        for axis_idx in sorted({t._phase_axis_idx for t in active}):
            self._switch_host_transport(axis_idx, TRANSPORT_PULSE)
            set_axis_mode.send([axis_idx, 0])
        for t in active:
            t._echeck_helper.start_checks()
            t._mode_tracker.transition(
                (TMCModeTracker.PHASE_DIRECT,),
                TMCModeTracker.PULSE,
                "phase mode exit",
            )
            structured_log.event(
                "phase_stepping",
                "exit",
                msg="phase mode exited (pulse stepping)",
                stepper=t.name,
                axis_idx=t._phase_axis_idx,
                mscnt=t._cached_mscnt,
            )
        arbiter.resume()

    def get_phase_config(self):
        if not self._phase_stepping:
            raise self.printer.config_error(
                "get_phase_config called on a %s without "
                "phase_stepping=True on the matching stepper section"
                % (type(self).__name__,)
            )
        if self._phase_bus_id is None or self._phase_cs_pin_id is None:
            self._phase_bus_id, self._phase_cs_pin_id = (
                self.mcu_tmc.tmc_spi.get_bus_and_cs_ids()
            )
        return (self._phase_bus_id, self._phase_cs_pin_id)

    def get_spi_oid(self):
        return self.mcu_tmc.tmc_spi.spi.oid
