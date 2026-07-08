# Interface to Klipper micro-controller code
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
import os
import zlib

from . import chelper, clocksync, msgproto, pins, serialhdl
from .extras.danger_options import get_danger_options
from .mcu_commands import CommandQueryWrapper, CommandWrapper
from .mcu_pins import (  # noqa: F401
    MAX_SCHEDULE_TICKS,
    MIN_SCHEDULE_LEAD,
    MCU_adc,
    MCU_digital_out,
    MCU_pwm,
)


class error(Exception):
    pass


# Minimum time host needs to get scheduled events queued into mcu
MIN_SCHEDULE_TIME = 0.100
# Maximum time all MCUs can internally schedule into the future.
# Directly caused by the limitation of MAX_SCHEDULE_TICKS.
MAX_NOMINAL_DURATION = 3.0


######################################################################
# Main MCU class
######################################################################


class MCU:
    error = error

    def __init__(self, config, clocksync):
        self._init_identity(config, clocksync)
        self._init_serial_port(config)
        self._init_restart_state(config)
        self._init_config_state()
        self._init_non_critical(config)
        self._init_event_handlers()

    def _init_identity(self, config, clocksync):
        self._config = config
        self._printer = printer = config.get_printer()
        self.danger_options = printer.lookup_object("danger_options")
        self.gcode = printer.lookup_object("gcode")
        self._clocksync = clocksync
        self._reactor = printer.get_reactor()
        self._name = config.get_name()
        declared_via_mcu_section = self._name == "mcu" or self._name.startswith(
            "mcu "
        )
        self._expect_native = declared_via_mcu_section
        if self._name.startswith("mcu "):
            self._name = self._name[4:]
        self._motion_engine = printer.lookup_object("motion_engine", None)
        self._engine_handle = None

    def _init_serial_port(self, config):
        wp = "mcu '%s': " % (self._name)
        self._serial = serialhdl.SerialReader(
            self._reactor, warn_prefix=wp, mcu=self
        )
        self._baud = 0
        if config.get("canbus_uuid", None) is not None:
            raise config.error(
                "CAN bus is not supported on the engine motion engine"
                " (mcu '%s' sets canbus_uuid)" % (self._name,)
            )
        self._serialport = config.get("serial")
        if not (
            self._serialport.startswith("/dev/rpmsg_")
            or self._serialport.startswith("/tmp/klipper_host_")
        ):
            self._baud = config.getint("baud", 250000, minval=2400)

    def _init_restart_state(self, config):
        restart_methods = [None, "arduino", "cheetah", "command", "rpi_usb"]
        self._restart_method = "command"
        if self._baud:
            self._restart_method = config.getchoice(
                "restart_method", restart_methods, None
            )
        self._reset_cmd = self._config_reset_cmd = None
        self._is_mcu_engine = False
        self._emergency_stop_cmd = None
        self._is_shutdown = self._is_timeout = False
        self._shutdown_clock = 0
        self._shutdown_msg = ""

    def _init_config_state(self):
        self._printer.lookup_object("pins").register_chip(self._name, self)
        self._oid_count = 0
        self._config_callbacks = []
        self._config_cmds = []
        self._restart_cmds = []
        self._init_cmds = []
        self._mcu_freq = 0.0
        self._reserved_move_slots = 0
        self._flush_callbacks = []
        self._get_status_info = {}
        self._stats_sumsq_base = 0.0
        self._mcu_tick_avg = 0.0
        self._mcu_tick_stddev = 0.0
        self._mcu_tick_awake = 0.0
        self._config_crc = 0

    def _init_non_critical(self, config):
        self.is_non_critical = config.getboolean("is_non_critical", False)
        if self.is_non_critical and self.get_name() == "mcu":
            raise error("Primary MCU cannot be marked as non-critical!")
        if self.is_non_critical:
            self.non_critical_recon_timer = self._reactor.register_timer(
                self.non_critical_recon_event
            )
        self.non_critical_disconnected = False
        self._non_critical_reconnect_event_name = (
            f"danger:non_critical_mcu_{self.get_name()}:reconnected"
        )
        self._non_critical_disconnect_event_name = (
            f"danger:non_critical_mcu_{self.get_name()}:disconnected"
        )
        self.reconnect_interval = (
            config.getfloat("reconnect_interval", 2.0) + 0.12
        )  # add small change to not collide with other events
        self._cached_init_state = False
        self._oid_count_post_inits = 0
        self._config_cmds_post_inits = []
        self._init_cmds_post_inits = []
        self._restart_cmds_post_inits = []

    def _init_event_handlers(self):
        self._printer.register_event_handler(
            "klippy:firmware_restart", self._firmware_restart
        )
        self._printer.register_event_handler(
            "klippy:mcu_identify", self._mcu_identify
        )
        self._printer.register_event_handler("klippy:connect", self._connect)
        self._printer.register_event_handler("klippy:shutdown", self._shutdown)
        self._printer.register_event_handler(
            "klippy:disconnect", self._disconnect
        )
        self._printer.register_event_handler("klippy:ready", self._ready)

    # Serial callbacks
    def _handle_mcu_stats(self, params):
        count = params["count"]
        tick_sum = params["sum"]
        c = 1.0 / (count * self._mcu_freq)
        self._mcu_tick_avg = tick_sum * c
        tick_sumsq = params["sumsq"] * self._stats_sumsq_base
        diff = count * tick_sumsq - tick_sum**2
        self._mcu_tick_stddev = c * math.sqrt(max(0.0, diff))
        self._mcu_tick_awake = tick_sum / self._mcu_freq

    def _handle_shutdown(self, params):
        if self._is_shutdown:
            return
        self._is_shutdown = True
        clock = params.get("clock")
        if clock is not None:
            self._shutdown_clock = self.clock32_to_clock64(clock)
        self._shutdown_msg = msg = params["static_string_id"]
        if get_danger_options().log_shutdown_info:
            logging.info(
                "MCU '%s' %s: %s\n%s\n%s\n%s",
                self._name,
                params["#name"],
                self._shutdown_msg,
                self.dump_debug(),
                self._clocksync.dump_debug(),
                self._serial.dump_debug(),
            )
        prefix = "MCU '%s' shutdown: " % (self._name,)
        is_latched_shutdown = params["#name"] == "is_shutdown"
        if is_latched_shutdown:
            prefix = "Previous MCU '%s' shutdown: " % (self._name,)
            self._check_restart(
                "MCU '%s' latched in shutdown state at connect" % (self._name,)
            )

        append_msgs = []
        if (
            msg.startswith("ADC out of range")
            or msg.startswith("Thermocouple reader fault")
        ) and not get_danger_options().temp_ignore_limits:
            pheaters = self._printer.lookup_object("heaters")
            heaters = [
                pheaters.lookup_heater(n) for n in pheaters.available_heaters
            ]
            for heater in heaters:
                if hasattr(heater, "is_adc_faulty") and heater.is_adc_faulty():
                    append_msgs.append(
                        {
                            "heater": heater.name,
                            "last_temp": "{:.2f}".format(heater.last_temp),
                            "min_temp": heater.min_temp,
                            "max_temp": heater.max_temp,
                        }
                    )
            sensor_names = [
                sensor
                for sensor in self._printer.objects
                if (
                    sensor.startswith("temperature_sensor")
                    or sensor.startswith("temperature_fan")
                )
            ]
            for sensor_name in sensor_names:
                sensor = self._printer.lookup_object(sensor_name)
                if hasattr(sensor, "is_adc_faulty") and sensor.is_adc_faulty():
                    append_msgs.append(
                        {
                            sensor_name.split(" ")[0]: sensor.name,
                            "last_temp": "{:.2f}".format(sensor.last_temp),
                            "min_temp": sensor.min_temp,
                            "max_temp": sensor.max_temp,
                        }
                    )

        self._printer.invoke_async_shutdown(
            prefix + msg + error_help(msg=msg, append_msgs=append_msgs)
        )

    def _handle_starting(self, params):
        if not self._is_shutdown and not self.is_non_critical:
            self._printer.invoke_async_shutdown(
                "MCU '%s' spontaneous restart" % (self._name,)
            )

    # Connection phase
    def _check_restart(self, reason):
        start_reason = self._printer.get_start_args().get("start_reason")
        if start_reason == "firmware_restart":
            return
        logging.info(
            "Attempting automated MCU '%s' restart: %s", self._name, reason
        )
        self._printer.request_exit("firmware_restart")
        self._reactor.pause(self._reactor.monotonic() + 2.000)
        raise error("Attempt MCU '%s' restart failed" % (self._name,))

    def _connect_file(self, pace=False):
        # In a debugging mode.  Open debug output file and read data dictionary
        start_args = self._printer.get_start_args()
        if self._name == "mcu":
            out_fname = start_args.get("debugoutput")
            dict_fname = start_args.get("dictionary")
        else:
            out_fname = start_args.get("debugoutput") + "-" + self._name
            dict_fname = start_args.get("dictionary_" + self._name)
        outfile = open(out_fname, "wb")
        dfile = open(dict_fname, "rb")
        dict_data = dfile.read()
        dfile.close()
        self._serial.connect_file(outfile, dict_data)
        self._clocksync.connect_file(self._serial, pace)
        # Handle pacing
        if not pace:

            def dummy_estimated_print_time(eventtime):
                return 0.0

            self.estimated_print_time = dummy_estimated_print_time

    def handle_non_critical_disconnect(self):
        self.non_critical_disconnected = True
        self._clocksync.disconnect()
        self._disconnect()
        self._reactor.update_timer(
            self.non_critical_recon_timer, self._reactor.NOW
        )
        self._printer.send_event(self._non_critical_disconnect_event_name)
        self.gcode.respond_info(f"mcu: '{self._name}' disconnected!", log=True)

    def non_critical_recon_event(self, eventtime):
        success = self.recon_mcu()
        if success:
            self.gcode.respond_info(
                f"mcu: '{self._name}' reconnected!", log=True
            )
            return self._reactor.NEVER
        else:
            return eventtime + self.reconnect_interval

    def _send_config(self, prev_crc):
        if not self._cached_init_state:
            # first time config, we haven't created callback oids yet
            # so save the oid count for state reset later
            self._oid_count_post_inits = self._oid_count
            self._config_cmds_post_inits = self._config_cmds.copy()
            self._init_cmds_post_inits = self._init_cmds.copy()
            self._restart_cmds_post_inits = self._restart_cmds.copy()
            self._cached_init_state = True
        # Build config commands
        for cb in self._config_callbacks:
            cb()

        local_config_cmds = self._config_cmds.copy()

        local_config_cmds.insert(
            0, "allocate_oids count=%d" % (self._oid_count,)
        )

        # Resolve pin names
        ppins = self._printer.lookup_object("pins")
        pin_resolver = ppins.get_pin_resolver(self._name)
        for cmdlist in (local_config_cmds, self._restart_cmds, self._init_cmds):
            for i, cmd in enumerate(cmdlist):
                cmdlist[i] = pin_resolver.update_command(cmd)
        # Calculate config CRC
        encoded_config = "\n".join(local_config_cmds).encode()
        self._config_crc = zlib.crc32(encoded_config) & 0xFFFFFFFF
        local_config_cmds.append("finalize_config crc=%d" % (self._config_crc,))
        if prev_crc is not None and self._config_crc != prev_crc:
            self._check_restart("CRC mismatch")
            raise error("MCU '%s' CRC does not match config" % (self._name,))
        # Transmit config messages (if needed)
        self.register_response(self._handle_starting, "starting")
        try:
            if prev_crc is None:
                logging.info(
                    "Sending MCU '%s' printer configuration...", self._name
                )
                for c in local_config_cmds:
                    logging.info("[config-send] mcu=%s cmd=%s", self._name, c)
                    self._serial.send(c)
            else:
                for c in self._restart_cmds:
                    logging.info(
                        "[config-send-restart] mcu=%s cmd=%s", self._name, c
                    )
                    self._serial.send(c)
            # Transmit init messages
            for c in self._init_cmds:
                self._serial.send(c)
        except msgproto.enumeration_error as e:
            enum_name, enum_value = e.get_enum_params()
            if enum_name == "pin":
                # Raise pin name errors as a config error (not a protocol error)
                raise self._printer.config_error(
                    "Pin '%s' is not a valid pin name on mcu '%s'"
                    % (enum_value, self._name)
                )
            raise

    def _recover_latched_peripheral_shutdown(self, get_config_cmd, exc):
        is_motion_mcu = (
            "runtime_reset" in self._serial.get_msgparser().messages_by_name
        )
        if is_motion_mcu:
            raise error(
                "MCU '%s' is latched in shutdown and requires a power"
                " cycle (clear_shutdown cannot reset its timer and runtime"
                " state): %s" % (self._name, exc)
            )
        logging.info(
            "MCU '%s' latched the shutdown broadcast from a previous host"
            " session; sending clear_shutdown and retrying: %s",
            self._name,
            exc,
        )
        self._serial.send("clear_shutdown")
        config_params = get_config_cmd.send()
        self._is_shutdown = False
        self._shutdown_msg = ""
        return config_params

    def _send_get_config(self):
        get_config_cmd = self.lookup_query_command(
            "get_config",
            "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
        )
        if self.is_fileoutput():
            return {"is_config": 0, "move_count": 500, "crc": 0}
        try:
            config_params = get_config_cmd.send()
        except Exception as e:
            if "shutdown state" not in str(e):
                raise
            config_params = self._recover_latched_peripheral_shutdown(
                get_config_cmd, e
            )
        if self._is_shutdown:
            raise error(
                "MCU '%s' error during config: %s"
                % (self._name, self._shutdown_msg)
            )
        if config_params["is_shutdown"]:
            self._check_restart(
                "MCU '%s' was in shutdown state at config time" % (self._name,)
            )
            raise error(
                "Can not update MCU '%s' config as it is shutdown"
                % (self._name,)
            )
        return config_params

    def _log_info(self):
        msgparser = self._serial.get_msgparser()
        app = msgparser.get_app_info()
        message_count = len(msgparser.get_messages())
        version, build_versions = msgparser.get_version_info()
        log_info = [
            f"Loaded MCU '{self._name}' {message_count} commands ({app} {version} / {build_versions})",
            "MCU '%s' config: %s"
            % (
                self._name,
                " ".join(
                    ["%s=%s" % (k, v) for k, v in self.get_constants().items()]
                ),
            ),
        ]
        return "\n".join(log_info)

    def recon_mcu(self):
        res = self._mcu_identify()
        if not res:
            return False
        self.reset_to_initial_state()
        self.non_critical_disconnected = False
        self._connect()
        self._printer.send_event(self._non_critical_reconnect_event_name)
        return True

    def reset_to_initial_state(self):
        if self._cached_init_state:
            self._oid_count = self._oid_count_post_inits
            self._config_cmds = self._config_cmds_post_inits.copy()
            self._init_cmds = self._init_cmds_post_inits.copy()
            self._restart_cmds = self._restart_cmds_post_inits.copy()
        self._reserved_move_slots = 0

    def _connect(self):
        if self.non_critical_disconnected:
            self._reactor.update_timer(
                self.non_critical_recon_timer,
                self._reactor.NOW + self.reconnect_interval,
            )
            return
        config_params = self._send_get_config()
        if not config_params["is_config"]:
            if self._restart_method == "rpi_usb":
                # Only configure mcu after usb power reset
                self._check_restart("full reset before config")
            # Not configured - send config and issue get_config again
            self._send_config(None)
            config_params = self._send_get_config()
            if not config_params["is_config"] and not self.is_fileoutput():
                raise error("Unable to configure MCU '%s'" % (self._name,))
        else:
            # if the mcu crc match the initial crc, the mcu lost comms but not
            # power and is reconnecting
            if not self._config_crc == config_params["crc"]:
                start_reason = self._printer.get_start_args().get(
                    "start_reason"
                )
                if start_reason == "firmware_restart":
                    raise error(
                        "Failed automated reset of MCU '%s'" % (self._name,)
                    )
                # Already configured - send init commands
                self._send_config(config_params["crc"])
        move_count = config_params["move_count"]
        if move_count < self._reserved_move_slots:
            raise error("Too few moves available on MCU '%s'" % (self._name,))
        # Log config information
        move_msg = "Configured MCU '%s' (%d moves)" % (self._name, move_count)
        logging.info(move_msg)
        log_info = self._log_info() + "\n" + move_msg
        self._printer.set_rollover_info(self._name, log_info, log=False)

    def _check_serial_exists(self):
        rts = self._restart_method != "cheetah"
        return self._serial.check_connect(self._serialport, self._baud, rts)

    def _mcu_identify(self):
        if not self._identify_check_serial_available():
            return False
        self._identify_connect_serial()
        self._identify_log_and_reserve_pins()
        self._identify_set_mcu_freq()
        self._identify_lookup_commands_and_restart_method()
        self._identify_record_version_info()
        self._identify_register_responses()
        self._identify_setup_motion_engine()
        return True

    def _identify_check_serial_available(self):
        if self.is_non_critical and not self._check_serial_exists():
            self.non_critical_disconnected = True
            if self.is_non_critical:
                self._get_status_info["non_critical_disconnected"] = True
            return False
        else:
            self.non_critical_disconnected = False
            if self.is_non_critical:
                self._get_status_info["non_critical_disconnected"] = False
            return True

    def _identify_connect_serial(self):
        if self.is_fileoutput():
            self._connect_file()
        else:
            resmeth = self._restart_method
            if resmeth == "rpi_usb" and not os.path.exists(self._serialport):
                # Try toggling usb power
                self._check_restart("enable power")
            try:
                if self._baud:
                    # Cheetah boards require RTS to be deasserted
                    # else a reset will trigger the built-in bootloader.
                    rts = resmeth != "cheetah"
                    self._serial.connect_uart(self._serialport, self._baud, rts)
                else:
                    self._serial.connect_pipe(self._serialport)
                self._clocksync.connect(self._serial)
            except serialhdl.error as e:
                raise error(str(e))

    def _identify_log_and_reserve_pins(self):
        if get_danger_options().log_startup_info:
            logging.info(self._log_info())
        ppins = self._printer.lookup_object("pins")
        pin_resolver = ppins.get_pin_resolver(self._name)
        for cname, value in self.get_constants().items():
            if cname.startswith("RESERVE_PINS_"):
                for pin in value.split(","):
                    pin_resolver.reserve_pin(pin, cname[13:])

    def _identify_set_mcu_freq(self):
        self._mcu_freq = self.get_constant_float("CLOCK_FREQ")
        if MAX_NOMINAL_DURATION * self._mcu_freq > MAX_SCHEDULE_TICKS:
            max_possible = MAX_SCHEDULE_TICKS / self._mcu_freq
            raise error(
                "Too high clock speed for MCU '%s' " % (self._name,)
                + "to be able to resolve a maximum nominal duration "
                + "of %ds. " % (MAX_NOMINAL_DURATION,)
                + "Max possible duration: %ds" % (max_possible,)
            )

    def _identify_lookup_commands_and_restart_method(self):
        self._stats_sumsq_base = self.get_constant_float("STATS_SUMSQ_BASE")
        self._emergency_stop_cmd = self.lookup_command("emergency_stop")
        self._reset_cmd = self.try_lookup_command("reset")
        self._config_reset_cmd = self.try_lookup_command("config_reset")
        ext_only = self._reset_cmd is None and self._config_reset_cmd is None
        if ext_only:
            msgparser = self._serial.get_msgparser()
            all_cmds = sorted(msgparser.messages_by_name.keys())
            logging.warning(
                "MCU '%s' has no reset/config_reset command. "
                "Available commands (%d): %s",
                self._name,
                len(all_cmds),
                ", ".join(c for c in all_cmds if "reset" in c.lower())
                or "(none with 'reset')",
            )
        msgparser = self._serial.get_msgparser()
        mbaud = msgparser.get_constant("SERIAL_BAUD", None)
        if self._restart_method is None and mbaud is None and not ext_only:
            self._restart_method = "command"
        if msgparser.get_constant("CANBUS_BRIDGE", 0):
            self._is_mcu_engine = True
            self._printer.register_event_handler(
                "klippy:firmware_restart", self._firmware_restart_engine
            )

    def _identify_record_version_info(self):
        msgparser = self._serial.get_msgparser()
        app = msgparser.get_app_info()
        version, build_versions = msgparser.get_version_info()
        self._get_status_info["app"] = app
        self._get_status_info["mcu_version"] = version
        self._get_status_info["mcu_build_versions"] = build_versions
        self._get_status_info["mcu_constants"] = msgparser.get_constants()
        if app in ("Klipper", "Danger-Klipper"):
            pconfig = self._printer.lookup_object("configfile")
            pconfig.runtime_warning(
                f"MCU {self._name!r} currently has firmware compiled for {app} (version {version})."
                f" It is recommended to re-flash for best compatiblity with Kalico"
            )

    def _identify_register_responses(self):
        self.register_response(self._handle_shutdown, "shutdown")
        self.register_response(self._handle_shutdown, "is_shutdown")
        self.register_response(self._handle_mcu_stats, "stats")

    def _identify_setup_motion_engine(self):
        msgparser = self._serial.get_msgparser()
        raw_dict = msgparser.get_raw_data_dictionary()
        if self._motion_engine is not None:
            if raw_dict:
                if isinstance(raw_dict, str):
                    raw_dict = raw_dict.encode("utf-8")
                self._motion_engine.set_msgproto_dict(raw_dict)
            if self._engine_handle is None:
                self._engine_handle = self._motion_engine.claim_mcu(
                    self._name,
                    self._serialport or "",
                    int(self._baud or 0),
                )
            engine = self._motion_engine
            handle = self._engine_handle
            if not self._mcu_freq:
                raise error(
                    "MCU '%s': CLOCK_FREQ unknown at engine claim time"
                    % (self._name,)
                )
            self._motion_engine.set_nominal_clock_freq(
                handle, int(self._mcu_freq)
            )

            reactor = self._reactor

            def _engine_clock_est_cb(
                freq, offset, last_clock, b=engine, h=handle, r=reactor
            ):
                host_now_raw = r.monotonic()
                try:
                    b.set_clock_est(
                        h,
                        float(freq),
                        float(offset),
                        int(last_clock),
                        host_now_raw,
                    )
                except Exception:
                    logging.exception("motion_engine: set_clock_est failed")

            self._clocksync.set_clock_est_callback(_engine_clock_est_cb)

    def _ready(self):
        if self.is_fileoutput():
            return
        # Check that reported mcu frequency is in range
        mcu_freq = self._mcu_freq
        systime = self._reactor.monotonic()
        get_clock = self._clocksync.get_clock
        calc_freq = get_clock(systime + 1) - get_clock(systime)
        freq_diff = abs(mcu_freq - calc_freq)
        mcu_freq_mhz = int(mcu_freq / 1000000.0 + 0.5)
        calc_freq_mhz = int(calc_freq / 1000000.0 + 0.5)
        if freq_diff > mcu_freq * 0.01 and mcu_freq_mhz != calc_freq_mhz:
            pconfig = self._printer.lookup_object("configfile")
            msg = "MCU '%s' configured for %dMhz but running at %dMhz!" % (
                self._name,
                mcu_freq_mhz,
                calc_freq_mhz,
            )
            pconfig.runtime_warning(msg)

    # Config creation helpers
    def setup_pin(self, pin_type, pin_params):
        pcs = {
            "digital_out": MCU_digital_out,
            "pwm": MCU_pwm,
            "adc": MCU_adc,
        }
        if pin_type not in pcs:
            raise pins.error("pin type %s not supported on mcu" % (pin_type,))
        return pcs[pin_type](self, pin_params)

    def create_oid(self):
        self._oid_count += 1
        return self._oid_count - 1

    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        if is_init:
            self._init_cmds.append(cmd)
        elif on_restart:
            self._restart_cmds.append(cmd)
        else:
            self._config_cmds.append(cmd)

    def get_query_slot(self, oid):
        slot = self.seconds_to_clock(oid * 0.01)
        t = int(self.estimated_print_time(self._reactor.monotonic()) + 1.5)
        return self.print_time_to_clock(t) + slot

    def seconds_to_clock(self, time):
        return int(time * self._mcu_freq)

    def min_schedule_time(self):
        return MIN_SCHEDULE_TIME

    def max_nominal_duration(self):
        return MAX_NOMINAL_DURATION

    # Wrapper functions
    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def get_non_critical_reconnect_event_name(self):
        return self._non_critical_reconnect_event_name

    def get_non_critical_disconnect_event_name(self):
        return self._non_critical_disconnect_event_name

    def register_response(self, cb, msg, oid=None):
        self._serial.register_response(cb, msg, oid)

    def lookup_command(self, msgformat):
        return CommandWrapper(self._serial, msgformat)

    def lookup_query_command(self, msgformat, respformat, oid=None):
        return CommandQueryWrapper(
            self._serial,
            msgformat,
            respformat,
            oid,
            self._printer.command_error,
        )

    def try_lookup_command(self, msgformat):
        try:
            return self.lookup_command(msgformat)
        except self._serial.get_msgparser().error as e:
            logging.info(
                "MCU '%s' try_lookup_command('%s') failed: %s (available: %s)",
                self._name,
                msgformat,
                e,
                ", ".join(
                    sorted(
                        self._serial.get_msgparser().messages_by_name.keys()
                    )[:20]
                )
                + (
                    "..."
                    if len(self._serial.get_msgparser().messages_by_name) > 20
                    else ""
                ),
            )
            return None

    def get_enumerations(self):
        return self._serial.get_msgparser().get_enumerations()

    def get_constants(self):
        return self._serial.get_msgparser().get_constants()

    def get_constant_float(self, name):
        return self._serial.get_msgparser().get_constant_float(name)

    def print_time_to_clock(self, print_time):
        return self._clocksync.print_time_to_clock(print_time)

    def clock_to_print_time(self, clock):
        return self._clocksync.clock_to_print_time(clock)

    def estimated_print_time(self, eventtime):
        return self._clocksync.estimated_print_time(eventtime)

    def clock32_to_clock64(self, clock32):
        return self._clocksync.clock32_to_clock64(clock32)

    # Restarts
    def _disconnect(self):
        self._serial.disconnect()

    def _shutdown(self, force=False):
        if self._emergency_stop_cmd is None or (
            self._is_shutdown and not force
        ):
            return
        self._emergency_stop_cmd.send()

    def _restart_arduino(self):
        logging.info("Attempting MCU '%s' reset", self._name)
        self._disconnect()
        serialhdl.arduino_reset(self._serialport, self._reactor)

    def _restart_cheetah(self):
        logging.info("Attempting MCU '%s' Cheetah-style reset", self._name)
        self._disconnect()
        serialhdl.cheetah_reset(self._serialport, self._reactor)

    def _restart_via_command(self):
        if (
            self._reset_cmd is None and self._config_reset_cmd is None
        ) or not self._clocksync.is_active():
            logging.info(
                "Unable to issue reset command on MCU '%s'", self._name
            )
            return
        try:
            if self._motion_engine is not None:
                self._motion_engine.engine_mark_expected_disconnect(
                    self._engine_handle
                )
        except Exception:
            logging.exception(
                "MCU '%s' engine_mark_expected_disconnect failed"
                " (continuing with reset)",
                self._name,
            )
        if self._reset_cmd is None:
            # Attempt reset via config_reset command
            logging.info("Attempting MCU '%s' config_reset command", self._name)
            self._is_shutdown = True
            self._shutdown(force=True)
            self._reactor.pause(self._reactor.monotonic() + 0.015)
            self._config_reset_cmd.send()
        else:
            # Attempt reset via reset command
            logging.info("Attempting MCU '%s' reset command", self._name)
            self._reset_cmd.send()
        self._reactor.pause(self._reactor.monotonic() + 0.015)
        self._disconnect()

    def _restart_rpi_usb(self):
        logging.info("Attempting MCU '%s' reset via rpi usb power", self._name)
        self._disconnect()
        chelper.run_hub_ctrl(0)
        self._reactor.pause(self._reactor.monotonic() + 2.0)
        chelper.run_hub_ctrl(1)

    def _firmware_restart(self, force=False):
        logging.info(
            "[firmware-restart-trace] mcu=%s force=%s _is_mcu_engine=%s "
            "non_critical_disconnected=%s _restart_method=%s "
            "_reset_cmd_present=%s clocksync_active=%s",
            self._name,
            force,
            self._is_mcu_engine,
            self.non_critical_disconnected,
            self._restart_method,
            self._reset_cmd is not None,
            self._clocksync.is_active()
            if self._clocksync is not None
            else "no-clocksync",
        )
        if (
            self._is_mcu_engine and not force
        ) or self.non_critical_disconnected:
            return
        if self._restart_method == "rpi_usb":
            self._restart_rpi_usb()
        elif self._restart_method == "command":
            self._restart_via_command()
        elif self._restart_method == "cheetah":
            self._restart_cheetah()
        else:
            self._restart_arduino()

    def _firmware_restart_engine(self):
        self._firmware_restart(True)

    # Move queue tracking
    def request_move_queue_slot(self):
        self._reserved_move_slots += 1

    def register_flush_callback(self, callback):
        self._flush_callbacks.append(callback)

    def flush_moves(self, print_time, clear_history_time):
        clock = self.print_time_to_clock(print_time)
        if clock < 0:
            return
        for cb in self._flush_callbacks:
            cb(print_time, clock)

    def check_active(self, print_time, eventtime):
        self._clocksync.calibrate_clock(print_time, eventtime)
        if (
            self._clocksync.is_active()
            or self.is_fileoutput()
            or self._is_timeout
        ):
            return
        if self.is_non_critical:
            self.handle_non_critical_disconnect()
            return
        self._is_timeout = True
        logging.info(
            "Timeout with MCU '%s' (eventtime=%f)", self._name, eventtime
        )
        if get_danger_options().log_shutdown_info:
            logging.info(
                "MCU '%s' disconnected: Timeout\n%s\n%s\n%s",
                self._name,
                self.dump_debug(),
                self._clocksync.dump_debug(),
                self._serial.dump_debug(),
            )
        self._printer.invoke_shutdown(
            "Lost communication with MCU '%s'" % (self._name,)
        )

    # Misc external commands
    def is_fileoutput(self):
        return self._printer.get_start_args().get("debugoutput") is not None

    def is_shutdown(self):
        return self._is_shutdown

    def get_shutdown_clock(self):
        return self._shutdown_clock

    def get_status(self, eventtime=None):
        return dict(self._get_status_info)

    def dump_debug(self):
        out = []
        cmds = self._config_cmds

        out.append(
            f"Dumping config commands, {len(cmds)} commands, {self._oid_count} oids"
        )
        for idx, cmd in enumerate(cmds):
            out.append(f"Config {idx}: {cmd}")

        return "\n".join(out)

    def stats(self, eventtime):
        load = "mcu_awake=%.03f mcu_task_avg=%.06f mcu_task_stddev=%.06f" % (
            self._mcu_tick_awake,
            self._mcu_tick_avg,
            self._mcu_tick_stddev,
        )
        stats = " ".join(
            [
                load,
                self._serial.stats(eventtime),
                self._clocksync.stats(eventtime),
            ]
        )
        parts = [s.split("=", 1) for s in stats.split()]
        last_stats = {k: (float(v) if "." in v else int(v)) for k, v in parts}
        self._get_status_info["last_stats"] = last_stats
        return False, "%s: %s" % (self._name, stats)


Common_MCU_errors = {
    ("Timer too close",): """
This often indicates the host computer is overloaded. Check
for other processes consuming excessive CPU time, high swap
usage, disk errors, overheating, unstable voltage, or
similar system problems on the host computer.""",
    ("Missed scheduling of next ",): """
This is generally indicative of an intermittent
communication failure between micro-controller and host.""",
    (
        "ADC out of range",
        "Thermocouple reader fault",
    ): """
This generally occurs when a heater temperature exceeds
its configured min_temp or max_temp.""",
    (
        "Rescheduled timer in the past",
        "Stepper too far in past",
    ): """
This generally occurs when the micro-controller has been
requested to step at a rate higher than it is capable of
obtaining.""",
    ("Command request",): """
This generally occurs in response to an M112 G-Code command
or in response to an internal error in the host software.""",
}


def error_help(msg, append_msgs=None):
    if append_msgs is None:
        append_msgs = []
    for prefixes, help_msg in Common_MCU_errors.items():
        for prefix in prefixes:
            if msg.startswith(prefix):
                if append_msgs:
                    for append in append_msgs:
                        line = append
                        if isinstance(append, dict):
                            line = ", ".join(
                                [
                                    f"{str(k)}: {str(v)}"
                                    for k, v in append.items()
                                ]
                            )
                        help_msg = "\n".join([help_msg, str(line)])
                return help_msg
    return ""


def add_printer_objects(config):
    printer = config.get_printer()
    reactor = printer.get_reactor()
    mainsync = clocksync.ClockSync(reactor)
    printer.add_object("mcu", MCU(config.getsection("mcu"), mainsync))
    for s in config.get_prefix_sections("mcu "):
        printer.add_object(
            s.section, MCU(s, clocksync.SecondarySync(reactor, mainsync))
        )


def get_printer_mcu(printer, name):
    if name == "mcu":
        return printer.lookup_object(name)
    return printer.lookup_object("mcu " + name)
