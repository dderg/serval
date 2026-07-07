# Interface to Klipper micro-controller code
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import chelper


class error(Exception):
    pass


class MCU_trsync:
    REASON_ENDSTOP_HIT = 1
    REASON_HOST_REQUEST = 2
    REASON_PAST_END_TIME = 3
    REASON_COMMS_TIMEOUT = 4

    def __init__(self, mcu, trdispatch):
        self._mcu = mcu
        self._trdispatch = trdispatch
        self._reactor = mcu.get_printer().get_reactor()
        self._steppers = []
        self._trdispatch_mcu = None
        self._oid = mcu.create_oid()
        self._trsync_start_cmd = self._trsync_set_timeout_cmd = None
        self._trsync_trigger_cmd = self._trsync_query_cmd = None
        self._stepper_stop_cmd = None
        self._trigger_completion = None
        self._home_end_clock = None
        mcu.register_config_callback(self._build_config)
        printer = mcu.get_printer()
        printer.register_event_handler("klippy:shutdown", self._shutdown)

    def get_mcu(self):
        return self._mcu

    def get_oid(self):
        return self._oid

    def add_stepper(self, stepper):
        if stepper in self._steppers:
            return
        self._steppers.append(stepper)

    def get_steppers(self):
        return list(self._steppers)

    def _build_config(self):
        mcu = self._mcu
        # Setup config
        mcu.add_config_cmd("config_trsync oid=%d" % (self._oid,))
        mcu.add_config_cmd(
            "trsync_start oid=%d report_clock=0 report_ticks=0 expire_reason=0"
            % (self._oid,),
            on_restart=True,
        )
        # Lookup commands
        self._trsync_start_cmd = mcu.lookup_command(
            "trsync_start oid=%c report_clock=%u report_ticks=%u"
            " expire_reason=%c",
        )
        self._trsync_set_timeout_cmd = mcu.lookup_command(
            "trsync_set_timeout oid=%c clock=%u"
        )
        self._trsync_trigger_cmd = mcu.lookup_command(
            "trsync_trigger oid=%c reason=%c"
        )
        self._trsync_query_cmd = mcu.lookup_query_command(
            "trsync_trigger oid=%c reason=%c",
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=self._oid,
        )
        self._stepper_stop_cmd = mcu.lookup_command(
            "stepper_stop_on_trigger oid=%c trsync_oid=%c"
        )
        self._trdispatch_mcu = None

    def _shutdown(self):
        tc = self._trigger_completion
        if tc is not None:
            self._trigger_completion = None
            tc.complete(False)

    def _handle_trsync_state(self, params):
        logging.info(
            "[trsync-diag] _handle_trsync_state mcu=%s oid=%d "
            "can_trigger=%s trigger_reason=%s clock=%s "
            "has_completion=%s",
            self._mcu._name,
            self._oid,
            params.get("can_trigger"),
            params.get("trigger_reason"),
            params.get("clock"),
            self._trigger_completion is not None,
        )
        if not params["can_trigger"]:
            tc = self._trigger_completion
            if tc is not None:
                self._trigger_completion = None
                reason = params["trigger_reason"]
                is_failure = reason >= self.REASON_COMMS_TIMEOUT
                logging.info(
                    "[trsync-diag] completing trigger: reason=%d is_failure=%s",
                    reason,
                    is_failure,
                )
                self._reactor.async_complete(tc, is_failure)
        elif self._home_end_clock is not None:
            clock = self._mcu.clock32_to_clock64(params["clock"])
            if clock >= self._home_end_clock:
                self._home_end_clock = None
                self._trsync_trigger_cmd.send(
                    [self._oid, self.REASON_PAST_END_TIME]
                )

    def start(
        self, print_time, report_offset, trigger_completion, expire_timeout
    ):
        self._trigger_completion = trigger_completion
        self._home_end_clock = None
        clock = self._mcu.print_time_to_clock(print_time)
        expire_ticks = self._mcu.seconds_to_clock(expire_timeout)
        expire_clock = clock + expire_ticks
        report_ticks = self._mcu.seconds_to_clock(expire_timeout * 0.3)
        report_clock = clock + int(report_ticks * report_offset + 0.5)
        serial = self._mcu._serial
        serial.send(
            "trsync_start oid=%d report_clock=%d report_ticks=%d"
            " expire_reason=%d"
            % (
                self._oid,
                report_clock & 0xFFFFFFFF,
                report_ticks,
                self.REASON_COMMS_TIMEOUT,
            )
        )
        serial.send(
            "trsync_set_timeout oid=%d clock=%d"
            % (self._oid, expire_clock & 0xFFFFFFFF)
        )

    def set_home_end_time(self, home_end_time):
        self._home_end_clock = self._mcu.print_time_to_clock(home_end_time)

    def stop(self):
        self._trigger_completion = None
        return self.REASON_ENDSTOP_HIT


class TriggerDispatch:
    def __init__(self, mcu):
        self._mcu = mcu
        self._trigger_completion = None
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(mcu, self._trdispatch)]

    def get_oid(self):
        return self._trsyncs[0].get_oid()

    def add_stepper(self, stepper):
        trsyncs = {trsync.get_mcu(): trsync for trsync in self._trsyncs}
        trsync = trsyncs.get(stepper.get_mcu())
        if trsync is None:
            trsync = MCU_trsync(stepper.get_mcu(), self._trdispatch)
            self._trsyncs.append(trsync)
        trsync.add_stepper(stepper)
        # Check for unsupported multi-mcu shared stepper rails
        sname = stepper.get_name()
        if sname.startswith("stepper_"):
            for ot in self._trsyncs:
                for s in ot.get_steppers():
                    if ot is not trsync and s.get_name().startswith(sname[:9]):
                        cerror = self._mcu.get_printer().config_error
                        raise cerror(
                            "Multi-mcu homing not supported on"
                            " multi-mcu shared axis"
                        )

    def get_steppers(self):
        return [s for trsync in self._trsyncs for s in trsync.get_steppers()]

    def start(self, print_time):
        raise error(
            "TriggerDispatch.start(): probe homing is not supported on the "
            "engine motion engine"
        )

    def wait_end(self, end_time):
        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(end_time)
        if self._mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()

    def stop(self):
        raise error(
            "TriggerDispatch.stop(): probe homing is not supported on the "
            "engine motion engine"
        )
