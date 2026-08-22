# Per-MCU command/response channel through the motion engine
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import threading

import serial

from . import msgproto, structured_log
from .extras.danger_options import get_danger_options


class error(Exception):
    pass


# MCU timers compare 32-bit clocks: a waketime more than 2^31 ticks ahead of
# the MCU's now reads as the past and trips "Timer too close".  Commands are
# held host-side until within 2^30 ticks, deep inside the half-range on every
# supported clock frequency.
MCU_TIMER_HORIZON = 1 << 30

# This sentinel is a serialqueue priority marker, not an MCU deadline.  It is
# used by low-priority devices such as LEDs and displays, which must be sent
# immediately instead of being deferred to a (far-future) timer horizon.
BACKGROUND_PRIORITY_CLOCK = 0x7FFFFFFF00000000

# Engine-path commands carrying a near-term reqclock (heater PWM schedules
# ~0.3 s ahead) die as "Timer too close" if delivery eats the margin.  Sends
# whose remaining margin is already below this threshold get logged so a
# late-arrival crash can be split into generated-late vs delivered-late.
DEADLINE_MARGIN_WARN = 0.150

REACTOR_STALL_LOG_S = 0.5

IFF_UP = 0x1


class EngineCommandChannel:
    def __init__(self, reactor, warn_prefix="", mcu=None):
        self.reactor = reactor
        self.warn_prefix = warn_prefix
        self.mcu = mcu
        self.engine_mcu = mcu.engine_mcu if mcu is not None else None
        self._event_poller_timer = None
        self._poller_expected_wake = None
        self._poller_stall_logged = False
        self._engine_detached = False
        self.msgparser = msgproto.MessageParser(warn_prefix=warn_prefix)
        self.lock = threading.Lock()
        # Message handlers
        self.handlers = {}
        self.register_response(self._handle_unknown_init, "#unknown")
        self.register_response(self.handle_output, "#output")
        # Sent message notification tracking
        self.last_notify_id = 0
        self.pending_notifications = {}

    def _engine_handle_status_event(self, ev):
        name = "kalico_status_v6"
        prev = getattr(self, "_last_status_state", None)
        cur = (ev.get("engine_status"), ev.get("last_fault"))
        if prev != cur:
            self._last_status_state = cur
            logging.info(
                "%s[engine-async] kalico_status_v6 "
                "engine_status=%s last_fault=%s",
                self.warn_prefix,
                cur[0],
                cur[1],
            )
        return name

    def _engine_handle_output_event(self, ev, now):
        ev["#name"] = "#output"
        ev["#sent_time"] = now
        ev["#receive_time"] = now
        ev["#msg"] = ev.get("msg", "")
        with self.lock:
            hdl = self.handlers.get(("#output", None), self.handle_default)
        try:
            hdl(ev)
        except Exception:
            logging.exception(
                "%sException in engine output callback",
                self.warn_prefix,
            )

    def _engine_handle_response_event(self, ev, now):
        name = ev.get("name", "")
        ev["#name"] = name
        # Use CLOCK_MONOTONIC_RAW stamps when the Rust engine supplied
        # them (non-zero); this happens for "clock" responses dispatched
        # via engine_get_clock_async so _handle_clock sees honest RTTs.
        sent_raw = ev.get("#sent_time_raw", 0.0)
        recv_raw = ev.get("#receive_time_raw", 0.0)
        if sent_raw != 0.0 and recv_raw != 0.0:
            ev["#sent_time"] = sent_raw
            ev["#receive_time"] = recv_raw
        elif name == "clock":
            # A clock sample without wire stamps (missed interception,
            # duplicate, late arrival) must be DROPPED by clocksync —
            # fabricating sent==recv here would feed half_rtt=0 into
            # min_half_rtt and permanently bias the estimate.
            # _handle_clock's `if not sent_time: return` does the drop.
            ev["#sent_time"] = 0.0
            ev["#receive_time"] = now
        else:
            ev["#sent_time"] = now
            ev["#receive_time"] = now
        oid = ev.get("oid")
        with self.lock:
            hdl = (
                self.handlers.get((name, oid))
                or self.handlers.get((name, None))
                or self.handle_default
            )
        try:
            hdl(ev)
        except Exception:
            logging.exception(
                "%sException in engine response callback (name=%s, oid=%s)",
                self.warn_prefix,
                name,
                oid,
            )

    def _engine_dispatch_named_event(self, ev, name, now):
        ev["#name"] = name
        ev["#sent_time"] = now
        ev["#receive_time"] = now
        hdl_key = (name, None)
        with self.lock:
            hdl = self.handlers.get(hdl_key, None)
        if hdl is None:
            hdl = self.handle_default
        try:
            hdl(ev)
        except Exception:
            logging.exception(
                "%sException in engine event callback", self.warn_prefix
            )

    def _engine_event_poller(self, eventtime):
        if self.engine_mcu is None or not self.engine_mcu.is_claimed():
            return self.reactor.NEVER
        expected_wake = self._poller_expected_wake
        if expected_wake is not None:
            lateness = eventtime - expected_wake
            if lateness > REACTOR_STALL_LOG_S:
                if not self._poller_stall_logged:
                    structured_log.event(
                        "mcu-comms",
                        "reactor_poller_late",
                        level=logging.WARNING,
                        late_s=round(lateness, 3),
                    )
                    self._poller_stall_logged = True
            else:
                self._poller_stall_logged = False
        now = eventtime
        for _ in range(32):
            try:
                ev = self.engine_mcu.take_runtime_event()
            except RuntimeError as e:
                if not self._is_engine_transport_drop(e):
                    raise
                break
            if ev is None:
                break
            ev_type = ev.get("type")
            if ev_type == "status":
                name = self._engine_handle_status_event(ev)
            elif ev_type == "credit_freed":
                # Handled directly by Rust EventDispatcher; skip Python routing.
                continue
            elif ev_type == "fault":
                name = "runtime_fault"
            elif ev_type == "endstop_tripped":
                name = "kalico_endstop_tripped"
            elif ev_type == "output":
                self._engine_handle_output_event(ev, now)
                continue
            elif ev_type == "response":
                self._engine_handle_response_event(ev, now)
                continue
            else:
                continue
            self._engine_dispatch_named_event(ev, name, now)
        next_wake = eventtime + 0.001
        self._poller_expected_wake = next_wake
        return next_wake

    def _error(self, msg, *params):
        raise error(self.warn_prefix + (msg % params))

    def _attach_flags(self):
        return (
            bool(getattr(self.mcu, "is_non_critical", False)),
            bool(getattr(self.mcu, "_expect_native", True)),
        )

    def _finish_connect(self):
        identify_data = self.engine_mcu.get_identify_data()
        logging.info(
            "%sengine identify done (%d bytes)",
            self.warn_prefix,
            len(identify_data),
        )
        msgparser = msgproto.MessageParser(warn_prefix=self.warn_prefix)
        msgparser.process_identify(identify_data)
        self.msgparser = msgparser
        self.register_response(self.handle_unknown, "#unknown")
        self.register_response(lambda params: None, "kalico_status_v6")
        self._poller_expected_wake = None
        self._poller_stall_logged = False
        self._event_poller_timer = self.reactor.register_timer(
            self._engine_event_poller, self.reactor.NOW
        )

    def connect_pipe(self, filename, baud=0):
        logging.info("%sStarting connect", self.warn_prefix)
        # claim() normally happens in _mcu_identify after connect_pipe
        # returns; claiming here (idempotently) gives attach_serial a
        # handle to bind to.
        handle = self.engine_mcu.claim(filename, baud)
        klippy_non_critical, expect_native = self._attach_flags()
        logging.info(
            "%sengine attach_serial %s (handle=%s, non_critical=%s,"
            " expect_native=%s)",
            self.warn_prefix,
            filename,
            handle,
            klippy_non_critical,
            expect_native,
        )
        self.engine_mcu.attach_serial(
            filename,
            baud,
            timeout_s=30.0,
            klippy_non_critical=klippy_non_critical,
            expect_native=expect_native,
        )
        self._finish_connect()

    def connect_uart(self, serialport, baud, rts=True):
        self.connect_pipe(serialport, baud)

    def connect_canbus(self, interface, uuid, timeout_s=30.0):
        logging.info("%sStarting canbus connect", self.warn_prefix)
        handle = self.engine_mcu.claim("%s:%s" % (interface, uuid), 0)
        klippy_non_critical, expect_native = self._attach_flags()
        logging.info(
            "%sengine attach_canbus %s uuid=%s (handle=%s, non_critical=%s,"
            " expect_native=%s)",
            self.warn_prefix,
            interface,
            uuid,
            handle,
            klippy_non_critical,
            expect_native,
        )
        self.engine_mcu.attach_canbus(
            interface,
            uuid,
            timeout_s=timeout_s,
            klippy_non_critical=klippy_non_critical,
            expect_native=expect_native,
        )
        self._finish_connect()

    def check_connect(self, serialport, baud, rts=True):
        serial_dev = serial.Serial(baudrate=baud, timeout=0, exclusive=False)
        serial_dev.port = serialport
        serial_dev.rts = rts
        try:
            serial_dev.open()
        except Exception:
            return False
        serial_dev.close()
        return True

    def check_canbus_connect(self, interface):
        try:
            with open("/sys/class/net/%s/flags" % (interface,)) as f:
                flags = int(f.read().strip(), 16)
        except OSError:
            return False
        return bool(flags & IFF_UP)

    def set_clock_est(self, freq, conv_time, last_clock):
        if not self.engine_mcu.available():
            return
        self.engine_mcu.set_clock_est(
            freq, conv_time, last_clock, True, self.reactor.monotonic()
        )

    def disconnect(self):
        # Stop the event poller BEFORE releasing the handle: a tick landing
        # after release would take_runtime_event() an unknown handle and the
        # resulting error would kill the reactor on the failed-connect path.
        if self._event_poller_timer is not None:
            self.reactor.unregister_timer(self._event_poller_timer)
            self._event_poller_timer = None
        # Post-disconnect sends are defined no-ops — klippy components
        # legitimately fire commands during the disconnect dispatch.
        self._engine_detached = True
        # Release the serial port through the engine so firmware_restart's
        # arduino_reset() can open it for the DTR toggle.  Mirrors
        # mainline's disconnect() which closes the FD before the reset.
        if (
            self.engine_mcu is not None
            and self.engine_mcu.available()
            and self.engine_mcu.is_claimed()
        ):
            # Fail loud: a silently-swallowed detach failure masks exactly the
            # class of bug (leaked fd holding the pts in exclusive mode) that
            # causes the next process's attach_serial to spin on EBUSY. Log for
            # context, then re-raise so the failure surfaces.
            try:
                self.engine_mcu.detach_serial()
            except Exception:
                logging.exception("engine detach_serial failed")
                raise
        for pn in self.pending_notifications.values():
            pn.complete(None)
        self.pending_notifications.clear()

    def stats(self, eventtime):
        return "motion_engine=1"

    def get_reactor(self):
        return self.reactor

    def get_msgparser(self):
        return self.msgparser

    # Serial response callbacks
    def register_response(self, callback, name, oid=None):
        with self.lock:
            if callback is None:
                del self.handlers[name, oid]
            else:
                self.handlers[name, oid] = callback

    def _is_engine_transport_drop(self, exc):
        if "transport closed" not in str(exc):
            return False
        self._engine_detached = True
        return True

    # Command sending
    def engine_get_clock_async(self):
        """Send a get_clock request through the engine with RAW timestamp
        capture.  The response arrives via take_runtime_event as a
        PassthroughResponse with sent_time_raw/recv_time_raw filled in."""
        if not self.engine_mcu.available():
            self._error(
                "engine_get_clock_async() called without a motion engine"
            )
        if not self.engine_mcu.is_claimed():
            self._error("engine_get_clock_async() called before claim_mcu")
        try:
            self.engine_mcu.get_clock_async()
        except RuntimeError as e:
            if not self._is_engine_transport_drop(e):
                raise

    def send(self, msg, minclock=0, reqclock=0):
        if not self.engine_mcu.available():
            self._error("send() called without a motion engine")
        if self._engine_detached:
            return
        if reqclock and self._reqclock_holds_or_warns(
            msg.split()[0], reqclock, lambda: self.send(msg, minclock, reqclock)
        ):
            return
        try:
            self.engine_mcu.send(msg)
        except RuntimeError as e:
            if not self._is_engine_transport_drop(e):
                raise

    def send_args(self, name, args, minclock=0, reqclock=0):
        if not self.engine_mcu.available():
            self._error("send_args() called without a motion engine")
        if self._engine_detached:
            return
        if reqclock and self._reqclock_holds_or_warns(
            name,
            reqclock,
            lambda: self.send_args(name, args, minclock, reqclock),
        ):
            return
        try:
            self.engine_mcu.send_args(name, args)
        except RuntimeError as e:
            if not self._is_engine_transport_drop(e):
                raise

    def _reqclock_holds_or_warns(self, command_name, reqclock, resend):
        if self._held_until_timer_horizon(resend, reqclock):
            return True
        # This sentinel is a priority marker, not a clock deadline; it has no
        # deadline margin to evaluate.
        if reqclock != BACKGROUND_PRIORITY_CLOCK:
            self._warn_if_deadline_margin_thin(command_name, reqclock)
        return False

    def _warn_if_deadline_margin_thin(self, command_name, reqclock):
        clocksync = self.mcu._clocksync
        est_clock = clocksync.get_clock(self.reactor.monotonic())
        margin = (reqclock - est_clock) / clocksync.mcu_freq
        if margin < DEADLINE_MARGIN_WARN:
            structured_log.event(
                "mcu-comms",
                "thin_deadline_margin",
                level=logging.WARNING,
                msg="engine-path command sent with thin clock margin",
                command=command_name,
                margin_s=margin,
            )

    def _held_until_timer_horizon(self, resend, reqclock):
        if reqclock == BACKGROUND_PRIORITY_CLOCK:
            return False
        clocksync = self.mcu._clocksync
        est_clock = clocksync.get_clock(self.reactor.monotonic())
        if reqclock - est_clock <= MCU_TIMER_HORIZON:
            return False
        release_systime = clocksync.estimate_clock_systime(
            reqclock - MCU_TIMER_HORIZON
        )
        self.reactor.register_callback(lambda et: resend(), release_systime)
        return True

    def send_with_response(self, msg, response):
        if not self.engine_mcu.available():
            self._error("send_with_response() called without a motion engine")
        if self._engine_detached:
            raise error("serial connection closed")
        try:
            params = self.engine_mcu.call(msg, response)
        except RuntimeError as e:
            if not self._is_engine_transport_drop(e):
                raise
            raise error("serial connection closed")
        return self._stamp_response_times(params)

    def send_with_response_args(self, name, args, response):
        if not self.engine_mcu.available():
            self._error(
                "send_with_response_args() called without a motion engine"
            )
        if self._engine_detached:
            raise error("serial connection closed")
        try:
            params = self.engine_mcu.call_args(name, args, response)
        except RuntimeError as e:
            if not self._is_engine_transport_drop(e):
                raise
            raise error("serial connection closed")
        return self._stamp_response_times(params)

    def _stamp_response_times(self, params):
        # Use CLOCK_MONOTONIC_RAW timestamps if the Rust engine supplied them
        # (non-zero means a real RTT was measured on the wire).  Fall back to
        # reactor.monotonic() only when both sides stamp the same instant (the
        # old behaviour, which gives half_rtt=0 and breaks min_half_rtt).
        sent_raw = params.get("#sent_time_raw", 0.0)
        recv_raw = params.get("#receive_time_raw", 0.0)
        if sent_raw != 0.0 and recv_raw != 0.0:
            params["#sent_time"] = sent_raw
            params["#receive_time"] = recv_raw
        else:
            now = self.reactor.monotonic()
            params["#sent_time"] = now
            params["#receive_time"] = now
        return params

    # Dumping debug lists
    def dump_debug(self):
        return "EngineCommandChannel: engine mode"

    # Default message handlers
    def _handle_unknown_init(self, params):
        logging.debug(
            "%sUnknown message %d (len %d) while identifying",
            self.warn_prefix,
            params["#msgid"],
            len(params["#msg"]),
        )

    def handle_unknown(self, params):
        logging.warning(
            "%sUnknown message type %d: %s",
            self.warn_prefix,
            params["#msgid"],
            repr(params["#msg"]),
        )

    def handle_output(self, params):
        logging.info(
            "%s%s: %s", self.warn_prefix, params["#name"], params["#msg"]
        )

    def handle_default(self, params):
        if get_danger_options().log_serial_reader_warnings:
            logging.warning("%s got %s", self.warn_prefix, params)
