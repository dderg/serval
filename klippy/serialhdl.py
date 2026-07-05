# Serial port management for firmware communication
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os
import threading

import serial

from . import chelper, msgproto, structured_log, util
from .extras.danger_options import get_danger_options


class error(Exception):
    pass


# MCU timers compare 32-bit clocks: a waketime more than 2^31 ticks ahead of
# the MCU's now reads as the past and trips "Timer too close".  Commands are
# held host-side until within 2^30 ticks, deep inside the half-range on every
# supported clock frequency.
MCU_TIMER_HORIZON = 1 << 30

# Engine-path commands carrying a near-term reqclock (heater PWM schedules
# ~0.3 s ahead) die as "Timer too close" if delivery eats the margin.  Sends
# whose remaining margin is already below this threshold get logged so a
# late-arrival crash can be split into generated-late vs delivered-late.
DEADLINE_MARGIN_WARN = 0.150


class SerialReader:
    def __init__(self, reactor, warn_prefix="", mcu=None):
        self.reactor = reactor
        self.warn_prefix = warn_prefix
        self.mcu = mcu
        # Serial port
        self.serial_dev = None
        self._event_poller_timer = None
        self._engine_detached = False
        self.msgparser = msgproto.MessageParser(warn_prefix=warn_prefix)
        self.ffi_main, self.ffi_lib = chelper.get_ffi()
        self.serialqueue = None
        self.default_cmd_queue = None
        self.stats_buf = None
        # Threading
        self.lock = threading.Lock()
        self.background_thread = None
        # Message handlers
        self.handlers = {}
        self.register_response(self._handle_unknown_init, "#unknown")
        self.register_response(self.handle_output, "#output")
        # Sent message notification tracking
        self.last_notify_id = 0
        self.pending_notifications = {}

    def _bg_thread(self):
        response = self.ffi_main.new("struct pull_queue_message *")
        while True:
            self.ffi_lib.serialqueue_pull(self.serialqueue, response)
            count = response.len
            if count < 0:
                break
            if response.notify_id:
                params = {
                    "#sent_time": response.sent_time,
                    "#receive_time": response.receive_time,
                }
                completion = self.pending_notifications.pop(response.notify_id)
                self.reactor.async_complete(completion, params)
                continue
            params = self.msgparser.parse(response.msg[0:count])
            params["#sent_time"] = response.sent_time
            params["#receive_time"] = response.receive_time
            hdl = (params["#name"], params.get("oid"))
            try:
                with self.lock:
                    hdl = self.handlers.get(hdl, self.handle_default)
                    hdl(params)
            except:
                logging.exception(
                    "%sException in serial callback", self.warn_prefix
                )

    def _engine_event_poller(self, eventtime):
        if self.mcu is None:
            return self.reactor.NEVER
        engine = self.mcu._motion_engine
        handle = self.mcu._engine_handle
        if handle is None:
            return self.reactor.NEVER
        now = eventtime
        for _ in range(32):
            try:
                ev = engine.take_runtime_event(handle)
            except RuntimeError as e:
                if not self._is_engine_transport_drop(e):
                    raise
                break
            if ev is None:
                break
            ev_type = ev.get("type")
            if ev_type == "status":
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
            elif ev_type == "credit_freed":
                # Handled directly by Rust EventDispatcher; skip Python routing.
                continue
            elif ev_type == "fault":
                name = "runtime_fault"
            elif ev_type == "endstop_tripped":
                name = "kalico_endstop_tripped"
            elif ev_type == "output":
                name = "#output"
                ev["#name"] = "#output"
                ev["#sent_time"] = now
                ev["#receive_time"] = now
                ev["#msg"] = ev.get("msg", "")
                with self.lock:
                    hdl = self.handlers.get(
                        ("#output", None), self.handle_default
                    )
                try:
                    hdl(ev)
                except Exception:
                    logging.exception(
                        "%sException in engine output callback",
                        self.warn_prefix,
                    )
                continue
            elif ev_type == "response":
                name = ev.get("name", "")
                if name == "trsync_state":
                    logging.info(
                        "%s[engine-poller] trsync_state response: "
                        "oid=%s can_trigger=%s trigger_reason=%s",
                        self.warn_prefix,
                        ev.get("oid"),
                        ev.get("can_trigger"),
                        ev.get("trigger_reason"),
                    )
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
                    if name == "trsync_state":
                        logging.info(
                            "%s[engine-poller] trsync_state handler "
                            "lookup: key=(%s,%s) found=%s",
                            self.warn_prefix,
                            name,
                            oid,
                            hdl is not self.handle_default,
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
                continue
            else:
                continue
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
        return eventtime + 0.001

    def _error(self, msg, *params):
        raise error(self.warn_prefix + (msg % params))

    def _get_identify_data(self, eventtime):
        # Query the "data dictionary" from the micro-controller
        identify_data = b""
        while True:
            msg = "identify offset=%d count=%d" % (len(identify_data), 40)
            try:
                params = self.send_with_response(msg, "identify_response")
            except error as e:
                logging.exception(
                    "%sWait for identify_response", self.warn_prefix
                )
                return None
            if params["offset"] == len(identify_data):
                msgdata = params["data"]
                if not msgdata:
                    # Done
                    return identify_data
                identify_data += msgdata

    def _start_session(self, serial_dev, serial_fd_type=b"u", client_id=0):
        self.serial_dev = serial_dev
        self.serialqueue = self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc(
                serial_dev.fileno(), serial_fd_type, client_id
            ),
            self.ffi_lib.serialqueue_free,
        )
        self.background_thread = threading.Thread(target=self._bg_thread)
        self.background_thread.start()
        # Obtain and load the data dictionary from the firmware
        completion = self.reactor.register_callback(self._get_identify_data)
        identify_data = completion.wait(self.reactor.monotonic() + 5.0)
        if identify_data is None:
            logging.info("%sTimeout on connect", self.warn_prefix)
            self.disconnect()
            return False
        msgparser = msgproto.MessageParser(warn_prefix=self.warn_prefix)
        msgparser.process_identify(identify_data)
        self.msgparser = msgparser
        self.register_response(self.handle_unknown, "#unknown")
        # Setup baud adjust
        if serial_fd_type == b"c":
            wire_freq = msgparser.get_constant_float("CANBUS_FREQUENCY", None)
        else:
            wire_freq = msgparser.get_constant_float("SERIAL_BAUD", None)
        if wire_freq is not None:
            self.ffi_lib.serialqueue_set_wire_frequency(
                self.serialqueue, wire_freq
            )
        receive_window = msgparser.get_constant_int("RECEIVE_WINDOW", None)
        if receive_window is not None:
            self.ffi_lib.serialqueue_set_receive_window(
                self.serialqueue, receive_window
            )
        return True

    def check_canbus_connect(
        self, canbus_uuid, canbus_nodeid, canbus_iface="can0"
    ):
        import can  # XXX

        try:
            uuid = int(canbus_uuid, 16)
        except ValueError:
            uuid = -1
        if uuid < 0 or uuid > 0xFFFFFFFFFFFF:
            self._error("Invalid CAN uuid")

        CANBUS_ID_ADMIN = 0x3F0
        CMD_QUERY_UNASSIGNED = 0x00
        CMD_QUERY_UNASSIGNED_EXTENDED = 0x01
        RESP_NEED_NODEID = 0x20
        RESP_HAVE_NODEID = 0x21
        filters = [
            {
                "can_id": CANBUS_ID_ADMIN + 1,
                "can_mask": 0x7FF,
                "extended": False,
            }
        ]

        msg = can.Message(
            arbitration_id=CANBUS_ID_ADMIN,
            data=[CMD_QUERY_UNASSIGNED, CMD_QUERY_UNASSIGNED_EXTENDED],
            is_extended_id=False,
        )
        try:
            bus = can.interface.Bus(
                channel=canbus_iface,
                can_filters=filters,
                bustype="socketcan",
            )
            bus.send(msg)
        except (can.CanError, os.error) as e:
            logging.warning("%scan issue: %s", self.warn_prefix, e)
            return False

        start_time = curtime = self.reactor.monotonic()
        while True:
            tdiff = start_time + 1.0 - curtime
            if tdiff <= 0.0:
                break
            msg = bus.recv(tdiff)
            curtime = self.reactor.monotonic()
            if (
                msg is None
                or msg.arbitration_id != CANBUS_ID_ADMIN + 1
                or msg.dlc < 7
                or msg.data[0] not in (RESP_NEED_NODEID, RESP_HAVE_NODEID)
            ):
                continue
            found_uuid = sum(
                [v << ((5 - i) * 8) for i, v in enumerate(msg.data[1:7])]
            )
            # logging.info(f"found_uuid: {hex(found_uuid)[2:]}")
            if found_uuid == uuid:
                self.disconnect()
                bus.close = bus.shutdown  # XXX
                return True
        bus.close = bus.shutdown  # XXX
        # logging.info(f"couldn't find uuid: {hex(uuid)[2:]}")
        return False

    def connect_canbus(self, canbus_uuid, canbus_nodeid, canbus_iface="can0"):
        import can  # XXX

        txid = canbus_nodeid * 2 + 256
        filters = [{"can_id": txid + 1, "can_mask": 0x7FF, "extended": False}]
        # Prep for SET_NODEID command
        try:
            uuid = int(canbus_uuid, 16)
        except ValueError:
            uuid = -1
        if uuid < 0 or uuid > 0xFFFFFFFFFFFF:
            self._error("Invalid CAN uuid")
        uuid = [(uuid >> (40 - i * 8)) & 0xFF for i in range(6)]
        CANBUS_ID_ADMIN = 0x3F0
        CMD_SET_NODEID = 0x01
        set_id_cmd = [CMD_SET_NODEID] + uuid + [canbus_nodeid]
        set_id_msg = can.Message(
            arbitration_id=CANBUS_ID_ADMIN,
            data=set_id_cmd,
            is_extended_id=False,
        )
        # Start connection attempt
        logging.info("%sStarting CAN connect", self.warn_prefix)
        start_time = self.reactor.monotonic()
        while True:
            if self.reactor.monotonic() > start_time + 90.0:
                self._error("Unable to connect")
            try:
                bus = can.interface.Bus(
                    channel=canbus_iface,
                    can_filters=filters,
                    bustype="socketcan",
                )
                bus.send(set_id_msg)
            except (can.CanError, os.error) as e:
                logging.warning(
                    "%sUnable to open CAN port: %s", self.warn_prefix, e
                )
                self.reactor.pause(self.reactor.monotonic() + 5.0)
                continue
            bus.close = bus.shutdown  # XXX
            ret = self._start_session(bus, b"c", txid)
            if not ret:
                continue
            # Verify correct canbus_nodeid to canbus_uuid mapping
            try:
                params = self.send_with_response("get_canbus_id", "canbus_id")
                got_uuid = bytearray(params["canbus_uuid"])
                if got_uuid == bytearray(uuid):
                    break
            except:
                logging.exception(
                    "%sError in canbus_uuid check", self.warn_prefix
                )
            logging.info(
                "%sFailed to match canbus_uuid - retrying..", self.warn_prefix
            )
            self.disconnect()

    def connect_pipe(self, filename, baud=0):
        logging.info("%sStarting connect", self.warn_prefix)
        engine = self.mcu._motion_engine
        # claim_mcu may not have been called yet (it normally happens in
        # _mcu_identify after connect_pipe returns). Allocate the handle
        # here so attach_serial has something to bind to; the later guard
        # in _mcu_identify will skip the second claim_mcu call.
        if self.mcu._engine_handle is None:
            self.mcu._engine_handle = engine.claim_mcu(
                self.mcu._name,
                filename,
                baud,
            )
        handle = self.mcu._engine_handle
        klippy_non_critical = bool(getattr(self.mcu, "is_non_critical", False))
        expect_native = bool(getattr(self.mcu, "_expect_native", True))
        logging.info(
            "%sengine attach_serial %s (handle=%s, non_critical=%s,"
            " expect_native=%s)",
            self.warn_prefix,
            filename,
            handle,
            klippy_non_critical,
            expect_native,
        )
        engine.attach_serial(
            handle,
            filename,
            baud,
            timeout_s=30.0,
            klippy_non_critical=klippy_non_critical,
            expect_native=expect_native,
        )
        identify_data = engine.get_identify_data(handle)
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
        self._event_poller_timer = self.reactor.register_timer(
            self._engine_event_poller, self.reactor.NOW
        )

    def connect_uart(self, serialport, baud, rts=True):
        self.connect_pipe(serialport, baud)

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

    def connect_file(self, debugoutput, dictionary, pace=False):
        self.serial_dev = debugoutput
        self.msgparser.process_identify(dictionary, decompress=False)
        self.serialqueue = self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc(self.serial_dev.fileno(), b"f", 0),
            self.ffi_lib.serialqueue_free,
        )
        self.default_cmd_queue = self.alloc_command_queue()

    def set_clock_est(self, freq, conv_time, conv_clock, last_clock):
        if self.mcu._motion_engine is None:
            return
        host_now_raw = self.reactor.monotonic()
        self.mcu._motion_engine.set_clock_est(
            self.mcu._engine_handle,
            float(freq),
            float(conv_time),
            int(conv_clock),
            host_now_raw,
        )

    def disconnect(self):
        # Stop the event poller BEFORE releasing the handle: a tick landing
        # after release would take_runtime_event() an unknown handle and the
        # resulting error would kill the reactor on the failed-connect path.
        if self._event_poller_timer is not None:
            self.reactor.unregister_timer(self._event_poller_timer)
            self._event_poller_timer = None
        # Post-disconnect sends are defined no-ops, mirroring mainline's
        # `if self.serialqueue is None: return` contract — klippy components
        # legitimately fire commands during the disconnect dispatch.
        self._engine_detached = True
        # Release the serial port through the engine so firmware_restart's
        # arduino_reset() can open it for the DTR toggle.  Mirrors
        # mainline's disconnect() which closes the FD before the reset.
        engine = getattr(self.mcu, "_motion_engine", None)
        handle = getattr(self.mcu, "_engine_handle", None)
        if engine is not None and handle is not None:
            # Fail loud: a silently-swallowed detach failure masks exactly the
            # class of bug (leaked fd holding the pts in exclusive mode) that
            # causes the next process's attach_serial to spin on EBUSY. Log for
            # context, then re-raise so the failure surfaces.
            try:
                engine.detach_serial(handle)
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

    def get_serialqueue(self):
        return None

    def get_default_command_queue(self):
        return self.default_cmd_queue

    # Serial response callbacks
    def register_response(self, callback, name, oid=None):
        with self.lock:
            if callback is None:
                del self.handlers[name, oid]
            else:
                self.handlers[name, oid] = callback

    def _check_noncritical_disconnected(self):
        if self.mcu is not None and self.mcu.non_critical_disconnected:
            self._error("non-critical MCU is disconnected")

    def _is_engine_transport_drop(self, exc):
        if "transport closed" not in str(exc):
            return False
        self._engine_detached = True
        return True

    # Command sending
    def engine_get_clock_async(self):
        """Send a get_clock request through the engine with RAW timestamp
        capture.  Used by clocksync._get_clock_event to replace the no-op
        raw_send path.  The response arrives via take_runtime_event as a
        PassthroughResponse with sent_time_raw/recv_time_raw filled in.

        This no-arg form is the hasattr target in clocksync._get_clock_event
        (``hasattr(self.serial, "engine_get_clock_async")``); it resolves the
        MCU handle internally.  MotionEngineWrapper also exposes a
        engine_get_clock_async(handle) method — passing a wrapper object where
        a SerialReader is expected would TypeError at the hasattr call site
        because the wrapper's method requires an explicit handle argument."""
        engine = getattr(self.mcu, "_motion_engine", None)
        if engine is None:
            return
        handle = self.mcu._engine_handle
        if handle is None:
            return
        try:
            engine.engine_get_clock_async(handle)
        except RuntimeError as e:
            if not self._is_engine_transport_drop(e):
                raise

    def raw_send(self, cmd, minclock, reqclock, cmd_queue):
        self._check_noncritical_disconnected()
        if self.serialqueue is not None:
            if cmd_queue is None:
                cmd_queue = self.default_cmd_queue
            if cmd_queue is not None:
                self.ffi_lib.serialqueue_send(
                    self.serialqueue,
                    cmd_queue,
                    cmd,
                    len(cmd),
                    minclock,
                    reqclock,
                    0,
                )

    def raw_send_wait_ack(self, cmd, minclock, reqclock, cmd_queue):
        self._check_noncritical_disconnected()
        if self.serialqueue is not None:
            if cmd_queue is None:
                cmd_queue = self.default_cmd_queue
            if cmd_queue is not None:
                self.ffi_lib.serialqueue_send(
                    self.serialqueue,
                    cmd_queue,
                    cmd,
                    len(cmd),
                    minclock,
                    reqclock,
                    0,
                )

    def send(self, msg, minclock=0, reqclock=0):
        engine = getattr(self.mcu, "_motion_engine", None)
        if engine is not None:
            if self._engine_detached:
                return
            if reqclock and self._held_until_timer_horizon(
                msg, minclock, reqclock
            ):
                return
            if reqclock:
                self._warn_if_deadline_margin_thin(msg, reqclock)
            handle = self.mcu._engine_handle
            try:
                engine.engine_send(handle, msg)
            except RuntimeError as e:
                if not self._is_engine_transport_drop(e):
                    raise
        elif self.serialqueue is not None:
            cmd = self.msgparser.create_command(msg)
            self.ffi_lib.serialqueue_send(
                self.serialqueue,
                self.default_cmd_queue,
                cmd,
                len(cmd),
                minclock,
                reqclock,
                0,
            )

    def _warn_if_deadline_margin_thin(self, msg, reqclock):
        clocksync = self.mcu._clocksync
        est_clock = clocksync.get_clock(self.reactor.monotonic())
        margin = (reqclock - est_clock) / clocksync.mcu_freq
        if margin < DEADLINE_MARGIN_WARN:
            structured_log.event(
                "mcu-comms",
                "thin_deadline_margin",
                level=logging.WARNING,
                msg="engine-path command sent with thin clock margin",
                command=msg.split()[0],
                margin_s=margin,
            )

    def _held_until_timer_horizon(self, msg, minclock, reqclock):
        clocksync = self.mcu._clocksync
        est_clock = clocksync.get_clock(self.reactor.monotonic())
        if reqclock - est_clock <= MCU_TIMER_HORIZON:
            return False
        release_systime = clocksync.estimate_clock_systime(
            reqclock - MCU_TIMER_HORIZON
        )
        self.reactor.register_callback(
            lambda et: self.send(msg, minclock, reqclock), release_systime
        )
        return True

    def send_with_response(self, msg, response):
        engine = getattr(self.mcu, "_motion_engine", None)
        if engine is not None:
            if self._engine_detached:
                raise error("serial connection closed")
            try:
                params = engine.engine_call(
                    self.mcu._engine_handle,
                    msg,
                    response,
                )
            except RuntimeError as e:
                if not self._is_engine_transport_drop(e):
                    raise
                raise error("serial connection closed")
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
        raise error("send_with_response requires motion engine")

    def alloc_command_queue(self):
        if self.serialqueue is not None:
            return self.ffi_main.gc(
                self.ffi_lib.serialqueue_alloc_commandqueue(),
                self.ffi_lib.serialqueue_free_commandqueue,
            )
        return None

    # Dumping debug lists
    def dump_debug(self):
        return "SerialReader: engine mode (no C serialqueue)"

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


# Class to send a query command and return the received response
class SerialRetryCommand:
    def __init__(self, serial, name, oid=None):
        self.serial = serial
        self.name = name
        self.oid = oid
        self.last_params = None
        self.serial.register_response(self.handle_callback, name, oid)

    def handle_callback(self, params):
        self.last_params = params

    def get_response(self, cmds, cmd_queue, minclock=0, reqclock=0, retry=True):
        retries = 5
        retry_delay = 0.010
        if not retry:
            retries = 0
        while 1:
            for cmd in cmds[:-1]:
                self.serial.raw_send(cmd, minclock, reqclock, cmd_queue)
            self.serial.raw_send_wait_ack(
                cmds[-1], minclock, reqclock, cmd_queue
            )
            params = self.last_params
            if params is not None:
                self.serial.register_response(None, self.name, self.oid)
                return params
            if retries <= 0:
                self.serial.register_response(None, self.name, self.oid)
                raise error("Unable to obtain '%s' response" % (self.name,))
            reactor = self.serial.reactor
            reactor.pause(reactor.monotonic() + retry_delay)
            retries -= 1
            retry_delay *= 2.0


# Attempt to place an AVR stk500v2 style programmer into normal mode
def stk500v2_leave(ser, reactor):
    logging.debug("Starting stk500v2 leave programmer sequence")
    util.clear_hupcl(ser.fileno())
    origbaud = ser.baudrate
    # Request a dummy speed first as this seems to help reset the port
    ser.baudrate = 2400
    ser.read(1)
    # Send stk500v2 leave programmer sequence
    ser.baudrate = 115200
    reactor.pause(reactor.monotonic() + 0.100)
    ser.read(4096)
    ser.write(b"\x1b\x01\x00\x01\x0e\x11\x04")
    reactor.pause(reactor.monotonic() + 0.050)
    res = ser.read(4096)
    logging.debug("Got %s from stk500v2", repr(res))
    ser.baudrate = origbaud


def cheetah_reset(serialport, reactor):
    # Fysetc Cheetah v1.2 boards have a weird stateful circuitry for
    # configuring the bootloader. This sequence takes care of disabling it for
    # sure.
    # Open the serial port with RTS asserted
    ser = serial.Serial(baudrate=2400, timeout=0, exclusive=True)
    ser.port = serialport
    ser.rts = True
    ser.open()
    ser.read(1)
    reactor.pause(reactor.monotonic() + 0.100)
    # Toggle DTR
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    # Deassert RTS
    reactor.pause(reactor.monotonic() + 0.100)
    ser.rts = False
    reactor.pause(reactor.monotonic() + 0.100)
    # Toggle DTR again
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    reactor.pause(reactor.monotonic() + 0.100)
    ser.close()


# Attempt an arduino style reset on a serial port
def arduino_reset(serialport, reactor):
    # First try opening the port at a different baud
    ser = serial.Serial(serialport, 2400, timeout=0, exclusive=True)
    ser.read(1)
    reactor.pause(reactor.monotonic() + 0.100)
    # Then toggle DTR
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    reactor.pause(reactor.monotonic() + 0.100)
    ser.close()
