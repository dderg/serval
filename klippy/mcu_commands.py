# Interface to Klipper micro-controller code
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import time

from . import serialhdl


def _format_engine_msg(cmd, data):
    parts = [cmd.name]
    for i, (name, _) in enumerate(cmd.param_names):
        val = data[i]
        if isinstance(val, (bytes, bytearray)):
            val = val.hex()
        elif (
            isinstance(val, (list, tuple))
            and val
            and all(isinstance(x, int) for x in val)
        ):
            val = bytes(val).hex()
        parts.append("%s=%s" % (name, val))
    return " ".join(parts)


######################################################################
# Command transmit helper classes
######################################################################


# Class to retry sending of a query command until a given response is received
class RetryAsyncCommand:
    TIMEOUT_TIME = 5.0
    RETRY_TIME = 0.500

    def __init__(self, serial, name, oid=None):
        self.serial = serial
        self.name = name
        self.oid = oid
        self.reactor = serial.get_reactor()
        self.completion = self.reactor.completion()
        self.min_query_time = self.reactor.monotonic()
        self.need_response = True
        self.serial.register_response(self.handle_callback, name, oid)

    def handle_callback(self, params):
        if self.need_response and params["#sent_time"] >= self.min_query_time:
            self.need_response = False
            self.reactor.async_complete(self.completion, params)

    def get_response(self, cmds, cmd_queue, minclock=0, reqclock=0, retry=True):
        (cmd,) = cmds
        self.serial.raw_send_wait_ack(cmd, minclock, reqclock, cmd_queue)
        self.min_query_time = 0.0
        timeout_time = query_time = self.reactor.monotonic()
        if retry:
            timeout_time += self.TIMEOUT_TIME
        while 1:
            params = self.completion.wait(query_time + self.RETRY_TIME)
            if params is not None:
                self.serial.register_response(None, self.name, self.oid)
                return params
            query_time = self.reactor.monotonic()
            if query_time > timeout_time:
                self.serial.register_response(None, self.name, self.oid)
                raise serialhdl.error(
                    "Timeout on wait for '%s' response" % (self.name,)
                )
            self.serial.raw_send(cmd, minclock, minclock, cmd_queue)


# Wrapper around query commands
class CommandQueryWrapper:
    def __init__(
        self,
        serial,
        msgformat,
        respformat,
        oid=None,
        cmd_queue=None,
        is_async=False,
        error=serialhdl.error,
    ):
        self._serial = serial
        self._cmd = serial.get_msgparser().lookup_command(msgformat)
        serial.get_msgparser().lookup_command(respformat)
        self._response = respformat.split()[0]
        self._oid = oid
        self._error = error
        self._xmit_helper = serialhdl.SerialRetryCommand
        if is_async:
            self._xmit_helper = RetryAsyncCommand
        if cmd_queue is None:
            cmd_queue = serial.get_default_command_queue()
        self._cmd_queue = cmd_queue

    def _do_send(self, cmds, minclock, reqclock, retry):
        xh = self._xmit_helper(self._serial, self._response, self._oid)
        reqclock = max(minclock, reqclock)
        try:
            return xh.get_response(
                cmds, self._cmd_queue, minclock, reqclock, retry
            )
        except serialhdl.error as e:
            raise self._error(str(e))

    def _engine_send(self, data):
        msg = _format_engine_msg(self._cmd, data)
        _t0 = time.monotonic()
        logging.info(
            "[py-trace] _engine_send enter cmd=%s response=%s",
            getattr(self._cmd, "msgformat", "<unknown>"),
            self._response,
        )
        try:
            r = self._serial.send_with_response(msg, self._response)
            _dt_ms = (time.monotonic() - _t0) * 1000.0
            if _dt_ms > 5.0:
                logging.info(
                    "[py-trace] _engine_send exit OK cmd=%s dt_ms=%.2f",
                    getattr(self._cmd, "msgformat", "<unknown>"),
                    _dt_ms,
                )
            return r
        except serialhdl.error as e:
            _dt_ms = (time.monotonic() - _t0) * 1000.0
            logging.info(
                "[py-trace] _engine_send exit ERR cmd=%s dt_ms=%.2f err=%s",
                getattr(self._cmd, "msgformat", "<unknown>"),
                _dt_ms,
                e,
            )
            raise self._error(str(e))
        except Exception as e:
            _dt_ms = (time.monotonic() - _t0) * 1000.0
            logging.info(
                "[py-trace] _engine_send exit EXC cmd=%s dt_ms=%.2f exc=%s msg=%s",
                getattr(self._cmd, "msgformat", "<unknown>"),
                _dt_ms,
                type(e).__name__,
                e,
            )
            raise

    def send(self, data=(), minclock=0, reqclock=0, retry=True):
        if getattr(self._serial, "mcu", None) and getattr(
            self._serial.mcu, "_motion_engine", None
        ):
            return self._engine_send(data)
        cmds = self._cmd.encode(data)
        return self._do_send(cmds, minclock, reqclock, retry)

    def send_with_preface(
        self,
        preface_cmd,
        preface_data=(),
        data=(),
        minclock=0,
        reqclock=0,
        retry=True,
    ):
        preface_cmd.send(preface_data, minclock=minclock)
        return self._engine_send(data)


# Wrapper around command sending
class CommandWrapper:
    def __init__(self, serial, msgformat, cmd_queue=None):
        self._serial = serial
        msgparser = serial.get_msgparser()
        self._cmd = msgparser.lookup_command(msgformat)
        if cmd_queue is None:
            cmd_queue = serial.get_default_command_queue()
        self._cmd_queue = cmd_queue
        self._msgtag = msgparser.lookup_msgid(msgformat) & 0xFFFFFFFF

    def send(self, data=(), minclock=0, reqclock=0):
        if getattr(self._serial, "mcu", None) and getattr(
            self._serial.mcu, "_motion_engine", None
        ):
            self._serial.send(
                _format_engine_msg(self._cmd, data), minclock, reqclock
            )
        else:
            self._serial.raw_send(
                self._cmd.encode(data), minclock, reqclock, self._cmd_queue
            )

    def send_wait_ack(self, data=(), minclock=0, reqclock=0):
        if getattr(self._serial, "mcu", None) and getattr(
            self._serial.mcu, "_motion_engine", None
        ):
            self._serial.send(
                _format_engine_msg(self._cmd, data), minclock, reqclock
            )
        else:
            self._serial.raw_send_wait_ack(
                self._cmd.encode(data), minclock, reqclock, self._cmd_queue
            )

    def get_command_tag(self):
        return self._msgtag
