# Interface to Klipper micro-controller code
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
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


# Wrapper around query commands
class CommandQueryWrapper:
    def __init__(
        self,
        serial,
        msgformat,
        respformat,
        oid=None,
        error=serialhdl.error,
    ):
        self._serial = serial
        self._cmd = serial.get_msgparser().lookup_command(msgformat)
        serial.get_msgparser().lookup_command(respformat)
        self._response = respformat.split()[0]
        self._oid = oid
        self._error = error

    def _engine_send(self, data):
        msg = _format_engine_msg(self._cmd, data)
        try:
            return self._serial.send_with_response(msg, self._response)
        except serialhdl.error as e:
            raise self._error(str(e))

    def send(self, data=(), minclock=0, reqclock=0, retry=True):
        return self._engine_send(data)

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
    def __init__(self, serial, msgformat):
        self._serial = serial
        msgparser = serial.get_msgparser()
        self._cmd = msgparser.lookup_command(msgformat)
        self._msgtag = msgparser.lookup_msgid(msgformat) & 0xFFFFFFFF

    def send(self, data=(), minclock=0, reqclock=0):
        self._serial.send(
            _format_engine_msg(self._cmd, data), minclock, reqclock
        )

    def send_wait_ack(self, data=(), minclock=0, reqclock=0):
        self._serial.send(
            _format_engine_msg(self._cmd, data), minclock, reqclock
        )

    def get_command_tag(self):
        return self._msgtag
