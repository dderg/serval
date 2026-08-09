# Interface to Klipper micro-controller code
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import serialhdl


def _command_args(cmd, data):
    return [(name, data[i]) for i, (name, _) in enumerate(cmd.param_names)]


def _command_delivery(minclock, reqclock, delivery):
    if delivery is not None:
        return delivery
    if minclock or reqclock:
        return serialhdl.CommandDelivery.TIMED
    return serialhdl.CommandDelivery.IMMEDIATE


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

    def _engine_send(
        self, data, minclock=0, reqclock=0, delivery=None, preface=None
    ):
        try:
            return self._serial.send_with_response_args(
                self._cmd.name,
                _command_args(self._cmd, data),
                self._response,
                _command_delivery(minclock, reqclock, delivery),
                minclock,
                reqclock,
                preface,
            )
        except serialhdl.error as e:
            raise self._error(str(e))

    def send(self, data=(), minclock=0, reqclock=0, retry=True, delivery=None):
        return self._engine_send(data, minclock, reqclock, delivery)

    def send_with_preface(
        self,
        preface_cmd,
        preface_data=(),
        data=(),
        minclock=0,
        reqclock=0,
        retry=True,
        delivery=None,
    ):
        preface = (
            preface_cmd._cmd.name,
            _command_args(preface_cmd._cmd, preface_data),
        )
        return self._engine_send(data, minclock, reqclock, delivery, preface)


# Wrapper around command sending
class CommandWrapper:
    def __init__(self, serial, msgformat):
        self._serial = serial
        msgparser = serial.get_msgparser()
        self._cmd = msgparser.lookup_command(msgformat)
        self._msgtag = msgparser.lookup_msgid(msgformat) & 0xFFFFFFFF

    def send(self, data=(), minclock=0, reqclock=0, delivery=None):
        self._serial.send_args(
            self._cmd.name,
            _command_args(self._cmd, data),
            _command_delivery(minclock, reqclock, delivery),
            minclock,
            reqclock,
        )

    def send_wait_ack(self, data=(), minclock=0, reqclock=0, delivery=None):
        self.send(data, minclock, reqclock, delivery)

    def get_command_tag(self):
        return self._msgtag
