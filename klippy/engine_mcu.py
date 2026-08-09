# One MCU's claim on the native motion engine
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class EngineMcu:
    """Owns the (engine, handle) pair for one MCU.

    Every call that needs both the motion engine and this MCU's integer
    handle goes through here, so no other object has to reach into MCU
    internals to find them.
    """

    def __init__(self, printer, name):
        self._engine = printer.lookup_object("motion_engine", None)
        self._name = name
        self._handle = None

    def available(self):
        return self._engine is not None

    def is_claimed(self):
        return self._handle is not None

    def handle(self):
        return self._handle

    def claim(self, serial_path, baud):
        if self._handle is None:
            self._handle = self._engine.claim_mcu(self._name, serial_path, baud)
        return self._handle

    def attach_serial(
        self, serial_path, baud, timeout_s, klippy_non_critical, expect_native
    ):
        self._engine.attach_serial(
            self._handle,
            serial_path,
            baud,
            timeout_s=timeout_s,
            klippy_non_critical=klippy_non_critical,
            expect_native=expect_native,
        )

    def attach_canbus(
        self, interface, uuid, timeout_s, klippy_non_critical, expect_native
    ):
        self._engine.attach_canbus(
            self._handle,
            interface,
            uuid,
            timeout_s=timeout_s,
            klippy_non_critical=klippy_non_critical,
            expect_native=expect_native,
        )

    def detach_serial(self):
        self._engine.detach_serial(self._handle)

    def get_identify_data(self):
        return self._engine.get_identify_data(self._handle)

    def take_runtime_event(self):
        return self._engine.take_runtime_event(self._handle)

    def get_clock_async(self):
        self._engine.engine_get_clock_async(self._handle)

    def send(self, msg):
        self._engine.engine_send(self._handle, msg)

    def send_args(self, name, args):
        self._engine.engine_send_args(self._handle, name, args)

    def call(self, msg, response):
        return self._engine.engine_call(self._handle, msg, response)

    def call_args(self, name, args, response):
        return self._engine.engine_call_args(self._handle, name, args, response)

    def set_clock_est(self, freq, offset, last_clock, host_now_raw):
        self._engine.set_clock_est(
            self._handle,
            float(freq),
            float(offset),
            int(last_clock),
            host_now_raw,
        )

    def set_nominal_clock_freq(self, freq_hz):
        self._engine.set_nominal_clock_freq(self._handle, freq_hz)

    def set_msgproto_dict(self, raw_dict):
        self._engine.set_msgproto_dict(raw_dict)

    def mark_expected_disconnect(self):
        self._engine.engine_mark_expected_disconnect(self._handle)
