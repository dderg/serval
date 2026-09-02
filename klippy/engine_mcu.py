import concurrent.futures


class EngineMcu:
    """Owns the (engine, handle) pair for one MCU.

    Every call that needs both the motion engine and this MCU's integer
    handle goes through here, so no other object has to reach into MCU
    internals to find them.
    """

    def __init__(self, printer, name):
        self._engine = printer.lookup_object("motion_engine", None)
        self._reactor = printer.get_reactor()
        self._name = name
        self._handle = None
        self._calls = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="mcu-call-%s" % (name,)
        )
        printer.register_event_handler(
            "klippy:disconnect", self._shutdown_calls
        )

    def _shutdown_calls(self):
        self._calls.shutdown(wait=False, cancel_futures=True)

    def _wait_call(self, call):
        future = self._calls.submit(call)
        while not future.done():
            self._reactor.pause(self._reactor.monotonic() + 0.001)
        return future.result()

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

    def take_endpoint_death(self):
        return self._engine.take_endpoint_death(self._handle)

    def get_clock_async(self):
        self._engine.engine_get_clock_async(self._handle)

    def send(self, msg):
        self._engine.engine_send(self._handle, msg)

    def send_args(self, name, args):
        self._engine.engine_send_args(self._handle, name, args)

    def call(self, msg, response):
        return self._wait_call(
            lambda: self._engine.engine_call(self._handle, msg, response)
        )

    def call_args(self, name, args, response):
        return self._wait_call(
            lambda: self._engine.engine_call_args(
                self._handle, name, args, response
            )
        )

    def set_clock_est(self, freq, offset, last_clock, converged, host_now_raw):
        self._engine.set_clock_est(
            self._handle,
            float(freq),
            float(offset),
            int(last_clock),
            bool(converged),
            float(host_now_raw),
        )

    def invalidate_clock_est(self):
        self._engine.invalidate_clock_est(self._handle)

    def set_nominal_clock_freq(self, freq_hz):
        self._engine.set_nominal_clock_freq(self._handle, freq_hz)

    def set_msgproto_dict(self, raw_dict):
        self._engine.set_msgproto_dict(raw_dict)

    def mark_expected_disconnect(self):
        self._engine.engine_mark_expected_disconnect(self._handle)
