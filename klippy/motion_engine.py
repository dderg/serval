import importlib.util
import os
import pathlib
import sys

from . import engine_wait, structured_log


def _load_native():
    """Load the _motion_engine extension. Prefer the sibling build, then
    $KALICO_NATIVE_DIR by explicit path (the CI image installs natives there
    while bind-mounting the checkout over the image's tree). Returns None when
    no build is present so `import klippy` succeeds in a native-less env; use
    sites fail loudly instead."""
    try:
        from . import _motion_engine as native

        return native
    except ImportError:
        pass
    native_dir = os.environ.get("KALICO_NATIVE_DIR")
    if native_dir:
        path = pathlib.Path(native_dir) / "_motion_engine.so"
        if path.is_file():
            try:
                spec = importlib.util.spec_from_file_location(
                    "klippy._motion_engine", path
                )
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                sys.modules["klippy._motion_engine"] = module
                return module
            except ImportError:
                pass
    return None


_native = _load_native()

NATIVE_BUILD_HINT = "make -f Makefile.rust motion-engine"


def native_class(name):
    """Return a class exported by the native _motion_engine module, failing
    loudly when the native build is absent so construction (not import) is the
    point of failure."""
    if _native is None:
        raise RuntimeError(
            "klippy requires the native _motion_engine module for %s; "
            "build it with '%s'." % (name, NATIVE_BUILD_HINT)
        )
    return getattr(_native, name)


_PRINT_ACTIVE_EVENTS = (
    "print_stats:start_printing",
    "print_stats:paused_printing",
)
_PRINT_FINISH_EVENTS = (
    "print_stats:complete_printing",
    "print_stats:error_printing",
    "print_stats:cancelled_printing",
    "print_stats:reset",
)

ENDPOINT_CALL_DEADLINE_S = 30.0


def attach_structured_logging(native, printer, events_dir):
    if events_dir:
        native.init_logging(events_dir)
    native.set_session_context(
        structured_log.get_session(), structured_log.get_print()
    )

    def _push_ctx(*_args):
        native.set_session_context(
            structured_log.get_session(), structured_log.get_print()
        )

    def _clear_ctx(*_args):
        native.set_session_context(structured_log.get_session(), "")

    for ev in _PRINT_ACTIVE_EVENTS:
        printer.register_event_handler(ev, _push_ctx)
    for ev in _PRINT_FINISH_EVENTS:
        printer.register_event_handler(ev, _clear_ctx)


_STUB_NOOP_METHODS = frozenset(
    {
        "shutdown",
        "motion_lead_secs",
    }
)


class _StubEngine:
    """Stand-in for MotionEngineWrapper when _motion_engine is unavailable
    (e.g. CI without the cdylib). Usable for import/boot/config tests only:
    every method raises RuntimeError except the lifecycle helpers in
    _STUB_NOOP_METHODS and the telemetry getters defined below, so a test
    reaching real motion under the stub fails loud instead of silently
    no-oping."""

    def pump_backlog(self):
        return 0

    def queued_motion_secs(self):
        return 0.0

    def dispatched_lead_secs(self):
        return 0.0

    def pending_channel_moves(self):
        return 0

    def input_channel_capacity(self):
        return 8192

    def __getattr__(self, name):
        if name in _STUB_NOOP_METHODS:

            def _noop(*args, **kwargs):
                return None

            return _noop

        def _raise(*args, **kwargs):
            raise RuntimeError(
                "_motion_engine not built: cannot call "
                "%r on the stub engine. The klippy motion path was "
                "exercised without the real Rust engine. Build the "
                "cdylib (e.g. `make -f Makefile.rust motion-engine`) "
                "to exercise real motion, or restrict this test to "
                "import/boot/config only." % (name,)
            )

        return _raise


class MotionEngineWrapper:
    """Printer object 'motion_engine'. Methods below add host-side logic
    (waits, defaults that differ from the native signature, type coercion);
    everything else delegates to the native PyO3 class via __getattr__."""

    def __init__(self, printer):
        if _native is None:
            raise ImportError("_motion_engine not available")
        self._engine = _native.MotionEngine()
        self._printer = printer
        self._reactor = printer.get_reactor()

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        return getattr(self._engine, name)

    def get_engine(self):
        return self._engine

    def _wait_endpoint_call(self, call_id, what):
        # Endpoint round-trips run on a background thread; blocking the
        # reactor here would starve heater PWM refreshes ("Timer too close").
        engine_wait.wait_for(
            self._printer,
            lambda: self._engine.endpoint_call_done(call_id) or None,
            what,
            ENDPOINT_CALL_DEADLINE_S,
        )

    def set_drive_limits(self, mcu_handle, drives):
        self._wait_endpoint_call(
            self._engine.set_drive_limits_start(mcu_handle, drives),
            "set_drive_limits",
        )

    def restore_drive_limits(self, mcu_handle, slots):
        self._wait_endpoint_call(
            self._engine.restore_drive_limits_start(mcu_handle, slots),
            "restore_drive_limits",
        )

    def arm_sensorless_endstop(
        self, mcu_handle, slot, endstop_id, torque_trip_tenth_pct, enable
    ):
        self._wait_endpoint_call(
            self._engine.arm_sensorless_endstop_start(
                mcu_handle,
                slot,
                endstop_id,
                torque_trip_tenth_pct,
                bool(enable),
            ),
            "arm_sensorless_endstop",
        )

    def disarm_sensorless_endstop(self, mcu_handle, slot, endstop_id):
        self._wait_endpoint_call(
            self._engine.arm_sensorless_endstop_start(
                mcu_handle, slot, endstop_id, 0, False
            ),
            "disarm_sensorless_endstop",
        )

    def finalize_homed_axis(self, mcu_handle, axis, pos_mm):
        self._wait_endpoint_call(
            self._engine.finalize_homed_axis_start(
                mcu_handle, axis, list(pos_mm)
            ),
            "finalize_homed_axis",
        )

    def set_torque(self, mcu_handle, value, print_time):
        self.set_torque_deferred(mcu_handle, value, print_time)()

    def set_torque_deferred(self, mcu_handle, value, print_time):
        call_id = self._engine.set_torque_start(
            mcu_handle, bool(value), print_time
        )
        return lambda: self._wait_endpoint_call(call_id, "set_torque")

    def get_identify_data(self, mcu_handle):
        return bytes(self._engine.get_identify_data(mcu_handle))

    def set_nominal_clock_freq(self, mcu_handle, freq_hz):
        return self._engine.set_nominal_clock_freq(mcu_handle, int(freq_hz))

    def engine_call(self, mcu_handle, msg, response, timeout_s=15.0):
        return self._engine.engine_call(mcu_handle, msg, response, timeout_s)

    def wait_moves(self):
        flush_id = self._engine.wait_moves_start()
        engine_wait.wait_for(
            self._printer,
            lambda: self._engine.wait_moves_poll(flush_id) or None,
            "wait_moves flush",
            engine_wait.UNBOUNDED,
        )

    def set_position(self, x, y, z):
        return self._engine.set_position(x, y, z, self._reactor.monotonic())

    def queued_motion_secs(self):
        return self._engine.queued_motion_secs() or 0.0

    def dispatched_lead_secs(self):
        return self._engine.dispatched_lead_secs() or 0.0

    def pending_channel_moves(self):
        return self._engine.pending_channel_moves() or 0

    def pump_backlog(self):
        return self._engine.pump_backlog() or 0

    def motion_state_at(self, mcu, clock=None, print_time=None):
        """Per-axis (pos, vel, accel) at a clock, in GCODE space: the bridge
        unwarps the machine-space motion history through the active bed mesh,
        so results are directly comparable to toolhead positions."""
        if (clock is None) == (print_time is None):
            raise ValueError(
                "motion_state_at: specify exactly one of clock= or print_time="
            )
        if print_time is not None:
            clock = mcu.get_clocksync().print_time_to_clock(print_time)
        return self._engine.motion_state_at_clock(
            mcu.get_engine_handle(), int(clock), self._reactor.monotonic()
        )
