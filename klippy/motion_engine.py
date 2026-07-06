try:
    from . import _motion_engine as _native
except ImportError:
    _native = None

from . import structured_log

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


_STUB_MOTION_METHODS = frozenset(
    {
        "submit_nudge",
        "init_planner",
        "submit_move",
        "submit_dwell",
        "submit_bezier",
        "submit_quadratic",
        "wait_moves",
        "fence_start",
        "fence_poll",
        "drain_motion",
        "motion_drain_poll",
        "motion_drain_finalize",
        "set_position",
        "get_last_move_time",
        "set_velocity_cap",
        "set_accel_cap",
        "set_square_corner_velocity",
        "update_post_processor",
        "set_bed_mesh",
        "clear_bed_mesh",
        "fallback_clock_conversions",
        "dispatched_segment_count",
        "register_phase_bus",
        "register_phase_motor",
        "get_mcu_capabilities",
        "ring_depth_for_axis",
        "claim_mcu",
        "claim_ethercat_node",
        "release_mcu",
        "detach_serial",
        "attach_serial",
        "set_clock_est",
        "set_msgproto_dict",
        "engine_call",
        "engine_send",
        "set_torque_start",
        "set_drive_limits_start",
        "restore_drive_limits_start",
        "arm_sensorless_endstop_start",
        "endpoint_call_done",
        "take_drive_fault",
        "take_endpoint_death",
        "finalize_homed_axis_start",
        "sdo_read",
        "sdo_write",
    }
)


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


class _StubEngine:
    """Stand-in for MotionEngineWrapper when _motion_engine is unavailable
    (e.g. CI without the cdylib). Usable for import/boot/config tests only: any
    motion-issuing method raises RuntimeError instead of returning None, so a
    test reaching real motion under the stub fails loud. Non-motion lifecycle
    helpers stay no-ops so config-only boots tear down cleanly.
    """

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
        if name in _STUB_MOTION_METHODS:

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

        def _noop(*args, **kwargs):
            return None

        return _noop


class MotionEngineWrapper:
    """Thin wrapper registered as printer object 'motion_engine'."""

    def __init__(self, reactor):
        if _native is None:
            raise ImportError("_motion_engine not available")
        self._engine = _native.MotionEngine()
        self._reactor = reactor

    def get_engine(self):
        return self._engine

    def claim_mcu(self, label, serial_path, baud):
        return self._engine.claim_mcu(label, serial_path, baud)

    def claim_ethercat_node(
        self,
        label,
        socket_path,
        interface,
        endpoint,
        cycle_us,
        dynamics_profile,
        drives,
    ):
        return self._engine.claim_ethercat_node(
            label,
            socket_path,
            interface,
            endpoint,
            cycle_us,
            dynamics_profile,
            drives,
        )

    def _wait_endpoint_call(self, call_id):
        # Endpoint round-trips run on a background thread; blocking the
        # reactor here would starve heater PWM refreshes ("Timer too close").
        while not self._engine.endpoint_call_done(call_id):
            self._reactor.pause(self._reactor.monotonic() + 0.005)

    def set_drive_limits(
        self, mcu_handle, slot, following_error_counts, max_torque_tenth_pct
    ):
        self._wait_endpoint_call(
            self._engine.set_drive_limits_start(
                mcu_handle, slot, following_error_counts, max_torque_tenth_pct
            )
        )

    def restore_drive_limits(self, mcu_handle, slot):
        self._wait_endpoint_call(
            self._engine.restore_drive_limits_start(mcu_handle, slot)
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
            )
        )

    def disarm_sensorless_endstop(self, mcu_handle, slot, endstop_id):
        self._wait_endpoint_call(
            self._engine.arm_sensorless_endstop_start(
                mcu_handle, slot, endstop_id, 0, False
            )
        )

    def take_drive_fault(self, mcu_handle):
        return self._engine.take_drive_fault(mcu_handle)

    def take_endpoint_death(self, mcu_handle):
        return self._engine.take_endpoint_death(mcu_handle)

    def stop_node(self, mcu_handle):
        return self._engine.stop_node(mcu_handle)

    def finalize_homed_axis(self, mcu_handle, axis, pos_mm):
        self._wait_endpoint_call(
            self._engine.finalize_homed_axis_start(
                mcu_handle, axis, list(pos_mm)
            )
        )

    def set_torque(self, mcu_handle, value, print_time):
        self._wait_endpoint_call(
            self._engine.set_torque_start(mcu_handle, bool(value), print_time)
        )

    def start_servo_capture(self, mcu_handle, path, started_utc, drives):
        return self._engine.start_servo_capture(
            mcu_handle, path, started_utc, drives
        )

    def stop_servo_capture(self, mcu_handle):
        return self._engine.stop_servo_capture(mcu_handle)

    def sdo_read(self, mcu_handle, slot, index, subindex):
        return self._engine.sdo_read(mcu_handle, slot, index, subindex)

    def sdo_write(self, mcu_handle, slot, index, subindex, size, value):
        return self._engine.sdo_write(
            mcu_handle, slot, index, subindex, size, value
        )

    def resonance_buzz(
        self,
        mcu_handle,
        axis_mask,
        sign_mask,
        freq_start_millihz,
        freq_end_millihz,
        amplitude_nm,
        duration_ms,
        ramp_ms,
    ):
        return self._engine.resonance_buzz(
            mcu_handle,
            axis_mask,
            sign_mask,
            freq_start_millihz,
            freq_end_millihz,
            amplitude_nm,
            duration_ms,
            ramp_ms,
        )

    def release_mcu(self, handle):
        return self._engine.release_mcu(handle)

    def detach_serial(self, handle):
        return self._engine.detach_serial(handle)

    def shutdown(self):
        return self._engine.shutdown()

    def set_clock_est(self, handle, freq, offset, last_clock, host_now_raw):
        return self._engine.set_clock_est(
            handle, freq, offset, last_clock, host_now_raw
        )

    def set_nominal_clock_freq(self, mcu_handle, freq_hz):
        return self._engine.set_nominal_clock_freq(mcu_handle, int(freq_hz))

    def engine_get_clock_async(self, handle):
        return self._engine.engine_get_clock_async(handle)

    def attach_serial(
        self,
        mcu_handle,
        serial_path,
        baud,
        timeout_s=30.0,
        klippy_non_critical=False,
        expect_native=True,
    ):
        """klippy_non_critical feeds the per-MCU criticality gate: a
        non-critical MCU's transport drop does not abort klippy, a critical
        motion MCU's does. A Klipper-protocol-only attach (identify timed out)
        is always treated as non-critical. expect_native=False skips the
        native identify probe entirely (plugin-attached foreign peripherals
        like the Beacon never answer it; probing them stalls connect).
        """
        return self._engine.attach_serial(
            mcu_handle,
            serial_path,
            baud,
            timeout_s,
            klippy_non_critical,
            expect_native,
        )

    def get_identify_data(self, mcu_handle):
        return bytes(self._engine.get_identify_data(mcu_handle))

    def get_mcu_capabilities(self, mcu_handle):
        return self._engine.get_mcu_capabilities(mcu_handle)

    def ring_depth_for_axis(self, mcu_handle, axis_idx):
        return self._engine.ring_depth_for_axis(mcu_handle, axis_idx)

    def register_phase_bus(self, mcu_handle, bus_id, rate, timeout_s=5.0):
        """Call once per bus_id, BEFORE any register_phase_motor for that bus.
        Per-motor CS GPIOs are registered separately
        (each TMC5160 on a shared bus needs its own CS). No-op on stock MCUs.
        """
        return self._engine.register_phase_bus(
            mcu_handle,
            bus_id,
            rate,
            timeout_s,
        )

    def register_phase_motor(
        self, mcu_handle, motor_idx, bus_id, cs_pin_id, slot_idx, timeout_s=5.0
    ):
        """Call once per phase-stepped motor, AFTER register_phase_bus.
        slot_idx is the kinematic slot whose commanded position drives this
        motor's XDIRECT output."""
        return self._engine.register_phase_motor(
            mcu_handle,
            motor_idx,
            bus_id,
            cs_pin_id,
            slot_idx,
            timeout_s,
        )

    def engine_call(self, mcu_handle, msg, response, timeout_s=15.0):
        return self._engine.engine_call(mcu_handle, msg, response, timeout_s)

    def engine_send(self, mcu_handle, msg):
        return self._engine.engine_send(mcu_handle, msg)

    def engine_mark_expected_disconnect(self, mcu_handle):
        """Mark an imminent transport drop so the reactor's EXIT_ON_FAULT guard
        treats it as graceful instead of a wedge. Called before the firmware
        `reset` command (NVIC_SystemReset drops USB-CDC).
        """
        return self._engine.engine_mark_expected_disconnect(mcu_handle)

    def take_runtime_event(self, mcu_handle):
        return self._engine.take_runtime_event(mcu_handle)

    def on_credit_freed(
        self, mcu_handle, retired_through_segment_id, free_slots
    ):
        return self._engine.on_credit_freed(
            mcu_handle,
            retired_through_segment_id,
            free_slots,
        )

    def set_msgproto_dict(self, dict_json):
        return self._engine.set_msgproto_dict(dict_json)

    def init_planner(
        self,
        axes,
        limits,
        post_processors,
        mcus,
        kinematics_axes,
        cartesian_limits,
        arc_fit=None,
        max_extrude_only_velocity=None,
        max_extrude_only_accel=None,
        fit_tolerance_mm=None,
        fit_tolerance_accel_mm_s2=None,
    ):
        return self._engine.init_planner(
            axes,
            limits,
            post_processors,
            mcus,
            kinematics_axes,
            cartesian_limits,
            arc_fit,
            max_extrude_only_velocity,
            max_extrude_only_accel,
            fit_tolerance_mm,
            fit_tolerance_accel_mm_s2,
        )

    def submit_move(self, dx, dy, dz, de, feedrate):
        return self._engine.submit_move(dx, dy, dz, de, feedrate)

    def wait_moves(self):
        flush_id = self._engine.wait_moves_start()
        while not self._engine.wait_moves_poll(flush_id):
            self._reactor.pause(self._reactor.monotonic() + 0.005)

    def drain_motion(self):
        return self._engine.drain_motion()

    def motion_drain_poll(self):
        return self._engine.motion_drain_poll()

    def motion_drain_finalize(self):
        return self._engine.motion_drain_finalize()

    def submit_dwell(self, duration_s):
        return self._engine.submit_dwell(duration_s)

    def submit_bezier(self, i, j, p, q, dx, dy, dz, de, feedrate):
        return self._engine.submit_bezier(i, j, p, q, dx, dy, dz, de, feedrate)

    def submit_quadratic(self, i, j, dx, dy, dz, de, feedrate):
        return self._engine.submit_quadratic(i, j, dx, dy, dz, de, feedrate)

    def set_position(self, x, y, z):
        return self._engine.set_position(x, y, z, self._reactor.monotonic())

    def motion_drained(self):
        return self._engine.motion_drained()

    def home_axis_start(
        self,
        axis,
        direction,
        speed_mm_s,
        max_travel_mm,
        endstop_id,
        endstop_mcu,
    ):
        return self._engine.home_axis_start(
            axis, direction, speed_mm_s, max_travel_mm, endstop_id, endstop_mcu
        )

    def submit_nudge(
        self, mcu_id, axis_idx, motor_mask, delta_mm, speed, accel
    ):
        return self._engine.submit_nudge(
            mcu_id, axis_idx, motor_mask, delta_mm, speed, accel
        )

    def home_axis_poll(self):
        return self._engine.home_axis_poll()

    def home_abort(self):
        return self._engine.home_abort()

    def arm_remote_trigger(self, mcu_handle, trsync_oid, endstop_id):
        return self._engine.arm_remote_trigger(
            mcu_handle, trsync_oid, endstop_id
        )

    def disarm_remote_trigger(self, endstop_id):
        return self._engine.disarm_remote_trigger(endstop_id)

    def effective_limits(self):
        return self._engine.effective_limits()

    def set_velocity_cap(self, velocity):
        return self._engine.set_velocity_cap(velocity)

    def set_accel_cap(self, accel):
        return self._engine.set_accel_cap(accel)

    def set_square_corner_velocity(self, square_corner_velocity):
        return self._engine.set_square_corner_velocity(square_corner_velocity)

    def update_post_processor(self, name, key, value):
        return self._engine.update_post_processor(name, key, value)

    def set_bed_mesh(
        self, points, x_min, y_min, dx, dy, nx, ny, tension, fade, zx, zy
    ):
        return self._engine.set_bed_mesh(
            points, x_min, y_min, dx, dy, nx, ny, tension, fade, zx, zy
        )

    def clear_bed_mesh(self):
        return self._engine.clear_bed_mesh()

    def bed_mesh_gcode_z(self, x, y, z_machine):
        return self._engine.bed_mesh_gcode_z(x, y, z_machine)

    def get_last_move_time(self):
        return self._engine.get_last_move_time()

    def fence_start(self, force):
        return self._engine.fence_start(force)

    def fence_poll(self, fence_id):
        return self._engine.fence_poll(fence_id)

    def queued_motion_secs(self):
        return self._engine.queued_motion_secs() or 0.0

    def dispatched_lead_secs(self):
        return self._engine.dispatched_lead_secs() or 0.0

    def pending_channel_moves(self):
        return self._engine.pending_channel_moves() or 0

    def input_channel_capacity(self):
        return self._engine.input_channel_capacity()

    def pump_backlog(self):
        return self._engine.pump_backlog() or 0

    def motion_lead_secs(self):
        return self._engine.motion_lead_secs()

    def fallback_clock_conversions(self):
        return self._engine.fallback_clock_conversions()

    def dispatched_segment_count(self):
        return self._engine.dispatched_segment_count()

    def motion_state_at(self, mcu, clock=None, print_time=None):
        if (clock is None) == (print_time is None):
            raise ValueError(
                "motion_state_at: specify exactly one of clock= or print_time="
            )
        if print_time is not None:
            clock = mcu.print_time_to_clock(print_time)
        return self._engine.motion_state_at_clock(
            mcu._engine_handle, int(clock), self._reactor.monotonic()
        )

    def live_motor_positions(self):
        return self._engine.live_motor_positions()

    def query_motor_positions(self, timeout_s=0.25):
        return self._engine.query_motor_positions(timeout_s)
