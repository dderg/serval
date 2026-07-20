from klippy.extras import stepper_enable as _stepper_enable_mod


class _FakeClockSync:
    def __init__(self, debug="fake clocksync"):
        self._debug = debug

    def dump_debug(self):
        return self._debug


class FakeMcu:
    def __init__(
        self,
        printer=None,
        name="mcu",
        handle=0,
        est_print_time=None,
        print_time_offset=0.0,
        query_cmd=None,
        state_cmd=None,
        non_critical_disconnected=False,
        clocksync_debug="fake clocksync",
    ):
        self._printer = printer
        self._name = name
        self._handle = handle
        self._est_print_time = est_print_time
        self._print_time_offset = print_time_offset
        self._oid_count = 0
        self.config_callbacks = []
        self.config_cmds = []
        self.query_cmd = query_cmd
        self.state_cmd = state_cmd
        self.non_critical_disconnected = non_critical_disconnected
        self._clocksync = _FakeClockSync(clocksync_debug)

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def get_engine_handle(self):
        return self._handle

    def create_oid(self):
        oid = self._oid_count
        self._oid_count += 1
        return oid

    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append(cmd)

    def estimated_print_time(self, eventtime):
        if self._est_print_time is not None:
            return self._est_print_time
        return eventtime + self._print_time_offset

    def print_time_to_clock(self, print_time):
        return int(print_time * 1_000_000)

    def seconds_to_clock(self, seconds):
        return int(seconds * 1_000_000)

    def lookup_command(self, msgformat):
        return self.query_cmd

    def lookup_query_command(self, msgformat, respformat, oid=None):
        return self.state_cmd


class FakeStepper:
    def __init__(
        self,
        name="stepper",
        handle=0,
        mcu=None,
        pulse_duration=0.000002,
        step_both_edge=False,
        step_dist=0.0125,
    ):
        self._name = name
        self._active_callbacks = []
        self._mcu = mcu if mcu is not None else FakeMcu(handle=handle)
        self._pulse = (pulse_duration, step_both_edge)
        self._step_dist = step_dist
        self.current_helper = None

    def get_name(self, short=False):
        if short and self._name.startswith("stepper_"):
            return self._name[len("stepper_") :]
        return self._name

    def get_mcu(self):
        return self._mcu

    def get_pulse_duration(self):
        return self._pulse

    def get_step_dist(self):
        return self._step_dist

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def set_tmc_current_helper(self, helper):
        self.current_helper = helper

    def setup_default_pulse_duration(self, pulse_duration, step_both_edge):
        pass


class FakeRail:
    def __init__(
        self,
        name=None,
        steppers=None,
        motor_name=None,
        chain_index=None,
        ff_config=(False, 30.0),
        dynamics_profile=None,
    ):
        self._name = name
        self._steppers = list(steppers) if steppers is not None else []
        self._motor_name = motor_name
        self._chain_index = chain_index
        self._ff_config = ff_config
        self._dynamics_profile = dynamics_profile

    def get_name(self, short=False):
        if short and self._name and self._name.startswith("stepper_"):
            return self._name[len("stepper_") :]
        return self._name

    def get_steppers(self):
        return list(self._steppers)

    def get_motor_name(self):
        return self._motor_name

    def get_chain_index(self):
        return self._chain_index

    def get_ff_config(self):
        return self._ff_config

    def get_dynamics_profile(self):
        return self._dynamics_profile


class FakeEnableLine:
    def __init__(self, enabled=False, dedicated=True):
        self._enabled = enabled
        self.dedicated = dedicated
        self.enabled_at = []
        self.disabled_at = []
        self.state_callback = None

    def is_motor_enabled(self):
        return self._enabled

    def energize(self, print_time):
        self.enabled_at.append(print_time)
        return None

    def motor_enable(self, print_time):
        self.enabled_at.append(print_time)

    def motor_disable(self, print_time):
        self.disabled_at.append(print_time)

    def register_state_callback(self, callback):
        self.state_callback = callback

    def has_dedicated_enable(self):
        return self.dedicated


class _ToolheadLookup:
    def __init__(self, toolhead):
        self._toolhead = toolhead

    def lookup_object(self, name):
        assert name == "toolhead"
        return self._toolhead


class FakeStepperEnable:
    def __init__(
        self,
        toolhead=None,
        names=(),
        enable_line=None,
        enabled=None,
        real_methods=False,
    ):
        self.printer = _ToolheadLookup(toolhead)
        self.enable_lines = {n: FakeEnableLine() for n in names}
        if enable_line is None and enabled is not None:
            enable_line = FakeEnableLine(enabled=enabled)
        self._enable_line = enable_line
        self.calls = []
        if real_methods:
            self.motor_debug_enable = _stepper_enable_mod.PrinterStepperEnable.motor_debug_enable.__get__(
                self
            )
            self.motor_enable_group = _stepper_enable_mod.PrinterStepperEnable.motor_enable_group.__get__(
                self
            )

    def lookup_enable(self, name):
        if self._enable_line is not None:
            return self._enable_line
        return self.enable_lines[name]

    def motor_debug_enable(self, stepper, enable):
        self.calls.append((stepper, enable))

    def motor_enable_group(self, stepper_names):
        self.calls.append(("group", tuple(stepper_names)))


class _FakePinEndstop:
    def __init__(self, pin):
        self.pin = pin
        self.steppers = []

    def add_stepper(self, stepper):
        self.steppers.append(stepper)


class FakePins:
    def __init__(self, chip=None):
        self.chip = chip
        self.chips = {}

    def register_chip(self, chip_name, chip):
        self.chips[chip_name] = chip

    def lookup_pin(
        self, pin_desc, can_invert=False, can_pullup=False, share_type=None
    ):
        return {
            "pin": pin_desc,
            "invert": False,
            "pullup": False,
            "chip": self.chip,
            "chip_name": "mcu",
        }

    def setup_pin(self, pin_type, pin_desc):
        assert pin_type == "endstop"
        return _FakePinEndstop(pin_desc)


class FakeNode:
    def __init__(
        self,
        handle=0,
        slots=None,
        name="node_x",
        cycle_us=250,
        dynamics_profile=None,
    ):
        self.name = name
        self._handle = handle
        self._slots = dict(slots) if slots else {}
        self._torque_motors = set()
        self.torque_calls = []
        self.waiter_calls = 0
        self.calls = []
        self._cycle_us = cycle_us
        self.dynamics_profile = dynamics_profile

    def get_engine_handle(self):
        return self._handle

    def get_dynamics_profile(self):
        return self.dynamics_profile

    def get_drive_count(self):
        return len(self._slots)

    def get_slot_for_motor(self, motor_name):
        return self._slots.get(motor_name)

    def get_cycle_us(self):
        return self._cycle_us

    def set_motor_torque(self, motor_name, value, print_time):
        self.calls.append((motor_name, value, print_time))
        self.torque_calls.append((motor_name, value, print_time))
        if value:
            first = not self._torque_motors
            self._torque_motors.add(motor_name)
            if first:

                def waiter():
                    self.waiter_calls += 1

                return waiter
        else:
            self._torque_motors.discard(motor_name)
        return None


class FakeServoCapture:
    def __init__(
        self, events=None, stop_result=("/tmp/fake.scap", 4321, 250.0)
    ):
        self.events = events if events is not None else []
        self.starts = []
        self._stop_result = stop_result

    def start_capture_to(self, path, servos):
        self.events.append("capture_start")
        self.starts.append((path, list(servos)))
        return path

    def stop_capture(self):
        self.events.append("capture_stop")
        return self._stop_result
