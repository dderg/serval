AXIS_ENDSTOP_IDS = (0, 1, 2)
PROVIDER_ID_FIRST = len(AXIS_ENDSTOP_IDS)
ENDSTOP_ID_MAX = 255
UNBOUND_MOTOR = 0xFF
UNBOUND_STEPPER = 0xFF

_ALLOCATOR_OBJECT = "motion_endstop_allocator"


class MotorBinding:
    """Ties an endstop switch to one motor of a kinematic lane. A bound
    endstop's trip freezes only that motor instead of stopping the MCU, which
    is what lets a dual-motor axis square itself against two switches."""

    def __init__(self, lane_idx, stepper_idx, mcu, motor_name):
        self.lane_idx = lane_idx
        self.stepper_idx = stepper_idx
        self.mcu = mcu
        self.motor_name = motor_name


class MotionEndstop:
    def __init__(self, pin_params, endstop_id, binding=None):
        self.mcu = pin_params["chip"]
        self.endstop_id = endstop_id
        self.pin = pin_params["pin"]
        self.pullup = pin_params["pullup"]
        self.invert = pin_params["invert"]
        self.binding = binding
        self.motor_name = None if binding is None else binding.motor_name
        if binding is not None and binding.mcu is not self.mcu:
            raise self.mcu.get_printer().config_error(
                "endstop pin '%s' is on MCU '%s' but motor '%s' is driven by"
                " MCU '%s': a motor-bound endstop switch must be wired to its"
                " own motor's MCU"
                % (
                    self.pin,
                    self.mcu.get_name(),
                    binding.motor_name,
                    binding.mcu.get_name(),
                )
            )
        self.oid = self.mcu.create_oid()
        self._query_cmd = None
        self._state_cmd = None
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        motor = UNBOUND_MOTOR if self.binding is None else self.binding.lane_idx
        stepper = (
            UNBOUND_STEPPER
            if self.binding is None
            else self.binding.stepper_idx
        )
        self.mcu.add_config_cmd(
            "config_endstop oid=%d endstop_id=%d pin=%s pull_up=%d invert=%d"
            " motor=%d stepper=%d"
            % (
                self.oid,
                self.endstop_id,
                self.pin,
                self.pullup,
                self.invert,
                motor,
                stepper,
            )
        )
        self._query_cmd = self.mcu.lookup_command(
            "query_endstop oid=%c rest_ticks=%u"
        )
        self._state_cmd = self.mcu.lookup_query_command(
            "endstop_query_state oid=%c",
            "endstop_state oid=%c armed=%c pin_value=%c tripped=%c"
            " trip_clock=%u",
            oid=self.oid,
        )

    def is_triggered(self):
        params = self._state_cmd.send([self.oid])
        return bool(params["pin_value"] ^ self.invert)

    def query_trip_state(self):
        params = self._state_cmd.send([self.oid])
        return {
            "tripped": bool(params["tripped"]),
            "trip_clock": params["trip_clock"],
        }

    def arm(self, poll_period):
        rest_ticks = self.mcu.seconds_to_clock(poll_period)
        if rest_ticks <= 0:
            raise ValueError(
                "endstop %d (pin %s): arm rest_ticks must be positive"
                % (self.endstop_id, self.pin)
            )
        self._query_cmd.send([self.oid, rest_ticks])

    def query_endstop(self, print_time):
        return self.is_triggered()

    def engine_mcu_handle(self):
        return self.mcu.get_engine_handle()


class RemoteMotionEndstop:
    """Endstop whose trigger is a trsync on a non-engine-driven MCU (e.g. a
    Beacon-class probe). Arming registers a Rust-side relay that translates
    the trsync's terminal report into a engine endstop trip; the device-side
    arming dance (trsync_start, heartbeats, probe commands) is the
    provider's job, via trip_move_begin/trip_move_end."""

    def __init__(self, printer, mcu, trsync_oid):
        self._printer = printer
        self.mcu = mcu
        self.trsync_oid = trsync_oid
        self.endstop_id = allocate_provider_id(printer)

    def engine_mcu_handle(self):
        return self.mcu.get_engine_handle()

    def is_triggered(self):
        return False

    def arm(self, poll_period):
        del poll_period
        engine = self._printer.lookup_object("motion_engine")
        engine.arm_remote_trigger(
            self.engine_mcu_handle(), self.trsync_oid, self.endstop_id
        )

    def disarm(self):
        engine = self._printer.lookup_object("motion_engine")
        engine.disarm_remote_trigger(self.endstop_id)

    def query_endstop(self, print_time):
        return False


class _ProviderIdAllocator:
    def __init__(self):
        self._next_id = PROVIDER_ID_FIRST

    def allocate(self):
        if self._next_id > ENDSTOP_ID_MAX:
            raise ValueError("out of engine endstop ids")
        endstop_id = self._next_id
        self._next_id += 1
        return endstop_id


def allocate_provider_id(printer):
    allocator = printer.lookup_object(_ALLOCATOR_OBJECT, None)
    if allocator is None:
        allocator = _ProviderIdAllocator()
        printer.add_object(_ALLOCATOR_OBJECT, allocator)
    return allocator.allocate()
