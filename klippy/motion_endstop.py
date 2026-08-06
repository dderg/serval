from .mcu import STEPPING_MODE_STEPCOMPRESS

AXIS_ENDSTOP_IDS = (0, 1, 2)
PROVIDER_ID_FIRST = len(AXIS_ENDSTOP_IDS)
ENDSTOP_ID_MAX = 255
UNBOUND_MOTOR = 0xFF
UNBOUND_STEPPER = 0xFF

_ALLOCATOR_OBJECT = "motion_endstop_allocator"
_TRIP_STOP_OBJECT = "motion_endstop_trip_stop"

TRIGGER_REASON_ENDSTOP = 1
TRIGGER_REASON_HOST_DISARM = 2
DISARM_REST_TICKS = 0


def endstop_entry(endstops, provider, trigger_position):
    entry = {
        "endstops": list(endstops),
        "provider": provider,
        "trigger_position": trigger_position,
    }
    if len(entry["endstops"]) == 1:
        entry["endstop"] = entry["endstops"][0]
    return entry


def entry_endstops(entry):
    endstops = entry.get("endstops")
    if endstops is None:
        return [entry["endstop"]]
    return endstops


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
    def __init__(self, pin_params, endstop_id, binding=None, group=False):
        self.mcu = pin_params["chip"]
        self.endstop_id = endstop_id
        self.pin = pin_params["pin"]
        self.pullup = pin_params["pullup"]
        self.invert = pin_params["invert"]
        self.binding = binding
        self.group = group
        self.motor_name = None if binding is None else binding.motor_name
        self.oid = self.mcu.create_oid()
        self._query_cmd = None
        self._state_cmd = None
        self._trip_stop = None
        if self.mcu.get_stepping_mode() == STEPPING_MODE_STEPCOMPRESS:
            self._trip_stop = _StepcompressTripStop(self.mcu, self.oid)
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        bound_locally = (
            self.binding is not None and self.binding.mcu is self.mcu
        )
        motor = self.binding.lane_idx if bound_locally else UNBOUND_MOTOR
        stepper = self.binding.stepper_idx if bound_locally else UNBOUND_STEPPER
        self.mcu.add_config_cmd(
            "config_endstop oid=%d endstop_id=%d pin=%s pull_up=%d invert=%d"
            " motor=%d stepper=%d group=%d"
            % (
                self.oid,
                self.endstop_id,
                self.pin,
                self.pullup,
                self.invert,
                motor,
                stepper,
                int(self.group),
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
        if self._trip_stop is not None:
            self._trip_stop.build_config()

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
        if self._trip_stop is not None:
            self._trip_stop.arm()
        engine = self.mcu.get_printer().lookup_object("motion_engine")
        engine.note_endstop_arm(self.engine_mcu_handle(), self.endstop_id)
        self._query_cmd.send([self.oid, rest_ticks])

    def disarm(self):
        self._query_cmd.send([self.oid, DISARM_REST_TICKS])
        if self._trip_stop is not None:
            self._trip_stop.disarm()

    def query_endstop(self, print_time):
        return self.is_triggered()

    def engine_mcu_handle(self):
        return self.mcu.get_engine_handle()

    def remote_freeze(self):
        if self.binding is None:
            return None
        return (
            self.binding.mcu.get_engine_handle(),
            self.binding.lane_idx,
            self.binding.stepper_idx,
        )


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

    def remote_freeze(self):
        return None

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


class _StepcompressStepperRegistry:
    def __init__(self):
        self._by_mcu = {}

    def register(self, mcu, stepper_oids):
        known = self._by_mcu.setdefault(id(mcu), [])
        for oid in stepper_oids:
            if oid not in known:
                known.append(oid)

    def stepper_oids(self, mcu):
        return self._by_mcu.get(id(mcu), [])


def _stepcompress_registry(printer):
    registry = printer.lookup_object(_TRIP_STOP_OBJECT, None)
    if registry is None:
        registry = _StepcompressStepperRegistry()
        printer.add_object(_TRIP_STOP_OBJECT, registry)
    return registry


def register_stepcompress_steppers(printer, mcu, stepper_oids):
    _stepcompress_registry(printer).register(mcu, stepper_oids)


class _StepcompressTripStop:
    """MCU-side trip stop for a stepcompress endstop: the classic step
    queues are cleared inside the endstop's trigger IRQ instead of waiting
    for the host's Stop round-trip, which at homing speed is hundreds of
    microsteps of overtravel."""

    def __init__(self, mcu, endstop_oid):
        self._mcu = mcu
        self._endstop_oid = endstop_oid
        self._trsync_oid = mcu.create_oid()
        self._start_cmd = None
        self._trigger_cmd = None
        self._stop_on_trigger_cmd = None
        self._arm_cmd = None
        self._clear_cmd = None
        self._armed_stepper_oids = []

    def build_config(self):
        self._mcu.add_config_cmd("config_trsync oid=%d" % (self._trsync_oid,))
        self._start_cmd = self._mcu.lookup_command(
            "trsync_start oid=%c report_clock=%u report_ticks=%u"
            " expire_reason=%c"
        )
        self._trigger_cmd = self._mcu.lookup_command(
            "trsync_trigger oid=%c reason=%c"
        )
        self._stop_on_trigger_cmd = self._mcu.lookup_command(
            "stepper_stop_on_trigger oid=%c trsync_oid=%c"
        )
        self._arm_cmd = self._mcu.lookup_command(
            "endstop_arm_trsync oid=%c trsync_oid=%c trigger_reason=%c"
        )
        self._clear_cmd = self._mcu.lookup_command(
            "endstop_clear_trsync oid=%c"
        )

    def arm(self):
        stepper_oids = _stepcompress_registry(
            self._mcu.get_printer()
        ).stepper_oids(self._mcu)
        if not stepper_oids:
            raise ValueError(
                "endstop oid %d: stepcompress mcu '%s' has no classic"
                " steppers registered; the trip move would run without an"
                " mcu-side stop" % (self._endstop_oid, self._mcu.get_name())
            )
        self._start_cmd.send([self._trsync_oid, 0, 0, 0])
        for oid in stepper_oids:
            self._stop_on_trigger_cmd.send([oid, self._trsync_oid])
        self._arm_cmd.send(
            [self._endstop_oid, self._trsync_oid, TRIGGER_REASON_ENDSTOP]
        )
        self._armed_stepper_oids = stepper_oids

    def disarm(self):
        if not self._armed_stepper_oids:
            return
        self._armed_stepper_oids = []
        self._clear_cmd.send([self._endstop_oid])
        self._trigger_cmd.send([self._trsync_oid, TRIGGER_REASON_HOST_DISARM])
