AXIS_ENDSTOP_IDS = (0, 1, 2)
PROVIDER_ID_FIRST = len(AXIS_ENDSTOP_IDS)
ENDSTOP_ID_MAX = 255

_ALLOCATOR_OBJECT = "motion_endstop_allocator"


class MotionEndstop:
    def __init__(self, pin_params, endstop_id):
        self.mcu = pin_params["chip"]
        self.endstop_id = endstop_id
        self.pin = pin_params["pin"]
        self.pullup = pin_params["pullup"]
        self.invert = pin_params["invert"]
        self.oid = self.mcu.create_oid()
        self._query_cmd = None
        self._state_cmd = None
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_endstop oid=%d endstop_id=%d pin=%s pull_up=%d invert=%d"
            % (self.oid, self.endstop_id, self.pin, self.pullup, self.invert)
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
