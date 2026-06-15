import contextlib
import logging

from klippy import structured_log
from klippy.bridge_endstop import AXIS_ENDSTOP_IDS, BridgeEndstop
from klippy.extras.danger_options import get_danger_options

HOMING_POLL_PERIOD = 0.001
TRIP_DEADLINE_MARGIN = 5.0
_DRAIN_PAUSE_TIMEOUT = 60.0


def _endstop_section(config, axis_name):
    section = "axis " + axis_name
    if config.has_section(section):
        return section
    return None


def _homing_motor_names(rail):
    steppers = rail.get_steppers()
    if not steppers:
        return [rail.get_name()]
    return [s.get_name() for s in steppers]


@contextlib.contextmanager
def _servo_drive_limits(bridge, handle, limits):
    if handle is None or limits is None:
        yield
        return
    bridge.set_drive_limits(handle, limits[0], limits[1])
    try:
        yield
    except BaseException:
        try:
            bridge.restore_drive_limits(handle)
        except Exception:
            logging.warning(
                "homing: restore_drive_limits failed while handling a"
                " homing error",
                exc_info=True,
            )
        raise
    bridge.restore_drive_limits(handle)


def _run_servo_guarded_trip(
    gcmd, bridge, axis, stepper_enable, rail, servo_handle, servo_limits, trip
):
    try:
        with _servo_drive_limits(bridge, servo_handle, servo_limits):
            result = trip()
        _check_servo_drive_fault(gcmd, bridge, axis, servo_handle)
    except BaseException:
        if servo_handle is not None:
            stepper_enable.motor_debug_enable(rail.get_name(), False)
        raise
    return result


def _check_servo_drive_fault(gcmd, bridge, axis, servo_handle):
    if servo_handle is None:
        return
    fault = bridge.take_drive_fault(servo_handle)
    if fault is not None:
        raise gcmd.error(
            "%s homing: drive fault 0x%04x at endstop contact — "
            "following-error/torque limit exceeded" % ("XYZ"[axis], fault)
        )


def _homed_axis_position(provider, axis, trip_pos, final_pos, trigger_height):
    if provider is not None and hasattr(provider, "measured_trip_position"):
        measured = provider.measured_trip_position(axis, trip_pos, final_pos)
        if measured is not None:
            return measured
    return trigger_height + (final_pos[axis] - trip_pos[axis])


def _commit_and_seed(
    toolhead,
    bridge,
    axis,
    direction,
    hi,
    trip_pos,
    final_pos,
    trigger_height,
    provider,
    servo_handle,
):
    overshoot = final_pos[axis] - trip_pos[axis]
    newpos = list(toolhead.get_position())
    newpos[axis] = _homed_axis_position(
        provider, axis, trip_pos, final_pos, trigger_height
    )
    toolhead.set_position(newpos, homing_axes=[axis])
    structured_log.event(
        "homing",
        "axis_homed",
        msg="homing: %s trigger=%.4f overshoot=%+.4f set %s=%.4f"
        % ("XYZ"[axis], trigger_height, overshoot, "XYZ"[axis], newpos[axis]),
        axis="XYZ"[axis],
        trigger_height=trigger_height,
        overshoot=overshoot,
        homed_position=newpos[axis],
    )
    if hi.retract_dist:
        retractpos = list(toolhead.get_position())
        retractpos[axis] -= direction * hi.retract_dist + overshoot
        toolhead.move(retractpos, hi.retract_speed)
        toolhead.wait_moves()
    if servo_handle is not None:
        bridge.finalize_homed_axis(
            servo_handle, axis, toolhead.get_position()[axis]
        )


def _run_homing_attempts(
    gcmd,
    toolhead,
    axis,
    direction,
    hi,
    trigger_height,
    provider,
    first_max_travel,
    tolerance,
    approach,
):
    start_pos = toolhead.get_position()
    trip_pos, final_pos = approach(hi.speed, first_max_travel)
    traveled = abs(trip_pos[axis] - start_pos[axis])
    needs_rehome = _trigger_too_early(traveled, hi.min_home_dist, tolerance)
    structured_log.event(
        "homing",
        "needs_rehome",
        msg="homing: %s needs rehome: %s (traveled=%.4f min_home_dist=%.4f)"
        % ("XYZ"[axis], needs_rehome, traveled, hi.min_home_dist),
        axis="XYZ"[axis],
        needs_rehome=needs_rehome,
        traveled=traveled,
        min_home_dist=hi.min_home_dist,
    )
    if not needs_rehome:
        return trip_pos, final_pos
    haltpos = list(toolhead.get_position())
    haltpos[axis] = final_pos[axis]
    toolhead.set_position(haltpos, homing_axes=[axis])
    backoff = list(toolhead.get_position())
    backoff[axis] = trip_pos[axis] - direction * hi.min_home_dist
    toolhead.move(backoff, hi.retract_speed)
    toolhead.wait_moves()
    start_pos = toolhead.get_position()
    trip_pos, final_pos = approach(hi.speed, 2.0 * hi.min_home_dist)
    traveled = abs(trip_pos[axis] - start_pos[axis])
    if _trigger_too_early(traveled, hi.min_home_dist, tolerance):
        raise gcmd.error(
            "%s early homing trigger: endstop tripped after only %.2fmm on "
            "re-approach (min_home_dist %.2fmm) — false trigger or "
            "stuck/miswired endstop" % ("XYZ"[axis], traveled, hi.min_home_dist)
        )
    return trip_pos, final_pos


def _trigger_too_early(traveled, min_home_dist, tolerance):
    if min_home_dist <= 0.0:
        return False
    return traveled < min_home_dist and (min_home_dist - traveled) >= tolerance


def _verify_latched_trip(gcmd, axis, endstop, doorbell_clock):
    query = getattr(endstop, "query_trip_state", None)
    if query is None:
        return
    latch = query()
    if not latch["tripped"]:
        raise gcmd.error(
            "%s endstop: doorbell event arrived but the MCU latch shows no"
            " trip — duplicate or stale trip event" % ("XYZ"[axis],)
        )
    if latch["trip_clock"] != (doorbell_clock & 0xFFFFFFFF):
        raise gcmd.error(
            "%s endstop: latch/doorbell clock mismatch — latch=%d"
            " doorbell_low32=%d"
            % (
                "XYZ"[axis],
                latch["trip_clock"],
                doorbell_clock & 0xFFFFFFFF,
            )
        )


def _no_trigger_error_message(axis, endstop, max_travel):
    base = "%s endstop did not trigger within %.1fmm of travel" % (
        "XYZ"[axis],
        max_travel,
    )
    query = getattr(endstop, "query_trip_state", None)
    if query is None:
        return base
    latch = query()
    if latch["tripped"]:
        return (
            "%s endstop tripped (latched clock %d) but the trip event was"
            " lost — doorbell never reached the host"
            % ("XYZ"[axis], latch["trip_clock"])
        )
    return base


class Homing:
    def __init__(self, config):
        self.printer = config.get_printer()
        self._config = config
        self._axes = None

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("G28", self.cmd_G28, desc="Home")
        gcode.register_command(
            "_HOME_TEST",
            self.cmd_HOME_TEST,
            desc="Bench only: home one axis with override SPEED/MAX_TRAVEL",
        )

    def resolve_endstops(self):
        if self._config is None:
            raise self.printer.config_error(
                "homing: resolve_endstops called twice"
            )
        config, self._config = self._config, None
        ppins = self.printer.lookup_object("pins")

        self._axes = {}
        for axis_index, axis_name in enumerate("xyz"):
            section = _endstop_section(config, axis_name)
            if section is None:
                continue
            axis_config = config.getsection(section)
            endstop_pin = axis_config.get("endstop_pin", None)
            if endstop_pin is None:
                continue
            pin_params = ppins.parse_pin(
                endstop_pin, can_invert=True, can_pullup=True
            )
            chip = pin_params["chip"]
            if hasattr(chip, "setup_bridge_endstop"):
                entry = self._provider_entry(
                    axis_config, axis_index, chip, pin_params
                )
            elif hasattr(chip, "create_oid"):
                entry = {
                    "endstop": BridgeEndstop(
                        pin_params, AXIS_ENDSTOP_IDS[axis_index]
                    ),
                    "provider": None,
                    "trigger_height": None,
                }
            else:
                raise config.error(
                    "endstop_pin '%s' in [%s]: chip '%s' is neither an MCU"
                    " nor a virtual endstop provider"
                    % (endstop_pin, section, pin_params["chip_name"])
                )
            self._axes[axis_index] = entry

        query_endstops = self.printer.load_object(config, "query_endstops")
        for axis_index in sorted(self._axes):
            query_endstops.register_endstop(
                self._axes[axis_index]["endstop"], "xyz"[axis_index]
            )

    def _provider_entry(self, axis_config, axis_index, chip, pin_params):
        endstop = chip.setup_bridge_endstop(pin_params, axis_index)
        trigger_height = None
        if hasattr(chip, "get_position_endstop"):
            trigger_height = chip.get_position_endstop()
            if axis_config.get("position_endstop", None) is not None:
                raise axis_config.error(
                    "[%s] must not set position_endstop: its virtual endstop"
                    " '%s' supplies the trigger height"
                    % (axis_config.get_name(), pin_params["chip_name"])
                )
        return {
            "endstop": endstop,
            "provider": chip,
            "trigger_height": trigger_height,
        }

    def cmd_G28(self, gcmd):
        if self._axes is None:
            raise gcmd.error("G28: homing endstops were never resolved")
        requested = [
            i for i, a in enumerate("XYZ") if gcmd.get(a, None) is not None
        ]
        if not requested:
            requested = sorted(self._axes.keys())
        toolhead = self.printer.lookup_object("toolhead")
        bridge = self.printer.lookup_object("motion_bridge")
        kin = toolhead.get_kinematics()
        for axis in requested:
            entry = self._axes.get(axis)
            if entry is None:
                raise gcmd.error("G28: axis %s has no endstop" % ("XYZ"[axis],))
            self._home_axis(gcmd, toolhead, bridge, kin, axis, entry)

    def cmd_HOME_TEST(self, gcmd):
        if self._axes is None:
            raise gcmd.error("_HOME_TEST: homing endstops were never resolved")
        axis_name = gcmd.get("AXIS").upper()
        if axis_name not in ("X", "Y", "Z"):
            raise gcmd.error("_HOME_TEST: AXIS must be X, Y, or Z")
        axis = "XYZ".index(axis_name)
        entry = self._axes.get(axis)
        if entry is None:
            raise gcmd.error("_HOME_TEST: axis %s has no endstop" % axis_name)
        speed = gcmd.get_float("SPEED", None, above=0.0)
        max_travel = gcmd.get_float("MAX_TRAVEL", None, above=0.0)
        toolhead = self.printer.lookup_object("toolhead")
        bridge = self.printer.lookup_object("motion_bridge")
        kin = toolhead.get_kinematics()
        self._home_axis(
            gcmd, toolhead, bridge, kin, axis, entry, speed, max_travel
        )

    def _guarded_approach(
        self,
        gcmd,
        toolhead,
        bridge,
        axis,
        direction,
        speed,
        max_travel,
        entry,
        stepper_enable,
        rail,
        servo_handle,
        servo_limits,
    ):
        return _run_servo_guarded_trip(
            gcmd,
            bridge,
            axis,
            stepper_enable,
            rail,
            servo_handle,
            servo_limits,
            lambda: self.trip_move(
                gcmd,
                toolhead,
                bridge,
                axis,
                direction,
                speed,
                max_travel,
                entry,
            ),
        )

    def _home_axis(
        self,
        gcmd,
        toolhead,
        bridge,
        kin,
        axis,
        entry,
        speed_override=None,
        max_travel_override=None,
    ):
        rail = kin._axis_rails().get(axis)
        if rail is None:
            raise gcmd.error("G28: no rail for axis %s" % ("XYZ"[axis],))
        hi = rail.get_homing_info()
        pos_min, pos_max = rail.get_range()
        trigger_height = entry["trigger_height"]
        if trigger_height is None:
            trigger_height = hi.position_endstop
        direction = 1.0 if hi.positive_dir else -1.0
        speed = speed_override if speed_override is not None else hi.speed
        max_travel = (
            max_travel_override
            if max_travel_override is not None
            else abs(pos_max - pos_min)
        )

        stepper_enable = self.printer.lookup_object("stepper_enable")
        homing_deltas = [0.0, 0.0, 0.0]
        homing_deltas[axis] = 1.0
        homing_names = []
        for active_rail in kin.active_rails(*homing_deltas):
            homing_names.extend(_homing_motor_names(active_rail))
        stepper_enable.motor_enable_group(homing_names)

        servo_handle = None
        servo_limits = None
        if hasattr(rail, "get_node_name"):
            node = self.printer.lookup_object(
                "ethercat_node " + rail.get_node_name()
            )
            servo_handle = node.get_bridge_handle()
            servo_limits = rail.get_homing_drive_limits()

        self._set_homing_current(toolhead, rail, pre_homing=True)
        try:
            provider = entry["provider"]
            tolerance = get_danger_options().homing_elapsed_distance_tolerance

            def approach(spd, mt):
                return self._guarded_approach(
                    gcmd,
                    toolhead,
                    bridge,
                    axis,
                    direction,
                    spd,
                    mt,
                    entry,
                    stepper_enable,
                    rail,
                    servo_handle,
                    servo_limits,
                )

            trip_pos, final_pos = _run_homing_attempts(
                gcmd,
                toolhead,
                axis,
                direction,
                hi,
                trigger_height,
                provider,
                max_travel,
                tolerance,
                approach,
            )
            _commit_and_seed(
                toolhead,
                bridge,
                axis,
                direction,
                hi,
                trip_pos,
                final_pos,
                trigger_height,
                provider,
                servo_handle,
            )
            _check_servo_drive_fault(gcmd, bridge, axis, servo_handle)
        except BaseException:
            try:
                self._set_homing_current(toolhead, rail, pre_homing=False)
            except Exception:
                logging.exception(
                    "homing: current restore failed during error unwind"
                )
            raise
        else:
            self._set_homing_current(toolhead, rail, pre_homing=False)

    def _set_homing_current(self, toolhead, rail, pre_homing):
        print_time = toolhead.get_last_move_time()
        dwell_time = 0.0
        for current_helper in rail.get_tmc_current_helpers():
            if current_helper is None:
                continue
            dwell_time = max(
                dwell_time,
                current_helper.set_current_for_homing(print_time, pre_homing),
            )
        if dwell_time:
            toolhead.dwell(dwell_time)

    def _drain_motion_before_arming_device(self, gcmd, bridge, axis):
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + _DRAIN_PAUSE_TIMEOUT
        while not bridge.motion_drained():
            if reactor.monotonic() > deadline:
                raise gcmd.error(
                    "%s trip move: motion did not drain within %.0fs before"
                    " homing" % ("XYZ"[axis], _DRAIN_PAUSE_TIMEOUT)
                )
            reactor.pause(reactor.monotonic() + 0.005)

    def trip_move(
        self, gcmd, toolhead, bridge, axis, direction, speed, max_travel, entry
    ):
        endstop = entry["endstop"]
        endstop_mcu = endstop.bridge_mcu_handle()
        if endstop_mcu is None:
            raise gcmd.error(
                "trip_move: endstop MCU for axis %s is not attached to the"
                " bridge" % ("XYZ"[axis],)
            )
        toolhead.wait_moves()
        self._drain_motion_before_arming_device(gcmd, bridge, axis)
        provider = entry["provider"]
        if provider is not None and hasattr(provider, "trip_move_begin"):
            provider.trip_move_begin(entry)
        try:
            endstop.arm(HOMING_POLL_PERIOD)
            bridge.home_axis_start(
                axis,
                direction,
                speed,
                max_travel,
                endstop.endstop_id,
                endstop_mcu,
            )
            reactor = self.printer.get_reactor()
            deadline = (
                reactor.monotonic() + max_travel / speed + TRIP_DEADLINE_MARGIN
            )
            while True:
                try:
                    result = bridge.home_axis_poll()
                except Exception as e:
                    bridge.home_abort()
                    raise gcmd.error(
                        "%s trip move failed: %s" % ("XYZ"[axis], e)
                    )
                if result is not None:
                    break
                if reactor.monotonic() > deadline:
                    bridge.home_abort()
                    raise gcmd.error(
                        _no_trigger_error_message(axis, endstop, max_travel)
                    )
                reactor.pause(reactor.monotonic() + 0.010)
        finally:
            disarm = getattr(endstop, "disarm", None)
            if disarm is not None:
                try:
                    disarm()
                except Exception:
                    logging.exception(
                        "trip_move: remote trigger disarm failed during unwind"
                    )
            if provider is not None and hasattr(provider, "trip_move_end"):
                provider.trip_move_end(entry)
        trip_pos, final_pos, trip_clock = result
        _verify_latched_trip(gcmd, axis, endstop, trip_clock)
        return trip_pos, final_pos


def load_config(config):
    return Homing(config)
