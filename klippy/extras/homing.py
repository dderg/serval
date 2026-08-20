import contextlib
import logging

from klippy import engine_wait, structured_log
from klippy.extras.danger_options import get_danger_options
from klippy.motion_endstop import (
    AXIS_ENDSTOP_IDS,
    MotionEndstop,
    MotorBinding,
    allocate_provider_id,
    endstop_entry,
    entry_endstops,
)

HOMING_POLL_PERIOD = 0.001
HOMING_TRAVEL_MARGIN_FACTOR = 1.5
_DRAIN_PAUSE_TIMEOUT = 60.0


def _endstop_section(config, axis_name):
    section = "axis " + axis_name
    if config.has_section(section):
        return section
    return None


def _parse_keyed_endstop_pins(axis_config, section, endstop_pin):
    pins_by_motor = {}
    for line in endstop_pin.split("\n"):
        entry = line.strip()
        if not entry:
            continue
        if ":" not in entry:
            raise axis_config.error(
                "[%s] endstop_pin line '%s' must read 'motor_name: pin'"
                % (section, entry)
            )
        motor_name, pin = entry.split(":", 1)
        motor_name = motor_name.strip()
        pin = pin.strip()
        if not motor_name or not pin:
            raise axis_config.error(
                "[%s] endstop_pin line '%s' must read 'motor_name: pin'"
                % (section, entry)
            )
        if motor_name in pins_by_motor:
            raise axis_config.error(
                "[%s] endstop_pin lists motor '%s' twice"
                % (section, motor_name)
            )
        pins_by_motor[motor_name] = pin
    if not pins_by_motor:
        raise axis_config.error("[%s] endstop_pin is empty" % (section,))
    return pins_by_motor


def _lane_motors(axis_config, kin, axis_index, section):
    axis_name = "xyz"[axis_index]
    if kin.coupled_xy() and axis_index in (0, 1):
        raise axis_config.error(
            "[%s] per-motor endstop_pin needs an axis that maps to exactly one"
            " motor lane; %s kinematics drives x and y through a shared lane"
            % (section, kin.kind)
        )
    position, lane = next(
        (
            (position, lane)
            for position, lane in enumerate(kin.lanes())
            if lane[1] == axis_name
        ),
        (None, None),
    )
    if lane is None:
        raise axis_config.error(
            "[%s] per-motor endstop_pin: no motor lane drives axis %s"
            % (section, axis_name)
        )
    lane_idx = lane[0]
    if lane_idx != axis_index:
        raise axis_config.error(
            "[%s] per-motor endstop_pin: axis %s is driven by lane %d, not its"
            " own lane" % (section, axis_name, lane_idx)
        )
    steppers = kin.rails[position].get_steppers()
    motor_names = list(lane[2])
    if len(steppers) != len(motor_names):
        raise axis_config.error(
            "[%s] per-motor endstop_pin: lane %d declares motors %s but drives"
            " %d steppers"
            % (section, lane_idx, ", ".join(motor_names), len(steppers))
        )
    return motor_names, steppers


def _check_motor_keys(axis_config, section, motor_names, pins_by_motor):
    known = set(motor_names)
    for motor_name in pins_by_motor:
        if motor_name not in known:
            raise axis_config.error(
                "[%s] endstop_pin names motor '%s', which does not drive this"
                " axis; its motors are %s"
                % (section, motor_name, ", ".join(motor_names))
            )
    for motor_name in motor_names:
        if motor_name not in pins_by_motor:
            raise axis_config.error(
                "[%s] endstop_pin is missing motor '%s'; a per-motor"
                " endstop_pin must list every motor of the axis (%s)"
                % (section, motor_name, ", ".join(motor_names))
            )


def _homing_motor_names(rail):
    steppers = rail.get_steppers()
    if not steppers:
        return [rail.get_name()]
    return [s.get_name() for s in steppers]


@contextlib.contextmanager
def _servo_drive_limits(engine, handle, drives):
    if handle is None or not drives:
        yield
        return
    slots = [slot for slot, _, _ in drives]
    engine.set_drive_limits(handle, drives)
    try:
        yield
    except BaseException:
        try:
            engine.restore_drive_limits(handle, slots)
        except Exception:
            logging.warning(
                "homing: restore_drive_limits failed while handling a"
                " homing error",
                exc_info=True,
            )
        raise
    engine.restore_drive_limits(handle, slots)


def _drive_limits_by_handle(servo_rails):
    grouped = {}
    for entry in servo_rails:
        if entry["handle"] is None or entry["limits"] is None:
            continue
        grouped.setdefault(entry["handle"], []).append(
            (entry["slot"], entry["limits"][0], entry["limits"][1])
        )
    return grouped


def _run_servo_guarded_trip(
    gcmd,
    engine,
    axis,
    stepper_enable,
    servo_rails,
    trip,
):
    try:
        with contextlib.ExitStack() as stack:
            for handle, drives in _drive_limits_by_handle(servo_rails).items():
                stack.enter_context(_servo_drive_limits(engine, handle, drives))
            result = trip()
        _check_servo_drive_fault(gcmd, engine, axis, servo_rails)
    except BaseException:
        for entry in servo_rails:
            stepper_enable.motor_debug_enable(entry["rail"].get_name(), False)
        raise
    return result


def _servo_handles(servo_rails):
    handles = []
    for entry in servo_rails:
        if all(entry["handle"] is not h for h in handles):
            handles.append(entry["handle"])
    return handles


def _check_servo_drive_fault(gcmd, engine, axis, servo_rails):
    for handle in _servo_handles(servo_rails):
        fault = engine.take_drive_fault(handle)
        if fault is not None:
            raise gcmd.error(
                "%s homing: drive fault 0x%04x at endstop contact — "
                "following-error/torque limit exceeded" % ("XYZ"[axis], fault)
            )


def _homed_axis_position(provider, axis, trip_pos, final_pos, trigger_position):
    if provider is not None and hasattr(provider, "measured_trip_position"):
        measured = provider.measured_trip_position(axis, trip_pos, final_pos)
        if measured is not None:
            return measured
    return trigger_position + (final_pos[axis] - trip_pos[axis])


def _commit_and_seed(
    toolhead,
    engine,
    axis,
    direction,
    hi,
    trip_pos,
    final_pos,
    trigger_position,
    provider,
    servo_handle,
):
    overshoot = final_pos[axis] - trip_pos[axis]
    newpos = list(toolhead.get_position())
    newpos[axis] = _homed_axis_position(
        provider, axis, trip_pos, final_pos, trigger_position
    )
    toolhead.set_position(newpos, homing_axes=[axis])
    structured_log.event(
        "homing",
        "axis_homed",
        msg="homing: %s trigger=%.4f overshoot=%+.4f set %s=%.4f"
        % ("XYZ"[axis], trigger_position, overshoot, "XYZ"[axis], newpos[axis]),
        axis="XYZ"[axis],
        trigger_position=trigger_position,
        overshoot=overshoot,
        homed_position=newpos[axis],
    )
    if hi.retract_dist:
        retractpos = list(toolhead.get_position())
        retractpos[axis] -= direction * hi.retract_dist + overshoot
        toolhead.move(retractpos, hi.retract_speed)
        toolhead.wait_moves()
    if servo_handle is not None:
        engine.finalize_homed_axis(
            servo_handle, axis, toolhead.get_position()[:3]
        )


def _run_homing_attempts(
    gcmd,
    toolhead,
    axis,
    direction,
    hi,
    speed,
    first_max_travel,
    tolerance,
    trigger_position,
    approach,
):
    start_pos = toolhead.get_position()
    trip_pos, final_pos = approach(speed, first_max_travel)
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
    haltpos[axis] = trigger_position + (final_pos[axis] - trip_pos[axis])
    toolhead.set_position(haltpos, homing_axes=[axis])
    backoff = list(toolhead.get_position())
    backoff[axis] = trigger_position - direction * hi.min_home_dist
    toolhead.move(backoff, hi.retract_speed)
    toolhead.wait_moves()
    start_pos = toolhead.get_position()
    # The early trip may have been spurious mid-travel (StallGuard blip),
    # leaving the real switch far beyond the relabeled coordinates; the
    # re-approach gets the full travel budget and the too-early re-check
    # below still rejects a stuck or miswired endstop.
    trip_pos, final_pos = approach(speed, first_max_travel)
    traveled = abs(trip_pos[axis] - start_pos[axis])
    if _trigger_too_early(traveled, hi.min_home_dist, tolerance):
        raise gcmd.error(
            "%s early homing trigger: endstop tripped after only %.2fmm on "
            "re-approach (min_home_dist %.2fmm) — false trigger or "
            "stuck/miswired endstop" % ("XYZ"[axis], traveled, hi.min_home_dist)
        )
    return trip_pos, final_pos


def _homing_max_travel(hi, pos_min, pos_max):
    if hi.positive_dir:
        homing_span = hi.position_endstop - pos_min
    else:
        homing_span = pos_max - hi.position_endstop
    return HOMING_TRAVEL_MARGIN_FACTOR * homing_span


def _trigger_too_early(traveled, min_home_dist, tolerance):
    if min_home_dist <= 0.0:
        return False
    return traveled < min_home_dist and (min_home_dist - traveled) >= tolerance


def _endstop_label(axis, endstop):
    motor_name = getattr(endstop, "motor_name", None)
    if motor_name is None:
        return "%s endstop" % ("XYZ"[axis],)
    return "%s endstop %s" % ("XYZ"[axis], motor_name)


def _latched_trip(gcmd, axis, endstop):
    query = getattr(endstop, "query_trip_state", None)
    if query is None:
        return None
    latch = query()
    if not latch["tripped"]:
        raise gcmd.error(
            "%s: doorbell event arrived but the MCU latch shows no"
            " trip — duplicate or stale trip event"
            % (_endstop_label(axis, endstop),)
        )
    return latch


def _verify_latched_trip(gcmd, axis, endstop, doorbell_clock):
    latch = _latched_trip(gcmd, axis, endstop)
    if latch is None:
        return
    if latch["trip_clock"] != (doorbell_clock & 0xFFFFFFFF):
        raise gcmd.error(
            "%s: latch/doorbell clock mismatch — latch=%d"
            " doorbell_low32=%d"
            % (
                _endstop_label(axis, endstop),
                latch["trip_clock"],
                doorbell_clock & 0xFFFFFFFF,
            )
        )


def _verify_latched_trips(gcmd, axis, endstops, doorbell_clock):
    """The doorbell carries the clock of the trip that resolved the run; the
    other switches of a multi-endstop axis tripped earlier and only have to
    show a latched trip."""
    if len(endstops) == 1:
        _verify_latched_trip(gcmd, axis, endstops[0], doorbell_clock)
        return
    latched = [_latched_trip(gcmd, axis, e) for e in endstops]
    observed = [latch for latch in latched if latch is not None]
    if not observed:
        return
    if all(
        latch["trip_clock"] != (doorbell_clock & 0xFFFFFFFF)
        for latch in observed
    ):
        raise gcmd.error(
            "%s homing: no endstop latch matches the doorbell clock"
            " (doorbell_low32=%d, latches=%s)"
            % (
                "XYZ"[axis],
                doorbell_clock & 0xFFFFFFFF,
                ", ".join(str(latch["trip_clock"]) for latch in observed),
            )
        )


def _no_trigger_error_message(axis, endstops, max_travel):
    base = "%s endstop did not trigger within %.1fmm of travel" % (
        "XYZ"[axis],
        max_travel,
    )
    latched = []
    for endstop in endstops:
        query = getattr(endstop, "query_trip_state", None)
        latched.append((endstop, None if query is None else query()))
    silent = [
        endstop
        for endstop, latch in latched
        if latch is not None and not latch["tripped"]
    ]
    tripped = [
        (endstop, latch)
        for endstop, latch in latched
        if latch is not None and latch["tripped"]
    ]
    if not tripped:
        if len(endstops) > 1 and silent:
            return "%s (%s never tripped)" % (
                base,
                ", ".join(_endstop_label(axis, e) for e in silent),
            )
        return base
    lost = ", ".join(
        "%s (latched clock %d)" % (_endstop_label(axis, e), latch["trip_clock"])
        for e, latch in tripped
    )
    if silent:
        return (
            "%s tripped but the trip event was lost — doorbell never reached"
            " the host; still waiting on %s"
            % (lost, ", ".join(_endstop_label(axis, e) for e in silent))
        )
    return (
        "%s tripped but the trip event was lost — doorbell never reached"
        " the host" % (lost,)
    )


class HomingState:
    def __init__(self, axes):
        self._axes = list(axes)

    def get_axes(self):
        return list(self._axes)


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

    def resolve_endstops(self, kin):
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
            if "\n" in endstop_pin:
                entry = self._keyed_entry(
                    ppins, kin, axis_config, axis_index, endstop_pin
                )
            else:
                entry = self._single_entry(
                    config, ppins, axis_config, axis_index, endstop_pin
                )
            self._axes[axis_index] = entry

        query_endstops = self.printer.load_object(config, "query_endstops")
        for axis_index in sorted(self._axes):
            axis_name = "xyz"[axis_index]
            for endstop in entry_endstops(self._axes[axis_index]):
                motor_name = getattr(endstop, "motor_name", None)
                query_endstops.register_endstop(
                    endstop,
                    axis_name
                    if motor_name is None
                    else "%s:%s" % (axis_name, motor_name),
                )

    def _single_entry(
        self, config, ppins, axis_config, axis_index, endstop_pin
    ):
        pin_params = ppins.parse_pin(
            endstop_pin, can_invert=True, can_pullup=True
        )
        chip = pin_params["chip"]
        if hasattr(chip, "setup_motion_endstop"):
            return self._provider_entry(
                axis_config, axis_index, chip, pin_params
            )
        if not hasattr(chip, "create_oid"):
            raise config.error(
                "endstop_pin '%s' in [%s]: chip '%s' is neither an MCU"
                " nor a virtual endstop provider"
                % (
                    endstop_pin,
                    axis_config.get_name(),
                    pin_params["chip_name"],
                )
            )
        return endstop_entry(
            [MotionEndstop(pin_params, AXIS_ENDSTOP_IDS[axis_index])],
            None,
            None,
        )

    def _keyed_entry(self, ppins, kin, axis_config, axis_index, endstop_pin):
        section = axis_config.get_name()
        pins_by_motor = _parse_keyed_endstop_pins(
            axis_config, section, endstop_pin
        )
        motor_names, steppers = _lane_motors(
            axis_config, kin, axis_index, section
        )
        _check_motor_keys(axis_config, section, motor_names, pins_by_motor)
        lane_mcu = steppers[0].get_mcu()
        for motor_name, mcu_stepper in zip(motor_names, steppers):
            if mcu_stepper.get_mcu() is not lane_mcu:
                raise axis_config.error(
                    "[%s] keyed endstop_pin: motor '%s' is driven by MCU '%s'"
                    " but the lane's first motor '%s' is on MCU '%s'; a"
                    " multi-endstop axis must live on one MCU"
                    % (
                        section,
                        motor_name,
                        mcu_stepper.get_mcu().get_name(),
                        motor_names[0],
                        lane_mcu.get_name(),
                    )
                )
        endstops = []
        for stepper_idx, motor_name in enumerate(motor_names):
            pin_params = ppins.parse_pin(
                pins_by_motor[motor_name], can_invert=True, can_pullup=True
            )
            chip = pin_params["chip"]
            if hasattr(chip, "setup_motion_endstop"):
                raise axis_config.error(
                    "[%s] keyed endstop_pin: motor '%s' uses virtual endstop"
                    " chip '%s'; virtual endstops drive one switch per axis"
                    % (section, motor_name, pin_params["chip_name"])
                )
            if not hasattr(chip, "create_oid"):
                raise axis_config.error(
                    "[%s] keyed endstop_pin: motor '%s' pin chip '%s' is not"
                    " an MCU" % (section, motor_name, pin_params["chip_name"])
                )
            endstop_id = (
                AXIS_ENDSTOP_IDS[axis_index]
                if stepper_idx == 0
                else allocate_provider_id(self.printer)
            )
            endstops.append(
                MotionEndstop(
                    pin_params,
                    endstop_id,
                    MotorBinding(
                        axis_index,
                        stepper_idx,
                        steppers[stepper_idx].get_mcu(),
                        motor_name,
                        steppers[stepper_idx].get_oid(),
                    ),
                    group=True,
                )
            )
        return endstop_entry(endstops, None, None)

    def _provider_entry(self, axis_config, axis_index, chip, pin_params):
        endstop = chip.setup_motion_endstop(pin_params, axis_index)
        trigger_position = None
        if hasattr(chip, "get_position_endstop"):
            trigger_position = chip.get_position_endstop()
            if axis_config.get("position_endstop", None) is not None:
                raise axis_config.error(
                    "[%s] must not set position_endstop: its virtual endstop"
                    " '%s' supplies the trigger position"
                    % (axis_config.get_name(), pin_params["chip_name"])
                )
        return endstop_entry([endstop], chip, trigger_position)

    def cmd_G28(self, gcmd):
        if self._axes is None:
            raise gcmd.error("G28: homing endstops were never resolved")
        requested = [
            i for i, a in enumerate("XYZ") if gcmd.get(a, None) is not None
        ]
        if not requested:
            requested = sorted(self._axes.keys())
        toolhead = self.printer.lookup_object("toolhead")
        engine = self.printer.lookup_object("motion_engine")
        kin = toolhead.get_kinematics()
        for axis in requested:
            entry = self._axes.get(axis)
            if entry is None:
                raise gcmd.error("G28: axis %s has no endstop" % ("XYZ"[axis],))
            self._home_axis(gcmd, toolhead, engine, kin, axis, entry)
        self._emit_home_rails_end(kin, requested)

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
        engine = self.printer.lookup_object("motion_engine")
        kin = toolhead.get_kinematics()
        self._home_axis(
            gcmd, toolhead, engine, kin, axis, entry, speed, max_travel
        )
        self._emit_home_rails_end(kin, [axis])

    def _emit_home_rails_end(self, kin, homed_axes):
        axis_rails = kin._axis_rails()
        rails = [axis_rails[axis] for axis in homed_axes]
        self.printer.send_event(
            "homing:home_rails_end", HomingState(homed_axes), rails
        )

    def _guarded_approach(
        self,
        gcmd,
        toolhead,
        engine,
        axis,
        direction,
        speed,
        max_travel,
        entry,
        stepper_enable,
        servo_rails,
    ):
        return _run_servo_guarded_trip(
            gcmd,
            engine,
            axis,
            stepper_enable,
            servo_rails,
            lambda: self.trip_move(
                gcmd,
                toolhead,
                engine,
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
        engine,
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
        trigger_position = entry["trigger_position"]
        if trigger_position is None:
            trigger_position = hi.position_endstop
        direction = 1.0 if hi.positive_dir else -1.0
        speed = speed_override if speed_override is not None else hi.speed
        max_travel = (
            max_travel_override
            if max_travel_override is not None
            else _homing_max_travel(hi, pos_min, pos_max)
        )

        stepper_enable = self.printer.lookup_object("stepper_enable")
        homing_deltas = [0.0, 0.0, 0.0]
        homing_deltas[axis] = 1.0
        active_rails = kin.active_rails(*homing_deltas)
        homing_names = []
        for active_rail in active_rails:
            homing_names.extend(_homing_motor_names(active_rail))
        stepper_enable.motor_enable_group(homing_names)

        servo_rails = self._active_servo_rails(gcmd, axis, active_rails)
        servo_handle = next(
            (sr["handle"] for sr in servo_rails if sr["rail"] is rail), None
        )

        self._set_homing_current(toolhead, active_rails, pre_homing=True)
        try:
            provider = entry["provider"]
            tolerance = get_danger_options().homing_elapsed_distance_tolerance

            def approach(spd, mt):
                return self._guarded_approach(
                    gcmd,
                    toolhead,
                    engine,
                    axis,
                    direction,
                    spd,
                    mt,
                    entry,
                    stepper_enable,
                    servo_rails,
                )

            trip_pos, final_pos = _run_homing_attempts(
                gcmd,
                toolhead,
                axis,
                direction,
                hi,
                speed,
                max_travel,
                tolerance,
                trigger_position,
                approach,
            )
            _commit_and_seed(
                toolhead,
                engine,
                axis,
                direction,
                hi,
                trip_pos,
                final_pos,
                trigger_position,
                provider,
                servo_handle,
            )
            _check_servo_drive_fault(gcmd, engine, axis, servo_rails)
        except BaseException:
            try:
                self._set_homing_current(
                    toolhead, active_rails, pre_homing=False
                )
            except Exception:
                logging.exception(
                    "homing: current restore failed during error unwind"
                )
            kin.clear_homing_state([axis])
            raise
        else:
            self._set_homing_current(toolhead, active_rails, pre_homing=False)

    def _active_servo_rails(self, gcmd, axis, active_rails):
        servo_rails = []
        for active_rail in active_rails:
            if not hasattr(active_rail, "get_motors"):
                continue
            for motor in active_rail.get_motors():
                limits = motor.get_homing_drive_limits()
                if limits[1] <= 0:
                    raise gcmd.error(
                        "%s homing: motor %s drives this axis but has no"
                        " homing torque limit (its axis has no endstop_pin)"
                        % ("XYZ"[axis], motor.get_motor_name())
                    )
                node = self.printer.lookup_object(
                    "ethercat_node " + motor.get_node_name()
                )
                servo_rails.append(
                    {
                        "rail": active_rail,
                        "handle": node.get_engine_handle(),
                        "slot": node.get_slot_for_motor(motor.get_motor_name()),
                        "limits": limits,
                    }
                )
        return servo_rails

    def _set_homing_current(self, toolhead, rails, pre_homing):
        print_time = toolhead.get_last_move_time()
        dwell_time = 0.0
        seen = set()
        for rail in rails:
            for current_helper in rail.get_tmc_current_helpers():
                if current_helper is None or id(current_helper) in seen:
                    continue
                seen.add(id(current_helper))
                dwell_time = max(
                    dwell_time,
                    current_helper.set_current_for_homing(
                        print_time, pre_homing
                    ),
                )
        if dwell_time:
            toolhead.dwell(dwell_time)

    def _drain_motion_before_arming_device(self, gcmd, engine, axis):
        try:
            engine_wait.wait_for(
                self.printer,
                lambda: engine.motion_drained() or None,
                "%s homing motion drain" % ("XYZ"[axis],),
                _DRAIN_PAUSE_TIMEOUT,
            )
        except engine_wait.EngineWaitTimeout:
            raise gcmd.error(
                "%s trip move: motion did not drain within %.0fs before"
                " homing" % ("XYZ"[axis], _DRAIN_PAUSE_TIMEOUT)
            )

    def _abort_trip_and_adopt_stop_position(self, gcmd, toolhead, engine, axis):
        """A trip move that never triggered still physically moved the
        toolhead; the host must adopt the engine's reconciled stop position
        or every later hop/retract decision runs on the pre-trip height —
        on a tilted bed that sends the next G28's travel move through the
        bed at the lowest corner."""
        stop_pos = engine.home_abort()
        if stop_pos is None:
            raise gcmd.error(
                "%s trip move aborted but the toolhead stop position could"
                " not be reconciled — position is unknown; run"
                " FIRMWARE_RESTART before any further motion" % ("XYZ"[axis],)
            )
        newpos = list(toolhead.get_position())
        newpos[:3] = stop_pos
        toolhead.set_position(newpos)
        structured_log.event(
            "homing",
            "trip_aborted_position_adopted",
            msg="homing: %s trip aborted; toolhead position reconciled to"
            " %.4f,%.4f,%.4f"
            % ("XYZ"[axis], stop_pos[0], stop_pos[1], stop_pos[2]),
            axis="XYZ"[axis],
            stop_x=stop_pos[0],
            stop_y=stop_pos[1],
            stop_z=stop_pos[2],
        )

    def trip_move(
        self, gcmd, toolhead, engine, axis, direction, speed, max_travel, entry
    ):
        endstops = entry_endstops(entry)
        for endstop in endstops:
            if endstop.engine_mcu_handle() is None:
                raise gcmd.error(
                    "trip_move: %s is not attached to the engine"
                    % (_endstop_label(axis, endstop),)
                )
        toolhead.wait_moves()
        self._drain_motion_before_arming_device(gcmd, engine, axis)
        provider = entry["provider"]
        if provider is not None and hasattr(provider, "trip_move_begin"):
            provider.trip_move_begin(entry)
        try:
            for endstop in endstops:
                endstop.arm(HOMING_POLL_PERIOD)
            engine.home_axis_start(
                axis,
                direction,
                speed,
                max_travel,
                [
                    (e.endstop_id, e.engine_mcu_handle(), e.remote_freeze())
                    for e in endstops
                ],
            )
            try:
                result = engine_wait.wait_for(
                    self.printer,
                    engine.home_axis_poll,
                    "%s trip move" % ("XYZ"[axis],),
                    max_travel / speed
                    + get_danger_options().homing_trip_deadline_margin,
                    interval_s=0.010,
                )
            except engine_wait.EngineWaitTimeout:
                self._abort_trip_and_adopt_stop_position(
                    gcmd, toolhead, engine, axis
                )
                raise gcmd.error(
                    _no_trigger_error_message(axis, endstops, max_travel)
                )
            except Exception as e:
                self._abort_trip_and_adopt_stop_position(
                    gcmd, toolhead, engine, axis
                )
                raise gcmd.error("%s trip move failed: %s" % ("XYZ"[axis], e))
        finally:
            for endstop in endstops:
                disarm = getattr(endstop, "disarm", None)
                if disarm is None:
                    continue
                try:
                    disarm()
                except Exception:
                    logging.exception(
                        "trip_move: remote trigger disarm failed during unwind"
                    )
            if provider is not None and hasattr(provider, "trip_move_end"):
                provider.trip_move_end(entry)
        trip_pos, final_pos, trip_clock = result
        _verify_latched_trips(gcmd, axis, endstops, trip_clock)
        reconciled = list(toolhead.get_position())
        reconciled[:3] = final_pos
        toolhead.set_position(reconciled)
        return trip_pos, final_pos


def load_config(config):
    return Homing(config)
