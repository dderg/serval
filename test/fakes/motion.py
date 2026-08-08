class FakeKin:
    def __init__(
        self,
        rails=(),
        kind=None,
        coupled_xy=None,
        lanes=None,
        limits=None,
        parked_dirty=(),
        axis_rails=None,
        active_rails_result=None,
        get_steppers_result=None,
        get_status_ranges=None,
        check_move_fn=None,
    ):
        self.rails = list(rails)
        self.kind = kind
        self._coupled_xy = (
            (kind == "corexy") if coupled_xy is None else coupled_xy
        )
        self._lanes = (
            list(lanes)
            if lanes is not None
            else [
                (i, axis, [])
                for i, axis in enumerate(("x", "y", "z")[: len(self.rails)])
            ]
        )
        self.limits = list(limits) if limits is not None else [(1.0, -1.0)] * 3
        self._parked_dirty = list(parked_dirty)
        self._axis_rails_map = axis_rails
        self._active_rails_result = active_rails_result
        self._get_steppers_result = get_steppers_result
        self._get_status_ranges = get_status_ranges
        self._check_move_fn = check_move_fn
        self.cleared = []
        self.parked = []
        self.checked = []

    def coupled_xy(self):
        return self._coupled_xy

    def lanes(self):
        return self._lanes

    def claimed_axes(self):
        return [axis for _, axis, _ in self._lanes]

    def mcu_tag(self, lanes_on_mcu):
        on_mcu = set(lanes_on_mcu)
        if self.coupled_xy() and 0 in on_mcu and 1 in on_mcu:
            return 0
        return 1

    def get_steppers(self):
        if self._get_steppers_result is not None:
            return self._get_steppers_result
        return [s for rail in self.rails for s in rail.get_steppers()]

    def active_rails(self, dx, dy, dz):
        if self._active_rails_result is not None:
            return self._active_rails_result
        moved = [abs(dx) > 1e-9, abs(dy) > 1e-9, abs(dz) > 1e-9]
        if self.coupled_xy():
            moved[0] = moved[1] = moved[0] or moved[1]
        return [
            self.rails[lane_idx]
            for lane_idx, _axis, _motors in self._lanes
            if moved[lane_idx]
        ]

    def calc_position(self, stepper_positions):
        def rail_pos(rail):
            vals = [
                stepper_positions.get(s.get_name(), 0.0)
                for s in rail.get_steppers()
            ]
            return sum(vals) / len(vals) if vals else 0.0

        return [rail_pos(rail) for rail in self.rails]

    def _axis_rails(self):
        if self._axis_rails_map is not None:
            return dict(self._axis_rails_map)
        return {i: rail for i, rail in enumerate(self.rails)}

    def parked_dirty_axes(self):
        return list(self._parked_dirty)

    def clear_parked_dirty(self, axes):
        self.cleared.append(list(axes))
        self._parked_dirty = [a for a in self._parked_dirty if a not in axes]

    def mark_servo_parked(self, axes):
        self.parked.append(tuple(axes))
        for axis in axes:
            if axis not in self._parked_dirty:
                self._parked_dirty.append(axis)

    def note_z_not_homed(self):
        self.clear_homing_state([2])

    def clear_homing_state(self, axes):
        for i in axes:
            self.limits[i] = (1.0, -1.0)

    def set_position(self, newpos, homing_axes=()):
        for axis in homing_axes:
            if axis < len(self.rails):
                self.limits[axis] = self.rails[axis].get_range()
            self._parked_dirty = [a for a in self._parked_dirty if a != axis]

    def check_move(self, move):
        self.checked.append(tuple(move.end_pos))
        if self._check_move_fn is not None:
            self._check_move_fn(move)

    def get_status(self, eventtime):
        from klippy import gcode as gcode_mod

        ranges = self._get_status_ranges or [
            r.get_range() for r in self.rails[:3]
        ]
        (x_min, x_max), (y_min, y_max), (z_min, z_max) = ranges
        homed = "".join(
            a
            for i, a in enumerate("xyz")
            if self.limits[i][0] <= self.limits[i][1]
        )
        return {
            "homed_axes": homed,
            "axis_minimum": gcode_mod.Coord(x_min, y_min, z_min, 0.0),
            "axis_maximum": gcode_mod.Coord(x_max, y_max, z_max, 0.0),
        }


class FakeEngine:
    def __init__(self, raises=None, **returns):
        self.calls = []
        self.raises = raises
        self._returns = dict(returns)
        self._returns.setdefault("motion_drained", True)
        self._returns.setdefault("query_motor_positions", {})
        self._returns.setdefault("live_motor_positions", {})
        self._returns.setdefault("sdo_read", (2, 100))
        self._returns.setdefault("sdo_write", (2, 100))
        self._returns.setdefault("stop_servo_capture", (0, 1234, None))
        self._returns.setdefault("engine_call", {})
        self._returns.setdefault("queued_motion_secs", 0.0)
        self._returns.setdefault("dispatched_lead_secs", 0.0)
        self._returns.setdefault("get_last_move_time", 0.0)

    def _call(self, name, *args):
        self.calls.append((name,) + args)
        if self.raises is not None:
            raise self.raises
        value = self._returns.get(name)
        if isinstance(value, list):
            return value.pop(0) if value else None
        return value

    def motion_lead_secs(self):
        return self._call("motion_lead_secs")

    def motion_state_at(self, mcu, clock=None, print_time=None):
        return self._call("motion_state_at", mcu, clock, print_time)

    def shutdown(self):
        return self._call("shutdown")

    def resonance_buzz(
        self,
        handle,
        slot_mask,
        slot_sign_mask,
        freq_start_millihz,
        freq_end_millihz,
        amplitude_nm,
        duration_ms,
        ramp_ms,
    ):
        return self._call(
            "resonance_buzz",
            handle,
            slot_mask,
            slot_sign_mask,
            freq_start_millihz,
            freq_end_millihz,
            amplitude_nm,
            duration_ms,
            ramp_ms,
        )

    def set_torque(self, handle, value, print_time):
        return self._call("set_torque", handle, value, print_time)

    def set_torque_deferred(self, handle, value, print_time):
        return self._call("set_torque_deferred", handle, value, print_time) or (
            lambda: None
        )

    def query_motor_positions(self):
        return self._call("query_motor_positions")

    def finalize_homed_axis(self, handle, axis, pos_mm):
        return self._call("finalize_homed_axis", handle, axis, pos_mm)

    def arm_remote_trigger(self, mcu_handle, trsync_oid, endstop_id):
        return self._call(
            "arm_remote_trigger", mcu_handle, trsync_oid, endstop_id
        )

    def disarm_remote_trigger(self, endstop_id):
        return self._call("disarm_remote_trigger", endstop_id)

    def frontier_print_time(self, mcu_handle):
        return self._call("frontier_print_time", mcu_handle)

    def queued_motion_secs(self):
        return self._call("queued_motion_secs")

    def submit_move(self, dx, dy, dz, de, feedrate):
        return self._call("submit_move", dx, dy, dz, de, feedrate)

    def dispatched_lead_secs(self):
        return self._call("dispatched_lead_secs")

    def get_last_move_time(self):
        return self._call("get_last_move_time")

    def motion_drained(self):
        return self._call("motion_drained")

    def set_position(self, x, y, z):
        return self._call("set_position", x, y, z)

    def live_motor_positions(self):
        return self._call("live_motor_positions")

    def sdo_read(self, handle, slot, index, subindex):
        return self._call("sdo_read", handle, slot, index, subindex)

    def sdo_write(self, handle, slot, index, subindex, size, value):
        return self._call(
            "sdo_write", handle, slot, index, subindex, size, value
        )

    def set_strain_comp(self, handle, slot_a, slot_b, *args):
        return self._call("set_strain_comp", handle, slot_a, slot_b, *args)

    def set_drive_limits(self, handle, drives):
        return self._call("set_drive_limits", handle, drives)

    def restore_drive_limits(self, handle, slots):
        return self._call("restore_drive_limits", handle, slots)

    def take_drive_fault(self, handle):
        return self._call("take_drive_fault", handle)

    def take_endpoint_death(self, handle):
        return self._call("take_endpoint_death", handle)

    def arm_sensorless_endstop(
        self, handle, slot, endstop_id, torque_trip_tenth_pct, enable
    ):
        return self._call(
            "arm_sensorless_endstop",
            handle,
            slot,
            endstop_id,
            torque_trip_tenth_pct,
            enable,
        )

    def disarm_sensorless_endstop(self, handle, slot, endstop_id):
        return self._call("disarm_sensorless_endstop", handle, slot, endstop_id)

    def start_servo_capture(self, handle, path, started_utc, drives):
        return self._call(
            "start_servo_capture", handle, path, started_utc, drives
        )

    def stop_servo_capture(self, handle):
        return self._call("stop_servo_capture", handle)

    def engine_send(self, handle, msg):
        return self._call("engine_send", handle, msg)

    def engine_call(self, handle, msg, response):
        return self._call("engine_call", handle, msg, response)

    def engine_get_clock_async(self, handle):
        return self._call("engine_get_clock_async", handle)

    def claim_mcu(self, name, serial_path, baud):
        self.calls.append(("claim_mcu", name, serial_path, baud))
        return self._returns.get("claim_mcu", 7)

    def fence_start(self, force):
        return self._call("fence_start", force)

    def fence_print_time_poll(self, fence_id, mcu_handle):
        return self._call("fence_print_time_poll", fence_id, mcu_handle)

    def submit_dwell(self, delay):
        return self._call("submit_dwell", delay)

    def submit_nudge(
        self, mcu_id, axis_idx, motor_mask, delta_mm, speed, accel
    ):
        return self._call(
            "submit_nudge", mcu_id, axis_idx, motor_mask, delta_mm, speed, accel
        )


class FakeToolhead:
    def __init__(
        self,
        kin=None,
        position=None,
        last_move_time=0.0,
        move_time_step=0.0,
        max_velocity=300.0,
        max_accel=3000.0,
        max_axis_accel=None,
        motor_binding=None,
        follower_steppers=(),
        mcu=None,
        engine=None,
        planner_ready=True,
        extruder=None,
        nudge_duration=None,
        resync_delay=0.0,
    ):
        self.kin = kin
        self.position = (
            list(position) if position is not None else [0.0, 0.0, 0.0, 0.0]
        )
        self.calls = []
        self._last_move_time = last_move_time
        self._move_time_step = move_time_step
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self._max_axis_accel = max_axis_accel
        self._motor_binding = motor_binding
        self.follower_steppers = list(follower_steppers)
        self.mcu = mcu
        self.engine = engine
        self._planner_ready = planner_ready
        self.extruder = extruder
        self._nudge_duration = nudge_duration
        self._resync_delay = resync_delay

    def get_kinematics(self):
        return self.kin

    def get_position(self):
        return list(self.position)

    def set_position(self, newpos, homing_axes=()):
        self.calls.append(("set_position", list(newpos), tuple(homing_axes)))
        self.position[:] = newpos
        if self.kin is not None:
            self.kin.set_position(newpos, homing_axes)

    def manual_move(self, coord, speed):
        self.calls.append(("manual_move", tuple(coord), speed))
        for i, c in enumerate(coord):
            if c is not None:
                self.position[i] = c

    def move(self, newpos, speed):
        self.calls.append(("move", list(newpos), speed))
        self.position[:] = newpos

    def dwell(self, delay):
        self.calls.append(("dwell", delay))
        self._last_move_time += delay

    def wait_moves(self):
        self.calls.append(("wait_moves",))

    def wait_moves_and_mcu(self):
        self.calls.append(("wait_moves_and_mcu",))

    def wait_until_print_time(self, print_time):
        self.calls.append(("wait_until_print_time", print_time))

    def flush_step_generation(self):
        self.calls.append(("flush_step_generation",))

    def get_last_move_time(self):
        self.calls.append(("get_last_move_time",))
        self._last_move_time += self._move_time_step
        return self._last_move_time

    def get_max_velocity(self):
        return self.max_velocity, self.max_accel

    def get_max_axis_accel(self, axis_idx):
        return self._max_axis_accel

    def get_motor_binding(self, stepper_name):
        return self._motor_binding

    def submit_nudge(self, mcu_id, axis_idx, motor_idx, dist, speed, accel):
        self.calls.append(
            ("submit_nudge", mcu_id, axis_idx, motor_idx, dist, speed, accel)
        )
        return self._nudge_duration

    def get_extruder(self):
        return self.extruder

    def set_extruder(self, extruder, extrude_pos):
        self.calls.append(("set_extruder", extruder, extrude_pos))
        self.extruder = extruder
        self.position[3] = extrude_pos

    def resync_parked_servos(self):
        self.calls.append(("resync_parked_servos",))
        self._last_move_time += self._resync_delay


class FakeMotion:
    def __init__(
        self,
        kin=None,
        engine=None,
        commanded_pos=None,
        axis_sections=(),
        max_z_velocity=10.0,
        max_z_accel=100.0,
        max_velocity=300.0,
        max_accel=3000.0,
        mcu=None,
    ):
        self.kin = kin
        self.engine = engine if engine is not None else FakeEngine()
        self.commanded_pos = (
            list(commanded_pos)
            if commanded_pos is not None
            else [0.0, 0.0, 0.0, 0.0]
        )
        self.axis_sections = list(axis_sections)
        self.max_z_velocity = max_z_velocity
        self.max_z_accel = max_z_accel
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.mcu = mcu
        self.set_position_calls = []

    def get_position(self):
        return list(self.commanded_pos)

    def _await_clock_sync(self):
        pass

    def set_position(self, newpos, homing_axes=()):
        self.set_position_calls.append((list(newpos), tuple(homing_axes)))
        self.commanded_pos[:] = newpos
