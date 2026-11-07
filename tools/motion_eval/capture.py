"""Run klippy (batch mode, or live with a real MCU) and record what the
planner produced.

Invoked as a subprocess by run.py and live.py:
    python -m tools.motion_eval.capture CAPTURE_JSON <klippy args...>

ToolHead is wrapped (not modified) so that every move appended to the
toolhead trapq, the stepper geometry, the input shaper parameters in force
for each move and the junction limiter's results are written to
CAPTURE_JSON once klippy exits. MOTION_EVAL_LAZY_FLUSH_AT (comma separated
add_move indices) makes the lookahead flush lazily after those moves, as it
does when its flush time runs out. MOTION_EVAL_EXIT ends a live run.
"""

from __future__ import annotations

import collections
import json
import math
import os
import sys
import time

from klippy import chelper, printer, serialhdl, toolhead
from klippy.extras import input_shaper, motion_queuing

TRAPQ_APPEND_FIELDS = (
    "print_time",
    "accel_t",
    "cruise_t",
    "decel_t",
    "start_x",
    "start_y",
    "start_z",
    "axes_r_x",
    "axes_r_y",
    "axes_r_z",
    "start_v",
    "cruise_v",
    "accel",
)

DRAIN_TIMEOUT_S = 30.0
DRAIN_POLL_S = 0.005
UNIT_COORDS = (
    (0.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
LINEARITY_PROBE = (37.0, -11.0, 5.0)


class UnsupportedRun(Exception):
    pass


def _stepper_linear_map(stepper):
    _, ffi_lib = chelper.get_ffi()
    sk = stepper.get_stepper_kinematics()

    def pos(coord):
        return ffi_lib.itersolve_calc_position_from_coord(sk, *coord)

    origin, ex, ey, ez = (pos(c) for c in UNIT_COORDS)
    row = [ex - origin, ey - origin, ez - origin]
    predicted = origin + sum(r * c for r, c in zip(row, LINEARITY_PROBE))
    actual = pos(LINEARITY_PROBE)
    if abs(predicted - actual) > 1e-9 * max(1.0, abs(actual)):
        raise UnsupportedRun(
            "stepper %s kinematics are not linear in xyz (predicted %r, got %r)"
            % (stepper.get_name(), predicted, actual)
        )
    return {"offset": origin, "row": row}


def _shaper_state(printer_obj):
    shaper_obj = printer_obj.lookup_object("input_shaper", None)
    if shaper_obj is None:
        return {}
    state = {}
    for shaper in shaper_obj.get_shapers():
        axis = shaper.get_axis()
        if isinstance(shaper, input_shaper.AxisInputShaper):
            state[axis] = {
                "kind": "pulses",
                "type": shaper.get_type(),
                "A": list(shaper.A),
                "T": list(shaper.T),
            }
        elif isinstance(shaper, input_shaper.AxisInputSmoother):
            state[axis] = {
                "kind": "smoother",
                "type": shaper.get_type(),
                "coeffs": list(shaper.coeffs),
                "smooth_time": shaper.smooth_time,
            }
        else:
            raise UnsupportedRun("unknown shaper class %r" % (type(shaper),))
    return state


HISTORY_BATCH = 4096
HISTORY_END_CLOCK = (1 << 63) - 1


class StepHistory:
    def __init__(self, steppers):
        self.steppers = steppers
        self.entries = {s.get_name(): [] for s in steppers}
        self.newest_first_clock = {s.get_name(): -1 for s in steppers}
        self.pulled_at_disconnect = False

    def pull(self, *_flush_times):
        for stepper in self.steppers:
            name = stepper.get_name()
            newest = self.newest_first_clock[name]
            batch = HISTORY_BATCH
            while True:
                data, count = stepper.dump_steps(
                    batch, max(newest, 0), HISTORY_END_CLOCK
                )
                if count < batch:
                    break
                batch *= 2
            fresh = [
                [data[i].first_clock, data[i].step_count, data[i].interval,
                 data[i].add]
                for i in range(count)
                if data[i].first_clock > newest
            ]  # fmt: skip
            fresh.reverse()
            self.entries[name].extend(fresh)
            if fresh:
                self.newest_first_clock[name] = fresh[-1][0]


class Capture:
    def __init__(self):
        self.toolhead = None
        self.printer = None
        self.timing = {"lookahead_s": 0.0, "junction_limiter_s": 0.0}
        self.moves = []
        self.linear_maps = {}
        self.steppers = None
        self.shapers = []
        self.settings = None
        self.undrained_serial = False
        self.history = None
        self.added = 0
        self.junctions = []
        self.flushes = []
        self.flush_work = []
        self.reactor_meter = None
        self.lazy_flush_at = frozenset(
            int(i)
            for i in os.environ.get("MOTION_EVAL_LAZY_FLUSH_AT", "").split(",")
            if i
        )

    def attach(self, toolhead_obj):
        self.toolhead = toolhead_obj
        self.printer = toolhead_obj.printer
        for stepper in toolhead_obj.get_kinematics().get_steppers():
            self.linear_maps[stepper.get_name()] = _stepper_linear_map(stepper)
        self.history = StepHistory(toolhead_obj.get_kinematics().get_steppers())
        self.printer.lookup_object("motion_queuing").register_flush_callback(
            self.history.pull
        )
        real_append = toolhead_obj.trapq_append
        real_set_position = toolhead_obj.set_position
        real_flush = self._timed(toolhead_obj.lookahead.flush, "lookahead_s")
        requested = collections.deque()

        def flush(lazy=False):
            queued = len(toolhead_obj.lookahead.queue)
            start = time.perf_counter()
            moves = real_flush(lazy=lazy)
            self.flush_work.append(
                [queued, len(moves), time.perf_counter() - start]
            )
            self.flushes.append([self.added, lazy, len(moves)])
            requested.extend(
                (math.sqrt(m.max_cruise_v2), m.motion_eval_max_accel)
                for m in moves
                if m.is_kinematic_move
            )
            return moves

        def trapq_append(tq, *args):
            if tq == toolhead_obj.trapq:
                if not self.moves:
                    self._snapshot_start()
                if not requested:
                    raise UnsupportedRun(
                        "toolhead trapq move that did not pass the lookahead"
                        " (drip/homing move)"
                    )
                move = dict(zip(TRAPQ_APPEND_FIELDS, args))
                move["max_cruise_v"], move["max_accel"] = requested.popleft()
                move["shaper_epoch"] = len(self.shapers) - 1
                self.moves.append(move)
            return real_append(tq, *args)

        def set_position(newpos, homing_axes=""):
            if self.moves:
                raise UnsupportedRun(
                    "toolhead.set_position after motion started (homing or"
                    " SET_KINEMATIC_POSITION inside the gcode) is not supported"
                )
            return real_set_position(newpos, homing_axes)

        toolhead_obj.trapq_append = trapq_append
        toolhead_obj.set_position = set_position
        toolhead_obj.lookahead.flush = flush
        real_add_move = self._timed(
            toolhead_obj.lookahead.add_move, "lookahead_s"
        )

        def add_move(move):
            move.motion_eval_max_accel = toolhead_obj.max_accel
            index = self.added
            self.added += 1
            want_flush = real_add_move(move)
            return want_flush or index in self.lazy_flush_at

        toolhead_obj.lookahead.add_move = add_move
        self.printer.register_event_handler("klippy:ready", self._on_ready)

    def _on_ready(self):
        self.printer.lookup_object("gcode").register_command(
            "MOTION_EVAL_EXIT", self._cmd_exit
        )
        corner_deviation = self.printer.lookup_object("corner_deviation", None)
        self.reactor_meter = ReactorMeter(
            self.printer.get_reactor(),
            lambda: corner_deviation is not None and corner_deviation.building,
        )
        self._note_shaper()
        shaper_obj = self.printer.lookup_object("input_shaper", None)
        if shaper_obj is not None:
            real_update = shaper_obj._update_input_shaping

            def update_input_shaping(*args, **kwargs):
                self.toolhead.flush_step_generation()
                try:
                    real_update(*args, **kwargs)
                finally:
                    self._note_shaper()

            shaper_obj._update_input_shaping = update_input_shaping
        limiter = getattr(self.toolhead, "junction_limiter", None)
        if limiter is None:
            return
        timed = self._timed(limiter, "junction_limiter_s")

        def logged(prev_move, move, max_start_v2):
            v2 = timed(prev_move, move, max_start_v2)
            self.junctions.append([self.added - 1, max_start_v2, v2])
            return v2

        self.toolhead.junction_limiter = logged

    def _cmd_exit(self, gcmd):
        self.toolhead.wait_moves()
        self.printer.request_exit("exit")

    def _note_shaper(self):
        """Start a new epoch when the shaper changed. Called after the
        toolhead flushed every move planned under the previous one."""
        state = _shaper_state(self.printer)
        if not self.shapers or self.shapers[-1] != state:
            self.shapers.append(state)

    def _timed(self, func, key):
        timing = self.timing
        clock = time.perf_counter

        def timed(*args, **kwargs):
            start = clock()
            try:
                return func(*args, **kwargs)
            finally:
                timing[key] += clock() - start

        return timed

    def _snapshot_start(self):
        th = self.toolhead
        self.settings = {
            "max_velocity": th.max_velocity,
            "max_accel": th.max_accel,
            "square_corner_velocity": th.square_corner_velocity,
            "minimum_cruise_ratio": th.min_cruise_ratio,
        }
        self.steppers = {}
        for stepper in th.get_kinematics().get_steppers():
            mcu = stepper.get_mcu()
            name = stepper.get_name()
            self.steppers[name] = {
                "oid": stepper.get_oid(),
                "mcu": mcu.get_name(),
                "clock_freq": mcu.get_constant_float("CLOCK_FREQ"),
                "step_dist": stepper.get_step_dist(),
                "invert_dir": bool(stepper.get_dir_inverted()[0]),
                "commanded_pos_at_mcu_zero": stepper.mcu_to_commanded_position(
                    0
                ),
                "linear_map": self.linear_maps[name],
            }

    def finish(self):
        if not self.moves:
            raise UnsupportedRun("gcode produced no toolhead motion")
        if self.undrained_serial:
            raise UnsupportedRun(
                "serial queue did not drain to the output file within %r s"
                % DRAIN_TIMEOUT_S
            )
        if not self.history.pulled_at_disconnect:
            raise UnsupportedRun("step history was not read at disconnect")
        for stepper in self.toolhead.get_kinematics().get_steppers():
            name = stepper.get_name()
            self.steppers[name]["final_mcu_position"] = (
                stepper.get_mcu_position()
            )
            self.steppers[name]["step_history"] = self.history.entries[name]
        limiter = self.printer.lookup_object("corner_deviation", None)
        return {
            "moves": self.moves,
            "steppers": self.steppers,
            "shapers": self.shapers,
            "junctions": self.junctions,
            "flushes": self.flushes,
            "flush_work": self.flush_work,
            "reactor": self.reactor_meter.summary(),
            "added_moves": self.added,
            "settings": self.settings,
            "timing": self.timing,
            "corner_deviation": None
            if limiter is None
            else limiter.get_status(None),
        }


REACTOR_TICK_S = 0.005
REACTOR_WORST_KEPT = 8


class ReactorMeter:
    """A reactor timer due every REACTOR_TICK_S, from klippy:ready on.
    cpu_s: the main-thread CPU spent between two of its runs, i.e. the
    longest stretch some callback held the reactor. late_s: how long after
    it was due a run started (wall clock; includes waiting for the CPU on a
    throttled host and for the GIL). worst: the REACTOR_WORST_KEPT largest
    of each as [seconds after ready, cpu_s, late_s]; worst_building: the
    largest late_s while building() (a corner table thread runs)."""

    def __init__(self, reactor, building):
        self.reactor = reactor
        self.building = building
        self.start = self.due = reactor.monotonic()
        self.last_cpu = time.thread_time()
        self.worst_cpu = []
        self.worst_late = []
        self.worst_building = []
        self.building_s = 0.0
        self.ticks = 0
        reactor.register_timer(self.tick, self.due)

    def tick(self, eventtime):
        now_cpu = time.thread_time()
        now = self.reactor.monotonic()
        sample = [now - self.start, now_cpu - self.last_cpu, now - self.due]
        kept = [(self.worst_cpu, 1), (self.worst_late, 2)]
        if self.building():
            kept.append((self.worst_building, 2))
            self.building_s = sample[0]
        for worst, index in kept:
            worst.append(sample)
            worst.sort(key=lambda s: -s[index])
            del worst[REACTOR_WORST_KEPT:]
        self.last_cpu = now_cpu
        self.ticks += 1
        self.due = eventtime + REACTOR_TICK_S
        return self.due

    def summary(self):
        return {
            "max_cpu_s": self.worst_cpu[0][1],
            "max_late_s": self.worst_late[0][2],
            "worst_cpu": self.worst_cpu,
            "worst_late": self.worst_late,
            "worst_building": self.worst_building,
            "building_until_s": self.building_s,
            "ticks": self.ticks,
        }


def _queued_bytes(reader):
    reader.ffi_lib.serialqueue_get_stats(
        reader.serialqueue, reader.stats_buf, len(reader.stats_buf)
    )
    stats = dict(
        field.split("=", 1)
        for field in reader.ffi_main.string(reader.stats_buf).decode().split()
    )
    return int(stats["ready_bytes"]) + int(stats["upcoming_bytes"])


def drain_before_disconnect(capture):
    real_disconnect = serialhdl.SerialReader.disconnect

    def disconnect(self):
        if self.serialqueue is not None:
            if capture.history is not None:
                capture.history.pull()
                capture.history.pulled_at_disconnect = True
            deadline = time.monotonic() + DRAIN_TIMEOUT_S
            while _queued_bytes(self):
                if time.monotonic() > deadline:
                    capture.undrained_serial = True
                    break
                time.sleep(DRAIN_POLL_S)
        real_disconnect(self)

    serialhdl.SerialReader.disconnect = disconnect


def main():
    capture_path = sys.argv[1]
    capture = Capture()
    real_init = toolhead.ToolHead.__init__

    def init(self, config):
        real_init(self, config)
        capture.attach(self)

    toolhead.ToolHead.__init__ = init
    drain_before_disconnect(capture)
    motion_queuing.MOVE_HISTORY_EXPIRE = math.inf
    sys.argv = ["klippy"] + sys.argv[2:]
    printer.main()
    with open(capture_path, "w") as f:
        json.dump(capture.finish(), f)


if __name__ == "__main__":
    main()
