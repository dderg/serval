"""Runs one scenario on a booted SimWorld and raises the first violated
invariant.

The print files toggle a mark pin at each rest probe (M400, SET_PIN,
dwell); a paced wait reads every motor's step counter the moment the pin
flips, and again while the dwell still holds, so the rest is checked on
printer time however fast the world runs. Nothing inside a paced wait
talks to klippy: it reads pins and step counters from the MCU shims."""

from __future__ import annotations

import bisect
import dataclasses
import pathlib
import threading
import time
from typing import Callable, Optional

from tools.sim.fuzzing import printer, program
from tools.sim.fuzzing.checks import (
    DURATION_TOLERANCE,
    ENDPOINT_DEATH_GRACE_S,
    ESTOP_DEADLINE_S,
    ESTOP_REASON,
    MOTION_STILL_S,
    PACER_BRAKE,
    Check,
    Violation,
    event_offsets,
    event_wall_ns,
    events_since,
    lattice_counts,
    same_position,
    written_panic,
)
from tools.sim.fuzzing.printer import (
    CANCEL_END_STATE,
    DONE_PIN_LINE,
    MARK_PIN_LINE,
    MESH_PROFILE,
    MeshKind,
    Motor,
)
from tools.sim.fuzzing.scenario import START_POSITION, Scenario
from tools.sim.fuzzing.timeline import CHANGES_STATE, ENDS_PRINT, Act, Action
from tools.sim.world import HarnessPacer, SimError

NS_PER_S = 1_000_000_000
PRINT_TIMEOUT_S = 600.0
COMMAND_TIMEOUT_S = 120.0
CHUNK_S = 2.0
POLL_S = 0.005
PIN_RESET_S = 5.0
PROBE_RECHECK_S = 0.3
PIN_EDGE_FRESH_NS = 50_000_000
BRAKE_SETTLE_S = 1.0
FAULT_WINDOW_S = 3.0
RESTART_DEADLINE_S = 5.0
READY_TIMEOUT_S = 120.0
IDLE_SLACK_S = 2.0
TRAVEL_FEED = 6000
POST_CHECK_XY = (120.0, 110.0)
WALL_SAMPLE_S = 0.002
PANIC_POLL_REAL_S = 0.2
EVENT_STAMP_NS = 1_000_000
RESTARTING = "Restarting printer"
UNKNOWN_COMMAND = 'Unknown command:"SET_INPUT_SHAPER"'
MARK_CHIP = 0


def _motor_position(motor: Motor, position) -> float:
    x, y, z, e = position
    return {"x": x, "y": y, "a": x + y, "b": x - y, "z": z, "e": e}[motor.axis]


def _xyz(position) -> str:
    return " ".join(f"{a}{v:.6f}" for a, v in zip("XYZE", position))


def probes_to_skip(unseen: int, paused: bool, edge_age_ns: int) -> int:
    """Probes to count unchecked: every one that flipped while paused,
    every one of several that flipped between two looks, and a single
    flip older than PIN_EDGE_FRESH_NS when seen (a blocking command kept
    the harness away; the toolhead may have left the rest since). A
    single fresh flip is checked."""
    if paused or unseen > 1 or edge_age_ns > PIN_EDGE_FRESH_NS:
        return unseen
    return 0


@dataclasses.dataclass
class _Recheck:
    at_ns: int
    steps: dict
    label: str
    mismatch: Optional[Violation]
    at_next_rest: bool


class _WallMap:
    """(wall, virtual) clock pairs, to place an event's wall-clock `_time`
    on the virtual timeline the stop windows are kept in."""

    def __init__(self, vclock):
        self._vclock = vclock
        self._walls: list[int] = []
        self._virtuals: list[int] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(WALL_SAMPLE_S):
            self._walls.append(time.time_ns())
            self._virtuals.append(self._vclock.now_ns())

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)

    def virtual_span(self, wall_ns: int) -> tuple[int, int]:
        low = max(bisect.bisect_right(self._walls, wall_ns) - 1, 0)
        high = min(
            bisect.bisect_left(self._walls, wall_ns + EVENT_STAMP_NS),
            len(self._walls) - 1,
        )
        return self._virtuals[low], self._virtuals[high]


class _PanicWatch:
    """Reads klippy.stdout on its own thread: a panicked engine thread
    freezes the virtual clock, and a paced wait would sit out the wedge
    timeout (VirtualDeadline.WEDGE_REAL_S, 60 s real) before anyone looked."""

    def __init__(self, stdout: pathlib.Path):
        self._stdout = stdout
        self._text = b""
        self.found: Optional[str] = None
        self.seen = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(PANIC_POLL_REAL_S):
            with self._stdout.open("rb") as stream:
                stream.seek(len(self._text))
                self._text += stream.read()
            self.found = written_panic(self._text.decode(errors="replace"))
            if self.found:
                self.seen.set()
                return

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)


class Session:
    def __init__(self, scenario: Scenario, world, config_text: str):
        self.scenario = scenario
        self.spec = scenario.printer
        self.world = world
        self.motors = printer.motors(self.spec, config_text)
        self.macros = printer.macros(self.spec)
        self.controls = {
            m.process: world.sim_control(m.process) for m in self.motors
        }
        self.marks = world.sim_control("h7")
        self.base: dict[str, tuple[int, float]] = {}
        self.commanded_shutdowns: list[str] = []
        self.windows: list[tuple[int, int]] = []
        self.notes: list[str] = []
        self.durations: dict[int, float] = {}
        self.wall = _WallMap(world.vclock)
        self.panics = _PanicWatch(world.log_dir / "klippy.stdout")
        self.offsets = event_offsets(world.log_dir)
        self.probes: list[program.Probe] = []
        self.next_probe = 0
        self.edge_base = 0
        self.paused = False
        self.recheck: Optional[_Recheck] = None
        self.watching = False
        self.print_index = 0

    # ------------------------------------------------------------ clock

    def _now_ns(self) -> int:
        return self.world.vclock.now_ns()

    def _paced(self, until: Callable[[], bool], what: str) -> None:
        deadline = self.world.vclock.deadline(PRINT_TIMEOUT_S)
        try:
            with HarnessPacer(self.world.vclock) as pacer:
                while True:
                    self._service_probes()
                    if until():
                        return
                    if deadline.expired():
                        raise SimError(f"timeout waiting for {what}")
                    if not pacer.sleep(POLL_S, stop=self.panics.seen):
                        raise Violation(Check.PANIC, self.panics.found)
        except SimError as error:
            check = Check.WEDGE if "wedged" in str(error) else Check.HARNESS
            raise Violation(check, str(error)) from error

    def stop_watchers(self) -> None:
        self.wall.stop()
        self.panics.stop()

    def _hold(self, seconds: float, what: str) -> None:
        end_ns = self._now_ns() + int(seconds * NS_PER_S)
        self._paced(lambda: self._now_ns() >= end_ns, what)

    # ------------------------------------------------------------ klippy

    def _gcode(
        self, script: str, check: Check, timeout: float = COMMAND_TIMEOUT_S
    ) -> dict:
        response = self.world.gcode(script, timeout=timeout)
        if not response:
            raise Violation(
                check, f"{script!r}: no response within {timeout} s"
            )
        if response.get("error"):
            raise Violation(check, f"{script!r}: {response['error']}")
        return response

    def _status(self, *objects: str) -> dict:
        return self.world.status({name: None for name in objects})

    def _toolhead(self) -> list[float]:
        return list(self._status("toolhead")["toolhead"]["position"])

    def _print_state(self) -> dict:
        return self._status("print_stats").get("print_stats", {})

    # ------------------------------------------------------------ steps

    def _checked(self) -> list[Motor]:
        """Motors whose rest position the gcode alone fixes. The extruder
        lays the shaped path's length, less than the commanded E where a
        blend cuts a corner (docs/rewrite/shaper.md, projected
        followers); a loaded mesh warps Z. Both still must hold still."""
        mesh = self.spec.mesh is not MeshKind.NONE
        return [
            m
            for m in self.motors
            if m.axis != "e" and not (mesh and m.axis == "z")
        ]

    def _steps(self) -> dict[str, int]:
        return {
            m.name: self.controls[m.process].step_position(m.step_line)["steps"]
            for m in self.motors
        }

    def _rebase(self, position) -> None:
        steps = self._still_steps("re-homing at the start position")
        for motor in self._checked():
            self.base[motor.name] = (
                steps[motor.name],
                _motor_position(motor, position),
            )

    def _allowed(self, motor: Motor, position) -> set[int]:
        base_steps, base_mm = self.base[motor.name]
        offset = _motor_position(motor, position) - base_mm
        return {
            base_steps + c for c in lattice_counts(offset, motor.steps_per_mm)
        }

    def _check_rest(self, steps: dict, position, label: str) -> None:
        mismatch = self._rest_mismatch(steps, position, label)
        if mismatch:
            raise mismatch

    def _rest_mismatch(
        self, steps: dict, position, label: str
    ) -> Optional[Violation]:
        for motor in self._checked():
            allowed = self._allowed(motor, position)
            if steps[motor.name] in allowed:
                continue
            nearest = min(allowed, key=lambda s: abs(s - steps[motor.name]))
            return Violation(
                Check.REST_STEPS,
                f"{label}: motor {motor.name} rests {steps[motor.name] - nearest:+d} "
                f"steps off the commanded position ({_xyz(position)}: "
                f"{steps[motor.name]} steps, allowed {sorted(allowed)})",
            )
        return None

    # ----------------------------------------------------------- probes

    def _service_probes(self) -> None:
        """Counts mark-pin edges while a print body runs; a cancel, a
        restart or an e-stop drops the pin to its shutdown value, no probe.
        A probe whose pin flipped while the toolhead was paused, or while
        a blocking command kept the harness from watching, is counted but
        not checked: its rest is not where the toolhead stood then."""
        if not self.watching:
            return
        if self.recheck and self._now_ns() >= self.recheck.at_ns:
            self._finish_recheck()
        unseen = self._mark_edges() - self.edge_base - self.next_probe
        if unseen <= 0:
            return
        edge_age_ns = self._now_ns() - self._mark_edge_ns()
        skipped = probes_to_skip(unseen, self.paused, edge_age_ns)
        if skipped:
            self._skip_probes(skipped, edge_age_ns)
            return
        self._take_probe()

    def _mark_edges(self) -> int:
        return self.marks.gpio_edges(MARK_CHIP, MARK_PIN_LINE)

    def _mark_edge_ns(self) -> int:
        return self.marks.gpio_last_edge_ns(MARK_CHIP, MARK_PIN_LINE)

    def _skip_probes(self, count: int, edge_age_ns: int) -> None:
        first = self.next_probe
        if first + count > len(self.probes):
            raise Violation(
                Check.HARNESS,
                f"mark pin flipped {first + count} times; the model has "
                f"{len(self.probes)} probes",
            )
        self.notes.append(
            f"print {self.print_index} probes {first}..{first + count - 1} "
            f"flipped unwatched (paused={self.paused}, last edge "
            f"{edge_age_ns / NS_PER_S:.3f} s before it was seen): not checked"
        )
        self.next_probe += count
        self.recheck = None

    def _take_probe(self) -> None:
        index = self.next_probe
        level = self._pin(MARK_PIN_LINE)
        if index >= len(self.probes) or self.probes[index].value != level:
            raise Violation(
                Check.HARNESS,
                f"mark pin went {level} at probe {index}; the model has "
                f"{[p.value for p in self.probes]}",
            )
        steps = self._steps()
        label = (
            f"print {self.print_index} rest probe {index} "
            f"({self._step_timing()})"
        )
        mismatch = self._rest_mismatch(
            steps, self.probes[index].position, label
        )
        self.recheck = _Recheck(
            self._now_ns() + int(PROBE_RECHECK_S * NS_PER_S),
            steps,
            label,
            mismatch,
            self._at_next_rest(steps, index),
        )
        self.next_probe += 1

    def _at_next_rest(self, steps: dict, index: int) -> bool:
        """Whether `steps` are where probe `index + 1` rests: the pin of
        probe `index` then flipped a whole move late (#893)."""
        following = index + 1
        if following >= len(self.probes):
            return False
        position = self.probes[following].position
        return self._rest_mismatch(steps, position, "") is None

    def _step_timing(self) -> str:
        """When the mark pin was seen against the last step any motor
        took: a probe seen after motion resumed has a last step at or
        just before the pin."""
        last_ns = max(
            self.controls[m.process].get_step_times(m.step_line)["last_ns"]
            for m in self.motors
        )
        return (
            f"pin seen {self._now_ns() / NS_PER_S:.3f} s, last step "
            f"{last_ns / NS_PER_S:.3f} s"
        )

    def _finish_recheck(self) -> None:
        """Motors that move within PROBE_RECHECK_S of the pin, inside the
        dwell after it, or that stand at the next probe's rest, mean the
        pin flipped after the next move began (#893): the pin, not the
        rest, is out of order. Motors still for that long anywhere else
        off the commanded position are a rest error."""
        recheck, self.recheck = self.recheck, None
        steps = self._steps()
        moved = {
            name: steps[name] - recheck.steps[name]
            for name in steps
            if steps[name] != recheck.steps[name]
        }
        if moved:
            raise Violation(
                Check.PIN_ORDER,
                f"{recheck.label}: motors moved within {PROBE_RECHECK_S} s "
                f"of the pin, inside the G4 P{program.PROBE_HOLD_MS} after "
                f"it\nsteps moved: {moved}; now {self._step_timing()}",
            )
        if recheck.mismatch and recheck.at_next_rest:
            raise Violation(
                Check.PIN_ORDER,
                f"{recheck.label}: motors stand at the next probe's rest: "
                f"the pin flipped after the move that follows its M400\n"
                f"{recheck.mismatch.evidence}",
            )
        if recheck.mismatch:
            raise recheck.mismatch

    def _settle_recheck(self) -> None:
        if self.recheck is not None:
            at_ns = self.recheck.at_ns
            self._paced(
                lambda: self._now_ns() >= at_ns, "the pending rest recheck"
            )

    def _pin(self, line: int) -> int:
        return self.marks.gpio_output(MARK_CHIP, line)

    # ------------------------------------------------------------ setup

    def _start_state(self) -> None:
        for command in printer.heater_targets(self.spec):
            self._gcode(command, Check.RECOVERY)
        if self.spec.mesh is MeshKind.NONE:
            return
        mesh = (
            f"BED_MESH_PROFILE LOAD={MESH_PROFILE}"
            if self.spec.mesh_at_start
            else "BED_MESH_CLEAR"
        )
        self._gcode(mesh, Check.RECOVERY)

    def _home_here(self) -> None:
        x, y, z = START_POSITION
        self._gcode(f"SET_KINEMATIC_POSITION X={x} Y={y} Z={z}", Check.RECOVERY)

    def setup(self) -> None:
        """After boot or a restart: unhomed, wherever the motors stand."""
        self._start_state()
        self._home_here()
        self._rebase(self._toolhead())

    def _rehome(self) -> None:
        x, y, z, _ = self._toolhead()
        self._gcode(f"SET_KINEMATIC_POSITION X={x} Y={y} Z={z}", Check.RECOVERY)
        sx, sy, sz = START_POSITION
        self._gcode(
            f"G90\nG1 X{sx} Y{sy} Z{sz} F{TRAVEL_FEED}\nM400", Check.RECOVERY
        )
        self._home_here()
        self._rebase(self._toolhead())

    def _post_check(self, after: str) -> None:
        state = self.world._printer_state()
        if state != "ready":
            raise Violation(
                Check.RECOVERY, f"after {after} klippy is {state!r}"
            )
        self._rehome()
        x, y = POST_CHECK_XY
        self._gcode(f"G1 X{x} Y{y} F{TRAVEL_FEED}\nM400", Check.RECOVERY)
        position = self._toolhead()
        if not all(
            same_position(p, e) for p, e in zip(position[:2], POST_CHECK_XY)
        ):
            raise Violation(
                Check.RECOVERY,
                f"after {after} the post-check move ended at {position}",
            )
        label = f"post-check move after {after}"
        self._check_rest(self._still_steps(label), position, label)

    # ------------------------------------------------------------ prints

    def run(self) -> None:
        self.setup()
        for index, name in enumerate(self.scenario.prints):
            if index:
                self._start_state()
                self._rehome()
            self._print(index, name)
        self._compare_durations()
        self._check_idle_timeout()
        self._check_brakes()

    def _arm(self, lines: list[str]) -> None:
        self._gcode(
            f"SET_PIN PIN={printer.MARK_PIN} VALUE=0\n"
            f"SET_PIN PIN={printer.DONE_PIN} VALUE=0",
            Check.HARNESS,
        )
        try:
            self.world.wait_until(
                lambda: (
                    not (self._pin(MARK_PIN_LINE) or self._pin(DONE_PIN_LINE))
                ),
                PIN_RESET_S,
                "the mark pins resetting before the print",
                poll=POLL_S,
            )
        except SimError as error:
            raise Violation(Check.HARNESS, str(error)) from error
        start = tuple(START_POSITION) + (self._toolhead()[3],)
        model = program.Model(start, self.macros)
        model.run(lines)
        self.probes = model.probes
        self.final_position = tuple(model.position)
        self.next_probe = 0
        self.edge_base = self._mark_edges()
        self.paused = False
        self.recheck = None
        self.watching = True

    def _print(self, index: int, name: str) -> None:
        self.print_index = index
        lines = program.render(self.scenario.program(name))
        path = self.world.gcode_dir / f"fuzz_{name}.gcode"
        path.write_text("\n".join(lines) + "\n")
        self._arm(lines)
        start_ns = self._now_ns()
        self._gcode(
            f"SDCARD_PRINT_FILE FILENAME={path.name}", Check.PRINT_OUTCOME
        )
        for action in self.scenario.actions_for(index):
            if not self._advance(
                start_ns, start_ns + int(action.at_s * NS_PER_S)
            ):
                self.notes.append(
                    f"skipped (print ended first): {action.to_line()}"
                )
                continue
            self._settle_recheck()
            ends_print = action.act in ENDS_PRINT
            if ends_print:
                self.watching = False
            self._execute(action)
            if ends_print:
                return
        self._advance(start_ns, None)
        self._finish_print(index, start_ns)

    def _advance(self, start_ns: int, until_ns: Optional[int]) -> bool:
        """Run the print to `until_ns` (False if it finished first) or, with
        None, to its done pin."""
        deadline_ns = start_ns + int(PRINT_TIMEOUT_S * NS_PER_S)
        while True:
            chunk_ns = self._now_ns() + int(CHUNK_S * NS_PER_S)
            stop_ns = chunk_ns if until_ns is None else min(chunk_ns, until_ns)
            self._paced(
                lambda: (
                    self._pin(DONE_PIN_LINE) == 1 or self._now_ns() >= stop_ns
                ),
                "the print reaching its next mark",
            )
            if self._pin(DONE_PIN_LINE) == 1:
                return until_ns is None
            if until_ns is not None and self._now_ns() >= until_ns:
                return True
            self._check_printing(deadline_ns)

    def _check_printing(self, deadline_ns: int) -> None:
        if self.world.klippy_proc.poll() is not None:
            raise Violation(Check.KLIPPY_EXIT, "klippy exited during the print")
        self._check_no_panic()
        stats = self._print_state()
        if stats.get("state") in ("error", "cancelled"):
            raise Violation(
                Check.PRINT_OUTCOME,
                f"print ended {stats.get('state')!r} before its last line: "
                f"{stats.get('message', '')}",
            )
        if self._now_ns() > deadline_ns:
            raise Violation(
                Check.PRINT_OUTCOME,
                f"print still running after {PRINT_TIMEOUT_S} s",
            )

    def _check_no_panic(self) -> None:
        """A panicked engine thread can leave klippy up and the print
        never ending: stop at the panic instead of the print timeout."""
        if self.panics.found:
            raise Violation(Check.PANIC, self.panics.found)

    def _finish_print(self, index: int, start_ns: int) -> None:
        """The body ended (done pin): END_PRINT, if any, runs to the end of
        the file with no M400 after it, so the stream ends in the pacer's
        closing brake, as test_print_stats_completes_with_the_motion
        requires of a print without a trailing M400."""
        body_end_ns = self._now_ns()
        self._settle_recheck()
        self.watching = False
        if self.next_probe != len(self.probes):
            raise Violation(
                Check.HARNESS,
                f"saw {self.next_probe} of {len(self.probes)} rest probes",
            )
        try:
            self.world.wait_print_done(timeout=PRINT_TIMEOUT_S)
        except SimError as error:
            raise Violation(Check.PRINT_OUTCOME, str(error)) from error
        self._gcode("M400", Check.PRINT_OUTCOME)
        self._allow_brakes(body_end_ns)
        self.notes.append(
            f"print {index}: start {start_ns / NS_PER_S:.3f} s, body end "
            f"{body_end_ns / NS_PER_S:.3f} s, complete {self._now_ns() / NS_PER_S:.3f} s"
        )
        self._check_final_position()
        self._check_rest(
            self._still_steps("print end"), self.final_position, "print end"
        )
        if index in self._compared_prints():
            self.durations[index] = self._motion_end_s() - start_ns / NS_PER_S

    def _check_final_position(self) -> None:
        position = self._toolhead()
        axes = "XYE" if self.spec.mesh is not MeshKind.NONE else "XYZE"
        for axis, measured, expected in zip(
            "XYZE", position, self.final_position
        ):
            if axis in axes and not same_position(measured, expected):
                raise Violation(
                    Check.PRINT_OUTCOME,
                    f"toolhead {axis} at {measured:.6f}, last commanded {expected:.6f}",
                )

    def _still_steps(self, label: str) -> dict:
        """The step counters after M400, once they held for the recheck
        interval: a motor still moving then was not at rest."""
        first = self._steps()
        self._hold(PROBE_RECHECK_S, f"{label}: the rest recheck")
        second = self._steps()
        moved = {
            n: second[n] - first[n] for n in second if second[n] != first[n]
        }
        if moved:
            raise Violation(
                Check.REST_STEPS,
                f"{label}: motors moved {PROBE_RECHECK_S} s after M400: {moved}",
            )
        return second

    # ------------------------------------------------------- comparisons

    def _quiet(self, index: int) -> bool:
        return not self.scenario.actions_for(index)

    def _comparable_pairs(self) -> list[tuple[int, int]]:
        prints = self.scenario.prints
        pairs = []
        for first in range(len(prints)):
            for second in range(first + 1, len(prints)):
                if self._comparable(first, second):
                    pairs.append((first, second))
        return pairs

    def _comparable(self, first: int, second: int) -> bool:
        prints = self.scenario.prints
        if prints[first] != prints[second]:
            return False
        if program.Kind.MESH in self.scenario.program(prints[first]).kinds():
            return False
        if not (self._quiet(first) and self._quiet(second)):
            return False
        between = (
            action
            for index in range(first, second + 1)
            for action in self.scenario.actions_for(index)
        )
        return not any(a.act in CHANGES_STATE for a in between)

    def _compared_prints(self) -> set[int]:
        return {index for pair in self._comparable_pairs() for index in pair}

    def _xy_motors(self) -> list[Motor]:
        return [m for m in self.motors if m.axis in ("x", "y", "a", "b")]

    def _last_xy_step_s(self) -> float:
        return (
            max(
                self.controls[m.process].get_step_times(m.step_line)["last_ns"]
                for m in self._xy_motors()
            )
            / NS_PER_S
        )

    def _motion_end_s(self) -> float:
        still_ns = int(MOTION_STILL_S * NS_PER_S)
        self._paced(
            lambda: (
                self._now_ns() - self._last_xy_step_s() * NS_PER_S >= still_ns
            ),
            f"X/Y still for {MOTION_STILL_S} s",
        )
        return self._last_xy_step_s()

    def _compare_durations(self) -> None:
        for first, second in self._comparable_pairs():
            if first not in self.durations or second not in self.durations:
                continue
            a, b = self.durations[first], self.durations[second]
            if abs(b - a) > DURATION_TOLERANCE * a:
                raise Violation(
                    Check.SESSION_REPEAT,
                    f"print {first} moved for {a:.2f} s, print {second} of the same "
                    f"file for {b:.2f} s (tolerance {DURATION_TOLERANCE:.0%})",
                )

    # ------------------------------------------------------------ idle

    def _check_idle_timeout(self) -> None:
        if self.spec.idle_timeout_s is None:
            return
        self._hold(self.spec.idle_timeout_s + IDLE_SLACK_S, "the idle timeout")
        status = self._status("idle_timeout", "extruder")
        state = status["idle_timeout"]["state"]
        target = status["extruder"]["target"]
        if state != "Idle" or target != 0.0:
            raise Violation(
                Check.IDLE_TIMEOUT,
                f"{self.spec.idle_timeout_s + IDLE_SLACK_S} s idle: state {state!r}, "
                f"extruder target {target}",
            )

    # ----------------------------------------------------------- brakes

    def _allow_brakes(
        self, start_ns: int, end_ns: Optional[int] = None
    ) -> None:
        end_ns = self._now_ns() if end_ns is None else end_ns
        self.windows.append((start_ns, end_ns + int(BRAKE_SETTLE_S * NS_PER_S)))

    def _allowed_brake(self, low_ns: int, high_ns: int) -> bool:
        return any(
            start <= high_ns and low_ns <= end for start, end in self.windows
        )

    def _check_brakes(self) -> None:
        self.wall.stop()
        for event in events_since(self.world.log_dir, self.offsets):
            if event.get("event") != PACER_BRAKE:
                continue
            low, high = self.wall.virtual_span(event_wall_ns(event))
            if self._allowed_brake(low, high):
                continue
            fields = {
                k: event.get(k)
                for k in ("cause", "runway_s", "reserve_s", "cap_s")
            }
            raise Violation(
                Check.PACER_BRAKE,
                f"pacer brake at {low / NS_PER_S:.3f} s outside any commanded stop: {fields}",
            )

    # ----------------------------------------------------------- actions

    def _execute(self, action: Action) -> None:
        handler = {
            Act.PAUSE_RESUME: self._pause_resume,
            Act.CANCEL: self._cancel,
            Act.FIRMWARE_RESTART: self._firmware_restart,
            Act.HOST_RESTART: self._host_restart,
            Act.ESTOP_RESTART: self._estop_restart,
            Act.SET_PA: self._set_pa,
            Act.SET_POST_PROCESSOR: self._set_post_processor,
            Act.SET_INPUT_SHAPER: self._set_input_shaper,
            Act.MESH_LOAD: lambda a: self._gcode(
                f"BED_MESH_PROFILE LOAD={MESH_PROFILE}", Check.COMMAND_RESULT
            ),
            Act.MESH_CLEAR: lambda a: self._gcode(
                "BED_MESH_CLEAR", Check.COMMAND_RESULT
            ),
            Act.HEATER: self._heater,
            Act.LINK_FAULT: self._link_fault,
        }[action.act]
        handler(action)

    def _expect_state(self, wanted: str, after: str) -> None:
        state = self._print_state().get("state")
        if state != wanted:
            raise Violation(
                Check.STOP_STATE,
                f"after {after} print_stats is {state!r}, not {wanted!r}",
            )

    def _pause_resume(self, action: Action) -> None:
        start_ns = self._now_ns()
        self.paused = True
        self._gcode("PAUSE", Check.COMMAND_RESULT)
        self._expect_state("paused", "PAUSE")
        self._hold(float(action.param("hold_s")), "the pause hold")
        jog = action.param("jog")
        if jog:
            dx, dy, dz = jog.split(",")
            self._gcode(
                f"G91\nG1 X{dx} Y{dy} Z{dz} F{TRAVEL_FEED}\nG90\nM400",
                Check.COMMAND_RESULT,
            )
        self._gcode("RESUME", Check.COMMAND_RESULT)
        self._service_probes()
        self.paused = False
        state = self._print_state().get("state")
        if state not in ("printing", "complete"):
            raise Violation(
                Check.STOP_STATE, f"after RESUME print_stats is {state!r}"
            )
        self._allow_brakes(start_ns)

    def _cancel(self, action: Action) -> None:
        start_ns = self._now_ns()
        self._gcode("CANCEL_PRINT", Check.COMMAND_RESULT)
        self._gcode("M400", Check.COMMAND_RESULT)
        self._allow_brakes(start_ns)
        self._expect_state(
            CANCEL_END_STATE[self.spec.cancel_macro], "CANCEL_PRINT"
        )
        sd_card = self._status("virtual_sdcard")["virtual_sdcard"]
        if sd_card.get("is_active"):
            raise Violation(
                Check.STOP_STATE,
                "virtual_sdcard still active after CANCEL_PRINT",
            )
        self._post_check("CANCEL_PRINT")

    def _await_restart(self, what: str, deadline_s: float) -> None:
        try:
            self.world.expect_log(RESTARTING, timeout=deadline_s)
        except SimError as error:
            raise Violation(Check.RECOVERY, f"{what}: {error}") from error
        if not self.world._wait_ready(READY_TIMEOUT_S):
            raise Violation(
                Check.RECOVERY,
                f"klippy not ready {READY_TIMEOUT_S} s after {what}",
            )
        try:
            self.world._wait_clock_sync(READY_TIMEOUT_S)
        except SimError as error:
            raise Violation(Check.RECOVERY, f"{what}: {error}") from error

    def _after_restart(self, start_ns: int, what: str) -> None:
        self._allow_brakes(start_ns)
        self.setup()
        self._post_check(what)

    def _firmware_restart(self, action: Action) -> None:
        start_ns = self._now_ns()
        self.world.mark_log()
        self._restart_firmware("FIRMWARE_RESTART")
        self._after_restart(start_ns, "FIRMWARE_RESTART")

    def _restart_firmware(self, what: str) -> None:
        try:
            self.world.firmware_restart(timeout=READY_TIMEOUT_S)
        except SimError as error:
            raise Violation(Check.RECOVERY, f"{what}: {error}") from error
        self._await_restart(what, RESTART_DEADLINE_S)

    def _host_restart(self, action: Action) -> None:
        """RESTART completes queued moves first (docs/G-Codes.md), so it
        restarts once the buffered runway has played out, not within the
        FIRMWARE_RESTART deadline."""
        start_ns = self._now_ns()
        self.world.mark_log()
        self.world.gcode("RESTART", timeout=RESTART_DEADLINE_S)
        self._await_restart("RESTART", COMMAND_TIMEOUT_S)
        self._after_restart(start_ns, "RESTART")

    def _estop_restart(self, action: Action) -> None:
        start_ns = self._now_ns()
        self.world.emergency_stop()
        self.commanded_shutdowns.append(ESTOP_REASON)
        state = self.world._printer_state()
        latency_s = (self._now_ns() - start_ns) / NS_PER_S
        if state != "shutdown":
            raise Violation(
                Check.STOP_STATE, f"after emergency_stop klippy is {state!r}"
            )
        if latency_s > ESTOP_DEADLINE_S:
            raise Violation(
                Check.ESTOP_LATENCY,
                f"shutdown {latency_s:.3f} s after emergency_stop",
            )
        self._hold(float(action.param("delay_s")), "the e-stop delay")
        restart_ns = self._now_ns()
        self.world.mark_log()
        self._restart_firmware("FIRMWARE_RESTART after an e-stop")
        grace_ns = restart_ns + int(ENDPOINT_DEATH_GRACE_S * NS_PER_S)
        self._paced(
            lambda: self._now_ns() >= grace_ns, "the endpoint-death grace"
        )
        if self.world.klippy_proc.poll() is not None:
            raise Violation(
                Check.KLIPPY_EXIT,
                "klippy exited after FIRMWARE_RESTART following an e-stop",
            )
        self._after_restart(start_ns, "FIRMWARE_RESTART after an e-stop")

    def _set_pa(self, action: Action) -> None:
        script = f"SET_PRESSURE_ADVANCE ADVANCE={action.param('advance')}"
        if action.param("smooth_time"):
            script += f" SMOOTH_TIME={action.param('smooth_time')}"
        self._gcode(script, Check.COMMAND_RESULT)

    def _set_post_processor(self, action: Action) -> None:
        self._gcode(
            f"SET_POST_PROCESSOR NAME={action.param('name')} "
            f"{action.param('param').upper()}={action.param('value')}",
            Check.COMMAND_RESULT,
        )

    def _set_input_shaper(self, action: Action) -> None:
        self.world.mark_log()
        self._gcode(
            f"SET_INPUT_SHAPER SHAPER_FREQ_X={action.param('freq')}",
            Check.COMMAND_RESULT,
        )
        try:
            self.world.expect_log(UNKNOWN_COMMAND, timeout=RESTART_DEADLINE_S)
        except SimError as error:
            raise Violation(
                Check.COMMAND_RESULT, f"SET_INPUT_SHAPER: {error}"
            ) from error

    def _heater(self, action: Action) -> None:
        target = float(action.param("target"))
        if action.param("style") == "m104":
            script = f"M104 S{target:g}"
        else:
            script = f"SET_HEATER_TEMPERATURE HEATER=extruder TARGET={target:g}"
        self._gcode(script, Check.COMMAND_RESULT)
        measured = self._status("extruder")["extruder"]["target"]
        if measured != target:
            raise Violation(
                Check.COMMAND_RESULT, f"{script}: extruder target is {measured}"
            )

    def _link_fault(self, action: Action) -> None:
        start_ns = self._now_ns()
        mcu = action.param("mcu")
        kind = action.param("kind")
        if kind == "drop":
            self.world.inject(
                mcu, drop_command="get_clock", count=int(action.param("count"))
            )
            by_s = 0.0
        elif kind == "delay":
            by_s = float(action.param("by_s"))
            self.world.inject(mcu, delay_command="get_clock", by_s=by_s)
        else:
            by_s = float(action.param("by_s"))
            self.world.inject(mcu, stall_s=by_s)
        self._allow_brakes(
            start_ns, self._now_ns() + int((by_s + FAULT_WINDOW_S) * NS_PER_S)
        )
