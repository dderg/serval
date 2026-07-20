from __future__ import annotations

import json
import logging
import os
import subprocess
import time
from typing import Any, overload

from ... import structured_log
from .. import servo_param, servo_strokes
from .common import (
    DEFAULT_CAPTURES_ROOT,
    DEFAULT_DYNAMICS_DIR,
    REPO_ROOT,
    VERDICT_ABORT_FLAGS,
    _git_rev,
    _utc_now,
)
from .dynamics import parse_dynamics_profile
from .params import NOTCH_MODE_ADDR, NOTCH_READBACK, SYNC_LOSS_COUNT_ADDR
from .sweep import ExperimentRun, SweepEngine, SweepStep, _OverrideGcmd


class CalibrationHost:
    def __init__(self, config: Any):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.servos = config.getlist("servos", ["stepper_x", "stepper_y"])
        self.rated_torque_nm = config.getfloat(
            "rated_torque_nm", None, above=0.0
        )
        self.rotor_inertia_kgm2 = config.getfloat(
            "rotor_inertia_kgm2", None, above=0.0
        )
        self.bounds: servo_strokes.Bounds = {
            "X": (
                config.getfloat("x_start", 20.0),
                config.getfloat("x_end", 200.0),
            ),
            "Y": (
                config.getfloat("y_start", 20.0),
                config.getfloat("y_end", 200.0),
            ),
        }
        self.accels = config.getfloatlist("accels", [5000.0, 10000.0, 20000.0])
        self.speeds = config.getfloatlist("speeds", [100.0, 400.0])
        self.iterations = config.getint("iterations", 3, minval=1)
        self.accel_chip_name = config.get("accel_chip", None)
        self.dwell_ms = config.getint("dwell_ms", 700, minval=0)
        self.travel_speed = config.getfloat("travel_speed", 100.0, above=0.0)
        self.captures_root = config.get("captures_root", DEFAULT_CAPTURES_ROOT)
        self.dynamics_dir = os.path.expanduser(DEFAULT_DYNAMICS_DIR)
        self.servo_cal_binary = config.get(
            "servo_cal_binary",
            os.path.join(REPO_ROOT, "rust", "target", "snapshot", "servo-cal"),
        )
        self.journal_params = self._parse_journal_params(config)
        self._active_run: ExperimentRun | None = None
        self._capture_sync_loss: (
            tuple[str, list[str], dict[str, int]] | None
        ) = None
        self._last_sweep_run: ExperimentRun | None = None
        self._last_sweep_results: dict[str, Any] | None = None
        self._engine = SweepEngine(self)
        for name in (
            "SERVO_MEASURE_TRACKING",
            "SERVO_MEASURE_DIFFERENTIAL",
            "SERVO_MEASURE_RINGDOWN",
            "SERVO_DIFF_DAMPER",
            "SERVO_MEASURE_STRAIN_MAP",
            "SERVO_MEASURE_STRAIN_RESPONSE",
            "SERVO_STRAIN_COMP_TUNE",
            "SERVO_MEASURE_INERTIA",
            "SERVO_FIT_DYNAMICS",
            "SERVO_CALIBRATE_INERTIA_RATIO",
            "SERVO_SHOW_TUNING",
            "SERVO_SET_INERTIA_RATIO",
            "SERVO_APPLY_GAINS",
            "SERVO_CALIBRATE_GAINS",
            "SERVO_REFINE_DYNAMICS",
            "SERVO_TUNE_DYNAMICS",
            "SERVO_SWEEP_INERTIA",
            "SERVO_SWEEP_ACCEL",
            "SERVO_AUTOTUNE",
        ):
            self.gcode.register_command(
                name,
                getattr(self, "cmd_" + name),
                desc=getattr(self, "cmd_" + name + "_help"),
            )

    def _kin(self) -> Any:
        return self.printer.lookup_object("toolhead").get_kinematics()

    def _parse_journal_params(
        self, config: Any
    ) -> list[tuple[str, str | None]]:
        entries: list[tuple[str, str | None]] = []
        for raw in config.getlist("journal_params", []):
            addr, _sep, type_token = raw.partition(":")
            addr = addr.strip()
            type_token = type_token.strip() or None
            if (
                type_token is not None
                and type_token not in servo_param.TYPE_TOKENS
            ):
                raise config.error(
                    "[servo_calibration] journal_params: unknown type %r "
                    "(use u8/u16/u32/i8/i16/i32)" % (type_token,)
                )
            entries.append((addr, type_token))
        return entries

    def _servo_capture(self) -> Any:
        return self.printer.lookup_object("servo_capture")

    def _run_dir(self, tag: str) -> tuple[str, str]:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        root = os.path.expanduser(self.captures_root)
        run_dir = os.path.join(root, "%s_%s" % (tag, stamp))
        os.makedirs(run_dir, exist_ok=True)
        return run_dir, stamp

    def _resolve_motor(self, servo: str) -> Any:
        from .. import servo_axis

        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo, "SERVO_CALIBRATION"
        )
        return motor

    def _motor_manifest(self, motor: Any) -> dict[str, Any]:
        return {
            "name": motor.get_motor_name(),
            "invert": motor.get_invert_direction(),
            "rotation_distance": motor.get_rotation_distance(),
            "counts_per_mm": motor.get_counts_per_mm(),
        }

    def _ff_lead_us(self, gcmd: Any, motors: list[Any]) -> float:
        leads = set()
        for motor in motors:
            get_node_name = getattr(motor, "get_node_name", None)
            if get_node_name is None:
                leads.add(0.0)
                continue
            node = self.printer.lookup_object(
                "ethercat_node " + get_node_name()
            )
            profile_path = node.get_dynamics_profile()
            if profile_path is None:
                leads.add(0.0)
                continue
            profile_path = os.path.expanduser(profile_path)
            try:
                with open(profile_path) as f:
                    profile = parse_dynamics_profile(f.read())
            except (OSError, ValueError) as e:
                raise gcmd.error(
                    "failed to load dynamics profile %s: %s" % (profile_path, e)
                )
            leads.add(profile.get("ff_lead_us", 0.0))
        if len(leads) > 1:
            raise gcmd.error(
                "motors span dynamics profiles that disagree on "
                "ff_lead_us (%s); the analyzer needs a single "
                "per-run value" % (sorted(leads),)
            )
        return leads.pop() if leads else 0.0

    def _belts(self, rails: list[Any] | None) -> str | None:
        if not rails:
            return None
        return ",".join(
            "+".join(
                "%s:%d"
                % (
                    m.get_motor_name(),
                    -1 if m.get_invert_direction() else 1,
                )
                for m in servo_strokes.rail_motors_in_slot_order(r)
            )
            for r in rails
        )

    def _read_journal(
        self, servo: str, addr: str, type_token: str | None
    ) -> int:
        node, slot = self._resolve_node_slot(servo)
        index, subindex = servo_param.parse_address(addr)
        size, raw = servo_param.read_param(
            self.printer, node, slot, index, subindex
        )
        if type_token is not None:
            return servo_param.decode_typed(raw, size, type_token)
        return raw

    def _ambient(self, gcmd: Any, servos: list[str]) -> dict[str, Any]:
        journal: dict[str, dict[str, int]] = {}
        for servo in servos:
            readings: dict[str, int] = {}
            for addr, type_token in self.journal_params:
                try:
                    readings[addr] = self._read_journal(servo, addr, type_token)
                except (RuntimeError, ValueError) as e:
                    raise gcmd.error(
                        "journal_params readback failed for %s %s: %s"
                        % (servo, addr, e)
                    )
            journal[servo] = readings
        return {
            "journal_params": journal,
            "notches": {
                servo: self._notch_state(gcmd, servo) for servo in servos
            },
            "param_writes_since_last_run": servo_param.drain_param_writes(),
        }

    def _begin_run(
        self,
        gcmd: Any,
        experiment: str,
        tag: str,
        axis: str,
        servos: list[str],
        stroke_plan: dict[str, Any],
        belts_rails: list[Any] | None = None,
    ) -> ExperimentRun:
        run_dir, stamp = self._run_dir(tag)
        kin = self._kin()
        motors = [self._resolve_motor(s) for s in servos]
        manifest = {
            "version": 1,
            "experiment": experiment,
            "command": gcmd.get_commandline(),
            "tag": tag,
            "created_utc": _utc_now(),
            "axis": axis,
            "kinematics": getattr(kin, "kind", None),
            "git_rev": _git_rev(),
            "session_id": structured_log.get_session(),
            "stroke_plan": stroke_plan,
            "motors": [self._motor_manifest(m) for m in motors],
            "ff_lead_us": self._ff_lead_us(gcmd, motors),
            "belts": self._belts(belts_rails),
            "spatial": servo_strokes.spatial_frame(kin),
            "steps": [],
            "ambient": self._ambient(gcmd, servos),
        }
        run = ExperimentRun(run_dir, stamp, manifest)
        run.write()
        structured_log.event(
            "calibration",
            "run_start",
            run_dir=run_dir,
            experiment=experiment,
            tag=tag,
            axis=axis,
        )
        self._active_run = run
        return run

    def _on_step_complete(self, step: SweepStep) -> None:
        if self._active_run is not None:
            self._active_run.record_step(step)

    def _servo_cal(self, gcmd: Any) -> str:
        if not os.path.exists(self.servo_cal_binary):
            raise gcmd.error(
                "servo-cal binary not found at %s - build it with: "
                "cargo build --profile snapshot -p servo-ident"
                % (self.servo_cal_binary,)
            )
        return self.servo_cal_binary

    def _read_results(self, gcmd: Any, run_dir: str) -> dict[str, Any]:
        path = os.path.join(run_dir, "results.json")
        try:
            with open(path) as f:
                return json.load(f)
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "failed to read results.json from %s: %s" % (run_dir, e)
            )

    def _run_analyze(
        self, gcmd: Any, run: ExperimentRun, incremental: bool = False
    ) -> dict[str, Any]:
        binary = self._servo_cal(gcmd)
        argv = [binary, "analyze", run.run_dir]
        if incremental:
            argv.append("--incremental")
        self._run(gcmd, argv, 120.0)
        return self._read_results(gcmd, run.run_dir)

    def _analyze_and_report(
        self, gcmd: Any, run: ExperimentRun
    ) -> dict[str, Any]:
        results = self._run_analyze(gcmd, run)
        verdict = results.get("verdict") or {}
        step = verdict.get("recommended_step")
        reason = verdict.get("reason") or "no reason given"
        flags = verdict.get("flags") or []
        duration_s = round(time.time() - run.started_s, 3)
        gcmd.respond_info(
            "verdict: %s (%s) | run %s"
            % (step if step else "no step", reason, run.run_dir)
        )
        structured_log.event(
            "calibration",
            "run_done",
            run_dir=run.run_dir,
            recommended_step=step,
            flags=flags,
            duration_s=duration_s,
        )
        return results

    def _step_headline(
        self, results: dict[str, Any], step_name: str
    ) -> tuple[float, float]:
        """(ferr_peak, overshoot) in encoder counts, maxed over every drive
        and move of the named step - the before/after APPLY verification
        reads off this, not the mm-scaled `combined` block, so it works
        identically on a single-drive step and a CoreXY one."""
        for step in results.get("steps") or []:
            if step.get("name") != step_name:
                continue
            ferr_peak = 0.0
            overshoot = 0.0
            for drive in (step.get("drives") or {}).values():
                for move in (drive.get("metrics") or {}).get("moves") or []:
                    ferr_peak = max(ferr_peak, move.get("ferr_peak", 0.0))
                    overshoot = max(overshoot, move.get("overshoot", 0.0))
            return ferr_peak, overshoot
        raise self.printer.command_error(
            "step %r missing from results.json" % (step_name,)
        )

    def _step_metric_mean(
        self,
        gcmd: Any,
        results: dict[str, Any],
        step_name: str,
        metric: str,
    ) -> float:
        """Mean of one per-move metric over the named step's drives - the
        refinement objective, so mean (not max): lower variance under stroke
        noise, and constant per-drive offsets do not move the argmin."""
        for step in results.get("steps") or []:
            if step.get("name") != step_name:
                continue
            step_drives = step.get("drives") or {}
            values = [
                move[metric]
                for drive in step_drives.values()
                for move in (drive.get("metrics") or {}).get("moves") or []
                if metric in move
            ]
            if not values:
                raise gcmd.error(
                    "step %s carries no %r move metrics in results.json"
                    % (step_name, metric)
                )
            return sum(values) / len(values)
        raise gcmd.error("step %r missing from results.json" % (step_name,))

    def _step_flags(self, results: dict[str, Any], step_name: str) -> list[str]:
        for step in results.get("steps") or []:
            if step.get("name") == step_name:
                return list(step.get("flags") or [])
        return []

    def _check_clean_verdict(
        self,
        gcmd: Any,
        stage: str,
        run: ExperimentRun,
        results: dict[str, Any],
        require_step: bool,
    ) -> dict[str, Any]:
        """SERVO_AUTOTUNE's shared abort gate: a null recommendation is only
        fatal when this stage's job is to promote one (require_step); a
        torque/resonance flag on the chosen step is always fatal, dry run
        or not - continuing past a flagged step is unsafe regardless of
        whether anything gets written."""
        verdict = results.get("verdict") or {}
        step_name = verdict.get("recommended_step")
        if require_step and step_name is None:
            raise gcmd.error(
                "SERVO_AUTOTUNE: aborting at stage %r (run %s): no "
                "recommendation - %s"
                % (
                    stage,
                    run.run_dir,
                    verdict.get("reason") or "no reason given",
                )
            )
        if step_name is not None:
            flags = set(verdict.get("flags") or [])
            flags |= set(self._step_flags(results, step_name))
            bad = sorted(flags & VERDICT_ABORT_FLAGS)
            if bad:
                raise gcmd.error(
                    "SERVO_AUTOTUNE: aborting at stage %r (run %s): verdict "
                    "flags %s on step %r" % (stage, run.run_dir, bad, step_name)
                )
        return verdict

    def _issue_apply_writes(
        self, gcmd: Any, applies: list[dict[str, Any]]
    ) -> None:
        if not applies:
            return
        lines = [
            "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=%s"
            % (a["servo"], a["addr"], a["value"], a["type"])
            for a in applies
        ]
        with servo_param.suppress_write_log():
            self.gcode.run_script_from_command("\n".join(lines))
        for a in applies:
            node, slot = self._resolve_node_slot(a["servo"])
            index, subindex = servo_param.parse_address(a["addr"])
            size, raw = servo_param.read_param(
                self.printer, node, slot, index, subindex
            )
            value = servo_param.decode_typed(raw, size, a["type"])
            if value != a["value"]:
                raise gcmd.error(
                    "APPLY readback mismatch on %s %s: wrote %d, read %d"
                    % (a["servo"], a["addr"], a["value"], value)
                )

    def _chosen_swept(
        self, run: ExperimentRun, step_name: str
    ) -> dict[str, Any]:
        for step in run.manifest["steps"]:
            if step["name"] == step_name:
                return step["swept"]
        raise self.printer.command_error(
            "step %r missing from manifest %s" % (step_name, run.manifest_path)
        )

    def _apply_verdict(
        self,
        gcmd: Any,
        run: ExperimentRun,
        results: dict[str, Any],
        axis: str,
    ) -> None:
        verdict = results.get("verdict") or {}
        step_name = verdict.get("recommended_step")
        apply = verdict.get("apply")
        if step_name is None or apply is None:
            raise gcmd.error(
                "APPLY=1: nothing to apply - verdict on run %s: %s"
                % (run.run_dir, verdict.get("reason") or "no reason given")
            )
        self._issue_apply_writes(gcmd, apply)
        before = self._step_headline(results, step_name)
        swept = self._chosen_swept(run, step_name)
        overrides = {"ACCEL": swept["accel"]} if "accel" in swept else {}
        verify_gcmd = _OverrideGcmd(gcmd, overrides) if overrides else gcmd
        verify_run, verify_results = self._measure_tracking(
            verify_gcmd, axis, "verify_%s" % (run.stamp,)
        )
        verify_step_name = verify_results["steps"][0]["name"]
        after = self._step_headline(verify_results, verify_step_name)
        gcmd.respond_info(
            "APPLY verified (%s): ferr_peak %.0f -> %.0f counts, "
            "overshoot %.0f -> %.0f counts | sweep %s -> verify %s"
            % (
                step_name,
                before[0],
                after[0],
                before[1],
                after[1],
                run.run_dir,
                verify_run.run_dir,
            )
        )

    @overload
    def _floats(self, text: str) -> list[float]: ...
    @overload
    def _floats(self, text: None) -> None: ...
    def _floats(self, text: str | None) -> list[float] | None:
        return servo_strokes.parse_floats(text)

    def _motor(
        self, gcmd: Any, required: bool
    ) -> tuple[float | None, float | None]:
        torque = gcmd.get_float("TORQUE_NM", self.rated_torque_nm)
        inertia = gcmd.get_float("INERTIA_KGM2", self.rotor_inertia_kgm2)
        if required:
            if torque is None:
                raise gcmd.error(
                    "TORQUE_NM required - set rated_torque_nm in "
                    "[servo_calibration] or pass TORQUE_NM= (motor rated torque, N*m)"
                )
            if inertia is None:
                raise gcmd.error(
                    "INERTIA_KGM2 required - set rotor_inertia_kgm2 in "
                    "[servo_calibration] or pass INERTIA_KGM2= (rotor inertia, kg*m^2)"
                )
        elif (torque is None) != (inertia is None):
            raise gcmd.error(
                "TORQUE_NM and INERTIA_KGM2 must be given together"
            )
        return torque, inertia

    def _servo(self, gcmd: Any) -> str:
        default = self.servos[0] if len(self.servos) == 1 else None
        servo = gcmd.get("SERVO", default)
        if servo is None:
            raise gcmd.error(
                "SERVO= is required - name the drive explicitly (e.g. SERVO=motor_a)"
            )
        return servo

    def _servos(self, gcmd: Any, axis: str | None = None) -> list[str]:
        servo = gcmd.get("SERVO", None)
        if servo is not None:
            return [s.strip() for s in servo.split(",") if s.strip()]
        if axis is None:
            axis = gcmd.get("AXIS", None)
        if axis is not None:
            return servo_strokes.axis_servos(gcmd, self._kin(), axis.upper())
        if len(self.servos) == 1:
            return [self.servos[0]]
        raise gcmd.error(
            "AXIS= or SERVO= is required (SERVO= accepts a comma list)"
        )

    def _reject_corexy_only_params(self, gcmd: Any) -> None:
        bad = [
            p
            for p in ("SERVOS", "X_START", "X_END", "Y_START", "Y_END")
            if gcmd.get(p, None) is not None
        ]
        if bad:
            raise gcmd.error(
                "%s require coupled_xy kinematics - the active kinematics "
                "is not CoreXY" % (", ".join(bad),)
            )

    def _strokes(
        self,
        axis: str,
        start: float,
        end: float,
        speed: float,
        accel: float,
        iterations: int,
        dwell: int,
    ) -> None:
        servo_strokes.emit_strokes(
            self.gcode,
            lambda u: "%s%.3f" % (axis, u),
            start,
            end,
            1.0,
            speed,
            accel,
            iterations,
            dwell,
        )

    def _goto_xy(self, x: float, y: float, dwell: int) -> None:
        servo_strokes.goto_xy(self.gcode, self.travel_speed, x, y, dwell)

    def _prep(self, axis: str, dwell: int) -> None:
        servo_strokes.prep(self.printer, self.gcode, axis, dwell)

    def _restore(self) -> None:
        self.gcode.run_script_from_command("RESET_VELOCITY_LIMIT")

    def _start_capture(self, name: str, servos: list[str]) -> None:
        if self._active_run is None:
            raise self.printer.command_error(
                "servo capture requested without an active experiment run"
            )
        self._capture_sync_loss = (
            name,
            list(servos),
            self._sync_loss_counts(servos),
        )
        self._servo_capture().start_capture_to(
            self._active_run.step_scap(name), servos
        )

    def _stop_capture(self) -> None:
        self._servo_capture().stop_capture()
        self._check_sync_loss()

    def _sync_loss_counts(self, servos: list[str]) -> dict[str, int]:
        """C13.04 per drive - the drive's own EtherCAT sync loss counter.
        The drive silently tolerates up to C13.02 (default 8) consecutive
        lost/late sync events before faulting, so this counter is the only
        way to see the tolerated ones. A failed read aborts the command
        (not the printer): the counter is diagnostics, and a CoE abort here
        means the drive does not expose it where expected."""
        counts = {}
        for servo in servos:
            try:
                counts[servo] = self._read_param(servo, SYNC_LOSS_COUNT_ADDR)
            except Exception as e:
                raise self.printer.command_error(
                    "reading EtherCAT sync loss counter C13.04 (%s) failed "
                    "for %s: %s" % (SYNC_LOSS_COUNT_ADDR, servo, e)
                )
        return counts

    def _check_sync_loss(self) -> None:
        if self._capture_sync_loss is None:
            return
        name, servos, before = self._capture_sync_loss
        self._capture_sync_loss = None
        after = self._sync_loss_counts(servos)
        deltas = {
            servo: (after[servo] - before[servo]) & 0xFFFF for servo in servos
        }
        hits = {servo: d for servo, d in deltas.items() if d}
        if not hits:
            return
        detail = ", ".join(
            "%s +%d" % (servo, d) for servo, d in sorted(hits.items())
        )
        self.gcode.respond_info(
            "WARNING step %s: EtherCAT sync loss count (C13.04) incremented "
            "during the capture: %s. The drive tolerated lost/late sync "
            "cycles without faulting (it only faults after C13.02 "
            "consecutive losses) - this step's tracking metrics are "
            "contaminated." % (name, detail)
        )
        structured_log.event(
            "calibration",
            "sync_loss",
            step=name,
            drives=detail,
            total=sum(hits.values()),
        )

    def _accel_chip(self, gcmd: Any) -> tuple[Any, str | None]:
        chip_name = gcmd.get("ACCEL_CHIP", self.accel_chip_name)
        if chip_name is None:
            return None, None
        return self.printer.lookup_object(chip_name.strip()), chip_name

    def _write_accel_csv(
        self, gcmd: Any, aclient: Any, chip_name: str, step_name: str
    ) -> str:
        if not aclient.has_valid_samples():
            raise gcmd.error(
                "accelerometer %r measured no data for step %s"
                % (chip_name, step_name)
            )
        assert self._active_run is not None, "accel CSV written outside a run"
        path = self._active_run.step_accel_csv(step_name)
        with open(path, "w") as f:
            f.write("#time,accel_x,accel_y,accel_z\n")
            for t, accel_x, accel_y, accel_z in aclient.get_samples():
                f.write(
                    "%.6f,%.6f,%.6f,%.6f\n" % (t, accel_x, accel_y, accel_z)
                )
        gcmd.respond_info("Accelerometer data written to %s" % (path,))
        return path

    def _run(self, gcmd: Any, argv: list[str], timeout: float) -> str:
        reactor = self.printer.get_reactor()
        label = os.path.basename(argv[0])
        try:
            proc = subprocess.Popen(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=lambda: os.nice(10),
            )
        except Exception:
            logging.exception("servo_calibration: failed to launch %s", label)
            raise gcmd.error("Error launching %s" % (label,))
        assert proc.stdout is not None, "Popen was given stdout=PIPE"
        fd = proc.stdout.fileno()
        buf = [""]
        output: list[str] = []

        def emit(data: str) -> None:
            buf[0] += data
            if "\n" in buf[0]:
                head, _, buf[0] = buf[0].rpartition("\n")
                gcmd.respond_info(head)
                output.append(head)

        def on_readable(eventtime: float) -> None:
            try:
                emit(os.read(fd, 4096).decode())
            except Exception:
                pass

        hdl = reactor.register_fd(fd, on_readable)
        gcmd.respond_info("Running %s ..." % (label,))
        eventtime = reactor.monotonic()
        endtime = eventtime + timeout
        complete = False
        while eventtime < endtime:
            eventtime = reactor.pause(eventtime + 0.05)
            if proc.poll() is not None:
                complete = True
                break
        reactor.unregister_fd(hdl)
        if not complete:
            proc.terminate()
            raise gcmd.error("%s timed out after %.0fs" % (label, timeout))
        while True:
            data = os.read(fd, 4096).decode()
            if not data:
                break
            emit(data)
        if buf[0]:
            gcmd.respond_info(buf[0])
            output.append(buf[0])
        if proc.returncode:
            raise gcmd.error(
                "%s exited with code %d" % (label, proc.returncode)
            )
        return "\n".join(output)

    def _notch_state(self, gcmd: Any, servo: str) -> dict[str, Any]:
        state: dict[str, Any] = {
            "mode": self._read_notch_param(gcmd, servo, NOTCH_MODE_ADDR)
        }
        for (label, _addrs), (freq, width, depth) in zip(
            NOTCH_READBACK, self._read_notches(gcmd, servo)
        ):
            state[label] = {"freq_hz": freq, "width": width, "depth": depth}
        return state

    def _read_notches(
        self, gcmd: Any, servo: str
    ) -> list[tuple[int, int, int]]:
        return [
            (
                self._read_notch_param(gcmd, servo, addrs[0]),
                self._read_notch_param(gcmd, servo, addrs[1]),
                self._read_notch_param(gcmd, servo, addrs[2]),
            )
            for _label, addrs in NOTCH_READBACK
        ]

    def _read_notch_param(self, gcmd: Any, servo: str, addr: str) -> int:
        try:
            return self._read_param(servo, addr)
        except (RuntimeError, ValueError) as e:
            raise gcmd.error(
                "notch readback failed for %s %s: %s" % (servo, addr, e)
            )

    def _pattern_geometry_params(
        self, gcmd: Any
    ) -> tuple[list[tuple[float, float]], float, float, dict[str, Any]]:
        inset = gcmd.get_float("BOUND", 20.0, minval=0.0)
        small = gcmd.get_float("SMALL_SIZE", 20.0, above=0.0)
        x_lo, x_hi = self._config_bounds(gcmd, "X")
        y_lo, y_hi = self._config_bounds(gcmd, "Y")
        points, start_x, start_y = servo_strokes.pattern_geometry(
            gcmd, x_lo, x_hi, y_lo, y_hi, inset, small
        )
        plan = {
            "pattern": {
                "x_bounds": [x_lo, x_hi],
                "y_bounds": [y_lo, y_hi],
                "inset": inset,
                "small_size": small,
                "segments": len(points),
            }
        }
        return points, start_x, start_y, plan

    def _corexy_rails(self, gcmd: Any, axis: str) -> list[Any] | None:
        kin = self._kin()
        if kin.coupled_xy() and axis in ("X", "Y"):
            return servo_strokes.axis_rails(gcmd, kin, axis)
        return None

    def _read_param(self, servo: str, addr: str) -> int:
        from .. import servo_param

        node, slot = self._resolve_node_slot(servo)
        handle = node.get_engine_handle()
        if handle is None:
            raise self.printer.command_error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        index, subindex = servo_param.parse_address(addr)
        _size, raw = engine.sdo_read(handle, slot, index, subindex)
        return raw

    def _resolve_node_slot(self, servo: str) -> tuple[Any, int]:
        from .. import servo_axis

        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo, "SERVO_CALIBRATION"
        )
        node = self.printer.lookup_object(
            "ethercat_node " + motor.get_node_name()
        )
        return node, node.get_slot_for_motor(motor.get_motor_name())

    def _measure_tracking(
        self, gcmd: Any, axis: str, name: str
    ) -> tuple[ExperimentRun, dict[str, Any]]:
        """The SERVO_MEASURE_TRACKING body - shared with APPLY=1's
        verification stroke and every SERVO_AUTOTUNE tracking stage."""
        plan = servo_strokes.build_plan(gcmd, self._kin(), self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 3, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        servos = plan.servos
        rails = plan.rails
        belts_rails = (
            rails
            if not plan.diagonal and len(rails) == 2 and axis in ("X", "Y")
            else None
        )
        stroke_plan = {
            "start": plan.start,
            "end": plan.end,
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd, "tracking", name, axis, servos, stroke_plan, belts_rails
        )
        try:
            for prep_axis in plan.prep:
                self._prep(prep_axis, dwell)
            self._start_capture(name, servos)
            servo_strokes.emit_strokes(
                self.gcode,
                plan.coord,
                plan.start,
                plan.end,
                plan.th_per_unit,
                speed,
                accel,
                iterations,
                dwell,
            )
            self._stop_capture()
            self._restore()
            run.record_step(SweepStep(name, {}, []))
            results = self._analyze_and_report(gcmd, run)
        finally:
            self._active_run = None
        return run, results

    def _dynamics_out_path(
        self, gcmd: Any, run: ExperimentRun, name: str
    ) -> str:
        os.makedirs(self.dynamics_dir, exist_ok=True)
        path = os.path.join(
            self.dynamics_dir, "dynamics_%s_%s.toml" % (name, run.stamp)
        )
        if os.path.exists(path):
            raise gcmd.error(
                "dynamics profile %s already exists (never overwritten)"
                % (path,)
            )
        return path

    def _config_bounds(self, gcmd: Any, axis: str) -> tuple[float, float]:
        lo, hi = self.bounds.get(axis, (None, None))
        if lo is None or hi is None:
            raise gcmd.error(
                "no stroke bounds configured for axis %s" % (axis,)
            )
        return lo, hi
