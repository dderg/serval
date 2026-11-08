"""Motor commands from a planned gcode: motion_eval's capture of klippy.

The motors are driven by the recorded step stream (each microstep holds
its electrical angle until the next step, no driver interpolation), or,
for comparison, by the continuous executed (shaped) motion. The
reference for the nozzle is the executed motion: an infinitely stiff
printer would follow it exactly.
"""

from __future__ import annotations

import dataclasses
import functools
import math

import numpy as np

from tools.motion_eval import accel_residual, analysis, evaluate, oscillator

DT = accel_residual.DT
SETTLE_S = 0.3
MOTORS = ("stepper_x", "stepper_y")
COREXY_ROWS = {"stepper_x": (1.0, 1.0, 0.0), "stepper_y": (1.0, -1.0, 0.0)}


class DriveError(Exception):
    pass


@dataclasses.dataclass
class Drive:
    t0: float
    n: int
    reference: np.ndarray
    windows: list
    step_times: tuple
    step_positions: tuple
    rotation_distance: float
    pole_pairs: int
    clock_freq: float

    @property
    def t(self):
        return self.t0 + DT * np.arange(1, self.n + 1)

    def _angle(self, motor_mm):
        return (
            2.0
            * math.pi
            * self.pole_pairs
            * np.asarray(motor_mm)
            / self.rotation_distance
        )

    def step_command(self, t):
        out = []
        for times, positions in zip(self.step_times, self.step_positions):
            i = np.searchsorted(times, t, side="right")
            out.append(self._angle(positions[i]))
        return np.vstack(out)

    @functools.cached_property
    def velocity(self):
        return np.gradient(self.reference, DT, axis=0)

    def command_rate(self, t):
        """Rate of the continuous command (rad/s), from the executed
        motion's velocity."""
        t_ref = self.t
        vx = np.interp(t, t_ref, self.velocity[:, 0])
        vy = np.interp(t, t_ref, self.velocity[:, 1])
        return np.vstack([self._angle(vx + vy), self._angle(vx - vy)])

    def continuous_command(self, t):
        t_ref = self.t
        x = np.interp(t, t_ref, self.reference[:, 0])
        y = np.interp(t, t_ref, self.reference[:, 1])
        return np.vstack([self._angle(x + y), self._angle(x - y)])


def _check_corexy(steppers):
    for name, row in COREXY_ROWS.items():
        lm = steppers[name]["linear_map"]
        if tuple(lm["row"]) != row or lm["offset"] != 0.0:
            raise DriveError(
                "%s maps xyz by %r + %r; the model needs corexy a = x + y,"
                " b = x - y" % (name, lm["row"], lm["offset"])
            )


def from_gcode(text, printer_cfg, pole_pairs) -> Drive:
    raw = accel_residual.replay(text, printer_cfg)
    cap = raw.capture
    steppers = cap["steppers"]
    _check_corexy(steppers)
    seg, epochs, t_start, n, _, motion = evaluate.executed_motion(cap, DT)
    settle = int(math.ceil(SETTLE_S / DT))
    xy = np.stack([motion["x"].pos, motion["y"].pos], axis=-1)
    reference = np.concatenate([xy, np.repeat(xy[-1:], settle, axis=0)])
    support = (
        min(evaluate.kernel_support(e.kernels)[0] for e in epochs),
        max(evaluate.kernel_support(e.kernels)[1] for e in epochs),
    )
    step_times, step_positions = [], []
    for name in MOTORS:
        st, trace = steppers[name], raw.traces[name]
        after = analysis.stepper_positions_mm(st, trace)
        step_times.append(trace.clocks / st["clock_freq"])
        step_positions.append(
            np.concatenate([[st["commanded_pos_at_mcu_zero"]], after])
        )
    distances = {
        float(accel_residual.config_option(printer_cfg, m, "rotation_distance"))
        for m in MOTORS
    }
    if len(distances) != 1:
        raise DriveError("stepper_x and stepper_y rotation_distance differ")
    return Drive(
        t0=t_start - DT,
        n=n + settle,
        reference=reference,
        windows=oscillator.constant_velocity_windows(seg, support),
        step_times=tuple(step_times),
        step_positions=tuple(step_positions),
        rotation_distance=distances.pop(),
        pole_pairs=pole_pairs,
        clock_freq=steppers[MOTORS[0]]["clock_freq"],
    )
