"""Prototype: junction speed from the shaped corner deviation (harness only).

Replaces the square_corner_velocity term of toolhead.Move.calc_junction by
the highest junction speed whose corner, after input shaping, stays within a
geometric budget. Everything else in calc_junction (cruise, extruder,
centripetal and lookahead limits) is unchanged. Installed into the klippy
subprocess by capture.py; no upstream file is modified.

Corner model: decelerate at `a` into the vertex at speed v, change direction
instantly, accelerate at `a` away. Per axis the commanded position around
the vertex is P(tau) = u1*min(tau,0)*v + u2*max(tau,0)*v
                     + a/2*(u2*max(tau,0)^2 - u1*min(tau,0)^2),
linear in v, so the shaped position is A(t) + v*B(t) with A, B built from
four kernel moments per axis that do not depend on the corner. The corner
deviation is the harness metric: the larger of the shaped path's distance
to the two legs and the vertex's distance to the shaped path.

Curve cap: an isolated-vertex model misses curves drawn as many short
segments, where the kernel spans several vertices and pulls the path inward
by ~sigma^2/2 * v^2 * curvature. Each junction is therefore also limited to
v^2 <= 2*budget / (sigma^2 * turn / mean segment length), with sigma^2 the
larger kernel variance of X and Y.
"""

from __future__ import annotations

import json
import math

import numpy as np

from klippy import toolhead

from .analysis import Kernel

MODEL_DT = 1e-4
SPEED_SAMPLES = 48
REFINE_STEPS = 12
MIN_XY_DIRECTION = 1e-9


class CornerModel:
    def __init__(self, kernels):
        reach = max(max(abs(s) for s in k.support) for k in kernels)
        self.t = np.arange(-reach - MODEL_DT, reach + 2 * MODEL_DT, MODEL_DT)
        moments = [self._moments(k) for k in kernels]
        self.N, self.Q, self.N2, self.Q2 = (
            np.stack([m[i] for m in moments], axis=-1) for i in range(4)
        )
        self.variance = max(k.second_moment() for k in kernels)

    def _moments(self, kernel):
        if kernel.kind == "smoother":
            m_lo, w = kernel.grid_weights(MODEL_DT)
            shifts = (m_lo + np.arange(len(w))) * MODEL_DT
        else:
            w, shifts = kernel.weights, kernel.shifts
        tau = self.t[:, None] + shifts[None, :]
        before = np.minimum(tau, 0.0)
        after = np.maximum(tau, 0.0)
        return (
            before @ w,
            after @ w,
            (before * before) @ w,
            (after * after) @ w,
        )

    def deviation(self, u1, u2, accel, speeds):
        B = u1 * self.N + u2 * self.Q
        A = 0.5 * accel * (u2 * self.Q2 - u1 * self.N2)
        S = A[None] + speeds[:, None, None] * B[None]
        legs = []
        for direction in (-u1, u2):
            unit = direction / np.linalg.norm(direction)
            along = np.maximum(S @ unit, 0.0)
            legs.append(np.linalg.norm(S - along[..., None] * unit, axis=-1))
        outward = np.minimum(*legs).max(axis=-1)
        a, b = S[:, :-1], S[:, 1:]
        ab = b - a
        L2 = (ab * ab).sum(axis=-1)
        u = np.clip(
            -(a * ab).sum(axis=-1) / np.where(L2 > 0.0, L2, 1.0), 0.0, 1.0
        )
        inward = np.linalg.norm(a + u[..., None] * ab, axis=-1).min(axis=-1)
        return np.maximum(outward, inward)

    def max_speed(self, u1, u2, accel, v_cap, budget):
        speeds = np.linspace(0.0, v_cap, SPEED_SAMPLES)
        ok = self.deviation(u1, u2, accel, speeds) <= budget
        if not ok[0]:
            return 0.0
        if ok.all():
            return v_cap
        bad = int(np.argmin(ok))
        lo, hi = speeds[bad - 1], speeds[bad]
        for _ in range(REFINE_STEPS):
            mid = 0.5 * (lo + hi)
            if self.deviation(u1, u2, accel, np.array([mid]))[0] <= budget:
                lo = mid
            else:
                hi = mid
        return lo


class ShaperCornerLimit:
    def __init__(self, printer_obj, budget_mm, shaper_state):
        self.printer = printer_obj
        self.budget = budget_mm
        self.shaper_state = shaper_state
        self.models = {}

    def model(self):
        state = self.shaper_state(self.printer)
        key = json.dumps(state, sort_keys=True)
        if key not in self.models:
            self.models[key] = CornerModel(
                [Kernel.from_shaper(state.get(axis)) for axis in "xy"]
            )
        return self.models[key]

    def max_v2(self, prev_move, move, cap_v2):
        u1 = np.array(prev_move.axes_r[:2])
        u2 = np.array(move.axes_r[:2])
        n1, n2 = np.linalg.norm(u1), np.linalg.norm(u2)
        if min(n1, n2) < MIN_XY_DIRECTION:
            return cap_v2
        model = self.model()
        turn = math.acos(min(1.0, max(-1.0, float(u1 @ u2) / (n1 * n2))))
        mean_xy_length = 0.5 * (prev_move.move_d * n1 + move.move_d * n2)
        curvature = turn / mean_xy_length
        if model.variance > 0.0 and curvature > 0.0:
            cap_v2 = min(
                cap_v2, 2.0 * self.budget / (model.variance * curvature)
            )
        accel = min(prev_move.accel, move.accel)
        v = model.max_speed(u1, u2, accel, math.sqrt(cap_v2), self.budget)
        return v * v


def install(toolhead_obj, budget_mm, shaper_state):
    limiter = ShaperCornerLimit(toolhead_obj.printer, budget_mm, shaper_state)

    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        ea_v2 = [
            ea.calc_junction(prev_move, self, e_index + 3)
            for e_index, ea in enumerate(self.toolhead.extra_axes)
        ]
        max_start_v2 = min(
            [
                self.max_cruise_v2,
                prev_move.max_cruise_v2,
                prev_move.next_junction_v2,
                prev_move.max_start_v2 + prev_move.delta_v2,
            ]
            + ea_v2
        )
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -(
            axes_r[0] * prev_axes_r[0]
            + axes_r[1] * prev_axes_r[1]
            + axes_r[2] * prev_axes_r[2]
        )
        sin_theta_d2 = math.sqrt(max(0.5 * (1.0 - junction_cos_theta), 0.0))
        cos_theta_d2 = math.sqrt(max(0.5 * (1.0 + junction_cos_theta), 0.0))
        if 1.0 - sin_theta_d2 > 0.0 and cos_theta_d2 > 0.0:
            quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2
            max_start_v2 = min(
                max_start_v2,
                self.delta_v2 * quarter_tan_theta_d2,
                prev_move.delta_v2 * quarter_tan_theta_d2,
            )
            max_start_v2 = limiter.max_v2(prev_move, self, max_start_v2)
        self.max_start_v2 = max_start_v2
        self.max_smoothed_v2 = min(
            max_start_v2, prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2
        )

    toolhead.Move.calc_junction = calc_junction
