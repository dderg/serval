"""Module 3: extruder stepper, filament compression and melt flow.

The extruder rotor (motor.py's model, through lever = metres of filament
per radian) pushes filament position u = lever * theta. Filament and melt
compress by c = u - e_out against the spring k = back_pressure *
filament_area / time_constant; the nozzle passes filament at rate
c / time_constant, capped at max_flow / filament_area. So with the rotor
on its command, e_out follows u through the first-order lag
u = e_out + time_constant * de_out/dt that pressure advance assumes, and
the filament force k * c loads the rotor (an extruder pole slip when it
outgrows the motor).
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from . import motor, native, parts

SUBSTEPS = 10
CHUNK_SAMPLES = 40000


@dataclasses.dataclass
class Result:
    t: np.ndarray
    e_out: np.ndarray
    volume: np.ndarray
    delta: np.ndarray
    delta_absmax: np.ndarray
    force: np.ndarray

    @property
    def flow(self):
        """mm^3/s leaving the nozzle in each sample interval."""
        dt = self.t[1] - self.t[0]
        return np.diff(self.volume, prepend=self.volume[0]) / dt


def electrical(ex: parts.Extrusion) -> motor.Electrical:
    return motor.Electrical.of(ex.motor, ex.driver)


def lead_time(ex: parts.Extrusion) -> float:
    """Stiffness-proportional damping of the rotor's magnetic mode at
    damping_ratio: 2 zeta / w_n."""
    e = electrical(ex)
    w = math.sqrt(e.magnetic_stiffness / ex.motor.rotor_inertia)
    return 2.0 * ex.damping_ratio / w


def angle_per_mm(ex: parts.Extrusion) -> float:
    """Commanded electrical angle per mm of filament."""
    return electrical(ex).pole_pairs / (ex.lever * 1e3)


def run(
    ex: parts.Extrusion,
    t0,
    dt_out,
    n_out,
    command,
    command_rate,
    ideal=False,
    substeps=SUBSTEPS,
) -> Result:
    """From rest (no compression) at the command's position at t0; sample
    s is the state at t0 + (s + 1) * dt_out. command(t) / command_rate(t):
    the extruder motor's commanded electrical angle and its smooth rate."""
    ffi, lib = native.load()
    e = electrical(ex)
    st = ffi.new("struct ps_extruder *")
    mo = st.motor
    mo.resistance = e.resistance
    mo.inductance = e.inductance
    mo.torque_constant = e.torque_constant
    mo.pole_pairs = e.pole_pairs
    mo.detent_torque = e.detent_torque
    mo.supply_voltage = e.supply_voltage
    mo.current_amplitude = e.current_amplitude
    mo.lead_time = lead_time(ex)
    mo.dof = 0
    mo.ideal = int(ideal)
    st.inertia = ex.motor.rotor_inertia
    st.lever = ex.lever
    st.stiffness = ex.stiffness
    st.time_constant = ex.time_constant
    st.max_rate = ex.max_flow / ex.filament_area * 1e-3
    state = ffi.new("struct ps_extruder_state *")
    phi0 = float(command(np.array([t0]))[0])
    state.theta = phi0 / e.pole_pairs
    state.omega = 0.0
    state.e_out = ex.lever * state.theta
    state.current[0] = e.current_amplitude * math.cos(phi0)
    state.current[1] = e.current_amplitude * math.sin(phi0)
    e0 = state.e_out
    dt = dt_out / substeps
    bufs = [np.empty(n_out) for _ in range(4)]
    for start in range(0, n_out, CHUNK_SAMPLES):
        count = min(CHUNK_SAMPLES, n_out - start)
        t_fine = t0 + (start * substeps + np.arange(count * substeps + 1)) * dt
        cmd = np.ascontiguousarray(command(t_fine), dtype=float)
        rate = np.ascontiguousarray(command_rate(t_fine), dtype=float)
        for name, arr in (("command", cmd), ("command_rate", rate)):
            if arr.shape != t_fine.shape or not np.isfinite(arr).all():
                raise ValueError(
                    "%s returned %r / non-finite" % (name, arr.shape)
                )
        chunk = [np.empty(count) for _ in range(4)]
        lib.ps_extruder_run(
            st,
            state,
            dt,
            substeps,
            count,
            ffi.cast("double *", cmd.ctypes.data),
            ffi.cast("double *", rate.ctypes.data),
            *(ffi.cast("double *", b.ctypes.data) for b in chunk),
        )
        for buf, c in zip(bufs, chunk):
            buf[start : start + count] = c
    e_out, delta, absmax, force = bufs
    if not np.isfinite(e_out).all():
        raise FloatingPointError("extruder simulation diverged")
    e_out_mm = (e_out - e0) * 1e3
    return Result(
        t=t0 + dt_out * np.arange(1, n_out + 1),
        e_out=e_out_mm,
        volume=e_out_mm * ex.filament_area,
        delta=delta,
        delta_absmax=absmax,
        force=force,
    )
