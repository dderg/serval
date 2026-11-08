"""Time integration of mechanics + motors (kernel.c), in chunks.

Semi-implicit Euler at dt_out / substeps: currents first (exact RL step
under the clipped dead-beat voltage), then torques, velocities, positions.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from . import mechanics, motor, native

SUBSTEPS = 10
CHUNK_SAMPLES = 40000


@dataclasses.dataclass(frozen=True)
class Model:
    system: mechanics.System
    electrical: motor.Electrical
    ideal_motors: bool = False


@dataclasses.dataclass
class Result:
    t: np.ndarray
    nozzle: np.ndarray
    sensor: np.ndarray
    delta: np.ndarray
    delta_absmax: np.ndarray
    torque: np.ndarray
    current: np.ndarray


def _system_struct(ffi, model: Model):
    sysm = model.system
    n = sysm.n
    if n > native.MAX_DOF:
        raise ValueError("%d DOF > kernel limit %d" % (n, native.MAX_DOF))
    st = ffi.new("struct ps_system *")
    st.n = n
    st.n_motors = len(sysm.rotors)
    st.n_out = 4
    for i in range(n):
        st.inv_mass[i] = 1.0 / sysm.mass[i]
        for j in range(n):
            st.stiffness[i * native.MAX_DOF + j] = sysm.stiffness[i, j]
            st.damping[i * native.MAX_DOF + j] = sysm.damping[i, j]
    rows = np.vstack([sysm.nozzle, sysm.sensor])
    for o in range(4):
        for j in range(n):
            st.out_rows[o * native.MAX_DOF + j] = rows[o, j]
    e = model.electrical
    for m, dof in enumerate(sysm.rotors):
        mo = st.motors[m]
        mo.resistance = e.resistance
        mo.inductance = e.inductance
        mo.torque_constant = e.torque_constant
        mo.pole_pairs = e.pole_pairs
        mo.detent_torque = e.detent_torque
        mo.supply_voltage = e.supply_voltage
        mo.current_amplitude = e.current_amplitude
        mo.lead_time = sysm.motor_damping / sysm.magnetic_stiffness
        mo.dof = dof
        mo.ideal = int(model.ideal_motors)
    return st


def _initial_state(ffi, model: Model, phi_cmd):
    """At rest with every rotor on its commanded angle and every other
    coordinate where the springs balance (K_ff q_f = -K_fr q_r)."""
    sysm = model.system
    p = model.electrical.pole_pairs
    st = ffi.new("struct ps_state *")
    rotors = list(sysm.rotors)
    free = [i for i in range(sysm.n) if i not in rotors]
    q = np.zeros(sysm.n)
    q[rotors] = np.asarray(phi_cmd, dtype=float) / p
    if free:
        k = sysm.stiffness
        q[free] = np.linalg.solve(
            k[np.ix_(free, free)], -k[np.ix_(free, rotors)] @ q[rotors]
        )
    for i in range(sysm.n):
        st.q[i] = q[i]
        st.qd[i] = 0.0
    amp = model.electrical.current_amplitude
    for m, phi in enumerate(phi_cmd):
        st.current[m][0] = amp * math.cos(phi)
        st.current[m][1] = amp * math.sin(phi)
    return st


def run(
    model: Model, t0, dt_out, n_out, command, command_rate, substeps=SUBSTEPS
) -> Result:
    """Simulate from rest at the command's position at t0; sample s is
    the state at t0 + (s + 1) * dt_out. command(t) -> (2, len(t))
    commanded electrical angles (rad) of motors A and B; command_rate(t)
    their smooth rates (rad/s), which the load-angle dashpot damps
    against (a step stream's own rate is a train of impulses)."""
    ffi, lib = native.load()
    sysp = _system_struct(ffi, model)
    state = _initial_state(ffi, model, command(np.array([t0]))[:, 0])
    dt = dt_out / substeps
    nm = len(model.system.rotors)
    out = np.empty((n_out, 4))
    delta = np.empty((n_out, nm))
    absmax = np.empty((n_out, nm))
    torque = np.empty((n_out, nm))
    current = np.empty((n_out, nm))
    for start in range(0, n_out, CHUNK_SAMPLES):
        count = min(CHUNK_SAMPLES, n_out - start)
        k = np.arange(count * substeps + 1)
        t_fine = t0 + (start * substeps + k) * dt
        cmd = np.ascontiguousarray(command(t_fine), dtype=float)
        rate = np.ascontiguousarray(command_rate(t_fine), dtype=float)
        for name, arr in (("command", cmd), ("command_rate", rate)):
            if arr.shape != (nm, len(t_fine)) or not np.isfinite(arr).all():
                raise ValueError(
                    "%s returned %r / non-finite" % (name, arr.shape)
                )
        sl = slice(start, start + count)
        bufs = [
            np.empty((count, 4)),
            *(np.empty((count, nm)) for _ in range(4)),
        ]
        lib.ps_run(
            sysp,
            state,
            dt,
            substeps,
            count,
            ffi.cast("double *", cmd.ctypes.data),
            ffi.cast("double *", rate.ctypes.data),
            *(ffi.cast("double *", b.ctypes.data) for b in bufs),
        )
        out[sl], delta[sl], absmax[sl], torque[sl], current[sl] = bufs
    if not np.isfinite(out).all():
        raise FloatingPointError("simulation diverged (non-finite positions)")
    return Result(
        t=t0 + dt_out * np.arange(1, n_out + 1),
        nozzle=out[:, :2],
        sensor=out[:, 2:],
        delta=delta,
        delta_absmax=absmax,
        torque=torque,
        current=current,
    )


def pole_slips(delta) -> np.ndarray:
    """Electrical cycles lost per motor: the load angle's nearest multiple
    of 2 pi changes each time it passes an odd multiple of pi."""
    cycles = np.round(np.asarray(delta) / (2.0 * math.pi))
    return np.abs(np.diff(cycles, axis=0)).sum(axis=0)
