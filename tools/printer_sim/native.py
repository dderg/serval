"""Build and load kernel.c (cffi ABI mode), cached by source hash."""

from __future__ import annotations

import functools
import hashlib
import os
import pathlib
import subprocess
import tempfile

import cffi

HERE = pathlib.Path(__file__).resolve().parent
SOURCE = HERE / "kernel.c"
BUILD = HERE / ".build"
MAX_DOF = 8
MAX_MOTORS = 2
MAX_OUT = 4
CDEF = """
struct ps_motor {
    double resistance, inductance, torque_constant, pole_pairs;
    double detent_torque, supply_voltage, current_amplitude, lead_time;
    int dof, ideal;
};
struct ps_system {
    int n, n_motors, n_out;
    double inv_mass[%(dof)d];
    double stiffness[%(dof2)d];
    double damping[%(dof2)d];
    double out_rows[%(out)d];
    struct ps_motor motors[%(motors)d];
};
struct ps_state {
    double q[%(dof)d], qd[%(dof)d];
    double current[%(motors)d][2];
};
int ps_run(const struct ps_system *sys, struct ps_state *st, double dt,
           int substeps, int n_samples, const double *cmd,
           const double *cmd_rate, double *out, double *delta,
           double *delta_absmax, double *torque, double *current);
struct ps_extruder {
    struct ps_motor motor;
    double inertia, lever, stiffness, time_constant, max_rate;
};
struct ps_extruder_state {
    double theta, omega, e_out;
    double current[2];
};
int ps_extruder_run(const struct ps_extruder *ex, struct ps_extruder_state *st,
                    double dt, int substeps, int n_samples, const double *cmd,
                    const double *cmd_rate, double *e_out, double *delta,
                    double *delta_absmax, double *force);
int ps_deposit(double *grid, int nx, int ny, double x0, double y0, double dx,
               int n, const double *x, const double *y, const double *dv,
               const double *width);
""" % {
    "dof": MAX_DOF,
    "dof2": MAX_DOF * MAX_DOF,
    "out": MAX_OUT * MAX_DOF,
    "motors": MAX_MOTORS,
}
CFLAGS = ("-O2", "-shared", "-fPIC", "-std=c99", "-Wall", "-Werror")


@functools.lru_cache(maxsize=None)
def load():
    source = SOURCE.read_bytes()
    key = hashlib.sha256(source + " ".join(CFLAGS).encode()).hexdigest()[:16]
    lib_path = BUILD / ("kernel-%s.so" % (key,))
    if not lib_path.exists():
        BUILD.mkdir(exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=BUILD, suffix=".so")
        os.close(fd)
        subprocess.run(
            ["cc", *CFLAGS, "-o", tmp, str(SOURCE), "-lm"],
            check=True,
        )
        os.replace(tmp, lib_path)
    ffi = cffi.FFI()
    ffi.cdef(CDEF)
    return ffi, ffi.dlopen(str(lib_path))
