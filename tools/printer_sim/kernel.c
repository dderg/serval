#include <math.h>
#include <string.h>

#define PS_MAX_DOF 8
#define PS_MAX_MOTORS 2
#define PS_MAX_OUT 4

struct ps_motor {
    double resistance, inductance, torque_constant, pole_pairs;
    double detent_torque, supply_voltage, current_amplitude, lead_time;
    int dof, ideal;
};

struct ps_system {
    int n, n_motors, n_out;
    double inv_mass[PS_MAX_DOF];
    double stiffness[PS_MAX_DOF * PS_MAX_DOF];
    double damping[PS_MAX_DOF * PS_MAX_DOF];
    double out_rows[PS_MAX_OUT * PS_MAX_DOF];
    struct ps_motor motors[PS_MAX_MOTORS];
};

struct ps_state {
    double q[PS_MAX_DOF], qd[PS_MAX_DOF];
    double current[PS_MAX_MOTORS][2];
};

static double clamp(double v, double lim)
{
    return v > lim ? lim : (v < -lim ? -lim : v);
}

static double motor_step(const struct ps_motor *m, double *i, double phi,
                         double w, double phi_cmd, double dt)
{
    double s = sin(phi), c = cos(phi);
    double alpha = exp(-dt * m->resistance / m->inductance);
    double gain = (1.0 - alpha) / m->resistance;
    double emf_a = -m->torque_constant * w * s;
    double emf_b = m->torque_constant * w * c;
    double target_a = m->current_amplitude * cos(phi_cmd);
    double target_b = m->current_amplitude * sin(phi_cmd);
    double v_a = clamp(emf_a + (target_a - alpha * i[0]) / gain,
                       m->supply_voltage);
    double v_b = clamp(emf_b + (target_b - alpha * i[1]) / gain,
                       m->supply_voltage);
    i[0] = alpha * i[0] + (v_a - emf_a) * gain;
    i[1] = alpha * i[1] + (v_b - emf_b) * gain;
    return m->torque_constant * (i[1] * c - i[0] * s)
        - m->detent_torque * sin(4.0 * phi);
}

/* Advance n_samples output samples of `substeps` steps of dt each.
 * cmd[m * (n_samples * substeps + 1) + k] is motor m's commanded
 * electrical angle at step k and cmd_rate its smooth rate (rad/s). The
 * current vector leads the command by lead_time times the speed error,
 * which damps the load angle and never lifts the torque above K * I.
 * Per sample writes: out (n_out values), load angle and its largest
 * magnitude over the sample, torque and current amplitude per motor. */
int ps_run(const struct ps_system *sys, struct ps_state *st, double dt,
           int substeps, int n_samples, const double *cmd,
           const double *cmd_rate, double *out, double *delta,
           double *delta_absmax, double *torque, double *current)
{
    int n = sys->n, nm = sys->n_motors;
    long stride = (long)n_samples * substeps + 1;
    double tau[PS_MAX_MOTORS] = {0};
    int prescribed[PS_MAX_DOF] = {0};
    for (int m = 0; m < nm; m++)
        prescribed[sys->motors[m].dof] = sys->motors[m].ideal;
    for (int s = 0; s < n_samples; s++) {
        double absmax[PS_MAX_MOTORS] = {0};
        for (int sub = 0; sub < substeps; sub++) {
            long k = (long)s * substeps + sub;
            for (int m = 0; m < nm; m++) {
                const struct ps_motor *mo = &sys->motors[m];
                if (mo->ideal)
                    continue;
                double phi = mo->pole_pairs * st->q[mo->dof];
                double w = st->qd[mo->dof];
                double lead = mo->lead_time
                    * (cmd_rate[m * stride + k] - mo->pole_pairs * w);
                tau[m] = motor_step(mo, st->current[m], phi, w,
                                    cmd[m * stride + k] + lead, dt);
            }
            double f[PS_MAX_DOF];
            for (int a = 0; a < n; a++) {
                double acc = 0.0;
                for (int b = 0; b < n; b++)
                    acc -= sys->stiffness[a * PS_MAX_DOF + b] * st->q[b]
                        + sys->damping[a * PS_MAX_DOF + b] * st->qd[b];
                f[a] = acc;
            }
            for (int m = 0; m < nm; m++)
                if (!sys->motors[m].ideal)
                    f[sys->motors[m].dof] += tau[m];
            for (int a = 0; a < n; a++) {
                if (prescribed[a])
                    continue;
                st->qd[a] += dt * sys->inv_mass[a] * f[a];
                st->q[a] += dt * st->qd[a];
            }
            for (int m = 0; m < nm; m++) {
                const struct ps_motor *mo = &sys->motors[m];
                double next = cmd[m * stride + k + 1];
                if (mo->ideal) {
                    double prev = cmd[m * stride + k];
                    st->q[mo->dof] = next / mo->pole_pairs;
                    st->qd[mo->dof] = (next - prev) / (mo->pole_pairs * dt);
                }
                double d = fabs(next - mo->pole_pairs * st->q[mo->dof]);
                if (d > absmax[m])
                    absmax[m] = d;
            }
        }
        long end = (long)(s + 1) * substeps;
        for (int o = 0; o < sys->n_out; o++) {
            double acc = 0.0;
            for (int b = 0; b < n; b++)
                acc += sys->out_rows[o * PS_MAX_DOF + b] * st->q[b];
            out[(long)s * sys->n_out + o] = acc;
        }
        for (int m = 0; m < nm; m++) {
            const struct ps_motor *mo = &sys->motors[m];
            long i = (long)s * nm + m;
            delta[i] = cmd[m * stride + end] - mo->pole_pairs * st->q[mo->dof];
            delta_absmax[i] = absmax[m];
            torque[i] = mo->ideal ? NAN : tau[m];
            current[i] = mo->ideal ? NAN
                : hypot(st->current[m][0], st->current[m][1]);
        }
    }
    return 0;
}

struct ps_extruder {
    struct ps_motor motor;
    double inertia, lever, stiffness, time_constant, max_rate;
};

struct ps_extruder_state {
    double theta, omega, e_out;
    double current[2];
};

/* One extruder: rotor driven by its motor against the filament spring
 * (stiffness, N/m of compression lever * theta - e_out), the melt
 * draining the compression with time_constant at most max_rate (m/s of
 * filament). motor.ideal holds the rotor on the command. Per sample
 * writes e_out, load angle, its largest magnitude and the filament
 * force. */
int ps_extruder_run(const struct ps_extruder *ex, struct ps_extruder_state *st,
                    double dt, int substeps, int n_samples, const double *cmd,
                    const double *cmd_rate, double *e_out, double *delta,
                    double *delta_absmax, double *force)
{
    const struct ps_motor *mo = &ex->motor;
    double p = mo->pole_pairs;
    for (int s = 0; s < n_samples; s++) {
        double absmax = 0.0, f = 0.0;
        for (int sub = 0; sub < substeps; sub++) {
            long k = (long)s * substeps + sub;
            if (mo->ideal) {
                st->omega = (cmd[k + 1] - cmd[k]) / (p * dt);
                st->theta = cmd[k + 1] / p;
            } else {
                double lead = mo->lead_time * (cmd_rate[k] - p * st->omega);
                double tau = motor_step(mo, st->current, p * st->theta,
                                        st->omega, cmd[k] + lead, dt);
                f = ex->stiffness * (ex->lever * st->theta - st->e_out);
                st->omega += dt * (tau - f * ex->lever) / ex->inertia;
                st->theta += dt * st->omega;
            }
            double rate = (ex->lever * st->theta - st->e_out) / ex->time_constant;
            st->e_out += dt * clamp(rate, ex->max_rate);
            double d = fabs(cmd[k + 1] - p * st->theta);
            if (d > absmax)
                absmax = d;
        }
        long end = (long)(s + 1) * substeps;
        e_out[s] = st->e_out;
        delta[s] = cmd[end] - p * st->theta;
        delta_absmax[s] = absmax;
        force[s] = ex->stiffness * (ex->lever * st->theta - st->e_out);
    }
    return 0;
}

/* Spread volume dv[i] (mm^3) evenly over the grid cells whose centres lie
 * within width[i] / 2 of the segment (x[i], y[i]) - (x[i+1], y[i+1]); a
 * footprint that catches no centre goes to the nearest cell. Returns -1
 * if a footprint leaves the grid. */
int ps_deposit(double *grid, int nx, int ny, double x0, double y0, double dx,
               int n, const double *x, const double *y, const double *dv,
               const double *width)
{
    for (int i = 0; i + 1 < n; i++) {
        if (dv[i] <= 0.0)
            continue;
        double ax = x[i], ay = y[i], bx = x[i + 1], by = y[i + 1];
        double r = 0.5 * width[i];
        double ex = bx - ax, ey = by - ay, len2 = ex * ex + ey * ey;
        int i0 = (int)floor((fmin(ax, bx) - r - x0) / dx);
        int i1 = (int)ceil((fmax(ax, bx) + r - x0) / dx);
        int j0 = (int)floor((fmin(ay, by) - r - y0) / dx);
        int j1 = (int)ceil((fmax(ay, by) + r - y0) / dx);
        if (i0 < 0 || j0 < 0 || i1 >= nx || j1 >= ny)
            return -1;
        long count = 0;
        for (int pass = 0; pass < 2; pass++) {
            double h = pass ? dv[i] / (count * dx * dx) : 0.0;
            for (int j = j0; j <= j1; j++) {
                double cy = y0 + (j + 0.5) * dx;
                for (int ii = i0; ii <= i1; ii++) {
                    double cx = x0 + (ii + 0.5) * dx;
                    double u = len2 > 0.0
                        ? ((cx - ax) * ex + (cy - ay) * ey) / len2 : 0.0;
                    u = u < 0.0 ? 0.0 : (u > 1.0 ? 1.0 : u);
                    double qx = ax + u * ex - cx, qy = ay + u * ey - cy;
                    if (qx * qx + qy * qy > r * r)
                        continue;
                    if (pass)
                        grid[(long)j * nx + ii] += h;
                    else
                        count++;
                }
            }
            if (!pass && !count) {
                int ci = (int)floor((0.5 * (ax + bx) - x0) / dx);
                int cj = (int)floor((0.5 * (ay + by) - y0) / dx);
                grid[(long)cj * nx + ci] += dv[i] / (dx * dx);
                break;
            }
        }
    }
    return 0;
}
