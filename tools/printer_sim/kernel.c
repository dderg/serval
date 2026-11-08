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
