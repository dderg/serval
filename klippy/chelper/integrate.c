// Helpers to integrate the smoothing weight function
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020-2023  Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2026       Magnum Opus foundation
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "compiler.h" // unlikely
#include "integrate.h"
#include "trapq.h" // struct move

#include <string.h>

/****************************************************************
 * Piecewise smoother: antiderivative computation
 *
 * Kernel w(tau) is piecewise-polynomial, sum_{pieces} [t_s, t_e] with
 *   w(tau) = sum_{j=0..deg} coeffs[j] * tau^j  on [t_s, t_e].
 *
 * The accumulated antiderivative at t is
 *   F_k(t) = int_{support_start}^{t} tau^k * w(tau) dtau, k = 0..10.
 *
 * For piecewise kernels we find the piece containing t, then
 *   F_k(t) = piece.m_start.m[k] + int_{t_s}^{t} tau^k * w(tau) dtau.
 *
 * The integral int_{t_s}^{t} tau^k * (sum_j c_j tau^j) dtau factors as
 *   sum_j c_j * (t^(k+j+1) - t_s^(k+j+1)) / (k+j+1).
 *
 * For linear moves, only m[0], m[1], m[2] are consumed downstream.
 ****************************************************************/

// Total "cumulative power" we ever need: up to moment index 10 plus the
// top kernel degree (5), plus 1 for the integral = 17 distinct t-powers.
#define MAX_POWER_T 17

static inline void
zero_antiderivatives(smoother_antiderivatives* ad)
{
    memset(ad, 0, sizeof(*ad));
}

// Compute moments contributed by a single piece evaluated on [a, t]:
//   contrib[k] = int_{a}^{t} tau^k * (sum_j c_j tau^j) dtau
// for k = 0..SMOOTHER_NUM_MOMENTS-1. When t == piece->t_end this yields
// the full piece integral.
static inline void
piece_partial_integral(const struct smoother_piece* p, double t,
                       smoother_antiderivatives* out)
{
    zero_antiderivatives(out);
    // Precompute powers of t and t_start up to MAX_POWER_T.
    double tpow[MAX_POWER_T + 1];
    double apow[MAX_POWER_T + 1];
    tpow[0] = 1.0;
    apow[0] = 1.0;
    double a = p->t_start;
    int i;
    for (i = 1; i <= MAX_POWER_T; ++i) {
        tpow[i] = tpow[i-1] * t;
        apow[i] = apow[i-1] * a;
    }
    for (int k = 0; k < SMOOTHER_NUM_MOMENTS; ++k) {
        double acc = 0.0;
        for (int j = 0; j <= SMOOTHER_MAX_DEGREE; ++j) {
            double c = p->coeffs[j];
            if (c == 0.0) continue;
            int pw = k + j + 1;
            // (t^pw - a^pw) / pw
            acc += c * (tpow[pw] - apow[pw]) / (double)pw;
        }
        out->m[k] = acc;
    }
}

// Compute the full-piece integral contribution (endpoint moments). Faster
// than piece_partial_integral when t == t_end because we can skip the
// t-powers array.
static inline void
piece_full_integral(const struct smoother_piece* p,
                    smoother_antiderivatives* out)
{
    piece_partial_integral(p, p->t_end, out);
}

inline smoother_antiderivatives
calc_antiderivatives(const struct smoother* sm, double t)
{
    smoother_antiderivatives out;
    zero_antiderivatives(&out);
    if (unlikely(!sm->n_pieces))
        return out;
    // Linear scan: at most SMOOTHER_MAX_PIECES = 9 iterations.
    for (int i = 0; i < sm->n_pieces; ++i) {
        const struct smoother_piece* p = &sm->pieces[i];
        if (t <= p->t_start) {
            // Before this piece: use the prior-piece cumulative endpoint.
            // For the first piece this is zero (empty start state).
            return p->m_start;
        }
        if (t <= p->t_end) {
            // Inside piece i.
            smoother_antiderivatives partial;
            piece_partial_integral(p, t, &partial);
            for (int k = 0; k < SMOOTHER_NUM_MOMENTS; ++k)
                out.m[k] = p->m_start.m[k] + partial.m[k];
            return out;
        }
    }
    // Past support end: full cumulative.
    return sm->pieces[sm->n_pieces - 1].m_end;
}

inline smoother_antiderivatives
diff_antiderivatives(const smoother_antiderivatives* ad1
                     , const smoother_antiderivatives* ad2)
{
    smoother_antiderivatives out;
    for (int k = 0; k < SMOOTHER_NUM_MOMENTS; ++k)
        out.m[k] = ad2->m[k] - ad1->m[k];
    return out;
}

inline double
integrate_move(const struct move* m, int axis, double base, double t0
               , const smoother_antiderivatives* s)
{
    // Linear-move integrand: position(t) = start + axis_r *
    //     (start_v * t + half_accel * t^2). Moments 0..2 of the kernel
    // suffice; m[3..10] stay unused.
    double axis_r = m->axes_r.axis[axis - 'x'];
    double start_v = m->start_v * axis_r;
    double half_accel = m->half_accel * axis_r;
    // Substitute the integration variable tnew = t0 - t to simplify integrals
    double accel = 2. * half_accel;
    base += (half_accel * t0 + start_v) * t0;
    start_v += accel * t0;
    return base * s->m[0] - start_v * s->m[1] + half_accel * s->m[2];
}

inline double
integrate_velocity(const struct move* m, int axis, double t0
                   , const smoother_antiderivatives* s)
{
    double axis_r = m->axes_r.axis[axis - 'x'];
    double start_v = m->start_v * axis_r;
    double accel = 2. * m->half_accel * axis_r;
    start_v += accel * t0;
    return start_v * s->m[0] - accel * s->m[1];
}

/****************************************************************
 * Kernel sampling (direct evaluation w(t) — used by debug / tests)
 ****************************************************************/

double
smoother_eval(const struct smoother* sm, double t)
{
    if (unlikely(!sm->n_pieces))
        return 0.0;
    for (int i = 0; i < sm->n_pieces; ++i) {
        const struct smoother_piece* p = &sm->pieces[i];
        if (t >= p->t_start && t <= p->t_end) {
            // Horner-form evaluation of ascending-power coeffs.
            double v = p->coeffs[SMOOTHER_MAX_DEGREE];
            for (int k = SMOOTHER_MAX_DEGREE - 1; k >= 0; --k)
                v = v * t + p->coeffs[k];
            return v;
        }
    }
    return 0.0;
}

/****************************************************************
 * Smoother initialization
 ****************************************************************/

int
init_smoother(int n_pieces, const double piece_buf[], double t_sm,
              struct smoother* sm)
{
    if (n_pieces < 0 || n_pieces > SMOOTHER_MAX_PIECES)
        return -1;
    memset(sm, 0, sizeof(*sm));
    sm->n_pieces = n_pieces;
    sm->hst = 0.5 * t_sm;
    if (!n_pieces || t_sm <= 0.0)
        return 0;
    // Parse each piece: [t_start, t_end, c_0, c_1, c_2, c_3, c_4, c_5].
    int i, k;
    for (i = 0; i < n_pieces; ++i) {
        struct smoother_piece* p = &sm->pieces[i];
        const double* src = piece_buf + i * 8;
        p->t_start = src[0];
        p->t_end = src[1];
        for (k = 0; k <= SMOOTHER_MAX_DEGREE; ++k)
            p->coeffs[k] = src[2 + k];
    }
    // Optional normalization: rescale coeffs so the full kernel has unit
    // integral over its support. Pre-normalized piece buffers (as emitted
    // by shaper_defs.INPUT_SMOOTHERS) already satisfy this to ~1e-9; we
    // still run the normalization pass for robustness against hand-crafted
    // configs or future callers that skip normalization in Python.
    double total = 0.0;
    smoother_antiderivatives endpoint;
    zero_antiderivatives(&endpoint);
    for (i = 0; i < n_pieces; ++i) {
        smoother_antiderivatives full;
        piece_full_integral(&sm->pieces[i], &full);
        total += full.m[0];
    }
    if (total == 0.0)
        return -1;
    double inv_norm = 1.0 / total;
    for (i = 0; i < n_pieces; ++i)
        for (k = 0; k <= SMOOTHER_MAX_DEGREE; ++k)
            sm->pieces[i].coeffs[k] *= inv_norm;
    // Precompute per-piece cumulative endpoints.
    zero_antiderivatives(&endpoint);
    for (i = 0; i < n_pieces; ++i) {
        struct smoother_piece* p = &sm->pieces[i];
        p->m_start = endpoint;
        smoother_antiderivatives full;
        piece_full_integral(p, &full);
        for (k = 0; k < SMOOTHER_NUM_MOMENTS; ++k)
            endpoint.m[k] += full.m[k];
        p->m_end = endpoint;
    }
    // Cached full-support endpoints (used by range_integrate fast path).
    sm->m_hst = sm->pieces[0].m_start;
    sm->p_hst = sm->pieces[n_pieces - 1].m_end;
    sm->pm_diff = diff_antiderivatives(&sm->m_hst, &sm->p_hst);
    // Centroid shift t_offs = <tau> = M_1 / M_0. For a normalized kernel
    // M_0 = 1, so t_offs = pm_diff.m[1].
    sm->t_offs = sm->pm_diff.m[1];
    return 0;
}
