// Trapezoidal velocity movement queue
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026       Magnum Opus foundation (Plan 5 D2b)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt, fmin
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // unlikely, __visible
#include "linear_quintic.h" // build_linear_as_quintic_coeffs
#include "trapq.h" // move_get_coord

// Allocate a new 'move' object. memset(0) produces a valid MOVE_LINEAR with
// zero velocity/accel/axes_r — the MOVE_LINEAR = 0 invariant (see trapq.h)
// guarantees downstream dispatch treats this as a linear zero-motion stub.
struct move *
move_alloc(void)
{
    struct move *m = malloc(sizeof(*m));
    memset(m, 0, sizeof(*m));
    return m;
}

// Evaluate one phase of a quintic move at phase-local time delta via Horner.
// Per-axis position polynomial x(t) = sum_k c_k * delta_t^k, delta_t = t - t_ps.
static inline struct coord
quintic_phase_eval(const struct move_quintic_phase *ph, double delta_t)
{
    // Horner form, descending powers. c[10] is the top coefficient.
    struct coord r = ph->c[MOVE_QUINTIC_POLY_COEFFS - 1];
    int k;
    for (k = MOVE_QUINTIC_POLY_COEFFS - 2; k >= 0; --k) {
        r.x = r.x * delta_t + ph->c[k].x;
        r.y = r.y * delta_t + ph->c[k].y;
        r.z = r.z * delta_t + ph->c[k].z;
    }
    return r;
}

// Return the quintic phase containing move_time, and the phase-local time.
// Note: each phase stores its absolute-move-local t_end; the phase's local
// origin is the previous phase's t_end (or 0 for accel).
static inline const struct move_quintic_phase *
quintic_pick_phase(const struct move *m, double move_time, double *out_delta)
{
    const struct move_quintic_phase *ph = &m->u.quintic.accel;
    double phase_start = 0.0;
    if (move_time > m->u.quintic.accel.t_end) {
        phase_start = m->u.quintic.accel.t_end;
        ph = &m->u.quintic.cruise;
        if (move_time > m->u.quintic.cruise.t_end) {
            phase_start = m->u.quintic.cruise.t_end;
            ph = &m->u.quintic.decel;
        }
    }
    *out_delta = move_time - phase_start;
    return ph;
}

// Return the distance moved given a time in a move. For quintic moves the
// "distance" is a scalar along the curve arc length — not well-defined as a
// single number without the parameter, so we return the projected distance
// along the axis-sum magnitude at move_time. This entry point is used by
// consumers that want a scalar distance (kin_shaper's get_axis_position and
// kin_extruder's pressure-advance path). For quintic we synthesize this as
// the scalar sqrt of the position delta from start_pos — cheap and correct
// for straight-segment cases, approximate for curved blends; blend consumers
// go through move_get_coord directly.
inline double
move_get_distance(const struct move *m, double move_time)
{
    if (likely(m->kind == MOVE_LINEAR))
        return (m->u.lin.start_v + m->u.lin.half_accel * move_time) * move_time;
    // MOVE_QUINTIC_POLY_T: compute the chord distance from start_pos to the
    // evaluated position — not the true arc-length, but a monotonic scalar
    // that callers treating this as a "progress" measure can consume. Pure
    // quintic consumers should use move_get_coord instead.
    struct coord p = move_get_coord(m, move_time);
    double dx = p.x - m->start_pos.x;
    double dy = p.y - m->start_pos.y;
    double dz = p.z - m->start_pos.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Return the XYZ coordinates given a time in a move. Dispatches on kind:
// MOVE_LINEAR evaluates start_pos + axes_r * (start_v*t + half_accel*t^2);
// MOVE_QUINTIC_POLY_T picks the phase and evaluates the per-axis polynomial
// in phase-local time via Horner.
inline struct coord
move_get_coord(const struct move *m, double move_time)
{
    if (likely(m->kind == MOVE_LINEAR)) {
        double move_dist = (m->u.lin.start_v + m->u.lin.half_accel * move_time)
                           * move_time;
        return (struct coord) {
            .x = m->start_pos.x + m->u.lin.axes_r.x * move_dist,
            .y = m->start_pos.y + m->u.lin.axes_r.y * move_dist,
            .z = m->start_pos.z + m->u.lin.axes_r.z * move_dist };
    }
    // MOVE_QUINTIC_POLY_T: phase-local polynomial. c[0] is position at phase
    // start, so no explicit start_pos add here — the per-phase polynomial
    // already carries the absolute position via its c[0] term (chosen at
    // emit-time by compose_phase_polynomials).
    double delta_t;
    const struct move_quintic_phase *ph = quintic_pick_phase(m, move_time,
                                                             &delta_t);
    return quintic_phase_eval(ph, delta_t);
}

#define NEVER_TIME 9999999999999999.9

// Allocate a new 'trapq' object
struct trapq * __visible
trapq_alloc(void)
{
    struct trapq *tq = malloc(sizeof(*tq));
    memset(tq, 0, sizeof(*tq));
    list_init(&tq->moves);
    list_init(&tq->history);
    struct move *head_sentinel = move_alloc(), *tail_sentinel = move_alloc();
    tail_sentinel->print_time = tail_sentinel->move_t = NEVER_TIME;
    list_add_head(&head_sentinel->node, &tq->moves);
    list_add_tail(&tail_sentinel->node, &tq->moves);
    return tq;
}

// Free memory associated with a 'trapq' object
void __visible
trapq_free(struct trapq *tq)
{
    while (!list_empty(&tq->moves)) {
        struct move *m = list_first_entry(&tq->moves, struct move, node);
        list_del(&m->node);
        free(m);
    }
    while (!list_empty(&tq->history)) {
        struct move *m = list_first_entry(&tq->history, struct move, node);
        list_del(&m->node);
        free(m);
    }
    free(tq);
}

// Update the list sentinels
void
trapq_check_sentinels(struct trapq *tq)
{
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    if (tail_sentinel->print_time)
        // Already up to date
        return;
    struct move *m = list_prev_entry(tail_sentinel, node);
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move,node);
    if (m == head_sentinel) {
        // No moves at all on this list
        tail_sentinel->print_time = NEVER_TIME;
        return;
    }
    tail_sentinel->print_time = m->print_time + m->move_t;
    tail_sentinel->start_pos = move_get_coord(m, m->move_t);
}

#define MAX_NULL_MOVE 1.0

// Add a move to the trapezoid velocity queue
void
trapq_add_move(struct trapq *tq, struct move *m)
{
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    struct move *prev = list_prev_entry(tail_sentinel, node);
    if (prev->print_time + prev->move_t < m->print_time) {
        // Add a null move to fill time gap (MOVE_LINEAR with zeroed lin union
        // via move_alloc's memset — the MOVE_LINEAR=0 invariant guarantees
        // this parses as a linear zero-motion stub).
        struct move *null_move = move_alloc();
        null_move->start_pos = m->start_pos;
        if (!prev->print_time && m->print_time > MAX_NULL_MOVE)
            // Limit the first null move to improve numerical stability
            null_move->print_time = m->print_time - MAX_NULL_MOVE;
        else
            null_move->print_time = prev->print_time + prev->move_t;
        null_move->move_t = m->print_time - null_move->print_time;
        list_add_before(&null_move->node, &tail_sentinel->node);
    }
    list_add_before(&m->node, &tail_sentinel->node);
    tail_sentinel->print_time = 0.;
}

// Plan 8 Chunk 1 Task 2: dispatch through the quintic path.
//
// Construct a 99-double degenerate-quintic coefficient buffer representing
// the accel/cruise/decel trapezoid and delegate to trapq_append_quintic.
// After this, no MOVE_LINEAR structs are produced by the append entry point.
void __visible
trapq_append(struct trapq *tq, double print_time
             , double accel_t, double cruise_t, double decel_t
             , double start_pos_x, double start_pos_y, double start_pos_z
             , double axes_r_x, double axes_r_y, double axes_r_z
             , double start_v, double cruise_v, double accel)
{
    double coeff_buf[99];
    build_linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r_x, axes_r_y, axes_r_z,
        start_pos_x, start_pos_y, start_pos_z,
        coeff_buf);
    double move_t = accel_t + cruise_t + decel_t;
    // Total path distance is the sum of per-phase displacements along
    // axes_r; the arc length is that distance scaled by |axes_r|.
    double accel_d = start_v * accel_t + 0.5 * accel * accel_t * accel_t;
    double cruise_d = cruise_v * cruise_t;
    double decel_d = cruise_v * decel_t - 0.5 * accel * decel_t * decel_t;
    double total_d = accel_d + cruise_d + decel_d;
    double axes_r_mag = sqrt(axes_r_x * axes_r_x + axes_r_y * axes_r_y
                             + axes_r_z * axes_r_z);
    double arc_length = total_d * axes_r_mag;
    // v_cap_min is the minimum instantaneous velocity over the trapezoid.
    // For a classical trapezoid the velocity is monotone on each phase, so
    // the extrema live at the endpoints: start_v, cruise_v, and the decel
    // end velocity (cruise_v - accel * decel_t). Clamp at 0 to guard
    // against tiny FP negatives when the planner brings the end velocity
    // to zero.
    double decel_end_v = cruise_v - accel * decel_t;
    double v_cap_min = fmin(fmin(start_v, cruise_v), decel_end_v);
    if (v_cap_min < 0.0) v_cap_min = 0.0;
    trapq_append_quintic(
        tq, print_time,
        accel_t,                  // t_accel_end
        accel_t + cruise_t,       // t_decel_start
        move_t, arc_length, v_cap_min,
        start_pos_x, start_pos_y, start_pos_z,
        coeff_buf);
}

// Plan 5 D2b — emit a single quintic trapq entry. Phase boundaries are
// absolute move-local times (relative to print_time). coeff_buf has 99
// doubles: 3 phases × 11 coeffs × 3 axes, each phase in order
//   c[0].x, c[0].y, c[0].z, c[1].x, c[1].y, c[1].z, ..., c[10].x, c[10].y,
//   c[10].z. Within each phase the polynomial is in phase-local time
//   delta_t = t_move_local - t_phase_start.
void __visible
trapq_append_quintic(struct trapq *tq, double print_time
                     , double t_accel_end, double t_decel_start
                     , double move_t, double arc_length, double v_cap_min
                     , double start_pos_x, double start_pos_y
                     , double start_pos_z, const double coeff_buf[])
{
    struct move *m = move_alloc();
    m->print_time = print_time;
    m->move_t = move_t;
    m->kind = MOVE_QUINTIC_POLY_T;
    m->start_pos.x = start_pos_x;
    m->start_pos.y = start_pos_y;
    m->start_pos.z = start_pos_z;
    m->u.quintic.arc_length = arc_length;
    m->u.quintic.v_cap_min = v_cap_min;
    m->u.quintic.accel.t_end = t_accel_end;
    m->u.quintic.cruise.t_end = t_decel_start;
    m->u.quintic.decel.t_end = move_t;
    // Unpack coeff_buf into per-phase per-axis arrays.
    struct move_quintic_phase *phases[3] = {
        &m->u.quintic.accel, &m->u.quintic.cruise, &m->u.quintic.decel,
    };
    int p, k;
    const double *src = coeff_buf;
    for (p = 0; p < 3; ++p) {
        struct move_quintic_phase *ph = phases[p];
        for (k = 0; k < MOVE_QUINTIC_POLY_COEFFS; ++k) {
            ph->c[k].x = src[0];
            ph->c[k].y = src[1];
            ph->c[k].z = src[2];
            src += 3;
        }
    }
    trapq_add_move(tq, m);
}

// Return non-zero if the move carries any motion content. For linear moves
// this is the original (start_v || half_accel) test; for quintic moves the
// arc_length is the canonical zero-motion indicator.
static inline int
move_is_nonnull(const struct move *m)
{
    if (m->kind == MOVE_LINEAR)
        return m->u.lin.start_v != 0.0 || m->u.lin.half_accel != 0.0;
    return m->u.quintic.arc_length != 0.0;
}

// Expire any moves older than `print_time` from the trapezoid velocity queue
void __visible
trapq_finalize_moves(struct trapq *tq, double print_time
                     , double clear_history_time)
{
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move,node);
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    // Move expired moves from main "moves" list to "history" list
    for (;;) {
        struct move *m = list_next_entry(head_sentinel, node);
        if (m == tail_sentinel) {
            tail_sentinel->print_time = NEVER_TIME;
            break;
        }
        if (m->print_time + m->move_t > print_time)
            break;
        list_del(&m->node);
        if (move_is_nonnull(m))
            list_add_head(&m->node, &tq->history);
        else
            free(m);
    }
    // Free old moves from history list
    if (list_empty(&tq->history))
        return;
    struct move *latest = list_first_entry(&tq->history, struct move, node);
    for (;;) {
        struct move *m = list_last_entry(&tq->history, struct move, node);
        if (m == latest || m->print_time + m->move_t > clear_history_time)
            break;
        list_del(&m->node);
        free(m);
    }
}

// Note a position change in the trapq history
void __visible
trapq_set_position(struct trapq *tq, double print_time
                   , double pos_x, double pos_y, double pos_z)
{
    // Flush all moves from trapq
    trapq_finalize_moves(tq, NEVER_TIME, 0);

    // Prune any moves in the trapq history that were interrupted
    while (!list_empty(&tq->history)) {
        struct move *m = list_first_entry(&tq->history, struct move, node);
        if (m->print_time < print_time) {
            if (m->print_time + m->move_t > print_time)
                m->move_t = print_time - m->print_time;
            break;
        }
        list_del(&m->node);
        free(m);
    }

    // Add a marker to the trapq history (MOVE_LINEAR via move_alloc's memset;
    // kind defaults to 0 = MOVE_LINEAR).
    struct move *m = move_alloc();
    m->print_time = print_time;
    m->start_pos.x = pos_x;
    m->start_pos.y = pos_y;
    m->start_pos.z = pos_z;
    list_add_head(&m->node, &tq->history);
}

// Return history of movement queue. Linear moves project to pull_move's
// (start_v, accel, start_xyz, xyz_r) fields as today. Quintic moves project
// to a degenerate linear-equivalent approximation for motion_report consumers
// that haven't yet been updated (Plan 5 D6 / Task 17 bumps the schema).
int __visible
trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
                  , double start_time, double end_time)
{
    int res = 0;
    struct move *m;
    list_for_each_entry(m, &tq->history, node) {
        if (start_time >= m->print_time + m->move_t || res >= max)
            break;
        if (end_time <= m->print_time)
            continue;
        p->print_time = m->print_time;
        p->move_t = m->move_t;
        p->kind = (int)m->kind;
        p->start_x = m->start_pos.x;
        p->start_y = m->start_pos.y;
        p->start_z = m->start_pos.z;
        if (m->kind == MOVE_LINEAR) {
            p->start_v = m->u.lin.start_v;
            p->accel = 2. * m->u.lin.half_accel;
            p->x_r = m->u.lin.axes_r.x;
            p->y_r = m->u.lin.axes_r.y;
            p->z_r = m->u.lin.axes_r.z;
        } else {
            // Quintic projected to a chord-straight linear approximation for
            // legacy consumers. Proper serialization comes in the websocket
            // schema (Task 17 adds a `kind` field; downstream consumers that
            // read pull_move.kind == 1 should ignore (start_v, accel, xyz_r)
            // since a single linear trapezoid cannot represent a quintic).
            struct coord end = move_get_coord(m, m->move_t);
            double dx = end.x - m->start_pos.x;
            double dy = end.y - m->start_pos.y;
            double dz = end.z - m->start_pos.z;
            double chord = sqrt(dx * dx + dy * dy + dz * dz);
            double inv_t = m->move_t > 0.0 ? 1.0 / m->move_t : 0.0;
            double inv_chord = chord > 0.0 ? 1.0 / chord : 0.0;
            // Approximate: average velocity along the chord.
            p->start_v = chord * inv_t;
            p->accel = 0.0;
            p->x_r = dx * inv_chord;
            p->y_r = dy * inv_chord;
            p->z_r = dz * inv_chord;
        }
        p++;
        res++;
    }
    return res;
}
