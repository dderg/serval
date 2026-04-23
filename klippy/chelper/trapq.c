// Trapezoidal velocity movement queue
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026       Magnum Opus foundation (Plan 5 D2b)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // unlikely, __visible
#include "trapq.h" // move_get_coord

// Allocate a new 'move' object. memset(0) produces a null-motion quintic
// move: all phase t_ends are 0, all polynomial coeffs zero. move_get_coord
// detects this via the all-zero t_end check and returns start_pos, which
// is the degenerate behaviour every caller of a zeroed struct move wants
// (trapq sentinels, time-gap fills, itersolve stack stubs).
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
// origin is the previous phase's t_end (or 0 for the first phase).
// Linear scan over phases[0..n_phases-1]. If move_time exceeds the last
// phase's t_end (numerical rounding past move_t), the last phase is
// returned with a delta relative to its own start.
static inline const struct move_quintic_phase *
quintic_pick_phase(const struct move *m, double move_time, double *out_delta)
{
    int n = m->n_phases;
    double phase_start = 0.0;
    int i;
    for (i = 0; i < n - 1; ++i) {
        if (move_time <= m->phases[i].t_end) {
            *out_delta = move_time - phase_start;
            return &m->phases[i];
        }
        phase_start = m->phases[i].t_end;
    }
    // Fall-through: last phase (or only phase).
    *out_delta = move_time - phase_start;
    return &m->phases[n - 1];
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
    // Chord distance from start_pos to the evaluated position — not the true
    // arc-length, but a monotonic scalar consumers treating this as a
    // "progress" measure can use. Pure quintic consumers should use
    // move_get_coord instead. For degenerate-quintic straight lines, chord
    // equals arc-length.
    struct coord p = move_get_coord(m, move_time);
    double dx = p.x - m->start_pos.x;
    double dy = p.y - m->start_pos.y;
    double dz = p.z - m->start_pos.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Return the XYZ coordinates given a time in a move. Picks the phase and
// evaluates the per-axis polynomial in phase-local time via Horner.
// c[0] carries absolute position (chosen at emit-time by
// compose_phase_polynomials), so no explicit start_pos add here.
inline struct coord
move_get_coord(const struct move *m, double move_time)
{
    // Null-move fallback: a memset-zeroed struct move (trapq sentinels,
    // time-gap fills, itersolve stack stubs) has n_phases == 0.
    // Return start_pos so zero-motion moves project to their anchor point
    // regardless of move_time.
    if (m->n_phases == 0)
        return m->start_pos;
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
        // Null-move time-gap fill. move_alloc's memset zeroes all phase
        // t_ends, which move_get_coord detects and returns start_pos.
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

// Plan 8 Chunk 2 — emit a single quintic trapq entry with variable-length
// phase layout. Phase boundaries are absolute move-local times (relative to
// print_time): phase_t_ends[i] is the t_end of phase i, monotonic
// non-decreasing, with phase_t_ends[n_phases-1] == move_t. coeff_buf is
// n_phases * MOVE_QUINTIC_POLY_COEFFS * 3 doubles, each phase in order
//   c[0].x, c[0].y, c[0].z, c[1].x, c[1].y, c[1].z, ..., c[14].x, c[14].y,
//   c[14].z. Within each phase the polynomial is in phase-local time
//   delta_t = t_move_local - t_phase_start.
void __visible
trapq_append_quintic(struct trapq *tq, double print_time
                     , int n_phases, const double *phase_t_ends
                     , double move_t, double arc_length, double v_cap_min
                     , double start_pos_x, double start_pos_y
                     , double start_pos_z, const double *coeff_buf)
{
    struct move *m = move_alloc();
    m->print_time = print_time;
    m->move_t = move_t;
    m->start_pos.x = start_pos_x;
    m->start_pos.y = start_pos_y;
    m->start_pos.z = start_pos_z;
    m->arc_length = arc_length;
    m->v_cap_min = v_cap_min;
    if (n_phases < 0)
        n_phases = 0;
    if (n_phases > MOVE_MAX_PIECES)
        n_phases = MOVE_MAX_PIECES;
    m->n_phases = n_phases;
    int p, k;
    const double *src = coeff_buf;
    for (p = 0; p < n_phases; ++p) {
        struct move_quintic_phase *ph = &m->phases[p];
        ph->t_end = phase_t_ends[p];
        for (k = 0; k < MOVE_QUINTIC_POLY_COEFFS; ++k) {
            ph->c[k].x = src[0];
            ph->c[k].y = src[1];
            ph->c[k].z = src[2];
            src += 3;
        }
    }
    trapq_add_move(tq, m);
}

// Return non-zero if the move carries any motion content. arc_length is
// the canonical zero-motion indicator for quintic moves.
static inline int
move_is_nonnull(const struct move *m)
{
    return m->arc_length != 0.0;
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

    // Add a marker to the trapq history. move_alloc's memset gives a null
    // quintic move (all phase t_ends zero); start_pos is set below and
    // move_get_coord will return it for any move_time.
    struct move *m = move_alloc();
    m->print_time = print_time;
    m->start_pos.x = pos_x;
    m->start_pos.y = pos_y;
    m->start_pos.z = pos_z;
    list_add_head(&m->node, &tq->history);
}

// Return history of movement queue. All moves are quintic; project to a
// chord-straight linear approximation for legacy motion_report consumers
// (average velocity along the chord, zero accel). Proper quintic
// serialization comes with the websocket schema bump (Plan 5 D6 / Task 17).
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
        p->start_x = m->start_pos.x;
        p->start_y = m->start_pos.y;
        p->start_z = m->start_pos.z;
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
        p++;
        res++;
    }
    return res;
}
