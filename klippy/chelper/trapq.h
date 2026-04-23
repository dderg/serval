#ifndef TRAPQ_H
#define TRAPQ_H

#include "list.h" // list_node

struct coord {
    union {
        struct {
            double x, y, z;
        };
        double axis[3];
    };
};

// Plan 8 Chunk 1 Task 8 — flattened struct move. Every move is a quintic
// polynomial payload: three phases (accel/cruise/decel), each carrying a
// per-axis position-in-t polynomial with up to 11 coefficients. Null-motion
// moves (trapq sentinels, time-gap fills, itersolve stack stubs) are a
// memset-zeroed struct move with start_pos set — move_get_coord recognizes
// the all-zero phase t_ends and returns start_pos.

// Per-phase position polynomial: x(t) = sum_k c_k * (t - t_phase_start)^k,
// evaluated in a phase-local time coordinate. 11 coeffs per axis (degree 10
// for accel/decel where quintic ∘ degree-2 s(t) gives degree 10, c_6..c_10 = 0
// for cruise where quintic ∘ degree-1 s(t) gives degree 5).
#define MOVE_QUINTIC_POLY_COEFFS 11

struct move_quintic_phase {
    double t_end;                 /* phase end (relative to move start) */
    struct coord c[MOVE_QUINTIC_POLY_COEFFS];
};

struct move {
    double print_time, move_t;
    struct coord start_pos;
    double arc_length;
    struct move_quintic_phase accel, cruise, decel;
    double v_cap_min;             /* Option Z upstream junction cap */
    struct list_node node;
};

struct trapq {
    struct list_head moves, history;
};

struct pull_move {
    double print_time, move_t;
    double start_v, accel;
    double start_x, start_y, start_z;
    double x_r, y_r, z_r;
};

struct move *move_alloc(void);
double move_get_distance(const struct move *m, double move_time);
struct coord move_get_coord(const struct move *m, double move_time);
struct trapq *trapq_alloc(void);
void trapq_free(struct trapq *tq);
void trapq_check_sentinels(struct trapq *tq);
void trapq_add_move(struct trapq *tq, struct move *m);
void trapq_append(struct trapq *tq, double print_time
                  , double accel_t, double cruise_t, double decel_t
                  , double start_pos_x, double start_pos_y, double start_pos_z
                  , double axes_r_x, double axes_r_y, double axes_r_z
                  , double start_v, double cruise_v, double accel);

// Plan 5 D2b — direct-quintic trapq emit. Three phases, each a degree-10
// position-in-t polynomial in phase-local time. coeff_buf layout:
//   per phase: MOVE_QUINTIC_POLY_COEFFS * 3 doubles  (c[0].x, c[0].y, c[0].z,
//                                                     c[1].x, c[1].y, c[1].z,
//                                                     ..., c[10].z)
// so coeff_buf is 3 phases * 11 * 3 = 99 doubles. t_accel_end / t_decel_start
// / move_t are the phase boundaries in absolute move-local time. Zero-length
// phases are allowed (t_accel_end==0 collapses the accel phase, etc).
void trapq_append_quintic(struct trapq *tq, double print_time
                          , double t_accel_end, double t_decel_start
                          , double move_t, double arc_length, double v_cap_min
                          , double start_pos_x, double start_pos_y
                          , double start_pos_z, const double coeff_buf[]);

void trapq_finalize_moves(struct trapq *tq, double print_time
                          , double clear_history_time);
void trapq_set_position(struct trapq *tq, double print_time
                        , double pos_x, double pos_y, double pos_z);
int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
                      , double start_time, double end_time);

#endif // trapq.h
