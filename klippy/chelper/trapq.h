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

// Plan 8 Chunk 2 — variable-length phases[] upgrade. Every move is a quintic
// polynomial payload: up to MOVE_MAX_PIECES phases, each carrying a per-axis
// position-in-t polynomial with up to 15 coefficients. Null-motion moves
// (trapq sentinels, time-gap fills, itersolve stack stubs) are a memset-zeroed
// struct move with start_pos set and n_phases == 0 — move_get_coord detects
// the n_phases == 0 sentinel and returns start_pos.

// Per-phase position polynomial: x(t) = sum_k c_k * (t - t_phase_start)^k,
// evaluated in a phase-local time coordinate. 15 coeffs per axis — Chunk 2
// widens the slot to make room for Chunk 3's bs-shaped composed moves where
// the effective polynomial degree exceeds the legacy 10. Existing consumers
// (linear-as-quintic, QuinticShape.compose_phase_polynomials) populate only
// c[0..10] and leave c[11..14] = 0; the 11-moment integrator in integrate.c
// still truncates at SMOOTHER_NUM_MOMENTS = 11 per Chunk 2 §3.5.
#define MOVE_QUINTIC_POLY_COEFFS 15
#define MOVE_MAX_PIECES 32

struct move_quintic_phase {
    double t_end;                 /* phase end (relative to move start) */
    struct coord c[MOVE_QUINTIC_POLY_COEFFS];
};

struct move {
    double print_time, move_t;
    struct coord start_pos;
    double arc_length;
    double v_cap_min;             /* Option Z upstream junction cap */
    int n_phases;                 /* 0 = null/sentinel move; 1..MOVE_MAX_PIECES otherwise */
    struct move_quintic_phase phases[MOVE_MAX_PIECES];
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
// Direct-quintic trapq emit. n_phases phases in [1, MOVE_MAX_PIECES], each a
// degree-up-to-14 position-in-t polynomial in phase-local time. phase_t_ends
// gives the absolute move-local t_end of each phase (monotonic, last equals
// move_t). coeff_buf layout:
//   per phase: MOVE_QUINTIC_POLY_COEFFS * 3 doubles  (c[0].x, c[0].y, c[0].z,
//                                                     c[1].x, c[1].y, c[1].z,
//                                                     ..., c[14].z)
// so coeff_buf is n_phases * 15 * 3 doubles. Zero-length phases are allowed
// (phase_t_ends[i] == phase_t_ends[i-1] collapses phase i).
void trapq_append_quintic(struct trapq *tq, double print_time
                          , int n_phases, const double *phase_t_ends
                          , double move_t, double arc_length, double v_cap_min
                          , double start_pos_x, double start_pos_y
                          , double start_pos_z, const double *coeff_buf);

void trapq_finalize_moves(struct trapq *tq, double print_time
                          , double clear_history_time);
void trapq_set_position(struct trapq *tq, double print_time
                        , double pos_x, double pos_y, double pos_z);
int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
                      , double start_time, double end_time);

#endif // trapq.h
