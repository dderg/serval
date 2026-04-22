#ifndef INTEGRATE_H
#define INTEGRATE_H

// Plan 5 Magnum Opus foundation: piecewise-polynomial smoother kernel with
// 11 antiderivative moments. Linear-move consumers use only moments 0..2
// (bit-identical with the pre-Plan-5 single-polynomial path); the degree-5
// kernel pieces and 11-moment space are sized for the future direct-quintic
// fused-kernel path (D2a / D3).

#define SMOOTHER_MAX_PIECES 9
#define SMOOTHER_MAX_DEGREE 5
#define SMOOTHER_NUM_MOMENTS 11

typedef struct {
    // Antiderivatives m_k = int_{support_start}^{t} tau^k * w(tau) dtau for
    // k = 0..10. Linear-move integrate_move uses m[0..2]; the higher moments
    // are reserved for the direct-quintic position-in-t polynomial dispatch
    // that Task 9 (D2a/D3) will add. Until then they stay zero for any
    // piecewise kernel with degree <= 5 convolved against linear moves.
    double m[SMOOTHER_NUM_MOMENTS];
} smoother_antiderivatives;

struct smoother_piece {
    // Ascending power-basis coefficients: w(tau) = sum_k coeffs[k] * tau^k.
    double coeffs[SMOOTHER_MAX_DEGREE + 1];
    double t_start, t_end;
    // Cached antiderivative endpoints: m_start accumulates moments from the
    // kernel-support start up to this piece's t_start; m_end up to t_end.
    // Cached at init time to avoid re-summing prior pieces for every query.
    smoother_antiderivatives m_start, m_end;
};

struct smoother {
    int n_pieces;
    struct smoother_piece pieces[SMOOTHER_MAX_PIECES];
    double hst;        // half-support: 0.5 * t_sm. 0 disables the smoother.
    double t_offs;     // centroid shift along t; computed at init.
    // Cached endpoint moments over the full support (support_start ->
    // support_end) for the fast path in range_integrate when the move
    // is fully contained in [support_start, support_end].
    smoother_antiderivatives m_hst, p_hst, pm_diff;
};

struct move;

// Piecewise smoother init. `piece_buf` layout: n_pieces * 8 doubles, per
// piece [t_start, t_end, c_0, c_1, c_2, c_3, c_4, c_5]. n_pieces == 0
// disables the smoother (identity / none).
int init_smoother(int n_pieces, const double piece_buf[], double t_sm,
                  struct smoother* sm);

// Evaluate the kernel at t. Returns 0 outside support.
double smoother_eval(const struct smoother* sm, double t);

double integrate_move(const struct move* m, int axis, double base, double t0
                      , const smoother_antiderivatives* s);
double integrate_velocity(const struct move* m, int axis, double t0
                          , const smoother_antiderivatives* s);

smoother_antiderivatives
calc_antiderivatives(const struct smoother* sm, double t);
smoother_antiderivatives
diff_antiderivatives(const smoother_antiderivatives* ad1
                     , const smoother_antiderivatives* ad2);

#endif // integrate.h
