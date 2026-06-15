//! Clarabel SOCP construction + solve. INTERNAL — Clarabel types do not
//! escape this module.
//!
//! Clarabel solves: `Ax + s = b`, `s ∈ K`
//!
//! The kalico `ConstraintBundle` uses: `A_k * x + b_rhs ∈ K`.
//! These are equal when `A_clarabel = -A_k` and `b_clarabel = b_rhs`:
//!   `s = b_rhs - (-A_k)*x = A_k*x + b_rhs` ∈ K  ✓
//!
//! This negation is applied uniformly to every row of A regardless of cone type.
//!
//! # Cone mapping (kalico → Clarabel 0.11)
//!
//! | kalico `Cone`            | Clarabel `SupportedConeT`  |
//! |--------------------------|---------------------------|
//! | `Zero`                   | `ZeroConeT(dim)`           |
//! | `Nonneg`                 | `NonnegativeConeT(dim)`    |
//! | `SecondOrder`            | `SecondOrderConeT(dim)`    |
//! | `RotatedSecondOrder`     | (not emitted by `build()`) |
//!
//! `RotatedSecondOrderConeT` does not exist in Clarabel 0.11. `constraints::build_chain()`
//! never emits `Cone::RotatedSecondOrder`; jerk constraints use the norm-form
//! identity `z² ≤ u·v ↔ ||(2z, u-v)|| ≤ u+v` (standard SOC). The variant exists
//! for exhaustiveness but `solve()` returns `SolverSetupError` if a bundle contains it.
//!
//! # Clarabel `SolverStatus` → kalico `SolverStatus`
//!
//! | Clarabel                         | kalico                             |
//! |----------------------------------|------------------------------------|
//! | `Solved`                         | `SolverStatus::Solved`             |
//! | `AlmostSolved`                   | `SolverStatus::SolvedInexact{..}`  |
//! | `PrimalInfeasible`               | `SolverStatus::Infeasible`         |
//! | `DualInfeasible`                 | `SolverStatus::Infeasible`         |
//! | `AlmostPrimalInfeasible`         | `SolverStatus::Infeasible`         |
//! | `AlmostDualInfeasible`           | `SolverStatus::Infeasible`         |
//! | `MaxIterations`                  | `SolverStatus::MaxIter{..}`        |
//! | `MaxTime`                        | `SolverStatus::MaxIter{..}`        |
//! | `NumericalError`                 | `SolverStatus::Infeasible`         |
//! | `InsufficientProgress`           | `SolverStatus::MaxIter{..}`        |
//! | `CallbackTerminated`             | `SolverStatus::Infeasible`         |
//! | `Unsolved`                       | `SolverStatus::Infeasible`         |

// clippy::doc_markdown fires on unicode-math and CamelCase names in docs here.
#![allow(clippy::doc_markdown)]

use clarabel::algebra::CscMatrix;
use clarabel::solver::{
    DefaultSettings, DefaultSolver, IPSolver, SolverStatus as ClarabelStatus,
    SupportedConeT::{NonnegativeConeT, SecondOrderConeT, ZeroConeT},
};

use crate::topp::constraints::{Cone, ConstraintBundle};
use crate::topp::scaling::SolverScale;

// ---------------------------------------------------------------------------
// Per-thread Clarabel call counters — always compiled, near-zero overhead.
// One Cell<u32> increment per Clarabel call (~1 ns); negligible versus the
// SOCP solve time (~1-50 ms).  Integration tests read these via the public
// `counters` module; production code ignores the counts.
// ---------------------------------------------------------------------------
use std::cell::Cell;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Snapshot of Clarabel invocation counts for one top-level schedule call.
/// Reset with `counters::reset()` before a solve; read with
/// `counters::snapshot()` after.
#[derive(Debug, Clone, Copy, Default)]
pub struct SolveCounters {
    /// Total `DefaultSolver::new` + `.solve()` invocations.
    pub clarabel_calls_total: u32,
    /// Calls with no trust region: base SOCP solve + path-jerk SLP outer iters.
    pub clarabel_calls_path_jerk: u32,
    /// SLP9 calls with a trust region box (inner backtrack probes).
    pub clarabel_calls_slp9_tr: u32,
    /// SLP9 calls without a trust region (no-TR fallback after failed backtrack).
    pub clarabel_calls_slp9_no_tr: u32,
    /// 1 when `run_slp9_loop` triggers the uniform-damp restoration path.
    pub slp9_restoration_fired: u32,
    /// 1 when `ToleranceMode::Auto` fires the tight (1e-8) second pass.
    pub auto_second_pass_fired: u32,
    /// Number of `schedule_chain_*` invocations (one per chain solve, including
    /// the velocity-clamp re-solves inside the parallel joining loop).
    pub chains_scheduled: u32,
    /// Sum of `ChainGrid::n_points()` over every `schedule_chain_*` invocation.
    /// This is the total number of discretization nodes the solver touched.
    pub grid_points_scheduled: u64,
}

thread_local! {
    static COUNTERS: Cell<SolveCounters> = const { Cell::new(SolveCounters {
        clarabel_calls_total: 0,
        clarabel_calls_path_jerk: 0,
        clarabel_calls_slp9_tr: 0,
        clarabel_calls_slp9_no_tr: 0,
        slp9_restoration_fired: 0,
        auto_second_pass_fired: 0,
        chains_scheduled: 0,
        grid_points_scheduled: 0,
    })};
}

/// Public counter accessors.  Integration tests and benchmarks call
/// `counters::reset()` before a solve and `counters::snapshot()` after.
pub mod counters {
    use super::{
        COUNTERS, Cell, G_CHAINS_SCHEDULED, G_CLARABEL_PATH_JERK, G_CLARABEL_SLP9_NO_TR,
        G_CLARABEL_SLP9_TR, G_CLARABEL_TOTAL, G_GRID_POINTS_SCHEDULED, Ordering, SolveCounters,
    };

    pub fn reset() {
        COUNTERS.with(|c| c.set(SolveCounters::default()));
        G_CLARABEL_TOTAL.store(0, Ordering::Relaxed);
        G_CLARABEL_PATH_JERK.store(0, Ordering::Relaxed);
        G_CLARABEL_SLP9_TR.store(0, Ordering::Relaxed);
        G_CLARABEL_SLP9_NO_TR.store(0, Ordering::Relaxed);
        G_CHAINS_SCHEDULED.store(0, Ordering::Relaxed);
        G_GRID_POINTS_SCHEDULED.store(0, Ordering::Relaxed);
    }

    pub fn snapshot() -> SolveCounters {
        COUNTERS.with(Cell::get)
    }

    /// Process-global aggregate across every worker thread that ran a chain
    /// since the last [`reset`]. Use this — not [`snapshot`] — when the solve
    /// fans out across threads (it always does via `multi::parallel`).
    pub fn snapshot_global() -> SolveCounters {
        let thread_local = COUNTERS.with(Cell::get);
        SolveCounters {
            clarabel_calls_total: G_CLARABEL_TOTAL.load(Ordering::Relaxed),
            clarabel_calls_path_jerk: G_CLARABEL_PATH_JERK.load(Ordering::Relaxed),
            clarabel_calls_slp9_tr: G_CLARABEL_SLP9_TR.load(Ordering::Relaxed),
            clarabel_calls_slp9_no_tr: G_CLARABEL_SLP9_NO_TR.load(Ordering::Relaxed),
            chains_scheduled: G_CHAINS_SCHEDULED.load(Ordering::Relaxed),
            grid_points_scheduled: G_GRID_POINTS_SCHEDULED.load(Ordering::Relaxed),
            slp9_restoration_fired: thread_local.slp9_restoration_fired,
            auto_second_pass_fired: thread_local.auto_second_pass_fired,
        }
    }

    pub(super) fn inc_clarabel(has_tr: bool) {
        G_CLARABEL_TOTAL.fetch_add(1, Ordering::Relaxed);
        COUNTERS.with(|c| {
            let mut s = c.get();
            s.clarabel_calls_total += 1;
            let in_slp9 = super::IN_SLP9_PHASE.with(Cell::get);
            if in_slp9 {
                if has_tr {
                    s.clarabel_calls_slp9_tr += 1;
                    G_CLARABEL_SLP9_TR.fetch_add(1, Ordering::Relaxed);
                } else {
                    s.clarabel_calls_slp9_no_tr += 1;
                    G_CLARABEL_SLP9_NO_TR.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                s.clarabel_calls_path_jerk += 1;
                G_CLARABEL_PATH_JERK.fetch_add(1, Ordering::Relaxed);
            }
            c.set(s);
        });
    }

    pub(super) fn mark_restoration() {
        COUNTERS.with(|c| {
            let mut s = c.get();
            s.slp9_restoration_fired = 1;
            c.set(s);
        });
    }

    pub fn mark_auto_second_pass() {
        COUNTERS.with(|c| {
            let mut s = c.get();
            s.auto_second_pass_fired = 1;
            c.set(s);
        });
    }

    pub(crate) fn inc_chain_schedule(grid_points: usize) {
        G_CHAINS_SCHEDULED.fetch_add(1, Ordering::Relaxed);
        G_GRID_POINTS_SCHEDULED.fetch_add(grid_points as u64, Ordering::Relaxed);
        COUNTERS.with(|c| {
            let mut s = c.get();
            s.chains_scheduled += 1;
            s.grid_points_scheduled += grid_points as u64;
            c.set(s);
        });
    }
}

// Sentinel: true while execution is inside an SLP9 outer loop.
// Process-global mirror of the per-call counters. The schedule fans out across
// worker threads (`thread::scope` in `multi::parallel`), so the thread-local
// `COUNTERS` only ever see the work done on the thread that happened to run a
// given chain. The globals aggregate across all worker threads so a caller on
// the orchestrating thread can read total per-append solver WORK. Reset before
// a top-level solve, snapshot after; cross-thread visibility is the point.
static G_CLARABEL_TOTAL: AtomicU32 = AtomicU32::new(0);
static G_CLARABEL_PATH_JERK: AtomicU32 = AtomicU32::new(0);
static G_CLARABEL_SLP9_TR: AtomicU32 = AtomicU32::new(0);
static G_CLARABEL_SLP9_NO_TR: AtomicU32 = AtomicU32::new(0);
static G_CHAINS_SCHEDULED: AtomicU32 = AtomicU32::new(0);
static G_GRID_POINTS_SCHEDULED: AtomicU64 = AtomicU64::new(0);

thread_local! {
    static IN_SLP9_PHASE: Cell<bool> = const { Cell::new(false) };
}

// RAII guard that sets `IN_SLP9_PHASE = true` on construction and restores
// the previous value on drop — safe even if `run_slp9_loop` nests.
struct Slp9PhaseGuard {
    prev: bool,
}

impl Slp9PhaseGuard {
    fn enter() -> Self {
        let prev = IN_SLP9_PHASE.with(Cell::get);
        IN_SLP9_PHASE.with(|c| c.set(true));
        Self { prev }
    }
}

impl Drop for Slp9PhaseGuard {
    fn drop(&mut self) {
        IN_SLP9_PHASE.with(|c| c.set(self.prev));
    }
}

#[derive(Debug, Clone)]
pub(crate) enum SlpCut {
    /// Weight-based path-jerk cut. Rows scaled by `h̄²` to stay O(1).
    PathJerkWeights {
        i: usize,
        b_bar: f64,
        j_path: f64,
        idx: [usize; 3],
        w: [f64; 3],
        h_bar: f64,
    },
    AxisJerk(AxisJerkCut),
    Follower(crate::topp::follower::FollowerCut),
    FollowerWindowed(crate::topp::follower::WindowedCut),
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct AxisJerkCut {
    /// Grid index this cut is anchored at. The anchor is `idx[anchor_pos]`.
    pub i: usize,
    #[allow(dead_code)]
    pub axis: usize,
    /// Column indices for the three stencil b-variables (from `stencil::stencil_at`).
    pub idx: [usize; 3],
    /// b″ weights matching `idx` order (from `stencil::b_dd_weights(hl, hr)`).
    pub w: [f64; 3],
    /// Iterate `b̄` values in stencil order `[b̄_{idx[0]}, b̄_{idx[1]}, b̄_{idx[2]}]`.
    pub b_bars: [f64; 3],
    pub a_bar_i: f64,
    pub cp: f64,
    pub cpp: f64,
    pub cppp: f64,
    pub j_lim_inflated: f64,
}

/// Gradient of `j_axis` at iterate `(b̄, ā)` w.r.t. stencil b-values and `a_i`.
/// Used by the cut appender and exposed for numerical identity tests.
pub struct AxisJerkGradient {
    /// Gradient w.r.t. `b̄` in stencil order `[∂/∂b̄_{idx[0]}, ..., ∂/∂b̄_{idx[2]}]`.
    pub b: [f64; 3],
    /// Gradient w.r.t. `ā_i`.
    pub a: f64,
}

/// Test-support export: computes the linearization coefficients (= gradient of
/// `j_axis`) for an interior (anchor_pos=1) stencil with the given non-uniform
/// spacings and `b_floor = 0`.
pub fn axis_jerk_gradient_for_test(
    b_bars: &[f64; 3],
    a_bar: f64,
    cp: f64,
    cpp: f64,
    cppp: f64,
    h_intervals: &[f64; 2],
) -> AxisJerkGradient {
    let w = crate::topp::stencil::b_dd_weights(h_intervals[0], h_intervals[1]);
    let b_anchor = b_bars[1].max(0.0);
    let s = b_anchor.sqrt();
    let b_dd = w[0] * b_bars[0] + w[1] * b_bars[1] + w[2] * b_bars[2];
    let anchor_coeff = 1.5 * cppp * s
        + 3.0 * cpp * a_bar / (2.0 * s.max(f64::MIN_POSITIVE))
        + cp * (w[1] * s / 2.0 + b_dd / (4.0 * s.max(f64::MIN_POSITIVE)));
    AxisJerkGradient {
        b: [cp * s * w[0] / 2.0, anchor_coeff, cp * s * w[2] / 2.0],
        a: 3.0 * cpp * s,
    }
}

#[derive(Debug, Clone)]
pub(crate) struct SolverResult {
    pub b: Vec<f64>,
    pub a: Vec<f64>,
    pub status: SolverStatus,
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum SolverStatus {
    Solved,
    SolvedInexact { residual: f64 },
    Infeasible,
    MaxIter { residual: f64 },
}

#[derive(Debug, thiserror::Error)]
pub(crate) enum SolverSetupError {
    #[error("invalid constraint bundle: {0}")]
    InvalidBundle(String),
}

fn build_p_zero(n_vars: usize) -> CscMatrix<f64> {
    CscMatrix::<f64> {
        m: n_vars,
        n: n_vars,
        colptr: vec![0usize; n_vars + 1],
        rowval: vec![],
        nzval: vec![],
    }
}

fn map_clarabel_cones(
    bundle: &ConstraintBundle,
) -> Result<Vec<clarabel::solver::SupportedConeT<f64>>, SolverSetupError> {
    let mut out = Vec::with_capacity(bundle.cones.len());
    for &(ref cone, dim) in &bundle.cones {
        let c = match cone {
            Cone::Zero => ZeroConeT(dim),
            Cone::Nonneg => NonnegativeConeT(dim),
            Cone::SecondOrder => SecondOrderConeT(dim),
            Cone::RotatedSecondOrder => {
                return Err(SolverSetupError::InvalidBundle(
                    "RotatedSecondOrderConeT is not supported in Clarabel 0.11; \
                     constraints::build_chain() should never emit it"
                        .to_owned(),
                ));
            }
        };
        out.push(c);
    }
    Ok(out)
}

/// Exhaustive match against Clarabel 0.11.1; a new variant will fail to compile.
fn map_status(status: ClarabelStatus, residual: f64) -> SolverStatus {
    match status {
        ClarabelStatus::Solved => SolverStatus::Solved,
        ClarabelStatus::AlmostSolved => SolverStatus::SolvedInexact { residual },
        ClarabelStatus::MaxIterations
        | ClarabelStatus::MaxTime
        | ClarabelStatus::InsufficientProgress => SolverStatus::MaxIter { residual },
        ClarabelStatus::PrimalInfeasible
        | ClarabelStatus::DualInfeasible
        | ClarabelStatus::AlmostPrimalInfeasible
        | ClarabelStatus::AlmostDualInfeasible
        | ClarabelStatus::NumericalError
        | ClarabelStatus::CallbackTerminated
        | ClarabelStatus::Unsolved => SolverStatus::Infeasible,
    }
}

/// Variable layout (pinned in `constraints.rs`): `x[0..n_grid]` → `b_i`,
/// `x[n_grid..2*n_grid]` → `a_i`.
fn extract_solution(x: &[f64], n_grid: usize, status: SolverStatus) -> SolverResult {
    let b: Vec<f64> = x[..n_grid].to_vec();
    let a: Vec<f64> = x[n_grid..2 * n_grid].to_vec();
    SolverResult { b, a, status }
}

#[allow(dead_code)]
pub(crate) fn solve(bundle: &ConstraintBundle) -> Result<SolverResult, SolverSetupError> {
    solve_with_cuts(bundle, &[], 1e-8, &SolverScale::identity())
}

/// Append one per-axis Cartesian jerk SLP cut as two `Nonneg` rows.
///
/// Unified first-order Taylor linearization of `j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛`
/// at iterate `(b̄, ā)`. Uses weight-based formula (3b): `b̄″ = w·b̄` (dot over stencil triple),
/// `S = √(max(b̄_anchor, b_floor))`, `anchor_pos = idx.iter().position(|&x| x == i)`.
///
/// ```text
///   coeff on b at idx[k], k ≠ anchor_pos:   c'·S·w[k]/2
///   coeff on b at anchor:  (3/2)·c'''·S + 3·c''·ā/(2S) + c'·(w[anchor]·S/2 + b̄″/(4S))
///   coeff on a_i:          3·c''·S
///   K:  −(1/2)·c'''·S3 − (3/2)·c''·ā·S − c'·b̄″·S/4
/// ```
///
/// Identity verified (uniform w reproduces legacy 3-case formulas exactly) by
/// `tests/step9_cut_identity.rs`.
#[allow(clippy::too_many_arguments)]
fn append_axis_jerk_cut_to_clarabel(
    cut: &AxisJerkCut,
    b_floor: f64,
    n_rows: &mut usize,
    rowval: &mut [Vec<usize>],
    nzval: &mut [Vec<f64>],
    b_rhs: &mut Vec<f64>,
    n_grid: usize,
) {
    let i = cut.i;
    let cp = cut.cp;
    let cpp = cut.cpp;
    let cppp = cut.cppp;
    let j = cut.j_lim_inflated;

    // Variable layout (pinned in constraints.rs): b at 0..n_grid, a at n_grid..2*n_grid.
    let off_b = 0usize;
    let off_a = n_grid;

    let anchor_pos = cut
        .idx
        .iter()
        .position(|&x| x == i)
        .expect("cut.i must appear in cut.idx");

    let b_anchor = cut.b_bars[anchor_pos].max(b_floor);
    let s = b_anchor.sqrt();
    let s3 = b_anchor * s;
    let a_i = cut.a_bar_i;
    let b_dd = cut.w[0] * cut.b_bars[0] + cut.w[1] * cut.b_bars[1] + cut.w[2] * cut.b_bars[2];

    let s_safe = if s > 0.0 { s } else { f64::MIN_POSITIVE };
    let alpha_b_anchor = 1.5 * cppp * s
        + 3.0 * cpp * a_i / (2.0 * s_safe)
        + cp * (cut.w[anchor_pos] * s / 2.0 + b_dd / (4.0 * s_safe));
    let alpha_a_i = 3.0 * cpp * s;
    let k_const = -0.5 * cppp * s3 - 1.5 * cpp * a_i * s - cp * b_dd * s / 4.0;

    let entries_extra: [(usize, f64); 3] = {
        let mut entries = [(0usize, 0.0f64); 3];
        let mut extra_slot = 0;
        for k in 0..3 {
            if k == anchor_pos {
                continue;
            }
            let col = off_b + cut.idx[k];
            let coeff = cp * s * cut.w[k] / 2.0;
            entries[extra_slot] = (col, coeff);
            extra_slot += 1;
        }
        entries[2] = (off_a + i, alpha_a_i);
        entries
    };

    let anchor_b_col = off_b + i;

    // Row-∞-norm scaling: cp·√b/h² grows as O(N²) with grid refinement,
    // reaching 1.9e6 on fixture_4 (146-cut case) vs a-column coefficients ~10.
    // A 40 000:1 in-row spread causes QDLDL to return infeasible/maxiter on
    // every trust-region subproblem and stalls the SLP. Dividing every
    // coefficient and both RHS values by row_scale is a feasible-set-exact
    // transformation for Nonneg rows (positive scalar on a ≥ 0 constraint).
    let row_scale = entries_extra
        .iter()
        .map(|&(_, a)| a.abs())
        .fold(alpha_b_anchor.abs(), f64::max);

    if row_scale == 0.0 {
        // All coefficients are zero: vacuous constraint, skip both rows.
        return;
    }

    let alpha_b_anchor_s = alpha_b_anchor / row_scale;
    let entries_extra_s: [(usize, f64); 3] = entries_extra.map(|(col, a)| (col, a / row_scale));
    let rhs_pos = (j - k_const) / row_scale;
    let rhs_neg = (j + k_const) / row_scale;

    let pos_row = *n_rows;
    push_nz(rowval, nzval, anchor_b_col, pos_row, alpha_b_anchor_s);
    for &(col, alpha) in &entries_extra_s {
        if alpha != 0.0 {
            push_nz(rowval, nzval, col, pos_row, alpha);
        }
    }
    b_rhs.push(rhs_pos);
    *n_rows += 1;

    let neg_row = *n_rows;
    push_nz(rowval, nzval, anchor_b_col, neg_row, -alpha_b_anchor_s);
    for &(col, alpha) in &entries_extra_s {
        if alpha != 0.0 {
            push_nz(rowval, nzval, col, neg_row, -alpha);
        }
    }
    b_rhs.push(rhs_neg);
    *n_rows += 1;
}

#[inline]
fn push_nz(rowval: &mut [Vec<usize>], nzval: &mut [Vec<f64>], col: usize, row: usize, v: f64) {
    if v != 0.0 {
        rowval[col].push(row);
        nzval[col].push(v);
    }
}

/// L∞ trust region on `(b, a)` around iterate `(b̄, ā)`.
///
/// Box rows enforce
/// `b̄_i·(1−ρ_b) ≤ b_i ≤ b̄_i·(1+ρ_b)` and
/// `ā_i ± ρ_a·max(|ā_i|, A_TR_FLOOR)` for all interior grid points.
/// Boundary `b` rows are skipped — block (a) pins them exactly.
#[derive(Debug, Clone, Copy)]
pub(crate) struct TrustRegion {
    pub rho_b: f64,
    pub rho_a: f64,
}

/// Floor on |ā_i| for the a-trust-region radius: prevents near-zero iterates
/// from producing a zero-width TR that pins `a` in place. ≈ a_max.
const A_TR_FLOOR: f64 = 5_000.0;

/// Floor on `b̄_i` for the b-trust-region radius. Prevents near-zero iterates
/// from producing a near-zero TR Clarabel can't satisfy against centripetal
/// caps. (50 mm/s)² = 2500.
const B_TR_FLOOR: f64 = 2_500.0;

fn solve_with_cuts(
    bundle: &ConstraintBundle,
    cuts: &[SlpCut],
    tol: f64,
    scale: &SolverScale,
) -> Result<SolverResult, SolverSetupError> {
    solve_with_cuts_and_trust_region(bundle, cuts, None, &[], &[], tol, scale)
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn solve_with_cuts_and_trust_region(
    bundle: &ConstraintBundle,
    cuts: &[SlpCut],
    trust_region: Option<TrustRegion>,
    b_bar: &[f64],
    a_bar: &[f64],
    tol: f64,
    scale: &SolverScale,
) -> Result<SolverResult, SolverSetupError> {
    let n_vars = bundle.n_vars;
    let n_grid = bundle.n_grid;

    let mut cones_clarabel = map_clarabel_cones(bundle)?;
    let cut_rows: usize = cuts
        .iter()
        .map(|c| match c {
            SlpCut::FollowerWindowed(_) => 1,
            _ => 2,
        })
        .sum();
    if cut_rows > 0 {
        cones_clarabel.push(NonnegativeConeT(cut_rows));
    }
    // Trust-region rows: 2 per interior b_i (boundary b pinned by block (a)),
    // 2 per a_i.
    let tr_rows = if trust_region.is_some() {
        if n_grid >= 2 {
            2 * (n_grid - 2) + 2 * n_grid
        } else {
            0
        }
    } else {
        0
    };
    if tr_rows > 0 {
        cones_clarabel.push(NonnegativeConeT(tr_rows));
    }

    let mut rowval_per_col: Vec<Vec<usize>> = bundle.base_csc_rowval.clone();
    let mut nzval_per_col: Vec<Vec<f64>> = bundle.base_csc_nzval.clone();
    let mut n_rows = bundle.base_n_rows;

    #[cfg(debug_assertions)]
    {
        let mut ref_rowval: Vec<Vec<usize>> = vec![Vec::new(); n_vars];
        let mut ref_nzval: Vec<Vec<f64>> = vec![Vec::new(); n_vars];
        for (row_idx, row) in bundle.a_rows.iter().enumerate() {
            for (col, &v) in row.iter().enumerate() {
                if v != 0.0 {
                    ref_rowval[col].push(row_idx);
                    ref_nzval[col].push(-v);
                }
            }
        }
        debug_assert_eq!(rowval_per_col, ref_rowval, "base CSC rowval mismatch");
        debug_assert_eq!(nzval_per_col, ref_nzval, "base CSC nzval mismatch");
    }

    let mut b_rhs: Vec<f64> = bundle.b_rhs.clone();
    debug_assert!(
        bundle.j_path_at.iter().all(|&j| j > 0.0),
        "bundle must carry positive j_path_at"
    );
    let b_floor = scale.to_scaled_b(SLP_B_FLOOR);
    for cut in cuts {
        match cut {
            SlpCut::PathJerkWeights {
                i,
                b_bar,
                j_path: cut_j_path,
                idx,
                w,
                h_bar: cut_h_bar,
            } => {
                let b_bar_floored = b_bar.max(b_floor);
                append_path_jerk_cut_weights(
                    *i,
                    b_bar_floored,
                    *cut_j_path,
                    *cut_h_bar,
                    *idx,
                    *w,
                    &mut n_rows,
                    &mut rowval_per_col,
                    &mut nzval_per_col,
                    &mut b_rhs,
                    n_grid,
                );
            }
            SlpCut::AxisJerk(axis_cut) => {
                append_axis_jerk_cut_to_clarabel(
                    axis_cut,
                    b_floor,
                    &mut n_rows,
                    &mut rowval_per_col,
                    &mut nzval_per_col,
                    &mut b_rhs,
                    n_grid,
                );
            }
            SlpCut::Follower(fc) => {
                for sign in [1.0_f64, -1.0] {
                    let row = n_rows;
                    for &(col, g) in &fc.entries[..fc.n_entries] {
                        push_nz(&mut rowval_per_col, &mut nzval_per_col, col, row, sign * g);
                    }
                    b_rhs.push(if sign > 0.0 { fc.rhs_pos } else { fc.rhs_neg });
                    n_rows += 1;
                }
            }
            SlpCut::FollowerWindowed(wc) => {
                let row = n_rows;
                for &(col, g) in &wc.entries {
                    push_nz(&mut rowval_per_col, &mut nzval_per_col, col, row, g);
                }
                b_rhs.push(wc.rhs);
                n_rows += 1;
            }
        }
    }

    if let Some(tr) = trust_region {
        debug_assert_eq!(b_bar.len(), n_grid);
        debug_assert_eq!(a_bar.len(), n_grid);
        let b_tr_floor = scale.to_scaled_b(B_TR_FLOOR);
        let a_tr_floor = scale.to_scaled_accel(A_TR_FLOOR);
        let off_b = 0;
        for i in 1..n_grid.saturating_sub(1) {
            let bb = b_bar[i].max(0.0);
            let radius = tr.rho_b * bb.max(b_tr_floor);
            let lo = bb - radius;
            let hi = bb + radius;
            let row_lo = n_rows;
            push_nz(
                &mut rowval_per_col,
                &mut nzval_per_col,
                off_b + i,
                row_lo,
                -1.0,
            );
            b_rhs.push(-lo);
            n_rows += 1;
            let row_hi = n_rows;
            push_nz(
                &mut rowval_per_col,
                &mut nzval_per_col,
                off_b + i,
                row_hi,
                1.0,
            );
            b_rhs.push(hi);
            n_rows += 1;
        }
        let off_a = n_grid;
        for i in 0..n_grid {
            let ab = a_bar[i];
            let radius = tr.rho_a * ab.abs().max(a_tr_floor);
            let lo = ab - radius;
            let hi = ab + radius;
            let row_lo = n_rows;
            push_nz(
                &mut rowval_per_col,
                &mut nzval_per_col,
                off_a + i,
                row_lo,
                -1.0,
            );
            b_rhs.push(-lo);
            n_rows += 1;
            let row_hi = n_rows;
            push_nz(
                &mut rowval_per_col,
                &mut nzval_per_col,
                off_a + i,
                row_hi,
                1.0,
            );
            b_rhs.push(hi);
            n_rows += 1;
        }
    }

    let mut colptr: Vec<usize> = Vec::with_capacity(n_vars + 1);
    let mut rowval: Vec<usize> = Vec::new();
    let mut nzval: Vec<f64> = Vec::new();
    colptr.push(0);
    for col in 0..n_vars {
        rowval.extend_from_slice(&rowval_per_col[col]);
        nzval.extend_from_slice(&nzval_per_col[col]);
        colptr.push(nzval.len());
    }
    let a_csc = CscMatrix {
        m: n_rows,
        n: n_vars,
        colptr,
        rowval,
        nzval,
    };

    let p_zero = build_p_zero(n_vars);
    let q: &[f64] = &bundle.objective;

    // verbose=false: diagnostics via kalico telemetry.
    // max_iter=1000: SLP-cut SOCPs condition more tightly than the base SOCP;
    //   200 iters produces InsufficientProgress on the CL-2024 counterexample
    //   (a no-TR / path-jerk SLP case).
    // max_iter=200 for trust-region subproblems: TR boxes strictly shrink the
    //   feasible set so feasible TR solves converge faster; a TR subproblem
    //   hitting MaxIter is discarded by run_slp9_loop's cand_ratio check and
    //   the no-TR fallback provides an unconditional escape path. This caps
    //   wasted IPM iterations on infeasible TR probes without affecting the
    //   CL-2024 case (which is a no-TR solve).
    // reduced_tol_*=1e-3: lets Clarabel report AlmostSolved; dropping these
    //   restores Clarabel defaults and silently changes AlmostSolved semantics.
    // direct_solve_method="qdldl", max_threads=1: determinism pin — single-
    //   threaded QDLDL keeps the joining-loop early-bail deterministic.
    let max_iter: u32 = if trust_region.is_some() { 200 } else { 1000 };
    #[allow(clippy::similar_names)]
    let settings = DefaultSettings::<f64> {
        verbose: false,
        max_iter,
        tol_gap_abs: tol,
        tol_gap_rel: tol,
        tol_feas: tol,
        reduced_tol_gap_abs: 1e-3,
        reduced_tol_gap_rel: 1e-3,
        reduced_tol_feas: 1e-3,
        direct_solve_method: "qdldl".to_string(),
        max_threads: 1,
        ..Default::default()
    };

    let mut solver = DefaultSolver::new(&p_zero, q, &a_csc, &b_rhs, &cones_clarabel, settings)
        .map_err(|e| SolverSetupError::InvalidBundle(e.to_string()))?;

    counters::inc_clarabel(trust_region.is_some());

    solver.solve();

    let soln = &solver.solution;
    let residual = soln.r_prim.max(soln.r_dual);
    let status = map_status(soln.status, residual);
    Ok(extract_solution(&soln.x, n_grid, status))
}

/// Hard cap; Lee 2024 reports ~5–30 iterations in practice.
const SLP_MAX_OUTER_ITERS: u32 = 50;

/// Looser than `verify::EPS_FEAS`: the SLP predicate uses a raw FD estimate
/// of `b''(s)`, which is noisy near constraint-switch kinks (~1–2% spurious
/// violations). Real violations are ~143% on the CL-2024 counterexample.
const SLP_EPS_FEAS: f64 = 5e-2;

/// Avoids `1/√0` in the path-jerk linearization.
const SLP_B_FLOOR: f64 = 1.0;

/// Below this `b̄` a violator does not receive a cut. `α = J·h²/b̄^{3/2}`
/// diverges as b̄ → 0, producing steep rows that wreck the inner SOCP's
/// conditioning. ≈ (10 mm/s)².
const SLP_B_CUT_FLOOR: f64 = 100.0;

/// Warm-up before divergence rule fires; iterates routinely bounce for several
/// iterations before settling (Lee 2024: 5–30 typical).
const SLP_WARMUP_ITERS: u32 = 8;

const SLP_MIN_IMPROVEMENT: f64 = 0.01;
const SLP_NO_IMPROVEMENT_WINDOW: usize = 10;

#[derive(Debug, Clone, Copy)]
pub(crate) enum SlpOutcome {
    /// No violators within `SLP_EPS_FEAS`. `outer_iters = 0` means the
    /// base SOCP was already feasible.
    Converged {
        outer_iters: u32,
    },
    MaxIters {
        last_max_ratio: f64,
    },
    Diverged {
        last_max_ratio: f64,
        outer_iters: u32,
    },
    InnerSolverFailure,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct JerkViolator {
    #[allow(dead_code)]
    pub i: usize,
    pub ratio: f64,
}

#[inline]
fn max_ratio(vs: &[JerkViolator]) -> f64 {
    vs.iter().map(|v| v.ratio).fold(0.0_f64, f64::max)
}

/// Append one path-jerk SLP cut using weight-based b″ for non-uniform grids.
///
/// Row (sign-paired) scaled by `h̄²` so row magnitudes match the legacy uniform
/// row and remain O(1) regardless of grid refinement:
///   `3J·h̄²/√b̄ − α·h̄²·b_i − h̄²·Σ_k w_k·b_{idx[k]} ≥ 0`
/// where `α = J/b̄^{3/2}`.
///
/// For uniform spacing h̄=h the weights `w_k = b_dd_weights(h,h)` give
/// `h²·w_k = [1, -2, 1]`, reproducing the legacy `append_path_jerk_cut_to_clarabel`
/// row exactly. For non-uniform junctions this is a feasible-set-identical
/// positive scaling (`h̄² > 0`).
#[allow(clippy::too_many_arguments)]
fn append_path_jerk_cut_weights(
    i: usize,
    b_bar: f64,
    j_path: f64,
    h_bar: f64,
    idx: [usize; 3],
    w: [f64; 3],
    n_rows: &mut usize,
    rowval: &mut [Vec<usize>],
    nzval: &mut [Vec<f64>],
    b_rhs: &mut Vec<f64>,
    n_grid: usize,
) {
    debug_assert!(i < n_grid);
    debug_assert!(idx[0] < n_grid && idx[1] < n_grid && idx[2] < n_grid);
    debug_assert!(h_bar > 0.0);
    let h2 = h_bar * h_bar;
    let sqrt_b = b_bar.sqrt();
    let alpha = j_path * h2 / (b_bar * sqrt_b);
    let rhs = 3.0 * j_path * h2 / sqrt_b;

    // anchor_pos: which element of idx is i (for adding alpha to the right column).
    let anchor_pos = idx
        .iter()
        .position(|&x| x == i)
        .expect("i must appear in idx");

    // Sign-convention: A_clarabel = -A_k.
    // The two Nonneg rows are:
    //   (+): rhs - alpha·b_i - h²·b_dd ≥ 0
    //   (−): rhs - alpha·b_i + h²·b_dd ≥ 0
    // In Clarabel form A_clarabel·x + rhs ≥ 0:
    //   non-anchor k: A_clarabel[idx[k]] = ∓h²·w[k]  (- for Row+, + for Row-)
    //   anchor:       A_clarabel[idx[anchor]] = ∓h²·w[anchor] − alpha
    for &neg_b_dd in &[true, false] {
        let row = *n_rows;
        let sign_b_dd: f64 = if neg_b_dd { -1.0 } else { 1.0 };
        for k in 0..3 {
            let coeff = if k == anchor_pos {
                sign_b_dd * h2 * w[k] - alpha
            } else {
                sign_b_dd * h2 * w[k]
            };
            push_nz(rowval, nzval, idx[k], row, coeff);
        }
        b_rhs.push(rhs);
        *n_rows += 1;
    }
    let _ = n_grid;
}

/// Path-jerk violators using per-point `b_dd_weights` for non-uniform spacing.
pub(crate) fn find_jerk_violators_chain(
    b: &[f64],
    h_intervals: &[f64],
    j_path_at: &[f64],
) -> Vec<JerkViolator> {
    let n = b.len();
    if n < 3 {
        return Vec::new();
    }
    let mut out = Vec::new();
    for i in 1..n - 1 {
        let bi = b[i];
        if bi <= 0.0 {
            continue;
        }
        let (idx, hl, hr) = crate::topp::stencil::stencil_at(i, n, h_intervals);
        let w = crate::topp::stencil::b_dd_weights(hl, hr);
        let b_dd = w[0] * b[idx[0]] + w[1] * b[idx[1]] + w[2] * b[idx[2]];
        let ratio = b_dd.abs() * bi.sqrt() / (2.0 * j_path_at[i]);
        if ratio > 1.0 + SLP_EPS_FEAS {
            out.push(JerkViolator { i, ratio });
        }
    }
    out
}

/// Per-axis ratio scan for a chain grid. Includes a second pass over junction
/// duals so both geometries at shared junction points are evaluated.
pub(crate) fn max_axis_ratio_chain(
    result: &SolverResult,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
) -> f64 {
    let n = result.b.len();
    debug_assert_eq!(chain.s.len(), n);
    let mut worst: f64 = 0.0;
    for i in 0..n {
        let s_dddot = crate::topp::stencil::s_dddot_at_weights(&result.b, i, &chain.h_intervals);
        let s_dot = result.b[i].max(0.0).sqrt();
        let s_dot3 = s_dot * s_dot * s_dot;
        let s_ddot = result.a[i];
        let geom = &chain.geom[i];
        let lim = chain.limits_at(i);
        let jerk = jerk_vector(geom, s_dot3, s_dot, s_ddot, s_dddot);
        for (_, set) in lim.spatial_sets() {
            if !set.j_max.is_finite() {
                continue;
            }
            let ratio = crate::restricted_norm(&jerk, set.axes) / set.j_max;
            if ratio > worst {
                worst = ratio;
            }
        }
    }
    for jct in &chain.junctions {
        let i = jct.idx;
        let s_dddot = crate::topp::stencil::s_dddot_at_weights(&result.b, i, &chain.h_intervals);
        let s_dot = result.b[i].max(0.0).sqrt();
        let s_dot3 = s_dot * s_dot * s_dot;
        let s_ddot = result.a[i];
        let geom = &jct.geom;
        let lim = &chain.limits[jct.limits_idx];
        let jerk = jerk_vector(geom, s_dot3, s_dot, s_ddot, s_dddot);
        for (_, set) in lim.spatial_sets() {
            if !set.j_max.is_finite() {
                continue;
            }
            let ratio = crate::restricted_norm(&jerk, set.axes) / set.j_max;
            if ratio > worst {
                worst = ratio;
            }
        }
    }
    match windows {
        Some(w) => worst.max(crate::topp::follower::max_windowed_ratio(
            w, chain, &result.b, &result.a,
        )),
        None => worst.max(crate::topp::follower::max_pa_ratio(
            &result.b, &result.a, chain,
        )),
    }
}

fn jerk_vector(
    geom: &crate::topp::chain::PointGeom,
    s_dot3: f64,
    s_dot: f64,
    s_ddot: f64,
    s_dddot: f64,
) -> [f64; 3] {
    let mut jerk = [0.0_f64; 3];
    for (ax, j) in jerk.iter_mut().enumerate() {
        *j = geom.c_triple_prime[ax] * s_dot3
            + 3.0 * geom.c_double_prime[ax] * s_dot * s_ddot
            + geom.c_prime[ax] * s_dddot;
    }
    jerk
}

fn set_jerk_projection(
    geom: &crate::topp::chain::PointGeom,
    jerk: &[f64; 3],
    axes: crate::AxisSet,
) -> Option<(f64, f64, f64, f64)> {
    let jn = crate::restricted_norm(jerk, axes);
    if jn <= f64::MIN_POSITIVE {
        return None;
    }
    let mut cp = 0.0;
    let mut cpp = 0.0;
    let mut cppp = 0.0;
    for ax in axes.indices() {
        let u = jerk[ax] / jn;
        cp += u * geom.c_prime[ax];
        cpp += u * geom.c_double_prime[ax];
        cppp += u * geom.c_triple_prime[ax];
    }
    Some((jn, cp, cpp, cppp))
}

const SLP9_MAX_OUTER_ITERS: u32 = 30;
const SLP9_WARN_AT_ITER: u32 = 15;
pub(crate) const SLP9_EPS_FEAS: f64 = 5e-2;
const SLP9_RHO_B_INIT: f64 = 0.50;
const SLP9_RHO_A_INIT: f64 = 0.25;
const SLP9_RHO_B_MIN: f64 = 0.005;
const SLP9_RHO_B_MAX: f64 = 0.60;
const SLP9_RHO_A_MIN: f64 = 0.01;
const SLP9_RHO_A_MAX: f64 = 1.00;
const SLP9_MAX_BACKTRACKS: u32 = 3;
const SLP9_TARGET_DECAY: f64 = 0.85;
const SLP9_TARGET_MARGIN: f64 = 1e-3;
const SLP9_CUT_PLACEMENT_FRACTION: f64 = 0.9;
const SLP9_DAMP_TARGET_RATIO: f64 = 0.9;

/// Cut builder for a chain grid. Uses per-point stencil weights from
/// `chain.h_intervals`. Junction dual points receive extra cuts (dual geometry
/// and limits evaluated at the same stencil triple). Cuts are placed for all
/// violators above `target_floor` and tightened to `target_ratio`.
pub(crate) fn build_axis_jerk_cuts_chain(
    result: &SolverResult,
    chain: &crate::topp::chain::ChainGrid,
    target_ratio: f64,
) -> Vec<SlpCut> {
    let n = result.b.len();
    let mut cuts: Vec<SlpCut> = Vec::new();

    for i in 0..n {
        let (idx, hl, hr) = crate::topp::stencil::stencil_at(i, n, &chain.h_intervals);
        let w = crate::topp::stencil::b_dd_weights(hl, hr);
        let s_dddot = crate::topp::stencil::s_dddot_at_weights(&result.b, i, &chain.h_intervals);
        let s_dot = result.b[i].max(0.0).sqrt();
        let s_dot3 = s_dot * s_dot * s_dot;
        let s_ddot = result.a[i];
        let b_bars: [f64; 3] = [result.b[idx[0]], result.b[idx[1]], result.b[idx[2]]];
        let geom = &chain.geom[i];
        let lim = chain.limits_at(i);
        let jerk = jerk_vector(geom, s_dot3, s_dot, s_ddot, s_dddot);
        for (set_idx, set) in lim.spatial_sets() {
            if !set.j_max.is_finite() {
                continue;
            }
            let Some((jn, cp, cpp, cppp)) = set_jerk_projection(geom, &jerk, set.axes) else {
                continue;
            };
            let ratio = jn / set.j_max;
            let j_lim = if ratio > SLP9_CUT_PLACEMENT_FRACTION * target_ratio {
                set.j_max * target_ratio
            } else if ratio > SLP9_EPS_FEAS {
                set.j_max
            } else {
                continue;
            };
            cuts.push(SlpCut::AxisJerk(AxisJerkCut {
                i,
                axis: set_idx,
                idx,
                w,
                b_bars,
                a_bar_i: result.a[i],
                cp,
                cpp,
                cppp,
                j_lim_inflated: j_lim,
            }));
        }
        // Junction dual: same stencil triple, right-side geometry and limits.
        for jct in chain.junctions.iter().filter(|jct| jct.idx == i) {
            let jlim = &chain.limits[jct.limits_idx];
            let jgeom = &jct.geom;
            let jjerk = jerk_vector(jgeom, s_dot3, s_dot, s_ddot, s_dddot);
            for (set_idx, set) in jlim.spatial_sets() {
                if !set.j_max.is_finite() {
                    continue;
                }
                let Some((jn, cp, cpp, cppp)) = set_jerk_projection(jgeom, &jjerk, set.axes) else {
                    continue;
                };
                let ratio = jn / set.j_max;
                let j_lim = if ratio > SLP9_CUT_PLACEMENT_FRACTION * target_ratio {
                    set.j_max * target_ratio
                } else if ratio > SLP9_EPS_FEAS {
                    set.j_max
                } else {
                    continue;
                };
                cuts.push(SlpCut::AxisJerk(AxisJerkCut {
                    i,
                    axis: set_idx,
                    idx,
                    w,
                    b_bars,
                    a_bar_i: result.a[i],
                    cp,
                    cpp,
                    cppp,
                    j_lim_inflated: j_lim,
                }));
            }
        }
    }
    cuts
}

/// Path-jerk SLP outer loop for chain grids (non-uniform spacing).
///
/// Clone of `slp_solve` control flow; calls `find_jerk_violators_chain` and
/// emits weight-based path-jerk cuts via `append_path_jerk_cut_weights`.
/// Wired into the schedule entry in Task 8.
pub(crate) fn slp_solve_chain(
    bundle: &ConstraintBundle,
    tol: f64,
    scale: &SolverScale,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    debug_assert!(bundle.j_path_at.iter().all(|&j| j > 0.0));
    let n = bundle.n_grid;

    let mut cuts: Vec<SlpCut> = Vec::new();
    let mut last_result = solve_with_cuts(bundle, &cuts, tol, scale)?;

    if matches!(
        last_result.status,
        SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
    ) {
        return Ok((last_result, SlpOutcome::InnerSolverFailure));
    }

    let violators =
        find_jerk_violators_chain(&last_result.b, &bundle.h_intervals, &bundle.j_path_at);
    if violators.is_empty() {
        return Ok((last_result, SlpOutcome::Converged { outer_iters: 0 }));
    }

    let mut best_result = last_result.clone();
    let mut best_ratio_so_far = max_ratio(&violators);
    let mut max_ratio_history: Vec<f64> = Vec::new();
    let mut best_ratio_history: Vec<f64> = Vec::new();
    max_ratio_history.push(best_ratio_so_far);
    best_ratio_history.push(best_ratio_so_far);
    let b_cut_floor = scale.to_scaled_b(SLP_B_CUT_FLOOR);

    for outer in 1..=SLP_MAX_OUTER_ITERS {
        cuts.clear();
        let mut added = 0_usize;
        for i in 1..n - 1 {
            let b_bar = last_result.b[i];
            if b_bar < b_cut_floor {
                continue;
            }
            let (idx, hl, hr) = crate::topp::stencil::stencil_at(i, n, &bundle.h_intervals);
            let w = crate::topp::stencil::b_dd_weights(hl, hr);
            let h_bar = 0.5 * (bundle.h_intervals[i - 1] + bundle.h_intervals[i]);
            cuts.push(SlpCut::PathJerkWeights {
                i,
                b_bar,
                j_path: bundle.j_path_at[i],
                idx,
                w,
                h_bar,
            });
            added += 1;
        }
        if added == 0 {
            return Ok((best_result, SlpOutcome::Converged { outer_iters: outer }));
        }

        let new_result = solve_with_cuts(bundle, &cuts, tol, scale)?;
        if matches!(
            new_result.status,
            SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
        ) {
            return Ok((
                best_result,
                SlpOutcome::MaxIters {
                    last_max_ratio: best_ratio_so_far,
                },
            ));
        }
        last_result = new_result;

        let new_violators =
            find_jerk_violators_chain(&last_result.b, &bundle.h_intervals, &bundle.j_path_at);
        if new_violators.is_empty() {
            return Ok((last_result, SlpOutcome::Converged { outer_iters: outer }));
        }

        let cur_max = max_ratio(&new_violators);
        max_ratio_history.push(cur_max);
        let prev_best = *best_ratio_history.last().unwrap_or(&f64::INFINITY);
        let cur_best = prev_best.min(cur_max);
        best_ratio_history.push(cur_best);
        if cur_max < best_ratio_so_far {
            best_ratio_so_far = cur_max;
            best_result = last_result.clone();
        }
        let _ = cur_best;

        if outer > SLP_WARMUP_ITERS && best_ratio_history.len() > SLP_NO_IMPROVEMENT_WINDOW {
            let len = best_ratio_history.len();
            let baseline = best_ratio_history[len - 1 - SLP_NO_IMPROVEMENT_WINDOW];
            let current = best_ratio_history[len - 1];
            let improvement = (baseline - current) / baseline.max(1.0);
            if improvement < SLP_MIN_IMPROVEMENT {
                return Ok((
                    best_result,
                    SlpOutcome::Diverged {
                        last_max_ratio: best_ratio_so_far,
                        outer_iters: outer,
                    },
                ));
            }
        }
    }

    Ok((
        best_result,
        SlpOutcome::MaxIters {
            last_max_ratio: best_ratio_so_far,
        },
    ))
}

enum Slp9LoopOutcome {
    Done(SolverResult, SlpOutcome),
    TrFloorStall {
        best_result: SolverResult,
        best_ratio: f64,
        outer_iters: u32,
        accepted_total: u32,
    },
}

#[allow(clippy::too_many_arguments)]
#[allow(clippy::too_many_arguments)]
fn run_slp9_loop(
    bundle: &ConstraintBundle,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
    start: SolverResult,
    tol: f64,
    scale: &SolverScale,
    path_outer_iters: u32,
    outer_iters_already: u32,
    rho_b_init: f64,
    rho_a_init: f64,
    rho_b_max: f64,
    rho_a_max: f64,
) -> Result<Slp9LoopOutcome, SolverSetupError> {
    let _phase_guard = Slp9PhaseGuard::enter();

    let mut last_result = start.clone();
    let mut best_result = start;
    let mut best_ratio = max_axis_ratio_chain(&last_result, chain, windows);
    let mut rho_b = rho_b_init;
    let mut rho_a = rho_a_init;
    let mut accepted_total: u32 = 0;

    for outer in 1..=SLP9_MAX_OUTER_ITERS {
        let global_outer = outer_iters_already + outer;
        if global_outer == SLP9_WARN_AT_ITER {
            eprintln!(
                "slp9_chain warning: per-axis SLP not converged at iter {global_outer} \
                 (best ratio = {best_ratio:.4})",
            );
        }

        let target_floor = (1.0 + SLP9_EPS_FEAS) * (1.0 - SLP9_TARGET_MARGIN);
        let target_ratio = (best_ratio * SLP9_TARGET_DECAY).max(target_floor);
        let mut cuts = build_axis_jerk_cuts_chain(&last_result, chain, target_ratio);
        match windows {
            Some(w) => cuts.extend(
                crate::topp::follower::build_windowed_follower_cuts(
                    &last_result.b,
                    &last_result.a,
                    chain,
                    w,
                    target_ratio,
                    SLP9_EPS_FEAS,
                    SLP9_CUT_PLACEMENT_FRACTION,
                    SLP9_TARGET_DECAY,
                    scale.to_scaled_b(SLP_B_FLOOR),
                )
                .into_iter()
                .map(SlpCut::FollowerWindowed),
            ),
            None => cuts.extend(
                crate::topp::follower::build_follower_pa_cuts(
                    &last_result.b,
                    &last_result.a,
                    chain,
                    target_ratio,
                    SLP9_EPS_FEAS,
                    SLP9_CUT_PLACEMENT_FRACTION,
                    SLP9_TARGET_DECAY,
                    scale.to_scaled_b(SLP_B_FLOOR),
                )
                .into_iter()
                .map(SlpCut::Follower),
            ),
        }
        if cuts.is_empty() {
            return Ok(Slp9LoopOutcome::Done(
                last_result,
                SlpOutcome::Converged {
                    outer_iters: path_outer_iters + global_outer,
                },
            ));
        }

        let mut accepted: Option<SolverResult> = None;
        for backtrack in 0..=SLP9_MAX_BACKTRACKS {
            let bt_i32 = i32::try_from(backtrack).unwrap_or(i32::MAX);
            let tr = TrustRegion {
                rho_b: rho_b * 0.5_f64.powi(bt_i32),
                rho_a: rho_a * 0.5_f64.powi(bt_i32),
            };
            let candidate = solve_with_cuts_and_trust_region(
                bundle,
                &cuts,
                Some(tr),
                &last_result.b,
                &last_result.a,
                tol,
                scale,
            )?;
            if matches!(
                candidate.status,
                SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
            ) {
                continue;
            }
            let cand_ratio = max_axis_ratio_chain(&candidate, chain, windows);
            if cand_ratio < best_ratio {
                accepted = Some(candidate);
                best_ratio = cand_ratio;
                break;
            }
        }
        if accepted.is_none() {
            let candidate = solve_with_cuts(bundle, &cuts, tol, scale)?;
            if !matches!(
                candidate.status,
                SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
            ) {
                let cand_ratio = max_axis_ratio_chain(&candidate, chain, windows);
                if cand_ratio < best_ratio {
                    accepted = Some(candidate);
                    best_ratio = cand_ratio;
                }
            }
        }

        if let Some(new_result) = accepted {
            last_result = new_result.clone();
            best_result = new_result;
            rho_b = (rho_b * 1.5).min(rho_b_max);
            rho_a = (rho_a * 1.5).min(rho_a_max);
            accepted_total += 1;

            if best_ratio <= 1.0 + SLP9_EPS_FEAS {
                return Ok(Slp9LoopOutcome::Done(
                    last_result,
                    SlpOutcome::Converged {
                        outer_iters: path_outer_iters + global_outer,
                    },
                ));
            }
        } else {
            rho_b = (rho_b * 0.5).max(SLP9_RHO_B_MIN);
            rho_a = (rho_a * 0.5).max(SLP9_RHO_A_MIN);
            if rho_b <= SLP9_RHO_B_MIN * 1.0001 && rho_a <= SLP9_RHO_A_MIN * 1.0001 {
                return Ok(Slp9LoopOutcome::TrFloorStall {
                    best_result,
                    best_ratio,
                    outer_iters: path_outer_iters + global_outer,
                    accepted_total,
                });
            }
        }
    }

    Ok(Slp9LoopOutcome::Done(
        best_result,
        SlpOutcome::MaxIters {
            last_max_ratio: best_ratio,
        },
    ))
}

/// Uniform time dilation of the iterate: `b ← λ²·b`, `a ← λ²·a`. Every
/// constraint-family ratio decreases monotonically as λ → 0, so this can always
/// restore a feasible point while staying on the `b' = 2a` motion manifold.
fn damp_profile_uniform(result: &SolverResult, lambda: f64) -> SolverResult {
    let l2 = lambda * lambda;
    SolverResult {
        b: result.b.iter().map(|&b| b * l2).collect(),
        a: result.a.iter().map(|&a| a * l2).collect(),
        status: result.status,
    }
}

fn uniform_damp_for_feasibility(
    result: &SolverResult,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
    current_ratio: f64,
) -> (SolverResult, f64) {
    let mut lambda = (SLP9_DAMP_TARGET_RATIO / current_ratio).cbrt().min(1.0);
    for _ in 0..32 {
        let damped = damp_profile_uniform(result, lambda);
        let ratio = max_axis_ratio_chain(&damped, chain, windows);
        if ratio <= SLP9_DAMP_TARGET_RATIO {
            return (damped, ratio);
        }
        lambda *= 0.75;
    }
    let damped = damp_profile_uniform(result, lambda);
    let ratio = max_axis_ratio_chain(&damped, chain, windows);
    (damped, ratio)
}

/// Cheap O(N) feasible seed for the recovery descent when the direct
/// feasibility descent stalled. Uniform time dilation (`b ← λ²b, a ← λ²a`)
/// reduces every constraint ratio monotonically and stays on the `b' = 2a`
/// motion manifold (zero Clarabel calls). Returns `None` when even full
/// dilation cannot reach `ratio ≤ 1.0` — the curve is genuinely infeasible and
/// must not be laundered into success.
fn seed_feasible_point(
    result: &SolverResult,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
    current_ratio: f64,
) -> Option<SolverResult> {
    let (uniform, uniform_ratio) =
        uniform_damp_for_feasibility(result, chain, windows, current_ratio);
    (uniform_ratio <= 1.0).then_some(uniform)
}

const SLP9_POLISH_MAX_ITERS: u32 = 12;
const SLP9_POLISH_MIN_GAIN: f64 = 1e-6;
const SLP9_POLISH_MAX_BACKTRACKS: u32 = 6;
const SLP9_POLISH_RHO_B_INIT: f64 = 0.25;
const SLP9_POLISH_RHO_A_INIT: f64 = 0.50;
const SLP9_POLISH_RHO_B_MAX: f64 = 0.60;
const SLP9_POLISH_RHO_A_MAX: f64 = 1.00;
const SLP9_RECOVER_MAX_PROJECTIONS: u32 = 4;

fn profile_time(result: &SolverResult, h_intervals: &[f64]) -> f64 {
    h_intervals
        .iter()
        .enumerate()
        .map(|(i, h)| {
            let v_sum = result.b[i].max(0.0).sqrt() + result.b[i + 1].max(0.0).sqrt();
            if v_sum > 0.0 {
                2.0 * h / v_sum
            } else {
                f64::INFINITY
            }
        })
        .sum()
}

/// The SLP9 loop only ever reduces constraint violation; descending from the
/// follower-free iterate it stops at first feasibility, which can sit a few
/// percent below the local optimum. Re-maximize speed under full-cap cuts
/// rebuilt at each accepted iterate while staying feasible.
fn build_polish_cuts(
    current: &SolverResult,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
    scale: &SolverScale,
) -> Vec<SlpCut> {
    let mut cuts = build_axis_jerk_cuts_chain(current, chain, 1.0);
    if let Some(w) = windows {
        cuts.extend(
            crate::topp::follower::build_windowed_follower_cuts(
                &current.b,
                &current.a,
                chain,
                w,
                1.0,
                SLP9_EPS_FEAS,
                SLP9_CUT_PLACEMENT_FRACTION,
                SLP9_TARGET_DECAY,
                scale.to_scaled_b(SLP_B_FLOOR),
            )
            .into_iter()
            .map(SlpCut::FollowerWindowed),
        );
    }
    cuts
}

fn polish_axis_jerk(
    bundle: &ConstraintBundle,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
    start: SolverResult,
    tol: f64,
    scale: &SolverScale,
) -> Result<SolverResult, SolverSetupError> {
    let mut current = start;
    let mut current_time = profile_time(&current, &bundle.h_intervals);
    let mut rho_b = SLP9_POLISH_RHO_B_INIT;
    let mut rho_a = SLP9_POLISH_RHO_A_INIT;
    for _ in 0..SLP9_POLISH_MAX_ITERS {
        let cuts = build_polish_cuts(&current, chain, windows, scale);
        let mut accepted: Option<(SolverResult, f64)> = None;
        for backtrack in 0..=SLP9_POLISH_MAX_BACKTRACKS {
            let bt = i32::try_from(backtrack).unwrap_or(i32::MAX);
            let tr = if backtrack == 0 {
                None
            } else {
                Some(TrustRegion {
                    rho_b: rho_b * 0.5_f64.powi(bt - 1),
                    rho_a: rho_a * 0.5_f64.powi(bt - 1),
                })
            };
            let candidate = solve_with_cuts_and_trust_region(
                bundle, &cuts, tr, &current.b, &current.a, tol, scale,
            )?;
            if matches!(
                candidate.status,
                SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
            ) {
                continue;
            }
            let ratio = max_axis_ratio_chain(&candidate, chain, windows);
            let t = profile_time(&candidate, &bundle.h_intervals);
            if ratio <= 1.0 + SLP9_EPS_FEAS && t < current_time * (1.0 - SLP9_POLISH_MIN_GAIN) {
                accepted = Some((candidate, t));
                break;
            }
        }
        match accepted {
            Some((cand, t)) => {
                current = cand;
                current_time = t;
                rho_b = (rho_b * 1.5).min(SLP9_POLISH_RHO_B_MAX);
                rho_a = (rho_a * 1.5).min(SLP9_POLISH_RHO_A_MAX);
            }
            None => break,
        }
    }
    Ok(current)
}

/// Recovers a feasible, manifold-consistent profile from a uniformly-damped
/// seed when the direct feasibility descent stalled. The uniform-damp seed is
/// feasible but off the `b' = 2a` motion manifold and carries a scaled entry
/// `b[0]`; a short trust-region-boxed cutting-plane projection re-pins
/// `b[0] = v_start²` and lands on a feasible on-manifold profile (the only kind
/// valid as an output trajectory) in a handful of Clarabel solves — not a full
/// `run_slp9_loop` re-descent, which would re-stall and burn the whole budget.
/// A final time-maximizing polish nudges the projected profile toward the local
/// time-optimum while staying feasible. Returns the recovered profile and its
/// axis-jerk ratio; the seed is the feasibility floor.
fn recover_speed_from_seed(
    bundle: &ConstraintBundle,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
    seed: SolverResult,
    tol: f64,
    scale: &SolverScale,
) -> Result<(SolverResult, f64), SolverSetupError> {
    let mut best = seed.clone();
    let mut best_time = profile_time(&best, &bundle.h_intervals);
    let mut linearization = seed;

    for _ in 0..SLP9_RECOVER_MAX_PROJECTIONS {
        let target = max_axis_ratio_chain(&linearization, chain, windows).min(1.0);
        let cuts = build_axis_jerk_cuts_chain(&linearization, chain, target);
        let tr = TrustRegion {
            rho_b: SLP9_RHO_B_MAX,
            rho_a: SLP9_RHO_A_MAX,
        };
        let cand = solve_with_cuts_and_trust_region(
            bundle,
            &cuts,
            Some(tr),
            &linearization.b,
            &linearization.a,
            tol,
            scale,
        )?;
        if matches!(
            cand.status,
            SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
        ) {
            break;
        }
        let cand_ratio = max_axis_ratio_chain(&cand, chain, windows);
        let cand_time = profile_time(&cand, &bundle.h_intervals);
        if cand_ratio <= 1.0 + SLP9_EPS_FEAS
            && cand_time <= best_time * (1.0 + SLP9_POLISH_MIN_GAIN)
        {
            best = cand.clone();
            best_time = cand_time;
        }
        linearization = cand;
    }

    let polished = polish_axis_jerk(bundle, chain, windows, best, tol, scale)?;
    let polished_ratio = max_axis_ratio_chain(&polished, chain, windows);
    Ok((polished, polished_ratio))
}

#[allow(clippy::too_many_lines)]
pub(crate) fn slp_solve_with_axis_jerk_chain(
    bundle: &ConstraintBundle,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
    tol: f64,
    scale: &SolverScale,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let (result, outcome) =
        slp_solve_with_axis_jerk_chain_inner(bundle, chain, windows, tol, scale)?;
    if let (Some(w), SlpOutcome::Converged { .. }) = (windows, &outcome) {
        let polished = polish_axis_jerk(bundle, chain, Some(w), result, tol, scale)?;
        return Ok((polished, outcome));
    }
    Ok((result, outcome))
}

#[allow(clippy::too_many_lines)]
fn slp_solve_with_axis_jerk_chain_inner(
    bundle: &ConstraintBundle,
    chain: &crate::topp::chain::ChainGrid,
    windows: Option<&crate::topp::follower::FollowerWindows>,
    tol: f64,
    scale: &SolverScale,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let (path_result, path_outcome) = slp_solve_chain(bundle, tol, scale)?;

    if matches!(
        path_outcome,
        SlpOutcome::InnerSolverFailure | SlpOutcome::Diverged { .. } | SlpOutcome::MaxIters { .. }
    ) {
        return Ok((path_result, path_outcome));
    }

    debug_assert_eq!(chain.s.len(), path_result.b.len());

    let path_outer_iters = match path_outcome {
        SlpOutcome::Converged { outer_iters } => outer_iters,
        _ => 0,
    };

    let initial_max = max_axis_ratio_chain(&path_result, chain, windows);
    if initial_max <= 1.0 + SLP9_EPS_FEAS {
        return Ok((
            path_result,
            SlpOutcome::Converged {
                outer_iters: path_outer_iters,
            },
        ));
    }

    let loop_outcome = run_slp9_loop(
        bundle,
        chain,
        windows,
        path_result,
        tol,
        scale,
        path_outer_iters,
        0,
        SLP9_RHO_B_INIT,
        SLP9_RHO_A_INIT,
        SLP9_RHO_B_MAX,
        SLP9_RHO_A_MAX,
    )?;
    match loop_outcome {
        Slp9LoopOutcome::Done(result, outcome) => Ok((result, outcome)),
        Slp9LoopOutcome::TrFloorStall {
            best_result,
            best_ratio,
            outer_iters,
            accepted_total: _,
        } => {
            if best_ratio <= 1.0 + SLP9_EPS_FEAS {
                let polished = polish_axis_jerk(bundle, chain, windows, best_result, tol, scale)?;
                let polished_ratio = max_axis_ratio_chain(&polished, chain, windows);
                return Ok((
                    polished,
                    SlpOutcome::Diverged {
                        last_max_ratio: polished_ratio,
                        outer_iters,
                    },
                ));
            }
            let Some(seed) = seed_feasible_point(&best_result, chain, windows, best_ratio) else {
                return Ok((
                    best_result,
                    SlpOutcome::Diverged {
                        last_max_ratio: best_ratio,
                        outer_iters,
                    },
                ));
            };
            counters::mark_restoration();
            let (recovered, recovered_ratio) =
                recover_speed_from_seed(bundle, chain, windows, seed, tol, scale)?;
            Ok((
                recovered,
                SlpOutcome::Diverged {
                    last_max_ratio: recovered_ratio,
                    outer_iters,
                },
            ))
        }
    }
}

#[cfg(test)]
mod tests;
