//! Timeline: resolves a u64 MCU clock timestamp to the Bézier piece covering
//! that instant, returning a reference to the piece and the piece-local time
//! in seconds.
//!
//! # Architecture
//!
//! The Timeline is Layer 1 in the three-layer ISR evaluation stack:
//!
//! ```text
//! Timeline  get_piece(axis, now_cycles) → (&Piece, t_local_sec)
//!   └─ Evaluator  eval_position(&Piece, t_local) → mm
//!       └─ Step dispatch  quantize → step pulses
//! ```
//!
//! The hot path is a single `u64` comparison against the cached piece end.
//! When the cache is valid, no searching occurs: the function computes
//! `t_local = (now - piece_start) * inv_clock_hz` and returns.
//!
//! # no_std
//!
//! This module is `no_std`-compatible. It uses `heapless::Vec` for fixed-
//! capacity piece storage (no heap allocation).
//!
//! # Example
//!
//! ```rust
//! use runtime::timeline::{Timeline, TimedPiece};
//! use runtime::monomial::BezierPieceMonomial;
//!
//! const CLOCK_HZ: u32 = 520_000_000;
//! const INV_CLOCK_HZ: f32 = 1.0 / 520_000_000.0;
//!
//! let piece = BezierPieceMonomial {
//!     coeffs: [0.0, 10.0, 0.0, 0.0],
//!     vel_coeffs: [10.0, 0.0, 0.0],
//!     duration: 0.1,
//! };
//! let timed = TimedPiece {
//!     piece,
//!     start_cycles: 0,
//!     end_cycles: (0.1 * CLOCK_HZ as f32) as u64,
//! };
//! let mut timeline = Timeline::new(INV_CLOCK_HZ);
//! timeline.push_piece(0, timed).ok();
//!
//! let result = timeline.get_piece(0, 26_000_000);
//! assert!(result.is_some());
//! ```

use heapless::Vec;

use crate::monomial::BezierPieceMonomial;

/// Maximum number of pieces buffered per axis.
pub const MAX_PIECES_PER_AXIS: usize = 16;

/// Number of axes supported by the Timeline.
pub const N_AXES: usize = 4;

/// A Bézier piece with its absolute timing interval expressed in CPU cycles.
///
/// `start_cycles` is the first cycle for which this piece is valid.
/// `end_cycles` is the first cycle that belongs to the *next* piece
/// (i.e. the half-open interval `[start_cycles, end_cycles)`).
#[derive(Clone, Copy, Debug)]
pub struct TimedPiece {
    pub piece: BezierPieceMonomial,
    pub start_cycles: u64,
    pub end_cycles: u64,
}

/// Per-axis cursor state: index of the currently-active piece within the
/// per-axis `Vec`.
#[derive(Clone, Copy, Debug)]
struct AxisCursor {
    /// Index into the `pieces` vec of the currently-cached piece, or `None`
    /// if no piece has been loaded yet.
    current: Option<usize>,
}

impl AxisCursor {
    const fn new() -> Self {
        Self { current: None }
    }
}

/// Timeline: maps `(axis, now_cycles)` to the piece covering that instant.
///
/// Generic over the number of axes (`N`) and the maximum pieces per axis
/// (`CAP`). The public API uses the concrete alias [`Timeline`] which is
/// parameterised for the H7 MCU defaults.
///
/// Internally each axis holds a `heapless::Vec<TimedPiece, CAP>` and a
/// cursor tracking the index of the currently-active piece. The hot path
/// executes when the cursor is valid and `now_cycles < end_cycles`: one
/// subtraction, one comparison, one f32 multiply.
#[derive(Debug)]
pub struct TimelineInner<const N: usize, const CAP: usize> {
    /// `inv_clock_hz`: multiply instead of divide for t_local conversion.
    inv_clock_hz: f32,
    /// Per-axis piece queues.
    pieces: [Vec<TimedPiece, CAP>; N],
    /// Per-axis cursor.
    cursors: [AxisCursor; N],
}

// `Vec` from heapless is not `Copy`, so we cannot derive the array init
// trivially. Use a const fn approach with `core::array::from_fn` on stable
// Rust 1.63+. `heapless::Vec` does implement `Default` (empty vec).
impl<const N: usize, const CAP: usize> TimelineInner<N, CAP> {
    /// Construct an empty Timeline with the given `inv_clock_hz` reciprocal.
    ///
    /// `inv_clock_hz = 1.0 / clock_hz` should be pre-computed once to
    /// keep the hot path multiply-only.
    pub fn new(inv_clock_hz: f32) -> Self {
        Self {
            inv_clock_hz,
            pieces: core::array::from_fn(|_| Vec::new()),
            cursors: [AxisCursor::new(); N],
        }
    }

    /// Append a [`TimedPiece`] to the given axis's queue.
    ///
    /// Returns `Err(timed)` if the queue is full (capacity `CAP`).
    pub fn push_piece(&mut self, axis: usize, timed: TimedPiece) -> Result<(), TimedPiece> {
        let Some(q) = self.pieces.get_mut(axis) else {
            return Err(timed);
        };
        q.push(timed).map_err(|e| e)
    }

    /// Resolve `now_cycles` to the piece covering that instant on `axis`.
    ///
    /// Returns `Some((&piece, t_local_sec))` when a valid piece is found,
    /// where `t_local_sec = (now_cycles - piece.start_cycles) * inv_clock_hz`.
    ///
    /// Returns `None` when:
    /// - `axis >= N`
    /// - the axis queue is empty
    /// - all pieces on the axis have been exhausted (now is past the last end)
    ///
    /// # Hot path
    ///
    /// When the cursor is valid and `now_cycles < end_cycles`, this executes:
    /// one `u64` subtraction, one `u64` comparison, one `f32` multiply, and
    /// a pointer return — no searching.
    pub fn get_piece(&mut self, axis: usize, now_cycles: u64) -> Option<(&BezierPieceMonomial, f32)> {
        if axis >= N {
            return None;
        }
        let inv_hz = self.inv_clock_hz;
        let cursor = &mut self.cursors[axis];
        let pieces = &self.pieces[axis];
        let len = pieces.len();

        let mut idx = cursor.current.unwrap_or(0);

        while idx < len {
            let tp = &pieces[idx];
            if now_cycles < tp.end_cycles {
                cursor.current = Some(idx);
                let delta = now_cycles.saturating_sub(tp.start_cycles);
                return Some((&tp.piece, delta as f32 * inv_hz));
            }
            idx += 1;
        }

        None
    }
}

/// Concrete Timeline for the H7 MCU (4 axes, 16 pieces per axis).
pub type Timeline = TimelineInner<N_AXES, MAX_PIECES_PER_AXIS>;
