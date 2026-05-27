//! Timeline: resolves a u64 MCU clock timestamp to the Bézier piece covering
//! that instant, returning a reference to the piece and the piece-local time
//! in seconds.
//!
//! The current piece (32 bytes) is copied into the cursor at load and
//! advance time. The hot path reads from the local copy — no pointer
//! chasing, no pool lookup, no unsafe. The CurvePool is only touched on
//! piece transitions (~once per millisecond).

use crate::curve_pool::{CurveHandle, CurvePool};
use crate::monomial::BezierPieceMonomial;

pub const N_AXES: usize = 4;

#[derive(Clone, Copy, Debug)]
struct AxisCursor {
    piece: Option<BezierPieceMonomial>,
    handle: CurveHandle,
    piece_idx: u16,
    piece_count: u16,
    piece_start_cycles: u64,
    piece_end_cycles: u64,
}

impl AxisCursor {
    const fn empty() -> Self {
        Self {
            piece: None,
            handle: CurveHandle::UNUSED_SENTINEL,
            piece_idx: 0,
            piece_count: 0,
            piece_start_cycles: 0,
            piece_end_cycles: 0,
        }
    }

    fn is_active(&self) -> bool {
        self.piece.is_some()
    }
}

/// Timeline: maps `(axis, now_cycles)` to the piece covering that instant.
///
/// Each axis cursor holds a COPY of the current `BezierPieceMonomial` (32
/// bytes). The hot path reads this copy directly — zero unsafe, zero pointer
/// chasing, optimal cache locality.
///
/// The `CurvePool` is accessed only at piece transitions to copy the next
/// piece. That is the only `unsafe` site, and it runs ~once per millisecond.
///
/// ISR-exclusive. Populate via `load_axis` before evaluation begins.
#[derive(Debug)]
pub struct Timeline {
    inv_clock_hz: f32,
    clock_hz: f32,
    axes: [AxisCursor; N_AXES],
}

impl Timeline {
    pub fn new(clock_hz: f32) -> Self {
        Self {
            inv_clock_hz: 1.0 / clock_hz,
            clock_hz,
            axes: [AxisCursor::empty(); N_AXES],
        }
    }

    /// Load a curve for one axis. Copies piece 0 from the pool into the
    /// cursor and sets up timing. No unsafe — uses `pool.copy_piece`.
    pub fn load_axis(
        &mut self,
        axis: usize,
        handle: CurveHandle,
        segment_start_cycles: u64,
        pool: &CurvePool,
    ) -> bool {
        if axis >= N_AXES {
            return false;
        }
        if handle == CurveHandle::UNUSED_SENTINEL {
            self.axes[axis] = AxisCursor::empty();
            return false;
        }
        let Some(count) = pool.piece_count(handle) else {
            self.axes[axis] = AxisCursor::empty();
            return false;
        };
        if count == 0 {
            self.axes[axis] = AxisCursor::empty();
            return false;
        }
        let Some(first_piece) = pool.copy_piece(handle, 0) else {
            self.axes[axis] = AxisCursor::empty();
            return false;
        };
        let duration_cycles = (first_piece.duration * self.clock_hz) as u64;
        self.axes[axis] = AxisCursor {
            piece: Some(first_piece),
            handle,
            piece_idx: 0,
            piece_count: count,
            piece_start_cycles: segment_start_cycles,
            piece_end_cycles: segment_start_cycles + duration_cycles,
        };
        true
    }

    pub fn reset(&mut self) {
        self.axes = [AxisCursor::empty(); N_AXES];
    }

    pub fn clear_axis(&mut self, axis: usize) {
        if axis < N_AXES {
            self.axes[axis] = AxisCursor::empty();
        }
    }

    pub fn all_idle(&self) -> bool {
        self.axes.iter().all(|c| !c.is_active())
    }

    pub fn axis_active(&self, axis: usize) -> bool {
        axis < N_AXES && self.axes[axis].is_active()
    }

    /// Resolve `now_cycles` to the piece covering that instant on `axis`.
    ///
    /// Hot path: no unsafe. Reads from the cached piece copy in the cursor.
    /// Returns `NeedsAdvance` when the current piece is exhausted — the
    /// caller must then call `advance_piece` with a pool reference.
    pub fn get_piece(
        &mut self,
        axis: usize,
        now_cycles: u64,
    ) -> GetPieceResult {
        if axis >= N_AXES {
            return GetPieceResult::Idle;
        }
        let inv_hz = self.inv_clock_hz;
        let cursor = &mut self.axes[axis];
        let Some(piece) = cursor.piece.as_ref() else {
            return GetPieceResult::Idle;
        };

        if now_cycles < cursor.piece_end_cycles {
            let delta = now_cycles.saturating_sub(cursor.piece_start_cycles);
            return GetPieceResult::Hit(piece, delta as f32 * inv_hz);
        }

        GetPieceResult::NeedsAdvance
    }

    /// Advance to the next piece, copying it from the CurvePool.
    /// Call this when `get_piece` returns `NeedsAdvance`.
    /// No unsafe — uses `pool.copy_piece`.
    pub fn advance_piece(
        &mut self,
        axis: usize,
        now_cycles: u64,
        pool: &CurvePool,
    ) -> GetPieceResult {
        if axis >= N_AXES {
            return GetPieceResult::Idle;
        }
        let clock_hz = self.clock_hz;
        let inv_hz = self.inv_clock_hz;
        let cursor = &mut self.axes[axis];

        loop {
            cursor.piece_idx += 1;
            let Some(next_piece) = pool.copy_piece(cursor.handle, cursor.piece_idx as usize) else {
                *cursor = AxisCursor::empty();
                return GetPieceResult::Idle;
            };

            cursor.piece_start_cycles = cursor.piece_end_cycles;
            let duration_cycles = (next_piece.duration * clock_hz) as u64;
            cursor.piece_end_cycles = cursor.piece_start_cycles + duration_cycles;
            cursor.piece = Some(next_piece);

            if now_cycles < cursor.piece_end_cycles {
                let delta = now_cycles.saturating_sub(cursor.piece_start_cycles);
                return GetPieceResult::Hit(
                    cursor.piece.as_ref().unwrap(),
                    delta as f32 * inv_hz,
                );
            }
        }
    }
}

/// Result of `get_piece`.
#[derive(Debug)]
pub enum GetPieceResult<'a> {
    /// The piece covers `now_cycles`. Contains the piece reference and t_local in seconds.
    Hit(&'a BezierPieceMonomial, f32),
    /// The current piece is exhausted. Caller must call `advance_piece` with a pool reference.
    NeedsAdvance,
    /// No active curve on this axis.
    Idle,
}

// Test helpers — load pieces without CurvePool.
#[cfg(any(test, feature = "host"))]
impl Timeline {
    pub fn test_load_pieces(
        &mut self,
        axis: usize,
        pieces: &[BezierPieceMonomial],
        segment_start_cycles: u64,
    ) {
        if axis >= N_AXES || pieces.is_empty() {
            return;
        }
        let first = pieces[0];
        let duration_cycles = (first.duration * self.clock_hz) as u64;
        self.axes[axis] = AxisCursor {
            piece: Some(first),
            handle: CurveHandle::UNUSED_SENTINEL,
            piece_idx: 0,
            piece_count: pieces.len() as u16,
            piece_start_cycles: segment_start_cycles,
            piece_end_cycles: segment_start_cycles + duration_cycles,
        };
    }

    /// Test-only advance that reads from a slice instead of the pool.
    pub fn test_advance_piece(
        &mut self,
        axis: usize,
        now_cycles: u64,
        pieces: &[BezierPieceMonomial],
    ) -> GetPieceResult {
        if axis >= N_AXES {
            return GetPieceResult::Idle;
        }
        let clock_hz = self.clock_hz;
        let inv_hz = self.inv_clock_hz;
        let cursor = &mut self.axes[axis];

        loop {
            cursor.piece_idx += 1;
            if cursor.piece_idx >= cursor.piece_count {
                *cursor = AxisCursor::empty();
                return GetPieceResult::Idle;
            }
            let next_piece = pieces[cursor.piece_idx as usize];

            cursor.piece_start_cycles = cursor.piece_end_cycles;
            let duration_cycles = (next_piece.duration * clock_hz) as u64;
            cursor.piece_end_cycles = cursor.piece_start_cycles + duration_cycles;
            cursor.piece = Some(next_piece);

            if now_cycles < cursor.piece_end_cycles {
                let delta = now_cycles.saturating_sub(cursor.piece_start_cycles);
                return GetPieceResult::Hit(
                    cursor.piece.as_ref().unwrap(),
                    delta as f32 * inv_hz,
                );
            }
        }
    }
}
