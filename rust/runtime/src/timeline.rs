//! Timeline: resolves a u64 MCU clock timestamp to the Bézier piece covering
//! that instant, returning a reference to the piece and the piece-local time
//! in seconds.
//!
//! Piece data lives in the `CurvePool` — the Timeline stores only lightweight
//! cursors (pool slot + piece index + timing). No piece data is copied.
//!
//! # Architecture
//!
//! ```text
//! Timeline  get_piece(axis, now_cycles, pool) → (&Piece, t_local_sec)
//!   └─ Evaluator  eval_position(&Piece, t_local) → mm
//!       └─ Step dispatch  quantize → step pulses
//! ```
//!
//! The hot path is a u64 comparison against the cached piece end time.
//! When valid, the function dereferences a cached pointer to the piece in
//! the CurvePool, computes `t_local = (now - start) * inv_clock_hz`, and
//! returns. No pool lookup, no generation check on the hot path.
//!
//! # no_std
//!
//! This module is `no_std`-compatible. Fixed-size arrays only, no heap.

#![allow(unsafe_code)]

use crate::cubic_curve::LoadedCubicCurve;
use crate::curve_pool::{CurveHandle, CurvePool};
use crate::monomial::BezierPieceMonomial;

pub const N_AXES: usize = 4;

/// Per-axis cursor: tracks which piece we're currently evaluating.
///
/// The `curve_ptr` is a cached raw pointer into a CurvePool slot, resolved
/// once at segment load / piece advance time. The hot path dereferences it
/// directly — no atomic generation check, no pool lookup.
///
/// ISR-exclusive state. Only valid while the CurvePool slot's generation
/// matches (guaranteed by the host not retiring the slot until the ISR
/// advances past it).
#[derive(Clone, Copy, Debug)]
struct AxisCursor {
    /// Cached pointer to the LoadedCubicCurve in the CurvePool. Null when
    /// no curve is active for this axis.
    curve_ptr: *const LoadedCubicCurve,
    /// Index of the current piece within `curve.pieces[]`.
    piece_idx: u16,
    /// Number of pieces in the current curve (cached from `curve.piece_count`).
    piece_count: u16,
    /// Absolute start time of the current piece in MCU clock cycles.
    piece_start_cycles: u64,
    /// Absolute end time of the current piece (= start + duration_cycles).
    /// Half-open: the piece covers `[start, end)`.
    piece_end_cycles: u64,
}

impl AxisCursor {
    const fn empty() -> Self {
        Self {
            curve_ptr: core::ptr::null(),
            piece_idx: 0,
            piece_count: 0,
            piece_start_cycles: 0,
            piece_end_cycles: 0,
        }
    }

    fn is_active(&self) -> bool {
        !self.curve_ptr.is_null()
    }
}

/// Timeline: maps `(axis, now_cycles)` to the piece covering that instant.
///
/// Piece data is NOT owned — it lives in the `CurvePool`. The Timeline
/// holds only per-axis cursors (pointer + index + timing).
///
/// ISR-exclusive. Populate via `load_axis` under IRQ-disabled context.
/// After loading, the ISR calls `get_piece` on every tick.
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

    /// Load a curve for one axis. Resolves the handle through the pool,
    /// caches the pointer, and sets up timing for piece 0.
    ///
    /// `segment_start_cycles` is the absolute time when this segment starts
    /// (the segment's `t_start`). Piece 0 starts at this time; subsequent
    /// pieces chain from there using each piece's `duration`.
    ///
    /// Returns `true` if the axis was loaded, `false` if the handle was
    /// unused or the pool lookup failed (axis left idle).
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
        let Some(curve_ptr) = pool.lookup_active(handle) else {
            self.axes[axis] = AxisCursor::empty();
            return false;
        };
        // SAFETY: pool.lookup_active returned Some, meaning the slot's
        // generation matched. The ISR is the sole reader; the foreground
        // will not retire this slot until the ISR advances past it.
        let curve = unsafe { &*curve_ptr };
        if curve.piece_count == 0 {
            self.axes[axis] = AxisCursor::empty();
            return false;
        }
        let duration_cycles = (curve.pieces[0].duration * self.clock_hz) as u64;
        self.axes[axis] = AxisCursor {
            curve_ptr,
            piece_idx: 0,
            piece_count: curve.piece_count,
            piece_start_cycles: segment_start_cycles,
            piece_end_cycles: segment_start_cycles + duration_cycles,
        };
        true
    }

    /// Clear all axes. Used after cancel/flush/homing abort.
    pub fn reset(&mut self) {
        self.axes = [AxisCursor::empty(); N_AXES];
    }

    /// Clear a single axis.
    pub fn clear_axis(&mut self, axis: usize) {
        if axis < N_AXES {
            self.axes[axis] = AxisCursor::empty();
        }
    }

    /// Returns true if all axes are idle (no active curves).
    pub fn all_idle(&self) -> bool {
        self.axes.iter().all(|c| !c.is_active())
    }

    /// Returns true if the given axis has an active curve.
    pub fn axis_active(&self, axis: usize) -> bool {
        axis < N_AXES && self.axes[axis].is_active()
    }

    /// Resolve `now_cycles` to the piece covering that instant on `axis`.
    ///
    /// Returns `Some((&piece, t_local_sec))` on hit, `None` when idle or
    /// all pieces exhausted.
    ///
    /// # Hot path
    ///
    /// When `now_cycles < piece_end_cycles`: one pointer dereference into
    /// the CurvePool, one u64 subtract, one f32 multiply. No pool lookup,
    /// no atomic load, no scanning.
    pub fn get_piece(
        &mut self,
        axis: usize,
        now_cycles: u64,
    ) -> Option<(&BezierPieceMonomial, f32)> {
        if axis >= N_AXES {
            return None;
        }
        let cursor = &mut self.axes[axis];
        if !cursor.is_active() {
            return None;
        }

        // Hot path: current piece still covers now.
        if now_cycles < cursor.piece_end_cycles {
            let delta = now_cycles.saturating_sub(cursor.piece_start_cycles);
            let t_local = delta as f32 * self.inv_clock_hz;
            // SAFETY: curve_ptr was validated at load_axis time. The pool
            // slot's generation is still valid (the foreground cannot retire
            // it while the ISR references it). piece_idx < piece_count was
            // established at load and maintained by advance_piece.
            let piece = unsafe {
                &(*cursor.curve_ptr).pieces[cursor.piece_idx as usize]
            };
            return Some((piece, t_local));
        }

        // Slow path: advance through pieces.
        self.advance_piece(axis, now_cycles)
    }

    /// Advance the cursor to the next piece(s) until we find one covering
    /// `now_cycles`, or exhaust the curve.
    fn advance_piece(
        &mut self,
        axis: usize,
        now_cycles: u64,
    ) -> Option<(&BezierPieceMonomial, f32)> {
        let clock_hz = self.clock_hz;
        let inv_hz = self.inv_clock_hz;
        let cursor = &mut self.axes[axis];

        loop {
            cursor.piece_idx += 1;
            if cursor.piece_idx >= cursor.piece_count {
                // Curve exhausted — axis goes idle.
                *cursor = AxisCursor::empty();
                return None;
            }

            // SAFETY: same as get_piece — curve_ptr valid, piece_idx < piece_count.
            let piece = unsafe {
                &(*cursor.curve_ptr).pieces[cursor.piece_idx as usize]
            };
            cursor.piece_start_cycles = cursor.piece_end_cycles;
            let duration_cycles = (piece.duration * clock_hz) as u64;
            cursor.piece_end_cycles = cursor.piece_start_cycles + duration_cycles;

            if now_cycles < cursor.piece_end_cycles {
                let delta = now_cycles.saturating_sub(cursor.piece_start_cycles);
                return Some((piece, delta as f32 * inv_hz));
            }
            // This piece is also in the past — keep advancing.
        }
    }
}

// Test-only helpers. Tests can't use CurvePool easily (it's tightly coupled
// to the slot/generation machinery), so we provide a way to set up cursors
// from raw piece data for unit testing.
#[cfg(any(test, feature = "host"))]
impl Timeline {
    /// Test helper: load a single piece directly (no CurvePool).
    /// The piece data must outlive the Timeline.
    ///
    /// # Safety
    /// `curve` must point to a valid `LoadedCubicCurve` that outlives
    /// all subsequent `get_piece` calls on this axis.
    pub unsafe fn test_load_axis_raw(
        &mut self,
        axis: usize,
        curve: *const LoadedCubicCurve,
        piece_count: u16,
        segment_start_cycles: u64,
        clock_hz: f32,
    ) {
        if axis >= N_AXES || piece_count == 0 {
            return;
        }
        let first_duration = unsafe { (*curve).pieces[0].duration };
        let duration_cycles = (first_duration * clock_hz) as u64;
        self.axes[axis] = AxisCursor {
            curve_ptr: curve,
            piece_idx: 0,
            piece_count,
            piece_start_cycles: segment_start_cycles,
            piece_end_cycles: segment_start_cycles + duration_cycles,
        };
    }
}
