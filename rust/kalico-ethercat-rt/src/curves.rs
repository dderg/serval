//! Curve storage + single-channel piece-walking evaluator.

#![allow(unsafe_code)]

use runtime::cubic_curve::{LoadedCubicCurve, WirePiece};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::monomial::eval_position_velocity;

use crate::wire::wire_pieces_from_bytes;

pub struct CurveStore {
    pool: CurvePool,
}

#[derive(Debug, PartialEq, Eq)]
pub enum LoadError {
    BadPieceBytes,
    PoolReject,
}

impl CurveStore {
    pub fn new() -> Self {
        Self { pool: CurvePool::new() }
    }

    /// Load a curve into `slot_idx`; return the packed handle to put in a response.
    pub fn load(&self, slot_idx: u16, piece_count: u8, pieces_bytes: &[u8]) -> Result<u32, LoadError> {
        let wire: Vec<WirePiece> =
            wire_pieces_from_bytes(piece_count, pieces_bytes).map_err(|_| LoadError::BadPieceBytes)?;
        let handle =
            self.pool.try_alloc_and_load(slot_idx as usize, &wire).ok_or(LoadError::PoolReject)?;
        Ok(handle.pack())
    }

    /// Borrow a loaded curve by packed handle. Returns None if stale/empty.
    ///
    /// SAFETY: the pool slot is not mutated while we hold this in the
    /// single-threaded DC loop.
    pub fn with_curve<R>(&self, handle_packed: u32, f: impl FnOnce(&LoadedCubicCurve) -> R) -> Option<R> {
        let handle = CurveHandle::unpack(handle_packed);
        let ptr = self.pool.lookup_active(handle)?;
        // SAFETY: pointer valid for the lifetime of this call; no concurrent mutation.
        let curve: &LoadedCubicCurve = unsafe { &*ptr };
        Some(f(curve))
    }

    pub fn reset(&self) {
        self.pool.reset_all_retired_to_current();
    }
}

impl Default for CurveStore {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for CurveStore {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CurveStore").finish_non_exhaustive()
    }
}

impl core::fmt::Debug for ChannelTrack {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ChannelTrack")
            .field("handle_packed", &self.handle_packed)
            .field("t_start_ns", &self.t_start_ns)
            .field("t_end_ns", &self.t_end_ns)
            .field("cursor", &self.cursor)
            .field("piece_start_ns", &self.piece_start_ns)
            .finish()
    }
}

/// Evaluate position (mm) of a loaded curve at a given piece cursor + piece-local seconds.
pub fn eval_curve_at(curve: &LoadedCubicCurve, cursor: usize, t_local_s: f32) -> f32 {
    let (pos, _vel) = eval_position_velocity(&curve.pieces[cursor], t_local_s);
    pos
}

/// Active-segment state for one channel.
pub struct ChannelTrack {
    handle_packed: u32,
    t_start_ns: u64,
    t_end_ns: u64,
    cursor: usize,
    piece_start_ns: u64,
}

impl ChannelTrack {
    pub fn arm(handle_packed: u32, t_start_ns: u64, t_end_ns: u64) -> Self {
        Self { handle_packed, t_start_ns, t_end_ns, cursor: 0, piece_start_ns: t_start_ns }
    }

    pub fn is_done(&self, now_ns: u64) -> bool {
        now_ns >= self.t_end_ns
    }

    /// Advance the cursor past elapsed pieces and return current position (mm).
    /// Returns None if the curve is gone or the cursor is exhausted.
    pub fn sample(&mut self, store: &CurveStore, now_ns: u64) -> Option<f32> {
        if now_ns < self.t_start_ns {
            return store.with_curve(self.handle_packed, |c| eval_curve_at(c, 0, 0.0));
        }
        loop {
            // Get current piece duration; break when cursor is at or past the
            // last piece — the caller clamps t_local_s to piece duration below.
            let piece_info = store.with_curve(self.handle_packed, |c| {
                let count = c.piece_count as usize;
                if self.cursor + 1 >= count {
                    // On the last piece: don't advance further.
                    None
                } else {
                    Some(c.pieces[self.cursor].duration)
                }
            })?; // None if handle is stale — propagate
            let Some(dur_s) = piece_info else { break };
            let dur_ns = (dur_s as f64 * 1e9) as u64;
            if now_ns.saturating_sub(self.piece_start_ns) >= dur_ns && dur_ns > 0 {
                self.piece_start_ns += dur_ns;
                self.cursor += 1;
            } else {
                break;
            }
        }
        let t_local_s = (now_ns.saturating_sub(self.piece_start_ns)) as f32 / 1e9;
        let cursor = self.cursor;
        store.with_curve(self.handle_packed, |c| {
            let idx = cursor.min(c.piece_count as usize - 1);
            eval_curve_at(c, idx, t_local_s.min(c.pieces[idx].duration))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one_piece_bytes(bp: [f32; 4], dur: f32) -> (u8, Vec<u8>) {
        let mut v = Vec::new();
        for x in bp {
            v.extend_from_slice(&x.to_bits().to_le_bytes());
        }
        v.extend_from_slice(&dur.to_bits().to_le_bytes());
        (1, v)
    }

    #[test]
    fn ease_curve_endpoints() {
        let store = CurveStore::new();
        // Bernstein [0,0,10,10] => smooth 0->10 with zero velocity at both ends.
        let (pc, bytes) = one_piece_bytes([0.0, 0.0, 10.0, 10.0], 1.0);
        let handle = store.load(0, pc, &bytes).unwrap();

        let p0 = store.with_curve(handle, |c| eval_curve_at(c, 0, 0.0)).unwrap();
        let p1 = store.with_curve(handle, |c| eval_curve_at(c, 0, 1.0)).unwrap();
        let pmid = store.with_curve(handle, |c| eval_curve_at(c, 0, 0.5)).unwrap();

        assert!((p0 - 0.0).abs() < 1e-4, "start={p0}");
        assert!((p1 - 10.0).abs() < 1e-3, "end={p1}");
        assert!((pmid - 5.0).abs() < 1e-3, "mid={pmid}"); // symmetric ease => exactly half
    }
}

#[cfg(test)]
mod walk_tests {
    use super::*;

    fn two_piece_bytes() -> (u8, Vec<u8>) {
        // piece 0: 0->10 (ease), 1s ; piece 1: 10->0 (ease), 1s
        let mut v = Vec::new();
        for x in [0.0f32, 0.0, 10.0, 10.0] {
            v.extend_from_slice(&x.to_bits().to_le_bytes());
        }
        v.extend_from_slice(&1.0f32.to_bits().to_le_bytes());
        for x in [10.0f32, 10.0, 0.0, 0.0] {
            v.extend_from_slice(&x.to_bits().to_le_bytes());
        }
        v.extend_from_slice(&1.0f32.to_bits().to_le_bytes());
        (2, v)
    }

    #[test]
    fn walks_two_pieces_continuously() {
        let store = CurveStore::new();
        let (pc, bytes) = two_piece_bytes();
        let handle = store.load(0, pc, &bytes).unwrap();
        let t0 = 1_000_000_000u64; // 1s in ns
        let mut track = ChannelTrack::arm(handle, t0, t0 + 2_000_000_000);

        let at = |track: &mut ChannelTrack, off_ns: u64| track.sample(&store, t0 + off_ns).unwrap();

        assert!((at(&mut track, 0) - 0.0).abs() < 1e-3);              // start of piece 0
        assert!((at(&mut track, 1_000_000_000) - 10.0).abs() < 1e-2); // boundary -> piece 1 start = 10
        assert!((at(&mut track, 2_000_000_000) - 0.0).abs() < 1e-2);  // end of piece 1
        assert!(track.is_done(t0 + 2_000_000_000));
    }
}
