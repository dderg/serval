#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommitOutcome {
    Applied,
    Stale,
    Overcommit,
}

/// ## Cursor invariants (ISR/host safety boundary)
///
/// - `head` — monotonic valid frontier (wrapping u32). Advanced **only** by
///   `commit_head`; `write_slot` does **not** advance it.
/// - `retired` — monotonic retire counter (wrapping u32). Incremented one per
///   `advance_counter`. Purely a flow-control frontier.
/// - `tail` — physical read cursor in `[0, ring_depth)`. Invariant:
///   `tail == retired % ring_depth` — both advance only in `advance_counter`
///   starting from 0, so no division is needed on the hot path.
///
/// Occupancy: `head.wrapping_sub(retired)`.
#[derive(Debug, Clone, Copy)]
pub struct RingDescriptor {
    pub ring_offset: usize,
    pub ring_depth: usize,
    pub head: u32,
    pub retired: u32,
    pub tail: usize,
}

impl RingDescriptor {
    #[inline]
    pub const fn new_unconfigured() -> Self {
        Self {
            ring_offset: 0,
            ring_depth: 0,
            head: 0,
            retired: 0,
            tail: 0,
        }
    }

    #[inline]
    pub const fn new(offset: usize, depth: usize) -> Self {
        Self {
            ring_offset: offset,
            ring_depth: depth,
            head: 0,
            retired: 0,
            tail: 0,
        }
    }

    #[inline]
    pub fn is_configured(&self) -> bool {
        self.ring_depth > 0
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.head.wrapping_sub(self.retired) as usize
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.head == self.retired
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.len() == self.ring_depth
    }

    #[inline]
    pub fn write_slot(&self, storage: &mut [PieceEntry], physical_slot: usize, entry: PieceEntry) {
        if self.ring_depth == 0 || physical_slot >= self.ring_depth {
            return;
        }
        debug_assert!(
            self.ring_offset + physical_slot < storage.len(),
            "ring slot out of storage bounds"
        );
        // SAFETY: `configure_axis` guarantees `ring_offset + ring_depth <=
        // storage.len()`; `physical_slot < ring_depth` is checked by the guard
        // at the top of this function, so `ring_offset + physical_slot <
        // storage.len()` holds unconditionally.
        #[allow(clippy::indexing_slicing)]
        {
            storage[self.ring_offset + physical_slot] = entry;
        }
    }

    #[inline]
    pub fn commit_head(&mut self, new_head: u32) -> CommitOutcome {
        let cur = self.head.wrapping_sub(self.retired);
        let proposed = new_head.wrapping_sub(self.retired);
        if proposed <= cur {
            return CommitOutcome::Stale;
        }
        if proposed > self.ring_depth as u32 {
            return CommitOutcome::Overcommit;
        }
        self.head = new_head;
        CommitOutcome::Applied
    }

    #[inline]
    pub fn push(&mut self, storage: &mut [PieceEntry], entry: PieceEntry) -> Result<(), ()> {
        if self.is_full() || self.ring_depth == 0 {
            return Err(());
        }
        let physical_slot = (self.head as usize) % self.ring_depth;
        self.write_slot(storage, physical_slot, entry);
        self.head = self.head.wrapping_add(1);
        Ok(())
    }

    #[inline]
    pub fn front_slot(&self) -> Option<usize> {
        if self.is_empty() {
            return None;
        }
        Some(self.ring_offset + self.tail)
    }

    #[inline]
    pub fn peek<'s>(&self, storage: &'s [PieceEntry]) -> Option<&'s PieceEntry> {
        if self.is_empty() {
            return None;
        }
        storage.get(self.ring_offset + self.tail)
    }

    /// Storage index of the k-th unretired entry (0 = front), without
    /// consuming anything — lookahead readers must not retire pieces the
    /// realtime cursor has not passed yet.
    #[inline]
    pub fn slot_at(&self, k: usize) -> Option<usize> {
        if k >= self.len() {
            return None;
        }
        Some(self.ring_offset + (self.tail + k) % self.ring_depth)
    }

    #[inline]
    pub fn advance_counter(&mut self) {
        if self.ring_depth == 0 || self.is_empty() {
            return;
        }
        self.retired = self.retired.wrapping_add(1);
        self.tail += 1;
        if self.tail >= self.ring_depth {
            self.tail = 0;
        }
    }

    /// Touches only consumer-owned cursors (`retired`, `tail`) — never `head`.
    #[inline]
    pub fn drain(&mut self) {
        if self.ring_depth == 0 {
            return;
        }
        self.retired = self.head;
        self.tail = (self.head as usize) % self.ring_depth;
    }

    #[inline]
    pub fn retired_count(&self) -> u32 {
        self.retired
    }
}

/// Maximum Chebyshev coefficients per piece (degree ≤ 7).
pub const MAX_PIECE_COEFFS: usize = 8;
/// Full ring-slot size — also the maximum wire entry length.
pub const PIECE_ENTRY_BYTES: usize = 48;
/// Wire entry header: everything before the coefficient block.
pub const PIECE_WIRE_HEADER_LEN: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PieceWireError {
    BadCoeffCount(u8),
    Truncated { need: usize, have: usize },
}

/// Layout contract (C ABI, matches `mcu_protocol_schema.h`; the wire entry is
/// a truncated prefix of the ring slot — header plus `coeff_count` f32s):
///
/// ```text
/// offset  0 ..  8 : start_time  (u64, little-endian MCU clock cycles)
/// offset  8 .. 12 : duration    (f32, piece duration in seconds)
/// offset 12       : motor_mask  (u8, 0 => normal full-axis move)
/// offset 13       : coeff_count (u8, 1..=8 — fail loud outside)
/// offset 14 .. 16 : _reserved   ([u8; 2], must be zero)
/// offset 16 .. 48 : coeffs      ([f32; 8], Chebyshev on u ∈ [−1, 1],
///                                u = 2·(t − start)/duration − 1)
/// total           : 48 bytes, align 8
/// ```
///
/// # Example
///
/// ```rust
/// use runtime::piece_ring::PieceEntry;
///
/// // pos(u) = 50 + 50·u over 10 ms: 0 → 100 at a constant 10 m/s.
/// let mut entry = PieceEntry::zeroed();
/// entry.duration = 0.01;
/// entry.coeff_count = 2;
/// entry.coeffs[0] = 50.0;
/// entry.coeffs[1] = 50.0;
/// assert_eq!(entry.pos_start(), 0.0);
/// assert_eq!(entry.pos_end(), 100.0);
/// assert_eq!(entry.vel_end(), 10_000.0);
/// assert_eq!(entry.wire_len(), 24);
/// ```
#[derive(Clone, Copy, Debug)]
#[repr(C, align(8))]
pub struct PieceEntry {
    pub start_time: u64,
    pub duration: f32,
    pub motor_mask: u8,
    pub coeff_count: u8,
    #[allow(clippy::pub_underscore_fields)]
    pub _reserved: [u8; 2],
    pub coeffs: [f32; MAX_PIECE_COEFFS],
}

const _: () = {
    assert!(core::mem::size_of::<PieceEntry>() == PIECE_ENTRY_BYTES);
    assert!(core::mem::align_of::<PieceEntry>() == 8);
    assert!(core::mem::offset_of!(PieceEntry, duration) == 8);
    assert!(core::mem::offset_of!(PieceEntry, motor_mask) == 12);
    assert!(core::mem::offset_of!(PieceEntry, coeff_count) == 13);
    assert!(core::mem::offset_of!(PieceEntry, coeffs) == PIECE_WIRE_HEADER_LEN);
};

#[inline]
#[allow(clippy::cast_possible_truncation)]
pub fn stepper_sel_from_mask(mask: u8) -> Result<u8, ()> {
    if mask == 0 {
        return Ok(0);
    }
    if mask.count_ones() != 1 {
        return Err(());
    }
    Ok(mask.trailing_zeros() as u8 + 1)
}

impl PieceEntry {
    #[inline]
    #[must_use]
    pub const fn zeroed() -> Self {
        Self {
            start_time: 0,
            duration: 0.0,
            motor_mask: 0,
            coeff_count: 1,
            _reserved: [0; 2],
            coeffs: [0.0; MAX_PIECE_COEFFS],
        }
    }

    /// Bytes this entry occupies on the wire — the header plus only the
    /// coefficients actually present.
    #[inline]
    #[must_use]
    pub const fn wire_len(&self) -> usize {
        PIECE_WIRE_HEADER_LEN + 4 * self.coeff_count as usize
    }

    /// `end = start_time + ⌊duration × clock_freq⌋`
    ///
    /// The cast truncates toward zero — the ISR advances to the next piece when
    /// `current_time >= end_time`, so truncating ensures we never overshoot
    /// by a fractional cycle.
    #[inline]
    pub fn end_time(&self, clock_freq: f32) -> u64 {
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let cycles = (self.duration * clock_freq) as u64;
        self.start_time + cycles
    }

    fn live_coeffs(&self) -> &[f32] {
        let n = (self.coeff_count as usize).clamp(1, MAX_PIECE_COEFFS);
        self.coeffs.get(..n).unwrap_or(&self.coeffs)
    }

    /// Position at the piece start: `T_k(−1) = (−1)^k`.
    #[inline]
    pub fn pos_start(&self) -> f32 {
        let mut sum = 0.0;
        let mut sign = 1.0_f32;
        for &a in self.live_coeffs() {
            sum += sign * a;
            sign = -sign;
        }
        sum
    }

    /// Position at the piece end: `T_k(1) = 1`.
    #[inline]
    pub fn pos_end(&self) -> f32 {
        self.live_coeffs().iter().sum()
    }

    /// Velocity at the piece start: `T_k′(−1) = (−1)^{k+1}·k²`, × `2/duration`.
    #[inline]
    #[allow(clippy::cast_precision_loss)]
    pub fn vel_start(&self) -> f32 {
        if self.duration <= 0.0 {
            return 0.0;
        }
        let mut sum = 0.0;
        let mut sign = -1.0_f32;
        for (k, &a) in self.live_coeffs().iter().enumerate() {
            sum += sign * (k * k) as f32 * a;
            sign = -sign;
        }
        sum * 2.0 / self.duration
    }

    /// Velocity at the piece end: `T_k′(1) = k²`, × `2/duration`.
    #[inline]
    #[allow(clippy::cast_precision_loss)]
    pub fn vel_end(&self) -> f32 {
        if self.duration <= 0.0 {
            return 0.0;
        }
        let mut sum = 0.0;
        for (k, &a) in self.live_coeffs().iter().enumerate() {
            sum += (k * k) as f32 * a;
        }
        sum * 2.0 / self.duration
    }

    /// # Example
    ///
    /// ```rust
    /// use runtime::piece_ring::PieceEntry;
    ///
    /// let p = PieceEntry { start_time: 1, duration: 0.001, ..PieceEntry::zeroed() };
    /// let b = p.to_le_bytes();
    /// assert_eq!(b.len(), 48);
    /// assert_eq!(&b[0..8], &1u64.to_le_bytes());
    /// ```
    #[inline]
    pub fn to_le_bytes(&self) -> [u8; PIECE_ENTRY_BYTES] {
        let mut b = [0u8; PIECE_ENTRY_BYTES];
        b[0..8].copy_from_slice(&self.start_time.to_le_bytes());
        b[8..12].copy_from_slice(&self.duration.to_le_bytes());
        b[12] = self.motor_mask;
        b[13] = self.coeff_count;
        b[14..16].copy_from_slice(&self._reserved);
        for (k, c) in self.coeffs.iter().enumerate() {
            let off = PIECE_WIRE_HEADER_LEN + 4 * k;
            if let Some(dst) = b.get_mut(off..off + 4) {
                dst.copy_from_slice(&c.to_le_bytes());
            }
        }
        b
    }

    #[inline]
    pub fn from_le_bytes(b: &[u8; PIECE_ENTRY_BYTES]) -> Self {
        let rd4 = |off: usize| {
            let mut w = [0u8; 4];
            if let Some(src) = b.get(off..off + 4) {
                w.copy_from_slice(src);
            }
            f32::from_le_bytes(w)
        };
        let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
        for (k, c) in coeffs.iter_mut().enumerate() {
            *c = rd4(PIECE_WIRE_HEADER_LEN + 4 * k);
        }
        Self {
            start_time: u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]),
            duration: rd4(8),
            motor_mask: b[12],
            coeff_count: b[13],
            _reserved: [b[14], b[15]],
            coeffs,
        }
    }

    /// Append this entry's wire form — the 16-byte header followed by exactly
    /// `coeff_count` coefficients.
    #[cfg(feature = "host")]
    pub fn to_wire_bytes(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.start_time.to_le_bytes());
        out.extend_from_slice(&self.duration.to_le_bytes());
        out.push(self.motor_mask);
        out.push(self.coeff_count);
        out.extend_from_slice(&self._reserved);
        for c in self.live_coeffs() {
            out.extend_from_slice(&c.to_le_bytes());
        }
    }

    /// Parse one variable-length wire entry from the front of `bytes`,
    /// zero-filling the coefficient tail. Returns the entry and its consumed
    /// wire length. The single wire parser — mcu-protocol decode walking and
    /// the EtherCAT ring both route through here.
    pub fn parse_wire(bytes: &[u8]) -> Result<(Self, usize), PieceWireError> {
        let header = bytes
            .get(..PIECE_WIRE_HEADER_LEN)
            .ok_or(PieceWireError::Truncated {
                need: PIECE_WIRE_HEADER_LEN,
                have: bytes.len(),
            })?;
        let coeff_count = header.get(13).copied().unwrap_or(0);
        if coeff_count == 0 || coeff_count as usize > MAX_PIECE_COEFFS {
            return Err(PieceWireError::BadCoeffCount(coeff_count));
        }
        let wire_len = PIECE_WIRE_HEADER_LEN + 4 * coeff_count as usize;
        if bytes.len() < wire_len {
            return Err(PieceWireError::Truncated {
                need: wire_len,
                have: bytes.len(),
            });
        }
        let mut full = [0u8; PIECE_ENTRY_BYTES];
        if let (Some(dst), Some(src)) = (full.get_mut(..wire_len), bytes.get(..wire_len)) {
            dst.copy_from_slice(src);
        }
        Ok((Self::from_le_bytes(&full), wire_len))
    }
}

#[cfg(test)]
mod tests;
