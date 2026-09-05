// The universal trajectory currency: a uniformly-spaced run of positions.
//
// A `SampleRun` is `count` positions on one lane, the first at `start_clock`
// and each subsequent one `interval_ticks` later. The interval is uniform
// inside a run but free to change between runs, so a resonance buzz or an
// impulse shaper may raise the rate for a stretch without renegotiating the
// lane. `count == 1` is a legal run.
//
// Runs abut exactly: the next run's `start_clock` equals the previous run's
// `end_clock`. A hole in the stream is a bug in whatever produced it, so
// `LaneCursor::accept` reports it as a fault instead of padding or clamping
// the stream back into shape. Crossing a real discontinuity requires an
// explicit `LaneCursor::anchor`, which is exactly what the `sample_anchor`
// wire command carries.
//
// Positions are per-motor fixed point, declared at config time: a phase lane
// counts LUT phase quanta (1024 per electrical cycle, so one 256-microstep is
// one quantum), an EtherCAT lane counts drive counts.
//
// Compiles for both `no_std` MCU targets and the host, and allocates nothing.

use core::iter::Iterator;

/// Wire ceiling on one `sample_run` payload. A Klipper block payload is 59
/// bytes; msgid, oid, the varint interval, the count and the buffer length
/// claim the rest.
pub const SAMPLE_RUN_DATA_MAX: usize = 48;

/// Wire ceiling on the samples in one `sample_run`. Even all-1-byte deltas
/// cannot outrun [`SAMPLE_RUN_DATA_MAX`], so this is the count cap the
/// encoder stops at, and it fits the `count=%c` argument.
pub const SAMPLE_RUN_COUNT_MAX: usize = SAMPLE_RUN_DATA_MAX;

/// Widest varint a single in-range delta can occupy: 32 magnitude bits plus
/// the zigzag sign bit, seven bits to a byte.
pub const SAMPLE_DELTA_BYTES_MAX: usize = 5;

pub const FAULT_SAMPLE_NOT_ANCHORED: u16 = 1;
pub const FAULT_SAMPLE_DISCONTINUITY: u16 = 2;
pub const FAULT_SAMPLE_ZERO_COUNT: u16 = 3;
pub const FAULT_SAMPLE_ZERO_INTERVAL: u16 = 4;
pub const FAULT_SAMPLE_SPAN_OVERFLOW: u16 = 5;
pub const FAULT_SAMPLE_CAPACITY: u16 = 6;
pub const FAULT_SAMPLE_COUNT_MISMATCH: u16 = 7;
pub const FAULT_SAMPLE_DELTA_OVERFLOW: u16 = 8;
pub const FAULT_SAMPLE_POSITION_OVERFLOW: u16 = 9;
pub const FAULT_SAMPLE_TRUNCATED: u16 = 10;
pub const FAULT_SAMPLE_TRAILING: u16 = 11;
pub const FAULT_SAMPLE_COUNT_EXCEEDED: u16 = 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleRunError {
    NotAnchored {
        start_clock: u64,
    },
    Discontinuity {
        expected_clock: u64,
        start_clock: u64,
    },
    ZeroCount {
        start_clock: u64,
    },
    ZeroInterval {
        start_clock: u64,
    },
    SpanOverflow {
        start_clock: u64,
        interval_ticks: u32,
        count: u16,
    },
    Capacity {
        capacity: usize,
    },
    CountMismatch {
        count: u16,
        samples: usize,
    },
    CountExceeded {
        count: usize,
        cap: usize,
    },
    DeltaOverflow {
        index: usize,
        delta: i64,
    },
    PositionOverflow {
        index: usize,
        position: i64,
    },
    Truncated {
        index: usize,
    },
    Trailing {
        consumed: usize,
        len: usize,
    },
}

impl SampleRunError {
    pub const fn fault_code(&self) -> u16 {
        match self {
            Self::NotAnchored { .. } => FAULT_SAMPLE_NOT_ANCHORED,
            Self::Discontinuity { .. } => FAULT_SAMPLE_DISCONTINUITY,
            Self::ZeroCount { .. } => FAULT_SAMPLE_ZERO_COUNT,
            Self::ZeroInterval { .. } => FAULT_SAMPLE_ZERO_INTERVAL,
            Self::SpanOverflow { .. } => FAULT_SAMPLE_SPAN_OVERFLOW,
            Self::Capacity { .. } => FAULT_SAMPLE_CAPACITY,
            Self::CountMismatch { .. } => FAULT_SAMPLE_COUNT_MISMATCH,
            Self::CountExceeded { .. } => FAULT_SAMPLE_COUNT_EXCEEDED,
            Self::DeltaOverflow { .. } => FAULT_SAMPLE_DELTA_OVERFLOW,
            Self::PositionOverflow { .. } => FAULT_SAMPLE_POSITION_OVERFLOW,
            Self::Truncated { .. } => FAULT_SAMPLE_TRUNCATED,
            Self::Trailing { .. } => FAULT_SAMPLE_TRAILING,
        }
    }

    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::NotAnchored { .. } => "sample run before any anchor",
            Self::Discontinuity { .. } => "sample run does not abut previous run",
            Self::ZeroCount { .. } => "sample run carries no samples",
            Self::ZeroInterval { .. } => "sample run interval is zero",
            Self::SpanOverflow { .. } => "sample run span overflows the clock",
            Self::Capacity { .. } => "sample run exceeds lane capacity",
            Self::CountMismatch { .. } => "sample run count disagrees with its samples",
            Self::CountExceeded { .. } => "sample run count exceeds the wire cap",
            Self::DeltaOverflow { .. } => "sample delta does not fit the wire",
            Self::PositionOverflow { .. } => "sample position overflows the lane",
            Self::Truncated { .. } => "sample run payload ends mid-delta",
            Self::Trailing { .. } => "sample run payload has trailing bytes",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct SampleRunHeader {
    pub start_clock: u64,
    pub interval_ticks: u32,
    pub count: u16,
}

impl SampleRunHeader {
    pub const fn new(start_clock: u64, interval_ticks: u32, count: u16) -> Self {
        Self {
            start_clock,
            interval_ticks,
            count,
        }
    }

    pub const fn span_ticks(&self) -> u64 {
        self.interval_ticks as u64 * self.count as u64
    }

    /// The clock the next abutting run must start at.
    pub const fn end_clock(&self) -> u64 {
        self.start_clock.wrapping_add(self.span_ticks())
    }

    /// The clock of the last sample this run carries.
    pub const fn last_sample_clock(&self) -> u64 {
        self.start_clock
            .wrapping_add(self.interval_ticks as u64 * (self.count as u64).saturating_sub(1))
    }

    fn validate(&self) -> Result<(), SampleRunError> {
        if self.count == 0 {
            return Err(SampleRunError::ZeroCount {
                start_clock: self.start_clock,
            });
        }
        if self.interval_ticks == 0 {
            return Err(SampleRunError::ZeroInterval {
                start_clock: self.start_clock,
            });
        }
        if self.start_clock.checked_add(self.span_ticks()).is_none() {
            return Err(SampleRunError::SpanOverflow {
                start_clock: self.start_clock,
                interval_ticks: self.interval_ticks,
                count: self.count,
            });
        }
        Ok(())
    }
}

/// A borrowed run: the currency every consumer reads.
#[derive(Debug, Clone, Copy)]
pub struct SampleRunView<'a> {
    header: SampleRunHeader,
    samples: &'a [i32],
}

impl<'a> SampleRunView<'a> {
    pub fn new(header: SampleRunHeader, samples: &'a [i32]) -> Result<Self, SampleRunError> {
        header.validate()?;
        if usize::from(header.count) != samples.len() {
            return Err(SampleRunError::CountMismatch {
                count: header.count,
                samples: samples.len(),
            });
        }
        Ok(Self { header, samples })
    }

    pub const fn header(&self) -> SampleRunHeader {
        self.header
    }

    pub const fn samples(&self) -> &'a [i32] {
        self.samples
    }

    pub fn iter(&self) -> impl Iterator<Item = (u64, i32)> + 'a {
        let start_clock = self.header.start_clock;
        let interval = u64::from(self.header.interval_ticks);
        self.samples
            .iter()
            .copied()
            .enumerate()
            .map(move |(index, position)| {
                (start_clock.wrapping_add(interval * index as u64), position)
            })
    }

    pub fn first_position(&self) -> Option<i32> {
        self.samples.first().copied()
    }

    pub fn last_position(&self) -> Option<i32> {
        self.samples.last().copied()
    }

    /// Fold an additive overlay run onto this one in place. Overlay runs share
    /// the lane's clock grid, so their headers must match exactly — the
    /// relativization that makes the overlay additive happens where the
    /// overlay is produced, mirroring `OverlayFrame`.
    pub fn overlay_onto(
        base: &mut [i32],
        base_header: SampleRunHeader,
        overlay: &SampleRunView<'_>,
    ) -> Result<(), SampleRunError> {
        if base_header != overlay.header {
            return Err(SampleRunError::Discontinuity {
                expected_clock: base_header.start_clock,
                start_clock: overlay.header.start_clock,
            });
        }
        if usize::from(base_header.count) != base.len() {
            return Err(SampleRunError::CountMismatch {
                count: base_header.count,
                samples: base.len(),
            });
        }
        for (index, (slot, nudge)) in base.iter_mut().zip(overlay.samples.iter()).enumerate() {
            let sum = i64::from(*slot) + i64::from(*nudge);
            *slot = i32::try_from(sum).map_err(|_| SampleRunError::PositionOverflow {
                index,
                position: sum,
            })?;
        }
        Ok(())
    }
}

/// An owned, fixed-capacity run. C owns the storage on the MCU; this mirror is
/// `#[repr(C)]` so the seam is a plain struct pointer.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SampleRunBuf<const CAP: usize> {
    samples: [i32; CAP],
    start_clock: u64,
    interval_ticks: u32,
    len: u16,
}

impl<const CAP: usize> SampleRunBuf<CAP> {
    pub const fn new(start_clock: u64, interval_ticks: u32) -> Self {
        Self {
            samples: [0; CAP],
            start_clock,
            interval_ticks,
            len: 0,
        }
    }

    pub fn reset(&mut self, start_clock: u64, interval_ticks: u32) {
        self.start_clock = start_clock;
        self.interval_ticks = interval_ticks;
        self.len = 0;
    }

    pub const fn start_clock(&self) -> u64 {
        self.start_clock
    }

    pub const fn interval_ticks(&self) -> u32 {
        self.interval_ticks
    }

    pub const fn len(&self) -> usize {
        self.len as usize
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub const fn is_full(&self) -> bool {
        self.len as usize >= CAP
    }

    pub const fn capacity(&self) -> usize {
        CAP
    }

    /// The clock the next pushed sample will carry.
    pub const fn next_clock(&self) -> u64 {
        self.start_clock
            .wrapping_add(self.interval_ticks as u64 * self.len as u64)
    }

    pub fn push(&mut self, position: i32) -> Result<(), SampleRunError> {
        let slot = self
            .samples
            .get_mut(self.len as usize)
            .ok_or(SampleRunError::Capacity { capacity: CAP })?;
        *slot = position;
        self.len += 1;
        Ok(())
    }

    pub fn header(&self) -> SampleRunHeader {
        SampleRunHeader::new(self.start_clock, self.interval_ticks, self.len)
    }

    pub fn samples(&self) -> &[i32] {
        self.samples.get(..self.len as usize).unwrap_or(&[])
    }

    pub fn view(&self) -> Result<SampleRunView<'_>, SampleRunError> {
        SampleRunView::new(self.header(), self.samples())
    }

    pub fn last_position(&self) -> Option<i32> {
        self.samples().last().copied()
    }
}

/// Per-lane abutment state: where the stream is anchored and where the next
/// run must start. One cursor per lane, and a separate one per overlay lane.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LaneCursor {
    next_clock: Option<u64>,
    position: i32,
}

impl LaneCursor {
    pub const fn new() -> Self {
        Self {
            next_clock: None,
            position: 0,
        }
    }

    /// Cross a sanctioned discontinuity: the stream restarts at `clock` from
    /// absolute `position`. Everything the cursor expected is discarded.
    pub fn anchor(&mut self, clock: u64, position: i32) {
        self.next_clock = Some(clock);
        self.position = position;
    }

    pub fn unanchor(&mut self) {
        self.next_clock = None;
    }

    pub const fn is_anchored(&self) -> bool {
        self.next_clock.is_some()
    }

    pub const fn next_clock(&self) -> Option<u64> {
        self.next_clock
    }

    pub const fn position(&self) -> i32 {
        self.position
    }

    /// Admit a run onto the lane. Fails loudly on an unanchored lane, a
    /// clock hole, or a degenerate header; never repairs the stream.
    pub fn accept(&mut self, header: &SampleRunHeader) -> Result<(), SampleRunError> {
        header.validate()?;
        let expected_clock = self.next_clock.ok_or(SampleRunError::NotAnchored {
            start_clock: header.start_clock,
        })?;
        if expected_clock != header.start_clock {
            return Err(SampleRunError::Discontinuity {
                expected_clock,
                start_clock: header.start_clock,
            });
        }
        self.next_clock = Some(header.end_clock());
        Ok(())
    }

    /// Record where the accepted run left the lane, so the next run's deltas
    /// decode against it.
    pub fn commit(&mut self, last_position: i32) {
        self.position = last_position;
    }

    /// Accept a run and adopt its trailing position in one step.
    pub fn admit(&mut self, run: &SampleRunView<'_>) -> Result<(), SampleRunError> {
        self.accept(&run.header())?;
        if let Some(last) = run.last_position() {
            self.commit(last);
        }
        Ok(())
    }
}

/// Encode `samples` as zigzag-LEB128 first differences against
/// `base_position`, returning the bytes written.
///
/// Chosen over fixed two-byte i16 differences with a 4-byte escape by
/// measurement, not assumption: on the shaped bench print
/// (`cargo run --release -p motion-core --example sample_encoding_bench`) a
/// 2 kHz phase lane costs 2601 B/s with varints against 4584 B/s with i16, and
/// a 4 kHz lane 5112 B/s against 9167 B/s — 43 % and 44 % less wire. Varints
/// also pack 24..48 samples into a run where i16 manages 9..24, so each run
/// amortises its header over twice the motion.
pub fn encode_deltas(
    base_position: i32,
    samples: &[i32],
    out: &mut [u8],
) -> Result<usize, SampleRunError> {
    if samples.len() > SAMPLE_RUN_COUNT_MAX {
        return Err(SampleRunError::CountExceeded {
            count: samples.len(),
            cap: SAMPLE_RUN_COUNT_MAX,
        });
    }
    let mut previous = i64::from(base_position);
    let mut written = 0usize;
    for (index, position) in samples.iter().copied().enumerate() {
        let position = i64::from(position);
        let delta = position - previous;
        if i32::try_from(delta).is_err() {
            return Err(SampleRunError::DeltaOverflow { index, delta });
        }
        written =
            write_varint(zigzag_encode(delta), out, written).ok_or(SampleRunError::Capacity {
                capacity: out.len(),
            })?;
        previous = position;
    }
    Ok(written)
}

/// Decode a `sample_run` payload into absolute positions. `count` is the
/// wire's own count field, so it is authoritative: a payload that ends early
/// or carries bytes past the last delta is a fault, not a shorter run.
pub fn decode_deltas(
    base_position: i32,
    data: &[u8],
    count: usize,
    out: &mut [i32],
) -> Result<(), SampleRunError> {
    let capacity = out.len();
    if count > capacity {
        return Err(SampleRunError::Capacity { capacity });
    }
    let mut previous = i64::from(base_position);
    let mut consumed = 0usize;
    for index in 0..count {
        let (encoded, next) =
            read_varint(data, consumed).ok_or(SampleRunError::Truncated { index })?;
        let delta = zigzag_decode(encoded);
        if i32::try_from(delta).is_err() {
            return Err(SampleRunError::DeltaOverflow { index, delta });
        }
        let position = previous + delta;
        let position = i32::try_from(position)
            .map_err(|_| SampleRunError::PositionOverflow { index, position })?;
        let slot = out
            .get_mut(index)
            .ok_or(SampleRunError::Capacity { capacity })?;
        *slot = position;
        previous = i64::from(position);
        consumed = next;
    }
    if consumed != data.len() {
        return Err(SampleRunError::Trailing {
            consumed,
            len: data.len(),
        });
    }
    Ok(())
}

/// Bytes `encode_deltas` would spend, without writing them. Lets a producer
/// close a run exactly at [`SAMPLE_RUN_DATA_MAX`] instead of overshooting and
/// backing out.
pub fn delta_bytes(base_position: i32, position: i32) -> Result<usize, SampleRunError> {
    let delta = i64::from(position) - i64::from(base_position);
    if i32::try_from(delta).is_err() {
        return Err(SampleRunError::DeltaOverflow { index: 0, delta });
    }
    Ok(varint_len(zigzag_encode(delta)))
}

const fn zigzag_encode(delta: i64) -> u64 {
    ((delta << 1) ^ (delta >> 63)) as u64
}

const fn zigzag_decode(encoded: u64) -> i64 {
    ((encoded >> 1) as i64) ^ -((encoded & 1) as i64)
}

const fn varint_len(mut value: u64) -> usize {
    let mut len = 1;
    while value >= 0x80 {
        value >>= 7;
        len += 1;
    }
    len
}

fn write_varint(mut value: u64, out: &mut [u8], at: usize) -> Option<usize> {
    let mut index = at;
    loop {
        let byte = (value as u8) & 0x7f;
        value >>= 7;
        let slot = out.get_mut(index)?;
        index += 1;
        if value == 0 {
            *slot = byte;
            return Some(index);
        }
        *slot = byte | 0x80;
    }
}

fn read_varint(data: &[u8], at: usize) -> Option<(u64, usize)> {
    let mut index = at;
    let mut shift = 0u32;
    let mut accumulator = 0u64;
    loop {
        let byte = *data.get(index)?;
        index += 1;
        accumulator |= u64::from(byte & 0x7f) << shift;
        if byte & 0x80 == 0 {
            return Some((accumulator, index));
        }
        shift += 7;
        if shift >= 64 {
            return None;
        }
    }
}

#[cfg(test)]
#[path = "sample_run_tests.rs"]
mod sample_run_tests;
