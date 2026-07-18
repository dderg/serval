//! Native `.scap` reader, ported from `scripts/servo_capture.py` `load_capture`.
//!
//! A capture is a single JSON header line followed by fixed-size binary
//! records. The header names per-drive blocks and per-channel dtypes/offsets;
//! a channel at offset >= `RECORD_PREFIX_SIZE` is per-drive and its effective
//! offset shifts by `drive_idx * block_size`. Malformed input fails loudly
//! with a one-line reason — never a partial or padded result.

use serde::Deserialize;

pub const RECORD_PREFIX_SIZE: usize = 9;
pub const FLAG_MOTION_ACTIVE: i64 = 1 << 1;
const SUPPORTED_VERSIONS: [i64; 2] = [1, 2];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dtype {
    U8,
    U16,
    I16,
    I32,
    U32,
    U64,
    F32,
}

impl Dtype {
    fn parse(s: &str) -> Result<Dtype, String> {
        match s {
            "u8" => Ok(Dtype::U8),
            "u16" => Ok(Dtype::U16),
            "i16" => Ok(Dtype::I16),
            "i32" => Ok(Dtype::I32),
            "u32" => Ok(Dtype::U32),
            "u64" => Ok(Dtype::U64),
            "f32" => Ok(Dtype::F32),
            other => Err(format!("unknown channel dtype {other:?}")),
        }
    }

    pub(crate) fn itemsize(self) -> usize {
        match self {
            Dtype::U8 => 1,
            Dtype::U16 | Dtype::I16 => 2,
            Dtype::I32 | Dtype::U32 | Dtype::F32 => 4,
            Dtype::U64 => 8,
        }
    }

    pub(crate) fn read_i64(self, b: &[u8]) -> i64 {
        match self {
            Dtype::U8 => b[0] as i64,
            Dtype::U16 => u16::from_le_bytes([b[0], b[1]]) as i64,
            Dtype::I16 => i16::from_le_bytes([b[0], b[1]]) as i64,
            Dtype::I32 => i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as i64,
            Dtype::U32 => u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as i64,
            Dtype::U64 => {
                u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]) as i64
            }
            Dtype::F32 => f32::from_le_bytes([b[0], b[1], b[2], b[3]]) as i64,
        }
    }

    /// Write `v` into a signed-integer channel's bytes, saturating to the
    /// dtype's range. Used by `demo`'s record patcher, never by the reader.
    pub(crate) fn write_i64_saturating(self, b: &mut [u8], v: i64) -> Result<(), String> {
        match self {
            Dtype::I16 => {
                let clamped = v.clamp(i16::MIN as i64, i16::MAX as i64) as i16;
                b[..2].copy_from_slice(&clamped.to_le_bytes());
                Ok(())
            }
            Dtype::I32 => {
                let clamped = v.clamp(i32::MIN as i64, i32::MAX as i64) as i32;
                b[..4].copy_from_slice(&clamped.to_le_bytes());
                Ok(())
            }
            other => Err(format!("write_i64_saturating: unsupported dtype {other:?}")),
        }
    }

    fn read_f64(self, b: &[u8]) -> f64 {
        match self {
            Dtype::F32 => f32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
            _ => self.read_i64(b) as f64,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct Drive {
    pub name: String,
    pub counts_per_mm: f64,
    #[serde(default)]
    pub rotation_distance: f64,
    #[serde(default)]
    pub invert: bool,
}

#[derive(Debug, Clone)]
pub struct Channel {
    pub name: String,
    pub dtype: Dtype,
    pub offset: usize,
}

#[derive(Debug, Deserialize)]
struct ChannelRaw {
    name: String,
    dtype: String,
    offset: usize,
}

#[derive(Debug, Deserialize)]
struct HeaderRaw {
    version: i64,
    cycle_ns: u64,
    record_size: usize,
    drives: Vec<Drive>,
    channels: Vec<ChannelRaw>,
}

#[derive(Debug, Clone)]
pub struct Header {
    pub version: i64,
    pub cycle_ns: u64,
    pub record_size: usize,
    pub drives: Vec<Drive>,
    pub channels: Vec<Channel>,
    block_size: usize,
    /// Where the per-drive blocks start — derived from the header's own
    /// global channels, not a compile-time constant, so captures from
    /// endpoints with more or fewer global channels all load.
    prefix: usize,
}

impl Header {
    /// Parse and validate one scap v2 JSON header line (without its
    /// trailing newline) — the first line of a `.scap` file and the greeting
    /// of the live tap socket alike.
    pub fn parse_line(line: &[u8]) -> Result<Header, String> {
        let raw: HeaderRaw =
            serde_json::from_slice(line).map_err(|e| format!("header parse: {e}"))?;
        if !SUPPORTED_VERSIONS.contains(&raw.version) {
            return Err(format!("unsupported capture version {}", raw.version));
        }
        if raw.cycle_ns == 0 {
            return Err("header cycle_ns is 0".to_string());
        }
        let n_drives = raw.drives.len();
        if n_drives == 0 {
            return Err("header lists no drives".to_string());
        }
        let record_size = raw.record_size;
        let mut channels = Vec::with_capacity(raw.channels.len());
        for c in raw.channels {
            let dtype = Dtype::parse(&c.dtype)?;
            channels.push(Channel {
                name: c.name,
                dtype,
                offset: c.offset,
            });
        }
        // The prefix (global region before the per-drive blocks) ends where
        // the last known global channel ends; older captures carry only
        // cycle_index+flags, newer ones add the RT-loop health counters.
        const GLOBAL_CHANNELS: [&str; 5] = [
            "cycle_index",
            "flags",
            "skip_count",
            "late_frames",
            "frame_lateness_ns",
        ];
        let prefix = channels
            .iter()
            .filter(|c| GLOBAL_CHANNELS.contains(&c.name.as_str()))
            .map(|c| c.offset + c.dtype.itemsize())
            .max()
            .ok_or("header lists no global channels")?;
        if record_size <= prefix {
            return Err(format!("record_size {record_size} has no per-drive body"));
        }
        let body_size = record_size - prefix;
        if body_size % n_drives != 0 {
            return Err(format!(
                "record_size {record_size} is not aligned to {n_drives} drive block(s)"
            ));
        }
        let block_size = body_size / n_drives;
        for c in &channels {
            let last_off = if c.offset >= prefix {
                c.offset + (n_drives - 1) * block_size
            } else {
                c.offset
            };
            if last_off + c.dtype.itemsize() > record_size {
                return Err(format!(
                    "channel {:?} at offset {} overruns record_size {}",
                    c.name, c.offset, record_size
                ));
            }
        }
        Ok(Header {
            version: raw.version,
            cycle_ns: raw.cycle_ns,
            record_size,
            drives: raw.drives,
            channels,
            block_size,
            prefix,
        })
    }

    pub fn block_size(&self) -> usize {
        self.block_size
    }

    pub fn channel(&self, name: &str) -> Option<&Channel> {
        self.channels.iter().find(|c| c.name == name)
    }

    pub fn eff_offset(&self, ch: &Channel, drive_idx: usize) -> usize {
        if ch.offset >= self.prefix {
            ch.offset + drive_idx * self.block_size
        } else {
            ch.offset
        }
    }
}

#[derive(Debug)]
pub struct Scap {
    pub header: Header,
    pub n_records: usize,
    body: Vec<u8>,
}

impl Scap {
    pub fn load(path: &str) -> Result<Scap, String> {
        if path.ends_with(".failed.scap") {
            return Err(format!(
                "{path} is a FAILED capture (ring overflow or writer error); its \
                 gaps would poison every metric. Re-run the capture."
            ));
        }
        let bytes = std::fs::read(path).map_err(|e| format!("read {path}: {e}"))?;
        Scap::from_bytes(&bytes).map_err(|e| format!("{path}: {e}"))
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Scap, String> {
        let nl = bytes
            .iter()
            .position(|&b| b == b'\n')
            .ok_or("capture has no header line")?;
        let header = Header::parse_line(&bytes[..nl])?;
        let body = bytes[nl + 1..].to_vec();
        let n_records = body.len() / header.record_size;
        Ok(Scap {
            header,
            n_records,
            body,
        })
    }

    pub fn fs(&self) -> f64 {
        1e9 / self.header.cycle_ns as f64
    }

    pub fn drive_names(&self) -> Vec<String> {
        self.header.drives.iter().map(|d| d.name.clone()).collect()
    }

    pub fn drive_index(&self, name: &str) -> Option<usize> {
        self.header.drives.iter().position(|d| d.name == name)
    }

    pub fn channel(&self, name: &str) -> Option<&Channel> {
        self.header.channel(name)
    }

    pub fn has_channel(&self, name: &str) -> bool {
        self.channel(name).is_some()
    }

    pub fn read_i64(&self, drive_idx: usize, name: &str) -> Result<Vec<i64>, String> {
        let ch = self
            .channel(name)
            .ok_or_else(|| format!("capture has no channel {name:?}"))?;
        let off = self.header.eff_offset(ch, drive_idx);
        let rs = self.header.record_size;
        let mut out = Vec::with_capacity(self.n_records);
        for r in 0..self.n_records {
            out.push(ch.dtype.read_i64(&self.body[r * rs + off..]));
        }
        Ok(out)
    }

    pub fn read_f64(&self, drive_idx: usize, name: &str) -> Result<Vec<f64>, String> {
        let ch = self
            .channel(name)
            .ok_or_else(|| format!("capture has no channel {name:?}"))?;
        let off = self.header.eff_offset(ch, drive_idx);
        let rs = self.header.record_size;
        let mut out = Vec::with_capacity(self.n_records);
        for r in 0..self.n_records {
            out.push(ch.dtype.read_f64(&self.body[r * rs + off..]));
        }
        Ok(out)
    }
}
