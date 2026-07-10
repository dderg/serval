//! Live capture tailing: incremental decode of a growing `.scap` so the
//! dashboard can plot following error while the capture is still being
//! written. The endpoint writer (`ethercat-rt/src/capture.rs`) writes each
//! record unbuffered as it drains the ring, so a reader on the same host
//! sees data at record granularity — no flush knob needed.
//!
//! The tail request/response contract is byte offsets, not record indexes:
//! the first request passes `offset=0`, every response carries
//! `next_offset` (always record-aligned), and the client echoes it back.
//! A misaligned offset is a client bug and fails loud.

use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

use serde_json::json;

use crate::scap::{Scap, FLAG_MOTION_ACTIVE};

const HEADER_LINE_CAP: usize = 65_536;
/// At the 4 kHz DC cycle this is 5 s of backlog per request — a client that
/// falls further behind catches up over successive polls.
const MAX_TAIL_RECORDS: usize = 20_000;
const MAX_POINTS_PER_RESPONSE: usize = 2_000;

/// Newest top-level `.scap` in `captures_root` (manual/live captures land
/// flat; calibration runs write into subdirectories). `.failed.scap` files
/// are skipped — their gaps would draw garbage.
pub fn newest_flat_scap(captures_root: &Path) -> Result<Option<PathBuf>, String> {
    let entries = std::fs::read_dir(captures_root)
        .map_err(|e| format!("read_dir {}: {e}", captures_root.display()))?;
    let mut newest: Option<(std::time::SystemTime, PathBuf)> = None;
    for entry in entries {
        let entry = entry.map_err(|e| format!("read_dir entry: {e}"))?;
        let path = entry.path();
        let name = match path.file_name().and_then(|n| n.to_str()) {
            Some(n) => n,
            None => continue,
        };
        if !path.is_file() || !valid_capture_name(name) {
            continue;
        }
        let mtime = entry
            .metadata()
            .and_then(|m| m.modified())
            .map_err(|e| format!("stat {}: {e}", path.display()))?;
        if newest.as_ref().is_none_or(|(t, _)| mtime > *t) {
            newest = Some((mtime, path));
        }
    }
    Ok(newest.map(|(_, p)| p))
}

pub fn valid_capture_name(name: &str) -> bool {
    name.strip_suffix(".scap").is_some_and(|stem| {
        !stem.is_empty()
            && !stem.ends_with(".failed")
            && stem
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    })
}

struct HeaderSlice {
    bytes: Vec<u8>,
    data_start: u64,
    record_size: u64,
}

fn read_header_slice(file: &mut std::fs::File, path: &Path) -> Result<HeaderSlice, String> {
    let mut buf = vec![0u8; HEADER_LINE_CAP];
    file.seek(SeekFrom::Start(0))
        .map_err(|e| format!("seek {}: {e}", path.display()))?;
    let mut filled = 0usize;
    while filled < buf.len() {
        let n = file
            .read(&mut buf[filled..])
            .map_err(|e| format!("read {}: {e}", path.display()))?;
        if n == 0 {
            break;
        }
        filled += n;
        if buf[..filled].contains(&b'\n') {
            break;
        }
    }
    let nl = buf[..filled]
        .iter()
        .position(|&b| b == b'\n')
        .ok_or_else(|| {
            format!(
                "{}: no header newline in the first {filled} bytes",
                path.display()
            )
        })?;
    let header: serde_json::Value = serde_json::from_slice(&buf[..nl])
        .map_err(|e| format!("{}: header parse: {e}", path.display()))?;
    let record_size = header["record_size"]
        .as_u64()
        .ok_or_else(|| format!("{}: header has no record_size", path.display()))?;
    if record_size == 0 {
        return Err(format!("{}: record_size 0", path.display()));
    }
    let mut bytes = buf[..nl + 1].to_vec();
    bytes.truncate(nl + 1);
    Ok(HeaderSlice {
        bytes,
        data_start: (nl + 1) as u64,
        record_size,
    })
}

/// Decode records `[offset, min(EOF, offset + MAX_TAIL_RECORDS))` of a
/// possibly-growing capture into a JSON payload for the live chart:
/// per-drive following error and torque, the motion flag, and the
/// record-aligned `next_offset` to poll from. `offset == 0` means "from the
/// first record"; any other offset must be record-aligned (i.e. a prior
/// response's `next_offset`).
pub fn tail_scap(path: &Path, offset: u64) -> Result<serde_json::Value, String> {
    let mut file =
        std::fs::File::open(path).map_err(|e| format!("open {}: {e}", path.display()))?;
    let header = read_header_slice(&mut file, path)?;
    let file_len = file
        .metadata()
        .map_err(|e| format!("stat {}: {e}", path.display()))?
        .len();

    let start = if offset == 0 {
        header.data_start
    } else {
        offset
    };
    if start < header.data_start || (start - header.data_start) % header.record_size != 0 {
        return Err(format!(
            "offset {offset} is not record-aligned (data starts at {}, record_size {})",
            header.data_start, header.record_size
        ));
    }
    let complete_records = (file_len.saturating_sub(start)) / header.record_size;
    let take = complete_records.min(MAX_TAIL_RECORDS as u64);
    let next_offset = start + take * header.record_size;

    let mut chunk = vec![0u8; (take * header.record_size) as usize];
    file.seek(SeekFrom::Start(start))
        .map_err(|e| format!("seek {}: {e}", path.display()))?;
    file.read_exact(&mut chunk)
        .map_err(|e| format!("read {}: {e}", path.display()))?;

    let mut stitched = header.bytes;
    stitched.extend_from_slice(&chunk);
    let cap = Scap::from_bytes(&stitched).map_err(|e| format!("{}: {e}", path.display()))?;

    let stride = (cap.n_records / MAX_POINTS_PER_RESPONSE).max(1);
    let thin_i64 = |v: Vec<i64>| -> Vec<i64> { v.into_iter().step_by(stride).collect() };

    let mut drives = serde_json::Map::new();
    let mut moving: Option<Vec<bool>> = None;
    for (idx, name) in cap.drive_names().iter().enumerate() {
        let ferr = thin_i64(cap.read_i64(idx, "following_error")?);
        let torque = thin_i64(cap.read_i64(idx, "torque_actual")?);
        if moving.is_none() {
            let flags = cap.read_i64(idx, "flags")?;
            moving = Some(
                flags
                    .into_iter()
                    .step_by(stride)
                    .map(|f| f & FLAG_MOTION_ACTIVE != 0)
                    .collect(),
            );
        }
        drives.insert(name.clone(), json!({"ferr": ferr, "torque": torque}));
    }

    let first_record = (start - header.data_start) / header.record_size;
    Ok(json!({
        "name": path.file_name().and_then(|n| n.to_str()),
        "cycle_ns": cap.header.cycle_ns,
        "fs_hz": cap.fs(),
        "first_record": first_record,
        "n_records": cap.n_records,
        "stride": stride,
        "next_offset": next_offset,
        "eof_bytes": file_len,
        "drives": drives,
        "moving": moving.unwrap_or_default(),
    }))
}
