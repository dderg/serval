use crate::enqueue::subdivide_bernstein;

pub const MAX_CORRECTION_PIECE_SECS: f64 = 0.25;

#[derive(Debug, Clone, Copy)]
pub struct ProfilePiece {
    pub coeffs: [f64; 4],
    pub duration: f64,
}

pub fn profile_duration(delta_mm: f64, speed: f64, accel: f64) -> Result<f64, String> {
    let d = delta_mm.abs();
    if !(d > 0.0) || !(speed > 0.0) || !(accel > 0.0) {
        return Err(format!(
            "correction profile needs delta!=0, speed>0, accel>0; got {delta_mm} {speed} {accel}"
        ));
    }
    let v = speed.min((d * accel).sqrt());
    let t_ramp = v / accel;
    let d_cruise = d - v * t_ramp;
    Ok(2.0 * t_ramp + d_cruise / v)
}

pub fn plan_correction_profile(
    delta_mm: f64,
    speed: f64,
    accel: f64,
) -> Result<Vec<ProfilePiece>, String> {
    profile_duration(delta_mm, speed, accel)?;
    let mut out = Vec::new();
    push_segment(&mut out, 0.0, delta_mm, speed, accel);
    Ok(subdivide_all(out))
}

fn subdivide_all(pieces: Vec<ProfilePiece>) -> Vec<ProfilePiece> {
    pieces
        .into_iter()
        .flat_map(|p| {
            subdivide_bernstein(p.coeffs, p.duration, MAX_CORRECTION_PIECE_SECS)
                .into_iter()
                .map(|(coeffs, duration)| ProfilePiece { coeffs, duration })
        })
        .collect()
}

/// Append one trapezoid for `delta_mm` starting at absolute position `p_start`.
fn push_segment(out: &mut Vec<ProfilePiece>, p_start: f64, delta_mm: f64, speed: f64, accel: f64) {
    let sign = delta_mm.signum();
    let d = delta_mm.abs();
    let v = speed.min((d * accel).sqrt());
    let t_ramp = v / accel;
    let d_ramp = 0.5 * accel * t_ramp * t_ramp;
    let d_cruise = d - 2.0 * d_ramp;
    push_quad(out, p_start, sign, 0.0, 0.0, accel, t_ramp);
    if d_cruise > 1e-12 {
        push_lin(out, p_start, sign, d_ramp, v, d_cruise / v);
    }
    push_quad(out, p_start, sign, d_ramp + d_cruise, v, -accel, t_ramp);
}

fn push_quad(out: &mut Vec<ProfilePiece>, base: f64, sign: f64, p0: f64, v0: f64, a: f64, t: f64) {
    if t <= 0.0 {
        return;
    }
    let b0 = p0;
    let b1 = p0 + v0 * t / 3.0;
    let b2 = p0 + 2.0 * v0 * t / 3.0 + a * t * t / 6.0;
    let b3 = p0 + v0 * t + 0.5 * a * t * t;
    out.push(ProfilePiece {
        coeffs: [
            base + sign * b0,
            base + sign * b1,
            base + sign * b2,
            base + sign * b3,
        ],
        duration: t,
    });
}

fn push_lin(out: &mut Vec<ProfilePiece>, base: f64, sign: f64, p0: f64, v: f64, t: f64) {
    if t <= 0.0 {
        return;
    }
    out.push(ProfilePiece {
        coeffs: [
            base + sign * p0,
            base + sign * (p0 + v * t / 3.0),
            base + sign * (p0 + 2.0 * v * t / 3.0),
            base + sign * (p0 + v * t),
        ],
        duration: t,
    });
}

const SEGMENT_EPS_MM: f64 = 1e-5;

/// Plan a gapless piece sequence for relative motor-space moves; sub-epsilon
/// segments are skipped and at least one real segment is required.
pub fn plan_correction_sequence(
    segments: &[f64],
    speed: f64,
    accel: f64,
) -> Result<Vec<ProfilePiece>, String> {
    if !(speed > 0.0) || !(accel > 0.0) {
        return Err(format!(
            "correction sequence needs speed>0, accel>0; got {speed} {accel}"
        ));
    }
    let mut out = Vec::new();
    let mut pos = 0.0;
    let mut any = false;
    for &s in segments {
        if !s.is_finite() {
            return Err(format!("correction sequence segment is not finite: {s}"));
        }
        if s.abs() < SEGMENT_EPS_MM {
            continue;
        }
        push_segment(&mut out, pos, s, speed, accel);
        pos += s;
        any = true;
    }
    if !any {
        return Err(
            "correction sequence is empty or has no segment above SEGMENT_EPS_MM".to_string(),
        );
    }
    Ok(subdivide_all(out))
}

const DEMUX_BUFFER_BYTES: usize = 512;
const FRAME_ENVELOPE_BYTES: usize = 4;
const FRAME_CRC_BYTES: usize = 2;
const MESSAGE_HEADER_BYTES: usize = 7;
const CORRECTION_BODY_HEADER_BYTES: usize = 9;
const PIECE_BYTES: usize = 32;

pub const MAX_CORRECTION_PIECES_PER_MSG: usize = (DEMUX_BUFFER_BYTES
    - FRAME_ENVELOPE_BYTES
    - FRAME_CRC_BYTES
    - MESSAGE_HEADER_BYTES
    - CORRECTION_BODY_HEADER_BYTES)
    / PIECE_BYTES;
const _: () = assert!(
    MAX_CORRECTION_PIECES_PER_MSG < runtime::stepping_state::CORRECTION_RING_DEPTH,
    "each chunk must fit the MCU correction ring"
);

pub fn to_piece_entries(
    pieces: &[ProfilePiece],
    project: impl Fn(f64) -> u64,
    start_host_secs: f64,
) -> Vec<runtime::piece_ring::PieceEntry> {
    let mut t = start_host_secs;
    pieces
        .iter()
        .map(|p| {
            #[allow(clippy::cast_possible_truncation)]
            let entry = runtime::piece_ring::PieceEntry {
                start_time: project(t),
                coeffs: [
                    p.coeffs[0] as f32,
                    p.coeffs[1] as f32,
                    p.coeffs[2] as f32,
                    p.coeffs[3] as f32,
                ],
                duration: p.duration as f32,
                motor_mask: 0,
                _reserved: [0; 3],
            };
            t += p.duration;
            entry
        })
        .collect()
}

pub fn to_overlay_piece_entries(
    pieces: &[ProfilePiece],
    project: impl Fn(f64) -> u64,
    start_host_secs: f64,
    motor_mask: u8,
) -> Vec<(runtime::piece_ring::PieceEntry, f64)> {
    let mut t = start_host_secs;
    pieces
        .iter()
        .map(|p| {
            #[allow(clippy::cast_possible_truncation)]
            let entry = runtime::piece_ring::PieceEntry {
                start_time: project(t),
                coeffs: [
                    p.coeffs[0] as f32,
                    p.coeffs[1] as f32,
                    p.coeffs[2] as f32,
                    p.coeffs[3] as f32,
                ],
                duration: p.duration as f32,
                motor_mask,
                _reserved: [0; 3],
            };
            let host_secs = t;
            t += p.duration;
            (entry, host_secs)
        })
        .collect()
}

pub fn chunk_correction_messages(
    axis_idx: u8,
    motor_idx: u8,
    entries: &[runtime::piece_ring::PieceEntry],
) -> Vec<kalico_protocol::messages::PushCorrectionPieces> {
    let mut out = Vec::new();
    let mut head: u32 = 0;
    for chunk in entries.chunks(MAX_CORRECTION_PIECES_PER_MSG) {
        #[allow(clippy::cast_possible_truncation)]
        let start_slot = (head % runtime::stepping_state::CORRECTION_RING_DEPTH as u32) as u16;
        let mut pieces_bytes = Vec::with_capacity(chunk.len() * 32);
        for e in chunk {
            pieces_bytes.extend_from_slice(&e.to_le_bytes());
        }
        head += chunk.len() as u32;
        #[allow(clippy::cast_possible_truncation)]
        out.push(kalico_protocol::messages::PushCorrectionPieces {
            axis_idx,
            motor_idx,
            piece_count: chunk.len() as u8,
            start_slot,
            new_head: head,
            pieces_bytes,
        });
    }
    out
}

#[cfg(test)]
mod tests;
