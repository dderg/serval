#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing
)]

use super::*;

#[test]
fn piece_entry_to_le_bytes_matches_field_layout() {
    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    coeffs[0] = 1.0;
    coeffs[1] = 2.0;
    coeffs[2] = 3.0;
    coeffs[3] = 4.0;
    let p = PieceEntry {
        start_time: 0x0102_0304_0506_0708,
        duration: 0.5,
        motor_mask: 0,
        coeff_count: 4,
        _reserved: [0; 2],
        coeffs,
    };
    let b = p.to_le_bytes();
    assert_eq!(b.len(), PIECE_ENTRY_BYTES);
    assert_eq!(&b[0..8], &0x0102_0304_0506_0708u64.to_le_bytes());
    assert_eq!(&b[8..12], &0.5f32.to_le_bytes());
    assert_eq!(b[12], 0);
    assert_eq!(b[13], 4);
    assert_eq!(&b[14..16], &[0u8; 2]);
    assert_eq!(&b[16..20], &1.0f32.to_le_bytes());
    assert_eq!(&b[20..24], &2.0f32.to_le_bytes());
    assert_eq!(&b[24..28], &3.0f32.to_le_bytes());
    assert_eq!(&b[28..32], &4.0f32.to_le_bytes());
    assert_eq!(&b[32..48], &[0u8; 16]);
}

#[test]
fn motor_mask_round_trips_at_byte_12() {
    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    coeffs[0] = 1.0;
    coeffs[1] = 2.0;
    coeffs[2] = 3.0;
    coeffs[3] = 4.0;
    let p = PieceEntry {
        start_time: 7,
        duration: 0.5,
        motor_mask: 0b0000_0100,
        coeff_count: 4,
        _reserved: [0; 2],
        coeffs,
    };
    let b = p.to_le_bytes();
    assert_eq!(b[12], 0b0000_0100);
    let r = PieceEntry::from_le_bytes(&b);
    assert_eq!(r.motor_mask, 0b0000_0100);
    assert_eq!(r.start_time, 7);
}

#[test]
fn stepper_sel_from_mask_cases() {
    assert_eq!(stepper_sel_from_mask(0), Ok(0));
    assert_eq!(stepper_sel_from_mask(0b0000_0001), Ok(1));
    assert_eq!(stepper_sel_from_mask(0b0000_1000), Ok(4));
    assert_eq!(stepper_sel_from_mask(0b1000_0000), Ok(8));
    assert!(stepper_sel_from_mask(0b0000_0011).is_err());
}

#[test]
fn wire_len_scales_with_coeff_count() {
    let mut p = PieceEntry::zeroed();
    p.coeff_count = 1;
    assert_eq!(p.wire_len(), PIECE_WIRE_HEADER_LEN + 4);
    p.coeff_count = 8;
    assert_eq!(p.wire_len(), PIECE_WIRE_HEADER_LEN + 32);
    assert_eq!(p.wire_len(), PIECE_ENTRY_BYTES);
}

#[test]
fn to_wire_bytes_round_trips_through_parse_wire() {
    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    coeffs[0] = 1.5;
    coeffs[1] = -2.5;
    coeffs[2] = 3.5;
    let entry = PieceEntry {
        start_time: 123_456,
        duration: 0.02,
        motor_mask: 0b0000_0010,
        coeff_count: 3,
        _reserved: [0; 2],
        coeffs,
    };

    let mut wire = Vec::new();
    entry.to_wire_bytes(&mut wire);
    assert_eq!(wire.len(), entry.wire_len());

    let (parsed, consumed) = PieceEntry::parse_wire(&wire).expect("parse must succeed");
    assert_eq!(consumed, entry.wire_len());
    assert_eq!(parsed.start_time, entry.start_time);
    assert_eq!(parsed.duration, entry.duration);
    assert_eq!(parsed.motor_mask, entry.motor_mask);
    assert_eq!(parsed.coeff_count, entry.coeff_count);
    assert_eq!(&parsed.coeffs[..3], &entry.coeffs[..3]);
}

#[test]
fn parse_wire_zero_fills_coeff_tail_when_reread_at_full_width() {
    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    coeffs[0] = 10.0;
    coeffs[1] = 20.0;
    let entry = PieceEntry {
        start_time: 1,
        duration: 0.01,
        motor_mask: 0,
        coeff_count: 2,
        _reserved: [0; 2],
        coeffs,
    };

    let mut wire = Vec::new();
    entry.to_wire_bytes(&mut wire);
    assert_eq!(wire.len(), PIECE_WIRE_HEADER_LEN + 8);

    let (parsed, _consumed) = PieceEntry::parse_wire(&wire).expect("parse must succeed");
    assert_eq!(parsed.coeffs[0], 10.0);
    assert_eq!(parsed.coeffs[1], 20.0);
    for &c in &parsed.coeffs[2..] {
        assert_eq!(
            c, 0.0,
            "coefficient tail beyond coeff_count must be zero-filled"
        );
    }
}

#[test]
fn parse_wire_rejects_zero_coeff_count() {
    let mut header = [0u8; PIECE_WIRE_HEADER_LEN];
    header[13] = 0;
    match PieceEntry::parse_wire(&header) {
        Err(PieceWireError::BadCoeffCount(0)) => {}
        other => panic!("expected BadCoeffCount(0), got {other:?}"),
    }
}

#[test]
fn parse_wire_rejects_coeff_count_above_max() {
    let mut header = [0u8; PIECE_WIRE_HEADER_LEN];
    header[13] = 9;
    match PieceEntry::parse_wire(&header) {
        Err(PieceWireError::BadCoeffCount(9)) => {}
        other => panic!("expected BadCoeffCount(9), got {other:?}"),
    }
}

#[test]
fn parse_wire_rejects_truncated_header() {
    let short = [0u8; PIECE_WIRE_HEADER_LEN - 1];
    match PieceEntry::parse_wire(&short) {
        Err(PieceWireError::Truncated { need, have }) => {
            assert_eq!(need, PIECE_WIRE_HEADER_LEN);
            assert_eq!(have, PIECE_WIRE_HEADER_LEN - 1);
        }
        other => panic!("expected Truncated header, got {other:?}"),
    }
}

#[test]
fn parse_wire_rejects_truncated_coeffs() {
    let mut header = [0u8; PIECE_WIRE_HEADER_LEN];
    header[13] = 3;
    let short = header.to_vec();
    match PieceEntry::parse_wire(&short) {
        Err(PieceWireError::Truncated { need, have }) => {
            assert_eq!(need, PIECE_WIRE_HEADER_LEN + 12);
            assert_eq!(have, PIECE_WIRE_HEADER_LEN);
        }
        other => panic!("expected Truncated coeffs, got {other:?}"),
    }
}

#[test]
fn to_le_bytes_from_le_bytes_48_byte_round_trip() {
    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    for (k, c) in coeffs.iter_mut().enumerate() {
        *c = k as f32 + 0.25;
    }
    let entry = PieceEntry {
        start_time: 0xDEAD_BEEF_0000_1234,
        duration: 0.03,
        motor_mask: 0b0000_0101,
        coeff_count: 8,
        _reserved: [0; 2],
        coeffs,
    };

    let bytes = entry.to_le_bytes();
    assert_eq!(bytes.len(), PIECE_ENTRY_BYTES);
    let back = PieceEntry::from_le_bytes(&bytes);
    assert_eq!(back.start_time, entry.start_time);
    assert_eq!(back.duration, entry.duration);
    assert_eq!(back.motor_mask, entry.motor_mask);
    assert_eq!(back.coeff_count, entry.coeff_count);
    assert_eq!(back.coeffs, entry.coeffs);
}

#[test]
fn endpoints_of_linear_piece() {
    let p0 = 2.0_f64;
    let p1 = 12.0_f64;
    let duration = 0.02_f64;
    let mono = [p0, (p1 - p0) / duration];
    let cheb = nurbs::chebyshev::monomial_tau_to_chebyshev(&mono, duration);
    assert_eq!(cheb.len(), 2);

    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    coeffs[0] = cheb[0] as f32;
    coeffs[1] = cheb[1] as f32;
    let entry = PieceEntry {
        start_time: 0,
        duration: duration as f32,
        motor_mask: 0,
        coeff_count: 2,
        _reserved: [0; 2],
        coeffs,
    };

    assert!((entry.pos_start() - p0 as f32).abs() < 1e-4);
    assert!((entry.pos_end() - p1 as f32).abs() < 1e-4);
    let expected_vel = ((p1 - p0) / duration) as f32;
    assert!((entry.vel_start() - expected_vel).abs() < 1e-2);
    assert!((entry.vel_end() - expected_vel).abs() < 1e-2);
}

#[test]
fn endpoints_of_quadratic_piece() {
    let c0 = 1.0_f64;
    let c1 = 5.0_f64;
    let c2 = 20.0_f64;
    let duration = 0.05_f64;
    let mono = [c0, c1, c2];
    let cheb = nurbs::chebyshev::monomial_tau_to_chebyshev(&mono, duration);
    assert_eq!(cheb.len(), 3);

    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    for (dst, &src) in coeffs.iter_mut().zip(cheb.iter()) {
        *dst = src as f32;
    }
    let entry = PieceEntry {
        start_time: 0,
        duration: duration as f32,
        motor_mask: 0,
        coeff_count: cheb.len() as u8,
        _reserved: [0; 2],
        coeffs,
    };

    let p_at_0 = c0;
    let p_at_d = c0 + c1 * duration + c2 * duration * duration;
    let v_at_0 = c1;
    let v_at_d = c1 + 2.0 * c2 * duration;

    assert!(
        (entry.pos_start() - p_at_0 as f32).abs() < 1e-3,
        "pos_start mismatch: got {} want {}",
        entry.pos_start(),
        p_at_0
    );
    assert!(
        (entry.pos_end() - p_at_d as f32).abs() < 1e-3,
        "pos_end mismatch: got {} want {}",
        entry.pos_end(),
        p_at_d
    );
    assert!(
        (entry.vel_start() - v_at_0 as f32).abs() < 1e-1,
        "vel_start mismatch: got {} want {}",
        entry.vel_start(),
        v_at_0
    );
    assert!(
        (entry.vel_end() - v_at_d as f32).abs() < 1e-1,
        "vel_end mismatch: got {} want {}",
        entry.vel_end(),
        v_at_d
    );
}

fn ring(depth: usize) -> RingDescriptor {
    RingDescriptor::new(0, depth)
}

#[test]
fn checked_commit_applies_contiguous_block() {
    let mut r = ring(8);
    assert_eq!(r.commit_head_checked(0, 3, 3), CommitOutcome::Applied);
    assert_eq!(r.head, 3);
    assert_eq!(r.commit_head_checked(3, 2, 5), CommitOutcome::Applied);
    assert_eq!(r.head, 5);
}

#[test]
fn checked_commit_refuses_gap_after_lost_frame() {
    let mut r = ring(8);
    assert_eq!(r.commit_head_checked(0, 3, 3), CommitOutcome::Applied);
    // Frame for slots 3..5 was lost; the next windowed frame declares 5..7.
    assert_eq!(r.commit_head_checked(5, 2, 7), CommitOutcome::Gap);
    assert_eq!(r.head, 3, "a gapped commit must not move the head");
    // Replay of the lost frame, then the gapped one, heals the stream.
    assert_eq!(r.commit_head_checked(3, 2, 5), CommitOutcome::Applied);
    assert_eq!(r.commit_head_checked(5, 2, 7), CommitOutcome::Applied);
}

#[test]
fn checked_commit_treats_full_duplicate_as_stale_noop() {
    let mut r = ring(8);
    assert_eq!(r.commit_head_checked(0, 3, 3), CommitOutcome::Applied);
    assert_eq!(r.commit_head_checked(0, 3, 3), CommitOutcome::Stale);
    assert_eq!(r.head, 3);
}

#[test]
fn checked_commit_refuses_slot_mismatch_even_when_head_math_adds_up() {
    let mut r = ring(8);
    assert_eq!(r.commit_head_checked(0, 2, 2), CommitOutcome::Applied);
    // new_head - head == piece_count but the declared slot is wrong.
    assert_eq!(r.commit_head_checked(3, 2, 4), CommitOutcome::Gap);
}

#[test]
fn checked_commit_still_detects_overcommit() {
    let mut r = ring(4);
    assert_eq!(r.commit_head_checked(0, 5, 5), CommitOutcome::Overcommit);
}

#[test]
fn checked_commit_survives_wrapping_counters() {
    let mut r = ring(8);
    r.head = u32::MAX - 1;
    r.retired = u32::MAX - 1;
    r.tail = ((u32::MAX - 1) % 8) as usize;
    let slot = ((u32::MAX - 1) % 8) as u16;
    assert_eq!(
        r.commit_head_checked(slot, 4, r.head.wrapping_add(4)),
        CommitOutcome::Applied
    );
    assert_eq!(r.head, (u32::MAX - 1).wrapping_add(4));
}

#[test]
fn slot_is_live_tracks_committed_unretired_span() {
    let mut r = ring(8);
    assert_eq!(r.commit_head_checked(0, 3, 3), CommitOutcome::Applied);
    assert!(r.slot_is_live(0));
    assert!(r.slot_is_live(2));
    assert!(
        !r.slot_is_live(3),
        "slots at and past the head are writable"
    );
    r.advance_counter();
    assert!(!r.slot_is_live(0), "retired slots are writable again");
    assert!(r.slot_is_live(1));
}
