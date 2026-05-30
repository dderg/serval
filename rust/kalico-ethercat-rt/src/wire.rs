//! Wire helpers: piece-bytes -> WirePiece, control-message decode, responses.

use runtime::cubic_curve::WirePiece;

#[derive(Debug, PartialEq, Eq)]
pub enum PiecesError {
    BadLength,
}

/// Split a `LoadCurveCubic.pieces_bytes` blob into `WirePiece`s.
pub fn wire_pieces_from_bytes(piece_count: u8, bytes: &[u8]) -> Result<Vec<WirePiece>, PiecesError> {
    let n = piece_count as usize;
    if bytes.len() != n * 20 {
        return Err(PiecesError::BadLength);
    }
    let mut out = Vec::with_capacity(n);
    for chunk in bytes.chunks_exact(20) {
        let rd = |i: usize| u32::from_le_bytes([chunk[i], chunk[i + 1], chunk[i + 2], chunk[i + 3]]);
        out.push(WirePiece {
            bp0_bits: rd(0),
            bp1_bits: rd(4),
            bp2_bits: rd(8),
            bp3_bits: rd(12),
            duration_bits: rd(16),
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_one_piece() {
        let bytes = {
            let mut v = Vec::new();
            for x in [0.0f32, 0.0, 10.0, 10.0] {
                v.extend_from_slice(&x.to_bits().to_le_bytes());
            }
            v.extend_from_slice(&0.5f32.to_bits().to_le_bytes());
            v
        };
        let pieces = wire_pieces_from_bytes(1, &bytes).unwrap();
        assert_eq!(pieces.len(), 1);
        assert_eq!(f32::from_bits(pieces[0].bp2_bits), 10.0);
        assert_eq!(f32::from_bits(pieces[0].duration_bits), 0.5);
    }

    #[test]
    fn rejects_wrong_length() {
        assert!(matches!(wire_pieces_from_bytes(2, &[0u8; 20]), Err(PiecesError::BadLength)));
    }
}
