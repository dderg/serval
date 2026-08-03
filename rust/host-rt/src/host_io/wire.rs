pub use mcu_transport::frame::crc16_ccitt;
pub use mcu_transport::klipper_frame::{
    MESSAGE_DEST, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_SEQ_MASK, MESSAGE_SYNC,
    MESSAGE_TRAILER_SIZE,
};

/// How many command bytes one Klipper message block can carry.
///
/// The block — not the command — is the unit the mcu CRCs, sequence-checks
/// and acknowledges (`command_find_and_dispatch` in `src/command.c` calls
/// `command_send_ack` once per block), and the unit the protocol's 4-bit
/// sequence number counts. One command per block therefore spends five
/// framing bytes and one 5-byte mcu ack on every command, and burns one of
/// the sixteen in-flight sequence numbers on it: at the ~1450 commands/s a
/// dense layer produces, the whole unacked window is worth 11 ms of motion.
/// Packing a burst into full blocks is what buys that window back.
pub const BLOCK_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;

/// Greedily pack encoded commands into block payloads of at most
/// [`BLOCK_PAYLOAD_MAX`] bytes, preserving order.
pub fn pack_blocks(payloads: &[Vec<u8>]) -> Result<Vec<Vec<u8>>, String> {
    let mut blocks: Vec<Vec<u8>> = Vec::new();
    for payload in payloads {
        if payload.len() > BLOCK_PAYLOAD_MAX {
            return Err(format!(
                "encoded command is {} bytes, above the {BLOCK_PAYLOAD_MAX}-byte block payload \
                 a Klipper message can carry",
                payload.len()
            ));
        }
        match blocks.last_mut() {
            Some(open) if open.len() + payload.len() <= BLOCK_PAYLOAD_MAX => {
                open.extend_from_slice(payload);
            }
            _ => {
                let mut open = Vec::with_capacity(BLOCK_PAYLOAD_MAX);
                open.extend_from_slice(payload);
                blocks.push(open);
            }
        }
    }
    Ok(blocks)
}

pub fn build_frame(payload: &[u8], seq: u8) -> Vec<u8> {
    let msglen = MESSAGE_MIN + payload.len();
    let seq_byte = (seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
    let mut frame = Vec::with_capacity(msglen);
    frame.push(msglen as u8);
    frame.push(seq_byte);
    frame.extend_from_slice(payload);
    let crc = crc16_ccitt(&frame);
    frame.push((crc >> 8) as u8);
    frame.push((crc & 0xFF) as u8);
    frame.push(MESSAGE_SYNC);
    frame
}

#[doc(hidden)]
pub fn extract_packet(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    while !buf.is_empty() {
        let msglen = buf[0] as usize;
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&msglen) {
            buf.remove(0);
            continue;
        }
        if buf.len() < msglen {
            return None;
        }
        let seq_byte = buf[1];
        if (seq_byte & !MESSAGE_SEQ_MASK) != MESSAGE_DEST || buf[msglen - 1] != MESSAGE_SYNC {
            buf.remove(0);
            continue;
        }
        let crc_off = msglen - MESSAGE_TRAILER_SIZE;
        let crc_expected = (u16::from(buf[crc_off]) << 8) | u16::from(buf[crc_off + 1]);
        let crc_actual = crc16_ccitt(&buf[..crc_off]);
        if crc_expected != crc_actual {
            buf.remove(0);
            continue;
        }
        let pkt = buf[..msglen].to_vec();
        buf.drain(..msglen);
        return Some(pkt);
    }
    None
}

pub fn decode_absolute(prev_abs: u64, wire_seq: u8) -> u64 {
    let delta = (u64::from(wire_seq).wrapping_sub(prev_abs)) & 0x0F;
    prev_abs.wrapping_add(delta)
}

pub fn build_retransmit_buffer<'a>(frames: impl IntoIterator<Item = &'a [u8]>) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.push(MESSAGE_SYNC);
    for frame in frames {
        buf.extend_from_slice(frame);
    }
    buf
}

#[cfg(test)]
mod tests;
