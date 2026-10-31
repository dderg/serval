use host_rt::host_io::wire::{
    BLOCK_PAYLOAD_MAX, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_SEQ_MASK, decode_absolute, pack_blocks,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

const MAX_COMMANDS: usize = 40;
const SEQ_EPOCH: u64 = 16;

/// Fewest blocks an ordered, unsplittable command list can occupy, by dynamic
/// programming over every legal cut. Independent of the greedy walk under test:
/// a contiguous partition has an optimal-substructure minimum, so `pack_blocks`
/// must land on exactly this count or it is wasting sequence numbers.
fn fewest_blocks(lens: &[usize]) -> Option<usize> {
    let mut best = vec![usize::MAX; lens.len() + 1];
    best[0] = 0;
    for end in 1..=lens.len() {
        let mut span = 0usize;
        for start in (0..end).rev() {
            span += lens[start];
            if span > BLOCK_PAYLOAD_MAX {
                break;
            }
            if best[start] != usize::MAX {
                best[end] = best[end].min(best[start] + 1);
            }
        }
    }
    (best[lens.len()] != usize::MAX).then_some(best[lens.len()])
}

/// Byte offsets a block may end on: the end of any command.
fn command_edges(lens: &[usize]) -> Vec<usize> {
    let mut edges = vec![0usize];
    let mut at = 0usize;
    for len in lens {
        at += len;
        edges.push(at);
    }
    edges
}

/// Length of the first command of each block, by replaying the command list
/// against the produced block sizes. Fails when a block boundary falls inside a
/// command.
fn first_command_of_each_block(lens: &[usize], blocks: &[Vec<u8>]) -> Option<Vec<usize>> {
    let mut heads = Vec::with_capacity(blocks.len());
    let mut command = 0usize;
    let mut consumed = 0usize;
    for block in blocks {
        heads.push(*lens.get(command)?);
        let block_end = consumed + block.len();
        while consumed < block_end {
            consumed += lens.get(command)?;
            command += 1;
        }
        if consumed != block_end {
            return None;
        }
    }
    (command == lens.len()).then_some(heads)
}

fn arb_commands(min_len: usize, max_len: usize) -> impl Strategy<Value = Vec<Vec<u8>>> {
    prop::collection::vec(
        prop::collection::vec(any::<u8>(), min_len..=max_len),
        0..=MAX_COMMANDS,
    )
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/wire_blocks_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn packing_is_lossless_maximal_and_uses_fewest_blocks(
        commands in arb_commands(1, BLOCK_PAYLOAD_MAX),
    ) {
        let lens: Vec<usize> = commands.iter().map(Vec::len).collect();
        let blocks = pack_blocks(&commands).expect("every command fits one block");

        prop_assert_eq!(blocks.concat(), commands.concat());

        for block in &blocks {
            prop_assert!(
                block.len() <= BLOCK_PAYLOAD_MAX,
                "block of {} bytes exceeds the {BLOCK_PAYLOAD_MAX}-byte payload",
                block.len()
            );
            prop_assert!(MESSAGE_MIN + block.len() <= MESSAGE_MAX);
        }

        prop_assert_eq!(
            Some(blocks.len()),
            fewest_blocks(&lens),
            "block count {} is not the minimum for lens {:?}",
            blocks.len(),
            lens
        );

        let edges = command_edges(&lens);
        let mut at = 0usize;
        for block in &blocks {
            at += block.len();
            prop_assert!(
                edges.contains(&at),
                "block boundary at byte {at} splits a command; edges {edges:?}"
            );
        }

        let heads = first_command_of_each_block(&lens, &blocks)
            .expect("block boundaries align with command boundaries");
        for pair in blocks.windows(2).zip(heads.iter().skip(1)) {
            let (open, next_head) = (&pair.0[0], *pair.1);
            prop_assert!(
                open.len() + next_head > BLOCK_PAYLOAD_MAX,
                "block of {} bytes was closed while a {next_head}-byte command still fit",
                open.len()
            );
        }
    }

    #[test]
    fn a_command_wider_than_a_block_is_rejected_naming_its_size(
        commands in arb_commands(0, BLOCK_PAYLOAD_MAX + 8),
    ) {
        let first_oversize = commands.iter().find(|c| c.len() > BLOCK_PAYLOAD_MAX);
        match (pack_blocks(&commands), first_oversize) {
            (Err(err), Some(offender)) => prop_assert!(
                err.starts_with(&format!("encoded command is {} bytes", offender.len())),
                "error must name the first oversize command ({} bytes): {err}",
                offender.len()
            ),
            (Ok(blocks), None) => {
                prop_assert_eq!(blocks.concat(), commands.concat());
                for block in &blocks {
                    prop_assert!(block.len() <= BLOCK_PAYLOAD_MAX);
                }
            }
            (Ok(_), Some(offender)) => prop_assert!(
                false,
                "a {}-byte command was packed into a {BLOCK_PAYLOAD_MAX}-byte block",
                offender.len()
            ),
            (Err(err), None) => prop_assert!(false, "every command fits, yet: {err}"),
        }
    }

    #[test]
    fn decode_absolute_is_the_unique_nibble_match_in_the_next_epoch(
        prev_abs in prop_oneof![
            0u64..SEQ_EPOCH * 4,
            any::<u64>(),
            (u64::MAX - 2 * SEQ_EPOCH)..=u64::MAX,
        ],
        wire_seq in any::<u8>(),
    ) {
        let got = decode_absolute(prev_abs, wire_seq);

        let candidates: Vec<u64> = (0..SEQ_EPOCH)
            .map(|step| prev_abs.wrapping_add(step))
            .filter(|abs| (*abs as u8) & MESSAGE_SEQ_MASK == wire_seq & MESSAGE_SEQ_MASK)
            .collect();
        prop_assert_eq!(
            candidates.len(),
            1,
            "one epoch must hold exactly one match for nibble {}",
            wire_seq & MESSAGE_SEQ_MASK
        );
        prop_assert_eq!(got, candidates[0]);

        prop_assert!(
            got.wrapping_sub(prev_abs) < SEQ_EPOCH,
            "decoded {got} is more than one epoch past {prev_abs}"
        );
        prop_assert_eq!(
            (got as u8) & MESSAGE_SEQ_MASK,
            wire_seq & MESSAGE_SEQ_MASK,
            "decoded absolute must carry the wire nibble"
        );
        prop_assert_eq!(
            got,
            decode_absolute(prev_abs, wire_seq & MESSAGE_SEQ_MASK),
            "only the sequence nibble of the wire byte may matter"
        );
    }

    #[test]
    fn a_repeated_wire_nibble_does_not_advance_the_absolute_sequence(
        prev_abs in any::<u64>(),
    ) {
        let echoed = (prev_abs as u8) & MESSAGE_SEQ_MASK;
        prop_assert_eq!(decode_absolute(prev_abs, echoed), prev_abs);
    }
}
