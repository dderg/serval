use crate::clock::Clock;
use std::sync::mpsc::sync_channel;
use std::time::Duration;

use crate::host_io::fire_and_forget_depth::FIRE_AND_FORGET_HIGH_WATER;
use crate::host_io::parser::ArgValue;
use crate::host_io::test_harness::ReactorHarness;
use crate::host_io::window::MAX_PENDING_BLOCKS;
use crate::host_io::wire::{
    MESSAGE_HEADER_SIZE, MESSAGE_TRAILER_SIZE, extract_packet, pack_blocks,
};
use crate::host_io::{McuHostIo, ReactorCommand};
use crate::transport::TransportError;

const DRAIN_TICK_LIMIT: usize = 500;

fn fill_window(h: &mut ReactorHarness) -> Vec<Vec<u8>> {
    let payloads: Vec<Vec<u8>> = (0..MAX_PENDING_BLOCKS).map(|i| vec![i as u8]).collect();
    for payload in &payloads {
        let (tx, _rx) = sync_channel(1);
        h.reactor
            .dispatch_submission(
                u64::from(payload[0]),
                payload.clone(),
                "noop".into(),
                tx,
                h.clock.now() + Duration::from_secs(60),
            )
            .expect("submission dispatches into an empty window");
    }
    assert!(crate::host_io::window::is_full(&h.reactor.unacked_window));
    payloads
}

fn fill_to_high_water(h: &mut ReactorHarness) -> Vec<Vec<u8>> {
    let payloads: Vec<Vec<u8>> = (0..FIRE_AND_FORGET_HIGH_WATER)
        .map(|i| vec![0xF0, i as u8])
        .collect();
    for payload in &payloads {
        h.reactor
            .dispatch_fire_and_forget(payload.clone(), false)
            .expect("a payload the reactor accepts is never refused");
    }
    assert_eq!(
        h.reactor.outbound.pending_fire_and_forget.len(),
        FIRE_AND_FORGET_HIGH_WATER
    );
    assert!(h.reactor.outbound.fire_and_forget_depth.at_high_water());
    payloads
}

fn burst_payloads() -> Vec<Vec<u8>> {
    (0..40u8).map(|i| vec![i; 20]).collect()
}

fn written_command_bytes(h: &ReactorHarness) -> Vec<u8> {
    let mut wire = h.tx_log();
    let mut bytes = Vec::new();
    while let Some(packet) = extract_packet(&mut wire) {
        let crc_off = packet.len() - MESSAGE_TRAILER_SIZE;
        bytes.extend_from_slice(&packet[MESSAGE_HEADER_SIZE..crc_off]);
    }
    bytes
}

fn drain_until_quiet(h: &mut ReactorHarness) {
    for _ in 0..DRAIN_TICK_LIMIT {
        if h.reactor.outbound.pending_outbound_order.is_empty() {
            return;
        }
        h.feed_ack_all();
        h.tick();
    }
    panic!(
        "outbound queues never drained: {} fire-and-forget blocks left",
        h.reactor.outbound.pending_fire_and_forget.len()
    );
}

#[test]
fn a_batch_accepted_past_the_high_water_mark_keeps_every_block_in_order() {
    let mut h = ReactorHarness::new();
    let burst = burst_payloads();
    let blocks = pack_blocks(&burst).expect("the burst packs");
    assert!(
        blocks.len() > 1,
        "the burst must span several blocks for this to say anything about ordering"
    );
    h.reactor
        .outbound
        .fire_and_forget_depth
        .reserve(blocks.len())
        .expect("admitted before the backlog reached its high water mark");
    let submissions = fill_window(&mut h);
    let filler = fill_to_high_water(&mut h);

    h.submission_tx
        .send(ReactorCommand::FireAndForgetBatch {
            blocks: blocks.clone(),
            enqueued_at: std::time::Instant::now(),
        })
        .expect("the reactor is listening");
    h.tick();

    assert_eq!(
        h.reactor.outbound.pending_fire_and_forget.len(),
        FIRE_AND_FORGET_HIGH_WATER + blocks.len(),
        "a batch the reactor accepted must be queued whole, not trimmed to the high water mark"
    );
    assert_eq!(
        h.reactor
            .outbound
            .pending_fire_and_forget
            .iter()
            .skip(FIRE_AND_FORGET_HIGH_WATER)
            .map(|(payload, _, _)| payload.clone())
            .collect::<Vec<_>>(),
        blocks,
        "the batch must queue behind the backlog in packing order"
    );

    drain_until_quiet(&mut h);

    let expected: Vec<u8> = submissions
        .iter()
        .chain(filler.iter())
        .chain(burst.iter())
        .flatten()
        .copied()
        .collect();
    assert_eq!(
        written_command_bytes(&h),
        expected,
        "every accepted command must reach the wire exactly once, in submission order"
    );
}

#[test]
fn the_published_depth_reopens_once_the_backlog_drains() {
    let mut h = ReactorHarness::new();
    fill_window(&mut h);
    fill_to_high_water(&mut h);

    drain_until_quiet(&mut h);

    assert_eq!(h.reactor.outbound.fire_and_forget_depth.queued_blocks(), 0);
    assert!(!h.reactor.outbound.fire_and_forget_depth.at_high_water());
}

#[test]
fn a_batch_is_refused_whole_while_the_reactor_is_at_its_high_water_mark() {
    let (tx, rx) = std::sync::mpsc::channel();
    let io = McuHostIo::from_submission_tx_for_test(tx, None, queue_step_parser());
    let frames = vec![queue_step(100)];

    io.fire_and_forget_depth()
        .publish(FIRE_AND_FORGET_HIGH_WATER);
    let refused = io.send_args_batch(&frames).expect_err("gate must refuse");
    assert!(
        matches!(refused, TransportError::Backpressure),
        "a bulk sender must be told to retry the whole burst, got {refused:?}"
    );
    assert!(rx.try_recv().is_err(), "a refused burst queues nothing");

    io.fire_and_forget_depth()
        .publish(FIRE_AND_FORGET_HIGH_WATER - 1);
    io.send_args_batch(&frames)
        .expect("below the high water mark the gate lets the burst through");
}

#[test]
fn concurrent_admissions_cannot_collectively_pass_the_high_water_mark() {
    use crate::host_io::fire_and_forget_depth::FireAndForgetDepth;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};

    let depth = Arc::new(FireAndForgetDepth::default());
    let admitted = Arc::new(AtomicUsize::new(0));
    let threads: Vec<_> = (0..8)
        .map(|_| {
            let depth = Arc::clone(&depth);
            let admitted = Arc::clone(&admitted);
            std::thread::spawn(move || {
                for _ in 0..FIRE_AND_FORGET_HIGH_WATER {
                    if depth.reserve(1).is_ok() {
                        admitted.fetch_add(1, Ordering::Relaxed);
                    }
                }
            })
        })
        .collect();
    for t in threads {
        t.join().expect("no reserver panics");
    }

    assert_eq!(
        admitted.load(Ordering::Relaxed),
        FIRE_AND_FORGET_HIGH_WATER,
        "racing senders must not admit more single-block bursts than the high water mark"
    );
    assert_eq!(depth.queued_blocks(), FIRE_AND_FORGET_HIGH_WATER);
    assert!(depth.at_high_water());
}

#[test]
fn a_reservation_the_reactor_processed_reopens_the_gate() {
    use crate::host_io::fire_and_forget_depth::FireAndForgetDepth;

    let depth = FireAndForgetDepth::default();
    depth
        .reserve(FIRE_AND_FORGET_HIGH_WATER)
        .expect("the first burst is admitted");
    assert!(depth.at_high_water());
    let refused = depth.reserve(1).expect_err("the gate is shut");
    assert!(
        matches!(refused, TransportError::Backpressure),
        "{refused:?}"
    );

    depth.release(FIRE_AND_FORGET_HIGH_WATER);

    assert_eq!(depth.queued_blocks(), 0);
    depth.reserve(1).expect("the gate reopened");
}

#[test]
fn shutdown_zeroes_the_depth_and_refuses_later_bursts() {
    let mut h = ReactorHarness::new();
    fill_window(&mut h);
    fill_to_high_water(&mut h);

    h.submission_tx
        .send(ReactorCommand::Shutdown)
        .expect("the reactor is listening");
    h.tick();

    let depth = &h.reactor.outbound.fire_and_forget_depth;
    assert_eq!(depth.queued_blocks(), 0, "shutdown must publish zero");
    assert!(!depth.at_high_water());
    let refused = depth
        .reserve(1)
        .expect_err("a closed reactor admits nothing");
    assert!(matches!(refused, TransportError::Closed), "{refused:?}");
}

fn queue_step_parser() -> std::sync::Arc<crate::host_io::parser::MsgProtoParser> {
    let mut commands = indexmap::IndexMap::new();
    commands.insert(
        "queue_step oid=%c interval=%u count=%hu add=%hi".to_string(),
        20,
    );
    commands.insert("kalico_wire_probe clock=%u".to_string(), 21);
    let dictionary = crate::host_io::parser::DataDictionary {
        commands,
        responses: indexmap::IndexMap::new(),
        output: indexmap::IndexMap::new(),
        enumerations: indexmap::IndexMap::new(),
        config: serde_json::json!({}),
        version: "v".into(),
        app: "kalico".into(),
        build_versions: None,
        license: None,
    };
    std::sync::Arc::new(
        crate::host_io::parser::MsgProtoParser::from_dictionary(dictionary)
            .expect("a two-command dictionary"),
    )
}

fn queue_step(interval: i64) -> (&'static str, Vec<(String, ArgValue)>) {
    (
        "queue_step",
        vec![
            ("oid".to_string(), ArgValue::Int(6)),
            ("interval".to_string(), ArgValue::Int(interval)),
            ("count".to_string(), ArgValue::Int(4)),
            ("add".to_string(), ArgValue::Int(0)),
        ],
    )
}

/// The bench's short-rest resume handed the transport 299 step commands at
/// once. They pack into a fraction as many blocks; claiming one block per
/// command shut the gate on the wire probe and the next pass's burst until
/// the reactor got the cpu back.
#[test]
fn a_burst_claims_the_blocks_it_packs_into_not_one_per_command() {
    let (tx, rx) = std::sync::mpsc::channel();
    let io = McuHostIo::from_submission_tx_for_test(tx, None, queue_step_parser());
    let burst: Vec<_> = (0..299).map(|i| queue_step(200_000 + i)).collect();
    let headroom = 80;
    io.fire_and_forget_depth()
        .publish(FIRE_AND_FORGET_HIGH_WATER - headroom);

    io.send_args_batch(&burst)
        .expect("the burst fits the headroom");
    let Ok(ReactorCommand::FireAndForgetBatch { blocks, .. }) = rx.try_recv() else {
        panic!("the burst reached the reactor's channel");
    };
    assert!(
        blocks.len() < headroom,
        "{} blocks for {} commands",
        blocks.len(),
        burst.len()
    );
    assert_eq!(
        io.fire_and_forget_depth().queued_blocks(),
        FIRE_AND_FORGET_HIGH_WATER - headroom + blocks.len()
    );
    io.send_args_batch(&[(
        "kalico_wire_probe",
        vec![("clock".to_string(), ArgValue::Int(7))],
    )])
    .expect("the probe still fits behind the burst");
}
