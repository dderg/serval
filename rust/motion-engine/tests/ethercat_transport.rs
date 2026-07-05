use crossbeam_channel::unbounded;
use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use _motion_engine::drain::DrainLedger;
use _motion_engine::pump::{
    AxisKey, EnqueueMsg, HeartbeatMsg, PieceSink, PumpMsg, SendError, WireSink, run_pump,
};
use runtime::piece_ring::PieceEntry;

fn piece(t: u64) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: t,
            duration: 0.001,
            ..PieceEntry::zeroed()
        },
        t as f64,
    )
}

#[test]
fn wire_sink_missing_transport_is_hard_error() {
    use std::collections::HashMap;

    let sink = WireSink {
        transports: HashMap::new(),
        timeout: Duration::from_secs(1),
        freq_of: Arc::new(|_| None),
    };
    let (p, _) = piece(0);
    let result = sink.send_frame(
        AxisKey {
            mcu_id: 99,
            axis: 0,
        },
        &[p],
        0,
        1,
        8,
    );
    assert!(
        result.is_err(),
        "missing transport must be a hard error, not silent drop"
    );
    let err = result.unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.contains("no transport for mcu_id 99"),
        "error must name the offending mcu_id; got: {msg}"
    );
}

#[derive(Clone)]
struct PerMcuCountSink {
    calls: Arc<Mutex<std::collections::HashMap<u32, u32>>>,
}

impl PerMcuCountSink {
    fn new() -> Self {
        Self {
            calls: Arc::new(Mutex::new(std::collections::HashMap::new())),
        }
    }
}

impl PieceSink for PerMcuCountSink {
    fn send_frame(
        &self,
        key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        *self.calls.lock().unwrap().entry(key.mcu_id).or_insert(0) += 1;
        Ok(0)
    }
}

#[test]
fn pump_routes_both_serial_and_ethercat_mcu_ids() {
    let sink = PerMcuCountSink::new();
    let counts = Arc::clone(&sink.calls);

    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            |_k| 8u32,
            |_| None,
            |_| {},
            |_, _| {},
            std::sync::Arc::new(_motion_engine::drain::DrainLedger::new()),
            |_| {},
            Arc::new(AtomicU64::new(0)),
        );
    });

    data.send(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![piece(0)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
    })
    .unwrap();
    data.send(EnqueueMsg {
        key: AxisKey { mcu_id: 2, axis: 0 },
        pieces: vec![piece(1)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
    })
    .unwrap();

    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    loop {
        let c1 = counts.lock().unwrap().get(&1).copied().unwrap_or(0);
        let c2 = counts.lock().unwrap().get(&2).copied().unwrap_or(0);
        if c1 >= 1 && c2 >= 1 {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "pump did not service both mcu_ids within deadline (mcu1={c1} mcu2={c2})"
        );
        std::thread::yield_now();
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let final_c1 = counts.lock().unwrap().get(&1).copied().unwrap_or(0);
    let final_c2 = counts.lock().unwrap().get(&2).copied().unwrap_or(0);
    assert!(
        final_c1 >= 1,
        "serial MCU (mcu_id=1) must be serviced at least once"
    );
    assert!(
        final_c2 >= 1,
        "EtherCAT MCU (mcu_id=2) must be serviced at least once"
    );
}

#[test]
fn heartbeat_retirement_drains_pump_ledger() {
    let sink = PerMcuCountSink::new();
    let ledger = Arc::new(DrainLedger::new());
    let ledger_pump = Arc::clone(&ledger);

    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            |_k| 8u32,
            |_| None,
            |_| {},
            |_, _| {},
            ledger_pump,
            |_| {},
            Arc::new(AtomicU64::new(0)),
        );
    });

    let barrier = |ctl: &crossbeam_channel::Sender<PumpMsg>| {
        let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
        ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
        ack_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("pump must ack the barrier");
    };

    assert!(ledger.drained(), "empty pump is trivially drained");

    data.send(EnqueueMsg {
        key: AxisKey {
            mcu_id: 42,
            axis: 0,
        },
        pieces: vec![piece(0)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
    })
    .unwrap();
    barrier(&ctl);
    assert!(
        !ledger.drained(),
        "a pushed but unretired wire piece must keep the ledger undrained"
    );

    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 42,
        retired_counts: vec![1],
    }))
    .unwrap();
    barrier(&ctl);
    assert!(
        ledger.drained(),
        "heartbeat retirement matching the pushed count must drain the ledger"
    );
    ledger
        .wait_drained(Duration::from_millis(100))
        .expect("wait_drained agrees");

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}
