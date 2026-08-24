//! Retirement credit for a lane that two endpoints both speak for.

use super::pump_loop::Pump;
use super::*;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::Duration;
use trajectory::ClockedMotorSpan;

struct NullSink;

impl SpanSink for NullSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }
}

const DUAL: AxisKey = AxisKey { mcu_id: 0, axis: 2 };

fn pump_with_pushed(pushed: u32) -> Pump<NullSink> {
    let mut queues = BTreeMap::new();
    let mut q = AxisQueue::new(64);
    q.pushed = pushed;
    queues.insert(DUAL, q);
    Pump {
        queues,
        junctions: JunctionTracker::default(),
        cohort: None,
        halted: BTreeMap::new(),
        sink: NullSink,
        callbacks: PumpCallbacks::noop(64),
        history: None,
        ledger: Arc::new(crate::drain::DrainLedger::new()),
        pending_barrier_acks: Vec::new(),
        backlog: Arc::new(AtomicU64::new(0)),
        holding_ahead: false,
        data_open: true,
        intake_batch_open: false,
        consumption_stall: super::stall::ConsumptionStallWatch::new(Duration::from_secs(60)),
        mem_probe: super::memstat::MemPressureProbe::new(),
    }
}

fn report(pump: &mut Pump<NullSink>, retired_by: RetiredBy, retired: u32) {
    report_split(pump, retired_by, retired, retired);
}

fn report_split(pump: &mut Pump<NullSink>, retired_by: RetiredBy, consumed: u32, retired: u32) {
    pump.handle_control_msg(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: DUAL.mcu_id,
        axes: vec![DUAL.axis],
        consumed_counts: Some(vec![consumed]),
        retired_counts: vec![retired],
        retired_by,
    }));
    pump.publish_ledger();
}

#[test]
fn the_idle_transport_cannot_erase_the_active_transports_credit() {
    let mut pump = pump_with_pushed(5);

    report(&mut pump, RetiredBy::Phase, 5);
    assert!(pump.ledger.drained(), "the phase side finished all 5 spans");

    report(&mut pump, RetiredBy::Pulse, 0);
    assert!(
        pump.ledger.drained(),
        "the pulse endpoint is a member of the same dual lane and keeps reporting its frozen \
         odometer; it must not walk the axis back to unretired: {:?}",
        pump.ledger.lagging_axes()
    );
}

#[test]
fn a_transport_switch_mid_drain_carries_the_credit_already_earned() {
    let mut pump = pump_with_pushed(3);
    report(&mut pump, RetiredBy::Phase, 3);
    assert!(pump.ledger.drained());

    pump.queues.get_mut(&DUAL).unwrap().pushed = 5;
    report(&mut pump, RetiredBy::Pulse, 0);
    assert!(
        !pump.ledger.drained(),
        "2 spans went out through the transport that just adopted the lane"
    );
    report(&mut pump, RetiredBy::Phase, 3);
    assert!(
        !pump.ledger.drained(),
        "the outgoing transport's final odometer covers only its own 3 spans"
    );

    report(&mut pump, RetiredBy::Pulse, 2);
    assert!(
        pump.ledger.drained(),
        "3 retired before the switch plus 2 after it account for all 5 pushed: {:?}",
        pump.ledger.lagging_axes()
    );
    let q = &pump.queues[&DUAL];
    assert_eq!((q.retired, q.consumed), (5, 5));
}

/// An endpoint counts a view consumed when it releases it — it has converted
/// every sample and can take the successor — and retired only once the mcu has
/// played it back. The two odometers move independently, and only retirement
/// drains the lane.
#[test]
fn consumption_frees_the_ring_while_only_playback_drains_the_lane() {
    let mut pump = pump_with_pushed(4);

    report_split(&mut pump, RetiredBy::Pulse, 4, 0);
    let q = &pump.queues[&DUAL];
    assert_eq!(
        (q.consumed, q.retired),
        (4, 0),
        "release credit must not imply playback"
    );
    assert_eq!(q.room(), q.ring_depth, "consumed views hold no ring slot");
    assert!(
        !pump.ledger.drained(),
        "nothing has played back yet: {:?}",
        pump.ledger.lagging_axes()
    );

    report_split(&mut pump, RetiredBy::Pulse, 4, 4);
    assert!(
        pump.ledger.drained(),
        "playback of every pushed view drains the lane: {:?}",
        pump.ledger.lagging_axes()
    );
}

#[test]
fn halt_keeps_pushed_views_until_the_endpoint_publishes_abandonment() {
    let mut pump = pump_with_pushed(100);
    report(&mut pump, RetiredBy::Pulse, 36);
    let (ack, _) = std::sync::mpsc::sync_channel(1);

    pump.handle_control_msg(PumpMsg::Halt {
        keys: vec![DUAL],
        ack,
    });
    assert_eq!(pump.queues[&DUAL].pushed, 100);

    report(&mut pump, RetiredBy::Pulse, 100);
    assert!(
        pump.ledger.drained(),
        "the endpoint abandonment must close every view retained in the pump odometer: {:?}",
        pump.ledger.lagging_axes()
    );
}

#[test]
fn an_axis_no_endpoint_speaks_for_never_drains() {
    let pump = pump_with_pushed(5);
    pump.publish_ledger();

    let error = pump
        .ledger
        .wait_drained(Duration::from_millis(20))
        .expect_err("nothing retires a lane no endpoint owns");
    assert!(
        error.contains("mcu0 axis2: pending 0 pushed 5 retired 0"),
        "{error}"
    );
}
