//! Back-to-back nudges (FORCE_MOVE) on one stepcompress lane, driven through
//! the real nudge profile, the real span cutter and the real shim, then
//! replayed against a model of `src/stepper_classic.c`'s `stepper_load_next`.

use super::*;

use std::sync::atomic::AtomicU64;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

const MCU_ID: u32 = 1;
const OID: u32 = 0;
const AXIS: u8 = 3;
const CYCLES_PER_SECOND: f64 = 50_000_000.0;
const ROTATION_DISTANCE: f64 = 7.73;
const MICROSTEP_DISTANCE: f64 = ROTATION_DISTANCE / (200.0 * 16.0);
const STEP_PULSE_TICKS: i64 = 100;
const MIN_REARM_CYCLES: u64 = STEP_REARM_PULSES * STEP_PULSE_TICKS as u64;
const NUDGE_MM: f64 = 0.3;
const LEAD_SECS: f64 = 0.25;

struct Bench {
    endpoint: StepcompressEndpoint,
    now: Arc<AtomicU64>,
    frames: Arc<Mutex<Vec<StepFrame>>>,
    _heartbeats: crossbeam_channel::Receiver<PumpMsg>,
}

fn bench() -> Bench {
    let now = Arc::new(AtomicU64::new(0));
    let now_for_clock = Arc::clone(&now);
    let clock_of: ClockSource =
        Arc::new(move |_| Some((now_for_clock.load(Ordering::Relaxed), CYCLES_PER_SECOND)));
    let frames = Arc::new(Mutex::new(Vec::new()));
    let frames_for_egress = Arc::clone(&frames);
    let egress: FrameEgress = Arc::new(move |burst| {
        for (name, args) in burst {
            let arg = |key: &str| -> i64 {
                match args.iter().find(|(k, _)| k == key).map(|(_, v)| v) {
                    Some(ArgValue::Int(v)) => *v,
                    other => panic!("missing int arg {key}: {other:?}"),
                }
            };
            let frame = match *name {
                "queue_step" => StepFrame::QueueStep {
                    oid: arg("oid") as u32,
                    interval: arg("interval") as u32,
                    count: arg("count") as u16,
                    add: arg("add") as i16,
                },
                "set_next_step_dir" => StepFrame::SetNextStepDir {
                    oid: arg("oid") as u32,
                    dir: arg("dir") as u8,
                },
                "reset_step_clock" => StepFrame::ResetStepClock {
                    oid: arg("oid") as u32,
                    clock: arg("clock") as u32,
                },
                _ => continue,
            };
            frames_for_egress.lock_ok().push(frame);
        }
        Ok(())
    });
    let motor = MotorConfig {
        oid: OID,
        microstep_distance: MICROSTEP_DISTANCE,
        invert_dir: true,
        cycles_per_second: CYCLES_PER_SECOND,
        min_rearm_cycles: MIN_REARM_CYCLES,
        encoder: StepEncoder::Classic {
            max_error_ticks: step_shim::compress::DEFAULT_MAX_ERROR_TICKS,
        },
    };
    let (tx, _rx) = crossbeam_channel::unbounded();
    let endpoint = StepcompressEndpoint::new(
        MCU_ID,
        StepShim::new(vec![motor], SHIM_RING_DEPTH),
        &[StepLaneConfig {
            axis: AXIS as usize,
            oid: OID,
        }],
        egress,
        tx,
        clock_of,
        1024,
        Arc::new(|_| Ok(0)),
        None,
        BARRIER_ACK_DEADLINE_SECONDS,
    )
    .expect("one motor on one axis builds a stepcompress endpoint");
    Bench {
        endpoint,
        now,
        frames,
        _heartbeats: _rx,
    }
}

/// The projection the pump sink freezes for a stepcompress mcu: an affine,
/// unrounded host-seconds -> mcu-cycles map.
fn project_exact(host_ref: f64, mcu_ref: f64) -> impl Fn(u32, f64) -> f64 {
    move |_, host_secs| mcu_ref + (host_secs - host_ref) * CYCLES_PER_SECOND
}

/// One nudge as the planner's profile, anchored on the mcu clock and cut into
/// the bounded views the endpoint admits. `base_mm` is where the lane already
/// stands: a nudge profile travels from zero, so a follow-up nudge carries the
/// displacement of the one before it as a hold term and the seam stays
/// position-continuous.
fn nudge_spans(
    delta_mm: f64,
    base_mm: f64,
    t_start_base: f64,
    t0: f64,
    host_now: f64,
    host_ref: f64,
    mcu_ref: f64,
) -> (Vec<ClockedMotorSpan>, f64) {
    let profile = NudgeProfile::try_new(delta_mm, 5.0, 1000.0, t_start_base).unwrap();
    let dur = profile.duration();
    let (t_start, t_end) = (profile.t_start(), profile.t_end());
    let signal = MotorSpan::try_new(
        Arc::from(vec![
            MotorGroup::Independent(MotorTerm {
                source_axis: AXIS as usize,
                axis: ContinuousAxis::Nudge(profile),
                scale: 1.0,
            }),
            MotorGroup::Independent(MotorTerm {
                source_axis: AXIS as usize,
                axis: ContinuousAxis::Hold {
                    position: base_mm,
                    t_start,
                    t_end,
                },
                scale: 1.0,
            }),
        ]),
        t_start,
        t_end,
        1,
        u32::MAX,
        false,
    )
    .unwrap();
    let spans = crate::enqueue::clock_span(
        Arc::new(signal),
        MCU_ID,
        AXIS as usize,
        &crate::enqueue::EnqueueCtx {
            t0,
            epoch: crate::anchor::StreamEpoch::Continuation,
            host_now,
            lead_secs: LEAD_SECS,
            epoch_freq: &|_| None,
            project_exact: project_exact(host_ref, mcu_ref),
            clock_freq_hz: &|_| CYCLES_PER_SECOND,
            lane_is_phase: &|_| false,
        },
    )
    .unwrap();
    assert!(
        spans.len() > 1,
        "a nudge longer than the dispatch span bound must arrive as several views"
    );
    (spans, dur)
}

/// Faithful replay of `stepper_event_full` + `stepper_load_next` from
/// `src/stepper_classic.c` under `CONFIG_MCU_SIM` (two scheduler events per
/// step, `min_next_time = waketime + step_pulse_ticks`). Returns the worst
/// `behind` the mcu would log as `motion.step_load_late`.
#[derive(Default)]
struct Stepper {
    next_step_time: i64,
    waketime: i64,
    count: i64,
    interval: i64,
    add: i64,
    worst_behind: i64,
}

impl Stepper {
    fn reset_clock(&mut self, clock: i64) {
        assert_eq!(self.count, 0, "Can't reset time when stepper active");
        self.next_step_time = clock;
        self.waketime = clock;
    }

    /// `stepper_load_next`. `was_active` mirrors the C: the caller has not
    /// written `s->count` back yet when it loads from the end of a move.
    fn load_next(&mut self, queue: &mut std::collections::VecDeque<(i64, i64, i64)>) -> bool {
        let Some((interval, count, add)) = queue.pop_front() else {
            self.count = 0;
            return false;
        };
        let was_active = self.count != 0;
        let min_next_time = self.waketime;
        self.add = add;
        self.interval = interval + add;
        self.next_step_time += interval;
        self.waketime = self.next_step_time;
        self.count = count * 2;
        if was_active && self.next_step_time < min_next_time {
            let behind = self.next_step_time - min_next_time;
            self.worst_behind = self.worst_behind.min(behind);
            self.waketime = min_next_time;
        }
        true
    }

    fn run(&mut self, queue: &mut std::collections::VecDeque<(i64, i64, i64)>) {
        if !self.load_next(queue) {
            return;
        }
        loop {
            let min_next_time = self.waketime + STEP_PULSE_TICKS;
            let count = self.count - 1;
            if count & 1 == 1 {
                self.count = count;
                self.waketime = min_next_time;
                continue;
            }
            if count != 0 {
                self.next_step_time += self.interval;
                self.interval += self.add;
                if self.next_step_time < min_next_time {
                    self.count = count;
                    self.waketime = min_next_time;
                } else {
                    self.count = count;
                    self.waketime = self.next_step_time;
                }
                continue;
            }
            self.waketime = min_next_time;
            if !self.load_next(queue) {
                return;
            }
        }
    }
}

/// Replay the captured wire frames against the mcu model, honouring the
/// command ordering: a `reset_step_clock` may only land on an idle stepper,
/// so everything queued before it is executed first.
fn worst_step_load_late(frames: &[StepFrame]) -> i64 {
    let mut mcu = Stepper::default();
    let mut queue = std::collections::VecDeque::new();
    for frame in frames {
        match *frame {
            StepFrame::ResetStepClock { clock, .. } => {
                mcu.run(&mut queue);
                mcu.reset_clock(i64::from(clock));
            }
            StepFrame::SetNextStepDir { .. } => {}
            StepFrame::QueueStep {
                interval,
                count,
                add,
                ..
            } => queue.push_back((i64::from(interval), i64::from(count), i64::from(add))),
            StepFrame::QueueStepHp { .. } => {
                panic!("the classic mcu model cannot replay an hp frame")
            }
        }
    }
    mcu.run(&mut queue);
    mcu.worst_behind
}

impl Bench {
    fn push(&mut self, spans: Vec<ClockedMotorSpan>) -> Result<(), SendError> {
        self.endpoint.send_frames(
            MCU_ID,
            &[AxisFrame {
                axis: AXIS,
                spans,
                new_head: 0,
                room: SHIM_RING_DEPTH,
                guard_recorded_ns: 0,
                guard_mcu_clock: 0,
            }],
        )
    }

    fn advance_to(&mut self, clock: u64) -> Result<(), SendError> {
        let step = (CYCLES_PER_SECOND * 0.01) as u64;
        let mut at = self.now.load(Ordering::Relaxed);
        while at < clock {
            at = (at + step).min(clock);
            self.now.store(at, Ordering::Relaxed);
            self.endpoint.tick()?;
        }
        Ok(())
    }
}

/// Two nudges on one lane with nothing draining between them: the wire stream
/// the endpoint produces must be one a classic stepper can execute — no run
/// loaded inside the pending unstep of the run before it, which is what
/// "Stepper too far in past" reports.
#[test]
fn back_to_back_nudges_stay_ahead_of_the_mcu_pending_unstep() {
    let mut b = bench();
    let host_now = 10.0_f64;
    let mcu_ref = (host_now * CYCLES_PER_SECOND) as u64 as f64;
    b.now.store(
        (mcu_ref - LEAD_SECS * CYCLES_PER_SECOND) as u64,
        Ordering::Relaxed,
    );

    let t0 = host_now + LEAD_SECS;
    let (first, dur) = nudge_spans(NUDGE_MM, 0.0, 0.0, t0, host_now, host_now, mcu_ref);
    b.push(first).unwrap();
    let (second, dur2) = nudge_spans(-NUDGE_MM, NUDGE_MM, dur, t0, host_now, host_now, mcu_ref);
    b.push(second).unwrap();
    b.advance_to(((t0 + dur + dur2 + 0.5) * CYCLES_PER_SECOND) as u64)
        .unwrap();

    let frames = b.frames.lock_ok().clone();
    assert!(
        frames
            .iter()
            .filter(|f| matches!(f, StepFrame::SetNextStepDir { .. }))
            .count()
            == 2,
        "the second nudge must reverse direction, or this exercises nothing"
    );
    let worst = worst_step_load_late(&frames);
    assert!(
        worst == 0,
        "the second nudge loaded {} cycles behind the mcu's pending unstep \
         ({MIN_REARM_CYCLES} cycles of re-arm)",
        -worst
    );
}
