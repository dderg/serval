//! Back-to-back nudges (FORCE_MOVE) on one stepcompress lane, driven through
//! the real nudge planner, the real overlay flattener and the real shim, then
//! replayed against a model of `src/stepper_classic.c`'s `stepper_load_next`.

use super::*;

use runtime::piece_ring::PieceEntry;
use std::sync::atomic::AtomicU64;

const MCU_ID: u32 = 1;
const OID: u32 = 0;
const AXIS: u8 = 3;
const CYCLES_PER_SECOND: f64 = 50_000_000.0;
const SAMPLE_RATE_HZ: f64 = 10_000.0;
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
        microstep_distance: MICROSTEP_DISTANCE as f32,
        invert_dir: true,
        max_steps_per_sample: (100.0 / MICROSTEP_DISTANCE / SAMPLE_RATE_HZ).ceil() as u32,
        sample_rate_hz: SAMPLE_RATE_HZ as f32,
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
        vec![AXIS as usize],
        vec![OID],
        egress,
        tx,
        clock_of,
        1024,
    );
    Bench {
        endpoint,
        now,
        frames,
        _heartbeats: _rx,
    }
}

/// The projection the pump sink freezes for a stepcompress mcu: an affine
/// host-seconds -> mcu-cycles map.
fn project(host_ref: f64, mcu_ref: f64) -> impl Fn(u32, f64) -> u64 {
    move |_, host_secs| (mcu_ref + (host_secs - host_ref) * CYCLES_PER_SECOND).round() as u64
}

fn nudge_pieces(
    delta_mm: f64,
    t_start_base: f64,
    t0: f64,
    host_now: f64,
    host_ref: f64,
    mcu_ref: f64,
) -> (Vec<PieceEntry>, f64) {
    let planned =
        crate::nudge::plan_nudge_profile(AXIS, delta_mm, 5.0, 1000.0, 1, t_start_base).unwrap();
    let dur: f64 = planned
        .iter()
        .map(|p| p.piece.u_end - p.piece.u_start)
        .sum();
    let project = project(host_ref, mcu_ref);
    let mut out = Vec::new();
    for np in &planned {
        let flat = crate::enqueue::flatten_bezier_pieces(
            std::slice::from_ref(&np.piece),
            &crate::enqueue::FlattenCtx {
                t0,
                mcu_id: MCU_ID,
                axis_idx: AXIS as usize,
                host_now,
                project: &project,
                max_piece_secs: None,
                motor_mask: np.motor_mask,
            },
        );
        out.extend(flat.into_iter().map(|(entry, _)| entry));
    }
    (out, dur)
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
    fn push(&mut self, pieces: Vec<PieceEntry>) -> Result<(), SendError> {
        self.endpoint.send_frames(
            MCU_ID,
            &[AxisFrame {
                axis: AXIS,
                pieces,
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
    let (first, dur) = nudge_pieces(NUDGE_MM, 0.0, t0, host_now, host_now, mcu_ref);
    b.push(first).unwrap();
    let (second, dur2) = nudge_pieces(-NUDGE_MM, dur, t0, host_now, host_now, mcu_ref);
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
