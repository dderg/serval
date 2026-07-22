#![cfg(feature = "motion-module-stepper")]

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

use runtime::dispatch_stepper::dispatch_axis;
use runtime::engine::Engine;
use runtime::error::FaultCode;
use runtime::error::RUNTIME_OK;
use runtime::piece_ring::PieceEntry;
use runtime::state::SharedState;
use runtime::step_queue::pop as queue_pop;
use runtime::step_queue::{STEP_QUEUE_DEPTH, StepQueue};
use runtime::stepping_state::{AxisConfig, MAX_STEPPERS_PER_AXIS, StepMode, StepperRef};
use runtime::stepping_state::{MAX_AXES, StepperBindingRust, TMC_CS_OID_NONE};

fn make_stepper() -> StepperRef {
    StepperRef {
        stepper_oid: 0,
        position_count: AtomicI32::new(0),
        overlay_step_frame: AtomicI32::new(0),
        tmc_cs_oid: None,
        last_coil_A: AtomicI16::new(0),
        last_coil_B: AtomicI16::new(0),
        phase_offset_microsteps: AtomicI32::new(0),
        phase_offset_target: AtomicI32::new(0),
        last_phase_target: AtomicI32::new(0),
    }
}

fn make_axis(mode: StepMode, microstep_distance: f32) -> AxisConfig {
    let mut steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers.push(make_stepper());
    AxisConfig {
        mode: AtomicU8::new(mode as u8),
        steppers,
        microstep_distance,
        ..AxisConfig::new_unconfigured()
    }
}

#[test]
fn pulse_partial_push_commits_position_count_for_pushed_steps() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    q.tail = (STEP_QUEUE_DEPTH as u16) - 1;
    q.head = 0;
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        0,
        q_ptr,
        &shared,
        0.05,
        2000.0,
        0.0,
        25e-6,
        1_000,
        520_000_000.0,
        /* overlay_just_armed */ false,
    );

    assert_eq!(
        q.tail.wrapping_sub(q.head),
        STEP_QUEUE_DEPTH as u16,
        "queue should be exactly full (31 prefill + 1 push)"
    );
    assert_eq!(
        axis.last_step_count, 1,
        "last_step_count must reflect pushes that landed, not requested target"
    );
    assert_eq!(
        axis.steppers[0].position_count.load(Ordering::Acquire),
        1,
        "position_count must commit for pushed steps before fault"
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepQueueOverflow.as_i32()
    );
}

#[test]
fn pulse_queue_overflow_latches_fault() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    q.tail = STEP_QUEUE_DEPTH as u16;
    q.head = 0;
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        2,
        &mut axis,
        0,
        q_ptr,
        &shared,
        0.0125,
        1000.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
        /* overlay_just_armed */ false,
    );

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepQueueOverflow.as_i32()
    );
    assert_eq!(shared.queue_overflow_count[2].load(Ordering::Acquire), 1);
}

#[test]
fn pulse_steps_per_sample_exceeded_hard_faults() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        1,
        &mut axis,
        0,
        q_ptr,
        &shared,
        0.5,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
        /* overlay_just_armed */ false,
    );

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepsPerSampleExceeded.as_i32(),
        "over-threshold delta must latch StepsPerSampleExceeded"
    );
    assert_eq!(q.tail, q.head, "no steps may be enqueued on overrun");
    assert_eq!(
        axis.last_step_count, 0,
        "baseline must not advance on fault"
    );
    assert_eq!(
        shared.fault_detail.load(Ordering::Acquire),
        (1u32 << 16) | 40,
        "fault_detail encodes axis index and saturated step count"
    );
}

const TIMING_CLOCK_HZ: u32 = 520_000_000;
const TIMING_MICROSTEP_DISTANCE: f32 = 0.001;
const TIMING_VELOCITY_MM_PER_SEC: f32 = 40.0;

fn constant_velocity_piece(start_time: u64, duration: f32) -> PieceEntry {
    let half_distance = TIMING_VELOCITY_MM_PER_SEC * duration * 0.5;
    let mut coeffs = [0.0; 8];
    coeffs[0] = half_distance;
    coeffs[1] = half_distance;
    PieceEntry {
        start_time,
        duration,
        motor_mask: 0,
        coeff_count: 2,
        _reserved: [0; 2],
        coeffs,
    }
}

#[allow(unsafe_code)]
fn drain_step_times(queue: &mut StepQueue, times: &mut std::vec::Vec<u32>) {
    let queue_ptr: *mut StepQueue = queue;
    while let Some(entry) = unsafe { queue_pop(queue_ptr) } {
        times.push(entry.cycle_abs);
    }
}

fn assert_tick_dispatch_matches_curve(sample_rate_hz: u32) {
    let mut engine = Engine::new(TIMING_CLOCK_HZ, sample_rate_hz);
    let sample_period_cycles = u64::from(engine.sample_period_cycles);
    let t0 = sample_period_cycles;
    let mut storage = vec![PieceEntry::zeroed(); 64];
    let bindings = [StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }];
    assert_eq!(
        engine.configure_axis(
            0,
            StepMode::Pulse,
            TIMING_MICROSTEP_DISTANCE,
            64,
            &bindings,
            storage.len(),
        ),
        RUNTIME_OK
    );

    let mut queue = StepQueue::new();
    let mut queues = [core::ptr::null_mut(); MAX_AXES];
    queues[0] = &mut queue;
    engine.test_install_step_queues(queues);
    let shared = SharedState::new();
    assert_eq!(
        engine.push_pieces(0, &[constant_velocity_piece(t0, 0.005)], &mut storage),
        RUNTIME_OK
    );

    let mut dispatched_times = std::vec::Vec::new();
    let mut discarded_steps = 0_usize;
    for sample_index in 0..=10 {
        engine.tick(
            t0 + sample_index * sample_period_cycles,
            &shared,
            &mut storage,
        );
        let mut sample_times = std::vec::Vec::new();
        drain_step_times(&mut queue, &mut sample_times);
        if sample_index <= 1 {
            discarded_steps += sample_times.len();
        } else {
            dispatched_times.extend(sample_times);
        }
    }

    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(!dispatched_times.is_empty());

    let cycles_per_mm = f64::from(TIMING_CLOCK_HZ) / f64::from(TIMING_VELOCITY_MM_PER_SEC);
    let mut max_abs_error_cycles = 0.0_f64;
    let mut total_error_cycles = 0.0_f64;
    for (offset, &cycle_abs) in dispatched_times.iter().enumerate() {
        let step_position = (discarded_steps + offset) as f64
            * f64::from(TIMING_MICROSTEP_DISTANCE)
            + f64::from(TIMING_MICROSTEP_DISTANCE) * 0.5;
        let crossing_cycle = t0 as f64 + step_position * cycles_per_mm;
        let error_cycles = f64::from(cycle_abs) - crossing_cycle;
        max_abs_error_cycles = max_abs_error_cycles.max(error_cycles.abs());
        total_error_cycles += error_cycles;
    }

    let mean_error_cycles = total_error_cycles / dispatched_times.len() as f64;
    let inter_step_cycles = f64::from(TIMING_MICROSTEP_DISTANCE) * cycles_per_mm;
    let sample_period_cycles_f64 = sample_period_cycles as f64;
    let tolerance_cycles = (inter_step_cycles * 0.5).min(sample_period_cycles_f64 * 0.25);
    assert!(
        max_abs_error_cycles < tolerance_cycles,
        "sample_rate_hz={sample_rate_hz}, mean_offset_cycles={mean_error_cycles:.1}, mean_offset_us={:.3}, max_abs_error_cycles={max_abs_error_cycles:.1}, tolerance_cycles={tolerance_cycles:.1}",
        mean_error_cycles * 1_000_000.0 / f64::from(TIMING_CLOCK_HZ),
    );
}

#[test]
fn tick_dispatch_tracks_curve_crossings_at_100us() {
    assert_tick_dispatch_matches_curve(10_000);
}

#[test]
fn tick_dispatch_tracks_curve_crossings_at_200us() {
    assert_tick_dispatch_matches_curve(5_000);
}
