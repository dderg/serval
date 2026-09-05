import logging
import math
import struct
from collections import defaultdict, namedtuple

from . import stepper
from .extras import servo_axis
from .mcu import SAMPLE_COMMANDS
from .motion_endstop import register_stepcompress_steppers
from .stepper import DEFAULT_STEP_PULSE_DURATION

McuTopology = namedtuple(
    "McuTopology",
    [
        "mcu_id",
        "axes",
        "kinematics",
        "max_motor_velocity",
        "lane_kinds",
        "motor_counts",
        "microstep_distance",
        "invert_dir",
        "stepper_oids",
        "move_queue_slots",
        "step_pulse_seconds",
        "high_precision_step_compress",
        "stepcompress_max_error_secs",
        "phase_sample_rate",
        "phase_ring_depth",
    ],
)

CORNER_DEVIATION_SCV_FACTOR = math.sqrt(2.0) - 1.0
DEFAULT_SQUARE_CORNER_VELOCITY = 5.0


def corner_deviation_from_scv(scv, max_accel):
    return scv * scv * CORNER_DEVIATION_SCV_FACTOR / max_accel


def scv_from_corner_deviation(corner_deviation, max_accel):
    return math.sqrt(corner_deviation * max_accel / CORNER_DEVIATION_SCV_FACTOR)


STEP_MODE_MODULATED = 0
STEP_MODE_STEP_TIME = 1
LANE_KIND_PULSE = 0
LANE_KIND_PHASE = 1
LANE_KIND_PHASE_WITH_PULSE = 2


def lane_kind_of(step_mode, needs_pulse_windows):
    """A phase lane is phase-capable, not phase-only: a phase-stepped motor
    homed on StallGuard also carries a classic step/dir binding, and the host
    routes it by whichever mode it is in."""
    if step_mode != STEP_MODE_MODULATED:
        return LANE_KIND_PULSE
    if needs_pulse_windows:
        return LANE_KIND_PHASE_WITH_PULSE
    return LANE_KIND_PHASE


MOTION_SAMPLE_RATE_CONSTANT = "MOTION_SAMPLE_RATE_HZ"
SAMPLE_RUNS_PER_LANE_CONSTANT = "SAMPLE_RUNS_PER_LANE"
PHASE_STEPPING_CAPABILITY_BIT = 0x1
FIRMWARE_MAX_PHASE_STEPPED_MOTORS = 16
MODE_PULSE = 0
MODE_PHASE = 1
TMC_CS_OID_NONE = 0xFF
FLAGS_DEFAULT = 0
UNUSED_EXTRUSION_PER_XY_BITS = 0


def declared_axis_order(motion):
    return [name for name, _, _, _ in motion.axis_sections]


def build_follower_steppers(motion, config):
    if motion.kinematics_decl is None:
        raise config.error("[kinematics] section is required")
    _kind, _lanes, followers = motion.kinematics_decl
    motion.follower_steppers = [
        stepper.PrinterStepper(
            config.getsection("motor " + motor_name), name=motor_name
        )
        for _axis, motors, _slot in followers
        for motor_name in motors
    ]


STEP_EDGE_FLOOR_SECONDS = 0.000001
STEP_ISR_BUDGET_FRACTION = 0.5


def motor_velocity_ceiling(mcu_stepper):
    """The fastest motor-frame velocity (mm/s) the MCU can physically step:
    the ISR budget fraction of real time divided by the cost of one step —
    the 1us edge floor, plus the pulse-width busy-wait when the driver only
    steps on rising edges. Mirrors the MCU's per-sample step budget in
    src/stepper.c command_kalico_configure_axis."""
    pulse_duration, both_edge = mcu_stepper.get_pulse_duration()
    if pulse_duration is None:
        pulse_duration = DEFAULT_STEP_PULSE_DURATION
    per_step_s = STEP_EDGE_FLOOR_SECONDS + (
        0.0 if both_edge else pulse_duration
    )
    return STEP_ISR_BUDGET_FRACTION / per_step_s * mcu_stepper.get_step_dist()


def step_pulse_width(mcu_stepper):
    """The settle the mcu enforces around every pulse: src/stepper_classic.c
    re-arms its timer at waketime + step_pulse_ticks, both after a step (for
    the unstep) and after a direction toggle. Both-edge drivers step on every
    edge and configure zero ticks."""
    pulse_duration, both_edge = mcu_stepper.get_pulse_duration()
    if both_edge:
        return 0.0
    if pulse_duration is None:
        pulse_duration = DEFAULT_STEP_PULSE_DURATION
    return pulse_duration


def build_axis_to_handle(motion):
    axis_to_handle = {}
    motion._axis_velocity_ceiling = axis_ceiling = {}
    for lane_idx, _axis_name, _motor_names in motion.kin.lanes():
        rail = motion.kin.rails[lane_idx]
        if isinstance(rail, servo_axis.ServoRail):
            node = motion.printer.lookup_object(
                "ethercat_node " + rail.get_node_name(), None
            )
            if node is None:
                continue
            handle = node.get_engine_handle()
            ceiling = float("inf")
        else:
            steppers = rail.get_steppers()
            if not steppers:
                continue
            handle = steppers[0].get_mcu().get_engine_handle()
            ceiling = min(motor_velocity_ceiling(s) for s in steppers)
        if handle is None:
            continue
        axis_to_handle[lane_idx] = handle
        axis_ceiling[lane_idx] = ceiling

    fm = motion.printer.lookup_object("force_move", None)
    for _name, motors, slot_idx in motion._follower_slots():
        if fm is None:
            continue
        followers = [fm.steppers.get(m) for m in motors]
        if any(s is None for s in followers):
            continue
        primary = followers[0]
        handle = primary.get_mcu().get_engine_handle()
        if handle is None:
            continue
        axis_to_handle[slot_idx] = handle
        axis_ceiling[slot_idx] = min(
            motor_velocity_ceiling(s) for s in followers
        )
    return axis_to_handle


def engine_mcus_by_handle(motion):
    by_handle = {}
    for name, mcu in motion.printer.lookup_objects(module="mcu"):
        handle = mcu.get_engine_handle()
        if handle is None:
            continue
        by_handle[handle] = (name, mcu)
    return by_handle


def ethercat_handles(motion):
    handles = set()
    for _name, node in motion.printer.lookup_objects(module="ethercat_node"):
        handle = node.get_engine_handle()
        if handle is not None:
            handles.add(handle)
    return handles


def _reject_phase_lane_conflicts(motion, name, mcu_obj, phase_axes):
    for argstring in SAMPLE_COMMANDS:
        if mcu_obj.try_lookup_command(argstring) is None:
            raise motion.printer.config_error(
                "mcu '%s': axes %s request phase_stepping: 1, whose sample-run "
                "executor needs firmware built with CONFIG_SAMPLE_STEPPING, "
                "but the mcu does not provide the '%s' command. Reflash with "
                "SAMPLE_STEPPING enabled or set phase_stepping: 0."
                % (name, phase_axes, argstring.split(" ")[0])
            )


def _phase_sample_rate(motion, name, mcu_obj, phase_axes):
    rate = mcu_obj.get_constants().get(MOTION_SAMPLE_RATE_CONSTANT)
    if rate is None:
        raise motion.printer.config_error(
            "mcu '%s': axes %s request phase_stepping: 1, but the mcu does not "
            "advertise %s, so its firmware has no sample executor. Reflash "
            "with CONFIG_SAMPLE_STEPPING or set phase_stepping: 0."
            % (name, phase_axes, MOTION_SAMPLE_RATE_CONSTANT)
        )
    rate = float(rate)
    if not math.isfinite(rate) or rate <= 0.0:
        raise motion.printer.config_error(
            "mcu '%s': advertised %s must be a finite positive frequency in "
            "Hz, got %r" % (name, MOTION_SAMPLE_RATE_CONSTANT, rate)
        )
    return rate


def _phase_ring_depth(motion, name, mcu_obj, phase_axes):
    depth = mcu_obj.get_constants().get(SAMPLE_RUNS_PER_LANE_CONSTANT)
    if depth is None:
        raise motion.printer.config_error(
            "mcu '%s': axes %s request phase_stepping: 1, but the mcu does not "
            "advertise %s, so the host cannot pace its sample stream against "
            "the lane ring. Reflash with CONFIG_SAMPLE_STEPPING or set "
            "phase_stepping: 0."
            % (name, phase_axes, SAMPLE_RUNS_PER_LANE_CONSTANT)
        )
    depth = int(depth)
    if depth <= 0:
        raise motion.printer.config_error(
            "mcu '%s': advertised %s must be a positive run count, got %r"
            % (name, SAMPLE_RUNS_PER_LANE_CONSTANT, depth)
        )
    return depth


def _reject_pulse_lane_conflicts(motion, name, mcu_obj, hp_motors):
    if not any(hp_motors):
        return
    if (
        mcu_obj.try_lookup_command(
            "queue_step_hp oid=%c interval=%u count=%hu"
            " add=%hi add2=%hi shift=%hi"
        )
        is not None
    ):
        return
    raise motion.printer.config_error(
        "mcu '%s': motors %s request high_precision_step_compress: True, "
        "but the mcu does not provide queue_step_hp. Reflash with "
        "HIGH_PREC_STEP or disable high_precision_step_compress."
        % (name, hp_motors)
    )


def derive_mcu_topology(motion, axis_to_handle):
    by_handle = {}
    for axis_idx, handle in axis_to_handle.items():
        by_handle.setdefault(handle, []).append(axis_idx)
    ceilings = getattr(motion, "_axis_velocity_ceiling", {})
    mcu_by_handle = engine_mcus_by_handle(motion)
    endpoint_handles = ethercat_handles(motion)
    slot_steppers = motion._build_slot_steppers()
    topo = []
    for handle in sorted(by_handle):
        axes = sorted(by_handle[handle])
        max_motor_velocity = [ceilings.get(a, float("inf")) for a in axes]
        name, mcu_obj = mcu_by_handle.get(handle, (str(handle), None))
        max_error_secs = (
            0.0 if mcu_obj is None else mcu_obj.get_stepcompress_max_error()
        )
        (
            _present_mask,
            _invert_mask,
            _steps_per_mm,
            step_modes,
            bind_list,
            slot_microstep_distance,
            slot_invert_dir,
            slot_oids,
            slot_step_pulse_seconds,
            slot_dual_transport,
        ) = _build_slot_masks(
            motion.printer, mcu_obj, slot_steppers, len(mcu_by_handle)
        )
        lane_kinds = [
            lane_kind_of(step_modes[a], slot_dual_transport[a]) for a in axes
        ]
        phase_axes = [
            a
            for a, k in zip(axes, lane_kinds)
            if k in (LANE_KIND_PHASE, LANE_KIND_PHASE_WITH_PULSE)
        ]
        pulse_axes = [
            a
            for a, k in zip(axes, lane_kinds)
            if k in (LANE_KIND_PULSE, LANE_KIND_PHASE_WITH_PULSE)
        ]
        move_queue_slots = 0
        phase_sample_rate = 0.0
        phase_ring_depth = 0
        if mcu_obj is not None:
            if phase_axes:
                _reject_phase_lane_conflicts(motion, name, mcu_obj, phase_axes)
                phase_sample_rate = _phase_sample_rate(
                    motion, name, mcu_obj, phase_axes
                )
                phase_ring_depth = _phase_ring_depth(
                    motion, name, mcu_obj, phase_axes
                )
            if pulse_axes and handle not in endpoint_handles:
                move_queue_slots = mcu_obj.get_move_queue_slots()
                if move_queue_slots <= 0:
                    raise motion.printer.config_error(
                        "mcu '%s': step/dir axes %s need the mcu's advertised "
                        "move_count, but none was received before the motion "
                        "planner was built" % (name, pulse_axes)
                    )
        motor_counts = []
        motor_microstep_distance = []
        motor_invert_dir = []
        motor_oids = []
        motor_step_pulse_seconds = []
        motor_high_precision_step_compress = []
        for axis, kind in zip(axes, lane_kinds):
            bindings = (
                [binding for binding in bind_list if binding[0] == axis]
                if kind in (LANE_KIND_PULSE, LANE_KIND_PHASE_WITH_PULSE)
                else []
            )
            if not bindings:
                motor_counts.append(1)
                motor_microstep_distance.append(slot_microstep_distance[axis])
                motor_invert_dir.append(slot_invert_dir[axis])
                motor_oids.append(slot_oids[axis])
                motor_step_pulse_seconds.append(slot_step_pulse_seconds[axis])
                motor_high_precision_step_compress.append(False)
                continue
            motor_counts.append(len(bindings))
            steppers = dict(slot_steppers[axis])
            for _axis, stepper_name, oid, invert_dir in bindings:
                motor_microstep_distance.append(
                    steppers[stepper_name].get_step_dist()
                )
                motor_invert_dir.append(bool(invert_dir))
                motor_oids.append(oid)
                motor_step_pulse_seconds.append(
                    step_pulse_width(steppers[stepper_name])
                )
                motor_high_precision_step_compress.append(
                    bool(
                        getattr(
                            steppers[stepper_name],
                            "high_precision_step_compress",
                            False,
                        )
                    )
                )
        if (
            mcu_obj is not None
            and pulse_axes
            and handle not in endpoint_handles
        ):
            _reject_pulse_lane_conflicts(
                motion, name, mcu_obj, motor_high_precision_step_compress
            )
        topo.append(
            McuTopology(
                handle,
                axes,
                motion.kin.mcu_tag(axes),
                max_motor_velocity,
                lane_kinds,
                motor_counts,
                motor_microstep_distance,
                motor_invert_dir,
                motor_oids,
                move_queue_slots,
                motor_step_pulse_seconds,
                motor_high_precision_step_compress,
                max_error_secs,
                phase_sample_rate,
                phase_ring_depth,
            )
        )
    return topo


def init_planner(motion):
    engine_mcus = []
    for name, mcu in motion.printer.lookup_objects(module="mcu"):
        handle = mcu.get_engine_handle()
        if handle is None:
            continue
        engine_mcus.append((name, mcu, handle))
    if not engine_mcus:
        logging.warning(
            "Motion: no MCU engine handles available; skipping init_planner"
        )
        return

    axis_to_handle = motion._build_axis_to_handle()
    topology = motion._derive_mcu_topology(axis_to_handle)
    if not topology:
        logging.warning(
            "Motion: no axis->MCU assignment resolved; skipping init_planner"
        )
        return

    mcu_by_handle = {handle: mcu for _name, mcu, handle in engine_mcus}
    slots_by_name = {
        axis_name: lane_idx
        for lane_idx, axis_name, _motor_names in motion.kin.lanes()
    }
    slots_by_name.update(
        {
            axis_name: slot_idx
            for axis_name, _motors, slot_idx in motion._follower_slots()
        }
    )
    motion._axis_mcus = {}
    for axis_name, slot_idx in slots_by_name.items():
        handle = axis_to_handle.get(slot_idx)
        if handle in mcu_by_handle:
            motion._axis_mcus[axis_name] = mcu_by_handle[handle]

    try:
        motion.engine.init_planner(motion._motion_config_text, topology)
        motion._configure_axes_per_mcu(engine_mcus)
        motion._planner_ready = True
        motion._register_engine_wakeup()

    except Exception:
        logging.exception("Motion: init_planner failed")
        raise


def follower_slots(motion):
    _kind, _lanes, followers = motion.kinematics_decl
    return list(followers)


def build_slot_steppers(motion):
    slot_steppers = [[], [], [], []]
    for lane_idx, _axis_name, _motor_names in motion.kin.lanes():
        slot_steppers[lane_idx] = [
            (s.get_name(), s) for s in motion.kin.rails[lane_idx].get_steppers()
        ]
    fm = motion.printer.lookup_object("force_move", None)
    for _name, motors, slot_idx in motion._follower_slots():
        entries = []
        for motor_name in motors:
            s = None if fm is None else fm.steppers.get(motor_name)
            if s is not None:
                entries.append((motor_name, s))
        slot_steppers[slot_idx] = entries
    return slot_steppers


def _build_slot_masks(printer, mcu_obj, slot_steppers, num_engine_mcus):
    present_mask = 0
    invert_mask = 0
    steps_per_mm = [0.0, 0.0, 0.0, 0.0]
    step_modes = [STEP_MODE_STEP_TIME] * 4
    microstep_distance = [0.0, 0.0, 0.0, 0.0]
    invert_dir = [False, False, False, False]
    stepper_oids = [0, 0, 0, 0]
    step_pulse_seconds = [0.0, 0.0, 0.0, 0.0]
    dual_transport = [False, False, False, False]
    bind_list = []
    for i in range(4):
        on_this_mcu = []
        for sname, s in slot_steppers[i]:
            if num_engine_mcus > 1:
                try:
                    s_mcu = s.get_mcu()
                except AttributeError:
                    s_mcu = None
                if s_mcu is not None and s_mcu is not mcu_obj:
                    continue
            on_this_mcu.append((sname, s))
        if not on_this_mcu:
            continue
        primary_name, primary = on_this_mcu[0]
        step_dist = primary.get_step_dist()
        if step_dist <= 0.0:
            continue
        steps_per_mm[i] = 1.0 / step_dist
        microstep_distance[i] = step_dist
        stepper_oids[i] = primary.get_oid()
        step_pulse_seconds[i] = step_pulse_width(primary)
        present_mask |= 1 << i
        if getattr(primary, "_invert_dir", False):
            invert_mask |= 1 << i
            invert_dir[i] = True
        if getattr(primary, "phase_stepping", False):
            step_modes[i] = STEP_MODE_MODULATED
            dual_transport[i] = _slot_needs_pulse_windows(printer, on_this_mcu)
        for sname, s in on_this_mcu:
            inv = 1 if getattr(s, "_invert_dir", False) else 0
            bind_list.append((i, sname, s.get_oid(), inv))
    return (
        present_mask,
        invert_mask,
        steps_per_mm,
        step_modes,
        bind_list,
        microstep_distance,
        invert_dir,
        stepper_oids,
        step_pulse_seconds,
        dual_transport,
    )


def _slot_needs_pulse_windows(printer, slot):
    """A phase-stepped motor homed on StallGuard runs its trip move through
    the classic step queue, so the lane needs a pulse binding alongside its
    phase binding. Anything else keeps a phase binding only, and a mode switch
    on it is refused."""
    for sname, _s in slot:
        tmc = _lookup_phase_tmc(printer, sname)
        if tmc is not None and tmc.needs_pulse_mode_windows():
            return True
    return False


PHASE_STEPPING_DRIVER_TYPES = ("tmc5160", "tmc2240")


def _lookup_phase_tmc(printer, stepper_name):
    for driver_type in PHASE_STEPPING_DRIVER_TYPES:
        tmc = printer.lookup_object("%s %s" % (driver_type, stepper_name), None)
        if tmc is not None:
            return tmc
    return None


def _configure_phase_stepping_groups(
    motion, slot_steppers, step_modes, coupled
):
    phase_configs = []
    any_phase_stepping = False
    xy_coupled = coupled
    phase_groups = {}
    for i, slot in enumerate(slot_steppers):
        if step_modes[i] != STEP_MODE_MODULATED or not slot:
            continue
        group_key = "xy" if (xy_coupled and i in (0, 1)) else i
        slot_tmcs = phase_groups.setdefault(group_key, [])
        for stepper_name, stepper_obj in slot:
            tmc = _lookup_phase_tmc(motion.printer, stepper_name)
            if tmc is None:
                raise motion.printer.config_error(
                    "phase_stepping=True on stepper '%s' requires "
                    "a %s section (current driver type "
                    "or absence of such a section is "
                    "incompatible with phase stepping)"
                    % (
                        stepper_name,
                        " or ".join(
                            "[%s %s]" % (t, stepper_name)
                            for t in PHASE_STEPPING_DRIVER_TYPES
                        ),
                    )
                )
            if not hasattr(tmc, "get_phase_config"):
                raise motion.printer.config_error(
                    "phase_stepping=True on stepper '%s' requires "
                    "a %s driver; found driver type with no "
                    "phase-stepping support"
                    % (
                        stepper_name,
                        " or ".join(PHASE_STEPPING_DRIVER_TYPES),
                    )
                )
            bus_id, cs_pin_id = tmc.get_phase_config()
            tmc.set_phase_stepper_oid(stepper_obj.get_oid())
            slot_tmcs.append(tmc)
            phase_configs.append((bus_id, cs_pin_id, i))
            any_phase_stepping = True
    for group in phase_groups.values():
        for tmc in group:
            tmc.set_phase_group(group)
    if len(phase_configs) > FIRMWARE_MAX_PHASE_STEPPED_MOTORS:
        raise motion.printer.config_error(
            "phase_stepping enabled on %d motors but the firmware "
            "supports up to %d phase-stepped motors total per MCU."
            % (len(phase_configs), FIRMWARE_MAX_PHASE_STEPPED_MOTORS)
        )
    return phase_configs, any_phase_stepping


def _validate_firmware_capabilities(
    motion, mcu_handle, name, slot_steppers, step_modes
):
    mcu_caps = motion.engine.get_mcu_capabilities(mcu_handle)
    for i in range(4):
        if step_modes[i] == STEP_MODE_MODULATED and not (
            mcu_caps & PHASE_STEPPING_CAPABILITY_BIT
        ):
            slot_name = (
                slot_steppers[i][0][0] if slot_steppers[i] else "motor_%d" % i
            )
            raise motion.printer.config_error(
                "Stepper '%s' requests phase_stepping: 1, but MCU "
                "'%s' did not advertise the PHASE_STEPPING capability "
                "in its IdentifyResponse (caps=0x%x). This usually "
                "means kalico-native identify timed out, which in "
                "turn usually means the MCU's firmware was built "
                "without CONFIG_RUNTIME=y. Rebuild that MCU "
                "with CONFIG_RUNTIME=y (and the small or "
                "large runtime profile for the chip family) and "
                "reflash." % (slot_name, name, mcu_caps)
            )
    return mcu_caps


def _send_axis_configuration(
    motion,
    mcu_handle,
    name,
    configure_axis_cmd,
    bind_list,
    steps_per_mm,
    step_modes,
    phase_configs,
    any_phase_stepping,
):
    if any_phase_stepping:
        seen_buses = set()
        for bus_id, _cs_pin_id, _slot_idx in phase_configs:
            if bus_id == 0xFF:
                continue
            if bus_id in seen_buses:
                continue
            seen_buses.add(bus_id)
            logging.info("register_phase_bus mcu=%s bus_id=%d", name, bus_id)
            motion.engine.register_phase_bus(
                mcu_handle,
                bus_id,
                rate=2_000_000,
            )
        for motor_idx, (bus_id, cs_pin_id, slot_idx) in enumerate(
            phase_configs,
        ):
            if bus_id == 0xFF:
                continue
            logging.info(
                "register_phase_motor mcu=%s motor=%d bus=%d cs=%d slot=%d",
                name,
                motor_idx,
                bus_id,
                cs_pin_id,
                slot_idx,
            )
            motion.engine.register_phase_motor(
                mcu_handle,
                motor_idx,
                bus_id,
                cs_pin_id,
                slot_idx,
            )
    axis_bindings = defaultdict(list)
    for slot_idx, sname, oid, inv in bind_list:
        axis_bindings[slot_idx].append((sname, oid, inv))

    for axis_idx, bindings in axis_bindings.items():
        spm = steps_per_mm[axis_idx] if axis_idx < len(steps_per_mm) else 0.0
        if spm <= 0:
            continue
        microstep_distance = 1.0 / spm
        microstep_bits = struct.unpack(
            "<I", struct.pack("<f", microstep_distance)
        )[0]
        extrusion_bits = UNUSED_EXTRUSION_PER_XY_BITS
        blob = bytearray()
        for motor_idx, (sname, oid, inv) in enumerate(bindings):
            motion._motor_bindings[sname] = (
                mcu_handle,
                axis_idx,
                motor_idx,
            )
            blob.append(oid)
            blob.append(inv & 0x01)
            tmc_oid = TMC_CS_OID_NONE
            if step_modes[axis_idx] == STEP_MODE_MODULATED:
                tmc = _lookup_phase_tmc(motion.printer, sname)
                if tmc is not None:
                    tmc_oid = tmc.get_spi_oid()
            blob.append(tmc_oid)
            blob.append(FLAGS_DEFAULT)
        axis_mode = (
            MODE_PHASE
            if step_modes[axis_idx] == STEP_MODE_MODULATED
            else MODE_PULSE
        )
        configure_axis_cmd.send(
            [
                axis_idx,
                axis_mode,
                microstep_bits,
                extrusion_bits,
                len(bindings),
                bytes(blob),
            ]
        )


def _configure_stepcompress_mcu(
    motion,
    name,
    mcu_obj,
    mcu_handle,
    present_mask,
    bind_list,
    microstep_distance,
):
    if present_mask == 0:
        logging.info(
            "Motion: no step/dir lanes on MCU %s; skipping stepcompress "
            "motor binding",
            name,
        )
        return
    axis_bindings = defaultdict(list)
    for slot_idx, sname, oid, inv in bind_list:
        axis_bindings[slot_idx].append((sname, oid, inv))
    for axis_idx, bindings in axis_bindings.items():
        for motor_idx, (sname, _oid, _inv) in enumerate(bindings):
            motion._motor_bindings[sname] = (mcu_handle, axis_idx, motor_idx)
    register_stepcompress_steppers(
        motion.printer,
        mcu_obj,
        [oid for _slot, _sname, oid, _inv in bind_list],
    )
    logging.info(
        "Motion: stepcompress lanes mcu=%s present=0x%x "
        "microstep_distance=%s bindings=%s — host-side step generation",
        name,
        present_mask,
        microstep_distance,
        bind_list,
    )


def _configure_one_mcu(
    motion,
    name,
    mcu_obj,
    mcu_handle,
    slot_steppers,
    coupled,
    awd_default,
    num_engine_mcus,
):
    (
        present_mask,
        invert_mask,
        steps_per_mm,
        step_modes,
        bind_list,
        microstep_distance,
        _invert_dir,
        _stepper_oids,
        _step_pulse_seconds,
        dual_transport,
    ) = _build_slot_masks(
        motion.printer, mcu_obj, slot_steppers, num_engine_mcus
    )
    phase_binds = [
        b for b in bind_list if step_modes[b[0]] == STEP_MODE_MODULATED
    ]
    pulse_binds = [
        b
        for b in bind_list
        if step_modes[b[0]] != STEP_MODE_MODULATED or dual_transport[b[0]]
    ]
    pulse_mask = 0
    for slot_idx, _sname, _oid, _inv in pulse_binds:
        pulse_mask |= 1 << slot_idx
    _configure_stepcompress_mcu(
        motion,
        name,
        mcu_obj,
        mcu_handle,
        pulse_mask,
        pulse_binds,
        microstep_distance,
    )
    if not phase_binds:
        return
    phase_configs, any_phase_stepping = _configure_phase_stepping_groups(
        motion, slot_steppers, step_modes, coupled
    )
    awd_mask = awd_default & present_mask
    mcu_caps = _validate_firmware_capabilities(
        motion, mcu_handle, name, slot_steppers, step_modes
    )
    try:
        configure_axis_cmd = mcu_obj.lookup_command(
            "kalico_configure_axis axis_idx=%c mode=%c"
            " microstep_distance=%u extrusion_per_xy_mm=%u"
            " stepper_count=%c steppers=%*s"
        )
    except Exception:
        logging.info(
            "Motion: mcu=%s lacks kalico_configure_axis "
            "(no new stepping redesign command); skipping runtime "
            "binding",
            name,
        )
        return

    try:
        reset_cmd = mcu_obj.lookup_command("runtime_reset")
    except Exception:
        reset_cmd = None
    if reset_cmd is not None:
        reset_cmd.send([])
        logging.info(
            "Motion: sent runtime_reset to mcu=%s",
            name,
        )

    _send_axis_configuration(
        motion,
        mcu_handle,
        name,
        configure_axis_cmd,
        phase_binds,
        steps_per_mm,
        step_modes,
        phase_configs,
        any_phase_stepping,
    )
    logging.info(
        "Motion: configure_axes mcu=%s kin=%s "
        "present=0x%x awd=0x%x invert=0x%x steps_per_mm=%s "
        "step_modes=%s mcu_caps=0x%x runtime_bindings=%s "
        "phase_configs=%s any_phase_stepping=%s "
        "phase_motor_count=%d",
        name,
        motion.kin.kind,
        present_mask,
        awd_mask,
        invert_mask,
        steps_per_mm,
        step_modes,
        mcu_caps,
        phase_binds,
        phase_configs,
        any_phase_stepping,
        len(phase_configs),
    )


def configure_axes_per_mcu(motion, engine_mcus):
    coupled = motion.kin.coupled_xy()
    awd_default = 0b0011 if coupled else 0b0000

    slot_steppers = motion._build_slot_steppers()

    for name, mcu_obj, mcu_handle in engine_mcus:
        _configure_one_mcu(
            motion,
            name,
            mcu_obj,
            mcu_handle,
            slot_steppers,
            coupled,
            awd_default,
            len(engine_mcus),
        )
