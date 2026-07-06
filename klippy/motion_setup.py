import logging
import struct
from collections import defaultdict

from . import motion_kinematics, stepper
from .arc_fit_config import arc_fit_from_config
from .extras import servo_axis

_LEGACY_STEPPER_AXES = frozenset("xyzab")
_LEGACY_SERVO_SECTIONS = ("servo_x", "servo_y", "servo_z")

STEP_MODE_MODULATED = 0
STEP_MODE_STEP_TIME = 1
PHASE_STEPPING_CAPABILITY_BIT = 0x1
FIRMWARE_MAX_PHASE_STEPPED_MOTORS = 16
MODE_PULSE = 0
MODE_PHASE = 1
TMC_CS_OID_NONE = 0xFF
FLAGS_DEFAULT = 0
UNUSED_EXTRUSION_PER_XY_BITS = 0


def _is_legacy_stepper_role_section(name):
    if not name.startswith("stepper_"):
        return False
    suffix = name[len("stepper_") :]
    if not suffix or suffix[0] not in _LEGACY_STEPPER_AXES:
        return False
    return suffix[1:] == "" or suffix[1:].isdigit()


def reject_legacy_role_sections(config):
    for sc in config.get_prefix_sections("stepper_"):
        if _is_legacy_stepper_role_section(sc.get_name()):
            raise config.error(
                "role-encoding motor sections are not supported: name the "
                "motor freely (e.g. [motor a]) and assign it in [kinematics] "
                "role lists / [axis <name>] motors:"
            )
    for name in _LEGACY_SERVO_SECTIONS:
        if config.has_section(name):
            raise config.error(
                "role-encoding servo sections are not supported: declare a "
                "[<motor>] section with 'drive: servo' and assign it in "
                "[kinematics]"
            )


def read_axes(motion, config):
    reject_legacy_role_sections(config)
    if config.has_section("firmware_retraction"):
        raise config.error(
            "[firmware_retraction] is not supported: it presupposes an "
            "extruder concept the motion system does not have"
        )
    if config.has_section("input_shaper"):
        raise config.error(
            "[input_shaper] is not supported: declare [post_processor "
            "<name>] sections and reference them from [axis] "
            "post_processors"
        )
    motion.axis_sections = []
    for sc in config.get_prefix_sections("axis "):
        name = sc.get_name().split(None, 1)[1]
        follows = [a.strip().lower() for a in sc.getlist("follows", [])]
        motors = [m.strip() for m in sc.getlist("motors", [])]
        post_processors = [p.strip() for p in sc.getlist("post_processors", [])]
        motion.axis_sections.append((name, follows, motors, post_processors))
    declared = {name for name, _, _, _ in motion.axis_sections}
    for _, axes, _, _, _ in motion.limit_sections:
        for a in axes:
            if a not in declared:
                raise config.error(
                    "[limit] references undeclared axis '%s' "
                    "(declare [axis %s])" % (a, a)
                )


def build_follower_steppers(motion, config):
    motion.follower_steppers = []
    claimed = set(motion_kinematics.read_claimed_axes(config))
    for name, _follows, motors, _pp in motion.axis_sections:
        if name in claimed or not motors:
            continue
        for motor_name in motors:
            motor_section, drive = motion_kinematics.resolve_motor_section(
                config, motor_name, "[axis %s] motors" % name
            )
            if drive != "stepper":
                raise config.error(
                    "[axis %s] motors references '%s' with drive: %s — "
                    "follower axes support stepper motors only"
                    % (name, motor_name, drive)
                )
            motion.follower_steppers.append(
                stepper.PrinterStepper(
                    motor_section,
                    name=motion_kinematics.motor_short_name(motor_section),
                )
            )


def read_post_processors(motion, config):
    motion.post_processor_sections = []
    for sc in config.get_prefix_sections("post_processor "):
        name = sc.get_name().split(None, 1)[1]
        ty = sc.get("type")
        params = [
            (opt, sc.getfloat(opt))
            for opt in sc.get_prefix_options("")
            if opt != "type"
        ]
        motion.post_processor_sections.append((name, ty, params))
    declared = {name for name, _, _ in motion.post_processor_sections}
    for axis_name, _, _, post_processors in motion.axis_sections:
        for ref in post_processors:
            if ref not in declared:
                raise config.error(
                    "[axis %s] references undeclared post_processor "
                    "'%s' (declare [post_processor %s])" % (axis_name, ref, ref)
                )


def read_arc_fit(motion, config):
    motion.arc_fit = arc_fit_from_config(config)


def read_limits(motion, config):
    for key in motion.UNSUPPORTED_LIMIT_KEYS:
        if config.get(key, None) is not None:
            raise config.error("[printer] %s is not supported" % key)
    motion._max_velocity = config.getfloat("max_velocity", above=0.0)
    motion._max_accel = config.getfloat("max_accel", above=0.0)
    motion._square_corner_velocity = config.getfloat(
        "square_corner_velocity", 5.0, minval=0.0
    )
    max_jerk = config.getfloat("max_jerk", motion._max_accel * 2.0, minval=0.0)
    motion.max_jerk = max_jerk if max_jerk > 0.0 else float("inf")
    motion.max_z_velocity = config.getfloat(
        "max_z_velocity",
        motion._max_velocity,
        above=0.0,
        maxval=motion._max_velocity,
    )
    motion.max_z_accel = config.getfloat(
        "max_z_accel",
        motion._max_accel,
        above=0.0,
        maxval=motion._max_accel,
    )
    motion.max_path_deviation = config.getfloat(
        "max_path_deviation", 0.005, above=0.0, maxval=1.0
    )
    motion.max_accel_deviation = config.getfloat(
        "max_accel_deviation", 50.0, above=0.0
    )
    motion.limit_sections = []
    for sc in config.get_prefix_sections("limit "):
        name = sc.get_name().split(None, 1)[1]
        axes = [a.strip().lower() for a in sc.getlist("axes")]
        v = sc.getfloat("max_velocity", None, above=0.0)
        a = sc.getfloat("max_accel", None, above=0.0)
        j = sc.getfloat("max_jerk", None, above=0.0)
        motion.limit_sections.append((name, axes, v, a, j))
    motion.min_cruise_ratio = 0.0
    motion.orig_cfg = {}


def declared_axis_order(motion):
    return [name for name, _, _, _ in motion.axis_sections]


def build_axis_to_handle(motion):
    axis_to_handle = {}
    for lane_idx, _axis_name, _motor_names in motion.kin.lanes():
        rail = motion.kin.rails[lane_idx]
        if isinstance(rail, servo_axis.ServoRail):
            node = motion.printer.lookup_object(
                "ethercat_node " + rail.get_node_name(), None
            )
            if node is None:
                continue
            handle = node.get_engine_handle()
        else:
            steppers = rail.get_steppers()
            if not steppers:
                continue
            handle = getattr(steppers[0].get_mcu(), "_engine_handle", None)
        if handle is None:
            continue
        axis_to_handle[lane_idx] = handle

    fm = motion.printer.lookup_object("force_move", None)
    for _name, motors, slot_idx in motion._follower_slots():
        if fm is None:
            continue
        primary = fm.steppers.get(motors[0])
        if primary is None:
            continue
        handle = getattr(primary.get_mcu(), "_engine_handle", None)
        if handle is None:
            continue
        axis_to_handle[slot_idx] = handle
    return axis_to_handle


def derive_mcu_topology(motion, axis_to_handle):
    by_handle = {}
    for axis_idx, handle in axis_to_handle.items():
        by_handle.setdefault(handle, []).append(axis_idx)
    topo = []
    for handle in sorted(by_handle):
        axes = sorted(by_handle[handle])
        topo.append((handle, axes, motion.kin.mcu_tag(axes)))
    return topo


def init_planner(motion):
    engine_mcus = []
    for name, mcu in motion.printer.lookup_objects(module="mcu"):
        handle = getattr(mcu, "_engine_handle", None)
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

    extruder = motion.printer.lookup_object("extruder", None)
    max_extrude_only_velocity = getattr(
        extruder, "max_extrude_only_velocity", None
    )
    max_extrude_only_accel = getattr(extruder, "max_extrude_only_accel", None)

    try:
        motion.engine.init_planner(
            list(motion.axis_sections),
            list(motion.limit_sections),
            list(motion.post_processor_sections),
            topology,
            motion.kin.claimed_axes(),
            (
                motion._max_velocity,
                motion._max_accel,
                motion.max_jerk,
                motion.max_z_velocity,
                motion.max_z_accel,
                motion._square_corner_velocity,
            ),
            arc_fit=motion.arc_fit,
            max_extrude_only_velocity=max_extrude_only_velocity,
            max_extrude_only_accel=max_extrude_only_accel,
            fit_tolerance_mm=motion.max_path_deviation,
            fit_tolerance_accel_mm_s2=motion.max_accel_deviation,
        )
        motion._configure_axes_per_mcu(engine_mcus)
        motion._planner_ready = True

    except Exception:
        logging.exception("Motion: init_planner failed")
        raise


def follower_slots(motion):
    claimed = set(motion.kin.claimed_axes())
    lane_slots = {
        lane_idx for lane_idx, _axis_name, _motor_names in motion.kin.lanes()
    }
    free_slots = [i for i in range(4) if i not in lane_slots]
    followers = [
        (name, motors)
        for name, _follows, motors, _pp in motion.axis_sections
        if name not in claimed and motors
    ]
    if len(followers) > len(free_slots):
        raise motion.printer.command_error(
            "%d follower axes declared but only %d motion slot(s) free of "
            "kinematics lanes" % (len(followers), len(free_slots))
        )
    return [
        (name, motors, slot)
        for (name, motors), slot in zip(followers, free_slots)
    ]


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


def _build_slot_masks(mcu_obj, slot_steppers, num_engine_mcus):
    present_mask = 0
    invert_mask = 0
    steps_per_mm = [0.0, 0.0, 0.0, 0.0]
    step_modes = [STEP_MODE_STEP_TIME] * 4
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
        present_mask |= 1 << i
        if getattr(primary, "_invert_dir", False):
            invert_mask |= 1 << i
        if getattr(primary, "phase_stepping", False):
            step_modes[i] = STEP_MODE_MODULATED
        for sname, s in on_this_mcu:
            inv = 1 if getattr(s, "_invert_dir", False) else 0
            bind_list.append((i, sname, s.get_oid(), inv))
    return present_mask, invert_mask, steps_per_mm, step_modes, bind_list


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
            tmc_name = "tmc5160 " + stepper_name
            try:
                tmc = motion.printer.lookup_object(tmc_name)
            except Exception:
                raise motion.printer.config_error(
                    "phase_stepping=True on stepper '%s' requires "
                    "a [tmc5160 %s] section (current driver type "
                    "or absence of TMC5160 section is "
                    "incompatible with phase stepping)"
                    % (stepper_name, stepper_name)
                )
            if not hasattr(tmc, "get_phase_config"):
                raise motion.printer.config_error(
                    "phase_stepping=True on stepper '%s' requires "
                    "a TMC5160 driver; found driver type with no "
                    "phase-stepping support" % stepper_name
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
                tmc_name = "tmc5160 " + sname
                try:
                    tmc = motion.printer.lookup_object(tmc_name)
                    tmc_oid = tmc.get_spi_oid()
                except Exception:
                    pass
            blob.append(tmc_oid)
            blob.append(FLAGS_DEFAULT)
        ring_depth = motion.engine.ring_depth_for_axis(mcu_handle, axis_idx)
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
                ring_depth,
                bytes(blob),
            ]
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
    present_mask, invert_mask, steps_per_mm, step_modes, bind_list = (
        _build_slot_masks(mcu_obj, slot_steppers, num_engine_mcus)
    )
    phase_configs, any_phase_stepping = _configure_phase_stepping_groups(
        motion, slot_steppers, step_modes, coupled
    )
    awd_mask = awd_default & present_mask
    if present_mask == 0:
        logging.info(
            "Motion: no steppers matched MCU %s; skipping configure_axes",
            name,
        )
        return
    mcu_caps = _validate_firmware_capabilities(
        motion, mcu_handle, name, slot_steppers, step_modes
    )
    try:
        configure_axis_cmd = mcu_obj.lookup_command(
            "kalico_configure_axis axis_idx=%c mode=%c"
            " microstep_distance=%u extrusion_per_xy_mm=%u"
            " stepper_count=%c ring_depth=%hu steppers=%*s"
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
        bind_list,
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
        [(m, n, o, i) for (m, n, o, i) in bind_list],
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
