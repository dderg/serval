//! Commands the snapshot cannot plan. Each is refused by name with the
//! reason; every other command the snapshot does not mirror is one the live
//! host ignores too (temperatures, fans, display, progress, object markers)
//! or does not know, which it drops with a warning.

const ARC: &str = "the live host drops arcs as an unknown command ([gcode_arcs] refuses to load), and straight segments planned in their place would misrepresent the print";
const CURVE: &str = "a curve move; the snapshot plans straight G0/G1 segments only";
const RETRACTION: &str = "firmware retraction; the live host drops it as an unknown command ([firmware_retraction] refuses to load), and the snapshot cannot know the move it stands for";
const UNKNOWN_MOTION: &str =
    "moves the toolhead by a homing, probing or calibration sequence the snapshot cannot know";
const MOTOR_DRIVE: &str = "drives motors outside the planned toolhead path";
const PLANNER_SETUP: &str =
    "changes post-processing, extruder or bed geometry the snapshot takes from its config file";
const PRINT_FLOW: &str =
    "pauses, resumes, cancels or skips print moves with motion the snapshot cannot know";
const HEATER_WAIT: &str = "blocks the host until a temperature is reached; whether the planner drains to rest meanwhile depends on heating the snapshot cannot know";

pub(super) fn numbered(
    letter: u8,
    major: u32,
    minor: Option<u32>,
    anchored: bool,
) -> Option<&'static str> {
    match (letter, major, minor) {
        (b'G', 2 | 3, _) => Some(ARC),
        (b'G', 5, _) => Some(CURVE),
        (b'G', 10 | 11, _) => Some(RETRACTION),
        (b'G', 28, _) => Some(UNKNOWN_MOTION),
        (b'M', 109 | 190, _) if anchored => Some(HEATER_WAIT),
        _ => None,
    }
}

pub(super) fn extended(name: &str, anchored: bool) -> Option<&'static str> {
    match name {
        "PROBE"
        | "PROBE_ACCURACY"
        | "PROBE_CALIBRATE"
        | "MANUAL_PROBE"
        | "ACCEPT"
        | "ABORT"
        | "ADJUSTED"
        | "NEXT"
        | "TESTZ"
        | "QUAD_GANTRY_LEVEL"
        | "Z_TILT_ADJUST"
        | "Z_TILT_AUTODETECT"
        | "Z_TILT_CALIBRATE"
        | "BED_MESH_CALIBRATE"
        | "BED_TILT_CALIBRATE"
        | "BED_SCREWS_ADJUST"
        | "SCREWS_TILT_CALCULATE"
        | "DELTA_CALIBRATE"
        | "DELTA_ANALYZE"
        | "Z_ENDSTOP_CALIBRATE"
        | "AXIS_TWIST_COMPENSATION_CALIBRATE"
        | "ATTACH_PROBE"
        | "DETACH_PROBE"
        | "MOVE_AVOIDING_DOCK"
        | "MOVE_TO_APPROACH_PROBE"
        | "MOVE_TO_DETACH_PROBE"
        | "MOVE_TO_DOCK_PROBE"
        | "MOVE_TO_EXTRACT_PROBE"
        | "MOVE_TO_INSERT_PROBE"
        | "NOZZLE_CLEANUP"
        | "TOOL_LOCATE_SENSOR"
        | "TOOL_CALIBRATE_TOOL_OFFSET" => Some(UNKNOWN_MOTION),
        "FORCE_MOVE"
        | "MANUAL_STEPPER"
        | "SET_STEPPER_ENABLE"
        | "STEPPER_BUZZ"
        | "DIAG_STEPPER_BUZZ"
        | "RESONANCE_BUZZ"
        | "RESONANCE_BUZZ_SWEEP"
        | "TEST_RESONANCES"
        | "SHAPER_CALIBRATE" => Some(MOTOR_DRIVE),
        "SET_PRESSURE_ADVANCE"
        | "SET_SMOOTH_TIME"
        | "SET_POST_PROCESSOR"
        | "SET_INPUT_SHAPER"
        | "BED_MESH_PROFILE"
        | "BED_MESH_CLEAR"
        | "BED_MESH_OFFSET"
        | "SET_SKEW"
        | "SKEW_PROFILE"
        | "SET_Z_THERMAL_ADJUST"
        | "ACTIVATE_EXTRUDER"
        | "SYNC_EXTRUDER_MOTION"
        | "SET_EXTRUDER_ROTATION_DISTANCE"
        | "SET_MIXING_EXTRUDER"
        | "SET_MIXING_EXTRUDER_GRADIENT"
        | "TUNING_TOWER" => Some(PLANNER_SETUP),
        "PAUSE" | "RESUME" | "CANCEL_PRINT" | "EXCLUDE_OBJECT" => Some(PRINT_FLOW),
        "TEMPERATURE_WAIT" if anchored => Some(HEATER_WAIT),
        _ => None,
    }
}
