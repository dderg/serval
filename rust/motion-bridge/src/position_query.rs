use crate::kinematics::KinematicsModule;
use std::collections::HashMap;

const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];

/// `motors[slot]` / `vmotors[slot]` are motor-space mm / mm-s, `None` if that slot
/// was not reported. `kin_tag` is the kinematics tag of the MCU owning the spatial
/// axes. Returns cartesian per-axis (pos, vel); axes with no data are omitted.
pub fn assemble_cartesian(
    motors: &[Option<f64>; 8],
    vmotors: &[Option<f64>; 8],
    kin_tag: u8,
) -> Result<HashMap<String, (f64, f64)>, String> {
    let kin = KinematicsModule::from_tag(kin_tag).map_err(|e| e.to_string())?;
    let spat = |arr: &[Option<f64>; 8]| {
        [
            arr[0].unwrap_or(0.0),
            arr[1].unwrap_or(0.0),
            arr[2].unwrap_or(0.0),
        ]
    };
    let pos_cart = kin.inverse(spat(motors));
    let vel_cart = kin.inverse(spat(vmotors));
    let mut out = HashMap::new();
    for axis in 0..3 {
        if motors[axis].is_some() || vmotors[axis].is_some() {
            out.insert(
                AXIS_NAMES[axis].to_string(),
                (pos_cart[axis], vel_cart[axis]),
            );
        }
    }
    if motors[3].is_some() || vmotors[3].is_some() {
        out.insert(
            "e".to_string(),
            (motors[3].unwrap_or(0.0), vmotors[3].unwrap_or(0.0)),
        );
    }
    Ok(out)
}

#[cfg(test)]
mod tests;
