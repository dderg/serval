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
mod tests {
    use super::*;
    use crate::dispatch::KINEMATICS_COREXY;

    #[test]
    fn cartesian_identity_passthrough() {
        let mut m = [None; 8];
        let mut v = [None; 8];
        m[0] = Some(10.0);
        m[1] = Some(20.0);
        m[2] = Some(5.0);
        m[3] = Some(2.0);
        v[0] = Some(1.0);
        v[1] = Some(-1.0);
        v[2] = Some(0.0);
        v[3] = Some(3.0);
        // tag 1 = cartesian
        let out = assemble_cartesian(&m, &v, 1).unwrap();
        assert_eq!(out["x"], (10.0, 1.0));
        assert_eq!(out["y"], (20.0, -1.0));
        assert_eq!(out["z"], (5.0, 0.0));
        assert_eq!(out["e"], (2.0, 3.0));
    }

    #[test]
    fn corexy_inverse_mix() {
        // motor A = x + y, motor B = x - y. For x=10, y=4: A=14, B=6.
        let mut m = [None; 8];
        let v = [None; 8];
        m[0] = Some(14.0);
        m[1] = Some(6.0);
        m[2] = Some(0.0);
        let out = assemble_cartesian(&m, &v, KINEMATICS_COREXY).unwrap();
        assert!((out["x"].0 - 10.0).abs() < 1e-9);
        assert!((out["y"].0 - 4.0).abs() < 1e-9);
    }
}
