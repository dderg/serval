/// Discriminants are the kinematics tag crossing the Python↔Rust
/// `init_planner` topology tuples; klippy mirrors them numerically.
/// Renumbering breaks that contract — dispatch.rs pins them with a
/// compile-time assert.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicTag {
    CoreXy = 0,
    Cartesian = 1,
}
