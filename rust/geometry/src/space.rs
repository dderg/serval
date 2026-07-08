//! Coordinate-space newtypes for the two cartesian frames the motion stack
//! operates in. Everything upstream of the lowerer is gcode space; the
//! lowerer's output, the wire, the motion history, and every measured
//! (trigger/probe) position is machine space. The bed surface transform is
//! the only thing that makes them differ, and [`SurfaceTransform`]-aware
//! conversions below are the only sanctioned crossing. Holding positions in
//! these wrappers turns a forgotten crossing into a type error instead of a
//! probing bug. See docs/rewrite/toolpath-surface-transforms.md.

use crate::surface::SurfaceTransform;

/// A cartesian position in gcode space: what the g-code stream, the planner,
/// and every host-side odometer reason about. The bed surface warp has NOT
/// been applied.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GcodePos(pub [f64; 3]);

/// A cartesian position in machine space: the lowerer's output frame, where
/// physical events (endstop trips, probe triggers, step counters, retained
/// motion history) live. The bed surface warp HAS been applied to Z.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MachinePos(pub [f64; 3]);

impl GcodePos {
    pub fn x(&self) -> f64 {
        self.0[0]
    }
    pub fn y(&self) -> f64 {
        self.0[1]
    }
    pub fn z(&self) -> f64 {
        self.0[2]
    }

    /// Forward warp: the machine position a toolhead commanded to this gcode
    /// position physically rests at. Identity when no mesh is active.
    pub fn to_machine(self, mesh: Option<&SurfaceTransform>) -> MachinePos {
        match mesh {
            Some(t) => MachinePos([
                self.0[0],
                self.0[1],
                self.0[2] + t.correction_at(self.0[0], self.0[1], self.0[2]),
            ]),
            None => MachinePos(self.0),
        }
    }
}

impl MachinePos {
    pub fn x(&self) -> f64 {
        self.0[0]
    }
    pub fn y(&self) -> f64 {
        self.0[1]
    }
    pub fn z(&self) -> f64 {
        self.0[2]
    }

    /// Inverse warp: the gcode position whose commanded motion rests at this
    /// machine position. Identity when no mesh is active.
    pub fn to_gcode(self, mesh: Option<&SurfaceTransform>) -> GcodePos {
        match mesh {
            Some(t) => GcodePos([
                self.0[0],
                self.0[1],
                t.gcode_z(self.0[0], self.0[1], self.0[2]),
            ]),
            None => GcodePos(self.0),
        }
    }
}

#[cfg(test)]
mod tests;
