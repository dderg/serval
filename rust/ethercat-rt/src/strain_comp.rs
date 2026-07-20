//! Belt strain compensation: a per-pair 2D lookup table of antisymmetric
//! position offsets, keyed on the commanded carriage position. The strain
//! map measures how much elastic differential torque each belt pair carries
//! at every (x, y) — belt thickness lumps, pitch nonuniformity, frame
//! geometry — and the host converts that field into offsets through the
//! measured pair stiffness. Every cycle the bank reconstructs (x, y) from
//! the streamed lane positions, interpolates each pair's grid, and feeds the
//! pair an equal-and-opposite offset: the rotors absorb the tension
//! variation instead of fighting through the belt, and the carriage never
//! moves.
//!
//! The offset rides on top of the streamed targets exactly like the trim's:
//! deliberately NOT part of the command anchor, so a sync or re-anchor never
//! bakes a live offset in. A 1x1 grid is a constant offset — that is how the
//! pair stiffness itself is measured (step a known offset, read the torque
//! response).
//!
//! The map's DC follows the mechanics: whenever torque drops (SERVO_SYNC,
//! M84, idle timeout), the free rotors relax the pair's differential strain
//! at wherever the carriage sits — including a hand-move while unpowered —
//! so the position where torque returns becomes the new physical zero. The
//! bank re-anchors there: it samples the grid at the re-engage position and
//! applies the map relative to that value, never re-racking a freshly
//! relaxed gantry. The cost is accuracy: the map's fractional error is now
//! measured from the re-engage position instead of the calibrated zero, so
//! re-anchoring at a field extreme can double the worst-case residual. A
//! SERVO_SYNC at the map's zero point restores the calibrated anchor.

use mcu_protocol::messages::SetStrainComp;

pub const ERR_COMP_BAD_SLOT: i32 = -856;
pub const ERR_COMP_BAD_GRID: i32 = -857;
pub const ERR_COMP_BAD_LANE: i32 = -858;
pub const ERR_COMP_SLOT_IN_USE: i32 = -859;
pub const ERR_COMP_BAD_KINEMATICS: i32 = -860;

/// The wire carries `u16` dims and a `u32` value count; cycle-time cost is
/// O(1) bilinear sampling regardless of size, so the binding constraint is
/// endpoint memory and upload latency. 2^20 values = 8 MB of f64 per pair
/// (a 0.5 mm grid over a 500 mm bed) - far past any physical field detail
/// while still refusing a nonsense upload.
pub const MAX_COMP_GRID_DIM: usize = u16::MAX as usize;
pub const MAX_COMP_GRID_VALUES: usize = 1 << 20;
pub const MAX_COMP_OFFSET_UM: i16 = 500;
/// Hard cap on how fast an applied offset may move, so enabling a map (or a
/// bad grid cell) can never yank the targets.
const MAX_COMP_SLEW_MM_S: f64 = 1.0;

pub const KIN_COREXY: u8 = 0;
pub const KIN_CARTESIAN: u8 = 1;

struct PairComp {
    slot_a: usize,
    slot_b: usize,
    lane_a: usize,
    lane_b: usize,
    kinematics: u8,
    nx: usize,
    ny: usize,
    x0: f64,
    y0: f64,
    dx: f64,
    dy: f64,
    values_mm: Vec<f64>,
    target_mm: f64,
    applied_mm: f64,
    anchor_xy: Option<(f64, f64)>,
    needs_rebias: bool,
    clearing: bool,
}

impl PairComp {
    /// Bilinear interpolation, clamped to the grid edges: outside the mapped
    /// region the field holds its border value instead of extrapolating.
    fn sample(&self, x: f64, y: f64) -> f64 {
        let fx = ((x - self.x0) / self.dx).clamp(0.0, (self.nx - 1) as f64);
        let fy = ((y - self.y0) / self.dy).clamp(0.0, (self.ny - 1) as f64);
        let ix = (fx.floor() as usize).min(self.nx.saturating_sub(2));
        let iy = (fy.floor() as usize).min(self.ny.saturating_sub(2));
        let (tx, ty) = if self.nx == 1 || self.ny == 1 {
            (
                if self.nx == 1 { 0.0 } else { fx - ix as f64 },
                if self.ny == 1 { 0.0 } else { fy - iy as f64 },
            )
        } else {
            (fx - ix as f64, fy - iy as f64)
        };
        let at = |gx: usize, gy: usize| {
            self.values_mm[gy.min(self.ny - 1) * self.nx + gx.min(self.nx - 1)]
        };
        let v00 = at(ix, iy);
        let v10 = at(ix + 1, iy);
        let v01 = at(ix, iy + 1);
        let v11 = at(ix + 1, iy + 1);
        (v00 * (1.0 - tx) + v10 * tx) * (1.0 - ty) + (v01 * (1.0 - tx) + v11 * tx) * ty
    }

    fn carriage_xy(&self, pa: f64, pb: f64) -> (f64, f64) {
        match self.kinematics {
            KIN_COREXY => ((pa + pb) * 0.5, (pa - pb) * 0.5),
            _ => (pa, pb),
        }
    }

    fn bias_mm(&self) -> f64 {
        self.anchor_xy.map_or(0.0, |(ax, ay)| self.sample(ax, ay))
    }
}

/// With re-anchoring, the applied offset can reach the grid's full span
/// (sample minus anchor sample), so the span — not just each value — must
/// stay inside the offset budget.
fn grid_span_um(values_um: &[i32]) -> i32 {
    let lo = values_um.iter().copied().min().unwrap_or(0);
    let hi = values_um.iter().copied().max().unwrap_or(0);
    hi - lo
}

/// Everything O(nx*ny) about a SetStrainComp — grid validation and the
/// um -> mm conversion — done on the socket reader thread at decode time.
/// A 113x109 map measured ~500 us when this ran in the RT dispatch (two
/// whole DC cycles, a latched frame-timing fault); `install` is left with
/// O(1) state checks and a Vec move.
#[derive(Debug)]
pub struct PreparedStrainComp {
    pub slot_a: u8,
    pub slot_b: u8,
    pub lane_a: u8,
    pub lane_b: u8,
    pub kinematics: u8,
    pub nx: u16,
    pub ny: u16,
    pub x0: f64,
    pub y0: f64,
    pub dx: f64,
    pub dy: f64,
    pub wire_values: usize,
    pub grid_rc: i32,
    pub values_mm: Vec<f64>,
}

impl PreparedStrainComp {
    pub fn prepare(msg: &SetStrainComp) -> Self {
        Self::from_values(
            msg.slot_a,
            msg.slot_b,
            msg.lane_a,
            msg.lane_b,
            msg.kinematics,
            msg.nx,
            msg.ny,
            f64::from(msg.x0),
            f64::from(msg.y0),
            f64::from(msg.dx),
            f64::from(msg.dy),
            &msg.values_um,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_values(
        slot_a: u8,
        slot_b: u8,
        lane_a: u8,
        lane_b: u8,
        kinematics: u8,
        nx: u16,
        ny: u16,
        x0: f64,
        y0: f64,
        dx: f64,
        dy: f64,
        values_um: &[i32],
    ) -> Self {
        let (nxu, nyu) = (usize::from(nx), usize::from(ny));
        let clearing = nxu == 0 || nyu == 0;
        let grid_ok = !clearing
            && nxu <= MAX_COMP_GRID_DIM
            && nyu <= MAX_COMP_GRID_DIM
            && nxu * nyu <= MAX_COMP_GRID_VALUES
            && values_um.len() == nxu * nyu
            && dx > 0.0
            && dy > 0.0
            && x0.is_finite()
            && y0.is_finite()
            && values_um
                .iter()
                .all(|v| v.abs() <= i32::from(MAX_COMP_OFFSET_UM))
            && grid_span_um(values_um) <= i32::from(MAX_COMP_OFFSET_UM);
        let grid_rc = if clearing || grid_ok {
            0
        } else {
            ERR_COMP_BAD_GRID
        };
        let values_mm = if grid_ok {
            values_um.iter().map(|&v| f64::from(v) * 1e-3).collect()
        } else {
            Vec::new()
        };
        Self {
            slot_a,
            slot_b,
            lane_a,
            lane_b,
            kinematics,
            nx,
            ny,
            x0,
            y0,
            dx,
            dy,
            wire_values: values_um.len(),
            grid_rc,
            values_mm,
        }
    }
}

pub struct StrainCompBank {
    slew_per_cycle_mm: f64,
    comps: Vec<PairComp>,
}

impl StrainCompBank {
    pub fn new(cycle_ns: i64) -> Self {
        assert!(cycle_ns > 0, "strain comp bank needs a positive cycle time");
        Self {
            slew_per_cycle_mm: MAX_COMP_SLEW_MM_S * cycle_ns as f64 * 1e-9,
            comps: Vec::new(),
        }
    }

    #[cfg(test)]
    #[allow(clippy::too_many_arguments)]
    pub fn set(
        &mut self,
        num_slaves: usize,
        slot_a: u8,
        slot_b: u8,
        lane_a: u8,
        lane_b: u8,
        kinematics: u8,
        nx: u16,
        ny: u16,
        x0: f64,
        y0: f64,
        dx: f64,
        dy: f64,
        values_um: &[i16],
    ) -> i32 {
        let values: Vec<i32> = values_um.iter().map(|&v| i32::from(v)).collect();
        self.install(
            num_slaves,
            PreparedStrainComp::from_values(
                slot_a, slot_b, lane_a, lane_b, kinematics, nx, ny, x0, y0, dx, dy, &values,
            ),
        )
    }

    /// RT-thread half of a SetStrainComp: state-dependent checks (slot and
    /// lane bounds, pair ownership, applied-offset carryover) and a Vec
    /// move. All O(nx*ny) work already happened in `PreparedStrainComp`.
    pub fn install(&mut self, num_slaves: usize, prepared: PreparedStrainComp) -> i32 {
        let (a, b) = (usize::from(prepared.slot_a), usize::from(prepared.slot_b));
        if a == b || a >= num_slaves || b >= num_slaves {
            return ERR_COMP_BAD_SLOT;
        }
        let same_pair =
            |c: &PairComp| (c.slot_a, c.slot_b) == (a, b) || (c.slot_a, c.slot_b) == (b, a);
        let (nx, ny) = (usize::from(prepared.nx), usize::from(prepared.ny));
        if nx == 0 || ny == 0 {
            // Clearing must not drop the applied offset on the floor — the
            // last written targets would keep it baked in as a standing
            // fight. Ramp it out through the slew limiter; update() removes
            // the pair once the applied offset reaches zero.
            for c in self.comps.iter_mut().filter(|c| same_pair(c)) {
                c.clearing = true;
            }
            return 0;
        }
        if prepared.kinematics != KIN_COREXY && prepared.kinematics != KIN_CARTESIAN {
            return ERR_COMP_BAD_KINEMATICS;
        }
        if usize::from(prepared.lane_a) >= num_slaves || usize::from(prepared.lane_b) >= num_slaves
        {
            return ERR_COMP_BAD_LANE;
        }
        if prepared.grid_rc != 0 {
            return prepared.grid_rc;
        }
        if self
            .comps
            .iter()
            .any(|c| !same_pair(c) && [c.slot_a, c.slot_b].iter().any(|&s| s == a || s == b))
        {
            return ERR_COMP_SLOT_IN_USE;
        }
        // A replaced map keeps ramping from the currently applied offset —
        // the slew limiter owns every transition, including enable and clear.
        // The anchor position survives too: the physical zero is wherever
        // torque last returned, and the new grid gets sampled there.
        let (applied, anchor_xy) = self
            .comps
            .iter()
            .find(|c| same_pair(c))
            .map_or((0.0, None), |c| (c.applied_mm, c.anchor_xy));
        self.comps.retain(|c| !same_pair(c));
        self.comps.push(PairComp {
            slot_a: a,
            slot_b: b,
            lane_a: usize::from(prepared.lane_a),
            lane_b: usize::from(prepared.lane_b),
            kinematics: prepared.kinematics,
            nx,
            ny,
            x0: prepared.x0,
            y0: prepared.y0,
            dx: prepared.dx,
            dy: prepared.dy,
            values_mm: prepared.values_mm,
            target_mm: applied,
            applied_mm: applied,
            anchor_xy,
            needs_rebias: false,
            clearing: false,
        });
        0
    }

    pub fn active(&self) -> bool {
        !self.comps.is_empty()
    }

    /// Torque was dropped: the rotors relaxed to neutral, so the physically
    /// applied offset is gone. Forget it and re-slew from zero on re-enable
    /// instead of stepping the freshly seeded targets by the stale amount.
    /// The relax also moved the pair's physical zero to wherever the
    /// carriage sits when torque returns, so positional grids re-anchor
    /// there; constant grids are the stiffness probe's deliberate offset
    /// and keep their value.
    pub fn reset_applied(&mut self) {
        for comp in &mut self.comps {
            comp.applied_mm = 0.0;
            comp.target_mm = 0.0;
            comp.needs_rebias = comp.nx * comp.ny > 1;
        }
    }

    pub fn snapshot(&self) -> Vec<(usize, usize, f64, f64, f64)> {
        self.comps
            .iter()
            .map(|c| (c.slot_a, c.slot_b, c.applied_mm, c.target_mm, c.bias_mm()))
            .collect()
    }

    /// One cycle: refresh each pair's target from the streamed lane
    /// positions (held when a lane is not streaming), slew the applied
    /// offset toward it, and accumulate the antisymmetric per-slot offsets.
    pub fn update(&mut self, lane_mm: &[Option<f64>], slave_axes: &[u8], offsets_mm: &mut [f64]) {
        // A cleared pair leaves only after a full update at zero, so the
        // final zero-offset write reaches the drive before active() drops.
        self.comps.retain(|c| !(c.clearing && c.applied_mm == 0.0));
        for comp in &mut self.comps {
            let lane_pos = |lane: usize| {
                slave_axes
                    .iter()
                    .zip(lane_mm.iter())
                    .find(|&(&axis, mm)| usize::from(axis) == lane && mm.is_some())
                    .and_then(|(_, mm)| *mm)
            };
            if comp.clearing {
                comp.target_mm = 0.0;
            } else if comp.nx == 1 && comp.ny == 1 {
                // A constant grid needs no position — this is the stiffness
                // probe's path, which runs entirely at standstill.
                comp.target_mm = comp.values_mm[0];
            } else if let (Some(pa), Some(pb)) = (lane_pos(comp.lane_a), lane_pos(comp.lane_b)) {
                let (x, y) = comp.carriage_xy(pa, pb);
                if comp.needs_rebias {
                    comp.anchor_xy = Some((x, y));
                    comp.needs_rebias = false;
                }
                comp.target_mm = comp.sample(x, y) - comp.bias_mm();
            }
            let step = (comp.target_mm - comp.applied_mm)
                .clamp(-self.slew_per_cycle_mm, self.slew_per_cycle_mm);
            comp.applied_mm += step;
            offsets_mm[comp.slot_a] += comp.applied_mm;
            offsets_mm[comp.slot_b] -= comp.applied_mm;
        }
    }
}

#[cfg(test)]
mod tests;
