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

pub const ERR_COMP_BAD_SLOT: i32 = -856;
pub const ERR_COMP_BAD_GRID: i32 = -857;
pub const ERR_COMP_BAD_LANE: i32 = -858;
pub const ERR_COMP_SLOT_IN_USE: i32 = -859;
pub const ERR_COMP_BAD_KINEMATICS: i32 = -860;

pub const MAX_COMP_GRID_VALUES: usize = 4096;
pub const MAX_COMP_GRID_DIM: usize = 64;
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

    #[allow(clippy::too_many_arguments)]
    pub fn set(
        &mut self,
        num_slaves: usize,
        slot_a: u8,
        slot_b: u8,
        lane_a: u8,
        lane_b: u8,
        kinematics: u8,
        nx: u8,
        ny: u8,
        x0: f64,
        y0: f64,
        dx: f64,
        dy: f64,
        values_um: &[i16],
    ) -> i32 {
        let (a, b) = (usize::from(slot_a), usize::from(slot_b));
        if a == b || a >= num_slaves || b >= num_slaves {
            return ERR_COMP_BAD_SLOT;
        }
        let same_pair =
            |c: &PairComp| (c.slot_a, c.slot_b) == (a, b) || (c.slot_a, c.slot_b) == (b, a);
        let (nx, ny) = (usize::from(nx), usize::from(ny));
        if nx == 0 || ny == 0 {
            self.comps.retain(|c| !same_pair(c));
            return 0;
        }
        if kinematics != KIN_COREXY && kinematics != KIN_CARTESIAN {
            return ERR_COMP_BAD_KINEMATICS;
        }
        if usize::from(lane_a) >= num_slaves || usize::from(lane_b) >= num_slaves {
            return ERR_COMP_BAD_LANE;
        }
        if nx > MAX_COMP_GRID_DIM
            || ny > MAX_COMP_GRID_DIM
            || nx * ny > MAX_COMP_GRID_VALUES
            || values_um.len() != nx * ny
            || !(dx > 0.0 && dy > 0.0 && x0.is_finite() && y0.is_finite())
            || values_um.iter().any(|v| v.abs() > MAX_COMP_OFFSET_UM)
        {
            return ERR_COMP_BAD_GRID;
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
        let applied = self
            .comps
            .iter()
            .find(|c| same_pair(c))
            .map_or(0.0, |c| c.applied_mm);
        self.comps.retain(|c| !same_pair(c));
        self.comps.push(PairComp {
            slot_a: a,
            slot_b: b,
            lane_a: usize::from(lane_a),
            lane_b: usize::from(lane_b),
            kinematics,
            nx,
            ny,
            x0,
            y0,
            dx,
            dy,
            values_mm: values_um.iter().map(|&v| f64::from(v) * 1e-3).collect(),
            target_mm: applied,
            applied_mm: applied,
        });
        0
    }

    pub fn active(&self) -> bool {
        !self.comps.is_empty()
    }

    /// Torque was dropped: the rotors relaxed to neutral, so the physically
    /// applied offset is gone. Forget it and re-slew from zero on re-enable
    /// instead of stepping the freshly seeded targets by the stale amount.
    pub fn reset_applied(&mut self) {
        for comp in &mut self.comps {
            comp.applied_mm = 0.0;
            comp.target_mm = 0.0;
        }
    }

    pub fn snapshot(&self) -> Vec<(usize, usize, f64, f64)> {
        self.comps
            .iter()
            .map(|c| (c.slot_a, c.slot_b, c.applied_mm, c.target_mm))
            .collect()
    }

    /// One cycle: refresh each pair's target from the streamed lane
    /// positions (held when a lane is not streaming), slew the applied
    /// offset toward it, and accumulate the antisymmetric per-slot offsets.
    pub fn update(&mut self, lane_mm: &[Option<f64>], slave_axes: &[u8], offsets_mm: &mut [f64]) {
        for comp in &mut self.comps {
            let lane_pos = |lane: usize| {
                slave_axes
                    .iter()
                    .zip(lane_mm.iter())
                    .find(|&(&axis, mm)| usize::from(axis) == lane && mm.is_some())
                    .and_then(|(_, mm)| *mm)
            };
            if comp.nx == 1 && comp.ny == 1 {
                // A constant grid needs no position — this is the stiffness
                // probe's path, which runs entirely at standstill.
                comp.target_mm = comp.values_mm[0];
            } else if let (Some(pa), Some(pb)) = (lane_pos(comp.lane_a), lane_pos(comp.lane_b)) {
                let (x, y) = comp.carriage_xy(pa, pb);
                comp.target_mm = comp.sample(x, y);
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
