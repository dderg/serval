use std::sync::Arc;

use geometry::path::lowering::PositionProfile;
use geometry::{Move, SurfaceTransform};
use trajectory::{AnalyticMoveSpan, ContinuousAxis, ContinuousSegment, RestSupport, SurfaceMode};

use crate::types::{BaseItem, Control, PlannedItem, PlannedMove, advance_odometer};

const REST_EPS_MM_S: f64 = 1e-9;
const WARP_BBOX_SAMPLES: usize = 8;
pub struct Lowerer {
    rest: RestSupport,
    odometer: Vec<f64>,
    t: f64,
    rest_hold_pending: bool,
    has_motion_history: bool,
    mesh: Option<Arc<SurfaceTransform>>,
}

impl Lowerer {
    pub fn new(rest: RestSupport, home_pos: Vec<f64>, t_start: f64) -> Self {
        Self {
            rest,
            odometer: home_pos,
            t: t_start,
            rest_hold_pending: true,
            has_motion_history: false,
            mesh: None,
        }
    }

    pub fn feed(&mut self, item: PlannedItem, output: &mut impl FnMut(BaseItem) -> bool) -> bool {
        let planned = match item {
            PlannedItem::Move(planned) => planned,
            PlannedItem::Drain => {
                if !self.emit_settle_hold(output) {
                    return false;
                }
                self.rest_hold_pending = true;
                return output(BaseItem::Drain);
            }
            PlannedItem::Control(control) => {
                match &control {
                    Control::Dwell { secs } => {
                        assert!(*secs >= 0.0, "lowerer: negative dwell {secs}");
                        self.t += secs;
                    }
                    Control::Reset { pos } => {
                        self.odometer.clone_from(pos);
                        self.t = 0.0;
                        self.rest_hold_pending = true;
                        self.has_motion_history = false;
                    }
                    Control::SetAxisChains(chains) => {
                        let next = chains.rest_support();
                        let settle = self.rest.after_motion.max(next.after_motion);
                        if self.has_motion_history
                            && settle > 0.0
                            && !self.emit_hold(self.t + settle, 0, output)
                        {
                            return false;
                        }
                        self.rest = next;
                    }
                    Control::SetMesh {
                        mesh,
                        gcode_z_rebase,
                    } => {
                        self.mesh = mesh.clone();
                        if let Some(z) = self.odometer.get_mut(2) {
                            *z = *gcode_z_rebase;
                        }
                    }
                    Control::Dispatch(_) => {}
                }
                return output(BaseItem::Control(control));
            }
        };
        self.emit_move(planned, output)
    }

    fn emit_move(
        &mut self,
        planned: PlannedMove,
        output: &mut impl FnMut(BaseItem) -> bool,
    ) -> bool {
        let hold_pad = if self.rest_hold_pending {
            self.rest.before_motion
        } else {
            0.0
        };
        self.rest_hold_pending = false;
        if hold_pad > 0.0
            && !self.emit_hold(
                self.t + hold_pad,
                planned.geometry.source.start_line,
                output,
            )
        {
            return false;
        }

        let source_distance_origin = planned
            .velocity
            .phases
            .first()
            .expect("lowerer: planned move must contain analytic phases")
            .s0;
        let duration: f64 = planned.velocity.phases.iter().map(|phase| phase.dt).sum();
        let t_start = self.t;
        let t_end = t_start + duration;
        let axis_count = planned
            .geometry
            .segment
            .followers
            .iter()
            .map(|follower| follower.axis_index + 1)
            .max()
            .unwrap_or(3)
            .max(self.odometer.len())
            .max(3);
        let mut starts = self.odometer.clone();
        starts.resize(axis_count, 0.0);
        let surface = classify_surface(self.mesh.as_ref(), &planned.geometry, &starts);
        let source_line = planned.geometry.source.start_line;
        let span = Arc::new(
            AnalyticMoveSpan::try_new(
                planned.geometry,
                Arc::from(planned.velocity.phases),
                source_distance_origin,
                t_start,
                t_end,
                Arc::from(starts),
                surface,
            )
            .unwrap_or_else(|error| panic!("lowerer: line {source_line}: {error}")),
        );
        let axes = (0..axis_count)
            .map(|axis| {
                if axis < 3 && span.source.segment.spatial.is_none() {
                    let surface_offset = if axis == 2 {
                        match &span.surface {
                            SurfaceMode::Constant(offset) => *offset,
                            SurfaceMode::None => 0.0,
                            SurfaceMode::Variable(_) => unreachable!(),
                        }
                    } else {
                        0.0
                    };
                    ContinuousAxis::Hold {
                        position: span.axis_start_positions[axis] + surface_offset,
                        t_start,
                        t_end,
                    }
                } else {
                    ContinuousAxis::Analytic {
                        span: Arc::clone(&span),
                        axis,
                    }
                }
            })
            .collect::<Vec<_>>();
        let segment = ContinuousSegment {
            axes: Arc::from(axes),
            followers: Arc::from(span.source.segment.followers.clone()),
            spatial_path: span.source.segment.spatial.is_some(),
            t_start,
            t_end,
            motor_mask: 0,
            source_line,
            rest_at_end: planned.velocity.exit_v <= REST_EPS_MM_S,
        };
        self.t = t_end;
        self.has_motion_history = true;
        advance_odometer(&mut self.odometer, &span.source);
        output(BaseItem::Seg(segment))
    }

    fn emit_settle_hold(&mut self, output: &mut impl FnMut(BaseItem) -> bool) -> bool {
        let settle = self.rest.after_motion;
        if self.rest_hold_pending || !self.has_motion_history || settle <= 0.0 {
            return true;
        }
        self.emit_hold(self.t + settle, 0, output)
    }

    fn emit_hold(
        &mut self,
        t_end: f64,
        source_line: u32,
        output: &mut impl FnMut(BaseItem) -> bool,
    ) -> bool {
        let t_start = self.t;
        let z_warp = rest_z_warp(self.mesh.as_deref(), &self.odometer);
        let axes = (0..self.odometer.len())
            .map(|axis| ContinuousAxis::Hold {
                position: self.odometer[axis] + if axis == 2 { z_warp } else { 0.0 },
                t_start,
                t_end,
            })
            .collect::<Vec<_>>();
        let segment = ContinuousSegment {
            axes: Arc::from(axes),
            followers: Arc::from([]),
            spatial_path: false,
            t_start,
            t_end,
            motor_mask: 0,
            source_line,
            rest_at_end: true,
        };
        if !output(BaseItem::Seg(segment)) {
            return false;
        }
        self.t = t_end;
        true
    }
}

fn classify_surface(
    mesh: Option<&Arc<SurfaceTransform>>,
    movement: &Move,
    start_pos: &[f64],
) -> SurfaceMode {
    let Some(surface) = mesh else {
        return SurfaceMode::None;
    };
    let Some(spatial) = movement.segment.spatial.as_ref() else {
        return SurfaceMode::Constant(surface.correction_at(
            start_pos[0],
            start_pos[1],
            start_pos[2],
        ));
    };
    let length = movement.segment.s_len();
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for sample in 0..=WARP_BBOX_SAMPLES {
        let point = spatial.point_at(length * sample as f64 / WARP_BBOX_SAMPLES as f64);
        for axis in 0..3 {
            lo[axis] = lo[axis].min(point[axis]);
            hi[axis] = hi[axis].max(point[axis]);
        }
    }
    let ds = length / WARP_BBOX_SAMPLES as f64;
    let pad = {
        use geometry::path::CurvatureProfile;
        spatial.kappa_peak().1.abs() * ds * ds / 8.0
    };
    let spread = surface.correction_spread_over(
        lo[0] - pad,
        hi[0] + pad,
        lo[1] - pad,
        hi[1] + pad,
        lo[2] - pad,
        hi[2] + pad,
    );
    if spread == 0.0 {
        let point = spatial.point_at(0.0);
        SurfaceMode::Constant(surface.correction_at(point[0], point[1], point[2]))
    } else {
        SurfaceMode::Variable(Arc::clone(surface))
    }
}

fn rest_z_warp(mesh: Option<&SurfaceTransform>, odometer: &[f64]) -> f64 {
    mesh.map_or(0.0, |surface| {
        surface.correction_at(
            odometer.first().copied().unwrap_or(0.0),
            odometer.get(1).copied().unwrap_or(0.0),
            odometer.get(2).copied().unwrap_or(0.0),
        )
    })
}
