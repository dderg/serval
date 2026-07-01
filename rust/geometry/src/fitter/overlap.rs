use crate::path::CurvatureProfile;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Line};

use super::CornerFitConfig;
use super::biclothoid::{self, Anchor, GeneralBlend};
use super::kernels::arc_len;
use super::vec3::{add, cross, dot, normalize, scale};

fn plane_of(arc: &Arc) -> [f64; 3] {
    normalize(cross(arc.u, arc.v))
}

fn midpoint(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    scale(add(a, b), 0.5)
}

#[allow(clippy::too_many_arguments)]
fn resolve(
    anchor_in: Anchor,
    anchor_out: Anchor,
    apex: [f64; 3],
    plane_n: [f64; 3],
    config: CornerFitConfig,
    delta: f64,
    budget_in: f64,
    budget_out: f64,
    best_effort: bool,
) -> Option<GeneralBlend> {
    let theta = dot(anchor_in.tangent, anchor_out.tangent)
        .clamp(-1.0, 1.0)
        .acos();
    if theta <= config.theta_min_rad {
        return None;
    }
    biclothoid::solve_general(
        anchor_in,
        anchor_out,
        apex,
        plane_n,
        delta,
        budget_in,
        budget_out,
        best_effort,
    )
}

pub(super) fn resolve_arc_arc(
    arc_in: &Arc,
    arc_out: &Arc,
    config: CornerFitConfig,
    delta: f64,
) -> Option<GeneralBlend> {
    let a_in = Anchor {
        pose: arc_in.point_at(arc_in.s_len()),
        tangent: arc_in.heading_at(arc_in.s_len()),
        kappa: arc_in.kappa(0.0),
    };
    let a_out = Anchor {
        pose: arc_out.point_at(0.0),
        tangent: arc_out.heading_at(0.0),
        kappa: arc_out.kappa(0.0),
    };
    let apex = midpoint(a_in.pose, a_out.pose);
    resolve(
        a_in,
        a_out,
        apex,
        plane_of(arc_in),
        config,
        delta,
        0.5 * arc_len(arc_in),
        0.5 * arc_len(arc_out),
        false,
    )
}

pub(super) fn resolve_arc_line(
    arc: &Arc,
    line: &Line,
    arc_is_in: bool,
    config: CornerFitConfig,
    delta: f64,
) -> Option<GeneralBlend> {
    let plane_n = plane_of(arc);
    let arc_budget = 0.5 * arc_len(arc);
    let line_budget = 0.5 * line.s_len();
    if arc_is_in {
        let a_in = Anchor {
            pose: arc.point_at(arc.s_len()),
            tangent: arc.heading_at(arc.s_len()),
            kappa: arc.kappa(0.0),
        };
        let a_out = Anchor {
            pose: line.point_at(0.0),
            tangent: line.heading_at(0.0),
            kappa: 0.0,
        };
        let apex = midpoint(a_in.pose, a_out.pose);
        resolve(
            a_in,
            a_out,
            apex,
            plane_n,
            config,
            delta,
            arc_budget,
            line_budget,
            true,
        )
    } else {
        let a_in = Anchor {
            pose: line.point_at(line.s_len()),
            tangent: line.heading_at(line.s_len()),
            kappa: 0.0,
        };
        let a_out = Anchor {
            pose: arc.point_at(0.0),
            tangent: arc.heading_at(0.0),
            kappa: arc.kappa(0.0),
        };
        let apex = midpoint(a_in.pose, a_out.pose);
        resolve(
            a_in,
            a_out,
            apex,
            plane_n,
            config,
            delta,
            line_budget,
            arc_budget,
            true,
        )
    }
}
