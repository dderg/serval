use crate::frontend::Move;
use crate::path::CurvatureProfile;
use crate::segment::FollowerDemand;

use super::emit::{SeamSide, blend_followers};
use super::move_ops::{is_travel, line_of};
use super::{CornerFitConfig, FitError, causal, kernels, overlap, ramps_admitted, span_tolerance};

/// A sealed arc run's reconstruction: the arc, its easing clothoids into the
/// neighbor lines, and any boundary blends resolved against them. Built only
/// once the run's extent and both neighbors are final — easing refits the
/// circle so the arc geometry does not exist before then.
pub struct RunFit {
    recon: kernels::Reconstruction,
    tol: f64,
    head_blend_trim: f64,
    tail_blend_trim: f64,
    head_line_extra: f64,
    tail_line_extra: f64,
}

impl RunFit {
    /// Reconstruct and ease a sealed run. `head`/`tail` are the adjoining
    /// moves when they are plain (not part of another run); `None` when the
    /// run abuts another run, a stream edge, or nothing. Returns `Ok(None)`
    /// when no valid reconstruction exists — the facets stay plain lines.
    /// A bare reconstruction whose extrusion ramp fails the kinematic gate
    /// dissolves the same way; an easing that fails it is dropped while the
    /// bare reconstruction stands.
    pub fn fit(
        facets: &[Move],
        head: Option<&Move>,
        tail: Option<&Move>,
        corner: CornerFitConfig,
    ) -> Result<Option<RunFit>, FitError> {
        let tol = span_tolerance(facets);
        if !tol.is_finite() {
            return Ok(None);
        }
        let travel_len = |m: &Move| {
            is_travel(m)
                .then(|| line_of(m).map(|l| l.s_len()))
                .flatten()
        };
        let Some(mut recon) = kernels::reconstruct(
            facets,
            tol,
            head.and_then(travel_len),
            tail.and_then(travel_len),
        )?
        else {
            return Ok(None);
        };
        if !construct_admitted(&recon, facets, corner.ramp_accel_budget_mm_s2) {
            return Ok(None);
        }
        let bare = recon.clone();
        let head_nb = head.and_then(|m| kernels::neighbor(m, true));
        let tail_nb = tail.and_then(|m| kernels::neighbor(m, false));
        kernels::ease_run(&mut recon, facets, head_nb.as_ref(), tail_nb.as_ref(), tol)?;
        if !construct_admitted(&recon, facets, corner.ramp_accel_budget_mm_s2) {
            recon = bare;
        }
        Ok(Some(RunFit {
            recon,
            tol,
            head_blend_trim: 0.0,
            tail_blend_trim: 0.0,
            head_line_extra: 0.0,
            tail_line_extra: 0.0,
        }))
    }

    /// Length consumed from the head neighbor's tail (easing plus boundary
    /// blend) — the neighbor's emission trim and the run's first-facet head
    /// trim, exactly as the batch fit applies them.
    #[must_use]
    pub fn head_boundary_trim(&self) -> f64 {
        self.recon.head_line_trim + self.head_line_extra
    }

    #[must_use]
    pub fn tail_boundary_trim(&self) -> f64 {
        self.recon.tail_line_trim + self.tail_line_extra
    }

    /// Easing consumption alone — the budget reduction junction classification
    /// applies two junctions out from the run.
    #[must_use]
    pub fn head_line_trim(&self) -> f64 {
        self.recon.head_line_trim
    }

    #[must_use]
    pub fn tail_line_trim(&self) -> f64 {
        self.recon.tail_line_trim
    }

    #[must_use]
    pub fn head_consumption(&self) -> f64 {
        self.recon.head_consumption
    }

    #[must_use]
    pub fn tail_consumption(&self) -> f64 {
        self.recon.tail_consumption
    }

    /// Blend the run's un-eased head into the bare neighbor line before it.
    /// Returns the blend's clothoid halves to emit between the neighbor and
    /// the run (empty when no blend applies or its extrusion ramp fails the
    /// kinematic gate — the seam then stays sharp and the planner stops).
    pub fn blend_head_with_line(
        &mut self,
        neighbor: &Move,
        run_first: &Move,
        corner: CornerFitConfig,
    ) -> Result<Vec<Move>, FitError> {
        if !self.recon.up.is_empty() {
            return Ok(Vec::new());
        }
        let Some(line) = line_of(neighbor) else {
            return Ok(Vec::new());
        };
        let Some(blend) = overlap::resolve_arc_line(&self.recon.arc, line, false, corner, self.tol)
        else {
            return Ok(Vec::new());
        };
        let Some(out) = general_blend(
            &blend,
            SeamSide {
                followers: &neighbor.segment.followers,
                seg_len: line.s_len(),
                trim: blend.trim_in,
            },
            SeamSide {
                followers: &self.recon.followers,
                seg_len: self.recon.arc.s_len(),
                trim: blend.trim_out,
            },
            neighbor,
            run_first,
            corner,
        )?
        else {
            return Ok(Vec::new());
        };
        self.head_line_extra = blend.trim_in;
        self.head_blend_trim = blend.trim_out;
        Ok(out)
    }

    /// Blend the run's un-eased tail into the bare neighbor line after it.
    pub fn blend_tail_with_line(
        &mut self,
        run_last: &Move,
        neighbor: &Move,
        corner: CornerFitConfig,
    ) -> Result<Vec<Move>, FitError> {
        if !self.recon.down.is_empty() {
            return Ok(Vec::new());
        }
        let Some(line) = line_of(neighbor) else {
            return Ok(Vec::new());
        };
        let Some(blend) = overlap::resolve_arc_line(&self.recon.arc, line, true, corner, self.tol)
        else {
            return Ok(Vec::new());
        };
        let Some(out) = general_blend(
            &blend,
            SeamSide {
                followers: &self.recon.followers,
                seg_len: self.recon.arc.s_len(),
                trim: blend.trim_in,
            },
            SeamSide {
                followers: &neighbor.segment.followers,
                seg_len: line.s_len(),
                trim: blend.trim_out,
            },
            run_last,
            neighbor,
            corner,
        )?
        else {
            return Ok(Vec::new());
        };
        self.tail_blend_trim = blend.trim_in;
        self.tail_line_extra = blend.trim_out;
        Ok(out)
    }

    /// Blend two adjacent runs' arcs at their shared junction.
    pub fn blend_tail_with_run(
        &mut self,
        next: &mut RunFit,
        run_last: &Move,
        next_first: &Move,
        corner: CornerFitConfig,
    ) -> Result<Vec<Move>, FitError> {
        if !(self.recon.down.is_empty() && next.recon.up.is_empty()) {
            return Ok(Vec::new());
        }
        let Some(blend) =
            overlap::resolve_arc_arc(&self.recon.arc, &next.recon.arc, corner, self.tol)
        else {
            return Ok(Vec::new());
        };
        let Some(out) = general_blend(
            &blend,
            SeamSide {
                followers: &self.recon.followers,
                seg_len: self.recon.arc.s_len(),
                trim: blend.trim_in,
            },
            SeamSide {
                followers: &next.recon.followers,
                seg_len: next.recon.arc.s_len(),
                trim: blend.trim_out,
            },
            run_last,
            next_first,
            corner,
        )?
        else {
            return Ok(Vec::new());
        };
        self.tail_blend_trim = blend.trim_in;
        next.head_blend_trim = blend.trim_out;
        Ok(out)
    }

    /// The run's replacement pieces: up-easing clothoids, the (blend-trimmed)
    /// arc, and down-easing clothoids. The first/last facets' remaining stubs
    /// are the caller's to emit around these.
    pub fn pieces(&self, m_start: &Move, m_end: &Move) -> Result<Vec<Move>, FitError> {
        let mut out = Vec::new();
        causal::emit_reconstruction(
            &mut out,
            &self.recon,
            m_start,
            m_end,
            self.head_blend_trim,
            self.tail_blend_trim,
        )?;
        Ok(out)
    }
}

fn general_blend(
    blend: &super::biclothoid::GeneralBlend,
    in_side: SeamSide,
    out_side: SeamSide,
    m_in: &Move,
    m_out: &Move,
    corner: CornerFitConfig,
) -> Result<Option<Vec<Move>>, FitError> {
    let (f_in, f_out) = blend_followers(
        &in_side,
        &out_side,
        blend.half1.s_len(),
        blend.half2.s_len(),
    );
    if !general_blend_admitted(
        blend,
        &f_in,
        &f_out,
        m_in,
        m_out,
        corner.ramp_accel_budget_mm_s2,
    ) {
        return Ok(None);
    }
    let mut out = Vec::with_capacity(2);
    causal::emit_general_blend(&mut out, blend, f_in, f_out, m_in, m_out)?;
    Ok(Some(out))
}

fn general_blend_admitted(
    blend: &super::biclothoid::GeneralBlend,
    f_in: &[FollowerDemand],
    f_out: &[FollowerDemand],
    m_in: &Move,
    m_out: &Move,
    accel_budget: f64,
) -> bool {
    ramps_admitted(
        accel_budget,
        f_in,
        &blend.half1,
        m_in.feedrate_mm_s,
        m_in.limits,
    ) && ramps_admitted(
        accel_budget,
        f_out,
        &blend.half2,
        m_out.feedrate_mm_s,
        m_out.limits,
    )
}

/// Every ramp the reconstruction carries — easing spirals and the arc — must
/// pass the kinematic gate on its carrying piece (the same move whose
/// feedrate and limits the emitted piece inherits).
fn construct_admitted(recon: &kernels::Reconstruction, facets: &[Move], accel_budget: f64) -> bool {
    let first = &facets[0];
    let last = facets.last().expect("run has facets");
    recon.up.iter().all(|c| {
        ramps_admitted(
            accel_budget,
            &recon.up_followers,
            c,
            first.feedrate_mm_s,
            first.limits,
        )
    }) && ramps_admitted(
        accel_budget,
        &recon.followers,
        &recon.arc,
        first.feedrate_mm_s,
        first.limits,
    ) && recon.down.iter().all(|c| {
        ramps_admitted(
            accel_budget,
            &recon.down_followers,
            c,
            last.feedrate_mm_s,
            last.limits,
        )
    })
}
