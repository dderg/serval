use crossbeam_channel::{Receiver, Sender};
use geometry::fitter::{
    JunctionPlan, RunFit, arc_candidate_fits, blend_moves, consumption_moves,
    facet_consumption_candidate, is_travel, merge_collinear_lines, plan_facet_consumption,
    plan_junction_reduced, spatial_end, spatial_start, trim_line_move,
};
use geometry::path::{Line, PathSegment, Segment};
use geometry::{CornerFitConfig, Move};

use crate::{CONTIGUITY_EPS_MM, Control, StreamInput, dist3};

const ALIGN_EPS_MM: f64 = 1e-9;
const MIN_RUN_FACETS: usize = 3;
/// Longest facet chain one consuming blend may replace — bounds how far the
/// stage buffers past the first squeezed facet before giving up on finding
/// the far anchor.
const MAX_CONSUMED_FACETS: usize = 8;

enum ClusterScan {
    /// The cluster's far anchor is not buffered yet and more input may come.
    WaitForInput,
    /// `decided[1..=n]` are consumable facets and `decided[n + 1]` is the far
    /// anchor piece.
    Anchor(usize),
    /// The chain runs into a run, end-of-stream, or past the facet cap —
    /// no consumption; the junctions resolve pairwise.
    NoAnchor,
}

/// First pipeline stage: reads raw moves, fits them into G2-continuous
/// geometry, and pushes the fitted pieces downstream.
///
/// It reads ahead only while the next move can still change how the buffered
/// prefix fits, and commits (emits) as soon as a fitted piece is final under
/// any future append. `Drain` (or the input closing) resolves and emits
/// everything unconditionally and is forwarded downstream; the fit stage
/// never gives up its lookahead on its own.
///
/// Structure is decided greedily at the undecided tail: the longest prefix
/// through which one arc still fits keeps growing, and the first move that
/// breaks the fit seals the run behind it — an arc that failed through a
/// fixed prefix stays failed no matter what arrives later. A run's
/// reconstruction (the arc plus the easing clothoids into its neighbor lines)
/// is computed only once the run's extent and both neighbors are final,
/// because easing into a tangent line refits the circle: partial arcs are
/// never emitted, and the lines around a still-growing run wait with it.
///
/// There is no head-restore: buffered moves keep their original geometry —
/// junction classification uses full raw lengths (minus explicit easing
/// reductions) exactly as the batch fit does — and length already paid out to
/// an emitted blend or easing is applied only when a body is finally emitted.
pub struct FitStage {
    corner: CornerFitConfig,
    decided: Vec<Element>,
    tail: Vec<Move>,
    tail_checked: usize,
    seam_head_trim: f64,
    seam_in_reduction: f64,
    consume_scan_start: usize,
    merge_absorbed: Vec<[f64; 3]>,
}

enum Element {
    Piece(Move),
    Run(RunElement),
}

struct RunElement {
    facets: Vec<Move>,
    fit: Option<RunFit>,
    head_blend: Vec<Move>,
    tail_blend: Option<Vec<Move>>,
}

impl RunElement {
    fn sealed(facets: Vec<Move>) -> Element {
        Element::Run(RunElement {
            facets,
            fit: None,
            head_blend: Vec::new(),
            tail_blend: None,
        })
    }
}

fn piece_of(e: &Element) -> Option<&Move> {
    match e {
        Element::Piece(m) => Some(m),
        Element::Run(_) => None,
    }
}

impl FitStage {
    pub fn new(corner: CornerFitConfig) -> Self {
        Self {
            corner,
            decided: Vec::new(),
            tail: Vec::new(),
            tail_checked: 1,
            seam_head_trim: 0.0,
            seam_in_reduction: 0.0,
            consume_scan_start: MAX_CONSUMED_FACETS,
            merge_absorbed: Vec::new(),
        }
    }

    /// Append a raw move to the undecided tail, merging it into the previous
    /// move when the junction between them is a sub-degree turn within the
    /// corner-deviation budget — see [`merge_collinear_lines`]. Slicers break
    /// gentle transitions into sub-degree facets; kept separate, the blends
    /// at those junctions squeeze the facet bodies into micrometre slivers
    /// whose lowered acceleration rings far past the machine limits.
    fn ingest(&mut self, m: Move) {
        if let Some(last) = self.tail.last_mut() {
            if let Some(merged) = merge_collinear_lines(last, &m, &self.merge_absorbed, self.corner)
            {
                let junction = spatial_end(last).expect("merged moves are spatial lines");
                self.merge_absorbed.push(junction);
                *last = merged;
                self.tail_checked = self.tail_checked.min(self.tail.len() - 1).max(1);
                return;
            }
        }
        self.merge_absorbed.clear();
        self.tail.push(m);
    }

    pub fn run(self, input: Receiver<StreamInput>, output: Sender<StreamInput>) {
        let mut driver = self.into_driver(output);
        while let Ok(item) = input.recv() {
            if !driver.feed(item) {
                return;
            }
        }
        driver.finish();
    }

    pub fn into_driver(self, output: Sender<StreamInput>) -> FitDriver {
        FitDriver {
            stage: self,
            out: TravelAligningSender::new(output),
        }
    }

    /// `Reset` drops all buffered fit state and forgets the emitted-geometry
    /// anchor (the timeline restarts elsewhere); every other token requires
    /// the fit buffers to have been drained first.
    fn forward_control(&mut self, ctrl: Control, out: &mut TravelAligningSender) -> bool {
        match &ctrl {
            Control::Reset { .. } => {
                self.decided.clear();
                self.tail.clear();
                self.merge_absorbed.clear();
                self.tail_checked = 1;
                self.seam_head_trim = 0.0;
                self.seam_in_reduction = 0.0;
                out.reset();
            }
            Control::Dwell { .. }
            | Control::SetAxisChains(_)
            | Control::SetMesh { .. }
            | Control::Nudge { .. }
            | Control::Barrier(_) => {
                assert!(
                    self.decided.is_empty() && self.tail.is_empty(),
                    "fit_stage: control token arrived with undrained moves — a Drain must precede it"
                );
            }
        }
        if let Control::SetMesh { gcode_z_rebase, .. } = &ctrl {
            out.rebase_gcode_z(*gcode_z_rebase);
        }
        out.forward(StreamInput::Control(ctrl))
    }

    /// Advance every stage as far as the buffered input allows: decide the
    /// undecided tail into elements, reconstruct sealed runs whose neighbors
    /// are known, resolve boundary blends, and emit the finished prefix. With
    /// `eof` the input ran empty (or closed), so everything decides and emits
    /// now rather than waiting for moves that may never come.
    fn resolve(&mut self, eof: bool, out: &mut TravelAligningSender) -> bool {
        self.decide_kinds(eof);
        self.resolve_runs(eof);
        let ok = self.emit_ready(eof, out);
        if eof && ok {
            debug_assert!(self.decided.is_empty() && self.tail.is_empty());
            self.seam_head_trim = 0.0;
            self.seam_in_reduction = 0.0;
        }
        ok
    }

    /// Greedy structure decision over the undecided tail. `tail_checked`
    /// facets at the front are known to fit one arc; each check either grows
    /// that prefix or breaks it, sealing a run (if long enough) or condemning
    /// the front move to be a plain piece.
    fn decide_kinds(&mut self, eof: bool) {
        while !self.tail.is_empty() {
            let n = self.tail.len();
            let mut broke = false;
            while self.tail_checked < n {
                if !arc_candidate_fits(&self.tail[..=self.tail_checked], self.corner) {
                    broke = true;
                    break;
                }
                self.tail_checked += 1;
            }
            if !broke && !eof {
                return;
            }
            if self.tail_checked >= MIN_RUN_FACETS {
                let facets: Vec<Move> = self.tail.drain(..self.tail_checked).collect();
                self.decided.push(RunElement::sealed(facets));
            } else {
                self.decided.push(Element::Piece(self.tail.remove(0)));
            }
            self.tail_checked = 1;
        }
    }

    /// Reconstruct sealed runs whose following element's kind is known (the
    /// tail neighbor determines easing and occupancy) and resolve their head
    /// boundary blends; then resolve tail blends once both sides'
    /// reconstructions exist. A run that fails to reconstruct dissolves back
    /// into plain pieces.
    fn resolve_runs(&mut self, eof: bool) {
        let mut idx = 0;
        while idx < self.decided.len() {
            let Element::Run(re) = &self.decided[idx] else {
                idx += 1;
                continue;
            };
            if re.fit.is_some() {
                idx += 1;
                continue;
            }
            if idx + 1 >= self.decided.len() && !eof {
                break;
            }
            let head = (idx > 0).then(|| &self.decided[idx - 1]).and_then(piece_of);
            let tail = self.decided.get(idx + 1).and_then(piece_of);
            let fit = RunFit::fit(&re.facets, head, tail, self.corner)
                .unwrap_or_else(|e| panic!("fit_stage: run reconstruction failed: {e:?}"));
            let head = head.cloned();
            match fit {
                Some(mut fit) => {
                    let Element::Run(re) = &mut self.decided[idx] else {
                        unreachable!()
                    };
                    re.head_blend = match &head {
                        Some(prev) => fit
                            .blend_head_with_line(prev, &re.facets[0], self.corner)
                            .unwrap_or_else(|e| panic!("fit_stage: head blend failed: {e:?}")),
                        None => Vec::new(),
                    };
                    re.fit = Some(fit);
                    idx += 1;
                }
                None => {
                    let Element::Run(re) = self.decided.remove(idx) else {
                        unreachable!()
                    };
                    tracing::warn!(
                        subsystem = "motion",
                        event = "arc_run_dissolved",
                        line_lo = re.facets.first().map_or(0, |m| m.source.start_line),
                        line_hi = re.facets.last().map_or(0, |m| m.source.start_line),
                        n_facets = re.facets.len(),
                        "arc run failed reconstruction; falling back to per-corner blending"
                    );
                    self.decided
                        .splice(idx..idx, re.facets.into_iter().map(Element::Piece));
                }
            }
        }
        self.resolve_tail_blends(eof);
    }

    fn resolve_tail_blends(&mut self, eof: bool) {
        let corner = self.corner;
        for idx in 0..self.decided.len() {
            let Element::Run(re) = &self.decided[idx] else {
                continue;
            };
            if re.fit.is_none() {
                break;
            }
            if re.tail_blend.is_some() {
                continue;
            }
            let blend = match self.decided.get(idx + 1) {
                None => {
                    if !eof {
                        break;
                    }
                    Vec::new()
                }
                Some(Element::Piece(next)) => {
                    let next = next.clone();
                    let Element::Run(re) = &mut self.decided[idx] else {
                        unreachable!()
                    };
                    let run_last = re.facets.last().expect("run has facets").clone();
                    re.fit
                        .as_mut()
                        .expect("fit checked above")
                        .blend_tail_with_line(&run_last, &next, corner)
                        .unwrap_or_else(|e| panic!("fit_stage: tail blend failed: {e:?}"))
                }
                Some(Element::Run(next)) => {
                    if next.fit.is_none() {
                        break;
                    }
                    let (front, back) = self.decided.split_at_mut(idx + 1);
                    let Element::Run(re) = &mut front[idx] else {
                        unreachable!()
                    };
                    let Element::Run(next) = &mut back[0] else {
                        unreachable!()
                    };
                    re.fit
                        .as_mut()
                        .expect("fit checked above")
                        .blend_tail_with_run(
                            next.fit.as_mut().expect("checked above"),
                            re.facets.last().expect("run has facets"),
                            &next.facets[0],
                            corner,
                        )
                        .unwrap_or_else(|e| panic!("fit_stage: arc-arc blend failed: {e:?}"))
                }
            };
            let Element::Run(re) = &mut self.decided[idx] else {
                unreachable!()
            };
            re.tail_blend = Some(blend);
        }
    }

    /// Emit the front elements whose every input is final. A plain piece
    /// needs its exit junction plan, which needs the next element's kind —
    /// and, for the easing budget reduction, the reconstruction of a run
    /// starting one move further. A run needs its reconstruction and both
    /// boundary blends.
    fn emit_ready(&mut self, eof: bool, out: &mut TravelAligningSender) -> bool {
        loop {
            match self.decided.first() {
                None => return true,
                Some(Element::Piece(_)) => match self.decided.get(1) {
                    None => {
                        if !eof {
                            return true;
                        }
                        if !self.emit_front_piece(0.0, out) {
                            return false;
                        }
                        self.seam_head_trim = 0.0;
                        self.seam_in_reduction = 0.0;
                    }
                    Some(Element::Run(next)) => {
                        let Some(fit) = &next.fit else { return true };
                        let trim_end = fit.head_boundary_trim();
                        if !self.emit_front_piece(trim_end, out) {
                            return false;
                        }
                        let Element::Run(re) = &mut self.decided[0] else {
                            unreachable!()
                        };
                        for half in std::mem::take(&mut re.head_blend) {
                            if !out.send(half) {
                                return false;
                            }
                        }
                        self.seam_head_trim = 0.0;
                        self.seam_in_reduction = 0.0;
                    }
                    Some(Element::Piece(mid)) => {
                        let front = piece_of(&self.decided[0]).expect("front matched a piece");
                        if facet_consumption_candidate(
                            front,
                            mid,
                            self.corner,
                            self.seam_in_reduction,
                        ) {
                            match self.consumable_cluster(eof) {
                                ClusterScan::WaitForInput => return true,
                                ClusterScan::Anchor(max_mids) => {
                                    match self.try_consumption(max_mids, out) {
                                        Some(false) => return false,
                                        Some(true) => continue,
                                        None => {}
                                    }
                                }
                                ClusterScan::NoAnchor => {}
                            }
                        }
                        let out_reduction = match self.decided.get(2) {
                            None if !eof => return true,
                            None | Some(Element::Piece(_)) => 0.0,
                            Some(Element::Run(after)) => match &after.fit {
                                None => return true,
                                Some(fit) => fit.head_line_trim(),
                            },
                        };
                        if !self.emit_pairwise(out_reduction, out) {
                            return false;
                        }
                    }
                },
                Some(Element::Run(re)) => {
                    if re.fit.is_none() || re.tail_blend.is_none() {
                        return true;
                    }
                    if !self.emit_front_run(out) {
                        return false;
                    }
                }
            }
        }
    }

    /// Emit the front piece's body and its exit-junction blend against the
    /// piece after it, carrying the blend trim to that piece's head.
    fn emit_pairwise(&mut self, out_reduction: f64, out: &mut TravelAligningSender) -> bool {
        let (Some(m), Some(next)) = (piece_of(&self.decided[0]), piece_of(&self.decided[1])) else {
            unreachable!("caller matched two front pieces")
        };
        let plan =
            plan_junction_reduced(m, next, self.corner, self.seam_in_reduction, out_reduction)
                .unwrap_or_else(|e| panic!("fit_stage: junction plan failed: {e:?}"));
        let (trim_end, next_head_trim, blend) = match plan {
            JunctionPlan::Blend(b) => (b.trim_in(), b.trim_out(), Some(b)),
            JunctionPlan::Unblended(_) => (0.0, 0.0, None),
        };
        let Element::Piece(m) = self.decided.remove(0) else {
            unreachable!()
        };
        let body = trim_line_move(&m, self.seam_head_trim, trim_end).unwrap_or_else(|e| {
            panic!(
                "fit_stage: trim of line {} failed: {e:?}",
                m.source.start_line
            )
        });
        if let Some(body) = body {
            if !out.send(body) {
                return false;
            }
        }
        if let Some(b) = blend {
            let next = piece_of(&self.decided[0]).expect("checked above");
            let halves = blend_moves(&b, &m, next).unwrap_or_else(|e| {
                panic!(
                    "fit_stage: blend at line {} failed: {e:?}",
                    next.source.start_line
                )
            });
            for half in halves {
                if !out.send(half) {
                    return false;
                }
            }
        }
        self.seam_head_trim = next_head_trim;
        self.seam_in_reduction = 0.0;
        true
    }

    /// How far the consumable chain behind the front piece extends: grow
    /// while each next buffered piece is itself a squeezed facet off the one
    /// before it, and report the first piece that isn't as the far anchor.
    /// When the chain runs into a run or end-of-stream instead of an anchor
    /// piece, the last facet itself can still anchor a shorter consumption.
    fn consumable_cluster(&self, eof: bool) -> ClusterScan {
        let mut n = 1;
        loop {
            match self.decided.get(n + 1) {
                None if !eof => return ClusterScan::WaitForInput,
                None | Some(Element::Run(_)) => {
                    return if n >= 2 {
                        ClusterScan::Anchor(n - 1)
                    } else {
                        ClusterScan::NoAnchor
                    };
                }
                Some(Element::Piece(next)) => {
                    let prev = piece_of(&self.decided[n]).expect("chain grew over pieces");
                    if n == MAX_CONSUMED_FACETS
                        || !facet_consumption_candidate(prev, next, self.corner, 0.0)
                    {
                        return ClusterScan::Anchor(n);
                    }
                    n += 1;
                }
            }
        }
    }

    /// Consume the longest prefix of the scanned facet chain that passes
    /// every gate: shrink from the longest candidate — a shorter prefix
    /// anchors on the facet that follows it, exactly as a lone squeezed
    /// facet anchors on any next piece. Each failed prefix costs a full
    /// chain-blend scan, and on uniform curves (circle facets) the feasible
    /// length is the same junction after junction, so the descent starts one
    /// above the last consumption's length instead of at `max_mids`: it
    /// re-earns longer prefixes one junction at a time and never re-pays for
    /// lengths that just failed. Returns the send result, or `None` when no
    /// prefix is consumable.
    fn try_consumption(&mut self, max_mids: usize, out: &mut TravelAligningSender) -> Option<bool> {
        let start_mids = max_mids
            .min(self.consume_scan_start.saturating_add(1))
            .max(1);
        for n_mids in (1..=start_mids).rev() {
            let front = piece_of(&self.decided[0]).expect("caller matched a front piece");
            let mids: Vec<&Move> = self.decided[1..=n_mids]
                .iter()
                .map(|e| piece_of(e).expect("scan matched pieces"))
                .collect();
            let after = piece_of(&self.decided[n_mids + 1]).expect("scan matched the anchor piece");
            let plan =
                plan_facet_consumption(front, &mids, after, self.corner, self.seam_in_reduction)
                    .unwrap_or_else(|e| panic!("fit_stage: facet consumption failed: {e:?}"));
            if let Some(fc) = plan {
                self.consume_scan_start = n_mids;
                return Some(self.emit_consumption(fc, n_mids, out));
            }
        }
        self.consume_scan_start = 0;
        None
    }

    /// Emit the consumption of the facet chain behind the front piece: the
    /// front piece's body trimmed at its tail, the blend segments spanning
    /// the consumed facets, and nothing of the facets themselves — their
    /// geometry and extrusion live in the blend. The trim the blend claims
    /// from the anchor piece's head is carried to its eventual emission.
    fn emit_consumption(
        &mut self,
        fc: geometry::fitter::FacetConsumption,
        n_mids: usize,
        out: &mut TravelAligningSender,
    ) -> bool {
        let Element::Piece(front) = self.decided.remove(0) else {
            unreachable!("caller matched a front piece")
        };
        let mids: Vec<Move> = self
            .decided
            .drain(..n_mids)
            .map(|e| {
                let Element::Piece(m) = e else {
                    unreachable!("caller matched consumable pieces")
                };
                m
            })
            .collect();
        let mid_refs: Vec<&Move> = mids.iter().collect();
        let after = piece_of(&self.decided[0]).expect("caller matched the anchor piece");
        let body = trim_line_move(&front, self.seam_head_trim, fc.trim_in()).unwrap_or_else(|e| {
            panic!(
                "fit_stage: trim of line {} failed: {e:?}",
                front.source.start_line
            )
        });
        if let Some(body) = body {
            if !out.send(body) {
                return false;
            }
        }
        let segments = consumption_moves(&fc, &front, &mid_refs, after).unwrap_or_else(|e| {
            panic!(
                "fit_stage: consumption of line {} failed: {e:?}",
                mids[0].source.start_line
            )
        });
        for seg in segments {
            if !out.send(seg) {
                return false;
            }
        }
        self.seam_head_trim = fc.trim_out();
        self.seam_in_reduction = 0.0;
        true
    }

    /// Emit the front piece's body with the given tail trim and drop it.
    fn emit_front_piece(&mut self, trim_end: f64, out: &mut TravelAligningSender) -> bool {
        let Element::Piece(m) = self.decided.remove(0) else {
            unreachable!("caller matched a front piece")
        };
        let body = trim_line_move(&m, self.seam_head_trim, trim_end).unwrap_or_else(|e| {
            panic!(
                "fit_stage: trim of line {} failed: {e:?}",
                m.source.start_line
            )
        });
        match body {
            Some(body) => out.send(body),
            None => true,
        }
    }

    /// Emit a fully resolved run: the first facet's remaining head stub, the
    /// reconstruction pieces, the last facet's tail stub, and the tail
    /// boundary blend — then carry the easing trims to the next element.
    fn emit_front_run(&mut self, out: &mut TravelAligningSender) -> bool {
        let Element::Run(re) = self.decided.remove(0) else {
            unreachable!("caller matched a front run")
        };
        let fit = re.fit.expect("checked by caller");
        let first = &re.facets[0];
        let last = re.facets.last().expect("run has facets");
        let head_stub = trim_line_move(first, 0.0, fit.head_consumption())
            .unwrap_or_else(|e| panic!("fit_stage: run head stub failed: {e:?}"));
        if let Some(stub) = head_stub {
            if !out.send(stub) {
                return false;
            }
        }
        for p in fit
            .pieces(first, last)
            .unwrap_or_else(|e| panic!("fit_stage: run emission failed: {e:?}"))
        {
            if !out.send(p) {
                return false;
            }
        }
        let tail_stub = trim_line_move(last, fit.tail_consumption(), 0.0)
            .unwrap_or_else(|e| panic!("fit_stage: run tail stub failed: {e:?}"));
        if let Some(stub) = tail_stub {
            if !out.send(stub) {
                return false;
            }
        }
        for half in re.tail_blend.expect("checked by caller") {
            if !out.send(half) {
                return false;
            }
        }
        self.seam_head_trim = fit.tail_boundary_trim();
        self.seam_in_reduction = fit.tail_line_trim();
        true
    }
}

/// Per-item drive of the fit stage for single-threaded hosts (the snapshot
/// harness, wasm): `feed` is one iteration of [`FitStage::run`]'s loop,
/// `finish` is its input-closed path — resolve and flush everything without
/// forwarding a `Drain`.
pub struct FitDriver {
    stage: FitStage,
    out: TravelAligningSender,
}

impl FitDriver {
    pub fn feed(&mut self, item: StreamInput) -> bool {
        match item {
            StreamInput::Move(m) => {
                self.stage.ingest(m);
                self.stage.resolve(false, &mut self.out)
            }
            StreamInput::Drain => {
                self.stage.resolve(true, &mut self.out)
                    && self.out.release(None)
                    && self.out.forward_drain()
            }
            StreamInput::Control(ctrl) => self.stage.forward_control(ctrl, &mut self.out),
        }
    }

    pub fn finish(&mut self) -> bool {
        self.stage.resolve(true, &mut self.out) && self.out.release(None)
    }
}

/// Streaming port of the batch fit's `align_travels`: a travel (non-extruding
/// line) is re-anchored onto the fitted neighbors' actual endpoints, which can
/// deviate from its raw endpoints after arc reconstruction. Its exit anchor is
/// the next spatial piece's start, so a travel is parked until that piece
/// arrives (non-spatial pieces queue behind it to preserve order); `release`
/// with no anchor keeps the travel's own end, exactly as the batch fit does at
/// the end of its window.
struct TravelAligningSender {
    tx: Sender<StreamInput>,
    last_spatial_end: Option<[f64; 3]>,
    parked_travel: Option<Move>,
    parked_tail: Vec<Move>,
}

impl TravelAligningSender {
    fn new(tx: Sender<StreamInput>) -> Self {
        Self {
            tx,
            last_spatial_end: None,
            parked_travel: None,
            parked_tail: Vec::new(),
        }
    }

    fn send(&mut self, m: Move) -> bool {
        let Some(start) = spatial_start(&m) else {
            if self.parked_travel.is_some() {
                self.parked_tail.push(m);
                return true;
            }
            return self.tx.send(m.into()).is_ok();
        };
        if !self.release(Some(start)) {
            return false;
        }
        if is_travel(&m) {
            self.parked_travel = Some(m);
            return true;
        }
        if let Some(prev_end) = self.last_spatial_end {
            let gap = dist3(prev_end, start);
            assert!(
                gap <= CONTIGUITY_EPS_MM,
                "fit_stage emitted discontinuous geometry at line {}: previous piece ends at \
                 {prev_end:?}, next starts at {start:?} ({gap:.9}mm gap)",
                m.source.start_line
            );
        }
        self.last_spatial_end = spatial_end(&m);
        self.tx.send(m.into()).is_ok()
    }

    fn release(&mut self, next_start: Option<[f64; 3]>) -> bool {
        let Some(travel) = self.parked_travel.take() else {
            debug_assert!(self.parked_tail.is_empty());
            return true;
        };
        let travel = align_travel(travel, self.last_spatial_end, next_start);
        self.last_spatial_end = spatial_end(&travel);
        if self.tx.send(travel.into()).is_err() {
            return false;
        }
        for m in self.parked_tail.drain(..) {
            if self.tx.send(m.into()).is_err() {
                return false;
            }
        }
        true
    }

    fn forward_drain(&self) -> bool {
        debug_assert!(self.parked_travel.is_none() && self.parked_tail.is_empty());
        self.tx.send(StreamInput::Drain).is_ok()
    }

    fn forward(&self, item: StreamInput) -> bool {
        debug_assert!(self.parked_travel.is_none() && self.parked_tail.is_empty());
        self.tx.send(item).is_ok()
    }

    /// A mesh swap renames the resting point's gcode Z (the machine position
    /// is invariant); the emitted-geometry anchor must adopt the new name or
    /// the next move looks discontinuous against a stale coordinate.
    fn rebase_gcode_z(&mut self, z: f64) {
        debug_assert!(self.parked_travel.is_none() && self.parked_tail.is_empty());
        if let Some(end) = self.last_spatial_end.as_mut() {
            end[2] = z;
        }
    }

    fn reset(&mut self) {
        self.last_spatial_end = None;
        self.parked_travel = None;
        self.parked_tail.clear();
    }
}

fn align_travel(m: Move, prev_end: Option<[f64; 3]>, next_start: Option<[f64; 3]>) -> Move {
    let Some(Segment::Line(line)) = &m.segment.spatial else {
        unreachable!("parked travel is always a line");
    };
    let a = prev_end.unwrap_or(line.start);
    let b = next_start.unwrap_or(line.end);
    if dist3(a, line.start) <= ALIGN_EPS_MM && dist3(b, line.end) <= ALIGN_EPS_MM {
        return m;
    }
    let line_no = m.source.start_line;
    let aligned = Line::try_new(a, b)
        .unwrap_or_else(|e| panic!("fit_stage: travel align of line {line_no} failed: {e:?}"));
    let segment = PathSegment::try_new(Segment::Line(aligned), m.segment.followers.clone())
        .unwrap_or_else(|e| panic!("fit_stage: travel align of line {line_no} failed: {e:?}"));
    Move {
        segment,
        feedrate_mm_s: m.feedrate_mm_s,
        limits: m.limits,
        source: m.source,
    }
}

#[cfg(test)]
mod tests;
