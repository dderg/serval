use super::{CompensationSide, SeamRamp};

const SEAM: f64 = 12.5;

/// The table a ramp corrects: span `before` up to the seam, `after` from
/// it, each `(ratio at the seam, slope)`.
struct Table {
    before: (f64, f64),
    after: (f64, f64),
}

impl Table {
    fn ratio(&self, s: f64) -> f64 {
        let (ratio, slope) = if s < SEAM { self.before } else { self.after };
        ratio + slope * (s - SEAM)
    }

    fn slope(&self, s: f64) -> f64 {
        if s < SEAM {
            self.before.1
        } else {
            self.after.1
        }
    }

    fn ramp(&self, half_width: f64) -> SeamRamp {
        let slope_step = self.after.1 - self.before.1;
        SeamRamp::new(
            SEAM,
            half_width,
            (self.after.0 - self.before.0, slope_step),
            CompensationSide::inward(
                slope_step,
                self.ratio(SEAM - half_width),
                self.ratio(SEAM + half_width),
            ),
        )
    }
}

fn tables() -> Vec<Table> {
    vec![
        Table {
            before: (0.0, 0.0),
            after: (0.033, 0.0),
        },
        Table {
            before: (0.05, 0.0),
            after: (0.0, 0.0),
        },
        Table {
            before: (0.0, -0.08),
            after: (0.0, 0.0),
        },
        Table {
            before: (0.0, 0.0),
            after: (0.0, 0.11),
        },
        Table {
            before: (0.04, 0.01),
            after: (0.02, -0.03),
        },
    ]
}

fn corrected(table: &Table, ramp: &SeamRamp, s: f64) -> (f64, f64) {
    (
        table.ratio(s) + ramp.ratio(s),
        table.slope(s) + ramp.slope(s),
    )
}

#[test]
fn a_ramped_seam_is_c1_across_the_seam_and_both_window_ends() {
    for table in tables() {
        let half = 0.7;
        let ramp = table.ramp(half);
        let eps = 1e-9;
        for point in [SEAM - half, SEAM, SEAM + half] {
            let (left_ratio, left_slope) = corrected(&table, &ramp, point - eps);
            let (right_ratio, right_slope) = corrected(&table, &ramp, point + eps);
            assert!(
                (left_ratio - right_ratio).abs() < 1e-9,
                "ratio jumps at {point}: {left_ratio} -> {right_ratio}"
            );
            assert!(
                (left_slope - right_slope).abs() < 1e-7,
                "slope jumps at {point}: {left_slope} -> {right_slope}"
            );
        }
        if table.before.1 == table.after.1 {
            let (mean, _) = corrected(&table, &ramp, SEAM);
            assert!(
                (mean - 0.5 * (table.before.0 + table.after.0)).abs() < 1e-15,
                "a pure step's ramp crosses its seam at the mean ratio"
            );
        }
    }
}

#[test]
fn a_ramp_lays_exactly_the_material_of_the_step_it_replaces() {
    for table in tables() {
        let half = 0.45;
        let ramp = table.ramp(half);
        let samples = 200_000;
        let width = 2.0 * half;
        let du = width / f64::from(samples);
        let mut integral = 0.0;
        for index in 0..samples {
            let s = ramp.start() + (f64::from(index) + 0.5) * du;
            integral += ramp.ratio(s) * du;
            let s_end = ramp.start() + f64::from(index + 1) * du;
            if index % 10_000 == 0 {
                assert!(
                    (ramp.extrusion(s_end) - integral).abs() < 1e-9,
                    "running extrusion {} drifts from the integrated ratio {integral}",
                    ramp.extrusion(s_end)
                );
            }
        }
        assert!(integral.abs() < 1e-10, "the ramp lays {integral} extra");
        assert!(
            ramp.extrusion(ramp.end() - 1e-12).abs() < 1e-15,
            "the closed-form extrusion closes on zero"
        );
        for outside in [
            ramp.start() - 1.0,
            ramp.start(),
            ramp.end(),
            ramp.end() + 3.0,
        ] {
            assert_eq!(ramp.extrusion(outside), 0.0);
            assert_eq!(ramp.ratio(outside), 0.0);
            assert_eq!(ramp.slope(outside), 0.0);
            assert_eq!(ramp.curvature(outside), 0.0);
        }
    }
}

#[test]
fn a_slope_step_never_leaves_the_range_the_table_spans() {
    for table in [
        Table {
            before: (0.0, -0.2),
            after: (0.0, 0.0),
        },
        Table {
            before: (0.0, 0.0),
            after: (0.0, 0.2),
        },
        Table {
            before: (0.0, -0.2),
            after: (0.0, 0.1),
        },
        Table {
            before: (0.05, 0.2),
            after: (0.05, 0.0),
        },
        Table {
            before: (0.05, 0.0),
            after: (0.05, -0.2),
        },
    ] {
        for half in [1e-3, 0.1, 0.24] {
            let ramp = table.ramp(half);
            let edges = [
                table.ratio(ramp.start()),
                table.ratio(ramp.end()),
                table.ratio(SEAM),
            ];
            let lowest = edges.into_iter().fold(f64::INFINITY, f64::min);
            let highest = edges.into_iter().fold(f64::NEG_INFINITY, f64::max);
            for index in 0..=10_000 {
                let s = ramp.start() + f64::from(index) / 10_000.0 * 2.0 * half;
                let (ratio, _) = corrected(&table, &ramp, s);
                assert!(
                    ratio >= lowest - 1e-15 && ratio <= highest + 1e-15,
                    "ratio {ratio} at {s} leaves [{lowest}, {highest}]"
                );
            }
        }
    }
}

#[test]
fn a_ramp_stays_within_its_bounds_and_its_curvature_is_its_slope_derivative() {
    for table in tables() {
        for half in [0.01, 0.5, 4.0] {
            let ramp = table.ramp(half);
            let ds = 1e-7 * half;
            for index in 1..1000 {
                let s = ramp.start() + f64::from(index) / 1000.0 * 2.0 * half;
                assert!(ramp.ratio(s).abs() <= ramp.ratio_bound());
                assert!(ramp.slope(s).abs() <= ramp.slope_bound());
                if (s - SEAM).abs() < 2.0 * ds {
                    continue;
                }
                let numeric = (ramp.slope(s + ds) - ramp.slope(s - ds)) / (2.0 * ds);
                let scale = ramp.slope_bound() / half;
                assert!(
                    (numeric - ramp.curvature(s)).abs() <= 1e-5 * scale,
                    "curvature {} vs differentiated slope {numeric} at {s}",
                    ramp.curvature(s)
                );
                let numeric_slope = (ramp.ratio(s + ds) - ramp.ratio(s - ds)) / (2.0 * ds);
                assert!(
                    (numeric_slope - ramp.slope(s)).abs() <= 1e-5 * ramp.slope_bound(),
                    "slope {} vs differentiated ratio {numeric_slope} at {s}",
                    ramp.slope(s)
                );
            }
        }
    }
}
