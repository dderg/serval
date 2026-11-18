//! The print-like fuzz draws of #897: crawl valleys whose ratio steps,
//! each owned with the next one inside its side, held the advance at zero
//! over ever more path, until the flow of the steps capped to the crawl's
//! window passed the extruder's limit. Each a corexy with X/Y smooth
//! shapers and a kernel-free linear advance, as the planner fuzz draws it.

use std::path::PathBuf;

pub struct Draw {
    pub name: &'static str,
    pub velocity: f64,
    pub accel: f64,
    pub extruder_accel: f64,
    pub advance: f64,
    pub shaper_hz: (f64, f64),
    pub gcode: &'static str,
}

impl Draw {
    /// The draw's printer, written where the planner reads it.
    pub fn config(&self) -> PathBuf {
        let text = format!(
            "[printer]\nmax_velocity: {}\nmax_accel: {}\nsquare_corner_velocity: 0\n\
             max_z_velocity: 25\nmax_z_accel: 100\n\
             [kinematics]\ntype: corexy\naxis_x: x\naxis_y: y\naxis_z: z\n\
             a_motors: a\nb_motors: b\nz_motors: z\n\
             [axis x]\npost_processors: shaper_x\n[axis y]\npost_processors: shaper_y\n\
             [axis z]\n[axis e]\nfollows: x, y, z\nmotors: extruder\npost_processors: pa\n\
             max_accel: {}\n\
             [post_processor pa]\ntype: linear_pressure_advance\nk: {}\n\
             [post_processor shaper_x]\ntype: smooth_mzv\nfrequency_hz: {}\n\
             [post_processor shaper_y]\ntype: smooth_zv\nfrequency_hz: {}\n\
             [motor a]\ndrive: stepper\n[motor b]\ndrive: stepper\n\
             [motor z]\ndrive: stepper\n[motor extruder]\ndrive: stepper\n\
             [extruder]\naxis: e\nmax_extrude_only_velocity: 50\n\
             max_extrude_only_accel: 5000\n",
            self.velocity,
            self.accel,
            self.extruder_accel,
            self.advance,
            self.shaper_hz.0,
            self.shaper_hz.1
        );
        let config = std::env::temp_dir().join(format!("{}_{}.cfg", self.name, std::process::id()));
        std::fs::write(&config, text).expect("temp config");
        config
    }
}

/// A reversal's crawl valley owned the ratio steps 0.9 and 2.5 mm after
/// it, where the leaders may cross at ~206 mm/s; capped to the crawl's
/// 0.24 mm step window their flow asked 6734 mm/s² of 6359.
pub const STEPS_FAR_PAST_A_REVERSAL: Draw = Draw {
    name: "steps_far_past_a_reversal",
    velocity: 386.464_809_702_620_87,
    accel: 51_666.607_271_297_22,
    extruder_accel: 6_358.545_634_714_979,
    advance: 0.025_743_938_421_182_125,
    shaper_hz: (105.768_579_750_565_04, 80.640_952_705_959),
    gcode: "G90
M83
G1 X148.223 Y158.619 F10632
G1 X150.159 Y151.491 E0.41688 F13620
G1 X150.794 Y146.569 E0.23578 F11562
G1 X155.997 Y145.112 E0.14745 F7538
G1 X155.077 Y145.258 E0.02331 F4309
G1 X153.500 Y145.541 E0.06676 F6878
G1 X145.159 Y142.668 F14889
",
};

/// A crawl at s=35.2 owned every ratio step of the stream, its zero
/// stretch [-58.8, 242.1] mm: the advance released over 300 mm.
pub const CRAWL_OWNING_THE_STREAM: Draw = Draw {
    name: "crawl_owning_the_stream",
    velocity: 908.862_933_821_992_7,
    accel: 28_901.750_307_894_705,
    extruder_accel: 11_472.974_600_592_206,
    advance: 0.021_265_714_318_443_735,
    shaper_hz: (64.274_465_664_686_35, 107.043_445_931_837_61),
    gcode: "G90
M83
G1 X150.0 Y150.0 Z0.2 F30000
G1 X150.317 Y150.061 E0.01199 F17177
G1 X150.341 Y150.112 E0.00154 F8244
G1 X133.255 Y143.541 E0.54937 F14150
G1 X121.207 Y146.203 E0.19994 F2629
G1 X117.143 Y148.252 E0.09724 F10118
G1 X120.016 Y146.908 F9410
G1 X123.046 Y147.038 E0.09821 F14565
G1 X123.813 Y146.771 E0.01420 F13879
G1 X125.799 Y144.365 E0.11057 F6858
G1 X127.455 Y138.496 F10623
G1 X127.447 Y138.378 E0.00210 F14565
G1 X127.296 Y138.472 E0.00359 F3455
G1 X127.701 Y138.309 E0.01515 F17897
G1 X127.790 Y138.262 F1639
G1 X127.592 Y138.346 E0.00301 F3284
G1 X127.555 Y138.392 E0.00154 F14870
G1 X127.892 Y136.617 E0.03256 F8766
G1 X128.141 Y137.130 F1531
G1 X129.133 Y139.512 E0.06778 F6837
G1 X132.854 Y145.194 E0.14629 F14403
G1 X130.724 Y143.001 E0.06928 F1444
G1 X134.154 Y142.335 E0.13937 F4104
G1 X136.345 Y142.949 E0.03151 F6945
G1 X136.540 Y142.858 E0.00502 F13586
G1 X150.662 Y129.775 E0.45254 F10880
G1 X151.223 Y128.496 E0.04337 F5066
",
};

/// A crawl at s=3.48 among three ratio steps 0.1 mm apart, its zero
/// stretch cascading back to 0.68 mm: their flow asked 95134 mm/s² of
/// 18903.
pub const CRAWL_AMONG_CLOSE_STEPS: Draw = Draw {
    name: "crawl_among_close_steps",
    velocity: 334.457_582_031_306_74,
    accel: 34_908.223_103_440_08,
    extruder_accel: 18_903.042_561_582_588,
    advance: 0.015_620_929_945_596_026,
    shaper_hz: (118.845_945_406_143_88, 109.247_894_231_317_47),
    gcode: "G90
M83
G1 X150.0 Y150.0 Z0.2 F30000
G1 X150.780 Y149.529 E0.04767 F3778
G1 X150.828 Y149.485 F2609
G1 X150.924 Y149.161 E0.01543 F5471
G1 X150.787 Y148.723 F4790
G1 X150.479 Y147.324 E0.08245 F10371
G1 X150.442 Y147.249 F16352
G1 X150.284 Y147.070 F16300
G1 X150.327 Y147.101 E0.00240 F8152
G1 X150.395 Y147.115 F4409
G1 X153.367 Y144.152 E0.23958 F17120
G1 X153.602 Y144.067 E0.00724 F13001
G1 X156.686 Y144.304 E0.14504 F13742
G1 X156.721 Y144.266 E0.00273 F2142
G1 X168.162 Y128.223 E1.04730 F8298
G1 X168.239 Y127.727 F9926
G1 X168.025 Y127.725 E0.00825 F8317
",
};

pub const DRAWS: [Draw; 3] = [
    STEPS_FAR_PAST_A_REVERSAL,
    CRAWL_OWNING_THE_STREAM,
    CRAWL_AMONG_CLOSE_STEPS,
];
