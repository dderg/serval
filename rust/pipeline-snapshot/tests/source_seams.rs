//! A kernel leader's planned source is fitted before the shaper convolves
//! it. Where a seam of that fit let the velocity step, a support edge of
//! the leader's kernel that has a slope turned the step into a jerk step of
//! the shaped path, and a kernel-free advance into an extruder acceleration
//! step: on this layer end of `voron_dense_top_layers_zhop.gcode` (lines
//! 1207-1283) under the bowden test config (k = 0.5), 0.76 mm/s at an arc's
//! brake seam made a 122 000 mm/s³ jerk step and took e to 1.6x its cap.

mod corpus;

const BOWDEN: &str = "rust/motion-core/tests/configs/p1_bowden_ramp.cfg";

const LAYER_END: &str = "G90
M83
G1 X120.133 Y150.489 Z41.8 F60000
G1 E-.3 F3600
G1 Z42.2
G1 X120.148 Y149.279 F60000
G1 Z41.8
G1 E.3 F2400
G1 F25953.656
G1 X119.87 Y149.531 E.0126
G1 X119.547 Y149.719 E.0126
G1 X119.193 Y149.831 E.01246
G1 X118.941 Y149.862 E.00855
G1 X118.688 Y149.858 E.00852
G1 X118.325 Y149.785 E.01245
G1 X117.982 Y149.633 E.0126
G1 X117.683 Y149.414 E.01246
G1 X117.506 Y149.227 E.00868
G1 X117.303 Y148.918 E.01244
G1 X117.172 Y148.567 E.0126
G1 X117.118 Y148.197 E.01258
G1 X117.144 Y147.823 E.01259
G1 X117.252 Y147.464 E.01261
G1 X117.431 Y147.141 E.01244
G1 X117.594 Y146.94 E.00869
G1 X117.873 Y146.703 E.01232
G1 X118.097 Y146.574 E.0087
G1 X118.444 Y146.451 E.01238
G1 X118.763 Y146.406 E.01084
G1 X119.068 Y146.417 E.01027
G1 X119.427 Y146.502 E.01241
G1 X119.663 Y146.607 E.0087
G1 X119.969 Y146.815 E.01242
G1 X120.226 Y147.086 E.01258
G1 X120.417 Y147.403 E.01245
G1 X120.508 Y147.645 E.0087
G1 X120.574 Y148.005 E.01231
G1 X120.574 Y148.263 E.00866
G1 X120.508 Y148.627 E.01246
G1 X120.362 Y148.972 E.01259
G1 X120.171 Y149.246 E.01126
G1 X120.546 Y149.524 F60000
G1 F20178.796
G1 X120.766 Y149.227 E.01599
G1 X120.949 Y148.808 E.01977
G1 X121.042 Y148.358 E.01987
G1 X121.049 Y147.987 E.01609
G1 X120.972 Y147.541 E.01954
G1 X120.831 Y147.176 E.01695
G1 X120.587 Y146.775 E.02031
G1 X120.274 Y146.449 E.01957
G1 X119.906 Y146.197 E.01926
G1 X119.555 Y146.046 E.01656
G1 X119.106 Y145.943 E.01995
G1 X118.715 Y145.932 E.01689
G1 X118.304 Y145.996 E.01801
G1 X117.877 Y146.153 E.0197
G1 X117.58 Y146.329 E.01492
G1 X117.238 Y146.626 E.01959
G1 X117.016 Y146.905 E.01546
G1 X116.803 Y147.304 E.01956
G1 X116.683 Y147.71 E.01829
G1 X116.644 Y148.186 E.02067
G1 X116.705 Y148.658 E.02059
G1 X116.867 Y149.106 E.02062
G1 X117.122 Y149.501 E.02033
G1 X117.352 Y149.755 E.01483
G1 X117.747 Y150.048 E.02127
G1 X118.161 Y150.232 E.0196
G1 X118.606 Y150.327 E.01971
G1 X118.947 Y150.337 E.01477
G1 X119.318 Y150.286 E.01618
G1 X119.765 Y150.141 E.02033
G1 X120.127 Y149.93 E.01815
G1 X120.481 Y149.611 E.02058
G1 X120.522 Y149.556 E.00299
G92 E0
G1 E-.3 F3600
G1 Z42.2
G1 X120.094 Y150.518 F60000
";

#[test]
fn a_layer_end_on_a_bowden_keeps_the_extruder_within_its_limits() {
    let config = corpus::repo().join(BOWDEN);
    let snapshot = corpus::plan("layer end", LAYER_END, &config).expect("the layer end plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, LAYER_END, &config)
        .expect("the bowden config limits its extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(
        excesses.is_empty(),
        "{}: {} extruder excesses: {}",
        audit.summary(),
        excesses.len(),
        excesses.join("; ")
    );
}
