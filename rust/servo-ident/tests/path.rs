use servo_ident::analyze::{path_indices, xy_path, MAX_PATH_POINTS};
use servo_ident::results::ManifestSpatial;
use servo_ident::scap::Scap;

fn spatial(modes: &[&str], axes: &[&str], frame: &[&[f64]]) -> ManifestSpatial {
    ManifestSpatial {
        modes: modes.iter().map(|s| s.to_string()).collect(),
        axes: axes.iter().map(|s| s.to_string()).collect(),
        frame: frame.iter().map(|r| r.to_vec()).collect(),
    }
}

fn corexy_capture(samples: &[(i32, i32, i32, i32)]) -> Scap {
    let header = "{\"version\":2,\"cycle_ns\":250000,\"record_size\":25,\
         \"drives\":[{\"name\":\"motor_a\",\"counts_per_mm\":1000.0},\
         {\"name\":\"motor_b\",\"counts_per_mm\":500.0}],\
         \"channels\":[{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0},\
         {\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8},\
         {\"name\":\"target_counts\",\"dtype\":\"i32\",\"offset\":9},\
         {\"name\":\"position_actual\",\"dtype\":\"i32\",\"offset\":13}]}";
    let mut b = header.as_bytes().to_vec();
    b.push(b'\n');
    for (r, &(ta, pa, tb, pb)) in samples.iter().enumerate() {
        b.extend_from_slice(&(r as u64).to_le_bytes());
        b.push(2u8);
        b.extend_from_slice(&ta.to_le_bytes());
        b.extend_from_slice(&pa.to_le_bytes());
        b.extend_from_slice(&tb.to_le_bytes());
        b.extend_from_slice(&pb.to_le_bytes());
    }
    Scap::from_bytes(&b).unwrap()
}

const COREXY_FRAME: [&[f64]; 2] = [&[0.5, -0.5], &[0.5, 0.5]];

#[test]
fn maps_counts_to_cartesian_mm_through_the_frame() {
    let cap = corexy_capture(&[(0, 0, 0, 0), (2000, 1000, -500, 500)]);
    let sp = spatial(&["x", "y"], &["motor_a", "motor_b"], &COREXY_FRAME);
    let path = xy_path(&cap, &sp).unwrap().unwrap();
    assert_eq!(path.cmd_x_mm, vec![0.0, 0.5 * 2.0 - 0.5 * -1.0]);
    assert_eq!(path.cmd_y_mm, vec![0.0, 0.5 * 2.0 + 0.5 * -1.0]);
    assert_eq!(path.act_x_mm, vec![0.0, 0.5 * 1.0 - 0.5 * 1.0]);
    assert_eq!(path.act_y_mm, vec![0.0, 0.5 * 1.0 + 0.5 * 1.0]);
}

#[test]
fn omits_path_when_the_frame_lacks_an_xy_mode() {
    let cap = corexy_capture(&[(0, 0, 0, 0)]);
    let sp = spatial(&["x"], &["motor_a", "motor_b"], &[&[0.5, 0.5]]);
    assert!(xy_path(&cap, &sp).unwrap().is_none());
}

#[test]
fn omits_path_when_a_frame_motor_is_not_captured() {
    let cap = corexy_capture(&[(0, 0, 0, 0)]);
    let sp = spatial(&["x", "y"], &["motor_a", "motor_z"], &COREXY_FRAME);
    assert!(xy_path(&cap, &sp).unwrap().is_none());
}

#[test]
fn rejects_a_frame_whose_shape_disagrees_with_its_labels() {
    let cap = corexy_capture(&[(0, 0, 0, 0)]);
    let sp = spatial(&["x", "y"], &["motor_a", "motor_b"], &[&[0.5, 0.5]]);
    let err = xy_path(&cap, &sp).unwrap_err();
    assert!(err.contains("frame shape"), "{err}");
}

#[test]
fn path_indices_keep_the_final_sample_within_budget() {
    assert_eq!(path_indices(0, 10), Vec::<usize>::new());
    assert_eq!(path_indices(3, 10), vec![0, 1, 2]);
    let idxs = path_indices(10_001, 4);
    assert_eq!(*idxs.last().unwrap(), 10_000);
    assert!(idxs.len() <= 5, "{}", idxs.len());
    assert!(idxs.windows(2).all(|w| w[0] < w[1]));
    let full = path_indices(1_000_000, MAX_PATH_POINTS);
    assert!(full.len() <= MAX_PATH_POINTS + 1);
    assert_eq!(*full.last().unwrap(), 999_999);
}
