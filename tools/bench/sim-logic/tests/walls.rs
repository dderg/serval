use bench_sim_logic::walls::{Wall, WallError, Walls};

const X_MIN_STEPS: i32 = 100;

fn x_min_wall(pin: u8) -> Wall {
    Wall {
        pin,
        active_high: true,
        at_or_below: true,
        coeffs: [1, 1, 0, 0],
        threshold: 2 * X_MIN_STEPS,
    }
}

#[test]
fn a_corexy_x_wall_reads_the_toolhead_not_one_motor() {
    let mut walls = Walls::default();
    walls.push(x_min_wall(0)).unwrap();
    assert_eq!(walls.levels(&[401, -200, 0, 0]), 0, "x = 100.5 steps");
    assert_eq!(
        walls.levels(&[300, -100, 0, 0]),
        0b1,
        "x = 100 is at the wall"
    );
    assert_eq!(walls.levels(&[150, 150, 0, 0]), 0, "pure y motion: x = 150");
    assert_eq!(
        walls.levels(&[-50, 200, 0, 0]),
        0b1,
        "x = 75 is past the wall"
    );
}

#[test]
fn an_active_low_pin_idles_high_and_drops_when_triggered() {
    let mut walls = Walls::default();
    walls
        .push(Wall {
            pin: 3,
            active_high: false,
            at_or_below: false,
            coeffs: [0, 0, -1, 0],
            threshold: 0,
        })
        .unwrap();
    assert_eq!(walls.idle_levels(), 0b1000);
    assert_eq!(walls.levels(&[0, 0, 5, 0]), 0b1000);
    assert_eq!(walls.levels(&[0, 0, 0, 0]), 0, "z = 0 touches the probe");
    assert_eq!(walls.levels(&[0, 0, -3, 0]), 0);
}

#[test]
fn walls_on_one_pin_are_ored() {
    let mut walls = Walls::default();
    walls.push(x_min_wall(1)).unwrap();
    walls
        .push(Wall {
            at_or_below: false,
            threshold: 2 * 1_000,
            ..x_min_wall(1)
        })
        .unwrap();
    assert_eq!(walls.levels(&[500, 0, 0, 0]), 0);
    assert_eq!(walls.levels(&[0, 0, 0, 0]), 0b10);
    assert_eq!(walls.levels(&[2_000, 0, 0, 0]), 0b10);
}

#[test]
fn unused_pins_stay_low() {
    let walls = Walls::default();
    assert_eq!(walls.levels(&[i32::MAX, i32::MIN, 0, 0]), 0);
}

#[test]
fn malformed_walls_are_refused() {
    let mut walls = Walls::default();
    assert_eq!(walls.push(x_min_wall(4)), Err(WallError::PinOutOfRange(4)));
    walls.push(x_min_wall(0)).unwrap();
    assert_eq!(
        walls.push(Wall {
            active_high: false,
            ..x_min_wall(0)
        }),
        Err(WallError::MixedPolarity(0))
    );
    for _ in 1..bench_sim_logic::walls::MAX_WALLS {
        walls.push(x_min_wall(2)).unwrap();
    }
    assert_eq!(walls.push(x_min_wall(2)), Err(WallError::TooManyWalls));
}

#[test]
fn large_positions_do_not_overflow_the_sum() {
    let mut walls = Walls::default();
    walls
        .push(Wall {
            coeffs: [127, 127, 0, 0],
            threshold: i32::MAX,
            at_or_below: false,
            ..x_min_wall(0)
        })
        .unwrap();
    assert_eq!(walls.levels(&[i32::MAX, i32::MAX, 0, 0]), 1);
}
