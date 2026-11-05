use bench_sim_logic::config::{parse, ConfigError, SimConfig, TAG_HEATER, TAG_WALL};
use bench_sim_logic::thermal::{BED_DEFAULT, EXTRUDER_DEFAULT};
use bench_sim_logic::walls::{Wall, WallError, Walls};

const GOLDEN: &str =
    "07000110e8030000f4ffffff0a00000000000000020a00030101000040060000020a0303000001000000000003\
    21000000a0410000c0410000803f0000803e0000c8415043c74700803b4500401c46040101";

fn hex(text: &str) -> Vec<u8> {
    let text: String = text.split_whitespace().collect();
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&text[i..i + 2], 16).unwrap())
        .collect()
}

#[test]
fn the_golden_payload_from_the_python_encoder_decodes() {
    let config = parse(&hex(GOLDEN)).unwrap();
    assert_eq!(config.id, 7);
    assert_eq!(config.origin, [1_000, -12, 10, 0]);
    let mut walls = Walls::default();
    walls
        .push(Wall {
            pin: 0,
            active_high: true,
            at_or_below: true,
            coeffs: [1, 1, 0, 0],
            threshold: 1_600,
        })
        .unwrap();
    walls
        .push(Wall {
            pin: 3,
            active_high: true,
            at_or_below: true,
            coeffs: [0, 0, 1, 0],
            threshold: 0,
        })
        .unwrap();
    assert_eq!(config.walls, walls);
    let extruder = config.heaters[0];
    assert_eq!(extruder.power_w, 20.0);
    assert_eq!(extruder.capacity_j_per_k, 24.0);
    assert_eq!(extruder.loss_w_per_k, 1.0);
    assert_eq!(extruder.fan_loss_w_per_k, 0.25);
    assert_eq!(extruder.ambient_c, 25.0);
    assert_eq!(extruder.r25_ohm, 102_022.625);
    assert_eq!(extruder.beta, 3000.0);
    assert_eq!(extruder.pullup_ohm, 10_000.0);
    assert_eq!(config.heaters[1], BED_DEFAULT);
    assert!(config.expect_mco);
}

#[test]
fn an_empty_config_is_the_boot_default_with_its_id() {
    let config = parse(&[9, 0]).unwrap();
    assert_eq!(
        config,
        SimConfig {
            id: 9,
            ..SimConfig::default()
        }
    );
    assert_eq!(config.heaters[0], EXTRUDER_DEFAULT);
}

#[test]
fn malformed_payloads_are_refused() {
    assert_eq!(parse(&[1]), Err(ConfigError::Truncated));
    assert_eq!(parse(&[1, 0, 99, 0]), Err(ConfigError::UnknownTag(99)));
    assert_eq!(
        parse(&[1, 0, TAG_WALL, 3, 0, 0, 0]),
        Err(ConfigError::BadLength {
            tag: TAG_WALL,
            len: 3
        })
    );
    assert_eq!(
        parse(&[1, 0, TAG_WALL, 10, 0, 0]),
        Err(ConfigError::Truncated)
    );
    let mut wall = vec![1, 0, TAG_WALL, 10, 7, 1, 1, 0, 0, 0, 0, 0, 0, 0];
    assert_eq!(
        parse(&wall),
        Err(ConfigError::Wall(WallError::PinOutOfRange(7)))
    );
    wall[4] = 0;
    wall[5] = 0x80;
    assert_eq!(parse(&wall), Err(ConfigError::BadFlag(0x80)));
    let mut heater = vec![1, 0, TAG_HEATER, 33, 0];
    heater.extend([0u8; 32]);
    assert_eq!(parse(&heater), Err(ConfigError::HeaterParams(0)));
    heater[4] = 2;
    assert_eq!(parse(&heater), Err(ConfigError::HeaterIndex(2)));
}
