use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use runtime::phase_lut::{COIL_AMPLITUDE, PHASE_LUT, PHASE_LUT_SIZE};
use runtime::sample_run::{
    SAMPLE_RUN_COUNT_MAX, SAMPLE_RUN_DATA_MAX, decode_deltas, encode_deltas,
};

fn valid_run() -> impl Strategy<Value = (i32, Vec<i32>)> {
    (
        -1_000_000_i32..=1_000_000,
        prop::collection::vec(-16_000_i32..=16_000, 0..=SAMPLE_RUN_COUNT_MAX / 3),
    )
        .prop_map(|(base, deltas)| {
            let mut position = base;
            let samples = deltas
                .into_iter()
                .map(|delta| {
                    position += delta;
                    position
                })
                .collect();
            (base, samples)
        })
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/wire_roundtrip.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn sample_delta_encoding_round_trips((base, samples) in valid_run()) {
        let mut wire = [0_u8; SAMPLE_RUN_DATA_MAX];
        let written = encode_deltas(base, &samples, &mut wire).expect("valid run fits wire cap");
        let mut decoded = vec![0_i32; samples.len()];
        decode_deltas(base, &wire[..written], samples.len(), &mut decoded).expect("encoded bytes decode");
        prop_assert_eq!(decoded, samples);
    }

    #[test]
    fn arbitrary_sample_payloads_never_panic(
        base in any::<i32>(),
        bytes in prop::collection::vec(any::<u8>(), 0..=SAMPLE_RUN_DATA_MAX + 8),
        count in 0_usize..=SAMPLE_RUN_COUNT_MAX + 8,
        output_len in 0_usize..=SAMPLE_RUN_COUNT_MAX + 8,
    ) {
        let mut output = vec![0_i32; output_len];
        let _ = decode_deltas(base, &bytes, count, &mut output);
    }

    #[test]
    fn phase_table_matches_integer_sine_and_cosine(index in 0_usize..PHASE_LUT_SIZE) {
        let angle = 2.0 * core::f64::consts::PI * index as f64 / PHASE_LUT_SIZE as f64;
        let expected_cos = (f64::from(COIL_AMPLITUDE) * libm::cos(angle)).round() as i16;
        let expected_sin = (f64::from(COIL_AMPLITUDE) * libm::sin(angle)).round() as i16;
        let (actual_cos, actual_sin) = PHASE_LUT[index];
        prop_assert!((actual_cos - expected_cos).abs() <= 1);
        prop_assert!((actual_sin - expected_sin).abs() <= 1);
    }

    #[test]
    fn phase_table_is_periodic_and_quadrant_monotone(
        index in 0_usize..PHASE_LUT_SIZE,
        periods in 0_usize..=16,
    ) {
        let wrapped = (index + periods * PHASE_LUT_SIZE) % PHASE_LUT_SIZE;
        prop_assert_eq!(PHASE_LUT[wrapped], PHASE_LUT[index]);

        let next = (index + 1) % PHASE_LUT_SIZE;
        let ((cos, sin), (next_cos, next_sin)) = (PHASE_LUT[index], PHASE_LUT[next]);
        match index / (PHASE_LUT_SIZE / 4) {
            0 => {
                prop_assert!(next_cos <= cos);
                prop_assert!(next_sin >= sin);
            }
            1 => {
                prop_assert!(next_cos <= cos);
                prop_assert!(next_sin <= sin);
            }
            2 => {
                prop_assert!(next_cos >= cos);
                prop_assert!(next_sin <= sin);
            }
            3 if next != 0 => {
                prop_assert!(next_cos >= cos);
                prop_assert!(next_sin >= sin);
            }
            _ => {}
        }
    }
}
