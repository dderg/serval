use super::result_codes;

#[test]
fn result_codes_are_stable() {
    assert_eq!(result_codes::OK, 0);
    assert_eq!(result_codes::RING_FULL, -309);
    assert_eq!(result_codes::STREAM_HALTED, -142);
    assert_eq!(result_codes::EC_PIECES_WHILE_HALTED, -315);
}

mod length_prefixes {
    use crate::codec::{Encode, put_len_u8, put_len_u32};
    use crate::messages::{CaptureDrive, StartCapture};

    fn capture(drives: usize) -> StartCapture {
        StartCapture {
            path: "p".into(),
            started_utc: "t".into(),
            drives: (0..drives)
                .map(|slot| CaptureDrive {
                    slot: 0,
                    name: format!("d{slot}"),
                })
                .collect(),
        }
    }

    #[test]
    fn the_largest_count_a_u8_prefix_holds_encodes() {
        let mut out = Vec::new();
        put_len_u8(&mut out, 255, "test");
        assert_eq!(out, [255]);
        assert!(!capture(255).encoded_to_vec().is_empty());
    }

    #[test]
    #[should_panic(expected = "StartCapture.drives: 256 elements exceed the u8 count prefix")]
    fn a_message_with_more_elements_than_its_u8_prefix_holds_is_refused() {
        let _ = capture(256).encoded_to_vec();
    }

    #[test]
    fn a_count_inside_the_u32_prefix_encodes() {
        let mut out = Vec::new();
        put_len_u32(&mut out, 7, "test");
        assert_eq!(out, [7, 0, 0, 0]);
    }
}
