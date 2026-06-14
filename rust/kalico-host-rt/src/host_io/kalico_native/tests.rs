use super::*;
use kalico_protocol::{Encode, StatusHeartbeat};

fn make_state() -> KalicoNativeState {
    KalicoNativeState::default()
}

#[test]
fn status_heartbeat_lifts_to_runtime_event() {
    let hb = StatusHeartbeat {
        engine_state: 1,
        fault_code: 0,
        retired_counts: vec![7, 0, 3],
        ff_saturation_count: 0,
        correction_retired_counts: vec![],
    };
    let mut body = Vec::new();
    hb.encode(&mut body);
    let mut st = make_state();
    match lift_event_to_runtime_event(&mut st, MessageKind::StatusHeartbeat, &body) {
        KalicoDispatchResult::Event(RuntimeEvent::Heartbeat {
            retired_counts,
            correction_retired_counts,
        }) => {
            assert_eq!(retired_counts, vec![7, 0, 3]);
            assert_eq!(correction_retired_counts, Vec::<u32>::new());
        }
        other => panic!("expected Heartbeat event, got {other:?}"),
    }
}
