use super::FenceRegistry;

#[test]
fn pending_until_dispatch_passes_the_fence_line() {
    let r = FenceRegistry::default();
    let id = r.alloc_id();
    r.arm(id, 10);
    assert_eq!(r.take(id), None);
    r.on_dispatch(10, 1.5);
    assert_eq!(
        r.take(id),
        None,
        "segments at the fence line must not resolve it"
    );
    r.on_dispatch(11, 2.5);
    assert_eq!(r.take(id), Some(Some(2.5)));
    assert_eq!(r.take(id), None, "take consumes the resolution");
}

#[test]
fn barrier_resolves_all_armed_fences() {
    let r = FenceRegistry::default();
    let a = r.alloc_id();
    let b = r.alloc_id();
    r.arm(a, 5);
    r.arm(b, 9);
    assert!(r.has_armed());
    r.resolve_armed(Some(7.0));
    assert!(!r.has_armed());
    assert_eq!(r.take(a), Some(Some(7.0)));
    assert_eq!(r.take(b), Some(Some(7.0)));
}

#[test]
fn reset_resolution_carries_none() {
    let r = FenceRegistry::default();
    let id = r.alloc_id();
    r.arm(id, 3);
    r.resolve_armed(None);
    assert_eq!(r.take(id), Some(None));
}

#[test]
fn direct_resolution_bypasses_arming() {
    let r = FenceRegistry::default();
    let id = r.alloc_id();
    r.resolve(id, Some(4.0));
    assert_eq!(r.take(id), Some(Some(4.0)));
}

#[test]
fn progress_only_resolves_fences_behind_the_line() {
    let r = FenceRegistry::default();
    let early = r.alloc_id();
    let late = r.alloc_id();
    r.arm(early, 5);
    r.arm(late, 20);
    r.on_dispatch(12, 3.0);
    assert_eq!(r.take(early), Some(Some(3.0)));
    assert_eq!(r.take(late), None);
    assert!(r.has_armed());
}
