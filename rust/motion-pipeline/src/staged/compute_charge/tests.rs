use super::*;

const SCALE: f64 = 2.0;

#[test]
fn a_thread_owes_its_cpu_time_since_it_last_paid_scaled() {
    let mut account = ChargeAccount::closed(SCALE);
    account.open_at(Duration::from_millis(100));
    assert_eq!(
        account.owe(Duration::from_millis(400)),
        Duration::from_millis(600)
    );
    assert_eq!(
        account.owe(Duration::from_millis(450)),
        Duration::from_millis(100)
    );
    assert_eq!(account.owe(Duration::from_millis(450)), Duration::ZERO);
}

#[test]
fn reopening_an_account_keeps_what_it_was_paid_through() {
    let mut account = ChargeAccount::closed(SCALE);
    account.open_at(Duration::from_millis(100));
    account.open_at(Duration::from_millis(300));
    assert_eq!(
        account.owe(Duration::from_millis(300)),
        Duration::from_millis(400)
    );
}

#[test]
#[should_panic(expected = "paid on the thread that opened it")]
fn paying_an_account_never_opened_fails_loudly() {
    ChargeAccount::closed(SCALE).owe(Duration::from_millis(1));
}

#[test]
fn a_scale_reads_as_a_positive_finite_number() {
    assert_eq!(parse_scale(" 2.5 "), Ok(2.5));
    for spec in ["", "0", "-1", "inf", "nan", "x"] {
        assert!(parse_scale(spec).is_err(), "{spec:?} must be refused");
    }
}

#[test]
fn this_thread_reads_its_own_cpu_time() {
    let before = thread_cpu_time();
    let spin_until = std::time::Instant::now() + Duration::from_millis(20);
    while std::time::Instant::now() < spin_until {}
    assert!(thread_cpu_time() > before);
}
