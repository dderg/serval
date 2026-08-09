use gcode::{ParseError, lex};

#[test]
fn rejects_nan_and_inf_spellings_in_any_param() {
    let sources = [
        "G5 XNaN Y0 I0 J3 P0 Q-3 F1000\n",
        "G5 X1 Ynan I0 J3 P0 Q-3 F1000\n",
        "G5 X1 Y-NaN I0 J3 P0 Q-3 F1000\n",
        "G5 X1 Y0 Iinf J3 P0 Q-3 F1000\n",
        "G5 X1 Y0 I+inf J3 P0 Q-3 F1000\n",
        "G5 X1 Y0 I3 J-inf P0 Q-3 F1000\n",
        "G5 X1 Y0 Iinfinity J3 P0 Q-3 F1000\n",
        "G5 X1 Y0 I0 J3 P0 Q-3 Finf\n",
        "G5 X1 Y0 I0 J3 P0 Q-3 ENaN F1000\n",
    ];
    for src in sources {
        let results: Vec<_> = lex(src).collect();
        assert!(
            results
                .iter()
                .any(|r| matches!(r, Err(ParseError::MalformedNumber { .. }))),
            "expected MalformedNumber for {src:?}, got {results:#?}"
        );
    }
}

#[test]
fn accepts_finite_floats() {
    let results: Vec<_> = lex("G5 X1.5 Y-2.7 I0 J3 P0 Q-3 F1000\n").collect();
    assert!(
        results.iter().all(Result::is_ok),
        "expected all-Ok for finite input, got {results:#?}"
    );
}
