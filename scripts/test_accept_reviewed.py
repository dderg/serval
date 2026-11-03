import accept_reviewed as ar
import pytest


def review(*cases, new=()):
    return {
        "base": "b" * 40,
        "head": "h" * 40,
        "expectations": ["cases=*;axes=e"],
        "summary": {"new": list(new), "unchanged": 3, "removed": []},
        "cases": [
            {
                "name": name,
                "verdict": verdict,
                "regressions": ["limit broken"]
                if verdict == "REGRESSION"
                else [],
                "suspicions": ["axis x moved"]
                if verdict == "SUSPICIOUS"
                else [],
            }
            for name, verdict in cases
        ],
    }


def test_expected_and_new_cases_are_accepted():
    accepted, errors = ar.decide(
        review(("a/p/c", "EXPECTED"), new=["n/p/c"]), {}
    )
    assert accepted == ["a/p/c", "n/p/c"]
    assert errors == []


def test_a_regression_is_refused_even_when_listed():
    accepted, errors = ar.decide(
        review(("a/p/c", "REGRESSION")), {"a/p/c": "looks fine"}
    )
    assert any("REGRESSION a/p/c: limit broken" in e for e in errors)


def test_a_suspicious_case_needs_an_explicit_reason():
    _, errors = ar.decide(review(("a/p/c", "SUSPICIOUS")), {})
    assert any("SUSPICIOUS a/p/c is not accepted" in e for e in errors)
    accepted, errors = ar.decide(
        review(("a/p/c", "SUSPICIOUS")),
        {"a/p/c": "x moves 1e-9 with the e fit"},
    )
    assert accepted == ["a/p/c"]
    assert errors == []


def test_listing_a_case_the_review_did_not_flag_is_an_error():
    _, errors = ar.decide(review(("a/p/c", "EXPECTED")), {"typo/p/c": "why"})
    assert any("did not flag: typo/p/c" in e for e in errors)


def test_a_reason_is_required():
    with pytest.raises(ar.AcceptError):
        ar.parse_suspicious(["a/p/c"])
    with pytest.raises(ar.AcceptError):
        ar.parse_suspicious(["a/p/c= "])
    assert ar.parse_suspicious(["a/p/c=because"]) == {"a/p/c": "because"}


def test_the_commit_message_carries_the_verdict_and_reasons():
    message = ar.commit_message(
        7,
        "Fit things",
        review(("a/p/c", "EXPECTED"), ("s/p/c", "SUSPICIOUS")),
        {"s/p/c": "x moves with e"},
    )
    subject, _, body = message.partition("\n\n")
    assert subject == "Accept snapshot baselines for #7"
    assert len(subject) <= ar.SUBJECT_LIMIT
    assert "2 changed cases, 1 expected, 1 suspicious" in body.replace(
        "\n  ", " "
    )
    assert "Suspicious, accepted: s/p/c: x moves with e" in body.replace(
        "\n  ", " "
    )
    assert all(len(line) <= ar.BODY_WIDTH for line in body.splitlines())
