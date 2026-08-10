from serval_bot.auth import sign, verify


def test_signed_request_verifies() -> None:
    timestamp, signature = sign("POST", "/github/comment", b"{}", "secret", timestamp="100")
    result = verify("POST", "/github/comment", b"{}", "secret", timestamp, signature, now=100)
    assert result.valid


def test_body_tampering_is_rejected() -> None:
    timestamp, signature = sign("POST", "/github/comment", b"{}", "secret", timestamp="100")
    result = verify("POST", "/github/comment", b'{"x":1}', "secret", timestamp, signature, now=100)
    assert not result.valid
    assert result.reason == "signature mismatch"


def test_expired_request_is_rejected() -> None:
    timestamp, signature = sign("POST", "/github/comment", b"{}", "secret", timestamp="100")
    result = verify("POST", "/github/comment", b"{}", "secret", timestamp, signature, now=131)
    assert not result.valid
    assert result.reason == "expired timestamp"
