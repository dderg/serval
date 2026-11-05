import gh_util as gu
import pytest

LIMITED = "GraphQL: API rate limit exceeded for user ID 1 (HTTP 403)"


def test_rate_limit_text_is_recognised_and_other_failures_are_not():
    assert gu.is_rate_limited(LIMITED)
    assert gu.is_rate_limited("HTTP 429: Too Many Requests")
    assert gu.is_rate_limited("You have triggered an abuse detection mechanism")
    assert not gu.is_rate_limited("HTTP 403: Resource not accessible")
    assert not gu.is_rate_limited("no checks reported")


def test_retry_after_is_read_when_present():
    assert gu.retry_after_seconds("HTTP 403\nRetry-After: 42\n") == 42.0
    assert gu.retry_after_seconds(LIMITED) is None


def test_backoff_doubles_is_capped_and_jittered_within_bounds():
    def middle():
        return 0.5

    assert gu.backoff_seconds(0, None, middle) == gu.BASE_BACKOFF_SECONDS
    assert gu.backoff_seconds(1, None, middle) == 2 * gu.BASE_BACKOFF_SECONDS
    assert gu.backoff_seconds(10, None, middle) == gu.MAX_BACKOFF_SECONDS
    low = gu.backoff_seconds(0, None, lambda: 0.0)
    high = gu.backoff_seconds(0, None, lambda: 1.0)
    assert low == gu.BASE_BACKOFF_SECONDS * (1 - gu.JITTER_FRACTION)
    assert high == gu.BASE_BACKOFF_SECONDS * (1 + gu.JITTER_FRACTION)
    assert gu.backoff_seconds(5, 7.0, middle) == 7.0


def scripted(*results):
    calls = iter(results)
    return lambda args: next(calls)


def test_a_rate_limited_call_is_retried_until_it_succeeds():
    sleeps = []
    code = gu.run_gh(
        ["pr", "view"],
        runner=scripted((1, LIMITED), (1, LIMITED), (0, "")),
        sleep=sleeps.append,
    )
    assert code == 0 and len(sleeps) == 2


def test_a_genuine_failure_is_returned_at_once_without_waiting():
    sleeps = []
    code = gu.run_gh(
        ["pr", "checks"],
        runner=scripted((1, "some checks were not successful")),
        sleep=sleeps.append,
    )
    assert code == 1 and sleeps == []


def test_a_pending_exit_code_passes_through():
    assert gu.run_gh(["pr", "checks"], runner=scripted((8, ""))) == 8


def test_persistent_rate_limiting_fails_loudly_after_the_attempt_cap():
    sleeps = []
    with pytest.raises(gu.GhRateLimitError, match="still failing transiently"):
        gu.run_gh(
            ["pr", "view"],
            runner=lambda args: (1, LIMITED),
            sleep=sleeps.append,
        )
    assert len(sleeps) == gu.MAX_ATTEMPTS


def test_network_blips_and_gateway_errors_are_retried_like_rate_limits():
    assert gu.is_rate_limited(
        "error connecting to api.github.com\ncheck your internet connection or https://githubstatus.com"
    )
    assert gu.is_rate_limited("HTTP 502: Bad Gateway")
    assert gu.is_rate_limited("dial tcp: i/o timeout")
    assert not gu.is_rate_limited("HTTP 404: Not Found")


SERVER_ERROR = (
    " ! [remote rejected] topic -> topic (Internal Server Error)\n"
    "error: failed to push some refs to 'https://github.com/o/r.git'"
)
NON_FAST_FORWARD = (
    " ! [rejected] topic -> topic (non-fast-forward)\n"
    "error: failed to push some refs"
)
AUTH = "git@github.com: Permission denied (publickey).\nfatal: Could not read from remote repository."


def test_server_and_network_errors_are_transient_but_auth_and_rejections_are_not():
    assert gu.is_transient_git_failure(SERVER_ERROR)
    assert gu.is_transient_git_failure(
        "fatal: unable to access: The requested URL returned error: 503"
    )
    assert gu.is_transient_git_failure(
        "fatal: Could not resolve host: github.com"
    )
    assert gu.is_transient_git_failure(
        "error: RPC failed; curl 56 Connection reset by peer"
    )
    assert not gu.is_transient_git_failure(NON_FAST_FORWARD)
    assert not gu.is_transient_git_failure(AUTH)
    assert not gu.is_transient_git_failure("remote: Authentication failed")


def test_a_push_that_hits_a_server_error_is_retried_until_it_works():
    sleeps = []
    code = gu.run_git(
        ["push", "-u", "origin", "t"],
        runner=scripted((1, SERVER_ERROR), (1, SERVER_ERROR), (0, "")),
        sleep=sleeps.append,
    )
    assert code == 0 and len(sleeps) == 2


def test_a_non_fast_forward_push_fails_at_once_without_waiting():
    sleeps = []
    code = gu.run_git(
        ["push", "origin", "t"],
        runner=scripted((1, NON_FAST_FORWARD)),
        sleep=sleeps.append,
    )
    assert code == 1 and sleeps == []


def test_an_auth_failure_fails_at_once_without_waiting():
    sleeps = []
    code = gu.run_git(
        ["fetch", "origin"], runner=scripted((128, AUTH)), sleep=sleeps.append
    )
    assert code == 128 and sleeps == []


def test_a_local_subcommand_is_never_retried_even_with_a_matching_message():
    sleeps = []
    code = gu.run_git(
        ["commit", "-m", "x"],
        runner=scripted((1, SERVER_ERROR)),
        sleep=sleeps.append,
    )
    assert code == 1 and sleeps == []


def test_global_options_before_the_subcommand_are_skipped():
    assert (
        gu.git_subcommand(["-C", "/some/dir", "fetch", "-q", "origin"])
        == "fetch"
    )
    assert gu.git_subcommand(["-c", "x=y", "push"]) == "push"
    assert gu.git_subcommand(["--no-pager", "log"]) == "log"
    assert gu.git_subcommand([]) is None


def test_persistent_server_errors_fail_loudly_after_the_cap():
    sleeps = []
    with pytest.raises(
        gu.GhRateLimitError, match="git still failing transiently"
    ):
        gu.run_git(
            ["push", "origin", "t"],
            runner=lambda args: (1, SERVER_ERROR),
            sleep=sleeps.append,
        )
    assert len(sleeps) == gu.GIT_MAX_ATTEMPTS - 1


def test_git_backoff_doubles_and_is_capped():
    middle = lambda: 0.5  # noqa: E731
    assert gu.git_backoff_seconds(0, middle) == gu.GIT_BASE_BACKOFF_SECONDS
    assert gu.git_backoff_seconds(1, middle) == 2 * gu.GIT_BASE_BACKOFF_SECONDS
    assert gu.git_backoff_seconds(20, middle) == gu.GIT_MAX_BACKOFF_SECONDS
