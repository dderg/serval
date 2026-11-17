# Scenario-fuzzer regressions

Saved scenarios of fixed product bugs, one directory per issue
(`issue-N/scenario.json`, `timeline.txt`). A fix PR moves its repro here from
`../repros/` (`git mv`) and drops its row from `../repros/README.md`;
`tools/sim/tests/test_fuzz_regressions.py` replays every directory and
expects a pass.
