#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-$(gh repo view --json nameWithOwner -q .nameWithOwner)}"
BRANCH="${2:-main}"

REQUIRED_CHECKS=(
  # ci-rust-runtime.yaml
  changes
  rust-test
  rust-clippy
  rust-fmt
  rust-loom
  rust-mcu-h7
  rust-mcu-f4
  rust-mcu-g0
  rust-mcu-f1
  rust-no-stepper
  rust-cbindgen-drift
  rust-c-smoke
  rust-ethercat-hw
  rust-deny
  rust-miri
  rust-panic-symbol-grep
  watchdog-canary
  # ci-build_test.yaml
  build
  # ci-lintformat.yaml
  ruff
  # ci-snapshot.yaml
  snapshot
)

contexts_json="$(printf '%s\n' "${REQUIRED_CHECKS[@]}" | jq -R . | jq -s .)"

echo "Applying branch protection to ${REPO}@${BRANCH} with required checks:"
printf '  - %s\n' "${REQUIRED_CHECKS[@]}"

gh api -X PUT "repos/${REPO}/branches/${BRANCH}/protection" \
  --input - <<JSON
{
  "required_status_checks": {
    "strict": true,
    "contexts": ${contexts_json}
  },
  "enforce_admins": false,
  "required_pull_request_reviews": null,
  "restrictions": null,
  "allow_force_pushes": false,
  "allow_deletions": false
}
JSON

echo "Done. Required checks must pass (or be skipped) before merge into ${BRANCH}."
