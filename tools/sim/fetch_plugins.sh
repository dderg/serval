#!/usr/bin/env bash
# Fetch third-party klippy plugins at pinned revs for the simulator.
#
# Only the beacon fork is needed: this tree's motion rewrite is supported
# by dderg/beacon_klipper branch `motion-stack-rename` (kalico-seam plus
# the bridge->engine rename; see docs/rewrite/beacon-fork-survey.md),
# NOT upstream beacon3d.
#
# To bump a pin: edit the rev below and re-run. Re-running is a no-op
# once the pinned rev is checked out.

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
DEST="${1:-$REPO_ROOT/tools/sim/third_party_repos}"

# url | dir name | pinned rev
PLUGINS=(
  "https://github.com/dderg/beacon_klipper.git|beacon_klipper|6dd54bd86432c2b99d2b7bfe97d5ab1dafed98c5"
)

mkdir -p "$DEST"

for entry in "${PLUGINS[@]}"; do
  IFS='|' read -r url name rev <<< "$entry"
  dir="$DEST/$name"

  if [ -d "$dir/.git" ]; then
    cur="$(git -C "$dir" rev-parse HEAD)"
    if [ "$cur" = "$rev" ]; then
      echo "OK    $name @ ${rev:0:7}"
      continue
    fi
    echo "RESET $name (${cur:0:7} -> ${rev:0:7})"
    git -C "$dir" fetch --quiet origin "$rev" 2>/dev/null \
      || git -C "$dir" fetch --quiet --tags
    git -C "$dir" checkout --quiet --detach "$rev"
  else
    if [ -e "$dir" ]; then
      echo "WIPE  $name (non-git leftover)"
      rm -rf "$dir"
    fi
    echo "CLONE $name @ ${rev:0:7}"
    git clone --quiet "$url" "$dir"
    git -C "$dir" checkout --quiet --detach "$rev"
  fi
done

echo "done: $DEST"
