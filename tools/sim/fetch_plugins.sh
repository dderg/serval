#!/usr/bin/env bash
# Fetch third-party klippy plugins at pinned revs for the simulator.
#
# The kalico-seam forks are required: this tree's motion rewrite is
# supported by dderg/beacon_klipper (branch `motion-stack-rename`
# lineage; see docs/rewrite/beacon-fork-survey.md) and
# dderg/cartographer-klipper (branch `kalico-seam`), NOT the upstream
# beacon3d / Cartographer3D repos.
#
# To bump a pin: edit the rev below and re-run. Re-running is a no-op
# once the pinned rev is checked out.

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
DEST="${1:-$REPO_ROOT/tools/sim/third_party_repos}"

# url | dir name | pinned rev
PLUGINS=(
  "https://github.com/dderg/beacon_klipper.git|beacon_klipper|563861d211a21b62eedf80906c8a55f70b0174d6"
  "https://github.com/dderg/cartographer-klipper.git|cartographer_klipper|e069a36dac9ebdc84ae72e27fc10e1a3a6d01015"
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
