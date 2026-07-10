#!/bin/sh
# servo-cal dashboard launcher. Lives OUTSIDE the repo (~/servo-cal/) because
# the repo checkout is branch-switched by the flash scripts and this file
# must survive a switch to a branch that predates servo-cal.
#
# Behavior, driven by the systemd unit's Restart=always:
#   - checked-out branch has no rust/servo-ident -> idle and re-check
#     (the unit stays green, nothing serves)
#   - branch has it -> cargo build --release -p servo-ident, then serve
#   - HEAD moves (flash script pull / checkout) or the server dies ->
#     exit, systemd relaunches, the new code gets rebuilt and served
#
# Overridable: KLIPPER_DIR, SERVO_CAL_DIR, SERVO_CAL_PORT, SERVO_CAL_HOST.
# The binary's own default bind is 127.0.0.1; the service exists to be
# reached from a browser on the LAN, so the launcher defaults to 0.0.0.0.
set -eu

KLIPPER=${KLIPPER_DIR:-"$HOME/klipper"}
CAPTURES=${SERVO_CAL_DIR:-"$HOME/printer_data/logs/servo_captures"}
PORT=${SERVO_CAL_PORT:-8085}
HOST=${SERVO_CAL_HOST:-0.0.0.0}
RECHECK_IDLE_S=60
RECHECK_HEAD_S=5

head_rev() {
    git -C "$KLIPPER" rev-parse HEAD
}

if [ ! -d "$KLIPPER/rust/servo-ident" ]; then
    branch=$(git -C "$KLIPPER" rev-parse --abbrev-ref HEAD)
    echo "servo-cal: branch '$branch' has no rust/servo-ident; idling ${RECHECK_IDLE_S}s"
    sleep "$RECHECK_IDLE_S"
    exit 0
fi

PATH="$HOME/.cargo/bin:$PATH"
export PATH
start_rev=$(head_rev)
echo "servo-cal: building at $start_rev"
cargo build --release --manifest-path "$KLIPPER/rust/Cargo.toml" -p servo-ident

"$KLIPPER/rust/target/release/servo-cal" serve \
    --dir "$CAPTURES" --port "$PORT" --host "$HOST" &
server=$!
trap 'kill "$server" 2>/dev/null || true' EXIT INT TERM

while kill -0 "$server" 2>/dev/null; do
    if [ "$(head_rev)" != "$start_rev" ]; then
        echo "servo-cal: HEAD moved from $start_rev; exiting so systemd rebuilds"
        exit 0
    fi
    sleep "$RECHECK_HEAD_S"
done
wait "$server"
