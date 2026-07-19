#!/bin/sh
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
server=
built_rev=

stop_server() {
    if [ -n "$server" ]; then
        kill "$server" 2>/dev/null || true
        wait "$server" 2>/dev/null || true
    fi
}

trap stop_server EXIT INT TERM

while true; do
    target_rev=$(head_rev)
    if [ "$target_rev" != "$built_rev" ]; then
        echo "servo-cal: building at $target_rev"
        cargo build --profile snapshot --manifest-path "$KLIPPER/rust/Cargo.toml" -p servo-ident
        if [ "$(head_rev)" != "$target_rev" ]; then
            continue
        fi
        built_rev=$target_rev
        stop_server
        "$KLIPPER/rust/target/snapshot/servo-cal" serve \
            --dir "$CAPTURES" --port "$PORT" --host "$HOST" &
        server=$!
    fi

    if ! kill -0 "$server" 2>/dev/null; then
        wait "$server"
        exit $?
    fi
    sleep "$RECHECK_HEAD_S"
done
