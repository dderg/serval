#!/bin/sh
set -eu

KLIPPER=${KLIPPER_DIR:-"$HOME/klipper"}
CAPTURES=${SERVO_CAL_DIR:-"$HOME/printer_data/logs/servo_captures"}
PORT=${SERVO_CAL_PORT:-8085}
HOST=${SERVO_CAL_HOST:-0.0.0.0}
BINARY="$KLIPPER/rust/target/snapshot/servo-cal"

if [ ! -x "$BINARY" ]; then
    echo "servo-cal: deploy must build $BINARY before starting the service" >&2
    exit 1
fi

exec "$BINARY" serve --dir "$CAPTURES" --port "$PORT" --host "$HOST"
