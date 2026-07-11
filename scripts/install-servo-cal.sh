#!/bin/sh
# Install (or update) the servo-cal dashboard service on a printer host:
#   scripts/install-servo-cal.sh <user@host>
#
# Copies config/servo-cal/{servo-cal.service,servo-cal-launcher.sh} to
# ~/servo-cal/ on the host, rewrites the unit's User=/paths for the remote
# user, and enables the unit via the symlink form (printer sudoers only
# passwordless-allow systemctl). Re-running updates the files and restarts
# the service. Does NOT touch moonraker.conf — add
#   [authorization]
#   cors_domains: http://<host>:8085
# yourself and restart moonraker once.
set -eu

if [ $# -ne 1 ]; then
    echo "usage: $0 <user@host>" >&2
    exit 1
fi
TARGET=$1
REMOTE_USER=${TARGET%@*}
if [ "$REMOTE_USER" = "$TARGET" ]; then
    echo "$0: pass user@host (the unit needs the remote username)" >&2
    exit 1
fi

HERE=$(cd "$(dirname "$0")/.." && pwd)
SRC="$HERE/config/servo-cal"

ssh "$TARGET" "mkdir -p ~/servo-cal"
scp "$SRC/servo-cal-launcher.sh" "$TARGET:servo-cal/servo-cal-launcher.sh"
sed -e "s|^User=pi$|User=$REMOTE_USER|" \
    -e "s|/home/pi/|/home/$REMOTE_USER/|g" \
    "$SRC/servo-cal.service" | ssh "$TARGET" "cat > ~/servo-cal/servo-cal.service"
# shellcheck disable=SC2029 # $REMOTE_USER expands client-side by design
ssh "$TARGET" "chmod +x ~/servo-cal/servo-cal-launcher.sh && \
    sudo systemctl enable /home/$REMOTE_USER/servo-cal/servo-cal.service && \
    sudo systemctl restart servo-cal && \
    sleep 2 && systemctl status servo-cal --no-pager -n 5"

echo
echo "dashboard: http://${TARGET#*@}:8085 (once the first build finishes)"
echo "reminder: moonraker.conf needs cors_domains: http://${TARGET#*@}:8085"
