#!/usr/bin/env bash
set -euo pipefail

# Slot processes create files inside their own slot-private trees, and root
# creates Git pool control data that must never be group-writable, so the
# default umask is restrictive.
umask 0027

/usr/sbin/groupadd -f -g 2000 omp
id -u bot >/dev/null 2>&1 || /usr/sbin/useradd -u 10001 -g omp -M -N -s /usr/sbin/nologin bot
/usr/sbin/usermod -g omp bot

for i in $(seq 1 8); do
    user="omp-$i"
    /usr/sbin/groupadd -f -g $((2000 + i)) "$user"
    id -u "$user" >/dev/null 2>&1 || /usr/sbin/useradd -u $((2000 + i)) -g "$user" -G omp -M -N -s /usr/sbin/nologin "$user"
    /usr/sbin/usermod -g "$user" -a -G omp "$user"
done

mkdir -p /data/db /data/logs /data/workspaces /data/sessions /data/omp-agent/config /data/omp-agent/auth
chown root:root /data
chmod 0755 /data
chown root:root /data/db /data/logs
chmod 0700 /data/db /data/logs
chown root:root /data/workspaces /data/sessions
chmod 0755 /data/workspaces /data/sessions
chown root:root /data/omp-agent /data/omp-agent/config /data/omp-agent/auth
chmod 0755 /data/omp-agent
chmod 0750 /data/omp-agent/config /data/omp-agent/auth

touch /data/serval-bot.sqlite
chown root:root /data/serval-bot.sqlite
chmod 0600 /data/serval-bot.sqlite
for db_file in /data/serval-bot.sqlite-wal /data/serval-bot.sqlite-shm; do
    if [ -e "$db_file" ]; then
        chown root:root "$db_file"
        chmod 0600 "$db_file"
    fi
done

exec "$@"
