#!/usr/bin/env bash
# Lightweight Pi health monitor: logs throttling/voltage/temperature and filters dmesg for power/I/O resets.
# Usage: ./monitor_pi_health.sh [logfile] [interval_seconds]

set -euo pipefail

LOGFILE="${1:-$HOME/pi_health.log}"
INTERVAL="${2:-10}"

touch "$LOGFILE"
echo "=== pi health monitor started $(date -Iseconds) ===" | tee -a "$LOGFILE"

# Follow kernel messages for power, throttling, USB resets, and filesystem errors.
sudo dmesg --follow --human | grep -Ei 'voltage|throttle|under-voltage|watchdog|reset|ext4|mmc0|usb disconnect' >>"$LOGFILE" &
DMESG_PID=$!

cleanup() {
  kill "$DMESG_PID" 2>/dev/null || true
}
trap cleanup EXIT

while true; do
  ts=$(date -Iseconds)
  throt=$(vcgencmd get_throttled 2>/dev/null || echo "unknown")
  temp=$(vcgencmd measure_temp 2>/dev/null || echo "temp=unknown")
  volts=$(vcgencmd measure_volts core 2>/dev/null || echo "volt=unknown")
  echo "$ts throttled=$throt temp=$temp volts=$volts" | tee -a "$LOGFILE"
  sleep "$INTERVAL"
done
