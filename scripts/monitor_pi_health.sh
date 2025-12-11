#!/usr/bin/env bash
# Lightweight Pi health monitor: logs throttling/voltage/temperature and filters dmesg for power/I/O resets.
# Usage: ./monitor_pi_health.sh [logfile_or_interval] [interval_seconds]

set -euo pipefail

# Parse args: if first arg is numeric, treat it as interval and skip logging to file.
LOGFILE=""
INTERVAL=10
if [[ $# -ge 1 ]]; then
  if [[ "$1" =~ ^[0-9]+$ ]]; then
    INTERVAL="$1"
  else
    LOGFILE="$1"
    INTERVAL="${2:-10}"
  fi
fi

if [[ -n "$LOGFILE" ]]; then
  touch "$LOGFILE"
  echo "=== pi health monitor started $(date -Iseconds) ===" | tee -a "$LOGFILE"
  sudo dmesg --follow --human | grep -Ei 'voltage|throttle|under-voltage|watchdog|reset|ext4|mmc0|usb disconnect' >>"$LOGFILE" &
else
  echo "=== pi health monitor (stdout only) started $(date -Iseconds) ==="
  sudo dmesg --follow --human | grep -Ei 'voltage|throttle|under-voltage|watchdog|reset|ext4|mmc0|usb disconnect' &
fi
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
  if [[ -n "$LOGFILE" ]]; then
    echo "$ts throttled=$throt temp=$temp volts=$volts" | tee -a "$LOGFILE"
  else
    echo "$ts throttled=$throt temp=$temp volts=$volts"
  fi
  sleep "$INTERVAL"
done
