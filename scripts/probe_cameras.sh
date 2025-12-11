#!/usr/bin/env bash
# Concise probe for which /dev/video* nodes can actually stream.
# Run on the Pi: chmod +x scripts/probe_cameras.sh && ./scripts/probe_cameras.sh

set -euo pipefail

command -v v4l2-ctl >/dev/null 2>&1 || {
  echo "Missing dependency: v4l2-ctl (install v4l-utils)" >&2
  exit 1
}

have_ffmpeg=1
command -v ffmpeg >/dev/null 2>&1 || have_ffmpeg=0

tmpdir=$(mktemp -d /tmp/camprobe.XXXXXX)
trap 'rm -rf "$tmpdir"' EXIT

echo "Probing /dev/video* (short summary)..."
echo

printf "%-12s %-8s %-20s %-30s\n" "Device" "Capture?" "Formats" "ffmpeg grab"
printf "%-12s %-8s %-20s %-30s\n" "------------" "--------" "--------------------" "------------------------------"

usable=()

for dev in /dev/video*; do
  [ -e "$dev" ] || continue

  if ! v4l2_out=$(v4l2-ctl -d "$dev" --list-formats-ext 2>&1); then
    printf "%-12s %-8s %-20s %-30s\n" "$dev" "no" "-" "v4l2-ctl failed"
    continue
  fi

  if grep -q "Video Capture" <<<"$v4l2_out"; then
    capture="yes"
  else
    printf "%-12s %-8s %-20s %-30s\n" "$dev" "no" "-" "control/metadata node"
    continue
  fi

  # Extract the list of format fourcc codes.
  fmts=$(printf "%s\n" "$v4l2_out" | sed -n "s/^[[:space:]]*\\[[0-9]\\+\\]: '\\([A-Z0-9]\\{3,4\\}\\)'.*/\\1/p" | paste -sd, -)
  [ -z "$fmts" ] && fmts="(none)"

  grab_result="(skip)"
  if [ "$have_ffmpeg" -eq 1 ]; then
    outfile="$tmpdir/$(basename "$dev").jpg"
    if grab_out=$(timeout 6s ffmpeg -loglevel error -y -f v4l2 -input_format mjpeg -framerate 5 -video_size 640x480 -i "$dev" -frames:v 1 "$outfile" 2>&1); then
      grab_result="OK (mjpeg) -> $outfile"
      usable+=("$dev")
    else
      # If MJPEG fails, try a quick YUYV attempt to disambiguate format issues.
      if grab2_out=$(timeout 6s ffmpeg -loglevel error -y -f v4l2 -input_format yuyv422 -framerate 5 -video_size 640x480 -i "$dev" -frames:v 1 "$outfile" 2>&1); then
        grab_result="OK (yuyv422) -> $outfile"
        usable+=("$dev")
      else
        # Take the first line of the last error for brevity.
        err_line=$(printf "%s\n%s\n" "$grab_out" "$grab2_out" | grep -m1 -v '^$' || true)
        grab_result="FAIL: $err_line"
      fi
    fi
  fi

  printf "%-12s %-8s %-20s %-30s\n" "$dev" "$capture" "$fmts" "$grab_result"
done

echo
if [ "${#usable[@]}" -gt 0 ]; then
  printf "Usable capture nodes: %s\n" "${usable[*]}"
else
  echo "No usable capture nodes found (check if devices are busy or permissions)."
fi
echo

if [ -d /dev/v4l/by-id ]; then
  echo "Stable names (symlinks) from /dev/v4l/by-id:"
  ls -l /dev/v4l/by-id || true
  echo
fi

echo "Done. Temporary files were in $tmpdir"
