#!/usr/bin/env bash
# Quick probe to find which /dev/videoN nodes can actually stream.
# Run on the Raspberry Pi: chmod +x scripts/probe_cameras.sh && ./scripts/probe_cameras.sh

set -euo pipefail

command -v v4l2-ctl >/dev/null 2>&1 || {
  echo "Missing dependency: v4l2-ctl (install v4l-utils)" >&2
  exit 1
}

have_ffmpeg=1
command -v ffmpeg >/dev/null 2>&1 || have_ffmpeg=0

tmpdir=$(mktemp -d /tmp/camprobe.XXXXXX)
trap 'rm -rf "$tmpdir"' EXIT

printf "Probing video nodes...\n\n"

for dev in /dev/video*; do
  [ -e "$dev" ] || continue
  printf "=== %s ===\n" "$dev"

  # List formats; this also tells us whether the node supports capture.
  if ! v4l2_out=$(v4l2-ctl -d "$dev" --list-formats-ext 2>&1); then
    printf "  v4l2-ctl failed: %s\n\n" "$v4l2_out"
    continue
  fi

  if grep -q "Video Capture" <<<"$v4l2_out"; then
    printf "  Supports Video Capture\n"
  else
    printf "  No Video Capture interface (likely metadata/control node)\n\n"
    continue
  fi

  printf "  Formats:\n"
  printf "%s\n" "$v4l2_out" | sed 's/^/    /'

  # Optional: attempt a single frame grab via ffmpeg to verify streaming.
  if [ "$have_ffmpeg" -eq 1 ]; then
    outfile="$tmpdir/$(basename "$dev").jpg"
    if timeout 6s ffmpeg -loglevel error -y -f v4l2 -i "$dev" -frames:v 1 "$outfile"; then
      printf "  ffmpeg test: OK (saved %s)\n" "$outfile"
    else
      printf "  ffmpeg test: FAILED (could not grab a frame)\n"
    fi
  else
    printf "  ffmpeg not installed; skipping frame grab test\n"
  fi

  printf "\n"
done

printf "Done. Temporary files were in %s\n" "$tmpdir"
