#!/usr/bin/env bash
set -euo pipefail

# ========= User Config =========
FRAMERATE=30
RESOLUTION="1280x720"
CAMS=("/dev/video1" "/dev/video3")  # adjust with v4l2-ctl --list-devices
PORTS=(8081 8083)
INSTALL_DIR="/opt/mjpg-streamer"
BIN="$INSTALL_DIR/mjpg-streamer-experimental/mjpg_streamer"
WWW="$INSTALL_DIR/mjpg-streamer-experimental/www"
WEB_ROOT="/var/www/html/mjpeg"
PLUGIN_DIR="$INSTALL_DIR/mjpg-streamer-experimental"
# =================================

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root: sudo bash $0 [install|start|stop|status]"
  exit 1
fi

ACTION="${1:-install}"

wait_for_apt() {
  echo "[*] Checking for apt/dpkg lock..."
  while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
    echo "    apt/dpkg busy; retrying in 5s..."
    sleep 5
  done
  while pgrep -x apt >/dev/null || pgrep -x apt-get >/dev/null || pgrep -x dpkg >/dev/null; do
    echo "    apt/dpkg process detected; retrying in 5s..."
    sleep 5
  done
}

install_mjpg_streamer() {
  wait_for_apt
  apt-get update
  apt-get install -y git build-essential cmake pkg-config libjpeg-dev imagemagick v4l-utils nginx

  if [[ ! -d "$INSTALL_DIR" ]]; then
    git clone https://github.com/jacksonliam/mjpg-streamer.git "$INSTALL_DIR"
  fi

  make -C "$PLUGIN_DIR"

  INPUT_PLUGIN=$(find "$PLUGIN_DIR" -maxdepth 2 -name input_uvc.so | head -n1 || true)
  OUTPUT_PLUGIN=$(find "$PLUGIN_DIR" -maxdepth 2 -name output_http.so | head -n1 || true)

  if [[ -z "$INPUT_PLUGIN" ]] || [[ -z "$OUTPUT_PLUGIN" ]]; then
    echo "Plugin build failed; could not locate input_uvc/output_http."
    exit 1
  fi

  mkdir -p "$WEB_ROOT"
  cat > "$WEB_ROOT/index.html" <<'HTML'
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Raspberry Pi — Dual Camera MJPEG</title>
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <style>
    body { font-family: system-ui; background:#0b1015; color:#e6eef7; margin:0; padding:1rem; }
    h1 { margin:0 0 1rem; font-size:1.2rem; }
    .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(320px,1fr)); gap:1rem; }
    .card { background:#151b22; border-radius:16px; padding:.75rem; }
    .title { font-weight:600; margin:.5rem 0; }
    img { width:100%; border-radius:12px; background:#000; }
    .hint { font-size:.9rem; opacity:.8; word-break:break-all; }
  </style>
</head>
<body>
  <h1>Raspberry Pi — Dual Camera MJPEG</h1>
  <div class="grid">
    <div class="card"><p class="title">cam_left</p><img id="img0"><div class="hint" id="h0"></div></div>
    <div class="card"><p class="title">cam_right</p><img id="img1"><div class="hint" id="h1"></div></div>
  </div>
  <script>
    const host = location.hostname || 'localhost';
    const ports = [8081, 8083];
    ports.forEach((port, idx) => {
      const url = `http://${host}:${port}/?action=stream`;
      document.getElementById(`img${idx}`).src = url;
      document.getElementById(`h${idx}`).textContent = url;
    });
  </script>
</body>
</html>
HTML

  systemctl enable --now nginx || true

  cat <<EOF
==============================================
Installation complete.
- Start streaming : sudo bash $0 start
- Stop streaming  : Ctrl+C in that terminal
==============================================
EOF
}

start_streaming() {
  if [[ ! -x "$BIN" ]]; then
    echo "mjpg-streamer binary not found at $BIN"
    echo "Run: sudo $0 install"
    exit 1
  fi
  INPUT_PLUGIN=$(find "$PLUGIN_DIR" -maxdepth 2 -name input_uvc.so | head -n1 || true)
  OUTPUT_PLUGIN=$(find "$PLUGIN_DIR" -maxdepth 2 -name output_http.so | head -n1 || true)
  if [[ -z "$INPUT_PLUGIN" ]] || [[ -z "$OUTPUT_PLUGIN" ]]; then
    echo "Required plugins missing. Run install first."
    exit 1
  fi
  declare -a pids=()
  LD_PATH="$(dirname "$INPUT_PLUGIN")"
  LD_PATH+=":$(dirname "$OUTPUT_PLUGIN")"
  LD_PATH+=":$PLUGIN_DIR"

  cleanup() {
    echo
    echo "[cleanup] Stopping mjpg-streamer processes..."
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
      fi
    done
    echo "Streams stopped."
  }

  trap cleanup INT TERM

  mkdir -p "$WEB_ROOT"

  for idx in "${!CAMS[@]}"; do
    DEV="${CAMS[$idx]}"
    PORT="${PORTS[$idx]}"

    CMD=(
      "$BIN"
      -i "input_uvc.so -d ${DEV} -r ${RESOLUTION} -f ${FRAMERATE}"
      -o "output_http.so -p ${PORT} -w ${WWW}"
    )

    echo "[start] ${DEV} on port ${PORT}"
    env LD_LIBRARY_PATH="$LD_PATH" "${CMD[@]}" &
    pids+=($!)
  done

  IP=$(hostname -I | awk '{print $1}')
  echo
  echo "Streams available at:"
  for idx in "${!CAMS[@]}"; do
    echo "  ${CAMS[$idx]} -> http://${IP}:${PORTS[$idx]}/?action=stream"
  done
  echo "Press Ctrl+C to stop."
  echo

  wait "${pids[@]}"
}

case "$ACTION" in
  install) install_mjpg_streamer ;;
  start)   start_streaming ;;
  status)  pgrep -f "$BIN" >/dev/null && echo "mjpg-streamer running." || echo "No mjpg-streamer process running." ;;
  *) echo "Usage: sudo bash $0 [install|start|status]" ;;
esac
