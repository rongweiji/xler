#!/usr/bin/env bash
# run_three_cams.sh — start 3 camera streams + a tiny website to view them
# Usage:
#   chmod +x run_three_cams.sh
#   ./run_three_cams.sh
#
# Then open: http://<PI_IP>:8080

set -euo pipefail

# ---------- CONFIG (change if your device numbers differ) ----------
CSI_DEV="${CSI_DEV:-/dev/video0}"     # CSI ribbon cam (via unicam)
USB1_DEV="${USB1_DEV:-/dev/video1}"   # USB cam 1
USB2_DEV="${USB2_DEV:-/dev/video3}"   # USB cam 2

CSI_PORT="${CSI_PORT:-9000}"
USB1_PORT="${USB1_PORT:-9001}"
USB2_PORT="${USB2_PORT:-9002}"
SITE_PORT="${SITE_PORT:-8080}"

# Common video settings (lower if CPU is high)
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-15}"

WEB_DIR="/opt/tri-cam-web"

# ---------- PRECHECKS ----------
if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (use: sudo $0)"; exit 1
fi

command -v hostname >/dev/null || { echo "hostname not found"; exit 1; }
PI_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [[ -z "${PI_IP:-}" ]]; then
  # Fallback to hostname
  PI_IP="raspberrypi.local"
fi

echo "[*] Using IP for page links: ${PI_IP}"

# ---------- INSTALL DEPENDENCIES ----------
echo "[*] Installing packages (this may take a minute)..."
apt-get update -y
apt-get install -y --no-install-recommends \
  libcamera-apps v4l-utils ustreamer python3

# ---------- CREATE WEB PAGE ----------
mkdir -p "$WEB_DIR"
cat > "${WEB_DIR}/index.html" <<HTML
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Raspberry Pi — 3 Cameras</title>
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <style>
    :root { color-scheme: dark; }
    body{margin:0;background:#0b0d10;color:#e7edf3;font-family:system-ui,Segoe UI,Arial}
    header{padding:14px 16px;font-weight:600}
    .grid{display:grid;gap:12px;padding:12px;grid-template-columns: repeat(auto-fit, minmax(320px,1fr));}
    .cam{background:#14181d;border-radius:12px;padding:10px;box-shadow:0 2px 8px rgba(0,0,0,.3)}
    .cam h2{margin:6px 0 10px;font-size:16px;font-weight:600}
    .cam img{width:100%;height:auto;border-radius:8px;background:#000}
    .meta{opacity:.75;font-size:12px;padding:0 12px 12px}
    a{color:#8ab4ff}
  </style>
</head>
<body>
  <header>Raspberry Pi — Live Cameras</header>
  <div class="grid">
    <div class="cam"><h2>CSI</h2><img src="http://${PI_IP}:${CSI_PORT}/stream" alt="CSI stream"/></div>
    <div class="cam"><h2>USB 1</h2><img src="http://${PI_IP}:${USB1_PORT}/stream" alt="USB1 stream"/></div>
    <div class="cam"><h2>USB 2</h2><img src="http://${PI_IP}:${USB2_PORT}/stream" alt="USB2 stream"/></div>
  </div>
  <div class="meta">
    Streams: <a href="http://${PI_IP}:${CSI_PORT}/stream">CSI</a> ·
    <a href="http://${PI_IP}:${USB1_PORT}/stream">USB1</a> ·
    <a href="http://${PI_IP}:${USB2_PORT}/stream">USB2</a>
  </div>
</body>
</html>
HTML

# ---------- START STREAMS ----------
pids=()

cleanup() {
  echo
  echo "[*] Stopping streams & web server..."
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  # Also terminate any libcamera-vid that was piped
  pkill -f "libcamera-vid --codec mjpeg" 2>/dev/null || true
  echo "[*] Done."
}
trap cleanup EXIT INT TERM

echo "[*] Starting CSI stream on :${CSI_PORT} from ${CSI_DEV} ..."
# For CSI, we pipe libcamera-vid (MJPEG) into ustreamer
# -t 0 = run forever, -n = no preview window, -o - = stdout
bash -lc "libcamera-vid --codec mjpeg --width ${WIDTH} --height ${HEIGHT} --framerate ${FPS} -t 0 -n -o - \
  | ustreamer --input - --format MJPEG --host 0.0.0.0 --port ${CSI_PORT}" &
pids+=($!)

echo "[*] Starting USB1 stream on :${USB1_PORT} from ${USB1_DEV} ..."
# If the cam supports MJPEG natively this will be efficient; otherwise ustreamer will convert.
ustreamer --device "${USB1_DEV}" --host 0.0.0.0 --port "${USB1_PORT}" \
  --resolution "${WIDTH}x${HEIGHT}" --fps "${FPS}" --persist &
pids+=($!)

echo "[*] Starting USB2 stream on :${USB2_PORT} from ${USB2_DEV} ..."
ustreamer --device "${USB2_DEV}" --host 0.0.0.0 --port "${USB2_PORT}" \
  --resolution "${WIDTH}x${HEIGHT}" --fps "${FPS}" --persist &
pids+=($!)

# ---------- START STATIC WEB SERVER ----------
echo "[*] Starting tiny web server on :${SITE_PORT} serving ${WEB_DIR}"
# Use python's simple server (lightweight, no config)
python3 -m http.server "${SITE_PORT}" --directory "${WEB_DIR}" >/dev/null 2>&1 &
pids+=($!)

echo
echo "============================================================"
echo " Open this on your laptop (same network):"
echo "   http://${PI_IP}:${SITE_PORT}"
echo
echo " Streams (direct):"
echo "   CSI  : http://${PI_IP}:${CSI_PORT}/stream"
echo "   USB1 : http://${PI_IP}:${USB1_PORT}/stream"
echo "   USB2 : http://${PI_IP}:${USB2_PORT}/stream"
echo "------------------------------------------------------------"
echo " Press Ctrl+C here to stop everything."
echo "============================================================"

# Keep the script running so trap can clean up on Ctrl+C
while true; do sleep 3600; done
