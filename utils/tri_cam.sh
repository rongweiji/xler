#!/usr/bin/env bash
set -euo pipefail

# ========= Config =========
FRAMERATE=30
RESOLUTION="1280x720"
CAMS=("/dev/video0" "/dev/video1" "/dev/video3")
NAMES=("cam0" "cam1" "cam3")
PORTS=(8080 8081 8083)
INSTALL_DIR="/opt/mjpg-streamer"
BIN="$INSTALL_DIR/mjpg-streamer-experimental/mjpg_streamer"
WWW="$INSTALL_DIR/mjpg-streamer-experimental/www"
WEB_ROOT="/var/www/html/mjpeg"
# ==========================

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root: sudo bash $0 [start|stop|restart|status|install]"
  exit 1
fi

ACTION="${1:-install}" # default action is install

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

create_services() {
  for i in "${!CAMS[@]}"; do
    NAME="${NAMES[$i]}"
    DEV="${CAMS[$i]}"
    PORT="${PORTS[$i]}"
    cat > "/etc/systemd/system/mjpg-streamer-${NAME}.service" <<SERVICE
[Unit]
Description=mjpg-streamer (${NAME} on ${DEV})
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=$BIN -i "input_uvc.so -d ${DEV} -r ${RESOLUTION} -f ${FRAMERATE}" -o "output_http.so -p ${PORT} -w ${WWW}"
Restart=always
RestartSec=2
User=root
WorkingDirectory=$(dirname "$BIN")

[Install]
WantedBy=multi-user.target
SERVICE
  done
  systemctl daemon-reload
}

install_mjpg_streamer() {
  wait_for_apt
  apt-get update
  apt-get install -y git build-essential cmake pkg-config libjpeg-dev imagemagick v4l-utils nginx

  if [[ ! -d "$INSTALL_DIR" ]]; then
    git clone https://github.com/jacksonliam/mjpg-streamer.git "$INSTALL_DIR"
    make -C "$INSTALL_DIR/mjpg-streamer-experimental"
  fi

  mkdir -p "$WEB_ROOT"
  cat > "$WEB_ROOT/index.html" <<'HTML'
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Raspberry Pi — 3-Camera MJPEG</title>
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
  <h1>Raspberry Pi — 3-Camera MJPEG</h1>
  <div class="grid">
    <div class="card"><p class="title">cam0</p><img id="img0"><div class="hint" id="h0"></div></div>
    <div class="card"><p class="title">cam1</p><img id="img1"><div class="hint" id="h1"></div></div>
    <div class="card"><p class="title">cam3</p><img id="img3"><div class="hint" id="h3"></div></div>
  </div>
  <script>
    const host = location.hostname || 'localhost';
    const ports = { img0:8080, img1:8081, img3:8083 };
    for (const [id,port] of Object.entries(ports)) {
      const url = `http://${host}:${port}/?action=stream`;
      document.getElementById(id).src = url;
      document.getElementById('h'+id.slice(-1)).textContent = url;
    }
  </script>
</body>
</html>
HTML

  create_services

  systemctl enable --now nginx
  for n in "${NAMES[@]}"; do
    systemctl enable --now "mjpg-streamer-${n}.service"
  done

  IP=$(hostname -I | awk '{print $1}')
  echo
  echo "=============================================="
  echo "MJPEG server installed and started!"
  echo "Open: http://${IP}/mjpeg/"
  for i in "${!NAMES[@]}"; do
    echo "  ${NAMES[$i]}: http://${IP}:${PORTS[$i]}/?action=stream"
  done
  echo "=============================================="
}

start_services() {
  for n in "${NAMES[@]}"; do
    systemctl start "mjpg-streamer-${n}.service" || true
  done
  echo "MJPEG streams started."
}

stop_services() {
  for n in "${NAMES[@]}"; do
    systemctl stop "mjpg-streamer-${n}.service" || true
  done
  echo "MJPEG streams stopped."
}

restart_services() {
  for n in "${NAMES[@]}"; do
    systemctl restart "mjpg-streamer-${n}.service" || true
  done
  echo "MJPEG streams restarted."
}

status_services() {
  for n in "${NAMES[@]}"; do
    echo "---- ${n} ----"
    systemctl --no-pager --full status "mjpg-streamer-${n}.service" | grep -E "Active:|ExecStart=" || true
  done
}

case "$ACTION" in
  install) install_mjpg_streamer ;;
  start)   start_services ;;
  stop)    stop_services ;;
  restart) restart_services ;;
  status)  status_services ;;
  *) echo "Usage: sudo bash $0 [install|start|stop|restart|status]" ;;
esac
