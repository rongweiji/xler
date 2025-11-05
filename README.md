# XLER on Raspberry Pi

Quick notes for bringing the omni-base controller online on Raspberry Pi OS (Bookworm or newer).

## Requirements
- Raspberry Pi OS with the desktop or Lite image
- Python 3.11+ (Pi OS ships it already) or a newer custom build
- Feetech USB-to-serial adapter connected to your motors

## Setup
```bash
sudo apt update
sudo apt install python3-venv python3-dev libatlas-base-dev
cd ~/xler                  # adjust if you cloned elsewhere
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

> Tip: when you open a fresh shell later, re-activate the environment with `source .venv/bin/activate`, or run scripts directly with `./.venv/bin/python`.

## Run the Base Controller
```bash
python xler.py
```

- Use `--port /dev/ttyUSB0` if auto-detection fails.
- Add `--show-config` to print the loaded YAML and exit.
- Optional overrides: `--speed`, `--motor-left`, `--motor-right`, `--motor-back`.
- Terminal-only telemetry shows commanded vs observed motion whenever the robot moves.

## Stereo Camera Recording (Optional)
- Set up devices in `xler.yaml` under the `camera` section (e.g. `/dev/video1` and `/dev/video3`) and define the output folders you want to use.
- Enable recording via CLI with:
  ```bash
  python xler.py --record-cameras
  ```
  or flip `camera.enabled` to `true` in `xler.yaml`.
- Adjust capture cadence with `--camera-frame-interval` (default 20 control loops per frame) and override device/output paths via `--camera-left`, `--camera-right`, or `--camera-output-dir`.
- Control resolution/FPS/pixel format via `camera.resolution`, `camera.fps`, and `camera.pixel_format` in `xler.yaml` (defaults: `1280x720`, `30`, `mjpeg`).
- Frames are saved as JPEGs with EXIF timestamps in `recordings/front_stereo_cam_left` and `recordings/front_stereo_cam_right`.

## Camera Helper Script (Optional)
To stream three USB cameras via MJPEG:
```bash
cd utils
chmod +x tri_cam.sh
sudo ./tri_cam.sh install     # first-time setup
sudo ./tri_cam.sh start       # start all streams
```
The default streams appear at `http://<pi-ip>/mjpeg/` and on ports 8080/8081/8083.

## Upgrading Dependencies
Inside the activated venv:
```bash
pip install --upgrade -r requirements.txt
```

If you ever want to start clean, remove `.venv/` and re-run the setup section above.
