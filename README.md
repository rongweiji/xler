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
- Adjust capture cadence via `camera.fps` in `xler.yaml` and override device/output paths via `--camera-left`, `--camera-right`, or `--camera-output-dir`.
- Control resolution/FPS/pixel format via `camera.resolution`, `camera.fps`, and `camera.pixel_format` in `xler.yaml` (defaults: `1280x720`, `30`, `mjpeg`).
- Frames are saved as JPEGs with EXIF timestamps in `recordings/front_stereo_cam_left` and `recordings/front_stereo_cam_right`.

### Recorded Files Layout

When recording is enabled, images and metadata are written under the `recordings/` directory by default. Each run creates a new `workspaceN` folder (e.g. `workspace1`, `workspace2`, ...):

```text
recordings/
├── workspace1/
│   ├── front_stereo_cam_left/
│   │   ├── 0000001.jpg
│   │   ├── 0000002.jpg
│   │   └── ...
│   ├── front_stereo_cam_right/
│   │   ├── 0000001.jpg
│   │   ├── 0000002.jpg
│   │   └── ...
│   └── frames_meta.json
├── workspace2/
│   └── ...
└── ...
```

- Each capture uses a monotonic, zero-padded 7-digit index for the filename (e.g. `0000001.jpg`). The same basename appears in both left/right folders for a given workspace.
- A `frames_meta.json` file in each `workspaceN` folder stores the mapping from filename to capture metadata, including the capture timestamp in **nanoseconds**:

```json
{
  "0000001.jpg": {
    "timestamp_ns": 1732372201500000000
  }
}
```

You can use this JSON file to pair images with their precise capture time and frame index during offline processing. Each workspace is self-contained, so you can archive or analyze them independently.

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
