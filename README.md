# xler
xlerobot on device - Feetech servo motor control library

## Installation

### Option 1: Using pip (recommended for macOS/Raspberry Pi)
```bash
pip install -r requirements.txt
```

Or install in development mode:
```bash
pip install -e .
```

### Option 2: Using conda (Windows/macOS)
```bash
conda env create -f environment.yml
conda activate xler
```

Or update existing environment:
```bash
conda env update -f environment.yml
```

## Platform-Specific Notes

### Windows
- Ensure you have the appropriate USB-to-Serial drivers installed
- Serial ports appear as `COM1`, `COM3`, etc.

### macOS
- Serial ports typically appear as `/dev/tty.usbserial-*` or `/dev/tty.SLAB_USBtoUART`
- You may need to grant terminal permission to access USB devices

### Raspberry Pi / Linux
- Serial ports typically appear as `/dev/ttyUSB0` or `/dev/ttyAMA0`
- Add your user to the dialout group for serial port access:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
  (then logout and login again)

## Quick Start

### Configuration
Before running, configure your motor IDs in `motors/config.yaml`:

```yaml
motor_ids:
  left: 1      # Change to your motor IDs
  right: 2
  back: 3
```

See [CONFIGURATION.md](CONFIGURATION.md) for detailed configuration guide.

### Basic Usage
```python
from motors.feetech import FeetechMotorsBus

bus = FeetechMotorsBus(port="/dev/ttyUSB0")  # or "COM3" on Windows
bus.connect()
# ... your code here
```

### Run Example Scripts

**Basic movement control:**
```bash
# Uses config from motors/config.yaml
python motors/examples/keyboard_controller_base_control.py

# Override motor IDs via CLI
python motors/examples/keyboard_controller_base_control.py --motor-left 5 --motor-right 6 --motor-back 7

# View current configuration
python motors/examples/keyboard_controller_base_control.py --show-config
```

**Robot scanning with video recording:**
```bash
# Basic usage
python xler_scan.py

# With custom settings
python xler_scan.py --speed 500 --camera-fps 60
```

## Features

- **🤖 Movement Control**: High-level omnidirectional base controller
- **⌨️ Input Handling**: Keyboard + DualSense (PS5) controller support
- **📹 Video Recording**: Automatic camera recording during robot operation
- **⚙️ Configuration**: YAML-based motor configuration with CLI override
- **🔧 Modular Design**: Reusable components for easy integration

## Project Structure

```
xler/
├── motors/
│   ├── config.yaml                 # Motor configuration
│   ├── movement_controller.py      # Movement logic
│   ├── input_handler.py            # Input abstraction
│   └── examples/                   # Example scripts
├── utils/
│   └── camera_recorder.py          # Video recording
├── xler_scan.py                    # Main scanning task
├── ARCHITECTURE.md                 # Architecture documentation
└── CONFIGURATION.md                # Configuration guide
```
