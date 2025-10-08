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
```python
from motors.feetech import FeetechMotorsBus

bus = FeetechMotorsBus(port="/dev/ttyUSB0")  # or "COM3" on Windows
bus.connect()
# ... your code here
```
