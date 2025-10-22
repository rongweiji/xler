# motors2 - LeKiwi 3-Wheel Base Control

Simple and accurate 3-wheel omnidirectional robot control using the **lekiwi_base kinematics approach**.

## Overview

This package implements the **exact same pattern as lekiwi_base** for 3-wheel omnidirectional robot control:
- ✅ Uses `motors.feetech.FeetechMotorsBus` (same as lekiwi_base)
- ✅ Matrix-based kinematics with rotation matrices
- ✅ Proper m/s to motor units conversion using wheel/base geometry
- ✅ Automatic velocity scaling to prevent motor saturation
- ✅ Simple keyboard-only control (WASD+QE)
- ✅ YAML configuration for easy customization

Based on: [egocentric_control/lekiwi_base](https://github.com/chongxi/egocentric_control/tree/master/real/lekiwi_base)

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure Your Robot

Edit `motors2/config.yaml` to match your motor IDs:

```yaml
motor_ids:
  left: 1      # Your left motor ID
  right: 2     # Your right motor ID
  back: 3      # Your back motor ID
```

### 3. Run the Controller

```bash
python xler2.py
```

**Controls:**
- `W` - Forward
- `S` - Backward
- `A` - Strafe Left
- `D` - Strafe Right
- `Q` - Rotate Left (CCW)
- `E` - Rotate Right (CW)
- `SPACE` - Stop
- `ESC` - Exit

## Architecture

```
motors2/
├── config.yaml          # Robot configuration (motor IDs, geometry, limits)
├── base_controller.py   # LeKiwi kinematics controller (uses motors.feetech)
├── keyboard_input.py    # Simple WASD+QE keyboard input
└── README.md           # This file

xler2.py                 # Main control script
```

**Key Pattern (like lekiwi_base):**
```python
# Import from motors/ folder (LeRobot motor abstractions)
from motors import Motor, MotorNormMode
from motors.feetech import FeetechMotorsBus, OperatingMode

# Create motors dict
motors = {
    "base_left_wheel": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "base_right_wheel": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "base_back_wheel": Motor(3, "sts3215", MotorNormMode.DEGREES),
}

# Create bus
bus = FeetechMotorsBus(port="/dev/cu.usbserial-XXX", motors=motors)
bus.connect()

# Create controller with LeKiwi kinematics
controller = LeKiwiBaseController(bus, config)
controller.configure()

# Move using m/s and deg/s
controller.move(x_vel=0.5, y_vel=0.0, theta_vel=0.0)
```

## Configuration

### Motor IDs

```yaml
motor_ids:
  left: 1    # Motor ID for left wheel
  right: 2   # Motor ID for right wheel
  back: 3    # Motor ID for back wheel
```

### Robot Geometry

These parameters define the physical robot dimensions and are used for accurate kinematics:

```yaml
geometry:
  wheel_radius_m: 0.05       # Wheel radius in meters
  base_radius_m: 0.125       # Distance from center to wheel
  # Wheel axis angles (left, back, right) in degrees
  wheel_axis_angles_deg: [150.0, -90.0, 30.0]
  max_wheel_raw: 3000        # Max motor speed before scaling
```

**Wheel Angles Explanation:**
- `150°` - Left wheel points 150° from front (front-left)
- `-90°` - Back wheel points -90° from front (straight back)
- `30°` - Right wheel points 30° from front (front-right)

### Control Limits

```yaml
control:
  frequency: 60                # Control loop Hz
  max_speed_ms: 1.0            # Max linear speed (m/s)
  max_rotation_degs: 180.0     # Max rotation speed (deg/s)
```

## Command-Line Options

```bash
# Auto-detect serial port
python xler2.py

# Specify port
python xler2.py --port /dev/cu.usbserial-XXXXX

# Override max speed
python xler2.py --speed 0.5

# Override motor IDs
python xler2.py --motor-left 5 --motor-right 6 --motor-back 4

# Show current configuration
python xler2.py --show-config
```

## Kinematics Details

### How It Works (Same as lekiwi_base)

The LeKiwi kinematics uses a **rotation matrix** to convert body frame velocities to wheel velocities:

1. **Input:** Body frame velocities (x, y, theta)
   - `x`: forward/backward in m/s
   - `y`: left/right in m/s
   - `theta`: rotation in deg/s

2. **Rotation Matrix:** For each wheel angle α:
   ```
   [cos(α)  sin(α)  base_radius]
   ```

3. **Calculation:**
   - Wheel linear speeds = Matrix × Velocity vector
   - Wheel angular speeds = Linear speeds / wheel_radius
   - Motor units = Angular speeds × (4096 steps / 360°)

4. **Velocity Scaling:** If any wheel exceeds `max_wheel_raw`, all wheels are scaled proportionally

### Code (from base_controller.py)

```python
# Body velocity vector
velocity_vector = np.array([x, y, theta_rad])

# Build rotation matrix
angles = np.radians(self.wheel_angles)
m = np.array([[np.cos(a), np.sin(a), self.base_radius] for a in angles])

# Calculate wheel linear speeds (m/s)
wheel_linear_speeds = m.dot(velocity_vector)

# Convert to wheel angular speeds (rad/s)
wheel_angular_speeds = wheel_linear_speeds / self.wheel_radius

# Convert to raw motor units with scaling
wheel_degps = wheel_angular_speeds * (180.0 / np.pi)
wheel_raw = [self._degps_to_raw(degps) for degps in wheel_degps]
```

## Comparison with lekiwi_base

| Feature | lekiwi_base | motors2 |
|---------|-------------|---------|
| **Motor Bus** | `FeetechMotorsBus` | ✅ Same |
| **Kinematics** | Matrix-based | ✅ Same algorithm |
| **Units** | Physical (m/s, deg/s) | ✅ Same |
| **Velocity Scaling** | Proportional | ✅ Same |
| **Robot Class** | Inherits from `Robot` | ❌ Standalone (no inheritance) |
| **Calibration** | Full calibration system | ❌ Not needed for velocity mode |
| **Input** | Xbox controller (pygame) | Keyboard only |
| **Use Case** | ML/Research with LeRobot | Simple robot control |

**Summary:** motors2 uses the **exact same motor communication and kinematics** as lekiwi_base, but simplified by removing the Robot class inheritance and ML framework overhead.

## Troubleshooting

### Motors don't move
1. Check power supply (6-12V)
2. Verify motor IDs match config: `python xler2.py --show-config`
3. Test connection: `python xler_scan.py`

### Wrong movement direction
1. Check wheel angles in `config.yaml`
2. Verify motor wiring and orientation
3. Adjust `wheel_axis_angles_deg` if needed

### Port not found
```bash
# List available ports (macOS)
ls /dev/cu.*

# Specify manually
python xler2.py --port /dev/cu.usbserial-XXXXX
```

## Examples

### Basic Movement
```bash
python xler2.py
# Press W to move forward at 1.0 m/s
# Press Q while moving to add rotation
```

### Slower Speed
```bash
python xler2.py --speed 0.3
# Now max speed is 0.3 m/s
```

### Custom Configuration
```bash
# Test with different motor IDs
python xler2.py --motor-left 5 --motor-right 6 --motor-back 4
```

## Technical Details

### Dependencies

**From motors/ folder (LeRobot abstractions):**
- `Motor` - Motor configuration dataclass
- `MotorNormMode` - Normalization mode enum
- `FeetechMotorsBus` - Feetech motor bus implementation
- `OperatingMode` - Motor operating mode enum

**Python packages:**
- `numpy` - For matrix operations
- `pyyaml` - For config file loading
- `feetech-servo-sdk` - For motor communication (via motors.feetech)

### Motor Names Convention

Motors must be named with this pattern to match lekiwi_base:
- `"base_left_wheel"` - Left motor
- `"base_right_wheel"` - Right motor
- `"base_back_wheel"` - Back motor

This is required because `base_controller.py` expects these exact names in the returned velocity dictionary.

## Credits

- Based on [lekiwi_base](https://github.com/chongxi/egocentric_control/tree/master/real/lekiwi_base) by chongxi
- Uses [LeRobot](https://github.com/huggingface/lerobot) motor abstractions
- Feetech motor SDK

## License

Apache License 2.0
