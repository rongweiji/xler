"""
Motor Configuration Loader

Loads motor configuration from YAML file with CLI argument override support.
Configuration file location: motors/config.yaml

Example:
    from motors.config_loader import load_motor_config

    # Load with defaults from config.yaml
    config = load_motor_config()

    # Override specific values
    config = load_motor_config(motor_left=5, speed=500)

    # Access config
    print(config['motor_ids'])  # {'left': 5, 'right': 2, 'back': 3}
    print(config['motor_settings']['default_speed'])  # 500
"""

import yaml
from pathlib import Path
from typing import Optional, Dict, Any


def load_motor_config(
    config_path: Optional[str] = None,
    motor_left: Optional[int] = None,
    motor_right: Optional[int] = None,
    motor_back: Optional[int] = None,
    speed: Optional[int] = None,
    port: Optional[str] = None,
    **kwargs
) -> Dict[str, Any]:
    """
    Load motor configuration from YAML file with optional CLI overrides.

    Args:
        config_path: Path to config file (default: motors/config.yaml)
        motor_left: Override left motor ID
        motor_right: Override right motor ID
        motor_back: Override back motor ID
        speed: Override default speed
        port: Override serial port
        **kwargs: Additional overrides

    Returns:
        Dict with motor configuration

    Example:
        config = load_motor_config(motor_left=5, speed=500)
    """
    # Default config path
    if config_path is None:
        config_path = Path(__file__).parent / "config.yaml"
    else:
        config_path = Path(config_path)

    # Load YAML config
    if config_path.exists():
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    else:
        # Fallback to default config if file doesn't exist
        print(f"⚠️  Config file not found: {config_path}")
        print("   Using default configuration")
        config = get_default_config()

    # Apply CLI overrides
    if motor_left is not None:
        config['motor_ids']['left'] = motor_left
    if motor_right is not None:
        config['motor_ids']['right'] = motor_right
    if motor_back is not None:
        config['motor_ids']['back'] = motor_back
    if speed is not None:
        config['motor_settings']['default_speed'] = speed
    if port is not None:
        config['serial']['port'] = port

    # Apply any additional kwargs as overrides
    for key, value in kwargs.items():
        if value is not None:
            # Try to find the key in the config and update it
            if key in config.get('motor_settings', {}):
                config['motor_settings'][key] = value
            elif key in config.get('serial', {}):
                config['serial'][key] = value
            elif key in config.get('control', {}):
                config['control'][key] = value

    return config


def get_default_config() -> Dict[str, Any]:
    """
    Get default motor configuration.

    Returns:
        Dict with default configuration
    """
    return {
        'motor_ids': {
            'left': 1,
            'right': 2,
            'back': 3,
        },
        'motor_settings': {
            'model': 'sts3215',
            'max_velocity': 1023,
            'torque_limit': 700,
            'default_speed': 400,
        },
        'serial': {
            'port': None,
            'baudrate': 1000000,
            'protocol_version': 0,
        },
        'control': {
            'frequency': 50,
            'print_interval': 5,
        }
    }


def save_motor_config(config: Dict[str, Any], config_path: Optional[str] = None):
    """
    Save motor configuration to YAML file.

    Args:
        config: Configuration dict to save
        config_path: Path to save config (default: motors/config.yaml)
    """
    if config_path is None:
        config_path = Path(__file__).parent / "config.yaml"
    else:
        config_path = Path(config_path)

    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f"✅ Configuration saved to: {config_path}")


def print_motor_config(config: Dict[str, Any]):
    """
    Print motor configuration in readable format.

    Args:
        config: Configuration dict to print
    """
    print("\n" + "="*60)
    print("  MOTOR CONFIGURATION")
    print("="*60)

    print("\n🤖 Motor IDs:")
    for role, motor_id in config['motor_ids'].items():
        print(f"  {role.capitalize():<6} : Motor ID {motor_id}")

    print("\n⚙️  Motor Settings:")
    settings = config['motor_settings']
    print(f"  Model          : {settings['model']}")
    print(f"  Max Velocity   : {settings['max_velocity']}")
    print(f"  Torque Limit   : {settings['torque_limit']}")
    print(f"  Default Speed  : {settings['default_speed']}")

    print("\n🔌 Serial Settings:")
    serial = config['serial']
    port_display = serial['port'] if serial['port'] else "auto-detect"
    print(f"  Port           : {port_display}")
    print(f"  Baudrate       : {serial['baudrate']}")
    print(f"  Protocol       : {serial['protocol_version']}")

    print("\n🎛️  Control Settings:")
    control = config['control']
    print(f"  Frequency      : {control['frequency']} Hz")
    print(f"  Print Interval : {control['print_interval']} cycles")

    print("="*60 + "\n")


def validate_motor_config(config: Dict[str, Any]) -> bool:
    """
    Validate motor configuration.

    Args:
        config: Configuration dict to validate

    Returns:
        True if valid, False otherwise
    """
    # Check required keys
    required_keys = ['motor_ids', 'motor_settings', 'serial', 'control']
    for key in required_keys:
        if key not in config:
            print(f"❌ Missing required config key: {key}")
            return False

    # Check motor IDs
    required_motor_roles = ['left', 'right', 'back']
    for role in required_motor_roles:
        if role not in config['motor_ids']:
            print(f"❌ Missing motor ID for role: {role}")
            return False

        motor_id = config['motor_ids'][role]
        if not isinstance(motor_id, int) or motor_id < 0 or motor_id > 253:
            print(f"❌ Invalid motor ID for {role}: {motor_id} (must be 0-253)")
            return False

    # Check for duplicate motor IDs
    motor_ids = list(config['motor_ids'].values())
    if len(motor_ids) != len(set(motor_ids)):
        print(f"❌ Duplicate motor IDs found: {motor_ids}")
        return False

    # Check velocity and torque limits
    max_vel = config['motor_settings'].get('max_velocity', 1023)
    if not isinstance(max_vel, int) or max_vel < 0 or max_vel > 1023:
        print(f"❌ Invalid max_velocity: {max_vel} (must be 0-1023)")
        return False

    torque = config['motor_settings'].get('torque_limit', 700)
    if not isinstance(torque, int) or torque < 0 or torque > 1023:
        print(f"❌ Invalid torque_limit: {torque} (must be 0-1023)")
        return False

    return True
