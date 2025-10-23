#!/usr/bin/env python
"""
Hardware Utilities for Motors2

Shared utilities for hardware detection and configuration.
"""

import glob
import platform
from typing import Optional


def find_serial_port() -> Optional[str]:
    """
    Auto-detect serial port for USB-to-Serial adapter.

    Checks common USB-to-Serial adapter patterns on different platforms.

    Returns:
        str: Detected port path, or None if not found

    Examples:
        >>> port = find_serial_port()
        >>> if port:
        ...     print(f"Found port: {port}")
    """
    system = platform.system()

    if system == "Darwin":  # macOS
        patterns = [
            "/dev/cu.usbserial*",
            "/dev/cu.SLAB_USBtoUART*",
            "/dev/cu.wchusbserial*",
            "/dev/cu.usbmodem*",
        ]
        for pattern in patterns:
            ports = glob.glob(pattern)
            if ports:
                return ports[0]

    elif system == "Linux":
        patterns = ["/dev/ttyUSB*", "/dev/ttyACM*"]
        for pattern in patterns:
            ports = glob.glob(pattern)
            if ports:
                return ports[0]

    elif system == "Windows":
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            if ports:
                return ports[0].device
        except ImportError:
            # pyserial not installed
            pass

    return None


def list_serial_ports() -> list[str]:
    """
    List all available serial ports.

    Returns:
        list: List of available serial port paths

    Examples:
        >>> ports = list_serial_ports()
        >>> for port in ports:
        ...     print(port)
    """
    system = platform.system()
    all_ports = []

    if system == "Darwin":  # macOS
        patterns = [
            "/dev/cu.usbserial*",
            "/dev/cu.SLAB_USBtoUART*",
            "/dev/cu.wchusbserial*",
            "/dev/cu.usbmodem*",
        ]
        for pattern in patterns:
            all_ports.extend(glob.glob(pattern))

    elif system == "Linux":
        patterns = ["/dev/ttyUSB*", "/dev/ttyACM*"]
        for pattern in patterns:
            all_ports.extend(glob.glob(pattern))

    elif system == "Windows":
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            all_ports = [p.device for p in ports]
        except ImportError:
            pass

    return sorted(set(all_ports))
