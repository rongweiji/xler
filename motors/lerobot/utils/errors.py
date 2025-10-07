"""Stub error classes for lerobot motors"""

class DeviceAlreadyConnectedError(Exception):
    """Raised when trying to connect to an already connected device"""
    pass

class DeviceNotConnectedError(Exception):
    """Raised when trying to use a device that is not connected"""
    pass
