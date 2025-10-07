# feetech_core.py
# Minimal Feetech (SCS/SMS-style) driver for Raspberry Pi / macOS
# Dependencies: pip install pyserial

from __future__ import annotations
import time
from dataclasses import dataclass
from typing import List, Optional

import serial
import serial.tools.list_ports

# -------- Protocol constants (adjust if your model differs) --------
HEADER = b"\xFF\xFF"

INST_PING  = 0x01
INST_READ  = 0x02
INST_WRITE = 0x03

# STS3215 register map (from lerobot):
REG_ID             = 0x05
REG_MAX_TORQUE     = 16    # 0x10 - 2 bytes (max torque limit, EEPROM)
REG_LOCK           = 0x30  # EEPROM lock (0=unlocked, 1=locked)
REG_OPERATING_MODE = 33    # 0x21 - 1 byte (0=position, 1=velocity, 2=PWM, 3=step)
REG_TORQUE_ENABLE  = 40    # 0x28 - 0=off, 1=on
REG_ACCELERATION   = 41    # 0x29 - acceleration (0-254)
REG_GOAL_POS       = 42    # 0x2A - 2 bytes (servo mode)
REG_GOAL_TIME      = 44    # 0x2C - 2 bytes (movement time in ms)
REG_GOAL_VELOCITY  = 46    # 0x2E - 2 bytes (velocity, bit15=direction)
REG_TORQUE_LIMIT   = 48    # 0x30 - 2 bytes (torque limit)
REG_PRESENT_POS    = 56    # 0x38 - 2 bytes (read-only)
REG_PRESENT_SPEED  = 58    # 0x3A - 2 bytes (read-only)
REG_MOVING         = 66    # 0x42 - 1 byte (moving status)

# -------------------------------------------------------------------

def _checksum(body: bytes) -> int:
    # Feetech SCS/SMS: checksum is bitwise NOT of sum(body) & 0xFF
    return (~sum(body) & 0xFF)

def _pkt(id_: int, instr: int, params: bytes) -> bytes:
    # body: [ID][LEN][INSTR][PARAMS...]
    length = 2 + len(params)  # INSTR + PARAMS + CHECKSUM (check added later)
    body = bytes([id_, length, instr]) + params
    return HEADER + body + bytes([_checksum(body)])

class SerialBus:
    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.03):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        # Clear buffers and let port stabilize
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.05)

    def write(self, data: bytes) -> int:
        return self.ser.write(data)

    def read(self, nbytes: int) -> bytes:
        return self.ser.read(nbytes)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    # ---- convenience ops ----
    def ping(self, sid: int) -> bool:
        self.ser.reset_input_buffer()  # Clear stale data
        self.write(_pkt(sid, INST_PING, b""))
        resp = self.read(6)  # FF FF ID LEN ERR CHK (typical)
        return len(resp) >= 4 and resp.startswith(HEADER) and resp[2] == sid

    def broadcast_ping(self) -> bool:
        self.write(_pkt(0xFE, INST_PING, b""))
        resp = self.read(6)
        return len(resp) >= 2 and resp.startswith(HEADER)

    def read2(self, sid: int, addr: int) -> Optional[int]:
        # read 2 bytes from addr
        self.ser.reset_input_buffer()  # Clear stale data
        self.write(_pkt(sid, INST_READ, bytes([addr, 2])))
        resp = self.read(7)  # FF FF ID LEN ERR [2 data] CHK
        if len(resp) < 7 or not resp.startswith(HEADER):
            return None
        data_lo, data_hi = resp[-3], resp[-2]
        return data_lo | (data_hi << 8)

    def write2(self, sid: int, addr: int, value: int) -> None:
        lo = value & 0xFF
        hi = (value >> 8) & 0xFF
        params = bytes([addr, 2, lo, hi])
        self.write(_pkt(sid, INST_WRITE, params))
        # optional: read status, but many setups run 'status return level' = 0

    def write1(self, sid: int, addr: int, value: int) -> None:
        params = bytes([addr, 1, value & 0xFF])
        self.write(_pkt(sid, INST_WRITE, params))

@dataclass
class Servo:
    id: int
    min_pos: int = 0
    max_pos: int = 1023
    default_speed: int = 0  # not used here (many models write speed at a different reg)

    def clamp(self, pos: int) -> int:
        return max(self.min_pos, min(self.max_pos, pos))

    def move_to(self, bus: SerialBus, pos: int, acceleration: int = 50) -> None:
        """Move to position with specified acceleration.
        Args:
            pos: target position (0-1023)
            acceleration: movement acceleration (0-254, 0=infinite)
        """
        pos = self.clamp(pos)
        # Set acceleration first (required for STS3215)
        bus.write1(self.id, REG_ACCELERATION, acceleration)
        # Then set goal position
        bus.write2(self.id, REG_GOAL_POS, pos)

    def present_position(self, bus: SerialBus) -> Optional[int]:
        return bus.read2(self.id, REG_PRESENT_POS)

    def enable_torque(self, bus: SerialBus) -> None:
        bus.write1(self.id, REG_TORQUE_ENABLE, 1)

    def disable_torque(self, bus: SerialBus) -> None:
        bus.write1(self.id, REG_TORQUE_ENABLE, 0)

    def set_velocity(self, bus: SerialBus, velocity: int) -> None:
        """Set goal velocity (for continuous rotation).
        Args:
            velocity: rotation velocity (0-4095, direction depends on mode)
        """
        velocity = max(0, min(4095, velocity))
        bus.write2(self.id, REG_GOAL_VELOCITY, velocity)

    def set_acceleration(self, bus: SerialBus, accel: int) -> None:
        """Set acceleration for movements (0-254, 0=infinite)"""
        accel = max(0, min(254, accel))
        bus.write1(self.id, REG_ACCELERATION, accel)

    def set_operating_mode(self, bus: SerialBus, mode: int) -> None:
        """Set operating mode (0=position/servo, 1=velocity/wheel, 2=PWM, 3=step).
        NOTE: Torque must be disabled before changing mode!
        """
        if mode not in [0, 1, 2, 3]:
            raise ValueError("Mode must be 0(position), 1(velocity), 2(PWM), or 3(step)")
        bus.write1(self.id, REG_OPERATING_MODE, mode)

    def set_torque_limit(self, bus: SerialBus, limit: int) -> None:
        """Set torque limit (0-1023, higher=more torque)"""
        limit = max(0, min(1023, limit))
        bus.write2(self.id, REG_TORQUE_LIMIT, limit)

    def rotate_continuous(self, bus: SerialBus, velocity: int, direction_cw: bool = True) -> None:
        """Rotate continuously at specified velocity in velocity mode.

        In velocity mode for STS servos, write speed to Goal_Velocity register (0x2E):
        - CW: 0 to 1023 (higher = faster)
        - CCW: Use bit 15 set (0x8000 | speed)
        - Stop: 0

        Args:
            velocity: rotation speed (0-1023)
            direction_cw: True for clockwise, False for counter-clockwise
        """
        velocity = max(0, min(1023, abs(velocity)))
        if not direction_cw:
            velocity |= 0x8000  # Set bit 15 for CCW direction

        bus.ser.reset_input_buffer()  # Clear any stale data
        # In velocity mode, write to Goal_Velocity register (0x2E = 46)
        bus.write2(self.id, REG_GOAL_VELOCITY, velocity)
        time.sleep(0.02)  # Let command process

    def stop_rotation(self, bus: SerialBus, num_retries: int = 5) -> None:
        """Stop continuous rotation by writing 0 to Goal_Velocity multiple times.

        Like XLeRobot, we retry the write multiple times to ensure it's received.
        """
        for i in range(num_retries):
            bus.ser.reset_input_buffer()  # Clear any stale data
            bus.write2(self.id, REG_GOAL_VELOCITY, 0)
            time.sleep(0.01)  # Small delay between retries

# -------- Helpers: port scan & ID scan --------

def find_feetech_port(baud: int = 1_000_000, verbose: bool = True) -> Optional[str]:
    """Scan serial ports, broadcast-ping, return first matching device."""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if verbose:
        print(f"[scan] checking ports: {ports}")
    for p in ports:
        try:
            b = SerialBus(p, baud=baud, timeout=0.03)
            ok = b.broadcast_ping()
            b.close()
            if ok:
                if verbose:
                    print(f"[scan] ✅ likely Feetech bus on: {p}")
                return p
        except Exception as e:
            if verbose:
                print(f"[scan] ⚠️ {p}: {e}")
    if verbose:
        print("[scan] ❌ no Feetech device found")
    return None

def scan_ids(bus: SerialBus, start: int = 1, end: int = 10, verbose: bool = True) -> List[int]:
    found: List[int] = []
    for sid in range(start, end + 1):
        try:
            ok = bus.ping(sid)
            if ok:
                found.append(sid)
                if verbose:
                    print(f"[ids] ✅ detected servo ID={sid}")
            else:
                if verbose:
                    print(f"[ids] .. ID={sid} no response")
        except Exception as e:
            if verbose:
                print(f"[ids] ⚠️ ID={sid} error: {e}")
    return found

def set_servo_id(bus: SerialBus, old_id: int, new_id: int) -> None:
    if not (1 <= new_id <= 253):
        raise ValueError("ID must be 1..253")
    # Unlock EEPROM, write new ID, lock EEPROM to save
    bus.write1(old_id, REG_LOCK, 0)  # unlock
    time.sleep(0.01)
    bus.write1(old_id, REG_ID, new_id)
    time.sleep(0.01)
    bus.write1(old_id, REG_LOCK, 1)  # lock (saves to EEPROM)
    time.sleep(0.1)  # let servo save
