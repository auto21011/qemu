# SPDX-License-Identifier: GPL-2.0-or-later
#
# tmp117_model.py
#
# Reference I2cDevice model implementing the TI TMP117 16-bit I2C
# temperature sensor register map. Selected with `python3 i2c_custom_proxy.py
# <sock> TMP117` after importing this module from i2c_custom_proxy.py.
# Verified against datasheet defaults sufficient for guest firmware to
# read temperature and twiddle config.
#
# Address: default 0x48 (GND); our model is address-agnostic and trusts
# QEMU to deliver transactions only when the i2c controller selects it
# (`address` field in each request is informational only).

import struct
import time

from i2c_custom_proxy import Device, register_device


class TMP117Model(Device):
    """
    Minimal TMP117 register map.

      0x00 R   Temperature (16-bit signed, LSB = 0.0078125 C)
      0x01 R   ID (0x0117)
      0x02 RW  High-limit
      0x03 RW  Low-limit
      0x06 RW  Configuration
    """

    REG_TEMP   = 0x00
    REG_ID     = 0x01
    REG_HI_LIM = 0x02
    REG_LO_LIM = 0x03
    REG_CONFIG = 0x06
    REG_COUNT  = 0x7
    ID_VALUE   = 0x0117
    CONFIG_DEFAULT = 0x0022  # defaults from datasheet rev-c.

    def __init__(self, **_):
        self.id_value = TMP117Model.ID_VALUE
        self.config = TMP117Model.CONFIG_DEFAULT
        self.hi = 0x6000   # ~ 24 C by LSB = 0.0078 C
        self.lo = 0x4000
        # Pointer register (writes select the next read target).
        self.ptr = TMP117Model.REG_TEMP
        # Pending read bytes pre-fetched for a RECV whose register was
        # selected by an earlier SEND (one-byte write of the ptr).
        # QEMU's protocol issues SEND(master-write) then RECV(master-read),
        # and many guest drivers write the pointer then immediately read.
        # We materialise the answer in recv() lazily so a non-current
        # SEND does not need to pre-stage. Default mode below.
        self._staged = b""

    # ---- helpers ----

    @staticmethod
    def _i16(v: int) -> int:
        """Coerce a 16-bit register value into a signed int16."""
        return struct.unpack(">h", struct.pack(">H", v & 0xFFFF))[0]

    def _read_reg(self, reg: int) -> bytes:
        # Two bytes big-endian, matching TMP117 datasheet.
        if reg == TMP117Model.REG_TEMP:
            # Synthetic temperature: ~ room temperature (25 C) with a slow
            # wander to make smoke tests recognisable (0x6000 = 24 C).
            # LSB = 7.8125 mC, so 24.0 C ~= 0x6000.
            t = 0x6000 + (int(time.time()) & 0x00FF)
            return struct.pack(">H", t & 0xFFFF)
        if reg == TMP117Model.REG_ID:
            return struct.pack(">H", self.id_value)
        if reg == TMP117Model.REG_HI_LIM:
            return struct.pack(">H", self.hi & 0xFFFF)
        if reg == TMP117Model.REG_LO_LIM:
            return struct.pack(">H", self.lo & 0xFFFF)
        if reg == TMP117Model.REG_CONFIG:
            return struct.pack(">H", self.config & 0xFFFF)
        # Unimplemented register: return 0xffff (datasheet reserved reads are
        # typically 0xffff).
        return b"\xff\xff"

    def _write_reg(self, reg: int, payload: bytes) -> None:
        if len(payload) < 2:
            return
        value = struct.unpack(">H", payload[:2])[0]
        if reg == TMP117Model.REG_HI_LIM:
            self.hi = value
        elif reg == TMP117Model.REG_LO_LIM:
            self.lo = value
        elif reg == TMP117Model.REG_CONFIG:
            self.config = value
        # Temperature register is read-only; writes are ignored.

    # ---- Device interface ----

    def send(self, address: int, payload: bytes) -> int:
        # First byte is conventionally the pointer register for TMP-class
        # sensors (datasheet 7.5.3). Bytes 1..2 are an optional 16-bit write.
        if not payload:
            return 0
        # TMP117 pointer is 3 bits (datasheet 7.5.1). Mask with 0x7 not
        # REG_COUNT-1 (REG_COUNT was a *number of registers*; it is NOT
        # the pointer bit width).
        self.ptr = payload[0] & 0x7
        if len(payload) >= 3:
            # Two-byte write: byte 0 was ptr, bytes 1..2 are value
            self._write_reg(self.ptr, payload[1:3])
        return 0

    def recv(self, address: int, rx_len_max: int) -> bytes:
        rx = self._read_reg(self.ptr)
        return rx[:rx_len_max]

    def ping(self, address: int) -> None:
        return None


# Self-register so import side effects make the device immediately usable
# from i2c_custom_proxy.main() with the CLI name "TMP117".
register_device("TMP117", TMP117Model)
