# SPDX-License-Identifier: GPL-2.0-or-later
#
# lm75_model.py
#
# Reference I2cDevice model implementing the industry-standard LM75
# digital temperature sensor (originally National/Texas Instruments;
# pin- and protocol-compatible with NXP/MAXIM/STM clones). Selected with
# `python3 i2c_custom_proxy.py <sock> LM75` after importing this module
# from i2c_custom_proxy.py. Behaviour covers everything guest firmware
# needs to read temperature and to reprogram T_OS / T_HYST limits and
# the shutdown / faults / polarity bits in the config register.
#
# Address: 0x48..0x4F (3 user-strappable LSBs). The model is
# address-agnostic and trusts the QEMU i2c controller to deliver
# transactions only when the slave is selected (the `address` field
# carried in each request is informational only).
#
# Datasheet register map (LM75A / classic LM75):
#   ptr 0x0  R   Temperature     (2 B, 9-bit signed * 0.5 C, MSB-first)
#   ptr 0x1  RW  Config          (1 B, default 0x00)
#   ptr 0x2  RW  T_HYST          (2 B, signed * 0.5 C)
#   ptr 0x3  RW  T_OS  (over-limit shutdown)  (2 B, signed * 0.5 C)
# Pointer auto-increments by 1 (mod 4) after a multi-byte read of a
# 2-byte register; per datasheet section 7.7 "Auto-increment pointer".

import struct
import time

from i2c_custom_proxy import Device, register_device


class LM75Model(Device):
    """
    Minimal LM75 register map.

      0x00 R   Temperature (2 B, 9-bit signed, LSB = 0.5 C)
      0x01 RW  Config      (1 B)
      0x02 RW  T_HYST      (2 B, signed, LSB = 0.5 C, default 75 C)
      0x03 RW  T_OS        (2 B, signed, LSB = 0.5 C, default 80 C)
    """

    REG_TEMP  = 0x0
    REG_CONF  = 0x1
    REG_THYST = 0x2
    REG_TOS   = 0x3
    # LM75 pointer register is P[2:0] (datasheet 7.5.3): defined pointers
    # are 0x0..0x3; values 0x4..0x7 are reserved and our `_read_reg`
    # returns 0xFFFF for them (matching typical LM75 reserved-read
    # behaviour).
    PTR_MASK  = 0x7

    # Config register bits (datasheet 7.7.2). Only the bits we honour.
    CONF_SHUTDOWN = 0x01
    CONF_OS_POL   = 0x02
    CONF_OS_INT   = 0x04   # 0 = comparator mode, 1 = interrupt mode
    CONF_FAULT_0  = 0x00   # fault queue = 1 (bits 4..3 = 00)
    CONF_FAULT_2  = 0x08   # fault queue = 2 (bits 4..3 = 01)
    CONF_FAULT_4  = 0x10   # fault queue = 4 (bits 4..3 = 10)
    CONF_FAULT_6  = 0x18   # fault queue = 6 (bits 4..3 = 11)
    CONF_RESERVED = 0xE0   # bits 7..5 reserved, read as 0.

    def __init__(self, **_):
        # Power-on reset defaults (datasheet 7.6.1).
        self.config = 0x00
        # T_OS / T_HYST are stored as the 16-bit raw register image that
        # the device returns on the wire: temperature in bits 15..7 (9-bit
        # two's complement), bits 6..0 always 0. Defaults 80 C / 75 C
        # correspond to 160 / 150 half-deg, which pack as 0x5000 / 0x4B00.
        self.t_os   = LM75Model._deg9_to_raw(160)    # 80 C
        self.t_hyst = LM75Model._deg9_to_raw(150)    # 75 C
        # Pointer: starts at 0 (Temperature) after power-up or in response
        # to any single-byte master write.
        self.ptr = LM75Model.REG_TEMP
        # Whether a master read of a 2-byte register auto-increments the
        # pointer afterward (datasheet-defined behaviour). We enable it
        # on the temp/hyst/os registers per 7.7. Set False for a one-shot
        # read of Config, which still auto-increments (datasheet is clear
        # it increments after reads of any length).
        self._auto_inc_enabled = True

    # ---- helpers ----

    @staticmethod
    def _c2deg9(two_bytes: bytes) -> int:
        """Decode the 2-byte LM75 temperature field as 9-bit signed.

        LM75 stores temperature as an MSB-first big-endian 16-bit word
        whose top 9 bits (bits 15..7) hold the temperature as a 9-bit
        two's complement integer in 0.5 C units; the low 7 bits are
        always 0. Truth-table examples drawn from datasheet 7.5.3.1:
            0x1900 -> +50 (i.e. +25.0 C)
            0x0080 -> +1  (i.e. +0.5 C)
            0xFF80 -> -1  (i.e. -0.5 C)
            0xE700 -> -50 (i.e. -25.0 C)
            0xC900 -> -110 (i.e. -55.0 C, full-scale negative)
        """
        if len(two_bytes) < 2:
            return 0
        raw = (two_bytes[0] << 8) | two_bytes[1]
        v9 = (raw >> 7) & 0x1FF
        if v9 & 0x100:           # 9-bit two's-complement: extend sign.
            v9 -= 0x200
        return v9

    @staticmethod
    def _deg9_to_raw(half_deg: int) -> int:
        """Encode an int (in half-degree units) as the 16-bit LM75 field.

        Inverse of `_c2deg9`. Produces the bytes a real LM75 would
        return for a temperature of `half_deg * 0.5 C` (9-bit two's
        complement placed in bits 15..7, low 7 bits forced to 0).
        """
        v9 = int(half_deg) & 0x1FF
        return (v9 << 7) & 0xFFFF

    def _read_reg(self, reg: int) -> bytes:
        if reg == LM75Model.REG_TEMP:
            # Synthesise ~+25 C (0x19 0x00 = 50 half-deg). A slow wander
            # in the low half-degree bit makes smoke tests recognisable.
            half_deg = 50 + (int(time.time()) & 0x7)
            return struct.pack(">H", LM75Model._deg9_to_raw(half_deg))
        if reg == LM75Model.REG_CONF:
            # 1-byte register; datasheet defines a 2-byte read returning
            # the config byte then a don't-care byte. We honour the 1-byte
            # shape and pad to 2 bytes for driver convenience.
            return bytes([self.config & 0x1F, 0x00])
        if reg == LM75Model.REG_THYST:
            return struct.pack(">H", self.t_hyst & 0xFFFF)
        if reg == LM75Model.REG_TOS:
            return struct.pack(">H", self.t_os & 0xFFFF)
        # Unimplemented pointer: datasheet says LM75 reserves 0x4..0x7 and
        # typically returns 0xFF for both bytes. Mirror that.
        return b"\xff\xff"

    def _write_reg(self, reg: int, payload: bytes) -> None:
        if reg == LM75Model.REG_TEMP:
            # Temperature is read-only; datasheet silently ignores writes.
            return
        if reg == LM75Model.REG_CONF:
            # 1-byte write is the canonical form; tolerate 2-byte writes
            # by taking the first byte (drivers shouldn't do this).
            self.config = (
                payload[0] & ~LM75Model.CONF_RESERVED
            ) if payload else 0
            return
        if reg == LM75Model.REG_THYST:
            if len(payload) >= 2:
                self.t_hyst = (payload[0] << 8) | payload[1]
            elif len(payload) == 1:
                # Align to MSB so a 1-byte write to bits 15..8 lands in
                # the high byte (matches TMP-class sensor ergonomics).
                self.t_hyst = payload[0] << 8
            return
        if reg == LM75Model.REG_TOS:
            if len(payload) >= 2:
                self.t_os = (payload[0] << 8) | payload[1]
            elif len(payload) == 1:
                self.t_os = payload[0] << 8
        # Reserved pointers: ignore.

    # ---- Device interface ----

    def send(self, address: int, payload: bytes) -> int:
        # First byte is the pointer write; remaining bytes (if any) are a
        # register write into the register the pointer now addresses.
        # Datasheet 7.5.3 (write protocol).
        if not payload:
            return 0
        # Pointer is the low 2 bits of byte 0 (datasheet 7.5.3).
        self.ptr = payload[0] & LM75Model.PTR_MASK
        if len(payload) >= 2:
            # 1- or 2-byte register write of the register at ptr.
            self._write_reg(self.ptr, payload[1:])
        return 0

    def recv(self, address: int, rx_len_max: int) -> bytes:
        # Encode the current register, truncate to the host's requested
        # length. After a full read of a 2-byte register, auto-increment
        # the pointer (datasheet 7.7).
        rx_full = self._read_reg(self.ptr)
        rx = rx_full[:rx_len_max]
        if self._auto_inc_enabled and rx_len_max >= 2 and len(rx_full) == 2:
            self.ptr = (self.ptr + 1) & LM75Model.PTR_MASK
        return rx

    def ping(self, address: int) -> None:
        # No real device handshake required; the QEMU side PING op already
        # verifies the socket path / model loading.
        return None


register_device("LM75", LM75Model)
