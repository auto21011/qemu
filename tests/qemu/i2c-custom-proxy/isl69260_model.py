# SPDX-License-Identifier: GPL-2.0-or-later
#
# isl69260_model.py
#
# PMBus device model for the Renesas ISL69260 digital multiphase PWM
# voltage regulator.  Selected with
#   python3 i2c_custom_proxy.py <sock> ISL69260 [--connect]
#
# Implements the subset of PMBus standard + manufacturer registers that
# the Linux pmbus core driver (drivers/hwmon/pmbus/isl68137.c, variant
# raa_dmpvr2_2rail) reads or writes during probe and normal operation.
# The driver uses direct data format with m=1, b=0 and per-class R
# exponents, so the on-wire 16-bit Y maps directly: milli = Y * 10^(3-R).

import struct
import time

from i2c_custom_proxy import Device, register_device

# PMBus standard command codes (pmbus.h)
PAGE                = 0x00
OPERATION           = 0x01
ON_OFF_CONFIG       = 0x02
CLEAR_FAULTS        = 0x03
WRITE_PROTECT       = 0x10
CAPABILITY          = 0x19
VOUT_MODE           = 0x20
VOUT_COMMAND        = 0x21
VOUT_TRIM           = 0x22
VOUT_MAX            = 0x24
VOUT_MARGIN_HIGH    = 0x25
VOUT_MARGIN_LOW     = 0x26
VOUT_OV_FAULT_LIMIT = 0x40
VOUT_OV_WARN_LIMIT  = 0x42
VOUT_UV_WARN_LIMIT  = 0x43
VOUT_UV_FAULT_LIMIT = 0x44
IOUT_OC_FAULT_LIMIT = 0x46
IOUT_OC_WARN_LIMIT  = 0x4A
OT_FAULT_LIMIT      = 0x4F
OT_WARN_LIMIT       = 0x51
VIN_OV_FAULT_LIMIT  = 0x55
VIN_OV_WARN_LIMIT   = 0x57
VIN_UV_WARN_LIMIT   = 0x58
VIN_UV_FAULT_LIMIT  = 0x59
STATUS_BYTE         = 0x78
STATUS_WORD         = 0x79
STATUS_VOUT         = 0x7A
STATUS_IOUT         = 0x7B
STATUS_INPUT        = 0x7C
STATUS_TEMPERATURE  = 0x7D
STATUS_CML          = 0x7E
STATUS_MFR_SPECIFIC = 0x80
READ_VIN            = 0x88
READ_IIN            = 0x89
READ_VOUT           = 0x8B
READ_IOUT           = 0x8C
READ_TEMP_1         = 0x8D
READ_TEMP_2         = 0x8E
READ_TEMP_3         = 0x8F
READ_POUT           = 0x96
READ_PIN            = 0x97
REVISION            = 0x98
MFR_ID              = 0x99
MFR_MODEL           = 0x9A
MFR_REVISION        = 0x9B
IC_DEVICE_ID        = 0xAD
IC_DEVICE_REV       = 0xAE
RAA_READ_VMON       = 0xC8

# Block-read registers (return [len][data...] on recv)
_BLK_REGS = {MFR_ID, MFR_MODEL, MFR_REVISION, IC_DEVICE_ID, IC_DEVICE_REV}


class ISL69260Model(Device):
    """Renesas ISL69260 (raa_dmpvr2_2rail, 2-page VR) PMBus model.

    Per-page defaults follow the typical OpenBMC CPU-VR configuration:
    page 0 = Vcore ~1.0 V, page 1 = Vsoc ~1.2 V.  Direct-format with
    m=1, b=0 means the on-wire word Y directly equals the milli-value
    (voltage/temperature) or micro-value (power) adjusted by 10^(3-R).
    """

    NUM_PAGES = 2

    def __init__(self, **_):
        self._page = 0
        self._pending_cmd = None
        self._regs: list[dict[int, bytes]] = [
            self._default_regs(0),
            self._default_regs(1),
        ]

    # ---- defaults ----

    @staticmethod
    def _default_regs(page: int) -> dict[int, bytes]:
        vout_cmd = 1000 if page == 0 else 1200
        vout_max = vout_cmd + 500
        d: dict[int, bytes] = {}
        # byte registers
        d[PAGE] = bytes([page])
        d[OPERATION] = bytes([0x40])
        d[ON_OFF_CONFIG] = bytes([0x1A])
        d[WRITE_PROTECT] = bytes([0x00])
        d[CAPABILITY] = bytes([0xB0])
        d[VOUT_MODE] = bytes([0x40])          # direct mode
        d[STATUS_BYTE] = bytes([0x00])
        d[STATUS_WORD] = bytes([0x00, 0x00])
        d[STATUS_VOUT] = bytes([0x00])
        d[STATUS_IOUT] = bytes([0x00])
        d[STATUS_INPUT] = bytes([0x00])
        d[STATUS_TEMPERATURE] = bytes([0x00])
        d[STATUS_CML] = bytes([0x00])
        d[STATUS_MFR_SPECIFIC] = bytes([0x00])
        d[REVISION] = bytes([0x22])           # PMBus 1.2
        # word registers (big-endian)
        d[VOUT_COMMAND] = struct.pack(">H", vout_cmd)
        d[VOUT_TRIM] = struct.pack(">H", 0)
        d[VOUT_MAX] = struct.pack(">H", vout_max)
        d[VOUT_MARGIN_HIGH] = struct.pack(">H", vout_cmd + 50)
        d[VOUT_MARGIN_LOW] = struct.pack(">H", vout_cmd - 50)
        d[VOUT_OV_FAULT_LIMIT] = struct.pack(">H", vout_cmd + 100)
        d[VOUT_OV_WARN_LIMIT] = struct.pack(">H", vout_cmd + 50)
        d[VOUT_UV_WARN_LIMIT] = struct.pack(">H", vout_cmd - 50)
        d[VOUT_UV_FAULT_LIMIT] = struct.pack(">H", vout_cmd - 100)
        d[IOUT_OC_FAULT_LIMIT] = struct.pack(">H", 300)    # 30 A
        d[IOUT_OC_WARN_LIMIT] = struct.pack(">H", 250)    # 25 A
        d[OT_FAULT_LIMIT] = struct.pack(">H", 110)        # 110 C
        d[OT_WARN_LIMIT] = struct.pack(">H", 95)          # 95 C
        d[VIN_OV_FAULT_LIMIT] = struct.pack(">H", 1300)
        d[VIN_OV_WARN_LIMIT] = struct.pack(">H", 1250)
        d[VIN_UV_WARN_LIMIT] = struct.pack(">H", 1050)
        d[VIN_UV_FAULT_LIMIT] = struct.pack(">H", 1000)
        # block registers ([len][data])
        d[MFR_ID] = bytes([3]) + b"REN"
        d[MFR_MODEL] = bytes([8]) + b"ISL69260"
        d[MFR_REVISION] = bytes([3, 0x01, 0x00, 0x00])
        d[IC_DEVICE_ID] = bytes([8]) + b"ISL69260"
        d[IC_DEVICE_REV] = bytes([2, 0x41, 0x01])
        return d

    # ---- register access ----

    def _cur(self) -> dict[int, bytes]:
        return self._regs[self._page]

    def _set_page(self, val: int) -> None:
        self._page = val % self.NUM_PAGES

    @staticmethod
    def _jitter(base: int, shift: int = 4, amp: int = 0x3) -> bytes:
        """Synthesise a 'live' 16-bit direct-format reading of `base`
        with a slow ``amp``-magnitude wander seeded by the wall clock
        right-shifted by ``shift`` bits."""
        return struct.pack(">H", base + ((int(time.time()) >> shift) & amp))

    _LIVE = {
        READ_VIN:   lambda: ISL69260Model._jitter(1200, 0, 0x3),
        READ_IIN:   lambda: ISL69260Model._jitter(300, 6, 0x3),
        READ_IOUT:  lambda: ISL69260Model._jitter(150, 6, 0x3),
        READ_TEMP_1: lambda: ISL69260Model._jitter(25, 4, 0x3),
        READ_TEMP_2: lambda: ISL69260Model._jitter(35, 5, 0x3),
        READ_TEMP_3: lambda: ISL69260Model._jitter(40, 6, 0x3),
        READ_POUT:  lambda: ISL69260Model._jitter(15, 5, 0x3),
        READ_PIN:   lambda: ISL69260Model._jitter(18, 5, 0x3),
        RAA_READ_VMON: lambda: struct.pack(">H", 0),
    }

    def _read_reg(self, cmd: int) -> bytes:
        if cmd in _BLK_REGS:
            return self._cur().get(cmd, b"\x00")
        gen = self._LIVE.get(cmd)
        if gen is not None:
            return gen()
        if cmd == READ_VOUT:
            return self._cur().get(VOUT_COMMAND, struct.pack(">H", 0))
        if cmd in self._cur():
            return self._cur()[cmd]
        return b"\xff\xff"

    def _write_reg(self, cmd: int, payload: bytes) -> None:
        if cmd == PAGE:
            self._set_page(payload[0])
            return
        if cmd == CLEAR_FAULTS:
            for sc in (STATUS_BYTE, STATUS_WORD, STATUS_VOUT, STATUS_IOUT,
                       STATUS_INPUT, STATUS_TEMPERATURE, STATUS_CML,
                       STATUS_MFR_SPECIFIC):
                if sc in self._cur():
                    self._cur()[sc] = (
                        bytes([0]) if sc < STATUS_WORD else bytes([0, 0])
                    )
            return
        if cmd in _BLK_REGS:
            if len(payload) > 1:
                length = payload[0]
                self._cur()[cmd] = bytes([length]) + payload[1:1 + length]
            return
        if cmd in self._cur():
            self._cur()[cmd] = payload[:len(self._cur()[cmd])]
        else:
            # Best-effort: store word writes to limit registers we model.
            if len(payload) == 2:
                self._cur()[cmd] = payload
            elif len(payload) == 1:
                self._cur()[cmd] = payload

    # ---- Device interface ----

    def send(self, address: int, payload: bytes) -> int:
        if not payload:
            return 0
        cmd = payload[0]
        self._pending_cmd = cmd
        if len(payload) == 1 and cmd == CLEAR_FAULTS:
            self._write_reg(cmd, b"")
            return 0
        if len(payload) >= 2:
            self._write_reg(cmd, payload[1:])
        return 0

    def recv(self, address: int, rx_len_max: int) -> bytes:
        cmd = self._pending_cmd
        self._pending_cmd = None
        if cmd is None:
            return b""
        data = self._read_reg(cmd)
        return data[:rx_len_max]

    def ping(self, address: int) -> None:
        return None


register_device("ISL69260", ISL69260Model)
