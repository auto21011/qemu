# SPDX-License-Identifier: GPL-2.0-or-later
#
# m25p80_stub_model.py
#
# Minimal SPI flash model demonstrating the spi_custom_proxy.Device ABC.
# Exposes a M25P80-style register interface: reading status register
# (0x00 = ready), and a simple read-data bytestream.
#
# Protocol notes (common M25P80 commands):
#   0x05  Read Status Register   -> 1 byte (0x00 = WIP clear)
#   0x03  Read Data              -> N data bytes from a page-sized FIFO
#   0x06  Write Enable           -> set WEL in SR
#   0x04  Write Disable          -> clear WEL
#
# The stub does NOT model erase/program cycles — it returns cycling
# bytes (0x00, 0x01, 0x02, ...) for any read-data command, which is
# sufficient to verify guest-to-proxy data flow on an ast2600-evb
# SPI bus.
#
# Selected with `python3 spi_custom_proxy.py <sock> M25P80_STUB`.

from spi_custom_proxy import Device, register_device


class M25P80StubModel(Device):
    """Minimal SPI flash model: status register + streaming read-data."""

    CMD_READ_STATUS = 0x05
    CMD_WRITE_EN    = 0x06
    CMD_WRITE_DIS   = 0x04
    CMD_READ        = 0x03
    CMD_READ_FAST   = 0x0B

    def __init__(self, **_):
        self.status = 0x00          # WIP=0, WEL=0
        self.addr = 0               # 3-byte address accumulator
        self.addr_bytes = 0         # count of address bytes collected
        self.command = 0x00         # current command byte
        self.waiting_cmd = True     # awaiting next command byte
        self.read_count = 0         # bytes since last READ start
        self.response_byte = 0x00   # dummy byte for dummy cycles

    def _reset_cmd(self):
        self.waiting_cmd = True
        self.addr = 0
        self.addr_bytes = 0
        self.read_count = 0

    def transfer(self, byte_in: int) -> int:
        if self.waiting_cmd:
            self.command = byte_in
            self.waiting_cmd = False
            self.addr = 0
            self.addr_bytes = 0
            self.read_count = 0

            if self.command in (self.CMD_READ_STATUS,):
                return 0xFF   # dummy byte before status
            if self.command in (self.CMD_WRITE_EN, self.CMD_WRITE_DIS):
                return 0x00
            if self.command in (self.CMD_READ, self.CMD_READ_FAST):
                return 0x00   # first dummy byte of READ
            # Unknown command: flush back to idle
            self._reset_cmd()
            return 0x00

        # Command byte has been consumed; process follow-up bytes.

        # Write Enable / Write Disable: single-byte commands.
        if self.command == self.CMD_WRITE_EN:
            self.status |= 0x02    # set WEL
            self._reset_cmd()
            return 0x00
        if self.command == self.CMD_WRITE_DIS:
            self.status &= ~0x02   # clear WEL
            self._reset_cmd()
            return 0x00

        # Read Status Register: dummy byte followed by 1 status byte.
        if self.command == self.CMD_READ_STATUS:
            if self.read_count == 0:
                # First byte: status register.
                self.read_count += 1
                return self.status
            # Subsequent bytes keep returning status.
            return self.status

        # READ (0x03): collect 3 address bytes, then stream data.
        if self.command == self.CMD_READ:
            if self.addr_bytes < 3:
                self.addr = ((self.addr << 8) | (byte_in & 0xFF))
                self.addr_bytes += 1
                return 0x00
            # Return streaming data bytes.
            val = self.addr & 0xFF
            self.addr = (self.addr + 1) & 0xFFFFFF
            return val

        # READ_FAST (0x0B): same but with 1 dummy byte after address.
        if self.command == self.CMD_READ_FAST:
            if self.addr_bytes < 3:
                self.addr = ((self.addr << 8) | (byte_in & 0xFF))
                self.addr_bytes += 1
                return 0x00
            # One dummy byte (takes one extra clock).
            if self.read_count == 3:
                self.read_count += 1
                return 0x00
            # Data bytes.
            val = self.addr & 0xFF
            self.addr = (self.addr + 1) & 0xFFFFFF
            return val

        # Fallback: unknown command in progress.
        self._reset_cmd()
        return 0x00

    def assert_cs(self) -> None:
        # CS# asserted: ready for a new command.
        self._reset_cmd()

    def release_cs(self) -> None:
        # CS# deasserted: reset.
        self._reset_cmd()

    def ping(self) -> None:
        pass


register_device("M25P80_STUB", M25P80StubModel)
