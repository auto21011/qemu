#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# selftest_isl69260.py - PMBus protocol-level self-test for the
# ISL69260 model.  Exercises the Linux pmbus core probe sequence
# (pmbus_init_common + pmbus_identify_common) and a representative
# set of READ_* registers per page, all without spawning QEMU.
#
# Exits 0 on pass, non-zero on any failure.

import os
import socket
import struct
import sys
import tempfile
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import i2c_custom_proxy as proxy  # noqa: E402
import isl69260_model  # noqa: E402  registers ISL69260 on import


def fail(msg: str) -> None:
    sys.stderr.write(f"FAIL: {msg}\n")
    sys.exit(1)


class PMBusClient:
    """Minimal client speaking the i2c-custom wire protocol."""

    def __init__(self, sock: socket.socket, address: int = 0x4d):
        self.s = sock
        self.address = address

    def _send_request(self, opcode: int, payload: bytes, rx_max: int) -> None:
        assert len(payload) <= proxy.MAX_XFER
        hdr = struct.pack(proxy.HEADER_FMT, opcode, len(payload), rx_max,
                          self.address)
        self.s.sendall(hdr + payload)

    def _recv_reply(self):
        rlen_bytes = self.s.recv(2)
        if len(rlen_bytes) != 2:
            raise AssertionError("short reply length read")
        rlen = struct.unpack(proxy.REPLY_LEN_FMT, rlen_bytes)[0]
        if rlen == proxy.NAK_RX_LEN:
            return None
        if rlen == 0:
            return b""
        got = bytearray()
        while len(got) < rlen:
            chunk = self.s.recv(rlen - len(got))
            if not chunk:
                raise AssertionError("short payload read")
            got.extend(chunk)
        return bytes(got)

    def send(self, payload: bytes):
        self._send_request(proxy.OP_SEND, payload, 0)
        return self._recv_reply()

    def recv(self, rx_max: int = 64):
        self._send_request(proxy.OP_RECV, b"", rx_max)
        return self._recv_reply()

    def ping(self):
        self._send_request(proxy.OP_PING, b"", 0)
        return self._recv_reply()

    def write_cmd(self, cmd: int, data: bytes = b""):
        return self.send(bytes([cmd]) + data)

    def read_cmd(self, cmd: int, rx_max: int):
        if self.send(bytes([cmd])) != b"":
            return None
        return self.recv(rx_max)


def run_proxy_in_thread(path: str, device) -> tuple:
    listener = proxy.make_listener(path)
    ready = threading.Event()

    def _run():
        listener.listen(8)
        ready.set()
        try:
            proxy.serve(listener, device)
        except OSError:
            pass

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    ready.wait(timeout=2.0)
    return t, listener


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "i2c-custom.sock")
        device = proxy.DEVICE_REGISTRY["ISL69260"]()

        try:
            t, listener = run_proxy_in_thread(path, device)
        except Exception as e:
            fail(f"proxy server start error: {e}")

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        deadline = 5.0
        start = os.times()[4]
        while os.times()[4] - start < deadline:
            try:
                sock.connect(path)
                break
            except (ConnectionRefusedError, FileNotFoundError):
                continue
        else:
            fail("could not connect to proxy within deadline")

        c = PMBusClient(sock)
        fails: list[str] = []

        def check(label: str, cond: bool, detail: str = ""):
            ok = "OK" if cond else "FAIL"
            status = f"  {label}: {ok}"
            if detail:
                status += f"  [{detail}]"
            print(status)
            if not cond:
                fails.append(label)

        try:
            # PING
            check("PING", c.ping() == b"")

            # ---- pmbus_init_common ----
            r = c.read_cmd(0x19, 1)       # CAPABILITY
            check("CAPABILITY = 0xB0", r == bytes([0xB0]),
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x79, 2)       # STATUS_WORD
            check("STATUS_WORD != 0xFFFF", r is not None and r != b"\xff\xff",
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x10, 1)       # WRITE_PROTECT
            check("WRITE_PROTECT = 0", r == bytes([0x00]),
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x98, 1)       # REVISION
            check("REVISION = 0x22", r == bytes([0x22]),
                  f"got {r.hex() if r else 'NAK'}")

            # ---- page 0 identify ----
            check("PAGE write=0",
                  c.write_cmd(0x00, bytes([0x00])) == b"")
            check("PAGE readback=0",
                  c.read_cmd(0x00, 1) == bytes([0x00]))

            r = c.read_cmd(0x20, 1)       # VOUT_MODE
            check("VOUT_MODE = 0x40 (direct)", r == bytes([0x40]),
                  f"got {r.hex() if r else 'NAK'}")

            # ---- VOUT_COMMAND page 0 ----
            r = c.read_cmd(0x21, 2)
            check("VOUT_COMMAND page0 = 1000",
                  r == struct.pack(">H", 1000),
                  f"got {r.hex() if r else 'NAK'}")

            # ---- switch to page 1 ----
            check("PAGE write=1",
                  c.write_cmd(0x00, bytes([0x01])) == b"")
            r = c.read_cmd(0x21, 2)
            check("VOUT_COMMAND page1 = 1200",
                  r == struct.pack(">H", 1200),
                  f"got {r.hex() if r else 'NAK'}")

            # ---- live sensor reads ----
            r = c.read_cmd(0x8B, 2)       # READ_VOUT page 1
            check("READ_VOUT page1 ~1200",
                  r is not None and abs(struct.unpack(">H", r)[0] - 1200) <= 5,
                  f"got {r.hex() if r else 'NAK'}")

            check("PAGE write=0 (back to page0)",
                  c.write_cmd(0x00, bytes([0x00])) == b"")
            r = c.read_cmd(0x8D, 2)      # READ_TEMP_1 (page 0)
            check("READ_TEMP_1 ~25 (raw, direct R=0)",
                  r is not None and abs(struct.unpack(">H", r)[0] - 25) <= 3,
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x88, 2)      # READ_VIN
            check("READ_VIN ~12000 mV",
                  r is not None and abs(struct.unpack(">H", r)[0] - 1200) <= 5,
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x89, 2)      # READ_IIN
            check("READ_IIN ~3000 mA",
                  r is not None and abs(struct.unpack(">H", r)[0] - 300) <= 10,
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x8C, 2)      # READ_IOUT
            check("READ_IOUT ~15000 mA",
                  r is not None and abs(struct.unpack(">H", r)[0] - 150) <= 20,
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x8E, 2)      # READ_TEMP_2
            check("READ_TEMP_2 ~35 (raw)",
                  r is not None and abs(struct.unpack(">H", r)[0] - 35) <= 3,
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x8F, 2)      # READ_TEMP_3
            check("READ_TEMP_3 ~40 (raw)",
                  r is not None and abs(struct.unpack(">H", r)[0] - 40) <= 3,
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x96, 2)      # READ_POUT
            check("READ_POUT ~15 W (raw)",
                  r is not None and abs(struct.unpack(">H", r)[0] - 15) <= 5,
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x97, 2)      # READ_PIN
            check("READ_PIN ~18 W (raw)",
                  r is not None and abs(struct.unpack(">H", r)[0] - 18) <= 5,
                  f"got {r.hex() if r else 'NAK'}")

            # ---- manufacturer-specific VMON ----
            r = c.read_cmd(0xC8, 2)      # RAA_DMPVR2_READ_VMON
            check("READ_VMON = 0",
                  r == struct.pack(">H", 0),
                  f"got {r.hex() if r else 'NAK'}")

            # ---- block reads ----
            # For block reads, the SMBus block protocol returns
            # [length byte][N data bytes] on the wire; the i2c-custom
            # proxy returns them as a single recv reply.
            r = c.read_cmd(0x99, 4)      # MFR_ID
            check("MFR_ID = REN",
                  r == bytes([3]) + b"REN",
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0x9A, 9)      # MFR_MODEL
            check("MFR_MODEL = ISL69260",
                  r == bytes([8]) + b"ISL69260",
                  f"got {r.hex() if r else 'NAK'}")

            r = c.read_cmd(0xAD, 9)      # IC_DEVICE_ID
            check("IC_DEVICE_ID = ISL69260",
                  r == bytes([8]) + b"ISL69260",
                  f"got {r.hex() if r else 'NAK'}")

            # ---- status registers clear (CLEAR_FAULTS) ----
            # Write a nonzero temp-status, clear faults, check cleared.
            check("write STATUS_TEMPERATURE (probe writes)",
                  c.write_cmd(0x7D, bytes([0x01])) == b"")
            r = c.read_cmd(0x7D, 1)
            check("STATUS_TEMPERATURE readback = 0x01",
                  r == bytes([0x01]),
                  f"got {r.hex() if r else 'NAK'}")

            check("CLEAR_FAULTS write",
                  c.write_cmd(0x03) == b"")
            r = c.read_cmd(0x7D, 1)
            check("STATUS_TEMPERATURE after CLEAR = 0",
                  r == bytes([0x00]),
                  f"got {r.hex() if r else 'NAK'}")

        finally:
            sock.close()
            listener.close()
            try:
                os.unlink(path)
            except OSError:
                pass

        print()
        if fails:
            print(f"FAILURES ({len(fails)}):")
            for f in fails:
                print(f"  - {f}")
            return 1
        print("selftest OK")
        return 0


if __name__ == "__main__":
    sys.exit(main())
