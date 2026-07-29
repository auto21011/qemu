#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# selftest.py — protocol-level self-test for the QEMU spi-custom proxy.
#
# Drives a M25P80_STUB model through the documented wire protocol WITHOUT
# spawning a separate process. Spawns the proxy server in a background
# thread and exercises:
#   1. PING handshake            -> expect rx_len == 0
#   2. ASSERT_CS                  -> expect rx_len == 0
#   3. SEND(0x05); RECV(0x05)     -> status=0x00, then 0x00 again
#   4. RELEASE_CS                 -> expect rx_len == 0
#   5. Unknown opcode             -> expect NAK (rx_len=0xffff)
#
# Exits 0 on pass, non-zero on any failure. Runs entirely in-process.
# CI workflow invokes this during the smoke-test step.

import os
import socket
import struct
import sys
import tempfile
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import spi_custom_proxy as proxy  # noqa: E402
import m25p80_stub_model  # noqa: E402  (registers M25P80_STUB on import)


OP_XFER = proxy.OP_XFER
OP_ASSERT_CS = proxy.OP_ASSERT_CS
OP_RELEASE_CS = proxy.OP_RELEASE_CS
OP_PING = proxy.OP_PING
HEADER_FMT = proxy.HEADER_FMT
HEADER_LEN = proxy.HEADER_LEN
REPLY_LEN_FMT = proxy.REPLY_LEN_FMT
NAK_RX_LEN = proxy.NAK_RX_LEN


class SpiProxySmokeClient:
    def __init__(self, client_sock: socket.socket):
        self.s = client_sock

    def _request(self, opcode, payload=b'', rx_max=0):
        assert len(payload) <= 1
        self.s.sendall(struct.pack(HEADER_FMT, opcode, len(payload), rx_max, 0) + payload)

    def _reply(self):
        rlen_bytes = self.s.recv(2)
        if len(rlen_bytes) != 2:
            raise AssertionError("short reply length read")
        rlen = struct.unpack(REPLY_LEN_FMT, rlen_bytes)[0]
        if rlen == NAK_RX_LEN:
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

    def ping(self):
        self._request(OP_PING, b"", 0)
        return self._reply()

    def xfer(self, byte_in):
        self._request(OP_XFER, bytes([byte_in]), 1)
        rx = self._reply()
        return rx

    def assert_cs(self):
        self._request(OP_ASSERT_CS, b"", 0)
        return self._reply()

    def release_cs(self):
        self._request(OP_RELEASE_CS, b"", 0)
        return self._reply()


def fail(msg):
    sys.stderr.write(f"FAIL: {msg}\n")
    sys.exit(1)


def run_proxy_in_thread(path, device):
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


def main():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "spi-custom-test.sock")
        device = proxy.DEVICE_REGISTRY["M25P80_STUB"]()

        try:
            t, listener = run_proxy_in_thread(path, device)
        except Exception as e:
            fail(f"proxy server start error: {e}")

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        deadline_start = os.times()[4]
        connected = False
        while os.times()[4] - deadline_start < 5.0:
            try:
                sock.connect(path)
                connected = True
                break
            except (ConnectionRefusedError, FileNotFoundError):
                continue
        if not connected:
            fail("could not connect to proxy within deadline")

        c = SpiProxySmokeClient(sock)
        try:
            # 1) PING
            if c.ping() != b"":
                fail("PING returned non-empty rx")

            # 2) ASSERT_CS
            if c.assert_cs() != b"":
                fail("ASSERT_CS returned non-empty rx")

            # 3) Read Status Register: cmd byte 0x05 -> dummy 0xFF,
            #    then status byte 0x00.
            rx1 = c.xfer(0x05)
            if rx1 != b"\xff":
                fail(f"xfer(0x05) cmd-ack expected 0xFF, got {rx1.hex() if rx else 'NAK'}")
            rx2 = c.xfer(0x00)
            if rx2 != b"\x00":
                fail(f"xfer(0x00) status-read expected 0x00, got {rx2.hex() if rx2 else 'NAK'}")

            # 4) RELEASE_CS
            if c.release_cs() != b"":
                fail("RELEASE_CS returned non-empty rx")

            # 5) Unknown opcode -> NAK (use opcode 0xEE).
            sock.sendall(struct.pack(HEADER_FMT, 0xEE, 0, 0, 0))
            rlen = struct.unpack(REPLY_LEN_FMT, sock.recv(2))[0]
            if rlen != NAK_RX_LEN:
                fail(f"unknown opcode did not NAK; rlen={rlen:#x}")
        finally:
            sock.close()
            listener.close()
            try:
                os.unlink(path)
            except OSError:
                pass

        sys.stderr.write("selftest OK\n")
        return 0


if __name__ == "__main__":
    sys.exit(main())
