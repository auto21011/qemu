#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# selftest.py — protocol-level self-test for the QEMU i2c-custom proxy.
#
# Drives a TMP117 model through the documented wire protocol WITHOUT
# spawning a separate process. Spawns the proxy server in a background
# thread (or in-process via direct socket client) and exercises:
#   1. PING handshake   -> expect rx_len == 0
#   2. SEND(0x00)         -> ptr=TEMP
#   3. RECV(max=64)       -> expect 2 bytes (big-endian TMP117 temperature)
#   4. SEND(0x01)         -> ptr=ID
#   5. RECV              -> expect b"\x01\x17"  (TMP117 ID register)
#   6. SEND(0x06, 0xAA, 0xBB)  -> writes config
#   7. SEND(0x06); RECV        -> expect b"\xaa\xbb"
#   8. An unsupported opcode  -> expect NAK (rx_len=0xffff)
#
# Exits 0 on pass, non-zero on any failure. Runs entirely in the same
# process; no QEMU is needed. The CI workflow invokes this during the
# smoke-test step.

import os
import socket
import struct
import sys
import tempfile
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import i2c_custom_proxy as proxy  # noqa: E402
import tmp117_model  # noqa: E402  (registers TMP117 on import failures raise)


OP_SEND = proxy.OP_SEND
OP_RECV = proxy.OP_RECV
OP_PING = proxy.OP_PING
HEADER_FMT = proxy.HEADER_FMT
HEADER_LEN = proxy.HEADER_LEN
REPLY_LEN_FMT = proxy.REPLY_LEN_FMT
NAK_RX_LEN = proxy.NAK_RX_LEN
MAX_XFER = proxy.MAX_XFER


class Tmp117SmokeClient:
    """Minimal client speaking the documented wire protocol."""

    def __init__(self, client_sock: socket.socket):
        self.s = client_sock
        self.address = 0x48

    def _send_request(self, opcode: int, payload: bytes, rx_max: int) -> None:
        assert len(payload) <= MAX_XFER
        header = struct.pack(HEADER_FMT, opcode, len(payload), rx_max,
                             self.address)
        self.s.sendall(header + payload)

    def _recv_reply(self) -> bytes:
        # 2-byte length prefix
        rlen_bytes = self.s.recv(2)
        if len(rlen_bytes) != 2:
            raise AssertionError("short reply length read")
        rlen = struct.unpack(REPLY_LEN_FMT, rlen_bytes)[0]
        if rlen == NAK_RX_LEN:
            return None   # sentry for NAK
        if rlen == 0:
            return b""
        # Read exactly rlen bytes
        got = bytearray()
        while len(got) < rlen:
            chunk = self.s.recv(rlen - len(got))
            if not chunk:
                raise AssertionError("short payload read")
            got.extend(chunk)
        return bytes(got)

    def ping(self) -> bool:
        self._send_request(OP_PING, b"", 0)
        rx = self._recv_reply()
        return rx == b""

    def send(self, payload: bytes) -> bool:
        self._send_request(OP_SEND, payload, 0)
        rx = self._recv_reply()
        return rx == b""

    def recv(self, rx_max: int = 64) -> bytes:
        self._send_request(OP_RECV, b"", rx_max)
        return self._recv_reply()


def run_proxy_in_thread(path: str, device):
    """Start the proxy server in a background thread blocking on accept."""
    listener = proxy.make_listener(path)  # binds immediately here
    server_ready = threading.Event()

    def _run():
        listener.listen(8)
        server_ready.set()
        try:
            proxy.serve(listener, device)
        except OSError:
            # closed underneath us during teardown; treat as stop.
            pass

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    server_ready.wait(timeout=2.0)
    return t, listener


def fail(msg: str) -> None:
    sys.stderr.write(f"FAIL: {msg}\n")
    sys.exit(1)


def main() -> int:
    # In-process client -> proxy server (threaded) -> TMP117 device.
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "i2c-custom.sock")

        # Sanity device setup
        device = proxy.DEVICE_REGISTRY["TMP117"]()

        # Bind the listener ourselves; our serve() loop expects a bound
        # (or already-listening) socket. make_listener also removes any
        # stale path; that's fine inside a tempdir.
        try:
            t, listener = run_proxy_in_thread(path, device)
        except Exception as e:
            fail(f"proxy server start error: {e}")

        # Small retry until connect succeeds (listen happens inside _run).
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        deadline = 5.0
        connected = False
        start = os.times()[4]
        while os.times()[4] - start < deadline:
            try:
                sock.connect(path)
                connected = True
                break
            except (ConnectionRefusedError, FileNotFoundError):
                continue
        if not connected:
            fail("could not connect to proxy within deadline")

        c = Tmp117SmokeClient(sock)
        try:
            # 1) PING
            if not c.ping():
                fail("PING returned non-empty rx")

            # 2-3) Read temperature register: SEND(ptr=0x00), RECV(max=64).
            if not c.send(bytes([0x00])):
                fail("SEND(ptr=0x00) reply not 0 bytes")
            rx_temp = c.recv(64)
            if rx_temp is None:
                fail("RECV temperature returned NAK")
            if len(rx_temp) != 2:
                fail(f"expected 2-byte temperature, got len={len(rx_temp)}")

            # 4-5) Read ID register: SEND(ptr=0x01), RECV.
            if not c.send(bytes([0x01])):
                fail("SEND(ptr=0x01) reply not 0 bytes")
            rx_id = c.recv(64)
            if rx_id != b"\x01\x17":
                fail(f"ID register mismatch: got {rx_id.hex()}, "
                     f"expected 0117")

            # 6) Write config: SEND(ptr=0x06, value=0xAABB).
            if not c.send(bytes([0x06]) + bytes([0xAA, 0xBB])):
                fail("SEND(0x06, AA, BB) reply not 0 bytes")

            # 7) Read config back.
            if not c.send(bytes([0x06])):
                fail("SEND(ptr=0x06) reply not 0 bytes")
            rx_cfg = c.recv(64)
            if rx_cfg != b"\xaa\xbb":
                fail(f"config readback mismatch: got {rx_cfg.hex()}, "
                     f"expected aabb")

            # 8) Unknown opcode -> expect NAK.
            # Use opcode 0xEE (none defined uses that) and rx_max=0.
            header = struct.pack(HEADER_FMT, 0xEE, 0, 0, 0x48)
            sock.sendall(header)
            rlen = struct.unpack(REPLY_LEN_FMT, sock.recv(2))[0]
            if rlen != NAK_RX_LEN:
                fail(f"unknown opcode did not NAK; rlen={rlen:#x}, "
                     f"expected 0xffff")
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
