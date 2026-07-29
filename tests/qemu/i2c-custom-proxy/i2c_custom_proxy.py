#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# i2c_custom_proxy.py
#
# Server-side companion for the QEMU `i2c-custom` device. Listens on a
# unix socket, decodes the binary frame protocol documented in
# docs/superpowers/specs/2026-07-29-i2c-custom-design.md, and dispatches
# each transaction to a Device subclass instance.
#
# Usage:
#   i2c_custom_proxy.py <socket_path> <device_name>
#
# where <device_name> selects a class registered in DEVICE_REGISTRY below
# (for example "TMP117", which uses tmp117_model.TMP117Model).
#
# This file has zero hard deps outside the Python stdlib so it can ship as
# a reference example inside the QEMU source tree.

import os
import socket
import struct
import sys
from abc import ABC, abstractmethod

# Wire protocol constants — must mirror include/hw/sensor/i2c-custom.h
OP_SEND = 1
OP_RECV = 2
OP_PING = 3

HEADER_FMT = "!BBBB"   # opcode, tx_len, rx_len_max, address
HEADER_LEN = struct.calcsize(HEADER_FMT)
REPLY_LEN_FMT = "!H"
REPLY_LEN_LEN = struct.calcsize(REPLY_LEN_FMT)
NAK_RX_LEN = 0xFFFF
MAX_XFER = 64


class Device(ABC):
    """Base class for an i2c device model driven by i2c_custom_proxy."""

    @abstractmethod
    def send(self, address: int, payload: bytes) -> int:
        """Master write. Returns int rx_len the proxy should immediately
        emit (without doing a follow-up RECV); 0 means 'no replies yet'.
        Sockets are stateful per connection so this allows a device to
        pre-stage read replies for a subsequent RECV op. Default impl is
        fine for many devices (no-op, return 0)."""
        return 0

    @abstractmethod
    def recv(self, address: int, rx_len_max: int) -> bytes:
        """Master read. Return up to rx_len_max bytes (>=0). Length of
        the returned bytes is the rx_len sent back on the wire. Returning
        b'' is legal (means 0 rx bytes)."""

    def ping(self, address: int) -> None:
        """Optional handshake; default no-op. Called once on QEMU realize."""
        return None


def recv_exact(sock: socket.socket, want: int, label: str) -> bytes:
    """Read exactly `want` bytes or raise EOFError."""
    got = bytearray()
    while len(got) < want:
        chunk = sock.recv(want - len(got))
        if not chunk:
            raise EOFError(f"{label}: peer closed after {len(got)} of {want}")
        got.extend(chunk)
    return bytes(got)


def reply(sock: socket.socket, rx: bytes) -> None:
    """Send a reply frame (rx_len-big-endian + payload)."""
    rx_len = len(rx)
    if rx_len > MAX_XFER:
        # Defensive: device misbehaved. NAK so QEMU notices.
        sock.sendall(struct.pack(REPLY_LEN_FMT, NAK_RX_LEN))
        return
    sock.sendall(struct.pack(REPLY_LEN_FMT, rx_len) + rx)


def reply_nak(sock: socket.socket) -> None:
    sock.sendall(struct.pack(REPLY_LEN_FMT, NAK_RX_LEN))


def handle_one_request(conn: socket.socket, device: Device) -> bool:
    """Read & dispatch exactly one request. Returns False on EOF."""
    try:
        hdr = recv_exact(conn, HEADER_LEN, "header")
    except EOFError:
        return False
    opcode, tx_len, rx_len_max, address = struct.unpack(HEADER_FMT, hdr)

    payload = b""
    if tx_len:
        payload = recv_exact(conn, tx_len, "payload")

    if opcode == OP_PING:
        try:
            device.ping(address)
        except Exception:
            reply_nak(conn)
            return True
        reply(conn, b"")
        return True

    if opcode == OP_SEND:
        try:
            device.send(address, payload)
        except Exception as e:
            sys.stderr.write(f"[i2c-custom-proxy] device.send raised: {e}\n")
            reply_nak(conn)
            return True
        # We deliberately do NOT pre-stage any rx for follow-up RECVs
        # here; recv() is called lazily by RECV. Devices that want a
        # register-write to side-effect a staged read should store the
        # pending reply in self.
        reply(conn, b"")
        return True

    if opcode == OP_RECV:
        if rx_len_max == 0:
            reply(conn, b"")
            return True
        try:
            rx = device.recv(address, rx_len_max)
        except Exception as e:
            sys.stderr.write(f"[i2c-custom-proxy] device.recv raised: {e}\n")
            reply_nak(conn)
            return True
        if not isinstance(rx, (bytes, bytearray)):
            sys.stderr.write(
                "[i2c-custom-proxy] device.recv returned non-bytes\n"
            )
            reply_nak(conn)
            return True
        reply(conn, bytes(rx))
        return True

    # Unknown opcode.
    reply_nak(conn)
    return True


def serve(listener: socket.socket, device: Device) -> None:
    listener.listen(8)
    listener.settimeout(None)
    sys.stderr.write(
        f"[i2c-custom-proxy] listening on {listener.getsockname()} with "
        f"device={device.__class__.__name__}\n"
    )
    sys.stderr.flush()
    while True:
        conn, _ = listener.accept()
        try:
            while handle_one_request(conn, device):
                pass
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()


def make_listener(path: str) -> socket.socket:
    if os.path.exists(path):
        os.unlink(path)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.bind(path)
    return s


# Registry populated lazily so importing just this file is harmless.
DEVICE_REGISTRY = {}


def register_device(name: str, cls):
    DEVICE_REGISTRY[name] = cls


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        sys.stderr.write(
            f"Usage: {argv[0]} <socket_path> <device_name>\n"
            f"Registered devices: {' '.join(DEVICE_REGISTRY) or '(none)'}\n"
        )
        return 2
    path, name = argv[1], argv[2]
    # Import the example model so it self-registers; if a user later adds
    # their own model module they can import it from here or implement
    # their own entry point wrapper.
    import tmp117_model  # noqa: F401  (registers "TMP117")

    if name not in DEVICE_REGISTRY:
        sys.stderr.write(
            f"Unknown device '{name}'. Registered: "
            f"{' '.join(DEVICE_REGISTRY) or '(none)'}\n"
        )
        return 2
    device = DEVICE_REGISTRY[name]()
    listener = make_listener(path)
    try:
        serve(listener, device)
    except KeyboardInterrupt:
        pass
    finally:
        listener.close()
        try:
            os.unlink(path)
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
