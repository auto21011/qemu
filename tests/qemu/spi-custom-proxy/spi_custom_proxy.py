#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# spi_custom_proxy.py
#
# Server-side companion for the QEMU `spi-custom` device. Listens on a
# unix socket, decodes the binary frame protocol, and dispatches each
# SPI transaction to a Device subclass instance.
#
# Protocol (4-byte header, up to 1-byte optional payload):
#
#   Request (QEMU -> proxy):
#     [0]  opcode     (1=XFER, 2=ASSERT_CS, 3=RELEASE_CS, 4=PING)
#     [1]  tx_len     (0 or 1; 1 only for XFER)
#     [2]  rx_len_max (0 or 1; 1 only for XFER)
#     [3]  pad        (reserved, always 0)
#     [4]  byte       (only if tx_len==1: the master->slave byte)
#
#   Reply (proxy -> QEMU):
#     [0..1] rx_len (big-endian u16); 0xffff = NAK
#     [2]    byte   (only if rx_len==1: the slave->master byte)
#
# Device subclasses implement:
#   transfer(byte_in: int) -> int          # return byte_out
#   assert_cs() -> None                    # CS active-low asserted
#   release_cs() -> None                   # CS deasserted
#   ping() -> None                         # optional handshake
#
# Usage:
#   python3 spi_custom_proxy.py <socket_path> <device_name>
#
# This file has zero hard deps outside the Python stdlib.

import os
import socket
import struct
import sys
import importlib
from abc import ABC, abstractmethod


def _own_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


_HERE = _own_dir()
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

if __name__ == "__main__" and "spi_custom_proxy" not in sys.modules:
    sys.modules["spi_custom_proxy"] = sys.modules["__main__"]

# Protocol constants mirroring include/hw/sensor/spi-custom.h

OP_XFER      = 1
OP_ASSERT_CS = 2
OP_RELEASE_CS = 3
OP_PING      = 4

HEADER_FMT = "!BBBB"   # opcode, tx_len, rx_len_max, pad
HEADER_LEN = struct.calcsize(HEADER_FMT)
REPLY_LEN_FMT = "!H"
REPLY_LEN_LEN = struct.calcsize(REPLY_LEN_FMT)
NAK_RX_LEN = 0xFFFF


class Device(ABC):
    """Base class for an SPI device model driven by spi_custom_proxy."""

    @abstractmethod
    def transfer(self, byte_in: int, /) -> int:
        """Full-duplex SPI word.

        Called once per SPI clock with the master->slave byte, must
        return the slave->master byte (low 8 bits). The device's
        internal state machine processes `byte_in` and produces a
        response, exactly like a real SPI peripheral.
        """
        return 0

    def assert_cs(self, /) -> None:
        """CS line asserted (active low in this model). Called on the
        HIGH->LOW edge of CS#."""

    def release_cs(self, /) -> None:
        """CS line deasserted (LOW->HIGH edge)."""

    def ping(self, /) -> None:
        """Optional handshake; default no-op. Called once on QEMU realize."""


def recv_exact(sock: socket.socket, want: int, label: str) -> bytes:
    """Read exactly `want` bytes or raise EOFError."""
    got = bytearray()
    while len(got) < want:
        chunk = sock.recv(want - len(got))
        if not chunk:
            raise EOFError(f"{label}: peer closed after {len(got)} of {want}")
        got.extend(chunk)
    return bytes(got)


def reply(sock: socket.socket, *bytes_list) -> None:
    """Send a reply frame (rx_len + payload)."""
    payload = b"".join(bytes_list)
    rx_len = len(payload)
    sock.sendall(struct.pack(REPLY_LEN_FMT, rx_len) + payload)


def reply_nak(sock: socket.socket) -> None:
    sock.sendall(struct.pack(REPLY_LEN_FMT, NAK_RX_LEN))


def handle_one_request(conn: socket.socket, device: Device) -> bool:
    """Read & dispatch exactly one request. Returns False on EOF."""
    try:
        hdr = recv_exact(conn, HEADER_LEN, "header")
    except EOFError:
        return False
    opcode, tx_len, rx_len_max, _pad = struct.unpack(HEADER_FMT, hdr)

    payload = b""
    if tx_len:
        try:
            payload = recv_exact(conn, tx_len, "payload")
        except EOFError:
            return False

    if opcode == OP_PING:
        try:
            device.ping()
        except Exception:
            reply_nak(conn)
            return True
        reply(conn, b"")
        return True

    if opcode == OP_XFER:
        byte_in = payload[0] if payload else 0
        try:
            byte_out = device.transfer(byte_in)
        except Exception as e:
            sys.stderr.write(f"[spi-custom-proxy] device.transfer raised: {e}\n")
            reply_nak(conn)
            return True
        reply(conn, bytes([byte_out & 0xFF]))
        return True

    if opcode == OP_ASSERT_CS:
        try:
            device.assert_cs()
        except Exception as e:
            sys.stderr.write(f"[spi-custom-proxy] device.assert_cs raised: {e}\n")
            reply_nak(conn)
            return True
        reply(conn, b"")
        return True

    if opcode == OP_RELEASE_CS:
        try:
            device.release_cs()
        except Exception as e:
            sys.stderr.write(f"[spi-custom-proxy] device.release_cs raised: {e}\n")
            reply_nak(conn)
            return True
        reply(conn, b"")
        return True

    # Unknown opcode.
    reply_nak(conn)
    return True


def make_listener(path: str) -> socket.socket:
    """Create a listening UNIX socket (proxy is server, QEMU is client).

    Use when QEMU is started with ``server=off``.  Start the proxy first,
    then start QEMU which connects to the socket.
    """
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.bind(path)
    return s


def connect_sock(path: str, device: Device) -> socket.socket:
    """Connect to an existing listening UNIX socket (proxy is client).

    Use when QEMU is started with ``server=on`` (QEMU listens).  Start
    QEMU first, then run the proxy with ``--connect``.
    """
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    import time as _time
    deadline = 15.0
    start = _time.monotonic()
    while True:
        try:
            s.connect(path)
            break
        except (ConnectionRefusedError, FileNotFoundError):
            if _time.monotonic() - start >= deadline:
                raise
            _time.sleep(0.1)
    sys.stderr.write(
        f"[spi-custom-proxy] connected to {path} with "
        f"device={device.__class__.__name__}\n"
    )
    sys.stderr.flush()
    return s


def serve(listener: socket.socket, device: Device) -> None:
    listener.listen(8)
    listener.settimeout(None)
    sys.stderr.write(
        f"[spi-custom-proxy] listening on {listener.getsockname()} with "
        f"device={device.__class__.__name__}\n"
    )
    sys.stderr.flush()
    while True:
        conn, _ = listener.accept()
        try:
            while handle_one_request(conn, device):
                pass
        except Exception:
            pass
        finally:
            conn.close()


def serve_conn(conn: socket.socket, device: Device) -> None:
    try:
        while handle_one_request(conn, device):
            pass
    except (ConnectionResetError, BrokenPipeError):
        pass


# Registry populated lazily.
DEVICE_REGISTRY = {}


def register_device(name: str, cls):
    DEVICE_REGISTRY[name] = cls


def _auto_import_models() -> None:
    import glob as _glob
    for _m in sorted(_glob.glob(os.path.join(_HERE, "*_model.py"))):
        _modname = os.path.splitext(os.path.basename(_m))[0]
        if _modname in sys.modules:
            continue
        try:
            importlib.import_module(_modname)
        except Exception as _e:
            sys.stderr.write(
                f"failed to import bundled model {_modname} "
                f"({type(_e).__name__}: {_e}); registered so far: "
                f"{' '.join(DEVICE_REGISTRY) or '(none)'}\n"
            )


def main(argv: list[str]) -> int:
    args = list(argv)
    connect_mode = "--connect" in args
    if connect_mode:
        args.remove("--connect")

    if len(args) < 3:
        sys.stderr.write(
            f"Usage: {args[0]} <socket_path> <device_name> [--connect]\n"
            f"  --connect  Connect to QEMU server socket (QEMU server=on)\n"
            f"  (default)  Create listener socket (QEMU server=off)\n"
            f"Registered devices: {' '.join(DEVICE_REGISTRY) or '(none)'}\n"
        )
        return 2
    path, name = args[1], args[2]
    _auto_import_models()

    if name not in DEVICE_REGISTRY:
        sys.stderr.write(
            f"Unknown device '{name}'. Registered: "
            f"{' '.join(DEVICE_REGISTRY) or '(none)'}\n"
        )
        return 2
    device = DEVICE_REGISTRY[name]()

    if connect_mode:
        conn = connect_sock(path, device)
        try:
            serve_conn(conn, device)
        except KeyboardInterrupt:
            pass
        finally:
            conn.close()
    else:
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
