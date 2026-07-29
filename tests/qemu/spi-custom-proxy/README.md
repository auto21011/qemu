# spi-custom QEMU device + python proxy

`spi-custom` is a QEMU SSI peripheral whose behavior is implemented by an
external Python process connected through a unix-socket chardev. It is the
SPI sibling of the `i2c-custom` device (`tests/qemu/i2c-custom-proxy/`).

The device appears in QEMU as a generic `-device spi-custom` peripheral
attached to any `SSIBus` (e.g. Aspeed SMC FMC, AST2600 `spi[N]`), and
proxies every full-duplex SPI byte over the socket to the Python side.

## Files

| File | Role |
|---|---|
| `hw/sensor/spi-custom.c` | QEMU SSI peripheral (C) |
| `include/hw/sensor/spi-custom.h` | Public header, state struct, opcodes |
| `tests/qemu/spi-custom-proxy/spi_custom_proxy.py` | Proxy server + `Device` ABC |
| `tests/qemu/spi-custom-proxy/m25p80_stub_model.py` | Reference model registered as `M25P80_STUB` |
| `tests/qemu/spi-custom-proxy/selftest.py` | In-process self-test |
| `docs/superpowers/specs/2026-07-30-spi-custom-design.md` | Design rationale |

## Wire protocol

Request (QEMU → proxy), 5 bytes:
```
[0] opcode      1=XFER  2=ASSERT_CS  3=RELEASE_CS  4=PING
[1] tx_len      0 or 1 (1 only for XFER)
[2] rx_len_max  0 or 1
[3] pad         0
[4] byte        master→slave byte (only for XFER)
```

Reply (proxy → QEMU), 2..3 bytes:
```
[0..1] rx_len  big-endian u16; 0xffff = NAK
[2]    byte    slave→master byte (only for XFER, rx_len=1)
```

`XFER` is one synchronous round-trip per SPI byte; `ASSERT_CS` /
`RELEASE_CS` bracket a transaction on CS edges; `PING` is the realize-time
probe.

## Authoring a Python SPI device

Subclass `Device` from `spi_custom_proxy` and implement:

```python
from spi_custom_proxy import Device, register_device

class MySensorModel(Device):
    def __init__(self, **_):
        self.state = WAIT_CMD

    def transfer(self, byte_in: int) -> int:
        # Self-contained byte-a-time state machine.
        ...

    def assert_cs(self) -> None:
        self.state = WAIT_CMD

    def release_cs(self) -> None:
        pass

    def ping(self) -> None:
        pass

register_device("MY_SENSOR", MySensorModel)
```

Save as `my_sensor_model.py` in this directory; the proxy auto-imports
every sibling `*_model.py` on startup so the device is immediately
selectable without editing `main()`.

## Running

```bash
# 1. Start a chardev unix socket:
#    -chardev socket,id=spibk,host=/tmp/spi.sock,server=on,wait=off

# 2. Run the Python side:
python3 spi_custom_proxy.py /tmp/spi.sock M25P80_STUB

# 3. Launch QEMU with spi-custom on an SSI bus, e.g. AST2600 FMC:
qemu-system-aarch64 -M ast2600-evb \
    -chardev socket,id=spibk,path=/tmp/spi.sock,server=on,wait=off \
    -device spi-custom,chardev=spibk,bus=...,cs=0
```

The SSI bus name/path for Aspeed SMC controllers is created with a `NULL`
name in `hw/ssi/aspeed_smc.c` — see the `-device help` output and QOM
`/machine/...` for the bus path. For board-mandated flashes (m25p80
hardcoded in `aspeed_board_init_flashes`) the driver may not coexist
cleanly for the same CS index; choose `cs=` accordingly.

## Self-test

```bash
python3 tests/qemu/spi-custom-proxy/selftest.py
# expect: selftest OK
```

This spawns the proxy in a background thread and exercises PING /
ASSERT_CS / XFER(status reg) / RELEASE_CS / unknown-opcode-NAK without
spawning QEMU.
