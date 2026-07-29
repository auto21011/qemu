# i2c-custom proxy

Python backend for the QEMU `i2c-custom` device. The device (in
`hw/sensor/i2c-custom.c`) translates guest i2c transactions into the
binary frame protocol documented in
`docs/superpowers/specs/2026-07-29-i2c-custom-design.md`. This directory
contains:

- `i2c_custom_proxy.py` — the unix-socket server speaking the wire
  protocol and dispatching to a `Device` subclass.
- `tmp117_model.py` — a reference `Device` implementing the TI TMP117
  16-bit I2C temperature sensor register map.
- `selftest.py` — a protocol-level self-test that exercises the proxy +
  TMP117 model end-to-end. Runs entirely in-process, no QEMU required.

## Quick start

```bash
# Terminal 1: start the proxy listening on a unix socket
python3 tests/qemu/i2c-custom-proxy/i2c_custom_proxy.py /tmp/i2c.sock TMP117
```

```bash
# Terminal 2: launch QEMU and let guest firmware talk to i2c bus 8
qemu-system-aarch64 -M ast2600-evb \
  -chardev socket,id=ic0,path=/tmp/i2c.sock,server=on,wait=off \
  -device i2c-custom,address=0x48,chardev=ic0,bus=aspeed.i2c.bus8 \
  -serial mon:stdio -nographic
```

## Running the self-test

```bash
python3 tests/qemu/i2c-custom-proxy/selftest.py
# prints "selftest OK" on success, exits non-zero on failure
```

## Writing your own device model

Subclass `Device` from `i2c_custom_proxy.py` and register it with
`register_device(name, cls)`. Two methods to implement:

```python
from i2c_custom_proxy import Device, register_device

class MySensor(Device):
    def send(self, address, payload: bytes) -> int:
        # Master write. Return the number of bytes you would like to
        # immediately emit (typically 0; reads are answered lazily by recv).
        return 0
    def recv(self, address, rx_len_max: int) -> bytes:
        # Master read: return up to rx_len_max bytes.
        return b"\x00" * rx_len_max

register_device("MYSENSOR", MySensor)
```

Then run:

```bash
python3 -c "from i2c_custom_proxy import main, register_device; \
import my_model; register_device('MYSENSOR', my_model.MySensor); \
import sys; sys.exit(main(['prog', '/tmp/i2c.sock', 'MYSENSOR']))"
```

(or add an `import my_model` to `i2c_custom_proxy.main`).

## Wire protocol

See the design doc and `include/hw/sensor/i2c-custom.h`. Summary, all
big-endian:

| Direction | Bytes | Meaning |
|-----------|-------|---------|
| QEMU->proxy | `[op][tx_len][rx_len_max][addr]` + `tx_len` payload bytes | header + tx bytes |
| proxy->QEMU | `[rx_len_hi][rx_len_lo]` + `rx_len` payload bytes | reply; `rx_len=0xffff` = NAK |

`op` is one of `1=SEND`, `2=RECV`, `3=PING`.
