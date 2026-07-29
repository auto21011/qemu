# `i2c-custom` device and Python proxy backend

**Status**: approved (2026-07-29)
**Owner**: auto2111
**Target machine**: `ast2600-evb` (also usable on any QEMU machine exposing an i2c bus)

## 1. Goal

Allow `ast2600-evb` (and any other QEMU machine exposing an i2c bus) to attach an
arbitrary guest-visible i2c slave device whose behavior is implemented by an
external process (typically Python) connected to QEMU via a unix-socket
`chardev`. This lets a developer model real i2c peripherals (sensors, EEPROMs,
PMICs, etc.) outside the QEMU C source tree, edit the model in Python, and have
the guest BMC firmware talk to it as if it were wired onto a real i2c bus.

## 2. Non-goals

- No new machine-level switch; the device is attached purely through the
  generic `-device` mechanism.
- No changes to `aspeed_ast2600_evb.c` (the existing fixed `tmp105` /
  `smbus_eeprom` wiring is preserved).
- Not a generic bus-master proxy (the device is a slave only).
- No streaming / async i2c (`send_async`); only the synchronous
  `send`/`recv`/`event` vtable slots are used.

## 3. Architecture

```
        guest (BMC firmware, e.g. ast2600-evb)
                       |
                       | MMIO read/write to aspeed_i2c
                       v
        +-------------------------------+
        | aspeed_i2c controller (master)|
        +-------------------------------+
                       | i2c bus transactions I2C_START_*/send/recv/FINISH
                       v
        +-------------------------------+
        | i2c-custom (QEMU i2c slave)   |  hw/sensor/i2c-custom.c
        |  - encodes tx into 4B frames  |
        |  - decodes rx-len-prefixed rxs |
        +-------------------------------+
                       | binary frame protocol
                       v
        +-------------------------------+
        | CharFrontend (qdev chr prop)  |  chardev/char-fe.h
        | -server=on unix-socket        |
        +-------------------------------+
                       | AF_UNIX SOCK_STREAM
                       v
        +-------------------------------+
        | Python proxy script           |  tests/qemu/i2c-custom-proxy/
        | - parses frames               |        i2c_custom_proxy.py
        | - dispatches to a device      |
        |   model class (e.g. TMP117)   |
        | - replies rx bytes + rc       |
        +-------------------------------+
```

The device driver mirrors QEMU's existing socket-bridged pattern
`hw/ipmi/ipmi_bmc_extern.c`, but uses a far simpler length-prefixed binary
framing protocol instead of IPMI's escaping scheme.

## 4. Components

### 4.1 `i2c-custom` device (`hw/sensor/i2c-custom.c` + `include/hw/sensor/i2c-custom.h`)

**QOM type**: `i2c-custom`, parent `TYPE_I2C_SLAVE`.

**State struct** (`I2CCustomState`):
- `CharFrontend chr;` — backend connection.
- `I2CSlave i2c;` (QOM parent).
- `uint8_t tx_buf[64]; int tx_len;` — accumulated bytes for the current master
  write (so we can hand a single chunk to the proxy in one frame).
- `uint8_t rx_buf[64]; int rx_len; int rx_pos;` — pre-fetched bytes for the
  master-read.
- `bool in_send_txn; bool in_recv_txn;` — set on START_SEND / START_RECV and
  cleared on FINISH, used to decide when to issue a request to the backend.
- `bool drained (NACK-on-disconnect);` — true whenever `chr_event(CLOSED)`
  fires; reset on `chr_event(OPENED)`. While drained we never block on the
  backend and translate all reads to 0xFF and all sends to NAK.

**Qdev properties**:
- `DEFINE_PROP_CHR("chardev", I2CCustomState, chr)` — required.
- The i2c slave `address` property is inherited from `I2CSlave`.

**Vtable implementation**:

`i2c_custom_event(s, event)`:
- `I2C_START_SEND`: reset `tx_len` to 0, set `in_send_txn=true`.
- `I2C_START_RECV`: set `in_recv_txn=true`; issue
  `i2c_custom_request(s, OP_RECV, tx_len=0, rx_len=64)` and immediately read
  the rx payload into `rx_buf` so the first `recv()` call has data ready.
  Set `rx_pos=0` and `rx_len` accordingly; NAK on backend disconnected
  (drained) or any I/O error -> everything NAKs, stores nothing.
- `I2C_FINISH`: if `in_send_txn` was active and `tx_len>0`, flush the buffered
  tx to the backend via `i2c_custom_request(s, OP_SEND, tx_len, rx_len=0)`,
  reading and discarding any reply conformant to the rx-len header.
  Reset both `in_*_txn` flags and `tx_len`.
- `I2C_NACK`: no-op (state resets in FINISH).
- `I2C_START_SEND_ASYNC`: not used (we don't implement `send_async`).

`i2c_custom_send(s, byte)`: append `byte` to `tx_buf[tx_len++]` (cap at
sizeof `tx_buf`, returning NAK if overrun). Return 0 (ACK) when `in_send_txn`.

`i2c_custom_recv(s)`: if `in_recv_txn` and there are bytes left at
`rx_pos < rx_len`, return `rx_buf[rx_pos++]`. Otherwise return 0xFF (NACK
implied by master side; we leave reply zero-filled).

**Frame I/O helpers** (`i2c_custom_request`):
- Build header cheerfully: 4 bytes
  `[opcode][tx_len][rx_len_max][address]` (all uint8; tx_len/rx_len_max
  clamped to 64).
- If `tx_len>0`: append `tx_len` payload bytes.
- `qemu_chr_fe_write_all(&s->chr, hdr, 4 + tx_len)`.
- Read 2-byte reply length from the frontend using a blocking read loop
  (`qemu_chr_fe_read_all` is non-blocking in some backends; we therefore
  use a small wait loop with `g_usleep(1000)` up to a 1-second timeout and
  yield to keep the main loop responsive).
- If `rx_max_request` > 0: read that many bytes back into the supplied
  caller buffer.
- Set `drained=true` and propagate a NAK if at any point the backend is
  closed (write returns 0/-1 or read times out).

**Realize**: requires `chardev`; calls
`qemu_chr_fe_set_handlers(&s->chr, i2c_custom_can_receive,
i2c_custom_receive, i2c_custom_chr_event, NULL, s, NULL, true);` exactly as
`ipmi_bmc_extern.c` does.

**Chardev callbacks**:
- `i2c_custom_can_receive` returns 0 (we never accept unsolicited
  inbound bytes — replies are consumed synchronously from within
  `i2c_custom_request`).
- `i2c_custom_receive` is a no-op (defensive placeholder required by the
  set_handlers signature).
- `i2c_custom_chr_event` toggles `drained` on
  `CHR_EVENT_OPENED` / `CHR_EVENT_CLOSED`.

**Migration**: include a `VMSTATE_STRUCT_VPOINTER`/`VMSTATE_UINT8_ARRAY` to
preserve `tx_buf`/`tx_buf` length and the `in_send_txn/in_recv_txn` state,
matching the minimal `tmp105` VMState pattern. CharBackend pointers are not
migrated (QEMU matches `ipmi_bmc_extern` here).

**Trace events**: three small ones in `hw/sensor/trace-events`:
- `i2c_custom_event_op(uint8_t op, uint8_t addr)`
- `i2c_custom_send(int len, uint8_t first)`
- `i2c_custom_recv(int len, uint8_t first)`

### 4.2 Kconfig / meson

```kconfig
# hw/sensor/Kconfig
config I2C_CUSTOM
    bool
    depends on I2C
    default y if I2C_DEVICES
```

```meson
# hw/sensor/meson.build
system_ss.add(when: 'CONFIG_I2C_CUSTOM', if_true: files('i2c-custom.c'))
```

This follows the exact `TMP105` precedent. `CONFIG_I2C_DEVICES` is already
enabled by default on `aarch64-softmmu` (see
`configs/devices/arm-softmmu/default.mak` and ancestors), so the device will
ship built in for `qemu-system-aarch64` without further action.

### 4.3 Wire protocol

Frames (`<big endian byte stream, all bytes literal uint8>`):

`request` (QEMU -> proxy):
| offset | field        | notes                                            |
|--------|--------------|--------------------------------------------------|
| 0      | `opcode`     | 1=SEND, 2=RECV, 3=PING (handshake/resync)        |
| 1      | `tx_len`     | 0..64                                            |
| 2      | `rx_len_max` | 0..64                                            |
| 3      | `address`    | i2c slave address (informational)                |
| 4..    | `tx_len` payload bytes                                          |

`reply` (proxy -> QEMU):
| offset | field            | notes                                                |
|--------|------------------|------------------------------------------------------|
| 0..1   | `rx_len` (u16 BE)| number of rx bytes following; 0xffff means NAK       |
| 2..    | `rx_len` payload bytes                                                |

A NAK reply (`rx_len == 0xffff`) tells QEMU to drop the current transaction
with a NAK; a half-open / EOF / 1s read timeout sets `drained=true`.

### 4.4 Python proxy

`tests/qemu/i2c-custom-proxy/i2c_custom_proxy.py`:
- AF_UNIX SOCK_STREAM server that listens on a path given as argv[1].
- Loop: accept a connection, then repeatedly:
  1. read a 4-byte request header (handle EOF -> disconnect).
  2. read `tx_len` payload bytes.
  3. dispatch by opcode to a `Device` subclass instance passed in argv[2]:
     - `device.send(addr, payload)` -> int rx_len (0..64) to fill the
       caller's rx_buf for a follow-up RECV (we keep state per session).
     - `device.recv(addr, rx_len_max)` -> bytes to send back.
     - `device.ping()` -> no-op, used by tests.
- Returns the reply in the documented format.
- On exception, replies `rx_len == 0xffff`.

A reference device implementation `TMP117Model` is included in the same
directory (`tmp117_model.py`) and supports the standard TMP117 register
map (temperature register 0x00 returns a synthetic value, configuration at
0x06 is R/W, high/low limits at 0x02/0x03 etc.).

`tests/qemu/i2c-custom-proxy/selftest.py`:
- Launches the proxy on a unix socket, opens a client connection from within
  the same process, drives a TMP117 register read and a write, and asserts
  expected return values. Exits 0 on pass, non-zero on fail.

### 4.5 Workflow update (`.github/workflows/build-qemu-static.yml`)

Add the new source files to the `paths:` triggers and the `pull_request:` path
list. Build step is unchanged — `aarch64-softmmu` + `--static`. Smoke test now
also does:
- Verify `i2c-custom` device appears in `qemu-system-aarch64 -device help`:
  `./qemu-system-aarch64 -device help | grep -E 'i2c-custom' | head -2`.
- Spawn the Python proxy in the background, start the VM with a single
  i2c-custom instance and a tiny register read, check the proxy consumes /
  responds (driven by a tiny inline Python test that uses the socket only —
  robust because we don't need the guest to actually run traffic for the build
  verification).

## 5. User-visible CLI

```
qemu-system-aarch64 -M ast2600-evb \
  -chardev socket,id=ic0,path=/tmp/i2c-proxy.sock,server=on,wait=off \
  -device i2c-custom,address=0x4d,chardev=ic0,bus=aspeed.i2c.bus8 \
  -serial mon:stdio -nographic
```

The Python proxy is started separately:

```
python3 tests/qemu/i2c-custom-proxy/i2c_custom_proxy.py /tmp/i2c-proxy.sock TMP117
```

## 6. Testing strategy

- **Self-test** (`selftest.py`): exercises the proxy + device-model classes
  in-process; no QEMU needed. Runs on the GitHub Action as a sanity gate.
- **Build verification** (Action smoke step): `-device help | grep i2c-custom`
  confirms the device is registered in the `aarch64-softmmu` target.
- Full guest-driven functional tests are out of scope for this commit; can be
  added later by adapting existing `qtest` infrastructure.

## 7. Risk / trade-offs

- **Blocking chardev reads**: a thin sleep/yield loop keeps the vCPU thread
  responsive. Worst case is one guest register read hanging for up to 1s; the
  device flips into the disconnected (`drained`) state thereafter and NAKs.
- **Frame length**: capped at 64 bytes which covers all standard i2c sensor
  register accesses. Larger transfers abort; flagged in trace events.
- **No async i2c**: any master using the aspeed i2c controller's DMA mode will
  still see ACK/recv synchronously, since aspeed_i2c drives the synchronous
  slave vtable. This is the same as `tmp105`/`tmp421`.

## 8. Files added/changed

| Path | kind |
|------|------|
| `hw/sensor/i2c-custom.c` | new |
| `include/hw/sensor/i2c-custom.h` | new |
| `hw/sensor/Kconfig` | +1 entry |
| `hw/sensor/meson.build` | +1 line |
| `hw/sensor/trace-events` | +3 events |
| `tests/qemu/i2c-custom-proxy/i2c_custom_proxy.py` | new |
| `tests/qemu/i2c-custom-proxy/tmp117_model.py` | new |
| `tests/qemu/i2c-custom-proxy/selftest.py` | new |
| `tests/qemu/i2c-custom-proxy/README.md` | new |
| `.github/workflows/build-qemu-static.yml` | extend `paths`, +1 smoke step |
| `docs/superpowers/specs/2026-07-29-i2c-custom-design.md` | new |
