# spi-custom design

Date: 2026-07-30
Author: auto2111

## Motivation

`i2c-custom` (see `docs/superpowers/specs/2026-07-29-i2c-custom-design.md`)
lets a developer attach a Python-implemented I2C device to a QEMU guest
without writing any new C code. `spi-custom` does the same for SPI/SSI
peripherals on any `SSIPeripheral`-using bus, including AST2600's SMC
(FMC and `spi[N]`) controllers.

SPI differs from I2C in three load-bearing ways:

1. **Full-duplex.** A single SPI clock writes a byte (`MOSI`) and reads a
   byte (`MISO`) at the same time. I2C is half-duplex: each transaction is
   either pure-write (`I2C_START_SEND`) or pure-read (`I2C_START_RECV`).
2. **No START/repeat-FINISH framing built into the bus.** Instead, the
   chip-select (`CS#`) line delimits a transaction; the master asserts
   `CS#`, clocks as many words as it likes, then deasserts `CS#`.
3. **Word width up to 32 bits.** SSI words can be 8..32 bits, though the
   byte-oriented peripherals (m25p80, SD, sensors) use 8-bit words.

The design reflects these differences.

## Wire protocol

See `tests/qemu/spi-custom-proxy/README.md`. Two divergences from
`i2c-custom`:

- The request header is 4 bytes plus an optional payload byte, not
  4 bytes plus a longer payload. SPI peripherals model byte streams.
- `XFER` carries both a `tx_byte` and a 1-byte response in a single
  round-trip — true full-duplex semantics, one syscall per SPI word.

## Device model choice: per-byte synchronous round-trip

Three options were considered (see the implementation's `spi-custom.c`
header comment for context). The chosen one is:

**Option A — per-byte synchronous transfer** (chosen).

- On every `SSIPeripheralClass.transfer(val)` call, the C device
  issues one synchronous socket round-trip to the Python process: it
  sends `XFER byte_in=val&0xff`, waits for the `rx_len + byte` reply,
  and returns the byte. `set_cs(true/false)` issues `ASSERT_CS` /
  `RELEASE_CS`. `realize` issues a `PING`.

Pros:

- Correct SPI semantics — Python `transfer(byte)` sees the master
  byte and must produce the MISO byte **for the same clock**. This
  matches the real hardware and supports stateful devices (flash,
  SD cards, register-style sensors) that decide `byte_out` based on
  prior bytes in the same transaction.
- Mirrors the SSIPeripheralClass `transfer` callback one-to-one;
  simplest Python `Device` API.
- Per-transaction latency (1 socket round-trip per byte, ~100µs-1ms)
  is identical to `i2c-custom`'s synchronous RECV loop and is
  acceptable for modelling.

Rejected alternatives:

- **Option B — CS-edge batched flush** (flush whole master→slave
  buffer on `set_cs(false)`, deliver pre-staged slave→master). Would
  require Python to pre-stage the entire reply without seeing the
  input sequence; impossible for stateful protocols (flash READ,
  SD multi-block). Also breaks real-time full-duplex happy path.
- **Option C — prefetch a chunk on `set_cs(true)`** then stream
  from the prefetch while sending master bytes on the next CS edge.
  Possible but complex; Python must guess reply sizes ahead of
  seeing the master bytes, which stateful SPI devices cannot.

## Bus attachment on AST2600-evb

The Aspeed SMC controllers (`hw/ssi/aspeed_smc.c`) create their SSI
buses via `ssi_create_bus(dev, NULL)` — the `NULL` name means the bus
inherits the controller's QOM path as its name. The flash sub-devices
are created with `object_initialize_child(obj, "flash[*]", ...)` and
attached via `qdev_realize_and_unref(dev, BUS(s->spi), ...)`.

For `-device spi-custom,bus=<path>`, the user must specify the QOM path
of the controller's SSI bus. The default board's flashes (m25p80 with
`w25q512jv` for `ast2600-evb`) already occupy CS 0..N-1 of FMC, so
`spi-custom` should target an unused CS index or one of the secondary
`spi[N]` controllers. Users may need to suppress default flashes (set
the machine's `fmc-model=""` property accordingly) to fully replace one
of the CS slots.

## Verification

- `tests/qemu/spi-custom-proxy/selftest.py` — in-process PING / XFER /
  ASSERT_CS / RELEASE_CS / unknown-opcode-NAK; exits 0 on pass.
- CI (`build-qemu-static.yml`) builds a static `qemu-system-aarch64`,
  greps `-device help` for `spi-custom`, and runs the Python selftest.
- Local Docker build under ubuntu:24.04 (same flags as CI) produces a
  statically-linked binary listing `spi-custom` on the `SSI` bus.

## Files added

- `hw/sensor/spi-custom.c`
- `include/hw/sensor/spi-custom.h`
- `tests/qemu/spi-custom-proxy/spi_custom_proxy.py`
- `tests/qemu/spi-custom-proxy/m25p80_stub_model.py`
- `tests/qemu/spi-custom-proxy/selftest.py`
- `tests/qemu/spi-custom-proxy/README.md`
- `docs/superpowers/specs/2026-07-30-spi-custom-design.md`

## Files modified

- `hw/sensor/Kconfig` — new `config SPI_CUSTOM`
- `hw/sensor/meson.build` — build rule
- `hw/sensor/trace-events` — 3 new trace events
- `.github/workflows/build-qemu-static.yml` — path triggers + smoke
  test + Python selftest step
