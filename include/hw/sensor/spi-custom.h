/*
 * QEMU spi-custom: SPI/SSI peripheral proxied to a unix-socket backend.
 *
 * The device translates guest SPI transactions into a tiny length-prefixed
 * binary protocol spoken over a CharFrontend (typically a
 * -chardev socket,server=on AF_UNIX SOCK_STREAM). An external process
 * (see tests/qemu/spi-custom-proxy/spi_custom_proxy.py) implements the
 * real device behavior.
 *
 * Implementation follows hw/sensor/i2c-custom.c as the canonical
 * qdev+chardev proxy pattern, and hw/block/m25p80.c for the
 * SSIPeripheral vtable shape (transfer/set_cs/realize, SSI_CS_LOW).
 *
 * SPI, unlike I2C, is full-duplex: a single `transfer(val)` clock
 * simultaneously writes one byte (master->slave) and reads one byte
 * (slave->master). Rather than buffering whole transactions on the
 * CS edges (which loses full-duplex semantics and forces the proxy
 * to pre-stage reply bytes it cannot yet know), this device issues
 * one synchronous socket round-trip per transfer() call. PING and
 * ASSERT_CS / RELEASE_CS ops bracket a transaction by tracking
 * chip-select transitions from set_cs(). The simple, fully-correct
 * protocol trades per-byte socket latency for SPI's true semantics.
 *
 * Copyright (c) 2026 auto2111
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_SENSOR_SPI_CUSTOM_H
#define HW_SENSOR_SPI_CUSTOM_H

#include "hw/ssi/ssi.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_SPI_CUSTOM "spi-custom"
OBJECT_DECLARE_SIMPLE_TYPE(SpiCustomState, SPI_CUSTOM)

/*
 * Opcodes for the request header. The slave→master byte is the
 * first (and only) byte of the proxy's reply for XFER; for ASSERT_CS
 * and RELEASE_CS the reply length is 0.
 */
enum {
    SPI_CUSTOM_OP_XFER      = 1,
    SPI_CUSTOM_OP_ASSERT_CS = 2,
    SPI_CUSTOM_OP_ReleaseCS = 3,
    SPI_CUSTOM_OP_PING      = 4,
};

struct SpiCustomState {
    SSIPeripheral parent;

    CharFrontend chr;

    /*
     * True whenever the chardev backend is closed/unreachable; all
     * transfers return 0x00 while this is set and ASSERT/RELEASE are
     * dropped. Mirrors i2c-custom's drained flag.
     */
    bool drained;

    /*
     * Cached chip-select state so set_cs() only fires an ASSERT or
     * RELEASE op on actual edges, not on redundant level writes.
     */
    bool cs_active;
};

#endif /* HW_SENSOR_SPI_CUSTOM_H */
