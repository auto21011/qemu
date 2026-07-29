/*
 * QEMU spi-custom: SPI peripheral proxied to a unix-socket backend.
 *
 * The device translates guest SPI transactions into a tiny length-prefixed
 * binary protocol spoken over a CharFrontend (typically a
 * -chardev socket,server=on AF_UNIX SOCK_STREAM). An external process
 * (see tests/qemu/spi-custom-proxy/spi_custom_proxy.py) implements the
 * real device behavior.
 *
 * This file mirrors hw/sensor/i2c-custom.c in structure and coding style;
 * differences are driven by the SPI vtable (SSIPeripheralClass) versus
 * I2C's (I2CSlaveClass).  SPI is full-duplex, so `transfer()` issues one
 * synchronous blocking round-trip per byte.
 *
 * Copyright (c) 2026 auto2111
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/ssi/ssi.h"
#include "hw/sensor/spi-custom.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "trace.h"

/* Total per-read timeout: ~1s (mirrors I2C_CUSTOM_RX_TIMEOUT_US). */
#define SPI_CUSTOM_RX_TIMEOUT_US  1000000
#define SPI_CUSTOM_RX_POLL_US        1000

/* ---- wire protocol ---- */

/*
 * Request, QEMU -> proxy:
 *   [0]  opcode           (1=XFER, 2=ASSERT_CS, 3=RELEASE_CS, 4=PING)
 *   [1]  tx_len           (0 for all except XFER which is 1)
 *   [2]  rx_len_max       (1 for XFER, 0 otherwise)
 *   [3]  pad              (reserved)
 *   [4]  byte             (only for XFER: the master->slave byte)
 *
 * Reply, proxy -> QEMU:
 *   [0..1] rx_len (big-endian u16); 0xffff means NAK
 *   [2]    byte (only for XFER: the slave->master byte)
 */
typedef struct SpiCustomReq {
    uint8_t opcode;
    uint8_t tx_len;
    uint8_t rx_len_max;
    uint8_t pad;
} SpiCustomReq;

/* ---- chardev callbacks ---- */

static int spi_custom_can_receive(void *opaque)
{
    /* Purely request/reply; no unsolicited inbound bytes. */
    return 0;
}

static void spi_custom_receive(void *opaque, const uint8_t *buf, int size)
{
    /* No-op: see spi_custom_can_receive(). */
}

static void spi_custom_chr_event(void *opaque, QEMUChrEvent event)
{
    SpiCustomState *s = SPI_CUSTOM(opaque);

    switch (event) {
    case CHR_EVENT_OPENED:
        s->drained = false;
        break;
    case CHR_EVENT_CLOSED:
        s->drained = true;
        break;
    case CHR_EVENT_BREAK:
    case CHR_EVENT_MUX_IN:
    case CHR_EVENT_MUX_OUT:
        break;
    }
}

/* ---- frame-level I/O helpers (mirrors i2c_custom_read_all) ---- */

static bool spi_custom_read_all(SpiCustomState *s, uint8_t *buf, size_t want)
{
    size_t got = 0;
    int64_t deadline = qemu_clock_get_us(QEMU_CLOCK_REALTIME) +
                       SPI_CUSTOM_RX_TIMEOUT_US;

    while (got < want) {
        int rc = qemu_chr_fe_read_all(&s->chr, buf + got, want - got);
        if (rc > 0) {
            got += rc;
            continue;
        }
        if (rc == 0) {
            return false;
        }
        if (qemu_clock_get_us(QEMU_CLOCK_REALTIME) >= deadline) {
            return false;
        }
        g_usleep(SPI_CUSTOM_RX_POLL_US);
    }
    return true;
}

/*
 * Issue a request to the backend and collect the reply.
 * On success returns 0; on NAK, I/O error, or timeout returns -1.
 * On XFER: @rx_out (if non-NULL) receives exactly 1 byte on success.
 */
static int spi_custom_request(SpiCustomState *s, uint8_t opcode,
                              uint8_t tx_byte, bool expect_rx,
                              uint8_t *rx_out)
{
    uint8_t out[5];
    uint8_t rlen_hi, rlen_lo;
    uint16_t rlen;
    uint8_t rx_byte = 0;

    if (s->drained) {
        return -1;
    }

    /*
     * Build a 5-byte header:
     *   [opcode, tx_len, rx_len_max, pad, byte(only if tx_len==1)]
     * We always attach the byte for XFER; for ASSERT/RELEASE/PING
     * the tx_len/rx_len_max are 0 and the Python side's header-only
     * decode skips the 5th byte (its tx_len is 0).
     */
    out[0] = opcode;
    out[1] = expect_rx ? 1 : 0;
    out[2] = expect_rx ? 1 : 0;
    out[3] = 0; /* pad */
    out[4] = tx_byte;

    trace_spi_custom_xfer(opcode, tx_byte);

    /* Only the first 4 bytes are header; the 5th byte is the tx payload
     * when tx_len >= 1. Send 5 unconditionally to keep the syscall tight;
     * the Python side reads header first then 0/1 payload bytes per tx_len.
     */
    if (qemu_chr_fe_write_all(&s->chr, out, 5) != 5) {
        s->drained = true;
        return -1;
    }

    /* Read 2-byte reply length (big-endian). */
    if (!spi_custom_read_all(s, &rlen_hi, 1) ||
        !spi_custom_read_all(s, &rlen_lo, 1)) {
        s->drained = true;
        return -1;
    }
    rlen = (uint16_t)((rlen_hi << 8) | rlen_lo);
    if (rlen == 0xffff) {
        /* Backend explicitly NAKed this transaction. */
        return -1;
    }

    /* For XFER, read the single reply byte. */
    if (expect_rx && rlen >= 1) {
        if (!spi_custom_read_all(s, &rx_byte, 1)) {
            s->drained = true;
            return -1;
        }
    }
    if (rx_out) {
        *rx_out = rx_byte;
    }
    return 0;
}

/* ---- SSI peripheral vtable ---- */

static uint32_t spi_custom_transfer(SSIPeripheral *ss, uint32_t val)
{
    SpiCustomState *s = SPI_CUSTOM(ss);
    uint8_t rx_byte = 0;

    if (s->drained) {
        return rx_byte;
    }

    /*
     * Issue a single-byte XFER. The lower 8 bits of @val are the
     * master-out byte; the single byte reply becomes the slave-in
     * byte. QEMU's SSIPeripheralClass transfer callback may pass a
     * wider val, but spi-custom is a byte-oriented proxy: low 8 bits
     * in, low 8 bits out, matching the shape used by hw/block/m25p80.c
     * (m25p80_transfer8).
     */
    if (spi_custom_request(s, SPI_CUSTOM_OP_XFER, val & 0xFF, true,
                           &rx_byte) < 0) {
        return 0;
    }

    trace_spi_custom_recv(rx_byte);
    return rx_byte;
}

static int spi_custom_set_cs(SSIPeripheral *ss, bool select)
{
    SpiCustomState *s = SPI_CUSTOM(ss);

    if (s->drained) {
        s->cs_active = select;
        return 0;
    }

    if (select == s->cs_active) {
        /* No edge; no proxy transaction needed. */
        return 0;
    }
    s->cs_active = select;

    if (select) {
        /* CS is now asserted (active low, so `select==true` means low). */
        spi_custom_request(s, SPI_CUSTOM_OP_ASSERT_CS, 0, false, NULL);
        trace_spi_custom_cs("assert");
    } else {
        spi_custom_request(s, SPI_CUSTOM_OP_ReleaseCS, 0, false, NULL);
        trace_spi_custom_cs("release");
    }
    return 0;
}

/* ---- realize / reset / vmstate / class ---- */

static void spi_custom_realize(SSIPeripheral *ss, Error **errp)
{
    SpiCustomState *s = SPI_CUSTOM(ss);

    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        error_setg(errp, "spi-custom requires a 'chardev' property");
        return;
    }

    qemu_chr_fe_set_handlers(&s->chr, spi_custom_can_receive,
                             spi_custom_receive, spi_custom_chr_event,
                             NULL, s, NULL, true);

    /* Probe the backend with a PING. */
    spi_custom_request(s, SPI_CUSTOM_OP_PING, 0, false, NULL);
}

static void spi_custom_reset(DeviceState *dev)
{
    SpiCustomState *s = SPI_CUSTOM(dev);

    s->cs_active = false;
    /* s->drained is driven by chardev events; do not touch it here. */
}

static const VMStateDescription spi_custom_vmstate = {
    .name = "spi-custom",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(drained, SpiCustomState),
        VMSTATE_BOOL(cs_active, SpiCustomState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property spi_custom_properties[] = {
    DEFINE_PROP_CHR("chardev", SpiCustomState, chr),
};

static void spi_custom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = spi_custom_realize;
    k->transfer = spi_custom_transfer;
    k->set_cs = spi_custom_set_cs;
    k->cs_polarity = SSI_CS_LOW;

    dc->vmsd = &spi_custom_vmstate;
    device_class_set_legacy_reset(dc, spi_custom_reset);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_props(dc, spi_custom_properties);
}

static void spi_custom_register_types(void)
{
    static const TypeInfo info = {
        .name = TYPE_SPI_CUSTOM,
        .parent = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(SpiCustomState),
        .class_init = spi_custom_class_init,
    };

    type_register_static(&info);
}

type_init(spi_custom_register_types)
