/*
 * QEMU i2c-custom: i2c slave proxied to a unix-socket backend.
 *
 * The device translates guest i2c transactions into a tiny length-prefixed
 * binary protocol spoken over a CharFrontend (typically a
 * -chardev socket,server=on AF_UNIX SOCK_STREAM). An external process
 * (see tests/qemu/i2c-custom-proxy/i2c_custom_proxy.py) implements the
 * real device behavior.
 *
 * Implementation follows hw/ipmi/ipmi_bmc_extern.c as the canonical
 * qdev+chardev proxy pattern, and hw/sensor/tmp105.c for the i2c-slave
 * vtable shape.
 *
 * Copyright (c) 2026 auto2111
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/i2c/i2c.h"
#include "hw/sensor/i2c-custom.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "trace.h"

/* Total per-read timeout: 100ms (avoid stalling vCPU / holding BQL). */
#define I2C_CUSTOM_RX_TIMEOUT_US  100000
/* Per-iteration sleep. */
#define I2C_CUSTOM_RX_POLL_US        200

/* ---- wire protocol & return codes ---- */

#define I2C_CUSTOM_REQ_OK        0
#define I2C_CUSTOM_REQ_NAK      -1
#define I2C_CUSTOM_REQ_ERR      -2

/*
 * Request, QEMU -> proxy:
 *   [0]  opcode           (1=SEND, 2=RECV, 3=PING)
 *   [1]  tx_len           (0..I2C_CUSTOM_MAX_XFER)
 *   [2]  rx_len_max       (0..I2C_CUSTOM_MAX_XFER)
 *   [3]  address          (informational)
 *   [4..] tx_len payload bytes
 *
 * Reply, proxy -> QEMU:
 *   [0..1] rx_len (big-endian u16); 0xffff means NAK
 *   [2..]  rx_len payload bytes
 */
typedef struct I2CCustomReq {
    uint8_t opcode;
    uint8_t tx_len;
    uint8_t rx_len_max;
    uint8_t address;
} I2CCustomReq;

/* ---- chardev callbacks (set_handlers signature requires all three) ---- */

static int i2c_custom_can_receive(void *opaque)
{
    /*
     * We never accept unsolicited inbound bytes. The protocol is purely
     * request/reply, and replies are consumed synchronously from within
     * i2c_custom_request().
     */
    return 0;
}

static void i2c_custom_receive(void *opaque, const uint8_t *buf, int size)
{
    /* No-op: see i2c_custom_can_receive(). */
}

static void i2c_custom_chr_event(void *opaque, QEMUChrEvent event)
{
    I2CCustomState *s = I2C_CUSTOM(opaque);

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
        /* no-op */
        break;
    }
}

/* ---- frame-level I/O helpers ---- */

/*
 * Synchronous read of exactly `want` bytes from the chardev frontend,
 * polling with a short sleep up to I2C_CUSTOM_RX_TIMEOUT_US in total.
 * Returns true on success (all bytes read), false on timeout or EOF.
 */
static bool i2c_custom_read_all(I2CCustomState *s, uint8_t *buf, size_t want)
{
    size_t got = 0;
    int64_t deadline = qemu_clock_get_us(QEMU_CLOCK_REALTIME) +
                       I2C_CUSTOM_RX_TIMEOUT_US;

    while (got < want) {
        int rc = qemu_chr_fe_read_all(&s->chr, buf + got, want - got);
        if (rc > 0) {
            got += rc;
            continue;
        }
        if (rc == 0) {
            /* EOF: backend closed underneath us. */
            return false;
        }
        /* rc < 0: poll and retry until timeout. */
        if (qemu_clock_get_us(QEMU_CLOCK_REALTIME) >= deadline) {
            return false;
        }
        g_usleep(I2C_CUSTOM_RX_POLL_US);
    }
    return true;
}

/*
 * Issue a request to the backend and collect the reply.
 *
 * On success:
 *   - stores actual received payload bytes (up to @rx_want) into @rx_out.
 *   - if @rx_actual != NULL, sets *@rx_actual to the actual received count.
 *   - returns I2C_CUSTOM_REQ_OK (0).
 *
 * On backend-side NAK (rx_len == 0xffff in the reply):
 *   - returns I2C_CUSTOM_REQ_NAK (-1). Does NOT set s->drained.
 *
 * On I/O error or timeout:
 *   - sets s->drained = true and returns I2C_CUSTOM_REQ_ERR (-2).
 */
static int i2c_custom_request(I2CCustomState *s, uint8_t opcode,
                              const uint8_t *tx, uint8_t tx_len,
                              uint8_t rx_want, uint8_t address,
                              uint8_t *rx_out, int *rx_actual)
{
    I2CCustomReq hdr;
    uint8_t rlen_buf[2];
    uint16_t rlen;

    if (rx_actual) {
        *rx_actual = 0;
    }

    if (s->drained) {
        return I2C_CUSTOM_REQ_ERR;
    }

    /* Clamp at compile-time-checked limits (callers must already obey). */
    if (tx_len > I2C_CUSTOM_MAX_XFER || rx_want > I2C_CUSTOM_MAX_XFER) {
        return I2C_CUSTOM_REQ_ERR;
    }

    hdr.opcode = opcode;
    hdr.tx_len = tx_len;
    hdr.rx_len_max = rx_want;
    hdr.address = address;

    trace_i2c_custom_event_op(opcode, address);

    /* Write header + tx payload in a single write where possible. */
    if (tx_len > 0) {
        uint8_t out[4 + I2C_CUSTOM_MAX_XFER];
        out[0] = hdr.opcode;
        out[1] = hdr.tx_len;
        out[2] = hdr.rx_len_max;
        out[3] = hdr.address;
        memcpy(out + 4, tx, tx_len);
        if (qemu_chr_fe_write_all(&s->chr, out, 4 + tx_len) !=
            4 + tx_len) {
            s->drained = true;
            return I2C_CUSTOM_REQ_ERR;
        }
    } else {
        if (qemu_chr_fe_write_all(&s->chr, (uint8_t *)&hdr, 4) != 4) {
            s->drained = true;
            return I2C_CUSTOM_REQ_ERR;
        }
    }

    /* Read 2-byte reply length (big-endian). */
    if (!i2c_custom_read_all(s, rlen_buf, 2)) {
        s->drained = true;
        return I2C_CUSTOM_REQ_ERR;
    }
    rlen = (uint16_t)((rlen_buf[0] << 8) | rlen_buf[1]);
    if (rlen == 0xffff) {
        /* Backend explicitly NAKed this transaction. */
        return I2C_CUSTOM_REQ_NAK;
    }

    /* Drain any excess reply bytes safely without stack overflow. */
    if (rlen > rx_want) {
        uint8_t skip[I2C_CUSTOM_MAX_XFER];
        size_t to_skip = rlen - rx_want;
        while (to_skip > 0) {
            size_t chunk = MIN(to_skip, sizeof(skip));
            if (!i2c_custom_read_all(s, skip, chunk)) {
                s->drained = true;
                return I2C_CUSTOM_REQ_ERR;
            }
            to_skip -= chunk;
        }
        rlen = rx_want;
    }

    if (rx_want > 0) {
        if (rlen > 0) {
            if (!i2c_custom_read_all(s, rx_out, rlen)) {
                s->drained = true;
                return I2C_CUSTOM_REQ_ERR;
            }
        }
        /* Pad remaining wanted bytes with 0xff. */
        if (rlen < rx_want) {
            memset(rx_out + rlen, 0xff, rx_want - rlen);
        }
        if (rx_actual) {
            *rx_actual = rlen;
        }
    }
    return I2C_CUSTOM_REQ_OK;
}

/* ---- i2c slave vtable ---- */

static int i2c_custom_event(I2CSlave *i2c, enum i2c_event event)
{
    I2CCustomState *s = I2C_CUSTOM(i2c);

    switch (event) {
    case I2C_START_SEND:
        if (s->drained) {
            return -1;
        }
        s->in_send_txn = true;
        s->in_recv_txn = false;
        s->tx_len = 0;
        break;

    case I2C_START_SEND_ASYNC:
        /* Not supported; we only implement the synchronous path. */
        return -1;

    case I2C_START_RECV: {
        int actual_len = 0;
        int rc;

        if (s->drained) {
            return -1;
        }

        /*
         * If this is a repeated START (no intervening I2C_FINISH), any
         * bytes accumulated during the preceding write phase must be
         * flushed to the backend so the device model sees the register
         * pointer before we start reading.
         */
        if (s->in_send_txn && s->tx_len > 0) {
            rc = i2c_custom_request(s, I2C_CUSTOM_OP_SEND,
                                    s->tx_buf, s->tx_len,
                                    0, i2c->address, NULL, NULL);
            if (rc != I2C_CUSTOM_REQ_OK) {
                s->in_send_txn = false;
                return -1;
            }
        }
        /*
         * Pre-fetch the entire master-read into rx_buf so subsequent
         * i2c_custom_recv() calls return bytes synchronously without
         * blocking each call separately.
         */
        s->in_recv_txn = true;
        s->in_send_txn = false;
        s->tx_len = 0;
        s->rx_len = 0;
        s->rx_pos = 0;
        rc = i2c_custom_request(s, I2C_CUSTOM_OP_RECV,
                                NULL, 0,
                                I2C_CUSTOM_MAX_XFER, i2c->address,
                                s->rx_buf, &actual_len);
        if (rc != I2C_CUSTOM_REQ_OK) {
            s->in_recv_txn = false;
            return -1;
        }
        s->rx_len = actual_len;
        break;
    }

    case I2C_FINISH:
        /*
         * If a master-write transaction just finished, flush the buffered
         * bytes to the backend. We don't expect a reply payload for SEND.
         */
        if (s->in_send_txn && s->tx_len > 0) {
            trace_i2c_custom_send(s->tx_len, s->tx_buf[0]);
            i2c_custom_request(s, I2C_CUSTOM_OP_SEND,
                               s->tx_buf, s->tx_len,
                               0, i2c->address, NULL, NULL);
        }
        s->in_send_txn = false;
        s->in_recv_txn = false;
        s->tx_len = 0;
        break;

    case I2C_NACK:
        /* Master NAKed a receive byte; end transaction. */
        s->in_send_txn = false;
        s->in_recv_txn = false;
        s->tx_len = 0;
        break;
    }

    return 0;
}

static int i2c_custom_send(I2CSlave *i2c, uint8_t data)
{
    I2CCustomState *s = I2C_CUSTOM(i2c);

    if (s->drained || !s->in_send_txn) {
        return -1;
    }
    if (s->tx_len >= I2C_CUSTOM_MAX_XFER) {
        /* Overrun: NAK this byte; FINISH will still flush what we have. */
        return -1;
    }
    s->tx_buf[s->tx_len++] = data;
    return 0;
}

static uint8_t i2c_custom_recv(I2CSlave *i2c)
{
    I2CCustomState *s = I2C_CUSTOM(i2c);

    if (s->in_recv_txn && s->rx_pos < s->rx_len) {
        uint8_t b = s->rx_buf[s->rx_pos++];
        trace_i2c_custom_recv(s->rx_len, b);
        return b;
    }
    /* No more data (or not in a recv txn): return 0xff, common "no device"
     * semantics for an unresponsive i2c slave. */
    return 0xff;
}

/* ---- realize / reset / vmstate / class ---- */

static void i2c_custom_instance_init(Object *obj)
{
    I2CCustomState *s = I2C_CUSTOM(obj);

    s->drained = true;
}

static void i2c_custom_realize(DeviceState *dev, Error **errp)
{
    I2CCustomState *s = I2C_CUSTOM(dev);

    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        error_setg(errp, "i2c-custom requires a 'chardev' property");
        return;
    }

    qemu_chr_fe_set_handlers(&s->chr, i2c_custom_can_receive,
                            i2c_custom_receive, i2c_custom_chr_event,
                            NULL, s, NULL, true);

    /*
     * Probe the backend with a PING. If already connected, drained will
     * clear to false; otherwise we stay in drained mode until CHR_EVENT_OPENED.
     */
    if (i2c_custom_request(s, I2C_CUSTOM_OP_PING, NULL, 0, 0, s->i2c.address,
                           NULL, NULL) == I2C_CUSTOM_REQ_OK) {
        s->drained = false;
    }
}

static void i2c_custom_reset(DeviceState *dev)
{
    I2CCustomState *s = I2C_CUSTOM(dev);

    s->tx_len = 0;
    s->rx_len = 0;
    s->rx_pos = 0;
    s->in_send_txn = false;
    s->in_recv_txn = false;
    /* s->drained is driven by chardev events; do not touch it here. */
}

static const VMStateDescription i2c_custom_vmstate = {
    .name = "i2c-custom",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(i2c, I2CCustomState),
        VMSTATE_UINT8_ARRAY(tx_buf, I2CCustomState, I2C_CUSTOM_MAX_XFER),
        VMSTATE_INT32(tx_len, I2CCustomState),
        VMSTATE_UINT8_ARRAY(rx_buf, I2CCustomState, I2C_CUSTOM_MAX_XFER),
        VMSTATE_INT32(rx_len, I2CCustomState),
        VMSTATE_INT32(rx_pos, I2CCustomState),
        VMSTATE_BOOL(in_send_txn, I2CCustomState),
        VMSTATE_BOOL(in_recv_txn, I2CCustomState),
        VMSTATE_BOOL(drained, I2CCustomState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property i2c_custom_properties[] = {
    DEFINE_PROP_CHR("chardev", I2CCustomState, chr),
};

static void i2c_custom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = i2c_custom_event;
    k->recv = i2c_custom_recv;
    k->send = i2c_custom_send;
    /* send_async intentionally unset: we don't support deferred ACK. */

    dc->realize = i2c_custom_realize;
    device_class_set_legacy_reset(dc, i2c_custom_reset);
    dc->vmsd = &i2c_custom_vmstate;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_props(dc, i2c_custom_properties);
}

static void i2c_custom_register_types(void)
{
    static const TypeInfo info = {
        .name = TYPE_I2C_CUSTOM,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(I2CCustomState),
        .instance_init = i2c_custom_instance_init,
        .class_init = i2c_custom_class_init,
    };

    type_register_static(&info);
}

type_init(i2c_custom_register_types)
