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
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "trace.h"

/* Total per-read timeout: ~1s. */
#define I2C_CUSTOM_RX_TIMEOUT_US  1000000
/* Per-iteration sleep; small enough to keep the iothread responsive. */
#define I2C_CUSTOM_RX_POLL_US        1000

/* ---- wire protocol ---- */

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
 *
 * qemu_chr_fe_read_all() can return a short count or -1 (EAGAIN), so we
 * loop until we either have all the bytes, see EOF (rc == 0), or exceed
 * the timeout budget. We yield the thread with g_usleep to keep iothread
 * responsiveness, mirroring the pattern used in hw/ipmi/ipmi_bmc_extern.c
 * (which uses a QEMUTimer for retries; we use a tight loop because each
 * i2c transaction is short and synchronous from the vCPU thread).
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
 *   - if @rx_want > 0, the reply payload (exactly @rx_want bytes) is stored
 *     into @rx_out.
 *   - returns 0.
 *
 * On backend-side NAK (rx_len == 0xffff in the reply) or any I/O error or
 * timeout, returns -1 and sets s->drained = true on EOF/timeout. No partial
 * rx data is left in @rx_out on failure.
 */
static int i2c_custom_request(I2CCustomState *s, uint8_t opcode,
                              const uint8_t *tx, uint8_t tx_len,
                              uint8_t rx_want, uint8_t address,
                              uint8_t *rx_out)
{
    I2CCustomReq hdr;
    uint8_t rlen_lo, rlen_hi;
    uint16_t rlen;

    if (s->drained) {
        return -1;
    }

    /* Clamp at compile-time-checked limits (callers must already obey). */
    if (tx_len > I2C_CUSTOM_MAX_XFER || rx_want > I2C_CUSTOM_MAX_XFER) {
        return -1;
    }

    hdr.opcode = opcode;
    hdr.tx_len = tx_len;
    hdr.rx_len_max = rx_want;
    hdr.address = address;

    trace_i2c_custom_event_op(opcode, address);

    /* Write header + tx payload in a single write where possible. */
    if (tx_len > 0) {
        /*
         * Build a tiny contiguous buffer: 4 bytes header + tx_len payload.
         * Avoids two write() syscalls and any reordering concerns with the
         * Python side (which reads 4-byte header then exactly tx_len bytes).
         */
        uint8_t out[4 + I2C_CUSTOM_MAX_XFER];
        out[0] = hdr.opcode;
        out[1] = hdr.tx_len;
        out[2] = hdr.rx_len_max;
        out[3] = hdr.address;
        memcpy(out + 4, tx, tx_len);
        if (qemu_chr_fe_write_all(&s->chr, out, 4 + tx_len) !=
            4 + tx_len) {
            s->drained = true;
            return -1;
        }
    } else {
        if (qemu_chr_fe_write_all(&s->chr, (uint8_t *)&hdr, 4) != 4) {
            s->drained = true;
            return -1;
        }
    }

    /* Read 2-byte reply length (big-endian). */
    if (!i2c_custom_read_all(s, &rlen_hi, 1) ||
        !i2c_custom_read_all(s, &rlen_lo, 1)) {
        s->drained = true;
        return -1;
    }
    rlen = (uint16_t)((rlen_hi << 8) | rlen_lo);
    if (rlen == 0xffff) {
        /* Backend explicitly NAKed this transaction. */
        return -1;
    }

    /* Drain any reply bytes we weren't prepared for (defensive). */
    if (rlen > rx_want) {
        uint8_t skip[I2C_CUSTOM_MAX_XFER];
        if (!i2c_custom_read_all(s, skip, rlen - rx_want)) {
            s->drained = true;
            return -1;
        }
        rlen = rx_want;
    }

    if (rx_want > 0) {
        if (!i2c_custom_read_all(s, rx_out, rlen)) {
            s->drained = true;
            return -1;
        }
        /* Pad remaining wanted bytes with 0xff (defensive; Python should
         * already send rx_want bytes for RECV ops). */
        if (rlen < rx_want) {
            memset(rx_out + rlen, 0xff, rx_want - rlen);
        }
    }
    return 0;
}

/* ---- i2c slave vtable ---- */

static int i2c_custom_event(I2CSlave *i2c, enum i2c_event event)
{
    I2CCustomState *s = I2C_CUSTOM(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->in_send_txn = true;
        s->in_recv_txn = false;
        s->tx_len = 0;
        break;

    case I2C_START_SEND_ASYNC:
        /* Not supported; we only implement the synchronous path. */
        return -1;

    case I2C_START_RECV: {
        /*
         * Pre-fetch the entire master-read into rx_buf so subsequent
         * i2c_custom_recv() calls return bytes synchronously without
         * blocking each call separately.
         */
        int rc;
        s->in_recv_txn = true;
        s->in_send_txn = false;
        s->tx_len = 0;
        s->rx_len = 0;
        s->rx_pos = 0;
        rc = i2c_custom_request(s, I2C_CUSTOM_OP_RECV,
                                NULL, 0,
                                I2C_CUSTOM_MAX_XFER, i2c->address,
                                s->rx_buf);
        if (rc < 0) {
            /* Drained / NAK: rx_buf stays zero-filled; recv() returns 0xff. */
            s->drained = true;
            break;
        }
        s->rx_len = I2C_CUSTOM_MAX_XFER;
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
                               0, i2c->address, NULL);
        }
        s->in_send_txn = false;
        s->in_recv_txn = false;
        s->tx_len = 0;
        break;

    case I2C_NACK:
        /* Master NAKed a receive byte; nothing to do here. */
        break;
    }

    return 0;
}

static int i2c_custom_send(I2CSlave *i2c, uint8_t data)
{
    I2CCustomState *s = I2C_CUSTOM(i2c);

    if (!s->in_send_txn) {
        /* Defensive: stray send outside a START_SEND..FINISH; NAK. */
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
     * Probe the backend with a PING so_chr_event() has had a chance to set
     * s->drained=false. If the peer is not yet connected (e.g. wait=off and
     * the python proxy hasn't started), this will time out and we'll just
     * operate in drained mode until the connection appears.
     */
    i2c_custom_request(s, I2C_CUSTOM_OP_PING, NULL, 0, 0, s->i2c.address,
                       NULL);
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

static void i2c_custom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = i2c_custom_event;
    k->recv = i2c_custom_recv;
    k->send = i2c_custom_send;
    /* send_async intentionally unset: we don't support deferred ACK. */

    dc->realize = i2c_custom_realize;
    dc->reset = i2c_custom_reset;
    dc->vmsd = &i2c_custom_vmstate;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const Property i2c_custom_properties[] = {
    DEFINE_PROP_CHR("chardev", I2CCustomState, chr),
};

static void i2c_custom_register_types(void)
{
    static const TypeInfo info = {
        .name = TYPE_I2C_CUSTOM,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(I2CCustomState),
        .class_init = i2c_custom_class_init,
    };

    type_register_static(&info);
}

type_init(i2c_custom_register_types)
