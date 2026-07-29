/*
 * QEMU i2c-custom: an i2c slave whose behavior is implemented by an
 * external process (typically Python) connected through a unix-socket
 * chardev. See docs/superpowers/specs/2026-07-29-i2c-custom-design.md.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_SENSOR_I2C_CUSTOM_H
#define HW_SENSOR_I2C_CUSTOM_H

#include "hw/i2c/i2c.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_I2C_CUSTOM "i2c-custom"
OBJECT_DECLARE_SIMPLE_TYPE(I2CCustomState, I2C_CUSTOM)

/* Maximum supported transfer size (bytes) on the wire frame. */
#define I2C_CUSTOM_MAX_XFER 64

/* Opcodes for the request header. */
enum {
    I2C_CUSTOM_OP_SEND = 1,
    I2C_CUSTOM_OP_RECV = 2,
    I2C_CUSTOM_OP_PING = 3,
};

struct I2CCustomState {
    I2CSlave i2c;

    CharFrontend chr;

    /* Bytes accumulated during a master-write (I2C_START_SEND..FINISH). */
    uint8_t tx_buf[I2C_CUSTOM_MAX_XFER];
    int tx_len;

    /* Pre-fetched rx for the current master-read (I2C_START_RECV). */
    uint8_t rx_buf[I2C_CUSTOM_MAX_XFER];
    int rx_len;
    int rx_pos;

    /* Current transaction direction flags. */
    bool in_send_txn;
    bool in_recv_txn;

    /* True whenever the chardev backend is closed/unreachable; all
     * sends NAK and all reads return 0xff while this is set. */
    bool drained;
};

#endif /* HW_SENSOR_I2C_CUSTOM_H */
