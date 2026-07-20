/*
 * ASPEED PWM/Tachometer Controller (AST2600 generation)
 *
 * Copyright (C) 2017-2021 IBM Corp.
 * Copyright (C) 2025 - AST2600 PWM + Fan Tach model with QOM-adjustable RPM.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Layout matches the AST2600 "aspeed,ast2600-pwm-tach" binding used by the
 * Linux driver drivers/hwmon/aspeed-g6-pwm-tach.c: 16 independent channels,
 * each with four 32-bit registers at a stride of 0x10 within a 0x100 region:
 *
 *   ch N:  base + N*0x10 + 0x00  ->  PWM_CTRL
 *          base + N*0x10 + 0x04  ->  PWM_DUTY_CYCLE
 *          base + N*0x10 + 0x08  ->  TACH_CTRL
 *          base + N*0x10 + 0x0C  ->  TACH_STS
 *
 * The PWM side just stores guest-programmed control/duty values; the tach
 * side returns a synthetic raw counter derived from a QOM-adjustable target
 * RPM scaled by the current duty cycle so that raising the guest-programmed
 * duty makes the guest read a proportionally higher RPM, matching real fan
 * behaviour.
 */

#ifndef ASPEED_PWM_H
#define ASPEED_PWM_H

#include "hw/core/sysbus.h"

#define TYPE_ASPEED_PWM "aspeed.pwm"
#define ASPEED_PWM(obj) OBJECT_CHECK(AspeedPWMState, (obj), TYPE_ASPEED_PWM)

/* AST2600 has 16 PWM/tach channels in a 0x100 register region. */
#define ASPEED_PWM_NR_CHANNELS 16
#define ASPEED_PWM_NR_REGS     (0x100 / sizeof(uint32_t))

/* Per-channel register stride. */
#define ASPEED_PWM_CH_STRIDE   0x10

/* PWM_CTRL bits */
#define ASPEED_PWM_CTRL_CLK_ENABLE       BIT(16)
#define ASPEED_PWM_CTRL_INVERSE          BIT(14)
#define ASPEED_PWM_CTRL_PIN_ENABLE       BIT(12)
#define ASPEED_PWM_CTRL_CLK_DIV_H_SHIFT  8
#define ASPEED_PWM_CTRL_CLK_DIV_H_MASK   (0xfu << 8)
#define ASPEED_PWM_CTRL_CLK_DIV_L_MASK   0xffu

/* PWM_DUTY_CYCLE bits */
#define ASPEED_PWM_DUTY_PERIOD_SHIFT     24
#define ASPEED_PWM_DUTY_PERIOD_MASK      (0xffu << 24)
#define ASPEED_PWM_DUTY_FALLING_SHIFT    8
#define ASPEED_PWM_DUTY_FALLING_MASK     (0xffu << 8)
#define ASPEED_PWM_DUTY_RISING_MASK      0xffu

/* TACH_CTRL bits */
#define ASPEED_TACH_CTRL_ENABLE          BIT(28)
#define ASPEED_TACH_CTRL_CLK_DIV_T_SHIFT 20
#define ASPEED_TACH_CTRL_CLK_DIV_T_MASK  (0xfu << 20)
#define ASPEED_TACH_CTRL_THRESHOLD_MASK  0xfffffu

/* TACH_STS bits */
#define ASPEED_TACH_STS_ISR              BIT(31)
#define ASPEED_TACH_STS_PWM_OUT          BIT(25)
#define ASPEED_TACH_STS_PWM_OEN          BIT(24)
#define ASPEED_TACH_STS_VALUE_UPDATE     BIT(21)
#define ASPEED_TACH_STS_FULL_MEASUREMENT BIT(20)
#define ASPEED_TACH_STS_VALUE_MASK       0xfffffu

/*
 * Per-channel runtime state. The guest-visible registers are kept in
 * `pwm_ctrl`, `pwm_duty`, `tach_ctrl` so that read-back round-trips the
 * values the guest wrote; the rest is model-internal.
 */
typedef struct AspeedPWMChannel {
    /* Guest-programmed PWM registers. */
    uint32_t pwm_ctrl;
    uint32_t pwm_duty_cycle;

    /* Guest-programmed tach control register. */
    uint32_t tach_ctrl;

    /*
     * Cached duty ratio in [0..1000] (per-mille). Updated whenever the guest
     * writes PWM_CTRL / PWM_DUTY_CYCLE. 0 means the output is inactive.
     */
    uint32_t duty_permille;

    /*
     * Target RPM at 100% duty, exposed as QOM property "fanN_target_rpm".
     * The actual RPM the guest reads is target_rpm * duty / 1000.
     */
    uint32_t target_rpm;
} AspeedPWMChannel;

typedef struct AspeedPWMState {
    /* <private> */
    SysBusDevice parent;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;

    /*
     * Reference clock driving the PWM/tach counter (Hz). Wires together the
     * formula in the Linux driver; defaults to 24 MHz (APB) and is overridable
     * via the "clock-freq" qdev property so boards can reflect SCU muxing.
     */
    uint32_t clock_freq;

    /* Pulses-per-revolution used to convert RPM to raw tach counter. */
    uint32_t pulse_per_revolution;

    AspeedPWMChannel ch[ASPEED_PWM_NR_CHANNELS];
} AspeedPWMState;

#endif /* ASPEED_PWM_H */
