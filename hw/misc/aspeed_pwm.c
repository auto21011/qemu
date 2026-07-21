/*
 * ASPEED PWM/Tachometer Controller (AST2600 generation)
 *
 * Copyright (C) 2017-2021 IBM Corp.
 * Copyright (C) 2025 - AST2600 PWM + Fan Tach model with QOM-adjustable RPM.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This models the AST2600 "aspeed,ast2600-pwm-tach" IP block described by
 * the Linux driver drivers/hwmon/aspeed-g6-pwm-tach.c. The block exposes 16
 * independent channels at stride 0x10 in a 0x100 region:
 *
 *   ch N:  base + N*0x10 + 0x00  ->  PWM_CTRL
 *          base + N*0x10 + 0x04  ->  PWM_DUTY_CYCLE
 *          base + N*0x10 + 0x08  ->  TACH_CTRL
 *          base + N*0x10 + 0x0C  ->  TACH_STS
 *
 * Model behaviour (Plan B):
 *   * PWM registers are stored as-written so guest reads round-trip. A cached
 *     duty_permille is recomputed on every PWM_CTRL / PWM_DUTY_CYCLE write.
 *   * TACH_STS reads synthesise a "measurement ready" answer using the
 *     inverse of the Linux driver's RPM formula:
 *
 *       kernel: rpm = (clk * 60) / (2 * raw * tach_div * pulse_pr)
 *       here:   raw = (clk * 60) / (2 * effective_rpm * tach_div * pulse_pr)
 *
 *     where effective_rpm = fanN_target_rpm * duty_permille / 1000, so raising
 *     the guest-programmed duty makes the guest observe a proportionally higher
 *     RPM. raw is reported in TACH_STS[19:0] with FULL_MEASUREMENT |
 *     VALUE_UPDATE set so the Linux driver's polling path returns immediately.
 *   * Each fan's full-speed RPM ("fan{N}_target_rpm") is exposed as a QOM
 *     property (settable via QMP `qom-set`), making fans externally adjustable.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/misc/aspeed_pwm.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "trace.h"

#define ASPEED_PWM_DEFAULT_CLOCK_HZ   24000000
#define ASPEED_PWM_DEFAULT_PULSE_REV  2
#define ASPEED_PWM_DEFAULT_TARGET_RPM 5000

/* ---- Per-channel helpers ----------------------------------------------- */

static uint32_t aspeed_pwm_calc_duty_permille(AspeedPWMChannel *c)
{
    uint32_t duty = c->pwm_duty_cycle;
    uint32_t ctrl = c->pwm_ctrl;
    uint32_t rising  = duty & ASPEED_PWM_DUTY_RISING_MASK;
    uint32_t falling = (duty & ASPEED_PWM_DUTY_FALLING_MASK) >>
                       ASPEED_PWM_DUTY_FALLING_SHIFT;
    uint32_t period  = (duty & ASPEED_PWM_DUTY_PERIOD_MASK) >>
                       ASPEED_PWM_DUTY_PERIOD_SHIFT;
    bool clk_en  = ctrl & ASPEED_PWM_CTRL_CLK_ENABLE;
    bool pin_en  = ctrl & ASPEED_PWM_CTRL_PIN_ENABLE;
    bool inverse = ctrl & ASPEED_PWM_CTRL_INVERSE;
    uint32_t ratio;

    if (!clk_en || !pin_en) {
        return 0;
    }

    if (period == 0) {
        period = 0xff;
    }

    if (rising == falling) {
        /* 100% duty (driver convention). */
        ratio = 1000;
    } else {
        if (falling > period) {
            ratio = 1000;
        } else if (falling <= rising) {
            ratio = 0;
        } else {
            ratio = (uint64_t)(falling - rising) * 1000 / (period + 1);
            if (ratio > 1000) {
                ratio = 1000;
            }
        }
    }

    if (inverse) {
        ratio = 1000 - ratio;
    }
    return ratio;
}

/*
 * Inverse of the Linux RPM formula (aspeed_tach_val_to_rpm):
 *   rpm = (clk * 60) / (raw * tach_div * pulse_pr)
 * -> raw = (clk * 60) / (effective_rpm * tach_div * pulse_pr)
 *
 * Note: the driver does NOT have a factor of 2 in the denominator.
 * tach_div = 4^n where n = CLK_DIV_T (0..15).
 * pulse_pr defaults to 2 (matching the Linux driver).
 */
static uint32_t aspeed_pwm_calc_tach_raw(AspeedPWMState *s,
                                         AspeedPWMChannel *c,
                                             uint32_t effective_rpm)
{
    uint32_t tach_div_n = (c->tach_ctrl & ASPEED_TACH_CTRL_CLK_DIV_T_MASK) >>
                          ASPEED_TACH_CTRL_CLK_DIV_T_SHIFT;
    uint64_t tach_div;
    uint64_t raw;

    /* tach_div = 4^n; protect against pathological shifts. */
    if (tach_div_n > 15) {
        tach_div_n = 15;
    }
    tach_div = (uint64_t)1u << (2 * tach_div_n);

    if (effective_rpm == 0) {
        /* Fan idle: return saturated max count. */
        return ASPEED_TACH_STS_VALUE_MASK;
    }

    raw = (uint64_t)s->clock_freq * 60;
    raw /= (uint64_t)effective_rpm * tach_div *
           (uint64_t)s->pulse_per_revolution;

    if (raw > ASPEED_TACH_STS_VALUE_MASK) {
        raw = ASPEED_TACH_STS_VALUE_MASK;
    }
    return (uint32_t)raw;
}

static uint32_t aspeed_pwm_effective_rpm(AspeedPWMChannel *c)
{
    return (uint64_t)c->target_rpm * c->duty_permille / 1000;
}

/* Rebuild cached duty; caller passes the channel index for tracing. */
static void aspeed_pwm_channel_recalc(AspeedPWMState *s, int ch)
{
    AspeedPWMChannel *c = &s->ch[ch];
    uint32_t new_duty = aspeed_pwm_calc_duty_permille(c);
    if (new_duty != c->duty_permille) {
        trace_aspeed_pwm_duty_update(ch, c->duty_permille, new_duty);
        c->duty_permille = new_duty;
    }
}

/* Commits a write to PWM_CTRL / PWM_DUTY_CYCLE. */
static void aspeed_pwm_channel_pwm_write(AspeedPWMState *s, int ch,
                                         hwaddr reg, uint32_t val)
{
    AspeedPWMChannel *c = &s->ch[ch];
    switch (reg & 0xc) {
    case 0x0: /* PWM_CTRL */
        c->pwm_ctrl = val;
        break;
    case 0x4: /* PWM_DUTY_CYCLE */
        c->pwm_duty_cycle = val;
        break;
    default:
        g_assert_not_reached();
    }
    aspeed_pwm_channel_recalc(s, ch);
}

/* ---- MMIO ------------------------------------------------------------- */

static uint64_t aspeed_pwm_read(void *opaque, hwaddr addr, unsigned int size)
{
    AspeedPWMState *s = ASPEED_PWM(opaque);
    uint32_t ch = addr >> 4;
    uint32_t off = addr & 0xc;
    uint32_t val = 0;

    if (ch >= ASPEED_PWM_NR_CHANNELS || addr >= 0x100) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: out-of-bounds read at 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }

    switch (off) {
    case 0x0: /* PWM_CTRL */
        val = s->ch[ch].pwm_ctrl;
        break;
    case 0x4: /* PWM_DUTY_CYCLE */
        val = s->ch[ch].pwm_duty_cycle;
        break;
    case 0x8: /* TACH_CTRL */
        val = s->ch[ch].tach_ctrl;
        break;
    case 0xc: { /* TACH_STS */
        uint32_t raw, sts;
        /*
         * Synthesise a "measurement ready" answer keyed on the inverse RPM
         * formula. The READ returns VALUE + FULL_MEASUREMENT + VALUE_UPDATE.
         * ISR is cleared on read (write-one-to-clear semantics also honoured
         * in the write path).
         */
        bool tach_en = s->ch[ch].tach_ctrl & ASPEED_TACH_CTRL_ENABLE;
        if (tach_en) {
            raw = aspeed_pwm_calc_tach_raw(s, &s->ch[ch],
                                           aspeed_pwm_effective_rpm(&s->ch[ch]));
        } else {
            raw = 0;
        }
        sts = ASPEED_TACH_STS_FULL_MEASUREMENT | ASPEED_TACH_STS_VALUE_UPDATE |
              (raw & ASPEED_TACH_STS_VALUE_MASK);
        if (s->ch[ch].pwm_ctrl & ASPEED_PWM_CTRL_PIN_ENABLE) {
            sts |= ASPEED_TACH_STS_PWM_OEN | ASPEED_TACH_STS_PWM_OUT;
        }
        val = sts;
        trace_aspeed_pwm_tach_update(ch, aspeed_pwm_effective_rpm(&s->ch[ch]),
                                     raw);
        break;
    }
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown read at 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    trace_aspeed_pwm_read(addr, val);
    return val;
}

static void aspeed_pwm_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned int size)
{
    AspeedPWMState *s = ASPEED_PWM(opaque);
    uint32_t ch = addr >> 4;
    uint32_t off = addr & 0xc;
    uint32_t val = data;

    trace_aspeed_pwm_write(addr, val);

    if (ch >= ASPEED_PWM_NR_CHANNELS || addr >= 0x100) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: out-of-bounds write at 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return;
    }

    switch (off) {
    case 0x0: /* PWM_CTRL */
    case 0x4: /* PWM_DUTY_CYCLE */
        aspeed_pwm_channel_pwm_write(s, ch, off, val);
        break;
    case 0x8: /* TACH_CTRL */
        s->ch[ch].tach_ctrl = val;
        break;
    case 0xc: /* TACH_STS - write 1 to clear ISR */
        if (val & ASPEED_TACH_STS_ISR) {
            /* ISR is write-1-to-clear; other bits RW0C/no-op. */
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown write at 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps aspeed_pwm_ops = {
    .read = aspeed_pwm_read,
    .write = aspeed_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

/* ---- QOM properties: fan{N}_target_rpm --------------------------------- */

static void aspeed_pwm_get_target_rpm(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    AspeedPWMChannel *c = opaque;
    int64_t value = c->target_rpm;

    visit_type_int(v, name, &value, errp);
}

static void aspeed_pwm_set_target_rpm(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    AspeedPWMState *s = ASPEED_PWM(obj);
    AspeedPWMChannel *c = opaque;
    int64_t value;

    if (!visit_type_int(v, name, &value, errp)) {
        return;
    }
    if (value < 0 || value > 0xfffff) {
        error_setg(errp,
                   "fan_target_rpm %" PRId64 " out of range (0..1048575)",
                   value);
        return;
    }

    c->target_rpm = (uint32_t)value;
    trace_aspeed_pwm_set_target_rpm(c - s->ch, c->target_rpm);
}

/* ---- Reset / realize / init ------------------------------------------- */

static void aspeed_pwm_reset_enter(Object *obj, ResetType type)
{
    AspeedPWMState *s = ASPEED_PWM(obj);
    int i;

    for (i = 0; i < ASPEED_PWM_NR_CHANNELS; i++) {
        AspeedPWMChannel *c = &s->ch[i];
        c->pwm_ctrl = 0;
        c->pwm_duty_cycle = 0;
        c->tach_ctrl = 0;
        c->duty_permille = 0;
        /* target_rpm is preserved across reset (QOM-settable); the
         * first reset before realize still initialises it via instance_init.
         */
    }
}

static void aspeed_pwm_reset_hold(Object *obj, ResetType type)
{
    AspeedPWMState *s = ASPEED_PWM(obj);
    qemu_irq_lower(s->irq);
}

static void aspeed_pwm_realize(DeviceState *dev, Error **errp)
{
    AspeedPWMState *s = ASPEED_PWM(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_pwm_ops, s,
                          TYPE_ASPEED_PWM, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
}

/* instance_init: wire per-channel QOM properties and defaults. */
static void aspeed_pwm_init(Object *obj)
{
    AspeedPWMState *s = ASPEED_PWM(obj);
    int i;

    for (i = 0; i < ASPEED_PWM_NR_CHANNELS; i++) {
        char name[32];
        s->ch[i].target_rpm = ASPEED_PWM_DEFAULT_TARGET_RPM;
        s->ch[i].duty_permille = 0;
        snprintf(name, sizeof(name), "fan%d_target_rpm", i);
        object_property_add(obj, name, "int",
                            aspeed_pwm_get_target_rpm,
                            aspeed_pwm_set_target_rpm,
                            NULL, &s->ch[i]);
    }
}

/* ---- VMState ---------------------------------------------------------- */

static bool aspeed_pwm_channel_target_needed(void *opaque)
{
    AspeedPWMChannel *c = opaque;
    return c->target_rpm != ASPEED_PWM_DEFAULT_TARGET_RPM;
}

static const VMStateDescription vmstate_aspeed_pwm_channel_target = {
    .name = "aspeed-pwm/channel-target-rpm",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = aspeed_pwm_channel_target_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(target_rpm, AspeedPWMChannel),
        VMSTATE_END_OF_LIST(),
    },
};

static const VMStateDescription vmstate_aspeed_pwm_channel = {
    .name = "aspeed-pwm/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(pwm_ctrl, AspeedPWMChannel),
        VMSTATE_UINT32(pwm_duty_cycle, AspeedPWMChannel),
        VMSTATE_UINT32(tach_ctrl, AspeedPWMChannel),
        VMSTATE_UINT32(duty_permille, AspeedPWMChannel),
        VMSTATE_END_OF_LIST(),
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_aspeed_pwm_channel_target,
        NULL,
    },
};

static const VMStateDescription vmstate_aspeed_pwm = {
    .name = TYPE_ASPEED_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(clock_freq, AspeedPWMState),
        VMSTATE_UINT32(pulse_per_revolution, AspeedPWMState),
        VMSTATE_STRUCT_ARRAY(ch, AspeedPWMState, ASPEED_PWM_NR_CHANNELS, 1,
                              vmstate_aspeed_pwm_channel, AspeedPWMChannel),
        VMSTATE_END_OF_LIST(),
    },
};

/* ---- Class / type ----------------------------------------------------- */

static const Property aspeed_pwm_properties[] = {
    DEFINE_PROP_UINT32("clock-freq", AspeedPWMState, clock_freq,
                       ASPEED_PWM_DEFAULT_CLOCK_HZ),
    DEFINE_PROP_UINT32("pulse-per-revolution", AspeedPWMState,
                       pulse_per_revolution, ASPEED_PWM_DEFAULT_PULSE_REV),
};

static void aspeed_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = aspeed_pwm_realize;
    rc->phases.enter = aspeed_pwm_reset_enter;
    rc->phases.hold = aspeed_pwm_reset_hold;
    dc->desc = "Aspeed AST2600 PWM/Tachometer Controller";
    dc->vmsd = &vmstate_aspeed_pwm;
    device_class_set_props(dc, aspeed_pwm_properties);
}

static const TypeInfo aspeed_pwm_types[] = {
    {
        .name = TYPE_ASPEED_PWM,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AspeedPWMState),
        .instance_init = aspeed_pwm_init,
        .class_init = aspeed_pwm_class_init,
    }
};

DEFINE_TYPES(aspeed_pwm_types)
