/*
 * Copyright (c) 2023 aesc silicon
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT elements_pwm

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/pwm.h>

#include <elements/drivers/ip_identification.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_PWM_LOG_LEVEL);

#define CYCLES_PER_SECOND	1000000

struct pwm_elements_config {
	DEVICE_MMIO_NAMED_ROM(regs);
	uint32_t clock_frequency;
	uint8_t channels;
};

struct pwm_elements_channel_regs {
	uint32_t control;
	uint32_t period_timer;
	uint32_t pulse_timer;
};

struct pwm_elements_regs {
	uint32_t config;
	uint32_t clock_div;
	struct pwm_elements_channel_regs channels[];
};

#define ELEMENTS_PWM_CHANNEL_ENABLE	0
#define ELEMENTS_PWM_CHANNEL_INVERT	1

#define DEV_CFG(dev)							       \
	((struct pwm_elements_config *)(dev)->config)
#define DEV_PWM(dev)							       \
	((struct pwm_elements_regs *)(DEVICE_MMIO_NAMED_GET(dev, regs)))
#define DEV_PWM_CHANNEL(dev, no)					       \
	((struct pwm_elements_channel_regs *)&((DEV_PWM(dev))->channels[no]))


static int pwm_elements_set_cycles(const struct device *dev, uint32_t channel,
			      uint32_t period_cycles, uint32_t pulse_cycles,
			      pwm_flags_t flags)
{
	const struct pwm_elements_config *cfg = DEV_CFG(dev);
	struct pwm_elements_channel_regs *channel_regs;
	uint32_t control;

	if (channel >= cfg->channels)
		return -EINVAL;

	channel_regs = DEV_PWM_CHANNEL(dev, channel);
	control = channel_regs->control;

	/* Select PWM inverted polarity (ie. active-low pulse). */
	if (flags & PWM_POLARITY_INVERTED) {
		control |= BIT(ELEMENTS_PWM_CHANNEL_INVERT);
	} else {
		control &= ~BIT(ELEMENTS_PWM_CHANNEL_INVERT);
	}

	/* If pulse_cycles is 0, switch PWM off and return. */
	if (pulse_cycles == 0) {
		control &= ~BIT(ELEMENTS_PWM_CHANNEL_ENABLE);
		channel_regs->control = control;
		return 0;
	}
	control |= BIT(ELEMENTS_PWM_CHANNEL_ENABLE);

	channel_regs->period_timer = period_cycles - 1;
	channel_regs->pulse_timer = pulse_cycles;
	channel_regs->control = control;

	return 0;
}

static int pwm_elements_get_cycles_per_sec(const struct device *dev,
				      uint32_t channel, uint64_t *cycles)
{
	*cycles = CYCLES_PER_SECOND;

	return 0;
}

static int pwm_elements_init(const struct device *dev)
{
	volatile struct pwm_elements_config *cfg = DEV_CFG(dev);
	volatile uintptr_t *base_addr = (volatile uintptr_t *)DEV_PWM(dev);
	volatile struct pwm_elements_regs *pwm;
	uint32_t clock_divider;

	DEVICE_MMIO_NAMED_MAP(dev, regs, K_MEM_CACHE_NONE);
	LOG_INF("IP core version: %i.%i.%i.",
		ip_id_get_major_version(base_addr),
		ip_id_get_minor_version(base_addr),
		ip_id_get_patchlevel(base_addr)
	);
	cfg->regs.addr = ip_id_relocate_driver(base_addr);
	LOG_INF("Relocate driver to address 0x%lx.", cfg->regs.addr);

	pwm = DEV_PWM(dev);

	cfg->channels = pwm->config & 0xFF;

	LOG_INF("Controller has %i channels.", cfg->channels);

	clock_divider = (cfg->clock_frequency / CYCLES_PER_SECOND) - 1;
	LOG_INF("Set clock divider to %i", clock_divider);
	pwm->clock_div = clock_divider;

	return 0;
}

/* PWM driver APIs structure */
static const struct pwm_driver_api pwm_elements_driver_api = {
	.set_cycles = pwm_elements_set_cycles,
	.get_cycles_per_sec = pwm_elements_get_cycles_per_sec,
};

#define PWM_ELEMENTS_INIT(no)						       \
	static const struct pwm_elements_config config##no = {		       \
		DEVICE_MMIO_NAMED_ROM_INIT(regs,			       \
					   DT_INST(no, elements_pwm)),	       \
		.clock_frequency = DT_INST_PROP(no, clock_frequency),	       \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(no, pwm_elements_init,			       \
			      NULL, NULL, &config##no,			       \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,	       \
			      &pwm_elements_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_ELEMENTS_INIT)
