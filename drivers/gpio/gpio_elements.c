/*
 * Copyright (c) 2024 aesc silicon
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file GPIO driver for the Elements
 */

#define DT_DRV_COMPAT elements_gpio

#include <errno.h>

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>

#include <lib/ip_identification.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_GPIO_LOG_LEVEL);

typedef void (*elements_cfg_func_t)(void);

struct gpio_elements_config {
	void (*bank_config)(const struct device *dev);
	struct gpio_driver_config common;

	DEVICE_MMIO_NAMED_ROM(regs);

	uint32_t available_pins;
	uint32_t available_banks;
	uint32_t irq_no;
	elements_cfg_func_t cfg_func;
};

struct gpio_elements_bank_regs {
	uint32_t read;
	uint32_t write;
	uint32_t direction;
	uint32_t high_ip;
	uint32_t high_ie;
	uint32_t low_ip;
	uint32_t low_ie;
	uint32_t rise_ip;
	uint32_t rise_ie;
	uint32_t fall_ip;
	uint32_t fall_ie;
};

struct gpio_elements_regs {
	uint32_t banks_pins;
	struct gpio_elements_bank_regs bank[];
};

struct gpio_elements_data {
	struct gpio_driver_data common;

	DEVICE_MMIO_NAMED_RAM(regs);

	sys_slist_t cb;
};

#define DEV_CFG(dev)						       \
	((struct gpio_elements_config *)((dev)->config))
#define DEV_GPIO(dev)							       \
	((struct gpio_elements_regs *)DEVICE_MMIO_NAMED_GET(dev, regs))
#define DEV_GPIO_BANK(dev)						       \
	((struct gpio_elements_bank_regs *)(DEV_GPIO(dev))->bank)
#define DEV_DATA(dev) ((struct gpio_elements_data *)(dev)->data)


static int gpio_elements_config(const struct device *dev, gpio_pin_t pin,
				gpio_flags_t flags)
{
	volatile struct gpio_elements_bank_regs *bank = DEV_GPIO_BANK(dev);

	/* Configure gpio direction */
	if (flags & GPIO_OUTPUT)
		bank->direction |= BIT(pin);
	else
		bank->direction &= ~BIT(pin);

	return 0;
}

static int gpio_elements_port_get_raw(const struct device *dev,
				   gpio_port_value_t *value)
{
	volatile struct gpio_elements_bank_regs *bank = DEV_GPIO_BANK(dev);

	*value = bank->read;

	return 0;
}

static int gpio_elements_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	volatile struct gpio_elements_bank_regs *bank = DEV_GPIO_BANK(dev);

	bank->write = (bank->write & ~mask) | (value & mask);

	return 0;
}

static int gpio_elements_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t mask)
{
	volatile struct gpio_elements_bank_regs *bank = DEV_GPIO_BANK(dev);

	bank->write |= mask;

	return 0;
}

static int gpio_elements_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t mask)
{
	volatile struct gpio_elements_bank_regs *bank = DEV_GPIO_BANK(dev);

	bank->write &= ~mask;

	return 0;
}

static int gpio_elements_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t mask)
{
	volatile struct gpio_elements_bank_regs *bank = DEV_GPIO_BANK(dev);

	bank->write ^= mask;

	return 0;
}

#ifdef CONFIG_GPIO_ELEMENTS_INTERRUPT

static void gpio_elements_irq_handler(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct gpio_elements_data *data = DEV_DATA(dev);
	volatile struct gpio_elements_bank_regs *bank = DEV_GPIO_BANK(dev);
	uint32_t pin_mask = 0;

	pin_mask |= bank->rise_ip;
	pin_mask |= bank->fall_ip;
	pin_mask |= bank->high_ip;
	pin_mask |= bank->low_ip;

	for (int pin = 0; pin < 32; pin++) {
		if (pin_mask & (0x1 << pin)) {
			/* Fire callback for pin */
			gpio_fire_callbacks(&data->cb, dev, BIT(pin));
			/*
			 * Write to either the rise_ip, fall_ip, high_ip or
			 * low_ip registers to indicate to GPIO controller
			 * that interrupt for the corresponding pin has been
			 * handled.
			 */
			bank->rise_ip = BIT(pin);
			bank->fall_ip = BIT(pin);
			bank->high_ip = BIT(pin);
			bank->low_ip = BIT(pin);
		}
	}
}

static int gpio_elements_pin_interrupt_configure(const struct device *dev,
						 gpio_pin_t pin,
						 enum gpio_int_mode int_mode,
						 enum gpio_int_trig int_trig)
{
	volatile struct gpio_elements_bank_regs *bank = DEV_GPIO_BANK(dev);
	volatile struct gpio_elements_config *cfg = DEV_CFG(dev);

	/* Disable all interrupts */
	bank->high_ie &= ~BIT(pin);
	bank->low_ie &= ~BIT(pin);
	bank->rise_ie &= ~BIT(pin);
	bank->fall_ie &= ~BIT(pin);

	/* Disable interrupt for the pin at PLIC level */
	if (int_mode & GPIO_INT_MODE_DISABLED) {
		irq_disable(irq_to_level_2(cfg->irq_no));
		return 0;
	}

	if (int_mode & GPIO_INT_EDGE) {
		/* Clear pending pits */
		bank->rise_ip = BIT(pin);
		bank->fall_ip = BIT(pin);

		/* Double edge */
		if ((int_trig & GPIO_INT_TRIG_BOTH) == GPIO_INT_TRIG_BOTH) {
			bank->rise_ie |= BIT(pin);
			bank->fall_ie |= BIT(pin);
		/* Rising edge */
		} else if (int_trig & GPIO_INT_TRIG_HIGH) {
			bank->rise_ie |= BIT(pin);
		/* Falling edge */
		} else {
			bank->fall_ie |= BIT(pin);
		}
	} else {
		/* Clear pending pits */
		bank->high_ip = BIT(pin);
		bank->low_ip = BIT(pin);

		/* Level High ? */
		if (int_trig & GPIO_INT_TRIG_HIGH) {
			bank->high_ie |= BIT(pin);
		} else {
			bank->low_ie |= BIT(pin);
		}
	}
	irq_enable(irq_to_level_2(cfg->irq_no));

	return 0;
}

static int gpio_elements_manage_callback(const struct device *dev,
					 struct gpio_callback *callback,
					 bool set)
{
	struct gpio_elements_data *data = DEV_DATA(dev);

	return gpio_manage_callback(&data->cb, callback, set);
}

#endif /* CONFIG_GPIO_INTERRUPT */

/**
 * @brief Initialize a GPIO controller
 *
 * Perform basic initialization of a GPIO controller
 *
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpio_elements_init(const struct device *dev)
{
	volatile struct gpio_elements_config *cfg = DEV_CFG(dev);
	volatile uintptr_t *base_addr = (volatile uintptr_t *)DEV_GPIO(dev);
	volatile struct gpio_elements_regs *gpio;
	volatile struct gpio_elements_bank_regs *bank;
	char version[7];

	DEVICE_MMIO_NAMED_MAP(dev, regs, K_MEM_CACHE_NONE);
	ip_id_get_version(base_addr, version);
	LOG_INF("IP core version: %s", version);
	cfg->regs.addr = ip_id_relocate_driver(base_addr);
	LOG_INF("Relocate driver to address 0x%lx.", cfg->regs.addr);

	gpio = DEV_GPIO(dev);
	bank = DEV_GPIO_BANK(dev);

	cfg->available_banks = (gpio->banks_pins >> 16) & 0xFFFF;
	cfg->available_pins = gpio->banks_pins & 0xFFFF;

	LOG_INF("Controller has a total of %i pin(s).", cfg->available_pins);
	LOG_INF("Controller has %i bank(s).", cfg->available_banks);

	if (cfg->available_banks > 1)
		LOG_WRN_ONCE("driver only supports up to 32 IOs.");

	bank->write = 0;
	bank->direction = 0;
	bank->high_ie = 0;
	bank->low_ie = 0;
	bank->rise_ie = 0;
	bank->fall_ie = 0;

#ifdef CONFIG_GPIO_ELEMENTS_INTERRUPT
	bank->high_ip = 0;
	bank->low_ip = 0;
	bank->rise_ip = 0;
	bank->fall_ip = 0;
	cfg->cfg_func();
#endif

	return 0;
}

static const struct gpio_driver_api gpio_elements_driver_api = {
	.pin_configure = gpio_elements_config,
	.port_get_raw = gpio_elements_port_get_raw,
	.port_set_masked_raw = gpio_elements_port_set_masked_raw,
	.port_set_bits_raw = gpio_elements_port_set_bits_raw,
	.port_clear_bits_raw = gpio_elements_port_clear_bits_raw,
	.port_toggle_bits = gpio_elements_port_toggle_bits,
#ifdef CONFIG_GPIO_ELEMENTS_INTERRUPT
	.pin_interrupt_configure = gpio_elements_pin_interrupt_configure,
	.manage_callback = gpio_elements_manage_callback,
#endif
};


#ifdef CONFIG_GPIO_ELEMENTS_INTERRUPT
#define ELEMENTS_GPIO_INIT(no)						     \
	static struct gpio_elements_data gpio_elements_dev_data_##no;	     \
	static void gpio_elements_irq_cfg_func_##no(void);		     \
	static struct gpio_elements_config gpio_elements_dev_cfg_##no = {    \
		DEVICE_MMIO_NAMED_ROM_INIT(regs,			     \
					   DT_INST(no, elements_gpio)),	     \
		.cfg_func = gpio_elements_irq_cfg_func_##no,		     \
		.irq_no = DT_IRQN(DT_INST(no, elements_gpio)),		     \
	};								     \
	DEVICE_DT_INST_DEFINE(no,					     \
			      gpio_elements_init,			     \
			      NULL,					     \
			      &gpio_elements_dev_data_##no,		     \
			      &gpio_elements_dev_cfg_##no,		     \
			      PRE_KERNEL_2,				     \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	     \
			      (void *)&gpio_elements_driver_api);	     \
	static void gpio_elements_irq_cfg_func_##no(void) {		     \
	IRQ_CONNECT(CONFIG_2ND_LVL_ISR_TBL_OFFSET +			     \
		    DT_IRQN(DT_INST(no, elements_gpio)),		     \
		    0,							     \
		    gpio_elements_irq_handler,				     \
		    DEVICE_DT_INST_GET(no),				     \
		    0);							     \
	}
#else
#define ELEMENTS_GPIO_INIT(no)						     \
	static struct gpio_elements_data gpio_elements_dev_data_##no;	     \
	static struct gpio_elements_config gpio_elements_dev_cfg_##no = {    \
		DEVICE_MMIO_NAMED_ROM_INIT(regs,			     \
					   DT_INST(no, elements_gpio)),	     \
	};								     \
	DEVICE_DT_INST_DEFINE(no,					     \
			      gpio_elements_init,			     \
			      NULL,					     \
			      &gpio_elements_dev_data_##no,		     \
			      &gpio_elements_dev_cfg_##no,		     \
			      PRE_KERNEL_2,				     \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	     \
			      (void *)&gpio_elements_driver_api);
#endif

DT_INST_FOREACH_STATUS_OKAY(ELEMENTS_GPIO_INIT)
