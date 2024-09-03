/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include "sw0.h"


static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(
	SOFTWARE0_BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;

void software0_handler(const struct device *gpio, struct gpio_callback *cb,
		       uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int software0_button(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: Failed to configure sw0 button\n", ret);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_LOW_0);
	if (ret != 0) {
		printk("Error %d: Failed to configure interrupt on pin 3\n",
		       ret);
		return 0;
	}

	gpio_init_callback(&button_cb_data, software0_handler, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
