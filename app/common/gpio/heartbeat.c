/*
 * Copyright (c) 2024 aesc silicon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include "heartbeat.h"

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(HEARTBEAT_NODE, gpios);

void heartbeat(void) {
	int ret;

	if (!device_is_ready(led.port)) {
		printk("LED device is not ready!");
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printk("Failed to configure LED pin: %d!", ret);
		return;
	}

	while (1) {
		ret = gpio_pin_set_dt(&led, 1);
		if (ret < 0) {
			printk("Failed to turn LED on: %d", ret);
			return;
		}
		k_sleep(K_MSEC(150));

		ret = gpio_pin_set_dt(&led, 0);
		if (ret < 0) {
			printk("Failed to turn LED off: %d", ret);
			return;
		}
		k_sleep(K_MSEC(50));

		ret = gpio_pin_set_dt(&led, 1);
		if (ret < 0) {
			printk("Failed to turn LED on: %d", ret);
			return;
		}
		k_sleep(K_MSEC(150));

		ret = gpio_pin_set_dt(&led, 0);
		if (ret < 0) {
			printk("Failed to turn LED off: %d", ret);
			return;
		}
		k_sleep(K_SECONDS(1));
	}
}
