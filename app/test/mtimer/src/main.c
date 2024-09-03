/*
 * Copyright (c) 2024 aesc silicon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

/* size of stack area used by each thread */
#define MTIMER_STACKSIZE 2048

/* scheduling priority used by each thread */
#define MTIMER_PRIORITY 5

#define HEARTBEAT_NODE		DT_PATH(gpio_leds, heartbeat)

#if !DT_NODE_HAS_STATUS(HEARTBEAT_NODE, okay)
#error "Unsupported board: heartbeat devicetree alias is not defined"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(HEARTBEAT_NODE, gpios);

void mtimer(void) {
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
		k_sleep(K_MSEC(1));

		ret = gpio_pin_set_dt(&led, 0);
		if (ret < 0) {
			printk("Failed to turn LED off: %d", ret);
			return;
		}
		k_sleep(K_MSEC(5));

		ret = gpio_pin_set_dt(&led, 1);
		if (ret < 0) {
			printk("Failed to turn LED on: %d", ret);
			return;
		}
		k_sleep(K_MSEC(10));

		ret = gpio_pin_set_dt(&led, 0);
		if (ret < 0) {
			printk("Failed to turn LED off: %d", ret);
			return;
		}
		k_sleep(K_SECONDS(1));
	}
}

K_THREAD_DEFINE(mtimer_tid, MTIMER_STACKSIZE, mtimer, NULL, NULL, NULL,
		MTIMER_PRIORITY, 0, 0);
