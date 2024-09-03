/*
 * Copyright (c) 2024 aesc silicon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/__assert.h>
#include "fader.h"

static const struct pwm_dt_spec red_pwm_led = PWM_DT_SPEC_GET(FADER_RED_NODE);
static const struct pwm_dt_spec green_pwm_led = PWM_DT_SPEC_GET(FADER_GREEN_NODE);
static const struct pwm_dt_spec blue_pwm_led = PWM_DT_SPEC_GET(FADER_BLUE_NODE);

int fader(void)
{
	uint32_t pulse_red, pulse_green, pulse_blue; /* pulse widths */
	int ret;

	if (!pwm_is_ready_dt(&red_pwm_led)) {
		printk("Error: PWM device %s is not ready\n",
		       red_pwm_led.dev->name);
		return 0;
	}

	if (!pwm_is_ready_dt(&green_pwm_led)) {
		printk("Error: PWM device %s is not ready\n",
		       green_pwm_led.dev->name);
		return 0;
	}

	if (!pwm_is_ready_dt(&blue_pwm_led)) {
		printk("Error: PWM device %s is not ready\n",
		       blue_pwm_led.dev->name);
		return 0;
	}

	ret = pwm_set_pulse_dt(&red_pwm_led, 0);
	if (ret != 0) {
		printk("Error %d: red init failed\n", ret);
		return 0;
	}

	ret = pwm_set_pulse_dt(&green_pwm_led, 0);
	if (ret != 0) {
		printk("Error %d: green init failed\n", ret);
		return 0;
	}

	ret = pwm_set_pulse_dt(&blue_pwm_led, 0);
	if (ret != 0) {
		printk("Error %d: blue init failed\n", ret);
		return 0;
	}

	while (1) {
		for (pulse_red = 0U; pulse_red <= red_pwm_led.period;
		     pulse_red += STEP_SIZE) {

			ret = pwm_set_pulse_dt(&red_pwm_led, pulse_red);
			if (ret != 0) {
				printk("Error %d: red write failed\n", ret);
				return 0;
			}

			for (pulse_green = 0U;
			     pulse_green <= green_pwm_led.period;
			     pulse_green += STEP_SIZE) {
				ret = pwm_set_pulse_dt(&green_pwm_led,
						       pulse_green);
				if (ret != 0) {
					printk("Error %d: green write failed\n",
					       ret);
					return 0;
				}

				for (pulse_blue = 0U;
				     pulse_blue <= blue_pwm_led.period;
				     pulse_blue += STEP_SIZE) {
					ret = pwm_set_pulse_dt(&blue_pwm_led,
							       pulse_blue);
					if (ret != 0) {
						printk("Error %d: "
						       "blue write failed\n",
						       ret);
						return 0;
					}
					k_sleep(K_MSEC(500));
				}
			}
		}
	}

	return 0;
}
