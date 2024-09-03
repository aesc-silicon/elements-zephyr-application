/*
 * Copyright (c) 2024 aesc silicon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "../../common/pwm/fader.h"
#include "../../common/gpio/heartbeat.h"
#include "../../common/gpio/sw0.h"

#ifdef CONFIG_PWM
K_THREAD_DEFINE(fader_tid, FADER_STACKSIZE, fader, NULL, NULL, NULL,
		FADER_PRIORITY, 0, 0);
#endif

#ifdef CONFIG_GPIO
K_THREAD_DEFINE(heartbeat_tid, HEARTBEAT_STACKSIZE, heartbeat, NULL, NULL, NULL,
		HEARTBEAT_PRIORITY, 0, 0);

K_THREAD_DEFINE(software0_button_tid, SOFTWARE0_BUTTON_STACKSIZE,
		software0_button, NULL, NULL, NULL, SOFTWARE0_BUTTON_PRIORITY, 0, 0);
#endif
