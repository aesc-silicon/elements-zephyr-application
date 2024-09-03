/*
 * Copyright (c) 2024 aesc silicon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "../../common/gpio/heartbeat.h"

#ifdef CONFIG_GPIO
K_THREAD_DEFINE(heartbeat_tid, HEARTBEAT_STACKSIZE, heartbeat, NULL, NULL, NULL,
		HEARTBEAT_PRIORITY, 0, 0);
#endif
