/*
 * Copyright (c) 2024 aesc silicon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* size of stack area used by each thread */
#define NOTHING_STACKSIZE 2048

/* scheduling priority used by each thread */
#define NOTHING_PRIORITY 5

void nothing(void)
{
	while (1) {
		k_sleep(K_SECONDS(1));
	}
}

K_THREAD_DEFINE(nothing_tid, NOTHING_STACKSIZE, nothing, NULL, NULL, NULL,
		NOTHING_PRIORITY, 0, 0);
