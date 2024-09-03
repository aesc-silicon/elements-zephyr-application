/*
 * Copyright (c) 2024 aesc silicon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/printk.h>

/* size of stack area used by each thread */
#define STACKSIZE 2048

/* scheduling priority used by each thread */
#define PRIORITY 7

void startup_reset(void)
{
	printk("Triggering warm reboot.\n");
	k_sleep(K_MSEC(5));
	sys_reboot(SYS_REBOOT_WARM);

	k_sleep(K_MSEC(1));
	printk("Reset failed!\n");

	while (1) {
		k_sleep(K_SECONDS(1));
	}
}

K_THREAD_DEFINE(startup_reset_tid, STACKSIZE, startup_reset, NULL, NULL, NULL,
		PRIORITY, 0, 0);
