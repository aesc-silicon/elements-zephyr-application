# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/pwm.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PWM_ELEMENTS pwm_elements.c)
