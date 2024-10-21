/**
 *  Copyright (c) 2024 aesc silicon
 *
 *  SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <zephyr/sys/__assert.h>
#include <lib/ip_identification.h>


struct elements_ip_id_header {
	uint32_t header;
	uint32_t version;
};


unsigned int ip_id_get_major_version(volatile uintptr_t *addr)
{
	volatile struct elements_ip_id_header *header =
		(struct elements_ip_id_header *)addr;
	return (header->version >> 24) & 0xFF;
}

unsigned int ip_id_get_minor_version(volatile uintptr_t *addr)
{
	volatile struct elements_ip_id_header *header =
		(struct elements_ip_id_header *)addr;
	return (header->version >> 16) & 0xFF;
}

unsigned int ip_id_get_patchlevel(volatile uintptr_t *addr)
{
	volatile struct elements_ip_id_header *header =
		(struct elements_ip_id_header *)addr;
	return header->version & 0xFFFF;
}

 unsigned int ip_id_get_api_version(volatile uintptr_t *addr)
{
	volatile struct elements_ip_id_header *header =
		(struct elements_ip_id_header *)addr;
	return (header->header >> 24) & 0xFF;
}

unsigned int ip_id_get_header_length(volatile uintptr_t *addr)
{
	volatile struct elements_ip_id_header *header =
		(struct elements_ip_id_header *)addr;
	return (header->header >> 16) & 0xFF;
}

static inline unsigned int ip_id_get_id(volatile uintptr_t *addr)
{
	volatile struct elements_ip_id_header *header =
		(struct elements_ip_id_header *)addr;
	return header->header & 0xFFFF;
}

/* Helper function */

void ip_id_get_version(volatile uintptr_t *addr, char *version)
{
	int res;
	int major = ip_id_get_major_version(addr);
	int minor = ip_id_get_minor_version(addr);
	int patch = ip_id_get_patchlevel(addr);

	res = sprintf(version, "%i.%i.%i", major, minor, patch);
	__ASSERT_NO_MSG(res != sizeof(version));
}

uintptr_t ip_id_relocate_driver(volatile uintptr_t *addr)
{
	return (uintptr_t)addr + ip_id_get_header_length(addr);
}
