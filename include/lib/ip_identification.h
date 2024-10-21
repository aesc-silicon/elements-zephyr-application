/**
 *  Copyright (c) 2024 aesc silicon
 *
 *  SPDX-License-Identifier: Apache-2.0
 */

#ifndef LIB_IP_IDENTIFICATION_H_
#define LIB_IP_IDENTIFICATION_H_

unsigned int ip_id_get_major_version(volatile uintptr_t *addr);
unsigned int ip_id_get_minor_version(volatile uintptr_t *addr);
unsigned int ip_id_get_patchlevel(volatile uintptr_t *addr);
unsigned int ip_id_get_api_version(volatile uintptr_t *addr);
unsigned int ip_id_get_header_length(volatile uintptr_t *addr);

void ip_id_get_version(volatile uintptr_t *addr, char *version);
uintptr_t ip_id_relocate_driver(volatile uintptr_t *addr);

#endif /* LIB_IP_IDENTIFICATION_H_ */
