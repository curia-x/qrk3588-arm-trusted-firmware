/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include <plat/common/platform.h>
#include <tools_share/tbbr_oid.h>

#include <qrk3588_rotpk.h>

int plat_get_rotpk_info(void *cookie, void **key_ptr,
		       unsigned int *key_len, unsigned int *flags)
{
	(void)cookie;
	if ((key_ptr == NULL) || (key_len == NULL) || (flags == NULL))
		return -EINVAL;

	*key_ptr = (void *)qrk3588_rotpk;
	*key_len = sizeof(qrk3588_rotpk);
	*flags = ROTPK_IS_HASH;
	return 0;
}

static int valid_counter(void *cookie)
{
	return (cookie != NULL) &&
	       ((strcmp(cookie, TRUSTED_FW_NVCOUNTER_OID) == 0) ||
		(strcmp(cookie, NON_TRUSTED_FW_NVCOUNTER_OID) == 0));
}

/* Development policy only: no trusted persistent anti-rollback storage. */
int plat_get_nv_ctr(void *cookie, unsigned int *nv_ctr)
{
	if (!valid_counter(cookie) || (nv_ctr == NULL))
		return -EINVAL;
	*nv_ctr = 0U;
	return 0;
}

int plat_set_nv_ctr(void *cookie, unsigned int nv_ctr)
{
	if (!valid_counter(cookie))
		return -EINVAL;
	return (nv_ctr == 0U) ? 0 : -EPERM;
}

int plat_get_mbedtls_heap(void **heap_addr, size_t *heap_size)
{
	return get_mbedtls_heap_helper(heap_addr, heap_size);
}
