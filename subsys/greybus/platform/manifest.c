/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/byteorder.h>

#ifdef CONFIG_GREYBUS_MANIFEST_BUILTIN
static const unsigned char greybus_manifest_builtin[] = {
#include "greybus_mnfb.inc"
};
#ifdef CONFIG_GREYBUS_CLICK_MANIFEST_BUILTIN
static unsigned char greybus_manifest_click1_fragment_builtin[] = {
#include "greybus_click_fragment1_mnfb.inc"
};
static unsigned char greybus_manifest_click2_fragment_builtin[] = {
#include "greybus_click_fragment2_mnfb.inc"
};
#else
#define greybus_manifest_click1_fragment_builtin NULL
#define greybus_manifest_click2_fragment_builtin NULL
#endif
#else
#define greybus_manifest_builtin NULL
#endif /* CONFIG_GREYBUS_MANIFEST_BUILTIN */

#include "../greybus-manifest.h"
#include "mikrobus.h"

int manifest_get(uint8_t **mnfb, size_t *mnfb_size)
{
	int r = -ENOENT;

	if (IS_ENABLED(CONFIG_GREYBUS_MANIFEST_BUILTIN)) {
		*mnfb = (uint8_t *)greybus_manifest_builtin;
		*mnfb_size = sizeof(greybus_manifest_builtin);
		r = 0;
	}

	return r;
}

int manifest_get_fragment(uint8_t **mnfb, size_t *mnfb_size, uint8_t id)
{
	int r = -ENOENT;

	if (IS_ENABLED(CONFIG_GREYBUS_CLICK_MANIFEST_BUILTIN)) {
		if(id == 0){
			*mnfb = (uint8_t *)greybus_manifest_click1_fragment_builtin;
			*mnfb_size = sizeof(greybus_manifest_click1_fragment_builtin);
		}
		else if(id == 1){
			*mnfb = (uint8_t *)greybus_manifest_click2_fragment_builtin;
			*mnfb_size = sizeof(greybus_manifest_click2_fragment_builtin);
		}
		else 
			return id;
		r = 0;
	}
	else
		r = manifest_get_fragment_clickid(mnfb, mnfb_size, id);
	return r;
}
