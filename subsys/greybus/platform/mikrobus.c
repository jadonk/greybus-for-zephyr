/*
 * Copyright (c) 2021 Vaishnav M A. BeagleBoard.org Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <zephyr.h>

#define DT_DRV_COMPAT zephyr_greybus_mikrobusid
#include <device.h>
#include <devicetree.h>
#include <errno.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <stdlib.h>
#include <sys/byteorder.h>
#include <stddef.h>

#include "w1-gpio.h"

#include "../greybus-manifest.h"

LOG_MODULE_REGISTER(greybus_platform_mikrobusid, 4);

unsigned char *greybus_manifest_click_fragment_clickid[2];

#define US_TO_SYS_CLOCK_HW_CYCLES(us) \
	((uint64_t)sys_clock_hw_cycles_per_sec() * (us) / USEC_PER_SEC + 1)

#define MIKROBUS_ENTER_ID_MODE_DELAY US_TO_SYS_CLOCK_HW_CYCLES(1000)

struct mikrobusid_config {
	unsigned int id;
	const char *cs_gpio_name;
	const char *rst_gpio_name;
    gpio_pin_t cs_pin;
	gpio_pin_t rst_pin;
	gpio_dt_flags_t cs_flags;
	gpio_dt_flags_t rst_flags;
};

struct mikrobusid_context {
	const struct device *cs_gpio;
	const struct device *rst_gpio;
	gpio_pin_t cs_pin;	
	gpio_pin_t rst_pin;
};

static void inline mikrobusid_delay(unsigned int cycles_to_wait)
{
	uint32_t start = k_cycle_get_32();

	/* Wait until the given number of cycles have passed */
	while (k_cycle_get_32() - start < cycles_to_wait) {
	}
}

static int mikrobusid_enter_id_mode(struct mikrobusid_context *context) {
	int i;

	if(!context)
		return -EINVAL;
	/* set RST LOW */
	gpio_pin_set(context->rst_gpio, context->rst_pin, 0);
	return 0;
}

static int mikrobusid_init(const struct device *dev) {

	struct mikrobusid_context *context = dev->data;
	const struct mikrobusid_config *const config =
			(const struct mikrobusid_config *)dev->config;
	struct w1_gpio_master *w1_master;
	struct w1_io_context *w1_io_context;
	struct greybus_manifest_header *manifest_header;
	struct w1_io *w1_gpio_io;
	int err;
	int found;
	size_t count;
	int iter;
	unsigned int mikrobusid = config->id;

	context->cs_gpio = device_get_binding(config->cs_gpio_name);
	if (!context->cs_gpio) {
		LOG_ERR("failed to get CS GPIO device");
		return -EINVAL;
	}

	err = gpio_config(context->cs_gpio, config->cs_pin,
			  config->cs_flags | GPIO_INPUT);
	if (err) {
		LOG_ERR("failed to configure CS GPIO pin (err %d)", err);
		return err;
	}
	
	context->rst_gpio = device_get_binding(config->rst_gpio_name);
	if (!context->rst_gpio) {
		LOG_ERR("failed to get RST GPIO device");
		return -EINVAL;
	}

	err = gpio_config(context->rst_gpio, config->rst_pin,
			  config->rst_flags | GPIO_OUTPUT_LOW);
	if (err) {
		LOG_ERR("failed to configure RST GPIO pin (err %d)", err);
		return err;
	}

	context->cs_pin = config->cs_pin;
	context->rst_pin = config->rst_pin;

	err = mikrobusid_enter_id_mode(context);
	if (err) {
		LOG_ERR("failed to enter mikrobus ID mode (err %d)", err);
		return err;
	}

	w1_master = malloc(sizeof(struct w1_gpio_master));
	if(!w1_master)
		return -ENOMEM;
	w1_master->io_context = malloc(sizeof(struct w1_io_context));
	if(!w1_master->io_context)
		return -ENOMEM;
	w1_io_context = w1_master->io_context;

	w1_io_context->w1_gpio = context->cs_gpio;
	w1_io_context->w1_pin = context->cs_pin;

	w1_gpio_init(w1_master);
	w1_gpio_io = w1_master->w1_gpio_io;

	k_sleep(K_MSEC(100));

	greybus_manifest_click_fragment_clickid[mikrobusid] = malloc(sizeof(struct w1_gpio_master));

	found = w1_gpio_io->reset_bus(w1_io_context);
	if(found) {
		LOG_ERR("w1 device not found");
		return -ENODEV;
	}
	LOG_INF("probed mikrobus Click ID adapter");
	// read greybus manifest header
	count = sizeof(struct greybus_manifest_header);
	w1_gpio_io->write_8(w1_io_context, 0xCC); //skip rom
	w1_gpio_io->write_8(w1_io_context, 0xF0); //mikrobus read eeprom
	w1_gpio_io->write_8(w1_io_context, 0);
	w1_gpio_io->write_8(w1_io_context, 0);
	k_sleep(K_MSEC(1));
	for(iter = 0; iter < count; iter++){
		greybus_manifest_click_fragment_clickid[mikrobusid][iter] = w1_gpio_io->read_8(w1_io_context);
	}
	LOG_HEXDUMP_DBG(greybus_manifest_click_fragment_clickid[mikrobusid], count, "manifest header:");
	manifest_header = (struct greybus_manifest_header *)greybus_manifest_click_fragment_clickid[mikrobusid];
	if(manifest_header->version_major > GREYBUS_VERSION_MAJOR) {
			LOG_ERR("manifest fragment version too new (%hhu.%hhu > %hhu.%hhu)",
						manifest_header->version_major, manifest_header->version_minor,
						GREYBUS_VERSION_MAJOR, GREYBUS_VERSION_MINOR);
			return false;
	}
	LOG_INF("manifest read from click ID adapter, size %d bytes", sys_le16_to_cpu(manifest_header->size));
	found = w1_gpio_io->reset_bus(w1_io_context);
	if(found) {
		LOG_ERR("w1 device not found");
		return -ENODEV;
	}
	count = sys_le16_to_cpu(manifest_header->size);
	greybus_manifest_click_fragment_clickid[mikrobusid] = realloc(greybus_manifest_click_fragment_clickid[mikrobusid], count);
	w1_gpio_io->write_8(w1_io_context, 0xCC); //skip rom
	w1_gpio_io->write_8(w1_io_context, 0xF0); //mikrobus read eeprom
	w1_gpio_io->write_8(w1_io_context, 0);
	w1_gpio_io->write_8(w1_io_context, 0);
	k_sleep(K_MSEC(1));
	for(iter = 0; iter < count; iter++){
		greybus_manifest_click_fragment_clickid[mikrobusid][iter] = w1_gpio_io->read_8(w1_io_context);
	}
	LOG_HEXDUMP_DBG(greybus_manifest_click_fragment_clickid[mikrobusid], count, "manifest fragment:");
    return 0;
}

int manifest_get_fragment_clickid(uint8_t **mnfb, size_t *mnfb_size, uint8_t id) {
	struct greybus_manifest_header *manifest_header;
	
	manifest_header = (struct greybus_manifest_header *)greybus_manifest_click_fragment_clickid[id];
	*mnfb = (uint8_t *)greybus_manifest_click_fragment_clickid[id];
	*mnfb_size = sys_le16_to_cpu(manifest_header->size);

	return 0;
}


#define DEFINE_MIKROBUS(_num)                                     \
																		\
static struct mikrobusid_context mikrobusid_dev_data_##_num;		\
									\
        static const struct mikrobusid_config 						\
			mikrobusid_config_##_num = {      						\
					.id  	= DT_INST_PROP(_num, id),		\
					.cs_gpio_name	= DT_INST_GPIO_LABEL(_num, cs_gpios),	\
					.rst_gpio_name	= DT_INST_GPIO_LABEL(_num, rst_gpios),	\
					.cs_pin		= DT_INST_GPIO_PIN(_num, cs_gpios),	\
					.rst_pin	= DT_INST_GPIO_PIN(_num, rst_gpios),	\
					.cs_flags	= DT_INST_GPIO_FLAGS(_num, cs_gpios),	\
					.rst_flags	= DT_INST_GPIO_FLAGS(_num, rst_gpios),	\
        };                                                              \
                                                                        \
        DEVICE_DT_INST_DEFINE(_num,										\
                            mikrobusid_init,					\
							NULL,									\
							&mikrobusid_dev_data_##_num,			\
                            &mikrobusid_config_##_num,				\
							POST_KERNEL,								\
                            CONFIG_GREYBUS_MIKROBUS_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_MIKROBUS);
