/*
 * Copyright (c) 2022 Harshil Bhatt
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr/drivers/uart.h>
#include <dt-bindings/greybus/greybus.h>
#include <greybus/greybus.h>
#include <greybus/platform.h>
#include <zephyr/zephyr.h>

#define DT_DRV_COMPAT zephyr_greybus_uart_controller
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(greybus_platform_uart_control, CONFIG_GREYBUS_LOG_LEVEL);

#include "../uart-gb.h"
#include "transport.h"

struct greybus_uart_control_config {
    const uint8_t id;
    const uint8_t bundle;
    const char *const greybus_uart_controller_name;
    const char *const bus_name;
};

struct greybus_uart_control_data {
    const struct device *greybus_uart_controller;
};

static int greybus_uart_control_init(const struct device *dev) {
    struct greybus_uart_control_data *drv_data =
        (struct greybus_uart_control_data *)dev->data;
    struct greybus_uart_control_config *config =
        (struct greybus_uart_control_config *)dev->config;
    int r;
    const struct device *bus;

    drv_data->greybus_uart_controller =
        device_get_binding(config->greybus_uart_controller_name);
    if (NULL == drv_data->greybus_uart_controller) {
		LOG_ERR("uart control: failed to get binding for device '%s'",
			config->greybus_uart_controller_name);
		return -ENODEV;
    }

    bus = device_get_binding(config->bus_name);
    if (NULL == bus) {
		LOG_ERR("uart control: failed to get binding for device '%s'", config->bus_name);
    	return -ENODEV;
    }

    r = gb_add_cport_device_mapping(config->id, drv_data->greybus_uart_controller);
    if (r < 0) {
		LOG_ERR("uart control: failed to add mapping between %u and %s", config->id, dev->name);
		return r;
    }

    LOG_DBG("probed cport %u: bundle: %u protocol: %u", config->id,
		config->bundle, CPORT_PROTOCOL_UART);
    return 0;
}

#define DEFINE_GREYBUS_UART_CONTROL(_num)										\
																				\
		BUILD_ASSERT(DT_PROP(DT_PARENT(DT_DRV_INST(_num)), bundle_class)		\
		== BUNDLE_CLASS_BRIDGED_PHY, "BUNDLE_CLASS_BRIDGED_PHY required"); 		\
																				\
		BUILD_ASSERT(DT_PROP(DT_DRV_INST(_num), cport_protocol) 				\
		== CPORT_PROTOCOL_UART, "CPORT_PROTOCOL_UART required"); 				\
																				\
		static struct greybus_uart_control_config								\
			greybus_uart_control_config_##_num = {								\
                .id = (uint8_t)DT_INST_PROP(_num, id), 							\
                .bundle = (uint8_t)DT_PROP(DT_PARENT(DT_DRV_INST(_num)), id), 	\
				.greybus_uart_controller_name = 								\
                    DT_LABEL(DT_PHANDLE(DT_DRV_INST(_num), 						\
                    		greybus_uart_controller)), 							\
				.bus_name = 													\
					DT_LABEL(DT_PARENT(DT_PARENT(DT_DRV_INST(_num)))),			\
        };																		\
        																		\
        static struct greybus_uart_control_data									\
			greybus_uart_control_data_##_num;									\
        																		\
        DEVICE_DT_INST_DEFINE(_num, 											\
                            greybus_uart_control_init, NULL,				\
							&greybus_uart_control_data_##_num,					\
                            &greybus_uart_control_config_##_num, POST_KERNEL,	\
                            CONFIG_GREYBUS_CPORT_INIT_PRIORITY, NULL);


DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_UART_CONTROL);
