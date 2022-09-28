/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifdef CONFIG_GPIO_SIM

#include <zephyr/drivers/gpio.h>
#include <drivers/gpio/gpio_sim.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <zephyr/zephyr.h>

#include "test-greybus-spi.h"

#ifndef GPIO_DEV_NAME
#define GPIO_DEV_NAME "GPIO_0"
#endif

static void gpio_sim_callback_handler(struct device *port,
				      struct gpio_callback *cb,
				      gpio_port_pins_t pins)
{
	printf("%s(): GPIO changed\n", __func__);
}

static struct gpio_callback gpio_sim_callback = {
	.handler = gpio_sim_callback_handler,
	.pin_mask = BIT(0),
};

void gpio_sim_setup(void)
{
	struct device *dev = device_get_binding(GPIO_DEV_NAME);
	__ASSERT(dev != NULL, "Device not found");
	int rc = gpio_add_callback(dev, &gpio_sim_callback);
	__ASSERT(rc == 0, "gpio_add_callback() failed: %d", rc);
	/* in the simulated case, we're using active-high logic */
	rc = gpio_pin_configure(dev, 0, GPIO_OUTPUT_LOW);
	__ASSERT(rc == 0, "gpio_pin_configure() failed: %d", rc);
}

#endif /* CONFIG_GPIO_SIM */
