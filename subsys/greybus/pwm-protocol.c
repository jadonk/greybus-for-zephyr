/**
 * Copyright (c) 2015 Google, Inc.
 * Copyright (c) 2022 Harshil Bhatt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <greybus/greybus.h>
#include <greybus/platform.h>
#include <zephyr/sys/byteorder.h>
#include <greybus-utils/utils.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(greybus_pwm, CONFIG_GREYBUS_LOG_LEVEL);

#include "pwm-gb.h"

/**
 * A Greybus PWM controller adhering to the Protocol specified herein shall
 * report major version 0, minor version 1.
 */
#define GB_PWM_VERSION_MAJOR 0
#define GB_PWM_VERSION_MINOR 1

/**
 * @brief Get this firmware supported PWM protocol vsersion.
 *
 * This function is called when PWM operates initialize in Greybus kernel.
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_pwm_protocol_version(struct gb_operation *operation)
{
    struct gb_pwm_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_PWM_VERSION_MAJOR;
    response->minor = GB_PWM_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

/**
 * @brief Get the number of generators supported from PWM controller.
 *
 * This function calls PWM controller driver to get the number of generator
 * supported and then fill into response buffer of operation pointer.
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_pwm_protocol_count(struct gb_operation *operation)
{
    struct pwm_dt_spec *pwmdev = NULL;
    struct gb_pwm_count_response *response;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    uint16_t count = 0;

    pwmdev->dev = bundle->dev[cport_idx];
    if (pwmdev->dev == NULL) {
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    count = popcount(pwmdev->channel);
    if (!count)
        return GB_OP_UNKNOWN_ERROR;
    
    /*
     * Per Greybus specification, the number of generators supported should be
     * one less than the actual number.
     */
    response->count = count - 1;

    return GB_OP_SUCCESS;
}

/**
 * @brief Activating a specific generator that system supported.
 *
 * This function will parse the gb_pwm_activate_request to get specific
 * generator number then calls PWM controller driver to activate it.
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_pwm_protocol_activate(struct gb_operation *operation)
{
    struct pwm_dt_spec *pwmdev = NULL;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_pwm_activate_request *request = 
        gb_operation_get_request_payload(operation);

    pwmdev->dev = bundle->dev[cport_idx];
    if (pwmdev->dev == NULL) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    if (request->which >= popcount(pwmdev->channel)) {
        return GB_OP_INVALID;
    }

    /* 
     * Zephyr doesn't seem to have an "on-off" setting for pwm
     */
    // ret = device_pwm_activate(bundle->dev, request->which);
    // if (ret) {
    //     LOG_INF("%s(): %x error in ops", __func__, ret);
    //     return GB_OP_UNKNOWN_ERROR;
    // }

    return GB_OP_SUCCESS;
}

/**
 * @brief Deactivate an active generator.
 *
 * This function will parse the device_pwm_deactivate to get specific
 * generator number then calls PWM controller driver to deactivate it.
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_pwm_protocol_deactivate(struct gb_operation *operation)
{
    struct pwm_dt_spec *pwmdev = NULL;
    struct gb_bundle *bundle =  gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_pwm_dectivate_request *request =
        gb_operation_get_request_payload(operation);

    pwmdev->dev = bundle->dev[cport_idx];
    if (pwmdev->dev == NULL) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    if (request->which >= popcount(pwmdev->channel)) {
        return GB_OP_INVALID;
    }

    /**
     * Zephyr doesn't seem to have an "on-off" setting for pwm
     */
    // ret = device_pwm_deactivate(bundle->dev, request->which);
    // if (ret) {
    //     LOG_INF("%s(): %x error in ops", __func__, ret);
    //     return GB_OP_UNKNOWN_ERROR;
    // }

    return GB_OP_SUCCESS;
}

/**
 * @brief Configure specific generator for a particular duty cycle and period.
 *
 * This function will parse the gb_pwm_config_request to get specific generator
 * number, duty and period, and then calls PWM controller driver to configure
 * the specific generator by this particular duty and period.
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_pwm_protocol_config(struct gb_operation *operation)
{
    struct pwm_dt_spec *pwmdev = NULL;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_pwm_config_request *request = 
        gb_operation_get_request_payload(operation);
    uint32_t duty, period;
    int ret;

    pwmdev->dev = bundle->dev[cport_idx];
    if (pwmdev->dev == NULL) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    if (request->which >= popcount(pwmdev->channel)) {
        return GB_OP_INVALID;
    }

    duty = sys_le32_to_cpu(request->duty);
    period = sys_le32_to_cpu(request->period);
    ret = pwm_set_dt(pwmdev, period, duty);
    if (ret) {
        LOG_INF("%s(): %x error in ops", __func__, ret);
        return GB_OP_UNKNOWN_ERROR;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Configure specific generator for a particular polarity.
 *
 * This function will parse the gb_pwm_polarity_request to get specific
 * generator number and polarity setting, and then calls PWM controller driver
 * to configure the specific generator by this particular polarity.
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_pwm_protocol_polarity(struct gb_operation *operation)
{
    struct pwm_dt_spec *pwmdev = NULL;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_pwm_polarity_request *request =
        gb_operation_get_request_payload(operation);

    pwmdev->dev = bundle->dev[cport_idx];
    if (pwmdev->dev == NULL) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    if (request->which >= popcount(pwmdev->channel)) {
        return GB_OP_INVALID;
    }

    pwmdev->flags = request->polarity;

    return GB_OP_SUCCESS;
}

/**
 * @brief Enable a specific generator to start toggling.
 *
 * This function will parse the gb_pwm_enable_request to get specific generator
 * number, and then calls PWM controller driver to start pulse toggling by duty
 * , period and polarity that previous configured in the specific generator.
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_pwm_protocol_enable(struct gb_operation *operation)
{
    struct pwm_dt_spec *pwmdev = NULL;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_pwm_enable_request *request =
        gb_operation_get_request_payload(operation);

    pwmdev->dev = bundle->dev[cport_idx];
    if (pwmdev->dev == NULL) {
        return GB_OP_INVALID;
    }
    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    if (request->which >= popcount(pwmdev->channel)) {
        return GB_OP_INVALID;
    }

    /* 
     * Zephyr doesn't seem to have an "on-off" setting for pwm
     */
    // ret = device_pwm_enable(bundle->dev, request->which);
    // if (ret) {
    //     LOG_INF("%s(): error %x in ops return", __func__, ret);
    //     return GB_OP_UNKNOWN_ERROR;
    // }

    return GB_OP_SUCCESS;
}

/**
 * @brief Disable a specific generator toggling.
 *
 * This function will parse the gb_pwm_enable_request to get specific generator
 * number, and then calls PWM controller driver to stop the specific generator
 * of pulse toggling .
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_pwm_protocol_disable(struct gb_operation *operation)
{
    struct pwm_dt_spec *pwmdev = NULL;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_pwm_disable_request *request
     = gb_operation_get_request_payload(operation);

    pwmdev->dev = bundle->dev[cport_idx];
    if (pwmdev->dev == NULL) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    if (request->which >= popcount(pwmdev->channel)) {
        return GB_OP_INVALID;
    }

    /* 
     * Zephyr doesn't seem to have an "on-off" setting for pwm
     */
    // ret = device_pwm_disable(bundle->dev, request->which);
    // if (ret) {
    //     LOG_INF("%s(): %x error in ops", __func__, ret);
    //     return GB_OP_UNKNOWN_ERROR;
    // }

    return GB_OP_SUCCESS;
}

/**
 * @brief Initial the PWM protocol code and open device driver.
 *
 * This function allocate a structure pointer that defined as 'pwm_info' static
 * pointer that to store internal data and PWM controller driver handle that
 * return by device_open().
 *
 * @param cport Assigned Cport number.
 * @param bundle Greybus bundle handle
 *
 * @return (0) on success, error code on failure.
 */
int gb_pwm_init(unsigned int cport, struct gb_bundle *bundle)
{
    unsigned int cport_idx = cport - bundle->cport_start;

    bundle->dev[cport_idx] = (struct device *)gb_cport_to_device(cport);
    if (!bundle->dev[cport_idx]) {
        return -EIO;
    }
    return 0;
}

/**
 * @brief Close device driver and free allocated resource.
 *
 * This function to close device driver and free any resource that allocated in
 * initialization.
 *
 * @param cport Assigned CPort number.
 * @param bundle Greybus bundle handle
 */
void gb_pwm_exit(unsigned int cport, struct gb_bundle *bundle)
{
	ARG_UNUSED(cport);
	ARG_UNUSED(bundle);    
}


/*
 * This structure is to define each PWM protocol operation of handling function.
 */
static struct gb_operation_handler gb_pwm_handlers[] = {
    GB_HANDLER(GB_PWM_PROTOCOL_VERSION, gb_pwm_protocol_version),
    GB_HANDLER(GB_PWM_PROTOCOL_COUNT, gb_pwm_protocol_count),
    GB_HANDLER(GB_PWM_PROTOCOL_ACTIVATE, gb_pwm_protocol_activate),
    GB_HANDLER(GB_PWM_PROTOCOL_DEACTIVATE, gb_pwm_protocol_deactivate),
    GB_HANDLER(GB_PWM_PROTOCOL_CONFIG, gb_pwm_protocol_config),
    GB_HANDLER(GB_PWM_PROTOCOL_POLARITY, gb_pwm_protocol_polarity),
    GB_HANDLER(GB_PWM_PROTOCOL_ENABLE, gb_pwm_protocol_enable),
    GB_HANDLER(GB_PWM_PROTOCOL_DISABLE, gb_pwm_protocol_disable),
};


/*
 * This structure of information is for PWM protocol fimware to register to
 * greybus.
 */
static struct gb_driver gb_pwm_driver = {
    .init = gb_pwm_init,
    .exit = gb_pwm_exit,
    .op_handlers = gb_pwm_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_pwm_handlers),
};


/**
 * @brief Register PWM protocol firmware to Greybus.
 *
 * This function is called when greybus core to enable PWM of Cport.
 */
void gb_pwm_register(int cport, int bundle)
{
    gb_register_driver(cport, bundle, &gb_pwm_driver);
}
