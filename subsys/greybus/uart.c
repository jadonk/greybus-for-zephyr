/*
 * Copyright (c) 2015 Google, Inc.
 * Copyright (c) 2022 Harshil Bhatt
 *
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

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <device.h>
#include <zephyr/drivers/uart.h>
#include <greybus/types.h>
#include <greybus/greybus.h>
#include <greybus-utils/utils.h>
#include <sys/byteorder.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_uart, CONFIG_GREYBUS_LOG_LEVEL);

#include "uart-gb.h"

#define GB_UART_VERSION_MAJOR   0
#define GB_UART_VERSION_MINOR   1

/**
 * @brief Protocol get version function.
 *
 * Returns the major and minor Greybus UART protocol version number supported
 * by the UART device.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_protocol_version(struct gb_operation *operation)
{
    struct gb_uart_proto_version_response *response = NULL;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_UART_VERSION_MAJOR;
    response->minor = GB_UART_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol send data function.
 *
 * Requests that the UART device begin transmitting characters. One or more
 * bytes to be transmitted will be supplied.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_send_data(struct gb_operation *operation)
{
    const struct device *dev;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    int ret, size;
    int sent = 0;
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    size_t request_size = gb_operation_get_request_payload_size(operation);
    struct gb_uart_send_data_request *request =
        gb_operation_get_request_payload(operation);
    
    dev = bundle->dev[cport_idx];
    if (dev == NULL) {
        return GB_OP_INVALID;
    }

    if (request_size < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    size = sys_le16_to_cpu(request->size);

    if (request_size < sizeof(*request) + size) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    ret = uart_tx(dev, request->data, size, NULL);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol set line coding function.
 *
 * Sets the line settings of the UART to the specified baud rate, format,
 * parity, and data bits.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_set_line_coding(struct gb_operation *operation)
{
    const struct device *dev;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    int ret;
    uint32_t baud;
    struct uart_config *cfg;
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_serial_line_coding_request *request =
        gb_operation_get_request_payload(operation);
    
    dev = bundle->dev[cport_idx];
    if (dev == NULL) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    baud = sys_le32_to_cpu(request->rate);

#warning setting up config causes bus faults
    switch (request->format) {
    case GB_SERIAL_0_5_STOP_BITS:
        cfg->stop_bits = UART_CFG_STOP_BITS_0_5;
    case GB_SERIAL_1_STOP_BITS:
        cfg->stop_bits = UART_CFG_STOP_BITS_1;
        break;
    case GB_SERIAL_1_5_STOP_BITS:
        cfg->stop_bits = UART_CFG_STOP_BITS_1_5;
        break;
    case GB_SERIAL_2_STOP_BITS:
        cfg->stop_bits = UART_CFG_STOP_BITS_2;
        break;
    default:
        return GB_OP_INVALID;
        break;
    }

    switch (request->parity) {
    case GB_SERIAL_NO_PARITY:
        cfg->parity = UART_CFG_PARITY_NONE;
        break;
    case GB_SERIAL_ODD_PARITY:
        cfg->parity = UART_CFG_PARITY_ODD;
        break;
    case GB_SERIAL_EVEN_PARITY:
        cfg->parity = UART_CFG_PARITY_EVEN;
        break;
    case GB_SERIAL_MARK_PARITY:
        cfg->parity = UART_CFG_PARITY_MARK;
        break;
    case GB_SERIAL_SPACE_PARITY:
        cfg->parity = UART_CFG_PARITY_SPACE;
        break;
    default:
        return GB_OP_INVALID;
        break;
    }

    if (request->data > 8 || request->data < 5) {
        return GB_OP_INVALID;
    }

    // cfg->data_bits = request->data;

    ret = uart_configure(dev, cfg);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol set RTS & DTR line status function.
 *
 * Controls RTS and DTR line states of the UART.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_set_control_line_state(struct gb_operation *operation)
{
    const struct device *dev;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    int ret;
    uint32_t modem_ctrl = 0;
    uint16_t control;
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_uart_set_control_line_state_request *request =
        gb_operation_get_request_payload(operation);

    dev = bundle->dev[cport_idx];
    if (dev == NULL) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }

    control = sys_le16_to_cpu(request->control);
    uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &modem_ctrl);
    if (control & GB_UART_CTRL_DTR) {
        modem_ctrl |= UART_LINE_CTRL_DTR;
    } else {
        modem_ctrl &= ~UART_LINE_CTRL_DTR;
    }
    uart_line_ctrl_set(dev, UART_LINE_CTRL_DTR, modem_ctrl);

    uart_line_ctrl_get(dev, UART_LINE_CTRL_RTS, &modem_ctrl);
    if (control & GB_UART_CTRL_RTS) {
        modem_ctrl |= UART_LINE_CTRL_RTS;
    } else {
        modem_ctrl &= ~UART_LINE_CTRL_RTS;
    }
    uart_line_ctrl_set(dev, UART_LINE_CTRL_RTS, modem_ctrl);

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol send break function.
 *
 * Requests that the UART generate a break condition on its transmit line.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_send_break(struct gb_operation *operation)
{
    const struct device *dev;
    struct uart_config uart_config;
    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);
    unsigned int cport_idx = operation->cport - bundle->cport_start;
    struct gb_uart_set_break_request *request =
        gb_operation_get_request_payload(operation);

    dev = bundle->dev[cport_idx];
    if (dev == NULL) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        LOG_ERR("dropping short message");
        return GB_OP_INVALID;
    }
    uint32_t baud = uart_config.baudrate;
    uart_config.baudrate = 1200;
    uart_poll_out(dev, "0");
    uart_config.baudrate = baud;
    
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol initialization function.
 *
 * This function perform the protocto initialization function, such as open
 * the cooperation device driver, launch threads, create buffers etc.
 *
 * @param cport CPort number
 * @param bundle Greybus bundle handle
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static int gb_uart_init(unsigned int cport, struct gb_bundle *bundle)
{
    unsigned int cport_idx = cport - bundle->cport_start;

    bundle->dev[cport_idx] = (struct device *)gb_cport_to_device(cport);
    if (!bundle->dev[cport_idx]) {
        return -EIO;
    }
    return 0;    
}

/**
 * @brief Protocol exit function.
 *
 * This function can be called when protocol terminated.
 *
 * @param cport CPort number
 * @param bundle Greybus bundle handle
 * @return None.
 */
static void gb_uart_exit(unsigned int cport, struct gb_bundle *bundle)
{
    ARG_UNUSED(cport);
    ARG_UNUSED(bundle);
}

static struct gb_operation_handler gb_uart_handlers[] = {
    GB_HANDLER(GB_UART_PROTOCOL_VERSION, gb_uart_protocol_version),
    GB_HANDLER(GB_UART_PROTOCOL_SEND_DATA, gb_uart_send_data),
    GB_HANDLER(GB_UART_PROTOCOL_SET_LINE_CODING, gb_uart_set_line_coding),
    GB_HANDLER(GB_UART_PROTOCOL_SET_CONTROL_LINE_STATE,
               gb_uart_set_control_line_state),
    GB_HANDLER(GB_UART_PROTOCOL_SEND_BREAK, gb_uart_send_break),
};

struct gb_driver uart_driver = {
    .init = gb_uart_init,
    .exit = gb_uart_exit,
    .op_handlers = (struct gb_operation_handler*) gb_uart_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_uart_handlers),
};

/**
 * @brief Protocol registering function.
 *
 * This function can be called by greybus to register the UART protocol.
 *
 * @param cport The number of CPort.
 * @param bundle Bundle number.
 * @return None.
 */
void gb_uart_register(int cport, int bundle)
{
    LOG_INF("%s(): cport %d bundle %d", __func__, cport, bundle);
    gb_register_driver(cport, bundle, &uart_driver);
}

