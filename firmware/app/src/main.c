/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/sensor.h>

#include <logging/log.h>
#include <drivers/uart.h>
#include "app_version.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define GPS_UART DT_LABEL(UART_1)

static const struct uart_config uart_cfg = {
		.baudrate = 9600,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

static void gps_uart_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		LOG_ERR("retval should always be 1\n");
		return;
	}

	/* Verify uart_irq_rx_ready() */
	if (uart_irq_rx_ready(dev)) {
		uint8_t recvData = 0U;

		/* Verify uart_fifo_read() */
		uart_fifo_read(dev, &recvData, 1);
		LOG_ERR("%c", recvData);
	}
}

void main(void)
{
	LOG_DBG("GPS based chain oiler %s\n", APP_VERSION_STR);

	const struct device *gps = device_get_binding("usart1");
	
	if (!gps) {
		LOG_ERR("Cannot get GPS UART device");
		return;
	}

	int ret = uart_configure(gps, &uart_cfg);

	if (ret == -ENOTSUP) {
		LOG_ERR("Failed to initialize UART");
		return;
	}

	uart_irq_callback_set(gps, gps_uart_callback);
	uart_irq_rx_enable(gps);

	LOG_INF("GPS uart is ready");

	while (1) {

		LOG_ERR("bip\r\n");

		k_sleep(K_MSEC(1000));
	}
}

