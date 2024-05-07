/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_H_

#include <zephyr/drivers/sensor.h>



#define PKT_BUF_SIZE     	12U // THIS BUFFER SIZE SHOULD BE VARIABLE BASED ON THE CMD 50
#define RX_QUEUE_SIZE     	2U
#define CMD_SIZE			1U
//commands
#define TURB = 0x00U
#define PH = 0x01U
#define TEMP = 0x02U
#define ALL = 0x03U


#define TURBIDITY_POS_INT 	0U//
#define TURBIDITY_POS_DEC 	2U
#define PH_POS_INT        	4U
#define PH_POS_DEC        	6U
#define TEMP_POS_INT      	8U
#define TEMP_POS_DEC      	10U
#define SINGLE_POS_INT		0U
#define SINGLE_POS_DEC		2U
//#define PKT_HEADER_POS    0U

#define PKT_CMD_POS			0U
#define PKT_SAM_TIME_POS	8U
//------------
#if defined WATER_OVER_1X
#define SAMPLINT_TIME            1U
#elif defined WATER_OVER_2X
#define SAMPLINT_TIME            2U
#elif defined WATER_OVER_4X
#define SAMPLINT_TIME            3U
#elif defined WATER_OVER_16X
#define SAMPLINT_TIME            4U
#endif
#if defined WATER_RES_1X
#define RESOLUTION            10U
#elif defined WATER_RES_2X
#define RESOLUTION            12U
#endif
//-----------
/** @brief water_sensor custom channels. */
enum water_channel
{
  /** water channel verification. */
  WATER_CHAN_PH = 0,
  WATER_CHAN_TURB = 1,
  WATER_CHAN_TEMP = 2,
  WATER_CHAN_ALL = 3

};
/** @} */
#endif /* ZEPHYR_INCLUDE */
//typedef struct sensor_value sensor_value_t;
struct sensor_value_t {
	/** Integer part of the value. */
	int16_t val1;
	/** Fractional part of the value (in one-millionth parts). */
	int16_t val2;
};
struct water_data
{
	/** RX queue buffer. */
	uint8_t rx_queue_buf[PKT_BUF_SIZE * RX_QUEUE_SIZE];
	/** RX/TX buffer. */
	uint8_t buf[PKT_BUF_SIZE];
	/** RX/TX buffer pointer. */
	uint8_t *buf_ptr;
	/** RX/TX buffer counter. */
	uint8_t buf_ctr;
	/** Expected reception data length. */
	uint8_t rx_data_len;
	/** Sensor value buffer. */
	sensor_value_t pH;
	sensor_value_t turb;
	sensor_value_t temp;
	/** RX queue. */
	struct k_msgq rx_queue;
};

struct water_config
{
	uint32_t addr;
	const struct device *uart;
};

typedef struct water_data water_data_t;
typedef struct water_config water_config_t;

/** @brief sensor UART configuration. */
static const struct uart_config uart_config = {
	.baudrate = 115200U,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_2,
};