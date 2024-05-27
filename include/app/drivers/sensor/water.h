/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
/**
 * @defgroup drivers Drivers
 * @{
 *
 * @brief This is a group of all driver-related code.
 *
 * This group contains all the driver classes, functions, and variables.
 * It's a central place to find everything related to drivers in this project.
 */

#define PKT_BUF_SIZE \
  12U  // THIS BUFFER SIZE SHOULD BE VARIABLE BASED ON THE CMD 50
#define RX_QUEUE_SIZE 2
#define CMD_SIZE 1

// commands
#define TURB 0x80
#define PH 0x90
#define TEMP 0xa0
#define ALL 0xb0

#define TURBIDITY_POS_INT 0
#define TURBIDITY_POS_DEC 2
#define PH_POS_INT 4
#define PH_POS_DEC 6
#define TEMP_POS_INT 8
#define TEMP_POS_DEC 10
#define SINGLE_POS_INT 0
#define SINGLE_POS_DEC 2
// #define PKT_HEADER_POS    0U

#define PKT_CMD_POS 0
#define PKT_SAM_TIME_POS 8
//------------
#if defined CONFIG_WATER_OVER_1X
#define SAMPLINT_TIME 1U
#elif defined CONFIG_WATER_OVER_2X
#define SAMPLINT_TIME 2U
#elif defined CONFIG_WATER_OVER_4X
#define SAMPLINT_TIME 3U
#elif defined CONFIG_WATER_OVER_8X
#define SAMPLINT_TIME 4U
#elif defined CONFIG_WATER_OVER_16X
#define SAMPLINT_TIME 5U
#endif
#if defined CONFIG_WATER_RES_1X
#define RESOLUTION 10U
#elif defined CONFIG_WATER_RES_2X
#define RESOLUTION 12U
#endif

/** @brief water_sensor custom channels. */
enum water_channel {
  /** water channel verification. */
  WATER_CHAN_PH = SENSOR_CHAN_PRIV_START,
  WATER_CHAN_TURB,
  WATER_CHAN_TEMP,
  WATER_CHAN_ALL

};
/*
 * sensor data structure
 */
typedef struct {
  /** Integer part of the value. */
  int16_t val1;
  /** Fractional part of the value (in one-millionth parts). */
  int16_t val2;
} sensor_value_t;

/*
 * water data structure
 */
struct water_data {
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
/*
 * water config structure
 */
struct water_config {
  const struct device *uart;
};

typedef struct water_data water_data_t;
typedef struct water_config water_config_t;

/**
 * @brief sensor UART configuration.
 *
 * \code{.cpp}
 * static const struct uart_config uart_config = {
 *     .baudrate = 115200U,
 *     .data_bits = UART_CFG_DATA_BITS_8,
 *     .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
 *     .parity = UART_CFG_PARITY_NONE,
 *     .stop_bits = UART_CFG_STOP_BITS_2,
 * };
 * \endcode
 */
static const struct uart_config myuart_config = {
    115200,
    UART_CFG_DATA_BITS_8,
    UART_CFG_FLOW_CTRL_NONE,
    UART_CFG_PARITY_NONE,
    UART_CFG_STOP_BITS_1,
};
/** @} */
#endif /* ZEPHYR_INCLUDE */