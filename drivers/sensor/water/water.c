/** @file water.c
 *  @brief Function prototypes for the sensornode driver
 *
 *  This contains the prototypes for the sensor node
 *  driver and eventually any macros, constants,
 *  or global variables you will need.
 *
 *  @author Amrith Harijayanthan Namboodiri
 *  @author Abdel Mujeeb
 *  @author Adithya
 *  @bug No known bugs.
 */

#define DT_DRV_COMPAT zephyr_water
#define WATER_INIT_PRIORITY 41

#include "app/drivers/sensor/water.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/types.h>

#define MY_RING_BUF_BYTES 1024
LOG_MODULE_REGISTER(water_driver, LOG_LEVEL_DBG);

/**
 * @defgroup drivers Drivers
 * @{
 *
 * @brief This is a group of all driver-related code.
 *
 * This group contains all the driver classes, functions, and variables.
 * It's a central place to find everything related to drivers in this project.
 */

/**
 * @brief UART RX handler.
 *
 * @param dev uart device
 * @param user_data sensor instance
 */
static void uart_cb_rx_handler(const struct device *dev, void *user_data) {
  int n = 0;

  const struct device *sensor = user_data;

  const struct water_config *config = sensor->config;
  struct water_data *data = sensor->data;

  uint8_t c;
  uint8_t mbuffer[20];

  if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) > 0)) {
    if (uart_irq_rx_ready(dev)) {
      while (uart_fifo_read(dev, &c, 1) == 1) {
        mbuffer[n++] = c;
      }

      if (n >= data->rx_data_len) {
        LOG_HEXDUMP_DBG(data->buf, data->buf_ctr, "RX");
        if (k_msgq_put(&data->rx_queue, mbuffer, K_NO_WAIT) < 0) {
          LOG_ERR("RX queue full, dropping packet");
        }
      }
    }
  }
}

/**
 * @brief Send a command to the sensor.
 *
 * @param dev Sensor instance.
 * @param cmd Command to send.
 * @param rx_data_len Expected data reception length in bytes
 *
 * @retval 0 Success , -EMSGSIZE If TX/RX data length is too large.
 *
 */
static int water_send(const struct device *dev, uint8_t cmd,
                      uint8_t rx_data_len)  //,uint8_t sam_time
{
  const struct water_config *config = dev->config;
  struct water_data *data = dev->data;

  if (rx_data_len > sizeof(data->buf)) {
    return -EMSGSIZE;
  }

  data->buf[PKT_CMD_POS] = cmd;

  /* store expected reception data length */
  data->rx_data_len = rx_data_len;

  data->buf_ctr = CMD_SIZE;
  data->buf_ptr = data->buf;

  LOG_INF("sending cmd %x", cmd);

  // send data uart polling
  uart_poll_out(config->uart, (uint8_t)cmd);
  k_msleep(500);

  return 0;
}

/**
 * Receive response from the sensor.
 *
 * @param dev Sensor instance.
 * @param rx_data_len rx data length
 *
 * @retval 0 Success, -EMSGSIZE If RX data length does not match current
 * settings, -EAGAIN If reception times out.
 *
 */
static int water_recv(const struct device *dev, uint8_t rx_data_len) {
  struct water_data *data = dev->data;
  uint8_t buf[PKT_BUF_SIZE];
  int ret;

  if (rx_data_len != data->rx_data_len) {
    return -EMSGSIZE;
  }

  ret = k_msgq_get(&data->rx_queue, buf, K_FOREVER);

  printf("water_recv : ");
  for (int i = 0; i < data->rx_data_len; i++) printf("%x ", (uint8_t)buf[i]);

  if (ret < 0) {
    return ret;
  }

  memcpy(data->rx_queue_buf, buf, rx_data_len);

  return 0;
}

/**
 * @defgroup drivers_water Water drivers
 * @ingroup drivers
 * @{
 *
 * @brief A custom driver for Water monitoring sensor
 *
 *
 *
 */

/**
 * @brief helper funtion to update values in the struct
 *
 *  format : [ (length) 8bit | (sensor cmd) 8bit | 8bit 8bit 8bit 8bit 8bit
 * 8bit
 * ]
 *
 *  @param dev device struct
 *  @param chan channel to update
 *  @return int 0 if success
 */
int update_value(const struct device *dev, enum sensor_channel chan) {
  water_data_t *data = dev->data;
  uint8_t rx_data_len;
  int ret;
  switch (chan) {
    case WATER_CHAN_PH:
      rx_data_len = 4;
      water_send(dev, (uint8_t)PH, rx_data_len);
      ret = water_recv(dev, rx_data_len);
      if (!ret) {
        data->pH.val1 = (data->rx_queue_buf[0] << 8) + data->rx_queue_buf[1];
        data->pH.val2 = (data->rx_queue_buf[2] << 8) + data->rx_queue_buf[3];

        LOG_DBG("%x %x %x %x", data->rx_queue_buf[0], data->rx_queue_buf[1],
                data->rx_queue_buf[2], data->rx_queue_buf[3]);
      }

      break;
    case WATER_CHAN_TEMP:
      rx_data_len = 4;
      water_send(dev, (uint8_t)TEMP, rx_data_len);
      ret = water_recv(dev, rx_data_len);
      if (!ret) {
        data->temp.val1 = (data->rx_queue_buf[0] << 8) + data->rx_queue_buf[1];
        data->temp.val2 = (data->rx_queue_buf[2] << 8) + data->rx_queue_buf[3];
        LOG_DBG("%x %x %x %x", data->rx_queue_buf[0], data->rx_queue_buf[1],
                data->rx_queue_buf[2], data->rx_queue_buf[3]);
      }
      break;
    case WATER_CHAN_TURB:
      rx_data_len = 4;
      water_send(dev, (uint8_t)TURB, rx_data_len);
      ret = water_recv(dev, rx_data_len);
      if (!ret) {
        data->turb.val1 = (data->rx_queue_buf[0] << 8) + data->rx_queue_buf[1];
        data->turb.val2 = (data->rx_queue_buf[2] << 8) + data->rx_queue_buf[3];
        LOG_DBG("%x %x %x %x", data->rx_queue_buf[0], data->rx_queue_buf[1],
                data->rx_queue_buf[2], data->rx_queue_buf[3]);
      }
      break;
    case WATER_CHAN_ALL:
      rx_data_len = 12;
      water_send(dev, (uint8_t)ALL, rx_data_len);
      ret = water_recv(dev, rx_data_len);
      if (!ret) {
        data->turb.val1 = (uint16_t)data->rx_queue_buf[TURBIDITY_POS_INT];
        data->turb.val2 = (uint16_t)data->rx_queue_buf[TURBIDITY_POS_DEC];
        data->pH.val1 =
            (uint16_t)
                data->rx_queue_buf[PH_POS_INT];  // write code to get data from
                                                 // rxbuffer and update
        data->pH.val2 = (uint16_t)data->rx_queue_buf[PH_POS_DEC];
        data->temp.val1 = (uint16_t)data->rx_queue_buf[TEMP_POS_INT];
        data->temp.val2 = (uint16_t)data->rx_queue_buf[TEMP_POS_DEC];
      }
      break;
    default:
      return -ENOTSUP;
  }

  if (ret < 0) {
    return ret;
  }

  return 0;
}

/** @brief water_sample fetch function to update the values from sensor to
 * struct
 *
 *  This function will fetch values from sensor and update it in the struct
 *
 *  @param dev device struct
 *  @param chan channel
 *  @return int returns 0 if succeeful otherwise Error code
 */
static int water_sample_fetch(const struct device *dev,
                              enum sensor_channel chan) {
  if (chan > WATER_CHAN_ALL || chan < WATER_CHAN_PH) {
    return -ENOTSUP;
  }

  return update_value(dev, chan);
}

/** @brief water_sample get function to store the values from sensor to
 * struct from user
 *
 *  This function will fetch values from the stored stuct memory and copy it
 * to the user passed stuct
 *
 *  @param dev device
 *  @param chan channel
 *  @param val struct sensor_value type struct to retrieve sensor value
 *  @return int returns 0 if succeeful otherwise Error code
 */
static int water_channel_get(const struct device *dev, enum sensor_channel chan,
                             struct sensor_value *val) {
  water_data_t *data = dev->data;
  enum water_channel my_chan = (enum water_channel)chan;
  switch (my_chan) {
    case WATER_CHAN_PH:
      val->val1 = data->pH.val1;
      val->val2 = data->pH.val2;
      break;
    case WATER_CHAN_TEMP:
      val->val1 = data->temp.val1;
      val->val2 = data->temp.val2;
      break;
    case WATER_CHAN_TURB:
      val->val1 = data->turb.val1;
      val->val2 = data->turb.val2;
      break;
    default:
      return -ENOTSUP;
  }
  return 0;
}

/**
 * @brief sensor sensor_driver_api configuration.
 *
 * \code{.cpp}
 * static const struct sensor_driver_api water_api = {
 *   .sample_fetch = water_sample_fetch,
 *   .channel_get = water_channel_get,
};
 * \endcode
 */
static const struct sensor_driver_api water_api = {
    .sample_fetch = water_sample_fetch,
    .channel_get = water_channel_get,
};

void initialize(const struct device *dev) {
  water_send(dev, RESOLUTION, 0);
  k_msleep(1000);

  water_send(dev, SAMPLINT_TIME, 0);
}

/** @brief water_init function checks for uart device and copies all
 * configuration
 *
 *  This function takes a struct with all configuration and initilises the
 * driver and sensor with it.
 *
 *  @param dev device
 *  @return int returns 0 if succeeful otherwise Error code
 */
static int water_init(const struct device *dev) {
  const struct water_config *config = dev->config;
  water_data_t *data = dev->data;
  volatile int ret;
  LOG_INF("Water_init started");

  if (!device_is_ready(config->uart)) {
    LOG_ERR("UART device not ready\n");
    return ENXIO;
  }

  if (ret < 0) {
    return ret;
  }

  // rx irq config
  uart_irq_callback_user_data_set(config->uart, uart_cb_rx_handler,
                                  (void *)dev);
  k_msgq_init(&data->rx_queue, data->rx_queue_buf, PKT_BUF_SIZE, RX_QUEUE_SIZE);
  uart_irq_err_enable(config->uart);

  // enable uart rx irq
  uart_irq_rx_enable(config->uart);

  k_msleep(1000);
  initialize(dev);
  LOG_INF("Water_init done");
  LOG_INF("starting commuication...");

  return 0;
}
/** @} */
/** @} */
#define WATER_INIT(i)                                         \
  static struct water_data water_data_##i;                    \
                                                              \
  static const struct water_config water_config_##i = {       \
                                                              \
      .uart = DEVICE_DT_GET(DT_INST_BUS(i)),                  \
                                                              \
  };                                                          \
                                                              \
  DEVICE_DT_INST_DEFINE(i, water_init, NULL, &water_data_##i, \
                        &water_config_##i, POST_KERNEL,       \
                        CONFIG_SENSOR_INIT_PRIORITY, &water_api);

DT_INST_FOREACH_STATUS_OKAY(WATER_INIT)
