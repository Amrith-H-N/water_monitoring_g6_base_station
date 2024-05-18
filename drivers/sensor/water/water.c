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

/**
 * @defgroup drivers Drivers
 * @{
 *
 * @brief This is a group of all driver-related code.
 *
 * This group contains all the driver classes, functions, and variables.
 * It's a central place to find everything related to drivers in this project.
 */

LOG_MODULE_REGISTER(water, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief UART RX handler.
 *
 * @param dev Sensor instance.
 */
static void uart_cb_rx_handler(const struct device *dev) {
  const struct water_config *config = dev->config;
  struct water_data *data = dev->data;
  int n;

  n = uart_fifo_read(config->uart, data->buf_ptr, PKT_BUF_SIZE - data->buf_ctr);
  data->buf_ctr += (uint8_t)n;
  data->buf_ptr += (uint8_t)n;

  if (data->buf_ctr >= data->rx_data_len) {
    LOG_HEXDUMP_DBG(data->buf, data->buf_ctr, "RX");
    if (k_msgq_put(&data->rx_queue, data->buf, K_NO_WAIT) < 0) {
      LOG_ERR("RX queue full, dropping packet");
    }
    data->buf_ctr = 0U;
    data->buf_ptr = data->buf;
  }
}

/**
 * @brief UART TX handler.
 *
 * @param dev Sensor instance.
 */
static void uart_cb_tx_handler(const struct device *dev) {
  const struct water_config *config = dev->config;
  struct water_data *data = dev->data;
  int n;

  if (data->buf_ctr > 0U) {
    n = uart_fifo_fill(config->uart, data->buf_ptr, data->buf_ctr);
    data->buf_ctr -= (uint8_t)n;
    data->buf_ptr += (uint8_t)n;
    return;
  }

  if (uart_irq_tx_complete(config->uart) > 0) {
    data->buf_ptr = data->buf;
    uart_irq_tx_disable(config->uart);
    uart_irq_rx_enable(config->uart);
  }
}

/**
 * @brief UART IRQ handler.
 *
 * @param dev UART device instance.
 * @param user_data User data (sensor instance).
 */
static void uart_cb_handler(const struct device *dev, void *user_data) {
  const struct device *sensor = user_data;

  if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) > 0)) {
    if (uart_irq_rx_ready(dev)) {
      uart_cb_rx_handler(sensor);
    }

    if (uart_irq_tx_ready(dev)) {
      uart_cb_tx_handler(sensor);
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
  // data->buf[PKT_SAM_TIME_POS] = sam_time;

  /* store expected reception data length */
  data->rx_data_len = rx_data_len;

  /* trigger transmission */
  data->buf_ctr = CMD_SIZE;
  data->buf_ptr = data->buf;
  LOG_HEXDUMP_DBG(data->buf, data->buf_ctr, "TX");
  uart_irq_rx_disable(config->uart);
  uart_irq_tx_enable(config->uart);

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
  if (ret < 0) {
    return ret;
  }

  memcpy(data->rx_queue_buf, buf, rx_data_len);

  return 0;
}
//--------------------------------------------------------------------

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
 *  format : [ (length) 8bit | (sensor cmd) 8bit | 8bit 8bit 8bit 8bit 8bit 8bit
 * ]
 *
 *  @param dev device struct
 *  @param chan channel to update
 *  @return int 0 if success
 */
int update_value(const struct device *dev, enum sensor_channel chan) {
  // water_config_t *config = dev->config;
  // water_data_t *data = dev->data;
  uint8_t rx_data_len;
  switch (chan) {
    case WATER_CHAN_PH:
      rx_data_len = 4;
      water_send(dev, PH, rx_data_len);
      water_recv(dev, rx_data_len);
      break;
    case WATER_CHAN_TEMP:
      rx_data_len = 4;
      water_send(dev, TEMP, rx_data_len);
      water_recv(dev, rx_data_len);
    case WATER_CHAN_TURB:
      rx_data_len = 4;
      water_send(dev, TURB, rx_data_len);
      water_recv(dev, rx_data_len);
    case WATER_CHAN_ALL:
      rx_data_len = 12;
      water_send(dev, ALL, rx_data_len);
      water_recv(dev, rx_data_len);
    default:
      return -ENOTSUP;
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

/** @brief water_sample get function to store the values from sensor to struct
 * from user
 *
 *  This function will fetch values from the stored stuct memory and copy it to
 * the user passed stuct
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
      data->pH.val1 =
          data->rx_queue_buf[SINGLE_POS_INT];  // write code to get data from
                                               // rxbuffer and update
      data->pH.val2 = data->rx_queue_buf[SINGLE_POS_DEC];
      break;
    case WATER_CHAN_TEMP:
      data->temp.val1 = data->rx_queue_buf[SINGLE_POS_INT];
      data->temp.val2 = data->rx_queue_buf[SINGLE_POS_DEC];
      break;
    case WATER_CHAN_TURB:
      data->turb.val1 = data->rx_queue_buf[SINGLE_POS_INT];
      data->turb.val2 = data->rx_queue_buf[SINGLE_POS_DEC];
      break;
    case WATER_CHAN_ALL:
      data->turb.val1 = data->rx_queue_buf[TURBIDITY_POS_INT];
      data->turb.val2 = data->rx_queue_buf[TURBIDITY_POS_DEC];
      data->pH.val1 =
          data->rx_queue_buf[PH_POS_INT];  // write code to get data from
                                           // rxbuffer and update
      data->pH.val2 = data->rx_queue_buf[PH_POS_DEC];
      data->temp.val1 = data->rx_queue_buf[TEMP_POS_INT];
      data->temp.val2 = data->rx_queue_buf[TEMP_POS_DEC];
      break;

    default:
      return -ENOTSUP;
  }
  return 0;
}

static const struct sensor_driver_api water_api = {
    .sample_fetch = water_sample_fetch,
    .channel_get = water_channel_get,
};

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
  printk("water_init started");
  // const struct device *water_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
  if (!device_is_ready(config->uart)) {
    LOG_ERR("UART device not ready\n");
    printk("water_init failed");
    return ENXIO;
  }
  // ret = uart_configure(config->uart->config, &uart_config);
  volatile struct uart_config cfg;

  ret = uart_config_get(config->uart, &cfg);
  cfg.baudrate = myuart_config.baudrate;
  // cfg.data_bits = myuart_config.data_bits;
  //  cfg.flow_ctrl = myuart_config.flow_ctrl;
  // cfg.parity = myuart_config.parity;
  // cfg.stop_bits = myuart_config.stop_bits;

  // // ret = uart_config_data_bits(config->uart, cfg.data_bits);
  ret = uart_configure(config->uart, &cfg);
  // ret = uart_config_get(config->uart, &cfg);

  if (ret < 0) {
    return ret;
  }

  // uart_irq_callback_user_data_set(config->uart, uart_cb_handler, (void
  // *)dev);

  // k_msgq_init(&data->rx_queue, data->rx_queue_buf, PKT_BUF_SIZE,
  // RX_QUEUE_SIZE);
  unsigned char ch = 'a';
  while (1) {
    uart_poll_out(config->uart, ch);
    k_msleep(1000);
  }
  // water_send(dev, RESOLUTION, 0);
  // water_send(dev, SAMPLINT_TIME, 0);
  printk("water_init ok");
  return 0;
}
/** @} */
/** @} */
// #define WATER_INIT(i)                                         \
//   static struct water_data water_data_##i;                    \
//                                                               \
//   static const struct water_config water_config_##i = {       \
//       /*.input = GPIO_DT_SPEC_INST_GET(i, input_gpios), */    \
//       .uart = DEVICE_DT_GET(DT_NODELABEL(uart1)),             \
//                                                               \
//   };                                                          \
//                                                               \
//   DEVICE_DT_INST_DEFINE(i, water_init, NULL, &water_data_##i, \
//                         &water_config_##i, POST_KERNEL,       \
//                         CONFIG_SENSOR_INIT_PRIORITY, &water_api);

// DT_INST_FOREACH_STATUS_OKAY(WATER_INIT)

#ifdef CONFIG_WATER

/* Move the DEVICE_DECLARE macro invocation here */
DEVICE_DECLARE(water);

static const struct water_config water_config = {
    .uart = DEVICE_DT_GET(DT_NODELABEL(uart1)),
};

static struct water_data water_data;

DEVICE_DEFINE(CONFIG_WATER, "water", water_init, NULL, &water_data,
              &water_config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
              &water_api);

#endif /* CONFIG_WATER */