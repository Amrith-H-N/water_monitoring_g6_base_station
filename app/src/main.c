/**
 * @file main.c
 * @brief Application to interact with the water sensor driver
 */

#include <app/drivers/sensor/water.h>
#include <app_version.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define SLEEP_TIME_MS 1000

int main(void) {
  printk("Zephyr Example Application %s\n", APP_VERSION_STRING);
  const struct device *water_dev = DEVICE_DT_GET(DT_NODELABEL(mywater));

  struct sensor_value ph, temp, turb;
  int ret;

  if (!device_is_ready(water_dev)) {
    LOG_DBG("Water sensor device not ready");
    printk("water sensor device not ready %d", device_is_ready(water_dev));
    return -1;
  }

  LOG_INF("Water sensor application started");

  ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_PH);

  ret = sensor_channel_get(water_dev, WATER_CHAN_PH, &ph);

  // while (1) {
  //   /* Get pH value */

  //   if (ret) {
  //     LOG_ERR("Failed to fetch pH value (err %d)", ret);
  //   } else {

  //     if (ret) {
  //       LOG_ERR("Failed to get pH value (err %d)", ret);
  //     } else {
  //       LOG_INF("pH: %d.%06d", ph.val1, ph.val2);
  //     }
  //   }

  //   /* Get temperature value */
  //   ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_TEMP);
  //   if (ret) {
  //     LOG_ERR("Failed to fetch temperature value (err %d)", ret);
  //   } else {
  //     ret = sensor_channel_get(water_dev, WATER_CHAN_TEMP, &temp);
  //     if (ret) {
  //       LOG_ERR("Failed to get temperature value (err %d)", ret);
  //     } else {
  //       LOG_INF("Temperature: %d.%06d", temp.val1, temp.val2);
  //     }
  //   }

  //   /* Get turbidity value */
  //   ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_TURB);
  //   if (ret) {
  //     LOG_ERR("Failed to fetch turbidity value (err %d)", ret);
  //   } else {
  //     ret = sensor_channel_get(water_dev, WATER_CHAN_TURB, &turb);
  //     if (ret) {
  //       LOG_ERR("Failed to get turbidity value (err %d)", ret);
  //     } else {
  //       LOG_INF("Turbidity: %d.%06d", turb.val1, turb.val2);
  //     }
  //   }

  //   k_msleep(SLEEP_TIME_MS);
  // }
  return 0;
}