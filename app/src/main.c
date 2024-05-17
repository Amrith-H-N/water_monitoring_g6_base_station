/**
 * @file main.c
 * @brief Application to interact with the water sensor driver
 */

#include <app_version.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define SLEEP_TIME_MS 1000

void main(void) {
  printk("Zephyr Example Application %s\n", APP_VERSION_STRING);
  const struct device *water_dev;
  water_dev = device_get_binding("water");

  struct sensor_value ph, temp, turb;
  int ret;

  if (!device_is_ready(water_dev)) {
    LOG_ERR("Water sensor device not ready");
    return;
  }

  LOG_INF("Water sensor application started");

  while (1) {
    /* Get pH value */
    ret = sensor_sample_fetch(water_dev, WATER_CHAN_PH);
    if (ret) {
      LOG_ERR("Failed to fetch pH value (err %d)", ret);
    } else {
      ret = sensor_channel_get(water_dev, WATER_CHAN_PH, &ph);
      if (ret) {
        LOG_ERR("Failed to get pH value (err %d)", ret);
      } else {
        LOG_INF("pH: %d.%06d", ph.val1, ph.val2);
      }
    }

    /* Get temperature value */
    ret = sensor_sample_fetch(water_dev, WATER_CHAN_TEMP);
    if (ret) {
      LOG_ERR("Failed to fetch temperature value (err %d)", ret);
    } else {
      ret = sensor_channel_get(water_dev, WATER_CHAN_TEMP, &temp);
      if (ret) {
        LOG_ERR("Failed to get temperature value (err %d)", ret);
      } else {
        LOG_INF("Temperature: %d.%06d", temp.val1, temp.val2);
      }
    }

    /* Get turbidity value */
    ret = sensor_sample_fetch(water_dev, WATER_CHAN_TURB);
    if (ret) {
      LOG_ERR("Failed to fetch turbidity value (err %d)", ret);
    } else {
      ret = sensor_channel_get(water_dev, WATER_CHAN_TURB, &turb);
      if (ret) {
        LOG_ERR("Failed to get turbidity value (err %d)", ret);
      } else {
        LOG_INF("Turbidity: %d.%06d", turb.val1, turb.val2);
      }
    }

    k_msleep(SLEEP_TIME_MS);
  }
}