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

#define SLEEP_TIME_MS 1000
LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

int main(void) {
  printk("Zephyr Example Application %s\n", APP_VERSION_STRING);

  LOG_INF("Zephyr base station main started");

  const struct device *water_dev =
      device_get_binding(DT_NODE_FULL_NAME(DT_NODELABEL(water)));

  struct sensor_value ph, temp, turb;
  int ret;

  if (!device_is_ready(water_dev)) {
    LOG_INF("water sensor device not ready %d", device_is_ready(water_dev));
    return -1;
  }

  /* Get PH value */
  ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_PH);
  if (ret) {
    LOG_ERR("Failed to fetch PH value (err %d)", ret);
  } else {
    ret = sensor_channel_get(water_dev, WATER_CHAN_PH, &ph);
    if (ret) {
      LOG_ERR("Failed to get PH value (err %d)", ret);
    } else {
      LOG_INF("PH: %d.%06d", ph.val1, ph.val2);
    }
  }

  /* Get temperature value */
  ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_TEMP);
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
  ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_TURB);
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

//   // while (1) {
//   //   /* Get pH value */

//   //   if (ret) {
//   //     LOG_ERR("Failed to fetch pH value (err %d)", ret);
//   //   } else {

//   //     if (ret) {
//   //       LOG_ERR("Failed to get pH value (err %d)", ret);
//   //     } else {
//   //       LOG_INF("pH: %d.%06d", ph.val1, ph.val2);
//   //     }
//   //   }

//   //   /* Get temperature value */
//   //   ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_TEMP);
//   //   if (ret) {
//   //     LOG_ERR("Failed to fetch temperature value (err %d)", ret);
//   //   } else {
//   //     ret = sensor_channel_get(water_dev, WATER_CHAN_TEMP, &temp);
//   //     if (ret) {
//   //       LOG_ERR("Failed to get temperature value (err %d)", ret);
//   //     } else {
//   //       LOG_INF("Temperature: %d.%06d", temp.val1, temp.val2);
//   //     }
//   //   }

//   //   /* Get turbidity value */
//   //   ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_TURB);
//   //   if (ret) {
//   //     LOG_ERR("Failed to fetch turbidity value (err %d)", ret);
//   //   } else {
//   //     ret = sensor_channel_get(water_dev, WATER_CHAN_TURB, &turb);
//   //     if (ret) {
//   //       LOG_ERR("Failed to get turbidity value (err %d)", ret);
//   //     } else {
//   //       LOG_INF("Turbidity: %d.%06d", turb.val1, turb.val2);
//   //     }
//   //   }

//   //   k_msleep(SLEEP_TIME_MS);
//   // }
  return 0;
}
// #include <app/drivers/sensor/water.h>
// #include <app_version.h>
// #include <stdio.h>
// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/sensor.h>
// #include <zephyr/drivers/uart.h>
// #include <zephyr/kernel.h>
// #include <zephyr/logging/log.h>

// #define SLEEP_TIME_MS 1000
// #define STACK_SIZE 1024

// LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

// // K_THREAD_STACK_DEFINE(sensor_reading_stack, STACK_SIZE);
// struct k_thread sensor_reading_thread_data;

// void sensor_reading_task() {
//     // const struct device *water_dev = device_get_binding(DT_NODE_FULL_NAME(DT_NODELABEL(water)));
//     // struct sensor_value ph, temp, turb;
//     // int ret;

//     // if (!device_is_ready(water_dev)) {
//     //     LOG_INF("water sensor device not ready %d", device_is_ready(water_dev));
//     //     return;
//     // }

//     while (1) {
// 		printk("\n working continuously!");
//         /* Get PH value */
//         // ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_PH);
//         // if (ret) {
//         //     LOG_ERR("Failed to fetch PH value (err %d)", ret);
//         // } else {
//         //     ret = sensor_channel_get(water_dev, WATER_CHAN_PH, &ph);
//         //     if (ret) {
//         //         LOG_ERR("Failed to get PH value (err %d)", ret);
//         //     } else {
//         //         LOG_INF("PH: %d.%06d", ph.val1, ph.val2);
//         //     }
//         // }

//         // /* Get temperature value */
//         // ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_TEMP);
//         // if (ret) {
//         //     LOG_ERR("Failed to fetch temperature value (err %d)", ret);
//         // } else {
//         //     ret = sensor_channel_get(water_dev, WATER_CHAN_TEMP, &temp);
//         //     if (ret) {
//         //         LOG_ERR("Failed to get temperature value (err %d)", ret);
//         //     } else {
//         //         LOG_INF("Temperature: %d.%06d", temp.val1, temp.val2);
//         //     }
//         // }

//         // /* Get turbidity value */
//         // ret = sensor_sample_fetch_chan(water_dev, WATER_CHAN_TURB);
//         // if (ret) {
//         //     LOG_ERR("Failed to fetch turbidity value (err %d)", ret);
//         // } else {
//         //     ret = sensor_channel_get(water_dev, WATER_CHAN_TURB, &turb);
//         //     if (ret) {
//         //         LOG_ERR("Failed to get turbidity value (err %d)", ret);
//         //     } else {
//         //         LOG_INF("Turbidity: %d.%06d", turb.val1, turb.val2);
//         //     }
//         // }

//         k_msleep(SLEEP_TIME_MS);
//     }
// }

// int main(void) {
//     printk("Zephyr Example Application %s\n", APP_VERSION_STRING);
//     LOG_INF("Zephyr base station main started");

//     // const struct device *water_dev = device_get_binding(DT_NODE_FULL_NAME(DT_NODELABEL(water)));

//     // if (!device_is_ready(water_dev)) {
//     //     LOG_INF("water sensor device not ready %d", device_is_ready(water_dev));
//     //     return -1;
//     // }

//     // k_thread_create(&sensor_reading_thread_data, sensor_reading_stack,
//     //                 K_THREAD_STACK_SIZEOF(sensor_reading_stack),
//     //                 sensor_reading_task, NULL, NULL, NULL,
//     //                 K_PRIO_PREEMPT(2), 0, K_NO_WAIT);

//     return 0;
// }
// #define STACKSIZE 512
// K_THREAD_DEFINE(sensor_reading_task1, STACKSIZE, sensor_reading_task, NULL, NULL, NULL, 0, 0,
//                 0);