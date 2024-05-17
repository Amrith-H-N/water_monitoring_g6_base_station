// /* Application Code */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main()
{

	printk("Zephyr Example Application %s\n", APP_VERSION_STRING);
	const struct device *sensor;
	sensor = device_get_binding("water");
	if (!device_is_ready(sensor))
	{
		LOG_ERR("Sensor not ready");
		return 0;
	}

	while (1)
	{

		k_sleep(K_MSEC(1000));
	}
}
