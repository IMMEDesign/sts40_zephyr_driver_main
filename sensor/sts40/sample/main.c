/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sts40_sample, LOG_LEVEL_INF);

/* Get the device from device tree */
static const struct device *const sts40 = DEVICE_DT_GET_ANY(sensirion_sts40);

int main(void)
{
    struct sensor_value temp;
    struct sensor_value serial;
    int ret;

    LOG_INF("STS40 Temperature Sensor Sample Application");

    /* Check if device is available */
    if (!device_is_ready(sts40)) {
        LOG_ERR("STS40 device %s is not ready", sts40->name);
        return -ENODEV;
    }

    LOG_INF("Found STS40 device: %s", sts40->name);

    /* Read serial number */
    ret = sensor_attr_get(sts40, SENSOR_CHAN_AMBIENT_TEMP, 
                         SENSOR_ATTR_CONFIGURATION, &serial);
    if (ret == 0) {
        LOG_INF("Serial number: 0x%08x", (uint32_t)serial.val1);
    } else {
        LOG_WRN("Failed to read serial number: %d", ret);
    }

    /* Main measurement loop */
    while (1) {
        /* Fetch new measurement */
        ret = sensor_sample_fetch(sts40);
        if (ret < 0) {
            LOG_ERR("Failed to fetch sample: %d", ret);
            k_sleep(K_SECONDS(1));
            continue;
        }

        /* Get temperature value */
        ret = sensor_channel_get(sts40, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        if (ret < 0) {
            LOG_ERR("Failed to get temperature: %d", ret);
            k_sleep(K_SECONDS(1));
            continue;
        }

        /* Convert sensor value to double for display */
        double temperature = sensor_value_to_double(&temp);

        LOG_INF("Temperature: %.2f °C", temperature);

        /* Wait 2 seconds before next measurement */
        k_sleep(K_SECONDS(2));
    }

    return 0;
}
