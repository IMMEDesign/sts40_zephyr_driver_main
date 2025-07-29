/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sensirion_sts40

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sts40.h"

/* Include the Sensirion library files */
#include "../embedded-i2c-sts4x/sts4x_i2c.h"
#include "../embedded-i2c-sts4x/sensirion_i2c_hal.h"

LOG_MODULE_REGISTER(STS40_SENSIRION, CONFIG_SENSOR_LOG_LEVEL);

/* Global I2C device for Sensirion HAL */
static const struct device *i2c_device = NULL;

/**
 * Sensirion HAL implementation for Zephyr
 */

void sensirion_i2c_hal_init(void)
{
    /* Device initialization is handled by Zephyr */
}

void sensirion_i2c_hal_free(void)
{
    /* Nothing to free in Zephyr context */
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count)
{
    if (i2c_device == NULL) {
        return -1;
    }
    
    return i2c_read(i2c_device, data, count, address);
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count)
{
    if (i2c_device == NULL) {
        return -1;
    }
    
    return i2c_write(i2c_device, data, count, address);
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds)
{
    k_usleep(useconds);
}

int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx)
{
    /* In this implementation, we use device tree configuration */
    /* Bus selection is handled during device initialization */
    return 0;
}

/**
 * Driver implementation using Sensirion library
 */

struct sts40_sensirion_config {
    struct i2c_dt_spec i2c;
    uint8_t precision;
    uint8_t i2c_address;
};

struct sts40_sensirion_data {
    int32_t temperature;
    uint32_t serial_number;
};

static int sts40_sensirion_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    const struct sts40_sensirion_config *config = dev->config;
    struct sts40_sensirion_data *data = dev->data;
    int16_t ret;
    
    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    /* Set the global I2C device for HAL */
    i2c_device = config->i2c.bus;
    
    /* Initialize driver with configured address */
    init_driver(config->i2c_address);

    /* Perform measurement based on precision setting */
    switch (config->precision) {
        case STS40_PRECISION_HIGH:
            ret = sts4x_measure_high_precision(&data->temperature);
            break;
        case STS40_PRECISION_MEDIUM:
            ret = sts4x_measure_medium_precision(&data->temperature);
            break;
        case STS40_PRECISION_LOW:
            ret = sts4x_measure_lowest_precision(&data->temperature);
            break;
        default:
            ret = sts4x_measure_high_precision(&data->temperature);
            break;
    }

    if (ret != 0) {
        LOG_ERR("Failed to measure temperature: %d", ret);
        return -EIO;
    }

    LOG_DBG("Temperature: %d milli-celsius", data->temperature);
    
    return 0;
}

static int sts40_sensirion_channel_get(const struct device *dev, enum sensor_channel chan,
                                      struct sensor_value *val)
{
    struct sts40_sensirion_data *data = dev->data;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    /* Convert milli-celsius to sensor_value */
    val->val1 = data->temperature / 1000;
    val->val2 = (data->temperature % 1000) * 1000;

    return 0;
}

static int sts40_sensirion_attr_set(const struct device *dev, enum sensor_channel chan,
                                   enum sensor_attribute attr, const struct sensor_value *val)
{
    return -ENOTSUP;
}

static int sts40_sensirion_attr_get(const struct device *dev, enum sensor_channel chan,
                                   enum sensor_attribute attr, struct sensor_value *val)
{
    const struct sts40_sensirion_config *config = dev->config;
    struct sts40_sensirion_data *data = dev->data;
    int16_t ret;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    if (attr == SENSOR_ATTR_CONFIGURATION) {
        /* Return serial number */
        i2c_device = config->i2c.bus;
        init_driver(config->i2c_address);
        
        ret = sts4x_serial_number(&data->serial_number);
        if (ret != 0) {
            LOG_ERR("Failed to read serial number: %d", ret);
            return -EIO;
        }
        
        val->val1 = data->serial_number;
        val->val2 = 0;
        return 0;
    }

    return -ENOTSUP;
}

static const struct sensor_driver_api sts40_sensirion_driver_api = {
    .sample_fetch = sts40_sensirion_sample_fetch,
    .channel_get = sts40_sensirion_channel_get,
    .attr_set = sts40_sensirion_attr_set,
    .attr_get = sts40_sensirion_attr_get,
};

static int sts40_sensirion_init(const struct device *dev)
{
    const struct sts40_sensirion_config *config = dev->config;
    int16_t ret;

    /* Check if I2C device is ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C device %s is not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    /* Set global I2C device for HAL */
    i2c_device = config->i2c.bus;
    
    /* Initialize Sensirion library */
    init_driver(config->i2c_address);
    
    /* Perform soft reset */
    ret = sts4x_soft_reset();
    if (ret != 0) {
        LOG_ERR("Failed to reset device: %d", ret);
        return -EIO;
    }

    LOG_INF("STS40 sensor (Sensirion library) initialized successfully");
    
    return 0;
}

/* Device instantiation macro for Sensirion library version */
#define STS40_SENSIRION_DEFINE(inst)                                          \
    static struct sts40_sensirion_data sts40_sensirion_data_##inst;          \
                                                                              \
    static const struct sts40_sensirion_config sts40_sensirion_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                   \
        .precision = DT_INST_ENUM_IDX_OR(inst, precision, STS40_PRECISION_HIGH), \
        .i2c_address = DT_INST_REG_ADDR(inst),                               \
    };                                                                        \
                                                                              \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, sts40_sensirion_init, NULL,           \
                                 &sts40_sensirion_data_##inst,                \
                                 &sts40_sensirion_config_##inst,              \
                                 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
                                 &sts40_sensirion_driver_api);

/* Only compile this version if specifically requested */
#ifdef CONFIG_STS40_USE_SENSIRION_LIB
DT_INST_FOREACH_STATUS_OKAY(STS40_SENSIRION_DEFINE)
#endif
