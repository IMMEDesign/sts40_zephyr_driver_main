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
#include <zephyr/sys/byteorder.h>

#include "sts40.h"

LOG_MODULE_REGISTER(STS40, CONFIG_SENSOR_LOG_LEVEL);

/**
 * Convert raw temperature ticks to milli-celsius
 */
static int32_t sts40_convert_temperature(uint16_t temp_ticks)
{
    /* Formula: T = -45 + 175 * (temp_ticks / 65535) */
    // return (((int64_t)temp_ticks * STS40_TEMP_SCALE_FACTOR * 1000) / STS40_TEMP_DIVISOR) + 
    //        (STS40_TEMP_OFFSET * 1000);

    // 1) Compute the raw “-45 + 175·(ticks/65535)” numerator:
    //    A = temp_ticks * 175
    int32_t A = (int32_t)temp_ticks * STS40_TEMP_SCALE_FACTOR;      
    //                              ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ :contentReference[oaicite:0]{index=0}

    // 2) Split into whole degrees and remainder:
    int32_t whole = A / STS40_TEMP_DIVISOR;    // integer °C
    int32_t rem   = A % STS40_TEMP_DIVISOR;    // leftover ticks

    // 3) Turn the leftover ticks into milli-°C (rounding to nearest):
    int32_t frac = (rem * 1000 + STS40_TEMP_DIVISOR/2) 
                   / STS40_TEMP_DIVISOR;      // 0 … 999

    // 4) Combine, and apply the –45 °C offset (in milli-°C):
    return whole * 1000          // °C → milli-°C
         + frac                  // add fractional milli-°C
         + (STS40_TEMP_OFFSET * 1000);
}

/**
 * Calculate CRC8 checksum (polynomial: 0x31, initialization: 0xFF)
 */
static uint8_t sts40_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    
    return crc;
}

/**
 * Read temperature measurement from STS40
 */
static int sts40_read_temperature(const struct device *dev, uint8_t precision_cmd)
{
    const struct sts40_config *config = dev->config;
    struct sts40_data *data = dev->data;
    uint8_t rx_buf[3];
    uint8_t tx_buf[1];
    uint32_t delay_us;
    int ret;

    /* Select measurement delay based on precision */
    switch (precision_cmd) {
        case STS40_CMD_MEASURE_HIGH_PRECISION:
            delay_us = STS40_MEASURE_DELAY_HIGH_PRECISION_US;
            break;
        case STS40_CMD_MEASURE_MEDIUM_PRECISION:
            delay_us = STS40_MEASURE_DELAY_MEDIUM_PRECISION_US;
            break;
        case STS40_CMD_MEASURE_LOW_PRECISION:
            delay_us = STS40_MEASURE_DELAY_LOW_PRECISION_US;
            break;
        default:
            return -EINVAL;
    }

    /* Send measurement command */
    tx_buf[0] = precision_cmd;
    ret = i2c_write_dt(&config->i2c, tx_buf, sizeof(tx_buf));
    if (ret < 0) {
        LOG_ERR("Failed to send measurement command: %d", ret);
        return ret;
    }

    /* Wait for measurement to complete */
    k_usleep(delay_us);

    /* Read measurement result */
    ret = i2c_read_dt(&config->i2c, rx_buf, sizeof(rx_buf));
    if (ret < 0) {
        LOG_ERR("Failed to read measurement: %d", ret);
        return ret;
    }

    /* Verify CRC */
    uint8_t crc = sts40_crc8(rx_buf, 2);
    if (crc != rx_buf[2]) {
        LOG_ERR("CRC mismatch: expected 0x%02x, got 0x%02x", crc, rx_buf[2]);
        return -EIO;
    }

    /* Convert to temperature */
    uint16_t temp_ticks = sys_get_be16(rx_buf);
    data->temperature = sts40_convert_temperature(temp_ticks);

    LOG_DBG("Temperature: %d milli-celsius", data->temperature);
    
    return 0;
}

/**
 * Read serial number from STS40
 */
static int sts40_read_serial_number(const struct device *dev)
{
    const struct sts40_config *config = dev->config;
    struct sts40_data *data = dev->data;
    uint8_t rx_buf[6];
    uint8_t tx_buf[1];  /* Only one byte needed for serial number command */
    int ret;

    /* Send read serial number command - single byte command */
    tx_buf[0] = STS40_CMD_READ_SERIAL;
    ret = i2c_write_dt(&config->i2c, tx_buf, sizeof(tx_buf));
    if (ret < 0) {
        LOG_ERR("Failed to send serial number command: %d", ret);
        return ret;
    }

    /* Wait a bit for command processing */
    k_usleep(1000);

    /* Read serial number */
    ret = i2c_read_dt(&config->i2c, rx_buf, sizeof(rx_buf));
    if (ret < 0) {
        LOG_ERR("Failed to read serial number: %d", ret);
        return ret;
    }

    /* Verify CRC for both 16-bit words */
    uint8_t crc1 = sts40_crc8(&rx_buf[0], 2);
    uint8_t crc2 = sts40_crc8(&rx_buf[3], 2);
    
    if (crc1 != rx_buf[2] || crc2 != rx_buf[5]) {
        LOG_ERR("Serial number CRC mismatch");
        return -EIO;
    }

    /* Combine the two 16-bit words into 32-bit serial number */
    data->serial_number = (sys_get_be16(&rx_buf[0]) << 16) | sys_get_be16(&rx_buf[3]);
    
    LOG_INF("Serial number: 0x%08x", data->serial_number);
    
    return 0;
}

/**
 * Perform soft reset
 */
static int sts40_soft_reset(const struct device *dev)
{
    const struct sts40_config *config = dev->config;
    uint8_t tx_buf[1];
    int ret;

    tx_buf[0] = STS40_CMD_SOFT_RESET;
    ret = i2c_write_dt(&config->i2c, tx_buf, sizeof(tx_buf));
    if (ret < 0) {
        LOG_ERR("Failed to send soft reset command: %d", ret);
        return ret;
    }

    /* Wait for reset to complete */
    k_msleep(1);
    
    return 0;
}

/**
 * Zephyr sensor API: sample fetch
 */
static int sts40_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    const struct sts40_config *config = dev->config;
    uint8_t precision_cmd;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    /* Select precision command based on configuration */
    switch (config->precision) {
        case STS40_PRECISION_HIGH:
            precision_cmd = STS40_CMD_MEASURE_HIGH_PRECISION;
            break;
        case STS40_PRECISION_MEDIUM:
            precision_cmd = STS40_CMD_MEASURE_MEDIUM_PRECISION;
            break;
        case STS40_PRECISION_LOW:
            precision_cmd = STS40_CMD_MEASURE_LOW_PRECISION;
            break;
        default:
            precision_cmd = STS40_CMD_MEASURE_HIGH_PRECISION;
            break;
    }

    return sts40_read_temperature(dev, precision_cmd);
}

/**
 * Zephyr sensor API: channel get
 */
static int sts40_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val)
{
    struct sts40_data *data = dev->data;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    /* Convert milli-celsius to sensor_value */
    val->val1 = data->temperature / 1000;
    val->val2 = (data->temperature % 1000) * 1000;

    return 0;
}

/**
 * Zephyr sensor API: attribute set
 */
static int sts40_attr_set(const struct device *dev, enum sensor_channel chan,
                         enum sensor_attribute attr, const struct sensor_value *val)
{
    return -ENOTSUP;
}

/**
 * Zephyr sensor API: attribute get
 */
static int sts40_attr_get(const struct device *dev, enum sensor_channel chan,
                         enum sensor_attribute attr, struct sensor_value *val)
{
    struct sts40_data *data = dev->data;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    if (attr == SENSOR_ATTR_CONFIGURATION) {
        /* Return serial number as configuration attribute */
        val->val1 = data->serial_number;
        val->val2 = 0;
        return 0;
    }

    return -ENOTSUP;
}

static const struct sensor_driver_api sts40_driver_api = {
    .sample_fetch = sts40_sample_fetch,
    .channel_get = sts40_channel_get,
    .attr_set = sts40_attr_set,
    .attr_get = sts40_attr_get,
};

/**
 * Device initialization
 */
static int sts40_init(const struct device *dev)
{
    const struct sts40_config *config = dev->config;
    int ret;

    /* Check if I2C device is ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C device %s is not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    /* Perform soft reset */
    ret = sts40_soft_reset(dev);
    if (ret < 0) {
        LOG_ERR("Failed to reset device: %d", ret);
        return ret;
    }

    /* Read and store serial number */
    ret = sts40_read_serial_number(dev);
    if (ret < 0) {
        LOG_ERR("Failed to read serial number: %d", ret);
        return ret;
    }

    LOG_INF("STS40 sensor initialized successfully");
    
    return 0;
}

/* Device instantiation macro */
#define STS40_DEFINE(inst)                                                    \
    static struct sts40_data sts40_data_##inst;                              \
                                                                              \
    static const struct sts40_config sts40_config_##inst = {                 \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                   \
        .precision = DT_INST_ENUM_IDX_OR(inst, precision, STS40_PRECISION_HIGH), \
    };                                                                        \
                                                                              \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, sts40_init, NULL,                     \
                                 &sts40_data_##inst, &sts40_config_##inst,   \
                                 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
                                 &sts40_driver_api);

DT_INST_FOREACH_STATUS_OKAY(STS40_DEFINE)
