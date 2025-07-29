/*
 * Copyright (c) 2025 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_STS40_STS40_H_
#define ZEPHYR_DRIVERS_SENSOR_STS40_STS40_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

/* STS40 I2C addresses */
#define STS40_I2C_ADDR_PRIMARY   0x46
#define STS40_I2C_ADDR_ALT       0x44
#define STS40_I2C_ADDR_ALT2      0x45

/* STS40 commands */
#define STS40_CMD_MEASURE_HIGH_PRECISION    0xFD
#define STS40_CMD_MEASURE_MEDIUM_PRECISION  0xF6
#define STS40_CMD_MEASURE_LOW_PRECISION     0xE0
#define STS40_CMD_READ_SERIAL               0x89
#define STS40_CMD_SOFT_RESET               0x94

/* Measurement delays in microseconds */
#define STS40_MEASURE_DELAY_HIGH_PRECISION_US   10000
#define STS40_MEASURE_DELAY_MEDIUM_PRECISION_US 10000
#define STS40_MEASURE_DELAY_LOW_PRECISION_US    10000

/* Temperature conversion constants */
#define STS40_TEMP_SCALE_FACTOR     175
#define STS40_TEMP_OFFSET          -45
#define STS40_TEMP_DIVISOR         65535

struct sts40_config {
    struct i2c_dt_spec i2c;
    uint8_t precision;
};

struct sts40_data {
    int32_t temperature;
    uint32_t serial_number;
};

/* Precision levels */
enum sts40_precision {
    STS40_PRECISION_HIGH = 0,
    STS40_PRECISION_MEDIUM = 1,
    STS40_PRECISION_LOW = 2
};

#endif /* ZEPHYR_DRIVERS_SENSOR_STS40_STS40_H_ */
