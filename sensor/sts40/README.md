# STS40 Temperature Sensor Driver for Zephyr

This directory contains a Zephyr driver for the Sensirion STS40 digital temperature sensor.

## Features

- Full Zephyr sensor API support
- I2C communication interface
- Three precision levels (high, medium, low)
- CRC data validation
- Serial number reading
- Soft reset functionality
- Device tree configuration

## Hardware Setup

The STS40 sensor communicates via I2C and supports three different addresses:
- `0x46` (primary/default address)
- `0x44` (alternative address 1) 
- `0x45` (alternative address 2)

Connect the sensor to your board's I2C bus:
- SDA: I2C data line
- SCL: I2C clock line
- VDD: 2.15V to 5.5V power supply
- GND: Ground

## Device Tree Configuration

Add the sensor to your device tree overlay or board's device tree:

```dts
&i2c0 {
    status = "okay";
    
    sts40: sts40@46 {
        compatible = "sensirion,sts40";
        reg = <0x46>;
        precision = "high";  /* Options: "high", "medium", "low" */
        status = "okay";
    };
};
```

## Kconfig Options

Enable the driver in your project configuration:

```kconfig
CONFIG_STS40=y
CONFIG_I2C=y
CONFIG_SENSOR=y
```

## API Usage

```c
#include <zephyr/drivers/sensor.h>

const struct device *sts40 = DEVICE_DT_GET_ANY(sensirion_sts40);

/* Fetch new measurement */
sensor_sample_fetch(sts40);

/* Get temperature value */
struct sensor_value temp;
sensor_channel_get(sts40, SENSOR_CHAN_AMBIENT_TEMP, &temp);

/* Convert to double */
double temperature_celsius = sensor_value_to_double(&temp);
```

## Sample Application

The `sample/` directory contains a complete example application demonstrating:
- Device initialization and verification
- Temperature measurement reading
- Serial number retrieval
- Error handling

To build and run the sample:

```bash
cd sample/
west build -b <your_board>
west flash
```

## File Structure

```
STS40/
├── sts40.h                 # Driver header file
├── sts40.c                 # Driver implementation
├── Kconfig                 # Configuration options
├── CMakeLists.txt          # Build configuration
├── sensirion,sts40.yaml    # Device tree binding
├── sample/                 # Sample application
│   ├── main.c             # Sample code
│   ├── CMakeLists.txt     # Sample build config
│   ├── prj.conf           # Sample configuration
│   └── app.overlay        # Sample device tree
└── README.md              # This file
```

## Integration with Existing Sensirion Code

This driver is designed to work alongside the existing Sensirion embedded-i2c-sts4x library if needed. The driver implements its own I2C communication and CRC calculation for better integration with Zephyr's device model, but you can also use the Sensirion HAL implementation by:

1. Copying the HAL files from `../embedded-i2c-sts4x/sample-implementations/zephyr_user_space/`
2. Including the Sensirion library files in your project
3. Using the Sensirion API directly instead of the Zephyr sensor API

## Troubleshooting

### Common Issues

1. **Device not found**: Check I2C address and wiring
2. **CRC errors**: Check I2C signal integrity and pull-up resistors
3. **Build errors**: Ensure all Kconfig options are enabled
4. **No measurements**: Verify power supply voltage (2.15V - 5.5V)

### Debug Tips

- Enable debug logging: `CONFIG_SENSOR_LOG_LEVEL_DBG=y`
- Use an oscilloscope or logic analyzer to verify I2C signals
- Check device tree configuration with `west build -t devicetree`
- Verify sensor presence with `i2cdetect` if available

## Specifications

- Temperature accuracy: ±0.2°C (typical)
- Temperature range: -40°C to +125°C
- Resolution: 16-bit
- Interface: I2C (up to 1 MHz)
- Supply voltage: 2.15V to 5.5V
- Supply current: 0.15 μA (sleep), 350 μA (measurement)

## License

This driver is licensed under the Apache License 2.0.
