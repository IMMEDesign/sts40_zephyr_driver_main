# STS40 Driver Integration Guide

This guide explains how to integrate the STS40 temperature sensor driver into your Zephyr project.

## Method 1: Add as External Module

### Step 1: Copy Driver Files

Copy the entire `STS40/` directory to your Zephyr project:

```
your_project/
├── drivers/
│   └── sensor/
│       └── sts40/          # Copy the STS40 directory here
├── src/
│   └── main.c
├── CMakeLists.txt
├── prj.conf
└── boards/
    └── your_board.overlay
```

### Step 2: Update Project CMakeLists.txt

Add the driver to your project's `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(your_project)

# Add the STS40 driver
add_subdirectory(drivers/sensor/sts40)

target_sources(app PRIVATE src/main.c)
```

### Step 3: Update Kconfig

Create or update `Kconfig` in your project root:

```kconfig
# Include driver Kconfig
source "drivers/sensor/sts40/Kconfig"

# Your other configurations...
```

### Step 4: Update Device Tree

Add the sensor to your board overlay or device tree:

```dts
&i2c0 {
    status = "okay";
    
    sts40: sts40@46 {
        compatible = "sensirion,sts40";
        reg = <0x46>;
        precision = "high";
        status = "okay";
    };
};
```

### Step 5: Configure Your Project

Add to your `prj.conf`:

```
CONFIG_STS40=y
CONFIG_I2C=y
CONFIG_SENSOR=y
```

## Method 2: Add to Zephyr Tree

### Step 1: Copy to Zephyr Source

Copy the driver files to Zephyr's driver tree:

```
$ZEPHYR_BASE/drivers/sensor/sts40/
```

### Step 2: Update Zephyr Kconfig

Edit `$ZEPHYR_BASE/drivers/sensor/Kconfig`:

```kconfig
# Add this line with other sensor drivers
rsource "sts40/Kconfig"
```

### Step 3: Update Zephyr CMakeLists

Edit `$ZEPHYR_BASE/drivers/sensor/CMakeLists.txt`:

```cmake
# Add this line
add_subdirectory_ifdef(CONFIG_STS40 sts40)
```

### Step 4: Add Device Tree Binding

Copy the binding file to:

```
$ZEPHYR_BASE/dts/bindings/sensor/sensirion,sts40.yaml
```

## Method 3: Using West Modules

### Step 1: Create Module Manifest

Create `zephyr/module.yml` in your driver repository:

```yaml
name: sts40-driver
build:
  cmake: .
  kconfig: Kconfig
```

### Step 2: Add to West Manifest

In your project's `west.yml`:

```yaml
manifest:
  projects:
    - name: sts40-driver
      url: https://github.com/your-username/sts40-driver
      revision: main
      path: modules/sts40-driver
```

### Step 3: Update and Use

```bash
west update
# Driver is now available in your project
```

## Method 4: Integration with Nordic nRF Connect SDK (NCS)

Since you're using NCS v2.9.0, you have two recommended approaches:

### Option A: Add to NCS Zephyr Tree (Recommended)

NCS already has a Sensirion sensor directory structure. Add your driver there:

1. **Copy driver files**:
   ```
   Copy: STS40/
   To: C:\ncs\v2.9.0\zephyr\drivers\sensor\sensirion\sts40\
   ```

2. **Update Sensirion Kconfig**:
   Edit `C:\ncs\v2.9.0\zephyr\drivers\sensor\sensirion\Kconfig`:
   ```kconfig
   # Add this line with other sensors
   rsource "sts40/Kconfig"
   ```

3. **CMakeLists.txt is already updated**:
   The file `C:\ncs\v2.9.0\zephyr\drivers\sensor\sensirion\CMakeLists.txt` 
   already includes: `add_subdirectory_ifdef(CONFIG_STS40 sts40)`

4. **Copy device tree binding**:
   ```
   Copy: sensirion,sts40.yaml
#---------------------------------------------
   # Copyright (c) 2025
# SPDX-License-Identifier: Apache-2.0

description: Sensirion STS40 Digital Temperature Sensor

compatible: "sensirion,sts40"

include: [sensor-device.yaml, i2c-device.yaml]

properties:
  precision:
    type: string
    default: "high"
    enum:
      - "high"
      - "medium" 
      - "low"
    description: |
      Measurement precision level:
      - high: Highest accuracy, longest measurement time (~10ms)
      - medium: Medium accuracy, medium measurement time (~10ms)  
      - low: Lowest accuracy, shortest measurement time (~10ms)

  reg:
    required: true
    description: |
      I2C address of the sensor. The STS40 supports three different addresses:
      - 0x46 (primary/default address)
      - 0x44 (alternative address 1)
      - 0x45 (alternative address 2)
#---------------------------------------------------------------------------
   To: C:\ncs\v2.9.0\zephyr\dts\bindings\sensor\sensirion,sts40.yaml
   ```

### Option B: Create NCS Module

1. **Create module directory**:
   ```
   C:\ncs\v2.9.0\modules\sts40-driver\
   ```

2. **Copy driver files to module**:
   ```
   C:\ncs\v2.9.0\modules\sts40-driver\
   ├── zephyr/
   │   ├── module.yml
   │   ├── CMakeLists.txt
   │   └── Kconfig
   ├── drivers/
   │   └── sensor/
   │       └── sts40/          # Your driver files here
   └── dts/
       └── bindings/
           └── sensor/
               └── sensirion,sts40.yaml
   ```

3. **Create module.yml**:
   ```yaml
   name: sts40-driver
   build:
     cmake: zephyr
     kconfig: zephyr/Kconfig
     settings:
       dts_root: .
   ```

4. **Create module CMakeLists.txt**:
   ```cmake
   add_subdirectory(drivers/sensor/sts40)
   ```

5. **Create module Kconfig**:
   ```kconfig
   source "drivers/sensor/sts40/Kconfig"
   ```

### Using with NCS Projects

After integration, use in any NCS project:

```c
// In your NCS application
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

const struct device *sts40 = DEVICE_DT_GET_ANY(sensirion_sts40);
```

Device tree configuration (same as before):
```dts
&i2c1 {
    status = "okay";
    
    sts40: sts40@46 {
        compatible = "sensirion,sts40";
        reg = <0x46>;
        precision = "high";
        status = "okay";
    };
};
```

**Recommendation**: Use Option A (add to Zephyr tree) since NCS already has the Sensirion infrastructure in place.

## Using the Driver in Your Application

### Basic Usage

```c
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

const struct device *sts40 = DEVICE_DT_GET_ANY(sensirion_sts40);

int main(void)
{
    struct sensor_value temp;
    
    if (!device_is_ready(sts40)) {
        return -ENODEV;
    }
    
    while (1) {
        sensor_sample_fetch(sts40);
        sensor_channel_get(sts40, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        
        printk("Temperature: %d.%02d °C\n", 
               temp.val1, temp.val2 / 10000);
        
        k_sleep(K_SECONDS(2));
    }
}
```

### Advanced Usage with Error Handling

```c
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

const struct device *sts40 = DEVICE_DT_GET_ANY(sensirion_sts40);

int read_temperature(double *temperature)
{
    struct sensor_value temp;
    int ret;
    
    ret = sensor_sample_fetch(sts40);
    if (ret < 0) {
        LOG_ERR("Failed to fetch sample: %d", ret);
        return ret;
    }
    
    ret = sensor_channel_get(sts40, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    if (ret < 0) {
        LOG_ERR("Failed to get temperature: %d", ret);
        return ret;
    }
    
    *temperature = sensor_value_to_double(&temp);
    return 0;
}

int main(void)
{
    double temperature;
    struct sensor_value serial;
    int ret;
    
    if (!device_is_ready(sts40)) {
        LOG_ERR("STS40 device not ready");
        return -ENODEV;
    }
    
    /* Read serial number */
    ret = sensor_attr_get(sts40, SENSOR_CHAN_AMBIENT_TEMP,
                         SENSOR_ATTR_CONFIGURATION, &serial);
    if (ret == 0) {
        LOG_INF("STS40 Serial: 0x%08x", (uint32_t)serial.val1);
    }
    
    while (1) {
        ret = read_temperature(&temperature);
        if (ret == 0) {
            LOG_INF("Temperature: %.2f °C", temperature);
        }
        
        k_sleep(K_SECONDS(2));
    }
}
```

## Configuration Options

### Device Tree Properties

```dts
sts40: sts40@46 {
    compatible = "sensirion,sts40";
    reg = <0x46>;                    /* I2C address: 0x44, 0x45, or 0x46 */
    precision = "high";              /* "high", "medium", or "low" */
    status = "okay";
};
```

### Kconfig Options

```
CONFIG_STS40=y                       # Enable driver
CONFIG_STS40_USE_SENSIRION_LIB=n     # Use original Sensirion library
CONFIG_STS40_PRECISION_HIGH=y        # Default precision level
```

## Board-Specific Integration

### nRF52840 DK Example

Create `boards/nrf52840dk_nrf52840.overlay`:

```dts
&i2c0 {
    compatible = "nordic,nrf-twi";
    status = "okay";
    sda-pin = <26>;
    scl-pin = <27>;
    
    sts40: sts40@46 {
        compatible = "sensirion,sts40";
        reg = <0x46>;
        precision = "high";
        status = "okay";
    };
};
```

### STM32 Nucleo Example

Create `boards/nucleo_f429zi.overlay`:

```dts
&i2c1 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_STANDARD>;
    
    sts40: sts40@46 {
        compatible = "sensirion,sts40";
        reg = <0x46>;
        precision = "medium";
        status = "okay";
    };
};
```

## Troubleshooting

### Common Build Issues

1. **Missing binding file**: Copy `sensirion,sts40.yaml` to appropriate location
2. **Kconfig not found**: Ensure Kconfig is properly sourced
3. **CMake errors**: Check CMakeLists.txt includes and paths

### Runtime Issues

1. **Device not ready**: Check I2C configuration and pinout
2. **CRC errors**: Verify I2C signal integrity and pull-up resistors
3. **No response**: Confirm sensor address and power supply

### Debug Commands

```bash
# Check device tree compilation
west build -t devicetree

# Enable debug logging
# Add to prj.conf:
CONFIG_SENSOR_LOG_LEVEL_DBG=y
CONFIG_LOG_MODE_IMMEDIATE=y

# Check I2C bus
# If your board supports it:
i2cdetect
```

## Performance Considerations

- **Precision vs Speed**: Higher precision takes longer (all ~10ms for STS40)
- **Power Consumption**: Consider measurement frequency for battery applications  
- **I2C Speed**: STS40 supports up to 1MHz I2C clock
- **CRC Overhead**: Native driver includes CRC validation

## Migration from Other Libraries

### From Arduino Libraries

Replace Arduino-style calls:

```c
// Arduino style
float temp = sts40.readTemperature();

// Zephyr style
struct sensor_value temp_val;
sensor_sample_fetch(sts40);
sensor_channel_get(sts40, SENSOR_CHAN_AMBIENT_TEMP, &temp_val);
float temp = sensor_value_to_double(&temp_val);
```

### From Sensirion Library

The driver provides both implementations:

```c
// Direct Sensirion library (with CONFIG_STS40_USE_SENSIRION_LIB=y)
// Uses familiar Sensirion API under the hood

// Native Zephyr (CONFIG_STS40_USE_SENSIRION_LIB=n)  
// Better integration with Zephyr ecosystem
```
