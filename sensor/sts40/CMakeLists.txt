# Copyright (c) 2025
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if(CONFIG_STS40_USE_SENSIRION_LIB)
    # Use Sensirion library implementation
    zephyr_library_sources_ifdef(CONFIG_STS40 sts40_sensirion.c)
    
    # Include Sensirion library files
    zephyr_library_sources(
        ../embedded-i2c-sts4x/sts4x_i2c.c
        ../embedded-i2c-sts4x/sensirion_common.c
        ../embedded-i2c-sts4x/sensirion_i2c.c
    )
    
    zephyr_library_include_directories(../embedded-i2c-sts4x)
else()
    # Use native Zephyr implementation
    zephyr_library_sources_ifdef(CONFIG_STS40 sts40.c)
endif()
