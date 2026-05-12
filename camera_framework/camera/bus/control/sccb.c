/**
 * @file    sccb.c
 * @brief   SCCB (Serial Camera Control Bus) communication layer
 *
 * This module implements the SCCB protocol over I2C for
 * reading and writing OV2640 sensor registers.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "sccb.h"
#include <string.h>

#define DBG_TAG "sccb"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;

static rt_err_t sccb_configure_pins(const char *i2c_bus_name)
{
    if (i2c_bus_name == RT_NULL)
    {
        LOG_E("SCCB: I2C bus name is null");
        return -RT_EINVAL;
    }

    if (strcmp(i2c_bus_name, "i2c1") == 0)
    {
        HAL_PIN_Set(SCCB_SCL_PIN, I2C1_SCL, PIN_PULLUP, 1);
        HAL_PIN_Set(SCCB_SDA_PIN, I2C1_SDA, PIN_PULLUP, 1);
        return RT_EOK;
    }

    if (strcmp(i2c_bus_name, "i2c2") == 0)
    {
        HAL_PIN_Set(SCCB_SCL_PIN, I2C2_SCL, PIN_PULLUP, 1);
        HAL_PIN_Set(SCCB_SDA_PIN, I2C2_SDA, PIN_PULLUP, 1);
        return RT_EOK;
    }

    LOG_E("SCCB: unsupported I2C bus name %s", i2c_bus_name);
    return -RT_EINVAL;
}

rt_err_t sccb_init(const char *i2c_bus_name)
{
    rt_err_t ret = sccb_configure_pins(i2c_bus_name);
    if (ret != RT_EOK)
    {
        return ret;
    }

    i2c_bus = rt_i2c_bus_device_find(i2c_bus_name);
    if (i2c_bus == RT_NULL)
    {
        LOG_E("SCCB: I2C bus %s not found!", i2c_bus_name);
        return -RT_ERROR;
    }
    ret = rt_device_open((rt_device_t)i2c_bus, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to open I2C bus: %d", ret);
        return ret;
    }
    
    struct rt_i2c_configuration configuration =
    {
        .mode = 0,
        .addr = 0,
        .timeout = SCCB_TIMEOUT_MS, //Waiting for timeout period (ms)
        .max_hz = SCCB_MAX_HZ, //I2C rate (hz)
    };
    // config I2C parameter
    return rt_i2c_configure(i2c_bus, &configuration);
}

void sccb_deinit(void)
{
    if (i2c_bus != RT_NULL)
    {
        rt_device_close((rt_device_t)i2c_bus);
        i2c_bus = RT_NULL;
        LOG_I("SCCB deinitialized");
    }
}

int sccb_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    rt_size_t res = 0;
    uint8_t buf[2];
    buf[0] = reg_addr;
    buf[1] = data;
    res = rt_i2c_master_send(i2c_bus, dev_addr, RT_I2C_WR , buf, 2);
    if(res<2) return -RT_ERROR;
    return RT_EOK;
}

uint8_t sccb_read(uint8_t dev_addr, uint8_t reg_addr)
{
    rt_size_t res = 0;
    uint8_t data = 0;
    res = rt_i2c_master_send(i2c_bus, dev_addr, RT_I2C_WR , &reg_addr, 1);
    if(res<1)
    {
        LOG_W("SCCB: failed to send register 0x%02X to device 0x%02X", reg_addr, dev_addr);
        return 0;
    }
    res = rt_i2c_master_recv(i2c_bus, dev_addr, RT_I2C_RD , &data, 1);
    if(res<1)
    {
        LOG_W("SCCB: failed to read from device 0x%02X register 0x%02X", dev_addr, reg_addr);
        return 0;
    }
    return data;
}