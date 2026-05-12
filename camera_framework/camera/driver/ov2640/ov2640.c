/**
 * @file    ov2640.c
 * @brief   OV2640 camera sensor driver and RT-Thread device interface
 *
 * This module implements the OV2640 sensor initialization, register
 * configuration, image parameter control, and RT-Thread standard
 * device driver interface (open/read/close/control).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "ov2640.h"
#include "ov2640_regs.h"
#include "ov2640_settings.h"
#include "rtthread.h"
#include "rthw.h"
#include "drivers/pin.h"

#define DBG_TAG "ov2640"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <string.h>

/*
 * Single-instance driver: the SCCB control bus (sccb.c) and DVP data bus
 * (dvp.c) currently keep file-scope state, so only one ov2640_device_t may
 * exist. Sensor-setter helpers below (ov2640_lock/unlock, ov2640_set_bank,
 * …) take no parameters and reach the active instance through this
 * singleton pointer, which is set in ov2640_device_register. A future
 * multi-instance refactor only needs to thread an ov2640_device_t* into
 * the setters; the per-instance fields already live on the struct.
 */
static ov2640_device_t *s_active_dev = RT_NULL;

/*
 *******************************************************************************
 * Module-level data
 *******************************************************************************
 */

static const camera_device_ops_t g_ov2640_device_ops = {
    .command_set = {
        OV2640_CMD_SET_PIXFORMAT,
        OV2640_CMD_SET_FRAMESIZE,
        OV2640_CMD_SET_QUALITY,
        OV2640_CMD_START_STREAM,
        OV2640_CMD_STOP_STREAM,
    }
};

/**
 * @brief Reset streaming state to inactive / all-zeroes.
 *
 * @param stream is a pointer to the stream state to reset.
 */
static void ov2640_stream_reset(ov2640_stream_state_t *stream)
{
    if (stream == RT_NULL)
    {
        return;
    }

    rt_memset(stream, 0, sizeof(*stream));
}

/**
 * @brief Return the static ops table for the OV2640 camera device.
 *
 * @return Return a pointer to the static @c camera_device_ops_t.
 */
const camera_device_ops_t *ov2640_get_device_ops(void)
{
    return &g_ov2640_device_ops;
}

/*
 *******************************************************************************
 * Thread safety helpers
 *******************************************************************************
 */

/**
 * @brief Lazily initialize the per-instance SCCB mutex.
 *
 * @param dev is the device instance whose @c sccb_lock is initialized.
 *            If @c NULL, the call is a no-op.
 */
static void ov2640_mutex_init(ov2640_device_t *dev)
{
    if (dev == RT_NULL || dev->sccb_lock_initialized)
    {
        return;
    }
    rt_mutex_init(&dev->sccb_lock, "ov2640_lock", RT_IPC_FLAG_PRIO);
    dev->sccb_lock_initialized = RT_TRUE;
}

/**
 * @brief Lock the active OV2640 instance's SCCB mutex.
 *
 * Uses @ref s_active_dev set by @ref ov2640_device_register. Returns
 * @c RT_EOK silently when no active instance is bound — this matches the
 * legacy behaviour of being safe to call before register/after unregister.
 *
 * @return Return RT_EOK on success or when there is no active instance.
 */
static rt_err_t ov2640_lock(void)
{
    ov2640_device_t *dev = s_active_dev;
    if (dev == RT_NULL)
    {
        return RT_EOK;
    }
    if (!dev->sccb_lock_initialized)
    {
        ov2640_mutex_init(dev);
    }
    return rt_mutex_take(&dev->sccb_lock, RT_WAITING_FOREVER);
}

/**
 * @brief Unlock the active OV2640 instance's SCCB mutex.
 */
static void ov2640_unlock(void)
{
    ov2640_device_t *dev = s_active_dev;
    if (dev != RT_NULL && dev->sccb_lock_initialized)
    {
        rt_mutex_release(&dev->sccb_lock);
    }
}

/*
 *******************************************************************************
 * Internal resolution and aspect-ratio tables
 *******************************************************************************
 */

typedef enum {
    ASPECT_RATIO_4X3,
    ASPECT_RATIO_3X2,
    ASPECT_RATIO_16X10,
    ASPECT_RATIO_5X3,
    ASPECT_RATIO_16X9,
    ASPECT_RATIO_21X9,
    ASPECT_RATIO_5X4,
    ASPECT_RATIO_1X1,
    ASPECT_RATIO_9X16
} aspect_ratio_t;

typedef struct {
        const uint16_t width;
        const uint16_t height;
        const aspect_ratio_t aspect_ratio;
} resolution_info_t;

static const resolution_info_t resolution[FRAMESIZE_INVALID] = {
    {   96,   96, ASPECT_RATIO_1X1   }, /* 96x96 */
    {  160,  120, ASPECT_RATIO_4X3   }, /* QQVGA */
    {  128,  128, ASPECT_RATIO_1X1   }, /* 128x128 */
    {  176,  144, ASPECT_RATIO_5X4   }, /* QCIF  */
    {  240,  176, ASPECT_RATIO_4X3   }, /* HQVGA */
    {  240,  240, ASPECT_RATIO_1X1   }, /* 240x240 */
    {  320,  240, ASPECT_RATIO_4X3   }, /* QVGA  */
    {  320,  320, ASPECT_RATIO_1X1   }, /* 320x320 */
    {  400,  296, ASPECT_RATIO_4X3   }, /* CIF   */
    {  480,  320, ASPECT_RATIO_3X2   }, /* HVGA  */
    {  640,  480, ASPECT_RATIO_4X3   }, /* VGA   */
    {  800,  600, ASPECT_RATIO_4X3   }, /* SVGA  */
    { 1024,  768, ASPECT_RATIO_4X3   }, /* XGA   */
    { 1280,  720, ASPECT_RATIO_16X9  }, /* HD    */
    { 1280, 1024, ASPECT_RATIO_5X4   }, /* SXGA  */
    { 1600, 1200, ASPECT_RATIO_4X3   }, /* UXGA  */
};

/*
 *******************************************************************************
 * Low-level SCCB register I/O
 *******************************************************************************
 */

/* Bank cache lives in ov2640_device_t::current_bank; helpers below access it
 * through s_active_dev so callers don't need to thread the device pointer. */

/**
 * @brief Invalidate the cached bank so the next access forces a BANK_SEL write.
 *
 * Call on deinit to ensure the driver state is consistent after re-open.
 */
static void ov2640_reset_bank_state(void)
{
    if (s_active_dev != RT_NULL)
    {
        s_active_dev->current_bank = (rt_uint8_t)BANK_MAX;
    }
}

/**
 * @brief Switch the active register bank if needed.
 *
 * Skips the SCCB write when @p bank already matches the cached value.
 *
 * @param bank is the target bank (BANK_SENSOR or BANK_DSP).
 *
 * @return Return 0 on success; negative SCCB error on failure.
 */
static int ov2640_set_bank(ov2640_bank_t bank)
{
    ov2640_device_t *dev = s_active_dev;
    rt_uint8_t cached = (dev != RT_NULL) ? dev->current_bank : (rt_uint8_t)BANK_MAX;
    if ((rt_uint8_t)bank == cached) return 0;
    int res = sccb_write(OV2640_ADDR, BANK_SEL, (uint8_t)bank);
    if(res) return res;
    if (dev != RT_NULL)
    {
        dev->current_bank = (rt_uint8_t)bank;
    }
    return 0;
}

/**
 * @brief  Write OV2640 single register
 * @param  bank: Bank ID
 * @param  reg: Register address
 * @param  data: Data to write
 * @return 0 on success, negative error code on failure
 */
static int ov2640_write_reg(ov2640_bank_t bank, uint8_t reg, uint8_t data)
{
    int ret;
    ret = ov2640_set_bank(bank);
    if(ret) return ret;
    ret = sccb_write(OV2640_ADDR, reg, data);
    if(ret) return ret;
    return 0;
}

/**
 * @brief  Write OV2640 multiple registers
 * @param  regs: Pointer to register array, ending with {0, 0}
 * @return RT_EOK on success, negative error code on failure
 */
static int ov2640_write_regs(const uint8_t (*regs)[2])
{
    int ret;
    const uint8_t (*reg_ptr)[2] = regs;

    while(((*reg_ptr)[0] != 0) || ((*reg_ptr)[1] != 0))
    {
        ret = sccb_write(OV2640_ADDR, (*reg_ptr)[0], (*reg_ptr)[1]);
        if(ret) return ret;
        reg_ptr++;
    }
    return 0;
}

/**
 * @brief  Read OV2640 single register
 * @param  data: Pointer to store read data
 * @return register value, if read fails, return 0
 */
static uint8_t ov2640_read_reg(uint8_t reg)
{
    return sccb_read(OV2640_ADDR, reg);
}

/**
 * @brief Get specific bits from OV2640 register, align to LSB by offset
 * @param reg: Register address
 * @param bank: Register bank
 * @param mask: Bit mask
 * @param offset: Bit offset
 * @param value: Pointer to store the extracted bits
 * @return aligned register bits, if read fails, return 0
 */
static uint8_t ov2640_get_bits(uint8_t reg,ov2640_bank_t bank, uint8_t mask, uint32_t offset)
{
    int ret;
    uint8_t reg_value;
    ret = ov2640_set_bank(bank);
    if(ret) return 0;
    reg_value = ov2640_read_reg(reg);
    return (reg_value >> offset) & mask;

}

/*
 *******************************************************************************
 * Sensor vtable implementations
 *******************************************************************************
 */

/* --- Lifecycle ------------------------------------------------------------ */

/**
 * @brief Reset OV2640 sensor to default settings.
 *
 * Sends the software reset command then writes the base CIF initialisation
 * register table.
 *
 * @param dev is a pointer to the sensor handle.
 *
 * @return Return 0 on success; negative error code on failure.
 */
static int ov2640_reset(ov2640_t *dev)
{
    int ret;   
    ov2640_lock();
    ov2640_write_reg(BANK_SENSOR, COM7, COM7_SRST);
    rt_thread_mdelay(10);
    ov2640_write_regs(ov2640_settings_cif);
    ov2640_unlock();
    return 0;
}

/**
 * @brief Read all sensor registers and populate @p dev->status.
 *
 * Queries AEC, AGC, AWB, gain ceiling, gamma, lens correction and geometry
 * settings over SCCB and mirrors them into the in-memory status struct so
 * callers can inspect parameters without further I2C transactions.
 *
 * @param dev is a pointer to the sensor handle.
 *
 * @return Return 0 on success; -1 if a bank switch fails.
 */
static int ov2640_init_status(ov2640_t *dev)
{
    int ret;
    uint8_t reg_value_h, reg_value_m, reg_value_l;
    uint8_t temp_value;
    
    ov2640_lock();
    
    dev->status.brightness = 0;
    dev->status.contrast = 0;
    dev->status.saturation = 0;
    dev->status.ae_level = 0;
    dev->status.special_effect = 0;
    dev->status.wb_mode = 0;

    // Read AEC value (16-bit value split across 3 registers)
    // AEC[15:10] from REG45, AEC[9:2] from AEC, AEC[1:0] from REG04
    reg_value_h = ov2640_get_bits(REG45, BANK_SENSOR, REG45_AEC_MASK, REG45_AEC_OFFSET);
    ret = ov2640_set_bank(BANK_SENSOR);
    if(ret != 0) return -1;
    reg_value_m = ov2640_read_reg(AEC);
    reg_value_l = ov2640_get_bits(REG04, BANK_SENSOR, REG04_AEC_MASK, REG04_AEC_OFFSET);
    dev->status.aec_value = ((uint16_t)reg_value_h << 10) | ((uint16_t)reg_value_m << 2) | reg_value_l; // 0 - 1200
    
    // Read quality
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) return -1;
    temp_value = ov2640_read_reg(QS);
    dev->status.quality = temp_value;
    
    // Read gain ceiling
    dev->status.gainceiling = ov2640_get_bits(COM9, BANK_SENSOR, COM9_GAINCEILING_MASK, COM9_GAINCEILING_OFFSET);
    
    // Read AWB
    dev->status.awb = ov2640_get_bits(CTRL1, BANK_DSP, CTRL1_AWB_MASK, CTRL1_AWB_OFFSET);
    
    // Read AWB gain
    dev->status.awb_gain = ov2640_get_bits(CTRL1, BANK_DSP, CTRL1_AWB_GAIN_MASK, CTRL1_AWB_GAIN_OFFSET);
    
    // Read AEC
    dev->status.aec = ov2640_get_bits(COM8, BANK_SENSOR, COM8_AEC_MASK, COM8_AEC_OFFSET);
    
    // Read AEC2
    dev->status.aec2 = ov2640_get_bits(CTRL0, BANK_DSP, CTRL0_AEC2_MASK, CTRL0_AEC2_OFFSET);
    
    // Read AGC
    dev->status.agc = ov2640_get_bits(COM8, BANK_SENSOR, COM8_AGC_MASK, COM8_AGC_OFFSET);
    
    // Read BPC
    dev->status.bpc = ov2640_get_bits(CTRL3, BANK_DSP, CTRL3_BPC_MASK, CTRL3_BPC_OFFSET);
    
    // Read WPC
    dev->status.wpc = ov2640_get_bits(CTRL3, BANK_DSP, CTRL3_WPC_MASK, CTRL3_WPC_OFFSET);
    
    // Read RAW GMA
    dev->status.raw_gma = ov2640_get_bits(CTRL1, BANK_DSP, CTRL1_RAW_GMA_MASK, CTRL1_RAW_GMA_OFFSET);
    
    // Read LENC
    dev->status.lenc = ov2640_get_bits(CTRL1, BANK_DSP, CTRL1_LENC_MASK, CTRL1_LENC_OFFSET);
    
    // Read horizontal mirror
    dev->status.hmirror = ov2640_get_bits(REG04, BANK_SENSOR, REG04_HMIRROR_MASK, REG04_HMIRROR_OFFSET);
    
    // Read vertical flip
    dev->status.vflip = ov2640_get_bits(REG04, BANK_SENSOR, REG04_VFLIP_MASK, REG04_VFLIP_OFFSET);
    
    // Read DCW
    dev->status.dcw = ov2640_get_bits(CTRL2, BANK_DSP, CTRL2_DCW_MASK, CTRL2_DCW_OFFSET);
    
    // Read color bar
    dev->status.colorbar = ov2640_get_bits(COM7, BANK_SENSOR, COM7_COLORBAR_MASK, COM7_COLORBAR_OFFSET);
    
    // Sharpness and denoise are not supported
    dev->status.sharpness = 0;
    dev->status.denoise = 0;
    
    ov2640_unlock();
    return 0;
}

/* --- Image format & resolution -------------------------------------------- */

/**
 * @brief Set pixel output format (RGB565, YUV422, JPEG, RAW8).
 *
 * @param dev      is a pointer to the sensor handle.
 * @param pixformat is the desired output format.
 *
 * @return Return 0 on success; -1 for an unsupported format.
 */
static int ov2640_set_pixformat(ov2640_t *dev, pixformat_t pixformat)
{
    int ret = 0;
    
    ov2640_lock();
    dev->pixformat = pixformat;

    switch(pixformat)
    {
        case PIXFORMAT_RGB565:
            ov2640_write_regs(ov2640_settings_rgb565);
            break;
        case PIXFORMAT_YUV422:
            ov2640_write_regs(ov2640_settings_yuv422);
            break;
        case PIXFORMAT_JPEG:
            ov2640_write_regs(ov2640_settings_jpeg3);
            break;
        case PIXFORMAT_RAW8:
            ov2640_write_regs(ov2640_settings_raw8);
            break;
        default:
            ret = -1;
    }
    ov2640_unlock();

    return ret;
}

/**
 * @brief Configure sensor window (crop and scale) settings
 * @param dev Pointer to ov2640_t structure
 * @param mode Sensor mode (CIF, SVGA, UXGA)
 * @param offset_x Horizontal offset
 * @param offset_y Vertical offset
 * @param max_x Maximum horizontal size
 * @param max_y Maximum vertical size
 * @param w Output width
 * @param h Output height
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_window(ov2640_t *dev, ov2640_sensor_mode_t mode,int offset_x, int offset_y, int max_x, int max_y, int w, int h)
{
    int ret;
    const uint8_t (*regs)[2];
    ov2640_clk_t c;
    c.reserved = 0;
    
    ov2640_lock();

    max_x /= 4;
    max_y /= 4;
    w /= 4;
    h /= 4;
    uint8_t win_regs[][2] = {
        {BANK_SEL, BANK_DSP},
        {HSIZE, max_x & 0xFF},
        {VSIZE, max_y & 0xFF},
        {XOFFL, offset_x & 0xFF},
        {YOFFL, offset_y & 0xFF},
        {VHYX, ((max_y >> 1) & 0X80) | ((offset_y >> 4) & 0X70) | ((max_x >> 5) & 0X08) | ((offset_x >> 8) & 0X07)},
        {TEST, (max_x >> 2) & 0X80},
        {ZMOW, (w)&0xFF},
        {ZMOH, (h)&0xFF},
        {ZMHH, ((h>>6)&0x04)|((w>>8)&0x03)},
        {0, 0}
    };

    if (dev->pixformat == PIXFORMAT_JPEG) {
        c.clk_2x = 1;
        c.clk_div = 0;
        c.pclk_auto = 0;
        c.pclk_div =6;
        if(mode == OV2640_MODE_UXGA) {
            c.pclk_div = 24;
        }
    } else {
        c.clk_2x = 1;
        c.clk_div =3;
        c.pclk_auto = 1;
        c.pclk_div = 4;
        if (mode == OV2640_MODE_CIF) {
            c.clk_div = 3;
        } else if(mode == OV2640_MODE_UXGA) {
            c.pclk_div = 12;
        }
    }

    if (mode == OV2640_MODE_CIF) {
        regs = ov2640_settings_to_cif;
    } else if (mode == OV2640_MODE_SVGA) {
        regs = ov2640_settings_to_svga;
    } else {
        regs = ov2640_settings_to_uxga;
    }

    ov2640_set_bank(BANK_DSP);
    ov2640_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
    ov2640_write_regs(regs);
    ov2640_write_regs(win_regs);
    ov2640_set_bank(BANK_SENSOR);
    ov2640_write_reg(BANK_SENSOR, CLKRC, c.clk);
    ov2640_set_bank(BANK_DSP);
    ov2640_write_reg(BANK_DSP, R_DVP_SP, c.pclk);
    ov2640_set_bank(BANK_DSP);
    ov2640_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_EN);

    rt_thread_mdelay(10);
    //required when changing resolution
    /* Note: ov2640_set_pixformat will acquire its own lock, so we need to release first */
    ov2640_unlock();
    ov2640_set_pixformat(dev, dev->pixformat);

    return 0;
}

/**
 * @brief Set frame size/resolution
 * @param dev Pointer to ov2640_t structure
 * @param framesize Desired frame size (QVGA, VGA, SVGA, UXGA, etc.)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_framesize(ov2640_t *dev, framesize_t framesize)
{
    if(framesize >= FRAMESIZE_INVALID) {
        return -1;
    }

    int ret = 0;
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    aspect_ratio_t ratio = resolution[framesize].aspect_ratio;
    uint16_t max_x = ratio_table[ratio].max_x;
    uint16_t max_y = ratio_table[ratio].max_y;
    uint16_t offset_x = ratio_table[ratio].offset_x;
    uint16_t offset_y = ratio_table[ratio].offset_y;
    ov2640_sensor_mode_t mode = OV2640_MODE_UXGA;

    dev->status.framesize = framesize;

    if (framesize <= FRAMESIZE_CIF) {
        mode = OV2640_MODE_CIF;
        max_x /= 4;
        max_y /= 4;
        offset_x /= 4;
        offset_y /= 4;
        if(max_y > 296){
            max_y = 296;
        }
    } else if (framesize <= FRAMESIZE_SVGA) {
        mode = OV2640_MODE_SVGA;
        max_x /= 2;
        max_y /= 2;
        offset_x /= 2;
        offset_y /= 2;
    }

    ret = ov2640_set_window(dev, mode, offset_x, offset_y, max_x, max_y, w, h);
    return ret;
}

/* --- Image quality -------------------------------------------------------- */

/**
 * @brief Set image contrast level.
 *
 * @param dev   is a pointer to the sensor handle.
 * @param level is the contrast level: -2 (low) ... +2 (high), 0 = default.
 *
 * @return Return 0 on success; negative SCCB error on failure.
 */
static int ov2640_set_contrast(ov2640_t *dev, int level)
{
    int ret = 0;
    
    ov2640_lock();
    level += 2; // Map -2~2 to 0~4
    ret = ov2640_write_reg(BANK_SENSOR, 0x81, (uint8_t)(0x14 + level * 4));
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.contrast = level - 2;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set image brightness level
 * @param dev Pointer to ov2640_t structure
 * @param level Brightness level (-2 to +2, 0 is default)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_brightness(ov2640_t *dev, int level)
{
    int ret = 0;
    
    ov2640_lock();
    if(level < -2) level = -2;
    if(level > 2) level = 2;
    level += 2; // Map -2~2 to 0~4
    ret = ov2640_write_reg(BANK_SENSOR, 0x9B, (uint8_t)(level * 0x20));
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.brightness = level - 2;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set image color saturation level
 * @param dev Pointer to ov2640_t structure
 * @param level Saturation level (-2 to +2, 0 is default)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_saturation(ov2640_t *dev, int level)
{
    int ret = 0;
    
    ov2640_lock();
    if(level < -2) level = -2;
    if(level > 2) level = 2;
    level += 2; // Map -2~2 to 0~4
    ret = ov2640_write_reg(BANK_DSP, 0xD9, (uint8_t)(0x40 + level * 0x10));
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.saturation = level - 2;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set image sharpness (not supported by OV2640)
 * @param dev Pointer to ov2640_t structure
 * @param level Sharpness level (ignored)
 * @return 0 (always success, feature not supported)
 */
static int ov2640_set_sharpness(ov2640_t *dev, int level)
{
    // OV2640 doesn't support sharpness adjustment
    dev->status.sharpness = 0;
    return 0;
}

/**
 * @brief Set denoise level (not supported by OV2640)
 * @param dev Pointer to ov2640_t structure
 * @param level Denoise level (ignored)
 * @return 0 (always success, feature not supported)
 */
static int ov2640_set_denoise(ov2640_t *dev, int level)
{
    // OV2640 doesn't support direct denoise level adjustment
    dev->status.denoise = 0;
    return 0;
}

/**
 * @brief Set AGC (Automatic Gain Control) gain ceiling
 * @param dev Pointer to ov2640_t structure
 * @param gainceiling Gain ceiling value (2x, 4x, 8x, 16x, 32x, 64x, 128x)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_gainceiling(ov2640_t *dev, gainceiling_t gainceiling)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(COM9);
    reg_value = (reg_value & ~COM9_GAINCEILING_MASK) | ((gainceiling << COM9_GAINCEILING_OFFSET) & COM9_GAINCEILING_MASK);
    ret = ov2640_write_reg(BANK_SENSOR, COM9, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.gainceiling = gainceiling;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set JPEG compression quality
 * @param dev Pointer to ov2640_t structure
 * @param quality Quality scale (0-63, lower = better quality, higher compression)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_quality(ov2640_t *dev, int quality)
{
    int ret = 0;
    
    ov2640_lock();
    // Quality scale: 0-63 (lower = better quality, higher compression)
    if(quality < 0) quality = 0;
    if(quality > 63) quality = 63;
    ret = ov2640_write_reg(BANK_DSP, QS, (uint8_t)quality);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.quality = quality;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable color bar test pattern
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_colorbar(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(COM7);
    if(enable) {
        reg_value |= COM7_COLORBAR_MASK;
    } else {
        reg_value &= ~COM7_COLORBAR_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, COM7, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.colorbar = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/* --- White balance, exposure & gain --------------------------------------- */

/**
 * @brief Enable or disable automatic white balance (AWB).
 *
 * @param dev    is a pointer to the sensor handle.
 * @param enable is 1 to enable AWB, 0 to disable.
 *
 * @return Return 0 on success; negative SCCB error on failure.
 */
static int ov2640_set_whitebal(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL1);
    if(enable) {
        reg_value |= CTRL1_AWB_MASK;
    } else {
        reg_value &= ~CTRL1_AWB_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL1, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.awb = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable automatic gain control (AGC)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AGC, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_gain_ctrl(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(COM8);
    if(enable) {
        reg_value |= COM8_AGC_MASK;
    } else {
        reg_value &= ~COM8_AGC_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, COM8, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.agc = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable automatic exposure control (AEC)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AEC, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_exposure_ctrl(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(COM8);
    if(enable) {
        reg_value |= COM8_AEC_MASK;
    } else {
        reg_value &= ~COM8_AEC_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, COM8, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.aec = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable horizontal mirror (flip image horizontally)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable horizontal mirror, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_hmirror(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(REG04);
    if(enable) {
        reg_value |= REG04_HMIRROR_MASK;
    } else {
        reg_value &= ~REG04_HMIRROR_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, REG04, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.hmirror = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable vertical flip (flip image vertically)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable vertical flip, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_vflip(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(REG04);
    if(enable) {
        reg_value |= REG04_VFLIP_MASK;
    } else {
        reg_value &= ~REG04_VFLIP_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, REG04, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.vflip = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable AEC2 (enhanced automatic exposure control)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AEC2, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_aec2(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL0);
    if(enable) {
        reg_value |= CTRL0_AEC2_MASK;
    } else {
        reg_value &= ~CTRL0_AEC2_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL0, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.aec2 = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable AWB gain control
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AWB gain, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_awb_gain(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL1);
    if(enable) {
        reg_value |= CTRL1_AWB_GAIN_MASK;
    } else {
        reg_value &= ~CTRL1_AWB_GAIN_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL1, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.awb_gain = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set manual AGC gain value
 * @param dev Pointer to ov2640_t structure
 * @param gain Gain value (0-30)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_agc_gain(ov2640_t *dev, int gain)
{
    int ret = 0;
    
    ov2640_lock();
    // AGC gain: 0-30 maps to register value
    if(gain < 0) gain = 0;
    if(gain > 30) gain = 30;
    ret = ov2640_write_reg(BANK_SENSOR, GAIN, (uint8_t)gain);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.agc_gain = gain;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set manual AEC exposure value
 * @param dev Pointer to ov2640_t structure
 * @param value Exposure value (0-1200)
 * @return 0 on success, negative error code on failure
 * @note Value is split across 3 registers: REG45[5:0], AEC[7:0], REG04[1:0]
 */
static int ov2640_set_aec_value(ov2640_t *dev, int value)
{
    int ret = 0;
    // AEC value: 0-1200, split across 3 registers
    // AEC[15:10] -> REG45[5:0]
    // AEC[9:2] -> AEC[7:0]
    // AEC[1:0] -> REG04[1:0]
    if(value < 0) value = 0;
    if(value > 1200) value = 1200;
    
    ov2640_lock();
    
    uint8_t reg45_val = ov2640_read_reg(REG45);
    reg45_val = (reg45_val & ~REG45_AEC_MASK) | ((value >> 10) & REG45_AEC_MASK);
    ret = ov2640_write_reg(BANK_SENSOR, REG45, reg45_val);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    ret = ov2640_write_reg(BANK_SENSOR, AEC, (uint8_t)((value >> 2) & 0xFF));
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    uint8_t reg04_val = ov2640_read_reg(REG04);
    reg04_val = (reg04_val & ~REG04_AEC_MASK) | (value & REG04_AEC_MASK);
    ret = ov2640_write_reg(BANK_SENSOR, REG04, reg04_val);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    dev->status.aec_value = value;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set special image effect
 * @param dev Pointer to ov2640_t structure
 * @param effect Effect mode:
 *        - 0: Normal
 *        - 1: Negative
 *        - 2: Black & White
 *        - 3: Reddish
 *        - 4: Greenish
 *        - 5: Bluish
 *        - 6: Sepia
 * @return 0 on success, -1 if effect is out of range
 */
static int ov2640_set_special_effect(ov2640_t *dev, int effect)
{
    int ret = 0;
    // Special effects: 0=Normal, 1=Negative, 2=B&W, 3=Reddish, 4=Greenish, 5=Bluish, 6=Sepia
    const uint8_t effects[][3] = {
        {0x00, 0x80, 0x80}, // Normal
        {0x40, 0x80, 0x80}, // Negative
        {0x18, 0x80, 0x80}, // B&W
        {0x18, 0x40, 0xC0}, // Reddish
        {0x18, 0x40, 0x40}, // Greenish
        {0x18, 0xA0, 0x40}, // Bluish
        {0x18, 0x40, 0xA0}, // Sepia
    };
    
    if(effect < 0 || effect > 6) return -1;
    
    ov2640_lock();
    
    ret = ov2640_write_reg(BANK_DSP, 0xD7, effects[effect][0]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, 0xD8, effects[effect][1]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, 0xD9, effects[effect][2]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    dev->status.special_effect = effect;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set white balance mode
 * @param dev Pointer to ov2640_t structure
 * @param mode White balance mode:
 *        - 0: Auto (AWB enabled)
 *        - 1: Sunny
 *        - 2: Cloudy
 *        - 3: Office
 *        - 4: Home
 * @return 0 on success, -1 if mode is out of range
 */
static int ov2640_set_wb_mode(ov2640_t *dev, int mode)
{
    int ret = 0;
    // WB modes: 0=Auto, 1=Sunny, 2=Cloudy, 3=Office, 4=Home
    const uint8_t wb_modes[][3] = {
        {0x00, 0x00, 0x00}, // Auto (AWB enabled)
        {0x52, 0x41, 0x00}, // Sunny
        {0x65, 0x41, 0x00}, // Cloudy
        {0x45, 0x51, 0x00}, // Office
        {0x42, 0x51, 0x00}, // Home
    };
    
    if(mode < 0 || mode > 4) return -1;
    
    if(mode == 0) {
        // Enable AWB (set_whitebal will acquire its own lock)
        ret = ov2640_set_whitebal(dev, 1);
    } else {
        // Disable AWB and set manual WB
        ret = ov2640_set_whitebal(dev, 0);
        if(ret != 0) return ret;
        
        ov2640_lock();
        ret = ov2640_write_reg(BANK_DSP, 0xCC, wb_modes[mode][0]);
        if(ret != 0) {
            ov2640_unlock();
            return ret;
        }
        ret = ov2640_write_reg(BANK_DSP, 0xCD, wb_modes[mode][1]);
        if(ret != 0) {
            ov2640_unlock();
            return ret;
        }
        ret = ov2640_write_reg(BANK_DSP, 0xCE, wb_modes[mode][2]);
        if(ret != 0) {
            ov2640_unlock();
            return ret;
        }
        ov2640_unlock();
    }
    
    dev->status.wb_mode = mode;
    return 0;
}

/**
 * @brief Set automatic exposure level
 * @param dev Pointer to ov2640_t structure
 * @param level AE level (-2 to +2, 0 is default)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_ae_level(ov2640_t *dev, int level)
{
    int ret = 0;
    // AE level: -2 to +2
    level += 2; // Map to 0~4
    if(level < 0) level = 0;
    if(level > 4) level = 4;
    
    const uint8_t ae_levels[5] = {0x40, 0x30, 0x24, 0x18, 0x10};
    
    ov2640_lock();
    ret = ov2640_write_reg(BANK_SENSOR, AEW, ae_levels[level]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_SENSOR, AEB, ae_levels[level]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    dev->status.ae_level = level - 2;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable DCW (Downsize Clock)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable DCW, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_dcw(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL2);
    if(enable) {
        reg_value |= CTRL2_DCW_MASK;
    } else {
        reg_value &= ~CTRL2_DCW_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL2, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.dcw = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable BPC (Black Pixel Correction)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable BPC, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_bpc(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL3);
    if(enable) {
        reg_value |= CTRL3_BPC_MASK;
    } else {
        reg_value &= ~CTRL3_BPC_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL3, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.bpc = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable WPC (White Pixel Correction)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable WPC, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_wpc(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL3);
    if(enable) {
        reg_value |= CTRL3_WPC_MASK;
    } else {
        reg_value &= ~CTRL3_WPC_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL3, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.wpc = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable RAW Gamma correction
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable gamma correction, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_raw_gma(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL1);
    if(enable) {
        reg_value |= CTRL1_RAW_GMA_MASK;
    } else {
        reg_value &= ~CTRL1_RAW_GMA_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL1, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.raw_gma = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable LENC (Lens Correction)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable lens correction, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_lenc(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL1);
    if(enable) {
        reg_value |= CTRL1_LENC_MASK;
    } else {
        reg_value &= ~CTRL1_LENC_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL1, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.lenc = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/*
 * NOTE: Raw register access wrappers (ov2640_get_reg / ov2640_set_reg) and
 * the placeholder vtable hooks (ov2640_set_res_raw / ov2640_set_pll /
 * ov2640_set_xclk) have been removed. XCLK is now owned by the DVP bus
 * layer (see dvp_config_t::xclk_pin / xclk_freq). Add real implementations
 * here if direct register tuning or PLL configuration is ever required.
 */

/*
 *******************************************************************************
 * Sensor public API
 *******************************************************************************
 */

/**
 * @brief Reset OV2640 and load the initial register state into the handle.
 *
 * Issues a software reset (CIF init table) and reads the configuration
 * registers into @p dev->status. The sensor is left in JPEG/UXGA defaults.
 *
 * @param dev is a pointer to the sensor handle to initialise.
 *
 * @return Return 0 on success; negative error code on failure.
 */
int ov2640_init(ov2640_t *dev)
{
    int ret;

    ret = ov2640_reset(dev);
    if (ret != 0)
    {
        return ret;
    }

    ret = ov2640_init_status(dev);
    if (ret != 0)
    {
        return ret;
    }

    return 0;
}

/*
 *******************************************************************************
 * RT-Thread device interface
 *******************************************************************************
 *
 * Control command definitions are in ov2640.h.
 */

/**
 * @brief Frame-ready callback registered with the bus adapter.
 *
 * For streaming mode, rotates ping-pong buffers and invokes the user
 * frame callback. For single-shot mode, releases @c frame_sem and halts
 * the bus. Always called from ISR / timer-thread context.
 *
 * @param self      is the bus adapter that fired the callback.
 * @param bus_frame describes the captured frame (buffer, length, sequence).
 * @param user      is the @c rt_device_t handle cast to @c void*.
 */
static void ov2640_frame_ready_callback(bus_adapter_t *self, const bus_frame_t *bus_frame, void *user)
{
    rt_device_t dev = (rt_device_t)user;
    ov2640_device_t *cam_dev = (ov2640_device_t *)dev->user_data;
    ov2640_stream_state_t *stream = &cam_dev->stream;
    rt_size_t frame_size = bus_frame ? bus_frame->length : 0;

    /*
     * Snapshot the streaming callback under an interrupt lock. The callback
     * pointer being non-NULL is the single source of truth for "streaming
     * is armed". STOP_STREAM clears it inside the same critical section so
     * we cannot observe a half-torn-down stream here.
     */
    rt_base_t level = rt_hw_interrupt_disable();
    camera_stream_frame_callback_t frame_callback = stream->frame_callback;
    void *callback_context = stream->callback_context;
    rt_hw_interrupt_enable(level);

    if (frame_callback != RT_NULL)
    {
        rt_uint8_t completed_index = stream->active_buffer_index;
        camera_stream_frame_t frame;

        frame.buffer = stream->buffers[completed_index];
        frame.buffer_size = stream->buffer_size;
        frame.frame_size = frame_size;
        frame.sequence = ++stream->sequence;
        frame.buffer_index = completed_index;

        stream->active_buffer_index ^= 1U;

        if (bus_adapter_rearm_capture(self,
                                      stream->buffers[stream->active_buffer_index],
                                      stream->buffer_size) != RT_EOK)
        {
            /* Re-arm failed: tear down stream state and abort the bus so
             * subsequent ISRs (if any) fall through the single-shot path. */
            rt_base_t lock = rt_hw_interrupt_disable();
            ov2640_stream_reset(stream);
            rt_hw_interrupt_enable(lock);
            bus_adapter_abort_capture(self);
        }

        frame_callback(callback_context, &frame);

        if (dev && dev->rx_indicate)
        {
            dev->rx_indicate(dev, frame_size);
        }
        return;
    }

    /* Notify blocking OV2640 read path via semaphore */
    rt_sem_release(&cam_dev->frame_sem);
    bus_adapter_stop(self);
    bus_adapter_abort_capture(self);
    if (dev && dev->rx_indicate)
    {
        dev->rx_indicate(dev, frame_size);
    }
}

/**
 * @brief RT-Thread device `init` hook (no-op; real initialisation is in `open`).
 *
 * @param dev is the RT-Thread device handle.
 *
 * @return Return RT_EOK.
 */
static rt_err_t ov2640_dev_init(rt_device_t dev)
{
    /* Device init is now handled in ov2640_open for proper re-initialization */
    return RT_EOK;
}

/**
 * @brief RT-Thread device `open` — power up sensor and initialise data bus.
 *
 * Starts XCLK, initialises the SCCB interface, resets the OV2640 and
 * binds the DVP bus adapter. The data bus is left in a ready-but-idle
 * state; capture is started by @c rt_device_read or @c OV2640_CMD_START_STREAM.
 *
 * @param dev   is the RT-Thread device handle.
 * @param oflag is unused (camera is always opened read-write).
 *
 * @return Return RT_EOK on success; @c -RT_ERROR on hardware failure.
 */
static rt_err_t ov2640_open(rt_device_t dev, rt_uint16_t oflag)
{
    ov2640_device_t *cam_dev = (ov2640_device_t *)dev->user_data;
    int ret;

    ov2640_stream_reset(&cam_dev->stream);

    /* Initialize frame ready semaphore */
    rt_sem_init(&cam_dev->frame_sem, "cam_frm", 0, RT_IPC_FLAG_FIFO);

    ret = sccb_init(SCCB_USE_IIC);
    if (ret != RT_EOK)
    {
        LOG_E("SCCB init failed: %d", ret);
        return -RT_ERROR;
    }

    /* Initialize OV2640 sensor */
    ret = ov2640_init(&cam_dev->sensor);
    if (ret != 0)
    {
        LOG_E("OV2640 init failed: %d", ret);
        sccb_deinit();
        return -RT_ERROR;
    }
    
    /* Default pixel format / framesize / quality are applied later via
     * OV2640_CMD_SET_* by the camera handle / application. */

    /* Locate the data bus adapter and prepare its configuration */
    cam_dev->data_bus = bus_adapter_find("dvp");
    if (cam_dev->data_bus == RT_NULL)
    {
        LOG_E("Data bus adapter 'dvp' not registered");
        sccb_deinit();
        return -RT_ERROR;
    }

    /* Mirror bus-agnostic fields; dvp_init() will self-configure hardware from Kconfig. */
    cam_dev->bus_snapshot.mode         = BUS_CAPTURE_MODE_JPEG;
    cam_dev->bus_snapshot.frame_buffer = NULL;
    cam_dev->bus_snapshot.buffer_size  = 0;

    /* Register high-level frame callback on the bus adapter */
    bus_adapter_set_frame_callback(cam_dev->data_bus, ov2640_frame_ready_callback, dev);

    ret = bus_adapter_init(cam_dev->data_bus);
    if (ret != 0)
    {
        LOG_E("Bus adapter init failed: %d", ret);
        sccb_deinit();
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief RT-Thread device `close` — abort capture and power down.
 *
 * Stops any ongoing DMA transfer, deinitialises the data bus (which also
 * stops XCLK via `dvp_deinit`) and SCCB, then destroys the frame semaphore.
 *
 * @param dev is the RT-Thread device handle.
 *
 * @return Return RT_EOK.
 */
static rt_err_t ov2640_close(rt_device_t dev)
{
    ov2640_device_t *cam_dev = (ov2640_device_t *)dev->user_data;

    ov2640_stream_reset(&cam_dev->stream);
    
    /* Stop any ongoing capture */
    bus_adapter_abort_capture(cam_dev->data_bus);
    
    /* Stop DVP hardware */
    bus_adapter_stop(cam_dev->data_bus);
    
    /* Deinitialize data bus */
    bus_adapter_deinit(cam_dev->data_bus);
    
    /* Deinitialize SCCB/I2C interface */
    sccb_deinit();

    /* Reset sensor bank state */
    ov2640_reset_bank_state();
    
    /* Destroy frame ready semaphore */
    rt_sem_detach(&cam_dev->frame_sem);
    
    LOG_I("Camera device closed");
    return RT_EOK;
}

#ifndef OV2640_CAMERA_READ_TIMEOUT_MS
#define OV2640_READ_TIMEOUT_MS    1000  /* 1 second timeout */
#else
#define OV2640_READ_TIMEOUT_MS    OV2640_CAMERA_READ_TIMEOUT_MS
#endif
#define OV2640_READ_TIMEOUT_TICKS    (OV2640_READ_TIMEOUT_MS * RT_TICK_PER_SECOND / 1000)

#ifndef OV2640_ENABLE_CAPTURE_TIMEOUT
#define OV2640_ENABLE_CAPTURE_TIMEOUT 1
#endif

#if OV2640_ENABLE_CAPTURE_TIMEOUT
static void ov2640_dump_timeout_state(ov2640_device_t *cam_dev,
                                      rt_size_t request_size,
                                      rt_err_t wait_result,
                                      rt_tick_t start_tick)
{
    rt_tick_t elapsed_ticks = rt_tick_get() - start_tick;
    unsigned long elapsed_ms = (unsigned long)elapsed_ticks * 1000UL / RT_TICK_PER_SECOND;

    LOG_W("Camera read timeout");
    LOG_W("  wait: result=%d elapsed=%lu ms req=%u",
        wait_result, elapsed_ms, (unsigned int)request_size);
    LOG_W("  sensor: pix=%d frame=%d quality=%u",
        (int)cam_dev->sensor.pixformat,
        (int)cam_dev->sensor.status.framesize,
        (unsigned int)cam_dev->sensor.status.quality);
    /* Delegate hardware-level diagnostics to the bus adapter. */
    bus_adapter_dump_state(cam_dev->data_bus);
}
#endif /* OV2640_ENABLE_CAPTURE_TIMEOUT */

/**
 * @brief RT-Thread device `read` — trigger a single-shot capture and block.
 *
 * Starts a DVP DMA transfer into @p buffer then waits on @c frame_sem.
 * If @c OV2640_ENABLE_CAPTURE_TIMEOUT is set the wait is bounded by
 * @c OV2640_READ_TIMEOUT_MS; a timeout dumps diagnostic state and returns 0.
 *
 * @param dev    is the RT-Thread device handle.
 * @param pos    is unused.
 * @param buffer is the destination frame buffer (must be DMA-accessible).
 * @param size   is the buffer size in bytes.
 *
 * @return Return the number of bytes captured; 0 on timeout or error.
 */
static rt_size_t ov2640_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    ov2640_device_t *cam_dev = (ov2640_device_t *)dev->user_data;
    rt_size_t frame_size = 0;
    rt_err_t result;
#if OV2640_ENABLE_CAPTURE_TIMEOUT
    rt_tick_t start_tick;
#endif

    /* Drain any stale semaphore tokens before starting capture */
    while (rt_sem_trytake(&cam_dev->frame_sem) == RT_EOK);
    
#if OV2640_ENABLE_CAPTURE_TIMEOUT
    start_tick = rt_tick_get();
#endif
    result = (rt_err_t)bus_adapter_start_capture(cam_dev->data_bus, buffer, size);
    if (result != RT_EOK)
    {
        LOG_E("Failed to start capture: %d", result);
        return 0;
    }

#if OV2640_ENABLE_CAPTURE_TIMEOUT
    /* Block until frame ready or timeout */
    result = rt_sem_take(&cam_dev->frame_sem, OV2640_READ_TIMEOUT_TICKS);
    
    if (result != RT_EOK)
    {
        ov2640_dump_timeout_state(cam_dev, size, result, start_tick);
        bus_adapter_abort_capture(cam_dev->data_bus);
        return 0;
    }
#else
    /* Block until frame ready indefinitely */
    result = rt_sem_take(&cam_dev->frame_sem, RT_WAITING_FOREVER);
#endif
         

    // // // Step 2: Stop GPTIM1 and reset its counter to 0
    // // HAL_GPT_Base_Stop(&handle->gptim);
    // // __HAL_GPT_SET_COUNTER(&handle->gptim, 0);

    /* Note: pingpong buffer auto-clear on next DMA write — no explicit memset needed */
    frame_size = bus_adapter_get_frame_size(cam_dev->data_bus);
    
    return frame_size;
}

/**
 * @brief RT-Thread device `write` — not supported by camera devices.
 *
 * @param dev    is the RT-Thread device handle.
 * @param pos    is unused.
 * @param buffer is unused.
 * @param size   is unused.
 *
 * @return Return 0 always.
 */
static rt_size_t ov2640_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    (void)dev; (void)pos; (void)buffer; (void)size;
    return 0;
}

/**
 * @brief Map a sensor-level @c pixformat_t to a generic @c bus_capture_mode_t.
 *
 * Centralizes the pixel-format → bus-capture-mode translation so that the
 * mapping lives in exactly one place. Returns @c RT_TRUE on success and
 * writes the resolved mode through @p out_mode; returns @c RT_FALSE for
 * formats that have no bus equivalent.
 */
static rt_bool_t ov2640_pixformat_to_bus_mode(pixformat_t format, bus_capture_mode_t *out_mode)
{
    switch (format)
    {
        case PIXFORMAT_JPEG:   *out_mode = BUS_CAPTURE_MODE_JPEG;   return RT_TRUE;
        case PIXFORMAT_RGB565: *out_mode = BUS_CAPTURE_MODE_RGB565; return RT_TRUE;
        case PIXFORMAT_YUV422: *out_mode = BUS_CAPTURE_MODE_YUV422; return RT_TRUE;
        case PIXFORMAT_RAW8:   *out_mode = BUS_CAPTURE_MODE_RAW;    return RT_TRUE;
        default:                                                    return RT_FALSE;
    }
}

/**
 *
 * Routes all @c OV2640_CMD_* values to the appropriate vtable function or
 * bus adapter op. Unknown commands return @c -RT_EINVAL.
 *
 * @param dev  is the RT-Thread device handle.
 * @param cmd  is one of the @c OV2640_CMD_* constants (see ov2640.h).
 * @param args is the command argument; type varies per command.
 *
 * @return Return RT_EOK on success; @c -RT_EINVAL for unknown commands;
 *         @c -RT_ERROR if the underlying operation fails.
 */
static rt_err_t ov2640_control(rt_device_t dev, int cmd, void *args)
{
    ov2640_device_t *cam_dev = (ov2640_device_t *)dev->user_data;
    int ret;
    
    if (cam_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    switch (cmd)
    {
        case OV2640_CMD_SET_PIXFORMAT:
        {
            pixformat_t format = (pixformat_t)(rt_ubase_t)args;
            ret = ov2640_set_pixformat(&cam_dev->sensor, format);
            if (ret != 0)
            {
                return -RT_ERROR;
            }

            /* Update bus capture mode accordingly and restart hardware */
            bus_capture_mode_t new_mode;
            if (!ov2640_pixformat_to_bus_mode(format, &new_mode))
            {
                return -RT_EINVAL;
            }

            /* If mode changed, restart the bus through the generic adapter
             * interface — no direct access to DVP-private state. */
            if (cam_dev->bus_snapshot.mode != new_mode)
            {
                bus_adapter_stop(cam_dev->data_bus);
                if (bus_adapter_set_mode(cam_dev->data_bus, new_mode) != BUS_OK)
                {
                    LOG_E("Failed to set bus mode");
                    return -RT_ERROR;
                }
                cam_dev->bus_snapshot.mode = new_mode;
                ret = bus_adapter_start(cam_dev->data_bus);
                if (ret != 0)
                {
                    LOG_E("Failed to restart bus after mode change");
                    return -RT_ERROR;
                }
            }

            return RT_EOK;
        }
        
        case OV2640_CMD_SET_FRAMESIZE:
        {
            if((framesize_t)(rt_ubase_t)args >= FRAMESIZE_INVALID)
            {
                return -RT_EINVAL;
            }
            framesize_t framesize = (framesize_t)(rt_ubase_t)args;
            ret = ov2640_set_framesize(&cam_dev->sensor, framesize);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_BRIGHTNESS:
        {
            int level = (int)(rt_base_t)args;
            ret = ov2640_set_brightness(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_CONTRAST:
        {
            int level = (int)(rt_base_t)args;
            ret = ov2640_set_contrast(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_SATURATION:
        {
            int level = (int)(rt_base_t)args;
            ret = ov2640_set_saturation(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_QUALITY:
        {
            int quality = (int)(rt_base_t)args;
            ret = ov2640_set_quality(&cam_dev->sensor, quality);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_HMIRROR:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_hmirror(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_VFLIP:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_vflip(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_COLORBAR:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_colorbar(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_WHITEBAL:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_whitebal(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_GAIN_CTRL:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_gain_ctrl(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_EXPOSURE_CTRL:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_exposure_ctrl(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_AEC2:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_aec2(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_AWB_GAIN:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_awb_gain(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_AGC_GAIN:
        {
            int gain = (int)(rt_base_t)args;
            ret = ov2640_set_agc_gain(&cam_dev->sensor, gain);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_AEC_VALUE:
        {
            int value = (int)(rt_base_t)args;
            ret = ov2640_set_aec_value(&cam_dev->sensor, value);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_SPECIAL_EFFECT:
        {
            int effect = (int)(rt_base_t)args;
            ret = ov2640_set_special_effect(&cam_dev->sensor, effect);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_WB_MODE:
        {
            int mode = (int)(rt_base_t)args;
            ret = ov2640_set_wb_mode(&cam_dev->sensor, mode);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_AE_LEVEL:
        {
            int level = (int)(rt_base_t)args;
            ret = ov2640_set_ae_level(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_DCW:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_dcw(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_BPC:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_bpc(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_WPC:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_wpc(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_RAW_GMA:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_raw_gma(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_LENC:
        {
            int enable = (int)(rt_base_t)args;
            ret = ov2640_set_lenc(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_GAINCEILING:
        {
            gainceiling_t gainceiling = (gainceiling_t)(rt_ubase_t)args;
            ret = ov2640_set_gainceiling(&cam_dev->sensor, gainceiling);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_SHARPNESS:
        {
            int level = (int)(rt_base_t)args;
            ret = ov2640_set_sharpness(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_SET_DENOISE:
        {
            int level = (int)(rt_base_t)args;
            ret = ov2640_set_denoise(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case OV2640_CMD_START_CAPTURE:
        {
            (void)args;
            ov2640_stream_reset(&cam_dev->stream);
            bus_adapter_start_capture(cam_dev->data_bus, RT_NULL, 0);
            return RT_EOK;
        }
        
        case OV2640_CMD_STOP_CAPTURE:
        {
            (void)args;
            ov2640_stream_reset(&cam_dev->stream);
            bus_adapter_abort_capture(cam_dev->data_bus);
            return RT_EOK;
        }
        
        case OV2640_CMD_GET_FRAME_SIZE:
        {
            uint32_t *size_ptr = (uint32_t *)args;
            if (size_ptr != RT_NULL)
            {
                *size_ptr = bus_adapter_get_frame_size(cam_dev->data_bus);
                return RT_EOK;
            }
            return -RT_EINVAL;
        }
        
        case OV2640_CMD_SET_FRAME_BUFFER:
        {
            uint8_t *buffer = (uint8_t *)args;
            if (buffer != RT_NULL)
            {
                cam_dev->bus_snapshot.frame_buffer = buffer;
                bus_adapter_update_buffer(cam_dev->data_bus, buffer, cam_dev->bus_snapshot.buffer_size);
                LOG_I("Bus frame buffer set to 0x%08X", (uint32_t)buffer);
                return RT_EOK;
            }
            return -RT_EINVAL;
        }
        
        case OV2640_CMD_SET_FRAME_BUFFER_SIZE:
        {
            uint32_t size = (uint32_t)(rt_ubase_t)args;
            if (size > 0)
            {
                cam_dev->bus_snapshot.buffer_size = size;
                bus_adapter_update_buffer(cam_dev->data_bus, cam_dev->bus_snapshot.frame_buffer, size);
                LOG_I("Bus frame buffer size set to %d bytes", size);
                return RT_EOK;
            }
            return -RT_EINVAL;
        }
        
        case OV2640_CMD_SET_PINGPONG_SIZE:
        {
            uint32_t size = (uint32_t)(rt_ubase_t)args;
            if (size > 0)
            {
                ret = bus_adapter_set_pingpong_size(cam_dev->data_bus, size);
                return (ret == 0) ? RT_EOK : -RT_ERROR;
            }
            return -RT_EINVAL;
        }

        case OV2640_CMD_START_STREAM:
        {
            camera_stream_start_args_t *stream_args = (camera_stream_start_args_t *)args;

            if (stream_args == RT_NULL ||
                stream_args->buffers[0] == RT_NULL ||
                stream_args->buffers[1] == RT_NULL ||
                stream_args->buffer_size == 0)
            {
                return -RT_EINVAL;
            }

            /*
             * Populate buffers and indices first; only publish frame_callback
             * (the "streaming armed" flag) after everything else is in place,
             * so an early ISR can never see a torn state. The callback store
             * is wrapped in a critical section to pair with the ISR's read
             * and STOP_STREAM's clear.
             */
            cam_dev->stream.buffers[0] = stream_args->buffers[0];
            cam_dev->stream.buffers[1] = stream_args->buffers[1];
            cam_dev->stream.buffer_size = stream_args->buffer_size;
            cam_dev->stream.active_buffer_index = 0;
            cam_dev->stream.sequence = 0;

            {
                rt_base_t level = rt_hw_interrupt_disable();
                cam_dev->stream.callback_context = stream_args->callback_context;
                cam_dev->stream.frame_callback   = stream_args->frame_callback;
                rt_hw_interrupt_enable(level);
            }

            ret = bus_adapter_start_capture(cam_dev->data_bus,
                                            cam_dev->stream.buffers[cam_dev->stream.active_buffer_index],
                                            cam_dev->stream.buffer_size);
            if (ret != RT_EOK)
            {
                rt_base_t level = rt_hw_interrupt_disable();
                ov2640_stream_reset(&cam_dev->stream);
                rt_hw_interrupt_enable(level);
                return -RT_ERROR;
            }

            return RT_EOK;
        }

        case OV2640_CMD_STOP_STREAM:
        {
            /*
             * Order matters:
             *  1) Abort the bus first so the DMA stops generating new
             *     frame-ready events.
             *  2) Clear stream state (including frame_callback) inside an
             *     interrupt lock. Any ISR already in flight observes either
             *     the old callback (and dispatches one final frame) or NULL
             *     (and falls through to the single-shot path which is a
             *     no-op when no thread is waiting on frame_sem).
             */
            bus_adapter_abort_capture(cam_dev->data_bus);
            {
                rt_base_t level = rt_hw_interrupt_disable();
                ov2640_stream_reset(&cam_dev->stream);
                rt_hw_interrupt_enable(level);
            }
            return RT_EOK;
        }
        
        default:
            return -RT_EINVAL;
    }
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops ov2640_ops =
{
    ov2640_dev_init,
    ov2640_open,
    ov2640_close,
    ov2640_read,
    ov2640_write,
    ov2640_control
};
#endif

/*
 *******************************************************************************
 * Device registration
 *******************************************************************************
 */

/**
 * @brief Register the OV2640 as an RT-Thread device named "ov2640".
 *
 * Allocates @c rt_device_t and @c ov2640_device_t on the heap, hooks the
 * RT-Thread device ops and calls @c rt_device_register. Automatically
 * invoked by @c INIT_DEVICE_EXPORT.
 *
 * @return Return RT_EOK on success; @c -RT_ENOMEM or device error on failure.
 */
/**
 * @brief Register the OV2640 as an RT-Thread device under @p name.
 *
 * Allocates @c rt_device_t and @c ov2640_device_t on the heap, hooks the
 * RT-Thread device ops and calls @c rt_device_register. The driver is
 * single-instance — see @ref s_active_dev for details — and calling this
 * more than once will overwrite the singleton pointer.
 *
 * @param name is the RT-Thread device name to register under; if @c NULL
 *             the default @ref OV2640_DEVICE_NAME is used.
 *
 * @return Return RT_EOK on success; @c -RT_ENOMEM or device error on failure.
 */
int ov2640_device_register(const char *name)
{
    rt_device_t dev;
    int ret;
    if (name == RT_NULL)
    {
        name = OV2640_DEVICE_NAME;
    }

    /* Allocate OV2640 device structure */
    dev = rt_malloc(sizeof(struct rt_device));
    if (dev == RT_NULL)
    {
        LOG_E("Failed to allocate ov2640 device memory");
        return -RT_ENOMEM;
    }
    
    rt_memset(dev, 0, sizeof(struct rt_device));
    
    /* Initialize device structure */
    dev->type = RT_Device_Class_Miscellaneous;
    
#ifdef RT_USING_DEVICE_OPS
    dev->ops = &ov2640_ops;
#else
    dev->init = ov2640_dev_init;
    dev->open = ov2640_open;
    dev->close = ov2640_close;
    dev->read = ov2640_read;
    dev->write = ov2640_write;
    dev->control = ov2640_control;
#endif
    dev->user_data = rt_malloc(sizeof(ov2640_device_t));
    if (dev->user_data == RT_NULL)
    {
        LOG_E("Failed to allocate ov2640 device user data memory");
        rt_free(dev);
        return -RT_ENOMEM;
    }
    rt_memset(dev->user_data, 0, sizeof(ov2640_device_t));

    /* Initialize per-instance state moved out of file scope (#6/#16). */
    ov2640_device_t *cam_dev = (ov2640_device_t *)dev->user_data;
    cam_dev->current_bank = (rt_uint8_t)BANK_MAX;
    ov2640_mutex_init(cam_dev);

    /* Publish the singleton pointer used by SCCB setter helpers. */
    s_active_dev = cam_dev;
    
    /* Register device */
    ret = rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to register ov2640 device: %d", ret);
        s_active_dev = RT_NULL;
        rt_free(dev->user_data);
        rt_free(dev);
        return ret;
    }
    
    LOG_I("ov2640 device '%s' registered successfully", name);
    return RT_EOK;
}

/**
 * @brief Register the OV2640 device using @ref OV2640_DEVICE_NAME.
 *
 * Wrapper used by @c INIT_DEVICE_EXPORT, which only accepts a parameter-less
 * function. Equivalent to @c ov2640_device_register(NULL).
 */
int ov2640_device_register_default(void)
{
    return ov2640_device_register(RT_NULL);
}

/**
 * @brief Unregister and free the OV2640 RT-Thread device.
 *
 * Finds the "ov2640" device, closes it if still open, unregisters it
 * and frees all heap allocations.
 *
 * @return Return RT_EOK on success; @c -RT_ENOSYS if device not found.
 */
int ov2640_device_unregister(void)
{
    rt_device_t dev;
    const char *name = OV2640_DEVICE_NAME;
    
    dev = rt_device_find(name);
    if (dev == RT_NULL)
    {
        LOG_W("ov2640 device '%s' not found", name);
        return -RT_ENOSYS;
    }
    
    /* Close device if still open */
    if (dev->ref_count > 0)
    {
        rt_device_close(dev);
    }
    
    /* Unregister device */
    rt_err_t ret = rt_device_unregister(dev);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to unregister ov2640 device: %d", ret);
        return ret;
    }
    
    /* Free user data */
    if (dev->user_data != RT_NULL)
    {
        ov2640_device_t *cam_dev = (ov2640_device_t *)dev->user_data;
        if (cam_dev == s_active_dev)
        {
            s_active_dev = RT_NULL;
        }
        if (cam_dev->sccb_lock_initialized)
        {
            rt_mutex_detach(&cam_dev->sccb_lock);
            cam_dev->sccb_lock_initialized = RT_FALSE;
        }
        rt_free(dev->user_data);
        dev->user_data = RT_NULL;
    }
    
    /* Free device structure */
    rt_free(dev);
    
    LOG_I("ov2640 device '%s' unregistered successfully", name);
    return RT_EOK;
}

INIT_DEVICE_EXPORT(ov2640_device_register_default);