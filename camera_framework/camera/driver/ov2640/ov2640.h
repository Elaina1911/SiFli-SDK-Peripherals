/**
 * @file    ov2640.h
 * @brief   OV2640 camera sensor RT-Thread device driver interface
 *
 * This header provides the OV2640 sensor data structures, camera
 * status definitions, RT-Thread device interface, and control
 * command definitions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef OV2640_H_
#define OV2640_H_

#include <stdint.h>
#include <stdbool.h>
#include "sccb.h"
#include "rtthread.h"
#include "rtconfig.h"
#include "../../handle/camera_handle.h"

#define OV2640_ADDR 0x30  /* 7-bit I2C/SCCB slave address */

/** RT-Thread device name used to register & locate the OV2640 instance.
 *  Defined to equal CAMERA_DEFAULT_DEVICE_NAME (from camera_handle.h, already
 *  included above) so the two stay in sync automatically. */
#define OV2640_DEVICE_NAME CAMERA_DEFAULT_DEVICE_NAME

/*
 *******************************************************************************
 * Data types
 *******************************************************************************
 */

/**
 * @brief Snapshot of all configurable camera sensor parameters.
 *
 * Updated by the driver each time a parameter is successfully written to the
 * sensor; callers can read this struct to query the current sensor state
 * without issuing I2C transactions.
 */
typedef struct {
    /* Resolution & format */
    framesize_t framesize;      /* 0 – FRAMESIZE_INVALID-1 */
    bool        scale;
    bool        binning;

    /* Image quality */
    uint8_t quality;            /* JPEG quality scale 0–63 (lower = better) */
    int8_t  brightness;         /* -2 – +2, 0 = default */
    int8_t  contrast;           /* -2 – +2, 0 = default */
    int8_t  saturation;         /* -2 – +2, 0 = default */
    int8_t  sharpness;          /* -2 – +2 (not supported on OV2640, always 0) */
    uint8_t denoise;            /* denoise level (not supported on OV2640, always 0) */
    uint8_t special_effect;     /* 0 = Normal, 1–6 = various effects */

    /* White balance */
    uint8_t wb_mode;            /* 0–4: auto / sunny / cloudy / office / home */
    uint8_t awb;                /* 1 = AWB enabled */
    uint8_t awb_gain;           /* 1 = AWB gain enabled */

    /* Exposure & gain */
    uint8_t  aec;               /* 1 = AEC enabled */
    uint8_t  aec2;              /* 1 = AEC2 (enhanced) enabled */
    int8_t   ae_level;          /* -2 – +2 AE compensation */
    uint16_t aec_value;         /* manual AEC value 0–1200 */
    uint8_t  agc;               /* 1 = AGC enabled */
    uint8_t  agc_gain;          /* manual AGC gain 0–30 */
    uint8_t  gainceiling;       /* AGC gain ceiling 0–6 (2x–128x) */

    /* Image processing */
    uint8_t bpc;                /* 1 = bad pixel correction enabled */
    uint8_t wpc;                /* 1 = white pixel correction enabled */
    uint8_t raw_gma;            /* 1 = RAW gamma enabled */
    uint8_t lenc;               /* 1 = lens correction enabled */
    uint8_t hmirror;            /* 1 = horizontal mirror enabled */
    uint8_t vflip;              /* 1 = vertical flip enabled */
    uint8_t dcw;                /* 1 = downsize crop & window enabled */
    uint8_t colorbar;           /* 1 = color bar test pattern enabled */
} camera_status_t;

/**
 * @brief AGC gain ceiling values.
 *
 * Passed to `set_gainceiling` to cap the automatic gain loop.
 */
typedef enum {
    GAINCEILING_2X,
    GAINCEILING_4X,
    GAINCEILING_8X,
    GAINCEILING_16X,
    GAINCEILING_32X,
    GAINCEILING_64X,
    GAINCEILING_128X,
} gainceiling_t;

/**
 * @brief OV2640 sensor handle.
 *
 * Holds sensor identity (from ID registers) and a mirror of the last-applied
 * configuration. Sensor operations are exposed as plain functions in
 * `ov2640.c` (see `ov2640_set_pixformat`, `ov2640_set_framesize`, …) — there
 * is no runtime vtable because the RT-Thread device layer is the only caller
 * and the sensor model is fixed at build time.
 */
typedef struct ov2640
{
    /* Sensor identification registers (read from HW during init) */
    uint8_t  MIDH;              /* Manufacturer ID high byte */
    uint8_t  MIDL;              /* Manufacturer ID low byte */
    uint16_t PID;               /* Product ID */
    uint8_t  VER;               /* Product version */

    /* Current format and status mirror */
    pixformat_t    pixformat;
    camera_status_t status;
} ov2640_t;

/**
 * @brief Initialize an OV2640 sensor handle.
 *
 * Populates all vtable function pointers and reads identification
 * registers from the sensor via SCCB. Must be called once before any
 * vtable operation.
 *
 * @param dev is a pointer to the sensor handle to initialize.
 *
 * @return Return 0 on success. Otherwise a negative error code is returned.
 */
int ov2640_init(ov2640_t *dev);

/*
 *******************************************************************************
 * RT-Thread device layer
 *******************************************************************************
 */

#include "data_bus_adapter.h"

/**
 * @brief Bus-agnostic runtime state mirrored from the active configuration.
 *
 * Replaces the former dvp_config_t snapshot so the sensor driver no longer
 * depends on a specific bus type. Only the fields actually queried at runtime
 * by the device-control handlers are kept here; all hardware-specific
 * parameters are passed to bus_adapter_init() as a local config struct and
 * are not retained in this instance.
 */
typedef struct {
    bus_capture_mode_t mode;         /**< current capture mode (bus-agnostic) */
    uint8_t           *frame_buffer; /**< current user frame buffer, may be NULL */
    uint32_t           buffer_size;  /**< size of frame_buffer in bytes */
} ov2640_bus_snapshot_t;

/**
 * @brief Streaming state for continuous multi-buffer capture.
 *
 * Maintained by the RT-Thread device layer; not accessed directly by sensor
 * logic. Buffers are ping-pong rotated on each frame-ready callback.
 *
 * The presence of a non-NULL @c frame_callback is the single authoritative
 * indicator that streaming is armed: the frame-ready ISR dispatches to the
 * streaming branch iff @c frame_callback is set, otherwise it treats the
 * frame as a single-shot completion. STOP_STREAM clears the callback under
 * an interrupt lock (after aborting the bus) so no in-flight ISR can see
 * a stale callback pointer.
 */
typedef struct
{
    uint8_t    *buffers[2];           /* ping-pong buffer pair */
    rt_size_t   buffer_size;          /* size of each buffer in bytes */
    rt_uint8_t  active_buffer_index;  /* index of the buffer currently being filled */
    rt_uint32_t sequence;             /* monotonically increasing frame counter */
    camera_stream_frame_callback_t frame_callback;  /* user callback per frame; NULL = single-shot mode */
    void       *callback_context;     /* opaque pointer forwarded to callback */
} ov2640_stream_state_t;

/**
 * @brief OV2640 RT-Thread device instance.
 *
 * One global instance is registered as RT-Thread device "ov2640" (see
 * @ref OV2640_DEVICE_NAME). Upper layers access it through the standard
 * @c rt_device_* API.
 *
 * @note This driver is currently single-instance: the SCCB control bus
 *       (sccb.c) and DVP data bus (dvp.c) both keep file-scope state, so
 *       only one @ref ov2640_device_t may exist at a time. The mutex and
 *       register-bank cache below live in the instance rather than file
 *       scope to make a future multi-instance refactor easier — the
 *       sensor setters reach the active instance through a singleton
 *       pointer cached at @ref ov2640_device_register time.
 */
typedef struct {
    ov2640_t              sensor;       /* sensor vtable and status */
    bus_adapter_t        *data_bus;     /* data bus backend (DVP by default) */
    ov2640_bus_snapshot_t bus_snapshot; /* bus-agnostic runtime state (mode, buffers) */
    struct rt_semaphore frame_sem; /* posted by frame callback, consumed by rt_device_read */
    ov2640_stream_state_t stream;  /* streaming state (inactive for single-shot capture) */
    struct rt_mutex sccb_lock;   /* serializes SCCB transactions for sensor setters */
    rt_bool_t       sccb_lock_initialized; /* RT_TRUE once @ref sccb_lock has been initialized */
    rt_uint8_t      current_bank; /* cached BANK_SEL value; ov2640_bank_t cast at use site */
} ov2640_device_t;

/*
 *******************************************************************************
 * Control commands  (passed as `cmd` to rt_device_control)
 *******************************************************************************
 *
 * Usage:
 *   rt_device_control(camera, OV2640_CMD_*, (void *)(uintptr_t)value);
 */

/* --- Image format & resolution ------------------------------------------ */
#define OV2640_CMD_SET_PIXFORMAT       0x01  /* pixformat_t  */
#define OV2640_CMD_SET_FRAMESIZE       0x02  /* framesize_t  */

/* --- Image quality ------------------------------------------------------- */
#define OV2640_CMD_SET_BRIGHTNESS      0x03  /* int -2…+2   */
#define OV2640_CMD_SET_CONTRAST        0x04  /* int -2…+2   */
#define OV2640_CMD_SET_SATURATION      0x05  /* int -2…+2   */
#define OV2640_CMD_SET_QUALITY         0x06  /* int  0…63   */
#define OV2640_CMD_SET_SHARPNESS       0x1D  /* int -2…+2 (no-op on OV2640) */
#define OV2640_CMD_SET_DENOISE         0x1E  /* int level  (no-op on OV2640) */
#define OV2640_CMD_SET_GAINCEILING     0x1C  /* gainceiling_t */

/* --- White balance ------------------------------------------------------- */
#define OV2640_CMD_SET_WHITEBAL        0x0A  /* int 0=off 1=on  */
#define OV2640_CMD_SET_AWB_GAIN        0x0E  /* int 0=off 1=on  */
#define OV2640_CMD_SET_WB_MODE         0x15  /* int 0=auto … 4=home */

/* --- Exposure & gain ----------------------------------------------------- */
#define OV2640_CMD_SET_GAIN_CTRL       0x0B  /* int 0=off 1=on   */
#define OV2640_CMD_SET_EXPOSURE_CTRL   0x0C  /* int 0=off 1=on   */
#define OV2640_CMD_SET_AEC2            0x0D  /* int 0=off 1=on   */
#define OV2640_CMD_SET_AGC_GAIN        0x0F  /* int 0…30         */
#define OV2640_CMD_SET_AEC_VALUE       0x13  /* int 0…1200       */
#define OV2640_CMD_SET_AE_LEVEL        0x16  /* int -2…+2        */

/* --- Image processing ---------------------------------------------------- */
#define OV2640_CMD_SET_HMIRROR         0x07  /* int 0=off 1=on   */
#define OV2640_CMD_SET_VFLIP           0x08  /* int 0=off 1=on   */
#define OV2640_CMD_SET_COLORBAR        0x09  /* int 0=off 1=on   */
#define OV2640_CMD_SET_SPECIAL_EFFECT  0x14  /* int 0=normal … 6 */
#define OV2640_CMD_SET_DCW             0x17  /* int 0=off 1=on   */
#define OV2640_CMD_SET_BPC             0x18  /* int 0=off 1=on   */
#define OV2640_CMD_SET_WPC             0x19  /* int 0=off 1=on   */
#define OV2640_CMD_SET_RAW_GMA         0x1A  /* int 0=off 1=on   */
#define OV2640_CMD_SET_LENC            0x1B  /* int 0=off 1=on   */

/* --- Data bus / DMA buffer ----------------------------------------------- */
#define OV2640_CMD_SET_FRAME_BUFFER      0x1F  /* void* frame buffer pointer  */
#define OV2640_CMD_SET_FRAME_BUFFER_SIZE 0x20  /* uint32_t buffer size bytes  */
#define OV2640_CMD_SET_PINGPONG_SIZE     0x21  /* uint32_t ping-pong size bytes */

/* --- Capture control ----------------------------------------------------- */
#define OV2640_CMD_START_CAPTURE       0x10  /* void* buffer (NULL = keep current) */
#define OV2640_CMD_STOP_CAPTURE        0x11  /* no argument */
#define OV2640_CMD_GET_FRAME_SIZE      0x12  /* uint32_t* out: captured bytes */
#define OV2640_CMD_START_STREAM        0x22  /* no argument */
#define OV2640_CMD_STOP_STREAM         0x23  /* no argument */

/*
 *******************************************************************************
 * Public API
 *******************************************************************************
 */

/**
 * @brief Register the OV2640 RT-Thread device under @p name.
 *
 * Creates and registers a device named @p name (or @ref OV2640_DEVICE_NAME
 * if @p name is @c NULL). The default-name variant
 * @ref ov2640_device_register_default is automatically invoked by
 * @c INIT_DEVICE_EXPORT.
 *
 * @param name is the device name to register under; @c NULL to use the
 *             default @ref OV2640_DEVICE_NAME.
 *
 * @return Return RT_EOK on success; negative error code on failure.
 *
 * @note The driver is single-instance — calling this more than once
 *       (even with different @p name values) will overwrite the
 *       singleton pointer used by the SCCB setters.
 */
int ov2640_device_register(const char *name);

/**
 * @brief Register the OV2640 device using @ref OV2640_DEVICE_NAME.
 *
 * Convenience wrapper installed by @c INIT_DEVICE_EXPORT; equivalent to
 * @c ov2640_device_register(NULL).
 */
int ov2640_device_register_default(void);

/**
 * @brief Unregister the OV2640 RT-Thread device.
 *
 * Closes the data bus, stops hardware and removes the device from
 * RT-Thread's device tree.
 *
 * @return Return RT_EOK on success; negative error code on failure.
 */
int ov2640_device_unregister(void);

/**
 * @brief Set the receive-indication callback for asynchronous frame notification.
 *
 * The callback is invoked each time a frame is ready, from ISR context.
 * It should only perform minimal work such as posting a semaphore.
 *
 * @param dev    is the RT-Thread device handle ("ov2640").
 * @param rx_ind is the callback function; pass NULL to clear.
 *
 * @return Return RT_EOK on success.
 */
rt_err_t ov2640_set_rx_indicate(rt_device_t dev, rt_err_t (*rx_ind)(rt_device_t dev, rt_size_t size));

/**
 * @brief Get the OV2640 RT-Thread device ops table.
 *
 * Returns the static `camera_device_ops_t` that wraps the OV2640 device
 * interface. Used by `camera_handle` to reach the device without coupling
 * to RT-Thread device symbols directly.
 *
 * @return Return a pointer to the static ops table.
 */
const camera_device_ops_t *ov2640_get_device_ops(void);

#endif /* OV2640_H_ */