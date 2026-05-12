/******************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file camera_handle.h
 * 
 * @par dependencies 
 * - <Driver_Layer>.h
 * - stdbool.h
 * - stdint.h
 * 
 * @author SiFli 思澈科技
 * 
 * @brief Provide the HAL APIs of camera handler 
 * and corresponding operations.
 * 
 * Processing flow:
 * 
 * Call directly.
 * 
 * @version V1.0 2026-4-3
 *
 * @note 1 tab == 4 spaces!
 * 
 *****************************************************************************/
 
#ifndef __CAMERA_HANDLE_H
#define __CAMERA_HANDLE_H

//******************************** Includes *********************************//
#include <stdint.h>
#include <rtdevice.h>
//******************************** Includes *********************************//

/**
 * Default RT-Thread device name used when the caller does not provide one via
 * `camera_handler_input_arg_t::device_name`. Keep in sync with the name passed
 * to `rt_device_register()` in the underlying sensor driver.
 */
#define CAMERA_DEFAULT_DEVICE_NAME "ov2640"

//******************************** Defines **********************************//
#ifdef __cplusplus
extern "C" {
#endif
//******************************** Defines **********************************//

//******************************** Typedefs *********************************//

typedef enum
{
  PIXFORMAT_RGB565,
  PIXFORMAT_YUV422,
  PIXFORMAT_JPEG,
  PIXFORMAT_RAW8,
  PIXFORMAT_INVALID
} pixformat_t;

typedef enum
{
  FRAMESIZE_96X96,
  FRAMESIZE_QQVGA,
  FRAMESIZE_128X128,
  FRAMESIZE_QCIF,
  FRAMESIZE_HQVGA,
  FRAMESIZE_240X240,
  FRAMESIZE_QVGA,
  FRAMESIZE_320X320,
  FRAMESIZE_CIF,
  FRAMESIZE_HVGA,
  FRAMESIZE_VGA,
  FRAMESIZE_SVGA,
  FRAMESIZE_XGA,
  FRAMESIZE_HD,
  FRAMESIZE_SXGA,
  FRAMESIZE_UXGA,
  FRAMESIZE_INVALID
} framesize_t;

typedef struct
{
  void *buffer;
  rt_size_t buffer_size;
  rt_size_t frame_size;
  rt_uint32_t sequence;
  rt_uint8_t buffer_index;
} camera_stream_frame_t;

typedef void (*camera_stream_frame_callback_t)(void *context,
                         const camera_stream_frame_t *frame);

/*
 * Argument struct for the sensor's start-stream control command. This is the
 * inter-layer contract between camera_handle.c and the underlying RT-Thread
 * camera device driver (e.g. ov2640) — the handle layer fills it in and the
 * driver consumes it inside its OV2640_CMD_START_STREAM handler.
 */
typedef struct
{
  void *buffers[2];
  rt_size_t buffer_size;
  camera_stream_frame_callback_t frame_callback;
  void *callback_context;
} camera_stream_start_args_t;

typedef struct
{
  int set_pixformat;
  int set_framesize;
  int set_quality;
  int start_stream;
  int stop_stream;
} camera_device_command_set_t;

/*
 * Camera device ops structure.
 *
 * The handle layer talks to RT-Thread device API directly via
 * rt_device_open/close/read/control; only the command_set indirection is
 * needed so that the same handle code can drive sensors that use different
 * integer command IDs.
 */
typedef struct
{
  camera_device_command_set_t command_set;
} camera_device_ops_t;

/*   Emulation of return case        */
typedef enum
{
  CAMERA_OK                = 0,     /* Operation completed successfully.  */
  CAMERA_ERROR             = 1,     /* Run-time error without case matched*/
  CAMERA_ERRORTIMEOUT      = 2,     /* Operation failed with timeout      */
  CAMERA_ERRORRESOURCE     = 3,     /* Resource not available.            */
  CAMERA_ERRORPARAMETER    = 4,     /* Parameter error.                   */
  CAMERA_ERRORNOMEMORY     = 5,     /* Out of memory.                     */
  CAMERA_ERRORISR          = 6,     /* Not allowed in ISR context         */
  CAMERA_RESERVED  = 0x7FFFFFFF     /* Reserved  May check the caller     */
} camera_handle_status_t;

typedef struct
{
  pixformat_t pixformat;
  framesize_t framesize;
  uint8_t quality;
} camera_capture_config_t;

typedef struct
{
  void *buffer;
  rt_size_t buffer_size;
  rt_size_t frame_size;
} camera_capture_request_t;

typedef struct
{
  void *buffers[2];
  rt_size_t buffer_size;
} camera_stream_config_t;

typedef struct
{
  struct rt_semaphore frame_sem;
  rt_bool_t sem_initialized;
  rt_bool_t is_active;
  camera_stream_frame_t ready_frames[2];
  rt_uint8_t head;
  rt_uint8_t tail;
  rt_uint8_t count;
  /* Total frames overwritten because the ready queue was full when a new
   * frame arrived. Useful for diagnosing "stuttering" in the application
   * consumer; reset to 0 on each camera_start_stream(). */
  rt_uint32_t dropped_count;
} camera_stream_runtime_t;
//******************************** Typedefs *********************************//


//**************************** Interface Structs ****************************//

typedef struct
{
  const char *device_name;
  const camera_device_ops_t *device_ops;
} camera_handler_all_input_arg_t;

typedef struct
{
  rt_device_t camera_device;
  camera_capture_config_t active_config;
  camera_stream_runtime_t stream;
  const char *device_name;
  const camera_device_ops_t *device_ops;
  rt_bool_t is_open;
} camera_handler_instance_t;

//**************************** Interface Structs ****************************//


//******************************** APIs *************************************//

// /**
//  * @brief Camera handler thread that processes camera events and frames.
//  *
//  * This thread processes camera-related events (capture requests, config
//  * changes, notifications). It polls or waits on the internal event queue,
//  * dispatches work to the underlying camera driver, and forwards completed
//  * frames or status updates to the registered callbacks or higher layers.
//  */
// void camera_handler_thread(void *argument);

/**
 * @brief Initialize a camera handler instance.
 *
 * Prepare a camera handler instance for operation. This sets up the required
 * interfaces (driver bindings, event queue, resources) and records initial
 * state needed by the handler.
 *
 * @param handler_instance A pointer to the camera handler instance to be initialized.
 * @param input_arg        A pointer to the input arguments containing driver and
 *                         platform-specific interfaces required by the handler.
 *
 * @return `CAMERA_OK` on success, otherwise an error code from `camera_handle_status_t`.
 */
camera_handle_status_t camera_handler_instance_init(
                        camera_handler_instance_t *instance,
                        camera_handler_all_input_arg_t *input_arg);


/**
 * @brief Capture a single frame using the previously configured settings.
 *
 * Blocking single-shot capture: the caller provides the destination buffer
 * via @p request, and on success @p request->frame_size is filled in with
 * the byte count of the captured image (e.g. JPEG payload length).
 *
 * @param[in]     instance The camera handler instance.
 * @param[in,out] request  Destination buffer + size; @c frame_size is filled
 *                         in on success.
 *
 * @return `CAMERA_OK` on success, otherwise a `camera_handle_status_t` error.
 */
camera_handle_status_t camera_capture_single(camera_handler_instance_t *instance,
                                             camera_capture_request_t *request);

/**
 * @brief Start continuous double-buffered frame streaming.
 *
 * Allocates the internal frame semaphore (first call only), resets the ready
 * queue, and commands the driver to begin DMA streaming using the two buffers
 * supplied in @p config.  Frames are delivered asynchronously to the internal
 * ready queue; call @ref camera_get_stream_frame to consume them.
 *
 * @param[in] instance  The camera handler instance (must be open).
 * @param[in] config    Two DMA-accessible frame buffers and their common size.
 *
 * @return @c CAMERA_OK on success; @c CAMERA_ERRORRESOURCE if the stream is
 *         already active or required ops are missing; otherwise another
 *         @c camera_handle_status_t error.
 */
camera_handle_status_t camera_start_stream(camera_handler_instance_t *instance,
                                           const camera_stream_config_t *config);

/**
 * @brief Retrieve the next available frame from the streaming queue.
 *
 * Blocks until a frame is ready or @p timeout elapses.  The returned @p frame
 * is a shallow copy of the queue entry; the embedded buffer pointer is valid
 * until the driver reuses that DMA buffer for the next capture, so the caller
 * should process or copy the data before the next two frames arrive.
 *
 * @param[in]  instance  The camera handler instance (stream must be active).
 * @param[out] frame     Filled with buffer pointer, sizes, and sequence number
 *                       on success.
 * @param[in]  timeout   Maximum wait in RT-Thread ticks; use
 *                       @c RT_WAITING_FOREVER to block indefinitely.
 *
 * @return @c CAMERA_OK on success; @c CAMERA_ERRORTIMEOUT if no frame arrived
 *         within @p timeout; @c CAMERA_ERRORRESOURCE if the stream is not
 *         active or the queue is unexpectedly empty.
 */
camera_handle_status_t camera_get_stream_frame(camera_handler_instance_t *instance,
                                               camera_stream_frame_t *frame,
                                               rt_int32_t timeout);

/**
 * @brief Stop continuous frame streaming.
 *
 * Commands the driver to cease DMA captures, marks the stream as inactive,
 * and resets the internal ready queue (including the @c dropped_count
 * diagnostic counter).  Safe to call when streaming is not active — returns
 * @c CAMERA_OK immediately in that case.
 *
 * @param[in] instance  The camera handler instance.
 *
 * @return @c CAMERA_OK on success; @c CAMERA_ERRORRESOURCE if the required
 *         stop-stream op is not registered.
 */
camera_handle_status_t camera_stop_stream(camera_handler_instance_t *instance);

/**
 * @brief Apply camera configuration changes.
 *
 * Push the requested pixel format / frame size / quality down to the sensor
 * driver, then wait for the auto-exposure / auto-white-balance loops to
 * converge before returning. The successfully-applied configuration is
 * stored in @p instance->active_config.
 *
 * @param[in] instance The camera handler instance that owns the device session.
 * @param[in] config   New capture configuration (pixformat / framesize / quality).
 *
 * @return `CAMERA_OK` if settings applied successfully, otherwise an error code
 *         from `camera_handle_status_t`.
 */
camera_handle_status_t camera_change_settings(camera_handler_instance_t *instance,
                                              const camera_capture_config_t *config);

/**
 * @brief Deinitializes the camera.
 *
 * This function deinitializes the camera, releasing any resources
 * that were allocated during initialization.
 *
 * @return An 8-bit signed integer representing the status of the deinitialization.
 *         - 0: Deinitialization successful.
 *         - Non-zero: Deinitialization failed.
 */
camera_handle_status_t camera_deinit(camera_handler_instance_t *instance);

/**
 * @brief Initializes the camera.
 *
 * This function initializes the camera, setting up any necessary
 * resources and configurations.
 *
 * @return An 8-bit signed integer representing the status of the initialization.
 *         - 0: Initialization successful.
 *         - Non-zero: Initialization failed.
 */
camera_handle_status_t camera_init(camera_handler_instance_t *instance);

//******************************** APIs *************************************//

#ifdef __cplusplus
}
#endif


#endif // __CAMERA_HANDLE_H