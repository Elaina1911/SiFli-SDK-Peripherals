/******************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file camera_handle.c
 * 
 * @par dependencies
 * - camera_handle.h  : public API and all type definitions
 * - rthw.h           : rt_hw_interrupt_disable/enable for ISR-safe queue ops
 * - rtdbg.h          : LOG_W / LOG_E macros (pulled in via DBG_TAG block)
 * 
 * @author SiFli 思澈科技
 * 
 * @brief Camera handle layer — sits between application code and the
 * underlying RT-Thread sensor device driver (e.g. ov2640).
 *
 * This module provides two capture modes:
 *
 *  - **Single-shot** (camera_capture_single):
 *    The caller supplies a destination buffer; the function blocks via
 *    rt_device_read until one frame is available or the driver's internal
 *    timeout fires, then returns the actual byte count in request->frame_size.
 *
 *  - **Streaming** (camera_start_stream / camera_get_stream_frame /
 *    camera_stop_stream):
 *    Double-buffered DMA streaming with a two-slot FIFO ready queue.
 *    Frames are pushed into the queue by camera_stream_frame_ready_callback,
 *    which runs in ISR context (via the bus-adapter callback chain) each time
 *    a DMA transfer completes.  The application thread dequeues frames one at
 *    a time via camera_get_stream_frame, which blocks on a semaphore until a
 *    frame is ready or the given timeout expires.  When the consumer is too
 *    slow and the two-slot queue is full, the oldest frame is silently
 *    overwritten and dropped_count is incremented.
 *
 * @par Processing flow (typical single-sensor use)
 *
 *  1. camera_handler_instance_init()  — zero-initialise instance + set
 *                                       JPEG/VGA/quality-10 defaults.
 *  2. camera_init()                   — rt_device_find + rt_device_open.
 *  3. camera_change_settings()        — push pixformat / framesize / quality
 *                                       then wait 500 ms for AEC/AWB settle.
 *  4a. camera_capture_single()        — single blocking frame grab, OR
 *  4b. camera_start_stream()          — arm DMA streaming,
 *      camera_get_stream_frame()      — dequeue frames (blocking),
 *      camera_stop_stream()           — stop DMA + drain queue.
 *  5. camera_deinit()                 — close RT-Thread device + detach
 *                                       frame semaphore.
 *
 * @version V1.0 2026-4-3
 *
 * @note 1 tab == 4 spaces!
 * 
 *****************************************************************************/
#include "camera_handle.h"
#include "rthw.h"

#define DBG_TAG "camera.handle"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/**
 * @brief Map an RT-Thread error code to the equivalent @c camera_handle_status_t.
 *
 * Only the subset of RT-Thread codes that can realistically be returned by the
 * APIs used in this file are mapped explicitly; every other code falls back to
 * the generic @c CAMERA_ERROR.
 */
static camera_handle_status_t camera_status_from_rt_err(rt_err_t err)
{
    switch (err)
    {
        case RT_EOK:
            return CAMERA_OK;
        case -RT_ETIMEOUT:
            return CAMERA_ERRORTIMEOUT;
        case -RT_ENOMEM:
            return CAMERA_ERRORNOMEMORY;
        case -RT_EINVAL:
            return CAMERA_ERRORPARAMETER;
        default:
            return CAMERA_ERROR;
    }
}

/**
 * @brief Reset the stream frame queue to an empty, zero-drop state.
 *
 * Clears @c head, @c tail, @c count, and @c dropped_count under an interrupt
 * lock so the operation is safe whether called from thread or ISR context.
 * Must be called before re-arming the stream (e.g. inside camera_start_stream)
 * and after stopping it.
 */
static void camera_stream_reset_queue(camera_handler_instance_t *instance)
{
    rt_base_t level = rt_hw_interrupt_disable();
    instance->stream.head = 0;
    instance->stream.tail = 0;
    instance->stream.count = 0;
    instance->stream.dropped_count = 0;
    rt_hw_interrupt_enable(level);
}

/**
 * @brief ISR-context frame-ready callback for streaming mode.
 *
 * Registered with the driver via @c camera_stream_start_args_t::frame_callback
 * and called (through the bus-adapter callback chain) each time a DMA transfer
 * completes.  The function inserts the new frame into the two-slot ready
 * queue under interrupt lock:
 *
 *  - If the queue has room (@c count < 2) the frame is enqueued and the
 *    semaphore is released so a waiting @ref camera_get_stream_frame returns.
 *  - If the queue is full the oldest slot is overwritten (tail advanced),
 *    @c stream.dropped_count is incremented, and **no** extra semaphore token
 *    is released (the existing token already covers the new frame).  A
 *    rate-limited @c LOG_W is emitted on the first drop and every 32nd
 *    drop thereafter to surface chronic consumer-side slowness.
 */
static void camera_stream_frame_ready_callback(void *context,
                                               const camera_stream_frame_t *frame)
{
    camera_handler_instance_t *instance = (camera_handler_instance_t *)context;
    rt_base_t level;

    if (instance == RT_NULL || frame == RT_NULL || !instance->stream.is_active)
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    rt_bool_t dropped = RT_FALSE;
    if (instance->stream.count == 2)
    {
        instance->stream.tail = (instance->stream.tail + 1) % 2;
        instance->stream.count--;
        instance->stream.dropped_count++;
        dropped = RT_TRUE;
    }
    rt_uint32_t dropped_total = instance->stream.dropped_count;

    instance->stream.ready_frames[instance->stream.head] = *frame;
    instance->stream.head = (instance->stream.head + 1) % 2;
    instance->stream.count++;
    rt_hw_interrupt_enable(level);

    /* Rate-limited warning: log on first drop and then every 32 drops to
     * avoid flooding while still surfacing chronic consumer slowness. */
    if (dropped && ((dropped_total == 1) || ((dropped_total & 0x1FU) == 0U)))
    {
        LOG_W("camera: stream queue full, dropped frames=%u", (unsigned int)dropped_total);
    }

    /*
     * Only release the semaphore when no frame was dropped.
     * When a drop occurs the oldest frame's semaphore token is re-used for
     * the new frame; releasing again would make sem_val exceed count and
     * cause camera_get_stream_frame() to return CAMERA_ERRORRESOURCE after
     * a successful rt_sem_take().
     */
    if (instance->stream.sem_initialized && !dropped)
    {
        rt_sem_release(&instance->stream.frame_sem);
    }
}

/**
 * @brief Initialize a camera handler instance with optional caller-supplied config.
 *
 * Zeroes the entire instance, then copies the device name and ops pointer from
 * @p input_arg (if provided).  If @p input_arg is @c NULL or its
 * @c device_name field is @c NULL the device name falls back to
 * @ref CAMERA_DEFAULT_DEVICE_NAME so the caller can skip the argument when
 * using a single-sensor board.  The active config is pre-loaded with
 * JPEG/VGA/quality-10 defaults; the stream queue is reset to empty.
 *
 * @note This function does NOT open the RT-Thread device; call
 *       @ref camera_init afterwards.
 */
camera_handle_status_t camera_handler_instance_init(camera_handler_instance_t *instance,
                                                    camera_handler_all_input_arg_t *input_arg)
{
    const camera_device_ops_t *device_ops = RT_NULL;

    if (instance == NULL)
        return CAMERA_ERRORPARAMETER;

    rt_memset(instance, 0, sizeof(*instance));

    if (input_arg != RT_NULL)
    {
        if (input_arg->device_name != RT_NULL)
        {
            instance->device_name = input_arg->device_name;
        }
        if (input_arg->device_ops != RT_NULL)
        {
            device_ops = input_arg->device_ops;
        }
    }

    if (instance->device_name == RT_NULL)
    {
        instance->device_name = CAMERA_DEFAULT_DEVICE_NAME;
    }

    instance->device_ops = device_ops;

    instance->active_config.pixformat = PIXFORMAT_JPEG;
    instance->active_config.framesize = FRAMESIZE_VGA;
    instance->active_config.quality = 10;
    instance->stream.sem_initialized = RT_FALSE;
    instance->stream.is_active = RT_FALSE;
    camera_stream_reset_queue(instance);

    return CAMERA_OK;
}

/**
 * @brief Open the RT-Thread camera device and mark the instance as ready.
 *
 * Locates the device by name via @c rt_device_find, opens it with
 * @c RT_DEVICE_OFLAG_RDWR, and sets @c instance->is_open.  Idempotent: if
 * the device is already open the function returns @c CAMERA_OK immediately.
 * The @c device_ops pointer must be set before calling this function.
 */
camera_handle_status_t camera_init(camera_handler_instance_t *instance)
{
    rt_err_t result;

    if (instance == RT_NULL)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (instance->is_open)
    {
        return CAMERA_OK;
    }

    instance->camera_device = rt_device_find(instance->device_name);
    if (instance->camera_device == RT_NULL)
    {
        return CAMERA_ERRORRESOURCE;
    }

    if (instance->device_ops == RT_NULL)
    {
        return CAMERA_ERRORRESOURCE;
    }

    result = rt_device_open(instance->camera_device, RT_DEVICE_OFLAG_RDWR);
    if (result != RT_EOK)
    {
        instance->camera_device = RT_NULL;
        return camera_status_from_rt_err(result);
    }

    instance->is_open = RT_TRUE;
    return CAMERA_OK;
}

/**
 * @brief Close the camera device and release all associated resources.
 *
 * If streaming is still active, @ref camera_stop_stream is called first.
 * Then the RT-Thread device is closed and the frame semaphore is detached
 * (if it was initialised).  After this call the instance can be re-used by
 * calling @ref camera_handler_instance_init followed by @ref camera_init.
 */
camera_handle_status_t camera_deinit(camera_handler_instance_t *instance)
{
    if (instance == RT_NULL)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (instance->stream.is_active)
    {
        camera_stop_stream(instance);
    }

    if (instance->is_open && instance->camera_device != RT_NULL)
    {
        rt_device_close(instance->camera_device);
    }

    instance->camera_device = RT_NULL;
    instance->is_open = RT_FALSE;

    if (instance->stream.sem_initialized)
    {
        rt_sem_detach(&instance->stream.frame_sem);
        instance->stream.sem_initialized = RT_FALSE;
    }

    return CAMERA_OK;
}

/**
 * @brief Push a new capture configuration to the sensor driver.
 *
 * Issues three @c rt_device_control calls in sequence — set_pixformat,
 * set_framesize, set_quality — using the integer command IDs stored in
 * @c instance->device_ops->command_set.  On full success the accepted
 * configuration is saved to @c instance->active_config and a 500 ms
 * blocking delay is inserted to let the sensor AEC / AWB loops converge.
 *
 * @note May not be called while streaming is active (@c stream.is_active);
 *       returns @c CAMERA_ERRORRESOURCE in that case.
 */
camera_handle_status_t camera_change_settings(camera_handler_instance_t *instance,
                                              const camera_capture_config_t *config)
{
    rt_err_t result;
    const camera_device_ops_t *device_ops;

    if (instance == RT_NULL || config == RT_NULL)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (!instance->is_open || instance->camera_device == RT_NULL)
    {
        return CAMERA_ERRORRESOURCE;
    }

    if (instance->stream.is_active)
    {
        return CAMERA_ERRORRESOURCE;
    }

    device_ops = instance->device_ops;
    if (device_ops == RT_NULL)
    {
        return CAMERA_ERRORRESOURCE;
    }

    result = rt_device_control(instance->camera_device,
                               device_ops->command_set.set_pixformat,
                               (void *)(rt_ubase_t)config->pixformat);
    if (result != RT_EOK)
    {
        return camera_status_from_rt_err(result);
    }

    result = rt_device_control(instance->camera_device,
                               device_ops->command_set.set_framesize,
                               (void *)(rt_ubase_t)config->framesize);
    if (result != RT_EOK)
    {
        return camera_status_from_rt_err(result);
    }

    result = rt_device_control(instance->camera_device,
                               device_ops->command_set.set_quality,
                               (void *)(rt_ubase_t)config->quality);
    if (result != RT_EOK)
    {
        return camera_status_from_rt_err(result);
    }

    instance->active_config = *config;
    /*
     * Wait for sensor AEC / AWB loops to converge after a format / framesize
     * change. 500 ms is empirically enough for the OV2640 to produce stable
     * frames; lower values give visibly under-exposed first frames.
     */
    rt_thread_mdelay(500);
    return CAMERA_OK;
}

/**
 * @brief Perform a single blocking frame capture via @c rt_device_read.
 *
 * Delegates to @c rt_device_read with the caller-provided buffer; the driver
 * blocks internally until one frame is available or its internal timeout
 * fires.  On success @p request->frame_size is set to the byte count returned
 * by the driver (e.g. compressed JPEG payload length).
 *
 * @note Not usable while streaming is active (@c stream.is_active).
 */
camera_handle_status_t camera_capture_single(camera_handler_instance_t *instance,
                                             camera_capture_request_t *request)
{
    rt_size_t read_size;

    if (instance == RT_NULL || request == RT_NULL)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (!instance->is_open || instance->camera_device == RT_NULL)
    {
        return CAMERA_ERRORRESOURCE;
    }

    if (request->buffer == RT_NULL || request->buffer_size == 0)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (instance->stream.is_active)
    {
        return CAMERA_ERRORRESOURCE;
    }

    if (instance->device_ops == RT_NULL)
    {
        return CAMERA_ERRORRESOURCE;
    }

    read_size = rt_device_read(instance->camera_device, 0,
                               request->buffer,
                               request->buffer_size);
    if (read_size == 0)
    {
        request->frame_size = 0;
        return CAMERA_ERRORTIMEOUT;
    }

    request->frame_size = read_size;
    return CAMERA_OK;
}

/**
 * @brief Arm the driver for continuous double-buffered DMA streaming.
 *
 * Lazily initialises the frame semaphore on the first call, drains any stale
 * tokens, resets the ready queue, and fills a @c camera_stream_start_args_t
 * (wiring in @ref camera_stream_frame_ready_callback as the internal frame
 * sink) before forwarding it to the driver via @c rt_device_control.  On
 * driver error the stream is rolled back to inactive and the queue is reset.
 *
 * @note The two buffers in @p config must remain valid (DMA-accessible)
 *       until @ref camera_stop_stream returns.
 */
camera_handle_status_t camera_start_stream(camera_handler_instance_t *instance,
                                           const camera_stream_config_t *config)
{
    camera_stream_start_args_t args;
    rt_err_t result;

    if (instance == RT_NULL || config == RT_NULL)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (!instance->is_open || instance->camera_device == RT_NULL)
    {
        return CAMERA_ERRORRESOURCE;
    }

    if (config->buffers[0] == RT_NULL ||
        config->buffers[1] == RT_NULL ||
        config->buffer_size == 0)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (instance->device_ops == RT_NULL ||
        instance->device_ops->command_set.start_stream == 0)
    {
        return CAMERA_ERRORRESOURCE;
    }

    if (instance->stream.is_active)
    {
        return CAMERA_ERRORRESOURCE;
    }

    if (!instance->stream.sem_initialized)
    {
        result = rt_sem_init(&instance->stream.frame_sem, "cam_strm", 0, RT_IPC_FLAG_FIFO);
        if (result != RT_EOK)
        {
            return camera_status_from_rt_err(result);
        }
        instance->stream.sem_initialized = RT_TRUE;
    }

    while (rt_sem_trytake(&instance->stream.frame_sem) == RT_EOK)
    {
    }

    camera_stream_reset_queue(instance);
    instance->stream.is_active = RT_TRUE;

    args.buffers[0] = config->buffers[0];
    args.buffers[1] = config->buffers[1];
    args.buffer_size = config->buffer_size;
    args.frame_callback = camera_stream_frame_ready_callback;
    args.callback_context = instance;

    result = rt_device_control(instance->camera_device,
                               instance->device_ops->command_set.start_stream,
                               &args);
    if (result != RT_EOK)
    {
        instance->stream.is_active = RT_FALSE;
        camera_stream_reset_queue(instance);
        return camera_status_from_rt_err(result);
    }

    return CAMERA_OK;
}

/**
 * @brief Dequeue one ready frame from the streaming ring buffer.
 *
 * Takes the frame semaphore with the given @p timeout.  On success the tail
 * entry is copied to @p frame under interrupt lock and the queue count is
 * decremented.  The spurious-empty guard (@c count == 0 after a successful
 * semaphore take) returns @c CAMERA_ERRORRESOURCE — this should not happen
 * in normal operation but defends against queue corruption.
 */
camera_handle_status_t camera_get_stream_frame(camera_handler_instance_t *instance,
                                               camera_stream_frame_t *frame,
                                               rt_int32_t timeout)
{
    rt_base_t level;
    rt_err_t result;

    if (instance == RT_NULL || frame == RT_NULL)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (!instance->stream.is_active || !instance->stream.sem_initialized)
    {
        return CAMERA_ERRORRESOURCE;
    }

    result = rt_sem_take(&instance->stream.frame_sem, timeout);
    if (result != RT_EOK)
    {
        return camera_status_from_rt_err(result);
    }

    level = rt_hw_interrupt_disable();
    if (instance->stream.count == 0)
    {
        rt_hw_interrupt_enable(level);
        return CAMERA_ERRORRESOURCE;
    }

    *frame = instance->stream.ready_frames[instance->stream.tail];
    instance->stream.tail = (instance->stream.tail + 1) % 2;
    instance->stream.count--;
    rt_hw_interrupt_enable(level);

    return CAMERA_OK;
}

/**
 * @brief Command the driver to stop DMA streaming and clean up queue state.
 *
 * Issues the stop_stream control command, marks @c stream.is_active as
 * @c RT_FALSE, and resets the ready queue (including @c dropped_count).
 * The driver error code is checked after the bookkeeping so that local state
 * is always cleaned up even if the driver returns an error.  Idempotent:
 * returns @c CAMERA_OK immediately if the stream is already inactive.
 */
camera_handle_status_t camera_stop_stream(camera_handler_instance_t *instance)
{
    rt_err_t result;

    if (instance == RT_NULL)
    {
        return CAMERA_ERRORPARAMETER;
    }

    if (!instance->stream.is_active)
    {
        return CAMERA_OK;
    }

    if (instance->device_ops == RT_NULL ||
        instance->device_ops->command_set.stop_stream == 0)
    {
        return CAMERA_ERRORRESOURCE;
    }

    result = rt_device_control(instance->camera_device,
                               instance->device_ops->command_set.stop_stream,
                               RT_NULL);
    instance->stream.is_active = RT_FALSE;
    camera_stream_reset_queue(instance);

    if (result != RT_EOK)
    {
        return camera_status_from_rt_err(result);
    }

    return CAMERA_OK;
}

