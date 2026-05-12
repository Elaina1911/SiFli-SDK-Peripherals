/********************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file dvp.c
 *
 * @par dependencies
 * - rtthread.h
 * - dvp.h
 * - stdint.h
 * - stdio.h
 * - string.h
 * - rthw.h
 * - rtdevice.h
 *
 * @author SiFli 思澈科技
 *
 * @brief DVP (Digital Video Port) software driver implementation.
 *
 * This module implements a software DVP interface used to capture camera
 * parallel data (D0..D7) using a GPTIM as pixel/line clock and DMA with
 * ping-pong buffering. It handles DMA callbacks, VSYNC interrupts and offers
 * a small API to start/stop capture, configure ping-pong buffers and query
 * capture status.
 *
 * Processing flow:
 * - Initialize with `dvp_init()` which allocates buffers and configures
 *   pins, DMA and timer resources.
 * - Hardware delivers pixels into a ping-pong DMA buffer. DMA callbacks
 *   call into this module to process halves or full transfers.
 * - For JPEG mode the driver searches SOI/EOI markers to form frames;
 *   for raw/pixel modes it copies fixed-size chunks until the user buffer
 *   is filled.
 * - The driver notifies the upper layer via the user callback registered with
 *   `bus_adapter_set_frame_callback()`.
 *
 * Notes:
 * - This implementation binds to board-specific timer/DMA instances and
 *   pin mappings; platform abstraction is minimal for performance reasons.
 * - Callers should treat these APIs as non-ISR (except callbacks) and
 *   avoid long-running work in callback contexts.
 *
 * @version V1.0 2026-4-3
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#include "dvp.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "rthw.h"
#include <stddef.h>
#include <rtdevice.h>

#define DBG_TAG "dvp"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define DEBUG_DVP 0

/* When OV2640_DVP_PINGPONG_USE_SECTION is enabled in Kconfig the buffer is
 * placed in the ".dvp_pingpong" section so the board link script can map it
 * to a dedicated SRAM region (e.g. DVP_SRAM on SF32LB56X).  When disabled
 * the buffer falls into normal .bss, which is sufficient for SF32LB52X.
 *
 * The array is sized by OV2640_DVP_PINGPONG_POOL_SIZE — the compile-time
 * ceiling.  dvp_set_pingpong_size() can change the active size at runtime to
 * any value in [1, POOL_SIZE] without recompiling. */
#ifdef OV2640_DVP_PINGPONG_USE_SECTION
uint8_t dvp_pingpong_buffer[OV2640_DVP_PINGPONG_POOL_SIZE]
    __attribute__((section(".dvp_pingpong")));
#else
uint8_t dvp_pingpong_buffer[OV2640_DVP_PINGPONG_POOL_SIZE];
#endif

#if DEBUG_DVP
#define DVP_DEBUG_SNAPSHOT_SIZE 6000U
static uint8_t g_dvp_debug_snapshot[DVP_DEBUG_SNAPSHOT_SIZE];
#endif

/*
 * DVP is a single hardware instance on this MCU. We expose the driver as a
 * `bus_adapter_t` so upper layers (e.g. ov2640.c) drive it through the generic
 * adapter interface; the public `dvp_*` symbols below are themselves the
 * `bus_adapter_ops_t` entries (no extra wrapper layer).
 */
static dvp_handle_t s_dvp_handle;
static bus_frame_ready_callback_t s_user_frame_callback;
static void *s_user_frame_callback_data;
static bus_adapter_t s_dvp_bus_adapter;

/* GPTIM2 used to generate XCLK (sensor master clock) via PWM */
static GPT_HandleTypeDef s_xclk_gptim;
static rt_bool_t         s_xclk_initialized = RT_FALSE;

/* Build a bus_frame_t from the current DVP capture state and dispatch it
 * directly to the user-registered bus callback. Called from the DMA ISR /
 * timer-thread context whenever a complete frame is ready. */
static void dvp_dispatch_frame(dvp_handle_t *handle)
{
    if (s_user_frame_callback == RT_NULL)
        return;

    bus_frame_t f;
    f.buffer    = handle->config.frame_buffer;
    f.length    = handle->capture.current_size;
    f.timestamp = rt_tick_get();
    f.sequence  = 0;
    s_user_frame_callback(&s_dvp_bus_adapter, &f, s_user_frame_callback_data);
}

/* 为免搬运直接存放数据申请的高速SRAM空间：600KB */
//uint8_t g_sram_debug_buffer[614400] __attribute__((section(".bss.sram")));

static dvp_handle_t *dvp_get_handle_from_dma(DMA_HandleTypeDef *hdma)
{
    if (hdma == RT_NULL)
    {
        return RT_NULL;
    }

    return (dvp_handle_t *)((uint8_t *)hdma - offsetof(dvp_handle_t, dma));
}

static GPT_TypeDef *dvp_get_timer_instance(const dvp_handle_t *handle)
{
    return (GPT_TypeDef *)handle->config.resources.timer_instance;
}

static GPIO_TypeDef *dvp_get_data_gpio_instance(const dvp_handle_t *handle)
{
    return (GPIO_TypeDef *)handle->config.resources.data_gpio_instance;
}

/**
 * @brief Search a memory buffer for the JPEG SOI marker.
 *
 * This function scans the specified buffer for the first occurrence of the
 * JPEG Start Of Image marker sequence `0xFF 0xD8`.
 *
 * @param buffer is a pointer to the input buffer to search.
 *
 * @param length is the size of the input buffer in bytes.
 *
 * @return Return the zero-based offset of the SOI marker when found.
 *         If the marker is not found or the buffer is too short, `-1` is returned.
 */
static int search_for_SOI(uint8_t *buffer, size_t length)
{
    if (length < 2) return -1;
    for (size_t i = 0; i < length - 1; i++)
    {
        if ((buffer[i] == (uint8_t)0xFF) && (buffer[i + 1] == (uint8_t)0xD8))
        {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Search a memory buffer for the JPEG EOI marker.
 *
 * This function scans the specified buffer for the first occurrence of the
 * JPEG End Of Image marker sequence `0xFF 0xD9`.
 *
 * @param buffer is a pointer to the input buffer to search.
 *
 * @param length is the size of the input buffer in bytes.
 *
 * @return Return the zero-based offset of the last byte of the EOI marker
 *         when found. If the marker is not found or the buffer is too short,
 *         `-1` is returned.
 */
static int search_for_EOI(uint8_t *buffer, size_t length)
{
    if (length < 2) return -1;
    for (size_t i = 0; i < length - 1; i++)
    {
        if ((buffer[i] == (uint8_t)0xFF) && (buffer[i + 1] == (uint8_t)0xD9))
        {
            return i + 1;
        }
    }
    return -1;
}

#if DEBUG_DVP
static void dump_buffer_hex(const uint8_t *buffer,
                            uint32_t length,
                            uint32_t pingpong_offset,
                            uint32_t frame_offset)
{
    uint32_t index = 0;
    char line[80];

    rt_kprintf("DVP raw chunk: pingpong_offset=%u frame_offset=%u bytes=%u\n",
               (unsigned int)pingpong_offset,
               (unsigned int)frame_offset,
               (unsigned int)length);
    while (index < length)
    {
        uint32_t line_bytes = ((length - index) > 16U) ? 16U : (length - index);
        int pos = rt_snprintf(line,
                              sizeof(line),
                              "F%04u P%04u: ",
                              (unsigned int)(frame_offset + index),
                              (unsigned int)(pingpong_offset + index));

        for (uint32_t i = 0; i < line_bytes; i++)
        {
            pos += rt_snprintf(&line[pos], sizeof(line) - pos, "%02X ", buffer[index + i]);
        }

        rt_kprintf("%s\n", line);
        index += line_bytes;
    }
}

static void snapshot_and_dump_buffer(const uint8_t *buffer,
                                     uint32_t length,
                                     uint32_t pingpong_offset,
                                     uint32_t frame_offset)
{
    uint32_t snapshot_length = (length <= DVP_DEBUG_SNAPSHOT_SIZE) ? length : DVP_DEBUG_SNAPSHOT_SIZE;

    rt_memcpy(g_dvp_debug_snapshot, buffer, snapshot_length);
    if (snapshot_length < length)
    {
        rt_kprintf("DVP raw chunk truncated: requested=%u snapshot=%u\n",
                   (unsigned int)length,
                   (unsigned int)snapshot_length);
    }

    dump_buffer_hex(g_dvp_debug_snapshot, snapshot_length, pingpong_offset, frame_offset);
}
#endif /* DEBUG_DVP */
static void dvp_reset_jpeg_boundary_state(dvp_handle_t *handle)
{
    handle->jpeg.prev_byte = 0;
    handle->jpeg.prev_byte_valid = 0;
}

static void dvp_update_jpeg_boundary_state(dvp_handle_t *handle,
                                           const uint8_t *buffer,
                                           uint32_t length)
{
    if (length == 0)
    {
        handle->jpeg.prev_byte_valid = 0;
        return;
    }

    handle->jpeg.prev_byte = buffer[length - 1];
    handle->jpeg.prev_byte_valid = 1;
}

static void dvp_stop_jpeg_capture(dvp_handle_t *handle, uint8_t frame_ready)
{
    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.enable_capture = 0;
    handle->capture.soi_found = 0;
    handle->capture.frame_ready = frame_ready;
    handle->capture.capture_started = 0;
    if (!frame_ready)
    {
        handle->capture.current_size = 0;
    }
    dvp_reset_jpeg_boundary_state(handle);
    rt_hw_interrupt_enable(level);
}

static rt_err_t dvp_append_jpeg_bytes(dvp_handle_t *handle,
                                      const uint8_t *source,
                                      uint32_t copy_size)
{
    uint8_t *full_buffer = handle->config.frame_buffer;
    uint32_t buffer_size = handle->config.buffer_size;

    if (handle->capture.current_size + copy_size > buffer_size)
    {
        LOG_E("DVP buffer overflow: current=%d, copy=%d, buffer=%d",
              handle->capture.current_size, copy_size, buffer_size);
        dvp_stop_jpeg_capture(handle, 0);
        return -RT_ERROR;
    }

    memcpy(&full_buffer[handle->capture.current_size], source, copy_size);

    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.current_size += copy_size;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

/**
 * @brief Process one RAW-format ping-pong buffer block.
 *
 * This function copies one half of the DMA ping-pong buffer into the user
 * frame buffer for fixed-size capture modes such as RAW, RGB565 and YUV422.
 * When the configured frame buffer becomes full, the function marks the frame
 * as ready and notifies the registered callback if present.
 *
 * @param handle is a pointer to the DVP handle.
 *
 * @param buffer_offset is the offset of the active half-buffer inside the
 *        ping-pong buffer. Use `0` for the first half and `half_size` for the
 *        second half.
 *
 * @return This function does not return a value.
 */
static void process_raw_data(dvp_handle_t *handle, uint32_t buffer_offset)
{
#if DEBUG_DVP
    /* 调试模式下采用 DMA_NORMAL 且直接抛入 SRAM，因此不需要分段搬运，触发中断时即1帧结束 */
    // extern uint8_t g_sram_debug_buffer[614400];
    // rt_kprintf("\n[DEBUG] --- FULL FRAME DUMP FROM SRAM (First 64 bytes) ---\n");
    // dump_buffer_hex(g_sram_debug_buffer, 64, 0, 0);

    // /* 停止后续采集 */
    // __HAL_DMA_DISABLE(&handle->dma);
    // handle->capture.frame_ready = 1;
    // handle->capture.soi_found = 0;
    // handle->capture.enable_capture = 0;
    // handle->capture.capture_started = 0;
#endif

    uint8_t *full_buffer = handle->config.frame_buffer;
    uint8_t *pingpong_buffer = handle->pingpong_buffer;
    uint32_t half_size = handle->config.pingpong_buffer_size / 2;
    uint8_t *source_ptr = &pingpong_buffer[buffer_offset];
    uint32_t buffer_size = handle->config.buffer_size;

    if (full_buffer == NULL || buffer_size == 0)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        handle->capture.enable_capture = 0;
        handle->capture.capture_started = 0;
        handle->capture.frame_ready = 0;
        rt_hw_interrupt_enable(level);
        return;
    }

    if (!handle->capture.soi_found)
        return;  /* Frame not started yet (waiting for VSYNC) */

    uint32_t remaining = buffer_size - handle->capture.current_size;
    if (remaining == 0)
        return;

    uint32_t copy_size = (half_size <= remaining) ? half_size : remaining;
    /* DEBUG: dump the last callback's pingpong buffer then halt to verify DMA data */
    // if (handle->capture.current_size >= 614400-5120 )
    // {
    //     __HAL_DMA_DISABLE(&handle->dma);
    //     rt_kprintf("\n[DEBUG] LAST pingpong dump (offset=%u, half=%u, frame_offset=%u):\n",
    //                (unsigned)buffer_offset, (unsigned)half_size,
    //                (unsigned)handle->capture.current_size);
    //     snapshot_and_dump_buffer(source_ptr,
    //                              copy_size < DVP_DEBUG_SNAPSHOT_SIZE ? copy_size : DVP_DEBUG_SNAPSHOT_SIZE,
    //                              buffer_offset, handle->capture.current_size);
    //     RT_ASSERT(0);
    // }
#if DEBUG_DVP
    /* 截取画面一半时的数据：总字节数614400，中间在 307200 左右 */
    //if (handle->capture.current_size >= (614400 / 8)*3)
    if (handle->dma.Instance->CNDTR!=0x500&&handle->dma.Instance->CNDTR!=0xA00)
    {
        __HAL_DMA_DISABLE(&handle->dma);
        rt_kprintf("\n[DEBUG] --- SOURCE DUMP (Pingpong Buffer) ---\n");
        snapshot_and_dump_buffer(source_ptr, 1400, buffer_offset, handle->capture.current_size);
        RT_ASSERT(0); // 打印完毕后卡死并停留在当前状态
    }
#endif
    /* Invalidate D-Cache for the pingpong half-buffer before CPU reads it.
     * DMA writes directly to SRAM3 physical memory, bypassing the CPU cache.
     * Without invalidation, the CPU memcpy would read stale cache lines and
     * produce corrupted rows (visible as periodic snow bands) in the frame. */
    //mpu_dcache_invalidate((uint32_t *)source_ptr, copy_size);
    memcpy(full_buffer + handle->capture.current_size, source_ptr, copy_size);

#if DEBUG_DVP
    if (handle->capture.current_size >= (614400 / 8)*3)
    {
        rt_kprintf("\n[DEBUG] --- DESTINATION DUMP (PSRAM FB) ---\n");
        snapshot_and_dump_buffer(full_buffer + handle->capture.current_size, copy_size, buffer_offset, handle->capture.current_size);
        RT_ASSERT(0); // 打印完毕后卡死并停留在当前状态
    }
#endif
    
    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.current_size += copy_size;

    if (handle->capture.current_size >= buffer_size)
    {     
        handle->capture.frame_ready = 1;
        handle->capture.soi_found = 0;
        handle->capture.enable_capture = 0;        
        handle->capture.capture_started = 0;
    }
    rt_hw_interrupt_enable(level);

    if (handle->capture.frame_ready)
    {
        /* Flush write-back DCache to PSRAM before display DMA reads the frame buffer */
        //mpu_dcache_clean(full_buffer, buffer_size);
        dvp_dispatch_frame(handle);
    }
}

/**
 * @brief Process one JPEG ping-pong buffer block.
 *
 * This function parses one half of the DMA ping-pong buffer in JPEG mode.
 * It searches the captured stream for SOI and EOI markers, copies valid data
 * into the user frame buffer and marks the frame ready after a complete JPEG
 * image is assembled.
 *
 * @param handle is a pointer to the DVP handle.
 *
 * @param buffer_offset is the offset of the active half-buffer inside the
 *        ping-pong buffer. Use `0` for the first half and `half_size` for the
 *        second half.
 *
 * @return This function does not return a value.
 */
static void process_jpeg_data(dvp_handle_t *handle, uint32_t buffer_offset)
{
    uint8_t *full_buffer = handle->config.frame_buffer;
    uint8_t *pingpong_buffer = handle->pingpong_buffer;
    uint32_t half_size = handle->config.pingpong_buffer_size / 2;
    uint8_t *source_ptr = &pingpong_buffer[buffer_offset];
    uint32_t buffer_size = handle->config.buffer_size;
    int index = 0;

    if(full_buffer == NULL || buffer_size == 0)
    {
        dvp_stop_jpeg_capture(handle, 0);
        return;
    }

    if (half_size == 0)
    {
        dvp_reset_jpeg_boundary_state(handle);
        return;
    }

    if (!handle->capture.soi_found &&
        handle->jpeg.prev_byte_valid &&
        handle->jpeg.prev_byte == 0xFF &&
        source_ptr[0] == 0xD8)
    {
        uint8_t soi_marker[2] = {0xFF, 0xD8};
        if (dvp_append_jpeg_bytes(handle, soi_marker, sizeof(soi_marker)) != RT_EOK)
        {
            return;
        }

        rt_base_t level = rt_hw_interrupt_disable();
        handle->capture.soi_found = 1;
        rt_hw_interrupt_enable(level);
        index = 1;
    }
    else if (handle->capture.soi_found &&
             handle->jpeg.prev_byte_valid &&
             handle->jpeg.prev_byte == 0xFF &&
             source_ptr[0] == 0xD9)
    {
        if (dvp_append_jpeg_bytes(handle, source_ptr, 1) != RT_EOK)
        {
            return;
        }

        rt_base_t level = rt_hw_interrupt_disable();
        handle->capture.frame_ready = 1;
        handle->capture.soi_found = 0;
        handle->capture.enable_capture = 0;
        handle->capture.capture_started = 0;
        dvp_reset_jpeg_boundary_state(handle);
        rt_hw_interrupt_enable(level);
        index = 1;
    }

    while(index<half_size && handle->capture.enable_capture)
    {
        if(!handle->capture.soi_found)
        {
            int soi_index = search_for_SOI(&source_ptr[index], half_size - index);
            if (soi_index >= 0)
            {
                rt_base_t level = rt_hw_interrupt_disable();
                handle->capture.soi_found = 1;
                rt_hw_interrupt_enable(level);
                index += soi_index;
                int eoi_index = search_for_EOI(&source_ptr[index], half_size - index);
                if (eoi_index >= 0)
                {
                    uint32_t copy_size = eoi_index + 1;
                    if (dvp_append_jpeg_bytes(handle, &source_ptr[index], copy_size) != RT_EOK)
                    {
                        break;
                    }

                    level = rt_hw_interrupt_disable();
                    handle->capture.frame_ready = 1;
                    handle->capture.soi_found = 0;
                    handle->capture.enable_capture = 0;
                    handle->capture.capture_started = 0;
                    dvp_reset_jpeg_boundary_state(handle);
                    rt_hw_interrupt_enable(level);
                    index += copy_size;
                }
                else
                {
                    uint32_t copy_size = half_size - index;
                    if (dvp_append_jpeg_bytes(handle, &source_ptr[index], copy_size) != RT_EOK)
                    {
                        break;
                    }
                    dvp_update_jpeg_boundary_state(handle, source_ptr, half_size);
                    break;
                }

            }
            else
            {
                dvp_update_jpeg_boundary_state(handle, source_ptr, half_size);
                break;
            }
        }

        if(handle->capture.soi_found)
        {
            int eoi_index = search_for_EOI(&source_ptr[index], half_size - index);
            if (eoi_index >= 0)
            {
                uint32_t copy_size = eoi_index + 1;
                if (dvp_append_jpeg_bytes(handle, &source_ptr[index], copy_size) != RT_EOK)
                {
                    break;
                }

                rt_base_t level = rt_hw_interrupt_disable();
                handle->capture.frame_ready = 1;
                handle->capture.soi_found = 0;
                handle->capture.enable_capture = 0;
                handle->capture.capture_started = 0;
                dvp_reset_jpeg_boundary_state(handle);
                rt_hw_interrupt_enable(level);
                index += copy_size;
            }
            else
            {
                uint32_t copy_size = half_size - index;
                if (dvp_append_jpeg_bytes(handle, &source_ptr[index], copy_size) != RT_EOK)
                {
                    break;
                }
                dvp_update_jpeg_boundary_state(handle, source_ptr, half_size);
                break;
            }
        }

        if(handle->capture.frame_ready)
        {
            dvp_dispatch_frame(handle);

            /*
             * Continue only when the callback explicitly rearms capture for the
             * next frame by clearing frame_ready via dvp_start_capture() or an
             * equivalent state reset. This avoids silently reusing the previous
             * frame buffer during background capture.
             */
            if(!handle->capture.enable_capture || handle->capture.frame_ready){
                break;
            }

            full_buffer = handle->config.frame_buffer;
            buffer_size = handle->config.buffer_size;
            if (full_buffer == NULL || buffer_size == 0)
            {
                dvp_stop_jpeg_capture(handle, 0);
                break;
            }
        }
    }
}



//******************************** Callbacks *********************************//
/**
 * @brief Handle DMA transfer-complete events.
 *
 * This callback is invoked when DMA completes transfer of the second half of
 * the ping-pong buffer. It dispatches the received data to the JPEG or fixed-
 * size data processing path according to the current DVP mode.
 *
 * @param hdma is a pointer to the DMA handle that triggered the callback.
 *
 * @return This function does not return a value.
 */
void dvp_dma_xfer_cplt_callback(DMA_HandleTypeDef *hdma)
{

    dvp_handle_t *handle = dvp_get_handle_from_dma(hdma);

    if (handle == RT_NULL || !handle->capture.enable_capture)
        return;

    uint32_t half_size = handle->config.pingpong_buffer_size / 2;

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        process_jpeg_data(handle, half_size);
    }
    else
    { 
        process_raw_data(handle, half_size);
        //hdma->Instance->CNDTR=0x2800;
        //__HAL_DMA_SET_COUNTER(hdma,10240);
    }
}

/**
 * @brief Handle DMA half-transfer events.
 *
 * This callback is invoked when DMA completes transfer of the first half of
 * the ping-pong buffer. It dispatches the received data to the JPEG or fixed-
 * size data processing path according to the current DVP mode.
 *
 * @param hdma is a pointer to the DMA handle that triggered the callback.
 *
 * @return This function does not return a value.
 */
void dvp_dma_half_xfer_cplt_callback(DMA_HandleTypeDef *hdma)
{
    //__HAL_DMA_DISABLE(hdma);
    // RT_ASSERT(0);
    dvp_handle_t *handle = dvp_get_handle_from_dma(hdma);

    if (handle == RT_NULL || !handle->capture.enable_capture)
        return;

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        process_jpeg_data(handle, 0);
    }
    else
    {
        process_raw_data(handle, 0);
        //hdma->Instance->CNDTR=0x1400;
        //__HAL_DMA_SET_COUNTER(hdma,5120);
    }
}

/**
 * @brief Handle DMA error events.
 *
 * This callback is invoked when a DMA transfer error occurs while receiving
 * image data from the camera interface.
 *
 * @param hdma is a pointer to the DMA handle that triggered the error.
 *
 * @return This function does not return a value.
 */
static void dvp_dma_error_callback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    LOG_E("DVP DMA Error occurred!");
}

/**
 * @brief Handle VSYNC rising-edge interrupts.
 *
 * This interrupt handler is triggered on the camera VSYNC rising edge. For
 * fixed-size capture modes it starts one capture cycle. For JPEG mode the
 * driver relies on SOI and EOI markers, so VSYNC is only used as timing
 * reference and does not directly restart capture.
 *
 * @param args is the user argument associated with the interrupt. It is not used.
 *
 * @return This function does not return a value.
 */
static void dvp_vsync_irq_handler(void *args)
{
    bus_adapter_t *self = (bus_adapter_t *)args;
    if (self == RT_NULL)
        return;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (handle == RT_NULL)
        return;

    // JPEG mode runs as a continuous byte stream and is parsed by SOI/EOI,
    // so do not restart DMA on each VSYNC.
    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        return;
    }

    // Fixed-size modes use VSYNC to align the DMA start to a frame boundary.
    if (!handle->capture.enable_capture)
    {
        return;
    }
    if (handle->capture.capture_started)
    {
        return;
    }
    if (handle->capture.frame_ready)
    {
        return;
    }
#if DEBUG_DVP
    GPIO_TypeDef *gpio = hwp_gpio1;
    GPIO_InitTypeDef GPIO_InitStruct;
    /* set GPIO1 pin10 to output mode */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = 74;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);
    /* set pin to high */
    HAL_GPIO_WritePin(gpio, 74, GPIO_PIN_SET);
#endif
    dvp_start(self);
    // __HAL_DMA_DISABLE(&handle->dma);
    //     //rt_kprintf("\n[DEBUG] --- SOURCE DUMP (Pingpong Buffer) ---\n");
    //     //snapshot_and_dump_buffer(source_ptr, 6000, buffer_offset, handle->capture.current_size);
    //     RT_ASSERT(0); // 打印完毕后卡死并停留在当前状态
}
//******************************** Callbacks *********************************//



//***************************** Initialization ******************************//
/**
 * @brief Configure DVP data input pins.
 *
 * This function configures the camera parallel data pins D0 to D7 as GPIO
 * inputs that match the DMA source register layout used by this driver.
 *
 * @param handle is a pointer to the DVP handle.
 *
 * @return Return `RT_EOK` on success.
 */
static int dvp_config_data_pins(dvp_handle_t *handle)
{
    GPIO_TypeDef *data_gpio = dvp_get_data_gpio_instance(handle);

    // Data pins must be mapped to GPIO1 bits 0-7 (because DMA reads low 8 bits of DIR register)
    for (int i = 0; i < 8; i++)
    {
        HAL_PIN_Set(handle->config.resources.data_pin_pad_base + i,
                    handle->config.resources.data_pin_func_base + i,
                    PIN_PULLUP,
                    1);
        
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = handle->config.resources.data_gpio_pin_base + i;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(data_gpio, &GPIO_InitStruct);
    }
    
    return RT_EOK;
}

/**
 * @brief Configure DVP control pins and interrupts.
 *
 * This function attaches the VSYNC GPIO interrupt used to detect frame start.
 *
 * @param handle is a pointer to the DVP handle.
 *
 * @return Return `RT_EOK` on success.
 */
static int dvp_config_control_pins(dvp_handle_t *handle)
{
    // Configure VSYNC interrupt
    rt_pin_attach_irq(handle->config.resources.vsync_pin, PIN_IRQ_MODE_RISING, dvp_vsync_irq_handler, &s_dvp_bus_adapter);
    rt_pin_irq_enable(handle->config.resources.vsync_pin, PIN_IRQ_ENABLE);
    
    // rt_pin_attach_irq(73, PIN_IRQ_MODE_FALLING, dvp_href_irq_handler, handle);
    // rt_pin_irq_enable(73, PIN_IRQ_ENABLE);
    return RT_EOK;
}

/**
 * @brief Configure the DMA channel used by the DVP receiver.
 *
 * This function initializes the DMA instance, installs transfer callbacks and
 * enables the corresponding interrupt.
 *
 * @param handle is a pointer to the DVP handle.
 *
 * @return Return `RT_EOK` on success. If DMA initialization fails,
 *         `-RT_ERROR` is returned.
 */
static int dvp_config_dma(dvp_handle_t *handle)
{
    handle->dma.Instance = handle->config.resources.dma_instance;
    handle->dma.Init.Request = handle->config.resources.dma_request;
    handle->dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle->dma.Init.PeriphInc = DMA_PINC_DISABLE;
    handle->dma.Init.MemInc = DMA_MINC_ENABLE;
    handle->dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    handle->dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    handle->dma.Init.Mode = DMA_CIRCULAR;
    handle->dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    handle->dma.Init.BurstSize = 0;
    handle->dma.XferCpltCallback = dvp_dma_xfer_cplt_callback;
    handle->dma.XferHalfCpltCallback = dvp_dma_half_xfer_cplt_callback;
    handle->dma.XferErrorCallback = dvp_dma_error_callback;
    
    if (HAL_DMA_Init(&handle->dma) != HAL_OK)
    {
        LOG_E("DVP DMA init failed!");
        return -RT_ERROR;
    }
    
    __HAL_LINKDMA(&handle->gptim, hdma[GPT_DMA_ID_UPDATE], handle->dma);
    HAL_NVIC_SetPriority(handle->config.resources.dma_irqn, 1, 0);
    HAL_NVIC_EnableIRQ(handle->config.resources.dma_irqn);
    
    return RT_EOK;
}
/**
 * @brief Configure the GPT timer used as DVP clock and capture engine.
 *
 * This function configures the board-specific timer pins and initializes the
 * GPT peripheral to use PCLK as external clock and HSYNC/HREF as the capture
 * synchronization source for DMA-driven sampling.
 *
 * @param handle is a pointer to the DVP handle.
 *
 * @return Return `RT_EOK` on success. If timer initialization fails,
 *         `-RT_ERROR` is returned.
 */
static int dvp_config_timer(dvp_handle_t *handle)
{
    HAL_PIN_Set(handle->config.resources.pclk_pin_pad,
                handle->config.resources.pclk_pin_func,
                PIN_PULLUP,
                1);
    HAL_PIN_Set(handle->config.resources.hsync_pin_pad,
                handle->config.resources.hsync_pin_func,
                PIN_PULLUP,
                1);

    HAL_RCC_EnableModule(handle->config.resources.timer_rcc_module);
    
    handle->gptim.Instance = handle->config.resources.timer_instance;
    handle->gptim.Init.Prescaler = 0;
    handle->gptim.Init.CounterMode = GPT_COUNTERMODE_DOWN;
    handle->gptim.Init.Period = 0x0;
    
    if (HAL_GPT_Base_Init(&handle->gptim) != HAL_OK)
    {
        LOG_E("DVP timer init failed!");
        return -RT_ERROR;
    }
    
    // Configure external clock source (PCLK via ETR)
    GPT_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = GPT_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = GPT_TRIGGERPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = GPT_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    
    if (HAL_GPT_ConfigClockSource(&handle->gptim, &sClockSourceConfig) != HAL_OK)
    {
        LOG_E("DVP timer clock config failed!");
        return -RT_ERROR;
    }
    
    GPT_SlaveConfigTypeDef sSlaveConfig = {0};
    sSlaveConfig.SlaveMode = GPT_SLAVEMODE_GATED;
    sSlaveConfig.InputTrigger = GPT_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity = GPT_INPUTCHANNELPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    
    if (HAL_GPT_SlaveConfigSynchronization(&handle->gptim, &sSlaveConfig) != HAL_OK)
    {
        LOG_E("DVP timer slave config failed!");
        return -RT_ERROR;
    }
    
    // Configure input capture channel 1 (HSYNC)
    GPT_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = GPT_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = GPT_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = GPT_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    
    if (HAL_GPT_IC_ConfigChannel(&handle->gptim, &sConfigIC, GPT_CHANNEL_1) != HAL_OK)
    {
        LOG_E("DVP timer IC config failed!");
        return -RT_ERROR;
    }
    
    // Enable DMA request
    __HAL_GPT_ENABLE_DMA(&handle->gptim, GPT_DMA_UPDATE);
    
    return RT_EOK;
}
/*
 *******************************************************************************
 * XCLK (sensor master clock) generation via GPTIM2
 *******************************************************************************
 */

/**
 * @brief Start XCLK output on a GPIO pin using GPTIM2 channel 1 PWM.
 *
 * Configures @p pin as GPTIM2_CH1 and generates a 50 % duty-cycle square
 * wave at @p freq Hz. If XCLK is already running the timer is stopped and
 * re-configured before restarting. A 10 ms stabilisation delay is inserted
 * after the PWM starts so the sensor clock is stable before any SCCB access.
 *
 * @param pin  is the PAx pin index (0-based; e.g. 5 for PA5).
 * @param freq is the desired XCLK frequency in Hz.
 */
static void dvp_xclk_start(int pin, uint32_t freq)
{
    HAL_StatusTypeDef status;
    uint32_t timer_clk;
    uint32_t period;
    GPT_OC_InitTypeDef sConfigOC = {0};

    if (s_xclk_initialized)
    {
        HAL_GPT_PWM_Stop(&s_xclk_gptim, GPT_CHANNEL_1);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        s_xclk_initialized = RT_FALSE;
    }

    HAL_PIN_Set(PAD_PA00 + pin, GPTIM2_CH1, PIN_NOPULL, 1);
    HAL_RCC_EnableModule(RCC_MOD_GPTIM2);

#if defined(SOC_SF32LB52X) && SOC_SF32LB52X == 1
    timer_clk = 24000000;
#else
    timer_clk = HAL_RCC_GetPCLKFreq(s_xclk_gptim.core, 1);
#endif

    period = (timer_clk / freq) - 1;
    if (period < 1 || period > 0xFFFF)
    {
        LOG_E("XCLK: frequency %u Hz out of range (timer_clk=%u Hz)",
              (unsigned)freq, (unsigned)timer_clk);
        return;
    }

    s_xclk_gptim.Instance         = hwp_gptim2;
    s_xclk_gptim.Init.Prescaler   = 0;
    s_xclk_gptim.Init.CounterMode = GPT_COUNTERMODE_UP;
    s_xclk_gptim.Init.Period      = period;

    status = HAL_GPT_Base_Init(&s_xclk_gptim);
    if (status != HAL_OK)
    {
        LOG_E("XCLK: GPTIM2 base init failed (%d)", status);
        return;
    }

    sConfigOC.OCMode     = GPT_OCMODE_PWM1;
    sConfigOC.Pulse      = period / 2 + 1;  /* 50 % duty cycle */
    sConfigOC.OCPolarity = GPT_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = GPT_OCFAST_DISABLE;

    status = HAL_GPT_PWM_ConfigChannel(&s_xclk_gptim, &sConfigOC, GPT_CHANNEL_1);
    if (status != HAL_OK)
    {
        LOG_E("XCLK: GPTIM2 PWM config failed (%d)", status);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        return;
    }

    status = HAL_GPT_PWM_Start(&s_xclk_gptim, GPT_CHANNEL_1);
    if (status != HAL_OK)
    {
        LOG_E("XCLK: GPTIM2 PWM start failed (%d)", status);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        return;
    }

    s_xclk_initialized = RT_TRUE;
    rt_thread_mdelay(10);  /* wait for clock to stabilise before SCCB access */
    LOG_I("XCLK: %u Hz on PA%d (period=%u)", (unsigned)freq, pin, (unsigned)period);
}

/**
 * @brief Stop XCLK PWM and restore the pin to GPIO input mode.
 *
 * @param pin is the PAx pin index that was used to output XCLK.
 */
static void dvp_xclk_stop(int pin)
{
    if (!s_xclk_initialized)
        return;

    HAL_GPT_PWM_Stop(&s_xclk_gptim, GPT_CHANNEL_1);
    HAL_GPT_Base_DeInit(&s_xclk_gptim);
    s_xclk_initialized = RT_FALSE;
    HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_NOPULL, 1);
    LOG_I("XCLK: stopped (PA%d)", pin);
}

/* Public function implementations */

/**
 * @brief Initialize a DVP instance.
 *
 * This function stores the user configuration, wires the internal frame
 * forwarder as the hardware callback, allocates the ping-pong DMA buffer,
 * initializes runtime state and configures the required pins, DMA and timer
 * resources.
 *
 * @param self is a pointer to the DVP bus adapter (use `dvp_get_bus_adapter()`
 *        to obtain the singleton).
 *
 * @param cfg is a pointer to a `dvp_config_t` describing hardware resources,
 *        capture mode and initial buffer settings.
 *
 * @return Return `RT_EOK` on success. Otherwise a negative RT-Thread error code
 *         is returned.
 */
int dvp_init(bus_adapter_t *self)
{
    if (self == NULL)
    {
        LOG_E("DVP init: invalid parameters!");
        return -RT_EINVAL;
    }
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    /* Build configuration from Kconfig macros — all DVP hardware parameters
     * belong here in the bus layer, not in the sensor driver above. */
    const dvp_config_t config = {
        .mode                 = BUS_CAPTURE_MODE_JPEG,
        .buffer_size          = 0,
        .frame_buffer         = NULL,
        .pingpong_buffer_size = OV2640_DVP_PINGPONG_BUFFER_SIZE,
        .resources            = DVP_DEFAULT_RESOURCE_CONFIG(OV2640_DVP_VSYNC_PIN),
        .xclk_pin             = OV2640_DVP_XCLK_PIN,
        .xclk_freq            = OV2640_DVP_XCLK_FREQ,
    };

    if (config.resources.timer_instance == RT_NULL ||
        config.resources.dma_instance == RT_NULL ||
        config.resources.data_gpio_instance == RT_NULL ||
        config.resources.data_pin_source_addr == 0)
    {
        LOG_E("DVP init: missing hardware resources!");
        return -RT_EINVAL;
    }

    // Save configuration; the user bus callback is registered separately via dvp_set_frame_callback
    memcpy(&handle->config, &config, sizeof(dvp_config_t));
    
    // Allocate ping-pong buffer
    if(handle->config.pingpong_buffer_size % 2 != 0)
    {
        LOG_W("DVP init: pingpong_buffer_size not even, rounding up");
        handle->config.pingpong_buffer_size += 1;
    }
    if (handle->config.pingpong_buffer_size > sizeof(dvp_pingpong_buffer))
    {
        LOG_E("DVP init: pingpong_buffer_size (%d) exceeds static buffer size (%d)",
              handle->config.pingpong_buffer_size, sizeof(dvp_pingpong_buffer));
        return -RT_EINVAL;
    }
    handle->pingpong_buffer = dvp_pingpong_buffer;
    
    // Initialize state
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;
    handle->capture.enable_capture = 1;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    
    // Configure hardware
    if (dvp_config_data_pins(handle) != 0)
        return -RT_ERROR;
    if (dvp_config_control_pins(handle) != 0)
        return -RT_ERROR;
    if (dvp_config_dma(handle) != 0)
        return -RT_ERROR;
    if (dvp_config_timer(handle) != 0)
        return -RT_ERROR;

    if (config.xclk_pin >= 0 && config.xclk_freq > 0)
        dvp_xclk_start(config.xclk_pin, config.xclk_freq);

    LOG_I("DVP initialized successfully");
    LOG_I("  Mode: %s",
               config.mode == BUS_CAPTURE_MODE_JPEG ? "JPEG" :
               config.mode == BUS_CAPTURE_MODE_RAW ? "RAW" :
               config.mode == BUS_CAPTURE_MODE_YUV422 ? "YUV422" : "RGB565");
    LOG_I("  Buffer size: %d bytes", config.buffer_size);
    LOG_I("  Pingpong buffer: %d bytes", config.pingpong_buffer_size);
    LOG_I("  VSYNC: PA%d (GPIO interrupt)", config.resources.vsync_pin);

    return RT_EOK;
}

/**
 * @brief Deinitialize a DVP instance.
 *
 * This function stops capture hardware, disables the VSYNC interrupt,
 * deinitializes DMA and timer resources and clears driver state. The static
 * ping-pong buffer is not freed.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @return Return `RT_EOK` on success. If `self` is `NULL`, `-RT_EINVAL`
 *         is returned.
 */
int dvp_deinit(bus_adapter_t *self)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    // Stop hardware
    dvp_stop(self);

    if (handle->config.xclk_pin >= 0)
        dvp_xclk_stop(handle->config.xclk_pin);

    // Deinitialize DMA
    HAL_NVIC_DisableIRQ(handle->config.resources.dma_irqn);
    HAL_DMA_DeInit(&handle->dma);

    // Deinitialize Timer
    __HAL_GPT_DISABLE_DMA(&handle->gptim, GPT_DMA_UPDATE);
    __HAL_GPT_DISABLE_IT(&handle->gptim, GPT_IT_UPDATE);
    HAL_GPT_Base_DeInit(&handle->gptim);

    // Ping-pong buffer is at a fixed address, no release needed
    handle->pingpong_buffer = NULL;
    
    // Disable VSYNC interrupt
    rt_pin_irq_enable(handle->config.resources.vsync_pin, PIN_IRQ_DISABLE);
    rt_pin_detach_irq(handle->config.resources.vsync_pin);
    
    // Reset state
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;
    handle->capture.enable_capture = 0;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    
    LOG_I("DVP deinitialized");
    return RT_EOK;
}

//***************************** Control Functions ******************************//
/**
 * @brief Start DVP hardware capture.
 *
 * This function aborts any active DMA, restarts the GPT timer and DMA engine,
 * clears capture state and arms the ping-pong buffer for a new hardware
 * capture cycle.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @return Return `RT_EOK` on success. Otherwise a negative RT-Thread error code
 *         is returned.
 */
int dvp_start(bus_adapter_t *self)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    GPT_TypeDef *timer_instance = dvp_get_timer_instance(handle);

    HAL_DMA_Abort(&handle->dma);
    HAL_GPT_Base_Stop(&handle->gptim);
    __HAL_GPT_SET_COUNTER(&handle->gptim, 0);
    __HAL_GPT_CLEAR_FLAG(&handle->gptim, GPT_FLAG_UPDATE);

    handle->capture.soi_found = (handle->config.mode == BUS_CAPTURE_MODE_JPEG) ? 0 : 1;
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    dvp_reset_jpeg_boundary_state(handle);

    if (HAL_GPT_Base_Start(&handle->gptim) != HAL_OK)
    {
        LOG_E("DVP VSYNC: GPTIM restart failed");
        HAL_DMA_Abort(&handle->dma);
        handle->capture.soi_found = 0;
        handle->capture.enable_capture = 0;
        return -RT_ERROR;
    }
    timer_instance->DIER &= ~GPT_DIER_UDE;
    for (int i = 0; i < 100; i++) { __NOP(); }
    timer_instance->DIER |= GPT_DIER_UDE;

    if (HAL_DMA_Start_IT(&handle->dma,
                         handle->config.resources.data_pin_source_addr,
                         (uint32_t)handle->pingpong_buffer,
                         (uint32_t)handle->config.pingpong_buffer_size) != HAL_OK)
    {
        LOG_E("DVP VSYNC: DMA restart failed");
        handle->capture.soi_found = 0;
        handle->capture.enable_capture = 0;
        handle->capture.capture_started = 0;
        return -RT_ERROR;
    }
    handle->capture.capture_started = 1;

    return RT_EOK;
}

/**
 * @brief Stop DVP hardware capture.
 *
 * This function stops the GPT timer, aborts the active DMA transfer and
 * clears the software capture-enable flag. Hardware pins and interrupt
 * attachments are left in place; use `dvp_deinit` to tear those down.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @return Return `RT_EOK` on success. If `self` is `NULL`, `-RT_EINVAL`
 *         is returned.
 */
int dvp_stop(bus_adapter_t *self)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    // Stop timer and DMA
    HAL_GPT_Base_Stop(&handle->gptim);
    HAL_DMA_Abort(&handle->dma);
    
    // Clear capture flag
    handle->capture.enable_capture = 0;
    handle->capture.capture_started = 0;
    
    LOG_D("DVP hardware stopped");
    return RT_EOK;
}

/**
 * @brief Arm the driver for one frame capture.
 *
 * This function optionally updates the user frame buffer, resets capture state
 * and enables the capture path. For JPEG mode it also calls `dvp_start()` to
 * restart the DMA/timer pair so that SOI detection begins immediately. For
 * fixed-size modes the hardware stays running; `dvp_vsync_irq_handler()` will
 * call `dvp_start()` on the next VSYNC rising edge.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @param new_buffer is an optional pointer to a new frame buffer. Pass `NULL`
 *        to keep the previously configured buffer.
 *
 * @param buffer_size is the size of `new_buffer` in bytes. Must be non-zero
 *        when `new_buffer` is not `NULL`.
 *
 * @return Return `RT_EOK` on success. Otherwise a negative RT-Thread error code
 *         is returned.
 */
int dvp_start_capture(bus_adapter_t *self, void *new_buffer, uint32_t buffer_size)
{

    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    // If new buffer specified, update configuration
    if (new_buffer != NULL)
    {
        if (buffer_size == 0)
        {
            LOG_E("DVP error: buffer_size must be provided when new_buffer is not NULL");
            return -RT_EINVAL;
        }
        
        handle->config.frame_buffer = (uint8_t *)new_buffer;
        handle->config.buffer_size = buffer_size;
        //LOG_D("DVP capture buffer updated to 0x%08X, size: %d bytes", 
        //           (uint32_t)new_buffer, buffer_size);
    }
    
    // Only set flags, don't operate hardware (hardware started by dvp_start)
    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;
    handle->capture.enable_capture = 1;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    rt_hw_interrupt_enable(level);

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        return dvp_start(self);
    }
    
    //LOG_D("DVP capture enabled.");
    return RT_EOK;
}

/**
 * @brief Re-arm capture state for the next frame without restarting the hardware stream.
 *
 * This function is designed to be called from inside the frame callback to
 * queue the next capture without stopping the running DMA. It resets software
 * capture counters and flags, optionally switching to a new user buffer.
 *
 * For JPEG mode it simply re-arms state flags (DMA is already circularly
 * running). For RAW/RGB565/YUV422 modes it sets `enable_capture = 1` and
 * `capture_started = 0` so that `dvp_vsync_irq_handler()` will call
 * `dvp_start()` safely from its own interrupt context on the next VSYNC edge.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @param new_buffer is an optional pointer to a new frame buffer. Pass `NULL`
 *        to keep the previously configured buffer.
 *
 * @param buffer_size is the size of `new_buffer` in bytes. Must be non-zero
 *        when `new_buffer` is not `NULL`.
 *
 * @return Return `RT_EOK` on success. Otherwise a negative RT-Thread error code
 *         is returned.
 */
int dvp_rearm_capture(bus_adapter_t *self, void *new_buffer, uint32_t buffer_size)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (new_buffer != NULL)
    {
        if (buffer_size == 0)
        {
            LOG_E("DVP error: buffer_size must be provided when new_buffer is not NULL");
            return -RT_EINVAL;
        }

        handle->config.frame_buffer = (uint8_t *)new_buffer;
        handle->config.buffer_size = buffer_size;
    }

    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;   /* always 0: JPEG waits for SOI, raw waits for VSYNC */
    handle->capture.enable_capture = 1;
    handle->capture.capture_started = (handle->config.mode == BUS_CAPTURE_MODE_JPEG) ? 1 : 0;
    dvp_reset_jpeg_boundary_state(handle);
    rt_hw_interrupt_enable(level);

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        return RT_EOK;
    }

    /*
     * Non-JPEG (RGB565 / YUV422 / RAW) stream mode:
     * DO NOT call dvp_start() here — this function is invoked from within the
     * DMA ISR (user bus callback chain), and dvp_start() calls HAL_DMA_Abort +
     * HAL_DMA_Start_IT which are not safe to call while the DMA ISR is still
     * executing.  Instead, just set enable_capture = 1 and capture_started = 0
     * above.  The dvp_vsync_irq_handler() will see these flags on the next
     * VSYNC rising edge and call dvp_start() from its own (safe) interrupt
     * context, ensuring frame-boundary alignment.
     */
    return RT_EOK;
}

/**
 * @brief Abort the current frame capture.
 *
 * This function clears the software capture-enable and frame-ready flags
 * without touching the hardware; the DMA and timer keep running, which
 * allows the driver to resume cleanly on the next `dvp_start_capture` call.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @return Return `RT_EOK` on success. If `self` is `NULL`, `-RT_EINVAL`
 *         is returned.
 */
int dvp_abort_capture(bus_adapter_t *self)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    // Only clear flags, hardware continues running (continue detecting SOI)
    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.enable_capture = 0;
    handle->capture.frame_ready = 0;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    rt_hw_interrupt_enable(level);
    
    LOG_D("DVP capture aborted.");
    return RT_EOK;
}

/**
 * @brief Get the size of the captured frame.
 *
 * This function returns the number of bytes written to the user frame buffer
 * for the active or most recently completed capture.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @return Return the captured frame size in bytes. If `self` is `NULL`, `0`
 *         is returned.
 */
uint32_t dvp_get_frame_size(bus_adapter_t *self)
{
    if (self == NULL)
        return 0;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    return handle->capture.current_size;
}

/**
 * @brief Reset software capture state (internal helper).
 *
 * Clears frame-ready related state and re-enables capture so the next
 * hardware event can fill the configured frame buffer. Not exposed in the
 * public header; external callers should use `dvp_start_capture` instead.
 *
 * @param handle is a pointer to the raw DVP handle.
 *
 * @return This function does not return a value.
 */
static void dvp_reset_capture(dvp_handle_t *handle)
{
    if (handle == NULL)
        return;
    
    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;
    handle->capture.enable_capture = 1;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    rt_hw_interrupt_enable(level);
}

/**
 * @brief Resize the DMA ping-pong buffer.
 *
 * This function stops the active DMA and timer pair, redirects the
 * hardware to the new size within the statically allocated pool and
 * updates the stored buffer-size field. DVP must be stopped before
 * calling this function; it is not safe to call while capture is running.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @param new_size is the requested new ping-pong buffer size in bytes.
 *        The value will be rounded up to the nearest even number. It must
 *        not exceed the size of the internal static buffer.
 *
 * @return Return `RT_EOK` on success. Otherwise a negative RT-Thread error code
 *         is returned.
 */
int dvp_set_pingpong_size(bus_adapter_t *self, uint32_t new_size)
{
    if (self == NULL || new_size == 0)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (new_size % 2 != 0)
        new_size += 1;

    if (new_size > sizeof(dvp_pingpong_buffer))
    {
        LOG_E("dvp_set_pingpong_size: %d bytes exceeds static buffer size (%d)",
              new_size, sizeof(dvp_pingpong_buffer));
        return -RT_EINVAL;
    }

    /* Stop DMA and timer before switching buffer */
    HAL_DMA_Abort(&handle->dma);
    HAL_GPT_Base_Stop(&handle->gptim);

    handle->pingpong_buffer = dvp_pingpong_buffer;
    handle->config.pingpong_buffer_size = new_size;
    LOG_I("Pingpong buffer resized to %d bytes", new_size);
    LOG_I("BUFFER ADDR: 0x%08X", (uint32_t)handle->pingpong_buffer);
    return RT_EOK;
}

/**
 * @brief Register a generic frame-ready callback.
 *
 * The callback is invoked directly from the DMA ISR / timer-thread context
 * when a complete frame has been assembled in the user frame buffer.
 *
 * @param self      is a pointer to the DVP bus adapter (unused, provided for
 *                  interface symmetry).
 *
 * @param callback  is the callback function to invoke when a frame is ready.
 *                  Pass `NULL` to deregister an existing callback.
 *
 * @param user_data is an opaque pointer forwarded to `callback` as its third
 *                  argument.
 *
 * @return Return `BUS_OK` on success.
 */
int dvp_set_frame_callback(bus_adapter_t *self,
                           bus_frame_ready_callback_t callback,
                           void *user_data)
{
    (void)self;
    s_user_frame_callback = callback;
    s_user_frame_callback_data = user_data;
    return BUS_OK;
}

/**
 * @brief Replace the user frame buffer pointer and size.
 *
 * This function swaps the destination buffer for incoming frame data without
 * resetting capture state or restarting the hardware. It is intended for
 * buffer-rotation schemes where the caller provides a fresh buffer before the
 * next `dvp_start_capture` or `dvp_rearm_capture` call.
 *
 * @param self is a pointer to the DVP bus adapter.
 *
 * @param buffer is the new frame buffer pointer. Must not be `NULL` (validated
 *        by the caller).
 *
 * @param size is the size of `buffer` in bytes.
 *
 * @return Return `BUS_OK` on success. If `self` is `NULL`, `-RT_EINVAL`
 *         is returned.
 */
int dvp_update_buffer(bus_adapter_t *self, void *buffer, uint32_t size)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;
    handle->config.frame_buffer = (uint8_t *)buffer;
    handle->config.buffer_size  = size;
    return BUS_OK;
}

/**
 * @brief Change the DVP capture mode at runtime.
 *
 * Updates the cached handle configuration. The caller is expected to have
 * already stopped the bus (`bus_adapter_stop`) before invoking this op;
 * this function only mutates configuration state.
 *
 * @param self is a pointer to the DVP bus adapter.
 * @param mode is the requested capture mode.
 *
 * @return Return `BUS_OK` on success.
 *         Return `BUS_ERR_INVALID` if `self` is NULL or `mode` is unknown.
 */
int dvp_set_mode(bus_adapter_t *self, bus_capture_mode_t mode)
{
    if (self == NULL)
        return BUS_ERR_INVALID;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (mode != BUS_CAPTURE_MODE_JPEG   &&
        mode != BUS_CAPTURE_MODE_RAW    &&
        mode != BUS_CAPTURE_MODE_YUV422 &&
        mode != BUS_CAPTURE_MODE_RGB565)
        return BUS_ERR_INVALID;

    handle->config.mode = mode;
    return BUS_OK;
}
//***************************** Control Functions ******************************//


//***************************** Bus Adapter Registration **********************//

/**
 * @brief Dump DVP hardware diagnostic state to the log.
 *
 * Called via bus_adapter_dump_state() when the upper layer detects a capture
 * timeout and needs bus-level diagnostics. Not safe to call from ISR context.
 */
void dvp_dump_state(bus_adapter_t *self)
{
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;
    uint32_t dma_counter = 0;
    uint32_t gpt_cnt = 0;
    uint32_t gpt_cr1 = 0;
    uint32_t gpt_dier = 0;
    GPT_TypeDef *gpt = (GPT_TypeDef *)handle->gptim.Instance;
    int vsync_level = rt_pin_read(handle->config.resources.vsync_pin);

    if (gpt != RT_NULL)
    {
        gpt_cnt = gpt->CNT;
        gpt_cr1 = gpt->CR1;
        gpt_dier = gpt->DIER;
    }
    if (handle->dma.Instance != RT_NULL)
    {
        dma_counter = __HAL_DMA_GET_COUNTER(&handle->dma);
    }

    LOG_W("  bus(dvp): mode=%d cfg_buf=%u",
        (int)handle->config.mode,
        (unsigned int)handle->config.buffer_size);
    LOG_W("  capture: ready=%u enable=%u soi=%u current=%u",
        (unsigned int)handle->capture.frame_ready,
        (unsigned int)handle->capture.enable_capture,
        (unsigned int)handle->capture.soi_found,
        (unsigned int)handle->capture.current_size);
    LOG_W("  mem: frame_buf=0x%08X pingpong=0x%08X pp_size=%u",
        (unsigned int)(rt_ubase_t)handle->config.frame_buffer,
        (unsigned int)(rt_ubase_t)handle->pingpong_buffer,
        (unsigned int)handle->config.pingpong_buffer_size);
    LOG_W("  dma: state=%d err=0x%08X req=%u counter=%u",
        (int)HAL_DMA_GetState(&handle->dma),
        (unsigned int)HAL_DMA_GetError(&handle->dma),
        (unsigned int)handle->dma.Init.Request,
        (unsigned int)dma_counter);
    LOG_W("  gpt: state=%d cnt=%u cr1=0x%08X dier=0x%08X",
        (int)handle->gptim.State,
        (unsigned int)gpt_cnt,
        (unsigned int)gpt_cr1,
        (unsigned int)gpt_dier);
    LOG_W("  vsync: pin=%u level=%d",
        (unsigned int)handle->config.resources.vsync_pin,
        vsync_level);
    {
        uint32_t dmac_isr  = hwp_dmac1->ISR;
        uint32_t dmac_ifcr = hwp_dmac1->IFCR;
        uint32_t dmac_ccr  = hwp_dmac1->CCR1;
        uint32_t dmac_cndtr = hwp_dmac1->CNDTR1;
        LOG_W("  dmac: ISR=0x%08X IFCR=0x%08X CCR=0x%08X CNDTR=%u",
            (unsigned int)dmac_isr,
            (unsigned int)dmac_ifcr,
            (unsigned int)dmac_ccr,
            (unsigned int)dmac_cndtr);
    }
}

/*
 * The public `dvp_*` functions above already match `bus_adapter_ops_t` exactly,
 * so the ops table just references them directly — no trampolines needed.
 */
static const bus_adapter_ops_t s_dvp_bus_ops = {
    .init              = dvp_init,
    .deinit            = dvp_deinit,
    .start             = dvp_start,
    .stop              = dvp_stop,
    .set_frame_callback = dvp_set_frame_callback,
    .start_capture     = dvp_start_capture,
    .rearm_capture     = dvp_rearm_capture,
    .abort_capture     = dvp_abort_capture,
    .update_buffer     = dvp_update_buffer,
    .set_pingpong_size = dvp_set_pingpong_size,
    .get_frame_size    = dvp_get_frame_size,
    .set_mode          = dvp_set_mode,
    .dump_state        = dvp_dump_state,
};

static bus_adapter_t s_dvp_bus_adapter = {
    .name = DVP_BUS_ADAPTER_NAME,
    .type = BUS_TYPE_DVP,
    .ops  = &s_dvp_bus_ops,
    .priv = &s_dvp_handle,
};

bus_adapter_t *dvp_get_bus_adapter(void)
{
    return &s_dvp_bus_adapter;
}

static int dvp_bus_adapter_register(void)
{
    int ret = bus_adapter_register(&s_dvp_bus_adapter);
    if (ret != BUS_OK)
    {
        LOG_E("DVP bus adapter register failed: %d", ret);
    }
    return ret;
}
INIT_BOARD_EXPORT(dvp_bus_adapter_register);
//***************************** Bus Adapter Registration **********************//