/******************************************************************************
 * @file    main.c
 * @brief   OV2640 take_photo_to_sdcard example - 流式（streaming）JPEG 采集，
 *          边采集边把每一帧写入 SD 卡。
 *
 * 仅支持 JPEG 像素格式，使用 camera_handle.h 的双缓冲流式 API：
 *   camera_handler_instance_init()  - 绑定设备名 + device_ops
 *   camera_init()                   - 打开 RT-Thread 设备
 *   camera_change_settings()        - 配置 JPEG + framesize + quality
 *   camera_start_stream()           - 启动双缓冲 DMA 连续采集
 *   camera_get_stream_frame() loop  - 取一帧 -> 立刻写 SD 卡 /photo/photo_NNN.jpg
 *   camera_stop_stream()            - 停止流式采集
 *   camera_deinit()                 - 关闭设备
 *
 * Note: low-level pin muxing for SCCB / DVP / XCLK is performed by the
 * OV2640 driver itself, so this example does not call HAL_PIN_Set().
 *****************************************************************************/

#include "rtthread.h"
#include "bf0_hal.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <rtdevice.h>
#include "mem_section.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "spi_msd.h"
#include "ov2640.h"
#include "camera_handle.h"

/* ------------------------------------------------------------------ *
 * PSRAM heap - holds the JPEG frame buffer (internal SRAM is too
 * small for VGA-and-above JPEG output).
 * ------------------------------------------------------------------ */
static uint8_t psram_heap_pool[4096 * 1024] L2_RET_BSS_SECT(psram_heap_pool);
static struct rt_memheap psram_memheap;

#define JPEG_MIN_BUFFER_SIZE    (64 * 1024)
#define JPEG_MAX_BUFFER_SIZE    (2 * 1024 * 1024)
#define PHOTO_DIR               "/photo"

/**
 * @brief Initialize the PSRAM heap pool.
 *
 * @return Return 0 on success (fixed value).
 */
int psram_heap_init(void)
{
    rt_memheap_init(&psram_memheap, "psram_heap", (void *)psram_heap_pool,
                    sizeof(psram_heap_pool));
    return 0;
}

/**
 * @brief Allocate memory from the PSRAM heap.
 *
 * @param size is the number of bytes to allocate.
 *
 * @return Return a pointer on success, RT_NULL on failure.
 */
void *psram_heap_malloc(uint32_t size)
{
    return rt_memheap_alloc(&psram_memheap, size);
}

/**
 * @brief Release memory previously returned by psram_heap_malloc().
 *
 * @param p is the pointer to free.
 */
void psram_heap_free(void *p)
{
    rt_memheap_free(p);
}

/* ------------------------------------------------------------------ *
 * SD card mount
 * ------------------------------------------------------------------ */

/**
 * @brief Locate the SD card device and mount it as the root FAT volume.
 */
void sdcard_init(void)
{
    rt_device_t msd = rt_device_find("sd0");
    if (msd == RT_NULL)
    {
        rt_kprintf("sd card not found\n");
        return;
    }

    if (dfs_mount("sd0", "/", "elm", 0, 0) != 0)
    {
        rt_kprintf("mount fs on tf card to / fail\n");
        rt_kprintf("sd card might not be formatted or is corrupted.\n");
        return;
    }

    rt_kprintf("mount fs on tf card to / success\n");
}

/* ------------------------------------------------------------------ *
 * Frame-size parsing and buffer-size computation
 * ------------------------------------------------------------------ */

/**
 * @brief Convert a frame-size name (e.g. "VGA") into the framesize_t enum.
 *
 * @param str is the frame-size string to look up.
 *
 * @return Return the matching framesize_t, or FRAMESIZE_INVALID if unknown.
 */
static framesize_t format_string_to_framesize(const char *str)
{
    if (strcmp(str, "QQVGA") == 0)      return FRAMESIZE_QQVGA;
    else if (strcmp(str, "QCIF") == 0)  return FRAMESIZE_QCIF;
    else if (strcmp(str, "QVGA") == 0)  return FRAMESIZE_QVGA;
    else if (strcmp(str, "CIF") == 0)   return FRAMESIZE_CIF;
    else if (strcmp(str, "VGA") == 0)   return FRAMESIZE_VGA;
    else if (strcmp(str, "SVGA") == 0)  return FRAMESIZE_SVGA;
    else if (strcmp(str, "XGA") == 0)   return FRAMESIZE_XGA;
    else if (strcmp(str, "HD") == 0)    return FRAMESIZE_HD;
    else if (strcmp(str, "SXGA") == 0)  return FRAMESIZE_SXGA;
    else if (strcmp(str, "UXGA") == 0)  return FRAMESIZE_UXGA;
    else                                return FRAMESIZE_INVALID;
}

/**
 * @brief Resolve a framesize_t into pixel width and height.
 *
 * @param size   is the input framesize_t.
 * @param width  is the output pointer that receives the width in pixels.
 * @param height is the output pointer that receives the height in pixels.
 *
 * @return Return RT_EOK on success, or -RT_EINVAL when @p size is unknown.
 */
static int framesize_to_resolution(framesize_t size, uint16_t *width, uint16_t *height)
{
    if (width == RT_NULL || height == RT_NULL)
    {
        return -RT_EINVAL;
    }

    switch (size)
    {
        case FRAMESIZE_QQVGA: *width = 160;  *height = 120;  break;
        case FRAMESIZE_QCIF:  *width = 176;  *height = 144;  break;
        case FRAMESIZE_QVGA:  *width = 320;  *height = 240;  break;
        case FRAMESIZE_CIF:   *width = 400;  *height = 296;  break;
        case FRAMESIZE_VGA:   *width = 640;  *height = 480;  break;
        case FRAMESIZE_SVGA:  *width = 800;  *height = 600;  break;
        case FRAMESIZE_XGA:   *width = 1024; *height = 768;  break;
        case FRAMESIZE_HD:    *width = 1280; *height = 720;  break;
        case FRAMESIZE_SXGA:  *width = 1280; *height = 1024; break;
        case FRAMESIZE_UXGA:  *width = 1600; *height = 1200; break;
        default:
            return -RT_EINVAL;
    }

    return RT_EOK;
}

/**
 * @brief Estimate a reasonable JPEG output buffer size for a given resolution.
 *
 * JPEG output length is variable; we use width * height as a rough upper
 * bound (about 1 byte per pixel for typical photographic content) and clamp
 * it to the [JPEG_MIN_BUFFER_SIZE, JPEG_MAX_BUFFER_SIZE] range.
 *
 * @param size is the target framesize_t.
 *
 * @return Return the buffer size in bytes (never zero).
 */
static rt_size_t calc_jpeg_buffer_size(framesize_t size)
{
    uint16_t width  = 0;
    uint16_t height = 0;
    if (framesize_to_resolution(size, &width, &height) != RT_EOK)
    {
        return JPEG_MAX_BUFFER_SIZE;
    }

    rt_size_t buf_size = (rt_size_t)width * (rt_size_t)height;
    if (buf_size < JPEG_MIN_BUFFER_SIZE)
    {
        buf_size = JPEG_MIN_BUFFER_SIZE;
    }
    if (buf_size > JPEG_MAX_BUFFER_SIZE)
    {
        buf_size = JPEG_MAX_BUFFER_SIZE;
    }
    return buf_size;
}

/* ------------------------------------------------------------------ *
 * SD-card output path helpers
 * ------------------------------------------------------------------ */

/**
 * @brief Create the photo directory if it does not exist.
 *
 * @return Return RT_EOK on success or when the directory already exists.
 */
static int ensure_photo_dir(void)
{
    int ret = mkdir(PHOTO_DIR, 0);
    if (ret == 0)
    {
        return RT_EOK;
    }

    int err = rt_get_errno();
    if (ret < 0 && (err == EEXIST || err == -EEXIST))
    {
        return RT_EOK;
    }

    return -RT_ERROR;
}

/* ------------------------------------------------------------------ *
 * MSH command: take_photo
 * ------------------------------------------------------------------ */

/**
 * @brief MSH command: 流式采集 JPEG 帧，每取到一帧立即写入 SD 卡
 *        /photo/photo_NNN.jpg。
 *
 * Usage:
 *   take_photo <framesize> <quality> <count>
 *
 *   framesize : QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA
 *   quality   : JPEG quality (0 = best, 63 = most compressed)
 *   count     : number of frames to capture (>= 1)
 *
 * Example:
 *   take_photo VGA 10 3
 *
 * @param argc is the argument count (4 including the command name).
 * @param argv is the argument vector.
 */
void take_photo(int argc, char **argv)
{
    camera_handler_instance_t      camera_instance;
    camera_handler_all_input_arg_t input_arg;
    camera_capture_config_t        cfg;
    camera_stream_config_t         stream_cfg;
    camera_stream_frame_t          frame;
    camera_handle_status_t         status;
    uint8_t                       *buffers[2] = { RT_NULL, RT_NULL };
    rt_bool_t                      stream_started = RT_FALSE;
    rt_size_t                      buffer_size;
    int                            quality;
    int                            count;

    if (argc != 4)
    {
        rt_kprintf("Usage: take_photo <framesize> <quality> <count>\n");
        rt_kprintf("Framesize options: QQVGA, QCIF, QVGA, CIF, VGA, SVGA, XGA, HD, SXGA, UXGA\n");
        rt_kprintf("quality: 0 (highest) to 63 (lowest)\n");
        rt_kprintf("count: number of photos to capture (>=1)\n");
        rt_kprintf("Example: take_photo VGA 10 3\n");
        return;
    }

    framesize_t framesize = format_string_to_framesize(argv[1]);
    if (framesize == FRAMESIZE_INVALID)
    {
        rt_kprintf("Unsupported framesize: %s\n", argv[1]);
        return;
    }

    quality = atoi(argv[2]);
    if (quality < 0 || quality > 63)
    {
        rt_kprintf("Quality must be between 0 and 63\n");
        return;
    }

    count = atoi(argv[3]);
    if (count <= 0)
    {
        rt_kprintf("Count must be >= 1\n");
        return;
    }

    if (ensure_photo_dir() != RT_EOK)
    {
        rt_kprintf("Failed to create or access %s\n", PHOTO_DIR);
        return;
    }

    /* 1) Prepare the camera handler instance. */
    input_arg.device_name = CAMERA_DEFAULT_DEVICE_NAME;     /* "ov2640" */
    input_arg.device_ops  = ov2640_get_device_ops();
    status = camera_handler_instance_init(&camera_instance, &input_arg);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to initialize camera handler (%d)\n", status);
        return;
    }

    /* 2) Open the underlying RT-Thread camera device. */
    status = camera_init(&camera_instance);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to open camera device (%d)\n", status);
        return;
    }

    /* 3) Configure JPEG + framesize + quality.  The handle layer inserts
     *    a 500 ms AEC/AWB settle delay internally. */
    cfg.pixformat = PIXFORMAT_JPEG;
    cfg.framesize = framesize;
    cfg.quality   = (uint8_t)quality;
    status = camera_change_settings(&camera_instance, &cfg);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to configure camera (%d)\n", status);
        goto close_camera;
    }

    /* 4) Allocate two PSRAM frame buffers for ping-pong streaming. */
    buffer_size = calc_jpeg_buffer_size(framesize);
    buffers[0] = psram_heap_malloc(buffer_size);
    buffers[1] = psram_heap_malloc(buffer_size);
    if (buffers[0] == RT_NULL || buffers[1] == RT_NULL)
    {
        rt_kprintf("Failed to allocate stream buffers (%u bytes each)\n",
                   (unsigned int)buffer_size);
        goto free_buffers;
    }

    rt_kprintf("Stream start: framesize=%s, quality=%d, buffer=%u bytes x2 @ %p / %p\n",
               argv[1], quality, (unsigned int)buffer_size, buffers[0], buffers[1]);

    /* 5) Start continuous double-buffered streaming. */
    stream_cfg.buffers[0]  = buffers[0];
    stream_cfg.buffers[1]  = buffers[1];
    stream_cfg.buffer_size = buffer_size;
    status = camera_start_stream(&camera_instance, &stream_cfg);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to start stream (%d)\n", status);
        goto free_buffers;
    }
    stream_started = RT_TRUE;

    /* 6) Pull `count` frames from the ready queue and write each one
     *    directly to the SD card. */
    for (int photo_idx = 0; photo_idx < count; photo_idx++)
    {
        rt_tick_t start_tick = rt_tick_get();
        status = camera_get_stream_frame(&camera_instance, &frame,
                                         5 * RT_TICK_PER_SECOND);
        if (status != CAMERA_OK)
        {
            rt_kprintf("Stream frame timeout/error: index=%d status=%d\n",
                       photo_idx, status);
            break;
        }

        if (frame.frame_size == 0 || frame.buffer == RT_NULL)
        {
            rt_kprintf("Empty stream frame at index=%d, skip\n", photo_idx);
            continue;
        }

        long wait_ms = (long)(rt_tick_get() - start_tick) * 1000L / RT_TICK_PER_SECOND;
        rt_kprintf("Frame[%d] seq=%lu buf=%u size=%u wait=%ld ms\n",
                   photo_idx,
                   (unsigned long)frame.sequence,
                   (unsigned int)frame.buffer_index,
                   (unsigned int)frame.frame_size,
                   wait_ms);

        char file_path[64];
        rt_snprintf(file_path, sizeof(file_path),
                    "%s/photo_%03d.jpg", PHOTO_DIR, photo_idx + 1);
        int fd = open(file_path, O_WRONLY | O_CREAT | O_TRUNC, 0);
        if (fd < 0)
        {
            rt_kprintf("Failed to open %s for writing\n", file_path);
            continue;
        }

        int written = write(fd, frame.buffer, frame.frame_size);
        close(fd);
        if (written != (int)frame.frame_size)
        {
            rt_kprintf("Write failed for %s (%d/%u bytes)\n",
                       file_path, written, (unsigned int)frame.frame_size);
            continue;
        }

        rt_kprintf("Saved %s (%u bytes)\n", file_path,
                   (unsigned int)frame.frame_size);
    }

    if (camera_instance.stream.dropped_count != 0)
    {
        rt_kprintf("Stream dropped %lu frame(s) due to slow consumer\n",
                   (unsigned long)camera_instance.stream.dropped_count);
    }

    camera_stop_stream(&camera_instance);
    stream_started = RT_FALSE;

free_buffers:
    if (stream_started)
    {
        camera_stop_stream(&camera_instance);
    }
    if (buffers[0] != RT_NULL) psram_heap_free(buffers[0]);
    if (buffers[1] != RT_NULL) psram_heap_free(buffers[1]);

close_camera:
    camera_deinit(&camera_instance);
}
MSH_CMD_EXPORT(take_photo, Stream JPEG photo(s) using ov2640 and save to SD card);

/**
 * @brief Program entry point: initialize the PSRAM heap, mount the SD card
 *        and idle, waiting for MSH commands.
 *
 * @return Return 0 (never actually returns; the main loop spins forever).
 */
int main(void)
{
    rt_kprintf("OV2640 Camera Take Photo to SD Card Example\n");
    psram_heap_init();
    sdcard_init();

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
