# take_photo_to_sdcard (Streaming) Example

[中文](README.md) | [English](README_EN.md)

## Overview

This example demonstrates how to use the OV2640 component's **handle-layer streaming API** (`camera_handle.h`) to capture JPEG frames using double-buffered continuous DMA and **write each frame to the SD card immediately** as it arrives — a true "capture-while-saving" pipeline.

Unlike `camera_capture_single`, the streaming API keeps DMA ping-pong transfers running in the background. The application simply polls for ready frames, resulting in lower per-frame latency and better throughput for burst captures.

Call sequence:

1. `camera_handler_instance_init`
2. `camera_init`
3. `camera_change_settings` (`PIXFORMAT_JPEG` + framesize + quality)
4. Allocate two PSRAM frame buffers, call `camera_start_stream`
5. Loop `camera_get_stream_frame`; write each frame to `/photo/photo_NNN.jpg` immediately
6. `camera_stop_stream`
7. `camera_deinit`

> Pin muxing for SCCB / DVP / XCLK is performed by the OV2640 driver during initialization. The application **does not** (and must not) call `HAL_PIN_Set()`.

## Hardware Requirements

- OV2640 camera module wired according to the default pins documented in the OV2640 component README
- A SPI / SDIO SD card registered as the `sd0` device in RT-Thread
- At least 4 MB PSRAM (the example allocates two JPEG frame buffers from a PSRAM heap)

## Usage

After boot the example initializes the PSRAM heap and mounts the SD card automatically:

```
OV2640 Camera Take Photo to SD Card Example
mount fs on tf card to / success
```

Once mounted, run from the MSH console:

```
msh> take_photo <framesize> <quality> <count>
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `framesize` | QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA | Resolution |
| `quality`   | 0 ~ 63 | JPEG quality (0 = best, 63 = most compressed) |
| `count`     | ≥ 1 | Number of frames to capture |

Example:

```
msh> take_photo VGA 10 3
Stream start: framesize=VGA, quality=10, buffer=307200 bytes x2 @ 0x6XXXXXXX / 0x6XXXXXXX
Frame[0] seq=1 buf=0 size=28456 wait=42 ms
Saved /photo/photo_001.jpg (28456 bytes)
Frame[1] seq=2 buf=1 size=28612 wait=41 ms
Saved /photo/photo_002.jpg (28612 bytes)
Frame[2] seq=3 buf=0 size=28391 wait=43 ms
Saved /photo/photo_003.jpg (28391 bytes)
```

## Output Files

Each captured frame is saved immediately as:

```
/photo/photo_001.jpg
/photo/photo_002.jpg
/photo/photo_003.jpg
...
```

Plug the SD card into a host PC and open the `.jpg` files directly to verify the result.

## Notes

- This example supports **JPEG pixel format only**. For RGB565 / YUV422 / RAW8 capture, use the corresponding commands in `cam_capture.c`.
- Streaming allocates **two** PSRAM frame buffers (ping-pong DMA). Each buffer is sized as `width × height`, capped at 2 MB.
- If the SD card write speed cannot keep up with the frame rate, the driver overwrites the oldest unconsumed frame. A `dropped N frame(s)` message is printed after the command finishes for diagnostics.
- Lower `quality` produces sharper images but larger files; higher `quality` increases compression and shrinks file size.
- `camera_change_settings` already inserts roughly 500 ms of AEC/AWB settle time internally — no extra delay is required from the caller.
- If you see `sd card not found` or `mount fs ... fail`: make sure the SD card is FAT-formatted, the `sd0` device is registered, and the SPI / clock pins are wired correctly.

## Related Documents

- Chinese version: [README.md](README.md)
- Component main documentation: [../../README_EN.md](../../README_EN.md)
