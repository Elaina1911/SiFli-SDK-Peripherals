# take_photo_to_sdcard Example

[中文](README.md) | [English](README_EN.md)

## Overview

This example demonstrates how to use the OV2640 component's **handle-layer API** (`camera_handle.h`) to capture JPEG frames and write them to a SD card filesystem mounted at `/` (default output directory: `/photo`).

The call sequence is identical to the [`take_photo`](../take_photo/README_EN.md) example:

1. `camera_handler_instance_init`
2. `camera_init`
3. `camera_change_settings` (`PIXFORMAT_JPEG` + framesize + quality)
4. Loop `camera_capture_single`; on success, write one `.jpg` file
5. `camera_deinit`

> Pin muxing for SCCB / DVP / XCLK is performed by the OV2640 driver during initialization. The application **does not** (and must not) call `HAL_PIN_Set()`.

## Hardware Requirements

- OV2640 camera module wired according to the default pins documented in the OV2640 component README
- A SPI / SDIO SD card registered as the `sd0` device in RT-Thread
- At least 4 MB PSRAM available (the example allocates the JPEG buffer from a PSRAM heap)

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
JPEG capture: framesize=VGA, quality=10, buffer=307200 bytes @ 0x6XXXXXXX
Saved /photo/photo_001.jpg (28456 bytes)
Saved /photo/photo_002.jpg (28612 bytes)
Saved /photo/photo_003.jpg (28391 bytes)
```

## Output Files

Captured photos are saved as:

```
/photo/photo_001.jpg
/photo/photo_002.jpg
/photo/photo_003.jpg
...
```

Plug the SD card into a host PC and open the `.jpg` files directly to verify the result.

## Notes

- JPEG output size is variable; the example estimates the buffer as `width × height` and clamps it to `[64 KB, 2 MB]`. Allocate enough PSRAM for high-resolution modes such as UXGA.
- Lower `quality` produces a sharper image but a larger file; higher `quality` increases compression and shrinks the file.
- After switching format or resolution, `camera_change_settings` already inserts roughly 500 ms of AEC/AWB settle time internally - no extra delay is required from the caller.
- If you see `sd card not found` or `mount fs ... fail`: make sure the SD card is FAT-formatted, the `sd0` device is registered, and the SPI / clock pins are wired correctly.

## Related Documents

- Chinese version: [README.md](README.md)
- Component main documentation: [../../README_EN.md](../../README_EN.md)
- Sibling RGB565 single-shot example: [`take_photo`](../take_photo/README_EN.md)
