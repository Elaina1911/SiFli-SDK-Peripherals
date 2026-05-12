# take_photo Example

[中文](README.md) | [English](README_EN.md)

## Overview

This example captures raw RGB565 frames from the `ov2640` device and prints the frame buffer address to the console. The captured pixels can then be exported from PSRAM using SDK tools and viewed as an image on the host.

This example is located in [examples/take_photo](examples/take_photo).

## Usage

Run in MSH:

```
msh> take_photo <framesize> <count>
```

Parameters:
- **framesize**: resolution — one of QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA
- **count**: number of frames to capture (>= 1)

Example:

```
msh> take_photo QVGA 1
```

## Sample Output

```
RGB565 capture: 320x240, 153600 bytes/frame, buffer @ 0x20100000
Frame 1 captured: 153600 bytes @ 0x20100000 (RGB565 320x240)
Export the buffer with the SDK script, e.g.:
  sftool ... read_mem 0x20100000 153600 rgb565.bin
```

## Exporting and Viewing

After capture, `take_photo` prints the PSRAM frame buffer start address (e.g. `0x20100000`) and byte count.

Use `jlinkbin2bmp.py` to read the frame buffer directly from the live target and generate a BMP in one step — no intermediate file needed.

Tools are located in: `C:\WORK\SiFli-SDK\main\tools\bin2bmp\`

```
python jlinkbin2bmp.py "<jlink_params>" rgb565 <width> <height> <addr>
```

Example (SWD at 12 MHz, QVGA buffer at `0x20100000`):

```
python jlinkbin2bmp.py "-if SWD -speed 12000" rgb565 320 240 0x20100000
```

## Notes

- The frame buffer is allocated in PSRAM via `psram_heap_malloc`. Internal SRAM is not large enough to hold frames bigger than QVGA in RGB565.
- When capturing multiple frames, the same buffer is reused; only the last frame remains in PSRAM after the command finishes.
- Pin muxing for SCCB / DVP / XCLK is handled internally by the OV2640 driver — the application does not need to call `HAL_PIN_Set()`.
