# take_photo 示例说明

[中文](README.md) | [English](README_EN.md)

## 示例简介

本示例演示如何通过 `ov2640` 设备采集 RGB565 原始图像帧，并将帧缓冲区地址打印到控制台，方便通过 SDK 工具将 PSRAM 中的数据导出到主机后查看。

该示例位于 [examples/take_photo](examples/take_photo)。

## 使用方法

在 MSH 控制台执行：

```
msh> take_photo <framesize> <count>
```

参数说明：
- **framesize**：分辨率，可选值：QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA
- **count**：采集帧数（>= 1）

示例：

```
msh> take_photo QVGA 1
```

## 输出示例

```
RGB565 capture: 320x240, 153600 bytes/frame, buffer @ 0x20100000
Frame 1 captured: 153600 bytes @ 0x20100000 (RGB565 320x240)
Export the buffer with the SDK script, e.g.:
  sftool ... read_mem 0x20100000 153600 rgb565.bin
```

## 导出与查看

采集完成后，`take_photo` 会打印 PSRAM 帧缓冲区的起始地址（如 `0x20100000`）和字节数。

使用 `jlinkbin2bmp.py` 直接从目标板读取帧缓冲区并生成 BMP，无需先导出 bin 文件：

工具目录：`C:\WORK\SiFli-SDK\main\tools\bin2bmp\`

```
python jlinkbin2bmp.py "<jlink参数>" rgb565 <width> <height> <addr>
```

示例（SWD 12 MHz，QVGA 缓冲区地址 `0x20100000`）：

```
python jlinkbin2bmp.py "-if SWD -speed 12000" rgb565 320 240 0x20100000
```

## 注意事项

- 帧缓冲区分配在 PSRAM（通过 `psram_heap_malloc`），内部 SRAM 无法容纳大于 QVGA 的 RGB565 帧。
- 多帧拍摄时复用同一缓冲区，仅最后一帧保留在 PSRAM 中。
- 引脚复用（SCCB / DVP / XCLK）由 OV2640 驱动内部完成，无需在应用层调用 `HAL_PIN_Set()`。
