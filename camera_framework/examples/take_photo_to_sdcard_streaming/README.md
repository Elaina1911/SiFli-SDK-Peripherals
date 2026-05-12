# take_photo_to_sdcard（流式采集）示例

[中文](README.md) | [English](README_EN.md)

## 示例简介

本示例演示如何使用 OV2640 摄像头组件的 **handle 层流式 API**（`camera_handle.h`）以双缓冲连续采集（streaming）方式获取 JPEG 帧，并在取到每一帧后**立即写入 SD 卡**，实现"边采集边存"。

与单帧 `camera_capture_single` 方式不同，流式 API 在底层持续进行 DMA 乒乓传输，应用层只需轮询取帧，延迟更低，更适合连续拍摄场景。

调用流程：

1. `camera_handler_instance_init`
2. `camera_init`
3. `camera_change_settings`（`PIXFORMAT_JPEG` + framesize + quality）
4. 分配两块 PSRAM 帧缓冲，调用 `camera_start_stream`
5. 循环 `camera_get_stream_frame`，每取到一帧立即写入 `/photo/photo_NNN.jpg`
6. `camera_stop_stream`
7. `camera_deinit`

> SCCB / DVP / XCLK 等引脚的复用由 OV2640 驱动在初始化时内部完成，**应用层不需要也不应该调用 `HAL_PIN_Set()`**。

## 硬件要求

- OV2640 摄像头模组，按照 OV2640 组件 README 中的默认引脚连接
- SPI / SDIO 接口的 SD 卡，并在 RT-Thread 中注册为 `sd0` 设备
- 至少 4 MB PSRAM（示例从 PSRAM heap 中分配两块 JPEG 帧缓冲）

## 使用方法

启动后会自动初始化 PSRAM heap 并挂载 SD 卡：

```
OV2640 Camera Take Photo to SD Card Example
mount fs on tf card to / success
```

挂载成功后在 MSH 控制台执行：

```
msh> take_photo <framesize> <quality> <count>
```

| 参数 | 取值 | 说明 |
|------|------|------|
| `framesize` | QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA | 分辨率 |
| `quality`   | 0 ~ 63 | JPEG 质量，0 = 最佳，63 = 最大压缩 |
| `count`     | ≥ 1 | 采集帧数 |

示例：

```
msh />
take_photo VGA 10 10
[I/dvp] DVP initialized successfully
[I/dvp]   Mode: JPEG
[I/dvp]   Buffer size: 0 bytes
[I/dvp]   Pingpong buffer: 1024 bytes
[I/dvp]   VSYNC: PA42 (GPIO interrupt)
Stream start: framesize=VGA, quality=10, buffer=307200 bytes x2 @ 6000001c / 6004b038
Frame[0] seq=1 buf=0 size=15252 wait=56 ms
Saved /photo/photo_001.jpg (15252 bytes)
Frame[1] seq=2 buf=1 size=15475 wait=4 ms
Saved /photo/photo_002.jpg (15475 bytes)
Frame[2] seq=3 buf=0 size=15460 wait=3 ms
Saved /photo/photo_003.jpg (15460 bytes)
Frame[3] seq=4 buf=1 size=15684 wait=6 ms
Saved /photo/photo_004.jpg (15684 bytes)
Frame[4] seq=5 buf=0 size=15578 wait=6 ms
Saved /photo/photo_005.jpg (15578 bytes)
Frame[5] seq=6 buf=1 size=15556 wait=7 ms
Saved /photo/photo_006.jpg (15556 bytes)
Frame[6] seq=7 buf=0 size=15497 wait=5 ms
Saved /photo/photo_007.jpg (15497 bytes)
Frame[7] seq=8 buf=1 size=15729 wait=5 ms
Saved /photo/photo_008.jpg (15729 bytes)
Frame[8] seq=9 buf=0 size=15637 wait=4 ms
Saved /photo/photo_009.jpg (15637 bytes)
Frame[9] seq=10 buf=1 size=15913 wait=5 ms
Saved /photo/photo_010.jpg (15913 bytes)
```

## 输出文件

每帧采集后立即生成：

```
/photo/photo_001.jpg
/photo/photo_002.jpg
/photo/photo_003.jpg
...
```

把 SD 卡接到 PC 上即可直接打开 `.jpg` 文件验证。

## 说明事项

- 本示例**仅支持 JPEG 像素格式**，RGB565/YUV422/RAW8 等格式请使用 `cam_capture.c` 中对应命令。
- 流式采集会分配 **两块** PSRAM 帧缓冲（双缓冲 DMA），每块大小约为 `width × height`，最大 2 MB。
- 若写 SD 卡速度跟不上出帧速率，驱动会覆盖最旧未消费帧；命令结束后会打印 `dropped N frame(s)` 供诊断。
- `quality` 越小图像越清晰，但文件越大；越大压缩越强，文件越小。
- `camera_change_settings` 内部已插入约 500 ms AEC/AWB 稳定延时，无需再手动等待。
- 若提示 `sd card not found` 或 `mount fs ... fail`，请确认：SD 卡已格式化为 FAT、`sd0` 设备已注册、SPI / 时钟引脚连接正确。

## 相关文档

- 英文版：[README_EN.md](README_EN.md)
- 组件主文档：[../../README.md](../../README.md)
