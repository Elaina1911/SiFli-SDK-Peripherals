# take_photo_to_sdcard 示例

[中文](README.md) | [English](README_EN.md)

## 示例简介

本示例演示如何使用 OV2640 摄像头组件的 **handle 层 API**（`camera_handle.h`）采集 JPEG 图像，并将其写入挂载在 `/` 下的 SD 卡文件系统，默认保存到 `/photo` 目录。

调用流程与 [`take_photo`](../take_photo/README.md) 示例完全一致：

1. `camera_handler_instance_init`
2. `camera_init`
3. `camera_change_settings`（`PIXFORMAT_JPEG` + framesize + quality）
4. 循环 `camera_capture_single`，每次成功后写入一个 `.jpg` 文件
5. `camera_deinit`

> SCCB / DVP / XCLK 等引脚的复用由 OV2640 驱动在初始化时内部完成，**应用层不需要也不应该调用 `HAL_PIN_Set()`**。

## 硬件要求

- OV2640 摄像头模组，按照 OV2640 组件 README 中的默认引脚连接
- SPI / SDIO 接口的 SD 卡，并在 RT-Thread 中注册为 `sd0` 设备
- 至少 4 MB PSRAM（示例从 PSRAM heap 中分配 JPEG 缓冲）

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
| `count`     | ≥ 1 | 拍照张数 |

示例：

```
msh> take_photo VGA 10 3
JPEG capture: framesize=VGA, quality=10, buffer=307200 bytes @ 0x6XXXXXXX
Saved /photo/photo_001.jpg (28456 bytes)
Saved /photo/photo_002.jpg (28612 bytes)
Saved /photo/photo_003.jpg (28391 bytes)
```

## 输出文件

每次拍摄完成后会生成：

```
/photo/photo_001.jpg
/photo/photo_002.jpg
/photo/photo_003.jpg
...
```

把 SD 卡接到 PC 上即可直接打开 `.jpg` 文件验证。

## 说明事项

- JPEG 输出长度不固定，示例按 `width × height` 估算缓冲大小并限制在 `[64 KB, 2 MB]` 之间；UXGA 等高分辨率请预留充足的 PSRAM。
- `quality` 越小图像越清晰，但文件越大；越大压缩越强，文件越小。
- 第一次切换分辨率/质量后，`camera_change_settings` 内部已经插入约 500 ms 的 AEC/AWB 稳定延时，无需再手动等待。
- 若提示 `sd card not found` 或 `mount fs ... fail`，请确认：SD 卡已格式化为 FAT、`sd0` 设备已注册、SPI / 时钟引脚连接正确。

## 相关文档

- 英文版：[README_EN.md](README_EN.md)
- 组件主文档：[../../README.md](../../README.md)
- 同款 RGB565 单帧示例：[`take_photo`](../take_photo/README.md)
