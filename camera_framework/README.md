# OV2640 摄像头组件

[中文](README.md) | [English](README_EN.md)

基于 RT-Thread 的 OV2640 摄像头组件，适用于 SiFli SF32 平台。当前版本已经从单一设备读图模型扩展为分层相机框架，包含传感器驱动层、DVP 数据总线层、通用 handle 层，以及面向上层的单次采集和连续采集接口。

## 当前能力
- 支持SF32LB52-LCD平台，SF32LB56-wlan-core平台
- 支持 JPEG、RGB565、YUV422、RAW8 像素格式
- 支持单次采集
- 支持连续采集，使用双缓冲流式取帧
- 支持 RT-Thread 设备接口
- 支持通用 handle 层接口，便于后续扩展多摄像头或替换底层驱动
- 支持常见图像参数调节：分辨率、质量、镜像、翻转、曝光、白平衡、增益、特效等

## 目录结构

```text
ov2640/
├── camera/
│   ├── bus/
│   │   └── data/
│   │       ├── dvp.c
│   │       └── dvp.h
│   │   └── control/
│   │       ├── sccb.c
│   │       └── sccb.h
│   ├── driver/
│   │   └── ov2640/
│   │       ├── ov2640.c
│   │       ├── ov2640.h
│   │       ├── ov2640_regs.h
│   │       ├── ov2640_settings.h
│   ├── handle/
│   │   ├── camera_handle.c
│   │   └── camera_handle.h
│   ├── camera_device_ops.h
│   └── camera_types.h
├── examples/
├── Kconfig
├── SConscript
├── README.md
└── README_EN.md
```

## 架构说明

```text
┌────────────────────────────────────────────────────┐
│                    应用层 / 示例                     │
│   rt_device_* 接口 / camera_handle_* 接口 / MSH    │
└───────────────────────┬────────────────────────────┘
                        │
┌───────────────────────▼────────────────────────────┐
│                 handle 层 camera_handle             │
│   单次采集封装、连续采集封装、流队列管理、通用适配   │
└───────────────────────┬────────────────────────────┘
                        │
┌───────────────────────▼────────────────────────────┐
│               OV2640 驱动层 camera/driver           │
│        传感器寄存器配置、RT-Thread 设备接口实现       │
└───────────────────────┬────────────────────────────┘
                        │
┌───────────────────────▼────────────────────────────┐
│                 DVP 数据层 camera/bus/data          │
│   GPIO + GPTIM + DMA、JPEG 解析、固定尺寸帧采集、重装填 │
└───────────────────────┬────────────────────────────┘
                        │
                   OV2640 硬件
```

## 硬件连接

| OV2640 引脚 | MCU 引脚 | 功能 | 说明 |
|-------------|----------|------|------|
| D0 ~ D7 | 平台固定数据引脚组 | 8-bit 并行数据 | 由 DVP 默认资源宏配置 |
| PCLK | 默认 PA41 | 像素时钟 | DVP 默认映射到 GPTIM1_ETR |
| HSYNC/HREF | 默认 PA43 | 行同步 | DVP 默认映射到 GPTIM1_CH1 |
| VSYNC | Kconfig 配置 | 帧同步 | 由中断触发一帧采集 |
| SDA | 默认 PA39 | SCCB 数据 | 用户在 open 前配置 |
| SCL | 默认 PA40 | SCCB 时钟 | 用户在 open 前配置 |
| XCLK | 默认 PA08 | 外部输入时钟 | 可选 GPTIM2_CH1 输出 |

说明：

- DVP 默认资源描述已收敛在 dvp.h 中，通过宏统一配置
- I2C、PCLK、HSYNC、XCLK 的 pinmux 仍需在打开设备前由板级代码完成

## Kconfig 配置

在 menuconfig 中进入 ov2640 camera 菜单：

### DVP 配置

| 选项 | 默认值 | 说明 |
|------|--------|------|
| OV2640_DVP_PINGPONG_BUFFER_SIZE | 8192 | DVP 乒乓缓冲区大小 |
| OV2640_DVP_VSYNC_PIN | 42 | VSYNC 引脚编号 |

### SCCB 配置

| 选项 | 默认值 | 说明 |
|------|--------|------|
| OV2640_SCCB_I2C_BUS_NAME | i2c1 | I2C 总线名称 |
| OV2640_SCCB_TIMEOUT_MS | 1000 | I2C 通信超时 |
| OV2640_SCCB_MAX_HZ | 100000 | I2C 最大频率 |

### 相机配置

| 选项 | 默认值 | 说明 |
|------|--------|------|
| OV2640_CAMERA_READ_TIMEOUT_MS | 1000 | 单次采集超时 |
| OV2640_DVP_XCLK_PIN | -1 | XCLK 输出引脚，-1 表示禁用 |
| OV2640_DVP_XCLK_FREQ | 12MHz | XCLK 输出频率 |

## 使用方式

### 1. 底层 RT-Thread 设备接口

适用于直接按设备方式控制 OV2640。

```c
rt_device_t dev = rt_device_find("ov2640");
rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);

rt_device_control(dev, OV2640_CMD_SET_PIXFORMAT, (void *)(rt_ubase_t)PIXFORMAT_JPEG);
rt_device_control(dev, OV2640_CMD_SET_FRAMESIZE, (void *)(rt_ubase_t)FRAMESIZE_VGA);
rt_device_control(dev, OV2640_CMD_SET_QUALITY, (void *)(rt_ubase_t)10);

rt_thread_mdelay(500);

rt_size_t size = rt_device_read(dev, 0, buffer, buffer_size);

rt_device_close(dev);
```

### 2. 通用 handle 层接口

适用于上层业务不希望直接依赖具体传感器驱动时使用。

```c
camera_handler_instance_t instance;
camera_handler_all_input_arg_t input_arg = {
    .device_name = "ov2640",
    .device_ops = ov2640_get_device_ops(),
};

camera_handler_instance_init(&instance, &input_arg);
camera_init(&instance);

camera_capture_config_t cfg = {
    .pixformat = PIXFORMAT_JPEG,
    .framesize = FRAMESIZE_VGA,
    .quality   = 10,
};
camera_change_settings(&instance, &cfg);
```

单次采集：

```c
camera_capture_request_t request = {
    .buffer = buffer,
    .buffer_size = buffer_size,
    .frame_size = 0,
};

camera_capture_single(&instance, &request);
```

连续采集：

```c
camera_stream_config_t stream_config = {
    .buffers = {buffer0, buffer1},
    .buffer_size = buffer_size,
};

camera_stream_frame_t frame;

camera_start_stream(&instance, &stream_config);
camera_get_stream_frame(&instance, &frame, 5 * RT_TICK_PER_SECOND);
camera_stop_stream(&instance);
```

## 控制命令

### 常用控制命令

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| OV2640_CMD_SET_PIXFORMAT | 0x01 | pixformat_t | 设置像素格式 |
| OV2640_CMD_SET_FRAMESIZE | 0x02 | framesize_t | 设置分辨率 |
| OV2640_CMD_SET_QUALITY | 0x06 | int | 设置 JPEG 质量 |
| OV2640_CMD_START_CAPTURE | 0x10 | 无 | 启动单次采集 |
| OV2640_CMD_STOP_CAPTURE | 0x11 | 无 | 停止单次采集 |
| OV2640_CMD_GET_FRAME_SIZE | 0x12 | uint32_t * | 获取当前帧大小 |
| OV2640_CMD_SET_FRAME_BUFFER | 0x1F | void * | 设置目标帧缓冲 |
| OV2640_CMD_SET_FRAME_BUFFER_SIZE | 0x20 | rt_size_t | 设置帧缓冲大小 |
| OV2640_CMD_SET_PINGPONG_SIZE | 0x21 | uint32_t | 设置 DVP 乒乓缓冲大小 |
| OV2640_CMD_START_STREAM | 0x22 | camera_stream_start_args_t * | 启动连续采集 |
| OV2640_CMD_STOP_STREAM | 0x23 | 无 | 停止连续采集 |

### 支持的像素格式

- PIXFORMAT_JPEG
- PIXFORMAT_RGB565
- PIXFORMAT_YUV422
- PIXFORMAT_RAW8

### 支持的分辨率

- FRAMESIZE_96X96
- FRAMESIZE_QQVGA
- FRAMESIZE_128X128
- FRAMESIZE_QCIF
- FRAMESIZE_HQVGA
- FRAMESIZE_240X240
- FRAMESIZE_QVGA
- FRAMESIZE_320X320
- FRAMESIZE_CIF
- FRAMESIZE_HVGA
- FRAMESIZE_VGA
- FRAMESIZE_SVGA
- FRAMESIZE_XGA
- FRAMESIZE_HD
- FRAMESIZE_SXGA
- FRAMESIZE_UXGA

## 示例命令

集成工程 src/main.c 中可见以下命令示例：

- take_photo：JPEG 单次拍照并保存到 SD 卡
- take_raw_photo：RAW8 拍照并转 BMP 保存
- take_yuv_photo：YUV422 拍照并转 BMP 保存
- take_rgb565_photo：RGB565 拍照并转 BMP 保存
- stream_capture：连续采集测试，只打印每帧耗时和帧大小，不保存文件

连续采集示例：

```text
msh> stream_capture jpeg VGA 30 10
msh> stream_capture rgb565 QVGA 60
```

## 注意事项

1. 连续采集使用双缓冲，调用方需要提供两块可持续复用的帧缓冲。
2. 单次采集和连续采集是两种互斥模式，流模式开启后不应再调用单次 camera_capture_single。
3. JPEG 模式建议按分辨率预留足够缓冲，VGA 常见可从 300 KB 起评估，UXGA 需要更大空间。
4. RAW8、YUV422、RGB565 属于定长帧模式，缓冲大小应按分辨率和像素格式精确计算。
5. 修改传感器分辨率、质量等参数后，建议等待约 500 ms 再开始采集。
6. 如果只出现首帧或持续超时，优先检查 VSYNC、PCLK、HSYNC、XCLK 配置以及 DVP 缓冲大小。

## 依赖项

- RT-Thread 设备框架
- RT-Thread I2C 驱动框架
- SiFli BSP HAL，含 GPIO、DMA、GPTIM
- 可用的 SCCB I2C 总线设备，默认 i2c1

## 相关文档

- 集成工程说明：工作区根目录 README.md
- 英文说明：README_EN.md
