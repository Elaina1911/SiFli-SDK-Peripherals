# Camera 框架使用与开发手册

> 适用范围：`camera/`（`project/sf-pkgs/full_deploy/host/ov2640/0.0.2/camera/`）  
> 目标平台：SF32LB52X / SF32LB56X，RT-Thread 操作系统

---

## 目录

1. [架构概述](#1-架构概述)  
2. [目录结构](#2-目录结构)  
3. [快速上手（用户指南）](#3-快速上手用户指南)  
   - 3.1 单帧拍照  
   - 3.2 连续流式采集  
   - 3.3 修改图像参数  
4. [配置参考](#4-配置参考)  
   - 4.1 Kconfig 选项  
   - 4.2 关键宏与常量  
5. [内存管理](#5-内存管理)  
6. [API 参考](#6-api-参考)  
   - 6.1 实例生命周期与状态机  
   - 6.2 Handle 层 API  
   - 6.3 错误码  
   - 6.4 OV2640 扩展控制命令表  
7. [开发指南——接入新摄像头传感器](#7-开发指南接入新摄像头传感器)  
8. [开发指南——接入新数据总线](#8-开发指南接入新数据总线)  
9. [分层交互时序](#9-分层交互时序)  
10. [常见问题](#10-常见问题)  

---

## 1. 架构概述

```
┌─────────────────────────────────────────────────────────┐
│                    应用层代码                            │
│         cam_capture.c / lcd_preview.c / …               │
└───────────────────────┬─────────────────────────────────┘
                        │ camera_handle.h API
┌───────────────────────▼─────────────────────────────────┐
│               Handle 层  camera_handle.c                │
│  · 统一错误码  · 流队列（双槽）  · 信号量同步              │
│  · 单帧 / 流 两种采集模式                                │
└───────────┬───────────────────────────┬─────────────────┘
            │ rt_device_find/open/      │ psram_heap.h
            │ read/control              │ (PSRAM 帧缓冲)
┌───────────▼───────────────────────────────────────────┐
│           RT-Thread 设备层  "ov2640"                   │
│               driver/ov2640/ov2640.c                  │
│  · 传感器寄存器配置   · 模式切换   · 帧分发              │
└──────────┬────────────────────────────────────────────┘
           │ bus_adapter_t * (data_bus_adapter.h)
  ┌────────▼──────────┐          ┌──────────────────┐
  │  DVP 数据总线      │          │  控制总线 SCCB    │
  │  bus/data/dvp.c   │          │  bus/control/     │
  │  GPTIM + DMA      │          │  sccb.c   I²C    │
  └───────────────────┘          └──────────────────┘
```

**关键设计原则**

| 原则 | 体现 |
|------|------|
| 传感器与总线解耦 | `ov2640_device_t` 仅持有 `bus_adapter_t *`，不直接包含 `dvp_handle_t` |
| 总线可替换 | 任何实现了 `bus_adapter_ops_t` 的模块都可注册并被传感器驱动使用 |
| ISR 安全的帧通知 | DMA ISR → `dvp_dispatch_frame()` → 用户回调，链路仅 3 层，无额外锁 |
| 丢帧可观测 | Handle 层维护 `dropped_count`，首次及每 32 次丢帧通过 `LOG_W` 告警 |

---

## 2. 目录结构

```
camera/
├── README.md                    ← 本文件
├── handle/
│   ├── camera_handle.h          ← 公共 API 与类型定义（用户只需包含此文件）
│   └── camera_handle.c          ← Handle 层实现
├── mem/
│   ├── psram_heap.h             ← PSRAM 帧缓冲分配器接口
│   └── psram_heap.c             ← PSRAM memheap 实现
├── bus/
│   ├── control/
│   │   ├── sccb.h               ← SCCB/I²C 控制总线接口
│   │   └── sccb.c
│   └── data/
│       ├── data_bus_adapter.h   ← 数据总线抽象层接口（注册/查找/分发）
│       ├── data_bus_adapter.c
│       ├── dvp.h                ← DVP 具体实现接口
│       └── dvp.c
└── driver/
    └── ov2640/
        ├── ov2640.h             ← OV2640 RT-Thread 设备驱动接口 + 控制命令表
        ├── ov2640.c
        ├── ov2640_regs.h        ← 寄存器定义
        └── ov2640_settings.h    ← 分辨率/格式预设表
```

---

## 3. 快速上手（用户指南）

应用层**只需**包含一个头文件：

```c
#include "camera_handle.h"
```

### 3.1 单帧拍照

```c
/* 1. 声明实例（可放全局或局部） */
static camera_handler_instance_t g_cam;

/* 2. ops 表：告知 Handle 层传感器命令 ID */
static const camera_device_ops_t g_cam_ops = {
    .command_set = {
        .set_pixformat = OV2640_CMD_SET_PIXFORMAT,
        .set_framesize = OV2640_CMD_SET_FRAMESIZE,
        .set_quality   = OV2640_CMD_SET_QUALITY,
        .start_stream  = OV2640_CMD_START_STREAM,
        .stop_stream   = OV2640_CMD_STOP_STREAM,
    },
};

void app_camera_init(void)
{
    camera_handler_all_input_arg_t arg = {
        .device_name = NULL,        /* NULL → 使用默认值 "ov2640" */
        .device_ops  = &g_cam_ops,
    };

    /* 初始化实例（不打开设备） */
    camera_handler_instance_init(&g_cam, &arg);

    /* 打开 RT-Thread 设备 */
    if (camera_init(&g_cam) != CAMERA_OK)
    {
        LOG_E("camera_init failed");
        return;
    }

    /* 可选：修改默认分辨率/格式（初始默认为 JPEG VGA quality=10） */
    camera_capture_config_t cfg = {
        .pixformat = PIXFORMAT_JPEG,
        .framesize = FRAMESIZE_SVGA,
        .quality   = 12,
    };
    camera_change_settings(&g_cam, &cfg);
}

/* 帧缓冲建议放 PSRAM */
static uint8_t *g_frame_buf;

void app_capture_one_frame(void)
{
    if (g_frame_buf == NULL)
        g_frame_buf = psram_heap_malloc(200 * 1024);  /* 200 KB for JPEG */

    camera_capture_request_t req = {
        .buffer      = g_frame_buf,
        .buffer_size = 200 * 1024,
    };

    camera_handle_status_t ret = camera_capture_single(&g_cam, &req);
    if (ret == CAMERA_OK)
    {
        /* req.frame_size 为本帧实际字节数 */
        do_something_with_jpeg(g_frame_buf, req.frame_size);
    }
}
```

### 3.2 连续流式采集

```c
/* 双缓冲：建议放 PSRAM */
static uint8_t *g_stream_buf[2];

void app_start_stream(void)
{
    psram_heap_init();
    for (int i = 0; i < 2; i++)
        g_stream_buf[i] = psram_heap_malloc(200 * 1024);

    camera_stream_config_t cfg = {
        .buffers[0]  = g_stream_buf[0],
        .buffers[1]  = g_stream_buf[1],
        .buffer_size = 200 * 1024,
    };

    if (camera_start_stream(&g_cam, &cfg) != CAMERA_OK)
    {
        LOG_E("start_stream failed");
        return;
    }
}

/* 消费线程 */
void stream_consumer_thread(void *param)
{
    camera_stream_frame_t frame;
    while (1)
    {
        /* 阻塞等待下一帧，超时 500 ms */
        camera_handle_status_t ret =
            camera_get_stream_frame(&g_cam, &frame, rt_tick_from_millisecond(500));
        if (ret != CAMERA_OK)
            continue;

        /* frame.buffer      → 帧数据指针（DMA 缓冲，处理须在下两帧到来前完成）
         * frame.frame_size  → 本帧字节数
         * frame.sequence    → 单调递增帧序号 */
        display_frame(frame.buffer, frame.frame_size);
    }
}

void app_stop_stream(void)
{
    camera_stop_stream(&g_cam);
}
```

> **注意**：`camera_get_stream_frame()` 返回的 `frame.buffer` 是 DMA 缓冲区的指针，**不复制数据**。调用方须在下一轮双缓冲（约 2 帧内）完成处理或另行拷贝，否则数据会被覆盖。

### 3.3 修改图像参数

`camera_change_settings()` 不可在流运行时调用；需先 `camera_stop_stream()`，修改完再 `camera_start_stream()`。

```c
camera_capture_config_t cfg = {
    .pixformat = PIXFORMAT_RGB565,
    .framesize = FRAMESIZE_QVGA,
    .quality   = 10,
};
camera_change_settings(&g_cam, &cfg);
/* 函数内部包含 500 ms AEC/AWB 收敛等待，属正常行为 */
```

若需修改亮度、对比度等细粒度参数，直接操作 RT-Thread 设备：

```c
rt_device_control(g_cam.camera_device,
                  OV2640_CMD_SET_BRIGHTNESS,
                  (void *)(rt_ubase_t)1);  /* +1 档亮度 */
```

完整命令 ID 见 `driver/ov2640/ov2640.h` 中的 `OV2640_CMD_*` 宏。

---

## 4. 配置参考

### 4.1 Kconfig 选项

通过 `menuconfig → ov2640 camera` 进入配置界面。

**DVP 配置**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `OV2640_DVP_PINGPONG_BUFFER_SIZE` | 8192 | DVP DMA 乒乓缓冲区大小（字节），范围 1024–65536。该缓冲区是 DMA 中转暂存区，与帧缓冲无关；对于宽度 > 640 的分辨率或非 JPEG 模式需适当调大。 |
| `OV2640_DVP_DATA_PIN_BASE` | 0 | DVP D0–D7 数据引脚起始编号，**请勿修改**，仅供内部使用。 |
| `OV2640_DVP_VSYNC_PIN` | 42 | VSYNC 信号连接的 GPIO 引脚编号（PAD_PA*x*）。用于帧同步中断，须与硬件原理图一致。 |

**SCCB（I²C 控制总线）配置**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `OV2640_SCCB_I2C_BUS_NAME` | `"i2c1"` | RT-Thread I²C 总线设备名，须与 `rt_device_register` 时一致。 |
| `OV2640_SCCB_TIMEOUT_MS` | 1000 | SCCB 单次 I²C 操作超时（毫秒）。总线频繁超时时可适当增大，或检查总线负载。 |
| `OV2640_SCCB_MAX_HZ` | 100000 | SCCB 时钟频率上限（Hz），范围 10000–400000。OV2640 标准为 100 kHz，部分板卡可提升至 400 kHz（Fast-mode）。 |

**摄像头通用配置**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `OV2640_CAMERA_READ_TIMEOUT_MS` | 1000 | 单帧同步读取超时（毫秒），范围 100–10000。低帧率或长曝光场景下出现 `CAMERA_ERRORTIMEOUT` 时可增大。 |
| `OV2640_DVP_XCLK_PIN` | -1 | XCLK 输出引脚编号（PAD_PA*x* 索引）。`-1` 表示不由驱动生成 XCLK（使用外部时钟源）。启用时需在 `main.c` 中手动配置引脚复用，详见下方说明。 |
| `OV2640_DVP_XCLK_FREQ` | 12000000 | XCLK 频率（Hz），可选 6 MHz（`OV2640_DVP_XCLK_FREQ_6M`）或 12 MHz（`OV2640_DVP_XCLK_FREQ_12M`）。GPTIM2 运行于 24 MHz，故仅支持这两档。 |

> **XCLK 引脚配置**：启用 XCLK 输出时，须在打开摄像头设备前手动配置引脚复用，例如：
> ```c
> HAL_PIN_Set(PAD_PA00 + OV2640_DVP_XCLK_PIN, GPTIM2_CH1, PIN_NOPULL, 1);
> ```
> 具体 `PAD_PA*` 偏移量取决于 XCLK 连接的引脚，参见硬件原理图。

### 4.2 关键宏与常量

| 宏 | 定义位置 | 说明 |
|----|---------|------|
| `OV2640_DEVICE_NAME` | `ov2640.h` | 驱动注册/查找用的 RT-Thread 设备名 |
| `CAMERA_DEFAULT_DEVICE_NAME` | `camera_handle.h` | Handle 层在未指定名称时的回退值，与上行保持一致 |
| `DVP_BUS_ADAPTER_NAME` | `dvp.h` | DVP 适配器在注册表中的名字（`"dvp"`） |
| `BUS_ADAPTER_MAX` | `data_bus_adapter.c` | 总线适配器注册表容量（默认 8） |
| `DEBUG_DVP` | `dvp.c` | 置 1 开启 DVP 调试快照及 hex dump（默认 0） |

---

## 5. 内存管理

帧缓冲推荐放置于 PSRAM，以避免挤占有限的内部 SRAM：

```c
psram_heap_init();                          /* 初始化 PSRAM 堆（幂等） */
void *buf = psram_heap_malloc(200 * 1024); /* 分配帧缓冲 */
psram_heap_free(buf);                       /* 释放 */
```

`psram_heap_init()` 使用 `INIT_BOARD_EXPORT` 在系统启动时自动调用，通常无需手动初始化。

**DMA 可达性要求**：DVP DMA 引擎对 PSRAM 地址可达，但若平台有 DMA 内存限制，请确认目标地址落在 DMA 能访问的地址窗口内。

**DVP DMA 乒乓缓冲区的放置约束**：乒乓缓冲区（`dvp_pingpong_buffer`）由链接脚本静态分配在 `.dvp_pingpong` 段，其起始地址 `DVP_PINGPONG_START_ADDR` 刻意**避开** `0x2000_0000–0x2001_FFFF`（128 KB zero-wait-cycle SRAM，与 Cortex-M33 D-TCM 共享）。原因：该区域的 DMA 访问存在 **3 个周期**的仲裁延迟，当 CPU 与 DMA 同时访问时，DMA 会发生采集丢数，造成画面花屏或帧数据损坏。若自行分配缓冲区（例如调试场景），请确保目标地址不落在 `0x2000_0000–0x2001_FFFF` 范围内。

**DCache 一致性**：若平台开启了 D-Cache，DMA 写入完成后 CPU 直接读取帧缓冲区可能命中脏缓存行而得到旧数据。必须在使用帧数据前执行 Cache Invalidate：

```c
/* DMA 完成后，CPU 读取前 */
mpu_dcache_invalidate((uint32_t *)frame.buffer, (uint32_t)frame.frame_size);
```

若帧数据需要由显示 DMA 读取（写回型 Cache），则在派发前执行 Clean：

```c
mpu_dcache_clean(frame.buffer, frame.frame_size);
```

DVP DMA 乒乓缓冲区本身建议通过 MPU 配置为 Non-Cacheable，以彻底消除一致性问题。帧目标缓冲区（PSRAM）同理，驱动层在派发帧回调前不会自动执行 Invalidate，由应用层负责。

---

## 6. API 参考

### 6.1 实例生命周期与状态机

```
       camera_handler_instance_init()
                    │
            ┌───────▼──────────┐
            │  UNINITIALIZED   │
            └───────┬──────────┘
       camera_init()│
            ┌───────▼──────────┐   camera_change_settings()
            │      IDLE        │◄─────────────────────────────┐
            └──────┬─────┬─────┘         (500 ms AEC 等待)    │
 camera_start_stream()   │ camera_capture_single()            │
            │            │ (阻塞直到帧就绪或超时)              │
   ┌────────▼────────┐   └────────────────────────────────────┘
   │   STREAMING     │
   │                 │ camera_get_stream_frame()（消费帧）
   └────────┬────────┘
            │ camera_stop_stream()
            ▼
           IDLE ──── camera_deinit() ──► RELEASED
```

| 状态 | 含义 | 可调 API |
|------|------|----------|
| UNINITIALIZED | 实例内存未初始化 | `camera_handler_instance_init` |
| IDLE | 设备已打开，未在采集 | `camera_change_settings`, `camera_capture_single`, `camera_start_stream`, `camera_deinit` |
| STREAMING | DMA 流运行中 | `camera_get_stream_frame`, `camera_stop_stream`, `camera_deinit` |
| RELEASED | `camera_deinit` 后 | — |

> `camera_deinit` 在 STREAMING 状态下会自动先调用 `camera_stop_stream`，再关闭设备和销毁信号量。

---

### 6.2 Handle 层 API

> **线程安全总则**：所有 Handle 层 API 均**不可从中断上下文调用**（内部使用信号量和 `rt_device_control`）。帧回调由框架内部注入，应用代码无需直接调用。

---

#### `camera_handler_instance_init`

```c
camera_handle_status_t camera_handler_instance_init(
    camera_handler_instance_t         *instance,
    camera_handler_all_input_arg_t    *input_arg);
```

将 `*instance` 清零并填入设备名和 ops 指针。**不打开 RT-Thread 设备，也不分配任何内核对象**。

| 参数 | 方向 | 说明 |
|------|------|------|
| `instance` | in | 要初始化的实例，不可为 NULL |
| `input_arg` | in/可 NULL | 若为 NULL 或 `device_name` 为 NULL，设备名回退为 `CAMERA_DEFAULT_DEVICE_NAME`（`"ov2640"`） |

默认预设：`pixformat = PIXFORMAT_JPEG`，`framesize = FRAMESIZE_VGA`，`quality = 10`。

---

#### `camera_init`

```c
camera_handle_status_t camera_init(camera_handler_instance_t *instance);
```

调用 `rt_device_find` + `rt_device_open`，标记 `instance->is_open = RT_TRUE`。**幂等**：已打开时立即返回 `CAMERA_OK`。

前置条件：`instance->device_ops` 必须已设置（否则返回 `CAMERA_ERRORRESOURCE`）。

---

#### `camera_deinit`

```c
camera_handle_status_t camera_deinit(camera_handler_instance_t *instance);
```

若当前处于 STREAMING 状态，内部先调用 `camera_stop_stream()`；再关闭 RT-Thread 设备，最后销毁信号量（若已创建）。实例可在 `camera_handler_instance_init` 后重新使用。

---

#### `camera_change_settings`

```c
camera_handle_status_t camera_change_settings(
    camera_handler_instance_t       *instance,
    const camera_capture_config_t   *config);
```

依次下发 `set_pixformat` → `set_framesize` → `set_quality` 三条控制命令，成功后保存至 `instance->active_config`，并**阻塞 500 ms** 等待 AEC/AWB 收敛。

| 约束 | 说明 |
|------|------|
| 不可在 STREAMING 期间调用 | 返回 `CAMERA_ERRORRESOURCE` |
| 设备必须已打开 | 否则返回 `CAMERA_ERRORRESOURCE` |

---

#### `camera_capture_single`

```c
camera_handle_status_t camera_capture_single(
    camera_handler_instance_t   *instance,
    camera_capture_request_t    *request);
```

启动一次 DMA 采集，在驱动信号量上阻塞，帧就绪后填写 `request->frame_size` 并返回。超时由 `OV2640_CAMERA_READ_TIMEOUT_MS` 控制。

| 参数字段 | 说明 |
|----------|------|
| `request->buffer` | 目标缓冲区，须 DMA 可达（推荐 PSRAM） |
| `request->buffer_size` | 缓冲区容量（字节），须足以容纳一帧 |
| `request->frame_size` | **输出**：本帧实际字节数（成功时填写） |

> **注意**：若函数超时返回，DVP DMA 可能仍在运行。建议超时后调用 `camera_deinit` 再重新 `camera_init` 彻底重置状态。

---

#### `camera_start_stream`

```c
camera_handle_status_t camera_start_stream(
    camera_handler_instance_t       *instance,
    const camera_stream_config_t    *config);
```

懒初始化帧信号量（仅第一次），排空遗留令牌，重置队列，向驱动发送 `START_STREAM` 命令（内部通过 `camera_stream_start_args_t` 注入帧回调）。

| 参数字段 | 说明 |
|----------|------|
| `config->buffers[0/1]` | 两块 DMA 可达的帧缓冲区，须在 `camera_stop_stream` 返回前保持有效 |
| `config->buffer_size` | 每块缓冲区大小（字节） |

---

#### `camera_get_stream_frame`

```c
camera_handle_status_t camera_get_stream_frame(
    camera_handler_instance_t   *instance,
    camera_stream_frame_t       *frame,
    rt_int32_t                   timeout);
```

在信号量上阻塞，成功时从队列尾部取出一帧浅拷贝到 `frame`。

| 参数 | 说明 |
|------|------|
| `timeout` | RT-Thread tick 数；`RT_WAITING_FOREVER` 永久等待；`0` 非阻塞轮询 |

`frame.buffer` 指向 DMA 缓冲区，**不复制数据**。必须在下两帧到来前完成处理或另行拷贝，否则数据被覆盖。

`frame.sequence` 是驱动层单调递增帧计数器，可用于检测丢帧：连续两次获取的 sequence 差值 > 1 说明 Handle 层队列满时发生了覆盖丢帧。

---

#### `camera_stop_stream`

```c
camera_handle_status_t camera_stop_stream(camera_handler_instance_t *instance);
```

向驱动发送 `STOP_STREAM` 命令（内部：先 `bus_adapter_abort_capture` 停 DMA，再在临界区内清空流状态），然后将队列与 `dropped_count` 归零。**幂等**：流未激活时返回 `CAMERA_OK`。

---

### 6.3 错误码

| 枚举值 | 数值 | 含义 | 典型触发场景 |
|--------|------|------|-------------|
| `CAMERA_OK` | 0 | 成功 | — |
| `CAMERA_ERROR` | 1 | 通用错误 | 驱动返回了未映射的 RT-Thread 错误码 |
| `CAMERA_ERRORTIMEOUT` | 2 | 操作超时 | `camera_capture_single` 超时；`camera_get_stream_frame` 等待超时 |
| `CAMERA_ERRORRESOURCE` | 3 | 资源不可用 | 设备未打开；ops 未注册；流已激活时调用 `camera_change_settings` |
| `CAMERA_ERRORPARAMETER` | 4 | 参数非法 | `instance` 或 `config` 为 NULL；`buffer_size == 0` |
| `CAMERA_ERRORNOMEMORY` | 5 | 内存不足 | 信号量创建失败（内核对象池耗尽） |
| `CAMERA_ERRORISR` | 6 | ISR 上下文不允许 | 预留，当前版本未使用 |

---

### 6.4 OV2640 扩展控制命令表

通过 `rt_device_control(instance->camera_device, OV2640_CMD_*, arg)` 直接访问传感器细粒度参数，绕过 Handle 层封装。整数参数须转型为 `(void *)(rt_ubase_t)value`。

**图像格式与分辨率**

| 命令 | 值 | 参数类型 | 说明 |
|------|----|----------|------|
| `OV2640_CMD_SET_PIXFORMAT` | 0x01 | `pixformat_t` | 像素格式（JPEG / RGB565 / YUV422 / RAW8） |
| `OV2640_CMD_SET_FRAMESIZE` | 0x02 | `framesize_t` | 帧分辨率（96×96 → UXGA 1600×1200） |

**图像质量**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_BRIGHTNESS` | 0x03 | int −2…+2 | 亮度 |
| `OV2640_CMD_SET_CONTRAST` | 0x04 | int −2…+2 | 对比度 |
| `OV2640_CMD_SET_SATURATION` | 0x05 | int −2…+2 | 饱和度 |
| `OV2640_CMD_SET_QUALITY` | 0x06 | int 0…63 | JPEG 压缩质量（值越小文件越小，画质越低） |
| `OV2640_CMD_SET_SHARPNESS` | 0x1D | int −2…+2 | 锐度（OV2640 硬件无此功能，写入无效） |
| `OV2640_CMD_SET_DENOISE` | 0x1E | int level | 降噪级别（OV2640 硬件无此功能，写入无效） |
| `OV2640_CMD_SET_GAINCEILING` | 0x1C | `gainceiling_t` | AGC 增益上限 |

**白平衡**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_WHITEBAL` | 0x0A | int 0/1 | 开关硬件白平衡 |
| `OV2640_CMD_SET_AWB_GAIN` | 0x0E | int 0/1 | 开关 AWB 增益 |
| `OV2640_CMD_SET_WB_MODE` | 0x15 | int 0–4 | 0=自动, 1=晴天, 2=阴天, 3=办公室, 4=家庭 |

**曝光与增益**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_GAIN_CTRL` | 0x0B | int 0/1 | 开关 AGC（自动增益控制） |
| `OV2640_CMD_SET_EXPOSURE_CTRL` | 0x0C | int 0/1 | 开关 AEC（自动曝光控制） |
| `OV2640_CMD_SET_AEC2` | 0x0D | int 0/1 | 开关 AEC2（带符号曝光补偿） |
| `OV2640_CMD_SET_AGC_GAIN` | 0x0F | int 0–30 | AGC 手动增益值 |
| `OV2640_CMD_SET_AEC_VALUE` | 0x13 | int 0–1200 | AEC 手动曝光值 |
| `OV2640_CMD_SET_AE_LEVEL` | 0x16 | int −2…+2 | 自动曝光目标亮度偏置 |

**图像处理**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_HMIRROR` | 0x07 | int 0/1 | 水平镜像 |
| `OV2640_CMD_SET_VFLIP` | 0x08 | int 0/1 | 垂直翻转 |
| `OV2640_CMD_SET_COLORBAR` | 0x09 | int 0/1 | 开启彩条测试图（用于验证数据通路） |
| `OV2640_CMD_SET_SPECIAL_EFFECT` | 0x14 | int 0–6 | 特效：0=正常, 1=负片, 2=黑白, 3=红调, 4=绿调, 5=蓝调, 6=复古 |
| `OV2640_CMD_SET_DCW` | 0x17 | int 0/1 | 数字裁剪缩放 |
| `OV2640_CMD_SET_BPC` | 0x18 | int 0/1 | 坏点校正 |
| `OV2640_CMD_SET_WPC` | 0x19 | int 0/1 | 白点校正 |
| `OV2640_CMD_SET_RAW_GMA` | 0x1A | int 0/1 | RAW gamma 开关 |
| `OV2640_CMD_SET_LENC` | 0x1B | int 0/1 | 镜头阴影校正 |

**数据总线与缓冲区**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_FRAME_BUFFER` | 0x1F | `void *` | 更新帧缓冲区指针（不重启 DMA） |
| `OV2640_CMD_SET_FRAME_BUFFER_SIZE` | 0x20 | `uint32_t` | 更新帧缓冲区大小（字节） |
| `OV2640_CMD_SET_PINGPONG_SIZE` | 0x21 | `uint32_t` | 运行时调整 DVP DMA 乒乓缓冲大小 |

---

## 7. 开发指南——接入新摄像头传感器

以下以接入虚构的 `OV5640` 为例，给出**可直接参考的最小完整实现**。各步骤严格对应框架的分层约定，如无特殊说明，不需要修改 Handle 层任何代码。

### 步骤一：创建驱动目录

```
camera/driver/ov5640/
    ov5640.h        ← 设备实例类型、命令 ID、注册声明
    ov5640.c        ← RT-Thread 设备 ops 实现
    ov5640_regs.h   ← 寄存器地址常量
```

> **为什么不放 `camera/` 根目录？** 框架把传感器驱动和总线适配器各自隔离，未来可以无摩擦地换传感器或换总线。

### 步骤二：定义流状态结构体与头文件

```c
/* ov5640.h */
#pragma once

#include "camera_handle.h"        /* camera_handle_status_t, camera_stream_frame_t … */
#include "data_bus_adapter.h"     /* bus_adapter_t */
#include "rtthread.h"
#include "rthw.h"                 /* rt_hw_interrupt_disable / enable */

#define OV5640_DEVICE_NAME  "ov5640"
#define OV5640_I2C_ADDR     0x3C   /* 7-bit */

/* ─── 控制命令 ID（与 Handle 层约定一一对应） ─── */
#define OV5640_CMD_SET_PIXFORMAT    0x01
#define OV5640_CMD_SET_FRAMESIZE    0x02
#define OV5640_CMD_SET_QUALITY      0x06
#define OV5640_CMD_START_STREAM     0x22
#define OV5640_CMD_STOP_STREAM      0x23
/* … 其余 SET_BRIGHTNESS / SET_HMIRROR 等按需添加 … */

/* ─── 流状态（与 ov2640_stream_state_t 结构相同） ─── */
typedef struct {
    void    *buffers[2];                    /* DMA 乒乓缓冲区指针 */
    uint32_t buffer_size;                   /* 每块缓冲区大小（字节） */
    uint8_t  active_buffer_index;           /* 当前 DMA 写入的缓冲区下标（0/1） */
    uint32_t sequence;                      /* 单调递增帧序号 */
    /*
     * frame_callback != RT_NULL  ⟺  流处于"已激活"状态。
     * ISR 和 START/STOP 命令均以临界区保护此字段的读写，
     * 严禁在临界区外直接写入。
     */
    camera_frame_callback_t  frame_callback;
    void                    *callback_context;
} ov5640_stream_state_t;

/* ─── 传感器设备私有数据 ─── */
typedef struct {
    bus_adapter_t        *data_bus;     /* DVP / CSI 适配器（通过名字查找） */
    ov5640_stream_state_t stream;
    rt_sem_t              frame_sem;    /* 单帧拍照信号量 */
    void                 *snap_buffer; /* 单帧拍照目标缓冲区 */
    uint32_t              snap_size;   /* 本次单帧拍照实际字节数 */
} ov5640_device_t;

int ov5640_device_register(void);
```

### 步骤三：实现帧回调（ISR 上下文）

帧回调在 DVP 中断 / DMA 完成中断中被调用。必须：

- 在临界区内**原子地读取** `frame_callback` 和 `callback_context`，防止与 START/STOP 竞争；
- 回调结束后立即重装 DMA（`rearm_capture`），若重装失败则中止采集并清除流状态。

```c
static void ov5640_frame_ready_callback(bus_adapter_t *self,
                                        const bus_frame_t *bus_frame,
                                        void *user_data)
{
    ov5640_device_t *cam = (ov5640_device_t *)user_data;
    rt_base_t level;

    /* ① 在临界区内原子地取出回调指针快照 */
    level = rt_hw_interrupt_disable();
    camera_frame_callback_t frame_callback    = cam->stream.frame_callback;
    void                   *callback_context  = cam->stream.callback_context;
    rt_hw_interrupt_enable(level);

    /* ② frame_callback == RT_NULL 表示流未激活，直接返回 */
    if (frame_callback == RT_NULL)
        return;

    /* ③ 构造应用侧帧描述符（浅拷贝，不复制像素数据） */
    camera_stream_frame_t frame = {
        .buffer       = bus_frame->buffer,
        .buffer_size  = cam->stream.buffer_size,
        .frame_size   = bus_frame->length,
        .sequence     = cam->stream.sequence++,
        .buffer_index = cam->stream.active_buffer_index,
    };

    /* ④ 切换乒乓缓冲区并重装 DMA */
    cam->stream.active_buffer_index ^= 1;
    void *next_buf = cam->stream.buffers[cam->stream.active_buffer_index];

    if (bus_adapter_rearm_capture(cam->data_bus, next_buf,
                                   cam->stream.buffer_size) != BUS_OK)
    {
        /* 重装失败：原子清除流状态，中止 DMA */
        level = rt_hw_interrupt_disable();
        rt_memset(&cam->stream, 0, sizeof(cam->stream)); /* frame_callback → NULL */
        rt_hw_interrupt_enable(level);
        bus_adapter_abort_capture(cam->data_bus);
        return;
    }

    /* ⑤ 通知 Handle 层 */
    frame_callback(callback_context, &frame);
}
```

### 步骤四：实现 RT-Thread 设备 ops

#### `ov5640_open`

```c
static rt_err_t ov5640_open(rt_device_t dev, rt_uint16_t oflag)
{
    ov5640_device_t *cam = (ov5640_device_t *)dev->user_data;

    /* 初始化控制总线（SCCB over I²C），传入传感器 7-bit 地址 */
    if (sccb_init(SCCB_USE_IIC) != 0)
    {
        LOG_E("ov5640: sccb init failed");
        return -RT_ERROR;
    }

    /* 查找数据总线适配器（确保 INIT_BOARD_EXPORT 已执行） */
    cam->data_bus = bus_adapter_find(DVP_BUS_ADAPTER_NAME);
    if (cam->data_bus == RT_NULL)
    {
        LOG_E("ov5640: data bus '%s' not found", DVP_BUS_ADAPTER_NAME);
        return -RT_ERROR;
    }

    /* 硬件上电 + 写入默认寄存器表 */
    ov5640_hw_reset();
    if (ov5640_write_default_regs() != 0)
        return -RT_ERROR;

    /* 初始化 DVP 总线（引脚、时钟、中断、乒乓 DMA） */
    dvp_config_t dvp_cfg = {
        .mode           = DVP_MODE_JPEG,
        .vsync_pin      = CONFIG_OV2640_DVP_VSYNC_PIN,
        .data_pin_base  = CONFIG_OV2640_DVP_DATA_PIN_BASE,
        .xclk_pin       = CONFIG_OV2640_DVP_XCLK_PIN,
        .xclk_freq      = CONFIG_OV2640_DVP_XCLK_FREQ,
        .pingpong_size  = CONFIG_OV2640_DVP_PINGPONG_BUFFER_SIZE,
    };
    if (bus_adapter_init(cam->data_bus, &dvp_cfg) != BUS_OK)
        return -RT_ERROR;

    rt_memset(&cam->stream, 0, sizeof(cam->stream));
    cam->frame_sem = RT_NULL;   /* 懒创建，在 read 时按需分配 */

    LOG_I("ov5640: opened");
    return RT_EOK;
}
```

#### `ov5640_close`

```c
static rt_err_t ov5640_close(rt_device_t dev)
{
    ov5640_device_t *cam = (ov5640_device_t *)dev->user_data;

    /* 若流仍在运行，强制停止 */
    rt_base_t level = rt_hw_interrupt_disable();
    int streaming = (cam->stream.frame_callback != RT_NULL);
    rt_hw_interrupt_enable(level);
    if (streaming)
        bus_adapter_abort_capture(cam->data_bus);

    bus_adapter_deinit(cam->data_bus);
    sccb_deinit();

    if (cam->frame_sem != RT_NULL)
    {
        rt_sem_delete(cam->frame_sem);
        cam->frame_sem = RT_NULL;
    }

    rt_memset(&cam->stream, 0, sizeof(cam->stream));
    LOG_I("ov5640: closed");
    return RT_EOK;
}
```

#### `ov5640_read`（单帧拍照模式）

`rt_device_read` 被 `camera_capture_single` 调用。参数 `size` 为缓冲区大小，返回实际字节数。

```c
static rt_size_t ov5640_read(rt_device_t dev, rt_off_t pos,
                              void *buffer, rt_size_t size)
{
    ov5640_device_t *cam = (ov5640_device_t *)dev->user_data;

    /* 懒创建信号量（只创建一次） */
    if (cam->frame_sem == RT_NULL)
    {
        cam->frame_sem = rt_sem_create("ov5640_snap", 0, RT_IPC_FLAG_FIFO);
        if (cam->frame_sem == RT_NULL)
            return 0;
    }

    cam->snap_buffer = buffer;
    cam->snap_size   = 0;

    /* 启动单帧 DMA，帧中断中调用 ov5640_snap_isr_callback */
    if (bus_adapter_start_capture(cam->data_bus, buffer, size,
                                   ov5640_snap_isr_callback, cam) != BUS_OK)
        return 0;

    /* 阻塞等待，超时由 Kconfig 控制 */
    rt_err_t ret = rt_sem_take(cam->frame_sem,
                               rt_tick_from_millisecond(OV5640_READ_TIMEOUT_MS));
    if (ret != RT_EOK)
    {
        bus_adapter_abort_capture(cam->data_bus);
        return 0;
    }

    return (rt_size_t)cam->snap_size;
}

/* 单帧拍照回调（ISR 上下文）*/
static void ov5640_snap_isr_callback(bus_adapter_t *self,
                                      const bus_frame_t *bus_frame,
                                      void *user_data)
{
    ov5640_device_t *cam = (ov5640_device_t *)user_data;
    cam->snap_size = bus_frame->length;
    rt_sem_release(cam->frame_sem);
}
```

#### `ov5640_control`（START/STOP 流 + 图像参数）

**关键约定**：
- `OV5640_CMD_START_STREAM`：先填充缓冲区信息，**最后**在临界区发布 `frame_callback`——这样 ISR 不会在缓冲区初始化完成前看到非 NULL 的回调指针。
- `OV5640_CMD_STOP_STREAM`：**先** `abort_capture` 停止 DMA，**再**在临界区清零流状态——防止 ISR 在清理期间读到半初始化状态。

```c
static rt_err_t ov5640_control(rt_device_t dev, int cmd, void *args)
{
    ov5640_device_t *cam = (ov5640_device_t *)dev->user_data;
    rt_base_t level;

    switch (cmd)
    {
    /* ── 格式 / 分辨率 / 质量 ── */
    case OV5640_CMD_SET_PIXFORMAT:
        return ov5640_set_pixformat(cam, (pixformat_t)(rt_ubase_t)args);

    case OV5640_CMD_SET_FRAMESIZE:
        return ov5640_set_framesize(cam, (framesize_t)(rt_ubase_t)args);

    case OV5640_CMD_SET_QUALITY:
        return ov5640_set_quality(cam, (int)(rt_ubase_t)args);

    /* ── 连续流启动 ── */
    case OV5640_CMD_START_STREAM:
    {
        camera_stream_start_args_t *sa = (camera_stream_start_args_t *)args;
        if (sa == RT_NULL || sa->frame_callback == RT_NULL)
            return -RT_EINVAL;

        /* ① 先在临界区外填充缓冲区信息（无竞态） */
        cam->stream.buffers[0]          = sa->buffers[0];
        cam->stream.buffers[1]          = sa->buffers[1];
        cam->stream.buffer_size         = sa->buffer_size;
        cam->stream.active_buffer_index = 0;
        cam->stream.sequence            = 0;

        /* ② 注册 DVP 帧回调并启动 DMA（此时 frame_callback 仍为 NULL） */
        bus_adapter_set_frame_callback(cam->data_bus,
                                       ov5640_frame_ready_callback, cam);
        if (bus_adapter_start(cam->data_bus) != BUS_OK)
        {
            level = rt_hw_interrupt_disable();
            rt_memset(&cam->stream, 0, sizeof(cam->stream));
            rt_hw_interrupt_enable(level);
            return -RT_ERROR;
        }

        /* ③ DMA 已运行，最后原子发布回调指针 ── ISR 从此刻起才"看到"流 */
        level = rt_hw_interrupt_disable();
        cam->stream.callback_context = sa->callback_context;
        cam->stream.frame_callback   = sa->frame_callback;
        rt_hw_interrupt_enable(level);

        return RT_EOK;
    }

    /* ── 连续流停止 ── */
    case OV5640_CMD_STOP_STREAM:
    {
        /* ① 先停 DMA，确保 ISR 不再触发 */
        bus_adapter_abort_capture(cam->data_bus);

        /* ② 再清除流状态 */
        level = rt_hw_interrupt_disable();
        rt_memset(&cam->stream, 0, sizeof(cam->stream));
        rt_hw_interrupt_enable(level);

        return RT_EOK;
    }

    default:
        return -RT_EINVAL;
    }
}
```

### 步骤五：注册 RT-Thread 设备

```c
/* ov5640.c 底部 */
static ov5640_device_t s_ov5640_dev;
static struct rt_device s_ov5640_rtdev;

static const struct rt_device_ops s_ov5640_ops = {
    .open    = ov5640_open,
    .close   = ov5640_close,
    .read    = ov5640_read,
    .write   = RT_NULL,
    .control = ov5640_control,
};

int ov5640_device_register(void)
{
    rt_memset(&s_ov5640_dev, 0, sizeof(s_ov5640_dev));
    s_ov5640_rtdev.ops       = &s_ov5640_ops;
    s_ov5640_rtdev.user_data = &s_ov5640_dev;
    return rt_device_register(&s_ov5640_rtdev, OV5640_DEVICE_NAME,
                               RT_DEVICE_FLAG_RDWR);
}
INIT_DEVICE_EXPORT(ov5640_device_register);
```

### 步骤六：更新应用侧 ops 表（Handle 层零改动）

```c
/* main.c 或 cam_capture.c */
static const camera_device_ops_t g_ov5640_ops = {
    .command_set = {
        .set_pixformat = OV5640_CMD_SET_PIXFORMAT,
        .set_framesize = OV5640_CMD_SET_FRAMESIZE,
        .set_quality   = OV5640_CMD_SET_QUALITY,
        .start_stream  = OV5640_CMD_START_STREAM,
        .stop_stream   = OV5640_CMD_STOP_STREAM,
    },
};

camera_handler_all_input_arg_t arg = {
    .device_name = OV5640_DEVICE_NAME,
    .device_ops  = &g_ov5640_ops,
};
camera_handler_instance_t g_cam;
camera_handler_instance_init(&g_cam, &arg);
camera_init(&g_cam);
```

### 常见陷阱

| 陷阱 | 现象 | 正确做法 |
|------|------|----------|
| START_STREAM 时先发布 `frame_callback` 再填充 buffers | ISR 在 buffers 有效前进入，写入野指针 | 先填 buffers，最后发布 `frame_callback` |
| STOP_STREAM 时先清 stream 再 `abort_capture` | ISR 可能在清零后重新写入 `sequence` 或 `active_buffer_index` | 先 `abort_capture`，再清 stream |
| 在回调中直接写 `cam->stream.frame_callback` | 无临界区保护，与 STOP 竞争 | 回调内仅读快照，写操作统一在临界区内 |
| `bus_adapter_find` 在 board init 前调用 | 返回 NULL | 确保在 RT-Thread 初始化链（`INIT_BOARD_EXPORT`）之后打开设备 |
| `ov5640_read` 超时后不调用 `abort_capture` | DMA 仍运行，下次 `read` 时信号量立即触发，得到脏数据 | 超时后调用 `abort_capture`，或 `camera_deinit` + 重新 `camera_init` |

---

## 8. 开发指南——接入新数据总线

以下以接入虚构的 `CSI`（MIPI Camera Serial Interface）为例，给出**所有 ops 函数的完整合约说明**。

> **命名约定**：适配器函数名前缀应与总线名保持一致（本例为 `csi_`），全局适配器实例命名为 `g_<busname>_adapter`，私有状态命名为 `s_<busname>_priv`。

### 步骤一：创建适配器模块

```
camera/bus/data/csi/
    csi.h       ← 适配器名字宏、配置结构体
    csi.c       ← bus_adapter_ops_t 实现 + 注册
```

### 步骤二：定义私有状态结构体

```c
/* csi.c */
#include "data_bus_adapter.h"
#include "rtthread.h"
#include "rthw.h"

#define CSI_BUS_ADAPTER_NAME  "csi"

typedef struct {
    /* 硬件相关 */
    volatile uint32_t *reg_base;        /* CSI 控制器寄存器基址 */
    void              *dma_handle;      /* 平台 DMA 句柄 */
    uint8_t            lane_count;
    uint32_t           bit_rate_mbps;

    /* 帧状态（在帧中断与 start_capture / abort_capture 之间共享） */
    void    *active_buffer;             /* 当前 DMA 写入目标 */
    uint32_t active_buffer_size;
    uint32_t last_frame_size;           /* 最近一帧实际字节数 */
    uint32_t frame_sequence;

    /* 上层注册的回调（连续流模式）*/
    bus_frame_ready_callback_t  user_callback;
    void                       *user_data;

    /* 单帧捕获信号量（单帧拍照模式）*/
    rt_sem_t snap_sem;
} csi_priv_t;

static csi_priv_t  s_csi_priv;
static bus_adapter_t g_csi_adapter;   /* 前向声明，供 ISR 使用 */
```

### 步骤三：实现各 ops 函数

#### `init` ── 硬件初始化

**合约**：上电、配置引脚 / 时钟、分配 DMA 资源，**不启动传输**。成功返回 `BUS_OK`（0），失败返回负数错误码。

```c
static int csi_init(bus_adapter_t *self, const void *cfg)
{
    csi_priv_t       *priv = (csi_priv_t *)self->priv;
    const csi_config_t *c  = (const csi_config_t *)cfg;

    priv->lane_count   = c->lane_count;
    priv->bit_rate_mbps = c->bit_rate_mbps;
    priv->reg_base     = (volatile uint32_t *)CSI_BASE_ADDR;

    /* 1. 配置引脚复用 */
    hal_pin_set_mux(c->d0_pin, PIN_MUX_CSI_D0);
    /* … 其余数据线 / 时钟 / 同步信号引脚 … */

    /* 2. 使能模块时钟 */
    HAL_RCC_EnableModule(RCC_MOD_CSI);

    /* 3. 分配 DMA 通道 */
    priv->dma_handle = platform_dma_alloc(DMA_CHANNEL_CSI);
    if (priv->dma_handle == NULL)
        return -BUS_ERR_NORESOURCE;

    /* 4. 分配单帧拍照信号量 */
    priv->snap_sem = rt_sem_create("csi_snap", 0, RT_IPC_FLAG_FIFO);
    if (priv->snap_sem == RT_NULL)
        return -BUS_ERR_NORESOURCE;

    return BUS_OK;
}
```

#### `deinit` ── 释放硬件资源

**合约**：在 `abort_capture` / `stop` 之后调用，释放 `init` 中分配的一切资源。

```c
static int csi_deinit(bus_adapter_t *self)
{
    csi_priv_t *priv = (csi_priv_t *)self->priv;
    HAL_RCC_DisableModule(RCC_MOD_CSI);
    platform_dma_free(priv->dma_handle);
    priv->dma_handle = NULL;
    if (priv->snap_sem != RT_NULL)
    {
        rt_sem_delete(priv->snap_sem);
        priv->snap_sem = RT_NULL;
    }
    return BUS_OK;
}
```

#### `start` / `stop` ── 连续流控制

**合约**：`start` 使能硬件时钟 / DMA 流水线，使其持续接收帧；`stop` 停止流但**不释放资源**（资源由 `deinit` 释放）。

```c
static int csi_start(bus_adapter_t *self)
{
    csi_priv_t *priv = (csi_priv_t *)self->priv;
    /* 使能帧中断 */
    platform_irq_enable(IRQ_CSI_FRAME_END, csi_frame_isr, self);
    /* 打开 CSI 数据流 */
    REG_WRITE(priv->reg_base, CSI_CTRL_EN, 1);
    return BUS_OK;
}

static int csi_stop(bus_adapter_t *self)
{
    csi_priv_t *priv = (csi_priv_t *)self->priv;
    REG_WRITE(priv->reg_base, CSI_CTRL_EN, 0);
    platform_irq_disable(IRQ_CSI_FRAME_END);
    return BUS_OK;
}
```

#### `start_capture` ── 单帧捕获

**合约**：配置 DMA 目标缓冲区并触发单次传输，**立即返回**（不阻塞）。传输完成由 ISR 通知（调用 `callback` 或释放 `snap_sem`）。若 `callback` 为 NULL，由适配器自行维护信号量（供 `ov_read` 等待）。

```c
static int csi_start_capture(bus_adapter_t *self,
                              void *buffer, uint32_t size,
                              bus_frame_ready_callback_t callback,
                              void *user_data)
{
    csi_priv_t *priv = (csi_priv_t *)self->priv;
    priv->active_buffer      = buffer;
    priv->active_buffer_size = size;
    priv->user_callback      = callback;   /* 可以为 NULL（单帧模式） */
    priv->user_data          = user_data;

    /* 配置 DMA，传输完成后触发 csi_frame_isr */
    platform_dma_setup(priv->dma_handle, buffer, size, csi_frame_isr, self);
    platform_dma_start(priv->dma_handle);
    return BUS_OK;
}
```

#### `rearm_capture` ── ISR 内重装 DMA（连续流核心）

**合约**：
- **只能在帧回调（ISR 上下文）内调用**——即在上层 `user_callback` 执行期间，或由传感器帧回调调用。  
- 更换 DMA 目标缓冲区并重新触发传输，**不停止时钟 / 流水线**，以实现零抖动乒乓切换。  
- 返回 `BUS_OK` 才能继续流；返回错误则上层（传感器帧回调）应调用 `abort_capture` 并停止流。

```c
static int csi_rearm_capture(bus_adapter_t *self,
                              void *next_buffer, uint32_t size)
{
    csi_priv_t *priv = (csi_priv_t *)self->priv;
    if (next_buffer == NULL || size == 0)
        return -BUS_ERR_PARAM;

    priv->active_buffer      = next_buffer;
    priv->active_buffer_size = size;

    /* 仅更新 DMA 目标地址，不重新启动 DMA 控制器 */
    platform_dma_update_dst(priv->dma_handle, next_buffer, size);
    return BUS_OK;
}
```

#### `abort_capture` ── 中止当前帧

**合约**：立即停止当前进行中的 DMA 传输，**不关闭硬件时钟**（调用方可能紧接着再次 `start_capture`）。可在线程或 ISR 上下文中调用。

```c
static int csi_abort_capture(bus_adapter_t *self)
{
    csi_priv_t *priv = (csi_priv_t *)self->priv;
    platform_dma_stop(priv->dma_handle);
    priv->active_buffer = NULL;
    return BUS_OK;
}
```

#### `get_frame_size` ── 查询最近帧大小

**合约**：在帧回调调用 `bus_frame_t.length` 字段之前，上层也可通过此 op 查询。返回字节数，0 表示尚无有效帧。

```c
static uint32_t csi_get_frame_size(bus_adapter_t *self)
{
    return ((csi_priv_t *)self->priv)->last_frame_size;
}
```

#### `set_frame_callback` ── 注册连续流回调

**合约**：仅保存指针，不做任何硬件操作。连续流下每帧就绪时 ISR 调用此回调。

```c
static int csi_set_frame_callback(bus_adapter_t *self,
                                   bus_frame_ready_callback_t callback,
                                   void *user_data)
{
    csi_priv_t *priv = (csi_priv_t *)self->priv;
    priv->user_callback = callback;
    priv->user_data     = user_data;
    return BUS_OK;
}
```

#### 帧中断 ISR

ISR 是连接硬件与上层的桥梁，须在其中：① 读取帧大小；② 构造 `bus_frame_t`；③ 调用 `user_callback`（若已注册）或释放 `snap_sem`（单帧模式）。

```c
/* 由平台中断向量调用，参数为 bus_adapter_t * */
static void csi_frame_isr(void *arg)
{
    bus_adapter_t *self = (bus_adapter_t *)arg;
    csi_priv_t    *priv = (csi_priv_t *)self->priv;

    /* ① 读取本帧实际传输字节数（平台 DMA 寄存器）*/
    uint32_t frame_bytes = platform_dma_get_transferred(priv->dma_handle);
    priv->last_frame_size = frame_bytes;

    /* ② 构造帧描述符（指向 DMA 缓冲区，不复制数据）*/
    bus_frame_t frame = {
        .buffer   = priv->active_buffer,
        .length   = frame_bytes,
        .sequence = priv->frame_sequence++,
    };

    /* ③-a 连续流模式：调用上层回调（传感器帧回调），由其负责 rearm_capture */
    if (priv->user_callback != NULL)
    {
        priv->user_callback(self, &frame, priv->user_data);
        return;
    }

    /* ③-b 单帧模式：释放信号量，上层 read 退出阻塞 */
    rt_sem_release(priv->snap_sem);
}
```

> **注意**：ISR 结束后硬件会继续准备下一帧，但 DMA 目标地址仍是旧缓冲区，直到 `rearm_capture` 更新。如果上层回调执行时间过长，下一帧可能会覆盖旧数据——这是乒乓设计存在的固有窗口，乒乓缓冲越大、写入速度越快，此窗口越安全。

### 步骤三：定义并注册适配器实例

```c
static const bus_adapter_ops_t s_csi_ops = {
    .init               = csi_init,
    .deinit             = csi_deinit,
    .start              = csi_start,
    .stop               = csi_stop,
    .set_frame_callback = csi_set_frame_callback,
    .start_capture      = csi_start_capture,
    .rearm_capture      = csi_rearm_capture,
    .abort_capture      = csi_abort_capture,
    .get_frame_size     = csi_get_frame_size,
    /* 不支持的 op 留 NULL；调用方须先检查是否为 NULL */
    .update_buffer      = NULL,
    .set_pingpong_size  = NULL,
};

static bus_adapter_t g_csi_adapter = {
    .name = CSI_BUS_ADAPTER_NAME,
    .type = BUS_TYPE_CSI,      /* 若需新类型，在 bus_adapter.h 的 bus_type_t 枚举中添加 */
    .ops  = &s_csi_ops,
    .priv = &s_csi_priv,
};

static int csi_adapter_register(void)
{
    return bus_adapter_register(&g_csi_adapter);
}
INIT_BOARD_EXPORT(csi_adapter_register);
```

### 步骤四：传感器驱动选择总线

```c
/* ov5640_open 中 */
cam->data_bus = bus_adapter_find(CSI_BUS_ADAPTER_NAME);
if (cam->data_bus == RT_NULL)
{
    LOG_E("ov5640: CSI adapter not found");
    return -RT_ERROR;
}
```

### `bus_adapter_ops_t` 完整字段合约

| 字段 | 是否必须 | 调用上下文 | 说明 |
|------|---------|-----------|------|
| `init` | 是 | 线程 | 硬件初始化；成功返回 `BUS_OK` |
| `deinit` | 是 | 线程 | 释放一切资源；在 `stop`/`abort_capture` 之后调用 |
| `start` | 是 | 线程 | 启动连续流；`set_frame_callback` 应在 `start` 之前调用 |
| `stop` | 是 | 线程 | 停止连续流；不释放资源 |
| `set_frame_callback` | 是 | 线程 | 仅保存指针；不做硬件操作 |
| `start_capture` | 推荐 | 线程 | 启动单帧 DMA；立即返回；完成由 ISR 通知 |
| `rearm_capture` | 推荐 | **ISR/回调** | 更新 DMA 目标地址；返回 `BUS_OK` 才能继续流 |
| `abort_capture` | 推荐 | 线程/ISR | 停止当前 DMA；不关时钟；幂等 |
| `get_frame_size` | 推荐 | 线程/ISR | 返回最近帧字节数；0 表示无有效帧 |
| `update_buffer` | 可选 | 线程 | 仅更换缓冲区指针，不改变 DMA 状态 |
| `set_pingpong_size` | 可选 | 线程 | 运行时调整内部中间缓冲大小（DVP 专用） |

---

## 9. 分层交互时序

### 单帧拍照

```
应用                Handle 层            OV2640 驱动            DVP
  │                    │                      │                   │
  ├─camera_capture_single()                   │                   │
  │                    ├─rt_device_read()─────►                   │
  │                    │                      ├─bus_adapter_start_capture()
  │                    │                      │                   ├─DMA start
  │                    │                      │                   │  ···帧到来···
  │                    │                      │◄──frame_isr()─────┤
  │                    │                      ├─rt_sem_release()  │
  │                    │◄─────────────────────┤                   │
  │◄──frame_size───────┤                      │                   │
```

### 连续流

```
应用线程             Handle 层             OV2640 驱动            DVP
  │                     │                      │                   │
  ├─camera_start_stream()                      │                   │
  │                     ├─rt_device_control(START_STREAM)         │
  │                     │                      ├─bus_adapter_set_frame_callback()
  │                     │                      ├─bus_adapter_start()
  │                     │                      │                   ├─DMA start
  │                     │                      │                   │
  │  (等待)             │                      │◄──dvp DMA ISR────┤
  │                     │◄──ov2640_frame_ready_callback()         │
  │                     ├─入队 + sem_release()  │                  │
  │◄─camera_get_stream_frame() 返回             │                  │
  │  处理帧 …           │                      │◄──dvp DMA ISR────┤（下一帧）
  │                     │◄──callback()          │                  │
  │                     ├─入队（若消费慢则覆盖旧帧 + LOG_W）        │
```

---

## 10. 常见问题

**Q1：`camera_init()` 返回 `CAMERA_ERRORRESOURCE`**  
确认 `ov2640_device_register()` 已被 `INIT_DEVICE_EXPORT` 调用，且 RT-Thread 设备树中存在名为 `"ov2640"` 的设备（可用 `list_device` MSH 命令验证）。

**Q2：`camera_init()` 返回 `CAMERA_ERRORRESOURCE`（device_ops 为 NULL）**  
调用 `camera_handler_instance_init()` 时必须传入非 NULL 的 `input_arg->device_ops`，Handle 层没有内建的 ops 表。

**Q3：串流一段时间后日志出现 `camera: stream queue full`**  
消费线程处理帧的速度慢于传感器出帧速度，`dropped_count` 持续增加。对策：降低帧率（`OV2640_CMD_SET_FRAMESIZE` 选小分辨率）、缩短消费线程处理时间，或提高消费线程优先级。

**Q4：`camera_change_settings()` 后首帧画面明显偏暗/偏色**  
这是正常现象——函数内有 500 ms AEC/AWB 收敛等待，若业务对稳定性要求更高，可在拍照前丢弃 2~3 帧。

**Q5：`DVP_DEFAULT_RESOURCE_CONFIG` 中的 VSYNC 引脚怎么确定**  
VSYNC 因板型差异较大，以宏参数形式传入：

```c
dvp_resource_config_t res = DVP_DEFAULT_RESOURCE_CONFIG(PAD_PA42 /* vsync */);
```

具体引脚参见硬件原理图。

**Q6：如何开启 DVP 调试快照**  
在 `dvp.c` 顶部将 `#define DEBUG_DVP 0` 改为 `1`，重新编译后 `dump_buffer_hex()` 和 `snapshot_and_dump_buffer()` 函数以及 6 KB 快照缓冲区才会编入固件。

**Q7：`bus_adapter_find()` 返回 NULL**  
DVP 适配器通过 `INIT_BOARD_EXPORT(dvp_adapter_register)` 在板级初始化阶段自动注册。若在板级初始化完成前调用（例如在 `main()` 首行），可能找不到适配器。确保在 RT-Thread 初始化完成后再打开摄像头。

**Q8：`camera_capture_single` 超时后，下次再调用立即得到一帧脏数据**  
超时时 DVP DMA 可能仍在运行。超时路径下应先调用 `camera_deinit`，再调用 `camera_init` 彻底重置硬件和信号量状态，否则残留 DMA 完成信号会令下次拍照提前退出并读到上次未完成的缓冲区内容。

**Q9：`camera_stop_stream` 之后队列里还剩一帧旧数据，影响下次流启动**  
`camera_stop_stream` 会清零队列，但若应用在停流之前未消费全部帧，重新启动流后首次 `camera_get_stream_frame` 会返回第一帧新数据（队列已清空，不会有旧帧残留）。若对旧帧有疑虑，可在 `camera_start_stream` 之后以 `timeout=0` 非阻塞轮询一次，丢弃可能存在的遗留令牌：

```c
camera_stream_frame_t stale;
while (camera_get_stream_frame(&g_cam, &stale, 0) == CAMERA_OK)
    ; /* 丢弃 */
```

**Q10：如何统计丢帧？**  
Handle 层维护 `instance->dropped_count`，每次队列满覆盖旧帧时递增，并输出 `LOG_W("camera: stream queue full, dropped=%u")`。停流后读取：

```c
camera_stop_stream(&g_cam);
LOG_I("dropped frames: %u", g_cam.dropped_count);
```

若希望在流运行期间实时监控，可在消费线程每次 `camera_get_stream_frame` 之后检查 `frame.sequence` 是否连续——相邻两次差值 > 1 表示 Handle 层队列发生了覆盖丢帧。
