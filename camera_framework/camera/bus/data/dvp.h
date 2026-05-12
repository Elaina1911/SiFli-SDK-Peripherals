/******************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file dvp.h
 * 
 * @par dependencies 
 * - rtthread.h
 * - bf0_hal.h
 * - stdint.h
 * 
 * @author SiFli 思澈科技
 * 
 * @brief Provide the HAL APIs of camera handler 
 * and corresponding operations.
 * 
 * Processing flow:
 * 
 * Call directly.
 * 
 * @version V1.0 2026-4-3
 *
 * @note 1 tab == 4 spaces!
 * 
 *****************************************************************************/

#ifndef __DVP_H__
#define __DVP_H__

//******************************** Includes *********************************//
#include "rtthread.h"
#include "bf0_hal.h"
#include <stdint.h>
//******************************** Includes *********************************//

//******************************** Defines **********************************//
#ifdef __cplusplus
extern "C" {
#endif

/* PCLK and HSYNC pin PAD numbers come from Kconfig.
 * The alternate function is GPTIM1_ETR / GPTIM1_CH1 on all supported platforms. */
#define DVP_PCLK_PIN_PAD           (PAD_PA00 + OV2640_DVP_PCLK_PIN)
#define DVP_PCLK_PIN_FUNC          GPTIM1_ETR
#define DVP_HSYNC_PIN_PAD          (PAD_PA00 + OV2640_DVP_HSYNC_PIN)
#define DVP_HSYNC_PIN_FUNC         GPTIM1_CH1

/* DATA bus pins and their GPIO/DMA source address are platform-specific. */
#ifdef SF32LB52X
#define DVP_DATA_PIN_PAD_BASE      PAD_PA00
#define DVP_DATA_PIN_FUNC_BASE     GPIO_A0
#define DVP_DATA_GPIO_PIN_BASE     0
#define DVP_DATA_PIN_SOURCE_ADDR   ((uint32_t)&hwp_gpio1->DIR)

#elif defined(SF32LB56X)
#define DVP_DATA_PIN_PAD_BASE      (PAD_PA00 + 64)
#define DVP_DATA_PIN_FUNC_BASE     (GPIO_A0 + 64)
#define DVP_DATA_GPIO_PIN_BASE     64
#define DVP_DATA_PIN_SOURCE_ADDR   ((uint32_t)&((GPIO1_TypeDef *)GPIO1_BASE)->DIR2)

#else
    #error "DVP default resource macros are not defined for this platform"
#endif

#define DVP_DATA_GPIO_INSTANCE     hwp_gpio1
#define DVP_TIMER_INSTANCE         hwp_gptim1
#define DVP_TIMER_RCC_MODULE       RCC_MOD_GPTIM1
#define DVP_DMA_INSTANCE           DMA1_Channel1
#define DVP_DMA_REQUEST            8
#define DVP_DMA_IRQn               DMAC1_CH1_IRQn

#define DVP_DEFAULT_RESOURCE_CONFIG(vsync_pin_) \
    { \
        .pclk_pin_pad = DVP_PCLK_PIN_PAD, \
        .pclk_pin_func = DVP_PCLK_PIN_FUNC, \
        .hsync_pin_pad = DVP_HSYNC_PIN_PAD, \
        .hsync_pin_func = DVP_HSYNC_PIN_FUNC, \
        .vsync_pin = (vsync_pin_), \
        .data_pin_pad_base = DVP_DATA_PIN_PAD_BASE, \
        .data_pin_func_base = DVP_DATA_PIN_FUNC_BASE, \
        .data_gpio_pin_base = DVP_DATA_GPIO_PIN_BASE, \
        .data_pin_source_addr = DVP_DATA_PIN_SOURCE_ADDR, \
        .data_gpio_instance = DVP_DATA_GPIO_INSTANCE, \
        .timer_instance = DVP_TIMER_INSTANCE, \
        .timer_rcc_module = DVP_TIMER_RCC_MODULE, \
        .dma_instance = DVP_DMA_INSTANCE, \
        .dma_request = DVP_DMA_REQUEST, \
        .dma_irqn = DVP_DMA_IRQn, \
    }

//******************************** Defines **********************************//

//******************************** Typedefs *********************************//
/* DVP capture mode is the same concept as bus_capture_mode_t — use it directly
 * so the two layers share one enum and no conversion/assert is needed. */
#include "data_bus_adapter.h"

/* Forward declaration */
typedef struct dvp_handle dvp_handle_t;

/* Board/resource description for one DVP instance. */
typedef struct {
    uint32_t pclk_pin_pad;
    uint32_t pclk_pin_func;
    uint32_t hsync_pin_pad;
    uint32_t hsync_pin_func;
    uint8_t vsync_pin;

    uint32_t data_pin_pad_base;
    uint32_t data_pin_func_base;
    uint32_t data_gpio_pin_base;
    uint32_t data_pin_source_addr;
    void *data_gpio_instance;

    void *timer_instance;
    uint32_t timer_rcc_module;

    void *dma_instance;
    uint32_t dma_request;
    int32_t dma_irqn;
} dvp_resource_config_t;

/* DVP configuration structure */
typedef struct {
    bus_capture_mode_t mode;        /* DVP capture mode                          */
    uint32_t buffer_size;           /* Image buffer size in bytes                */
    uint8_t *frame_buffer;          /* Frame buffer pointer (user provided)      */
    uint32_t pingpong_buffer_size;  /* Ping-pong DMA buffer size in bytes        */
    dvp_resource_config_t resources;

    /** PAx pin index for XCLK output via GPTIM2_CH1; -1 = XCLK disabled. */
    int      xclk_pin;
    /** XCLK output frequency in Hz; 0 = XCLK disabled. */
    uint32_t xclk_freq;
} dvp_config_t;

typedef struct {
    volatile uint8_t frame_ready;
    volatile uint32_t current_size;
    volatile uint8_t enable_capture;
    volatile uint8_t capture_started;
    volatile uint8_t soi_found;
} dvp_capture_state_t;

typedef struct {
    volatile uint8_t prev_byte;
    volatile uint8_t prev_byte_valid;
} dvp_jpeg_parser_state_t;

/* DVP handle structure */
struct dvp_handle{
    dvp_config_t config;
    GPT_HandleTypeDef gptim;
    DMA_HandleTypeDef dma;
    uint8_t *pingpong_buffer;
    dvp_capture_state_t capture;
    dvp_jpeg_parser_state_t jpeg;
};
/* Function declarations */

#define DVP_BUS_ADAPTER_NAME "dvp"

//******************************** Typedefs *********************************//

//******************************** Function *********************************//
/*
 * Public DVP API. The first argument matches `bus_adapter_t *self` so these
 * functions are used directly as `bus_adapter_ops_t` entries; no trampoline
 * layer exists. `self->priv` points at the DVP singleton handle.
 */

/**
 * @brief Initialize DVP interface.
 * @param self DVP bus adapter (use dvp_get_bus_adapter() to obtain the singleton).
 * @param cfg Pointer to a `dvp_config_t` describing hardware resources & mode.
 * @return 0 on success, negative on failure.
 */
int dvp_init(bus_adapter_t *self);

/** @brief Deinitialize DVP interface. */
int dvp_deinit(bus_adapter_t *self);

/** @brief Start DVP hardware (timer + DMA). */
int dvp_start(bus_adapter_t *self);

/** @brief Stop DVP hardware (timer + DMA). */
int dvp_stop(bus_adapter_t *self);

/** @brief Register a generic frame-ready callback. */
int dvp_set_frame_callback(bus_adapter_t *self,
                           bus_frame_ready_callback_t callback,
                           void *user_data);

/** @brief Arm the next frame capture, optionally swapping the user buffer. */
int dvp_start_capture(bus_adapter_t *self, void *new_buffer, uint32_t buffer_size);

/** @brief Re-arm capture from inside the frame callback (no HW restart). */
int dvp_rearm_capture(bus_adapter_t *self, void *new_buffer, uint32_t buffer_size);

/** @brief Abort current capture (HW keeps running). */
int dvp_abort_capture(bus_adapter_t *self);

/** @brief Replace user frame buffer pointer/size without touching state. */
int dvp_update_buffer(bus_adapter_t *self, void *buffer, uint32_t size);

/** @brief Get current captured frame size in bytes. */
uint32_t dvp_get_frame_size(bus_adapter_t *self);

/**
 * @brief Resize the ping-pong buffer (must stop DVP first).
 * @note For RAW mode, set to 2 * image_width for one-line ping-pong.
 */
int dvp_set_pingpong_size(bus_adapter_t *self, uint32_t new_size);

/**
 * @brief Switch the DVP capture mode (generic @ref bus_capture_mode_t).
 *
 * Caller is responsible for stopping the bus before invoking, and starting
 * it again after. Returns BUS_OK on success or BUS_ERR_INVALID on bad input.
 */
int dvp_set_mode(bus_adapter_t *self, bus_capture_mode_t mode);

/**
 * @brief Dump DVP hardware diagnostic state (DMA, GPT, VSYNC, DMAC registers) to the log.
 *
 * Called via @ref bus_adapter_dump_state when the upper layer needs bus-level
 * diagnostics (e.g. on capture timeout). Must not be called from ISR context.
 */
void dvp_dump_state(bus_adapter_t *self);

/**
 * @brief Get the DVP module's `bus_adapter_t` instance.
 *
 * The DVP driver itself implements `bus_adapter_ops_t`, so callers may use the
 * generic `bus_adapter_*` interface to drive DVP. The adapter is also auto-
 * registered in the global registry under `DVP_BUS_ADAPTER_NAME` at INIT_BOARD.
 */
bus_adapter_t *dvp_get_bus_adapter(void);

#ifdef __cplusplus
}
#endif

#endif /* __DVP_H__ */
