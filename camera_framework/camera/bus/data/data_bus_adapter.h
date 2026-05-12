/********************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file data_bus_adapter.h
 *
 * @par dependencies
 * - stdint.h
 * - stddef.h
 *
 * @author SiFli 思澈科技
 *
 * @brief Data bus adapter interface for camera drivers.
 *
 * This header defines a generic data-bus abstraction used by camera sensor
 * drivers (for example OV2640) to decouple upper-layer capture logic from
 * concrete transport backends such as DVP, CSI, or SPI.
 *
 * Main design points:
 * - A concrete backend registers one `bus_adapter_t` instance by name.
 * - Backend private state is carried through `bus_adapter_t::priv`.
 * - All operations are grouped in `bus_adapter_ops_t` and uniformly receive
 *   `bus_adapter_t *self` as the first argument.
 * - Thin wrapper APIs validate pointers and dispatch calls to the backend ops.
 *
 * Notes:
 * - `bus_adapter_set_frame_callback()` callback is typically invoked in bus ISR
 *   context; callback work should be short and non-blocking.
 * - Wrapper APIs return unified `bus_status_t` style error codes when dispatch
 *   is invalid or unsupported.
 *
 * @version V1.0 2026-5-11
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#ifndef __DATA_BUS_ADAPTER_H__
#define __DATA_BUS_ADAPTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef enum {
    BUS_TYPE_INVALID = 0,
    BUS_TYPE_DVP,
    BUS_TYPE_SPI,
    BUS_TYPE_DCMI,
    BUS_TYPE_MAX,
} bus_type_t;

/*
 * Generic capture pixel/stream mode advertised to bus adapters.
 *
 * Concrete adapters (e.g. DVP) may translate this to their native enum.
 * Values are stable and shared between upper-layer drivers and bus adapters,
 * so any new format must be appended at the tail to preserve ABI.
 */
typedef enum {
    BUS_CAPTURE_MODE_JPEG   = 0,  /* JPEG stream (variable length, SOI/EOI framed) */
    BUS_CAPTURE_MODE_RAW    = 1,  /* RAW8 / fixed-size capture                      */
    BUS_CAPTURE_MODE_YUV422 = 2,  /* YUV422 fixed-size capture                      */
    BUS_CAPTURE_MODE_RGB565 = 3,  /* RGB565 fixed-size capture                      */
} bus_capture_mode_t;

/* Error codes returned by bus_adapter_* APIs.
 * Negative values represent errors; BUS_OK (0) means success. */
typedef enum {
    BUS_OK              =  0,
    BUS_ERR_INVALID     = -1,  /* invalid argument (NULL self / bad cfg) */
    BUS_ERR_NOT_SUPPORTED = -2,/* op not implemented by this adapter */
    BUS_ERR_NO_SLOT     = -3,  /* registry full */
    BUS_ERR_HW          = -4,  /* underlying hardware reported failure */
} bus_status_t;

typedef struct bus_frame {
    void *buffer;
    uint32_t length;
    uint32_t timestamp;
    uint32_t sequence;
} bus_frame_t;

struct bus_adapter;
typedef struct bus_adapter bus_adapter_t;

/* Frame ready callback. Called from bus IRQ context; keep work minimal. */
typedef void (*bus_frame_ready_callback_t)(bus_adapter_t *self,
                                           const bus_frame_t *frame,
                                           void *user_data);

/*
 * Bus adapter operations.
 *
 * - init / deinit / start / stop are lifecycle.
 * - set_frame_callback registers a callback invoked when a complete frame is ready.
 * - start_capture / abort_capture / rearm_capture drive a single-frame capture
 *   sequence that the bus implementation uses to build its data path.
 * - update_buffer / set_pingpong_size / get_frame_size are runtime tunables
 *   common to DMA-driven bus implementations.
 *
 * Every op takes the owning adapter as `self`; implementations cast `self->priv`
 * to access their private state.
 *
 * Any op may be NULL if the bus does not support that operation; callers must
 * handle NULL gracefully (see bus_adapter_* helper wrappers below).
 */
typedef struct bus_adapter_ops {
    int (*init)(bus_adapter_t *self);
    int (*deinit)(bus_adapter_t *self);
    int (*start)(bus_adapter_t *self);
    int (*stop)(bus_adapter_t *self);
    int (*set_frame_callback)(bus_adapter_t *self,
                              bus_frame_ready_callback_t callback,
                              void *user_data);

    int (*start_capture)(bus_adapter_t *self, void *buffer, uint32_t size);
    int (*rearm_capture)(bus_adapter_t *self, void *buffer, uint32_t size);
    int (*abort_capture)(bus_adapter_t *self);
    int (*update_buffer)(bus_adapter_t *self, void *buffer, uint32_t size);
    int (*set_pingpong_size)(bus_adapter_t *self, uint32_t size);
    uint32_t (*get_frame_size)(bus_adapter_t *self);
    /*
     * Change the capture mode at runtime.
     *
     * Implementations are expected to (1) abort any in-flight transfer and
     * stop the data path before mutating internal state, and (2) leave the
     * adapter in the same lifecycle state as on entry (i.e. caller is still
     * responsible for re-issuing bus_adapter_start when needed). Returns
     * BUS_OK or a negative bus_status_t value.
     */
    int (*set_mode)(bus_adapter_t *self, bus_capture_mode_t mode);
    /*
     * Optional diagnostics hook. Invoked when the upper layer needs to dump
     * bus-specific hardware state (DMA counters, timer registers, etc.).
     * NULL means this adapter provides no diagnostic output.
     */
    void (*dump_state)(bus_adapter_t *self);
} bus_adapter_ops_t;

struct bus_adapter {
    const char *name;
    bus_type_t type;
    const bus_adapter_ops_t *ops;
    void *priv;
};

/**
 * @brief Register a bus adapter instance into the global registry.
 *
 * If another adapter with the same name already exists, this function returns
 * BUS_OK and keeps the original entry.
 *
 * @param adapter is a pointer to the adapter instance to register.
 *
 * @return Return BUS_OK on success.
 *         Return BUS_ERR_INVALID if adapter/name/ops is invalid.
 *         Return BUS_ERR_NO_SLOT when registry is full.
 */
int bus_adapter_register(bus_adapter_t *adapter);

/**
 * @brief Find a registered bus adapter by name.
 *
 * @param name is the adapter name string.
 *
 * @return Return the matching adapter pointer when found; otherwise return NULL.
 */
bus_adapter_t *bus_adapter_find(const char *name);

/*
 * Thin wrappers that validate self/ops and dispatch to ops table.
 *
 * Return convention:
 * - BUS_OK on success
 * - BUS_ERR_INVALID when self/ops is invalid
 * - BUS_ERR_NOT_SUPPORTED when the specific op is not implemented
 * - or underlying adapter return code
 */

/**
 * @brief Initialize adapter runtime and hardware resources.
 * @param self is the adapter instance.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_init(bus_adapter_t *self);

/**
 * @brief Deinitialize adapter runtime and hardware resources.
 * @param self is the adapter instance.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_deinit(bus_adapter_t *self);

/**
 * @brief Start adapter hardware data path.
 * @param self is the adapter instance.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_start(bus_adapter_t *self);

/**
 * @brief Stop adapter hardware data path.
 * @param self is the adapter instance.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_stop(bus_adapter_t *self);

/**
 * @brief Register frame callback invoked when one frame is ready.
 * @param self      is the adapter instance.
 * @param callback  is the callback function; NULL means unregister.
 * @param user_data is the opaque pointer forwarded to `callback`.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_set_frame_callback(bus_adapter_t *self,
                                   bus_frame_ready_callback_t callback,
                                   void *user_data);

/**
 * @brief Start one frame capture and optionally set destination buffer.
 * @param self is the adapter instance.
 * @param buffer is destination frame buffer pointer.
 * @param size is destination frame buffer size in bytes.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_start_capture(bus_adapter_t *self, void *buffer, uint32_t size);

/**
 * @brief Rearm capture state for next frame without full stop/start.
 * @param self is the adapter instance.
 * @param buffer is destination frame buffer pointer.
 * @param size is destination frame buffer size in bytes.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_rearm_capture(bus_adapter_t *self, void *buffer, uint32_t size);

/**
 * @brief Abort current capture sequence.
 * @param self is the adapter instance.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_abort_capture(bus_adapter_t *self);

/**
 * @brief Update destination frame buffer pointer and size.
 * @param self is the adapter instance.
 * @param buffer is destination frame buffer pointer.
 * @param size is destination frame buffer size in bytes.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_update_buffer(bus_adapter_t *self, void *buffer, uint32_t size);

/**
 * @brief Update ping-pong DMA buffer size.
 * @param self is the adapter instance.
 * @param size is requested ping-pong size in bytes.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_set_pingpong_size(bus_adapter_t *self, uint32_t size);

/**
 * @brief Query current captured frame size in bytes.
 * @param self is the adapter instance.
 * @return Return frame size when op is implemented; otherwise 0.
 */
uint32_t bus_adapter_get_frame_size(bus_adapter_t *self);

/**
 * @brief Change the bus capture mode at runtime.
 *
 * The adapter implementation must stop any in-flight capture before changing
 * mode. Callers are still responsible for re-issuing @ref bus_adapter_start
 * (or equivalent) when needed.
 *
 * @param self is the adapter instance.
 * @param mode is the requested generic capture mode.
 * @return See thin-wrapper return convention above.
 */
int bus_adapter_set_mode(bus_adapter_t *self, bus_capture_mode_t mode);

/**
 * @brief Dump bus adapter hardware diagnostic state to the log.
 *
 * Intended for debug/timeout diagnostics. No-op when the adapter does not
 * implement the @c dump_state op.
 *
 * @param self is the adapter instance.
 */
void bus_adapter_dump_state(bus_adapter_t *self);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_BUS_ADAPTER_H__ */
