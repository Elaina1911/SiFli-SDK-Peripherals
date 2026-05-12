/********************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file data_bus_adapter.c
 *
 * @par dependencies
 * - data_bus_adapter.h
 * - string.h
 *
 * @author SiFli 思澈科技
 *
 * @brief Data bus adapter registry and dispatch implementation.
 *
 * This module provides a lightweight static registry for `bus_adapter_t`
 * instances and implements thin wrapper APIs that validate pointers then
 * dispatch lifecycle/capture operations through `bus_adapter_ops_t`.
 *
 * Processing flow:
 * - Backend registers itself once via `bus_adapter_register()`.
 * - Upper layers locate backend by name with `bus_adapter_find()`.
 * - Wrapper APIs (`bus_adapter_*`) check `self` and op pointers.
 * - On valid dispatch, wrappers call into backend implementation directly.
 *
 * Notes:
 * - Registry is static and has a fixed capacity (`BUS_ADAPTER_MAX`).
 * - Duplicate registration by adapter name is treated as success.
 * - Wrappers return unified error codes for invalid/unsupported operations.
 *
 * @version V1.0 2026-5-11
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#include "data_bus_adapter.h"
#include <string.h>

#define BUS_ADAPTER_MAX 8
static bus_adapter_t *g_adapters[BUS_ADAPTER_MAX];
static int g_adapter_count = 0;

/**
 * @brief Register a bus adapter into the static registry.
 *
 * Registration is idempotent by adapter name: if a same-name adapter already
 * exists, this function returns BUS_OK and keeps the existing entry.
 *
 * @param adapter is a pointer to the adapter instance.
 *
 * @return Return BUS_OK on success.
 *         Return BUS_ERR_INVALID if adapter/name/ops is invalid.
 *         Return BUS_ERR_NO_SLOT when registry is full.
 */
int bus_adapter_register(bus_adapter_t *adapter)
{
    if (!adapter || !adapter->name || !adapter->ops)
        return BUS_ERR_INVALID;

    if (g_adapter_count >= BUS_ADAPTER_MAX)
        return BUS_ERR_NO_SLOT;

    for (int i = 0; i < g_adapter_count; i++) {
        if (strcmp(g_adapters[i]->name, adapter->name) == 0)
            return BUS_OK;
    }

    g_adapters[g_adapter_count++] = adapter;
    return BUS_OK;
}

/**
 * @brief Find a registered adapter by name.
 *
 * @param name is the adapter name string.
 *
 * @return Return adapter pointer when found; otherwise return NULL.
 */
bus_adapter_t *bus_adapter_find(const char *name)
{
    if (!name)
        return NULL;
    for (int i = 0; i < g_adapter_count; i++) {
        if (strcmp(g_adapters[i]->name, name) == 0)
            return g_adapters[i];
    }
    return NULL;
}

/*
 * Common dispatch helper used by thin wrapper APIs.
 *
 * - Return BUS_ERR_INVALID if self or self->ops is NULL.
 * - Return BUS_ERR_NOT_SUPPORTED if requested op is NULL.
 * - Otherwise forward call to the concrete adapter op.
 */
#define BUS_OP_DISPATCH(self_, member_, ...) \
    do { \
        if (!(self_) || !(self_)->ops) \
            return BUS_ERR_INVALID; \
        if (!(self_)->ops->member_) \
            return BUS_ERR_NOT_SUPPORTED; \
        return (self_)->ops->member_(__VA_ARGS__); \
    } while (0)

/**
 * @brief Wrapper for adapter init op.
 * @param self is the adapter instance.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_init(bus_adapter_t *self)
{
    BUS_OP_DISPATCH(self, init, self);
}

/**
 * @brief Wrapper for adapter deinit op.
 * @param self is the adapter instance.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_deinit(bus_adapter_t *self)
{
    BUS_OP_DISPATCH(self, deinit, self);
}

/**
 * @brief Wrapper for adapter start op.
 * @param self is the adapter instance.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_start(bus_adapter_t *self)
{
    BUS_OP_DISPATCH(self, start, self);
}

/**
 * @brief Wrapper for adapter stop op.
 * @param self is the adapter instance.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_stop(bus_adapter_t *self)
{
    BUS_OP_DISPATCH(self, stop, self);
}

/**
 * @brief Wrapper for frame callback registration op.
 * @param self      is the adapter instance.
 * @param callback  is the callback function; NULL means unregister.
 * @param user_data is the opaque pointer forwarded to `callback`.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_set_frame_callback(bus_adapter_t *self,
                                   bus_frame_ready_callback_t callback,
                                   void *user_data)
{
    BUS_OP_DISPATCH(self, set_frame_callback, self, callback, user_data);
}

/**
 * @brief Wrapper for start-capture op.
 * @param self is the adapter instance.
 * @param buffer is destination frame buffer pointer.
 * @param size is destination frame buffer size in bytes.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_start_capture(bus_adapter_t *self, void *buffer, uint32_t size)
{
    BUS_OP_DISPATCH(self, start_capture, self, buffer, size);
}

/**
 * @brief Wrapper for rearm-capture op.
 * @param self is the adapter instance.
 * @param buffer is destination frame buffer pointer.
 * @param size is destination frame buffer size in bytes.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_rearm_capture(bus_adapter_t *self, void *buffer, uint32_t size)
{
    BUS_OP_DISPATCH(self, rearm_capture, self, buffer, size);
}

/**
 * @brief Wrapper for abort-capture op.
 * @param self is the adapter instance.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_abort_capture(bus_adapter_t *self)
{
    BUS_OP_DISPATCH(self, abort_capture, self);
}

/**
 * @brief Wrapper for update-buffer op.
 * @param self is the adapter instance.
 * @param buffer is destination frame buffer pointer.
 * @param size is destination frame buffer size in bytes.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_update_buffer(bus_adapter_t *self, void *buffer, uint32_t size)
{
    BUS_OP_DISPATCH(self, update_buffer, self, buffer, size);
}

/**
 * @brief Wrapper for ping-pong-size op.
 * @param self is the adapter instance.
 * @param size is requested ping-pong size in bytes.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_set_pingpong_size(bus_adapter_t *self, uint32_t size)
{
    BUS_OP_DISPATCH(self, set_pingpong_size, self, size);
}

/**
 * @brief Wrapper for frame-size query op.
 * @param self is the adapter instance.
 * @return Return frame size when op is implemented; otherwise return 0.
 */
uint32_t bus_adapter_get_frame_size(bus_adapter_t *self)
{
    if (!self || !self->ops || !self->ops->get_frame_size)
        return 0;
    return self->ops->get_frame_size(self);
}

/**
 * @brief Wrapper for set-mode op.
 * @param self is the adapter instance.
 * @param mode is the requested generic capture mode.
 * @return BUS_OK / BUS_ERR_INVALID / BUS_ERR_NOT_SUPPORTED or op return code.
 */
int bus_adapter_set_mode(bus_adapter_t *self, bus_capture_mode_t mode)
{
    BUS_OP_DISPATCH(self, set_mode, self, mode);
}

/**
 * @brief Invoke the adapter's optional dump_state op for hardware diagnostics.
 * @param self is the adapter instance.
 */
void bus_adapter_dump_state(bus_adapter_t *self)
{
    if (!self || !self->ops || !self->ops->dump_state)
        return;
    self->ops->dump_state(self);
}
