/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_motion_gate

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// Averages are kept in 1/256 counts so small deltas survive the shift.
#define FIXED_ONE 256

struct motion_gate_config {
    uint8_t shift;
    int32_t open_percent;
    int32_t close_percent;
    uint32_t idle_ms;
};

struct motion_gate_data {
    bool open;
    int64_t last_event;
    int32_t avg_x;
    int32_t avg_y;
    int32_t avg_path;
};

static int motion_gate_handle_event(const struct device *dev, struct input_event *event,
                                    uint32_t param1, uint32_t param2,
                                    struct zmk_input_processor_state *state) {
    const struct motion_gate_config *cfg = dev->config;
    struct motion_gate_data *data = dev->data;

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now = k_uptime_get();
    if (now - data->last_event > cfg->idle_ms) {
        data->open = false;
        data->avg_x = data->avg_y = data->avg_path = 0;
    }
    data->last_event = now;

    int32_t dx = event->code == INPUT_REL_X ? event->value * FIXED_ONE : 0;
    int32_t dy = event->code == INPUT_REL_Y ? event->value * FIXED_ONE : 0;
    data->avg_x += (dx - data->avg_x) >> cfg->shift;
    data->avg_y += (dy - data->avg_y) >> cfg->shift;
    data->avg_path += (abs(event->value) * FIXED_ONE - data->avg_path) >> cfg->shift;

    // Real motion keeps heading one way, so its net average stays close to
    // its path average. Jitter cancels itself out.
    int32_t net = abs(data->avg_x) + abs(data->avg_y);
    bool was_open = data->open;
    if (data->open) {
        data->open = net * 100 >= cfg->close_percent * data->avg_path;
    } else {
        data->open = net * 100 >= cfg->open_percent * data->avg_path &&
                     data->avg_path >= FIXED_ONE / 2;
    }
    if (data->open != was_open) {
        LOG_DBG("motion gate %s", data->open ? "opened" : "closed");
    }

    return data->open ? ZMK_INPUT_PROC_CONTINUE : ZMK_INPUT_PROC_STOP;
}

static struct zmk_input_processor_driver_api motion_gate_driver_api = {
    .handle_event = motion_gate_handle_event,
};

#define MOTION_GATE_INST(n)                                                                        \
    static struct motion_gate_data motion_gate_data_##n = {};                                      \
    static const struct motion_gate_config motion_gate_config_##n = {                              \
        .shift = DT_INST_PROP(n, smoothing_shift),                                                 \
        .open_percent = DT_INST_PROP(n, open_percent),                                             \
        .close_percent = DT_INST_PROP(n, close_percent),                                           \
        .idle_ms = DT_INST_PROP(n, idle_ms),                                                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &motion_gate_data_##n, &motion_gate_config_##n,           \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &motion_gate_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MOTION_GATE_INST)
