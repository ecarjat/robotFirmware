#include "param_storage.h"

bool g_param_can_save = true;
int g_param_save_calls = 0;

bool param_storage_can_save(void)
{
    return g_param_can_save;
}

int param_storage_save(const robot_params_t *params)
{
    (void)params;
    g_param_save_calls++;
    return PARAM_OK;
}

void param_storage_get_defaults(robot_params_t *params)
{
    if (params) {
        *params = (robot_params_t){0};
    }
}

int param_storage_load(robot_params_t *params)
{
    if (params) {
        *params = (robot_params_t){0};
    }
    return PARAM_OK;
}

int param_storage_init(void)
{
    return PARAM_OK;
}

int param_storage_erase(void)
{
    return PARAM_OK;
}

int param_storage_stats(uint32_t *used_bytes, uint32_t *free_bytes,
                        uint32_t *record_count)
{
    if (used_bytes) {
        *used_bytes = 0U;
    }
    if (free_bytes) {
        *free_bytes = 0U;
    }
    if (record_count) {
        *record_count = 0U;
    }
    return PARAM_OK;
}

bool param_storage_is_dirty(const robot_params_t *params)
{
    (void)params;
    return false;
}
