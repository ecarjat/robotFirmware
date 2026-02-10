#include "hip_control.h"

static hip_state_t s_left_state = {0};
static hip_state_t s_right_state = {0};
static hip_command_t s_left_cmd = {0};
static hip_command_t s_right_cmd = {0};
static hip_target_t s_target = {0};
static uint32_t s_faults = 0U;

void hip_control_init(void)
{
    s_left_state = (hip_state_t){0};
    s_right_state = (hip_state_t){0};
    s_left_cmd = (hip_command_t){0};
    s_right_cmd = (hip_command_t){0};
    s_target = (hip_target_t){0};
    s_faults = 0U;
}

void hip_control_apply_params(const robot_params_t *params)
{
    (void)params;
}

void hip_control_tick(uint32_t now_ms)
{
    (void)now_ms;
}

void hip_control_set_target(const hip_target_t *target)
{
    if (target) {
        s_target = *target;
    }
}

void hip_control_get_target(hip_target_t *target)
{
    if (target) {
        *target = s_target;
    }
}

void hip_control_get_state(hip_state_t *left, hip_state_t *right)
{
    if (left) {
        *left = s_left_state;
    }
    if (right) {
        *right = s_right_state;
    }
}

void hip_control_get_command(hip_command_t *left, hip_command_t *right)
{
    if (left) {
        *left = s_left_cmd;
    }
    if (right) {
        *right = s_right_cmd;
    }
}

uint32_t hip_control_get_faults(void)
{
    return s_faults;
}

bool hip_control_on_can_rx(uint32_t id, const uint8_t *data, uint8_t len, uint32_t t_ms)
{
    (void)id;
    (void)data;
    (void)len;
    (void)t_ms;
    return false;
}

void hip_control_on_bus_off(void)
{
    s_faults |= HIP_FAULT_BUS_OFF;
}

void hip_control_on_bus_recovered(void)
{
    s_faults &= ~(uint32_t)HIP_FAULT_BUS_OFF;
}

void hip_control_fake_set_states(const hip_state_t *left, const hip_state_t *right)
{
    if (left) {
        s_left_state = *left;
    }
    if (right) {
        s_right_state = *right;
    }
}

void hip_control_fake_set_commands(const hip_command_t *left, const hip_command_t *right)
{
    if (left) {
        s_left_cmd = *left;
    }
    if (right) {
        s_right_cmd = *right;
    }
}

void hip_control_fake_set_faults(uint32_t faults)
{
    s_faults = faults;
}
