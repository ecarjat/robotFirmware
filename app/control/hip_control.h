#ifndef CONTROL_HIP_CONTROL_H
#define CONTROL_HIP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "param_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HIP_MODE_HOLD = 0,
    HIP_MODE_GROUND_FOLLOW = 1,
    HIP_MODE_JUMP = 2
} hip_mode_t;

typedef struct {
    float theta_rad;
    float theta_dot_rad_s;
    float pos_rev;
    float vel_rev_s;
    float height_m;
    float height_dot_m_s;
    float torque_nm;
    float torque_setpoint_nm;
    uint8_t valid;
    uint32_t last_rx_ms;
    uint8_t limit_upper;
    uint8_t limit_lower;
} hip_state_t;

typedef struct {
    float pos_cmd_rev;
    float vel_ff_rev_s;
    float torque_ff_nm;
} hip_command_t;

typedef struct {
    float height_ref_m;
    float height_rate_ref_m_s;
    float stiffness_n_m;
    float damping_n_s_m;
    hip_mode_t mode;
    bool enabled;
} hip_target_t;

typedef enum {
    HIP_FAULT_NONE = 0U,
    HIP_FAULT_BUS_OFF = 1U << 0,
    HIP_FAULT_HEARTBEAT_TIMEOUT = 1U << 1,
    HIP_FAULT_ENCODER_TIMEOUT = 1U << 2,
    HIP_FAULT_TORQUE_TIMEOUT = 1U << 3,
    HIP_FAULT_KINEMATICS = 1U << 4,
    HIP_FAULT_STALL_LEFT = 1U << 5,
    HIP_FAULT_STALL_RIGHT = 1U << 6,
    HIP_FAULT_STALL = (HIP_FAULT_STALL_LEFT | HIP_FAULT_STALL_RIGHT)
} hip_fault_t;

void hip_control_init(void);
void hip_control_apply_params(const robot_params_t *params);
void hip_control_tick(uint32_t now_ms);

void hip_control_set_target(const hip_target_t *target);
void hip_control_get_target(hip_target_t *target);
void hip_control_get_state(hip_state_t *left, hip_state_t *right);
void hip_control_get_command(hip_command_t *left, hip_command_t *right);
uint32_t hip_control_get_faults(void);

bool hip_control_on_can_rx(uint32_t id, const uint8_t *data, uint8_t len, uint32_t t_ms);
void hip_control_on_bus_off(void);
void hip_control_on_bus_recovered(void);
bool hip_control_program_node_id(uint8_t current_node_id, uint8_t new_node_id, bool save);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_HIP_CONTROL_H */
