#ifndef CONTROL_HIP_BEHAVIOR_H
#define CONTROL_HIP_BEHAVIOR_H

#include <stdbool.h>
#include <stdint.h>

#include "hip_control.h"
#include "motion_modes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HIP_BEHAVIOR_NORMAL = 0,
    HIP_BEHAVIOR_CROUCH = 1,
    HIP_BEHAVIOR_IMPULSE = 2,
    HIP_BEHAVIOR_FLIGHT = 3,
    HIP_BEHAVIOR_LANDING = 4
} hip_behavior_mode_t;

void hip_behavior_init(float nominal_height_m);
void hip_behavior_set_height_ref(float height_m);
void hip_behavior_request_jump(void);
void hip_behavior_cancel_jump(void);
void hip_behavior_set_imu_accel(float accel_z_g, bool valid);
uint8_t hip_behavior_get_phase_progress(void);
void hip_behavior_tick(uint32_t now_ms,
                       motion_mode_t motion_mode,
                       const hip_state_t *left,
                       const hip_state_t *right,
                       hip_target_t *target_out,
                       hip_behavior_mode_t *mode_out);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_HIP_BEHAVIOR_H */
