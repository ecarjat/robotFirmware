#ifndef CONTROL_LQR_LUT_H
#define CONTROL_LQR_LUT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool lqr_lut_eval(float hip_rad, float K_out[4]);
bool lqr_lut_eval_full(float hip_rad, float K_out[4], float* theta_eq, float* u_eq);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LQR_LUT_H */
