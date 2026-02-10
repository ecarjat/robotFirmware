#include "lqr_lut.h"
#include "lqr_lut_data.h"

#include <stddef.h>

bool lqr_lut_eval_full(float hip_rad, float K_out[4], float* theta_eq, float* u_eq)
{
    if (K_out == NULL) {
        return false;
    }

    if (hip_rad <= kHipLut[0]) {
        for (size_t i = 0; i < 4; ++i) {
            K_out[i] = kLqrLut[0][i];
        }
        if (theta_eq) *theta_eq = kThetaEq[0];
        if (u_eq) *u_eq = kUEq[0];
        return true;
    }

    if (hip_rad >= kHipLut[LQR_LUT_SIZE - 1]) {
        for (size_t i = 0; i < 4; ++i) {
            K_out[i] = kLqrLut[LQR_LUT_SIZE - 1][i];
        }
        if (theta_eq) *theta_eq = kThetaEq[LQR_LUT_SIZE - 1];
        if (u_eq) *u_eq = kUEq[LQR_LUT_SIZE - 1];
        return true;
    }

    for (size_t idx = 0; idx + 1 < LQR_LUT_SIZE; ++idx) {
        float x0 = kHipLut[idx];
        float x1 = kHipLut[idx + 1];
        if (hip_rad >= x0 && hip_rad <= x1) {
            float denom = x1 - x0;
            float t = (denom > 0.0f) ? ((hip_rad - x0) / denom) : 0.0f;
            for (size_t i = 0; i < 4; ++i) {
                float y0 = kLqrLut[idx][i];
                float y1 = kLqrLut[idx + 1][i];
                K_out[i] = y0 + (y1 - y0) * t;
            }
            if (theta_eq) {
                float y0 = kThetaEq[idx];
                float y1 = kThetaEq[idx + 1];
                *theta_eq = y0 + (y1 - y0) * t;
            }
            if (u_eq) {
                float y0 = kUEq[idx];
                float y1 = kUEq[idx + 1];
                *u_eq = y0 + (y1 - y0) * t;
            }
            return true;
        }
    }

    return false;
}

bool lqr_lut_eval(float hip_rad, float K_out[4])
{
    return lqr_lut_eval_full(hip_rad, K_out, NULL, NULL);
}
